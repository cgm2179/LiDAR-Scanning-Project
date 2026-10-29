"""Synthetic PiDAR captures for testing before the rig exists.

Ray-casts a box-shaped room (with box obstacles) and writes exactly what the
rig writes: planes/plane_<angle>.npy ([x_mm, y_mm, luminance], PiDAR's driver
convention, zero distance for no return) and an equirectangular pano.png.
The RigCalibration passed in is treated as the truth, so assembling with the
same one should put every point back on the room's surfaces.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from lidar_scanning.capture.manifest import approx_pose_matrix
from lidar_scanning.capture.rig import RigCalibration
from lidar_scanning.capture.station import stepper_rotation

# room faces in raycast order: wall x-, wall x+, wall y-, wall y+, floor (z-), ceiling (z+)
ROOM_FACE_COLORS = [(200, 80, 60), (60, 160, 80), (70, 110, 200), (220, 190, 60), (130, 130, 130), (240, 240, 240)]


@dataclass
class Box:
    lo: tuple
    hi: tuple
    color: tuple = (150, 90, 160)


@dataclass
class Scene:
    room: Box                                   # interior of the room
    obstacles: list = field(default_factory=list)

    def palette(self):
        return np.array(ROOM_FACE_COLORS + [b.color for b in self.obstacles], dtype=np.uint8)


def demo_scene():
    """6 x 4.5 x 2.6 m room with a column, a cabinet and a table, so registration has features."""
    return Scene(
        room=Box((0.0, 0.0, 0.0), (6.0, 4.5, 2.6)),
        obstacles=[
            Box((2.8, 1.8, 0.0), (3.2, 2.2, 2.6), (120, 70, 40)),     # column
            Box((5.2, 0.0, 0.0), (6.0, 0.6, 0.9), (40, 120, 120)),    # cabinet
            Box((0.5, 3.5, 0.0), (1.5, 4.5, 0.75), (160, 60, 140)),   # table
        ],
    )


def raycast(origins, directions, scene):
    """Distance to the first surface along each ray and that surface's id
    (0-5 room faces, 6+ obstacles in order)."""
    o = np.asarray(origins, float)
    d = np.asarray(directions, float)
    lo, hi = np.asarray(scene.room.lo, float), np.asarray(scene.room.hi, float)
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / d
        # inside the room: leave through the face each axis is heading toward
        t_axes = np.where(d > 0, (hi - o) * inv, np.where(d < 0, (lo - o) * inv, np.inf))
        axis = np.argmin(t_axes, axis=1)
        rows = np.arange(len(o))
        t = t_axes[rows, axis]
        surface = axis * 2 + (d[rows, axis] > 0)
        for k, box in enumerate(scene.obstacles):
            t1 = (np.asarray(box.lo) - o) * inv
            t2 = (np.asarray(box.hi) - o) * inv
            t_near = np.nanmax(np.minimum(t1, t2), axis=1)
            t_far = np.nanmin(np.maximum(t1, t2), axis=1)
            hit = (t_far >= t_near) & (t_near > 1e-9) & (t_near < t)
            t = np.where(hit, t_near, t)
            surface = np.where(hit, 6 + k, surface)
    return t, surface


def simulate_station(scene, pose, out_dir, rig=None, stepper_step_deg=0.5, beam_step_deg=0.5,
                     noise_m=0.005, pano_width=1024, seed=0):
    """Write one station's planes/ and pano.png as the rig at `pose` (4x4 station -> room) would."""
    import cv2

    rig = rig or RigCalibration()
    rng = np.random.default_rng(seed)
    out_dir = Path(out_dir)
    planes_dir = out_dir / "planes"
    planes_dir.mkdir(parents=True, exist_ok=True)
    R_pose, t_pose = pose[:3, :3], pose[:3, 3]

    # PiDAR driver: x = -d cos(a + 90°), y = d sin(a + 90°), in the LiDAR's vertical plane
    beams = np.radians(np.arange(0, 360, beam_step_deg))
    u = np.column_stack([-np.cos(beams + np.pi / 2), np.sin(beams + np.pi / 2)])
    local_dirs = np.column_stack([u[:, 0], np.zeros(len(u)), u[:, 1]])
    offset = np.asarray(rig.lidar_offset_m, float)

    for angle in np.arange(0, 180, stepper_step_deg):
        Rs = stepper_rotation(angle, rig)
        origin = R_pose @ (Rs @ offset) + t_pose
        t, _ = raycast(np.broadcast_to(origin, local_dirs.shape), local_dirs @ (R_pose @ Rs).T, scene)
        true_r = t + rng.normal(0, noise_m, len(t))
        measured = (true_r - rig.range_offset_m) / rig.range_scale     # undo the P2 correction
        measured = np.where(np.isfinite(t) & (t <= rig.max_range_m), measured, 0.0)
        mm = np.round(measured * 1000)
        data = np.column_stack([mm * u[:, 0], mm * u[:, 1], np.full(len(mm), 200.0)]).astype(np.float32)
        np.save(planes_dir / f"plane_{round(float(angle), 2)}.npy", data)

    width, height = pano_width, pano_width // 2
    cols, rows = np.meshgrid(np.arange(width) + 0.5, np.arange(height) + 0.5)
    longitude = (1 - cols.ravel() / width) * 2 * np.pi
    phi = longitude - np.radians(90 + rig.pano_yaw_deg)
    elevation = (0.5 - rows.ravel() / height) * np.pi
    dirs = np.column_stack([np.cos(elevation) * np.cos(phi), np.cos(elevation) * np.sin(phi), np.sin(elevation)])
    origin = R_pose @ np.asarray(rig.camera_offset_m, float) + t_pose
    _, surface = raycast(np.broadcast_to(origin, dirs.shape), dirs @ R_pose.T, scene)
    pano = scene.palette()[surface].reshape(height, width, 3)
    cv2.imwrite(str(out_dir / "pano.png"), cv2.cvtColor(pano, cv2.COLOR_RGB2BGR))


def default_stations():
    """Three stations in the demo room: (id, x, y, z, heading_deg)."""
    return [("S01", 1.5, 1.5, 1.3, 0.0), ("S02", 4.5, 1.3, 1.3, 30.0), ("S03", 4.2, 3.4, 1.25, -60.0)]


def simulate_capture(out_dir, scene=None, stations=None, rig=None, sketch_error=(0.2, 5.0), seed=0, **station_kwargs):
    """Write a full synthetic capture: one folder per station plus manifest.yaml and truth.json.

    The manifest's approx_pose values are the true poses plus sketch_error
    (metres, degrees) of random error, like a hand-drawn field sketch.
    Returns {station_id: true 4x4 pose in the room frame}.
    """
    scene = scene or demo_scene()
    stations = stations or default_stations()
    rng = np.random.default_rng(seed)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    truth, entries = {}, []
    ids = [s[0] for s in stations]
    for k, (sid, x, y, z, heading) in enumerate(stations):
        pose = approx_pose_matrix({"x_m": x, "y_m": y, "z_m": z, "heading_deg": heading})
        truth[sid] = pose
        simulate_station(scene, pose, out_dir / sid, rig, seed=seed + k, **station_kwargs)
        dx, dy = (float(v) for v in rng.normal(0, sketch_error[0], 2))
        dh = float(rng.normal(0, sketch_error[1]))
        lo, hi = scene.room.lo, scene.room.hi
        entries.append({
            "id": sid, "folder": sid, "room": "synthetic room", "storey": 1, "exterior": False,
            "neighbours": [n for n in ids if n != sid],
            "approx_pose": {"x_m": round(x + dx, 2), "y_m": round(y + dy, 2), "z_m": float(z),
                            "heading_deg": round(heading + dh, 1)},
            "disto": {"wall_1_m": float(hi[0] - lo[0]), "wall_2_m": float(hi[1] - lo[1]),
                      "floor_ceiling_m": float(hi[2] - lo[2])},
            "notes": "synthetic",
        })

    manifest = {"capture": {"id": out_dir.name, "date": None, "reference_station": ids[0]}, "stations": entries}
    (out_dir / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
    (out_dir / "truth.json").write_text(json.dumps({k: v.tolist() for k, v in truth.items()}, indent=1))
    return truth
