"""Assemble one station's 3D point cloud from PiDAR's LiDAR planes and panorama.

Follows PiDAR's merge_2D_points + angular_lookup, with three differences:
planes are ordered by the angle in their file name (PiDAR sorts names as text,
so plane_10.5 lands before plane_2.0), zero-distance returns are dropped, and
the P2 range correction and offsets are applied in metres.
"""

import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

PLANE_NAME = re.compile(r"^plane_(-?\d+(?:\.\d+)?)\.(npy|csv)$")
PANO_NAMES = ("pano.jpg", "pano.jpeg", "pano.png", "pano.tif", "pano.tiff")


@dataclass
class StationScan:
    points: np.ndarray            # N x 3, metres, station frame
    intensity: np.ndarray         # N, LiDAR luminance (0-255)
    colors: np.ndarray | None     # N x 3 in [0, 1] from the panorama, or None
    n_planes: int
    panorama: Path | None


def read_planes(planes_dir):
    """PiDAR LiDAR output: [(stepper_angle_deg, array N x 3 [x_mm, y_mm, luminance]), ...] by angle."""
    planes = []
    for path in Path(planes_dir).iterdir():
        match = PLANE_NAME.match(path.name)
        if not match:
            continue
        if match.group(2) == "npy":
            data = np.load(path)
        else:
            data = np.loadtxt(path, delimiter=",", ndmin=2)
        planes.append((float(match.group(1)), np.asarray(data, float)[:, :3]))
    if not planes:
        raise FileNotFoundError(f"no plane_<angle>.npy/.csv files in {planes_dir}")
    return sorted(planes, key=lambda p: p[0])


def stepper_rotation(angle_deg, rig):
    theta = np.radians(rig.rotation_sign * (rig.angle_scale * angle_deg + rig.angle_offset_deg))
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def assemble_points(planes, rig):
    """Stack the 2D planes into the station frame. Returns (points N x 3 m, intensity N)."""
    offset = np.asarray(rig.lidar_offset_m, float)
    all_points, all_intensity = [], []
    for angle, data in planes:
        xy = data[:, :2] / 1000.0
        lum = data[:, 2]
        r = np.hypot(xy[:, 0], xy[:, 1])
        keep = (r >= rig.min_range_m) & (r <= rig.max_range_m) & (lum >= rig.min_intensity)
        xy, lum, r = xy[keep], lum[keep], r[keep]
        xy = xy * ((rig.range_scale * r + rig.range_offset_m) / r)[:, None]
        # 2D-y is vertical: the LiDAR spins in the x-z plane of its own frame
        local = np.column_stack([xy[:, 0], np.zeros(len(xy)), xy[:, 1]]) + offset
        all_points.append(local @ stepper_rotation(angle, rig).T)
        all_intensity.append(lum)
    return np.vstack(all_points), np.concatenate(all_intensity)


def pano_pixels(directions, width, height, yaw_deg=0.0):
    """Equirectangular pixel (column, row) for each direction.

    Same mapping as PiDAR's get_sampling_coordinates, so a z_rotate value
    calibrated in PiDAR carries over as pano_yaw_deg.
    """
    d = np.asarray(directions, float)
    phi = np.arctan2(d[:, 1], d[:, 0])
    elevation = np.arctan2(d[:, 2], np.hypot(d[:, 0], d[:, 1]))
    longitude = np.mod(phi + np.radians(90 + yaw_deg), 2 * np.pi)
    col = np.floor((1 - longitude / (2 * np.pi)) * width).astype(int) % width
    row = np.clip(np.floor((0.5 - elevation / np.pi) * height).astype(int), 0, height - 1)
    return col, row


def read_panorama(path):
    """RGB uint8 array (H x W x 3). Raises ValueError if it isn't a 2:1 equirectangular image."""
    import cv2

    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"could not read panorama {path}")
    h, w = image.shape[:2]
    if abs(w / h - 2) > 0.05:
        raise ValueError(f"panorama {path} is {w}x{h}; expected an equirectangular 2:1 image")
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def colorize(points, pano_rgb, rig):
    """Colour each point from the panorama, seen from the camera's nodal point."""
    h, w = pano_rgb.shape[:2]
    col, row = pano_pixels(points - np.asarray(rig.camera_offset_m), w, h, rig.pano_yaw_deg)
    return pano_rgb[row, col].astype(float) / 255.0


def find_panorama(folder):
    for name in PANO_NAMES:
        if (Path(folder) / name).exists():
            return Path(folder) / name
    return None


def load_station(folder, rig, panorama=None):
    """Assemble a station folder: planes in <folder>/planes/ (or the folder itself,
    as PiDAR's DATA_DIR) and an optional pano.jpg/png/tif next to them."""
    folder = Path(folder)
    planes_dir = folder / "planes" if (folder / "planes").is_dir() else folder
    planes = read_planes(planes_dir)
    points, intensity = assemble_points(planes, rig)
    panorama = Path(panorama) if panorama else find_panorama(folder)
    colors = colorize(points, read_panorama(panorama), rig) if panorama else None
    return StationScan(points, intensity, colors, len(planes), panorama)


def to_open3d(scan):
    """Open3D point cloud, coloured from the panorama or, without one, grey by intensity."""
    import open3d as o3d

    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(scan.points))
    colors = scan.colors if scan.colors is not None else np.repeat(scan.intensity[:, None] / 255.0, 3, axis=1)
    pcd.colors = o3d.utility.Vector3dVector(np.clip(colors, 0, 1))
    return pcd
