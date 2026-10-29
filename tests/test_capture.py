"""P3 capture and stitching, checked on synthetic captures in PiDAR's exact file format."""

import numpy as np
import pytest

from lidar_scanning.capture.checks import floor_to_ceiling
from lidar_scanning.capture.manifest import approx_pose_matrix, load_manifest
from lidar_scanning.capture.rig import RigCalibration, load_rig
from lidar_scanning.capture.simulate import ROOM_FACE_COLORS, Box, Scene, demo_scene, simulate_capture, simulate_station
from lidar_scanning.capture.station import assemble_points, load_station, pano_pixels, read_planes
from lidar_scanning.fileio.pointcloud import read_points, write_e57
from lidar_scanning.registration.multistation import EdgeResult, edge_warnings, pose_difference, stitch

COARSE = {"stepper_step_deg": 1.0, "beam_step_deg": 1.0, "pano_width": 256}
EMPTY_ROOM = Scene(Box((0.0, 0.0, 0.0), (5.0, 4.0, 2.5)))
# a rig with every correction switched on, so the test proves each one is applied
RIG = RigCalibration(lidar_offset_m=(0.0, -0.037, 0.02), range_scale=1.002, range_offset_m=0.004,
                     angle_scale=0.998, angle_offset_deg=1.5, pano_yaw_deg=-15.5, camera_offset_m=(0, 0, 0.12))
POSE = approx_pose_matrix({"x_m": 1.4, "y_m": 1.1, "z_m": 1.3, "heading_deg": 25})


def distance_to_room(points_world, scene):
    lo, hi = np.asarray(scene.room.lo), np.asarray(scene.room.hi)
    return np.min(np.abs(np.concatenate([points_world - lo, points_world - hi], axis=1)), axis=1)


def to_world(points, pose):
    return points @ pose[:3, :3].T + pose[:3, 3]


@pytest.fixture(scope="module")
def empty_room_station(tmp_path_factory):
    folder = tmp_path_factory.mktemp("station")
    simulate_station(EMPTY_ROOM, POSE, folder, RIG, **COARSE)
    return folder


def test_read_planes_sorts_by_angle_and_reads_csv(tmp_path):
    row = np.array([[1000.0, 0.0, 200.0]])
    np.save(tmp_path / "plane_10.5.npy", row)
    np.savetxt(tmp_path / "plane_2.0.csv", row, delimiter=",")
    np.save(tmp_path / "plane_-1.0.npy", row)
    (tmp_path / "notes.txt").write_text("not a plane")
    assert [a for a, _ in read_planes(tmp_path)] == [-1.0, 2.0, 10.5]


def test_zero_returns_dropped():
    planes = [(0.0, np.array([[0.0, 0.0, 0.0], [2000.0, 0.0, 180.0]]))]
    points, intensity = assemble_points(planes, RigCalibration())
    assert len(points) == 1 and np.allclose(np.abs(points[0]), [2.0, 0, 0]) and intensity[0] == 180


def test_station_lands_on_room_surfaces(empty_room_station):
    scan = load_station(empty_room_station, RIG)
    d = distance_to_room(to_world(scan.points, POSE), EMPTY_ROOM)
    assert np.median(d) < 0.005 and (d < 0.02).mean() > 0.99
    wrong = RigCalibration(**{**RIG.__dict__, "rotation_sign": 1})
    d_wrong = distance_to_room(to_world(load_station(empty_room_station, wrong).points, POSE), EMPTY_ROOM)
    assert (d_wrong < 0.02).mean() < 0.9


def test_range_correction_is_applied(empty_room_station):
    uncorrected = RigCalibration(**{**RIG.__dict__, "range_scale": 1.0, "range_offset_m": 0.0})
    d = distance_to_room(to_world(load_station(empty_room_station, uncorrected).points, POSE), EMPTY_ROOM)
    assert np.median(d) > 0.005


def test_panorama_colours_match_surfaces(empty_room_station):
    scan = load_station(empty_room_station, RIG)
    world = to_world(scan.points, POSE)
    lo, hi = np.asarray(EMPTY_ROOM.room.lo), np.asarray(EMPTY_ROOM.room.hi)
    gaps = np.abs(np.concatenate([world - lo, world - hi], axis=1))[:, [0, 3, 1, 4, 2, 5]]
    face, on_face = np.argmin(gaps, axis=1), np.min(gaps, axis=1) < 0.01
    palette = np.array(ROOM_FACE_COLORS) / 255
    match = np.all(np.abs(scan.colors[on_face] - palette[face[on_face]]) < 0.02, axis=1)
    assert match.mean() > 0.97


def test_pano_pixels_match_pidar_convention():
    rng = np.random.default_rng(0)
    d = rng.normal(size=(2000, 3))
    w, h, yaw = 3600, 1800, -15.5
    col, row = pano_pixels(d, w, h, yaw)
    # PiDAR lib/pointcloud.py get_sampling_coordinates (rounds where we floor)
    theta = np.arccos(d[:, 2] / np.linalg.norm(d, axis=1))
    lon = (np.arctan2(d[:, 1], d[:, 0]) + np.deg2rad(90 + yaw) + 2 * np.pi) % (2 * np.pi)
    pid_col = np.clip(np.round((2 * np.pi - lon) / (2 * np.pi) * w), 0, w - 1)
    pid_row = np.clip(np.round((1 - ((np.pi / 2 - theta + np.pi / 2) % np.pi) / np.pi) * h), 0, h - 1)
    col_gap = np.minimum(np.abs(col - pid_col), w - np.abs(col - pid_col))
    assert col_gap.max() <= 1 and np.abs(row - pid_row).max() <= 1


def test_floor_to_ceiling(empty_room_station):
    result = floor_to_ceiling(load_station(empty_room_station, RIG).points)
    assert result["height_m"] == pytest.approx(2.5, abs=0.005)


def test_rig_yaml(tmp_path):
    (tmp_path / "rig.yaml").write_text("range_scale: 1.001\nlidar_offset_m: [0, -0.03, 0]\n")
    rig = load_rig(tmp_path / "rig.yaml")
    assert rig.range_scale == 1.001 and rig.lidar_offset_m == (0.0, -0.03, 0.0)
    (tmp_path / "bad.yaml").write_text("rang_scale: 1.0\n")
    with pytest.raises(ValueError, match="unknown rig settings"):
        load_rig(tmp_path / "bad.yaml")


def test_manifest(tmp_path):
    (tmp_path / "m.yaml").write_text("""
capture: {id: test, reference_station: B}
stations:
  - {id: A, neighbours: [B, C], approx_pose: {x_m: 1, y_m: 2, z_m: 0, heading_deg: 90}, disto: {floor_ceiling_m: 2.5, wall_1_m: null}}
  - {id: B, neighbours: [A]}
  - {id: C, approx_pose: {x_m: null, y_m: null, z_m: null, heading_deg: null}}
""")
    capture = load_manifest(tmp_path / "m.yaml")
    assert capture.reference == "B" and capture.edges() == [("A", "B"), ("A", "C")]
    assert set(capture.approx_poses()) == {"A"}
    assert capture.station("A").disto == {"floor_ceiling_m": 2.5}
    assert capture.station("A").folder == (tmp_path / "A").resolve()
    (tmp_path / "bad.yaml").write_text("stations:\n  - {id: A, neighbours: [Z]}\n")
    with pytest.raises(ValueError, match="unknown neighbours"):
        load_manifest(tmp_path / "bad.yaml")


def test_write_e57_applies_poses(tmp_path):
    pts = np.random.default_rng(0).uniform(-2, 2, (300, 3))
    pose = approx_pose_matrix({"x_m": 3.0, "y_m": -1.0, "z_m": 0.5, "heading_deg": 40})
    write_e57(tmp_path / "r.e57", [{"name": "S1", "points": pts, "pose": pose,
                                    "colors": np.full((300, 3), 0.5), "intensity": np.full(300, 100.0)}])
    assert np.allclose(read_points(tmp_path / "r.e57"), to_world(pts, pose), atol=1e-6)


def test_stitch_recovers_true_poses(tmp_path):
    truth = simulate_capture(tmp_path / "cap", seed=1, **COARSE)
    capture = load_manifest(tmp_path / "cap" / "manifest.yaml")
    from lidar_scanning.capture.station import to_open3d
    clouds = {s.id: to_open3d(load_station(s.folder, RigCalibration())) for s in capture.stations}
    poses, edges = stitch(clouds, capture.edges(), capture.reference, capture.approx_poses())
    for sid, pose in poses.items():
        rot, trans = pose_difference(pose, np.linalg.inv(truth["S01"]) @ truth[sid])
        assert rot < 0.1 and trans < 0.005, sid
    assert edge_warnings(edges) == []


def test_edge_warnings_flag_failed_registration():
    stuck = EdgeResult("A", "B", "global + ICP", 0.6, 0.005, np.eye(4), residual_rot_deg=0.0, residual_trans_m=0.0)
    off = EdgeResult("A", "C", "ICP from sketch", 0.6, 0.005, approx_pose_matrix({"x_m": 2}),
                     pruned=True, residual_rot_deg=40.0, residual_trans_m=0.3)
    warnings = edge_warnings([stuck, off])
    assert len(warnings) == 2 and "no movement" in warnings[0] and "pruned" in warnings[1]


def test_demo_scene_has_features():
    assert len(demo_scene().obstacles) >= 2
