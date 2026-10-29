"""Check the P2 acceptance tests recover known flaws planted in synthetic walls."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

from lidar_scanning.calibration import acceptance
from lidar_scanning.calibration.report import to_markdown
from lidar_scanning.fileio.pointcloud import read_points
from lidar_scanning.geometry.planes import crop_box, fit_plane

ROOT = Path(__file__).resolve().parents[1]


def make_wall(distance, width=3.0, height=2.4, noise=0.002, bow=0.0, n=20_000, seed=0):
    """Wall in the plane x = distance, facing the origin. `bow` bulges the middle toward +x."""
    rng = np.random.default_rng(seed)
    y = rng.uniform(-width / 2, width / 2, n)
    z = rng.uniform(0, height, n)
    x = distance + bow * (1 - (2 * y / width) ** 2) + rng.normal(0, noise, n)
    return np.column_stack([x, y, z])


def test_fit_plane_recovers_wall():
    plane = fit_plane(make_wall(3.0))
    assert abs(abs(plane.normal[0]) - 1) < 1e-4
    assert plane.distance_to(np.zeros(3)) == pytest.approx(3.0, abs=5e-4)


def test_crop_box():
    pts = np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2]], float)
    assert len(crop_box(pts, [0.5] * 3, [1.5] * 3)) == 1


def test_range_bias_constant_offset_corrected():
    walls = [(make_wall(d + 0.007, seed=k), d) for k, d in enumerate((2.0, 5.0, 8.0))]
    result = acceptance.range_bias(walls, model="offset")
    assert result.residuals["5.00 m raw bias (mm)"] == pytest.approx(7, abs=0.5)
    assert result.value < 1 and result.passed


def test_range_bias_scale_error_needs_linear_model():
    walls = [(make_wall(d * 1.002, seed=k), d) for k, d in enumerate((2.0, 5.0, 8.0))]
    assert not acceptance.range_bias(walls, model="offset").passed  # residuals ~±6 mm
    assert acceptance.range_bias(walls, model="linear").passed


def test_plane_flatness_tracks_noise():
    assert acceptance.plane_flatness(make_wall(3.0, noise=0.005)).value == pytest.approx(5, abs=0.3)
    assert not acceptance.plane_flatness(make_wall(3.0, noise=0.015)).passed


def test_plane_flatness_trim_drops_clutter():
    wall = make_wall(3.0, noise=0.005)
    clutter = make_wall(3.3, width=0.5, height=0.5, n=2_000, seed=1)
    pts = np.vstack([wall, clutter])
    assert not acceptance.plane_flatness(pts).passed
    trimmed = acceptance.plane_flatness(pts, trim_threshold_m=0.03)
    assert trimmed.value == pytest.approx(5, abs=0.5) and trimmed.passed


def test_straightness_detects_bow():
    bowed = acceptance.straightness(make_wall(3.0, width=5.0, bow=0.020, n=50_000))
    assert bowed.value == pytest.approx(20, rel=0.15) and not bowed.passed
    flat = acceptance.straightness(make_wall(3.0, width=5.0, noise=0.010, n=50_000))
    assert flat.passed


def test_straightness_warns_on_short_wall():
    assert any("shorter" in n for n in acceptance.straightness(make_wall(3.0, width=3.0)).notes)


def test_room_dimension_and_repeatability():
    dim, angle = acceptance.room_dimension(make_wall(0.0, seed=1), make_wall(4.0, seed=2))
    assert dim == pytest.approx(4.0, abs=1e-3) and angle < 0.5
    result = acceptance.repeatability([4.000, 4.003, 3.998, 4.001, 4.002])
    assert result.value == pytest.approx(1.92, abs=0.01) and result.passed


def test_registration_flags_rmse_bound():
    result = acceptance.registration(fitness=0.9, inlier_rmse_m=0.007, icp_threshold_m=0.05 * 0.4)
    assert result.passed
    assert any("cannot exceed" in n for n in result.notes)
    assert not acceptance.registration(0.7, 0.007, 0.02).passed


def test_room_accuracy():
    assert acceptance.room_accuracy({"length": (4.05, 4.0)}).passed
    assert not acceptance.room_accuracy({"length": (4.05, 4.0), "width": (3.1, 3.0)}).passed


def test_report_markdown():
    results = [acceptance.room_accuracy({"length": (4.05, 4.0)}),
               acceptance.room_accuracy({"length": (4.2, 4.0)})]
    text = to_markdown(results, meta={"room": None}, skipped={"Registration": "inputs not filled in"})
    assert "| PASS |" in text and "**FAIL**" in text and "not run" in text and "TBD" in text


def test_read_points_formats(tmp_path):
    import open3d as o3d
    import pye57

    pts = make_wall(2.0, n=500)
    o3d.io.write_point_cloud(str(tmp_path / "w.ply"), o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts)))
    np.save(tmp_path / "w.npy", np.column_stack([pts, np.ones(len(pts))]))
    e57 = pye57.E57(str(tmp_path / "w.e57"), mode="w")
    e57.write_scan_raw({"cartesianX": pts[:, 0], "cartesianY": pts[:, 1], "cartesianZ": pts[:, 2]})
    e57.close()
    for name in ("w.ply", "w.npy", "w.e57"):
        assert np.allclose(read_points(tmp_path / name), pts, atol=1e-6), name


def test_run_calibration_session(tmp_path):
    import open3d as o3d

    def save(name, pts):
        path = tmp_path / name
        o3d.io.write_point_cloud(str(path), o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts)))
        return str(path)

    spec = importlib.util.spec_from_file_location("run_calibration", ROOT / "scripts" / "run_calibration.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    session = {
        "range_bias": {"model": "offset", "walls": [
            {"scan": save(f"r{d}.ply", make_wall(d + 0.004, seed=d)), "box": None, "disto_m": float(d)}
            for d in (2, 5, 8)]},
        "plane_flatness": {"scan": save("flat.ply", make_wall(3.0, noise=0.004)), "box": None},
        "straightness": {"scan": None},
    }
    results, skipped = module.run(session, dict(acceptance.LIMITS))
    assert [r.test for r in results] == ["Range bias", "Plane flatness"]
    assert all(r.passed for r in results)
    assert skipped["Straightness"] == "inputs not filled in"


def test_register_pair_on_sample_clouds():
    paths = sorted((ROOT / "data/sample/extract/DemoICPPointClouds").glob("cloud_bin_[01].pcd"))
    if len(paths) < 2:
        pytest.skip("sample clouds not downloaded (run scripts/load_sample_clouds.py)")
    import open3d as o3d
    from lidar_scanning.registration.pairwise import register_pair

    o3d.utility.random.seed(0)
    source, target = (o3d.io.read_point_cloud(str(p)) for p in (paths[1], paths[0]))
    _, icp, _ = register_pair(source, target)
    assert icp.fitness > 0.9 and icp.inlier_rmse < 0.01
