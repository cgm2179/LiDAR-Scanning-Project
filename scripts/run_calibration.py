"""Run the P2 acceptance tests for one calibration session and write the report.

    python scripts/run_calibration.py phases/phase2_calibration/sessions/2026-11-05.yaml
    python scripts/run_calibration.py <session.yaml> --out somewhere/report.md

The report goes to phases/phase2_calibration/reports/<session date>_report.md
unless --out is given. Session format: phases/phase2_calibration/session_template.yaml. Pass lines come from
the `acceptance` section of config/pipeline.yaml (or pipeline.example.yaml).
"""

import argparse
from pathlib import Path

import yaml

from lidar_scanning.calibration import acceptance
from lidar_scanning.calibration.report import write_report
from lidar_scanning.fileio.pointcloud import read_points
from lidar_scanning.geometry.planes import crop_box

ROOT = Path(__file__).resolve().parents[1]
REPORTS = ROOT / "phases" / "phase2_calibration" / "reports"
_cache = {}


def load_limits():
    for name in ("pipeline.yaml", "pipeline.example.yaml"):
        path = ROOT / "config" / name
        if path.exists():
            limits = dict(acceptance.LIMITS)
            limits.update(yaml.safe_load(path.read_text()).get("acceptance") or {})
            return limits, path.name
    return dict(acceptance.LIMITS), "built-in defaults"


def points(scan, box=None):
    if scan not in _cache:
        _cache[scan] = read_points(ROOT / scan)
    pts = _cache[scan]
    if box:
        pts = crop_box(pts, box["min"], box["max"])
        if len(pts) < 3:
            raise ValueError(f"box {box} keeps {len(pts)} points of {scan}")
    return pts


def missing(*values):
    return any(v is None for v in values)


def run(session, limits):
    results, skipped = [], {}

    def attempt(name, fn):
        try:
            result = fn()
        except Exception as exc:
            skipped[name] = f"error: {exc}"
            return
        if result is None:
            skipped[name] = "inputs not filled in"
        else:
            results.append(result)

    def rb():
        cfg = session.get("range_bias") or {}
        walls = cfg.get("walls") or []
        if not walls or any(missing(w.get("scan"), w.get("disto_m")) for w in walls):
            return None
        return acceptance.range_bias([(points(w["scan"], w.get("box")), w["disto_m"]) for w in walls],
                                     cfg.get("model", "offset"), limits["range_bias_residual_mm"])

    def pf():
        cfg = session.get("plane_flatness") or {}
        if missing(cfg.get("scan")):
            return None
        return acceptance.plane_flatness(points(cfg["scan"], cfg.get("box")), limits["plane_rms_mm"],
                                         cfg.get("trim_threshold_m"))

    def st():
        cfg = session.get("straightness") or {}
        if missing(cfg.get("scan")):
            return None
        return acceptance.straightness(points(cfg["scan"], cfg.get("box")), limits["straightness_mm"],
                                       cfg.get("bin_m", 0.1))

    def rp():
        cfg = session.get("repeatability") or {}
        scans = cfg.get("scans") or []
        if not scans or missing(*scans, cfg.get("wall_a_box"), cfg.get("wall_b_box")):
            return None
        dims = []
        for s in scans:
            dim, _ = acceptance.room_dimension(points(s, cfg["wall_a_box"]), points(s, cfg["wall_b_box"]))
            dims.append(dim)
        return acceptance.repeatability(dims, limits["repeatability_sigma_mm"])

    def rg():
        cfg = session.get("registration") or {}
        if missing(cfg.get("source"), cfg.get("target")):
            return None
        import open3d as o3d
        from lidar_scanning.registration.pairwise import register_pair

        voxel = cfg.get("voxel_m", 0.05)
        threshold = cfg.get("icp_threshold_m") or voxel * 0.4
        clouds = [o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points(cfg[k]))) for k in ("source", "target")]
        _, icp, _ = register_pair(*clouds, voxel=voxel, icp_threshold=threshold)
        return acceptance.registration(icp.fitness, icp.inlier_rmse, threshold,
                                       limits["icp_fitness_min"], limits["icp_rmse_mm"])

    def ra():
        cfg = session.get("room_accuracy") or {}
        dims = cfg.get("dimensions") or []
        if missing(cfg.get("scan")) or not dims or any(
                missing(d.get("wall_a_box"), d.get("wall_b_box"), d.get("tape_m")) for d in dims):
            return None
        measured = {}
        for d in dims:
            dim, _ = acceptance.room_dimension(points(cfg["scan"], d["wall_a_box"]),
                                               points(cfg["scan"], d["wall_b_box"]))
            measured[d["name"]] = (dim, d["tape_m"])
        return acceptance.room_accuracy(measured, limits["room_accuracy_pct"])

    for name, fn in (("Range bias", rb), ("Plane flatness", pf), ("Straightness", st),
                     ("Repeatability", rp), ("Registration", rg), ("Room accuracy", ra)):
        attempt(name, fn)
    return results, skipped


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("session", help="session YAML")
    parser.add_argument("--out", help="report path (default: phase 2 reports folder, named by session date)")
    args = parser.parse_args()

    session = yaml.safe_load(Path(args.session).read_text())
    limits, limits_source = load_limits()
    results, skipped = run(session, limits)

    meta = dict(session.get("session") or {})
    meta["pass lines from"] = limits_source
    out = ROOT / args.out if args.out else REPORTS / f"{meta.get('date') or 'undated'}_report.md"
    out = write_report(results, out, meta=meta, skipped=skipped)

    for r in results:
        print(f"{'PASS' if r.passed else 'FAIL'}  {r.test:15s} {r.value:.3f} {r.unit} (line {r.limit})")
    for name, reason in skipped.items():
        print(f"SKIP  {name:15s} {reason}")
    print(f"Report: {out}")
