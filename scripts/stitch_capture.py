"""Assemble every station in a capture manifest, register them, export the registered E57.

    python scripts/stitch_capture.py phases/phase3_capture/captures/2026-11-20.yaml
    python scripts/stitch_capture.py <manifest.yaml> --rig config/rig.yaml --voxel 0.05

Writes to data/registered/<capture id>/:
    registered.e57      every station as its own scan with its registered pose (P4 input)
    merged_preview.ply  all stations merged, 2 cm voxels, for a quick look
    poses.json          4x4 station -> reference-station pose per station
and the stitch report to phases/phase3_capture/reports/<capture id>_stitch_report.md.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import open3d as o3d

from lidar_scanning.capture.checks import floor_to_ceiling
from lidar_scanning.capture.manifest import load_manifest
from lidar_scanning.capture.rig import load_rig
from lidar_scanning.capture.station import load_station, to_open3d
from lidar_scanning.fileio.pointcloud import write_e57
from lidar_scanning.registration.multistation import edge_warnings, stitch

ROOT = Path(__file__).resolve().parents[1]
REPORTS = ROOT / "phases" / "phase3_capture" / "reports"


def heading_deg(pose):
    return float(np.degrees(np.arctan2(pose[1, 0], pose[0, 0])))


def report(capture, rig_source, scans, checks, poses, edges, icp_threshold):
    lines = [f"# Stitch report: {capture.id}", "",
             f"- **date:** {capture.date or 'TBD'}",
             f"- **reference station:** {capture.reference} (registered frame = its station frame)",
             f"- **rig calibration:** {rig_source}", ""]
    warnings = edge_warnings(edges)
    lines += ["## Warnings", ""] + ([f"- **{w}**" for w in warnings] or ["- None."]) + [""]
    lines += ["## Stations", "",
             "| Station | Room | Storey | Points | Panorama | Floor–ceiling scan | Disto | Scan − Disto |",
             "|---|---|---|---|---|---|---|---|"]
    for s in capture.stations:
        scan, check = scans[s.id], checks[s.id]
        disto = s.disto.get("floor_ceiling_m")
        scan_h = f"{check['height_m']:.3f} m" if check else "not found"
        diff = f"{(check['height_m'] - disto) * 1000:+.1f} mm" if check and disto else "—"
        lines.append(f"| {s.id} | {s.room or '—'} | {s.storey if s.storey is not None else '—'} | "
                     f"{len(scan.points):,} | {'yes' if scan.colors is not None else 'no'} | {scan_h} | "
                     f"{f'{disto:.3f} m' if disto else '—'} | {diff} |")

    lines += ["", "## Registered pairs", "",
              "Residual = how far the optimised poses disagree with that pair's own registration. "
              "Large residuals on several pairs around one station point at that station.", "",
              "| Pair | Method | ICP fitness | ICP RMSE | Residual rot | Residual trans | Used as |",
              "|---|---|---|---|---|---|---|"]
    for e in edges:
        role = "pruned" if e.pruned else ("tree" if e.in_tree else "loop closure")
        lines.append(f"| {e.source} → {e.target} | {e.method} | {e.fitness:.3f} | {e.rmse_m * 1000:.1f} mm | "
                     f"{e.residual_rot_deg:.2f}° | {e.residual_trans_m * 1000:.1f} mm | {role} |")

    lines += ["", "## Station poses (reference frame)", "",
              "| Station | x (m) | y (m) | z (m) | heading (°) |", "|---|---|---|---|---|"]
    for sid, pose in poses.items():
        x, y, z = pose[:3, 3]
        lines.append(f"| {sid} | {x:.3f} | {y:.3f} | {z:.3f} | {heading_deg(pose):.1f} |")

    lines += ["", "## Notes", "",
              f"- ICP RMSE only counts points within {icp_threshold * 1000:.0f} mm, so it is bounded by that threshold.",
              "- Disto wall lengths and the floor closure check (summed rooms vs exterior, brief P3 exit) need "
              "room geometry from P4; they are not checked here.",
              "- Floor–ceiling heights assume each station was levelled."]
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("manifest")
    parser.add_argument("--rig", help="rig calibration YAML (default config/rig.yaml if present)")
    parser.add_argument("--voxel", type=float, default=0.05, help="registration voxel size, m")
    parser.add_argument("--icp-threshold", type=float, help="ICP max distance, m (default voxel * 0.4)")
    parser.add_argument("--no-sketch", action="store_true", help="ignore approx_pose; use global registration")
    parser.add_argument("--out", help="output folder (default data/registered/<capture id>)")
    args = parser.parse_args()

    rig_path = args.rig or (ROOT / "config" / "rig.yaml" if (ROOT / "config" / "rig.yaml").exists() else None)
    rig = load_rig(rig_path)
    capture = load_manifest(args.manifest)
    icp_threshold = args.icp_threshold or args.voxel * 0.4

    scans, checks, clouds = {}, {}, {}
    for s in capture.stations:
        scans[s.id] = load_station(s.folder, rig)
        checks[s.id] = floor_to_ceiling(scans[s.id].points)
        clouds[s.id] = to_open3d(scans[s.id])
        print(f"{s.id}: {scans[s.id].n_planes} planes, {len(scans[s.id].points):,} points")

    approx = None if args.no_sketch else capture.approx_poses()
    poses, edges = stitch(clouds, capture.edges(), capture.reference, approx, args.voxel, icp_threshold)
    for e in edges:
        print(f"{e.source} -> {e.target}: fitness {e.fitness:.3f}, residual "
              f"{e.residual_rot_deg:.2f}° / {e.residual_trans_m * 1000:.1f} mm{' (pruned)' if e.pruned else ''}")
    for w in edge_warnings(edges):
        print(f"WARNING: {w}")

    out = Path(args.out) if args.out else ROOT / "data" / "registered" / capture.id
    out.mkdir(parents=True, exist_ok=True)
    write_e57(out / "registered.e57", [
        {"name": sid, "points": scans[sid].points, "colors": scans[sid].colors,
         "intensity": scans[sid].intensity, "pose": poses[sid]} for sid in clouds])
    merged = o3d.geometry.PointCloud()
    for sid, pcd in clouds.items():
        merged += o3d.geometry.PointCloud(pcd).transform(poses[sid])
    o3d.io.write_point_cloud(str(out / "merged_preview.ply"), merged.voxel_down_sample(0.02))
    (out / "poses.json").write_text(json.dumps({sid: p.tolist() for sid, p in poses.items()}, indent=1))

    REPORTS.mkdir(parents=True, exist_ok=True)
    report_path = REPORTS / f"{capture.id}_stitch_report.md"
    report_path.write_text(report(capture, rig_path or "defaults (uncalibrated)", scans, checks, poses, edges,
                                  icp_threshold))
    print(f"Registered E57: {out / 'registered.e57'}")
    print(f"Report: {report_path}")
