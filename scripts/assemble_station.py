"""Assemble one station from PiDAR's output and save a coloured point cloud.

Quick check right after a scan (on the Pi or the laptop): did the LiDAR planes
and panorama come through, and does the floor-to-ceiling height match the Disto?

    python scripts/assemble_station.py data/raw/capture/2026-11-20/S01
    python scripts/assemble_station.py <station folder> --rig config/rig.yaml --pano other.jpg --out S01.ply
"""

import argparse
from pathlib import Path

import open3d as o3d

from lidar_scanning.capture.checks import floor_to_ceiling
from lidar_scanning.capture.rig import load_rig
from lidar_scanning.capture.station import load_station, to_open3d

ROOT = Path(__file__).resolve().parents[1]


def default_rig():
    path = ROOT / "config" / "rig.yaml"
    return path if path.exists() else None


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("station", help="folder with planes/ (or plane_*.npy) and pano.jpg")
    parser.add_argument("--rig", default=default_rig(), help="rig calibration YAML (default config/rig.yaml if present)")
    parser.add_argument("--pano", help="panorama path if it isn't pano.jpg/png/tif in the folder")
    parser.add_argument("--out", help="output .ply/.pcd (default <station>/station.ply)")
    parser.add_argument("--show", action="store_true", help="open an Open3D window")
    args = parser.parse_args()

    rig = load_rig(args.rig)
    scan = load_station(args.station, rig, args.pano)
    print(f"rig: {args.rig or 'defaults (uncalibrated)'}")
    print(f"{scan.n_planes} LiDAR planes, {len(scan.points):,} points")
    print(f"panorama: {scan.panorama or 'none (coloured by intensity)'}")

    height = floor_to_ceiling(scan.points)
    if height:
        print(f"floor to ceiling near station: {height['height_m']:.3f} m "
              f"(floor tilt {height['floor_tilt_deg']:.2f}°, ceiling tilt {height['ceiling_tilt_deg']:.2f}°)")
    else:
        print("floor to ceiling: floor or ceiling not found within 1.5 m of the station")

    pcd = to_open3d(scan)
    out = Path(args.out) if args.out else Path(args.station) / "station.ply"
    o3d.io.write_point_cloud(str(out), pcd)
    print(f"saved {out}")
    if args.show:
        o3d.visualization.draw_geometries([pcd], window_name=Path(args.station).name)
