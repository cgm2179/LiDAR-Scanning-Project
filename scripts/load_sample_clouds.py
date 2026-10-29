"""Load the sample point clouds PiDAR tests against and show them.

The samples are Open3D's DemoICPPointClouds (three overlapping scans of one
indoor room), the same data PiDAR's registration_test.py uses. They download
into data/sample/ on first run.

    python scripts/load_sample_clouds.py              # stats + Open3D window
    python scripts/load_sample_clouds.py --save out.png   # stats + top-down image, no window
"""

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

SAMPLE_DIR = Path(__file__).resolve().parents[1] / "data" / "sample"
COLORS = [(0.90, 0.40, 0.10), (0.10, 0.50, 0.85), (0.20, 0.65, 0.30)]


def load_samples():
    paths = o3d.data.DemoICPPointClouds(data_root=str(SAMPLE_DIR)).paths
    return [(Path(p).name, o3d.io.read_point_cloud(p)) for p in paths]


def describe(name, pcd):
    pts = np.asarray(pcd.points)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    print(f"{name}: {len(pts):,} points, normals={pcd.has_normals()}, colors={pcd.has_colors()}")
    print(f"  extent (m): x {hi[0]-lo[0]:.2f}  y {hi[1]-lo[1]:.2f}  z {hi[2]-lo[2]:.2f}")


def save_top_down(clouds, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))
    for (name, pcd), color in zip(clouds, COLORS):
        pts = np.asarray(pcd.voxel_down_sample(0.02).points)
        axes[0].scatter(pts[:, 0], pts[:, 1], s=0.2, color=color, label=name)
        axes[1].scatter(pts[:, 0], pts[:, 2], s=0.2, color=color)
    # Sample clouds are not gravity-aligned, so these are axis projections, not plan/elevation.
    axes[0].set(title="x-y projection, unregistered", xlabel="x (m)", ylabel="y (m)")
    axes[1].set(title="x-z projection", xlabel="x (m)", ylabel="z (m)")
    for ax in axes:
        ax.set_aspect("equal")
    axes[0].legend(markerscale=20, loc="upper right")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--save", help="write a top-down PNG instead of opening a window")
    args = parser.parse_args()

    clouds = load_samples()
    for name, pcd in clouds:
        describe(name, pcd)

    if args.save:
        save_top_down(clouds, args.save)
    else:
        for (_, pcd), color in zip(clouds, COLORS):
            pcd.paint_uniform_color(color)
        o3d.visualization.draw_geometries([pcd for _, pcd in clouds], window_name="PiDAR sample clouds")
