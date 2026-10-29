"""Try PiDAR's registration settings on the sample clouds.

Same steps and parameters as third_party/PiDAR/registration_test.py:
voxel downsample -> FPFH -> RANSAC global registration -> point-to-plane ICP.
The dataset has no ground-truth poses (its init.log is only a starting guess),
so accuracy is checked by loop closure: 2->0 registered directly should match
2->1 followed by 1->0.

    python scripts/try_registration.py
    python scripts/try_registration.py --voxel 0.03 --save merged.png
"""

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from lidar_scanning.registration.pairwise import register_pair

SAMPLE_DIR = Path(__file__).resolve().parents[1] / "data" / "sample"
# Brief P2 "Registration" pass line
PASS_FITNESS = 0.8
PASS_RMSE_MM = 20


def pose_error(T_est, T_ref):
    """Rotation error (deg) and translation error (mm) between two 4x4 poses."""
    dT = np.linalg.inv(T_ref) @ T_est
    angle = np.degrees(np.arccos(np.clip((np.trace(dT[:3, :3]) - 1) / 2, -1, 1)))
    return angle, np.linalg.norm(T_est[:3, 3] - T_ref[:3, 3]) * 1000


def save_merged(clouds, transforms, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = [(0.90, 0.40, 0.10), (0.10, 0.50, 0.85), (0.20, 0.65, 0.30)]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))
    for title, ax, use_T in (("Before (raw frames)", axes[0], False), ("After RANSAC + ICP, in cloud_bin_0 frame", axes[1], True)):
        for k, (pcd, color) in enumerate(zip(clouds, colors)):
            p = pcd.voxel_down_sample(0.02)
            if use_T:
                p.transform(transforms[k])
            pts = np.asarray(p.points)
            ax.scatter(pts[:, 0], pts[:, 2], s=0.2, color=color, label=f"cloud_bin_{k}")
        ax.set(title=title, xlabel="x (m)", ylabel="z (m)")
        ax.set_aspect("equal")
    axes[0].legend(markerscale=20, loc="upper right")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--voxel", type=float, default=0.05, help="downsample size in m (PiDAR: 0.05)")
    parser.add_argument("--icp-threshold", type=float, default=None, help="ICP max distance in m (PiDAR: voxel*0.4)")
    parser.add_argument("--seed", type=int, default=0, help="RANSAC random seed")
    parser.add_argument("--save", help="write a before/after PNG of all three clouds")
    args = parser.parse_args()
    icp_threshold = args.icp_threshold or args.voxel * 0.4
    o3d.utility.random.seed(args.seed)
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

    data = o3d.data.DemoICPPointClouds(data_root=str(SAMPLE_DIR))
    clouds = [o3d.io.read_point_cloud(p) for p in data.paths]

    print(f"voxel {args.voxel*1000:.0f} mm, ICP threshold {icp_threshold*1000:.0f} mm, seed {args.seed}\n")
    header = (f"{'pair':6s} {'RANSAC fit':>10s} {'ICP fit':>8s} {'ICP RMSE':>9s} "
              f"{'time':>6s}  P2 pass?")
    print(header)
    print("-" * len(header))

    T = {}
    for src, tgt in ((1, 0), (2, 1), (2, 0)):
        source, target = clouds[src], clouds[tgt]
        ransac, icp, seconds = register_pair(source, target, args.voxel, icp_threshold)
        rmse_mm = icp.inlier_rmse * 1000
        ok = icp.fitness > PASS_FITNESS and rmse_mm < PASS_RMSE_MM
        print(f"{src}->{tgt:<3d} {ransac.fitness:10.3f} {icp.fitness:8.3f} {rmse_mm:7.1f}mm "
              f"{seconds:5.1f}s  {'yes' if ok else 'no'}")
        T[(src, tgt)] = icp.transformation

    rot, trans = pose_error(T[(1, 0)] @ T[(2, 1)], T[(2, 0)])
    print(f"\nLoop closure (2->1->0 vs 2->0): {rot:.2f}° rotation, {trans:.1f} mm translation")
    print("ICP fitness = share of source points within the threshold of the target; capped by how much the pair overlaps.")
    print(f"ICP RMSE only counts points within {icp_threshold*1000:.0f} mm, so it can never exceed that threshold.")

    if args.save:
        save_merged(clouds, [np.eye(4), T[(1, 0)], T[(2, 0)]], args.save)
