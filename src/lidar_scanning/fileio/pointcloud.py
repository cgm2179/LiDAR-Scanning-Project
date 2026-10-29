"""Read point clouds into plain numpy arrays (N x 3, metres)."""

from pathlib import Path

import numpy as np


def read_points(path):
    """Load XYZ from .ply, .pcd, .xyz, .e57 or .npy.

    E57 files with several scans are merged, each placed by its own pose.
    .npy files keep their first three columns.
    """
    path = Path(path)
    suffix = path.suffix.lower()
    if suffix == ".e57":
        import pye57

        e57 = pye57.E57(str(path))
        scans = [e57.read_scan(i, ignore_missing_fields=True) for i in range(e57.scan_count)]
        pts = np.vstack([np.column_stack([s["cartesianX"], s["cartesianY"], s["cartesianZ"]]) for s in scans])
    elif suffix == ".npy":
        pts = np.load(path)[:, :3]
    else:
        import open3d as o3d

        pts = np.asarray(o3d.io.read_point_cloud(str(path)).points)
    if len(pts) == 0:
        raise ValueError(f"no points read from {path}")
    return np.asarray(pts, float)
