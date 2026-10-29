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


def write_e57(path, scans):
    """Write several scans to one E57, each in its own frame with its registered pose.

    scans: list of dicts with "name", "points" (N x 3, station frame), "pose"
    (4x4 station -> registered frame), optional "colors" (N x 3 in [0, 1]) and
    "intensity" (N). Readers that apply poses (read_points, CloudCompare) see
    every station in the registered frame.
    """
    import pye57
    from scipy.spatial.transform import Rotation

    e57 = pye57.E57(str(path), mode="w")
    try:
        for scan in scans:
            pts = np.asarray(scan["points"], float)
            data = {"cartesianX": pts[:, 0], "cartesianY": pts[:, 1], "cartesianZ": pts[:, 2]}
            if scan.get("colors") is not None:
                rgb = np.clip(np.round(np.asarray(scan["colors"]) * 255), 0, 255).astype(np.uint8)
                data.update(colorRed=rgb[:, 0], colorGreen=rgb[:, 1], colorBlue=rgb[:, 2])
            if scan.get("intensity") is not None:
                data["intensity"] = np.asarray(scan["intensity"], float)
            pose = np.asarray(scan["pose"], float)
            x, y, z, w = Rotation.from_matrix(pose[:3, :3]).as_quat()
            e57.write_scan_raw(data, name=scan["name"], rotation=np.array([w, x, y, z]),
                               translation=pose[:3, 3])
    finally:
        e57.close()
