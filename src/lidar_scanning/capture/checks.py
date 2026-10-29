"""Per-station checks of the scan against the Disto readings taken at that station."""

import numpy as np

from lidar_scanning.geometry.planes import fit_plane_ransac, plane_separation


def _horizontal_plane(points, threshold_m, max_tilt_deg, attempts=4):
    """First RANSAC plane within max_tilt_deg of horizontal (skips walls that RANSAC finds first)."""
    pts = points
    for _ in range(attempts):
        if len(pts) < 100:
            return None
        plane, mask = fit_plane_ransac(pts, threshold_m)
        tilt = np.degrees(np.arccos(min(1.0, abs(plane.normal[2]))))
        if tilt <= max_tilt_deg:
            return plane, tilt
        pts = pts[~mask]
    return None


def floor_to_ceiling(points, radius_m=1.5, threshold_m=0.02, max_tilt_deg=5.0, min_height_m=0.3):
    """Floor-to-ceiling height near the station, to compare with the Disto reading.

    Uses points within radius_m (horizontally) of the station, fits the floor
    below and the ceiling above. Assumes the rig was levelled (z = up).
    Returns {"height_m", "floor_tilt_deg", "ceiling_tilt_deg"} or None if
    either surface isn't found.
    """
    pts = np.asarray(points, float)
    near = pts[np.hypot(pts[:, 0], pts[:, 1]) <= radius_m]
    floor = _horizontal_plane(near[near[:, 2] < -min_height_m], threshold_m, max_tilt_deg)
    ceiling = _horizontal_plane(near[near[:, 2] > min_height_m], threshold_m, max_tilt_deg)
    if floor is None or ceiling is None:
        return None
    height, _ = plane_separation(floor[0], ceiling[0])
    return {"height_m": height, "floor_tilt_deg": floor[1], "ceiling_tilt_deg": ceiling[1]}
