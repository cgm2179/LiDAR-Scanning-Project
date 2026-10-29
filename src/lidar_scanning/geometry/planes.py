"""Plane fitting and point-to-plane measurements.

Shared by calibration (P2: flatness, straightness, range bias) and the
geometry pipeline (P4: wall planes). All units are metres.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Plane:
    """Plane n·x + d = 0 with unit normal n, plus the centroid of the fitted points."""

    normal: np.ndarray
    d: float
    centroid: np.ndarray

    def signed_distances(self, points):
        return np.asarray(points, float) @ self.normal + self.d

    def distance_to(self, point):
        return abs(float(np.dot(self.normal, point) + self.d))


def fit_plane(points):
    """Least-squares plane through `points` (N x 3) by SVD."""
    pts = np.asarray(points, float)
    if len(pts) < 3:
        raise ValueError(f"need at least 3 points to fit a plane, got {len(pts)}")
    centroid = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - centroid, full_matrices=False)
    normal = vt[-1]
    return Plane(normal, -float(normal @ centroid), centroid)


def fit_plane_ransac(points, threshold, iterations=1000, seed=0):
    """RANSAC plane (Open3D) refitted by least squares on its inliers.

    Returns (plane, inlier_mask). Use it to drop clutter and stray returns
    before measuring a wall.
    """
    import open3d as o3d

    pts = np.asarray(points, float)
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    o3d.utility.random.seed(seed)
    _, inliers = pcd.segment_plane(threshold, 3, iterations)
    mask = np.zeros(len(pts), bool)
    mask[inliers] = True
    return fit_plane(pts[mask]), mask


def crop_box(points, box_min, box_max):
    """Points inside an axis-aligned box [box_min, box_max]."""
    pts = np.asarray(points, float)
    inside = np.all((pts >= np.asarray(box_min)) & (pts <= np.asarray(box_max)), axis=1)
    return pts[inside]


def principal_in_plane_axis(points, plane):
    """Unit vector along the longest in-plane extent of `points` (the wall's length)."""
    pts = np.asarray(points, float) - plane.centroid
    in_plane = pts - np.outer(pts @ plane.normal, plane.normal)
    _, _, vt = np.linalg.svd(in_plane, full_matrices=False)
    return vt[0]


def plane_separation(plane_a, plane_b):
    """Distance between two near-parallel planes and the angle between them (deg).

    Distance is the mean of each centroid's distance to the other plane, so a
    small tilt doesn't bias it toward one side.
    """
    cos = abs(float(plane_a.normal @ plane_b.normal))
    angle = float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))
    separation = 0.5 * (plane_a.distance_to(plane_b.centroid) + plane_b.distance_to(plane_a.centroid))
    return separation, angle
