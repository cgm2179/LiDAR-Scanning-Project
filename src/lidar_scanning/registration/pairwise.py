"""Two-cloud registration: FPFH + RANSAC global registration, then point-to-plane ICP.

Same steps and default parameters as PiDAR's registration_test.py
(voxel 50 mm, ICP threshold = voxel * 0.4).
"""

import time

import open3d as o3d

reg = o3d.pipelines.registration


def preprocess(pcd, voxel):
    """Downsample, estimate normals and compute FPFH features."""
    down = pcd.voxel_down_sample(voxel)
    down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))
    down.orient_normals_towards_camera_location((0, 0, 0))
    fpfh = reg.compute_fpfh_feature(down, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100))
    return down, fpfh


def register_pair(source, target, voxel=0.05, icp_threshold=None, max_iteration=1_000_000, confidence=0.9):
    """Register `source` onto `target` (Open3D point clouds).

    Returns (ransac_result, icp_result, seconds). icp_result.transformation maps
    source into the target frame. Full-resolution clouds get normals estimated
    if they don't have them, since point-to-plane ICP needs target normals.
    """
    icp_threshold = icp_threshold or voxel * 0.4
    for pcd in (source, target):
        if not pcd.has_normals():
            pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel, max_nn=30))

    src_down, src_fpfh = preprocess(source, voxel)
    tgt_down, tgt_fpfh = preprocess(target, voxel)

    start = time.time()
    dist = voxel * 1.5
    ransac = reg.registration_ransac_based_on_feature_matching(
        src_down, tgt_down, src_fpfh, tgt_fpfh, True, dist,
        reg.TransformationEstimationPointToPoint(False), 4,
        [reg.CorrespondenceCheckerBasedOnEdgeLength(0.9), reg.CorrespondenceCheckerBasedOnDistance(dist)],
        reg.RANSACConvergenceCriteria(max_iteration, confidence))
    icp = reg.registration_icp(source, target, icp_threshold, ransac.transformation,
                               reg.TransformationEstimationPointToPlane())
    return ransac, icp, time.time() - start
