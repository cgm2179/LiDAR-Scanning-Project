"""Stitch many stations into one frame: pairwise registration + pose-graph optimisation.

1. Register every neighbour pair (ICP from sketched poses if both stations
   have one, otherwise global FPFH/RANSAC + ICP).
2. Build initial poses from a maximum-fitness spanning tree rooted at the
   reference station.
3. Optimise all poses together (Open3D pose graph); the extra pairs act as
   loop closures. Each pair's residual after optimisation shows how well the
   whole network agrees with it.
"""

from dataclasses import dataclass

import numpy as np
import open3d as o3d

from lidar_scanning.registration.pairwise import refine_pair, register_pair

reg = o3d.pipelines.registration


@dataclass
class EdgeResult:
    source: str
    target: str
    method: str
    fitness: float
    rmse_m: float
    transformation: np.ndarray      # maps source into target's frame
    in_tree: bool = False
    pruned: bool = False
    residual_rot_deg: float = float("nan")
    residual_trans_m: float = float("nan")


def pose_difference(T_a, T_b):
    """Rotation (deg) and translation (m) between two 4x4 transforms."""
    dT = np.linalg.inv(T_b) @ T_a
    angle = np.degrees(np.arccos(np.clip((np.trace(dT[:3, :3]) - 1) / 2, -1, 1)))
    return float(angle), float(np.linalg.norm(T_a[:3, 3] - T_b[:3, 3]))


def edge_warnings(edges, max_residual_m=0.02, max_residual_deg=0.5, min_move_m=0.05):
    """Plain-language warnings for pairs that look wrong.

    Fitness and RMSE can't catch these: floors and ceilings alone give a high
    fitness for any sideways slide, so a failed registration can still score well.
    """
    warnings = []
    for e in edges:
        moved = np.linalg.norm(e.transformation[:3, 3])
        if e.method.startswith("global") and moved < min_move_m:
            warnings.append(f"{e.source} → {e.target} came out as almost no movement ({moved * 1000:.0f} mm); "
                            "global registration probably failed. Add approx_pose for both stations.")
        if e.residual_trans_m > max_residual_m or e.residual_rot_deg > max_residual_deg:
            warnings.append(f"{e.source} → {e.target} disagrees with the other pairs by "
                            f"{e.residual_trans_m * 1000:.0f} mm / {e.residual_rot_deg:.1f}°"
                            f"{' and was pruned' if e.pruned else ''}; at least one registration around "
                            "these stations is wrong.")
    return warnings


def _spanning_tree(ids, edges, reference):
    """Kruskal on fitness, highest first; returns the edges used and the stations it reaches."""
    parent = {i: i for i in ids}

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    tree = []
    for e in sorted(edges, key=lambda e: -e.fitness):
        a, b = root(e.source), root(e.target)
        if a != b:
            parent[a] = b
            tree.append(e)
    reached = {i for i in ids if root(i) == root(reference)}
    return tree, reached


def _initial_poses(tree, reference):
    poses = {reference: np.eye(4)}
    remaining = list(tree)
    while remaining:
        for e in list(remaining):
            if e.target in poses and e.source not in poses:
                poses[e.source] = poses[e.target] @ e.transformation
            elif e.source in poses and e.target not in poses:
                poses[e.target] = poses[e.source] @ np.linalg.inv(e.transformation)
            else:
                continue
            remaining.remove(e)
    return poses


def stitch(clouds, edges, reference, approx_poses=None, voxel=0.05, icp_threshold=None, prune_threshold=0.25):
    """Register stations into the reference station's frame.

    clouds: {station_id: open3d PointCloud in its own station frame}
    edges: [(source_id, target_id), ...] pairs that overlap
    approx_poses: optional {station_id: 4x4 station -> building pose} from the field sketch
    Returns (poses {station_id: 4x4 station -> reference frame}, [EdgeResult]).
    """
    icp_threshold = icp_threshold or voxel * 0.4
    approx_poses = approx_poses or {}
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

    results = []
    for s, t in edges:
        if s in approx_poses and t in approx_poses:
            init = np.linalg.inv(approx_poses[t]) @ approx_poses[s]
            icp = refine_pair(clouds[s], clouds[t], init, voxel, icp_threshold)
            method = "ICP from sketch"
        else:
            _, icp, _ = register_pair(clouds[s], clouds[t], voxel, icp_threshold)
            method = "global + ICP"
        results.append(EdgeResult(s, t, method, icp.fitness, icp.inlier_rmse, np.asarray(icp.transformation)))

    ids = list(clouds)
    tree, reached = _spanning_tree(ids, results, reference)
    missing = sorted(set(ids) - reached)
    if missing:
        raise ValueError(f"stations not connected to {reference} by any neighbour pair: {missing}")
    for e in tree:
        e.in_tree = True
    initial = _initial_poses(tree, reference)

    index = {sid: k for k, sid in enumerate(ids)}
    graph = reg.PoseGraph()
    for sid in ids:
        graph.nodes.append(reg.PoseGraphNode(initial[sid]))
    for e in results:
        info = reg.get_information_matrix_from_point_clouds(clouds[e.source], clouds[e.target],
                                                            icp_threshold, e.transformation)
        graph.edges.append(reg.PoseGraphEdge(index[e.source], index[e.target], e.transformation, info,
                                             uncertain=not e.in_tree))
    reg.global_optimization(
        graph, reg.GlobalOptimizationLevenbergMarquardt(), reg.GlobalOptimizationConvergenceCriteria(),
        reg.GlobalOptimizationOption(max_correspondence_distance=icp_threshold,
                                     edge_prune_threshold=prune_threshold,
                                     preference_loop_closure=1.0,
                                     reference_node=index[reference]))

    poses = {sid: np.asarray(graph.nodes[index[sid]].pose) for sid in ids}
    kept = {(e.source_node_id, e.target_node_id) for e in graph.edges}
    for e in results:
        e.pruned = (index[e.source], index[e.target]) not in kept
        relative = np.linalg.inv(poses[e.target]) @ poses[e.source]
        e.residual_rot_deg, e.residual_trans_m = pose_difference(relative, e.transformation)
    return poses, results
