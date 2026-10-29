"""P2 acceptance tests (brief Section 5, P2 table).

Each test takes measured data (cropped wall points in metres, Disto/tape
readings in metres) and returns an AcceptanceResult holding the measured
value, the pass line, and the residuals behind it, so the calibration report
shows the disagreement, not just pass/fail.

Pass lines default to the brief's values (LIMITS). They are plausible, not
derived; override them from config/pipeline.yaml once you decide your own.
"""

from dataclasses import dataclass, field

import numpy as np

from lidar_scanning.geometry.planes import (
    fit_plane,
    fit_plane_ransac,
    plane_separation,
    principal_in_plane_axis,
)

LIMITS = {
    "range_bias_residual_mm": 5,
    "plane_rms_mm": 10,
    "straightness_mm": 15,
    "repeatability_sigma_mm": 10,
    "icp_fitness_min": 0.8,
    "icp_rmse_mm": 20,
    "room_accuracy_pct": 2,
}


@dataclass
class AcceptanceResult:
    test: str
    value: float
    unit: str
    limit: float
    passed: bool
    method: str
    residuals: dict = field(default_factory=dict)
    notes: list = field(default_factory=list)


def range_bias(walls, model="offset", limit_mm=LIMITS["range_bias_residual_mm"]):
    """Range bias: scanned distance to a flat wall vs Disto, at several ranges.

    walls: list of (wall_points, disto_m). Scan distance is the perpendicular
    distance from the scan origin (rotation axis) to the fitted wall plane, so
    the Disto reading must also be taken from the rig's axis, perpendicular to
    the wall.

    model: "offset" corrects by the mean bias; "linear" fits
    disto = scale * scan + offset (needs 3+ walls to leave a residual).
    Pass if every residual after correction is under limit_mm.
    """
    scan = np.array([fit_plane(pts).distance_to(np.zeros(3)) for pts, _ in walls])
    disto = np.array([d for _, d in walls], float)
    bias = scan - disto
    notes = []

    if model == "offset":
        offset = bias.mean()
        corrected = scan - offset
        notes.append(f"Correction: subtract {offset * 1000:+.1f} mm from every range.")
    elif model == "linear":
        scale, offset = np.polyfit(scan, disto, 1)
        corrected = scale * scan + offset
        notes.append(f"Correction: true = {scale:.5f} x scan {offset * 1000:+.1f} mm.")
        if len(walls) < 3:
            notes.append("Linear model with fewer than 3 walls fits exactly; residuals are meaningless.")
    else:
        raise ValueError(f"model must be 'offset' or 'linear', got {model!r}")

    after = corrected - disto
    residuals = {}
    for d, b, a in zip(disto, bias, after):
        residuals[f"{d:.2f} m raw bias (mm)"] = b * 1000
        residuals[f"{d:.2f} m after correction (mm)"] = a * 1000
    worst = float(np.abs(after).max() * 1000)
    return AcceptanceResult("Range bias", worst, "mm", limit_mm, worst < limit_mm,
                            f"Plane fit to a flat wall at {len(walls)} ranges vs Disto; {model} correction.",
                            residuals, notes)


def plane_flatness(points, limit_mm=LIMITS["plane_rms_mm"], trim_threshold_m=None):
    """Plane flatness: RMS distance of a known-flat wall's points to their best-fit plane.

    trim_threshold_m: if set, RANSAC first keeps only points within this
    distance of the dominant plane (drops clutter, edges, stray returns).
    """
    pts = np.asarray(points, float)
    notes = []
    if trim_threshold_m:
        plane, mask = fit_plane_ransac(pts, trim_threshold_m)
        notes.append(f"RANSAC trim at {trim_threshold_m * 1000:.0f} mm kept {mask.sum():,} of {len(pts):,} points.")
        pts = pts[mask]
    else:
        plane = fit_plane(pts)
    r = plane.signed_distances(pts)
    rms = float(np.sqrt(np.mean(r ** 2)) * 1000)
    residuals = {
        "RMS (mm)": rms,
        "95th percentile |residual| (mm)": float(np.percentile(np.abs(r), 95) * 1000),
        "max |residual| (mm)": float(np.abs(r).max() * 1000),
        "points": len(pts),
    }
    return AcceptanceResult("Plane flatness", rms, "mm", limit_mm, rms < limit_mm,
                            "Least-squares plane through a known-flat wall; RMS of point-to-plane residuals.",
                            residuals, notes)


def straightness(points, limit_mm=LIMITS["straightness_mm"], bin_m=0.1, min_points_per_bin=20):
    """Straightness: bow of a long (~5 m) flat wall, peak to valley.

    Fits one plane to the whole wall, then averages the residuals in bins along
    the wall's length. A flat wall should give a flat profile; angular
    calibration error bends it. The value is max - min of the binned profile.
    """
    pts = np.asarray(points, float)
    plane = fit_plane(pts)
    r = plane.signed_distances(pts)
    s = (pts - plane.centroid) @ principal_in_plane_axis(pts, plane)
    length = float(s.max() - s.min())

    edges = np.arange(s.min(), s.max() + bin_m, bin_m)
    idx = np.digitize(s, edges) - 1
    centres, means = [], []
    for b in range(len(edges) - 1):
        in_bin = idx == b
        if in_bin.sum() >= min_points_per_bin:
            centres.append(edges[b] + bin_m / 2 - s.min())
            means.append(r[in_bin].mean())
    if len(means) < 2:
        raise ValueError("too few populated bins along the wall; lower bin_m or min_points_per_bin")
    means = np.array(means)
    p2v = float((means.max() - means.min()) * 1000)

    peak, valley = int(means.argmax()), int(means.argmin())
    residuals = {
        f"peak at {centres[peak]:.2f} m (mm)": means[peak] * 1000,
        f"valley at {centres[valley]:.2f} m (mm)": means[valley] * 1000,
    }
    step = max(1, round(0.5 / bin_m))  # profile roughly every 0.5 m, plus the far end
    for k in sorted(set(range(0, len(means), step)) | {len(means) - 1}):
        residuals[f"profile at {centres[k]:.2f} m (mm)"] = means[k] * 1000
    notes = [f"Wall length measured: {length:.2f} m, {len(means)} bins of {bin_m * 1000:.0f} mm."]
    if length < 4.5:
        notes.append("Wall is shorter than the brief's 5 m; angular error shows less over a short wall.")
    return AcceptanceResult("Straightness", p2v, "mm", limit_mm, p2v < limit_mm,
                            "Plane fit along a long wall; peak-to-valley of the binned residual profile.",
                            residuals, notes)


def room_dimension(wall_a_points, wall_b_points):
    """Distance between two opposite walls (m) and the angle between them (deg)."""
    return plane_separation(fit_plane(wall_a_points), fit_plane(wall_b_points))


def repeatability(dimensions_m, limit_mm=LIMITS["repeatability_sigma_mm"]):
    """Repeatability: sample standard deviation of one room dimension over repeat scans
    from the same station without moving the rig."""
    dims = np.asarray(dimensions_m, float)
    if len(dims) < 2:
        raise ValueError("need at least 2 repeat scans")
    sigma = float(dims.std(ddof=1) * 1000)
    residuals = {f"scan {k + 1} minus mean (mm)": (d - dims.mean()) * 1000 for k, d in enumerate(dims)}
    residuals["mean dimension (mm)"] = float(dims.mean() * 1000)
    notes = [] if len(dims) == 5 else [f"{len(dims)} scans; the brief specifies 5."]
    return AcceptanceResult("Repeatability", sigma, "mm", limit_mm, sigma < limit_mm,
                            "Same room dimension from repeat scans at one station; sample σ.",
                            residuals, notes)


def registration(fitness, inlier_rmse_m, icp_threshold_m,
                 fitness_min=LIMITS["icp_fitness_min"], rmse_limit_mm=LIMITS["icp_rmse_mm"]):
    """Registration: two stations in adjacent rooms, door open. Passes on ICP fitness and RMSE."""
    rmse_mm = inlier_rmse_m * 1000
    residuals = {"ICP fitness": fitness, "ICP inlier RMSE (mm)": rmse_mm,
                 "ICP threshold (mm)": icp_threshold_m * 1000}
    notes = ["ICP fitness is the share of source points within the threshold of the target, "
             "so it also measures how much the two stations overlap."]
    if round(icp_threshold_m * 1000, 6) <= rmse_limit_mm:  # 0.05 * 0.4 m is 20.000000000000004 mm
        notes.append(f"RMSE only counts points within the {icp_threshold_m * 1000:.0f} mm threshold, so it cannot "
                     f"exceed the {rmse_limit_mm} mm pass line. Check accuracy with taped features or loop closure.")
    passed = fitness > fitness_min and rmse_mm < rmse_limit_mm
    return AcceptanceResult("Registration", fitness, "fitness", fitness_min, passed,
                            f"Global registration + point-to-plane ICP; pass needs fitness > {fitness_min} "
                            f"and RMSE < {rmse_limit_mm} mm.",
                            residuals, notes)


def room_accuracy(dimensions, limit_pct=LIMITS["room_accuracy_pct"]):
    """Room accuracy: fitted room dimensions vs tape.

    dimensions: {name: (scan_m, tape_m)}. Pass if every dimension is within limit_pct.
    """
    residuals = {}
    worst = 0.0
    for name, (scan_m, tape_m) in dimensions.items():
        pct = (scan_m - tape_m) / tape_m * 100
        residuals[f"{name} scan - tape (mm)"] = (scan_m - tape_m) * 1000
        residuals[f"{name} scan - tape (%)"] = pct
        worst = max(worst, abs(pct))
    return AcceptanceResult("Room accuracy", worst, "%", limit_pct, worst < limit_pct,
                            "Distance between fitted opposite wall planes vs tape; worst dimension.",
                            residuals)
