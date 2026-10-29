"""Rig calibration: the numbers that turn PiDAR's raw output into metres.

Values come from P2 calibration. The defaults reproduce PiDAR's own assembly,
so uncalibrated captures still load.

Station frame: origin on the stepper's rotation axis at LiDAR height, z up
along the axis, x/y fixed by stepper angle 0. Level the rig and z is gravity.
"""

from dataclasses import dataclass, fields
from pathlib import Path

import yaml


@dataclass
class RigCalibration:
    # LiDAR optical centre relative to the rotation axis, in the LiDAR-plane
    # frame before the stepper rotation (x along the plane, y across it, z up).
    lidar_offset_m: tuple = (0.0, 0.0, 0.0)
    # Stepper angle -> rotation about z. PiDAR rotates each plane by -angle.
    rotation_sign: int = -1
    angle_scale: float = 1.0        # achieved / commanded angle (check the gearing in P2)
    angle_offset_deg: float = 0.0
    # P2 range-bias correction: true range = range_scale * measured + range_offset_m
    range_scale: float = 1.0
    range_offset_m: float = 0.0
    min_range_m: float = 0.05       # drops zero (no-return) points and the rig itself
    max_range_m: float = 25.0       # STL-27L rated range
    min_intensity: float = 0.0
    # Panorama: PiDAR's z_rotate (pano heading vs stepper angle 0) and the
    # camera nodal point in the station frame (it sits on the axis, above or below the LiDAR).
    pano_yaw_deg: float = 0.0
    camera_offset_m: tuple = (0.0, 0.0, 0.0)


def load_rig(path=None):
    """RigCalibration from a YAML file (see config/rig.example.yaml); defaults if path is None."""
    if path is None:
        return RigCalibration()
    values = yaml.safe_load(Path(path).read_text()) or {}
    known = {f.name for f in fields(RigCalibration)}
    unknown = set(values) - known
    if unknown:
        raise ValueError(f"unknown rig settings in {path}: {sorted(unknown)}")
    for key in ("lidar_offset_m", "camera_offset_m"):
        if key in values:
            values[key] = tuple(float(v) for v in values[key])
    return RigCalibration(**values)
