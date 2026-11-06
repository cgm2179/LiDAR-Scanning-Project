"""One station's LiDAR sweep: step the rig, record a full LiDAR revolution at each
stepper angle, save each as a PiDAR-format plane file.

Stop-and-scan: the stepper holds still while a whole revolution is recorded,
so every point in a plane file really was taken at that file's angle. (PiDAR
keeps stepping while it reads and tags each batch with the angle after the
move.) Output is what capture/station.py reads: <out_dir>/plane_<angle>.npy.
"""

import json
import time
from pathlib import Path

import numpy as np

from lidar_scanning.rig.stl27l import to_pidar_points


def plane_filename(angle_deg):
    return f"plane_{round(float(angle_deg), 2)}.npy"


def capture_planes(reader, move_to, angles_deg, out_dir, settle_s=0.05):
    """Record one revolution per stepper angle.

    reader: STL27LReader (or anything with flush() and read_revolution())
    move_to: callable(angle_deg) that moves the stepper and returns once it's there
    Writes plane files plus capture_log.json (packet counts, CRC errors,
    skipped bytes, estimated dropped packets) and returns the log dict.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    started = time.time()
    planes = []
    for angle in angles_deg:
        move_to(angle)
        time.sleep(settle_s)
        reader.flush()
        dropped_before = reader.dropped
        packets = reader.read_revolution()
        np.save(out_dir / plane_filename(angle), to_pidar_points(packets))
        planes.append({"angle_deg": float(angle), "packets": len(packets),
                       "dropped": reader.dropped - dropped_before,
                       "speed_rps": float(np.median(packets.speed_dps) / 360) if len(packets) else 0.0})

    stats = reader.parser.stats
    log = {"planes": len(planes), "seconds": round(time.time() - started, 1),
           "packets": stats.packets, "crc_errors": stats.crc_errors,
           "skipped_bytes": stats.skipped_bytes, "dropped_estimate": reader.dropped,
           "per_plane": planes}
    (out_dir / "capture_log.json").write_text(json.dumps(log, indent=1))
    return log
