"""Write a synthetic three-station capture in PiDAR's format, to try P3 before the rig exists.

    python scripts/simulate_capture.py
    python scripts/stitch_capture.py data/sample/synthetic_capture/manifest.yaml

Each station folder gets planes/plane_<angle>.npy and pano.png exactly as the
rig writes them; manifest.yaml has sketch poses with ~0.2 m / 5° error, and
truth.json holds the true poses to compare against.
"""

import argparse
from pathlib import Path

from lidar_scanning.capture.simulate import simulate_capture

ROOT = Path(__file__).resolve().parents[1]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default=str(ROOT / "data" / "sample" / "synthetic_capture"))
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    truth = simulate_capture(args.out, seed=args.seed)
    print(f"Wrote {len(truth)} stations to {args.out}")
    print(f"Next: python scripts/stitch_capture.py {Path(args.out) / 'manifest.yaml'}")
