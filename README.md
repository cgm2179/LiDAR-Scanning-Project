# LiDAR Scanning Project

A tripod-mounted LiDAR + camera scanning station (Raspberry Pi, based on
[PiDAR](https://github.com/VoxoTech/PiDAR)) and a processing pipeline that turns
its point clouds into an EnergyPlus model of a small building, with no manual
entry of dimensions. Independent study, Columbia BERL.

## Layout

```
src/lidar_scanning/
  rig/           P1  Raspberry Pi scanner (STL-27L lidar, stepper, camera)
  calibration/   P2  acceptance tests
  registration/  P3  multi-station alignment, E57
  geometry/      P4  planes -> walls, rooms, openings
  schema/        P4  building geometry JSON
  energyplus/    P5  schema -> Honeybee -> IDF -> EnergyPlus
  validation/    P6  scan vs tape survey, energy-impact comparison
  thermal/       extension, deferred
  fileio/        shared readers/writers
phases/          one folder per phase: guide, procedures, sessions, reports
  phase2_calibration/
tests/           regression test (floor area within 2%)
scripts/         check_environment.py and command-line tools
config/          pipeline settings (copy the .example file)
data/            scans, not committed
docs/            decision log, calibration notes
third_party/     PiDAR clone, not committed
```

## Setup (laptop)

```bash
conda env create -f environment.yml
conda activate lidar
python scripts/check_environment.py
```

## Setup (Raspberry Pi)

```bash
sudo apt install -y python3-picamera2 hugin-tools
python3 -m venv --system-site-packages .venv && . .venv/bin/activate
pip install -r requirements-pi.txt && pip install -e .
python scripts/check_environment.py --pi
```

Current status: see [CurrentProjectCheckpoint.md](CurrentProjectCheckpoint.md).
