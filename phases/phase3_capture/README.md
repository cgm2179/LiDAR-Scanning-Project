# Phase 3 — Capture the house

Brief P3 (week 8). Scan the whole house, roughly 19–25 stations in half a day,
and stitch them into one registered point cloud: the input to P4.

**Status:** capture and stitching code written and checked on synthetic
PiDAR-format captures; waiting on the rig. Do not capture the house until
every P2 test passes.

## What the code accepts

Exactly what PiDAR writes on the Pi, per station:

| Sensor | File | Contents |
|---|---|---|
| LiDAR (STL-27L) | `planes/plane_<stepper angle>.npy` (or `.csv`) | one file per stepper position; rows `[x_mm, y_mm, luminance]` in the LiDAR's vertical spinning plane; zero distance = no return |
| Camera (fisheye + Hugin) | `pano.jpg` / `.png` / `.tif` | equirectangular 2:1 panorama stitched from the fisheye photos |

The plane files can also sit directly in the station folder (PiDAR's
`DATA_DIR`). The rig calibration from P2 (`config/rig.yaml`, template
`config/rig.example.yaml`) turns them into metres: LiDAR offset from the
axis, stepper angle scale, range correction, panorama yaw, camera offset.

## Run it

```bash
conda activate lidar
# right after each scan, check one station (works on the Pi too):
python scripts/assemble_station.py data/raw/capture/2026-11-20/S01
# after the day, stitch everything in the manifest:
python scripts/stitch_capture.py phases/phase3_capture/captures/2026-11-20.yaml
```

Outputs in `data/registered/<capture id>/` (not committed): `registered.e57`
(each station with its pose; P4 input), `merged_preview.ply`, `poses.json`.
The report goes to `reports/<capture id>_stitch_report.md`.

Try it now without the rig:

```bash
python scripts/simulate_capture.py
python scripts/stitch_capture.py data/sample/synthetic_capture/manifest.yaml
```

## How stitching works

1. Assemble each station from its planes + panorama (`capture/station.py`).
2. Register every neighbour pair from the manifest: ICP starting from the
   field-sketch poses if both stations have one, otherwise global FPFH/RANSAC + ICP.
3. Build starting poses from the best-fitting pairs (spanning tree from the
   reference station), then optimise all poses together; extra pairs act as
   loop closures (`registration/multistation.py`).
4. Report each pair's residual against the optimised network, and warn about
   pairs that look wrong.

## What the synthetic tests showed (2026-10-29)

- With sketch poses off by ~0.2 m / 5°, three stations came back within
  0.1 mm / 0.003° of the truth.
- **Without** sketch poses, global registration failed in the plain box room:
  floors and ceilings matched while walls didn't, a station landed 3 m off, and
  ICP still reported fitness 0.60 / RMSE 5.5 mm. The pose graph pruned the
  contradicting pair but couldn't fix the others. The report now warns when this
  happens, but the real fix is a rough position and heading for every station.

## Checked here vs later

| Check | Where |
|---|---|
| Floor-to-ceiling near each station vs Disto | stitch report (assumes the rig was levelled) |
| Pair consistency / failed registrations | stitch report, Warnings |
| Disto wall lengths | needs room geometry: P4 |
| Floor closure (summed rooms + walls vs exterior within 1 %, the P3 exit) | needs room geometry: P4 |

## Open decisions — `TBD (Cameron)`

- Station density and overlap: one per room plus doorway stations? The
  synthetic stations in one room overlapped 42–82 %.
- Registration strategy: sketch poses (recommended by the test above), targets,
  or surveyed control points.
- How to record sketch poses on site: tape from two walls + heading of stepper
  zero (compass? a mark on the floor?).
- Scanning order (brief: exterior first for the closure check).
- Spaces reached only from outside, or not at all: how they connect into the
  network, and how P4 represents a boundary that was never observed.
- Exterior station alignment: there's no shared overlap with interiors except
  through open doors and windows.
