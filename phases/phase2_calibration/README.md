# Phase 2 — Calibrate and acceptance test

Brief P2 (weeks 6–7). Prove the rig measures correctly in a room whose
dimensions are taped exhaustively, **before** going to the house. The
calibration report is deliverable 2, and every later accuracy claim rests on it.

**Status:** test code written and checked on synthetic data; waiting on the rig
for real scans.

## Pass lines (brief, plausible not derived)

| Test | Method | Pass |
|---|---|---|
| Range bias | Scan a flat wall at 2, 5 and 8 m; compare to Disto | Residual after correction < 5 mm |
| Plane flatness | Fit a plane to a known-flat wall | RMS residual < 10 mm |
| Straightness | Fit a plane along a 5 m wall | Deviation end to end < 15 mm |
| Repeatability | Five scans, same station, no movement | σ of room dimension < 10 mm |
| Registration | Two stations, adjacent rooms, door open | ICP fitness > 0.8, RMSE < 20 mm |
| Room accuracy | Compare fitted dimensions to tape | Within 2 % |

Straightness is the test that catches angular calibration error, the most
likely thing to be wrong and the hardest to notice. Don't skip it.

## What's in this folder

| Path | What |
|---|---|
| `procedure.md` | Test-day checklist: setup, what to scan, what to measure |
| `session_template.yaml` | Copy per session; scan paths, wall crop boxes, Disto/tape readings |
| `sessions/` | Filled-in session files, one per calibration day (`YYYY-MM-DD.yaml`) |
| `reports/` | Generated calibration reports (`YYYY-MM-DD_report.md`) |

Scans themselves go in `data/raw/calibration/YYYY-MM-DD/` (not committed;
back them up elsewhere).

## Where the code is

| Code | Does |
|---|---|
| `src/lidar_scanning/calibration/acceptance.py` | The six tests; each returns value, pass line, residuals |
| `src/lidar_scanning/calibration/report.py` | Markdown report writer |
| `src/lidar_scanning/geometry/planes.py` | Plane fit, RANSAC trim, crop box, wall separation |
| `scripts/run_calibration.py` | Runs a session file, writes the report |
| `tests/test_calibration.py` | Synthetic-wall checks for every test |

## Run a session

```bash
conda activate lidar
cp phases/phase2_calibration/session_template.yaml phases/phase2_calibration/sessions/2026-11-XX.yaml
# fill in scan paths, crop boxes, Disto/tape readings
python scripts/run_calibration.py phases/phase2_calibration/sessions/2026-11-XX.yaml
```

The report lands in `reports/` named by the session date. Tests whose inputs
are still blank are listed as "not run", so you can run it after each test.

## Open decisions — `TBD (Cameron)`

- Which room is the calibration room, and who tapes it.
- Disto: the BOM has none. Borrow, buy, or substitute?
- Range-bias correction model: `offset` (default) or `linear`. A 0.15 % scale
  error passed under `offset` in the synthetic run, just inside 5 mm.
- Straightness bin size (default 100 mm); flatness clutter trim (default off).
- Your own pass lines, replacing the brief's, in `config/pipeline.yaml`.
- Registration RMSE line can't fail with a 20 mm ICP threshold; what to use
  instead (taped features across the doorway, loop closure).

## Exit

Every test passes, results written up as the calibration report. Only then
move to P3 (capture the house).
