# Current Project Checkpoint

**Last updated:** 2026-10-29
**Current phase:** end of P0 (setup and planning) → starting P1 (build the rig)
**Code status:** repo scaffold only; no pipeline code written yet

Items marked `TBD (Cameron)` are decisions or facts only Cameron can supply.

## Phase tracker

| Phase | What | Status |
|---|---|---|
| P0 | Plan, read PiDAR + STL-27L manual, Python/Open3D setup, order parts | Mostly done (see below) |
| P1 | Build the rig; one-button scan → E57 | Not started — waiting on parts |
| P2 | Calibration + acceptance tests | Not started |
| P3 | Capture the house (~19–25 stations) | Not started |
| P4 | Point cloud → geometry JSON (core of the project) | Not started |
| P5 | Geometry JSON → EnergyPlus | Not started |
| P6 | Validation + write-up | Not started |
| Ext. | Thermal imaging | Deferred until P5 is done |

## Done so far

- **Sep–Oct 2026:** Project plan drafted (working draft v0.2, Oct 5). Explainer
  decks on LiDAR/Open3D (Oct 6) and on scanning solutions (Oct 22). Bill of
  materials for LiDAR + thermal built.
- **2026-10-15 mentor meeting:** presented hardware + Open3D pipeline idea.
  Mentor: order parts now. Follow-up questions assigned (see Open questions).
- **2026-10-22:** received the professor's drawing set for the test building and
  wrote full notes (kept outside this repo). Key facts: ~22'×34' footprint, two
  floors + basement + attic, all windows replaced with 4 known double-hung
  sizes, basement reached only by an exterior bulkhead (registration risk),
  heavy HVAC equipment in every room (clutter for scanning). Drawing
  dimensions are approximate, not validation ground truth.
- **2026-10-29:** created this repo: package layout per phase, requirements for
  laptop and Pi, `scripts/check_environment.py`, regression-test stub, decision
  log, agent hand-off files, no-AI-attribution commit hook.
- **2026-10-29:** created conda env `lidar` (Python 3.11.18) on Cameron's Mac;
  every import in `check_environment.py` passes (Open3D 0.20.0, honeybee-energy
  1.127.2). Open3D RANSAC and a Honeybee box room both ran in a smoke test.
  EnergyPlus 26.1.0 is installed at `/usr/local/bin/energyplus`; Honeybee has
  not yet been run against it end to end.

## Next steps

1. ~~Create the conda env and pass the import check~~ (done 2026-10-29).
   Use `conda activate lidar` for all laptop work.
2. Clone PiDAR into `third_party/PiDAR` and load its sample point clouds in
   Open3D (P0 exit item; lets P4 work start before the lidar arrives).
3. Parts order status: `TBD (Cameron)` — ordered? LiDAR lead time / US supplier?
4. Start `fileio/` (read PCD/PLY/E57) and a first `geometry/` RANSAC plane
   segmentation notebook on a sample cloud.
5. Pick a fixture scan and expected floor area so `tests/test_regression.py`
   can stop being skipped.
6. Start the calibration room plan for P2 (which room, taped dimensions, Disto
   availability — the BOM currently has no Disto).

## Open questions (from 2026-10-15 meeting)

- Point density / accuracy needed for a house-sized room: hand calc or only
  from calibration?
- Station positions across rooms and floors: registration gives xyz; tape/Disto
  is for verification. Still open: station overlap, registration strategy
  (features vs targets vs surveyed points).
- Windows, glass, mirrors: density voids + multi-echo returns + camera images;
  matte spray option makes glass a recessed rectangle instead of a gap.
- Has the renovation been built yet? Decides which geometry the scan captures.
  `TBD (Cameron)`
- US supplier for the STL-27L; coordinate order with Ryan. `TBD (Cameron)`

## Open decisions (brief Section 10) — all `TBD (Cameron)`

Pi 4 vs Pi 5 · rotation mechanism · battery strategy · thermal extension yes/no ·
station density · registration strategy · precedence rule (scan vs tape) ·
tolerance thresholds · RANSAC tuning approach · opening detection approach ·
single-face wall thickness · schema fields · zone granularity · attic/basement
representation · constructions source · what gets cut if late · thesis
contribution.
