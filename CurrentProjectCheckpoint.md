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
| P2 | Calibration + acceptance tests | Test code written and checked on synthetic data; needs real scans |
| P3 | Capture the house (~19–25 stations) | Capture + stitching code written, checked on synthetic captures; needs rig |
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

- **2026-10-29:** switched to a pull-request workflow: all changes go on a
  branch and into `main` by PR (AGENTS.md §1a).
- **2026-10-29:** P2 calibration code (branch `feature/calibration-tests`):
  - `calibration/acceptance.py`: the brief's six tests (range bias, plane
    flatness, straightness, repeatability, registration, room accuracy), each
    returning value, pass line and residuals.
  - `calibration/report.py`: Markdown calibration report.
  - `phases/phase2_calibration/`: Phase 2 home: README (pass lines, open
    decisions), `procedure.md` test-day checklist, `session_template.yaml`,
    `sessions/` and `reports/`.
  - `scripts/run_calibration.py` + `session_template.yaml`:
    fill in scan paths, crop boxes and Disto/tape readings, get the report.
  - Shared pieces: `geometry/planes.py` (plane fit, RANSAC trim, crop box,
    wall separation), `fileio/pointcloud.py` (PLY/PCD/E57/NPY),
    `registration/pairwise.py` (PiDAR-style RANSAC + ICP).
  - `tests/test_calibration.py`: 15 pytest checks that plant known flaws in
    synthetic walls (bias, scale error, noise, bow, clutter) and confirm each
    test measures them. A full synthetic session recovered a 6 mm bow as
    6.5 mm and a 12 mm length error as 12.1 mm.
  - Open choices, defaults in code, all `TBD (Cameron)`: range correction
    model (`offset` vs `linear`), straightness bin size (100 mm), flatness trim
    threshold (off), the pass lines themselves (brief values).
- **2026-10-29:** language decision (Cameron): stay in Python; C++ only for
  the Pi's LiDAR serial reader if a packet-loss test shows it's needed.
- **2026-10-29:** P3 capture + stitching code (branch `feature/p3-capture-stitching`):
  - `capture/`: rig calibration (`config/rig.example.yaml`), station assembly
    from PiDAR's exact output (`plane_<angle>.npy|csv` + Hugin `pano`),
    manifest, floor-to-ceiling check, and a simulator that writes synthetic
    captures in PiDAR's format.
  - `registration/multistation.py`: pairwise registration (ICP from field
    sketch poses, or global) + pose-graph optimisation + failure warnings.
  - Scripts: `assemble_station.py`, `stitch_capture.py` (→ registered E57 +
    report), `simulate_capture.py`. Phase folder `phases/phase3_capture/`.
  - Synthetic results: with sketch poses (~0.2 m / 5° error) three stations
    came back within 0.1 mm / 0.003°. **Without** sketch poses global
    registration failed in a plain room (station 3 m off, fitness still 0.60);
    warnings now flag this. Sketch poses look necessary: `TBD (Cameron)`.
  - Reviewed two pasted snippets (camera-overlay demo, C++ STL-27L driver);
    neither used; the driver's problems are listed under Open questions.

## Next steps

1. ~~Create the conda env and pass the import check~~ (done 2026-10-29).
   Use `conda activate lidar` for all laptop work.
2. ~~Clone PiDAR and load its sample point clouds~~ (done 2026-10-29).
   PiDAR (commit `2c7e6af`, 2024-03-12) is in `third_party/PiDAR`. It ships no
   scans; its tests use Open3D's DemoICPPointClouds (3 overlapping indoor RGB-D
   fragments, ~140–200k points each, not gravity-aligned, not lidar data).
   `python scripts/load_sample_clouds.py` downloads them to `data/sample/` and
   shows them.
   **Registration tried 2026-10-29** (`scripts/try_registration.py`, PiDAR
   settings: 50 mm voxel, FPFH, RANSAC, point-to-plane ICP at 20 mm). Same
   result for seeds 0–2:

   | pair | ICP fitness | ICP RMSE | P2 pass (>0.8, <20 mm) |
   |---|---|---|---|
   | 1→0 | 0.936 | 6.4 mm | yes |
   | 2→1 | 0.540 | 7.6 mm | no |
   | 2→0 | 0.771 | 6.8 mm | no |

   Loop closure (2→1→0 vs 2→0): 0.33° and 19.8 mm. All pairs visibly align.
   Findings for the P2 test design (`TBD (Cameron)` how to act on them):
   - ICP fitness measures overlap as much as alignment quality; the pairs that
     "fail" overlap less. Station overlap decides whether >0.8 is reachable.
   - ICP RMSE only counts points within the ICP threshold, so with a 20 mm
     threshold it can never exceed 20 mm; the RMSE pass line is automatic.
   - Loop closure (or taped references) is a better accuracy check; ~20 mm
     here on one room already approaches the 25 mm project tolerance.
   - The dataset's `init.log` is an initial guess, not ground truth.
3. Parts order status: `TBD (Cameron)` — ordered? LiDAR lead time / US supplier?
4. Start `fileio/` (read PCD/PLY/E57) and a first `geometry/` RANSAC plane
   segmentation notebook on a sample cloud.
5. Pick a fixture scan and expected floor area so `tests/test_regression.py`
   can stop being skipped.
6. Start the calibration room plan for P2 (which room, taped dimensions, Disto
   availability — the BOM currently has no Disto). When the rig scans, copy
   `phases/phase2_calibration/session_template.yaml` into `sessions/`, fill it
   in, and run `python scripts/run_calibration.py <session.yaml>`
   (checklist: `phases/phase2_calibration/procedure.md`).

## Open questions

- Pi LiDAR reader: measure packet loss with PiDAR's byte-by-byte reader at
  921600 baud once the rig runs. A pasted C++ driver (2026-10-29) is not usable
  as is: 2° per-point angle step instead of interpolating to the end angle, no
  CRC check, packets split across reads are dropped, serial port not in raw mode.

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
