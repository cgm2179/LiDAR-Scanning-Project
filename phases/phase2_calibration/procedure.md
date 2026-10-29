# Phase 2 test-day procedure

Working checklist for one calibration session. Items marked `TBD (Cameron)`
are yours to decide before the first session.

## Before the day

- [ ] Calibration room chosen: `TBD (Cameron)`. Needs a flat wall reachable at
      2, 5 and 8 m, a ~5 m flat wall, and an adjacent room through a door.
- [ ] Room taped exhaustively: every wall length, both diagonals,
      floor-to-ceiling in each corner. Record in the session file's `tape_m`.
- [ ] Disto available: `TBD (Cameron)` (not on the BOM).
- [ ] Rig charged, PiDAR commit noted, stepper gearing noted (`rig:` field).
- [ ] Copy `session_template.yaml` to `sessions/YYYY-MM-DD.yaml`.

## Every station

- [ ] Bubble level on the base: centred. An unlevel station tilts every plane.
- [ ] Note station position (sketch + tape from two walls).
- [ ] Scan saved to `data/raw/calibration/YYYY-MM-DD/<test>_<n>.ply|e57`.

## Tests

1. **Range bias.** Station facing one flat wall at ~2 m. Disto from the rig's
   rotation axis, perpendicular to the wall. Scan. Repeat at ~5 m and ~8 m.
   → `range_bias.walls[]`: scan, box around that wall, `disto_m`.
2. **Plane flatness.** Scan a wall known to be flat (no trim, outlets,
   pictures if possible). → `plane_flatness`.
3. **Straightness.** Station facing a ~5 m flat wall, as square-on as
   practical. → `straightness`.
4. **Repeatability.** One station, five scans, **do not touch the rig**
   between them. → `repeatability.scans`, plus boxes for two opposite walls.
5. **Registration.** One station in each of two adjacent rooms, door open,
   both stations seeing through the doorway. → `registration.source/target`.
   Also tape one feature visible from both (door width, door to corner) as an
   independent check.
6. **Room accuracy.** One scan of the taped room. → `room_accuracy`: boxes for
   each pair of opposite walls and the tape dimension.

## After

- [ ] Find crop boxes: open each scan in CloudCompare (or
      `open3d.visualization.draw_geometries`) and read wall coordinates.
- [ ] `python scripts/run_calibration.py phases/phase2_calibration/sessions/YYYY-MM-DD.yaml`
- [ ] Commit the session file and report on a branch, open a PR.
- [ ] Anything failing: log what you changed in `docs/decision_log.md` and
      re-run the failing test, not the whole day.
