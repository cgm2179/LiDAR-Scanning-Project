# Phase 3 field procedure

Checklist for capture day. `TBD (Cameron)` marks choices still open.

## Before the day

- [ ] Every P2 acceptance test passes, `config/rig.yaml` filled in from P2.
- [ ] Station plan drawn on the floor plan: ids, rough positions, which
      stations overlap (doorways get their own station if two rooms won't align).
- [ ] Copy `manifest_template.yaml` to `captures/<capture id>.yaml` and enter
      the planned stations and neighbours.
- [ ] Batteries, SD card space, Disto, tape, bubble level, floor marks.

## Order

1. Exterior stations first (brief): their overall dimensions are the closure
   check for the interiors.
2. Then each storey, moving through doorways so each new station overlaps one
   already scanned.

## Every station

- [ ] Level the rig (bubble centred).
- [ ] Mark the floor under the axis; tape its position from two walls and
      note the heading of stepper zero → `approx_pose`. `TBD (Cameron)`: how
      heading is measured.
- [ ] Run the scan; files land in `<station>/planes/` and `<station>/pano.jpg`.
- [ ] Three Disto readings (brief, not optional): two wall lengths and
      floor-to-ceiling → `disto`.
- [ ] Quick check: `python scripts/assemble_station.py <station folder>`;
      plane count looks right, floor-to-ceiling agrees with the Disto.
- [ ] Note anything odd (people walking through, open windows, mirrors) in `notes`.

## After

- [ ] Copy every station folder off the Pi to `data/raw/capture/<date>/` and
      back it up.
- [ ] `python scripts/stitch_capture.py phases/phase3_capture/captures/<capture id>.yaml`
- [ ] Read the Warnings section first. Re-scan or add a doorway station for any
      pair flagged as failed or inconsistent.
- [ ] Commit the manifest and report on a branch and open a PR.
