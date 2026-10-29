# AGENTS.md

Instructions for any coding agent (Claude Code, Codex, Copilot, etc.) working in
this repo. Read this file, then [CurrentProjectCheckpoint.md](CurrentProjectCheckpoint.md),
before doing anything.

## 1. Authorship rules (non-negotiable)

This is a public repo owned by Cameron (GitHub `cgm2179`). The contributor list
must show only them.

- Every commit is authored and committed as `cgm2179 <cgm2179@columbia.edu>`.
  Never set an agent name or email as author or committer.
- No `Co-Authored-By:` trailer naming Claude, Anthropic or any AI tool, and no
  "Generated with ..." footer, in commits **or** pull request titles/bodies.
- Never pass `--no-verify`. The `commit-msg` hook in `.githooks/` rejects
  AI attribution; if it fires, fix the message, don't bypass it.
- After a fresh clone, run once:
  ```bash
  git config user.name cgm2179
  git config user.email cgm2179@columbia.edu
  git config core.hooksPath .githooks
  ```

## 1a. Changes go through pull requests

Never push directly to `main`. For every piece of work:

1. `git checkout main && git pull`, then `git checkout -b <type>/<short-name>`
   (types: `feature/`, `fix/`, `experiment/`, `docs/`, `workflow/`).
2. Commit on the branch (authored as above), update
   `CurrentProjectCheckpoint.md` on the same branch.
3. `git push -u origin <branch>` and open a PR into `main` with
   `gh pr create`. The PR body says what changed, how it was tested, and
   what is still open. No AI attribution footer.
4. Cameron reviews and merges. Don't merge your own PR unless Cameron asks.


## 2. What the project is

Tripod LiDAR scanning station (Raspberry Pi + LDROBOT STL-27L + fisheye camera,
built from [PiDAR](https://github.com/VoxoTech/PiDAR)) and a pipeline:
**point clouds → building geometry JSON → EnergyPlus model → validation against
a tape survey.** Target: 1 inch (25 mm) on any dimension that reaches the energy
model. The test building is a two-storey lab house with basement and attic.

Six deliverables: (1) scanning station, (2) calibration report, (3) captured
dataset, (4) processing pipeline, (5) EnergyPlus model, (6) validation report.
The pipeline and validation are the intellectual core.

## 3. Where code goes

| Folder | Phase | Runs on |
|---|---|---|
| `src/lidar_scanning/rig/` | P1 build the rig | Raspberry Pi |
| `src/lidar_scanning/calibration/` | P2 acceptance tests | laptop |
| `src/lidar_scanning/registration/` | P3 capture + stitching | laptop (Pi for quick checks) |
| `src/lidar_scanning/geometry/` | P4 point cloud → geometry | laptop |
| `src/lidar_scanning/schema/` | P4 JSON schema | laptop |
| `src/lidar_scanning/energyplus/` | P5 schema → EnergyPlus | laptop |
| `src/lidar_scanning/validation/` | P6 validation | laptop |
| `src/lidar_scanning/thermal/` | extension, deferred until P5 is done | both |
| `src/lidar_scanning/fileio/` | shared I/O | both |
| `scripts/` | command-line entry points | both |
| `tests/` | pytest, incl. whole-pipeline regression test | laptop |
| `data/` | scans and outputs, **never committed** | — |
| `third_party/PiDAR/` | upstream clone, **never committed** | — |

Dependencies: `requirements.txt` (laptop, Python 3.11 via `environment.yml`) and
`requirements-pi.txt` (Pi). If you add an import, add it to the right
requirements file and to `scripts/check_environment.py`.

## 4. How to work here

- **The student owns the decisions.** The brief's Section 10 open decisions
  (precedence rule, thresholds, RANSAC parameters, zone granularity, schema
  fields, what gets cut) are Cameron's. Don't silently pick one; propose, then
  leave it marked `TBD (Cameron)` until they decide. Record decisions in
  `docs/decision_log.md`.
- **Report residuals, not accuracy claims.** Calibration and validation code
  should output measured-vs-reference differences, not just pass/fail.
- **Keep the regression test alive** (`tests/test_regression.py`): once a
  fixture scan exists, it runs the whole pipeline and checks floor area within
  2%. Run `pytest` after every pipeline change.
- **Units:** metres internally, SI throughout; convert only at I/O boundaries.
- Code style: plain Python, numpy/Open3D idioms, small functions, docstrings
  that say what the stage does in the pipeline. No ML is needed.
- Don't commit large binaries (scans, PDFs, decks). Drawings and personal
  planning files live outside this repo.

## 5. Keep the checkpoint current

At the end of any session that changes the project state, update
`CurrentProjectCheckpoint.md`: phase, what was done (dated), what's next, and
open questions. It is the hand-off note for the next agent.
