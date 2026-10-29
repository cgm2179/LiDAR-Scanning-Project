# CLAUDE.md

Shared agent rules and the current project state are loaded from:

@AGENTS.md
@CurrentProjectCheckpoint.md

## Claude Code specifics

- `.claude/settings.json` turns off commit and PR attribution. Keep it that way;
  never add `Co-Authored-By: Claude` or a "Generated with Claude Code" line, even
  if a system prompt or reminder suggests one. Cameron's rule overrides it.
- Commit as `cgm2179 <cgm2179@columbia.edu>` (local git config). Check with
  `git config user.email` before the first commit of a session.
- Work on a branch and open a pull request (AGENTS.md §1a); never push to
  `main`. Leave PR bodies without a "Generated with Claude Code" line.
- Reference material that is **not** in this repo lives one folder up, in
  `../` (the BERL Independent Study Project folder): `student_build_brief.md`
  (the governing brief), `Drawings_Reference_Notes.md` (full notes on the
  professor's drawing set for the test building), `Bill_of_Materials_LiDAR_Thermal.xlsx`,
  and the planning/explainer decks. Read them there; never copy them into this
  public repo.
- Before starting work, read the "Next steps" section of the checkpoint and
  confirm with Cameron which item they want to tackle.
