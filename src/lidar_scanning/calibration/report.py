"""Write acceptance results as a Markdown calibration report."""

from pathlib import Path


def _fmt(v):
    return f"{v:,.0f}" if isinstance(v, int) else f"{v:.3f}" if abs(v) < 1 else f"{v:.1f}"


def to_markdown(results, title="Calibration report", meta=None, skipped=None):
    lines = [f"# {title}", ""]
    for key, value in (meta or {}).items():
        lines.append(f"- **{key}:** {value if value is not None else 'TBD'}")
    if meta:
        lines.append("")

    lines += ["| Test | Result | Pass line | Pass |", "|---|---|---|---|"]
    for r in results:
        bound = ">" if r.unit == "fitness" else "<"
        lines.append(f"| {r.test} | {_fmt(r.value)} {r.unit} | {bound} {r.limit} {r.unit} | "
                     f"{'PASS' if r.passed else '**FAIL**'} |")
    for name, reason in (skipped or {}).items():
        lines.append(f"| {name} | not run | | {reason} |")
    lines.append("")

    for r in results:
        lines += [f"## {r.test}", "", r.method, "", "| Quantity | Value |", "|---|---|"]
        lines += [f"| {k} | {_fmt(v)} |" for k, v in r.residuals.items()]
        lines.append("")
        lines += [f"- {n}" for n in r.notes]
        if r.notes:
            lines.append("")
    return "\n".join(lines)


def write_report(results, path, **kwargs):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(to_markdown(results, **kwargs))
    return path
