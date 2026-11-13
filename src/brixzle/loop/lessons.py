"""Lessons carried between iterations and runs (issue #88).

The critic may propose up to two lessons per brief: a general statement a later design could use,
the design rules it rests on, and the failure cluster it came from. A lesson is a claim until the
next design tests it, so the loop writes it only once the next iteration has a scored design, with
that outcome as evidence: Z on the target before and after, kept or rejected, and both maps.

The file, ``lessons/lessons.jsonl`` at the repo root by default, has one JSON object per line and
is tracked in git, so lessons are shared by committing them. It is outside ``docs/`` and
``rules/``: a lesson becomes a design rule only by hand, in a reviewed PR. The same lesson again
(lower case, whitespace collapsed) adds evidence to its line, and its status is recomputed:
``supported`` if more of its evidence was kept than rejected, ``not_supported`` if fewer, ``mixed``
if equal. Both agents read them as unreviewed observations (``lessons_text``).
"""

from datetime import date
import json
import math
import os
from pathlib import Path

FILE = Path("lessons") / "lessons.jsonl"
LABEL = "simulated, uncalibrated contact; unreviewed"
MAX_CHARS = 300
SCORED = ("kept", "rejected")
HEADING = ("Lessons from earlier runs: unreviewed observations from earlier runs, simulated with uncalibrated "
           "contact. Weigh them, and prefer the design rules where they disagree.")


def norm(text):
    """The form two lessons are compared in: lower case, whitespace collapsed."""
    return " ".join(str(text).lower().split())


def load(path):
    path = Path(path)
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def status(evidence):
    kept = sum(e["decision"] == "kept" for e in evidence)
    rejected = sum(e["decision"] == "rejected" for e in evidence)
    return "supported" if kept > rejected else "not_supported" if kept < rejected else "mixed"


def _finite(v, where):
    if isinstance(v, float) and not math.isfinite(v):
        raise ValueError(f"a lesson must hold finite numbers only ({where})")
    if isinstance(v, dict):
        for k, x in v.items():
            _finite(x, f"{where}.{k}")
    elif isinstance(v, list):
        for i, x in enumerate(v):
            _finite(x, f"{where}[{i}]")


def _write(path, lines):
    """Rewrite the file through a temporary file and a rename."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text("".join(json.dumps(x, allow_nan=False) + "\n" for x in lines), encoding="utf-8")
    os.replace(tmp, path)


def evidence(run, before, after):
    """The evidence a scored iteration ``after`` gives for the lessons of ``before``'s brief, or None
    if ``after`` has no scored design."""
    if after.get("decision") not in SCORED or not after.get("z"):
        return None
    target = after.get("target") or next(iter(after["z"]))
    z_before = ((before.get("z") or {}).get(target) or {}).get("Z")
    return {"run": run, "iteration": after["iteration"], "target": target,
            "z_before": None if z_before is None else float(z_before), "z_after": float(after["z"][target]["Z"]),
            "decision": after["decision"], "map_before": (before.get("runs") or {}).get("map"),
            "map_after": (after.get("runs") or {}).get("map")}


def record(path, run, before, after, today=None):
    """Write each lesson of ``before``'s brief with the evidence of ``after`` (the next iteration);
    returns the lines written or updated. Nothing is written if ``after`` has no scored design."""
    proposed = ((before.get("brief") or {}).get("lessons")) or []
    ev = evidence(run, before, after)
    if not proposed or ev is None:
        return []
    _finite(ev, "evidence")
    lines = load(path)
    by_text = {norm(x["lesson"]): x for x in lines}
    touched = []
    for item in proposed:
        line = by_text.get(norm(item["lesson"]))
        if line is None:
            line = {"id": f"L{len(lines) + 1:04d}", "date": today or date.today().isoformat(),
                    "lesson": str(item["lesson"]).strip(), "rules": list(item.get("rules") or []),
                    "family": after.get("family"), "evidence": [], "status": None, "label": LABEL}
            lines.append(line)
            by_text[norm(item["lesson"])] = line
        line["evidence"].append(dict(ev))
        line["status"] = status(line["evidence"])
        touched.append(line)
    _write(path, lines)
    return touched


def lessons_text(path, limit=30):
    """The lessons for an agent's prompt, most evidence first, then the newest; '' if there are none."""
    lines = load(path)
    if not lines:
        return ""
    lines.sort(key=lambda x: (len(x["evidence"]), x.get("date", ""), x.get("id", "")), reverse=True)
    out = [HEADING]
    for x in lines[:limit]:
        kept = sum(e["decision"] == "kept" for e in x["evidence"])
        rejected = sum(e["decision"] == "rejected" for e in x["evidence"])
        out.append(f"- [{x['status']}] {x['lesson']} (rules {', '.join(x['rules']) or 'none'}; "
                   f"kept {kept}, rejected {rejected})")
    return "\n".join(out)
