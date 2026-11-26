"""The director's decisions, checked and logged (#165).

A decision (``decision.json``) is {action, design, term, brief, may_change, goal_change, evidence}:

- ``action``: ``fix`` (a failure of ``design``), ``push`` (one ``term`` of ``design``, as a branch) or
  ``stop``;
- ``may_change``: what the next design may change (change.py's unit names), 1 to 3 of them: one
  thing, which may need a parameter and the function that uses it;
- ``goal_change``: None, or {text, settings}: a new goal text and objective settings that may only
  tighten (a smaller maximum, a larger minimum, the same terms); it needs ``evidence``, run
  directories that exist.

``record`` refuses a bad decision with the reason, and otherwise appends it to decisions.jsonl,
applies a goal change to goal.json (with its history), writes the next iteration's brief.json
(its parent, what it may change, the goal) and next.json.
"""
import json
from pathlib import Path

from brixzle.director import run as RN

ACTIONS = ("fix", "push", "stop")
NEW_FAMILY = "new_family"
MAX_UNITS = 3
TIGHTER = {  # setting -> +1 if a larger value is tighter, -1 if a smaller one is
    "max_length_mm": -1, "max_release_mm": -1, "min_arm_overhang": +1, "max_bridge_riser_mm": -1,
    "grip_mm": -1, "grip_deg": -1, "tolerance_steps": +1,
}


def problems(decision, goal):
    d = decision
    if d.get("action") not in ACTIONS:
        return f"action must be one of {ACTIONS}"
    if d["action"] != "stop":
        mc = d.get("may_change") or []
        if not 1 <= len(mc) <= MAX_UNITS:
            return f"may_change must name 1 to {MAX_UNITS} units (one thing), got {len(mc)}"
        if not d.get("brief"):
            return "a brief is needed"
    gc = d.get("goal_change")
    if gc:
        ev = d.get("evidence") or []
        if not ev or not all(Path(e).is_dir() for e in ev):
            return "a goal change needs evidence: run directories that exist"
        for k, v in (gc.get("settings") or {}).items():
            if k not in TIGHTER:
                return f"setting {k} cannot be changed by the director"
            old = goal["settings"][k]
            if (v - old) * TIGHTER[k] < 0:
                return f"a goal change may only tighten; {k} {old} -> {v} would loosen it"
    return ""


def is_new_family(decision):
    return not decision.get("design") and list(decision.get("may_change") or []) == [NEW_FAMILY]


def explore_due(run, goal, iteration):
    """Why iteration ``iteration`` must try a new family (#206), or "" if it need not: the archive is
    empty (nothing to refine), every ``explore_every``-th iteration, or after ``stall_explore``
    iterations in a row that added nothing to the archive. Only for the error-radius goal."""
    settings = goal.get("settings") or {}
    if settings.get("stage") != "radius":
        return ""
    archive = RN.read_json(Path(run) / "archive.json", {"members": [], "offers": []})
    if not archive.get("members"):
        return "the archive is empty, so there is nothing to refine"
    every = int(settings.get("explore_every") or 0)
    if every and iteration % every == 0:
        return f"every {every}th iteration tries a new family"
    stall = int(settings.get("stall_explore") or 0)
    offers = [o for o in archive.get("offers", []) if str(o.get("name", "")).startswith("iter-")]
    if stall and len(offers) >= stall and not any(o.get("kept") for o in offers[-stall:]):
        return f"the last {stall} iterations added nothing to the archive"
    return ""


RULES_VERSION = "208"  # which requirements and score a design was judged under (#206; grid score #208)


def design_line(run, n):
    """One line for the record of designs tried (#206): iteration n's design, where it stopped and
    why (the critic's ``Mechanism:`` line), or None if it was not measured."""
    run = Path(run)
    d = run / f"iter-{int(n):02d}"
    v = RN.read_json(d / "verified.json")
    if not v or not v.get("measured"):
        return None
    m, brief = v["measured"], RN.read_json(d / "brief.json", {})
    mechanism = None
    diag = d / "diagnosis.md"
    if diag.exists():
        for line in diag.read_text().splitlines():
            if line.strip().lower().startswith("mechanism:"):
                mechanism = line.split(":", 1)[1].strip()[:300]
                break
    g = m.get("grid") or {}
    return {"time": RN.now(), "run": run.name, "iteration": int(n), "spec": m.get("spec"),
            "parent": brief.get("parent") or "", "new_family": brief.get("may_change") == [NEW_FAMILY],
            "brief": (brief.get("brief") or "")[:300], "green_mm": m.get("green_mm"), "weakest": g.get("weakest"),
            "z_rings": m.get("z_rings"),
            "overhang_steps": m.get("overhang_steps"), "stack_courses": m.get("stack_courses"),
            "violations": m.get("violations"), "kept": (v.get("offer") or {}).get("kept"),
            "reason": (v.get("offer") or {}).get("reason"), "mechanism": mechanism, "rules": RULES_VERSION}


def log_design(run, n):
    """Append iteration n's line to ``<run>/designs.jsonl`` and to the shared ``runs/designs.jsonl``
    (runs/ is git-ignored, so later runs read it), once."""
    run = Path(run)
    line = design_line(run, n)
    if line is None:
        return None
    mine = run / "designs.jsonl"
    if mine.exists() and any(json.loads(x).get("iteration") == line["iteration"]
                             for x in mine.read_text().splitlines() if x.strip()):
        return line
    text = json.dumps(line, default=str) + "\n"
    with open(mine, "a") as f:
        f.write(text)
    shared = run.resolve().parents[1] / "designs.jsonl"
    with open(shared, "a") as f:
        f.write(text)
    _library(run, n)
    return line


def _library(run, n):
    """Add the iteration to the shared design library (#215); a failure there never stops the loop."""
    from brixzle.director import library as LB
    try:
        LB.add_iteration(run, n, LB.default_library(run))
    except Exception as e:  # noqa: BLE001 - the library is for reading; the run's own record is kept
        (Path(run) / f"iter-{int(n):02d}" / "library-error.txt").write_text(f"{type(e).__name__}: {e}\n")


def record(run, decision):
    run = Path(run)
    goal = RN.read_json(run / "goal.json")
    why = problems(decision, goal)
    if not why and decision.get("action") != "stop":
        due = explore_due(run, goal, RN.read_json(run / "next.json", {}).get("iteration", 0) + 1)
        if due and not is_new_family(decision):
            why = (f"the next iteration must try a new family ({due}): set design to \"\" and "
                   f"may_change to [\"{NEW_FAMILY}\"], with a brief for an original concept")
    if why:
        return {"ok": False, "reason": why}
    if (goal.get("settings") or {}).get("stage") == "radius":
        log_design(run, RN.read_json(run / "next.json", {}).get("iteration", 0))
    row = {**decision, "time": RN.now()}
    with open(run / "decisions.jsonl", "a") as f:
        f.write(json.dumps(row, default=str) + "\n")
    gc = decision.get("goal_change")
    if gc:
        goal["settings"].update(gc.get("settings") or {})
        if gc.get("text"):
            goal["text"] = gc["text"]
        goal["history"].append({"time": row["time"], "text": gc.get("text"), "settings": gc.get("settings"),
                                "evidence": decision.get("evidence"), "by": "director"})
        RN.write_json(run / "goal.json", goal)
    nxt = RN.read_json(run / "next.json", {})
    if decision["action"] == "stop":
        RN.write_json(run / "next.json", {"step": "finish", "reason": decision.get("brief"), "time": row["time"]})
    else:
        n = nxt.get("iteration", 0) + 1
        brief = {"iteration": n, "parent": decision.get("design"), "action": decision["action"],
                 "term": decision.get("term"), "brief": decision.get("brief"), "may_change": decision.get("may_change"),
                 "goal": goal["text"], "settings": goal["settings"], "time": row["time"]}
        d = run / f"iter-{n:02d}"
        d.mkdir(exist_ok=True)
        RN.write_json(d / "brief.json", brief)
        RN.write_json(run / "next.json", {"step": "design", "iteration": n, "parent": decision.get("design"),
                                          "brief": str(d / "brief.json"), "time": row["time"]})
    return {"ok": True, "reason": ""}
