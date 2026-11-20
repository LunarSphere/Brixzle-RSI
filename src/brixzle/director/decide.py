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


def record(run, decision):
    run = Path(run)
    goal = RN.read_json(run / "goal.json")
    why = problems(decision, goal)
    if why:
        return {"ok": False, "reason": why}
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
