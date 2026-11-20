"""A loop run's directory (#165).

    runs/rsi/<name>/
      goal.json         the goal: text, objective settings (engine/objective.Settings), its history
      STATE.md          written by the director after every evaluation
      decisions.jsonl   every decision, with its evidence (decide.record)
      next.json         the next step: design | verify | critique | direct | finish
      archive.json      the Pareto archive of verified, feasible designs (archive.Archive)
      iter-NN/          one per iteration: brief.json, <module>/family.py, claim.json, verified.json,
                        diagnosis.md, decision.json
      summary.json      written when the run ends

``init_run`` also writes the loop's lock (guard.LOCK) at the repo root, which turns the bounds on.
"""
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess

from brixzle.director import guard as GU

STEPS = ("design", "check", "verify", "critique", "direct", "finish")


def now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def write_json(path, obj):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(obj, indent=1, allow_nan=False, default=str) + "\n")
    tmp.replace(path)


def read_json(path, default=None):
    path = Path(path)
    return json.loads(path.read_text()) if path.exists() else default


def _git(repo, *args):
    return subprocess.run(["git", "-C", str(repo), *args], capture_output=True, text=True, check=True).stdout


def init_run(run, goal, baseline, repo=".", budget=None, deny_read=(), settings=None, human_checkpoints=()):
    """Create the run, write the lock and return the run directory. Refuses an existing run or a
    second loop in the same checkout."""
    from brixzle.engine import objective as OB

    run, repo = Path(run).resolve(), Path(repo).resolve()
    if run.exists():
        raise FileExistsError(f"{run} exists: a run is never overwritten")
    if (repo / GU.LOCK).exists():
        raise FileExistsError(f"another loop holds {repo / GU.LOCK}")
    run.mkdir(parents=True)
    s = settings or asdict(OB.Settings())
    budget = {"iterations": 8, "wall_clock_min": 240, "stall": 3, **(budget or {})}
    write_json(run / "goal.json", {"text": goal, "baseline": baseline, "settings": s, "budget": budget,
                                   "human_checkpoints": list(human_checkpoints),
                                   "history": [{"time": now(), "text": goal, "by": "maintainer"}]})
    (run / "STATE.md").write_text(f"# State of the problem\n\n## Goal\n\n{goal}\n\n## Best designs\n\n"
                                  f"(none yet: the baseline {baseline} is scored first)\n\n## What failed and why\n\n(nothing yet)\n")
    (run / "decisions.jsonl").write_text("")
    write_json(run / "archive.json", {"members": [], "offers": []})
    write_json(run / "next.json", {"step": "design", "iteration": 0, "parent": baseline, "time": now()})
    lock = {"run": str(run), "started": now(), "head": _git(repo, "rev-parse", "HEAD").strip(),
            "status": _git(repo, "status", "--porcelain"), "deny_read": list(deny_read), "repo": str(repo)}
    write_json(repo / GU.LOCK, lock)
    return run


def finish(run, repo=".", reason=""):
    run, repo = Path(run).resolve(), Path(repo).resolve()
    goal = read_json(run / "goal.json", {})
    summary = {"finished": now(), "reason": reason, "goal": goal.get("text"),
               "iterations": len(sorted(run.glob("iter-*"))), "archive": read_json(run / "archive.json", {}),
               "tracked_changes": GU.changed_tracked(run)}
    write_json(run / "summary.json", summary)
    write_json(run / "next.json", {"step": "finish", "time": now(), "reason": reason})
    lock = repo / GU.LOCK
    if lock.exists() and read_json(lock, {}).get("run") == str(run):
        lock.unlink()
    return summary
