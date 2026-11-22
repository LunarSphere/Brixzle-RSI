"""python -m brixzle.director <step> ... : the deterministic steps of the self-steering loop (#165).

    init      --run DIR --goal TEXT [--baseline ab] [--iterations N] [--minutes M] [--deny-read P,..]
              [--checkpoints 3,6]
    status    --run DIR                       next step, budget used, archive
    baseline  --run DIR                       score the baseline (registry family) and seed the archive
    check     --run DIR --iter N               the candidate changed only what its brief allows
    verify    --run DIR --iter N               measure the candidate, compare its claim, offer it to the archive
    decide    --run DIR --iter N               validate and log iter-NN/decision.json
    guard     --run DIR                        the tracked tree is unchanged since the run began
    finish    --run DIR [--reason TEXT]

Each prints one JSON object. Exit status 1 means the step refused (the JSON says why).
"""
import argparse
import json
from pathlib import Path
import sys
import time

from brixzle.director import archive as AR
from brixzle.director import change as CH
from brixzle.director import decide as DE
from brixzle.director import guard as GU
from brixzle.director import run as RN
from brixzle.director import verify as VE


def iter_dir(run, n):
    return Path(run) / f"iter-{int(n):02d}"


def candidate(run, n):
    """(module dir, '<root>:<module>') of iteration n's candidate: the one package in iter-NN/."""
    d = iter_dir(run, n)
    pkgs = [p for p in d.iterdir() if p.is_dir() and (p / "family.py").exists()]
    if len(pkgs) != 1:
        raise ValueError(f"{d} must hold exactly one package with a family.py, found {[p.name for p in pkgs]}")
    return pkgs[0], f"{d}:{pkgs[0].name}"


def parent_file(run, n):
    brief = RN.read_json(iter_dir(run, n) / "brief.json", {})
    parent = brief.get("parent")
    if parent and ":" in parent:
        root, module = parent.rsplit(":", 1)
        return Path(root) / module / "family.py"
    if parent:
        from brixzle.families import registry
        return registry.source(parent)[1]
    return None


def budget_state(run):
    goal = RN.read_json(Path(run) / "goal.json")
    iters = len([d for d in Path(run).glob("iter-*") if d.name != "iter-00" and any(d.glob("*/family.py"))])
    started = goal["history"][0]["time"]
    from datetime import datetime
    minutes = (datetime.fromisoformat(RN.now()) - datetime.fromisoformat(started)).total_seconds() / 60
    b = goal["budget"]
    return {"iterations": iters, "max_iterations": b["iterations"], "minutes": round(minutes, 1),
            "max_minutes": b["wall_clock_min"], "exhausted": iters >= b["iterations"] or minutes >= b["wall_clock_min"]}


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m brixzle.director")
    ap.add_argument("step", choices=("init", "status", "baseline", "check", "verify", "decide", "guard", "finish"))
    ap.add_argument("--run", required=True)
    ap.add_argument("--iter", type=int)
    ap.add_argument("--goal")
    ap.add_argument("--baseline", default="ab")
    ap.add_argument("--iterations", type=int, default=8)
    ap.add_argument("--minutes", type=float, default=240)
    ap.add_argument("--deny-read", default="")
    ap.add_argument("--checkpoints", default="")
    ap.add_argument("--reason", default="")
    ap.add_argument("--settings", type=Path, help="goal settings JSON for init")
    a = ap.parse_args(argv)
    run = Path(a.run).resolve()
    out, ok = {}, True
    if a.step == "init":
        r = RN.init_run(run, a.goal, a.baseline, budget={"iterations": a.iterations, "wall_clock_min": a.minutes},
                        settings=json.loads(a.settings.read_text()) if a.settings else None,
                        deny_read=[d for d in a.deny_read.split(",") if d],
                        human_checkpoints=[int(c) for c in a.checkpoints.split(",") if c])
        out = {"run": str(r), "next": RN.read_json(r / "next.json")}
    elif a.step == "status":
        out = {"next": RN.read_json(run / "next.json"), "budget": budget_state(run),
               "archive": [m["name"] for m in RN.read_json(run / "archive.json")["members"]],
               "tracked_changes": GU.changed_tracked(run)}
    elif a.step == "baseline":
        goal = RN.read_json(run / "goal.json")
        d = run / "baseline"
        if (d / "verified.json").exists():
            out = RN.read_json(d / "verified.json")["offer"]
        else:
            m = _measure(run, goal["baseline"], d)
            out = _offer(run, {**_archive_terms(m), "violations": m["violations"],
                               "name": f"baseline:{goal['baseline']}", "spec": goal["baseline"]})
            RN.write_json(d / "verified.json", {"measured": m, "claim": None, "offer": out})
    elif a.step == "check":
        pkg, spec = candidate(run, a.iter)
        brief = RN.read_json(iter_dir(run, a.iter) / "brief.json", {})
        pf = parent_file(run, a.iter)
        if pf is None:  # a new family: nothing to freeze
            out = {"ok": True, "changed": ["new family"], "reason": "no parent"}
        else:
            out = CH.check(pf, pkg / "family.py", brief.get("may_change", []))
        RN.write_json(iter_dir(run, a.iter) / "change.json", out)
        ok = out["ok"]
    elif a.step == "verify":
        d = iter_dir(run, a.iter)
        pkg, spec = candidate(run, a.iter)
        if (d / "verified.json").exists():  # already verified: the archive took its offer once
            v = RN.read_json(d / "verified.json")
            print(json.dumps({"claim": v["claim"], "offer": v["offer"], "violations": v["measured"].get("violations")},
                             indent=1, default=str))
            return 0 if v["claim"]["ok"] else 1
        claim = RN.read_json(d / "claim.json", {"numbers": []})
        brief = RN.read_json(d / "brief.json", {})
        m = _measure(run, spec, d, measure_edge=brief.get("term") == "edge_z")
        cmp = VE.compare(claim, m)
        rec = {**_archive_terms(m), "violations": m["violations"],
               "name": f"iter-{a.iter:02d}:{pkg.name}", "spec": spec}
        offer = _offer(run, rec)
        out = {"claim": cmp, "offer": offer, "violations": m.get("violations"), "seconds": m["seconds"]}
        RN.write_json(d / "verified.json", {"measured": m, "claim": cmp, "offer": offer})
        ok = cmp["ok"]
    elif a.step == "decide":
        d = iter_dir(run, a.iter)
        out = DE.record(run, RN.read_json(d / "decision.json"))
        ok = out["ok"]
    elif a.step == "guard":
        ch = GU.changed_tracked(run)
        out, ok = {"tracked_changes": ch, "ok": not ch}, not ch
    elif a.step == "finish":
        out = RN.finish(run, reason=a.reason)
    print(json.dumps(out, indent=1, default=str))
    return 0 if ok else 1


def _measure(run, spec, directory, **options):
    settings = RN.read_json(run / "goal.json")["settings"]
    if settings.get("stage") == "held":
        baseline = RN.read_json(run / "baseline" / "verified.json", {})
        floor = settings.get("min_drop_z", VE.lookup(baseline, "measured.z.mean.Z"))
        options.update(measure_held=True, min_drop_z=floor, held_targets=tuple(settings.get("held_targets", ())))
    return VE.measure(spec, directory / "measured", **options)


def _archive_terms(measured):
    held = measured.get("held", {})
    counts = {f"held_{key}_z_counts": {k: value[k] for k in ("S", "J", "N")}
              for key, value in held.get("cases", {}).items()}
    return {**measured["objective"], **held.get("scores", {}), **counts}


def _offer(run, rec):
    path = Path(run) / "archive.json"
    data = RN.read_json(path, {"members": [], "offers": []})
    offers = data.setdefault("offers", [])
    for previous in offers:
        if previous["name"] == rec["name"]:
            return {key: saved for key, saved in previous.items() if key not in {"name", "time"}}
    settings = RN.read_json(Path(run) / "goal.json", {}).get("settings", {})
    defaults = AR.HELD_TERMS if settings.get("stage") == "held" else AR.LEGACY_TERMS
    arc = AR.Archive(data["members"], terms=settings.get("scored", defaults))
    res = arc.offer(rec)
    data["members"] = arc.members
    offers.append({"name": rec["name"], **res, "time": RN.now()})
    RN.write_json(path, data)
    return res


if __name__ == "__main__":
    sys.exit(main())
