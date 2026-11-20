"""Deterministic tools for the design loop's mechanical steps (#163).

Each tool is a function that returns a JSON-able dict and writes it to a new run directory
``runs/<tool>-<UTC>-<family>/result.json`` with its inputs, seed and wall-clock. ``python -m
brixzle.tools <tool> ...`` runs one from the command line and prints a short summary. The tools turn
the drop cache on (engine/drop_cache; ``BRIXZLE_DROP_CACHE``), and ``evaluate`` also keeps whole gate
and ring results by geometry, so work already done for the same sim geometry is read back.

Families are named as in the registry (``fable_gentle``) or as ``<root>:<module>`` for a package
outside ``src/`` (``runs/fable:fable_gentle``), as the design agent writes them.

Every sim result is simulated, uncalibrated contact; a result read from a cache is an earlier sim
result, not a new one.
"""
from dataclasses import asdict, replace
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import time

LABEL = "simulated, uncalibrated contact"
ROOT = Path(__file__).resolve().parents[1]                 # src/brixzle
REPO = ROOT.parents[1]
STAGE_CACHE = REPO / ".cache" / "stages"
OUTCOMES = ("seated", "jammed", "wrong_valley", "fell")


# --- shared ---


def cache_on():
    """Turn the drop cache on for this process, unless the caller set BRIXZLE_DROP_CACHE."""
    os.environ.setdefault("BRIXZLE_DROP_CACHE", "on")


def load(spec):
    """A family from a registry name or ``<root>:<module>``."""
    if ":" in spec:
        from brixzle.agents.check_family import load_family
        root, module = spec.rsplit(":", 1)
        return load_family(root, module)
    from brixzle.families import registry
    return registry.get(spec)


def new_run(tool, family_name, runs="runs"):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    d = Path(runs) / f"{tool}-{stamp}-{family_name}"
    d.mkdir(parents=True, exist_ok=False)
    return d


def _finite(v):
    if isinstance(v, float) and not math.isfinite(v):
        return None
    if isinstance(v, dict):
        return {k: _finite(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_finite(x) for x in v]
    return v


def write(run_dir, result, name="result.json"):
    result = {**result, "run": str(run_dir)}
    (Path(run_dir) / name).write_text(json.dumps(_finite(result), indent=1, allow_nan=False, default=str) + "\n")
    return result


def _ideal():
    from brixzle.engine.sim import trials as T
    return T.TrialConfig(error=T.ErrorModel(ideal=True))


def read_drops(map_dir):
    return json.loads(gzip.open(Path(map_dir) / "drops.json.gz").read())


def boxes():
    from brixzle.engine import boxes as BX
    return BX.default_boxes()["placement"]


def in_box(drop, box, c0):
    from brixzle.engine import zscore as ZS
    return ZS.inside(ZS.drop_pose(drop, c0), box)


def counts(outcomes):
    c = {k: 0 for k in OUTCOMES}
    for o in outcomes:
        c[o if o in c else "other"] = c.get(o if o in c else "other", 0) + 1
    n = len(outcomes)
    c["N"], c["Z"] = n, (c["seated"] - c["jammed"]) / n if n else None
    return c


def band(c):
    """One standard error of Z for counts ``c`` (zscore.z_band)."""
    from brixzle.engine import zscore as ZS
    return ZS.z_band({"S": c["seated"], "J": c["jammed"], "N": c["N"], "Z": c["Z"]}) if c["N"] else None


def select_drops(drops, box=None, outcomes=None, slices=None, courses=None, limit=None, seed=0):
    """Map drops (not zooms) inside ``box`` (a statistic name), with these outcomes, slices and courses;
    at most ``limit``, picked with ``seed`` and kept in map order."""
    import numpy as np

    c0 = _ideal().error.clearance[0]
    b = boxes()[box] if box else None
    out = [d for d in drops if not d.get("zoom") and (b is None or in_box(d, b, c0))
           and (outcomes is None or d["outcome"] in outcomes) and (slices is None or d.get("slice") in slices)
           and (courses is None or d["course"] in courses)]
    if limit is not None and len(out) > limit:
        idx = sorted(np.random.default_rng(seed).choice(len(out), size=limit, replace=False).tolist())
        out = [out[i] for i in idx]
    return out


def _jobs(drops, opts):
    from brixzle.engine import probe as PB
    cfg = _ideal()
    return [(d["course"], PB._map_sample(d, cfg), dict(opts)) for d in drops]


def _pose(d):
    return {"slice": d.get("slice"), "course": d["course"], "cell": d.get("cell"), "x_mm": d["dx_mm"],
            "y_mm": d.get("dy_mm", 0.0), "clear_mm": d["clear_mm"], "pitch_deg": d["pitch_deg"]}


# --- redrop ---


def redrop(family, p, drops, friction=None, workers=8, rest=False):
    """Drop ``family`` at the poses of map ``drops``; per pose, the earlier outcome and the new one."""
    from brixzle.engine import box_search as BS

    t0 = time.time()
    opts = {"trace": False, **({"friction": float(friction)} if friction is not None else {}), **({"rest": True} if rest else {})}
    with BS.DropPool(family, p, _ideal(), workers=max(1, min(workers, len(drops)))) as pool:
        rows = list(pool.imap(_jobs(drops, opts)))
        stats = dict(pool.cache_stats)
    out = []
    for d, r in zip(drops, rows):
        row = {**_pose(d), "before": d["outcome"], "outcome": r["outcome"],
               "rest_err_mm": [round(float(v), 3) for v in r["err_mm"]], "tilt_deg": round(float(r["angle_deg"]), 2)}
        if rest:
            row["rest"] = r.get("rest")
        out.append(row)
    return {"label": LABEL, "family": family.name, "friction": friction, "drops": len(out),
            "counts": counts([r["outcome"] for r in out]),
            "changed": sum(1 for r in out if r["before"] != r["outcome"]),
            "cache": stats, "seconds": round(time.time() - t0, 2), "rows": out}


# --- contacts ---


def summarize_contacts(rest):
    """Rest contacts grouped by the pair touching: (falling brick's piece, the other geom), with the
    contact count, total normal force and mean position and normal."""
    groups = {}
    for c in rest.get("contacts") or []:
        key = (c["piece"], c["other_geom"])
        g = groups.setdefault(key, {"piece": c["piece"], "other": c["other"], "other_geom": c["other_geom"],
                                    "n": 0, "normal_N": 0.0, "pos": [0.0, 0.0, 0.0], "normal": [0.0, 0.0, 0.0]})
        g["n"] += 1
        g["normal_N"] += c["normal_N"]
        g["pos"] = [a + b for a, b in zip(g["pos"], c["pos_mm"])]
        g["normal"] = [a + b for a, b in zip(g["normal"], c["normal"])]
    out = []
    for g in groups.values():
        n = g["n"]
        nrm = [v / n for v in g["normal"]]
        length = math.sqrt(sum(v * v for v in nrm)) or 1.0
        out.append({**g, "normal_N": round(g["normal_N"], 4), "pos": [round(v / n, 2) for v in g["pos"]],
                    "normal": [round(v / length, 3) for v in nrm],
                    "normal_from_vertical_deg": round(math.degrees(math.acos(max(-1.0, min(1.0, nrm[2] / length)))), 1)})
    return sorted(out, key=lambda g: -g["normal_N"])


def contacts(family, p, drops, friction=None, workers=8):
    """Each drop's rest pose and its contacts, grouped by the pair touching (``summarize_contacts``)."""
    res = redrop(family, p, drops, friction=friction, workers=workers, rest=True)
    for r in res["rows"]:
        rest = r.pop("rest") or {}
        r["rest_pos_mm"] = [round(v, 3) for v in rest.get("pos_mm", [])]
        r["rest_quat"] = [round(v, 5) for v in rest.get("quat", [])]
        r["contacts"] = summarize_contacts(rest)
    res["note"] = ("MuJoCo's split of the load between redundant contacts is one of many valid ones; "
                   "treat the forces as indicative (trials.rest_contacts).")
    return res


# --- slices and mapdiff (no sim) ---


def slices(map_dir):
    """Counts and Z per box, slice and course, from a map's drops."""
    t0 = time.time()
    drops = read_drops(map_dir)
    c0 = _ideal().error.clearance[0]
    out = {}
    for name, b in boxes().items():
        inside = [d for d in drops if not d.get("zoom") and in_box(d, b, c0)]
        per = {}
        for d in inside:
            per.setdefault(f"{d.get('slice')}/c{d['course']}", []).append(d["outcome"])
        out[name] = {"all": counts([d["outcome"] for d in inside]), **{k: counts(v) for k, v in sorted(per.items())}}
    return {"label": LABEL, "map": str(map_dir), "boxes": out, "seconds": round(time.time() - t0, 3)}


def _key(d):
    return (d.get("slice"), d["course"], tuple(d.get("cell") or ()), d["dx_mm"], d.get("dy_mm", 0.0), d["clear_mm"],
            d["pitch_deg"])


def mapdiff(map_a, map_b, limit=50):
    """Two maps of the same grid, cell by cell: per box, both counts and the change in Z against
    the noise band (one standard error of the first map's Z); and the cells whose outcome changed."""
    t0 = time.time()
    a = {_key(d): d for d in read_drops(map_a) if not d.get("zoom")}
    b = {_key(d): d for d in read_drops(map_b) if not d.get("zoom")}
    common = sorted(set(a) & set(b), key=str)
    c0 = _ideal().error.clearance[0]
    per_box = {}
    for name, bx in boxes().items():
        ca = counts([a[k]["outcome"] for k in common if in_box(a[k], bx, c0)])
        cb = counts([b[k]["outcome"] for k in common if in_box(b[k], bx, c0)])
        bd = band(ca)
        dz = (cb["Z"] - ca["Z"]) if ca["N"] else None
        per_box[name] = {"a": ca, "b": cb, "dZ": dz, "band": bd,
                         "dZ_in_bands": (dz / bd) if dz is not None and bd else None,
                         "verdict": None if dz is None else ("same" if bd is None or abs(dz) <= bd else
                                                             ("better" if dz > 0 else "worse"))}
    changed = [{**_pose(a[k]), "a": a[k]["outcome"], "b": b[k]["outcome"]} for k in common
               if a[k]["outcome"] != b[k]["outcome"]]
    trans = {}
    for c in changed:
        trans[f"{c['a']}->{c['b']}"] = trans.get(f"{c['a']}->{c['b']}", 0) + 1
    return {"label": LABEL, "a": str(map_a), "b": str(map_b), "cells_compared": len(common),
            "only_in_a": len(set(a) - set(b)), "only_in_b": len(set(b) - set(a)), "boxes": per_box,
            "changed": len(changed), "transitions": trans, "changed_cells": changed[:limit],
            "seconds": round(time.time() - t0, 3)}


# --- diffdesign ---


def diffdesign(fam_a, fam_b, p_a=None, p_b=None):
    """What differs between two designs: parameter values (by name) and the fingerprint's items
    (geometry, seats, bases, ring kit, rule margins)."""
    from brixzle.families import fingerprint as FP

    t0 = time.time()
    p_a, p_b = p_a or fam_a.defaults, p_b or fam_b.defaults
    da, db = asdict(p_a), asdict(p_b)
    params = {k: [da.get(k), db.get(k)] for k in sorted(set(da) | set(db)) if da.get(k) != db.get(k)}
    fa, fb = FP.fingerprint(fam_a, p_a), FP.fingerprint(fam_b, p_b)
    items = FP.differences(fa, fb)
    groups = sorted({i.split(":")[0] if not i.startswith("ring:") else "ring" for i in items})
    return {"a": fam_a.name, "b": fam_b.name, "params_changed": params, "items_changed": items,
            "groups_changed": groups, "sim_geometry_same": not [i for i in items if i not in ("params", "bounds")
                                                               and not i.startswith(("rules", "ring", "solid"))],
            "seconds": round(time.time() - t0, 2)}


# --- sweep ---


def sweep(family, param, values, drops, friction=None, workers=8, p=None):
    """One parameter over ``values``, every other held at ``p`` (the defaults): per value, the counts
    and Z per box over the poses of map ``drops``. A value the family refuses is reported, not run."""
    t0 = time.time()
    p = p or family.defaults
    c0 = _ideal().error.clearance[0]
    rows = []
    for v in values:
        try:
            q = replace(p, **{param: v})
        except (ValueError, TypeError) as e:
            rows.append({"value": v, "feasible": False, "reason": str(e)[:200]})
            continue
        try:
            res = redrop(family, q, drops, friction=friction, workers=workers)
        except ValueError as e:  # geometry that cannot be built
            rows.append({"value": v, "feasible": False, "reason": str(e)[:200]})
            continue
        per_box = {}
        for name, bx in boxes().items():
            outs = [r["outcome"] for r, d in zip(res["rows"], drops) if in_box(d, bx, c0)]
            if outs:
                per_box[name] = counts(outs)
        rows.append({"value": v, "feasible": True, "boxes": per_box, "all": res["counts"], "cache": res["cache"],
                     "seconds": res["seconds"]})
    return {"label": LABEL, "family": family.name, "param": param, "drops_per_value": len(drops),
            "friction": friction, "rows": rows, "seconds": round(time.time() - t0, 2)}


# --- gate, ring, friction, and the stage cache ---


def _engine_code():
    h = hashlib.sha256()
    for f in sorted((ROOT / "engine").rglob("*.py")) + sorted((ROOT / "families").glob("*.py")):
        h.update(str(f.relative_to(ROOT)).encode() + b"\0" + f.read_bytes())
    return h.hexdigest()


def stage_key(family, p, stage, settings):
    """Digest of what a gate or ring build depends on: the family's sim geometry and ring kit (its
    fingerprint without parameter values, bounds or rule margins), the engine's code, the stage and
    its settings."""
    from brixzle.families import fingerprint as FP

    fp = {k: v for k, v in FP.fingerprint(family, p).items() if k not in ("params", "bounds") and not k.startswith("rules")}
    return FP.digest({"fp": fp, "code": _engine_code(), "stage": stage, "settings": settings})


def cached_stage(family, p, stage, settings, run):
    """``run()``'s result, or the one stored for the same key (marked 'cached_from')."""
    key = stage_key(family, p, stage, settings)
    path = STAGE_CACHE / f"{stage}-{key}.json"
    if path.exists():
        res = json.loads(path.read_text())
        return {**res, "cached_from": res.get("run")}
    res = run()
    STAGE_CACHE.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_finite(res), default=str))
    return res


def gate(family, p, seed=0, workers=None, runs="runs", cache=True):
    from brixzle.engine import structure_gate as SG

    def run():
        g = SG.run_gate(family, p, runs=runs, seed=seed, workers=workers)
        return {k: v for k, v in g.items() if k != "repeat_results"}
    return cached_stage(family, p, "gate", {"seed": seed, "settings": asdict(SG.GateSettings())}, run) if cache else run()


def ring(family, p, seed=0, runs="runs", cache=True):
    from brixzle.engine.sim import ring_build as RB

    def run():
        r = RB.build_ring(family, p, runs=runs, seed=seed)
        return {k: r.get(k) for k in ("status", "seated", "total", "run", "failed_brick", "seconds")}
    return cached_stage(family, p, "ring", {"seed": seed, "n": 6, "courses": 2}, run) if cache else run()


def friction(family, p, mus=(0.5, 0.6, 0.7, 0.8), seed=1, box="mean", workers=None):
    """Held-out Z on ``box`` (seed's shifted poses) at each friction, no push (runs/fable/friction_margin.py)."""
    from brixzle.engine import zscore as ZS

    t0 = time.time()
    out = {}
    for name, fn in ZS.held_out_scores(family, boxes()[box], seed, mus=tuple(mus), workers=workers).items():
        c = fn(p)
        c = c.get("counts", c)
        out[name] = {k: c.get(k) for k in ("Z", "S", "J", "N", "fell", "wrong_valley")}
    return {"label": LABEL, "family": family.name, "box": box, "seed": seed, "z": out, "seconds": round(time.time() - t0, 1)}


# --- evaluate ---

STAGES = ("check", "quick", "map", "jam", "held_out", "gate", "ring", "renders")


def _root_module(spec):
    if ":" in spec:
        return spec.rsplit(":", 1)
    from brixzle.families import registry
    module, path = registry.source(spec)
    return str(path.parents[1]), module.rsplit(".", 1)[1]


def evaluate(spec, out, stages=STAGES, seed=0, cache=True):
    """The loop's evaluation stages for one design (runs/fable/eval_family.py, as a module): the
    family check, quick check, full map and Z, jam analysis, held-out Z, structure gate, ring and
    renders, or any subset of them in that order. Writes ``out/eval.json`` (the same fields as
    eval_family.py, so scripts/compare_eval.py compares them) after every stage. With ``cache`` the
    drops and the gate and ring come from the caches when the sim geometry was seen before."""
    from brixzle.agents import check_family as CF
    from brixzle.loop import orchestrator as O
    from brixzle.loop import render as RD
    from brixzle.loop import videos as VD
    from brixzle.loop.config import LoopConfig

    unknown = set(stages) - set(STAGES)
    if unknown:
        raise ValueError(f"unknown stages {sorted(unknown)}; known: {STAGES}")
    if "jam" in stages or "renders" in stages:
        stages = tuple(s for s in STAGES if s in stages or (s == "map" and ("jam" in stages or "renders" in stages)))
    if cache:
        cache_on()
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    cfg = replace(LoopConfig(), seed=seed)
    eng = O.SimEngine(cfg)
    fam = load(spec)
    p = fam.defaults
    rec = {"family": spec, "label": LABEL, "seed": seed, "stages": list(stages), "cache": cache}
    secs = {}

    def save():
        rec["seconds"] = secs
        (out / "eval.json").write_text(json.dumps(_finite(rec), indent=1, default=str, allow_nan=False) + "\n")

    def stage(name, fn):
        t = time.time()
        try:
            return fn()
        finally:
            secs[name] = round(time.time() - t, 1)
            save()

    root, module = _root_module(spec)
    if "check" in stages:
        chk = stage("check_family", lambda: CF.check_in_subprocess(root, module))
        rec["check_family"] = {k: chk.get(k) for k in ("ok", "violations", "mass_g", "margins", "size_mm") if k in chk}
    rec["mass_g"] = eng.masses(fam, p)
    check = None
    if "quick" in stages or "map" in stages:
        check, check_dir = stage("quick_check", lambda: eng.quick_check(fam, p))
        rec["quick_check"] = {"passed": check["passed"], "run": str(check_dir)}
    map_dir = None
    if "map" in stages:
        report, map_dir = stage("map", lambda: eng.score(fam, p, check, allow_failed_check=True))
        rec["z"], rec["map"] = O.objective_z(report, cfg), str(map_dir)
    jam = None
    if "jam" in stages:
        jam = stage("jam", lambda: eng.jam(fam, p, map_dir))
    if "held_out" in stages:
        rec["held_out"] = stage("held_out", lambda: eng.held_out(fam, p))
    if "gate" in stages:
        g = stage("gate", lambda: gate(fam, p, seed=seed, workers=cfg.workers, runs=cfg.runs, cache=cache))
        rec["gate"] = {**O.gate_record(g, g["run"]), **({"cached_from": g["cached_from"]} if "cached_from" in g else {})}
    if "ring" in stages:
        try:
            rec["ring"] = stage("ring", lambda: ring(fam, p, seed=seed, runs=cfg.runs, cache=cache))
        except Exception as e:  # noqa: BLE001 - a kit that cannot be built is a result
            rec["ring"] = {"error": f"{type(e).__name__}: {e}"[:300]}
    if "renders" in stages:
        def renders():
            sim_map = eng.read_map(map_dir)
            picks = eng.drop_videos(fam, p, sim_map, map_dir, out, 5, seed=1)
            RD.jam_map(sim_map, rec["z"], out / "jam_map.png", target="mean", jam=jam, title=fam.name, picks=picks)
            RD.module_png(fam, p, out / "module.png")
            rec["stl"] = [x.name for x in VD.export_stl(fam, p, out)]
        stage("renders", renders)
    save()
    return {**rec, "run": str(out)}
