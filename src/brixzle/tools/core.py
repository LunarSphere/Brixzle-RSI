# Edge-stage layout and tipped-jam terminology from LunarSphere/Brixzle-RSI src/brixzle/engine/edge_capture.py @ de8d1ff.
# Changed: use a fixed grid across three heights, with cache and contact replay.
"""Deterministic tools for the design loop's mechanical steps (#163).

Each tool is a function that returns a JSON-able dict and writes it to a new run directory
``runs/<tool>-<UTC>-<family>/result.json`` with its inputs, seed and wall-clock. ``python -m
brixzle.tools <tool> ...`` runs one from the command line and prints a short summary. The tools turn
the drop cache on (engine/drop_cache; ``BRIXZLE_DROP_CACHE``), and ``gate`` and ``ring`` also keep whole
results by geometry, so work already done for the same sim geometry is read back.

Families are named as in the registry (``fable_gentle``) or as ``<root>:<module>`` for a package
outside ``src/`` (``runs/fable:fable_gentle``), as the design agent writes them.

Every sim result is simulated, uncalibrated contact; a result read from a cache is an earlier sim
result, not a new one.
"""
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
import multiprocessing
import os
from pathlib import Path
import time

LABEL = "simulated, uncalibrated contact"
ROOT = Path(__file__).resolve().parents[1]                 # src/brixzle
REPO = ROOT.parents[1]
STAGE_CACHE = REPO / ".cache" / "stages"  # BRIXZLE_STAGE_CACHE moves it (#182: a sandboxed role writes only its run)


def stage_cache_dir():
    where = os.environ.get("BRIXZLE_STAGE_CACHE", "").strip()
    return Path(where) if where else STAGE_CACHE
OUTCOMES = ("seated", "jammed", "wrong_valley", "fell")


# --- shared ---


def cache_on():
    """Turn the drop cache on for this process, unless the caller set BRIXZLE_DROP_CACHE."""
    os.environ.setdefault("BRIXZLE_DROP_CACHE", "on")


def load(spec):
    """A family from a registry name or ``<root>:<module>``."""
    if ":" in spec:
        from brixzle.tools.check_family import load_family
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
    map_dir = Path(map_dir)
    drop_path = map_dir / "drops.json.gz"
    if drop_path.exists():
        with gzip.open(drop_path, "rt") as stream:
            return json.load(stream)
    result = json.loads((map_dir / "result.json").read_text())
    if result.get("kind") != "edgemap":
        raise ValueError(f"{map_dir} is not a placement map or edge map")
    return result["rows"]


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


def select_drops(drops, box=None, outcomes=None, slices=None, courses=None, limit=None, seed=0, edge_case=None):
    """Map drops (not zooms) inside ``box`` (a statistic name), with these outcomes, slices and courses;
    at most ``limit``, picked with ``seed`` and kept in map order."""
    import numpy as np

    c0 = _ideal().error.clearance[0]
    b = boxes()[box] if box else None
    out = [d for d in drops if not d.get("zoom") and (b is None or in_box(d, b, c0))
           and (outcomes is None or d["outcome"] in outcomes) and (slices is None or d.get("slice") in slices)
           and (courses is None or d["course"] in courses)
           and (edge_case is None or d.get("edge_case") == edge_case)]
    if limit is not None and len(out) > limit:
        idx = sorted(np.random.default_rng(seed).choice(len(out), size=limit, replace=False).tolist())
        out = [out[i] for i in idx]
    return out


def _map_sample(d, cfg):
    """The trial sample of a drop recorded without one (an older map's row), from its pose."""
    aim = cfg.error.aim_bias
    return {"dx": aim + float(d["dx_mm"]), "dy": float(d.get("dy_mm", 0.0)), "aim": aim, "yaw": 0.0, "roll": 0.0,
            "pitch": float(d["pitch_deg"]), "clear": float(d["clear_mm"]), "vel": [0.0, 0.0, 0.0]}


def _jobs(drops, opts):
    cfg = _ideal()
    return [(d["edge_case"], d["sample"], dict(opts)) if "edge_case" in d
            else (d["course"], _map_sample(d, cfg), dict(opts)) for d in drops]


def _pose(d):
    row = {"slice": d.get("slice"), "course": d["course"], "cell": d.get("cell"), "x_mm": d["dx_mm"],
           "y_mm": d.get("dy_mm", 0.0), "clear_mm": d["clear_mm"], "pitch_deg": d["pitch_deg"]}
    if "edge_case" in d:
        row["edge_case"] = d["edge_case"]
    return row


# --- half-supported seats ---


def edge_grid():
    """The fixed 200-drop stencil. Poses are X and Y offsets from aim, then height."""
    nominal = [(x, y, "nominal") for x in range(-3, 4) for y in range(-3, 4)]
    for offset in (-6, -5, -4, -0.5, 0.5, 4, 5, 6):
        nominal.extend(((offset, 0, "nominal"), (0, offset, "nominal")))
    nominal.extend((x, y, "nominal") for x in (-4, 4) for y in (-4, 4))
    bare = nominal + [(x, y, height) for height in ("low", "high") for x in (-2, 0, 2) for y in (-2, 0, 2)]
    corbel = [(0, 0, "nominal")]
    for offset in (-4, -2, -1, 1, 2, 4):
        corbel.extend(((offset, 0, "nominal"), (0, offset, "nominal")))
    return {"plus_bare": bare, "minus_bare": bare, "plus_anchored": corbel, "plus_cantilever": corbel}


def edgemap(family, p, workers=8):
    """Drop matched edge grids and two corbel stencils. Keep tipped jams in Z."""
    from brixzle.engine import box_search as BS
    from brixzle.engine import zscore as ZS
    from brixzle.engine.sim import trials as T

    t0 = time.time()
    cfg = _ideal()
    h = boxes()["mean"].half_widths["z_mm"]
    heights = {"low": 0.0, "nominal": h, "high": 2 * h}
    lattice = family.lattice(p)
    grid = edge_grid()
    layouts = {}
    poses, jobs = [], []
    for case, points in grid.items():
        placed, target, base = T.edge_layout(lattice, case)
        layouts[case] = {"placed": [asdict(b) for b in placed], "target": asdict(target), "base": base}
        for x, y, height in points:
            sample = ZS._sample(cfg, {"x": x, "y": y, "z": heights[height], "t": 0.0})
            poses.append({"slice": "edge", "edge_case": case, "course": target.course, "cell": len(poses),
                          "height": height, "dx_mm": x, "dy_mm": y, "clear_mm": sample["clear"],
                          "pitch_deg": sample["pitch"], "sample": sample})
            jobs.append((case, sample, {"trace": False}))

    with BS.DropPool(family, p, cfg, workers=workers) as pool:
        rows = []
        for pose, measured in zip(poses, pool.imap(jobs), strict=True):
            outcome = measured["outcome"]
            rows.append({**pose, "outcome": outcome,
                         "tipped": outcome == "jammed" and measured["settled"] and measured["angle_deg"] > 10.0,
                         **{k: measured[k] for k in ("angle_deg", "err_mm", "settled",
                                                    "release_world_offset_mm", "placed_moved_mm")}})
        cache = dict(pool.cache_stats)

    cases = {}
    for case in grid:
        selected = [r for r in rows if r["edge_case"] == case]
        cases[case] = {**counts([r["outcome"] for r in selected]), "tipped": sum(r["tipped"] for r in selected)}
    weaker = min(("plus_bare", "minus_bare"), key=lambda case: cases[case]["Z"])
    return {"kind": "edgemap", "label": LABEL, "family": family.name, "params": asdict(p),
            "config": T.as_dict(cfg), "tip_deg": 10.0, "grid_height_above_min_mm": heights,
            "layouts": layouts, "cases": cases, "weaker_bare_edge": weaker, "edge_z": cases[weaker]["Z"],
            "rows": rows, "cache": cache, "seconds": round(time.time() - t0, 3)}


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


def _sweep_value(args):
    family, p, param, value, drops, friction, workers = args
    try:
        candidate = replace(p, **{param: value})
    except (ValueError, TypeError) as error:
        return {"value": value, "feasible": False, "reason": str(error)[:200]}
    try:
        result = redrop(family, candidate, drops, friction=friction, workers=workers)
    except ValueError as error:
        return {"value": value, "feasible": False, "reason": str(error)[:200]}

    clearance = _ideal().error.clearance[0]
    per_box = {}
    for name, box in boxes().items():
        outcomes = [r["outcome"] for r, d in zip(result["rows"], drops) if in_box(d, box, clearance)]
        if outcomes:
            per_box[name] = counts(outcomes)
    return {"value": value, "feasible": True, "boxes": per_box, "all": result["counts"], "cache": result["cache"],
            "seconds": result["seconds"]}


def sweep(family, param, values, drops, friction=None, workers=8, p=None):
    """Re-use worker processes across sweep values. Each value owns its geometry and drop state."""
    t0 = time.time()
    p = p or family.defaults
    values = list(values)
    value_workers = min(workers, len(values), os.cpu_count() or 1)
    pose_workers = 1 if value_workers > 1 else workers
    jobs = [(family, p, param, value, drops, friction, pose_workers) for value in values]
    if value_workers > 1:
        with ProcessPoolExecutor(value_workers, mp_context=multiprocessing.get_context("spawn")) as executor:
            rows = list(executor.map(_sweep_value, jobs))
    else:
        rows = [_sweep_value(job) for job in jobs]
    return {"label": LABEL, "family": family.name, "param": param, "drops_per_value": len(drops),
            "friction": friction, "rows": rows, "seconds": round(time.time() - t0, 2)}


# --- gate, ring, and the stage cache ---


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
    path = stage_cache_dir() / f"{stage}-{key}.json"
    if path.exists():
        res = json.loads(path.read_text())
        return {**res, "cached_from": res.get("run")}
    res = run()
    stage_cache_dir().mkdir(parents=True, exist_ok=True)
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


# --- spec helpers ---


def _root_module(spec):
    if ":" in spec:
        return spec.rsplit(":", 1)
    from brixzle.families import registry
    module, path = registry.source(spec)
    return str(path.parents[1]), module.rsplit(".", 1)[1]


