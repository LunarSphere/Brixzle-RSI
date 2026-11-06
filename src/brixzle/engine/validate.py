"""Validate the placement predictor against the sim (issue #11): one X, Z, theta grid, both models.

Every grid pose is dropped in the MuJoCo sim (trials.DropRig, friction = TrialConfig.physics.friction)
and stepped by the analytic predictor (predictor.predict). Poses come from box_search.pose_sample at
scale 1, so the grid spans exactly the target box: X about the aim, Z as the release-height error
above the minimum clearance, theta as pitch.

Two predictor verdicts are compared with the sim's outcome:

- ``pred``: the predictor as the loop uses it, seated only if seated at every mu in predict.MUS;
- ``pred_sim_mu``: the predictor at the sim's own friction, the like-for-like check of the model.

Each pose is one cell of the map:

- ``S`` agree_seated: both seat;
- ``.`` agree_failed: neither seats (the failure class may still differ; see class_agreement);
- ``-`` conservative: the sim seats, the predictor does not (a design the tuner would undervalue);
- ``+`` optimistic: the predictor seats, the sim does not (the dangerous case: a design the tuner
  would overvalue).

The predictor's 'indeterminate' (a possible jam at the seat) counts as 'jammed' for the class.
The trusted X band is the widest band |x| <= t about the aim inside which every pose agrees.

Most grid poses sit far from the capture edge, where agreement is easy, so the agreement rate
alone flatters the predictor. Each model's capture edge is therefore also located: per course,
(Z, theta) row and side, the grid brackets the edge of the seated band that holds the aim (the
last seated point, the first failure outward), and ``edge_steps`` bisections refine it. The
predictor's edge minus the sim's, in mm, is what the tuner would get wrong (negative: the
predictor is conservative). Bisection assumes one edge inside the bracket; a hole further out is
seen only on the grid.

Sim results are simulated, uncalibrated contact; predictor results are predicted, quasi-static 2D.

    uv run python -m brixzle.engine.validate --family ab --statistic max
"""

import argparse
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
import json
import multiprocessing
import os
from pathlib import Path
import time

import numpy as np

from brixzle.engine import box_search as BS
from brixzle.engine import boxes as BX
from brixzle.engine.predictor import predict as PR
from brixzle.engine.sim import trials as T
from brixzle.families import builders as B

LABELS = {"sim": BS.LABEL, "pred": PR.LABEL}
VERDICTS = {"every_mu": "pred", "sim_mu": "pred_sim_mu"}
MODELS = ("sim", "pred", "pred_sim_mu")  # row keys: the sim and both predictor verdicts
CELLS = ("agree_seated", "agree_failed", "conservative", "optimistic")
GLYPH = {"agree_seated": "S", "agree_failed": ".", "conservative": "-", "optimistic": "+"}
DEFAULT_GRID = {"nx": 41, "nz": 3, "nt": 3}
DEFAULT_WORKERS = BS.DEFAULT_WORKERS
DEFAULT_EDGE_STEPS = 5  # bracket / 32: 0.06 mm on the default worst-box grid


def grid_units(nx, nz, nt):
    """(nx * nz * nt, 3) unit points (x, z, theta) on a regular grid over [-1, 1]; x varies fastest.

    An axis with one point sits at 0."""
    for name, n in (("nx", nx), ("nz", nz), ("nt", nt)):
        if not isinstance(n, (int, np.integer)) or isinstance(n, bool) or n < 1:
            raise ValueError(f"{name} must be an integer >= 1, got {n!r}")
    axis = [np.linspace(-1.0, 1.0, n) if n > 1 else np.zeros(1) for n in (nx, nz, nt)]
    return np.array([(x, z, a) for a in axis[2] for z in axis[1] for x in axis[0]], dtype=float) + 0.0


def outcome_class(outcome):
    return "jammed" if outcome == "indeterminate" else outcome


def cell(sim, pred):
    s, p = sim == "seated", pred == "seated"
    return ("agree_seated" if s else "agree_failed") if s == p else ("conservative" if s else "optimistic")


# --- the comparison ---

_WORKER = {}


def _init_worker(family, p, box, cfg, bodies, mus):
    _WORKER.clear()
    _WORKER.update(family=family, p=p, box=box, cfg=cfg, bodies=bodies, mus=mus, rigs={})


def _rig(course):
    w = _WORKER
    if course not in w["rigs"]:
        w["rigs"][course] = T.DropRig(w["family"], w["p"], w["cfg"], course=course, bodies=w["bodies"])
    return w["rigs"][course]


def _pose(job):
    course, unit = job
    w = _WORKER
    family, p, cfg = w["family"], w["p"], w["cfg"]
    sample = BS.pose_sample(w["box"], 1.0, np.asarray(unit), cfg)
    t0 = time.perf_counter()
    sim = _rig(course).drop(sample, trace=False)
    t1 = time.perf_counter()
    pred = PR.predict_pose(family, p, sample, course, mus=w["mus"], trace=False)
    t2 = time.perf_counter()
    per_mu = {k: r["outcome"] for k, r in pred["per_mu"].items()}
    loop_mus = [PR._mu_key(m) for m in PR.MUS]
    bad = [k for k in loop_mus if per_mu[k] != "seated"]
    worst = max(bad, key=float) if bad else None
    every = "seated" if worst is None else per_mu[worst]  # as predict.run_pose
    sim_key = PR._mu_key(cfg.physics.friction)
    return {"course": str(course), "unit": [float(v) for v in unit],
            "dx_mm": float(sample["dx"] - sample["aim"]), "clear_mm": float(sample["clear"]),
            "pitch_deg": float(sample["pitch"]), "sample": {k: v for k, v in sample.items() if k != "vel"},
            "sim": sim["outcome"], "sim_err_mm": [float(v) for v in sim["err_mm"]],
            "sim_angle_deg": float(sim["angle_deg"]),
            "pred": every, "pred_sim_mu": per_mu[sim_key], "per_mu": per_mu,
            "pred_reason": None if worst is None else pred["per_mu"][worst]["reason"],
            "pred_sim_mu_reason": pred["per_mu"][sim_key]["reason"],
            "per_mu_reason": {k: r["reason"] for k, r in pred["per_mu"].items()},
            "seconds": {"sim": t1 - t0, "pred": t2 - t1}}


def _seated(model, course, unit):
    """Does this model seat the pose at grid unit point ``unit`` (scale 1)?"""
    w = _WORKER
    sample = BS.pose_sample(w["box"], 1.0, np.asarray(unit, float), w["cfg"])
    if model == "sim":
        return _rig(course).drop(sample, trace=False)["outcome"] == "seated"
    mus = PR.MUS if model == "pred" else (float(w["cfg"].physics.friction),)
    r = PR.predict_pose(w["family"], w["p"], sample, course, mus=mus, trace=False, early_stop=True)
    return r["outcome"] == "seated"


def _edge(job):
    b, steps = job
    out = dict(b)
    if b["lo"] is not None and b["hi"] is not None:
        out["lo"], out["hi"] = bisect_edge(
            lambda t: _seated(b["model"], int(b["course"]), [b["side"] * t, b["unit_z"], b["unit_theta"]]),
            b["lo"], b["hi"], steps)
    return out


def bisect_edge(ok, lo, hi, steps):
    """Halve a bracket [lo, hi] with ok(lo) true and ok(hi) false ``steps`` times; (lo, hi)."""
    for _ in range(steps):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return lo, hi


def edge_brackets(rows):
    """Per course, (Z, theta) row, side (-1, +1) and model: the grid bracket (lo, hi) in |unit x| of
    the seated band holding the aim. lo is None if the aim itself fails; hi is None if the band
    reaches the end of the grid."""
    out = []
    for c in sorted({r["course"] for r in rows}):
        sel = [r for r in rows if r["course"] == c]
        for a in sorted({r["unit"][2] for r in sel}):
            for z in sorted({r["unit"][1] for r in sel}):
                line = [r for r in sel if r["unit"][1] == z and r["unit"][2] == a]
                for side in (-1, 1):
                    ray = sorted((r for r in line if r["unit"][0] * side >= 0), key=lambda r: abs(r["unit"][0]))
                    for model in MODELS:
                        lo = hi = None
                        if ray and ray[0]["unit"][0] == 0 and ray[0][model] == "seated":
                            lo = 0.0
                            for r in ray[1:]:
                                if r[model] != "seated":
                                    hi = abs(r["unit"][0])
                                    break
                                lo = abs(r["unit"][0])
                        out.append({"course": c, "unit_z": z, "unit_theta": a, "side": side, "model": model,
                                    "lo": lo, "hi": hi})
    return out


def validate(family, p, box, nx=DEFAULT_GRID["nx"], nz=DEFAULT_GRID["nz"], nt=DEFAULT_GRID["nt"], courses=(0, 1),
             cfg=None, workers=None, edge_steps=DEFAULT_EDGE_STEPS):
    """Run sim and predictor on every grid pose of a placement box (scale 1), then refine each
    model's capture edges (``edge_steps`` bisections); a JSON-safe result."""
    if box.task != "placement":
        raise ValueError("validation covers the placement predictor (R11)")
    cfg = cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    sim_mu = float(cfg.physics.friction)
    mus = tuple(sorted(set(PR.MUS) | {sim_mu}))
    units = grid_units(nx, nz, nt)
    jobs = [(c, u.tolist()) for c in courses for u in units]
    workers = min(workers or DEFAULT_WORKERS, os.cpu_count() or 1, len(jobs))
    t0 = time.time()
    bodies = {k: B.sim_bodies(family, p, k) for k in family.parts}
    init = (family, p, box, cfg, bodies, mus)
    if workers > 1:
        with ProcessPoolExecutor(workers, mp_context=multiprocessing.get_context("spawn"),
                                 initializer=_init_worker, initargs=init) as pool:
            rows = list(pool.map(_pose, jobs, chunksize=4))
            t_grid = time.time() - t0
            edges = list(pool.map(_edge, [(b, edge_steps) for b in edge_brackets(rows)]))
    else:
        _init_worker(*init)
        rows = [_pose(j) for j in jobs]
        t_grid = time.time() - t0
        edges = [_edge((b, edge_steps)) for b in edge_brackets(rows)]
    seconds = time.time() - t0
    w = box.half_widths["x_mm"]
    for e in edges:
        for k in ("lo", "hi"):
            e[f"{k}_mm"] = None if e[k] is None else float(e[k] * w)
        e["edge_mm"] = None if e["lo"] is None or e["hi"] is None else (e["lo_mm"] + e["hi_mm"]) / 2
    return {"labels": dict(LABELS), "family": family.name, "family_version": family.version,
            "params": p.to_dict(), "box": box.to_dict(), "grid": {"nx": nx, "nz": nz, "nt": nt},
            "courses": [str(c) for c in courses], "mus": list(PR.MUS), "sim_mu": sim_mu,
            "config": json.loads(json.dumps(T.as_dict(cfg))), "workers": workers,
            "edge_steps": edge_steps, "summary": summarize(rows, edges), "rows": rows, "edges": edges,
            "seconds": seconds, "stage_seconds": {"grid": t_grid, "edges": seconds - t_grid},
            "pose_seconds": {k: float(sum(r["seconds"][k] for r in rows)) for k in ("sim", "pred")}}


# --- the summary ---


def _band(rows, test):
    """Largest t such that every row with |unit x| <= t passes ``test``; None if the centre fails."""
    best = None
    for t in sorted({abs(r["unit"][0]) for r in rows}):
        if all(test(r) for r in rows if abs(r["unit"][0]) <= t + 1e-12):
            best = t
        else:
            break
    return best


def _band_mm(rows, t):
    if t is None:
        return None
    return float(max(abs(r["dx_mm"]) for r in rows if abs(r["unit"][0]) <= t + 1e-12))


def _stats(rows, key):
    cells = {c: 0 for c in CELLS}
    for r in rows:
        cells[cell(r["sim"], r[key])] += 1
    n = len(rows)
    same = sum(outcome_class(r["sim"]) == outcome_class(r[key]) for r in rows)
    trusted = _band(rows, lambda r: cell(r["sim"], r[key]) in ("agree_seated", "agree_failed"))
    sim_band = _band(rows, lambda r: r["sim"] == "seated")
    pred_band = _band(rows, lambda r: r[key] == "seated")
    return {"n": n, "agreement": (cells["agree_seated"] + cells["agree_failed"]) / n if n else 0.0,
            "class_agreement": same / n if n else 0.0, "cells": cells,
            "trusted_unit_x": trusted, "trusted_dx_mm": _band_mm(rows, trusted),
            "seated_band_unit_x": {"sim": sim_band, "pred": pred_band},
            "seated_band_dx_mm": {"sim": _band_mm(rows, sim_band), "pred": _band_mm(rows, pred_band)},
            "disagreements": [{"unit": r["unit"], "dx_mm": r["dx_mm"], "clear_mm": r["clear_mm"],
                               "pitch_deg": r["pitch_deg"], "sim": r["sim"], "pred": r[key],
                               "pred_reason": r.get(f"{key}_reason")}
                              for r in rows if cell(r["sim"], r[key]) in ("conservative", "optimistic")],
            "class_mismatches": _class_mismatches(rows, key)}


def _class_mismatches(rows, key):
    """Counts of 'sim class -> predictor class' where both fail but the class differs."""
    out = {}
    for r in rows:
        a, b = outcome_class(r["sim"]), outcome_class(r[key])
        if a != b and "seated" not in (a, b):
            out[f"{a} -> {b}"] = out.get(f"{a} -> {b}", 0) + 1
    return dict(sorted(out.items()))


def _edge_diffs(edges, key):
    """Pair the predictor's edges (row key ``key``) with the sim's on the same row and side."""
    sim = {(e["unit_z"], e["unit_theta"], e["side"]): e for e in edges if e["model"] == "sim"}
    pairs = []
    for e in edges:
        s = sim.get((e["unit_z"], e["unit_theta"], e["side"]))
        if e["model"] != key or s is None or e["edge_mm"] is None or s["edge_mm"] is None:
            continue
        pairs.append({"unit_z": e["unit_z"], "unit_theta": e["unit_theta"], "side": e["side"],
                      "sim_mm": s["edge_mm"], "pred_mm": e["edge_mm"], "diff_mm": e["edge_mm"] - s["edge_mm"],
                      "resolution_mm": max(e["hi_mm"] - e["lo_mm"], s["hi_mm"] - s["lo_mm"])})
    d = [q["diff_mm"] for q in pairs]
    return {"n": len(pairs), "mean_diff_mm": float(np.mean(d)) if d else None,
            "max_abs_diff_mm": float(np.max(np.abs(d))) if d else None,
            "worst_inner_sim_mm": min((q["sim_mm"] for q in pairs), default=None),
            "worst_inner_pred_mm": min((q["pred_mm"] for q in pairs), default=None), "pairs": pairs}


def summarize(rows, edges=None):
    """Agreement per course and overall, for both predictor verdicts (see the module docstring);
    with ``edges``, the predictor-minus-sim capture-edge differences per course."""
    out = {"courses": {}, "overall": {}}
    for c in sorted({r["course"] for r in rows}):
        sel = [r for r in rows if r["course"] == c]
        out["courses"][c] = {v: _stats(sel, k) for v, k in VERDICTS.items()}
        if edges is not None:
            mine = [e for e in edges if e["course"] == c]
            out["courses"][c]["edges"] = [{"verdict": v, **_edge_diffs(mine, k)} for v, k in VERDICTS.items()]
    out["overall"] = {v: {k2: v2 for k2, v2 in _stats(rows, k).items() if k2 not in
                          ("trusted_unit_x", "trusted_dx_mm", "seated_band_unit_x", "seated_band_dx_mm")}
                      for v, k in VERDICTS.items()}
    return out


def ascii_map(rows, course, key):
    """One line per (theta, z) over X: S agree seated, . agree failed, - conservative, + optimistic."""
    sel = [r for r in rows if r["course"] == course]
    lines = []
    for a in sorted({r["unit"][2] for r in sel}):
        for z in sorted({r["unit"][1] for r in sel}):
            line = sorted((r for r in sel if r["unit"][1] == z and r["unit"][2] == a), key=lambda r: r["unit"][0])
            if not line:
                continue
            q = line[0]
            lines.append(f"theta {q['pitch_deg']:+.2f} deg, clear {q['clear_mm']:5.2f} mm  "
                         + "".join(GLYPH[cell(r["sim"], r[key])] for r in line))
    return "\n".join(lines)


def _fmt(v, unit=""):
    return "none" if v is None else f"{v:.3g}{unit}"


def report_md(result):
    """A short markdown report of a validation result."""
    s, rows, box = result["summary"], result["rows"], result["box"]
    w = box["half_widths"]
    xs = sorted({r["dx_mm"] for r in rows})
    lines = [
        f"# Predictor validation: {result['family']} {result['family_version']}, placement {box['statistic']} box",
        "",
        f"- Sim: {result['labels']['sim']}, friction {result['sim_mu']:g}.",
        f"- Predictor: {result['labels']['pred']}; `every mu` = seated at every mu in "
        f"{result['mus']}, `sim mu` = at mu {result['sim_mu']:g} only.",
        f"- Grid: {result['grid']['nx']} X x {result['grid']['nz']} Z x {result['grid']['nt']} theta at scale 1 "
        f"of the box (X +/-{w['x_mm']:.3g} mm about the aim, Z 0-{2 * w['z_mm']:.3g} mm above the minimum "
        f"clearance, theta +/-{w['theta_deg']:.3g} deg); courses {', '.join(result['courses'])}.",
        f"- X runs from {xs[0]:+.1f} to {xs[-1]:+.1f} mm in the maps below (left to right).",
        f"- Wall-clock {result['seconds']:.1f} s on {result['workers']} workers: grid "
        f"{result['stage_seconds']['grid']:.1f} s, edges {result['stage_seconds']['edges']:.1f} s (grid summed per pose: sim "
        f"{result['pose_seconds']['sim']:.1f} s, predictor {result['pose_seconds']['pred']:.1f} s).",
        "",
        "| Course | Verdict | Agreement | Class agreement | S | . | - (conservative) | + (optimistic) "
        "| Trusted X band (mm) | Seated band sim / pred (mm) |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for c, per in s["courses"].items():
        for v in VERDICTS:
            st = per[v]
            k = st["cells"]
            band = st["seated_band_dx_mm"]
            lines.append(f"| {c} | {v.replace('_', ' ')} | {st['agreement']:.1%} | {st['class_agreement']:.1%} "
                         f"| {k['agree_seated']} | {k['agree_failed']} | {k['conservative']} | {k['optimistic']} "
                         f"| +/-{_fmt(st['trusted_dx_mm'])} | +/-{_fmt(band['sim'])} / +/-{_fmt(band['pred'])} |")
    for v, st in s["overall"].items():
        lines.append(f"| all | {v.replace('_', ' ')} | {st['agreement']:.1%} | {st['class_agreement']:.1%} "
                     f"| {st['cells']['agree_seated']} | {st['cells']['agree_failed']} | {st['cells']['conservative']} "
                     f"| {st['cells']['optimistic']} | | |")
    if "edges" in result:
        lines += ["", f"## Capture edges ({result['edge_steps']} bisections per grid bracket)", "",
                  "Per (Z, theta) row and side, the edge of the seated band about the aim; diff = predictor "
                  "minus sim (negative: conservative).", "",
                  "| Course | Verdict | Rows | Innermost sim edge (mm) | Innermost predictor edge (mm) "
                  "| Mean diff (mm) | Max abs diff (mm) |", "|---|---|---|---|---|---|---|"]
        for c, per in s["courses"].items():
            for d in per.get("edges", []):
                lines.append(f"| {c} | {d['verdict'].replace('_', ' ')} | {d['n']} | {_fmt(d['worst_inner_sim_mm'])} "
                             f"| {_fmt(d['worst_inner_pred_mm'])} | {_fmt(d['mean_diff_mm'])} "
                             f"| {_fmt(d['max_abs_diff_mm'])} |")
    lines += ["", "## Disagreements", ""]
    for c, per in s["courses"].items():
        for v in VERDICTS:
            st = per[v]
            lines.append(f"- Course {c}, {v.replace('_', ' ')}: class mismatches (both fail) "
                         f"{st['class_mismatches'] or 'none'}.")
            for d in st["disagreements"]:
                lines.append(f"  - dx {d['dx_mm']:+.2f} mm, clear {d['clear_mm']:.2f} mm, theta "
                             f"{d['pitch_deg']:+.2f} deg: sim {d['sim']}, predictor {d['pred']} ({d['pred_reason']})")
    lines += ["", "Maps: `S` both seat, `.` neither seats, `-` only the sim seats (conservative), "
              "`+` only the predictor seats (optimistic).", ""]
    for c in s["courses"]:
        for v, key in VERDICTS.items():
            lines += [f"## Course {c}, {v.replace('_', ' ')}", "", "```", ascii_map(rows, c, key), "```", ""]
    return "\n".join(lines)


def _new_run_dir(root, family, statistic):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    path = Path(root) / f"validate-{stamp}-{family}-placement-{statistic}"
    path.mkdir(parents=True, exist_ok=False)  # never overwrite an earlier run
    return path


def main(argv=None):
    from brixzle.families import registry

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--family", default="ab")
    ap.add_argument("--statistic", choices=BX.STATISTICS, default="max")
    ap.add_argument("--nx", type=int, default=DEFAULT_GRID["nx"])
    ap.add_argument("--nz", type=int, default=DEFAULT_GRID["nz"])
    ap.add_argument("--nt", type=int, default=DEFAULT_GRID["nt"])
    ap.add_argument("--courses", type=int, nargs="+", choices=(0, 1), default=[0, 1])
    ap.add_argument("--workers", type=int, default=None, help=f"parallel processes (default {DEFAULT_WORKERS})")
    ap.add_argument("--runs", default="runs")
    args = ap.parse_args(argv)
    family = registry.get(args.family)
    box = BX.default_boxes()["placement"][args.statistic]
    result = validate(family, family.defaults, box, args.nx, args.nz, args.nt, tuple(args.courses),
                      workers=args.workers)
    out = _new_run_dir(args.runs, family.name, args.statistic)
    (out / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    (out / "report.md").write_text(report_md(result) + "\n", encoding="utf-8")
    o = result["summary"]["overall"]
    print(f"{family.name} placement {args.statistic}: agreement every mu {o['every_mu']['agreement']:.1%} "
          f"(optimistic {o['every_mu']['cells']['optimistic']}), sim mu {o['sim_mu']['agreement']:.1%} "
          f"(optimistic {o['sim_mu']['cells']['optimistic']}), {len(result['rows'])} poses, "
          f"{result['seconds']:.1f} s -> {out}")


if __name__ == "__main__":
    main()
