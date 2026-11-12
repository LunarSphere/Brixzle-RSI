"""Sim verifier for R11 (placement) and R12 (pickup): the largest scale of a target box where every
trial succeeds.

A box (engine/boxes.py) gives half-widths in X, Z and theta around the aim point. At scale s
the verifier drops the brick at s times a fixed set of unit points: the centre, the 8 corners,
the 6 face centres and a few seeded interior points. A scale passes only if every drop seats
(no jam, fall or wrong valley). Bisection on s finds the largest passing scale; s >= 1 means
the design covers the target box. Both placements are searched, course 0 (onto the anchored
base) and course 1 (onto a seated course-0 brick), and the placement scale is the smaller.
Every drop logs its contact sequence (trials.DropRig).

Pickup (R12) uses the same search over the pickup box with the pickup trial (sim/pickup.py): a
scale passes only if every pick is 'picked' (captured, self-centred, gentle). The pickup score
also needs the clean-release check, run once per design.

Drops at one scale run in parallel worker processes, each with its own compiled scene. A drop
depends only on its pose, so parallel and serial searches give identical drops.

Pose mapping, around the aim (ErrorModel.aim_bias):
- X: dx = aim + s * x * half_width_x (along the lattice).
- Z: release-height error. The drone aims to release s * half_width_z above the minimum
  clearance (ErrorModel.clearance[0]), so the lowest corner releases at the minimum and
  nothing overlaps at release: clear = clearance[0] + s * half_width_z * (1 + z).
- theta: pitch, rotation in the XZ profile plane.
Y, roll and yaw are zero: the placement box is the profile plane.

Pickup pose mapping, centred on the bore axis: dx = s * x * half_width_x (across the tines),
dz = s * z * half_width_z, psi = s * a * half_width_psi (yaw).

Results are simulated, uncalibrated contact. Run it from the command line to record a run:

    uv run python -m brixzle.engine.box_search --family ab --statistic mean --seed 0
    uv run python -m brixzle.engine.box_search --task pickup --family ab --statistic mean --seed 0
"""

import argparse
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from dataclasses import replace
from datetime import datetime, timezone
import json
import math
import multiprocessing
import os
from pathlib import Path
import time

import numpy as np

from brixzle.engine import boxes as BX
from brixzle.engine.sim import pickup as PK
from brixzle.engine.sim import trials as T
from brixzle.engine.sim.trials import DropRig
from brixzle.families import builders as B

LABEL = "simulated, uncalibrated contact"
# Measured on a 14-core Mac (10 performance + 4 efficiency cores), 2026-11-06: a 19-drop round
# takes 3.9 s serially, 1.1 s on 4 workers, 0.8 s on 6, and 3.9 s again on 8 or more; each
# worker also costs ~1 s to start (imports, scene compile). 4 gave the fastest whole search.
DEFAULT_WORKERS = 4
CORNERS = np.array([(x, z, a) for x in (-1, 1) for z in (-1, 1) for a in (-1, 1)], dtype=float)
SUCCESS = {"placement": "seated", "pickup": "picked"}
FACES = np.array([v for i in range(3) for v in (np.eye(3)[i], -np.eye(3)[i])], dtype=float)

__all__ = ["DropRig", "DropPool", "unit_points", "pose_sample", "bisect_scale", "box_search"]


def unit_points(n_interior=4, seed=0):
    """(n, 3) unit-box points (x, z, theta) in [-1, 1]: centre, corners, faces, seeded interior.

    The centre comes first so a design that fails at the aim fails on the first drop."""
    interior = np.random.default_rng(seed).uniform(-0.9, 0.9, size=(n_interior, 3))
    return np.vstack([np.zeros((1, 3)), CORNERS, FACES, interior])


def pose_sample(box: BX.TargetBox, scale, unit, cfg):
    """The trial sample for a unit point at a box scale: ErrorModel.sample keys for placement,
    {dx, dz, psi} for pickup (``cfg`` is unused there)."""
    w = box.half_widths
    x, z, a = (float(v) for v in unit)
    if box.task == "pickup":
        # + 0.0 turns -0.0 into 0.0, so equal poses compare (and dedupe) equal.
        return {"dx": scale * x * w["x_mm"] + 0.0, "dz": scale * z * w["z_mm"] + 0.0,
                "psi": scale * a * w["psi_deg"] + 0.0}
    aim = cfg.error.aim_bias
    return {"dx": aim + scale * x * w["x_mm"], "dy": 0.0, "aim": aim,
            "yaw": 0.0, "roll": 0.0, "pitch": scale * a * w["theta_deg"] + 0.0,
            "clear": cfg.error.clearance[0] + scale * w["z_mm"] * (1 + z), "vel": [0.0, 0.0, 0.0]}


def bisect_scale(ok, s_max=2.0, tol=0.05):
    """Largest s in [0, s_max] with ok(s), assuming ok is monotone; (scale, evaluations).

    Tries the target scale 1 first, then bisects to within tol."""
    evals = []

    def check(s):
        result = bool(ok(s))
        evals.append({"scale": s, "ok": result})
        return result

    if check(1.0):
        lo, hi = 1.0, s_max
        if check(s_max):
            return s_max, evals
    else:
        if not check(0.0):
            return 0.0, evals
        lo, hi = 0.0, 1.0
    while hi - lo > tol:
        mid = (lo + hi) / 2
        if check(mid):
            lo = mid
        else:
            hi = mid
    return lo, evals


_WORKER = {}  # per process: the search's family, params, config, bodies, and one DropRig per course


def _init_worker(family, p, cfg, bodies):
    _WORKER.clear()
    _WORKER.update(family=family, p=p, cfg=cfg, bodies=bodies, rigs={})


def _drop(job):
    """Run one pose in this process's rig for the course (built once, from the shared bodies):
    course 0 or 1 is a placement drop, 'pick' a pickup.

    A placement job may carry options as a third item (the jam analysis, issue #49): 'friction'
    drops in a rig built with that friction coefficient, and 'rest' and 'trace' are passed to
    DropRig.drop."""
    course, sample, *rest = job
    opts = rest[0] if rest else {}
    w = _WORKER
    key = (course, opts["friction"]) if "friction" in opts else course
    if key not in w["rigs"]:
        if course == "pick":
            w["rigs"][key] = PK.PickRig(w["family"], w["p"], w["cfg"], bodies=w["bodies"])
        else:
            cfg = w["cfg"]
            if "friction" in opts:
                cfg = replace(cfg, physics=replace(cfg.physics, friction=float(opts["friction"])))
            w["rigs"][key] = DropRig(w["family"], w["p"], cfg, course=course, bodies=w["bodies"])
    rig = w["rigs"][key]
    if course == "pick":
        return rig.pick(sample)
    return rig.drop(sample, trace=opts.get("trace", True), rest=opts.get("rest", False))


class DropPool:
    """Drop processes for one design, for callers that run many poses in a row (the quick check,
    issue #47). Each worker builds one rig per course from the shared bodies, as box_search does.

        with DropPool(family, p, cfg, workers=4) as pool:
            rows = list(pool.imap([(course, sample), ...]))

    ``imap`` yields one row per job, in job order. With one worker the drops run lazily in this
    process, so a caller that stops reading stops the drops; with more, closing the pool cancels
    the drops that have not started. A one-worker pool uses this module's worker state, so keep
    only one open at a time."""

    def __init__(self, family, p, cfg, workers=None, bodies=None):
        if workers is not None and (not isinstance(workers, int) or isinstance(workers, bool) or workers < 1):
            raise ValueError(f"workers must be an integer >= 1, got {workers!r}")
        self.workers = min(workers or DEFAULT_WORKERS, os.cpu_count() or 1)
        bodies = bodies or {k: B.sim_bodies(family, p, k) for k in family.parts}  # built once, here
        self._ex = None
        if self.workers > 1:
            self._ex = ProcessPoolExecutor(self.workers, mp_context=multiprocessing.get_context("spawn"),
                                           initializer=_init_worker, initargs=(family, p, cfg, bodies))
        else:
            _init_worker(family, p, cfg, bodies)

    def imap(self, jobs):
        """Rows for (course, sample) or (course, sample, options) jobs, in order (see _drop)."""
        jobs = list(jobs)
        if self._ex is None:
            return (_drop(job) for job in jobs)
        return self._ex.map(_drop, jobs)

    def close(self):
        if self._ex is not None:
            self._ex.shutdown(wait=True, cancel_futures=True)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def _search_course(box, cfg, course, unit, s_max, tol, pool):
    drops = []
    simulated = [0]
    success = SUCCESS[box.task]

    def ok(scale):
        jobs = [(course, pose_sample(box, scale, u, cfg)) for u in unit]
        # A trial depends only on its pose, so points with the same pose (all of them at scale 0)
        # run once.
        keys = [json.dumps(sample, sort_keys=True) for _, sample in jobs]
        done = {}
        if pool is None:  # serial: stop the scale at its first failure
            rows = []
            for key, job in zip(keys, jobs):
                if key not in done:
                    done[key] = _drop(job)
                    simulated[0] += 1
                rows.append(done[key])
                if rows[-1]["outcome"] != success:
                    break
        else:  # parallel: run every distinct point at once
            distinct = list(dict.fromkeys(keys))
            done = dict(zip(distinct, pool.map(_drop, [jobs[keys.index(k)] for k in distinct])))
            simulated[0] += len(distinct)
            rows = [done[k] for k in keys]
        for k, (u, row) in enumerate(zip(unit, rows)):
            drops.append({"scale": scale, "point": k, "unit": u.tolist(), **row})
        return all(r["outcome"] == success for r in rows)

    scale, evals = bisect_scale(ok, s_max, tol)
    for e in evals:
        e["drops"] = sum(1 for d in drops if d["scale"] == e["scale"])
    return {"scale": scale, "passed": scale >= 1.0, "evaluations": evals,
            "failure_outcomes": sorted({d["outcome"] for d in drops if d["outcome"] != success}),
            "drops": drops, "drop_count": len(drops), "simulated_count": simulated[0]}


def box_search(family, p, box: BX.TargetBox, cfg=None, courses=(0, 1), seed=0, s_max=2.0, tol=0.05,
               n_interior=4, workers=None, release=True):
    """Bisect the scale of a placement or pickup box for one family; returns a JSON-safe result dict.

    Placement: ``courses`` 0 drops onto the anchored base, 1 onto a seated course-0 brick (for
    A/B, a B onto an A); the scale is the smaller over the courses searched. ``cfg`` is a
    trials.TrialConfig.
    Pickup: one course, 'pick' (``courses`` is ignored), plus the release check; passes only if
    the scale reaches 1 and the release is clean. ``cfg`` is a pickup.PickConfig.
    ``workers``: parallel trial processes (default DEFAULT_WORKERS, capped by the CPU count);
    1 runs serially in this process. ``release=False`` skips the pickup release check (it does not
    depend on the box, so a caller scoring several pickup boxes runs it once itself); the result
    then has no 'release' key and ``passed`` covers the scale only."""
    if box.task not in SUCCESS:
        raise ValueError(f"unknown box task {box.task!r}; expected one of {sorted(SUCCESS)}")
    pickup = box.task == "pickup"
    if pickup:
        cfg = cfg or PK.PickConfig()
        courses = ("pick",)
    else:
        cfg = cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    unit = unit_points(n_interior, seed)
    workers = min(workers or DEFAULT_WORKERS, os.cpu_count() or 1, len(unit))
    t0 = time.time()
    bodies = {k: B.sim_bodies(family, p, k) for k in family.parts}  # built once, here
    if workers > 1:
        pool = ProcessPoolExecutor(workers, mp_context=multiprocessing.get_context("spawn"),
                                   initializer=_init_worker, initargs=(family, p, cfg, bodies))
    else:
        pool = None
        _init_worker(family, p, cfg, bodies)
    try:
        # The courses bisect independently, so run them side by side on the same pool.
        with ThreadPoolExecutor(len(courses)) as threads:
            futures = {str(c): threads.submit(_search_course, box, cfg, c, unit, s_max, tol, pool) for c in courses}
            per_course = {c: f.result() for c, f in futures.items()}
    finally:
        if pool is not None:
            pool.shutdown()
    limiting = min(per_course, key=lambda c: per_course[c]["scale"])
    scale = per_course[limiting]["scale"]
    out = {
        "label": LABEL, "task": box.task,
        "family": family.name, "family_version": family.version, "params": p.to_dict(),
        "box": box.to_dict(), "seed": seed, "s_max": s_max, "tol": tol, "n_interior": n_interior,
        "workers": workers, "config": json.loads(json.dumps(cfg.to_dict() if pickup else T.as_dict(cfg))),
        "scale": scale, "passed": scale >= 1.0, "limiting_course": limiting,
        "courses": per_course,
        "drop_count": sum(r["drop_count"] for r in per_course.values()),
    }
    if pickup and release:
        out["release"] = PK.release_check(family, p, cfg, bodies=bodies)
        out["passed"] = scale >= 1.0 and out["release"]["passed"]
    out["seconds"] = time.time() - t0
    return out


def _new_run_dir(root, family, task, statistic):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    path = Path(root) / f"box-search-{stamp}-{family}-{task}-{statistic}"
    path.mkdir(parents=True, exist_ok=False)  # never overwrite an earlier run
    return path


def main(argv=None):
    from brixzle.families import registry

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--task", choices=sorted(SUCCESS), default="placement")
    ap.add_argument("--family", default="ab")
    ap.add_argument("--statistic", choices=BX.STATISTICS, default="mean")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--courses", type=int, nargs="+", choices=(0, 1), default=[0, 1], help="placement only")
    ap.add_argument("--tol", type=float, default=0.05)
    ap.add_argument("--workers", type=int, default=None, help=f"parallel drop processes (default {DEFAULT_WORKERS})")
    ap.add_argument("--runs", default="runs")
    args = ap.parse_args(argv)
    family = registry.get(args.family)
    box = BX.default_boxes()[args.task][args.statistic]
    result = box_search(family, family.defaults, box, courses=tuple(args.courses), seed=args.seed, tol=args.tol,
                        workers=args.workers)
    if not all(math.isfinite(d["angle_deg"]) for r in result["courses"].values() for d in r["drops"]):
        raise SystemExit("non-finite drop result")
    out = _new_run_dir(args.runs, family.name, args.task, args.statistic)
    (out / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    per = ", ".join(f"course {c} {r['scale']:.3f}" for c, r in result["courses"].items())
    extra = f"; release drift {result['release']['drift_mm']:.3f} mm" if "release" in result else ""
    print(f"{family.name} {args.task} {args.statistic}: scale {result['scale']:.3f} "
          f"({'passes' if result['passed'] else 'fails'}; {per}{extra}), {result['drop_count']} trials, "
          f"{result['workers']} workers, {result['seconds']:.1f} s -> {out}")


if __name__ == "__main__":
    main()
