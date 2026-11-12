"""End tune (issue #14): a short Py-BOBYQA run on the sim box scale, once, on the final family.

The loop does not tune numbers inside an iteration. When the design has settled, this moves the
family's bounded parameters to raise the scorer's sim box scale for the run's target (placement,
issue #46). It is short on purpose: a couple of dozen candidates and a wall-clock cap.

- **Variables:** the parameters in ``family.bounds``, each scaled to [0, 1]; ``fixed`` ones stay.
- **Objective:** minus the sim box scale (box_search) for the target box. The untuned design is
  scored first, and every point is scored once (repeats come from a memo).
- **Penalties:** a candidate that cannot be built (ValueError) or breaks a rule margin is not
  simulated. It gets a fixed objective worse than any score, so it can never be the result.
- **Budget:** ``maxfun`` scored candidates and ``max_seconds`` of wall-clock. A candidate is not
  started if it would run past the cap, judged by the slowest one so far.
- **Patience:** after the first probes, the search stops once ``patience`` scored candidates in a
  row have not beaten the best score. On A/B the gain came from the first probes and the later
  steps found nothing (docs/decisions.md, #14).
- **First probes together:** the search starts by stepping each parameter once. Those candidates
  do not depend on each other, so they are scored side by side in ``parallel`` processes (each
  running its box search serially), and the search then finds them in the memo.
- **Held-out check:** a tuned design that scores higher is scored again, with the untuned one,
  under conditions the search did not see (another seed, other friction). It is accepted only if
  none of them got worse; otherwise the untuned design is kept.

Results are simulated, uncalibrated contact. Run it from the command line to record a run:

    uv run python -m brixzle.engine.tune --family ab --statistic mean --seed 0
"""

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timezone
import json
from functools import partial
import math
import multiprocessing
import os
from pathlib import Path
import time

import numpy as np
import pybobyqa

from brixzle.engine import boxes as BX
from brixzle.engine import box_search as BS
from brixzle.engine.sim import trials as T
from brixzle.families import builders as B
from brixzle.families import rules as R

LABEL = BS.LABEL
PENALTY_RULE = 0.5        # objective of a candidate that breaks a rule (a score is >= 0, so -score <= 0)
PENALTY_INFEASIBLE = 1.0  # objective of a candidate that cannot be built
HELD_OUT_FRICTION = (0.2, 0.5)  # the ends of the friction range the predictor sweeps (architecture.md)

__all__ = ["TuneSettings", "free_names", "to_unit", "from_unit", "initial_probes", "rule_violations", "sim_score",
           "held_out_scores", "tune", "tune_family", "write_tune", "summary"]


@dataclass(frozen=True)
class TuneSettings:
    statistic: str = "mean"      # the run's target box
    fixed: tuple = ()            # bounded parameters to leave alone (for example U, if set at intake)
    maxfun: int = 24             # scored candidates, the untuned design included
    max_seconds: float = 420.0   # wall-clock cap on the search (the held-out check comes after it)
    rhobeg: float = 0.1          # first trust-region radius, as a share of each parameter's range
    rhoend: float = 0.01         # radius at which the search stops
    patience: int = 4            # stop after this many scored candidates in a row with no gain
    seed: int = 0                # seed of the box search's interior points
    tol: float = 0.05            # box-scale tolerance, as in the scorer
    n_interior: int = 4          # interior points, as in the scorer
    workers: int | None = None   # drop processes for one box search
    # Processes that score the first probes side by side (1: one after another). Measured on a
    # 14-core Mac, 2026-11-11: box_search is fastest on 4 to 6 processes and slows from 8.
    parallel: int = 6
    held_out: bool = True        # run the held-out check on a tuned design that scores higher

    def to_dict(self):
        return asdict(self)

    def validate(self, family):
        _need(self.statistic in BX.STATISTICS, f"statistic must be one of {BX.STATISTICS}, got {self.statistic!r}")
        unknown = [k for k in self.fixed if k not in family.bounds]
        _need(not unknown, f"fixed names parameters with no bounds: {unknown}")
        _need(len(set(self.fixed)) < len(family.bounds), "every bounded parameter is fixed: nothing to tune")
        _need(isinstance(self.maxfun, int) and not isinstance(self.maxfun, bool) and self.maxfun >= 1,
              f"maxfun must be an integer >= 1, got {self.maxfun!r}")
        _need(math.isfinite(self.max_seconds) and self.max_seconds > 0,
              f"max_seconds must be finite and > 0, got {self.max_seconds!r}")
        _need(isinstance(self.patience, int) and not isinstance(self.patience, bool) and self.patience >= 1,
              f"patience must be an integer >= 1, got {self.patience!r}")
        _need(isinstance(self.parallel, int) and not isinstance(self.parallel, bool) and self.parallel >= 1,
              f"parallel must be an integer >= 1, got {self.parallel!r}")
        _need(math.isfinite(self.rhobeg) and math.isfinite(self.rhoend) and 0 < self.rhoend < self.rhobeg <= 0.5,
              f"need 0 < rhoend < rhobeg <= 0.5, got rhobeg {self.rhobeg!r}, rhoend {self.rhoend!r}")


def _need(cond, msg):
    if not cond:
        raise ValueError(msg)


# --- the parameter vector ---


def free_names(family, fixed=()):
    """The bounded parameters the tune moves, in ``family.bounds`` order."""
    return tuple(k for k in family.bounds if k not in fixed)


def to_unit(family, p, names):
    """The named parameters of ``p``, each scaled to [0, 1] over its bounds."""
    return [(float(getattr(p, k)) - family.bounds[k][0]) / (family.bounds[k][1] - family.bounds[k][0])
            for k in names]


def from_unit(family, p, names, x):
    """``p`` with the named parameters set from a unit vector (clipped to the bounds)."""
    values = {}
    for k, v in zip(names, x, strict=True):
        lo, hi = family.bounds[k]
        values[k] = lo + (hi - lo) * min(1.0, max(0.0, float(v)))
    return replace(p, **values)


def initial_probes(x0, rhobeg):
    """The points Py-BOBYQA scores first with n + 1 interpolation points: each coordinate of ``x0``
    stepped up by ``rhobeg``, or down where that would pass the upper bound of 1
    (pybobyqa.controller.Controller.initialise_coordinate_directions, 1.5.0). A wrong guess costs
    only time: the search scores whatever it does not find in the memo."""
    out = []
    for i, v in enumerate(x0):
        x = list(x0)
        x[i] = v + rhobeg if v + rhobeg <= 1.0 else v - rhobeg
        out.append(x)
    return out


# --- the default rule check and score ---


def rule_violations(family, p):
    """The rule margins below zero, as 'part:key', over every part. Raises ValueError if the
    geometry cannot be built (the family contract's way to say a point is infeasible)."""
    out = []
    for part in family.parts:
        margins = R.margins(family, p, part, B.sim_bodies(family, p, part))
        out += [f"{part}:{k}" for k, v in margins.items() if not v >= 0]
    return out


@dataclass(frozen=True)
class RuleCheck:
    """``rule_violations`` for one family, as an object another process can be sent."""

    family: object

    def __call__(self, p):
        return rule_violations(self.family, p)


@dataclass(frozen=True)
class SimScore:
    """A score function for ``tune``: p -> the sim box search's scale, with what limited it.
    An object, not a closure, so it can be sent to another process."""

    family: object
    box: BX.TargetBox
    cfg: T.TrialConfig
    seed: int = 0
    tol: float = 0.05
    n_interior: int = 4
    workers: int | None = None

    def __call__(self, p):
        res = BS.box_search(self.family, p, self.box, cfg=self.cfg, seed=self.seed, tol=self.tol,
                            n_interior=self.n_interior, workers=self.workers)
        return {"scale": res["scale"], "limiting_course": res["limiting_course"], "drop_count": res["drop_count"],
                "failure_outcomes": res["courses"][res["limiting_course"]]["failure_outcomes"]}


def sim_score(family, box, cfg=None, seed=0, tol=0.05, n_interior=4, workers=None):
    """The default score function: the sim box scale of ``box`` (see SimScore)."""
    return SimScore(family, box, cfg or T.TrialConfig(error=T.ErrorModel(ideal=True)), seed, tol, n_interior, workers)


def held_out_scores(family, box, s: TuneSettings):
    """The held-out score functions: the next seed's interior points, at each end of the friction
    range. The search itself runs at the trial's default friction and ``s.seed``."""
    base = T.TrialConfig(error=T.ErrorModel(ideal=True))
    out = {}
    for mu in HELD_OUT_FRICTION:
        cfg = replace(base, physics=replace(base.physics, friction=mu))
        out[f"seed {s.seed + 1}, friction {mu}"] = sim_score(family, box, cfg=cfg, seed=s.seed + 1, tol=s.tol,
                                                             n_interior=s.n_interior, workers=s.workers)
    return out


# --- the search ---


class _Stop(Exception):
    """Raised inside the objective to end the search when a budget runs out."""


def _scale(result):
    scale = float(result["scale"])
    _need(math.isfinite(scale) and scale >= 0, f"a score must be a finite scale >= 0, got {result['scale']!r}")
    return scale


def _evaluate(check_fn, score_fn, q):
    """Check one candidate against the rules and, if it passes, score it. A top-level function, so
    it can run in another process. 'seconds' covers both steps."""
    t = time.time()
    row = {"status": "scored", "score": None, "reason": None}
    try:
        broken = check_fn(q)
    except ValueError as e:
        row.update(status="infeasible", objective=PENALTY_INFEASIBLE, reason=str(e))
    else:
        if broken:
            row.update(status="rule", objective=PENALTY_RULE, reason=", ".join(broken))
    if row["status"] == "scored":
        result = score_fn(q)
        row["score"] = _scale(result)
        row["objective"] = -row["score"]
        row["detail"] = {k: v for k, v in result.items() if k != "scale"}
    row["seconds"] = time.time() - t
    return row


def _held_out_scale(job):
    fn, q = job
    return _scale(fn(q))


def _process_map(processes):
    """map(fn, candidates) over fresh processes, in order; the default way to score the first probes."""

    def run(fn, candidates):
        candidates = list(candidates)
        n = min(processes, len(candidates), os.cpu_count() or 1)
        with ProcessPoolExecutor(n, mp_context=multiprocessing.get_context("spawn")) as ex:
            return list(ex.map(fn, candidates))

    return run


def tune(family, p, settings: TuneSettings = TuneSettings(), score_fn=None, check_fn=None, held_out_fns=None,
         clock=time.time, map_fn=None):
    """Tune ``p`` inside ``family.bounds``; returns a JSON-safe result dict (see the module docstring).

    ``score_fn(p)`` returns a dict with 'scale' (default: the sim box scale for the target box).
    ``check_fn(p)`` returns the broken rules and raises ValueError for a design that cannot be
    built (default: ``rule_violations``). ``held_out_fns`` maps a name to a score function
    (default: ``held_out_scores`` when ``settings.held_out``, else none). Inject all three to test
    without the sim. ``clock`` is the wall clock. ``map_fn(fn, candidates)`` scores the first
    probes together (default: ``settings.parallel`` processes, which needs functions that can be
    pickled; the defaults can).

    The result's 'params' are the tuned parameters if they were accepted (a higher score, and not
    worse on any held-out score), else the untuned ones."""
    s = settings
    s.validate(family)
    box = BX.default_boxes()["placement"][s.statistic]
    if score_fn is None:
        # With several candidates side by side, each box search runs serially in its own process.
        score_fn = sim_score(family, box, seed=s.seed, tol=s.tol, n_interior=s.n_interior, workers=s.workers)
        if held_out_fns is None and s.held_out:
            held_out_fns = held_out_scores(family, box, s)
    check_fn = check_fn or RuleCheck(family)
    held_out_fns = held_out_fns or {}
    if map_fn is None and s.parallel > 1:
        map_fn = _process_map(s.parallel)
    names = free_names(family, s.fixed)
    n = len(names)
    x0 = to_unit(family, p, names)
    _need(all(0.0 <= v <= 1.0 for v in x0), f"the starting parameters are outside the family's bounds: {p.to_dict()}")

    t0 = clock()
    evaluations, memo = [], {}
    state = {"scored": 0, "slowest": 0.0, "stopped": None, "best": -math.inf, "stale": 0, "searching": False}

    def clip(x):
        return [min(1.0, max(0.0, float(v))) for v in x]

    def key(x):
        return tuple(round(v, 9) for v in x)

    def record(x, row):
        q = from_unit(family, p, names, x)
        row = {"index": len(evaluations), "x": x, "values": {k: getattr(q, k) for k in names}, **row}
        if row["status"] == "scored":
            state["scored"] += 1
            if state["searching"]:  # patience counts the search's own steps, not the first probes
                state["stale"] = 0 if row["score"] > state["best"] else state["stale"] + 1
            state["best"] = max(state["best"], row["score"])
        state["slowest"] = max(state["slowest"], row["seconds"])
        evaluations.append(row)
        memo[key(x)] = row["objective"]
        return row

    def out_of_budget(count=1):
        """Why ``count`` more candidates may not start, or None. A rule check is cheap next to a
        score, so every candidate is counted as one that will be scored."""
        if state["scored"] + count > s.maxfun:
            return "maxfun"
        if state["stale"] >= s.patience:
            return "patience"
        if clock() - t0 + state["slowest"] > s.max_seconds:
            return "max_seconds"
        return None

    def objective(x):
        x = clip(x)
        if key(x) in memo:
            return memo[key(x)]
        state["stopped"] = out_of_budget()
        if state["stopped"]:
            raise _Stop
        state["searching"] = key(x) not in first_points
        t = clock()
        row = _evaluate(check_fn, score_fn, from_unit(family, p, names, x))
        row["seconds"] = clock() - t
        return record(x, row)["objective"]

    # The untuned design, before anything else.
    t = clock()
    first = _evaluate(check_fn, score_fn, p)
    first["seconds"] = clock() - t
    _need(first["status"] != "infeasible", f"the starting design cannot be built: {first['reason']}")
    _need(first["status"] != "rule", f"the starting design breaks rules: {first['reason']}")
    record(x0, first)

    # The first probes, side by side. Each is counted against maxfun as if it will be scored.
    probes = [clip(x) for x in initial_probes(x0, s.rhobeg)][:max(0, s.maxfun - state["scored"])]
    if map_fn is not None and probes and not out_of_budget():
        serial = replace(score_fn, workers=1) if isinstance(score_fn, SimScore) else score_fn
        rows = map_fn(partial(_evaluate, check_fn, serial), [from_unit(family, p, names, x) for x in probes])
        for x, row in zip(probes, rows, strict=True):
            record(x, row)

    solver = {"name": "Py-BOBYQA", "version": pybobyqa.__version__, "npt": n + 1, "exit": None}
    first_points = {key(x0), *(key(clip(x)) for x in initial_probes(x0, s.rhobeg))}
    try:
        soln = pybobyqa.solve(objective, np.array(x0), bounds=(np.zeros(n), np.ones(n)), npt=n + 1,
                              rhobeg=s.rhobeg, rhoend=s.rhoend, maxfun=max(4 * s.maxfun, n + 2),
                              scaling_within_bounds=False, do_logging=False)
        if soln.flag == soln.EXIT_INPUT_ERROR:
            raise ValueError(f"Py-BOBYQA rejected its inputs: {soln.msg}")
        solver["exit"] = soln.msg
        state["stopped"] = state["stopped"] or "solver"
    except _Stop:
        pass
    t_search = clock() - t0

    scored = [e for e in evaluations if e["status"] == "scored"]
    baseline = scored[0]
    best = max(scored, key=lambda e: (e["score"], -e["index"]))  # the earliest of equal scores
    improved = best["score"] > baseline["score"]
    p_tuned = from_unit(family, p, names, best["x"]) if improved else p

    held_out, t = {}, clock()
    if improved and held_out_fns:
        jobs = [(fn, q) for fn in held_out_fns.values() for q in (p, p_tuned)]
        if map_fn is not None:  # side by side, each box search serial in its own process
            jobs = [(replace(fn, workers=1) if isinstance(fn, SimScore) else fn, q) for fn, q in jobs]
            scales = map_fn(_held_out_scale, jobs)
        else:
            scales = [_held_out_scale(job) for job in jobs]
        for i, name in enumerate(held_out_fns):
            before, after = scales[2 * i], scales[2 * i + 1]
            held_out[name] = {"baseline": before, "tuned": after, "not_worse": after >= before}
    t_held = clock() - t
    accepted = improved and all(h["not_worse"] for h in held_out.values())

    def side(q, row):
        return {"params": q.to_dict(), "score": row["score"], "evaluation": row["index"]}

    return {
        "label": LABEL, "task": "placement", "family": family.name, "family_version": family.version,
        "statistic": s.statistic, "box": box.to_dict(), "settings": s.to_dict(),
        "names": list(names), "bounds": {k: list(family.bounds[k]) for k in names},
        "baseline": side(p, baseline), "tuned": side(p_tuned, best if improved else baseline),
        "improved": improved, "held_out": held_out, "accepted": accepted,
        "params": (p_tuned if accepted else p).to_dict(),
        "evaluations": evaluations, "sim_evaluations": state["scored"],
        "held_out_evaluations": 2 * len(held_out), "stopped": state["stopped"], "solver": solver,
        "timing": {"search": t_search, "held_out": t_held, "total": clock() - t0},
    }


def tune_family(family, p=None, settings: TuneSettings = TuneSettings(), runs="runs"):
    """Tune a family on the sim and write the run; returns (result, run directory)."""
    result = tune(family, family.defaults if p is None else p, settings)
    return result, write_tune(result, runs)


# --- output ---


def new_run_dir(root, family_name, statistic):
    """A new runs/tune-<UTC stamp>-<family>-<target> directory; never reuses one."""
    root = Path(root)
    while True:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        path = root / f"tune-{stamp}-{family_name}-{statistic}"
        try:
            path.mkdir(parents=True, exist_ok=False)
            return path
        except FileExistsError:
            continue


def write_tune(result, runs="runs"):
    """Write tune.json to a new run directory and return the directory."""
    text = json.dumps(result, indent=2, allow_nan=False) + "\n"  # fails on a non-finite number
    out = new_run_dir(runs, result["family"], result["statistic"])
    (out / "tune.json").write_text(text, encoding="utf-8")
    return out


def summary(result):
    base, tuned, t = result["baseline"], result["tuned"], result["timing"]
    moved = {k: (base["params"][k], tuned["params"][k]) for k in result["names"]
             if base["params"][k] != tuned["params"][k]}
    lines = [f"{result['family']} {result['statistic']} box ({LABEL}): untuned {base['score']:.3f}, "
             f"tuned {tuned['score']:.3f}; {result['sim_evaluations']} scored of "
             f"{len(result['evaluations'])} candidates, stopped by {result['stopped']}"]
    for name, h in result["held_out"].items():
        lines.append(f"  held out ({name}): untuned {h['baseline']:.3f}, tuned {h['tuned']:.3f}"
                     f"{'' if h['not_worse'] else '  <- worse'}")
    verdict = "accepted" if result["accepted"] else ("rejected by the held-out check" if result["improved"]
                                                    else "no better design found")
    lines.append(f"  tuned design {verdict}; " + (", ".join(f"{k} {a:g} -> {b:.4g}" for k, (a, b) in moved.items())
                                                  or "no parameter moved"))
    lines.append(f"  wall-clock: search {t['search']:.1f} s, held-out {t['held_out']:.1f} s, total {t['total']:.1f} s")
    return "\n".join(lines)


def parser():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    d = TuneSettings()
    ap.add_argument("--family", default="ab")
    ap.add_argument("--statistic", choices=BX.STATISTICS, default=d.statistic, help="the run's target box")
    ap.add_argument("--params", help="JSON file of the starting parameters (default: the family's defaults)")
    ap.add_argument("--fix", nargs="*", default=[], metavar="NAME", help="bounded parameters to leave alone")
    ap.add_argument("--maxfun", type=int, default=d.maxfun, help="scored candidates, the untuned design included")
    ap.add_argument("--max-seconds", type=float, default=d.max_seconds, help="wall-clock cap on the search")
    ap.add_argument("--seed", type=int, default=d.seed)
    ap.add_argument("--tol", type=float, default=d.tol)
    ap.add_argument("--workers", type=int, default=None, help=f"drop processes (default {BS.DEFAULT_WORKERS})")
    ap.add_argument("--patience", type=int, default=d.patience,
                    help="stop after this many scored candidates in a row with no gain")
    ap.add_argument("--parallel", type=int, default=d.parallel, help="processes that score the first probes")
    ap.add_argument("--no-held-out", action="store_true", help="skip the held-out check")
    ap.add_argument("--runs", default="runs")
    return ap


def main(argv=None):
    from brixzle.families import registry

    args = parser().parse_args(argv)
    family = registry.get(args.family)
    p = family.defaults
    if args.params:
        p = family.params_type.from_dict(json.loads(Path(args.params).read_text(encoding="utf-8")))
    settings = TuneSettings(statistic=args.statistic, fixed=tuple(args.fix), maxfun=args.maxfun,
                            max_seconds=args.max_seconds, seed=args.seed, tol=args.tol, workers=args.workers,
                            patience=args.patience, parallel=args.parallel, held_out=not args.no_held_out)
    result, out = tune_family(family, p, settings, runs=args.runs)
    print(summary(result) + f"\n  -> {out}")


if __name__ == "__main__":
    main()
