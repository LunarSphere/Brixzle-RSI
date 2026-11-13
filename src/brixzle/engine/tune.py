"""End tune (issues #14, #61): a short Py-BOBYQA run on Z, once, on the final family.

The loop does not tune numbers inside an iteration. When the design has settled, this moves the
family's bounded parameters to raise Z (engine/zscore.py) on the run's target box (placement,
issue #46). It is short on purpose: a couple of dozen candidates and a wall-clock cap.

- **Variables:** the parameters in ``family.bounds``, each scaled to [0, 1]; ``fixed`` ones stay.
- **Objective:** minus Z on the target box, from the map's cells inside the target boxes
  (zscore.ZScore: the same drops a full map counts). The untuned design is scored first, and every
  point is scored once (repeats come from a memo).
- **Other targets:** a candidate that lowers Z on any other target box (mean, p95, worst) below
  the untuned design's is refused: it gets a fixed objective worse than any score, so it can never
  be the result. On A/B the box-scale tune raised the mean box and lowered p95 (#14).
- **Penalties:** a candidate that cannot be built (ValueError) or breaks a rule margin is not
  simulated. It gets a fixed objective worse than any score, so it can never be the result.
- **Budget:** ``maxfun`` scored candidates and ``max_seconds`` of wall-clock. A candidate is not
  started if it would run past the cap, judged by the slowest one so far.
- **Patience:** after the first probes, the search stops once ``patience`` scored candidates in a
  row have not beaten the best score. On A/B the gain came from the first probes and the later
  steps found nothing (docs/decisions.md, #14).
- **First probes together:** the search starts by stepping each parameter once. Those candidates
  do not depend on each other, so they are scored side by side in ``parallel`` processes (each
  running its drops serially), and the search then finds them in the memo.
- **Held-out check:** a tuned design that scores higher is scored again, with the untuned one,
  under conditions the search did not see: the target box's poses shifted by the next seed, at
  friction 0.2 and 0.5 (zscore.HeldOutZ). It is accepted only if none of them got worse;
  otherwise the untuned design is kept.

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
from brixzle.engine import zscore as ZS
from brixzle.families import builders as B
from brixzle.families import rules as R

LABEL = ZS.LABEL
# Objectives are -Z, so a scored candidate lies in [-1, 1]; each penalty is worse than any score.
PENALTY_OTHER = 1.5       # objective of a candidate that lowers Z on another target box
PENALTY_RULE = 2.0        # objective of a candidate that breaks a rule
PENALTY_INFEASIBLE = 3.0  # objective of a candidate that cannot be built
HELD_OUT_FRICTION = ZS.HELD_OUT_FRICTION
TOL = 1e-12               # Z is a ratio of counts; equal counts give equal Z

__all__ = ["TuneSettings", "free_names", "to_unit", "from_unit", "initial_probes", "rule_violations", "z_score",
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
    seed: int = 0                # the held-out check shifts the poses with seed + 1
    workers: int | None = None   # drop processes for one candidate's Z
    # Processes that score the first probes side by side (1: one after another). Measured on a
    # 14-core Mac, 2026-11-11: drops are fastest on 4 to 6 processes and slow from 8.
    parallel: int = 6
    held_out: bool = True        # run the held-out check on a tuned design that scores higher
    y_error: bool = True         # score the X-Y disc (issue #86); False: X offsets only

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


def z_score(family, box, workers=None, y_error=True):
    """The default score function: Z on ``box``, and on every target box for the guard
    (zscore.ZScore over the map's cells inside them)."""
    return ZS.ZScore(family, box, dict(BX.default_boxes()["placement"]), workers=workers, y_error=y_error)


def held_out_scores(family, box, s: "TuneSettings"):
    """The held-out score functions: the target box's poses shifted by the next seed, at each end
    of the friction range. The search itself runs on the map's own cells at the default friction."""
    return ZS.held_out_scores(family, box, s.seed + 1, workers=s.workers, y_error=s.y_error)


# --- the search ---


class _Stop(Exception):
    """Raised inside the objective to end the search when a budget runs out."""


def _score(result):
    score = float(result["score"])
    _need(math.isfinite(score), f"a score must be finite, got {result['score']!r}")
    return score


def _target_z(result):
    """{statistic: Z} from a score function's 'targets' (counts or plain numbers), or {}."""
    return {k: float(v["Z"] if isinstance(v, dict) else v) for k, v in (result.get("targets") or {}).items()}


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
        row["score"] = _score(result)
        row["objective"] = -row["score"]
        row["detail"] = {k: v for k, v in result.items() if k != "score"}
    row["seconds"] = time.time() - t
    return row


def _held_out_score(job):
    fn, q = job
    return _score(fn(q))


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

    ``score_fn(p)`` returns a dict with 'score' (default: Z on the target box) and optionally
    'targets', {statistic: Z or counts}: a candidate whose Z on another target falls below the
    untuned design's is refused (status 'lowers_other').
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
        # With several candidates side by side, each runs its drops serially in its own process.
        score_fn = z_score(family, box, workers=s.workers, y_error=s.y_error)
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

    def guard(row):
        """Refuse a scored candidate that lowers Z on another target box below the untuned design's."""
        if row["status"] != "scored" or not evaluations:
            return row
        base, got = evaluations[0]["targets"], _target_z(row.get("detail", {}))
        lower = [f"{k} {base[k]:.3f} -> {got[k]:.3f}" for k in base
                 if k != s.statistic and k in got and got[k] < base[k] - TOL]
        if lower:
            row.update(status="lowers_other", objective=PENALTY_OTHER, reason="lowers Z on " + ", ".join(lower))
        return row

    def record(x, row):
        q = from_unit(family, p, names, x)
        row = {"index": len(evaluations), "x": x, "values": {k: getattr(q, k) for k in names}, **guard(row)}
        if not evaluations:
            row["targets"] = _target_z(row.get("detail", {}))
        if row["status"] in ("scored", "lowers_other"):
            state["scored"] += 1
            valid = row["status"] == "scored"
            if state["searching"]:  # patience counts the search's own steps, not the first probes
                state["stale"] = 0 if valid and row["score"] > state["best"] else state["stale"] + 1
            if valid:
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
        serial = ZS.with_workers(score_fn, 1)
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
        if map_fn is not None:  # side by side, each serial in its own process
            jobs = [(ZS.with_workers(fn, 1), q) for fn, q in jobs]
            scales = map_fn(_held_out_score, jobs)
        else:
            scales = [_held_out_score(job) for job in jobs]
        for i, name in enumerate(held_out_fns):
            before, after = scales[2 * i], scales[2 * i + 1]
            held_out[name] = {"baseline": before, "tuned": after, "not_worse": after >= before}
    t_held = clock() - t
    accepted = improved and all(h["not_worse"] for h in held_out.values())

    def side(q, row):
        return {"params": q.to_dict(), "score": row["score"], "evaluation": row["index"],
                "targets": _target_z(row.get("detail", {}))}

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
    lines = [f"{result['family']} {result['statistic']} box ({LABEL}): Z untuned {base['score']:+.3f}, "
             f"tuned {tuned['score']:+.3f}; {result['sim_evaluations']} scored of "
             f"{len(result['evaluations'])} candidates, stopped by {result['stopped']}"]
    for name, h in result["held_out"].items():
        lines.append(f"  held out ({name}): untuned {h['baseline']:+.3f}, tuned {h['tuned']:+.3f}"
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
    ap.add_argument("--seed", type=int, default=d.seed, help="the held-out check uses seed + 1")
    ap.add_argument("--workers", type=int, default=None, help="drop processes per candidate")
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
                            max_seconds=args.max_seconds, seed=args.seed, workers=args.workers,
                            patience=args.patience, parallel=args.parallel, held_out=not args.no_held_out)
    result, out = tune_family(family, p, settings, runs=args.runs)
    print(summary(result) + f"\n  -> {out}")


if __name__ == "__main__":
    main()
