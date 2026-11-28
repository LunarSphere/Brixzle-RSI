"""Quick random check (issue #47): a go/no-go gate before the full sim map.

The full map (#48) takes minutes, and most new designs should be turned back in seconds. So a new
design is first dropped at 40 poses inside the target box (X, Z and theta around the aim), in the
MuJoCo sim (trials.DropRig), half onto the base (course 0) and half across two seated bricks (course 1, running bond, #82):

- **Poses:** one seeded Latin hypercube over X, Z, theta and the course. Each of the three axes
  has exactly one pose in each of its 40 equal slices, and each course gets exactly half the
  poses, so the drops are spread evenly and no side of the box is missed.
- **Pass rule:** at least 32 of 40 seated. The check stops at the 9th failure, because 32 is then
  out of reach. Poses are judged in order, so the result does not depend on the worker count.
- **On a fail:** the failing poses go back to the design agent (#18) with their outcomes and
  contact traces, so it can fix the design without a full map.

Why 40 and 32 (binomial, worked on issue #47): a design that truly seats 60% of drops passes
0.6% of the time, and one that seats 90% passes 98.5% of the time. A design that seats every drop
in the box always passes, so the gate never turns back a design with Z = 1.

The check is not the score. The loop's objective is Z (engine/zscore.py, issue #61).
Placement only for now (issue #46). Results are simulated, uncalibrated contact.

**The quarter screen (#222).** ``quarter_screen`` is the director's cheap filter before the grid
score: the same 40 drops and the same rule (at least 32 seated, stopping at the 9th failure), but on
the score's four placements (``radius.QUARTERS``: the stack and overhangs 1, 2 and 3), 10 each,
interleaved, at seeded random positions spread evenly over the 16 mm disc (one Latin hypercube over
radius squared and angle), released in the grid's free drop. A design that cannot seat one of the
four placements fails it: 10 failures are more than the 8 allowed. Its Z, (S - J) / N per quarter
and over all its drops, has the form of the score but is not the score.

    uv run python -m brixzle.engine.quick_check --family ab --seed 0
    uv run python -m brixzle.engine.quick_check --quarters --family <name or root:module> --seed 0
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.stats import qmc

from brixzle.engine import box_search as BS
from brixzle.engine import boxes as BX
from brixzle.engine.sim import trials as T

LABEL = BS.LABEL
N_DROPS = 40
NEED_SEATED = 32
# One process is the fastest way to run this few drops. Measured on A/B V0, 2026-11-11 (14-core
# Mac): the 40 drops of the mean box took 5.2 s in one process and 8.6 s on 4 workers, and a check
# stopped at its 9th failure took 1.3-1.5 s against 7.8-8.5 s, because each worker process takes
# several seconds to start (imports, scene compile).
DEFAULT_WORKERS = 1
SUCCESS = BS.SUCCESS["placement"]


def max_failures(n=N_DROPS, need=NEED_SEATED):
    """The number of failures at which ``need`` seated of ``n`` is out of reach (9 for 32 of 40)."""
    for name, v in (("n", n), ("need", need)):
        if not isinstance(v, (int, np.integer)) or isinstance(v, bool):
            raise ValueError(f"{name} must be an integer, got {v!r}")
    if n < 1 or not 0 <= need <= n:
        raise ValueError(f"need 0 <= need <= n and n >= 1, got n = {n}, need = {need}")
    return n - need + 1


def check_poses(n=N_DROPS, seed=0, disc=False):
    """``n`` seeded poses as (course, unit point): the unit point is (x, z, theta) in [-1, 1], as
    box_search.pose_sample takes it at scale 1.

    One Latin hypercube in four dimensions: X, Z, theta, and a fourth that picks the course, so
    every axis is evenly covered and the courses split the poses evenly (exactly, for even n).

    ``disc`` (issue #86): a fifth dimension, and (X, Y) inside the unit disc from two of them
    (radius sqrt(u), angle 2 pi v, so the poses are even over the disc's area); the unit point
    is then (x, z, theta, y), as ``disc_sample`` takes it."""
    max_failures(n, 0)
    if not disc:
        u = qmc.LatinHypercube(d=4, seed=seed).random(n)
        return [(int(row[3] >= 0.5), 2.0 * row[:3] - 1.0) for row in u]
    u = qmc.LatinHypercube(d=5, seed=seed).random(n)
    out = []
    for row in u:
        rho, phi = math.sqrt(row[0]), 2.0 * math.pi * row[1]
        out.append((int(row[4] >= 0.5), np.array([rho * math.cos(phi), 2.0 * row[2] - 1.0, 2.0 * row[3] - 1.0,
                                                  rho * math.sin(phi)])))
    return out


def disc_sample(box, unit, cfg):
    """The trial sample of a disc pose (x, z, theta, y), each in [-1, 1], with (x, y) in the unit
    disc: X and Y scaled by the box's radius (issue #86)."""
    x, z, t, y = (float(v) for v in unit)
    s = BS.pose_sample(box, 1.0, (x, z, t), cfg)
    s["dy"] = y * box.half_widths["y_mm"] + 0.0
    return s


def tally(rows, n=N_DROPS, need=NEED_SEATED):
    """Judge drop results in pose order, stopping at the failure that puts ``need`` out of reach.

    ``rows`` is an iterable of drop rows (each with an 'outcome'); it is not read past the stop.
    Returns the counts, the verdict and the rows that were judged."""
    limit = max_failures(n, need)
    judged, failed = [], 0
    for row in rows:
        judged.append(row)
        failed += row["outcome"] != SUCCESS  # a non-finite pose is never classified as seated
        if failed >= limit or len(judged) == n:
            break
    seated = len(judged) - failed
    if failed < limit and len(judged) < n:
        raise ValueError(f"only {len(judged)} of {n} drops were run and the check is undecided")
    return {"passed": seated >= need, "seated": seated, "failed": failed, "dropped": len(judged),
            "stopped_early": len(judged) < n, "rows": judged}


UNSTABLE = "unstable"  # the sim returned a non-finite pose: reported as a failure, never written as NaN


def _num(v):
    v = float(v)
    return v if math.isfinite(v) else None


def drop_row(index, course, unit, sample, row):
    """One drop for check.json: the pose in physical units and the outcome. A failing drop keeps
    its contact trace and impact numbers for the design agent; a seated one does not need them.

    A drop whose result is not finite (the simulation blew up) is a failure with the outcome
    'unstable', and its non-finite numbers are written as null."""
    numbers = [*row["err_mm"], row["angle_deg"]]
    stable = all(math.isfinite(float(v)) for v in numbers)
    out = {"index": index, "course": course, "unit": [float(v) for v in unit],
           "dx_mm": float(sample["dx"] - sample["aim"]), "dy_mm": float(sample.get("dy", 0.0)),
           "clear_mm": float(sample["clear"]),
           "pitch_deg": float(sample["pitch"]), "outcome": row["outcome"] if stable else UNSTABLE,
           "err_mm": [_num(v) for v in row["err_mm"]], "angle_deg": _num(row["angle_deg"])}
    if out["outcome"] != SUCCESS:
        out.update(contacts=row.get("contacts", []), settled=bool(row.get("settled", False)),
                   time_s=_num(row.get("time", 0.0)), f_max_N=_num(row.get("f_max", 0.0)),
                   penetration_mm=_num(row.get("penetration_mm", 0.0)))
    return out


def quick_check(family, p, box, cfg=None, n=N_DROPS, need=NEED_SEATED, seed=0, workers=DEFAULT_WORKERS, disc=False):
    """Drop ``n`` evenly spread poses inside a placement box; passes if at least ``need`` seat.

    Returns a JSON-safe dict: the verdict and counts, every drop that was judged ('drops', in pose
    order) and the failing ones with their contact traces ('failures'). ``cfg`` is a
    trials.TrialConfig (default: the ideal error model, as the box search). ``workers``: drop
    processes; the default, 1, runs in this process and is the fastest for this few drops (see
    DEFAULT_WORKERS). The verdict and the drops are the same for any worker count.

    ``disc``: sample X and Y inside the box's X-Y disc (issue #86). The default samples along X only,
    as the loop's gate does (issue #97): Y error counts in the full map's Z, not in this filter."""
    if box.task != "placement":
        raise ValueError(f"the quick check covers placement boxes (issue #46), got {box.task!r}")
    limit = max_failures(n, need)
    cfg = cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    disc = bool(disc)
    poses = check_poses(n, seed, disc=disc)
    samples = [disc_sample(box, unit, cfg) if disc else BS.pose_sample(box, 1.0, unit, cfg) for _, unit in poses]
    t0 = time.time()
    with BS.DropPool(family, p, cfg, workers=min(workers or DEFAULT_WORKERS, n)) as pool:
        t_ready = time.time()
        result = tally(pool.imap([(course, s) for (course, _), s in zip(poses, samples)]), n, need)
        used = pool.workers
    seconds = time.time() - t0
    drops = [drop_row(i, course, unit, sample, row)
             for i, ((course, unit), sample, row) in enumerate(zip(poses, samples, result.pop("rows")))]
    outcomes = {}
    for d in drops:
        outcomes[d["outcome"]] = outcomes.get(d["outcome"], 0) + 1
    return {
        "label": LABEL, "task": "placement", "family": family.name, "family_version": family.version,
        "params": p.to_dict(), "box": box.to_dict(), "seed": seed, "n": n, "need": need, "max_failures": limit,
        "disc": disc,
        "config": json.loads(json.dumps(T.as_dict(cfg))), "workers": used,
        **result, "outcomes": dict(sorted(outcomes.items())),
        "drops": drops, "failures": [d for d in drops if d["outcome"] != SUCCESS],
        "seconds": seconds, "stage_seconds": {"build": t_ready - t0, "drops": seconds - (t_ready - t0)},
    }


SCREEN_RADIUS_MM = 16.0  # the disc of the screen's positions: the floor, the disc the score Z is counted over


def quarter_poses(n=N_DROPS, radius_mm=SCREEN_RADIUS_MM, seed=0, placements=None):
    """``n`` seeded (placement, x_mm, y_mm) inside the ``radius_mm`` disc, the placements taken in
    turn (so each gets n / 4 and a check stopped early has judged every one) and (x, y) from one
    Latin hypercube over radius squared and angle, so the positions are even over the disc's area."""
    from brixzle.engine import radius as RA

    placements = tuple(placements or RA.QUARTERS)
    max_failures(n, 0)
    if not (math.isfinite(radius_mm) and radius_mm > 0):
        raise ValueError(f"radius_mm must be finite and > 0, got {radius_mm!r}")
    u = qmc.LatinHypercube(d=2, seed=seed).random(n)
    out = []
    for k, (a, b) in enumerate(u):
        r, phi = radius_mm * math.sqrt(a), 2.0 * math.pi * b
        out.append((placements[k % len(placements)], round(r * math.cos(phi), 4), round(r * math.sin(phi), 4)))
    return out


def _quarter_counts(outcomes):
    s, j, n = outcomes.count(SUCCESS), outcomes.count("jammed"), len(outcomes)
    return {"S": s, "J": j, "N": n, "Z": round((s - j) / n, 4) if n else None}


def quarter_screen(family, p, seed=0, n=N_DROPS, need=NEED_SEATED, radius_mm=SCREEN_RADIUS_MM, drop=None):
    """The quarter screen (#222): ``quarter_poses`` dropped in order in this process, one drop rig
    per placement, judged by ``tally`` (at least ``need`` of ``n`` seated, stopping when that is out
    of reach). ``drop(index, placement, x_mm, y_mm) -> outcome`` replaces the sim (for tests).
    Returns the verdict, the counts and Z per quarter and in all, and every judged drop."""
    from brixzle.engine import gridscore as GS
    from brixzle.engine import radius as RA
    from brixzle.engine import zscore as ZS

    t0 = time.time()
    poses = quarter_poses(n, radius_mm, seed)
    if drop is None:
        span = family.lattice(p).span_voxels
        if span != 2:  # the corbel layouts are laid out for a two-voxel brick, as in gridscore
            return {"label": LABEL, "kind": "quarter_screen", "family": getattr(family, "name", None), "seed": seed,
                    "n": n, "need": need, "radius_mm": radius_mm, "passed": False, "seated": 0, "failed": 0,
                    "dropped": 0, "stopped_early": False, "outcomes": {}, "z": None, "quarters": {}, "drops": [],
                    "error": f"the placements are laid out for a two-voxel brick; span_voxels = {span}",
                    "seconds": round(time.time() - t0, 2)}
        cfg = RA._ideal_cfg(False)  # the free drop (#218)
        z = GS.release_z_mm(GS.GridSettings())
        rigs = {}

        def drop(k, pl, x, y):
            if pl not in rigs:
                rigs[pl] = T.DropRig(family, p, cfg, course=RA.PLACEMENTS[pl])
            row = rigs[pl].drop(ZS._sample(cfg, {"x": x, "y": y, "z": z, "t": 0.0}), trace=False)
            stable = all(math.isfinite(float(v)) for v in [*row.get("err_mm", []), row.get("angle_deg", 0.0)])
            return row["outcome"] if stable else UNSTABLE

    rows = ({"index": k, "placement": pl, "x_mm": x, "y_mm": y, "outcome": drop(k, pl, x, y)}
            for k, (pl, x, y) in enumerate(poses))
    res = tally(rows, n, need)
    judged = res.pop("rows")
    outcomes = {}
    for d in judged:
        outcomes[d["outcome"]] = outcomes.get(d["outcome"], 0) + 1
    quarters = {pl: _quarter_counts([d["outcome"] for d in judged if d["placement"] == pl])
                for pl in dict.fromkeys(pl for pl, _, _ in poses)}
    every = _quarter_counts([d["outcome"] for d in judged])
    return {"label": LABEL, "kind": "quarter_screen", "family": getattr(family, "name", None), "seed": seed,
            "n": n, "need": need, "radius_mm": radius_mm, **res, "outcomes": dict(sorted(outcomes.items())),
            "z": every["Z"], "quarters": quarters, "drops": judged, "seconds": round(time.time() - t0, 2)}


def new_run_dir(root, family_name, statistic):
    """A new runs/check-<UTC stamp>-<family>-<target> directory; never reuses one."""
    root = Path(root)
    while True:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        path = root / f"check-{stamp}-{family_name}-{statistic}"
        try:
            path.mkdir(parents=True, exist_ok=False)
            return path
        except FileExistsError:
            continue


def write_check(result, runs="runs"):
    """Write a check result to a new run directory as check.json; returns the directory."""
    text = json.dumps(result, indent=2, allow_nan=False) + "\n"  # validate before making the directory
    out = new_run_dir(runs, result["family"], result["box"]["statistic"])
    (out / "check.json").write_text(text, encoding="utf-8")
    return out


def summary(result):
    """One line for the terminal."""
    counts = ", ".join(f"{k} {v}" for k, v in result["outcomes"].items())
    early = f", stopped at failure {result['failed']}" if result["stopped_early"] else ""
    return (f"{result['family']} placement {result['box']['statistic']}: "
            f"{'PASS' if result['passed'] else 'FAIL'}, {result['seated']} seated of {result['dropped']} dropped "
            f"(needs {result['need']} of {result['n']}{early}; {counts}), seed {result['seed']}, "
            f"{result['workers']} workers, {result['seconds']:.1f} s ({LABEL})")


def parser():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--family", default="ab")
    ap.add_argument("--statistics", nargs="+", choices=BX.STATISTICS, default=list(BX.STATISTICS),
                    help="target boxes to check (default: mean, p95 and max)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n", type=int, default=N_DROPS)
    ap.add_argument("--need", type=int, default=NEED_SEATED)
    ap.add_argument("--workers", type=int, default=DEFAULT_WORKERS,
                    help=f"drop processes (default {DEFAULT_WORKERS}: the fastest for this few drops)")
    ap.add_argument("--runs", default="runs")
    ap.add_argument("--quarters", action="store_true",
                    help="the quarter screen (#222) on the stack and overhangs 1-3; --family may be root:module")
    return ap


def main(argv=None):
    from brixzle.families import registry

    args = parser().parse_args(argv)
    if args.quarters:
        from brixzle.tools import core as TC

        family = TC.load(args.family)
        result = quarter_screen(family, family.defaults, seed=args.seed, n=args.n, need=args.need)
        out = new_run_dir(args.runs, family.name, "quarters")
        (out / "screen.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        per = ", ".join(f"{pl} Z {v['Z']}" for pl, v in result["quarters"].items())
        print(f"{family.name} quarter screen: {'PASS' if result['passed'] else 'FAIL'}, {result['seated']} seated of "
              f"{result['dropped']} (needs {result['need']} of {result['n']}); Z {result['z']} ({per}); "
              f"{result['seconds']:.1f} s ({LABEL}) -> {out}")
        return
    family = registry.get(args.family)
    boxes = BX.default_boxes()["placement"]
    for stat in args.statistics:
        result = quick_check(family, family.defaults, boxes[stat], n=args.n, need=args.need, seed=args.seed,
                             workers=args.workers)
        out = write_check(result, args.runs)
        print(f"{summary(result)} -> {out}")


if __name__ == "__main__":
    main()
