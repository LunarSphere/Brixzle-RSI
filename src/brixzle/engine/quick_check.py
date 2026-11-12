"""Quick random check (issue #47): a go/no-go gate before the full sim map.

The full map (#48) takes minutes, and most new designs should be turned back in seconds. So a new
design is first dropped at 40 poses inside the target box (X, Z and theta around the aim), in the
MuJoCo sim (trials.DropRig), half onto the base (course 0) and half onto a seated brick (course 1):

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

    uv run python -m brixzle.engine.quick_check --family ab --seed 0
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


def check_poses(n=N_DROPS, seed=0):
    """``n`` seeded poses as (course, unit point): the unit point is (x, z, theta) in [-1, 1], as
    box_search.pose_sample takes it at scale 1.

    One Latin hypercube in four dimensions: X, Z, theta, and a fourth that picks the course, so
    every axis is evenly covered and the courses split the poses evenly (exactly, for even n)."""
    max_failures(n, 0)
    u = qmc.LatinHypercube(d=4, seed=seed).random(n)
    return [(int(row[3] >= 0.5), 2.0 * row[:3] - 1.0) for row in u]


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
           "dx_mm": float(sample["dx"] - sample["aim"]), "clear_mm": float(sample["clear"]),
           "pitch_deg": float(sample["pitch"]), "outcome": row["outcome"] if stable else UNSTABLE,
           "err_mm": [_num(v) for v in row["err_mm"]], "angle_deg": _num(row["angle_deg"])}
    if out["outcome"] != SUCCESS:
        out.update(contacts=row.get("contacts", []), settled=bool(row.get("settled", False)),
                   time_s=_num(row.get("time", 0.0)), f_max_N=_num(row.get("f_max", 0.0)),
                   penetration_mm=_num(row.get("penetration_mm", 0.0)))
    return out


def quick_check(family, p, box, cfg=None, n=N_DROPS, need=NEED_SEATED, seed=0, workers=DEFAULT_WORKERS):
    """Drop ``n`` evenly spread poses inside a placement box; passes if at least ``need`` seat.

    Returns a JSON-safe dict: the verdict and counts, every drop that was judged ('drops', in pose
    order) and the failing ones with their contact traces ('failures'). ``cfg`` is a
    trials.TrialConfig (default: the ideal error model, as the box search). ``workers``: drop
    processes; the default, 1, runs in this process and is the fastest for this few drops (see
    DEFAULT_WORKERS). The verdict and the drops are the same for any worker count."""
    if box.task != "placement":
        raise ValueError(f"the quick check covers placement boxes (issue #46), got {box.task!r}")
    limit = max_failures(n, need)
    cfg = cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    poses = check_poses(n, seed)
    samples = [BS.pose_sample(box, 1.0, unit, cfg) for _, unit in poses]
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
        "config": json.loads(json.dumps(T.as_dict(cfg))), "workers": used,
        **result, "outcomes": dict(sorted(outcomes.items())),
        "drops": drops, "failures": [d for d in drops if d["outcome"] != SUCCESS],
        "seconds": seconds, "stage_seconds": {"build": t_ready - t0, "drops": seconds - (t_ready - t0)},
    }


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
    return ap


def main(argv=None):
    from brixzle.families import registry

    args = parser().parse_args(argv)
    family = registry.get(args.family)
    boxes = BX.default_boxes()["placement"]
    for stat in args.statistics:
        result = quick_check(family, family.defaults, boxes[stat], n=args.n, need=args.need, seed=args.seed,
                             workers=args.workers)
        out = write_check(result, args.runs)
        print(f"{summary(result)} -> {out}")


if __name__ == "__main__":
    main()
