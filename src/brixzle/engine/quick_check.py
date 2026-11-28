"""The quarter screen (#222): a go/no-go filter before the grid score.

The director runs it in verify, after the MVP gate and before the grid (``director/verify.measure_radius``):
40 drops on the score's four placements (``radius.QUARTERS``: the stack and overhangs 1, 2 and 3),
10 each, interleaved, at seeded random positions spread evenly over the 16 mm disc (one Latin
hypercube over radius squared and angle), released in the grid's free drop (#218).

- **Pass rule (issue #47's):** at least 32 of 40 seated. The check stops at the 9th failure, because
  32 is then out of reach. Poses are judged in order, so the result does not depend on timing. A
  design that cannot seat one of the four placements fails: 10 failures are more than the 8 allowed.
- **Why 40 and 32** (binomial, worked on issue #47): a design that truly seats 60% of drops passes
  0.6% of the time, and one that seats 90% passes 98.5% of the time.

Its Z, (S - J) / N per quarter and over all its drops, has the form of the score but is not the score.
The older box-based quick check of the first loop was removed in #227. Results are simulated,
uncalibrated contact.

    uv run python -m brixzle.engine.quick_check --family <name or root:module> --seed 0
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
from brixzle.engine.sim import trials as T

LABEL = BS.LABEL
N_DROPS = 40
NEED_SEATED = 32
SUCCESS = BS.SUCCESS["placement"]
UNSTABLE = "unstable"  # the sim returned a non-finite pose: reported as a failure, never written as NaN
SCREEN_RADIUS_MM = 16.0  # the disc of the screen's positions: the floor, the disc the score Z is counted over


def max_failures(n=N_DROPS, need=NEED_SEATED):
    """The number of failures at which ``need`` seated of ``n`` is out of reach (9 for 32 of 40)."""
    for name, v in (("n", n), ("need", need)):
        if not isinstance(v, (int, np.integer)) or isinstance(v, bool):
            raise ValueError(f"{name} must be an integer, got {v!r}")
    if n < 1 or not 0 <= need <= n:
        raise ValueError(f"need 0 <= need <= n and n >= 1, got n = {n}, need = {need}")
    return n - need + 1


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


def parser():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--family", required=True, help="a registry name or <root>:<module>")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n", type=int, default=N_DROPS)
    ap.add_argument("--need", type=int, default=NEED_SEATED)
    ap.add_argument("--runs", default="runs")
    ap.add_argument("--quarters", action="store_true", help="accepted for older commands; the screen is always on the quarters")
    return ap


def main(argv=None):
    from brixzle.tools import core as TC

    args = parser().parse_args(argv)
    family = TC.load(args.family)
    result = quarter_screen(family, family.defaults, seed=args.seed, n=args.n, need=args.need)
    out = new_run_dir(args.runs, family.name, "quarters")
    (out / "screen.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    per = ", ".join(f"{pl} Z {v['Z']}" for pl, v in result["quarters"].items())
    print(f"{family.name} quarter screen: {'PASS' if result['passed'] else 'FAIL'}, {result['seated']} seated of "
          f"{result['dropped']} (needs {result['need']} of {result['n']}); Z {result['z']} ({per}); "
          f"{result['seconds']:.1f} s ({LABEL}) -> {out}")


if __name__ == "__main__":
    main()
