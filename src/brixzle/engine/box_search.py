"""Placement sim verifier (R11): the largest scale of a target box where every drop seats.

A box (engine/boxes.py) gives half-widths in X, Z and theta around the aim point. At scale s
the verifier drops the brick at s times a fixed set of unit points: the centre, the 8 corners,
the 6 face centres and a few seeded interior points. A scale passes only if every drop seats
(no jam, fall or wrong valley). Bisection on s finds the largest passing scale; s >= 1 means
the design covers the target box. Every drop logs its contact sequence (trials.DropRig).

Pose mapping, around the aim (ErrorModel.aim_bias):
- X: dx = aim + s * x * half_width_x (along the lattice).
- Z: release-height error. The drone aims to release s * half_width_z above the minimum
  clearance (ErrorModel.clearance[0]), so the lowest corner releases at the minimum and
  nothing overlaps at release: clear = clearance[0] + s * half_width_z * (1 + z).
- theta: pitch, rotation in the XZ profile plane.
Y, roll and yaw are zero: the placement box is the profile plane.

Results are simulated, uncalibrated contact. Run it from the command line to record a run:

    uv run python -m brixzle.engine.box_search --family ab --statistic mean --seed 0
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import numpy as np

from brixzle.engine import boxes as BX
from brixzle.engine.sim import trials as T
from brixzle.engine.sim.trials import DropRig

LABEL = "simulated, uncalibrated contact"
CORNERS = np.array([(x, z, a) for x in (-1, 1) for z in (-1, 1) for a in (-1, 1)], dtype=float)
FACES = np.array([v for i in range(3) for v in (np.eye(3)[i], -np.eye(3)[i])], dtype=float)

__all__ = ["DropRig", "unit_points", "pose_sample", "bisect_scale", "box_search"]


def unit_points(n_interior=4, seed=0):
    """(n, 3) unit-box points (x, z, theta) in [-1, 1]: centre, corners, faces, seeded interior.

    The centre comes first so a design that fails at the aim fails on the first drop."""
    interior = np.random.default_rng(seed).uniform(-0.9, 0.9, size=(n_interior, 3))
    return np.vstack([np.zeros((1, 3)), CORNERS, FACES, interior])


def pose_sample(box: BX.TargetBox, scale, unit, cfg: T.TrialConfig):
    """The drop sample (ErrorModel.sample keys) for a unit point at a box scale."""
    w = box.half_widths
    x, z, a = (float(v) for v in unit)
    aim = cfg.error.aim_bias
    return {"dx": aim + scale * x * w["x_mm"], "dy": 0.0, "aim": aim,
            "yaw": 0.0, "roll": 0.0, "pitch": scale * a * w["theta_deg"],
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


def box_search(family, p, box: BX.TargetBox, cfg=None, course=1, seed=0, s_max=2.0, tol=0.05, n_interior=4):
    """Bisect the scale of a placement box for one family; returns a JSON-safe result dict.

    course 1 drops onto a seated course-0 brick (for A/B, a B onto an A), the common case in a
    structure; course 0 drops onto the anchored base."""
    if box.task != "placement":
        raise ValueError("box_search verifies placement boxes; pickup is the pickup trial (#9)")
    cfg = cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    t0 = time.time()
    rig = DropRig(family, p, cfg, course=course)
    unit = unit_points(n_interior, seed)
    drops = []

    def ok(scale):
        for k, u in enumerate(unit):
            row = rig.drop(pose_sample(box, scale, u, cfg))
            drops.append({"scale": scale, "point": k, "unit": u.tolist(), **row})
            if row["outcome"] != "seated":
                return False  # fail fast: one failing pose fails the scale
        return True

    scale, evals = bisect_scale(ok, s_max, tol)
    for e in evals:
        e["drops"] = sum(1 for d in drops if d["scale"] == e["scale"])
    failures = [d for d in drops if d["outcome"] != "seated"]
    return {
        "label": LABEL,
        "family": family.name, "family_version": family.version, "params": p.to_dict(),
        "box": box.to_dict(), "course": course, "seed": seed, "s_max": s_max, "tol": tol,
        "n_interior": n_interior, "config": json.loads(json.dumps(T.as_dict(cfg))),
        "scale": scale, "passed": scale >= 1.0,
        "evaluations": evals,
        "failure_outcomes": sorted({d["outcome"] for d in failures}),
        "drops": drops,
        "drop_count": len(drops), "seconds": time.time() - t0,
    }


def _new_run_dir(root, family, statistic):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    path = Path(root) / f"box-search-{stamp}-{family}-placement-{statistic}"
    path.mkdir(parents=True, exist_ok=False)  # never overwrite an earlier run
    return path


def main(argv=None):
    from brixzle.families import registry

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--family", default="ab")
    ap.add_argument("--statistic", choices=BX.STATISTICS, default="mean")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--course", type=int, choices=(0, 1), default=1)
    ap.add_argument("--tol", type=float, default=0.05)
    ap.add_argument("--runs", default="runs")
    args = ap.parse_args(argv)
    family = registry.get(args.family)
    box = BX.default_boxes()["placement"][args.statistic]
    result = box_search(family, family.defaults, box, course=args.course, seed=args.seed, tol=args.tol)
    if not all(math.isfinite(d["angle_deg"]) for d in result["drops"]):
        raise SystemExit("non-finite drop result")
    out = _new_run_dir(args.runs, family.name, args.statistic)
    (out / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"{family.name} placement {args.statistic}: scale {result['scale']:.3f} "
          f"({'passes' if result['passed'] else 'fails'}), {result['drop_count']} drops, "
          f"{result['seconds']:.1f} s -> {out}")


if __name__ == "__main__":
    main()
