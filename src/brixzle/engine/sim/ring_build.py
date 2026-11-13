# From LunarSphere/aerial-assembly brixzle/src/brixzle/ring.py @ fc4f724 (assemble, _release, _expected)
# Changed: it builds what the family's RingKit plans (RingBrick poses and cells) instead of A/B's Brick list, with bodies from builders.ring_bodies and the kit's base as static pieces; the release lift is family.release_lift_mm plus the sampled clearance; the collapse test is #81's (a placed brick moves more than tolerance.drift, sag beyond half a course, or floor contact); it stops at the first brick that does not seat; a family with no kit gives status 'no_ring_kit'; the result is written to a run directory.
"""The ring build (issue #89): a closed n x n ring of a family's bricks, course by course.

Each brick of the kit's plan is released above its seat with the error model's sample (ideal by
default), judged against its seat carried on its actual supporter, and the build stops at the
first brick that does not seat or at a collapse. Every result is simulated, uncalibrated contact.

    uv run python -m brixzle.engine.sim.ring_build --family ab --courses 2
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from brixzle.engine.sim import assemble as A
from brixzle.engine.sim import trials as T
from brixzle.engine.sim.scene import activate, body_pose, build_scene
from brixzle.families import builders as B

LABEL = A.LABEL
NO_KIT = "no_ring_kit"


def _rot(q):
    return Rotation.from_quat([q[1], q[2], q[3], q[0]])


def _expected(data, info, order, j, ref):
    """Seat of brick j carried with its supporter ``ref`` (index)'s measured pose."""
    tpos, tquat = np.asarray(order[j].pos_mm), np.asarray(order[j].quat)
    if ref is None:
        return tpos, tquat
    rpos, rquat = body_pose(data, info, ref)
    ipos, iquat = np.asarray(order[ref].pos_mm), np.asarray(order[ref].quat)
    delta = _rot(rquat) * _rot(iquat).inv()
    q = (delta * _rot(tquat)).as_quat()
    return rpos + delta.apply(tpos - ipos), np.array([q[3], q[0], q[1], q[2]])


def _release(family, p, sample, seat):
    pos, quat = seat
    R = _rot(quat)
    lift = family.release_lift_mm(p) + sample["clear"]
    offset = R.apply([sample["dx"], sample["dy"], 0.])   # the aim bias is upstream of the stroke (local +x)
    err = Rotation.from_euler("xyz", [sample["roll"], sample["pitch"], sample["yaw"]], degrees=True)
    q = (err * R).as_quat()
    return pos + offset + np.array([0, 0, lift]), np.array([q[3], q[0], q[1], q[2]])


def _result(family, p, n, courses, cfg, seed, status, stages, total, t0):
    seated = sum(s["outcome"] == "seated" and not s["collapsed"] for s in stages)
    return {"structure": f"ring{n}x{courses}", "status": status, "seated": seated, "placed": len(stages),
            "total": total, "stages": stages, "seconds": time.time() - t0, "family": family.name,
            "params": p.to_dict(), "n": n, "courses": courses, "seed": seed, "config": T.as_dict(cfg),
            "label": LABEL}


def build_ring(family, p, n=6, courses=2, cfg=None, runs="runs", seed=0, record=None):
    """Build the kit's n x n ring of ``courses`` courses; returns the result with its run directory
    ('run'). Status: 'complete', 'placement_<outcome>', 'collapse' or 'no_ring_kit'."""
    t0 = time.time()
    cfg = cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    if family.ring is None:
        res = _result(family, p, n, courses, cfg, seed, NO_KIT, [], 0, t0)
        return {**res, "run": str(write_ring(res, runs))}
    lat = family.lattice(p)
    order = family.ring.plan(p, n, courses)
    parts = [b.part for b in order]
    bodies = {k: B.ring_bodies(family, p, k) for k in dict.fromkeys(parts)}
    model, _, info = build_scene(family, p, bodies, [], len(order), cfg.physics, visual=record is not None,
                                 body_parts=parts, static=family.ring.base(p, n))
    data = mujoco.MjData(model)
    owner = T._brick_geoms(model, info)
    rng = np.random.default_rng(seed)
    where, stages, status = {}, [], "complete"
    for j, b in enumerate(order):
        before = [body_pose(data, info, i) for i in range(j)]
        sup = [where[(b.course - 1, c)] for c in b.cells if (b.course - 1, c) in where]
        ref = max(set(sup), key=sup.count) if sup else None
        sample = cfg.error.sample(rng)
        seat = _expected(data, info, order, j, ref) if cfg.error.track_supporter else \
            (np.asarray(b.pos_mm), np.asarray(b.quat))
        pos, quat = _release(family, p, sample, seat)
        activate(model, data, info, j, pos, quat, linvel=sample["vel"])
        m = T.run_until_settled(model, data, info, j, cfg.tolerance, owner, watch=range(j), frames=record)
        epos, equat = _expected(data, info, order, j, ref)
        apos, aquat = body_pose(data, info, j)
        err = _rot(equat).inv().apply(apos - epos)        # in the brick frame
        angle = math.degrees((_rot(equat).inv() * _rot(aquat)).magnitude())
        outcome = T.classify(lat, err, angle, cfg.tolerance, m["floor"])
        moved, sag = 0., 0.
        for i in range(j):
            pi, qi = body_pose(data, info, i)
            turn = math.degrees((_rot(before[i][1]).inv() * _rot(qi)).magnitude())
            moved = max(moved, float(np.linalg.norm(pi - before[i][0])), turn * lat.U_mm * math.pi / 180)
            sag = max(sag, float(np.linalg.norm(pi - np.asarray(order[i].pos_mm))))
        collapsed = moved > cfg.tolerance.drift or sag > lat.H0_mm / 2 or m["floor"]
        stages.append({"brick": b.name, "part": b.part, **m, "outcome": outcome, "err_mm": err.tolist(),
                       "angle_deg": angle, "stage_motion_mm": moved, "sag_mm": sag, "collapsed": bool(collapsed)})
        for c in b.cells:
            where[(b.course, c)] = j
        if collapsed:
            status = "collapse"
            break
        if outcome != "seated":
            status = f"placement_{outcome}"
            break
    res = _result(family, p, n, courses, cfg, seed, status, stages, len(order), t0)
    return {**res, "run": str(write_ring(res, runs))}


def write_ring(result, runs="runs"):
    """Write ``runs/ring-<UTC>-<family>/ring.json``; never reuses a directory. Finite numbers only."""
    text = json.dumps(A._jsonable({**result, "label": LABEL}), indent=1, allow_nan=False) + "\n"
    root = Path(runs)
    while True:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        out = root / f"ring-{stamp}-{result['family']}"
        try:
            out.mkdir(parents=True, exist_ok=False)
            break
        except FileExistsError:
            continue
    (out / "ring.json").write_text(text, encoding="utf-8")
    return out


def failure_for_agent(result):
    """The ring's result for the design agent: the status and the stage where it stopped."""
    out = {"status": result["status"], "seated": result["seated"], "total": result["total"], "run": result.get("run")}
    if result["status"] == NO_KIT:
        out["fix"] = "give the family a ring kit (engine/ring_cells.kit_from_cell from one cell is the simplest)"
    elif result["stages"]:
        s = result["stages"][-1]
        out["stopped_at"] = {"brick": s["brick"], "part": s["part"], "outcome": s["outcome"],
                             "err_mm": [round(v, 2) for v in s["err_mm"]], "angle_deg": round(s["angle_deg"], 2),
                             "collapsed": s["collapsed"], "sag_mm": round(s["sag_mm"], 2)}
    return out


def main(argv=None):
    from brixzle.families import registry

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--family", default="ab")
    ap.add_argument("--n", type=int, default=6)
    ap.add_argument("--courses", type=int, default=2)
    ap.add_argument("--runs", default="runs")
    args = ap.parse_args(argv)
    fam = registry.get(args.family)
    r = build_ring(fam, fam.defaults, n=args.n, courses=args.courses, runs=args.runs)
    print(f"{r['structure']}: {r['status']}, {r['seated']} of {r['total']} seated, {r['seconds']:.1f} s ({LABEL}) "
          f"-> {r['run']}")


if __name__ == "__main__":
    main()
