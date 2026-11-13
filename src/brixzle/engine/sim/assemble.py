# From LunarSphere/aerial-assembly brixzle/src/brixzle/trials.py (assemble, push_test, expected_seat) and brixzle/src/brixzle/lattice.py (connectivity, unsupported, footprint, _path_clear, assembly_order) @ fc4f724
# Changed: they take a family and its params instead of BrickParams and cad bundles. The part is family.part_for, the seat family.seat_pose, the stroke axis family.insertion_axis, U and H0 come from family.lattice, and the approach stroke (tooth_L + slot_extra) is family.release_lift_mm; the path check's vertical sweep is 3 (lift + H0) instead of 3 (amplitude + H0). The footprint is builders.section's outer moved to the seat pose (identity, or a 180-degree turn about Z, which mirrors X). The support graph is a small dict instead of networkx. release_pose, pose_error, classify, run_until_settled, ErrorModel and Tolerance are reused from engine/sim/trials.py. assemble returns the result dict (with seconds, family, params, seed and config); _assemble also returns the scene, for push_test. write_build writes a run directory. New: an optional seeded release jitter, and repeat_builds / write_repeats, which judge a structure on seeded repeats (issue #98's finding: a symmetric ideal drop tips either way on noise).
"""Sequential assembly and push tests with any family (issue #81).

``assemble`` places a structure's bricks in a planned order, judges each against its seat on
its actual supporter, and detects collapse. ``push_test`` builds a structure ideally and pushes
its top brick sideways until it moves (R3). No drone dynamics. Every result is simulated,
uncalibrated contact, not a hardware rate.

``repeat_builds`` judges a structure on several seeded builds, each with a tiny release jitter: an
ideal drop onto a symmetric seat can tip either way on numerical noise (issue #98: A/B's bridge
flipped between 7 of 7 and 3 of 7 when friction moved by 5e-5), so one deterministic build is not
a reliable verdict. The structure passes when most repeats complete.

    uv run python -m brixzle.engine.sim.assemble wall --family ab --ideal
"""
import argparse
from datetime import datetime, timezone
import json
import math
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
import os
from pathlib import Path
import time

import mujoco
import numpy as np
from shapely import affinity
from shapely.ops import unary_union

from brixzle.engine.sim import trials as T
from brixzle.engine.sim.scene import activate, body_pose, build_scene
from brixzle.engine.structures import CATALOG, Structure, brick_name, structure_for, voxels
from brixzle.families import builders as B
from brixzle.families.interface import Family, Placement

LABEL = "simulated, uncalibrated contact"
REPEATS = 5        # builds per structure (maintainer, 2026-11-13: "e.g. 5"), judged by majority
JITTER_MM = 0.05   # release jitter in X and Y per brick: breaks symmetric ties, far below the drone's error


# --- the support graph (from lattice.py) ---


class SupportGraph:
    """Supporter -> supported, one edge per engaged voxel. Nodes are 'base<i>' and brick names."""

    def __init__(self):
        self.nodes, self.edges = {}, []

    def add_node(self, name, **data):
        self.nodes[name] = data

    def add_edge(self, u, v, voxel):
        self.edges.append((u, v, voxel))

    def predecessors(self, name):
        out = []
        for u, v, _ in self.edges:
            if v == name and u not in out:
                out.append(u)
        return out

    def in_degree(self, name):
        return len(self.predecessors(name))


def connectivity(structure: Structure, span=2):
    """Directed support graph; raises ValueError if two bricks use one voxel."""
    g = SupportGraph()
    for i, spec in enumerate(structure.bases):
        g.add_node(f'base{i}', kind='base', voxels=tuple(spec[:2]))
    for b in structure.bricks:
        g.add_node(brick_name(b), kind='brick', placement=b)
    occupied = {}
    for b in structure.bricks:
        for v in voxels(b, span):
            key = (b.course, v)
            if key in occupied:
                raise ValueError(f'Voxel {key} used by {occupied[key]} and {brick_name(b)}')
            occupied[key] = brick_name(b)
    for b in structure.bricks:
        for v in voxels(b, span):
            if b.course == 0:
                for i, spec in enumerate(structure.bases):
                    if spec[0] <= v < spec[1]:
                        g.add_edge(f'base{i}', brick_name(b), voxel=v)
            elif (b.course - 1, v) in occupied:
                g.add_edge(occupied[(b.course - 1, v)], brick_name(b), voxel=v)
    return g


def unsupported(graph):
    return [n for n, d in graph.nodes.items() if d['kind'] == 'brick' and graph.in_degree(n) == 0]


# --- footprints and assembly order ---


def _turned(quat):
    """False for the identity, True for a 180-degree turn about Z; ValueError otherwise."""
    q = np.asarray(quat, dtype=float)
    if np.allclose(q, [1, 0, 0, 0], atol=1e-9) or np.allclose(q, [-1, 0, 0, 0], atol=1e-9):
        return False
    if np.allclose(q, [0, 0, 0, 1], atol=1e-9) or np.allclose(q, [0, 0, 0, -1], atol=1e-9):
        return True
    raise ValueError(f'a structure footprint needs a seat pose that is the identity or a 180-degree turn '
                     f'about Z; got quaternion {q.tolist()}')


def footprint(family: Family, p, b: Placement, offset=(0., 0.), outers=None):
    """The brick's section outline (XZ) at its seat, moved by ``offset`` (mm)."""
    part = family.part_for(b.course, b.direction)
    outers = outers if outers is not None else {}
    if part not in outers:
        outers[part] = B.section(family, p, part).outer
    pos, quat = family.seat_pose(p, b)
    g = outers[part]
    if _turned(quat):
        g = affinity.scale(g, xfact=-1.0, yfact=1.0, origin=(0, 0))
    return affinity.translate(g, float(pos[0]) + offset[0], float(pos[2]) + offset[1])


def _path_clear(family: Family, p, b, placed_shapes, outers=None, samples=10):
    """Nominal approach: vertical drop, then the final stroke along the insertion axis."""
    d = np.asarray(family.insertion_axis(p, b), dtype=float)
    lift = family.release_lift_mm(p)
    lat = family.lattice(p)
    start = -lift*d
    offsets = [tuple(start*s) for s in np.linspace(0, 1, samples)]
    offsets += [(start[0], start[1] + dz) for dz in np.linspace(0, 3*(lift + lat.H0_mm), samples)]
    swept = unary_union([footprint(family, p, b, o, outers) for o in offsets]).buffer(-0.05)
    return not any(swept.intersection(s).area > 0.5 for s in placed_shapes)


def assembly_order(family: Family, p, structure: Structure):
    """Greedy topological order: supporters first, approach path clear of placed bricks.

    Prefers the lowest course, then the most-supported brick. Raises ValueError if the
    structure cannot be completed (e.g. a brick that must enter between two
    already placed neighbours against its stroke).
    """
    span = family.lattice(p).span_voxels
    graph = connectivity(structure, span)
    if unsupported(graph):
        raise ValueError(f'Unsupported bricks: {unsupported(graph)}')
    outers = {}
    placed, shapes, order = set(), [], []
    remaining = list(structure.bricks)
    bases = {n for n, d in graph.nodes.items() if d['kind'] == 'base'}
    while remaining:
        ready = [b for b in remaining
                 if all(u in placed or u in bases for u in graph.predecessors(brick_name(b)))]
        # Fill each course from the side the stroke points to, so the approach side stays open;
        # fall back to any clear brick.
        ready.sort(key=lambda b: (b.course, np.sign(family.insertion_axis(p, b)[0])*-b.i0))
        for b in ready:
            if _path_clear(family, p, b, shapes, outers):
                break
        else:
            raise ValueError(f'No clear insertion path for {[brick_name(b) for b in ready]}')
        order.append(b)
        placed.add(brick_name(b))
        shapes.append(footprint(family, p, b, outers=outers))
        remaining.remove(b)
    return order


# --- the trials (from trials.py) ---


def expected_seat(family: Family, p, data, info, placement, ref):
    """Seat of ``placement`` carried along with supporter ref=(k, placement_k)'s actual pose."""
    k, rp = ref
    tpos, tquat = family.seat_pose(p, placement)
    rpos, rquat = body_pose(data, info, k)
    ipos, iquat = family.seat_pose(p, rp)
    delta = T._rot(rquat)*T._rot(iquat).inv()
    q = (delta*T._rot(tquat)).as_quat()
    return rpos + delta.apply(np.asarray(tpos) - np.asarray(ipos)), np.array([q[3], q[0], q[1], q[2]])


def _assemble(family: Family, p, structure, cfg: T.TrialConfig, seed=0, stop_on_fail=True, record=None,
              jitter_mm=0.0):
    t0 = time.time()
    lat = family.lattice(p)
    order = assembly_order(family, p, structure)
    graph = connectivity(structure, lat.span_voxels)
    index = {brick_name(b): j for j, b in enumerate(order)}
    parts = [family.part_for(b.course, b.direction) for b in order]
    bodies = {k: B.sim_bodies(family, p, k) for k in dict.fromkeys(parts)}
    model, xml, info = build_scene(family, p, bodies, structure.bases, len(order), cfg.physics,
                                   visual=record is not None, body_parts=parts)
    data = mujoco.MjData(model)
    owner = T._brick_geoms(model, info)
    rng = np.random.default_rng(seed)
    jitter = np.random.default_rng([seed, 98])  # its own stream, so the error samples do not change
    stages, placed = [], []
    status = 'complete'
    for j, b in enumerate(order):
        before = [body_pose(data, info, i) for i in range(j)]
        sample = cfg.error.sample(rng)
        if b.course < cfg.error.exact_courses:
            sample = dict(dx=0., dy=0., yaw=0., roll=0., pitch=0., clear=cfg.error.clearance[0], vel=[0., 0., 0.])
        if jitter_mm > 0:
            ddx, ddy = jitter.uniform(-jitter_mm, jitter_mm, 2)
            sample = {**sample, 'dx': float(sample['dx'] + ddx), 'dy': float(sample['dy'] + ddy)}
        supporters = [u for u in graph.predecessors(brick_name(b)) if u in index]
        ref = (index[supporters[0]], order[index[supporters[0]]]) if supporters else None
        seat = expected_seat(family, p, data, info, b, ref) if cfg.error.track_supporter and ref else None
        pos, quat = T.release_pose(family, p, b, sample, seat)
        activate(model, data, info, j, pos, quat, linvel=sample['vel'])
        m = T.run_until_settled(model, data, info, j, cfg.tolerance, owner, watch=range(j), frames=record)
        err, angle = T.pose_error(family, p, data, info, j, b, ref)
        outcome = T.classify(lat, err, angle, cfg.tolerance, m['floor'])
        moved, sag = 0., 0.
        for i, prev in enumerate(placed):
            pos_i, quat_i = body_pose(data, info, i)
            turn = math.degrees((T._rot(before[i][1]).inv()*T._rot(quat_i)).magnitude())
            moved = max(moved, float(np.linalg.norm(pos_i - before[i][0])), turn*lat.U_mm*math.pi/180)
            sag = max(sag, float(np.linalg.norm(T.pose_error(family, p, data, info, i, prev)[0])))
        collapsed = moved > cfg.tolerance.drift or sag > lat.H0_mm/2 or m['floor']
        stages.append({'brick': brick_name(b), 'part': parts[j], 'sample': sample, **m, 'outcome': outcome,
                       'err_mm': err.tolist(), 'angle_deg': angle,
                       'stage_motion_mm': moved, 'sag_mm': sag, 'collapsed': bool(collapsed)})
        placed.append(b)
        if collapsed:
            status = 'collapse'
            break
        if outcome != 'seated' and stop_on_fail:
            status = f'placement_{outcome}'
            break
    stable = sum(1 for s in stages if s['outcome'] == 'seated' and not s['collapsed'])
    result = {'structure': structure.name, 'status': status, 'placed': len(stages), 'total': len(order),
              'stable_bricks': stable, 'max_sag_mm': max([s['sag_mm'] for s in stages], default=0.),
              'stages': stages, 'seconds': time.time() - t0, 'family': family.name, 'params': p.to_dict(),
              'seed': seed, 'jitter_mm': float(jitter_mm), 'config': T.as_dict(cfg), 'label': LABEL}
    return result, (model, data, info, xml)


def assemble(family: Family, p, structure, cfg: T.TrialConfig, seed=0, stop_on_fail=True, record=None,
             jitter_mm=0.0):
    """Place bricks in planned order with sampled errors; detect failures and collapse.

    A new brick is judged against its seat on its actual supporter. Collapse is
    motion of already placed bricks while the new one settles (beyond
    ``tolerance.drift``), floor contact, or cumulative sag beyond half a course.
    ``record`` optionally receives qpos frames for every stage. ``jitter_mm`` moves each release by a
    seeded uniform offset in X and Y within that bound (0: none). Returns the result dict:
    structure, status, placed, total, stable_bricks, max_sag_mm, stages and seconds.
    """
    return _assemble(family, p, structure, cfg, seed, stop_on_fail, record, jitter_mm)[0]


def _repeat(job):
    family, p, structure, cfg, seed, jitter_mm = job
    return assemble(family, p, structure, cfg, seed=seed, jitter_mm=jitter_mm)


def repeat_structures(family: Family, p, structures, cfg: T.TrialConfig, n=REPEATS, seed=0, jitter_mm=JITTER_MM,
                      workers=None):
    """``repeat_builds`` for several structures, every build in one process pool; returns
    {structure name: result}. ``workers`` defaults to one per build, up to the CPU count."""
    if not isinstance(n, int) or isinstance(n, bool) or n < 1:
        raise ValueError(f"n must be an integer >= 1, got {n!r}")
    if not (isinstance(jitter_mm, (int, float)) and math.isfinite(jitter_mm) and jitter_mm >= 0):
        raise ValueError(f"jitter_mm must be finite and >= 0, got {jitter_mm!r}")
    structures = list(structures)
    if len({s.name for s in structures}) != len(structures):
        raise ValueError("structure names must be unique")
    t0 = time.time()
    seeds = [seed + k for k in range(n)]
    jobs = [(family, p, s, cfg, k, float(jitter_mm)) for s in structures for k in seeds]
    workers = min(workers or len(jobs), len(jobs), os.cpu_count() or 1)
    if workers > 1:
        with ProcessPoolExecutor(workers, mp_context=multiprocessing.get_context("spawn")) as ex:
            builds = list(ex.map(_repeat, jobs))
    else:
        builds = [_repeat(job) for job in jobs]
    seconds = time.time() - t0
    return {s.name: _judged(family, p, s, cfg, seeds, jitter_mm, builds[i * n:(i + 1) * n], seconds, workers)
            for i, s in enumerate(structures)}


def repeat_builds(family: Family, p, structure, cfg: T.TrialConfig, n=REPEATS, seed=0, jitter_mm=JITTER_MM,
                  workers=None):
    """Build ``structure`` ``n`` times, with seeds seed .. seed + n - 1 and a release jitter of up to
    ``jitter_mm``; it passes when more than half complete. ``workers`` (default: one per repeat, up to
    the CPU count) runs the repeats in parallel processes; the results do not depend on it."""
    return repeat_structures(family, p, [structure], cfg, n, seed, jitter_mm, workers)[structure.name]


def _judged(family, p, structure, cfg, seeds, jitter_mm, builds, seconds, workers):
    n = len(builds)
    complete = sum(b['status'] == 'complete' for b in builds)
    return {'structure': structure.name, 'family': family.name, 'params': p.to_dict(), 'config': T.as_dict(cfg),
            'n': n, 'seeds': seeds, 'jitter_mm': float(jitter_mm), 'complete': complete, 'fraction': complete / n,
            'passed': complete > n / 2, 'statuses': [b['status'] for b in builds],
            'stable_bricks': [b['stable_bricks'] for b in builds], 'total': builds[0]['total'],
            'builds': builds, 'seconds': seconds, 'workers': workers, 'label': LABEL}


def push_test(family: Family, p, structure, cfg: T.TrialConfig, rate=0.2, limit=3.0, direction=(1, 0, 0)):
    """Ideal assembly, then a ramped horizontal force on the top brick until it moves (R3)."""
    ideal = T.TrialConfig(cfg.physics, T.ErrorModel(ideal=True, aim_bias=cfg.error.aim_bias), cfg.tolerance)
    result, (model, data, info, _) = _assemble(family, p, structure, ideal)
    if result['status'] != 'complete':
        return {'break_force_N': 0., 'status': result['status']}
    top = result['total'] - 1
    start, _ = body_pose(data, info, top)
    body = info['bodies'][top]
    f = 0.
    while f < limit:
        f += rate*model.opt.timestep
        data.xfrc_applied[body, :3] = np.asarray(direction)*f
        mujoco.mj_step(model, data)
        pos, _ = body_pose(data, info, top)
        if np.linalg.norm(pos - start) > cfg.tolerance.drift:
            return {'break_force_N': f, 'status': 'moved'}
    return {'break_force_N': limit, 'status': 'held'}


# --- run directory ---


def _jsonable(v):
    if isinstance(v, dict):
        return {str(k): _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, (np.floating, np.integer)):
        v = v.item()
    if isinstance(v, np.ndarray):
        return _jsonable(v.tolist())
    if isinstance(v, float) and not math.isfinite(v):
        raise ValueError('a build result holds a non-finite number')
    return v


def _write(result, runs, prefix, name):
    text = json.dumps(_jsonable({**result, 'label': LABEL}), indent=1, allow_nan=False) + "\n"
    root = Path(runs)
    while True:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        out = root / f"{prefix}-{stamp}-{result['family']}-{result['structure']}"
        try:
            out.mkdir(parents=True, exist_ok=False)
            break
        except FileExistsError:
            continue
    (out / name).write_text(text, encoding="utf-8")
    return out


def write_build(result, runs="runs"):
    """Write ``runs/build-<UTC>-<family>-<structure>/build.json``; never reuses a directory."""
    return _write(result, runs, "build", "build.json")


def write_repeats(result, runs="runs"):
    """Write ``runs/builds-<UTC>-<family>-<structure>/builds.json`` (every repeat, in full); never
    reuses a directory."""
    return _write(result, runs, "builds", "builds.json")


def main(argv=None):
    from brixzle.families import registry

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("structure", choices=sorted(CATALOG))
    ap.add_argument("--family", default="ab")
    ap.add_argument("--ideal", action="store_true", help="no placement error (the only mode #81 reports)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--repeats", type=int, default=REPEATS,
                    help=f"seeded builds with release jitter, judged by majority (default {REPEATS}); "
                         "1 builds once, without jitter")
    ap.add_argument("--runs", default="runs")
    args = ap.parse_args(argv)
    fam = registry.get(args.family)
    p = fam.defaults
    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True)) if args.ideal else T.TrialConfig()
    structure = structure_for(fam, p, args.structure)
    if args.repeats == 1:
        res = assemble(fam, p, structure, cfg, seed=args.seed)
        out = write_build(res, args.runs)
        print(f"{res['structure']}: {res['status']}, {res['stable_bricks']} of {res['total']} stable, "
              f"max sag {res['max_sag_mm']:.2f} mm, build {res['seconds']:.1f} s ({LABEL}) -> {out}")
        return
    res = repeat_builds(fam, p, structure, cfg, n=args.repeats, seed=args.seed)
    out = write_repeats(res, args.runs)
    print(f"{res['structure']}: {'passed' if res['passed'] else 'failed'}, {res['complete']} of {res['n']} complete "
          f"(stable bricks {res['stable_bricks']} of {res['total']}), {res['seconds']:.1f} s ({LABEL}) -> {out}")


if __name__ == "__main__":
    main()
