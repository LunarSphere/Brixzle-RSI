"""Seeded stage-B release scores and sequential Crazyflie corbel builds (#192)."""
from dataclasses import asdict, dataclass, fields, replace
from functools import cache
import json
import math
from pathlib import Path
from statistics import median
from time import perf_counter

import numpy as np

from brixzle.engine import boxes, objective, structures, zscore
from brixzle.engine.sim.drone import CF21B
from brixzle.engine.sim.held import ForkFitError, HeldConfig, HeldRig, WristConfig, mission_samples
from brixzle.engine.sim.pickup import tine_segment

RADII = objective.OVERHANG_SCORE_RADII_MM
R4_STEPS = objective.Settings.tolerance_steps


@dataclass(frozen=True)
class HeldMapSettings:
    seed: int = 192
    samples: int = 2
    builds: int = 3
    reaches: int = 4

    def __post_init__(self):
        for key, minimum in (("seed", 0), ("samples", 1), ("builds", 1), ("reaches", R4_STEPS)):
            value = getattr(self, key)
            if type(value) is not int or value < minimum:
                raise ValueError(f"{key} must be an integer >= {minimum}")


def gripper_config(spec, override=None):
    """Load candidate fork/wrist and motion settings. The physical drone and contact model are fixed."""
    if override is None:
        from brixzle.tools.core import _root_module
        root, module = _root_module(spec)
        path = Path(root) / module / "gripper.json"
        if not path.exists():
            return HeldConfig()
    else:
        path = Path(override)
    data = json.loads(path.read_text())
    allowed = {f.name for f in fields(HeldConfig)} - {"drone", "physics", "release_at_contact"}
    if not isinstance(data, dict) or data.keys() - allowed:
        raise ValueError(f"invalid gripper settings in {path}; allowed keys are {sorted(allowed)}")
    wrist = data.pop("wrist", {})
    if not isinstance(wrist, dict) or wrist.keys() - {f.name for f in fields(WristConfig)}:
        raise ValueError("invalid wrist settings")
    for name, value in wrist.items():
        if name != "rigid":
            if not isinstance(value, list) or not all(type(x) in (float, int) for x in value):
                raise ValueError(f"{name} needs three numbers")
            wrist[name] = tuple(value)
    if not all(type(v) in (int, float) for v in data.values()):
        raise ValueError("gripper dimensions and motion settings must be numbers")
    return HeldConfig(**data, wrist=WristConfig(**wrist))


def mean_samples(n, seed):
    """Condition empirical target draws on the mean disc, height and pitch without clipping."""
    box = boxes.default_boxes()["placement"]["mean"].half_widths
    rng = np.random.default_rng(seed)
    accepted = []
    while len(accepted) < n:
        _, s = mission_samples(int(rng.integers(2**32)))
        if (math.hypot(s.dx_m, s.dy_m)*1000 <= box["x_mm"] and abs(s.dz_m)*1000 <= box["z_mm"]
                and abs(math.degrees(s.dpitch_rad)) <= box["theta_deg"]):
            accepted.append(s)
    return accepted


@cache
def _disc(radius):
    return objective.radius_error(radius)


def disc_sample(sample, radius, seed):
    dx, dy = _disc(radius)._xy(np.random.default_rng([seed, 192]))
    return replace(sample, dx_m=dx*.001, dy_m=dy*.001)


def case_summary(rows):
    phases = {}
    for name in rows[0]["phases"]:
        observations = [r["phases"][name] for r in rows]
        passed = sum(p["passed"] for p in observations)
        skipped = sum(p["outcome"] == "skipped" for p in observations)
        phases[name] = dict(passed=passed, skipped=skipped, failed=len(rows)-passed-skipped,
                            wall_s=sum(p["wall_s"] for p in observations))
    return {**zscore.counts(r["outcome"] for r in rows), "phases": phases,
            "missions_passed": sum(r["passed"] for r in rows)}


def reach_summary(steps, limit, pitch_mm):
    """The zero course is fully supported. Only protruding, completed courses add reach."""
    by_length = []
    reliable = 0
    for length in range(1, limit + 1):
        successes = sum(n >= length for n in steps)
        if successes * 2 > len(steps):
            reliable = length
        by_length.append(dict(steps=length, reach_mm=length*pitch_mm, successes=successes,
                              builds=len(steps), rate=successes/len(steps)))
    return dict(by_length=by_length, median_steps=median(steps), reliable_steps=reliable,
                reliable_reach_mm=reliable*pitch_mm, r4_passed=reliable >= R4_STEPS)


def build_corbel(structure, stage_rig, seed, radius):
    """Carry forward every free body's measured pose and velocity; stop on a miss or collapse."""
    start = perf_counter()
    rng = np.random.default_rng(seed)
    states, stages = [], []
    steps = mission_steps = 0
    passed = True
    for i, b in enumerate(structure.bricks):
        draw = int(rng.integers(2**32))
        pickup, target = mission_samples(draw)
        if radius is not None:
            target = disc_sample(target, radius, draw)
        rig = stage_rig(i)
        row = rig.trial(target, pickup_sample=pickup, support_states=states)
        off = rig.off_seat()
        stages.append(dict(brick=structures.brick_name(b), draw_seed=draw, off_seat=off, trial=row))
        passed &= row["passed"]
        if row["outcome"] != "seated" or off:
            break
        states = rig.brick_states()
        if b.i0 == b.course:
            steps = b.course
            if passed:
                mission_steps = steps
    return dict(seed=seed, stages=stages, steps=steps, mission_steps=mission_steps, seconds=perf_counter()-start)


def same_fork(a, b):
    """Compare plate and shaft geometry relative to the mount in the carried pose."""
    def geometry(fork):
        mount = np.asarray(fork["drone_pos_mm"])
        segments = [tine_segment(fork, x, z) - mount for x, z in sorted(fork["tines"])]
        return np.concatenate(([fork["tine_r"], fork["tine_dx_dy"]],
                               (np.asarray(fork["plate"]) - mount).ravel(), np.ravel(segments)))

    left, right = geometry(a), geometry(b)
    return left.shape == right.shape and np.allclose(left, right, rtol=0, atol=1e-8)


def heldmap(family, p, cfg=HeldConfig(), settings=HeldMapSettings()):
    start = perf_counter()
    lat = family.lattice(p)
    report = dict(kind="heldmap", family=family.name, params=p.to_dict(), label=objective.LABEL,
                  wrist_defaults="guess", gripper=asdict(cfg), settings=asdict(settings),
                  mean_box=boxes.default_boxes()["placement"]["mean"].to_dict(),
                  approach_source="error-exp3-r4.json", hover_source="hover-exp3-r2.json",
                  rows=[], cases={}, first_reach_by_radius={}, builds={}, scores={}, violations=[])
    if lat.span_voxels != structures.SPAN:
        return dict(report, mission_passed=False, violations=["held_corbel_span"], seconds=perf_counter()-start)
    structure = structures.corbel(settings.reaches + 1, lat.span_voxels)

    fork = None
    max_payload = 0.

    def make_rig(placement, build=None):
        nonlocal fork, max_payload
        rig = HeldRig(family, p, placement, cfg, structure=build)
        if fork is not None and not same_fork(fork, rig.fork):
            raise ForkFitError("courses would require differently shaped mounted forks")
        fork = rig.fork
        max_payload = max(max_payload, rig.payload)
        return rig

    @cache
    def stage_rig(i):
        return make_rig(structure.bricks[i], structure)

    def record(name, placement, rows):
        report["rows"].extend(dict(case=name, placement=placement, trial=r) for r in rows)
        return rows

    sample_seeds = range(settings.seed, settings.seed + settings.samples)
    empirical = []
    try:
        targets = mean_samples(settings.samples, settings.seed)
        for name, placements in (("full", (0, 1)), ("plus_bare", ("plus_bare",)),
                                 ("minus_bare", ("minus_bare",)), ("first_reach", ("plus_anchored",))):
            rows = []
            for placement in placements:
                rig = make_rig(placement)
                rows.extend(record(name, placement, [rig.trial(s) for s in targets]))
                empirical.extend(record("mission_" + name, placement, [rig.trial(seed=s) for s in sample_seeds]))
                if placement == "plus_anchored":
                    first_reach = rig
            report["cases"][name] = case_summary(rows)
            report["scores"][f"held_{name}_z"] = report["cases"][name]["Z"]

        for radius in RADII:
            targets = [disc_sample(mission_samples(seed)[1], radius, seed) for seed in sample_seeds]
            rows = record(f"first_reach_{radius:g}mm", "plus_anchored",
                          [first_reach.trial(s) for s in targets])
            report["first_reach_by_radius"][f"{radius:g}"] = case_summary(rows)

        for radius in (None, *RADII):
            builds = [build_corbel(structure, stage_rig, seed, radius)
                      for seed in range(settings.seed, settings.seed + settings.builds)]
            key = "measured" if radius is None else f"{radius:g}"
            report["builds"][key] = dict(
                **reach_summary([b["steps"] for b in builds], settings.reaches, lat.U_mm),
                missions=reach_summary([b["mission_steps"] for b in builds], settings.reaches, lat.U_mm),
                builds=builds)
    except ForkFitError as exc:
        return dict(report, mission_passed=False, violations=["held_gripper_fit"], fit_error=str(exc),
                    seconds=perf_counter()-start)

    measured = report["builds"]["measured"]
    medians = {f"{r:g}": report["builds"][f"{r:g}"]["median_steps"] for r in RADII}
    report["median_steps_by_radius"] = medians
    report["scores"].update({f"held_median_steps_{r}mm": steps for r, steps in medians.items()})
    report["scores"].update(held_overhang_mean_steps=objective.overhang_mean_steps(medians),
                            held_mission_reach_mm=measured["missions"]["reliable_reach_mm"],
                            held_reliable_reach_mm=measured["reliable_reach_mm"],
                            held_reliable_steps=measured["reliable_steps"],
                            held_mission_rate=sum(r["passed"] for r in empirical)/len(empirical))
    report["missions"] = case_summary(empirical)
    report["reliable_reach_by_length"] = measured["by_length"]
    report["max_payload_kg"] = max_payload
    if report["max_payload_kg"] > CF21B.payload_max_g*.001:
        report["violations"].append("held_payload")
    if not measured["r4_passed"]:
        report["violations"].append("held_r4")
    report["mission_passed"] = (not report["violations"] and all(r["passed"] for r in empirical)
                                and measured["missions"]["r4_passed"])
    report["seconds"] = perf_counter()-start
    return report
