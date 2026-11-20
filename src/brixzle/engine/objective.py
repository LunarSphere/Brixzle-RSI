# The noisy corbel (radius_error, noisy_corbel, build_record) is from LunarSphere/Brixzle-RSI
# src/brixzle/engine/edge_capture.py @ a6a4152 (local branch 145-edge-capture, closed issue #148).
# Changed: the error is a disc of any radius (not the mean box only), there is no edge-stage comparison,
# and the steps are summarised per radius (#164).
"""What a real build needs, beside capture (#164): an error-tolerant overhang, a grip the arm can
hold, and a size and release height that keep the arm's reach.

Allround (simulated, uncalibrated contact) won capture Z on the wide boxes and then could not be
gripped from above (it shifts 1.45 mm and turns 0.80 deg over a 100 mm lift, #153), is 116 mm long
(the arm reaches a 9-step overhang, against 11 for A/B), releases from 62 mm, and seated 0 of 7
overhang bricks in the arm build. Capture Z alone rewarded that. These terms are the rest.

- **Error-tolerant overhang** (``overhang_tolerance``): the back-first corbel built with every
  brick released at a seeded error inside a disc of radius r about its supporter's seat (and the
  mean box's release heights), at r = 0, 1, 2, 3, 4, 5, 7, 10 and 14.8 mm, stopping one radius past the
  first whose median falls short. The gate builds it with ideal
  placement. At the mean box's 14.8 mm every design reached a median of 0 to 1 steps (#148), so one
  radius cannot rank them; the score is the largest radius at which the median build still reaches
  ``tolerance_steps``.
- **Grip** (``grip_rule``, ``grip_lift``): #153's top-down stock-jaw grasp. The rule at its strict
  setting, as the maintainer asked: the fingertip window within 2 mm of the centre of mass along the
  brick (#153 widens this to 4, 8 and 16 mm in turn) and at least 30 % fingertip coverage (#153
  accepts 15 %). The rule is geometry only (a few seconds). The lift then picks the brick from the
  mould station and lifts it 100 mm: it passes within 1 mm and 1 deg.
- **Size and release** (``size``, ``arm_reach``): brick length, depth and pitch, the release lift,
  and the longest overhang and the bridge(3) riser the SO-101 reaches with the real grasp (#152).

``evaluate_design`` records them all; ``violations``, ``dominates`` and ``rank`` apply ``Settings``.
These settings are the proposal of #164; the loop's acceptance rule does not use them yet.
"""
from dataclasses import asdict, dataclass, field
import contextlib
import math
import re
import time

import numpy as np

LABEL = "simulated, uncalibrated contact"
MEAN_RADIUS_MM = 14.8


@dataclass(frozen=True)
class Settings:
    """The proposed objective (#164). Hard constraints, then the scored terms."""

    max_length_mm: float = 70.0         # twinhook4 is 68 mm, A/B 50; allround's 116 cut the arm's overhang to 9
    max_release_mm: float = 30.0        # A/B 23, twinhook4 26; allround 62
    min_arm_overhang: int = 11          # A/B's, with the real grasp (#153)
    max_bridge_riser_mm: float = 10.0   # A/B and twinhook4 need none, gentle 10 (taller courses), allround 30
    grip_mm: float = 1.0                # #153's lift bar
    grip_deg: float = 1.0
    radii_mm: tuple = (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 7.0, 10.0, MEAN_RADIUS_MM)
    past_failure: int = 1               # radii still built after the first that fails (for the record)
    builds: int = 17                    # #148's count: one half-supported attempt each, at worst
    corbel_steps: int = 6
    tolerance_steps: int = 3            # the gate's own floor for the overhang (R4)
    scored: tuple = ("z_target", "tolerance_mm", "z_mu08", "overhang_n")


# --- the error-tolerant overhang ---


def radius_error(radius_mm, n=64):
    """Placement error spread evenly over a disc of ``radius_mm`` and the mean box's release heights,
    aimed at the supporter's measured pose. Angles and residual speed are zero."""
    from brixzle.engine import boxes as BX
    from brixzle.engine.sim import trials as T

    if not (math.isfinite(radius_mm) and radius_mm >= 0):
        raise ValueError(f"radius_mm must be finite and >= 0, got {radius_mm!r}")
    box = BX.default_boxes()["placement"]["mean"]
    c0 = T.ErrorModel().clearance[0]
    radii = (0.0,) if radius_mm == 0 else tuple(radius_mm * math.sqrt((k + 0.5) / n) for k in range(n))
    return T.ErrorModel(radial_mode="bootstrap", radii_mm=radii, sigma_yaw=0.0, sigma_tilt=0.0, sigma_vel=0.0,
                        clearance=(c0, c0 + 2 * box.half_widths["z_mm"]), track_supporter=True)


def _placement(brick):
    course, voxel = re.fullmatch(r"b(\d+)_(-?\d+)m?", brick).groups()
    return int(course), int(voxel)


def build_record(build):
    """One noisy corbel: the reaching steps seated from course 0 up, and where it stopped."""
    stages = build["stages"]
    good = {st["brick"] for st in stages if st["outcome"] == "seated" and not st["collapsed"]}
    steps = 0
    while f"b{steps}_{steps}" in good:
        steps += 1
    failed = stages[-1]["brick"] if build["status"] != "complete" and stages else None
    kind = None
    if failed is not None:
        c, v = _placement(failed)
        kind = "reach" if c >= 1 and v == c else "back" if v == c - 2 else "first"
    return {"status": build["status"], "steps": steps, "failed_brick": failed, "failed_kind": kind,
            "failed_outcome": stages[-1]["outcome"] if failed is not None else None,
            "seconds": float(build["seconds"])}


def noisy_corbel(family, p, radius_mm, builds=17, steps=6, seed=0, workers=None):
    """The corbel built ``builds`` times with error inside a disc of ``radius_mm`` (seeds seed..)."""
    from brixzle.engine import structures as S
    from brixzle.engine.sim import assemble as A
    from brixzle.engine.sim import trials as T

    t0 = time.time()
    cfg = T.TrialConfig(error=radius_error(radius_mm))
    res = A.repeat_builds(family, p, S.corbel(steps, family.lattice(p).span_voxels), cfg, n=builds, seed=seed,
                          jitter_mm=0.0, workers=workers)
    recs = [build_record(b) for b in res["builds"]]
    return {"radius_mm": radius_mm, "builds": recs, "seeds": res["seeds"],
            "median_steps": float(np.median([b["steps"] for b in recs])),
            "complete": sum(b["status"] == "complete" for b in recs),
            "failed_at": {k: sum(b["failed_kind"] == k for b in recs) for k in ("first", "back", "reach")},
            "seconds": round(time.time() - t0, 1)}


def tolerance_radius(median_by_radius, steps):
    """The largest radius, going up from the smallest, at which the median build reaches ``steps``
    before the first that does not; None if the smallest does not."""
    best = None
    for r in sorted(median_by_radius):
        if median_by_radius[r] >= steps:
            best = r
        else:
            break
    return best


def overhang_tolerance(family, p, settings=Settings(), seed=0, workers=None):
    """The noisy corbel at growing radii, until ``past_failure`` radii after the first whose median
    build falls short of ``tolerance_steps`` (larger radii cannot raise the tolerance radius)."""
    t0 = time.time()
    per, after = {}, None
    for r in sorted(settings.radii_mm):
        per[r] = noisy_corbel(family, p, r, settings.builds, settings.corbel_steps, seed, workers)
        if after is None and per[r]["median_steps"] < settings.tolerance_steps:
            after = 0
        elif after is not None:
            after += 1
        if after is not None and after >= settings.past_failure:
            break
    medians = {r: v["median_steps"] for r, v in per.items()}
    return {"label": LABEL, "by_radius": {str(r): v for r, v in per.items()}, "median_steps": {str(r): m for r, m in medians.items()},
            "tolerance_mm": tolerance_radius(medians, settings.tolerance_steps), "steps_needed": settings.tolerance_steps,
            "seconds": round(time.time() - t0, 1)}


# --- grip ---


@contextlib.contextmanager
def _grasp_bounds(com_x):
    from brixzle.engine.sim import grasp as G

    old = G.COM_X_MM
    G.COM_X_MM = com_x
    try:
        yield
    finally:
        G.COM_X_MM = old


STRICT = {"com_x_mm": (2.0,), "min_coverage": 0.3}


def grasps(family, p, strict=True):
    """{part: Grasp or the reason there is none}, at the strict setting or #153's."""
    from brixzle.engine.sim import grasp as G

    out = {}
    with _grasp_bounds(STRICT["com_x_mm"] if strict else G.COM_X_MM):
        for part in family.parts:
            try:
                out[part] = G.choose_grasp(family, p, part, **({"min_coverage": STRICT["min_coverage"]} if strict else {}))
            except ValueError as e:
                out[part] = str(e)
    return out


def grip_rule(family, p, strict=True):
    """Whether every part has a top-down stock-jaw grasp (geometry only; ``STRICT`` or #153's setting)."""
    t0 = time.time()
    g = grasps(family, p, strict)
    parts = {k: asdict(v) if not isinstance(v, str) else {"none": v} for k, v in g.items()}
    return {"strict": strict, "passed": all(not isinstance(v, str) for v in g.values()), "parts": parts,
            "seconds": round(time.time() - t0, 1)}


def grip_lift(family, p, settings=Settings(), strict=True):
    """Each part picked from the mould station with its grasp and lifted 100 mm (#153): its shift and
    turn in the jaws. Without a strict grasp, #153's relaxed one is lifted, for the record."""
    from brixzle.engine.sim import pickplace as PP

    t0 = time.time()
    g = grasps(family, p, strict)
    used = "strict"
    if any(isinstance(v, str) for v in g.values()):
        g, used = grasps(family, p, strict=False), "relaxed"
    out = {}
    for part, grasp in g.items():
        if isinstance(grasp, str):
            out[part] = {"error": grasp}
            continue
        try:
            r = PP.pick(PP.station_rig(family, p, part, grasp=grasp))
            out[part] = {k: float(r[k]) for k in ("slip_mm", "slip_deg", "lifted_mm")}
        except RuntimeError as e:
            out[part] = {"error": str(e)}
    ok = all("error" not in v and v["slip_mm"] <= settings.grip_mm and v["slip_deg"] <= settings.grip_deg for v in out.values())
    return {"label": LABEL, "grasp": used, "passed": ok and used == "strict", "parts": out,
            "seconds": round(time.time() - t0, 1)}


# --- size and the arm's reach ---


def size(family, p):
    from brixzle.families import builders as B

    bodies = [B.sim_bodies(family, p, part) for part in family.parts]
    pts = np.vstack([piece for b in bodies for piece in (*b.collision, *b.soft_collision)])
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    lat = family.lattice(p)
    return {"length_mm": float(hi[0] - lo[0]), "depth_mm": float(hi[1] - lo[1]), "height_mm": float(hi[2] - lo[2]),
            "pitch_mm": float(lat.U_mm), "course_mm": float(lat.H0_mm), "release_lift_mm": float(family.release_lift_mm(p))}


def arm_reach(family, p, env=None, up_to=12):
    """The longest +x overhang and the bridge(3) the SO-101 reaches from one base with the real grasp
    (#153's ``longest_overhang`` in scripts/arm_grasp.py), and the riser each base needs."""
    from brixzle.engine import structures as S
    from brixzle.engine.sim import arm as A
    from brixzle.engine.sim import pickplace as PP

    t0 = time.time()
    env = env if env is not None else A.top_down_envelope(step_mm=5.0)
    g = grasps(family, p, strict=False)
    if any(isinstance(v, str) for v in g.values()):
        return {"overhang": 0, "bridge3": None, "reason": "no grasp", "seconds": round(time.time() - t0, 1)}
    span = family.lattice(p).span_voxels
    over = {"courses": 0, "riser_mm": None}
    for n in range(up_to, 0, -1):
        st = S.overhang(n, +1, span)
        tcp, yaws = PP.grasp_tcp_mm(family, p, st.bricks, g, with_yaw=True)
        plan = A.plan_base(family, p, st, tcp, env, yaws=yaws)
        if plan is not None:
            over = {"courses": n, "riser_mm": float(plan[4])}
            break
    st = S.bridge(3, span)
    tcp, yaws = PP.grasp_tcp_mm(family, p, st.bricks, g, with_yaw=True)
    plan = A.plan_base(family, p, st, tcp, env, yaws=yaws)
    return {"overhang": over["courses"], "overhang_riser_mm": over["riser_mm"],
            "bridge_riser_mm": None if plan is None else float(plan[4]), "seconds": round(time.time() - t0, 1)}


# --- the objective ---


def violations(rec, s=Settings()):
    """The hard constraints ``rec`` breaks (``evaluate_design``'s record)."""
    out = []
    g = rec.get("grip", {})
    if not g.get("rule") or g.get("lift_mm") is None or g["lift_mm"] > s.grip_mm or g.get("lift_deg", 99) > s.grip_deg:
        out.append("grip")
    if rec["size"]["length_mm"] > s.max_length_mm:
        out.append("length")
    if rec["size"]["release_lift_mm"] > s.max_release_mm:
        out.append("release_lift")
    if rec["arm"]["overhang"] < s.min_arm_overhang:
        out.append("arm_overhang")
    riser = rec["arm"].get("bridge_riser_mm")
    if riser is None or riser > s.max_bridge_riser_mm:
        out.append("bridge_riser")
    return out


def _scores(rec, s):
    vals = {"z_target": rec["z"].get("mean"), "tolerance_mm": rec.get("tolerance_mm"), "z_mu08": rec.get("z_mu08"),
            "overhang_n": rec.get("overhang_n")}
    return [(-1.0 if vals[k] is None else float(vals[k])) for k in s.scored]


def dominates(a, b, s=Settings()):
    """``a`` is no worse than ``b`` on every scored term and better on one."""
    x, y = _scores(a, s), _scores(b, s)
    return all(u >= v for u, v in zip(x, y)) and any(u > v for u, v in zip(x, y))


def rank(recs, s=Settings()):
    """Feasible designs first, by Pareto layer (layer 1 is dominated by none); then the infeasible,
    by fewest violations. Ties by error-tolerance radius, then Z."""
    recs = [{**r, "violations": violations(r, s)} for r in recs]
    for r in recs:
        r["feasible"] = not r["violations"]
    out, left, layer = [], [r for r in recs if r["feasible"]], 1
    while left:
        front = [r for r in left if not any(dominates(o, r, s) for o in left if o is not r)]
        for r in sorted(front, key=lambda r: [-v for v in _scores(r, s)][1:2] + [-_scores(r, s)[0]]):
            out.append({**r, "layer": layer})
        left = [r for r in left if r not in front]
        layer += 1
    bad = sorted((r for r in recs if not r["feasible"]), key=lambda r: (len(r["violations"]), [-v for v in _scores(r, s)]))
    return out + [{**r, "layer": None} for r in bad]


def evaluate_design(family, p=None, settings=Settings(), env=None, z=None, z_mu08=None, overhang_n=None, workers=None):
    """Every term for one design. Z, Z at mu 0.8 and the gate's overhang come from its evaluation
    (passed in: they are not rerun here)."""
    p = p or family.defaults
    t0 = time.time()
    rule = grip_rule(family, p, strict=True)
    lift = grip_lift(family, p, settings)
    lift_parts = [v for v in lift["parts"].values() if "error" not in v]
    tol = overhang_tolerance(family, p, settings, workers=workers)
    rec = {"name": family.name, "label": LABEL, "settings": asdict(settings),
           "grip": {"rule": rule["passed"], "lift_grasp": lift["grasp"], "lift_passed": lift["passed"],
                    "lift_mm": max((v["slip_mm"] for v in lift_parts), default=None),
                    "lift_deg": max((v["slip_deg"] for v in lift_parts), default=None),
                    "rule_parts": rule["parts"], "lift_parts": lift["parts"]},
           "size": size(family, p), "arm": arm_reach(family, p, env), "overhang_tolerance": tol,
           "tolerance_mm": tol["tolerance_mm"], "z": z or {}, "z_mu08": z_mu08, "overhang_n": overhang_n,
           "seconds": {"grip_rule": rule["seconds"], "grip_lift": lift["seconds"], "tolerance": tol["seconds"]}}
    rec["seconds"]["total"] = round(time.time() - t0, 1)
    rec["violations"] = violations(rec, settings)
    return rec
