"""Analytic placement predictor (R11): step a released brick's 2D section through its contacts.

For one release pose and one friction coefficient mu:

1. drop straight down to the first contact (exact vertical distance);
2. classify the contact state (contact_modes, after Balkcom & Trinkle 2002 and Pang & Trinkle 2000);
   away from the seat, weak stability alone stops it (a possible jam, not further classified):
   - strongly stable: no solution moves (frictionless-stable, or weakly stable with no moving mode);
   - indeterminate: weakly stable, but some solution moves: a possible jam, counted as a failure;
   - moving: no equilibrium; take the first feasible moving mode (the previous one first);
3. step the pose along that mode's acceleration (velocity reset each step: the paper's model is at
   rest), bisecting a step that would penetrate, then re-detect contacts;
4. stop when strongly stable or indeterminate, when it falls off, or at the step limit.

A strongly stable end pose is classified like the sim (trials.classify): seated, wrong valley or
jammed. A pose is seated only if it seats at every mu (0.2-0.5). Each state in the trace records its
contacts, mode, sticking margin (P1: > 0 means gravity is outside the sticking cone), and the
wedging and opposing-faces flags (P2); a seat records its stability margin (P4).

Outputs are labelled "predicted, quasi-static 2D". No momentum, bounce or Y effects (the A/B gutter
shear) are modelled.
"""

import math

import numpy as np
from scipy.spatial.transform import Rotation

from brixzle.engine import box_search as BS
from brixzle.engine.predictor import planar as PL
from brixzle.engine.predictor.contact_modes import ContactSystem
from brixzle.engine.sim import trials as T
from brixzle.families import builders as B
from brixzle.families.interface import Lattice, Placement, Section

LABEL = "predicted, quasi-static 2D"
MUS = (0.2, 0.3, 0.4, 0.5)
G = np.array([0., -1., 0.])  # unit weight
# Largest point travel per step (mm). On A/B V0 (seed 0, tol 0.05) the predicted scales, course 0/1:
#   step 0.25 mm: mean 0.594/0.719, p95 0.375/0.375, max 0.281/0.250
#   step 0.5 mm:  mean 0.594/0.781, p95 0.375/0.375, max 0.281/0.281
#   step 1.0 mm:  mean 0.625/0.781, p95 0.375/0.375, max 0.250/0.281
# (issue #10 PR), so 0.5 mm agrees with 0.25 mm to within two bisection steps.
DS_MM = 0.5
MAX_CONTACTS = 4  # for mode enumeration (4^k LPs); stability tests use every contact


def _mu_key(mu):
    return f"{mu:g}"


def _spread(contacts, n):
    """The contacts reordered so the first n are far apart (farthest-point order), and how many
    are left over (enumerated as passive 'S': non-penetrating, no force)."""
    if len(contacts) <= n:
        return contacts, 0
    pts = np.array([c.point for c in contacts])
    chosen = [int(np.argmin(pts[:, 0]))]
    while len(chosen) < n:
        d = np.min(np.linalg.norm(pts[:, None] - pts[chosen][None], axis=-1), axis=1)
        chosen.append(int(np.argmax(d)))
    rest = [i for i in range(len(contacts)) if i not in chosen]
    return [contacts[i] for i in chosen + rest], len(rest)


def _slip_corner(sys, cs, com, body, U, mu):
    """A corner balanced on a corner holds only if every admissible normal holds it; if one face
    normal lets it slip, it slips (the balance is measure-zero). Returns the system to use."""
    for i, c in enumerate(cs):
        for n in c.alternatives:
            trial = list(cs)
            trial[i] = type(c)(c.point, n)
            alt = ContactSystem(trial, com, body.M(), U)
            if not alt.weakly_stable(G, mu):
                return alt, trial
    return sys, cs


def _advance(body, pose, acc, h):
    com = PL.com_world(body, pose) + acc[:2] * h
    th = pose[2] + acc[2] * h
    origin = com - PL.rot(th) @ np.asarray(body.com, float)
    return np.array([origin[0], origin[1], th])


def _end_class(pose, seat, U, tol):
    err = np.array([pose[0] - seat[0], 0., pose[1] - seat[1]])
    angle = abs(math.degrees(pose[2] - seat[2]))
    return T.classify(Lattice(U_mm=U, D_mm=0., H0_mm=0., span_voxels=0), err, angle, tol, False), err, angle


def simulate(body, env, start, seat, mu, U, tol=T.Tolerance(), ds=DS_MM, max_steps=600, trace=True,
             check_ambiguity=False):
    """One pose at one mu; returns a JSON-safe dict (outcome, reason, end pose, trace)."""
    pose = np.asarray(start, float).copy()
    floor = min(p.bounds[1] for p in env)
    states, prev_mode, prev_sig, reason, sys = [], None, None, None, None
    rmax = float(np.max(np.linalg.norm(np.asarray(body.outline.exterior.coords) - np.asarray(body.com), axis=1)))
    state, steps = "moving", 0
    while steps < max_steps:
        steps += 1
        cs = PL.contacts(body, pose, env)
        if not cs:
            d = PL.drop_distance(body, pose, env)
            if d is None or PL.com_world(body, pose)[1] - d < floor - 5:
                state, reason = "fell", "no support below"
                break
            pose[1] -= d
            continue
        com = PL.com_world(body, pose)
        sys = ContactSystem(cs, com, body.M(), U)               # every contact: stability, margins
        if sys.weakly_stable(G, mu) and any(c.alternatives for c in cs):
            sys, cs = _slip_corner(sys, cs, com, body, U, mu)
        ordered, dropped = _spread(cs, MAX_CONTACTS)
        red = sys if not dropped else ContactSystem(ordered, com, body.M(), U, n_free=MAX_CONTACTS)
        if not sys.weakly_stable(G, mu):  # frictionless stability implies weak, so test weak first
            state = "moving"
        elif sys.frictionless_stable(G):
            state = "strong"
        else:
            # Strong vs indeterminate only matters at the seat (both fail elsewhere: a possible jam),
            # and telling them apart enumerates 4^k modes, so only do it there.
            if _end_class(pose, seat, U, tol)[0] == "seated":
                state = "indeterminate" if red.moving_modes(G, mu, first=True) else "strong"
            else:
                state = "weak"
        mode = acc = None
        if state == "moving":
            mode, acc = red.motion(G, mu, prefer=prev_mode)
            if mode is None:
                state, reason = "jammed", "no consistent contact mode"
        sig = (len(cs), state, mode)
        if trace and (sig != prev_sig or state != "moving"):
            entry = {"step": steps, "pose": [float(v) for v in pose], "state": state, "mode": mode,
                     "contacts": [{"point": [float(v) for v in c.point], "normal": [float(v) for v in c.normal]}
                                  for c in cs], "contacts_dropped": dropped,
                     "stick_margin": float(sys.stick_margin(G, mu)), "wedged": bool(sys.wedged(mu)),
                     "opposing": bool(sys.opposing())}
            if check_ambiguity and state == "moving":
                entry["ambiguous"] = len(red.moving_modes(G, mu)) > 1
            states.append(entry)
        prev_sig = sig
        if state != "moving":
            break
        prev_mode = mode
        speed = float(np.linalg.norm(acc[:2]) + abs(acc[2]) * rmax)
        if speed < 1e-12:
            state, reason = "jammed", "zero acceleration"
            break
        h = ds / speed
        for _ in range(14):
            new = _advance(body, pose, acc, h)
            if PL.penetration(body, new, env) <= PL.EPS_MM:
                break
            h /= 2
        else:
            state, reason = "jammed", "blocked: every step penetrates"
            break
        pose = new
    else:
        state, reason = "jammed", "step limit"
    out = {"mu": mu, "steps": steps, "end_pose": [float(v) for v in pose]}
    if state == "strong":
        outcome, err, angle = _end_class(pose, seat, U, tol)
        out.update(err_mm=[float(v) for v in err], angle_deg=float(angle))
        out["seat_margin"] = float(sys.seat_margin(G)) if outcome == "seated" else 0.
        if outcome == "jammed":
            reason = "strongly stable away from the seat"
    elif state == "indeterminate":
        outcome, reason = "indeterminate", "at the seat, weakly stable but a moving mode exists (possible jam)"
    elif state == "weak":
        outcome, err, angle = _end_class(pose, seat, U, tol)
        out.update(err_mm=[float(v) for v in err], angle_deg=float(angle))
        reason = "weakly stable away from the seat (possible jam)"
    else:
        outcome = state
    out.update(outcome=outcome, reason=reason)
    if trace:
        out["trace"] = states
    return out


def run_pose(body, env, start, seat, U, mus=MUS, tol=T.Tolerance(), early_stop=False, **kw):
    """A pose at every mu; seated only if seated at all of them. ``early_stop`` stops at the first
    non-seated mu (highest mu first), for searches."""
    per = {}
    for mu in (sorted(mus, reverse=True) if early_stop else mus):
        r = simulate(body, env, start, seat, mu, U, tol, **kw)
        per[_mu_key(mu)] = r
        if early_stop and r["outcome"] != "seated":
            break
    bad = [r for r in per.values() if r["outcome"] != "seated"]
    seated = not bad and len(per) == len(mus)
    margins = [r["seat_margin"] for r in per.values() if r["outcome"] == "seated"]
    return {"label": LABEL, "outcome": "seated" if seated else max(bad, key=lambda r: r["mu"])["outcome"],
            "per_mu": per, "seat_margin": float(min(margins)) if margins else 0.}


# --- families ---

_SCENES = {}


def _theta(quat):
    v = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]]).apply([1., 0., 0.])
    return math.atan2(v[2], v[0])


def _placed(poly, pos, quat):
    from shapely import affinity
    return affinity.translate(affinity.rotate(poly, _theta(quat), origin=(0, 0), use_radians=True), pos[0], pos[2])


def scene(family, p, course, i0=1):
    """(body, env, target placement) for course 0 (onto the base) or 1 (onto a seated course-0 brick).

    ``i0`` is the first voxel of both bricks; the default 1 matches trials.DropRig (Placement(0, 1),
    Placement(1, 1)), which suits A/B's one-voxel base pattern. A family whose base repeats every
    two voxels (the wedge dummy) seats at i0 = 0."""
    key = (id(family), p, course, i0)
    if key in _SCENES:
        return _SCENES[key][1:]
    if course not in (0, 1):
        raise ValueError("course must be 0 or 1")
    base = family.base(p, 4, 1)
    if not isinstance(base, Section):
        raise NotImplementedError("the predictor needs the base as a 2D Section")
    env = [base.outer]
    target = Placement(0, i0)
    if course == 1:
        below = family.part_for(0, 1)
        pos, quat = family.seat_pose(p, Placement(0, i0))
        env.append(_placed(B.section(family, p, below).outer, pos, quat))
        target = Placement(1, i0)
    part = family.part_for(target.course, target.direction)
    bodies = B.sim_bodies(family, p, part)
    com = (bodies.com_mm[0], bodies.com_mm[2])
    body = PL.Body(B.section(family, p, part).outer, com=com, mass=bodies.mass_g,
                   inertia=float(bodies.inertia_g_mm2[1][1]))
    # Edges shorter than the contact tolerance (e.g. 5 um slivers at chamfered tooth tips) give
    # meaningless normals; simplify both sections at that tolerance.
    body = PL.Body(body.outline.simplify(PL.EPS_MM), body.com, body.mass, body.inertia)
    env = [e.simplify(PL.EPS_MM) for e in env]
    _SCENES[key] = (family, body, env, target)
    return body, env, target


def predict_pose(family, p, sample, course=0, mus=MUS, i0=1, **kw):
    """Predict one release pose. ``sample`` uses ErrorModel.sample keys (dx, pitch, clear; the rest
    default to 0); the release pose is trials.release_pose, as in the sim."""
    body, env, target = scene(family, p, course, i0)
    full = {"dx": 0., "dy": 0., "yaw": 0., "roll": 0., "pitch": 0., "clear": 0., **sample}
    pos, quat = T.release_pose(family, p, target, full)
    spos, squat = family.seat_pose(p, target)
    start = (pos[0], pos[2], _theta(quat))
    seat = (spos[0], spos[2], _theta(squat))
    out = run_pose(body, env, start, seat, family.lattice(p).U_mm, mus=mus, **kw)
    out.update(course=course, sample={k: float(v) for k, v in full.items() if k != "vel"})
    return out


def predicted_box(family, p, box, courses=(0, 1), mus=MUS, seed=0, s_max=2.0, tol=0.05, n_interior=4, cfg=None):
    """Largest predicted scale of a placement box (bisection, as box_search); the smaller over courses."""
    import time

    if box.task != "placement":
        raise ValueError("the predictor covers placement (R11)")
    cfg = cfg or T.TrialConfig(error=T.ErrorModel(ideal=True))
    unit = BS.unit_points(n_interior, seed)
    t0 = time.time()
    per_course = {}
    for course in courses:
        poses, cache = [], {}

        def ok(scale):
            for k, u in enumerate(unit):
                sample = BS.pose_sample(box, scale, u, cfg)
                key = tuple(round(sample[q], 12) for q in ("dx", "pitch", "clear"))
                if key not in cache:
                    cache[key] = predict_pose(family, p, sample, course, mus=mus, early_stop=True)
                r = cache[key]
                poses.append({"scale": scale, "point": k, "unit": u.tolist(), "outcome": r["outcome"],
                              "sample": r["sample"], "per_mu": r["per_mu"], "seat_margin": r["seat_margin"]})
                if r["outcome"] != "seated":
                    return False
            return True

        scale, evals = BS.bisect_scale(ok, s_max, tol)
        per_course[str(course)] = {"scale": scale, "evaluations": evals, "poses": poses, "predicted_count": len(cache),
                                   "failure_outcomes": sorted({q["outcome"] for q in poses if q["outcome"] != "seated"})}
    limiting = min(per_course, key=lambda c: per_course[c]["scale"])
    scale = per_course[limiting]["scale"]
    return {"label": LABEL, "family": family.name, "family_version": family.version, "params": p.to_dict(),
            "box": box.to_dict(), "mus": list(mus), "seed": seed, "tol": tol, "n_interior": n_interior,
            "scale": scale, "passed": scale >= 1.0, "limiting_course": limiting, "courses": per_course,
            "seconds": time.time() - t0}


def predicted_map(family, p, course=0, dxs=None, pitches=(-1.0, 0.0, 1.0), clear=5.0, mus=MUS):
    """Outcome over a small grid of X offsets (mm, about the aim) and pitches (deg) at one height."""
    dxs = list(np.arange(-12.0, 12.01, 2.0)) if dxs is None else list(dxs)
    aim = T.ErrorModel().aim_bias
    rows = []
    for pitch in pitches:
        for dx in dxs:
            r = predict_pose(family, p, {"dx": aim + float(dx), "pitch": float(pitch), "clear": clear}, course,
                             mus=mus, trace=False)
            rows.append({"dx_mm": float(dx), "pitch_deg": float(pitch), "outcome": r["outcome"],
                         "per_mu": {m: q["outcome"] for m, q in r["per_mu"].items()}})
    return {"label": LABEL, "course": course, "clear_mm": clear, "rows": rows}
