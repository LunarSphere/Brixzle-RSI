"""Analytic pickup predictor (R12 capture): Whitney 1982 quasi-static peg-in-hole, per tine.

Whitney, "Quasi-Static Assembly of Compliantly Supported Rigid Parts", J. Dyn. Sys. Meas. Control
104:65-77, 1982 (src/brixzle/rules/papers/quasistatic.pdf). Equation numbers below are the paper's.

Each tine is a peg (radius r) entering its bore (radius R, clearance ratio c = (R - r)/R, D = 2R)
along +Y, supported by the drone's hover spring with its compliance centre at the drone, Lg behind
the tine tip. Each plane is analysed on its own:

- XY: lateral error across the tines (dx) and yaw (psi); Kx = drone kp_x, K_theta = the yaw spring;
- YZ: vertical error (dz), no pitch error in the pickup box; Kx = drone kp_z, K_theta = pitch spring.

Per tine and plane, after mirroring so the lateral error e0 >= 0:

1. entry: the tine edge lands in the mouth, |e| < c R + w (the radial error, both planes together);
   else it hits the face ('jammed', reason 'face');
2. chamfer crossing (eqs. 1-8) when e1 = e0 - c R > 0: the chamfer must be steeper than friction,
   B = sin a - mu cos a > 0; the axial push it needs is Fx A / B (A = cos a + mu sin a), with Fx from
   the support's deflection (eq. 6b) at the chamfer's bottom;
3. one-point contact (eqs. 9-12) through the bore, needing an axial push of mu |Fx| (eqs. 47-48);
4. two-point contact (eqs. 13-24) from l2: wedged if l2 <= mu D (eq. 49); otherwise the push must
   keep the support forces inside the jamming parallelogram (eqs. 46, 55-56);
5. push: the axial push at each depth must not exceed what the hover spring gives, ky (L_end - l),
   with the setpoint stopping at full depth L_end (as sim/pickup.py's ENGAGE). The two planes'
   pushes add (separate x and z springs); the tines share the one drone, so the largest tine's
   need counts, not the sum (they share the spring's deflection).

A pick is 'picked' only if every tine passes in both planes at every mu in MUS. Within the clearance
(e1 <= 0) the peg is taken to start in one-point contact at the wall, which is conservative.

Assumptions, flagged per result where they apply: small angles (Whitney section 3); the clearance
ratio is far above Whitney's range (0.001-0.01; A/B has c ~ 0.23); in YZ the compliance centre (the
drone) sits above the tine axis, not on it; each tine is analysed with the whole support, which is
exact when the tines are in the same state (yaw moves both tips alike) and conservative otherwise;
the brick is held by the fixture.
This covers capture (R12 check 1) only: not self-centring, gentle contact or release.

Outputs are labelled "predicted, quasi-static 2D (Whitney 1982)".
"""

from dataclasses import dataclass
import math
import time

import numpy as np

from brixzle.engine import box_search as BS
from brixzle.engine.sim.drone import HoverGains
from brixzle.engine.sim.fork import ForkParams
from brixzle.families.interface import Placement

LABEL = "predicted, quasi-static 2D (Whitney 1982)"
MUS = (0.2, 0.3, 0.4, 0.5)
SEAT = Placement(0, 1)  # the waiting brick, as sim/pickup.py
N_DEPTH = 50            # depths checked through the bore (key points, not a dense sweep)
SMALL_ANGLE = 0.1       # rad: past this Whitney's small-angle forms are flagged
INSERT_TOL_MM = 2.0     # a tine counts as engaged within this of full depth (PickConfig.insert_tol_mm)


@dataclass(frozen=True)
class Peg:
    """One tine in one bore, in one plane. Lengths mm, forces N, Kx N/mm, Kth N mm/rad, alpha rad
    (Whitney's chamfer angle, from the bore's end face)."""

    r: float
    R: float
    w: float
    alpha: float
    Lg: float
    Kx: float
    Kth: float
    mu: float

    def __post_init__(self):
        vals = (self.r, self.R, self.w, self.alpha, self.Lg, self.Kx, self.Kth, self.mu)
        if not all(math.isfinite(v) for v in vals):
            raise ValueError("Peg values must be finite")
        if not (0 < self.r < self.R) or self.w < 0 or self.Lg <= 0 or self.Kx <= 0 or self.Kth < 0 or self.mu < 0:
            raise ValueError("need 0 < r < R, w >= 0, Lg > 0, Kx > 0, Kth >= 0, mu >= 0")
        if not 0 < self.alpha < math.pi / 2:
            raise ValueError("alpha must be between 0 and 90 degrees")

    @property
    def c(self):
        return (self.R - self.r) / self.R

    @property
    def D(self):
        return 2 * self.R


def _AB(g):
    return math.cos(g.alpha) + g.mu * math.sin(g.alpha), math.sin(g.alpha) - g.mu * math.cos(g.alpha)


def chamfer_crossing(g: Peg, e0, th0, z):
    """(theta, U) after axial travel z down the chamfer (eqs. 7-8), U0 = e0 + Lg theta0 (eq. 1)."""
    A, B = _AB(g)
    U0 = e0 + g.Lg * th0
    den = (g.Kx * g.Lg**2 + g.Kth) * B - g.Kx * g.Lg * g.r * A
    s = z / math.tan(g.alpha)
    return th0 + g.Kx * s * (g.Lg * B - g.r * A) / den, U0 - g.Kth * s * B / den


def one_point(g: Peg, e0, th0, l):
    """(theta, U) in one-point contact at depth l past the chamfer (eq. 10; U from eq. 9)."""
    cR = g.c * g.R
    e1 = e0 - cR
    C = g.Kx * (g.Lg - l - g.mu * g.r)
    th = (C * (e1 + g.Lg * th0) + g.Kth * th0) / (C * (g.Lg - l) + g.Kth)
    return th, cR + (g.Lg - l) * th


def two_point_onset(g: Peg, e0, th0):
    """(l2, l2') where two-point contact begins and ends (eqs. 21-24), or None if it never occurs."""
    cD = g.c * g.D
    a = g.Kx * (e0 + g.c * g.R + g.Lg * th0)
    beta = (g.Lg - g.mu * g.r) * a + g.Kx * g.Lg * cD + g.Kth * th0
    gamma = cD * (g.Kx * g.Lg**2 + g.Kth - g.Kx * g.Lg * g.mu * g.r)
    disc = beta**2 - 4 * a * gamma
    if a <= 0 or disc < 0:
        return None
    root = math.sqrt(disc)
    return (beta - root) / (2 * a), (beta + root) / (2 * a)


def support_forces(g: Peg, U0, U, th, th0):
    """(Fx, M) the support applies, at the peg tip (eq. 6b)."""
    fx = g.Kx * (U0 - U)
    return fx, g.Kx * g.Lg * (U0 - U) - g.Kth * (th - th0)


def two_point_fz(fx, m, r, mu, l):
    """Smallest axial push that keeps (Fx, M) inside the jamming parallelogram at depth l (eqs. 46,
    55-56): |Fx/Fz| <= 1/mu and |M/(r Fz) + mu (1 + lambda) Fx/Fz| <= lambda, lambda = l/(2 r mu)."""
    if mu == 0:
        return 0.0
    lam = l / (2 * r * mu)
    return max(mu * abs(fx), abs(m / r + mu * (1 + lam) * fx) / lam if lam > 0 else math.inf)


def insert(g: Peg, e0, th0, depth, f_avail=None, n=N_DEPTH):
    """One tine, one plane: {ok, reason, flags, push (N, per depth), depths (mm)}.

    ``f_avail(l)`` is the axial push available at depth l (N); default unlimited. The chamfer
    crossing is depth 0."""
    if e0 < 0:  # mirror so the lateral error is >= 0
        e0, th0 = -e0, -th0
    f_avail = f_avail or (lambda l: math.inf)
    cR = g.c * g.R
    flags = set()
    out = {"ok": False, "reason": None, "flags": flags, "depths": [], "push": []}
    if e0 >= cR + g.w:
        out["reason"] = "face: the tine lands outside the bore mouth"
        return out
    e1 = e0 - cR
    e0 = max(e0, cR)  # within the clearance: taken to start at the wall (conservative)
    U0 = e0 + g.Lg * th0
    push = []
    if e1 > 0:  # chamfer crossing (only reached when the chamfer is wider than e1)
        A, B = _AB(g)
        if B <= 0:
            out["reason"] = f"chamfer: flatter than friction (tan a = {math.tan(g.alpha):.3g} <= mu = {g.mu:g})"
            return out
        th, U = chamfer_crossing(g, e0, th0, e1 * math.tan(g.alpha))
        fx, _ = support_forces(g, U0, U, th, th0)
        push.append((0.0, abs(fx) * A / B, "chamfer"))
        if abs(th) > SMALL_ANGLE:
            flags.add("small-angle")
    onset = two_point_onset(g, e0, th0)
    l2, l2b = onset if onset else (math.inf, math.inf)
    if onset is None and th0 < 0:
        flags.add("two-point case not analysed (theta0 opposes e0)")
    if 0 < l2 < depth and l2 <= g.mu * g.D:
        out["reason"] = f"wedged: two-point contact at l2 = {l2:.3g} mm <= mu D = {g.mu * g.D:.3g} mm (eq. 49)"
        return out
    for l in np.linspace(0.0, depth, n):
        l = float(l)
        if l2 <= l <= l2b:  # two-point contact
            th = g.c * g.D / l
            U = U0 - (e0 + cR + g.Lg * (th0 - th))  # eqs. 16-17
            fx, m = support_forces(g, U0, U, th, th0)
            fz = two_point_fz(fx, m, g.r, g.mu, l)
            stage = "two-point"
            if th > math.sqrt(2 * g.c):
                flags.add("small-angle")
        else:
            th, U = one_point(g, e0, th0, l)
            fx, _ = support_forces(g, U0, U, th, th0)
            fz, stage = g.mu * abs(fx), "one-point"
        if abs(th) > SMALL_ANGLE:
            flags.add("small-angle")
        push.append((l, fz, stage))
    out["depths"] = [p[0] for p in push]
    out["push"] = [p[1] for p in push]
    out["stages"] = [p[2] for p in push]
    for l, fz, stage in push:
        if fz > f_avail(l):
            out["reason"] = f"push: {stage} needs {fz:.3g} N at {l:.3g} mm, the drone gives {f_avail(l):.3g} N"
            return out
    out["ok"] = True
    return out


# --- a family ---


def _stiffness(gains: HoverGains):
    """(Kx, Kth) per plane in N/mm and N mm/rad: XY uses kp_x and the yaw spring, YZ kp_z and pitch."""
    w2 = (2 * math.pi * gains.att_hz) ** 2
    return {"xy": (gains.kp[0] / 1000, gains.J[2] * w2 * 1000), "yz": (gains.kp[2] / 1000, gains.J[0] * w2 * 1000)}


def geometry(family, p, fork=None, bores=None):
    """The tines' bores and the fork geometry the predictor needs (no CAD).

    The drone (the compliance centre) is taken over the brick's centre in Y, so Lg = D/2 + the
    tines' protrusion beyond the far face (sim/fork.py puts it over the centre of mass; for a
    brick symmetric in Y, as A/B, that is the centre)."""
    fork = fork or ForkParams()
    part = family.part_for(SEAT.course, SEAT.direction)
    bs = tuple(family.pickup(p, part).bores)
    if bores is not None:
        bs = tuple(bores(bs))
    depth = float(family.lattice(p).D_mm)
    return {"bores": bs, "depth_mm": depth, "Lg_mm": depth / 2 + fork.protrude,
            "x_drone_mm": float(np.mean([b.x_mm for b in bs])), "protrude_mm": fork.protrude}


def predict_pick(family, p, sample, mus=MUS, gains=None, fork=None, bores=None, geo=None):
    """Predict one pick at error {dx, dz (mm), psi (deg)}: 'picked' or 'jammed', JSON-safe."""
    gains = gains or HoverGains()
    geo = geo or geometry(family, p, fork, bores)
    K = _stiffness(gains)
    psi = math.radians(float(sample["psi"]))
    dx, dz = float(sample["dx"]), float(sample["dz"])
    L_end = geo["depth_mm"] + geo["protrude_mm"]
    ky = gains.kp[1] / 1000
    n_t = len(geo["bores"])
    per_mu, flags = {}, set()
    for mu in sorted(mus, reverse=True):
        # Axial push per depth: the planes add (separate x and z springs); the tines share one drone,
        # so its spring deflection is shared and the largest tine need counts, not the sum.
        need = np.zeros(N_DEPTH)
        reason = None
        tines = []
        for b in geo["bores"]:
            r, R = b.d_mm / 2, b.radius_mm
            # Tip error across the tines: the drone offset, plus yaw about the drone's vertical axis.
            ex = dx + (b.x_mm - geo["x_drone_mm"]) * (math.cos(psi) - 1) - geo["Lg_mm"] * math.sin(psi)
            ez = dz
            if math.hypot(ex, ez) >= (R - r) + b.chamfer_w_mm:
                reason = f"face: a tine at x {b.x_mm:.3g} mm lands outside its bore mouth ({math.hypot(ex, ez):.3g} mm off)"
                break
            alpha = math.radians(b.chamfer_deg)
            res, tine_need = {}, np.zeros(N_DEPTH)
            for plane, e0, th0 in (("xy", ex, math.sin(psi)), ("yz", ez, 0.0)):
                g = Peg(r=r, R=R, w=b.chamfer_w_mm, alpha=alpha, Lg=geo["Lg_mm"], Kx=K[plane][0],
                        Kth=K[plane][1], mu=mu)
                q = insert(g, e0, th0, geo["depth_mm"])
                flags |= q["flags"]
                res[plane] = q
                if not q["ok"]:
                    reason = f"{q['reason']} ({plane}, tine at x {b.x_mm:.3g} mm)"
                    break
                tine_need += np.asarray(q["push"][-N_DEPTH:])
                if len(q["push"]) > N_DEPTH:  # the chamfer crossing comes first, at depth 0
                    tine_need[0] += q["push"][0]
            need = np.maximum(need, tine_need)
            tines.append({"x_mm": b.x_mm, "e_mm": [ex, ez]})
            if reason:
                break
        if reason is None:
            depths = np.linspace(0.0, geo["depth_mm"], N_DEPTH)
            avail = ky * (L_end - depths)
            check = depths <= L_end - INSERT_TOL_MM
            bad = np.flatnonzero(check & (need > avail))
            if len(bad):
                k = int(bad[0])
                reason = (f"push: the fork ({n_t} tines) needs {need[k]:.3g} N at {depths[k]:.3g} mm, "
                          f"the hover spring gives {avail[k]:.3g} N")
            peak = float(need.max())
        else:
            peak = None
        per_mu[f"{mu:g}"] = {"outcome": "picked" if reason is None else "jammed", "reason": reason,
                             "peak_push_N": peak}
        if reason is not None:
            break  # the highest mu first: stop at the first failure
    bad = [m for m, v in per_mu.items() if v["outcome"] != "picked"]
    picked = not bad and len(per_mu) == len(mus)
    c_ratio = [(b.radius_mm - b.d_mm / 2) / b.radius_mm for b in geo["bores"]]
    notes = ["compliance centre above the tine axis in YZ"]
    if max(c_ratio) > 0.01:
        notes.append(f"clearance ratio {max(c_ratio):.3g} is above Whitney's range (0.001-0.01)")
    return {"label": LABEL, "outcome": "picked" if picked else "jammed",
            "reason": None if picked else per_mu[bad[0]]["reason"], "per_mu": per_mu,
            "flags": sorted(flags), "notes": notes,
            "sample": {"dx": dx, "dz": dz, "psi": float(sample["psi"])}}


def predicted_pickup_box(family, p, box, mus=MUS, seed=0, s_max=2.0, tol=0.05, n_interior=4, gains=None,
                         fork=None, bores=None):
    """Largest predicted scale of a pickup box (the same unit points and bisection as box_search)."""
    if box.task != "pickup":
        raise ValueError("expected a pickup box")
    t0 = time.time()
    geo = geometry(family, p, fork, bores)
    unit = BS.unit_points(n_interior, seed)
    poses = []

    def ok(scale):
        for k, u in enumerate(unit):
            sample = BS.pose_sample(box, scale, u, None)
            r = predict_pick(family, p, sample, mus=mus, gains=gains, geo=geo)
            poses.append({"scale": scale, "point": k, "unit": u.tolist(), "outcome": r["outcome"],
                          "reason": r["reason"], "sample": r["sample"], "flags": r["flags"]})
            if r["outcome"] != "picked":
                return False
        return True

    scale, evals = BS.bisect_scale(ok, s_max, tol)
    notes = predict_pick(family, p, {"dx": 0.0, "dz": 0.0, "psi": 0.0}, mus=mus, gains=gains, geo=geo)["notes"]
    return {"label": LABEL, "family": family.name, "family_version": family.version, "params": p.to_dict(),
            "box": box.to_dict(), "mus": list(mus), "seed": seed, "tol": tol, "n_interior": n_interior,
            "scale": scale, "passed": scale >= 1.0, "evaluations": evals, "poses": poses,
            "failure_reasons": sorted({q["reason"].split(":")[0] for q in poses if q["reason"]}),
            "notes": notes, "geometry": {k: v for k, v in geo.items() if k != "bores"},
            "bores": [{"x_mm": b.x_mm, "z_mm": b.z_mm, "d_mm": b.d_mm, "clearance_mm": b.clearance_mm,
                       "chamfer_w_mm": b.chamfer_w_mm, "chamfer_deg": b.chamfer_deg} for b in geo["bores"]],
            "seconds": time.time() - t0}


def main(argv=None):
    """Record a predicted pickup box under runs/ (never overwrites a run)."""
    import argparse
    from datetime import datetime, timezone
    import json
    from pathlib import Path

    from brixzle.engine import boxes as BX
    from brixzle.families import registry

    ap = argparse.ArgumentParser(description="Predicted pickup box (R12 capture), Whitney 1982")
    ap.add_argument("--family", default="ab")
    ap.add_argument("--statistic", choices=BX.STATISTICS, default="mean")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--tol", type=float, default=0.05)
    ap.add_argument("--runs", default="runs")
    args = ap.parse_args(argv)
    family = registry.get(args.family)
    box = BX.default_boxes()["pickup"][args.statistic]
    result = predicted_pickup_box(family, family.defaults, box, seed=args.seed, tol=args.tol)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    out = Path(args.runs) / f"predict-{stamp}-{family.name}-pickup-{args.statistic}"
    out.mkdir(parents=True, exist_ok=False)
    (out / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"{family.name} pickup {args.statistic} ({LABEL}): scale {result['scale']:.3f} "
          f"({', '.join(result['failure_reasons']) or 'no failures'}), {result['seconds'] * 1000:.0f} ms -> {out}")


if __name__ == "__main__":
    main()
