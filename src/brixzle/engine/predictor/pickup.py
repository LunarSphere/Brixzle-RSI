"""Opening pickup predictor (R12): the fork's tines go through large through-Y openings.

Maintainer decision on issue #12: bricks are picked up through large openings (A/B's big windows),
not tight bores. With that much clearance there is no peg-in-hole jamming, so the check is
geometric and quasi-static, with the brick translating only:

1. capture region, per opening: the opening's overlap across the brick's depth (it sweeps with
   the extrusion's bands while the tine is straight), shifted by yaw along the depth (the tine
   crosses x at tan(psi) per mm of y, about the drone at y = 0, as sim/pickup.py), and shrunk by
   the tine radius. A tine's axis must lie inside it;
2. the fork is fitted to the brick: each tine's seat is its region's highest point at zero yaw
   (A/B: the window's acute top corner). F, the region of fork offsets that keep every tine inside
   its opening, is the intersection of (region_i - seat_i); the seat is F's origin;
3. aim: the drone aims the fork at the centre of the largest circle inside F (zero yaw), and the
   approach error (dx, dz) offsets it;
4. entry: the offset point must lie inside F at the pose's yaw, else a tine hits the face or a web
   ('jammed', reason 'face');
5. lift: the brick hangs on the tines, so the point rises to F's ceiling and slides up it while the
   edge's slope beats friction (tan > mu), until an apex, friction or a flat edge holds it. Where
   it stops is the carry error, which must be within carry_xz_mm (R12 self-centring) at every mu,
   else 'off_centre'.

Rotation of the hanging brick is not modelled (flagged in every result); the sim checks it. This
covers R12 capture and self-centring, not gentle contact or release. Outputs are labelled
"predicted, quasi-static 2D".
"""

import math
import time

import numpy as np
import shapely
from shapely import affinity
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient

from brixzle.engine import box_search as BS
from brixzle.engine.sim.fork import ForkParams
from brixzle.families.interface import Placement, Prismatic, grip_openings

LABEL = "predicted, quasi-static 2D"
MUS = (0.2, 0.3, 0.4, 0.5)
SEAT = Placement(0, 1)  # the waiting brick, as sim/pickup.py
CARRY_XZ_MM = 2.0       # R12 self-centring (PickConfig.carry_xz_mm)
FLAGS = ("translation only: the hanging brick's rotation is not modelled",)
EPS = 1e-9


def _samples(bands):
    """(y, z shift) at each band's ends and middle: the shift is linear in y within a band, so for a
    convex opening the ends are exact; the middle helps a non-convex one."""
    out = []
    for y0, y1, z0, z1 in bands:
        for u in (0.0, 0.5, 1.0):
            out.append((y0 + u * (y1 - y0), z0 + u * (z1 - z0)))
    return out


def capture_region(opening, bands, r, psi=0.0):
    """Where a straight tine's axis (its x at y = 0) may lie to clear ``opening`` along the whole
    depth: the intersection over the bands of the opening shifted by its band's z and by y tan(psi)
    in x, shrunk by the tine radius r. Empty if nothing clears."""
    t = math.tan(psi)
    region = None
    for y, dz in _samples(bands):
        q = affinity.translate(opening, xoff=y * t, yoff=dz)
        region = q if region is None else region.intersection(q)
    region = region.buffer(-r, join_style="mitre")
    return _largest(region)


def _largest(geom):
    """The largest polygon of a (multi)polygon, or an empty Polygon."""
    if geom.is_empty:
        return Polygon()
    if geom.geom_type == "Polygon":
        return geom
    polys = [g for g in getattr(geom, "geoms", []) if g.geom_type == "Polygon"]
    return max(polys, key=lambda g: g.area) if polys else Polygon()


def _ceiling(a, b):
    """Is edge a -> b (counter-clockwise ring) part of the ceiling, facing down into the region?"""
    return (b[0] - a[0]) < -EPS  # outward normal (dz, -dx) points up


def slide_up(region, t0, mu, max_edges=200):
    """Lift: the point t0 rises to the region's ceiling, then slides up it while tan(slope) > mu.
    Returns (point, reason): reason is 'apex', 'friction', 'flat' or 'no ceiling'."""
    region = orient(region, sign=1.0)
    V = np.asarray(region.exterior.coords)[:-1]
    n = len(V)
    t0 = np.asarray(t0, float)
    top = region.bounds[3] + 1.0
    hits = LineString([tuple(t0), (t0[0], top)]).intersection(region.exterior)
    pts = [np.asarray(c) for g in getattr(hits, "geoms", [hits]) if not g.is_empty for c in g.coords]
    pts = [q for q in pts if q[1] >= t0[1] - 1e-9]
    if not pts:
        return t0, "no ceiling"
    p = min(pts, key=lambda q: q[1])
    # The ceiling edge under p (an edge whose segment contains p and which faces down).
    edge = None
    for i in range(n):
        a, b = V[i], V[(i + 1) % n]
        if _ceiling(a, b) and LineString([a, b]).distance(Point(p)) < 1e-7:
            edge = i
            break
    if edge is None:
        return p, "flat"  # touched a vertex or wall only
    for _ in range(max_edges):
        a, b = V[edge], V[(edge + 1) % n]
        d = b - a
        L = float(np.hypot(*d))
        if abs(d[1]) <= 1e-9 * max(L, 1.0):
            return p, "flat"
        tan_slope = abs(d[1]) / max(abs(d[0]), EPS)
        if tan_slope <= mu:
            return p, "friction"
        # Slide toward the edge's upper end.
        up_is_b = d[1] > 0
        p = b.copy() if up_is_b else a.copy()
        nxt = (edge + 1) % n if up_is_b else (edge - 1) % n  # the edge that continues from that vertex
        na, nb = V[nxt], V[(nxt + 1) % n]
        rises = (nb[1] > p[1] + 1e-9) if up_is_b else (na[1] > p[1] + 1e-9)
        if not (_ceiling(na, nb) and rises):
            return p, "apex"
        edge = nxt
    return p, "apex"


def _seat(region):
    """The region's highest point (its apex for a peaked or slanted ceiling), mean of ties."""
    V = np.asarray(region.exterior.coords)[:-1]
    zmax = V[:, 1].max()
    return V[np.abs(V[:, 1] - zmax) < 1e-6].mean(axis=0)


def geometry(family, p, fork=None):
    """The openings, the fitted fork's seats and the aim, for the waiting brick (no CAD)."""
    fork = fork or ForkParams()
    g = family.geometry
    if not isinstance(g, Prismatic):
        raise NotImplementedError("the opening pickup predictor needs a Prismatic family")
    part = family.part_for(SEAT.course, SEAT.direction)
    openings = grip_openings(family.pickup(p, part))
    if not openings:
        raise ValueError(f"family {family.name!r} gives no pickup openings")
    bands = tuple(g.extrusion(p).resolved_bands())
    r = fork.tine_d / 2
    regions = [capture_region(o, bands, r) for o in openings]
    if any(q.is_empty for q in regions):
        raise ValueError("a pickup opening is too small for a straight tine through the whole depth")
    seats = [_seat(q) for q in regions]
    F0 = _fork_region(regions, seats)
    if F0.is_empty:
        raise ValueError("no fork offset keeps every tine inside its opening")
    aim = np.asarray(shapely.maximum_inscribed_circle(F0).coords[0])
    return {"openings": openings, "bands": bands, "r": r, "seats": [s.tolist() for s in seats],
            "aim": aim.tolist(), "aim_margin_mm": float(shapely.maximum_inscribed_circle(F0).length),
            "part": part, "_F": {0.0: F0}}


def _fork_region(regions, seats):
    F = None
    for q, s in zip(regions, seats):
        q = affinity.translate(q, -s[0], -s[1])
        F = q if F is None else F.intersection(q)
    return _largest(F)


def _F(geo, psi):
    key = round(psi, 12)
    if key not in geo["_F"]:
        regions = [capture_region(o, geo["bands"], geo["r"], psi) for o in geo["openings"]]
        geo["_F"][key] = Polygon() if any(q.is_empty for q in regions) else _fork_region(regions, geo["seats"])
    return geo["_F"][key]


def predict_pick(family, p, sample, mus=MUS, geo=None, carry_mm=CARRY_XZ_MM):
    """One pick at error {dx, dz (mm), psi (deg)}: 'picked', 'jammed' or 'off_centre', JSON-safe."""
    geo = geo or geometry(family, p)
    dx, dz, psi_deg = float(sample["dx"]), float(sample["dz"]), float(sample["psi"])
    F = _F(geo, math.radians(psi_deg))
    t0 = np.asarray(geo["aim"]) + [dx, dz]
    out = {"label": LABEL, "sample": {"dx": dx, "dz": dz, "psi": psi_deg}, "flags": list(FLAGS),
           "per_mu": {}, "carry_mm": None}
    if F.is_empty or not F.buffer(1e-9).contains(Point(t0)):
        out.update(outcome="jammed", reason="face: a tine lands outside its opening (on the face or a web)")
        return out
    worst = None
    for mu in sorted(mus, reverse=True):
        t, why = slide_up(F, t0, mu)
        carry = [float(t[0]), float(t[1])]
        ok = math.hypot(*carry) <= carry_mm
        out["per_mu"][f"{mu:g}"] = {"carry_mm": carry, "stop": why, "outcome": "picked" if ok else "off_centre"}
        if worst is None or math.hypot(*carry) > math.hypot(*worst):
            worst = carry
        if not ok:
            out.update(outcome="off_centre", carry_mm=carry,
                       reason=f"off_centre: hangs {math.hypot(*carry):.3g} mm from the seat at mu {mu:g} "
                              f"(stopped by {why})")
            return out
    out.update(outcome="picked", reason=None, carry_mm=worst)
    return out


def predicted_pickup_box(family, p, box, mus=MUS, seed=0, s_max=2.0, tol=0.05, n_interior=4, fork=None):
    """Largest predicted scale of a pickup box (the same unit points and bisection as box_search)."""
    if box.task != "pickup":
        raise ValueError("expected a pickup box")
    t0 = time.time()
    geo = geometry(family, p, fork)
    unit = BS.unit_points(n_interior, seed)
    poses = []

    def ok(scale):
        for k, u in enumerate(unit):
            r = predict_pick(family, p, BS.pose_sample(box, scale, u, None), mus=mus, geo=geo)
            poses.append({"scale": scale, "point": k, "unit": u.tolist(), "outcome": r["outcome"],
                          "reason": r["reason"], "sample": r["sample"], "carry_mm": r["carry_mm"]})
            if r["outcome"] != "picked":
                return False
        return True

    scale, evals = BS.bisect_scale(ok, s_max, tol)
    return {"label": LABEL, "family": family.name, "family_version": family.version, "params": p.to_dict(),
            "box": box.to_dict(), "mus": list(mus), "seed": seed, "tol": tol, "n_interior": n_interior,
            "scale": scale, "passed": scale >= 1.0, "evaluations": evals, "poses": poses,
            "failure_reasons": sorted({q["reason"].split(":")[0] for q in poses if q["reason"]}),
            "flags": list(FLAGS), "seats_mm": geo["seats"], "aim_mm": geo["aim"],
            "aim_margin_mm": geo["aim_margin_mm"], "seconds": time.time() - t0}


def main(argv=None):
    """Record a predicted pickup box under runs/ (never overwrites a run)."""
    import argparse
    from datetime import datetime, timezone
    import json
    from pathlib import Path

    from brixzle.engine import boxes as BX
    from brixzle.families import registry

    ap = argparse.ArgumentParser(description="Predicted pickup box through the fork openings (R12)")
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
