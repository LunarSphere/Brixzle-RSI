"""ab: the A/B baseline brick from aerial-assembly, through the family interface (issue #6).

It is the first family and the score the loop must beat. Prismatic: the XZ profile of
``profile.brick_parts`` with lightening holes, swept through the gutter bands of
``profile.y_bands``, with the two straight tine bores of ``profile.fork_channels``. This is the
geometry aerial-assembly's cad.build_brick produced; the shared builders now do the CAD.

Barbs (``barb_h > 0``) need soft contact in the sim, which the interface does not carry yet, so
they are rejected. Their bounds are not searched (CONCEPT_BOUNDS was not ported).
"""

import numpy as np
from shapely.errors import GEOSException

from brixzle.families import builders as B
from brixzle.families.interface import (
    Bore, Extrusion, Family, Lattice, Placement, PickupFeatures, Prismatic, Section,
)

from . import lattice as L
from . import profile as pr
from . import rules as R
from .params import BOUNDS, BrickParams

GUIDE_ANGLE_TOL_DEG = 0.05


def _check_supported(p: BrickParams):
    if p.barb_h > 0:
        raise ValueError("barbs (barb_h > 0) need soft contact, which the family interface does not support yet")


def _infeasible(what, e):
    # aerial-assembly's optimizer counted a geometry that fails to build as infeasible
    # (optimize.evaluate: geometry_builds = -1); the family contract's way to say so is ValueError.
    return ValueError(f"the A/B {what} cannot be built for these parameters: {e}")


def section(p: BrickParams, part):
    _check_supported(p)
    try:
        parts = pr.brick_parts(p, part)
        holes = pr.lightening_holes(p, parts, pr.fork_channels(p))
    except GEOSException as e:
        raise _infeasible("profile", e) from e
    outer = parts["outer"]
    if not outer.is_valid:
        raise _infeasible("profile", "the outline is not a valid polygon")
    # Guide faces: the outline edges on the long hopper ramp (theta) and the steep face (phi).
    guides = tuple(g for g in B.inclined_edges(outer)
                   if min(abs(g.angle_deg - p.theta), abs(g.angle_deg - p.phi)) < GUIDE_ANGLE_TOL_DEG)
    return Section(outer=outer, holes=tuple(holes), guide_faces=guides)


def extrusion(p: BrickParams):
    return Extrusion(depth_mm=p.D, bands=tuple(tuple(b) for b in pr.y_bands(p)))


def pickup(p: BrickParams, part):
    if not pr.fork_channels(p):
        raise ValueError("no position for the two fork bores (aerial-assembly fork_fits < 0)")
    return PickupFeatures(bores=tuple(Bore(x_mm=float(x), z_mm=float(z), d_mm=p.fork_d,
                                           clearance_mm=p.fork_clearance)
                                      for x, z in pr.fork_channels(p)))


def lattice(p: BrickParams):
    return Lattice(U_mm=p.U, D_mm=p.D, H0_mm=p.H0, span_voxels=2)


def part_for(course, direction):
    even = course % 2 == 0
    return ("A" if even else "B") if direction > 0 else ("B" if even else "A")


def seat_pose(p: BrickParams, b: Placement):
    return L.pose(p, b)


def insertion_axis(p: BrickParams, b: Placement):
    return np.asarray(L.insertion_axis(p, b), dtype=float)


def release_lift_mm(p: BrickParams):
    """Release height above the seat before the sampled clearance: sawtooth plus tooth stroke."""
    return p.amplitude + p.tooth_L + p.slot_extra


def base(p: BrickParams, voxels, direction):
    """Anchored base plate receiving course 0, as in aerial-assembly cad.build_base."""
    try:
        outer, _ = pr.base_outer(p, voxels, p.tooth_L + p.slot_extra + 4, direction, None)
    except GEOSException as e:
        raise _infeasible("base", e) from e
    return Section(outer=outer)


def extra_rules(p: BrickParams, part, bodies):
    return R.rules(p, part, bodies)


FAMILY = Family(
    name="ab",
    version="1.0.0",
    description="A/B sawtooth brick from aerial-assembly @ a649663: the baseline family.",
    params_type=BrickParams,
    bounds=BOUNDS,
    parts=("A", "B"),
    geometry=Prismatic(section=section, extrusion=extrusion),
    lattice=lattice,
    part_for=part_for,
    seat_pose=seat_pose,
    insertion_axis=insertion_axis,
    release_lift_mm=release_lift_mm,
    base=base,
    pickup=pickup,
    extra_rules=extra_rules,
)
