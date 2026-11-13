"""stackv: a stack-only V brick, kept as a test fixture (issue #82). Not a registered family.

This is the dummy wedge as it was before #82: a chevron of constant vertical thickness H0 spanning
two voxels, a V-groove on top at ``groove_deg`` and the matching V-ridge underneath. It nests only
straight on top of one brick (stack bond). A brick shifted one voxel, as in a running bond, lands
its ridge on the crest between two bricks, so it cannot lay a wall, a bridge or an overhang. Tests
use it as the plain V that the loop must reject (the `chevron` of run ab-concept2-20261113-mean).
"""

from dataclasses import dataclass
import math

import numpy as np
from shapely.geometry import Polygon

from brixzle.families.interface import (
    Extrusion, Family, FamilyParams, GuideFace, Lattice, Placement, Prismatic, Section, bore,
)

BASE_THICKNESS_MM = 6.0
RELEASE_CLEARANCE_MM = 1.0


@dataclass(frozen=True)
class StackVParams(FamilyParams):
    U: float = 25.0             # voxel pitch in X (the brick spans two voxels)
    D: float = 25.0             # depth in Y
    H0: float = 15.0            # course pitch (vertical thickness)
    groove_deg: float = 35.0    # groove and ridge flank angle from horizontal
    bore_d: float = 2.0         # bore diameter before clearance
    bore_clearance: float = 0.3  # extra radius on each bore
    bore_x_frac: float = 0.6    # bore |x| as a fraction of U
    bore_z_frac: float = 0.55   # bore height above the ridge flank, as a fraction of H0

    def __post_init__(self):
        super().__post_init__()
        if min(self.U, self.D, self.H0, self.bore_d) <= 0 or self.bore_clearance < 0:
            raise ValueError("U, D, H0 and bore_d must be > 0 and bore_clearance >= 0")
        if not 0 < self.groove_deg < 90:
            raise ValueError("groove_deg must be between 0 and 90")
        if not (0 < self.bore_x_frac < 1 and 0 < self.bore_z_frac < 1):
            raise ValueError("bore fractions must be between 0 and 1")


BOUNDS = {
    "U": (18.0, 32.0),
    "D": (18.0, 32.0),
    "H0": (12.0, 20.0),
    "groove_deg": (30.0, 45.0),
    "bore_x_frac": (0.4, 0.7),
    "bore_z_frac": (0.4, 0.6),
}


def _rise(p):
    return p.U * math.tan(math.radians(p.groove_deg))


def section(p: StackVParams, part):
    h = _rise(p)
    left_top, top_valley, right_top = (-p.U, h + p.H0), (0.0, p.H0), (p.U, h + p.H0)
    outer = Polygon([(-p.U, h), (0.0, 0.0), (p.U, h), right_top, top_valley, left_top])
    return Section(outer=outer, guide_faces=(GuideFace(left_top, top_valley), GuideFace(top_valley, right_top)))


def extrusion(p: StackVParams):
    return Extrusion(depth_mm=p.D)


def pickup(p: StackVParams, part):
    x = p.bore_x_frac * p.U
    z = x * math.tan(math.radians(p.groove_deg)) + p.bore_z_frac * p.H0
    return tuple(bore(s * x, z, p.bore_d + 2 * p.bore_clearance) for s in (-1, 1))


def lattice(p: StackVParams):
    return Lattice(U_mm=p.U, D_mm=p.D, H0_mm=p.H0, span_voxels=2)


def part_for(course, direction):
    return "A"


def seat_pose(p: StackVParams, b: Placement):
    pos = np.array([(b.i0 + 1) * p.U, 0.0, b.course * p.H0])
    quat = np.array([1.0, 0, 0, 0]) if b.direction > 0 else np.array([0.0, 0, 0, 1])
    return pos, quat


def insertion_axis(p: StackVParams, b: Placement):
    return np.array([0.0, -1.0])  # straight down into the groove


def release_lift_mm(p: StackVParams):
    return _rise(p) + RELEASE_CLEARANCE_MM


def base(p: StackVParams, voxels, direction):
    """A slab whose top is a row of V-grooves, in the base frame (x = 0 at voxel 0).

    A seated brick at Placement(0, i0) is centred at x = (i0 + 1) U (``seat_pose``), and the drop
    trial uses i0 = 1, so the grooves' bottoms sit at even multiples of U (0, 2U, 4U ...) and
    their crests at odd ones. Issue #70: they used to be one voxel over, so no wedge ever seated."""
    if voxels % 2:
        raise ValueError("a wedge base must cover an even number of voxels")
    h = _rise(p)
    top = [(0.0, 0.0)]
    for k in range(voxels // 2):
        x0 = 2 * k * p.U
        top += [(x0 + p.U, h), (x0 + 2 * p.U, 0.0)]
    outer = Polygon(top + [(voxels * p.U, -BASE_THICKNESS_MM), (0.0, -BASE_THICKNESS_MM)])
    return Section(outer=outer)


FAMILY = Family(
    name="stackv",
    version="0.1.0",
    description="Stack-only V (the pre-#82 wedge): a test fixture that cannot lay a running bond.",
    params_type=StackVParams,
    bounds=BOUNDS,
    parts=("A",),
    geometry=Prismatic(section=section, extrusion=extrusion),
    lattice=lattice,
    part_for=part_for,
    seat_pose=seat_pose,
    insertion_axis=insertion_axis,
    release_lift_mm=release_lift_mm,
    base=base,
    pickup=pickup,
)
