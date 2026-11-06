"""The family contract: every registered family passes, and each check catches its failure."""

import math
from dataclasses import replace

import numpy as np
import pytest
from shapely.geometry import Polygon

from brixzle.families import builders as B
from brixzle.families import contract as C
from brixzle.families import rules as R
from brixzle.families import registry
from brixzle.families.interface import Bore, PickupFeatures, Prismatic, Section, Solid3D
from brixzle.families.wedge import family as wedge_module
from brixzle.families.wedge.family import FAMILY as WEDGE


@pytest.mark.parametrize("name", registry.names())
def test_registered_family_passes_contract(name):
    assert C.check_family(registry.get(name)) == []


def _checks(family, **kwargs):
    return {v.check for v in C.check_family(family, samples=0, **kwargs)}


def test_rejects_default_outside_bounds():
    bad = replace(WEDGE, bounds={**WEDGE.bounds, "U": (30.0, 32.0)})
    assert "params" in _checks(bad)


def test_rejects_inverted_bounds():
    bad = replace(WEDGE, bounds={**WEDGE.bounds, "U": (32.0, 18.0)})
    assert "params" in _checks(bad)


def test_rejects_bounds_on_unknown_param():
    bad = replace(WEDGE, bounds={**WEDGE.bounds, "nope": (0.0, 1.0)})
    assert "params" in _checks(bad)


@pytest.mark.filterwarnings("ignore:.*encountered in divide:RuntimeWarning")  # zero-volume solid
def test_rejects_self_intersecting_section():
    def bowtie(p, part):
        return Section(outer=Polygon([(0, 0), (10, 10), (10, 0), (0, 10)]))

    bad = replace(WEDGE, geometry=Prismatic(section=bowtie, extrusion=wedge_module.extrusion))
    assert "section" in _checks(bad)


def test_rejects_bore_outside_the_brick():
    def outside(p, part):
        return PickupFeatures(bores=(Bore(x_mm=3 * p.U, z_mm=0.0, d_mm=2.0, clearance_mm=0.3),))

    assert "pickup" in _checks(replace(WEDGE, pickup=outside))


def test_thin_bore_wall_is_a_negative_margin_not_a_violation():
    def thin(p, part):
        f = wedge_module.pickup(p, part)
        b = f.bores[0]
        top = abs(b.x_mm) * math.tan(math.radians(p.groove_deg)) + p.H0
        return PickupFeatures(bores=(replace(b, z_mm=top - b.radius_mm - 0.5), *f.bores[1:]))

    bad = replace(WEDGE, pickup=thin)
    assert "pickup" not in _checks(bad)
    p = WEDGE.defaults
    margins = R.margins(bad, p, "A", B.sim_bodies(bad, p, "A"))
    # 0.5 mm below the slanted top vertically is (r + 0.5) cos(groove) - r perpendicular to it.
    r = wedge_module.pickup(p, "A").bores[0].radius_mm
    wall = (r + 0.5) * math.cos(math.radians(p.groove_deg)) - r
    assert margins["R13.bore_wall"] == pytest.approx(wall - 1.2, abs=1e-3)


def test_rejects_non_finite_margin():
    bad = replace(WEDGE, extra_rules=lambda p, part, bodies: {"R6.extra": math.nan})
    assert "rules" in _checks(bad)


def test_rejects_margin_for_unknown_requirement():
    bad = replace(WEDGE, extra_rules=lambda p, part, bodies: {"R99.extra": 1.0})
    assert "rules" in _checks(bad)


def test_rejects_non_convex_collision_piece():
    def solid(p, part):
        return B.solid(WEDGE, p, part, bored=False)

    def collision(p, part):
        cube = np.array([[x, y, z] for x in (-p.U, p.U) for y in (-p.D / 2, p.D / 2) for z in (0.0, p.H0)])
        return [np.vstack([cube, cube.mean(axis=0)])]  # an interior vertex: not a convex vertex set

    bad = replace(WEDGE, geometry=Solid3D(solid=solid, collision=collision,
                                          section=wedge_module.section))
    assert "sim_bodies" in _checks(bad)


def test_rejects_overlapping_stack():
    def low(p, placement):
        pos, quat = wedge_module.seat_pose(p, placement)
        return pos * np.array([1.0, 1.0, 0.5]), quat  # half the course pitch

    assert "lattice" in _checks(replace(WEDGE, seat_pose=low))


def test_rejects_non_unit_quaternion():
    def scaled(p, placement):
        pos, quat = wedge_module.seat_pose(p, placement)
        return pos, 2 * quat

    assert "lattice" in _checks(replace(WEDGE, seat_pose=scaled))


THRESHOLD_U = 25.5  # above the default U (25), below the largest seed-1 sample (25.69)


def test_some_sampled_points_exceed_the_threshold():
    """Precondition for the two tests below: defaults pass, and seed 1 samples U on both sides."""
    us = [v["U"] for v in C._sample_values(WEDGE, 3, 1)]
    assert WEDGE.defaults.U < THRESHOLD_U and min(us) < THRESHOLD_U < max(us)


def test_value_error_at_a_sampled_point_is_allowed():
    def picky(p, part):
        if p.U > THRESHOLD_U:
            raise ValueError("infeasible for this test")
        return wedge_module.section(p, part)

    ok = replace(WEDGE, geometry=Prismatic(section=picky, extrusion=wedge_module.extrusion))
    assert C.check_family(ok, samples=3, seed=1) == []


def test_other_exception_at_a_sampled_point_is_a_violation():
    def broken(p, part):
        if p.U > THRESHOLD_U:
            raise TypeError("bug")
        return wedge_module.section(p, part)

    bad = replace(WEDGE, geometry=Prismatic(section=broken, extrusion=wedge_module.extrusion))
    violations = C.check_family(bad, samples=3, seed=1)
    assert violations and all("TypeError" in v.message for v in violations)


def test_violation_says_where():
    bad = replace(WEDGE, extra_rules=lambda p, part, bodies: {"R99.extra": 1.0})
    (v,) = [v for v in C.check_family(bad, samples=0) if v.check == "rules"]
    assert v.where == "defaults, part A"
    assert "R99" in v.message


def _wedge_with_collision(collision):
    def solid(p, part):
        return B.solid(WEDGE, p, part, bored=False)

    return replace(WEDGE, geometry=Solid3D(solid=solid, collision=collision, section=wedge_module.section))


def test_rejects_collision_missing_material():
    def half(p, part):
        return B.collision(WEDGE, p, part)[0][:1]  # one of the chevron's two convex pieces

    assert "sim_bodies" in _checks(_wedge_with_collision(half))


def test_rejects_phantom_collision_material():
    def box(p, part):
        h = p.U * math.tan(math.radians(p.groove_deg))
        return [np.array([[x, y, z] for x in (-p.U, p.U) for y in (-p.D / 2, p.D / 2) for z in (0.0, h + p.H0)])]

    assert "sim_bodies" in _checks(_wedge_with_collision(box))
