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
from brixzle.families import interface as I
from brixzle.families.interface import GripFeature, Prismatic, Section, Solid3D
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


# --- grip features (R10 grippable, issue #50) ---


def _violations(family):
    return [v for v in C.check_family(family, samples=0) if v.check == "pickup"]


def test_a_brick_with_no_grip_feature_fails_r10():
    (v,) = _violations(replace(WEDGE, pickup=lambda p, part: ()))
    assert "R10" in v.message and "grip feature" in v.message


def test_rejects_bore_outside_the_brick():
    def outside(p, part):
        return (I.bore(x_mm=3 * p.U, z_mm=0.0, d_mm=2.6),)

    assert "pickup" in _checks(replace(WEDGE, pickup=outside))


def _wedge_with_window():
    """The wedge with a small square lightening hole at x = 5 mm, mid thickness."""
    def section(p, part):
        s = wedge_module.section(p, part)
        z = 5.0 * math.tan(math.radians(p.groove_deg)) + p.H0 / 2
        return replace(s, holes=(Polygon([(4, z - 1), (6, z - 1), (6, z + 1), (4, z + 1)]),))

    return replace(WEDGE, geometry=Prismatic(section=section, extrusion=wedge_module.extrusion)), section


def test_rejects_a_bore_that_breaks_into_a_hole():
    family, section = _wedge_with_window()

    def into_hole(p, part):
        c = section(p, part).holes[0].centroid
        return (I.bore(x_mm=c.x + 1.5, z_mm=c.y, d_mm=2.6),)

    (v,) = _violations(replace(family, pickup=into_hole))
    assert "breaks into hole" in v.message


def test_any_kind_of_feature_on_the_brick_makes_it_grippable():
    """No bore and no fork: a lip a hook could catch is enough for R10 (the gripper comes later)."""
    def lip(p, part):
        h = p.U * math.tan(math.radians(p.groove_deg))
        return (GripFeature(kind="lip", x_mm=-p.U + 2.0, z_mm=h + p.H0 - 1.0, size_mm=(4.0, 2.0)),)

    family = replace(WEDGE, pickup=lip)
    assert _violations(family) == []
    p = WEDGE.defaults
    margins = R.margins(family, p, "A", B.sim_bodies(family, p, "A"))
    assert margins["R10.grippable"] == 0.0 and "R13.bore_wall" not in margins


def test_a_section_hole_can_be_the_grip_feature():
    family, section = _wedge_with_window()
    family = replace(family, pickup=lambda p, part: (I.opening(section(p, part).holes[0]),))
    assert _violations(family) == []
    (f,) = family.pickup(WEDGE.defaults, "A")
    assert f.kind == "opening" and f.size_mm == pytest.approx((2.0, 2.0)) and f.x_mm == pytest.approx(5.0)


def test_rejects_a_grip_feature_off_the_brick():
    def far(p, part):
        return (GripFeature(kind="lip", x_mm=3 * p.U, z_mm=0.0, size_mm=(4.0, 2.0)),)

    (v,) = _violations(replace(WEDGE, pickup=far))
    assert "breaks out" in v.message


@pytest.mark.parametrize("feature", [
    GripFeature(kind="", x_mm=0.0, z_mm=20.0, size_mm=(2.0, 2.0)),
    GripFeature(kind="slot", x_mm=0.0, z_mm=20.0, size_mm=(0.0, 2.0)),
    GripFeature(kind="slot", x_mm=math.nan, z_mm=20.0, size_mm=(2.0, 2.0)),
    "a bore near the middle",
])
def test_rejects_a_malformed_grip_feature(feature):
    assert _violations(replace(WEDGE, pickup=lambda p, part: (feature,)))


def test_thin_bore_wall_is_a_negative_margin_not_a_violation():
    def thin(p, part):
        b, *rest = wedge_module.pickup(p, part)
        top = abs(b.x_mm) * math.tan(math.radians(p.groove_deg)) + p.H0
        return (replace(b, z_mm=top - b.radius_mm - 0.5), *rest)

    bad = replace(WEDGE, pickup=thin)
    assert "pickup" not in _checks(bad)
    p = WEDGE.defaults
    margins = R.margins(bad, p, "A", B.sim_bodies(bad, p, "A"))
    # 0.5 mm below the slanted top vertically is (r + 0.5) cos(groove) - r perpendicular to it.
    r = wedge_module.pickup(p, "A")[0].radius_mm
    wall = (r + 0.5) * math.cos(math.radians(p.groove_deg)) - r
    assert margins["R13.bore_wall"] == pytest.approx(wall - 1.2, abs=1e-3)


def test_the_loop_rule_set_checks_grippable_and_payload_but_not_the_fork():
    p = WEDGE.defaults
    bodies = B.sim_bodies(WEDGE, p, "A")
    margins = R.margins(WEDGE, p, "A", bodies)
    assert not {"R10.hangs_stable", "R10.tines_straddle_com"} & set(margins)
    assert margins["R10.grippable"] == 1.0  # two bores: one more than R10 needs
    # 35 g carries the brick and the gripper; until the gripper is designed it counts as 5 g.
    assert R.RuleSettings().payload_max == 35.0 and R.RuleSettings().gripper_allowance == 5.0
    assert margins["R10.within_payload"] == pytest.approx(35.0 - (bodies.mass_g + 5.0))
    heavier = R.margins(WEDGE, p, "A", bodies, R.RuleSettings(gripper_allowance=9.0))
    assert heavier["R10.within_payload"] == pytest.approx(35.0 - (bodies.mass_g + 9.0))


def test_the_fork_checks_are_kept_for_the_gripper_step():
    """R10.hangs_stable and R10.tines_straddle_com against a 2 mm fork, worked by hand for the wedge."""
    p = WEDGE.defaults
    bodies = B.sim_bodies(WEDGE, p, "A")
    fork = R.fork_margins(WEDGE, p, "A", bodies, tine_d_mm=2.0)
    assert set(fork) == {"R10.hangs_stable", "R10.tines_straddle_com"}
    x = p.bore_x_frac * p.U
    z = x * math.tan(math.radians(p.groove_deg)) + p.bore_z_frac * p.H0
    assert fork["R10.hangs_stable"] == pytest.approx(z + 1.0 - bodies.com_mm[2] - 1.0)
    assert fork["R10.tines_straddle_com"] == pytest.approx(x - abs(bodies.com_mm[0]) - 1.0)
    lip_only = replace(WEDGE, pickup=lambda p, part: (GripFeature("lip", 0.0, p.H0, (4.0, 2.0)),))
    assert R.fork_margins(lip_only, p, "A", bodies) == {}  # no bores, so nothing for a fork to check


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
