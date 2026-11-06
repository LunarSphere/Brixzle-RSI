"""The A/B baseline family reproduces aerial-assembly's V0 rule margins and CAD mass (issue #6).

Reference: tests/fixtures/ab_rules_reference.json, recorded by make_ab_rules_reference.py from
aerial-assembly @ a649663 (rules.py margins and cad.py mass for V0 parts A and B).

Margins that depend only on parameters must match exactly on every platform. Margins that depend on
the CadQuery mesh (mass, centre of mass) match to 0.05 across platforms, because tessellation
differs (see test_ab_drop_regression.py), and bit for bit on the reference platform.
"""

import json
import platform
from pathlib import Path

import pytest

from brixzle.families import builders as B
from brixzle.families import rules as R
from brixzle.families.ab.family import FAMILY as AB
from brixzle.families.ab.params import V0

REFERENCE = json.loads(
    (Path(__file__).parent / "fixtures" / "ab_rules_reference.json").read_text(encoding="utf-8")
)
ON_REFERENCE_PLATFORM = (
    platform.system() == "Darwin" and platform.machine() == "arm64"
    and REFERENCE["source"]["platform"].startswith("macOS") and "arm64" in REFERENCE["source"]["platform"]
)

# aerial-assembly rules.py margin -> brixzle key (requirement id . name)
AERIAL_TO_KEY = {
    "long_ramp_slides": "R6.long_ramp_slides",
    "steep_face_slides": "R6.steep_face_slides",
    "gutter_slides": "R6.gutter_slides",
    "capture_x": "R5.capture_x",
    "capture_y": "R5.capture_y",
    "nest_cone": "R11.nest_cone",
    "clearance_absorbs_tilt": "R11.clearance_absorbs_tilt",
    "seated_play": "R4.seated_play",
    "hook_depth": "R4.hook_depth",
    "tooth_strength": "R4.tooth_strength",
    "single_piece": "R13.single_piece",
    "tooth_printable": "R13.tooth_printable",
    "mass_min": "R9.mass_min",
    "mass_max": "R9.mass_max",
    "lip_web": "R11.lip_web",
    "lip_printable": "R11.lip_printable",
    "barb_fits": "R3.barb_fits",
    "fork_fits": "R10.fork_fits",
    "hangs_stable": "R10.hangs_stable",
    "tines_straddle_com": "R10.tines_straddle_com",
}
MESH_DEPENDENT = {"mass_min", "mass_max", "tooth_strength", "hangs_stable", "tines_straddle_com"}
PARTS = ("A", "B")


def _margins(part):
    return R.margins(AB, V0, part, B.sim_bodies(AB, V0, part))


def test_reference_is_sourced():
    assert REFERENCE["source"]["commit"].startswith("a649663")
    assert REFERENCE["params"] == V0.to_dict()


@pytest.mark.parametrize("part", PARTS)
def test_every_aerial_margin_has_a_key_and_nothing_else_is_reported(part):
    assert set(REFERENCE["parts"][part]["margins"]) == set(AERIAL_TO_KEY)
    # The shared R6.guide_slides (shallowest guide face) and R13.bore_wall are reported as well.
    assert set(_margins(part)) == set(AERIAL_TO_KEY.values()) | {"R6.guide_slides", "R13.bore_wall"}


@pytest.mark.parametrize("part", PARTS)
def test_rule_margins_match_aerial_assembly(part):
    ours, ref = _margins(part), REFERENCE["parts"][part]["margins"]
    for aerial, key in AERIAL_TO_KEY.items():
        tol = 0.05 if aerial in MESH_DEPENDENT else 1e-9
        assert ours[key] == pytest.approx(ref[aerial], abs=tol), (aerial, key)


@pytest.mark.skipif(not ON_REFERENCE_PLATFORM, reason="bit-exact mesh values only on the reference platform")
@pytest.mark.parametrize("part", PARTS)
def test_rule_margins_match_aerial_assembly_exactly(part):
    ours, ref = _margins(part), REFERENCE["parts"][part]["margins"]
    for aerial, key in AERIAL_TO_KEY.items():
        assert ours[key] == pytest.approx(ref[aerial], abs=1e-9), (aerial, key)


@pytest.mark.parametrize("part", PARTS)
def test_cad_mass_matches_aerial_assembly(part):
    ref = REFERENCE["parts"][part]["mass_g"]
    rel = 1e-9 if ON_REFERENCE_PLATFORM else 1e-3
    assert B.sim_bodies(AB, V0, part).mass_g == pytest.approx(ref, rel=rel)


def test_guide_slides_is_the_shallower_ramp():
    # The guide faces are the long ramp (theta) and the steep face (90 - alpha).
    friction = R.RuleSettings()
    margins = _margins("A")
    import math
    expect = min(V0.theta, V0.phi) - (math.degrees(math.atan(friction.mu_max)) + friction.slide_margin_deg)
    assert margins["R6.guide_slides"] == pytest.approx(expect)


def test_pickup_bores_are_the_aerial_fork_channels():
    bores = AB.pickup(V0, "A").bores
    assert [[b.x_mm, b.z_mm] for b in bores] == REFERENCE["parts"]["A"]["channels"]
    assert all(b.d_mm == V0.fork_d and b.clearance_mm == V0.fork_clearance for b in bores)


def test_barbs_are_rejected_until_soft_contact_is_supported():
    from dataclasses import replace

    with pytest.raises(ValueError, match="barb"):
        AB.geometry.section(replace(V0, barb_h=0.5), "A")


# Thinnest wall: part A, 1.222 mm to a lightening hole; part B, 1.300 mm to the outline.
@pytest.mark.parametrize("part,expect", [("A", 1.222 - 1.2), ("B", 1.300 - 1.2)])
def test_v0_bore_wall_margin_is_thin_but_positive(part, expect):
    # aerial-assembly cuts lightening holes to leave about p.wall = 1.2 mm round the bores.
    assert _margins(part)["R13.bore_wall"] == pytest.approx(expect, abs=1e-3)


def test_infeasible_profiles_raise_value_error():
    from dataclasses import replace

    from brixzle.families import contract as C
    # Contract sample 0 (seed 0) self-intersects in the sawtooth profile.
    values = C._sample_values(AB, 5, 0)[0]
    with pytest.raises(ValueError, match="cannot be built"):
        AB.geometry.section(replace(V0, **values), "A")


def test_parts_alternate_by_course_and_direction():
    assert [AB.part_for(k, 1) for k in range(4)] == ["A", "B", "A", "B"]
    assert [AB.part_for(k, -1) for k in range(2)] == ["B", "A"]


def test_contract_samples_include_feasible_designs():
    """Guards the contract run on ab against vacuity: infeasible samples are skipped, so some
    sampled points must be feasible for check_family to test anything beyond the defaults."""
    from dataclasses import replace

    from brixzle.families import contract as C

    feasible = 0
    for values in C._sample_values(AB, 5, 0):
        p = replace(V0, **values)
        try:
            for part in AB.parts:
                AB.geometry.section(p, part)
                AB.pickup(p, part)
            feasible += 1
        except ValueError:
            pass
    assert feasible >= 1
