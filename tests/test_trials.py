# From LunarSphere/aerial-assembly brixzle/tests/test_trials.py, test_ab.py, test_approach_error.py @ a649663
# Changed: only drop-trial tests kept (assembly, rules and fixture tests not ported); new imports;
# classify, ErrorModel validation and Tolerance tests added; trials take a family (A/B) and a course
# instead of BrickParams, bricks and a part. Results are simulated, uncalibrated contact.

import numpy as np
import pytest

from brixzle.engine.sim import trials as T
from brixzle.families.ab.family import FAMILY as AB
from brixzle.families.ab.params import V0
from brixzle.families.interface import Placement


@pytest.mark.parametrize("course", [0, 1])  # part A onto the base; part B onto a seated A
def test_aligned_drop_seats(course):
    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    rows, _ = T.drop_trials(AB, V0, cfg, offsets=[0.0], course=course)
    assert rows[0]["outcome"] == "seated"
    assert rows[0]["settled"]


def test_capture_covers_ten_mm_around_aim_in_x():
    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    aim = cfg.error.aim_bias
    rows, _ = T.drop_trials(AB, V0, cfg, offsets=[aim + d for d in (-10.0, -5.0, 0.0, 5.0, 10.0)])
    assert all(r["outcome"] == "seated" for r in rows), [(r["dx"], r["outcome"]) for r in rows]


def test_capture_margin_from_simulated_sweep():
    assert T.capture_margin(AB, V0, 5.0) >= 0
    assert T.capture_margin(AB, V0, 14.8) < 0


def test_capture_radius_is_measured_from_the_aim():
    rows = [{"dx": 3.0 + d, "aim": 3.0, "outcome": "seated" if abs(d) <= 4 else "jammed"}
            for d in range(-8, 9)]
    assert T.capture_radius(rows) == 4.0


def test_gauss_mode_draws_are_stable():
    err, rng, ref = T.ErrorModel(), np.random.default_rng(7), np.random.default_rng(7)
    for _ in range(5):
        s = err.sample(rng)
        dx, dy = np.clip(ref.normal(0, 4.0, 2), -12.0, 12.0)
        expect = dict(dx=float(dx + 3.0), dy=float(dy), yaw=float(ref.normal(0, 5.0)),
                      roll=float(ref.normal(0, 3.0)), pitch=float(ref.normal(0, 3.0)),
                      clear=float(ref.uniform(5.0, 30.0)), vel=ref.normal(0, 0.03, 3).tolist())
        assert {k: s[k] for k in expect} == expect


@pytest.mark.parametrize("kwargs", [
    {"radial_mode": "uniform"},
    {"radial_mode": "ring", "radius_mm": float("nan")},
    {"radial_mode": "ring", "radius_mm": -1.0},
    {"radial_mode": "bootstrap", "radii_mm": ()},
    {"radial_mode": "bootstrap", "radii_mm": (1.0, float("inf"))},
])
def test_error_model_rejects_invalid_settings(kwargs):
    with pytest.raises(ValueError):
        T.ErrorModel(**kwargs)


def test_tolerance_defaults():
    tol = T.Tolerance()
    assert (tol.seat_xz, tol.seat_y, tol.seat_angle, tol.drift) == (1.0, 2.5, 3.0, 2.0)


@pytest.mark.parametrize("err,angle,on_floor,expect", [
    ((0.5, 1.0, -0.5), 1.0, False, "seated"),
    ((0.5, 1.0, -0.5), 1.0, True, "fell"),
    ((2 * V0.U, 0.0, 0.0), 0.5, False, "wrong_valley"),
    ((V0.U, 0.0, 0.0), 0.5, False, "wrong_valley"),       # one voxel over (the baseline's failures)
    ((-V0.U + 1.5, 0.0, 0.0), 0.5, False, "wrong_valley"),
    ((V0.U / 2, 0.0, 0.0), 0.5, False, "jammed"),          # between valleys
    ((3.0, 0.0, 0.0), 0.5, False, "jammed"),
    ((0.0, 0.0, 0.0), 10.0, False, "jammed"),
    ((0.0, 3.0, 0.0), 0.0, False, "jammed"),
])
def test_classify(err, angle, on_floor, expect):
    assert T.classify(AB.lattice(V0), np.array(err), angle, T.Tolerance(), on_floor) == expect


# --- course 1 lands across two bricks (running bond, issue #82) ---


def _ideal(family, course, p=None):
    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    rig = T.DropRig(family, p or family.defaults, cfg, course=course)
    return rig, rig.drop(cfg.error.sample(None), trace=False)


def test_a_b_course_1_seats_across_two_bricks():
    rig, row = _ideal(AB, 1, V0)
    assert rig.placed == [Placement(0, 1), Placement(0, 3)] and rig.target == Placement(1, 2)
    assert rig.base == (0, 6)
    assert row["outcome"] == "seated", row


def test_course_0_still_drops_onto_the_base_alone():
    """A seated neighbour at Placement(0, 1 + span) was tried and left out (#82): it sits on A/B's
    approach side, which a real build keeps open (assembly order), and A/B then jams at its own aim."""
    rig, row = _ideal(AB, 0, V0)
    assert rig.placed == [] and rig.target == Placement(0, 1) and rig.base == (0, 4)
    assert row["outcome"] == "seated", row
    placed, target, base = T.rig_layout(AB.lattice(V0), 0, neighbour=True)
    assert placed == [Placement(0, 3)] and target == Placement(0, 1) and base == (0, 6)


def test_a_stack_only_v_does_not_seat_across_two_bricks(stack_v):
    _, row = _ideal(stack_v, 1)
    assert row["outcome"] != "seated", row


def test_a_one_voxel_brick_keeps_the_stack_drop():
    from brixzle.families.pyramid.family import FAMILY as PYRAMID

    rig = T.DropRig(PYRAMID, PYRAMID.defaults, T.TrialConfig(error=T.ErrorModel(ideal=True)), course=1)
    assert rig.placed == [Placement(0, 1)] and rig.target == Placement(1, 1)
