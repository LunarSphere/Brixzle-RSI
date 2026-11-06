"""Opening pickup predictor (issue #12): the fork goes through large through-Y openings.

Geometric and quasi-static, translation only: entry (each tine inside its opening along the whole
depth) and seat (on lift the brick slides on the tines until friction or a corner stops it).
Outputs are predicted, quasi-static 2D.
"""

import json
import math

import numpy as np
import pytest
from shapely.geometry import Polygon, box

from brixzle.engine.predictor import pickup as PP

FLAT = ((-12.5, 12.5, 0.0, 0.0),)  # one straight band through a 25 mm depth


def _roof(slope_deg=45.0):
    """A 10 mm wide opening with a peaked roof: walls to z 15, roof edges at ``slope_deg`` up to an
    apex at the top centre (the A/B windows peak in a corner instead; the slide is the same)."""
    h = 5.0 * math.tan(math.radians(slope_deg))
    return Polygon([(0, 0), (10, 0), (10, 15), (5, 15 + h), (0, 15)])


# --- capture region ---


def test_capture_region_shrinks_by_the_tine_radius():
    region = PP.capture_region(box(0, 0, 10, 20), FLAT, r=1.0)
    assert region.bounds == pytest.approx((1, 1, 9, 19))


def test_a_sheared_band_narrows_the_straight_through_opening():
    # The side band lifts the opening 5 mm at the face; a straight tine sees the overlap: 20 - 5 tall.
    bands = ((-3.0, 3.0, 0.0, 0.0), (3.0, 12.5, 0.0, 5.0), (-3.0, -12.5, 0.0, 5.0))
    region = PP.capture_region(box(0, 0, 10, 20), bands, r=1.0)
    x0, z0, x1, z1 = region.bounds
    assert (z1 - z0) == pytest.approx(20 - 5 - 2)
    assert (x1 - x0) == pytest.approx(8)


def test_yaw_narrows_the_opening_by_the_depth_times_tan_psi():
    psi = math.radians(2.0)
    region = PP.capture_region(box(0, 0, 10, 20), FLAT, r=1.0, psi=psi)
    x0, _, x1, _ = region.bounds
    assert (x1 - x0) == pytest.approx(8 - 25 * math.tan(psi), rel=1e-6)


# --- seat: sliding up on lift ---


def test_a_peaked_opening_slides_the_tine_into_its_apex():
    t, reason = PP.slide_up(_roof(45.0), np.array([8.0, 5.0]), mu=0.3)  # tan 45 = 1 > 0.3: slides
    assert t == pytest.approx((5.0, 20.0), abs=1e-6), reason
    assert "apex" in reason


def test_friction_holds_the_tine_on_a_roof_flatter_than_mu():
    t, reason = PP.slide_up(_roof(45.0), np.array([8.0, 5.0]), mu=1.5)  # tan 45 = 1 < 1.5: sticks
    assert t == pytest.approx((8.0, 17.0), abs=1e-6)  # where the vertical lift met the roof
    assert "friction" in reason


def test_a_flat_top_does_not_centre():
    t, reason = PP.slide_up(box(0, 0, 10, 20), np.array([3.0, 4.0]), mu=0.2)
    assert t == pytest.approx((3.0, 20.0)) and "flat" in reason


# --- the interface ---


def test_pickup_features_have_openings():
    from brixzle.families.interface import PickupFeatures

    assert PickupFeatures().openings == () and PickupFeatures().bores == ()


# --- A/B ---


@pytest.fixture(scope="module")
def ab():
    from brixzle.families.ab.family import FAMILY as AB
    from brixzle.families.ab.params import V0

    return AB, V0, PP.geometry(AB, V0)


def test_ab_is_picked_through_its_two_big_windows(ab):
    AB, V0, geo = ab
    assert len(geo["openings"]) == 2
    for o in geo["openings"]:
        assert o.area > 100  # the big windows (about 105-122 mm^2), not the small holes or the bores
    (x0, z0), (x1, z1) = geo["seats"]
    assert abs(x1 - x0) == pytest.approx(24.6, abs=1.0)  # about the bores' 24.7 mm spacing


def test_ab_aligned_pick_seats_on_the_fork(ab):
    AB, V0, geo = ab
    r = PP.predict_pick(AB, V0, {"dx": 0.0, "dz": 0.0, "psi": 0.0}, geo=geo)
    assert r["outcome"] == "picked", r["reason"]
    assert max(abs(v) for v in r["carry_mm"]) < 0.01
    json.dumps(r, allow_nan=False)


def test_ab_far_off_pick_lands_on_the_face(ab):
    AB, V0, geo = ab
    r = PP.predict_pick(AB, V0, {"dx": 8.0, "dz": 0.0, "psi": 0.0}, geo=geo)
    assert r["outcome"] == "jammed" and "face" in r["reason"]


def test_ab_predicted_pickup_box_beats_the_bores_but_not_the_target(ab):
    # The bores scored 0 (sim, #9). Through the windows the mean box is partly covered (< 1).
    from brixzle.engine import boxes as BX

    AB, V0, _ = ab
    r = PP.predicted_pickup_box(AB, V0, BX.default_boxes()["pickup"]["mean"])
    assert 0.0 < r["scale"] < 1.0 and r["label"] == PP.LABEL
    assert r["seconds"] < 2.0
    json.dumps(r, allow_nan=False)


def test_cli_records_a_new_run(tmp_path):
    args = ["--family", "ab", "--statistic", "mean", "--tol", "0.5", "--runs", str(tmp_path)]
    PP.main(args)
    PP.main(args)
    runs = sorted(tmp_path.iterdir())
    assert len(runs) == 2  # never overwrites
    assert json.loads((runs[0] / "result.json").read_text())["label"] == PP.LABEL
