"""Placement box search (issue #8): drops at box poses, bisection on scale, a contact trace per drop.

Sim results are simulated, uncalibrated contact.
"""

import numpy as np
import pytest

from brixzle.engine import box_search as BS
from brixzle.engine import boxes as BX
from brixzle.engine.sim import trials as T
from brixzle.families.ab.family import FAMILY as AB
from brixzle.families.ab.params import V0


# --- pure parts (no sim) ---


def test_unit_points_are_centre_corners_faces_then_interior():
    pts = BS.unit_points(n_interior=4, seed=0)
    assert pts.shape == (1 + 8 + 6 + 4, 3)
    assert np.array_equal(pts[0], [0, 0, 0])
    corners = {tuple(c) for c in pts[1:9]}
    assert corners == {(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)}
    faces = pts[9:15]
    assert all(np.count_nonzero(f) == 1 and np.abs(f).max() == 1 for f in faces)
    assert np.all(np.abs(pts[15:]) < 1)


def test_unit_points_are_reproducible_from_the_seed():
    assert np.array_equal(BS.unit_points(4, 3), BS.unit_points(4, 3))
    assert not np.array_equal(BS.unit_points(4, 3)[15:], BS.unit_points(4, 4)[15:])


@pytest.mark.parametrize("threshold", [0.0, 0.37, 1.0, 1.42, 1.99])
def test_bisection_finds_the_largest_passing_scale(threshold):
    calls = []

    def ok(s):
        calls.append(s)
        return s <= threshold

    scale, evals = BS.bisect_scale(ok, s_max=2.0, tol=0.05)
    assert threshold - 0.05 <= scale <= threshold
    assert [e["scale"] for e in evals] == calls


def test_bisection_caps_at_s_max_and_floors_at_zero():
    assert BS.bisect_scale(lambda s: True, s_max=2.0, tol=0.05)[0] == 2.0
    assert BS.bisect_scale(lambda s: s == 0.0, s_max=2.0, tol=0.05)[0] == 0.0
    assert BS.bisect_scale(lambda s: False, s_max=2.0, tol=0.05)[0] == 0.0


def test_lowest_corner_releases_at_the_minimum_clearance():
    box = BX.default_boxes()["placement"]["p95"]
    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    low = BS.pose_sample(box, 1.5, np.array([0.0, -1.0, 0.0]), cfg)
    high = BS.pose_sample(box, 1.5, np.array([0.0, 1.0, 0.0]), cfg)
    assert low["clear"] == pytest.approx(cfg.error.clearance[0])
    assert high["clear"] == pytest.approx(cfg.error.clearance[0] + 2 * 1.5 * box.half_widths["z_mm"])


def test_pose_sample_maps_x_and_theta_around_the_aim():
    box = BX.default_boxes()["placement"]["mean"]
    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    s = BS.pose_sample(box, 0.5, np.array([1.0, 0.0, -1.0]), cfg)
    assert s["dx"] == pytest.approx(cfg.error.aim_bias + 0.5 * box.half_widths["x_mm"])
    assert s["pitch"] == pytest.approx(-0.5 * box.half_widths["theta_deg"])
    assert (s["dy"], s["roll"], s["yaw"], s["vel"]) == (0.0, 0.0, 0.0, [0.0, 0.0, 0.0])


def test_rejects_a_pickup_box():
    with pytest.raises(ValueError, match="placement"):
        BS.box_search(AB, V0, BX.default_boxes()["pickup"]["mean"])


# --- with the sim (A/B V0) ---


@pytest.fixture(scope="module")
def baseline():
    # Coarse on purpose (tests stay fast): tol 0.5 and no interior points. The full-resolution
    # baseline runs (tol 0.05, 4 interior points) are recorded with the CLI, see the PR.
    box = BX.default_boxes()["placement"]["mean"]
    return box, BS.box_search(AB, V0, box, seed=0, tol=0.5, n_interior=0)


def test_aligned_drop_has_a_contact_trace_ending_on_the_support():
    rig = BS.DropRig(AB, V0, T.TrialConfig(error=T.ErrorModel(ideal=True)), course=1)
    row = rig.drop(BS.pose_sample(BX.default_boxes()["placement"]["mean"], 0.0, np.zeros(3), rig.cfg))
    assert row["outcome"] == "seated"
    trace = row["contacts"]
    assert trace and all({"piece", "other", "start_s", "end_s", "touches", "peak_N"} <= set(c) for c in trace)
    assert [c["start_s"] for c in trace] == sorted(c["start_s"] for c in trace)
    assert any(c["other"] == "brick0" and c["active_at_end"] for c in trace)  # resting on the seated A


def test_every_drop_in_the_search_has_a_contact_trace(baseline):
    _, result = baseline
    assert result["drops"]
    assert all(d["contacts"] for d in result["drops"])


def test_baseline_box_is_reproducible_from_the_seed(baseline):
    box, result = baseline
    again = BS.box_search(AB, V0, box, seed=0, tol=0.5, n_interior=0)
    assert again["scale"] == result["scale"]
    assert [d["outcome"] for d in again["drops"]] == [d["outcome"] for d in result["drops"]]


def test_result_records_its_inputs_and_is_json_safe(baseline):
    import json

    box, result = baseline
    assert 0.0 <= result["scale"] <= 2.0
    assert result["passed"] == (result["scale"] >= 1.0)
    assert result["family"] == "ab" and result["seed"] == 0 and result["box"] == box.to_dict()
    assert result["label"] == "simulated, uncalibrated contact"
    json.dumps(result, allow_nan=False)
