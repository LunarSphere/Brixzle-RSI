"""Pickup trial and pickup box (issue #9, R12): fork, fixture, hover-spring drone, four checks.

Sim results are simulated, uncalibrated contact.
"""

import json
import math

import numpy as np
import pytest
from shapely.geometry import box as rect

from brixzle.engine import box_search as BS
from brixzle.engine import boxes as BX
from brixzle.engine.sim import drone as DR
from brixzle.engine.sim import fork as FK
from brixzle.engine.sim import pickup as PK
from brixzle.families.ab.family import FAMILY as AB
from brixzle.families.ab.params import V0
from brixzle.families.interface import grip_bores


# --- pure parts (no sim) ---


def test_payload_limit_is_35_g():
    assert DR.DroneConfig().payload_max_g == 35.0


def test_thrust_margin_by_hand():
    d = DR.DroneConfig()
    # 4 x 0.2942 N (datasheet) minus the weight of drone (43.4 + 2.7 g) and a 20 g payload.
    expect = 4 * 0.2942 - (43.4 + 2.7 + 20.0) * 1e-3 * 9.81
    assert d.thrust_margin_N(20.0) == pytest.approx(expect)


def test_pickup_pose_sample_is_symmetric_about_the_bore_axis():
    box = BX.default_boxes()["pickup"]["mean"]
    w = box.half_widths
    s = BS.pose_sample(box, 0.5, np.array([1.0, -1.0, 1.0]), None)
    assert s == pytest.approx({"dx": 0.5 * w["x_mm"], "dz": -0.5 * w["z_mm"], "psi": 0.5 * w["psi_deg"]})
    assert BS.pose_sample(box, 0.7, np.zeros(3), None) == {"dx": 0.0, "dz": 0.0, "psi": 0.0}


def test_fork_tines_sit_on_the_bores_and_carry_the_brick_by_the_bore_tops():
    fork = FK.build_fork(AB, V0, "A")
    bores = grip_bores(AB.pickup(V0, "A"))
    assert len(fork["tines"]) == len(bores)
    for (x, z), b in zip(fork["tines"], bores):
        assert x == pytest.approx(b.x_mm)
        assert z + fork["tine_r"] == pytest.approx(b.z_mm + b.radius_mm)  # tine top on the bore top
    assert fork["y_tip"] > V0.D / 2 and fork["y_face"] < -V0.D / 2  # through the brick, plate clear of it
    assert fork["gripper_mass_g"] > 0 and fork["body_mass_kg"] > DR.DroneConfig().mass_g * 1e-3


def test_fixture_supports_the_brick_and_lets_it_lift_straight_out():
    fx = PK.fixture_section(AB, V0, "A")
    brick = PK.seated_outline(AB, V0, "A")
    # Nothing of the fixture lies in the brick's upward sweep...
    assert fx.intersection(PK.vertical_sweep(brick, 50.0)).area < 1e-6
    # ...and there is material right under the brick.
    x0, z0, x1, _ = brick.bounds
    assert fx.intersection(rect(x0, z0 - 2.0, x1, z0)).area > 1.0


def test_bored_collision_leaves_the_bores_open():
    from brixzle.families import builders as B

    pieces = PK.bored_collision(AB, V0, "A")
    for b in grip_bores(AB.pickup(V0, "A")):
        axis = np.array([[b.x_mm, y, b.z_mm] for y in np.linspace(-V0.D / 2, V0.D / 2, 11)])
        assert not B.inside_any(axis, pieces).any()
    # The material just outside a bore is still solid.
    b = grip_bores(AB.pickup(V0, "A"))[0]
    assert B.inside_any(np.array([[b.x_mm, 0.0, b.z_mm - b.radius_mm - 0.5]]), pieces).all()


# --- with the sim (A/B V0) ---


@pytest.fixture(scope="module")
def rig():
    return PK.PickRig(AB, V0)


def test_aligned_pick_succeeds(rig):
    row = rig.pick({"dx": 0.0, "dz": 0.0, "psi": 0.0})
    assert row["outcome"] == "picked", row["outcome"]
    assert math.hypot(row["carry_err_mm"][0], row["carry_err_mm"][2]) <= 2.0 and row["carry_angle_deg"] <= 3.0
    assert row["peak_engage_N"] <= row["force_cap_N"]
    assert row["contacts"] and any(c["other"] == "brick0" for c in row["contacts"])


def test_a_pick_far_off_the_bores_fails(rig):
    # 3 mm across the tines: past the 0.3 mm bore clearance, and A/B has no lead-in.
    row = rig.pick({"dx": 3.0, "dz": 0.0, "psi": 0.0})
    assert row["outcome"] != "picked"


def test_release_withdraws_without_moving_the_seated_brick():
    out = PK.release_check(AB, V0)
    assert out["passed"] and out["drift_mm"] <= 2.0


@pytest.fixture(scope="module")
def search():
    # Coarse on purpose (tests stay fast): tol 1 checks scales 1 and 0 only. The baseline runs are
    # recorded with the CLI, see the PR.
    box = BX.default_boxes()["pickup"]["mean"]
    return box, BS.box_search(AB, V0, box, seed=0, tol=1.0, n_interior=0, workers=1)


def test_pickup_box_search_records_task_release_and_traces(search):
    box, result = search
    assert result["task"] == "pickup" and set(result["courses"]) == {"pick"}
    assert result["release"]["drift_mm"] >= 0.0
    assert result["passed"] == (result["scale"] >= 1.0 and result["release"]["passed"])
    course = result["courses"]["pick"]
    assert all("contacts" in d for d in course["drops"])
    assert course["simulated_count"] < course["drop_count"]  # the 15 identical scale-0 picks ran once
    json.dumps(result, allow_nan=False)


def test_pickup_parallel_reproduces_serial(search):
    box, serial = search
    parallel = BS.box_search(AB, V0, box, seed=0, tol=1.0, n_interior=0, workers=2)
    assert parallel["scale"] == serial["scale"]
    par = {(d["scale"], d["point"]): (d["outcome"], d["carry_err_mm"]) for d in parallel["courses"]["pick"]["drops"]}
    for d in serial["courses"]["pick"]["drops"]:
        assert par[(d["scale"], d["point"])] == (d["outcome"], d["carry_err_mm"])


def test_the_fork_goes_through_the_openings_as_predicted(ab_bodies):
    # Issue #12: A/B is picked through its two big windows. The fork's tines sit at the predicted
    # seats; an aligned pick seats, 1 mm across lands on a web (the sheared windows leave a
    # 0.71 mm aim margin), matching the predictor. Simulated, uncalibrated contact.
    from brixzle.engine.predictor import pickup as PP
    from brixzle.engine.sim import pickup as PK
    from brixzle.families.ab.family import FAMILY as AB
    from brixzle.families.ab.params import V0

    rig = PK.PickRig(AB, V0, bodies=ab_bodies)
    assert rig.fork["tines"] == [tuple(s) for s in PP.geometry(AB, V0)["seats"]]
    for dx, want in ((0.0, "picked"), (1.0, "jammed")):
        sample = {"dx": dx, "dz": 0.0, "psi": 0.0}
        assert rig.pick(sample, trace=False)["outcome"] == want
        assert PP.predict_pick(AB, V0, sample)["outcome"] == want
