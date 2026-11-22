"""Held placement (#191), simulated, uncalibrated contact. All samples are deterministic."""
from dataclasses import replace
import json
import math
from pathlib import Path

import numpy as np
import pytest

from brixzle.agents.check_family import load_family
from brixzle.engine.sim import held
from brixzle.engine.sim.drone import CF21B
from brixzle.engine.sim.trials import DropRig, ErrorModel, TrialConfig
from brixzle.families.interface import Placement, bore


@pytest.fixture(scope="module")
def family():
    return load_family(Path(__file__).parent / "fixtures", "gentle_inboard_sill")


@pytest.fixture(scope="module")
def aligned(family):
    return held.held_trial(family, family.defaults, Placement(0, 1), held.HeldSample(), held.HeldConfig())


def test_aligned_brick_is_carried_and_seats(aligned):
    assert aligned["outcome"] == "seated" and aligned["settled"], aligned
    assert aligned["first_contact_s"] > 0.1
    assert aligned["carry_force_N"] > 0
    assert aligned["err_mm"] == pytest.approx([0, 0, 0], abs=1)
    assert aligned["label"] == "simulated, uncalibrated contact"
    assert aligned["wrist_defaults"] == "guess"
    assert 0 < aligned["gripper_mass_kg"] < aligned["payload_kg"] <= CF21B.payload_max_g * 1e-3
    json.dumps(aligned, allow_nan=False)


def test_y_withdrawal_leaves_the_seated_brick_in_place(aligned):
    assert aligned["phases"]["release"]["climbed"]
    assert aligned["before_release"] == "seated" and aligned["stayed_seated"]
    assert aligned["withdrawn"]
    assert aligned["release_drift_m"] < 0.002  # R12, docs/requirements.md
    assert aligned["outcome"] == "seated"


def test_resting_payload_passes_release_force_check(family):
    cfg = held.HeldConfig(press_depth_m=0.)
    row = held.held_trial(family, family.defaults, 0, held.HeldSample(), cfg)
    assert row["stayed_seated"]
    assert row["peak_support_N"] > row["payload_kg"] * held.G > row["force_cap_N"]
    assert row["phases"]["release"]["within_force_limit"]
    assert row["sustained_push_N"] <= row["force_cap_N"]


def test_sustained_overpressure_fails_release_force_check(family, monkeypatch):
    rig = held.HeldRig(family, family.defaults, 0)
    control = rig._control

    def excessive_press(pos, vel, acc, orientation, loaded, pressing):
        control(pos, vel, acc, orientation, loaded, pressing)
        if pressing:
            # Fault injection: let the carrier's weight bear on the seated payload.
            rig.data.xfrc_applied[rig.carrier, 2] = 0.

    monkeypatch.setattr(rig, "_control", excessive_press)
    row = rig.trial(held.HeldSample())
    assert row["sustained_push_N"] > row["force_cap_N"]
    assert not row["phases"]["release"]["within_force_limit"]
    assert not row["phases"]["release"]["passed"]


@pytest.mark.parametrize(("cap", "gentle"), [(0.01, True), (0.001, False)])
def test_grip_caps_sustained_fork_load(family, cap, gentle):
    row = held.held_trial(family, family.defaults, 0, held.HeldSample(), held.HeldConfig(press_force_N=cap))
    grip = row["phases"]["grip"]
    assert grip["captured"] and grip["peak_engage_N"] > cap
    assert (grip["sustained_engage_N"] <= cap) == gentle
    assert grip["within_force_limit"] == gentle


def test_beyond_guide_cannot_score_a_seat_with_a_retained_fork(family):
    row = held.held_trial(family, family.defaults, 0, held.HeldSample(dy_m=0.09), held.HeldConfig())
    assert not row["passed"] and not row["withdrawn"], row
    assert row["outcome"] == "jammed" and row["phases"]["release"]["outcome"] == "fork_retained"
    assert row["post_departure_outcome"] is None
    assert not row["settled"]


def test_rigid_release_on_first_support_contact_matches_drop(family):
    p = family.defaults
    sample = ErrorModel(ideal=True, aim_bias=2.).sample(np.random.default_rng(0))
    drop = DropRig(family, p, TrialConfig()).drop(sample, trace=False)
    cfg = held.HeldConfig(wrist=held.WristConfig(rigid=True), release_at_contact=True)
    rig = held.HeldRig(family, p, 0, cfg)
    assert rig.model.nv == 12  # free brick and free Crazyflie, with a fixed wrist
    row = rig.trial(held.HeldSample(dx_m=0.002))
    assert row["first_contact_s"] == row["release_s"]
    assert row["first_contact_s"] > 0 and row["withdrawn"]
    assert row["outcome"] == drop["outcome"] == "seated"


def test_small_bores_are_rejected(family):
    small = replace(family, pickup=lambda p, part: (bore(-8, 8, 8), bore(8, 8, 8)))
    with pytest.raises(ValueError, match="16 mm"):
        held.held_trial(small, small.defaults, 0, held.HeldSample(), held.HeldConfig())


@pytest.mark.parametrize("settings", [
    lambda: held.HeldConfig(descent_m_s=0),
    lambda: held.HeldConfig(press_force_N=float("inf")),
    lambda: held.HeldSample(dyaw_rad=float("nan")),
    lambda: held.WristConfig(angular_k=(1, -1, 1)),
    lambda: held.WristConfig(centre_m=(0, 0)),
    lambda: held.WristConfig(rigid="false"),
    lambda: held.HeldConfig(release_at_contact="false"),
])
def test_invalid_settings_fail_before_simulation(settings):
    with pytest.raises(ValueError):
        settings()


def test_through_fork_stops_at_the_force_limit_and_resets():
    from brixzle.families.ab.family import FAMILY as ab
    from brixzle.families.ab.params import V0

    cfg = held.HeldConfig(press_force_N=0.01, wobble_m=0.0002, wobble_hz=1.5)
    rig = held.HeldRig(ab, V0, 0, cfg)
    sample = held.HeldSample(dx_m=0.001, dy_m=-0.001, dyaw_rad=0.01)
    row = rig.trial(sample)
    assert row["descent_stop"] == "force" and row["peak_support_N"] >= cfg.press_force_N
    assert row["fork_layout"] == "through"
    repeat = rig.trial(sample)
    assert repeat["sustained_push_N"] == pytest.approx(row["sustained_push_N"])
    assert repeat["outcome"] == row["outcome"]
    assert repeat["err_mm"] == pytest.approx(row["err_mm"])


def test_edge_uses_the_settled_supporter_and_remote_wrist_centre(family):
    wrist = held.WristConfig(centre_m=(0.001, -0.002, 0.003))
    rig = held.HeldRig(family, family.defaults, "plus_bare", held.HeldConfig(wrist=wrist))
    model = rig.model
    joints = np.flatnonzero(model.jnt_bodyid == model.body("wrist").id)
    assert len(joints) == 6
    # Moving the joint pivot does not move the physical wrist mass.
    np.testing.assert_allclose(model.body_ipos[model.body("wrist").id], [0., 0., 0.])
    np.testing.assert_allclose(model.jnt_pos[joints], np.tile(wrist.centre_m, (6, 1)))
    np.testing.assert_allclose(model.jnt_stiffness[joints], wrist.linear_k + wrist.angular_k)
    np.testing.assert_allclose(model.dof_damping[model.jnt_dofadr[joints]], wrist.linear_c + wrist.angular_c)
    assert rig.ref == (0, Placement(0, 0))
    row = rig.trial(held.HeldSample())
    assert row["first_contact_s"] is not None and np.isfinite(row["err_mm"]).all()


@pytest.mark.parametrize("placement", [Placement(0, 1, 0), Placement(0, 0.5)])
def test_invalid_lattice_placement_is_rejected(family, placement):
    with pytest.raises(ValueError, match="placement"):
        held.HeldRig(family, family.defaults, placement)


def test_cannot_build_a_tine_that_ends_behind_its_plate():
    from brixzle.engine.sim.fork import build_fork
    from brixzle.families.ab.family import FAMILY as ab

    with pytest.raises(ValueError, match="tip"):
        build_fork(ab, ab.defaults, "A", y_tip_mm=-100.)


def test_press_sensor_sees_a_fork_strike_before_brick_contact(family):
    import mujoco

    rig = held.HeldRig(family, family.defaults, 0)
    model, data = rig.model, rig.data
    # The brick is parked. Bring the plate alone into the floor, clear of the base.
    from brixzle.engine.sim.scene import park
    park(model, data, rig.info, rig.j)
    data.qpos[rig.fa+3:rig.fa+7] = held._wxyz(rig.drone_in_brick)
    jaw = rig.fork
    floor_z = model.geom_pos[rig.info["floor"], 2]
    data.qpos[rig.fa:rig.fa+3] = rig.mount + [0.4, 0., floor_z - jaw["plate"][0][2] * 1e-3 - 0.0001]
    mujoco.mj_forward(model, data)
    loads = rig._contacts()
    assert loads.normals_N == loads.fork_strike_N and loads.reaction_N > 0
    assert not loads.supported and not loads.on_floor and loads.fork_brick_N == 0


def test_physical_mission_budget_and_timing(family, aligned):
    rig = held.HeldRig(family, family.defaults, 0)
    assert rig.model.nmocap == 0
    assert rig.model.body_mass[rig.model.body("carrier").id] == pytest.approx(CF21B.mass_g / 1000)
    assert not any("withdraw" in rig.model.joint(i).name for i in range(rig.model.njnt))
    phases = aligned["phases"]
    assert list(phases) == ["takeoff", "approach", "grip", "transport", "release"]
    assert all(phase["wall_s"] >= 0 for phase in phases.values())
    assert phases["takeoff"]["tw"] == pytest.approx(CF21B.t_w(aligned["gripper_mass_kg"]*1000))
    assert phases["transport"]["tw"] == pytest.approx(CF21B.t_w(aligned["payload_kg"]*1000))
    assert phases["grip"]["rise_m"] > 0
    assert phases["transport"]["peak_offset_m"] >= 0 and phases["transport"]["travel_m"] > 0
    assert aligned["peak_thrust_N"] <= CF21B.thrust_max_N() + 1e-12
    assert aligned["force_cap_N"] == pytest.approx(.5 * CF21B.thrust_margin_N(aligned["payload_kg"]*1000))
    assert aligned["passed"] == all(phase["passed"] for phase in phases.values())


def test_measured_samples_draw_from_recorded_approaches():
    from brixzle.engine.sim.approach_error import DATA_DIR, load
    radii = load(DATA_DIR / "error-exp3-r4.json")["radii_mm"]
    hover = json.loads((DATA_DIR / "hover-exp3-r2.json").read_text())["approaches"]
    a, b = held.mission_samples(191)
    assert a != b and (a, b) == held.mission_samples(191)
    for s in (a, b):
        assert any(math.isclose(math.hypot(s.dx_m, s.dy_m) * 1000, r) for r in radii)
        assert any(np.allclose([abs(s.dz_m)*1000, abs(math.degrees(s.dpitch_rad)), abs(math.degrees(s.dyaw_rad))],
                               [r["z_rms_mm"], r["pitch_rms_deg"], r["yaw_rms_deg"]]) for r in hover)


def test_missed_pick_cannot_pass_a_mission(family):
    rig = held.HeldRig(family, family.defaults, 0)
    row = rig.trial(held.HeldSample(), pickup_sample=held.HeldSample(dx_m=.09))
    assert not row["phases"]["grip"]["passed"] and not row["passed"]
    assert row["phases"]["release"]["outcome"] == "skipped"
    assert row["outcome"] is None and row["post_departure_outcome"] is None
    assert row["err_mm"] is None and not row["settled"]


def test_grip_checks_each_tine_and_release_reports_sustained_and_peak_loads(aligned):
    grip = aligned["phases"]["grip"]
    assert grip["tines_inserted"] == [True, True]
    assert len(grip["tines_after_lift"]) == 2
    assert grip["within_force_limit"] == (grip["sustained_engage_N"] <= aligned["force_cap_N"])
    assert grip["peak_engage_N"] >= grip["sustained_engage_N"] >= 0.
    transport = aligned["phases"]["transport"]
    assert not transport["passed"] or all(transport["tines_after_transport"])
    release = aligned["phases"]["release"]
    assert release["within_force_limit"] == (aligned["sustained_push_N"] <= aligned["force_cap_N"])
    assert aligned["peak_contact_normals_N"] > aligned["peak_support_N"] > 0.
    assert release["clean_withdrawal"]
    assert not aligned["passed"] or release["within_force_limit"]


def test_computed_takeoff_skips_flight_and_keeps_empirical_samples(family):
    from dataclasses import asdict

    rig = held.HeldRig(family, family.defaults, 0,
                       held.HeldConfig(drone=replace(CF21B, tw_min=10.)))
    row = rig.trial(seed=10)
    pick, target = held.mission_samples(10)
    assert row["pickup_sample"] == asdict(pick) and row["placement_sample"] == asdict(target)
    assert row["phases"]["takeoff"]["outcome"] == "insufficient_thrust"
    assert not row["passed"] and row["peak_thrust_N"] == 0.
    assert row["phases"]["release"]["outcome"] == "skipped"


def test_payload_limit_and_controller_saturation(family):
    cfg = held.HeldConfig(drone=replace(CF21B, payload_max_g=10.))
    rig = held.HeldRig(family, family.defaults, 0, cfg)
    row = rig.trial(held.HeldSample())
    assert row["phases"]["transport"]["outcome"] == "over_budget" and not row["passed"]
    for acc in ([1000, 1000, 1000], [1000, 1000, -1000]):
        rig._control(rig.command, np.zeros(3), acc, rig.orientation, loaded=True, pressing=True)
        force = rig.data.xfrc_applied[rig.carrier, :3]
        assert np.linalg.norm(force) <= CF21B.thrust_max_N() + 1e-12
        weight = (CF21B.mass_g*1e-3 + row["payload_kg"])*held.G
        assert weight-force[2] <= rig.force_cap + 1e-12


def test_station_holds_the_other_ab_part_level():
    from brixzle.families.ab.family import FAMILY as ab
    from brixzle.engine.sim.scene import body_pose

    rig = held.HeldRig(ab, ab.defaults, "plus_bare")
    pos, quat = body_pose(rig.start, rig.info, rig.j)
    # A/B lists the negative half of its V as a band running from the centre towards -Y.
    assert pos*1e-3 == pytest.approx(rig.station, abs=0.001)
    assert (held._R(quat)*rig.seat_R.inv()).magnitude() < math.radians(held.PickConfig.carry_angle_deg)


def test_one_pitch_beyond_the_guide_is_classified_after_departure(family):
    pitch_m = family.lattice(family.defaults).U_mm*1e-3
    row = held.held_trial(family, family.defaults, 0, held.HeldSample(dx_m=pitch_m))
    assert row["withdrawn"] and row["climbed"]
    assert row["post_departure_outcome"] == row["outcome"] == "wrong_valley"
    assert row["err_mm"][0] == pytest.approx(pitch_m*1000, abs=1)
    assert not row["passed"]


def test_mirrored_station_approach_starts_outside_the_structure(family):
    rig = held.HeldRig(family, family.defaults, Placement(0, 1, -1))
    target, _ = family.seat_pose(family.defaults, rig.target)
    station_delta = rig.seat_R.inv().apply(rig.station - target*1e-3)
    assert station_delta[1] < -rig.stroke_m
