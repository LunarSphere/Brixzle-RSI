"""Guarded held missions (#197). Dimensions from aerial-assembly at fc4f724."""
from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np
import pytest

from brixzle.agents.check_family import load_family
from brixzle.engine import heldmap, structures
from brixzle.engine.sim import held
from brixzle.engine.sim.scene import MM, activate


@pytest.fixture(scope="module")
def family():
    return load_family(Path(__file__).parent / "fixtures", "gentle_inboard_sill")


@pytest.fixture(scope="module")
def rig(family):
    return held.HeldRig(family, family.defaults, 1)


def test_compiled_guards_and_frame_follow_the_motor_plane(rig):
    m = rig.model
    xy = .03536
    for i, pos in enumerate(((xy, -xy), (-xy, -xy), (-xy, xy), (xy, xy))):
        geom = m.geom(f"cf_guard{i}")
        assert geom.type == mujoco.mjtGeom.mjGEOM_CYLINDER
        np.testing.assert_allclose(geom.size[:2], [.034, .0153])
        np.testing.assert_allclose(geom.pos, [*pos, 0.])
        assert geom.bodyid == rig.carrier and geom.contype and geom.conaffinity
    frame = m.geom("cf_frame")
    assert frame.type == mujoco.mjtGeom.mjGEOM_BOX and frame.bodyid == rig.carrier
    np.testing.assert_allclose(frame.size, [.041, .041, .0087])
    np.testing.assert_allclose(frame.pos, [0., 0., .0075])


def test_conservative_guard_budget(rig):
    guarded_mass = (43.4 + (37 - 34) + 2.7)*MM
    assert rig.model.body_mass[rig.carrier] == pytest.approx(guarded_mass)
    margin = 4*.2942 - (guarded_mass + rig.payload)*9.81
    assert rig.force_cap == pytest.approx(.5*margin)


def test_reject_short_guard_clearance_in_gripper_config(tmp_path):
    path = tmp_path / "gripper.json"
    path.write_text('{"clear_guard_m":0.0009}')
    with pytest.raises(ValueError, match="guard clearance"):
        heldmap.gripper_config("unused", path)
    path.write_text('{"clear_guard_m":0.012}')
    assert heldmap.gripper_config("unused", path).clear_guard_m == .012


def test_neighbour_guard_contact_stops_the_current_mission(rig, monkeypatch):
    # Put an earlier free brick into a motor disk at grip entry. The actual
    # solver contact must stop the mission before transport or a scored release.
    grip = rig._grip

    def obstruct(*args):
        centre = rig.data.geom_xpos[rig.model.geom("cf_guard0").id].copy()
        activate(rig.model, rig.data, rig.info, 0, centre/MM, [1., 0., 0., 0.])
        return grip(*args)

    monkeypatch.setattr(rig, "_grip", obstruct)
    row = rig.trial(held.HeldSample())
    assert not row["passed"] and row["outcome"] == "guard_strike"
    assert row["phases"]["grip"]["outcome"] == "guard_strike"
    assert row["strike"]["obstacle"] == "placed_brick"
    assert row["strike"]["brick"] == structures.brick_name(rig.placed[0])
    assert row["phases"]["transport"]["outcome"] == "skipped"
    assert heldmap.case_summary([row])["guard_strikes"] == 1


def test_bridge_mission_includes_both_anchored_bases(family):
    bridge = structures.bridge(1)
    rig = held.HeldRig(family, family.defaults, bridge.bricks[-1], structure=bridge)
    assert rig.model.geom("base1_0").id in rig.info["base_geoms"]
    assert rig.placed == bridge.bricks[:-1]


def test_brick_clearance_follows_the_physical_carrier_and_payload(rig):
    # A compliant wrist need not keep the brick at its nominal carry offset.
    d = rig.data
    d.qpos[rig.fa:rig.fa+7] = [*rig.mount, *held._wxyz(rig.drone_in_brick)]
    activate(rig.model, d, rig.info, rig.j, [0, 0, 0], [1, 0, 0, 0])
    assert rig.brick_guard_clearance_m() == pytest.approx(.008)
    activate(rig.model, d, rig.info, rig.j, [0, 0, 7.5], [1, 0, 0, 0])
    assert rig.brick_guard_clearance_m() == pytest.approx(.0005)


def test_nominal_reach_checks_earlier_bricks_even_overhead(family):
    from brixzle.engine.sim.guards import placement_clearances

    st = structures.wall(2, 1)
    first, second = st.bricks
    seat_pose = family.seat_pose
    def overhead(p, b):
        xyz, quat = seat_pose(p, b)
        return np.asarray(xyz) + [0, 0, 120 if b == first else 0], quat
    raised = replace(family, seat_pose=overhead)
    part = family.part_for(0, 1)
    fork, collision, *_ = held._fork_fit(family, family.defaults, part, held.HeldConfig())
    parts = {part: (fork, collision)}
    clear = placement_clearances(family, family.defaults, st.bricks, parts)
    assert all(not row["blocked_by"] for row in clear)
    blocked = placement_clearances(raised, family.defaults, st.bricks, parts)
    assert blocked == [dict(brick=structures.brick_name(first), blocked_by=[]),
                       dict(brick=structures.brick_name(second), blocked_by=[structures.brick_name(first)])]


def test_tall_station_causes_guard_strike(family):
    from brixzle.engine.sim.funnel import FunnelStation
    rig = held.HeldRig(family, family.defaults, 0, held.HeldConfig(station=FunnelStation(height_m=.055)))
    row = rig.trial(held.HeldSample())
    assert row["outcome"] == "guard_strike" and row["strike"]["obstacle"] == "station"


def test_fork_geometry_rejects_a_mount_below_the_guard_margin():
    from brixzle.engine.sim.fork import ForkParams
    with pytest.raises(ValueError, match="guard clearance"):
        ForkParams(clear_guard=.5)


@pytest.mark.parametrize("obstacle", ["ground", "structure", "payload"])
def test_guard_contact_names_the_scene_obstacle(rig, obstacle):
    # Isolate one collider at the edge of the first motor disk.
    m, d = rig.model, rig.data
    mujoco.mj_copyData(d, m, rig.start)
    d.qpos[rig.fa+3:rig.fa+7] = [1., 0., 0., 0.]
    if obstacle == "ground":
        d.qpos[rig.fa:rig.fa+3] = [.6, 0., m.geom("floor").pos[2]+.0152]
    elif obstacle == "payload":
        d.qpos[rig.fa:rig.fa+3] = [.6, 0., .3]
        activate(m, d, rig.info, rig.j, [654., -35., 290.], [1., 0., 0., 0.])
    else:
        # Park the earlier bricks and use the high point of a base hull.
        for j in range(rig.j):
            activate(m, d, rig.info, j, [600., 400.+j*100, 300.], [1., 0., 0., 0.])
        base = m.geom("base0_0")
        mesh = m.mesh(m.geom_dataid[base.id])
        vertices = m.mesh_vert[mesh.vertadr[0]:mesh.vertadr[0]+mesh.vertnum[0]]
        world = vertices @ d.geom_xmat[base.id].reshape(3, 3).T + d.geom_xpos[base.id]
        d.qpos[rig.fa:rig.fa+3] = world[world[:, 2].argmax()] + [-.03536, .03536, .014]
    mujoco.mj_forward(m, d)
    with pytest.raises(held._FlightFailure) as exc:
        rig._check_airframe()
    assert exc.value.outcome == "guard_strike" and exc.value.metrics["strike"]["obstacle"] == obstacle


def test_takeoff_checks_restored_neighbour_before_any_stroke(rig):
    # A carried-over neighbour already intersects a disk at the starting hover.
    mujoco.mj_copyData(rig.data, rig.model, rig.start)
    station = rig.data.xpos[rig.target_body].copy()
    rig._start_hover(station)
    states = rig.brick_states()[:rig.j]
    centre = rig.data.geom_xpos[rig.model.geom("cf_guard0").id].copy()
    states[0] = (centre/MM, [1., 0., 0., 0.], np.zeros(3), np.zeros(3))
    row = rig.trial(held.HeldSample(), support_states=states)
    assert row["phases"]["takeoff"]["outcome"] == "guard_strike" and not row["passed"]
    assert row["strike"]["obstacle"] == "placed_brick" and row["peak_thrust_N"] == 0.
    assert row["phases"]["approach"]["outcome"] == "skipped"


def test_carried_brick_losing_margin_on_descent_fails_release(family, monkeypatch):
    rig = held.HeldRig(family, family.defaults, 0)
    release = rig._release

    def close_gap(*args):
        # Fault injection isolates clearance during descent while the fork still
        # supports the payload. The independent contact sensor stays live.
        monkeypatch.setattr(rig, "brick_guard_clearance_m", lambda: .0005)
        return release(*args)

    monkeypatch.setattr(rig, "_release", close_gap)
    row = rig.trial(held.HeldSample())
    assert not row["passed"] and row["phases"]["release"]["outcome"] == "guard_clearance"
    assert row["guard_clearance_m"] == .0005 and "strike" not in row
