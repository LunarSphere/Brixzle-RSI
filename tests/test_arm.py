"""SO-101 arm in the scene (#152): the model loads, attaches to a brick scene, holds a pose, and
its top-down reach is measured."""
import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

from brixzle.engine.sim import arm as A
from brixzle.engine.sim.config import Physics
from brixzle.engine.sim.scene import build_scene
from brixzle.families import builders as B
from brixzle.families import registry

FABLE = Path(__file__).resolve().parents[1] / "runs" / "fable"


def get_family(name):
    """A/B from the registry; the fable designs from the local, git-ignored runs/fable (skipped
    when absent: they are not registered in the repo yet)."""
    if name == "ab":
        return registry.get("ab")
    if not (FABLE / name / "family.py").exists():
        pytest.skip(f"{name} is only available locally under runs/fable")
    from brixzle.agents.check_family import load_family
    return load_family(FABLE, name)

PHYS = Physics(timestep=0.001, friction=0.35, contact_timeconst=0.02)
ACTUATORS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def test_arm_model_loads_with_six_position_actuators():
    model = A.arm_spec().compile()
    assert mujoco.__version__.startswith("3.16")
    assert tuple(model.actuator(i).name for i in range(model.nu)) == ACTUATORS
    assert model.njnt == 6


def test_arm_attaches_to_a_brick_scene_without_moving_the_bricks_ids():
    fam = get_family("ab")
    p = fam.defaults
    bodies = {part: B.sim_bodies(fam, p, part) for part in fam.parts}
    base_model, xml, info = build_scene(fam, p, bodies, [(-1, 3)], 2, PHYS, visual=False)
    model, ainfo = A.attach_arm(xml, base_mm=(0.0, -200.0, None), yaw=0.0)
    assert [model.body(f"brick{j}").id for j in range(2)] == info["bodies"]
    assert model.nu == 6 and model.actuator(0).name == A.PREFIX + "shoulder_pan"
    # the arm's base stands on the scene's floor
    floor_top = model.geom_pos[model.geom("floor").id][2] + 0.0005
    assert model.body(A.PREFIX + "base").pos[2] == pytest.approx(floor_top, abs=1e-9)
    assert ainfo["tcp_site"] == model.site(A.PREFIX + "gripperframe").id
    assert model.opt.timestep == pytest.approx(0.001)


def test_arm_holds_a_pose_within_1mm_for_2s():
    model = A.arm_spec().compile()
    model.opt.timestep = 0.001
    idx = A.ArmIndex.of(model, prefix="")
    data = mujoco.MjData(model)
    q = np.array([0.3, -0.6, 0.9, 0.8, 0.2, 0.0])
    want = A.tcp_at(model, idx, q)
    data.qpos[idx.qpos] = q
    data.ctrl[idx.act] = q
    for _ in range(2000):
        mujoco.mj_step(model, data)
    got = data.site_xpos[idx.site].copy()
    assert np.linalg.norm(got - want) * 1000 < 1.0


def test_top_down_ik_reaches_a_near_point_and_rejects_a_far_one():
    model = A.arm_spec().compile()
    idx = A.ArmIndex.of(model, prefix="")
    near = A.ik_top_down(model, idx, np.array([0.20, 0.05, 0.05]), yaw=0.4)
    assert near.ok and near.err_mm < 0.5 and near.tilt_deg < 1.0 and near.yaw_err_deg < 1.0
    far = A.ik_top_down(model, idx, np.array([0.80, 0.0, 0.05]), yaw=0.0)
    assert not far.ok


def test_reach_envelope_and_overhang_courses_are_recorded():
    env = A.top_down_envelope(step_mm=20.0)
    assert env["reachable"].any() and not env["reachable"].all()
    r_max = env["r_mm"][env["reachable"].any(axis=0)].max()
    assert 150.0 < r_max < 500.0
    rec = A.max_overhang_courses(get_family("ab"), env)
    assert 1 <= rec["courses"] <= 12
    assert rec["base_mm"] is not None


def test_top_down_reach_has_a_ceiling_that_a_riser_lifts():
    env = A.top_down_envelope(step_mm=20.0)
    top = env["z_mm"][env["reachable"].any(axis=1)].max()
    assert top < 120.0                      # the wrist's range caps a straight-down pose
    point = np.array([170.0, 0.0, top + 60.0])
    axis = env["pan_axis_mm"]
    assert not A.reachable(env, (-axis[0], 0.0, 0.0), 0.0, point)
    assert A.reachable(env, (-axis[0], 0.0, 80.0), 0.0, point)


def test_overhang_plan_reports_its_riser():
    rec = A.max_overhang_courses(get_family("ab"), A.top_down_envelope(step_mm=20.0))
    assert rec["riser_mm"] >= 0.0
    floor = A.floor_mm(get_family("ab"), get_family("ab").defaults, A.S.overhang(1, +1, 2))
    assert rec["base_mm"][2] == pytest.approx(floor + rec["riser_mm"])


@pytest.mark.parametrize("name", ["fable_twinhook4", "fable_allround"])
def test_overhang_courses_for_the_fable_designs(name):
    rec = A.max_overhang_courses(get_family(name), A.top_down_envelope(step_mm=20.0))
    assert 1 <= rec["courses"] <= 12 and rec["base_mm"] is not None
