"""Scripted SO-101 motion in a brick scene (#153, #154): Cartesian moves by top-down IK, jaw
commands, and the pick from the mould station. Units: mm and degrees at the interface, SI inside.

Moves are straight lines for the tool point, split into short IK steps that the position servos
track; the controller is deterministic and has no learning in it.
"""
from dataclasses import dataclass, field
import math

import mujoco
import numpy as np

from brixzle.engine.sim import arm as A
from brixzle.engine.sim import grasp as G
from brixzle.engine.sim.config import Physics
from brixzle.engine.sim.scene import activate, body_pose, build_scene
from brixzle.families import builders as B

MM = 1e-3
PHYSICS = Physics(timestep=0.001, friction=0.35, contact_timeconst=0.02)
GRIP_TORQUE = 0.35       # N m: the gripper servo's torque limit in these trials (about 4 N at the fingertips; the
                         # STS3215 allows 2.94, but more squeeze pushed the bricks round in the jaws, #153)
IMPRATIO = 50            # stiff friction against MuJoCo's slow creep of squeezed parts (menagerie's file sets 10)
JAW_SOLREF = 0.005       # contact time constant of the jaws' pads (menagerie: 0.01); halves the brick's shift in
                         # the jaws as the load comes on. Uncalibrated, like all contact here.
SPEED_MM_S = 100.0       # tool point speed on Cartesian moves
STEP_MM = 2.0            # IK step along a move
CEILING_MM = 80.0        # keep the tool point this far under the arm's base at most (its top-down ceiling is ~95 mm)
APPROACH_MM = 30.0       # the jaws come down beside the brick, not over it, so a short approach clears it


def quat_mat(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, dtype=float))
    return R.reshape(3, 3)


def mat_quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).ravel())
    return q


@dataclass
class Rig:
    """A compiled scene with the arm attached, and its live state."""
    family: object
    p: object
    model: object
    data: object
    info: dict
    arm: object                    # ArmIndex
    grasp: object                  # Grasp, per the held part
    q: np.ndarray = field(default=None)   # the five placing joints last commanded
    jaw_mm: float = 60.0           # the jaw opening last commanded
    log: list = field(default_factory=list)

    @property
    def jaw(self):
        return G.jaws()

    def tcp_local_m(self):
        return self.model.site_pos[self.arm.site].copy()

    def gripper_world(self, brick_pos_mm, brick_quat):
        """(R, o mm): the gripper body's pose that holds a brick at this pose with ``grasp``."""
        Rb = quat_mat(brick_quat)
        Rg, og = self.grasp.gripper_frame(self.jaw)
        return Rb @ Rg, np.asarray(brick_pos_mm) + Rb @ og

    def tcp_target(self, R, o_mm):
        return (o_mm * MM + R @ self.tcp_local_m()), math.atan2(R[1, 0], R[0, 0])

    def ik(self, R, o_mm, q0=None):
        t, yaw = self.tcp_target(R, o_mm)
        return A.ik_top_down(self.model, self.arm, t, yaw, q0=q0, data=mujoco.MjData(self.model), wide=q0 is None)

    # --- running ---

    def _ctrl(self, q, jaw_mm):
        self.data.ctrl[self.arm.act[:5]] = q
        self.data.ctrl[self.arm.act[5]] = self.jaw.angle_for(jaw_mm)

    def hold(self, seconds):
        for _ in range(int(round(seconds / self.model.opt.timestep))):
            mujoco.mj_step(self.model, self.data)

    def set_jaw(self, jaw_mm, seconds=0.6):
        self.jaw_mm = jaw_mm
        self._ctrl(self.q, jaw_mm)
        self.hold(seconds)

    def move(self, R, o_from_mm, o_to_mm, speed=SPEED_MM_S, settle=0.2):
        """Tool along a straight line (gripper orientation fixed), IK every STEP_MM, servos tracking."""
        o_from, o_to = np.asarray(o_from_mm, float), np.asarray(o_to_mm, float)
        n = max(1, int(math.ceil(np.linalg.norm(o_to - o_from) / STEP_MM)))
        dt = (np.linalg.norm(o_to - o_from) / n) / speed
        for k in range(1, n + 1):
            res = self.ik(R, o_from + (o_to - o_from) * k / n, q0=self.q)
            if not res.ok:
                raise RuntimeError(f"IK failed on a move at step {k}/{n}: {res.err_mm:.2f} mm, tilt {res.tilt_deg:.2f} deg")
            self.q = res.q
            self._ctrl(self.q, self.jaw_mm)
            self.hold(dt)
        self.hold(settle)

    def go(self, R, o_mm, seconds=1.5):
        """Joint-space move to a pose (for the first approach from wherever the arm is)."""
        res = self.ik(R, o_mm)
        if not res.ok:
            raise RuntimeError(f"IK failed: {res.err_mm:.2f} mm, tilt {res.tilt_deg:.2f} deg")
        q0 = self.data.qpos[self.arm.qpos[:5]].copy()
        n = int(round(seconds / 0.02))
        for k in range(1, n + 1):
            self._ctrl(q0 + (res.q - q0) * k / n, self.jaw_mm)
            self.hold(0.02)
        self.q = res.q
        self.hold(0.3)

    # --- poses ---

    def brick_pose(self, j=0):
        pos, quat = body_pose(self.data, self.info, j)
        return pos, quat

    def gripper_pose_mm(self):
        g = self.arm.gripper
        return self.data.xmat[g].reshape(3, 3).copy(), self.data.xpos[g] / MM

    def brick_in_gripper(self, j=0):
        """The brick's pose in the gripper body's frame: (position mm, rotation)."""
        Rg, og = self.gripper_pose_mm()
        pos, quat = self.brick_pose(j)
        return Rg.T @ (pos - og), Rg.T @ quat_mat(quat)


def tune_grip(model):
    """The arm rig's grasping settings (see the constants): gripper torque limit, friction
    stiffness, and the jaw pads' contact stiffness (menagerie's jaw geoms are the priority-1 ones)."""
    gid = model.actuator(A.PREFIX + "gripper").id
    model.actuator_forcerange[gid] = [-GRIP_TORQUE, GRIP_TORQUE]
    model.opt.impratio = IMPRATIO
    jaw = model.geom_priority == 1
    model.geom_solref[jaw] = [JAW_SOLREF, 1.0]


def station_rig(family, p, part="A", station_mm=(200.0, 0.0), brick_yaw=0.0, arm_base_mm=(0.0, 0.0), arm_yaw=0.0,
                riser_mm=None, lift_mm=100.0, grasp=None, visual=False, physics=PHYSICS):
    """A scene with the arm (its base on a ``riser_mm`` block), the mould station at ``station_mm``
    on the floor and one brick seated in it. ``riser_mm`` None raises the base just enough for the
    tool point to reach ``lift_mm`` over the grasp: a straight-down pose tops out under 100 mm over
    the base (#152)."""
    grasp = G.choose_grasp(family, p, part) if grasp is None else grasp
    cr = G.cradle(family, p, part)
    bodies = {part: B.sim_bodies(family, p, part)}
    seat = {}

    def extra(asset, world):
        seat["z"] = G.add_cradle(asset, world, _rotated(cr, brick_yaw), station_mm, 0.0)

    _, xml, info = build_scene(family, p, bodies, [], 1, physics, visual=visual, body_parts=[part], extra=extra)
    if riser_mm is None:
        grasp_z = -cr.z_floor + grasp.z_mm          # the fixed pad's height over the floor
        riser_mm = float(np.clip(grasp_z + max(lift_mm, APPROACH_MM) - CEILING_MM, 0.0, 150.0))
    model, ainfo = A.attach_arm(xml, (arm_base_mm[0], arm_base_mm[1], riser_mm), yaw=arm_yaw)
    tune_grip(model)
    data = mujoco.MjData(model)
    rig = Rig(family=family, p=p, model=model, data=data, info=info, arm=ainfo["index"], grasp=grasp)
    quat = [math.cos(brick_yaw / 2), 0, 0, math.sin(brick_yaw / 2)]
    activate(model, data, info, 0, (station_mm[0], station_mm[1], seat["z"]), quat)
    # start the arm folded over the station, jaws open, servos holding that pose
    R, o = rig.gripper_world((station_mm[0], station_mm[1], seat["z"]), quat)
    res = rig.ik(R, o + np.array([0.0, 0.0, APPROACH_MM]))
    if not res.ok:
        raise RuntimeError("the station is out of the arm's reach")
    data.qpos[rig.arm.qpos[:5]] = res.q
    rig.q = res.q
    rig.jaw_mm = grasp.open_mm
    data.qpos[rig.arm.qpos[5]] = G.jaws().angle_for(grasp.open_mm)
    rig._ctrl(rig.q, rig.jaw_mm)
    mujoco.mj_forward(model, data)
    rig.hold(0.5)
    # The mould sits a little under the brick (its gap, and the erosion that keeps the sampled
    # surface under the brick between samples), so the brick settles about 1 mm: aim at where it rests.
    rig.placed = (station_mm[0], station_mm[1], seat["z"], quat)
    pos, q = rig.brick_pose()
    rig.station = (float(pos[0]), float(pos[1]), float(pos[2]), q)
    return rig


def _rotated(cr, yaw):
    if abs(yaw) > 1e-9:
        raise NotImplementedError("the mould station is built for a brick at yaw 0")
    return cr


def pick(rig, approach_mm=None, lift_mm=100.0):
    """Pick the station's brick: over it, down, close, lift. Returns the brick's pose in the jaws
    after closing and after the lift, and the change between them."""
    x, y, z, quat = rig.station
    R, o = rig.gripper_world((x, y, z), quat)
    up = np.array([0.0, 0.0, APPROACH_MM if approach_mm is None else approach_mm])
    rig.go(R, o + up)
    rig.move(R, o + up, o)
    rig.set_jaw(rig.grasp.squeeze_mm, seconds=0.8)
    p0, R0 = rig.brick_in_gripper()
    rig.move(R, o, o + np.array([0.0, 0.0, lift_mm]))
    rig.hold(0.5)
    p1, R1 = rig.brick_in_gripper()
    ang = math.degrees(math.acos(np.clip((np.trace(R0.T @ R1) - 1) / 2, -1.0, 1.0)))
    return {"slip_mm": float(np.linalg.norm(p1 - p0)), "slip_deg": float(ang),
            "lifted_mm": float(rig.brick_pose()[0][2] - z)}


# --- clearance at the placements ---


_TCP_LOCAL = None


def tcp_local_mm():
    """The tool point in the gripper body's frame (mm), from the bare arm model."""
    global _TCP_LOCAL
    if _TCP_LOCAL is None:
        m = A.arm_spec().compile()
        _TCP_LOCAL = m.site_pos[m.site(A.TCP_SITE).id] / MM
    return _TCP_LOCAL


def grasp_tcp_mm(family, p, placements, grasps, with_yaw=False):
    """The tool point (world, mm) when holding each placed brick at its seat with its part's grasp
    (and, ``with_yaw``, the jaw yaw at each, rad)."""
    jaw = G.jaws()
    out, yaws = [], []
    for b in placements:
        g = grasps[family.part_for(b.course, b.direction)]
        pos, quat = family.seat_pose(p, b)
        Rb = quat_mat(quat)
        Rg, og = g.gripper_frame(jaw)
        R = Rb @ Rg
        out.append(np.asarray(pos) + Rb @ og + R @ tcp_local_mm())
        yaws.append(math.atan2(R[1, 0], R[0, 0]))
    return (np.array(out), np.array(yaws)) if with_yaw else np.array(out)


def placement_clashes(family, p, structure, env=None, grasps=None, physics=PHYSICS, tries=30):
    """An arm base that clears the structure at every placement, kinematically (no dynamics).

    For each brick k, with bricks 0..k at their seats, the gripper is posed by IK at the grasp on
    brick k, first with the jaws closed on it, then opened to release; any contact between the arm
    and anything but brick k (an earlier brick, a base, the floor) is a clash. Candidate bases come
    from ``arm.plan_bases`` best first; the first with no clash wins, else the one with the fewest
    (of ``tries``). Returns {"base", "clashes", "unreached", "tried"}.
    """
    env = A.top_down_envelope(step_mm=5.0) if env is None else env
    parts = [family.part_for(b.course, b.direction) for b in structure.bricks]
    grasps = {pt: G.choose_grasp(family, p, pt) for pt in set(parts)} if grasps is None else grasps
    tcp, yaws = grasp_tcp_mm(family, p, structure.bricks, grasps, with_yaw=True)
    best, tried = None, 0
    for plan in A.plan_bases(family, p, structure, tcp, env, yaws=yaws):
        tried += 1
        res = _clashes_at(family, p, structure, parts, grasps, plan, physics)
        if best is None or len(res["clashes"]) < len(best["clashes"]):
            best = res
        if not res["clashes"] or tried >= tries:
            break
    if best is None:
        return {"base": None, "clashes": [], "unreached": list(range(len(parts))), "tried": 0}
    return {**best, "tried": tried}


def _clashes_at(family, p, structure, parts, grasps, plan, physics):
    from brixzle.engine.sim.scene import park
    bodies = {pt: B.sim_bodies(family, p, pt) for pt in set(parts)}
    _, xml, info = build_scene(family, p, bodies, structure.bases, len(parts), physics, visual=False, body_parts=parts)
    model, ainfo = A.attach_arm(xml, plan[:3], yaw=plan[3])
    data = mujoco.MjData(model)
    idx = ainfo["index"]
    arm_bodies = set(int(b) for b in idx.bodies)
    brick_of = {int(b): j for j, b in enumerate(info["bodies"])}
    jaw = G.jaws()
    clashes, unreached = [], []
    for k, b in enumerate(structure.bricks):
        for j in range(len(parts)):
            if j <= k:
                pos, quat = family.seat_pose(p, structure.bricks[j])
                activate(model, data, info, j, pos, quat)
            else:
                park(model, data, info, j)
        g = grasps[parts[k]]
        pos, quat = family.seat_pose(p, b)
        Rb = quat_mat(quat)
        Rg, og = g.gripper_frame(jaw)
        R, o = Rb @ Rg, np.asarray(pos) + Rb @ og
        t = o * MM + R @ model.site_pos[idx.site]
        res = A.ik_top_down(model, idx, t, math.atan2(R[1, 0], R[0, 0]), data=mujoco.MjData(model), wide=True)
        if not res.ok:
            unreached.append(k)
            continue
        for stage, opening in (("closed", g.width_mm), ("released", g.open_mm)):
            data.qpos[idx.qpos[:5]] = res.q
            data.qpos[idx.qpos[5]] = jaw.angle_for(opening)
            mujoco.mj_forward(model, data)
            for c in data.contact[:data.ncon]:
                b1, b2 = int(model.geom_bodyid[c.geom1]), int(model.geom_bodyid[c.geom2])
                if (b1 in arm_bodies) == (b2 in arm_bodies):
                    continue
                other = b2 if b1 in arm_bodies else b1
                if brick_of.get(other) == k or c.dist > 0:
                    continue
                arm_geom = model.geom(c.geom1 if b1 in arm_bodies else c.geom2)
                what = f"brick{brick_of[other]}" if other in brick_of else model.geom(c.geom2 if b1 in arm_bodies else c.geom1).name or "world"
                clashes.append({"brick": k, "stage": stage, "with": what, "arm": arm_geom.name or model.body(int(arm_geom.bodyid[0])).name,
                                "depth_mm": round(-c.dist / MM, 2)})
    return {"base": plan, "clashes": clashes, "unreached": unreached}
