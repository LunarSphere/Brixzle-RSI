"""The SO-101 arm (#152): load it, attach it to a brick scene, solve top-down inverse kinematics,
and measure where it can place a brick.

The model is mujoco_menagerie's ``robotstudio_so101`` (``brixzle/data/so101``; source, commit and the
reason for the choice are in its README). Brick geometry is in millimetres; MuJoCo is in SI.

"Top-down" means the gripper points straight down: its body's +z axis is world +z (the jaw tips
lie along its -z), and its +x axis, the direction the jaws close along, is horizontal at a chosen
yaw. The jaws are not used to grasp here; #153 chooses the grasp. Reach is judged at the nominal
grasp point, the centre of the brick's bounding box, until #153 replaces it.
"""
from dataclasses import dataclass
import math
from pathlib import Path

import mujoco
import numpy as np

from brixzle.engine import structures as S
from brixzle.families import builders as B

ARM_XML = Path(__file__).resolve().parents[2] / "data" / "so101" / "so101.xml"
PREFIX = "arm/"
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
ARM = JOINTS[:5]              # the joints that place the gripper; the last one opens the jaws
TCP_SITE = "gripperframe"
MM = 1e-3
PAN_MARGIN = math.radians(5.0)  # keep targets this far inside the pan joint's range
BASE_CLEAR_MM = 70.0            # the arm's base stays this far (in plan) from every brick and base


def arm_spec():
    """The arm as an MjSpec, with absolute mesh paths so it can be attached anywhere."""
    spec = mujoco.MjSpec.from_file(str(ARM_XML))
    mesh_dir = ARM_XML.parent / spec.meshdir
    for mesh in spec.meshes:
        mesh.file = str(mesh_dir / mesh.file)
    spec.meshdir = ""
    return spec


def _yaw_quat(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def attach_arm(xml, base_mm, yaw=0.0):
    """Compile the brick scene ``xml`` (``scene.build_scene``) with the arm attached under ``PREFIX``.

    ``base_mm`` is (x, y, z) of the arm's base in the scene's world frame (mm); z None stands it on
    the scene's floor. ``yaw`` turns the base about world z (rad; at 0 the arm reaches toward +x).
    The scene's own <option> stays; the arm's is not used. Bodies already in the scene keep their ids.
    Returns (model, info) with info's ``index`` (ArmIndex), ``tcp_site`` and ``base_m``.
    """
    spec = mujoco.MjSpec.from_string(xml)
    x, y, z = base_mm
    if z is None:
        floor = spec.geom("floor")
        z_m = float(floor.pos[2]) + 0.0005   # build_scene sinks the plane 0.5 mm under the lowest base
    else:
        z_m = z * MM
    frame = spec.worldbody.add_frame(pos=[x * MM, y * MM, z_m], quat=_yaw_quat(yaw))
    child = arm_spec()
    # The scene's solver settings win (the arm's file sets its own); copy them over so the attach
    # has no conflict to warn about.
    child.option.timestep, child.option.iterations = spec.option.timestep, spec.option.iterations
    child.option.ls_iterations, child.option.impratio = spec.option.ls_iterations, spec.option.impratio
    spec.attach(child, prefix=PREFIX, frame=frame)
    model = spec.compile()
    idx = ArmIndex.of(model, PREFIX)
    return model, {"index": idx, "tcp_site": idx.site, "base_m": np.array([x * MM, y * MM, z_m]), "yaw": yaw}


@dataclass(frozen=True)
class ArmIndex:
    """Addresses of the arm in a compiled model."""
    qpos: np.ndarray     # qpos address of each joint in JOINTS
    dof: np.ndarray      # dof address of each joint
    act: np.ndarray      # actuator id of each joint
    lo: np.ndarray       # joint range
    hi: np.ndarray
    site: int            # the tool point (``gripperframe``)
    gripper: int         # the gripper body (carries the fixed jaw)
    bodies: np.ndarray   # every arm body

    @classmethod
    def of(cls, model, prefix=PREFIX):
        js = [model.joint(prefix + j) for j in JOINTS]
        bodies = [i for i in range(model.nbody) if model.body(i).name.startswith(prefix)] if prefix else list(range(1, model.nbody))
        return cls(qpos=np.array([j.qposadr[0] for j in js]), dof=np.array([j.dofadr[0] for j in js]),
                   act=np.array([model.actuator(prefix + j).id for j in JOINTS]),
                   lo=np.array([j.range[0] for j in js]), hi=np.array([j.range[1] for j in js]),
                   site=model.site(prefix + TCP_SITE).id, gripper=model.body(prefix + "gripper").id,
                   bodies=np.array(bodies))


def tcp_at(model, idx, q, data=None):
    """World position (m) of the tool point with the arm's joints at ``q``."""
    data = mujoco.MjData(model) if data is None else data
    data.qpos[idx.qpos[:len(q)]] = q
    mujoco.mj_kinematics(model, data)
    return data.site_xpos[idx.site].copy()


@dataclass(frozen=True)
class IKResult:
    q: np.ndarray        # the five placing joints (rad)
    err_mm: float        # tool point position error
    tilt_deg: float      # angle between the gripper's axis and vertical
    yaw_err_deg: float   # jaw-axis yaw error
    ok: bool


def _rot_err(R, R_des):
    """Rotation vector (world frame) taking R to R_des."""
    E = R_des @ R.T
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, E.ravel())
    v = np.zeros(3)
    mujoco.mju_quat2Vel(v, q, 1.0)
    return v


def ik_top_down(model, idx, target_m, yaw, q0=None, data=None, iters=200, tol_mm=0.5, tol_deg=1.0, wide=False):
    """Damped least squares for the five placing joints: tool point at ``target_m`` (world, m),
    gripper pointing straight down, jaws closing along ``yaw``. Joint limits are respected.
    Seeds: ``q0`` if given, else a few elbow-up poses facing the target, and with ``wide`` a grid of
    60 more (for a pose found from scratch, e.g. far below a raised base; slow when it fails)."""
    data = mujoco.MjData(model) if data is None else data
    target = np.asarray(target_m, dtype=float)
    R_des = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    dof = idx.dof[:5]
    lo, hi = idx.lo[:5], idx.hi[:5]
    seeds = [np.asarray(q0, dtype=float)] if q0 is not None else _seeds(model, idx, target, data, wide)
    best = None
    jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    for q in seeds:
        q = np.clip(q.copy(), lo, hi)
        for _ in range(iters):
            data.qpos[idx.qpos[:5]] = q
            mujoco.mj_kinematics(model, data)
            mujoco.mj_comPos(model, data)
            ep = target - data.site_xpos[idx.site]
            er = _rot_err(data.xmat[idx.gripper].reshape(3, 3), R_des)
            if np.linalg.norm(ep) * 1000 < tol_mm * 0.2 and np.degrees(np.linalg.norm(er)) < tol_deg * 0.2:
                break
            mujoco.mj_jacSite(model, data, jacp, jacr, idx.site)
            J = np.vstack([jacp[:, dof], 0.05 * jacr[:, dof]])   # 0.05 m per rad: weigh 1 deg like ~1 mm
            e = np.concatenate([ep, 0.05 * er])
            lam = 1e-4
            dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), e)
            q = np.clip(q + np.clip(dq, -0.2, 0.2), lo, hi)
        res = _judge(model, idx, data, q, target, R_des, tol_mm, tol_deg)
        if best is None or (res.ok, -res.err_mm) > (best.ok, -best.err_mm):
            best = res
        if best.ok:
            break
    return best


def _judge(model, idx, data, q, target, R_des, tol_mm, tol_deg):
    data.qpos[idx.qpos[:5]] = q
    mujoco.mj_kinematics(model, data)
    R = data.xmat[idx.gripper].reshape(3, 3)
    err = float(np.linalg.norm(target - data.site_xpos[idx.site]) * 1000)
    tilt = float(np.degrees(math.acos(np.clip(R[2, 2], -1.0, 1.0))))
    yaw_err = float(np.degrees(abs(math.atan2(np.cross(R[:, 0], R_des[:, 0])[2], R[:, 0] @ R_des[:, 0]))))
    above = bool(np.all(data.xpos[idx.bodies, 2] > data.xpos[idx.bodies[0], 2] - 0.005))  # nothing under the base
    ok = err < tol_mm and tilt < tol_deg and yaw_err < tol_deg and above
    return IKResult(q=q.copy(), err_mm=err, tilt_deg=tilt, yaw_err_deg=yaw_err, ok=ok)


def _seeds(model, idx, target, data, wide=False):
    base = data.xpos[idx.bodies[0]] if data.xpos.any() else np.zeros(3)
    mujoco.mj_kinematics(model, data)
    base = data.xpos[idx.bodies[0]]
    pan = math.atan2(target[1] - base[1], target[0] - base[0]) - _base_yaw(model, data, idx)
    pan = (pan + math.pi) % (2 * math.pi) - math.pi
    first = [(0.0, 0.5, 1.0), (-0.8, 0.8, 1.2), (0.6, -0.2, 1.3), (0.3, 1.2, 0.2)]
    grid = [(l, e, w) for l in (-1.2, -0.6, 0.0, 0.6, 1.2) for e in (-0.6, 0.2, 0.9, 1.5) for w in (-0.6, 0.4, 1.3)] if wide else []
    return [np.array([pan, lift, elbow, wrist, 0.0]) for lift, elbow, wrist in first + grid]


def _base_yaw(model, data, idx):
    R = data.xmat[idx.bodies[0]].reshape(3, 3)
    return math.atan2(R[1, 0], R[0, 0])


# --- reach ---


def _forward(model, idx, data):
    """Unit vector in the base's plan along which the arm reaches at pan 0 (the base's +x, by test)."""
    mujoco.mj_kinematics(model, data)
    R = data.xmat[idx.bodies[0]].reshape(3, 3)
    return R[:, 0]


_ENV_CACHE = {}


def top_down_envelope(step_mm=5.0, r_max_mm=500.0, z_lo_mm=-100.0, z_hi_mm=450.0):
    """Where the bare arm (base at the origin) can put its tool point pointing straight down.

    Returns {"r_mm", "z_mm", "reachable"}: r is the plan distance from the pan axis, z the height
    above the base, ``reachable[i, j]`` for (z_mm[i], r_mm[j]). A cell counts only if the pose is
    reached at both jaw yaws along the reach line (the tool point sits 12 mm off the jaw axis), so
    any yaw is safe to assume. Memoised per grid.
    """
    key = (step_mm, r_max_mm, z_lo_mm, z_hi_mm)
    if key in _ENV_CACHE:
        return _ENV_CACHE[key]
    model = arm_spec().compile()
    idx = ArmIndex.of(model, prefix="")
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    axis = data.xpos[model.body("shoulder").id].copy()        # the pan axis passes through the shoulder
    fwd = _forward(model, idx, data)
    r = np.arange(0.0, r_max_mm + 1e-9, step_mm)
    z = np.arange(z_lo_mm, z_hi_mm + 1e-9, step_mm)
    ok = np.zeros((len(z), len(r)), dtype=bool)
    for yaw in (0.0, math.pi / 2):
        warm = None
        for i, zi in enumerate(z):
            warm_row = warm
            for j, rj in enumerate(r):
                t = np.array([axis[0] + fwd[0] * rj * MM, axis[1] + fwd[1] * rj * MM, zi * MM])
                res = ik_top_down(model, idx, t, math.atan2(fwd[1], fwd[0]) + yaw, q0=warm_row, data=data, iters=60)
                if not res.ok and warm_row is not None:
                    res = ik_top_down(model, idx, t, math.atan2(fwd[1], fwd[0]) + yaw, data=data, iters=120)
                ok[i, j] = (ok[i, j] if yaw else True) and res.ok
                if res.ok:
                    warm_row = res.q
                if j == 0:
                    warm = res.q if res.ok else warm
    out = {"r_mm": r, "z_mm": z, "reachable": ok, "pan_axis_mm": axis / MM, "pan_range": (idx.lo[0], idx.hi[0])}
    _ENV_CACHE[key] = out
    return out


def reachable(env, base_mm, base_yaw, point_mm):
    """Whether the arm, base at ``base_mm`` turned by ``base_yaw``, reaches ``point_mm`` top-down."""
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    ax = env["pan_axis_mm"]
    axis = np.array([base_mm[0] + c * ax[0] - s * ax[1], base_mm[1] + s * ax[0] + c * ax[1]])
    d = np.asarray(point_mm[:2]) - axis
    az = math.atan2(d[1], d[0]) - base_yaw
    az = (az + math.pi) % (2 * math.pi) - math.pi
    lo, hi = env["pan_range"]
    if not lo + PAN_MARGIN <= az <= hi - PAN_MARGIN:
        return False
    rr, zz = float(np.hypot(*d)), point_mm[2] - base_mm[2]
    step = env["r_mm"][1] - env["r_mm"][0]
    j, i = int(round(rr / step)), int(round((zz - env["z_mm"][0]) / step))
    if not (0 <= i < len(env["z_mm"]) and 0 <= j < len(env["r_mm"])):
        return False
    # the cell and its neighbours along r and z, so a point on a cell edge is not taken on trust
    sl = env["reachable"][max(i - 1, 0):i + 2, max(j - 1, 0):j + 2]
    return bool(sl.all())


def grasp_points_mm(family, p, placements):
    """Nominal grasp point of each placed brick (world, mm): its bounding box's centre."""
    out = []
    for b in placements:
        part = family.part_for(b.course, b.direction)
        lo, hi = B.sim_bodies(family, p, part).mesh.bounds
        c = (lo + hi) / 2
        pos, quat = family.seat_pose(p, b)
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, np.asarray(quat, dtype=float))
        out.append(np.asarray(pos, dtype=float) + R.reshape(3, 3) @ c)
    return np.array(out)


def _footprint_mm(family, p, structure):
    """Plan points the arm's base must keep clear of: each brick's and base's mesh corners."""
    pts = []
    for b in structure.bricks:
        part = family.part_for(b.course, b.direction)
        lo, hi = B.sim_bodies(family, p, part).mesh.bounds
        pos, quat = family.seat_pose(p, b)
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, np.asarray(quat, dtype=float))
        for cx in (lo[0], hi[0]):
            for cy in (lo[1], hi[1]):
                pts.append((np.asarray(pos) + R.reshape(3, 3) @ np.array([cx, cy, 0.0]))[:2])
    U = family.lattice(p).U_mm
    for base in structure.bases:
        v0, v1 = base[0], base[1]
        pieces, _ = B.base(family, p, v1 - v0, direction=base[2] if len(base) > 2 else 1)
        for piece in pieces:
            pts += list(np.asarray(piece)[:, :2] + np.array([v0 * U, 0.0]))
    return np.array(pts)


def floor_mm(family, p, structure):
    """The floor's height (mm), as ``scene.build_scene`` sets it: the lowest base, or 0."""
    zs = [0.0]
    for base in structure.bases:
        zs.append(float(B.base(family, p, base[1] - base[0], direction=base[2] if len(base) > 2 else 1)[1]))
    return min(zs)


RISERS_MM = tuple(range(0, 151, 10))   # heights of the block the arm's base may stand on


def plan_bases(family, p, structure, targets_mm, env, grid_mm=10.0, risers_mm=RISERS_MM, yaws=None):
    """Base poses (x, y, z mm, yaw rad, riser mm) from which every target is reached top-down,
    best first: the lowest riser, then the nearest the targets' centroid.

    The base stays ``BASE_CLEAR_MM`` (in plan) from every brick and base, and may stand on a riser
    (a block under it): a straight-down pose tops out under 100 mm above the base, because the wrist
    runs out of range, so a tall structure needs the base raised. The envelope screens candidates;
    with ``yaws`` (the jaw yaw at each target, rad) each survivor is then checked by exact IK.
    """
    foot = _footprint_mm(family, p, structure)
    z0 = floor_mm(family, p, structure)
    cen = targets_mm[:, :2].mean(axis=0)
    xs = np.arange(foot[:, 0].min() - 300, foot[:, 0].max() + 300 + 1e-9, grid_mm)
    ys = np.arange(foot[:, 1].min() - 350, foot[:, 1].max() + 350 + 1e-9, grid_mm)
    cands = [(x, y) for x in xs for y in ys if np.min(np.hypot(foot[:, 0] - x, foot[:, 1] - y)) >= BASE_CLEAR_MM]
    cands.sort(key=lambda q: math.hypot(q[0] - cen[0], q[1] - cen[1]))
    for riser in risers_mm:
        for x, y in cands:
            yaw = math.atan2(cen[1] - y, cen[0] - x)
            base = (x, y, z0 + riser)
            if not all(reachable(env, base, yaw, t) for t in targets_mm):
                continue
            if yaws is not None and not all(ik_from_base(base, yaw, t, ty).ok for t, ty in zip(targets_mm, yaws)):
                continue
            yield (float(x), float(y), float(z0 + riser), float(yaw), float(riser))


def plan_base(family, p, structure, targets_mm, env, **kw):
    """The best of ``plan_bases``, or None."""
    return next(plan_bases(family, p, structure, targets_mm, env, **kw), None)


_BARE = {}


def ik_from_base(base_mm, base_yaw, target_mm, jaw_yaw):
    """Exact top-down IK for an arm whose base is at ``base_mm`` turned by ``base_yaw``: the bare
    model, with the target moved into its frame."""
    if "m" not in _BARE:
        _BARE["m"] = arm_spec().compile()
        _BARE["i"] = ArmIndex.of(_BARE["m"], prefix="")
        _BARE["d"] = mujoco.MjData(_BARE["m"])
    c, s_ = math.cos(-base_yaw), math.sin(-base_yaw)
    d = (np.asarray(target_mm, float) - np.asarray(base_mm, float)) * MM
    local = np.array([c * d[0] - s_ * d[1], s_ * d[0] + c * d[1], d[2]])
    return ik_top_down(_BARE["m"], _BARE["i"], local, jaw_yaw - base_yaw, data=_BARE["d"], wide=True)


def max_overhang_courses(family, env, p=None, up_to=12):
    """The longest +x overhang (``structures.overhang``) whose every brick the arm reaches from one
    base, and that base. Nominal grasp points (see the module doc)."""
    p = family.defaults if p is None else p
    span = family.lattice(p).span_voxels
    best = {"courses": 0, "base_mm": None, "yaw": None, "riser_mm": None}
    for n in range(1, up_to + 1):
        st = S.overhang(n, +1, span)
        plan = plan_base(family, p, st, grasp_points_mm(family, p, st.bricks), env)
        if plan is None:
            break
        best = {"courses": n, "base_mm": plan[:3], "yaw": plan[3], "riser_mm": plan[4]}
    return best


def max_bridge_courses(family, env, p=None, up_to=3):
    """The largest ``structures.bridge(k)``, k <= ``up_to``, the arm reaches from one base."""
    p = family.defaults if p is None else p
    span = family.lattice(p).span_voxels
    for k in range(up_to, 0, -1):
        st = S.bridge(k, span)
        plan = plan_base(family, p, st, grasp_points_mm(family, p, st.bricks), env)
        if plan is not None:
            return {"arm_courses": k, "base_mm": plan[:3], "yaw": plan[3], "riser_mm": plan[4]}
    return {"arm_courses": 0, "base_mm": None, "yaw": None, "riser_mm": None}
