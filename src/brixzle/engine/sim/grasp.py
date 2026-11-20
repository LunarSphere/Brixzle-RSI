"""Grasping a brick with the SO-101's stock jaws (#153), and the pick station it is taken from.

**Jaws.** The fixed jaw and the moving jaw close along the gripper body's x axis; the moving jaw
swings on a hinge, so its tip rises as it opens. Everything here is measured from the model.

**Grasp.** The arm grasps top-down, closing across the brick's depth (brick Y) on its two outermost
side faces: the brick's widest part, so the fingers come down beside it without touching anything
else. The jaws never touch a guide face, a hook or a slot: those are on the top and bottom. Along the
brick, the grasp sits where the side faces offer the fingertips the most contact, near the centre of
mass so the brick does not pitch in the jaws. (allround's widest part is its wing tips, whose side
faces are 2 mm tall; its body's sides are under the wings, out of reach from above, and during a
placement the course below's wings fill the space under them.)

**Pick station.** A mould of the brick's underside, as a MuJoCo heightfield: the brick rests in it at
an exact pose, located in X, Y and yaw, and lifts straight out. All three designs seat with a
slanted stroke, so a brick seated on an ordinary base is hooked and cannot be lifted straight up; the
mould has no undercuts.
"""
from dataclasses import dataclass
import math

import mujoco
import numpy as np
import trimesh

from brixzle.engine.sim import arm as A
from brixzle.families import builders as B

MM = 1e-3
FIXED_PAD = "fixed_jaw_box4"      # the fixed jaw's fingertip pad (a thin box on the jaw's inner face)
FIXED_TIPS = ("fixed_jaw_sph_tip1", "fixed_jaw_sph_tip2", "fixed_jaw_sph_tip3")  # spheres at its tip, proud of the pad
MOVING_TIP = "moving_jaw_sph_tip1"
WINDOW_X_MM = 6.0                 # half-length along the brick of the fingertip contact window
WINDOW_Z_MM = 3.0                 # half-height of the window
CRADLE_STEP_MM = 1.0
CRADLE_GAP_MM = 0.2


# --- the jaws ---


@dataclass(frozen=True)
class Jaws:
    fixed_x_mm: float     # the fixed pad's inner face, gripper frame x
    pad_z_mm: float       # the fixed pad's centre height, gripper frame z (below the wrist: negative)
    pad_half_y_mm: float  # half-width of the fingers along gripper y (the brick's X)
    tip_below_mm: float   # how far the fingertips reach below the fixed pad's centre
    q: np.ndarray         # gripper joint angles sampled
    opening_mm: np.ndarray  # tip-to-pad opening at each angle
    rise_mm: np.ndarray     # how far the moving tip sits above the fixed pad at each angle

    def angle_for(self, opening_mm):
        """The gripper angle at which the jaws are ``opening_mm`` apart (opening grows with angle)."""
        return float(np.interp(opening_mm, self.opening_mm, self.q))

    def rise_at(self, opening_mm):
        return float(np.interp(opening_mm, self.opening_mm, self.rise_mm))


_JAWS = None


def jaws():
    """The stock jaws, measured from the bare arm model (memoised)."""
    global _JAWS
    if _JAWS is not None:
        return _JAWS
    m = A.arm_spec().compile()
    idx = A.ArmIndex.of(m, prefix="")
    d = mujoco.MjData(m)
    g = m.body("gripper").id
    pad = m.geom(FIXED_PAD)
    # the fixed jaw's innermost surface: its tip spheres stand 0.65 mm proud of the pad and touch first
    fixed_x = max([pad.pos[0] + pad.size[0]] + [m.geom(t).pos[0] + m.geom(t).size[0] for t in FIXED_TIPS]) / MM
    mujoco.mj_kinematics(m, d)
    fixed_geoms = [i for i in range(m.ngeom) if m.geom_bodyid[i] == g and m.geom_priority[i] == 1]
    lowest = min(m.geom_pos[i][2] - (m.geom_rbound[i] if m.geom_type[i] != mujoco.mjtGeom.mjGEOM_BOX else m.geom_size[i][2])
                 for i in fixed_geoms if m.geom_type[i] != mujoco.mjtGeom.mjGEOM_MESH)
    tip = m.geom(MOVING_TIP)
    qs = np.linspace(0.0, idx.hi[5], 120)
    opening, rise = [], []
    for q in qs:
        d.qpos[idx.qpos[5]] = q
        mujoco.mj_kinematics(m, d)
        R, o = d.xmat[g].reshape(3, 3), d.xpos[g]
        local = R.T @ (d.geom_xpos[tip.id] - o) / MM
        opening.append(local[0] - tip.size[0] / MM - fixed_x)
        rise.append(local[2] - pad.pos[2] / MM)
    opening, rise = np.array(opening), np.array(rise)
    keep = np.concatenate([[True], np.diff(opening) > 0])          # the monotonic part of the opening
    cut = int(np.argmin(keep)) if not keep.all() else len(qs)
    _JAWS = Jaws(fixed_x_mm=float(fixed_x), pad_z_mm=float(pad.pos[2] / MM), pad_half_y_mm=float(pad.size[1] / MM),
                 tip_below_mm=float((pad.pos[2] - lowest) / MM),
                 q=qs[:cut], opening_mm=opening[:cut], rise_mm=rise[:cut])
    return _JAWS


# --- the grasp ---


@dataclass(frozen=True)
class Grasp:
    """Where the jaws hold a brick, in the brick's frame (mm). The fixed jaw is on the -Y face."""
    x_mm: float          # along the brick
    z_mm: float          # height of the fixed pad's centre on the face
    width_mm: float      # distance between the two gripped faces
    coverage: float      # fraction of the fingertip window on each face that is brick (the lesser side)
    open_mm: float       # jaw opening on approach
    squeeze_mm: float    # jaw opening commanded to squeeze (under the width: the servo's torque limit sets the force)

    def gripper_frame(self, jaw=None):
        """(R, o): the gripper body's axes and origin in the brick's frame (mm). Gripper z is the
        brick's +z (straight down onto it), gripper x the brick's +Y (fixed jaw on the -Y face)."""
        jaw = jaws() if jaw is None else jaw
        gx, gz = np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])
        R = np.column_stack([gx, np.cross(gz, gx), gz])
        o = np.array([self.x_mm, -self.width_mm / 2 - jaw.fixed_x_mm, self.z_mm - jaw.pad_z_mm])
        return R, o


def _face_grid(mesh, y, xs, zs):
    """Boolean grid (z, x): whether the point (x, y, z) is inside the brick."""
    X, Z = np.meshgrid(xs, zs)
    pts = np.column_stack([X.ravel(), np.full(X.size, y), Z.ravel()])
    return mesh.contains(pts).reshape(X.shape)


def _window_mean(grid, hx, hz):
    """Mean of ``grid`` over a (2 hz + 1) x (2 hx + 1) window at each cell (zero outside)."""
    c = np.pad(grid.astype(float), ((hz + 1, hz), (hx + 1, hx))).cumsum(0).cumsum(1)
    s = c[2 * hz + 1:, 2 * hx + 1:] - c[:-2 * hz - 1, 2 * hx + 1:] - c[2 * hz + 1:, :-2 * hx - 1] + c[:-2 * hz - 1, :-2 * hx - 1]
    return s / ((2 * hx + 1) * (2 * hz + 1))


COM_X_MM = (2.0, 4.0, 8.0, 16.0)   # how far along the brick from the centre of mass the grasp may sit, tried in turn
ABOVE_COM_MM = 2.0                 # the fingertips grip at least this far above the centre of mass
TIP_CLEAR_MM = 0.5                 # the fingertips' clearance over the course underneath


def choose_grasp(family, p, part, step_mm=0.5, margin_mm=8.0, squeeze_mm=4.0, min_coverage=0.15):
    """The grasp with the most fingertip contact on the brick's two outermost side faces, held
    above the centre of mass and near it along the brick.

    A pinch on two small pads barely resists the brick pitching about the line between them, so
    the brick hangs from that line like a pendulum: it is steady only if its centre of mass is below
    the line and close to straight below it. So the window must sit ``ABOVE_COM_MM`` over the centre
    of mass and within ``COM_X_MM`` of it along the brick (the tightest bound that leaves a window
    with ``min_coverage``); among those, the most coverage, then the nearest the centre of mass."""
    sb = B.sim_bodies(family, p, part)
    mesh = sb.mesh
    lo, hi = mesh.bounds
    half = min(-lo[1], hi[1])
    com_x = float(sb.com_mm[0])
    jaw = jaws()
    xs = np.arange(lo[0], hi[0] + 1e-9, step_mm)
    zs = np.arange(lo[2], hi[2] + 1e-9, step_mm)
    hx, hz = int(round(WINDOW_X_MM / step_mm)), int(round(WINDOW_Z_MM / step_mm))
    cov = np.minimum(_window_mean(_face_grid(mesh, -half + 0.2, xs, zs), hx, hz),
                     _window_mean(_face_grid(mesh, half - 0.2, xs, zs), hx, hz))
    cov[:, :hx + 2], cov[:, -hx - 2:] = 0.0, 0.0          # keep the window on the brick's length
    com_z = float(sb.com_mm[2])
    cov[zs < com_z + ABOVE_COM_MM, :] = 0.0
    # The fingertips must end above the course underneath: its side faces are in the same planes,
    # one course pitch (H0) lower and, the profile repeating every voxel, at the same height along
    # the brick. So over the fingers' width the tips stay TIP_CLEAR_MM over (face top - H0).
    left, right = _face_grid(mesh, -half + 0.2, xs, zs), _face_grid(mesh, half - 0.2, xs, zs)
    has = left | right
    top_z = np.where(has.any(axis=0), zs[len(zs) - 1 - np.argmax(has[::-1], axis=0)], -np.inf)
    hf = int(round(max(jaw.pad_half_y_mm, WINDOW_X_MM) / step_mm))
    pad_t = np.pad(top_z, hf, constant_values=-np.inf)
    below = np.array([pad_t[i:i + 2 * hf + 1].max() for i in range(len(xs))]) - family.lattice(p).H0_mm
    cov[zs[:, None] - jaw.tip_below_mm < below[None, :] + TIP_CLEAR_MM] = 0.0
    for bound in COM_X_MM:
        c = np.where(np.abs(xs - com_x)[None, :] <= bound, cov, 0.0)
        if c.max() >= min_coverage:
            break
    else:
        raise ValueError(f"{family.name}: no side-face window above the centre of mass for the fingertips")
    top = float(c.max())
    i, j = min(zip(*np.nonzero(c >= 0.95 * top)), key=lambda ij: (abs(xs[ij[1]] - com_x), zs[ij[0]]))
    cov = c
    width = 2 * half
    if width + margin_mm > jaw.opening_mm.max():
        raise ValueError(f"{family.name}: {width:.1f} mm is wider than the jaws open")
    return Grasp(x_mm=float(xs[j]), z_mm=float(zs[i]), width_mm=float(width), coverage=float(cov[i, j]),
                 open_mm=float(width + margin_mm), squeeze_mm=float(max(width - squeeze_mm, jaw.opening_mm.min())))


# --- the pick station ---


@dataclass(frozen=True)
class Cradle:
    """A heightfield mould of a brick's underside, in the brick's frame (mm)."""
    x: np.ndarray        # column x
    y: np.ndarray        # row y
    height: np.ndarray   # (rows, cols): the mould's top, brick frame z
    z_floor: float       # the brick-frame height that stands on the floor (the mould's lowest point)


def cradle(family, p, part, step_mm=CRADLE_STEP_MM, gap_mm=CRADLE_GAP_MM):
    """The mould: at each (x, y) the brick's lowest surface point, eroded over the neighbouring
    cells (so the sampled surface stays under the brick between samples) and lowered by ``gap_mm``.
    Where nothing is above, the mould is at its floor."""
    mesh = B.sim_bodies(family, p, part).mesh
    lo, hi = mesh.bounds
    xs = np.arange(lo[0], hi[0] + 1e-9, step_mm)
    ys = np.arange(lo[1], hi[1] + 1e-9, step_mm)
    X, Y = np.meshgrid(xs, ys)
    origins = np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, lo[2] - 10.0)])
    dirs = np.tile([0.0, 0.0, 1.0], (len(origins), 1))
    locs, ray, _ = mesh.ray.intersects_location(origins, dirs, multiple_hits=False)
    z = np.full(len(origins), np.nan)
    z[ray] = locs[:, 2]
    z = z.reshape(X.shape)
    z_floor = float(lo[2]) - 1.0
    z = np.where(np.isnan(z), z_floor, z)
    pad = np.pad(z, 1, mode="edge")
    er = np.min([pad[1 + dy:1 + dy + z.shape[0], 1 + dx:1 + dx + z.shape[1]] for dy in (-1, 0, 1) for dx in (-1, 0, 1)], axis=0)
    return Cradle(x=xs, y=ys, height=np.maximum(er - gap_mm, z_floor), z_floor=z_floor)


def add_cradle(asset, world, cr, at_mm, floor_z_mm, name="cradle"):
    """MJCF for the mould centred under a brick whose frame origin is at ``at_mm`` (x, y) on a
    floor at ``floor_z_mm``; returns the brick-frame origin height (mm) that seats the brick in it."""
    import xml.etree.ElementTree as ET
    rx, ry = (cr.x[-1] - cr.x[0]) / 2, (cr.y[-1] - cr.y[0]) / 2
    cx, cy = (cr.x[-1] + cr.x[0]) / 2, (cr.y[-1] + cr.y[0]) / 2
    h = cr.height - cr.z_floor                      # >= 0, above the floor
    top = max(float(h.max()), 1e-3)
    elev = " ".join(f"{v / top:.6f}" for v in h.ravel())
    ET.SubElement(asset, "hfield", name=name, nrow=str(h.shape[0]), ncol=str(h.shape[1]),
                  size=f"{rx * MM:.9g} {ry * MM:.9g} {top * MM:.9g} {0.002:.9g}", elevation=elev)
    ET.SubElement(world, "geom", name=name, type="hfield", hfield=name, rgba=".5 .45 .4 1",
                  pos=f"{(at_mm[0] + cx) * MM:.9g} {(at_mm[1] + cy) * MM:.9g} {floor_z_mm * MM:.9g}",
                  contype="1", conaffinity="1")
    return floor_z_mm - cr.z_floor
