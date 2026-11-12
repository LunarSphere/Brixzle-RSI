"""Contract checks every brick family must pass before the engine will score it.

``check_family(family)`` returns a list of Violations (empty means the family conforms). It runs on
the defaults and on seeded random points inside the bounds, because the tuner will visit those.
At a sampled point a family may reject infeasible parameters, but only by raising ValueError;
any other exception, and any exception at the defaults, is a violation. The design agent (#18)
calls this as a tool, so messages say what to fix.
"""

from dataclasses import dataclass, fields, replace
import math
import re

import mujoco
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from shapely.geometry import Point, Polygon

from brixzle.families import builders as B
from brixzle.families import rules as R
from brixzle.families.interface import BORE, Family, GripFeature, Placement, Prismatic
from brixzle.requirements import load_requirements

TOUCH_MM = 0.5         # half the 1 mm seat tolerance: closer than this counts as touching, not overlap
VOLUME_REL = 0.01      # Prismatic straight extrusion: solid volume vs section area x depth
CHECKS = ("params", "section", "solid", "sim_bodies", "lattice", "rules", "pickup")


@dataclass(frozen=True)
class Violation:
    check: str    # one of CHECKS
    where: str    # 'family', 'defaults', 'sample 2', 'defaults, part A', ...
    message: str


class _Fail(Exception):
    pass


def _require(ok, message):
    if not ok:
        raise _Fail(message)


def check_family(family: Family, samples=3, seed=0) -> list[Violation]:
    out = _check_params(family)
    try:
        defaults = family.defaults
    except Exception as e:  # noqa: BLE001 - reported, not raised
        return out + [Violation("params", "defaults", f"defaults cannot be built: {type(e).__name__}: {e}")]
    out += _check_point(family, defaults, "defaults", sampled=False)
    for i, values in enumerate(_sample_values(family, samples, seed)):
        where = f"sample {i} {values}"
        try:
            p = replace(defaults, **values)
        except ValueError:
            continue  # infeasible point, rejected properly
        except Exception as e:  # noqa: BLE001
            out.append(Violation("params", where, f"{type(e).__name__}: {e}"))
            continue
        out += _check_point(family, p, where, sampled=True)
    return out


def _sample_values(family, samples, seed):
    names = {f.name for f in fields(family.params_type)}
    usable = {k: v for k, v in family.bounds.items()
              if k in names and len(v) == 2 and all(math.isfinite(x) for x in v) and v[0] < v[1]}
    rng = np.random.default_rng(seed)
    return [{k: float(rng.uniform(lo, hi)) for k, (lo, hi) in sorted(usable.items())}
            for _ in range(samples)]


def _check_params(family):
    msgs = []
    names = {f.name for f in fields(family.params_type)}
    for k, b in family.bounds.items():
        if k not in names:
            msgs.append(f"bounds name an unknown parameter {k!r}")
        elif len(b) != 2 or not all(math.isfinite(x) for x in b) or not b[0] < b[1]:
            msgs.append(f"bounds for {k!r} must be finite (lo, hi) with lo < hi, got {b}")
    try:
        p = family.defaults
        for f in fields(p):
            v = getattr(p, f.name)
            if isinstance(v, float) and not math.isfinite(v):
                msgs.append(f"default {f.name} is not finite")
        for k, b in family.bounds.items():
            if k in names and len(b) == 2 and not b[0] <= getattr(p, k) <= b[1]:
                msgs.append(f"default {k} = {getattr(p, k)} is outside its bounds {b}")
        if family.params_type.from_dict(p.to_dict()) != p:
            msgs.append("to_dict/from_dict does not round-trip the defaults")
        try:
            family.params_type.from_dict({**p.to_dict(), "__not_a_param__": 1.0})
            msgs.append("from_dict accepts unknown keys")
        except ValueError:
            pass
    except Exception as e:  # noqa: BLE001
        msgs.append(f"{type(e).__name__}: {e}")
    return [Violation("params", "family", m) for m in msgs]


def _check_point(family, p, where, sampled):
    """Run every check at one parameter point; a ValueError at a sampled point skips the point."""
    out = []
    steps = [(check, part, fn) for part in family.parts
             for check, fn in (("section", _check_section), ("solid", _check_solid),
                               ("sim_bodies", _check_sim_bodies), ("rules", _check_rules),
                               ("pickup", _check_pickup))]
    steps.append(("lattice", None, _check_lattice))
    for check, part, fn in steps:
        here = where if part is None else f"{where}, part {part}"
        try:
            fn(family, p, part)
        except _Fail as e:
            out.append(Violation(check, here, str(e)))
        except ValueError as e:
            if sampled:
                return []  # the family rejected this point as infeasible
            out.append(Violation(check, here, f"ValueError at the defaults: {e}"))
        except Exception as e:  # noqa: BLE001
            out.append(Violation(check, here, f"{type(e).__name__}: {e}"))
    return out


def _check_section(family, p, part):
    s = B.section(family, p, part)
    _require(isinstance(s.outer, Polygon), "section.outer must be a shapely Polygon")
    _require(s.outer.is_valid, "section.outer is not a valid polygon (self-intersecting?)")
    _require(s.outer.area > 0, "section.outer has no area")
    for i, h in enumerate(s.holes):
        _require(h.is_valid and h.area > 0, f"section hole {i} is not a valid polygon with area")
        _require(s.outer.contains(h), f"section hole {i} is not inside the outline")
    for i, g in enumerate(s.guide_faces):
        pts = [Point(g.a), Point(g.b), Point((np.add(g.a, g.b) / 2).tolist())]
        _require(all(s.outer.exterior.distance(q) < 1e-3 for q in pts),
                 f"guide face {i} {g.a}->{g.b} does not lie on the outline")


def _check_solid(family, p, part):
    shape = B.solid(family, p, part, bored=True)
    _require(shape.isValid(), "the CadQuery solid is not valid")
    mesh = B.solid_mesh(family, p, part, bored=True)
    _require(mesh.is_watertight, "the solid's mesh is not watertight")
    _require(len(mesh.split(only_watertight=False)) == 1, "the solid is more than one piece")
    g = family.geometry
    if isinstance(g, Prismatic):
        e = g.extrusion(p)
        if all(z0 == 0 and z1 == 0 for _, _, z0, z1 in e.resolved_bands()):
            s = g.section(p, part)
            expect = (s.outer.area - sum(h.area for h in s.holes)) * e.depth_mm
            got = B.solid_mesh(family, p, part, bored=False).volume
            _require(abs(got - expect) <= VOLUME_REL * expect,
                     f"solid volume {got:.1f} mm^3 differs from section area x depth {expect:.1f}")


def _numbers(values):
    return " ".join(f"{float(v):.9g}" for v in np.ravel(values))


def _compile(bodies):
    assets = "".join(f'<mesh name="c{i}" vertex="{_numbers(np.asarray(v) * 1e-3)}"/>'
                     for i, v in enumerate(bodies.collision))
    geoms = "".join(f'<geom type="mesh" mesh="c{i}"/>' for i in range(len(bodies.collision)))
    inertia = np.asarray(bodies.inertia_g_mm2) * 1e-9
    full = [inertia[0, 0], inertia[1, 1], inertia[2, 2], inertia[0, 1], inertia[0, 2], inertia[1, 2]]
    xml = (f'<mujoco><asset>{assets}</asset><worldbody><body name="brick"><freejoint/>'
           f'<inertial mass="{bodies.mass_g * 1e-3:.9g}" pos="{_numbers(np.asarray(bodies.com_mm) * 1e-3)}" '
           f'fullinertia="{_numbers(full)}"/>{geoms}</body></worldbody></mujoco>')
    mujoco.MjModel.from_xml_string(xml)


def _check_sim_bodies(family, p, part):
    bodies = B.sim_bodies(family, p, part)
    _require(len(bodies.collision) > 0, "no collision pieces")
    for i, piece in enumerate(bodies.collision):
        piece = np.asarray(piece)
        _require(piece.ndim == 2 and piece.shape[1] == 3 and B.math_finite(piece),
                 f"collision piece {i} must be a finite (n, 3) vertex array")
        _require(B.is_convex_vertex_set(piece),
                 f"collision piece {i} has vertices inside its hull; give convex vertex sets")
        _require(ConvexHull(piece).volume > 0, f"collision piece {i} has no volume")
    inside, outside = B.contact_points(family, p, part)
    # 1 um tolerance: decomposed pieces meet at seams, and a point on a seam belongs to both.
    missed = int((~B.inside_any(inside, bodies.collision, tol=1e-3)).sum())
    _require(missed == 0, f"{missed} of {len(inside)} points just inside the outline are not in the collision")
    phantom = int(B.inside_any(outside, bodies.collision, tol=-1e-3).sum())
    _require(phantom == 0, f"{phantom} of {len(outside)} points just outside the outline or off a guide face "
                           "are inside the collision (phantom material)")
    _require(B.math_finite(bodies.mass_g, bodies.com_mm, bodies.inertia_g_mm2),
             "mass, centre of mass and inertia must be finite")
    inertia = np.asarray(bodies.inertia_g_mm2)
    _require(np.allclose(inertia, inertia.T, rtol=1e-6, atol=1e-9) and np.all(np.linalg.eigvalsh(inertia) > 0),
             "inertia must be symmetric positive-definite")
    try:
        _compile(bodies)
    except ValueError as e:  # MuJoCo reports model errors as ValueError
        raise _Fail(f"MuJoCo cannot compile the sim bodies: {e}") from e


def _placed(family, p, placement):
    pos, quat = family.seat_pose(p, placement)
    w, x, y, z = quat
    rot = Rotation.from_quat([x, y, z, w])
    pieces, _ = B.collision(family, p, family.part_for(placement.course, placement.direction))
    return [rot.apply(np.asarray(v)) + np.asarray(pos) for v in pieces]


def _overlap(a, b):
    """Key points of a (each corner of each piece, a quarter of the way to its centre) inside b,
    with b shrunk by TOUCH_MM."""
    pts = np.vstack([v + 0.25 * (v.mean(axis=0) - v) for v in a])
    return int(B.inside_any(pts, b, tol=-TOUCH_MM).sum())


def _check_lattice(family, p, part=None):
    lat = family.lattice(p)
    _require(B.math_finite(lat.U_mm, lat.D_mm, lat.H0_mm) and min(lat.U_mm, lat.D_mm, lat.H0_mm) > 0,
             "lattice pitches must be finite and > 0")
    _require(isinstance(lat.span_voxels, int) and lat.span_voxels >= 1, "span_voxels must be an int >= 1")
    lift = family.release_lift_mm(p)
    _require(B.math_finite(lift) and lift > 0, "release_lift_mm must be finite and > 0")
    for course in (0, 1):
        for direction in (1, -1):
            pl = Placement(course, 0, direction)
            name = family.part_for(course, direction)
            _require(name in family.parts, f"part_for{(course, direction)} = {name!r} is not in parts")
            pos, quat = family.seat_pose(p, pl)
            _require(np.shape(pos) == (3,) and B.math_finite(pos), f"seat_pose({pl}) position must be finite (3,)")
            _require(np.shape(quat) == (4,) and abs(np.linalg.norm(quat) - 1) < 1e-9,
                     f"seat_pose({pl}) quaternion must be unit [w, x, y, z], got {list(quat)}")
            axis = np.asarray(family.insertion_axis(p, pl), dtype=float)
            _require(axis.shape == (2,) and abs(np.linalg.norm(axis) - 1) < 1e-9,
                     f"insertion_axis({pl}) must be a unit (x, z) vector, got {axis.tolist()}")
    lower = _placed(family, p, Placement(0, 0))
    pairs = {"the brick above": Placement(1, 0), "its neighbour in the course": Placement(0, lat.span_voxels)}
    for label, pl in pairs.items():
        upper = _placed(family, p, pl)
        n = _overlap(lower, upper) + _overlap(upper, lower)
        _require(n == 0, f"a seated brick overlaps {label} ({n} key points inside it)")
    base = B.base_pieces(family, p, 2 * lat.span_voxels, 1)
    for i, piece in enumerate(base):
        _require(B.math_finite(piece) and ConvexHull(piece).volume > 0, f"base piece {i} is degenerate")
    n = _overlap(lower, base) + _overlap(base, lower)
    _require(n == 0, f"a course-0 brick overlaps the base ({n} key points inside it)")


_RULE_KEY = re.compile(r"^(R\d+)\.[a-z0-9_]+$")


def _check_rules(family, p, part):
    ids = {r.id for r in load_requirements()}
    margins = R.margins(family, p, part, B.sim_bodies(family, p, part))
    for key, value in margins.items():
        m = _RULE_KEY.match(key)
        _require(m is not None, f"margin key {key!r} must look like 'R6.guide_slides'")
        _require(m.group(1) in ids, f"margin {key!r} names {m.group(1)}, which is not in docs/requirements.yaml")
        _require(isinstance(value, (int, float)) and math.isfinite(value), f"margin {key!r} = {value!r} is not finite")


def _check_pickup(family, p, part):
    """R10 grippable (issue #50): at least one grip feature, each one on the brick. No gripper is
    assumed, so nothing here checks a fork; see rules.fork_margins and the pickup predictor."""
    feats = family.pickup(p, part)
    _require(isinstance(feats, (tuple, list)), "pickup must return a tuple of GripFeature")
    _require(len(feats) >= 1, "R10: the brick needs at least one grip feature a passive gripper could hold "
                              "(a bore, slot, lip or opening); pickup returned none")
    s = B.section(family, p, part)
    x0, z0, x1, z1 = s.outer.bounds
    for i, f in enumerate(feats):
        _require(isinstance(f, GripFeature), f"grip feature {i} must be a GripFeature, got {type(f).__name__}")
        name = f"grip feature {i} ({f.kind!r})"
        _require(isinstance(f.kind, str) and f.kind.strip(), f"grip feature {i} needs a kind, such as 'bore' or 'lip'")
        _require(np.shape(f.size_mm) == (2,) and B.math_finite(f.x_mm, f.z_mm, f.size_mm) and min(f.size_mm) > 0,
                 f"{name} must have a finite position and a size (x, z) > 0")
        if f.kind == BORE:
            # The builders cut this one. Wall thickness around it is a margin (R13.bore_wall);
            # here only broken geometry fails.
            circle = Point(f.x_mm, f.z_mm).buffer(f.radius_mm)
            _require(s.outer.contains(circle),
                     f"{name} at ({f.x_mm:.2f}, {f.z_mm:.2f}) breaks out of the section outline")
            for j, h in enumerate(s.holes):
                _require(circle.disjoint(h), f"{name} breaks into hole {j}")
        elif f.outline is not None:
            o = f.outline
            _require(isinstance(o, Polygon) and o.is_valid and o.area > 0, f"{name}: outline must be a valid polygon")
            _require(s.outer.buffer(1e-6).contains(o), f"{name} breaks out of the section outline")
        else:
            # Only a position and a size: its box must sit within the brick's extent.
            hx, hz = f.size_mm[0] / 2, f.size_mm[1] / 2
            inside = (x0 - 1e-6 <= f.x_mm - hx and f.x_mm + hx <= x1 + 1e-6
                      and z0 - 1e-6 <= f.z_mm - hz and f.z_mm + hz <= z1 + 1e-6)
            _require(inside, f"{name} at ({f.x_mm:.2f}, {f.z_mm:.2f}) breaks out of the brick's extent "
                             f"(x {x0:.2f} to {x1:.2f}, z {z0:.2f} to {z1:.2f})")
