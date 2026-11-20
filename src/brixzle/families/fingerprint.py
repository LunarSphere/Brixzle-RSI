"""A family's geometry, reduced to exact per-item digests (issue #162).

Two families with equal fingerprints build the same sim scene for every structure the engine uses, so
under the same seed and poses they give the same map, gate and ring. This is how a port or a refactor
is checked to change nothing: every float is hashed by its exact bytes, not compared to a tolerance.

What is covered, at the given parameters:

- the parameter values, the bounds, the parts and the lattice;
- per part: the exact collision pieces, the 2D section (outline, holes, guide faces), the grip
  features, the release lift, and the printed solid's volume and bounding box;
- the compliant collision pieces and their contact settings, for mixed-material families;
- the seat pose and insertion axis of every brick in the gate's structures (tower, wall, bridge,
  overhang each way, corbel), and the anchored bases they use;
- the ring kit: its parts' collision pieces and solids, the plan of a 6 x 6, 2-course ring, its base;
- the family's own rule margins (``extra_rules``).

``fingerprint`` returns {item: sha256 hex}; ``differences`` names the items that differ.
"""

from dataclasses import asdict, fields, is_dataclass
import hashlib
import json

import numpy as np

from brixzle.families.interface import Family, Placement

RING_SIDE = 6
RING_COURSES = 2


class _Hasher:
    def __init__(self):
        self.h = hashlib.sha256()

    def add(self, x):
        h = self.h
        if x is None:
            h.update(b"N")
        elif isinstance(x, (bool, np.bool_)):
            h.update(b"b1" if x else b"b0")
        elif isinstance(x, (int, np.integer)):
            h.update(b"i" + str(int(x)).encode())
        elif isinstance(x, (float, np.floating)):
            h.update(b"f" + np.float64(x).tobytes())
        elif isinstance(x, str):
            h.update(b"s" + x.encode() + b"\0")
        elif isinstance(x, np.ndarray):
            a = np.ascontiguousarray(x, dtype=np.float64)
            h.update(b"a" + str(a.shape).encode() + a.tobytes())
        elif isinstance(x, dict):
            h.update(b"{")
            for k in sorted(x, key=str):
                self.add(str(k))
                self.add(x[k])
            h.update(b"}")
        elif isinstance(x, (list, tuple)):
            h.update(b"[" + str(len(x)).encode())
            for v in x:
                self.add(v)
            h.update(b"]")
        elif hasattr(x, "geom_type"):  # shapely
            self.add(x.geom_type)
            self.add(x.wkb_hex)
        elif is_dataclass(x) and not isinstance(x, type):
            self.add(type(x).__name__)
            self.add({f.name: getattr(x, f.name) for f in fields(x)})
        else:
            raise TypeError(f"cannot fingerprint {type(x).__name__}")
        return self

    def hex(self):
        return self.h.hexdigest()


def digest(x) -> str:
    return _Hasher().add(x).hex()


def _structures():
    from brixzle.engine import structures as S

    return {
        "tower": S.tower(3),
        "wall": S.wall(3, 4),
        "bridge": S.bridge(3),
        "overhang+": S.overhang(12, 1),
        "overhang-": S.overhang(12, -1),
        "corbel": S.corbel(12),
    }


def _bricks(structure):
    bricks = getattr(structure, "bricks", None)
    return list(bricks if bricks is not None else structure)


def _bases(structure):
    return list(getattr(structure, "bases", ()) or ())


def _solid_summary(shape):
    """Volume and bounding box of a CadQuery shape (Workplane or Shape)."""
    s = shape.val() if hasattr(shape, "val") else shape
    bb = s.BoundingBox()
    return [float(s.Volume()), [bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax]]


def fingerprint(family: Family, p=None, ring=True, solid=True) -> dict[str, str]:
    """{item: digest} of ``family`` at ``p`` (its defaults if None)."""
    p = family.defaults if p is None else p
    out = {
        "params": digest(asdict(p)),
        "bounds": digest({k: list(v) for k, v in family.bounds.items()}),
        "parts": digest(list(family.parts)),
        "geometry_type": digest(type(family.geometry).__name__),
        "lattice": digest(family.lattice(p)),
        "release_lift_mm": digest(float(family.release_lift_mm(p))),
    }
    g = family.geometry
    for part in family.parts:
        coll = g.collision(p, part) if getattr(g, "collision", None) else None
        out[f"collision:{part}"] = digest([np.asarray(c) for c in coll] if coll is not None else None)
        if family.soft is not None:
            pieces, contact = family.soft(p, part)
            if len(pieces):
                out[f"soft_collision:{part}"] = digest([np.asarray(c, float) for c in pieces])
                out[f"soft_contact:{part}"] = digest({key: contact[key] for key in ("timeconst", "dampratio", "friction", "solimp")
                                                      if key in contact})
        if getattr(g, "section", None):
            out[f"section:{part}"] = digest(g.section(p, part))
        if hasattr(g, "extrusion"):
            out[f"extrusion:{part}"] = digest(g.extrusion(p))
        out[f"pickup:{part}"] = digest(list(family.pickup(p, part)))
        if solid and hasattr(g, "solid"):
            out[f"solid:{part}"] = digest(_solid_summary(g.solid(p, part)))
        if family.extra_rules is not None:
            out[f"rules:{part}"] = digest(_rules(family, p, part))
    for name, st in _structures().items():
        poses = []
        for b in _bricks(st):
            pos, quat = family.seat_pose(p, b)
            poses.append([family.part_for(b.course, b.direction), np.asarray(pos, float), np.asarray(quat, float),
                          np.asarray(family.insertion_axis(p, b), float)])
        out[f"seats:{name}"] = digest(poses)
    for direction in (1, -1):
        out[f"base:{direction}"] = digest(_base_repr(family.base(p, 2, direction)))
    if ring and family.ring is not None:
        k = family.ring
        out["ring:parts"] = digest(list(k.parts))
        for part in k.parts:
            if k.collision is not None:
                out[f"ring:collision:{part}"] = digest([np.asarray(c) for c in k.collision(p, part)])
            if solid:
                out[f"ring:solid:{part}"] = digest(_solid_summary(k.solid(p, part)))
        out["ring:plan"] = digest(list(k.plan(p, RING_SIDE, RING_COURSES)))
        out["ring:base"] = digest([np.asarray(c) for c in k.base(p, RING_SIDE)])
    return out


def _rules(family, p, part):
    """The family's own rule margins. Most families' rules ignore the sim bodies; build them only
    for one that needs them."""
    try:
        r = family.extra_rules(p, part, None)
    except Exception:  # noqa: BLE001 - the rules read the bodies
        from brixzle.families import builders as B
        r = family.extra_rules(p, part, B.sim_bodies(family, p, part))
    return {k: float(v) for k, v in r.items()}


def _base_repr(b):
    if isinstance(b, (list, tuple)):
        return [np.asarray(x) if not hasattr(x, "geom_type") and not is_dataclass(x) else x for x in b]
    if hasattr(b, "val") or hasattr(b, "Volume"):
        return _solid_summary(b)
    return b


def differences(a: dict[str, str], b: dict[str, str]) -> list[str]:
    """The items whose digests differ, or that only one side has."""
    return sorted(k for k in set(a) | set(b) if a.get(k) != b.get(k))


def total(fp: dict[str, str]) -> str:
    """One digest for the whole fingerprint."""
    return hashlib.sha256(json.dumps(fp, sort_keys=True).encode()).hexdigest()
