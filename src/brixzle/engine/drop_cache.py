"""A cache of simulated drops (#163): a drop depends only on what the sim builds and on its pose, so
one already run is read back instead of run again.

**The key** of a drop is the digest of:

- **the sim geometry** (``geometry_key``): each part's collision pieces, mass, centre of mass and
  inertia (``SimBodies``, not the printed mesh, which the physics never sees); the lattice; the
  release lift; the seat pose, insertion axis and part of every brick in supported drop rigs; and the
  anchored bases those rigs use. Not the family's name, version or parameter values: two designs
  that build the same scene share their drops, so a refactor (#162) or a parameter the sim never sees
  costs nothing;
- **the sim's code**: the source of ``engine/sim/``, ``families/builders.py`` and
  ``engine/box_search.py``, so a change to the physics or the judging invalidates every entry;
- **the trial config** (physics, error model, tolerance), and **the job**: course, pose and
  options (trace, rest, friction).

Recorded drops (videos) and pickup jobs are never cached.

**Storage:** one SQLite file (``DropCache``), safe for several processes, the row stored as
compressed JSON. JSON keeps every float exactly (``repr``), so a read row equals the simulated one.
``BRIXZLE_DROP_CACHE`` turns it on for every ``DropPool`` in the process (a path, or ``on`` for
``.cache/drops.sqlite``, git-ignored); unset, the loop and the tests simulate every drop. The tools of
#163 turn it on.

Rows read from the cache are simulated, uncalibrated contact like any other: they are earlier sim
results, not new ones.
"""
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import zlib

import numpy as np

from brixzle.families import builders as B
from brixzle.families import fingerprint as FP

ROOT = Path(__file__).resolve().parents[1]
CODE_FILES = sorted((ROOT / "engine" / "sim").glob("*.py")) + [ROOT / "families" / "builders.py",
                                                                ROOT / "engine" / "box_search.py"]
DEFAULT_PATH = Path(__file__).resolve().parents[3] / ".cache" / "drops.sqlite"
_CODE = None


def code_version():
    """Digest of the source the drop's result depends on."""
    global _CODE
    if _CODE is None:
        h = hashlib.sha256()
        for f in CODE_FILES:
            h.update(f.name.encode() + b"\0" + f.read_bytes())
        _CODE = h.hexdigest()
    return _CODE


def _bodies_repr(b):
    return {"collision": [np.asarray(v, float) for v in b.collision], "soft": [np.asarray(v, float) for v in b.soft_collision],
            "source": b.collision_source, "soft_contact": dict(sorted((b.soft_contact or {}).items())), "mass_g": float(b.mass_g), "volume_mm3": float(b.volume_mm3),
            "com_mm": np.asarray(b.com_mm, float), "inertia": np.asarray(b.inertia_g_mm2, float)}


def geometry_key(family, p, bodies=None):
    """Digest of everything the drop rigs build from ``family`` at ``p`` (see the module docstring)."""
    from brixzle.engine.sim import trials as T

    bodies = bodies or {k: B.sim_bodies(family, p, k) for k in family.parts}
    lat = family.lattice(p)
    rigs, bases = [], set()
    layouts = [(course, T.rig_layout(lat, course)) for course in (0, 1)]
    if lat.span_voxels == 2:
        layouts.extend((case, T.edge_layout(lat, case)) for case in T.EDGE_CASES)
    for course, (placed, target, base) in layouts:
        rigs.append([course, base])
        for b in (*placed, target):
            pos, quat = family.seat_pose(p, b)
            rigs.append([course, b.course, b.i0, b.direction, family.part_for(b.course, b.direction),
                         np.asarray(pos, float), np.asarray(quat, float), np.asarray(family.insertion_axis(p, b), float)])
        bases.add((base[1] - base[0], base[2] if len(base) > 2 else 1))
    base_pieces = {f"{n}:{d}": [np.asarray(v, float) for v in B.base(family, p, n, d)[0]] for n, d in sorted(bases)}
    return FP.digest({
        "bodies": {k: _bodies_repr(v) for k, v in sorted(bodies.items())},
        "lattice": lat, "release_lift_mm": float(family.release_lift_mm(p)),
        "rigs": rigs, "bases": base_pieces,
    })


def cacheable(job):
    course, *rest = job
    opts = rest[1] if len(rest) > 1 else {}
    return course != "pick" and not opts.get("record")


def job_key(geometry, cfg, job):
    from brixzle.engine.sim import trials as T

    course, sample, *rest = job
    opts = rest[0] if rest else {}
    text = json.dumps({"code": code_version(), "geometry": geometry, "cfg": T.as_dict(cfg), "course": course,
                       "sample": sample, "opts": opts}, sort_keys=True, default=float)
    return hashlib.sha256(text.encode()).hexdigest()


class DropCache:
    """Drops by key, in one SQLite file."""

    def __init__(self, path=DEFAULT_PATH):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self._connect() as db:
            db.execute("CREATE TABLE IF NOT EXISTS drops (key TEXT PRIMARY KEY, row BLOB NOT NULL)")

    def _connect(self):
        db = sqlite3.connect(self.path, timeout=60)
        db.execute("PRAGMA journal_mode=WAL")
        return db

    def get_many(self, keys):
        """{key: row} of the keys that are stored."""
        out = {}
        keys = list(dict.fromkeys(keys))
        with self._connect() as db:
            for i in range(0, len(keys), 500):
                chunk = keys[i:i + 500]
                q = f"SELECT key, row FROM drops WHERE key IN ({','.join('?' * len(chunk))})"
                for k, blob in db.execute(q, chunk):
                    out[k] = json.loads(zlib.decompress(blob))
        return out

    def put_many(self, items):
        rows = [(k, zlib.compress(json.dumps(r, allow_nan=True).encode())) for k, r in items]
        if rows:
            with self._connect() as db:
                db.executemany("INSERT OR REPLACE INTO drops (key, row) VALUES (?, ?)", rows)

    def __len__(self):
        with self._connect() as db:
            return db.execute("SELECT COUNT(*) FROM drops").fetchone()[0]

    def __getstate__(self):
        return {"path": self.path}

    def __setstate__(self, state):
        self.path = state["path"]


def default():
    """The cache ``BRIXZLE_DROP_CACHE`` turns on: a path, or ``on`` for ``.cache/drops.sqlite``. Unset or
    ``off``: None, so the loop and the tests simulate every drop unless a tool turns it on."""
    where = os.environ.get("BRIXZLE_DROP_CACHE", "").strip()
    if not where or where.lower() == "off":
        return None
    return DropCache(DEFAULT_PATH if where.lower() == "on" else where)


def _plain(row):
    """A row as JSON gives it back (tuples as lists, numpy as Python), so cached and fresh rows compare equal."""
    return json.loads(json.dumps(row, default=lambda v: v.tolist() if hasattr(v, "tolist") else float(v)))

