"""Capture features that vary along Y, on a prismatic interlock (issue #105).

A ``Prismatic`` family can only shift its one XZ section in Z across Y (``Extrusion`` bands, as
A/B's V gutter). A funnel, a skirt that hangs beside the course below, a crown or a tapered seat
changes the section itself along Y, so it needs ``Solid3D``. Since #105 that is a normal choice
for capture features, with no reason to give (maintainer, 2026-11-13). The interlock may stay
prismatic.

``with_capture_features(section, extrusion, features)`` builds that ``Solid3D`` from three
module-level functions:

- ``section(p, part)`` and ``extrusion(p)``: the prismatic interlock, as for ``Prismatic``;
- ``features(p, part)``: a list of convex vertex sets (mm, brick frame) added to it, such as
  ``prism_x(yz_polygon, x0, x1)``, a convex Y-Z profile swept along X.

The collision is exact (the interlock's convex cut plus each feature's hull), so no convex
decomposition runs and the key-point check sees the real faces. The solid is the swept section
fused with each feature's hull. The predictor's section is the interlock's (it is only a guide
for a brick that varies along Y; the sim decides). ``core_extrusion(geometry)`` gives the
interlock's extrusion, so ``ring_cells.kit_from_cell`` can build a ring kit from a cell.
"""

from dataclasses import dataclass
from typing import Any, Callable

import cadquery as cq
import numpy as np
from scipy.spatial import ConvexHull

from brixzle.families.interface import Solid3D

PLANE_TOL = 1e-7


def hull_solid(vertices) -> cq.Solid:
    """The CadQuery solid of the convex hull of ``vertices`` (mm), one planar face per hull facet."""
    pts = np.asarray(vertices, dtype=float)
    hull = ConvexHull(pts)
    faces, seen = [], []
    for eq in hull.equations:
        if any(np.allclose(eq, s, atol=1e-6) for s in seen):
            continue
        seen.append(eq)
        on = pts[np.abs(pts @ eq[:3] + eq[3]) < 1e-6]
        on = np.unique(np.round(on, 9), axis=0)
        # Order the facet's vertices round its centre, counter-clockwise seen from outside.
        n = eq[:3]
        c = on.mean(axis=0)
        u = on[0] - c
        u /= np.linalg.norm(u)
        v = np.cross(n, u)
        ang = np.arctan2((on - c) @ v, (on - c) @ u)
        ring = on[np.argsort(ang)]
        faces.append(cq.Face.makeFromWires(cq.Wire.makePolygon([cq.Vector(*map(float, q)) for q in ring], close=True)))
    shell = cq.Shell.makeShell(faces)
    out = cq.Solid.makeSolid(shell)
    if out.Volume() < 0:
        raise ValueError("the hull solid came out inside-out")
    return out


def prism_x(yz, x0, x1) -> np.ndarray:
    """A convex Y-Z polygon ``yz`` [(y, z), ...] swept along X from ``x0`` to ``x1``: its vertex set."""
    yz = np.asarray(yz, dtype=float)
    if yz.ndim != 2 or yz.shape[1] != 2 or len(yz) < 3:
        raise ValueError("yz must be at least three (y, z) points")
    if not x1 > x0:
        raise ValueError("need x1 > x0")
    return np.vstack([np.column_stack([np.full(len(yz), x), yz[:, 0], yz[:, 1]]) for x in (x0, x1)])


@dataclass(frozen=True)
class CaptureGeometry:
    """The callables behind ``with_capture_features``; picklable when its three functions are
    module-level, so the sim's worker processes can rebuild it."""

    core_section: Callable[[Any, str], Any]
    core_extrusion: Callable[[Any], Any]
    features: Callable[[Any, str], list]

    def feature_sets(self, p, part):
        out = [np.asarray(v, dtype=float) for v in self.features(p, part)]
        for k, v in enumerate(out):
            if v.ndim != 2 or v.shape[1] != 3 or len(v) < 4:
                raise ValueError(f"capture feature {k} must be an (n >= 4, 3) vertex set in mm")
            if ConvexHull(v).volume <= 1e-6:
                raise ValueError(f"capture feature {k} has no volume")
        return out

    def solid(self, p, part):
        from brixzle.families import builders as B

        s = self.core_section(p, part)
        shape = B._swept(s.outer, s.holes, self.core_extrusion(p))
        for v in self.feature_sets(p, part):
            shape = shape.fuse(hull_solid(v))
        return shape.clean()

    def collision(self, p, part):
        from brixzle.families import builders as B

        core = B._pieces(self.core_section(p, part).outer, self.core_extrusion(p))
        return [*core, *self.feature_sets(p, part)]

    def section(self, p, part):
        return self.core_section(p, part)


def with_capture_features(section, extrusion, features) -> Solid3D:
    """A ``Solid3D`` made of a prismatic interlock (``section``, ``extrusion``) and convex capture
    features (``features(p, part)`` -> vertex sets), with exact collision."""
    g = CaptureGeometry(section, extrusion, features)
    return Solid3D(solid=g.solid, collision=g.collision, section=g.section)


def capture_geometry(geometry):
    """The CaptureGeometry behind a family's geometry, or None."""
    owner = getattr(getattr(geometry, "collision", None), "__self__", None)
    return owner if isinstance(owner, CaptureGeometry) else None


def core_extrusion(geometry):
    """The interlock's extrusion function of a ``with_capture_features`` geometry, or None."""
    g = capture_geometry(geometry)
    return g.core_extrusion if g is not None else None
