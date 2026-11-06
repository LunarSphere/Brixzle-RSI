"""Planar geometry for the predictor: a moving polygon over fixed polygons, in the XZ plane (mm).

A pose is (x, z, theta) of the body frame origin; theta (rad) rotates +x toward +z about that origin.
Contacts are vertex-edge pairs within ``eps`` in both directions (moving vertex on a fixed edge, fixed
vertex on a moving edge), each with a unit normal from the fixed body into the moving one.
"""

from dataclasses import dataclass
import math

import numpy as np
import shapely
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient

from brixzle.engine.predictor.contact_modes import Contact

EPS_MM = 0.01


@dataclass(frozen=True)
class Body:
    outline: Polygon  # body frame
    com: tuple        # (x, z), body frame
    mass: float       # any unit; only ratios matter (the load is normalised to the weight)
    inertia: float    # about the centre of mass, rotation in the XZ plane (mass x length^2)

    def M(self):
        """Generalised mass for (x, z, theta) normalised by the mass."""
        return np.diag([1., 1., self.inertia / self.mass])


def polygon_inertia(poly, about):
    """Polar second moment of area about ``about`` divided by the area (squared radius of gyration)."""
    x, z = np.asarray(orient(poly).exterior.coords).T
    x, z = x - about[0], z - about[1]
    cross = x[:-1] * z[1:] - x[1:] * z[:-1]
    area = cross.sum() / 2
    Ix = (cross * (z[:-1]**2 + z[:-1] * z[1:] + z[1:]**2)).sum() / 12
    Iz = (cross * (x[:-1]**2 + x[:-1] * x[1:] + x[1:]**2)).sum() / 12
    return (Ix + Iz) / area


def rot(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def place(body: Body, pose):
    """World vertices (n, 2), counter-clockwise, without the repeated last vertex."""
    pts = np.asarray(orient(body.outline).exterior.coords)[:-1]
    return pts @ rot(pose[2]).T + np.asarray(pose[:2], float)


def com_world(body: Body, pose):
    return rot(pose[2]) @ np.asarray(body.com, float) + np.asarray(pose[:2], float)


def env_edges(env):
    """(starts, ends, outward normals) of every fixed polygon's edges, counter-clockwise."""
    A, B = [], []
    for poly in env:
        pts = np.asarray(orient(poly).exterior.coords)
        A.append(pts[:-1]); B.append(pts[1:])
    A, B = np.vstack(A), np.vstack(B)
    return A, B, _outward(A, B)


def _reflex_ends(polys):
    """Per edge: is its start vertex / end vertex a reflex (concave) corner of its polygon?"""
    ra, rb = [], []
    for pts in polys:
        prev, nxt = np.roll(pts, 1, axis=0), np.roll(pts, -1, axis=0)
        d1, d2 = pts - prev, nxt - pts
        reflex = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0] < 0  # counter-clockwise: right turn
        ra.append(reflex); rb.append(np.roll(reflex, -1))
    return np.concatenate(ra), np.concatenate(rb)


def _outward(A, B):
    d = B - A
    n = np.column_stack([d[:, 1], -d[:, 0]])
    return n / np.linalg.norm(n, axis=1, keepdims=True)


def _distances(P, A, B):
    """(points x segments) distances from each point to each segment."""
    d = B - A
    w = P[:, None, :] - A[None, :, :]
    t = np.clip((w * d[None]).sum(-1) / (d * d).sum(-1)[None], 0, 1)
    Q = A[None] + t[..., None] * d[None]
    return np.linalg.norm(P[:, None, :] - Q, axis=-1)


def _ray_hits(P, A, B, down):
    """Vertical distance from each point to the segments along -z (down) or +z; inf if none."""
    dx = B[:, 0] - A[:, 0]
    keep = np.abs(dx) >= 1e-12  # vertical segments are never hit by a vertical ray
    A, B, dx = A[keep], B[keep], dx[keep]
    if not len(A):
        return np.full(len(P), np.inf)
    s = (P[:, 0, None] - A[None, :, 0]) / dx[None]
    z = A[None, :, 1] + s * (B[None, :, 1] - A[None, :, 1])
    gap = (P[:, 1, None] - z) if down else (z - P[:, 1, None])
    gap = np.where((s >= 0) & (s <= 1) & (gap >= -1e-9), gap, np.inf)
    return gap.min(axis=1)


def drop_distance(body: Body, pose, env):
    """How far the body falls straight down from ``pose`` before it touches; None if it misses."""
    V = place(body, pose)
    A, B, _ = env_edges(env)
    down = _ray_hits(V, A, B, down=True)
    W = np.vstack([np.asarray(orient(p).exterior.coords)[:-1] for p in env])
    up = _ray_hits(W, V, np.roll(V, -1, axis=0), down=False)
    d = min(down.min(), up.min())
    return None if not np.isfinite(d) else max(float(d), 0.)


def _vertex_edges(pts_list):
    """Per vertex (stacked over the polygons): its incoming and outgoing edge indices."""
    inc, out, off = [], [], 0
    for pts in pts_list:
        n = len(pts)
        idx = np.arange(n) + off
        inc.append(np.roll(idx, 1)); out.append(idx)
        off += n
    return np.concatenate(inc), np.concatenate(out)


_ENV, _BODY = {}, {}


def _env_data(env):
    """The fixed polygons' edges, normals, vertices, reflex flags and vertex-edge maps (cached)."""
    key = tuple(id(p) for p in env)
    hit = _ENV.get(key)
    if hit is None or any(a is not b for a, b in zip(hit[0], env)):
        pts = [np.asarray(orient(p).exterior.coords)[:-1] for p in env]
        A, B, nA = env_edges(env)
        inW, outW = _vertex_edges(pts)
        hit = _ENV[key] = (list(env), (A, B, nA, np.vstack(pts), _reflex_ends(pts)[0], inW, outW))
    return hit[1]


def _body_data(body):
    """Reflex flags and vertex-edge maps of the body outline (pose-independent, cached)."""
    hit = _BODY.get(id(body))
    if hit is None or hit[0] is not body:
        pts = np.asarray(orient(body.outline).exterior.coords)[:-1]
        inV, outV = _vertex_edges([pts])
        hit = _BODY[id(body)] = (body, (_reflex_ends([pts])[0], inV, outV))
    return hit[1]


def contacts(body: Body, pose, env, eps=EPS_MM, merge=0.05):
    """Contacts (world) between the placed body and the fixed polygons.

    - A vertex within eps of an edge's interior, and in front of it, touches that edge.
    - A vertex within eps of another vertex is a corner pair. In a concave (reflex) corner it touches
      both faces (a pair is two vertices within sqrt(2) eps). Convex on convex, the normal is undefined (a measure-zero balance): one contact
      along the fixed corner's mean normal, with the four face normals as ``alternatives``.
    """
    V = place(body, pose)
    VA, VB = V, np.roll(V, -1, axis=0)
    A, B, nA, W, rW, inW, outW = _env_data(env)
    nV = _outward(VA, VB)
    rV, inV, outV = _body_data(body)

    def touching(P, A, B, n):
        """(points x segments) mask: point within eps of the segment's interior, in front of it.
        Only pairs whose bounding boxes come within eps are tested (the rest are too far apart)."""
        out = np.zeros((len(P), len(A)), dtype=bool)
        lo, hi = np.minimum(A, B) - eps, np.maximum(A, B) + eps
        J = np.flatnonzero((hi[:, 0] >= P[:, 0].min()) & (lo[:, 0] <= P[:, 0].max())
                           & (hi[:, 1] >= P[:, 1].min()) & (lo[:, 1] <= P[:, 1].max()))
        if not len(J):
            return out
        I = np.flatnonzero((P[:, 0] >= lo[J, 0].min()) & (P[:, 0] <= hi[J, 0].max())
                           & (P[:, 1] >= lo[J, 1].min()) & (P[:, 1] <= hi[J, 1].max()))
        if not len(I):
            return out
        P, A, B, n = P[I], A[J], B[J], n[J]
        d = B - A
        L2 = (d * d).sum(-1)
        w = P[:, None, :] - A[None]
        t = (w * d[None]).sum(-1) / L2[None]
        tiny = eps / np.sqrt(L2)[None]
        inner = (t > tiny) & (t < 1 - tiny)
        front = (w * n[None]).sum(-1) >= -eps
        Q = A[None] + np.clip(t, 0, 1)[..., None] * d[None]
        near = ((P[:, None, :] - Q) ** 2).sum(-1) <= eps * eps
        out[np.ix_(I, J)] = near & front & inner
        return out

    hit1 = touching(V, A, B, nA)        # moving vertex k on env edge j
    hit2 = touching(W, VA, VB, nV)      # env vertex w on moving edge j
    out = []
    # Corner pairs within sqrt(2) eps: an edge contact excludes the last eps of each edge (``inner``),
    # where a vertex within eps of the edge lies up to sqrt(2) eps from its end vertex; a smaller pair
    # radius would miss it entirely (issue #36: the body then dropped straight through the corner).
    pairs = np.argwhere(np.linalg.norm(V[:, None] - W[None], axis=-1) <= math.sqrt(2) * eps)
    for k, w in pairs:
        hit1[k] = False
        hit2[w] = False
        if rW[w]:      # moving corner in a concave fixed corner: both fixed faces
            out += [Contact(tuple(V[k]), tuple(nA[j])) for j in (inW[w], outW[w])]
        elif rV[k]:    # fixed corner in a concave moving corner: both moving faces
            out += [Contact(tuple(W[w]), tuple(-nV[j])) for j in (inV[k], outV[k])]
        else:
            alts = [nA[inW[w]], nA[outW[w]], -nV[inV[k]], -nV[outV[k]]]
            n = nA[inW[w]] + nA[outW[w]]
            n = n / np.linalg.norm(n) if np.linalg.norm(n) > 1e-9 else nA[inW[w]]
            out.append(Contact(tuple(V[k]), tuple(n), alternatives=tuple(tuple(a) for a in alts)))
    for k, j in zip(*np.nonzero(hit1)):
        out.append(Contact(tuple(V[k]), tuple(nA[j])))
    for w, j in zip(*np.nonzero(hit2)):
        out.append(Contact(tuple(W[w]), tuple(-nV[j])))
    out = _line_extremes(out, eps)
    kept = []
    # Merge near-duplicates (within ``merge``, normals within 2.6 deg); a corner on two faces keeps
    # both. This keeps redundant pairs out of the mode analysis, but it can also merge two distinct
    # features (a body vertex on a fixed face and that face's end corner on a near-parallel body
    # face): predict.simulate then steps with ``merge=-1`` (nothing merged) if the merged motion
    # penetrates (issue #36).
    for c in out:
        if all(np.hypot(c.point[0] - q.point[0], c.point[1] - q.point[1]) > merge
               or np.dot(c.normal, q.normal) < 0.999 for q in kept):
            kept.append(c)
    return kept


def _line_extremes(cs, eps):
    """Collapse each straight line contact to its two end points.

    Contacts with the same normal on the same line (n . p equal within eps) form one flush face
    contact. Forces anywhere on a segment are combinations of forces at its ends, and a rigid body
    that does not penetrate at both ends does not penetrate in between, so the ends are exact.
    """
    groups = []
    for c in cs:
        n = np.asarray(c.normal, float)
        off = float(n @ np.asarray(c.point, float))
        for g in groups:
            if n @ g["n"] > 0.9999 and abs(off - g["off"]) <= eps:
                g["cs"].append(c)
                break
        else:
            groups.append({"n": n, "off": off, "cs": [c]})
    out = []
    for g in groups:
        if len(g["cs"]) <= 2:
            out += g["cs"]
            continue
        t = np.array([-g["n"][1], g["n"][0]])
        s = [t @ np.asarray(c.point, float) for c in g["cs"]]
        out += [g["cs"][int(np.argmin(s))], g["cs"][int(np.argmax(s))]]
    return out


_FIXED = {}


def _fixed(env):
    """The fixed polygons' union (prepared), its boundary and their vertices (cached per env)."""
    key = tuple(id(p) for p in env)
    hit = _FIXED.get(key)
    if hit is None or any(a is not b for a, b in zip(hit[0], env)):
        fixed = shapely.union_all(env)
        shapely.prepare(fixed)
        W = np.vstack([np.asarray(orient(p).exterior.coords)[:-1] for p in env])
        hit = _FIXED[key] = (list(env), (fixed, fixed.boundary, W))
    return hit[1]


def penetration(body: Body, pose, env):
    """Deepest vertex of either body inside the other (mm); 0 if nothing overlaps."""
    V = place(body, pose)
    poly = Polygon(V)
    fixed, boundary, W = _fixed(env)
    depth = 0.
    inside = shapely.contains_xy(fixed, V[:, 0], V[:, 1])
    if inside.any():
        depth = max(depth, float(shapely.distance(shapely.points(V[inside]), boundary).max()))
    inside = shapely.contains_xy(poly, W[:, 0], W[:, 1])
    if inside.any():
        depth = max(depth, float(shapely.distance(shapely.points(W[inside]), poly.boundary).max()))
    return depth


def _deepest_push(body: Body, pose, env):
    """(depth, translation) that lifts the deepest penetrating vertex of either body to the other's
    boundary; (0, None) if nothing overlaps."""
    poly = Polygon(place(body, pose))
    fixed = shapely.union_all(env)
    best = (0., None)
    V = np.asarray(poly.exterior.coords)[:-1]
    for k in np.flatnonzero(shapely.contains_xy(fixed, V[:, 0], V[:, 1])):
        q = np.asarray(shapely.shortest_line(shapely.Point(V[k]), fixed.boundary).coords[1])
        d = float(np.linalg.norm(q - V[k]))
        if d > best[0]:
            best = (d, q - V[k])           # move the body vertex out to the fixed boundary
    W = np.vstack([np.asarray(orient(p).exterior.coords)[:-1] for p in env])
    for k in np.flatnonzero(shapely.contains_xy(poly, W[:, 0], W[:, 1])):
        q = np.asarray(shapely.shortest_line(shapely.Point(W[k]), poly.boundary).coords[1])
        d = float(np.linalg.norm(q - W[k]))
        if d > best[0]:
            best = (d, W[k] - q)           # move the body's boundary out past the fixed vertex
    return best


def depenetrate(body: Body, pose, env, tol=1e-6, iters=4):
    """The pose translated out of a shallow overlap (issue #38: first-order steps drift into the
    surface when pivoting). Each pass lifts the deepest vertex to the boundary; the pose with the
    least penetration is returned, so it never makes things worse."""
    pose = np.asarray(pose, float).copy()
    best, best_pen = pose, penetration(body, pose, env)
    for _ in range(iters):
        if best_pen <= tol:
            break
        depth, t = _deepest_push(body, best, env)
        if t is None:
            break
        trial = best.copy()
        trial[:2] += t * (1 + 1e-6)  # a hair past the boundary, so it ends just outside
        pen = penetration(body, trial, env)
        if pen >= best_pen:
            break
        best, best_pen = trial, pen
    return best
