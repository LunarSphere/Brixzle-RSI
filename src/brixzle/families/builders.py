# From LunarSphere/aerial-assembly brixzle/src/brixzle/cad.py @ a649663
# Changed: _wire, _swept, _pieces and _mesh take an Extrusion instead of BrickParams; the bore cut and mass properties follow build_brick; Solid3D support (y = 0 slice, CoACD decomposition), coverage and caching are new.
"""Shared builders: everything the engine needs from a family, derived from its geometry.

Prismatic families give a section and an extrusion; the solid is the section swept through the
extrusion's Y bands, and collision is exact (convex parts of the outline, each swept through one
band, are convex because the sweep is affine). Solid3D families give a CadQuery solid; collision
is their exact convex pieces if given, otherwise a CoACD convex decomposition. In both cases the
round grip bores from ``family.pickup`` are cut here, so grip features and geometry cannot disagree.
Collision covers the unbored outline: other bricks never touch the inside of a bore.
"""

import math

import cadquery as cq
import coacd
import numpy as np
import trimesh
from scipy.spatial import ConvexHull
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import polygonize, unary_union

from brixzle.families.convex import convex_parts
from brixzle.families.interface import (
    Extrusion, Family, GuideFace, Prismatic, Section, SimBodies, grip_bores,
)

PLA_DENSITY = 1.24e-3  # g/mm^3, from aerial-assembly brixzle/src/brixzle/params.py @ a649663
SLICE_Y_MM = 1e-3      # slice just off y = 0 so mesh vertices on the plane don't make degenerate cuts
GUIDE_ANGLE_RANGE_DEG = (5.0, 85.0)  # outline edges in this range count as guide faces
COACD = dict(threshold=0.05, preprocess_mode="off", seed=0)  # input meshes are watertight

_cache: dict = {}


def _cached(kind, family, p, part, build):
    key = (kind, id(family), p, part)
    if key not in _cache:
        _cache[key] = (family, build())  # keep family alive so its id stays unique
    return _cache[key][1]


def _wire(coords, y=0., dz=0.):
    pts = [cq.Vector(float(x), float(y), float(z) + dz) for x, z in list(coords)[:-1]]
    return cq.Wire.makePolygon(pts, close=True)


def _swept(polygon, holes, extrusion: Extrusion):
    solids = []
    for y0, y1, z0, z1 in extrusion.resolved_bands():
        outer = _wire(polygon.exterior.coords, y0, z0)
        inner = [_wire(h.exterior.coords, y0, z0) for h in holes]
        solids.append(cq.Solid.extrudeLinear(outer, inner, cq.Vector(0, y1 - y0, z1 - z0)))
    shape = solids[0]
    for other in solids[1:]:
        shape = shape.fuse(other)
    return shape.clean()


def _pieces(polygon, extrusion: Extrusion):
    """Convex 3D vertex sets (mm) covering the banded sweep of polygon."""
    coords = np.asarray(polygon.simplify(1e-4).exterior.coords)[:-1]
    pieces = []
    for q in convex_parts(coords):
        for y0, y1, z0, z1 in extrusion.resolved_bands():
            near = np.column_stack([q[:, 0], np.full(len(q), y0), q[:, 1] + z0])
            far = np.column_stack([q[:, 0], np.full(len(q), y1), q[:, 1] + z1])
            pieces.append(np.vstack([near, far]))
    return pieces


def _mesh(shape, tolerance=0.05):
    vertices, faces = shape.tessellate(tolerance, 0.2)
    mesh = trimesh.Trimesh(np.array([v.toTuple() for v in vertices]), np.array(faces), process=True)
    mesh.fix_normals()
    return mesh


def _unbored_solid(family: Family, p, part):
    g = family.geometry
    if isinstance(g, Prismatic):
        s = g.section(p, part)
        return _swept(s.outer, s.holes, g.extrusion(p))
    return g.solid(p, part)


def solid(family: Family, p, part, bored=True):
    """The printable CadQuery solid of one part (mm), with the grip bores cut if ``bored``."""
    def build():
        shape = _unbored_solid(family, p, part)
        if bored:
            bb = shape.BoundingBox()
            for b in grip_bores(family.pickup(p, part)):
                bore = cq.Solid.makeCylinder(b.radius_mm, bb.ymax - bb.ymin + 2,
                                             cq.Vector(b.x_mm, bb.ymin - 1, b.z_mm), cq.Vector(0, 1, 0))
                shape = shape.cut(bore)
        return shape
    return _cached(("solid", bored), family, p, part, build)


def solid_mesh(family: Family, p, part, bored=True):
    return _cached(("mesh", bored), family, p, part, lambda: _mesh(solid(family, p, part, bored)))


def shell_mesh(family: Family, p, part):
    """The solid's outer shell: no grip bores and, for Prismatic, no lightening holes.

    Collision covers this shell (other bricks never touch the inside of a hole), so collision
    coverage is measured against it.
    """
    g = family.geometry
    if isinstance(g, Prismatic):
        def build():
            return _mesh(_swept(g.section(p, part).outer, (), g.extrusion(p)))
        return _cached("shell", family, p, part, build)
    return solid_mesh(family, p, part, bored=False)


def inclined_edges(polygon, angle_range=GUIDE_ANGLE_RANGE_DEG):
    """Outline edges whose angle from horizontal lies inside angle_range, as guide faces."""
    coords = list(polygon.simplify(1e-3).exterior.coords)
    faces = []
    for a, b in zip(coords[:-1], coords[1:]):
        face = GuideFace((float(a[0]), float(a[1])), (float(b[0]), float(b[1])))
        if angle_range[0] < face.angle_deg < angle_range[1]:
            faces.append(face)
    return tuple(faces)


def slice_section(mesh, y=SLICE_Y_MM):
    """The XZ section of a watertight mesh at Y = y, with inclined outline edges as guide faces."""
    segments = trimesh.intersections.mesh_plane(mesh, plane_normal=[0, 1, 0], plane_origin=[0, y, 0])
    xz = np.round(np.asarray(segments)[:, :, [0, 2]], 6)  # snap: shared endpoints differ by float noise
    lines = unary_union([LineString(s) for s in xz if not np.array_equal(s[0], s[1])])
    faces = list(polygonize(lines))
    if not faces:
        raise ValueError("the solid does not cross the y = 0 plane; give Solid3D a section")
    points = np.array([[f.representative_point().x, y, f.representative_point().y] for f in faces])
    material = unary_union([f for f, inside in zip(faces, mesh.contains(points)) if inside])
    if not isinstance(material, Polygon):
        raise ValueError("the y = 0 slice is not one piece; give Solid3D a section")
    material = material.simplify(1e-3)
    outer = Polygon(material.exterior)
    holes = tuple(Polygon(r) for r in material.interiors)
    return Section(outer=outer, holes=holes, guide_faces=inclined_edges(outer))


def section(family: Family, p, part) -> Section:
    """The predictor's 2D section: the family's own, or a slice of a Solid3D at y = 0."""
    g = family.geometry
    if isinstance(g, Prismatic):
        return g.section(p, part)
    if g.section is not None:
        return g.section(p, part)
    return _cached("section", family, p, part,
                   lambda: slice_section(solid_mesh(family, p, part, bored=False)))


def decompose(mesh):
    """Convex pieces (mm) of a watertight mesh by CoACD, seeded so it is reproducible."""
    coacd.set_log_level("error")
    parts = coacd.run_coacd(coacd.Mesh(np.asarray(mesh.vertices), np.asarray(mesh.faces)), **COACD)
    return [np.asarray(v, dtype=float) for v, _ in parts]


def collision(family: Family, p, part):
    """(convex pieces, 'exact' | 'decomposed') for one part."""
    g = family.geometry
    if isinstance(g, Prismatic):
        return _pieces(g.section(p, part).outer, g.extrusion(p)), "exact"
    if g.collision is not None:
        return [np.asarray(v, dtype=float) for v in g.collision(p, part)], "exact"
    pieces = _cached("decomposed", family, p, part,
                     lambda: decompose(solid_mesh(family, p, part, bored=False)))
    return pieces, "decomposed"


def sim_bodies(family: Family, p, part) -> SimBodies:
    """Collision pieces and mass properties (PLA, solid infill) of one part."""
    pieces, source = collision(family, p, part)
    mesh = solid_mesh(family, p, part, bored=True)
    mesh.density = PLA_DENSITY
    return SimBodies(collision=pieces, collision_source=source, mass_g=float(mesh.mass),
                     volume_mm3=float(mesh.volume), com_mm=mesh.center_mass.tolist(),
                     inertia_g_mm2=mesh.moment_inertia.tolist(), mesh=mesh)


def base(family: Family, p, voxels, direction=1):
    """(convex pieces, lowest z) of an anchored base, in mm, base frame: x = 0 at the first voxel."""
    out = family.base(p, voxels, direction)
    if isinstance(out, Section):
        # Extruded like the family's bricks (e.g. the A/B gutter shear), else straight through D.
        g = family.geometry
        extrusion = g.extrusion(p) if isinstance(g, Prismatic) else Extrusion(family.lattice(p).D_mm)
        return _pieces(out.outer, extrusion), float(out.outer.bounds[1])
    pieces = ([np.asarray(v, dtype=float) for v in out] if isinstance(out, (list, tuple))
              else decompose(_mesh(out)))
    return pieces, float(min(v[:, 2].min() for v in pieces))


def base_pieces(family: Family, p, voxels, direction=1):
    """Convex pieces (mm, base frame: x = 0 at the first voxel's left edge) of an anchored base."""
    return base(family, p, voxels, direction)[0]


def inside_any(points, pieces, tol=0.0):
    """Mask of points inside at least one convex piece (tol > 0 grows, < 0 shrinks each piece)."""
    points = np.asarray(points, dtype=float)
    inside = np.zeros(len(points), dtype=bool)
    for piece in pieces:
        eq = ConvexHull(piece).equations
        inside |= np.all(points @ eq[:, :3].T + eq[:, 3] <= tol, axis=1)
    return inside


CONTACT_OFFSET_MM = 0.5  # half the 1 mm seat tolerance (trials.Tolerance.seat_xz)


def _band_shift(extrusion: Extrusion, y):
    for y0, y1, z0, z1 in extrusion.resolved_bands():
        if min(y0, y1) <= y <= max(y0, y1):
            return z0 + (z1 - z0) * (y - y0) / (y1 - y0)
    raise ValueError(f"y = {y} is outside the extrusion")


def contact_points(family: Family, p, part, offset=CONTACT_OFFSET_MM):
    """(points that must be inside the collision, points that must be outside it), mm.

    A few key contact points rather than a volume sample: along every edge of the section outline
    (the faces other bricks and the base touch), at a quarter, half and three quarters of its
    length, ``offset`` inside and ``offset`` outside. Sharp corners are left out on purpose: a
    convex decomposition rounds them off slightly, which doesn't change how a brick seats.
    Prismatic: at three depths across Y, following the bands. Solid3D: in the y = 0 slice.
    """
    s = section(family, p, part)
    g = family.geometry
    if isinstance(g, Prismatic):
        e = g.extrusion(p)
        planes = [(y, _band_shift(e, y)) for y in (-0.4 * e.depth_mm, 0.0, 0.4 * e.depth_mm)]
    else:
        planes = [(SLICE_Y_MM, 0.0)]
    coords = np.asarray(s.outer.simplify(1e-3).exterior.coords)
    inside, outside = [], []
    for a, b in zip(coords[:-1], coords[1:]):
        length = np.linalg.norm(b - a)
        if length < 4 * offset:  # too short to hold a point clear of both corners
            continue
        n = np.array([-(b - a)[1], (b - a)[0]]) / length
        for t in (0.25, 0.5, 0.75):
            q = a + t * (b - a)
            out_dir = n if not s.outer.contains(Point(*(q + offset * n))) else -n
            # Keep only points that really are `offset` from the outline: on a feature thinner
            # than 2 * offset (a tooth tip, a chamfer) the inside point would leave the brick.
            for pt, want_inside, bucket in ((q - offset * out_dir, True, inside),
                                            (q + offset * out_dir, False, outside)):
                point = Point(*pt)
                if s.outer.contains(point) == want_inside and s.outer.exterior.distance(point) >= 0.99 * offset:
                    bucket.append(pt)

    def lift(xz):
        xz = np.asarray(xz).reshape(-1, 2)
        return np.vstack([np.column_stack([xz[:, 0], np.full(len(xz), y), xz[:, 1] + dz]) for y, dz in planes])
    return lift(inside), lift(outside)


def is_convex_vertex_set(piece, tol=1e-6):
    """True if every vertex lies on the piece's convex hull (no interior points)."""
    piece = np.unique(np.round(np.asarray(piece, dtype=float), 9), axis=0)
    if len(piece) < 4:
        return False
    eq = ConvexHull(piece).equations
    dist = piece @ eq[:, :3].T + eq[:, 3]  # <= 0 inside; 0 on a facet plane
    return bool(np.all(np.max(dist, axis=1) >= -tol))


def math_finite(*values):
    """True if every number in every value (scalars or nested arrays of any shape) is finite."""
    return all(math.isfinite(float(x)) for v in values for x in np.ravel(np.asarray(v, dtype=float)))
