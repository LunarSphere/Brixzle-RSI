# From LunarSphere/aerial-assembly brixzle/src/brixzle/ring.py @ fc4f724 (cell_profile, stroke_travel's stroke length)
# Changed: cell_profile becomes cell(p, part, hooks, exposed_lo, exposed_hi, cut_hi) -> Section for engine/ring_cells.kit_from_cell (teeth -> hooks, trim_lo/trim_hi -> exposed_lo/exposed_hi, extra_hi -> cut_hi, lighten from p.lighten); it uses this repo's port of profile.py.
"""A/B's ring kit (issue #89): one voxel of the A or B profile, and the kit built from it."""

import numpy as np
from shapely import affinity
from shapely.geometry import box

from brixzle.families.ab import profile as pr
from brixzle.families.interface import Section


def stroke_mm(p):
    """The slanted final stroke's length: the tooth and its slot clearance (not the sawtooth drop)."""
    return p.tooth_L + p.slot_extra


def cell(p, part="A", hooks=True, exposed_lo=False, exposed_hi=False, cut_hi=0.0):
    """One voxel of ``part`` (the brick's right voxel, x in [0, U]), centred on x = 0."""
    U = p.U
    top_m, bottom_m, top_rev, bottom_rev = pr.PART_FLAGS[part]
    x0 = p.end_gap / 2 if exposed_lo else 0.
    x1 = (U - p.end_gap / 2 if exposed_hi else U) - cut_hi
    body = pr._strip(p, x0, x1, top_mirrored=top_m, bottom_mirrored=bottom_m, bottom_offset=0.)
    feats = [body]
    if hooks:
        keel = U - p.valley_u if bottom_m else p.valley_u
        feats.append(pr.tooth(p, np.array([keel, 0.]), mirrored=bottom_m, reversed=bottom_rev))
    from shapely.ops import unary_union

    valley = U - p.valley_u if top_m else p.valley_u
    outer = unary_union(feats).difference(pr.slot(p, np.array([valley, p.H0]), mirrored=top_m, reversed=top_rev))
    outer = pr._largest(outer.buffer(-0.01, join_style=2).buffer(0.01, join_style=2))
    holes = []
    if p.lighten:
        parts = pr.brick_parts(p, part)
        voxel = box(0., outer.bounds[1] - 1, U, outer.bounds[3] + 1)
        for h in pr.lightening_holes(p, parts, pr.fork_channels(p)):
            # The brick's right voxel spans x in [0, U]: same profile as this cell.
            g = h.intersection(voxel).intersection(outer.buffer(-p.wall))
            for q in ([g] if g.geom_type == "Polygon" else getattr(g, "geoms", [])):
                if q.geom_type == "Polygon" and q.area > 6.:
                    holes.append(q)
    shift = lambda g: affinity.translate(g, -U / 2, 0.)  # noqa: E731
    return Section(outer=shift(outer), holes=tuple(shift(h) for h in holes))
