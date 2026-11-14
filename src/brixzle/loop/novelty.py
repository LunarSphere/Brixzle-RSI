"""Is a design a new concept, or a near copy of a reference? (issue #106)

In an explore round A/B is the score to beat, not the starting file. The last real run's designs
were all 14 to 18 line subclasses of A/B's parameters with a few numbers changed
(``runs/loop/ab4iter2-20261114-mean``), so the loop refuses an explore design whose XZ section
differs from the reference's by less than a share of the reference's area.

The comparison: each part's section (``builders.section``: the interlock's, for a Solid3D made with
capture features), moved so its centroid is at the origin, against each of the reference's parts;
the share is the area of the symmetric difference over the reference part's area, and the design's
share is the smallest over the pairs. A family that varies along Y (a Solid3D) is judged new by
construction: its capture is not in one section.
"""

from shapely import affinity

from brixzle.families import builders as B

NEAR_COPY_SHARE = 0.2


def _centred(polygon):
    c = polygon.centroid
    return affinity.translate(polygon, -c.x, -c.y)


def near_copy(family, p, ref, ref_p, share=NEAR_COPY_SHARE):
    """{'near_copy', 'share', 'reason'} for ``family`` at ``p`` against ``ref`` at ``ref_p``."""
    if B.varies_along_y(family):
        return {"near_copy": False, "share": None,
                "reason": "the family varies along Y (Solid3D), so it is judged new by construction"}
    best = None
    for part in family.parts:
        a = _centred(B.section(family, p, part).outer)
        for rpart in ref.parts:
            b = _centred(B.section(ref, ref_p, rpart).outer)
            d = a.symmetric_difference(b).area / b.area
            if best is None or d < best[0]:
                best = (d, part, rpart)
    d, part, rpart = best
    if d < share:
        return {"near_copy": True, "share": d,
                "reason": (f"a near copy of {ref.name}: part {part}'s section differs from {ref.name} part {rpart}'s by "
                           f"{d:.0%} of its area, under the {share:.0%} an explore design needs; write your own profile")}
    return {"near_copy": False, "share": d,
            "reason": f"part {part}'s section differs from {ref.name} part {rpart}'s by {d:.0%} of its area"}
