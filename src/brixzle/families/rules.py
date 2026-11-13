# From LunarSphere/aerial-assembly brixzle/src/brixzle/rules.py @ a649663
# Changed: R10.two_openings, R10.hangs_below and R10.straddles_com (carrying openings, issue #87) are new; only the RuleSettings values R6, R9 and R10 need are kept; margins are computed from interface data (guide faces, sim bodies, grip features) for any family instead of from BrickParams, and keyed by requirement id. R9 uses rules.py's formulas (mass_min, mass_max). The fork checks (hangs_stable, tines_straddle_com, with the lowest bore in place of the first) are in fork_margins, out of the loop's rule set since issue #50; R10.grippable and R10.within_payload (brick plus a gripper allowance against the 35 g payload) are new.
"""Shared analytic rules every family is checked against, before any simulation.

Each margin is signed: >= 0 means satisfied. Keys are '<requirement id>.<name>' so a report can
tie every margin to docs/requirements.yaml; a family adds its own through ``extra_rules``.

R10 is "grippable" (issue #50) and, since issue #87, "carried by two large openings": the brick
offers at least two through-Y openings a 5 mm circle fits in, above and either side of its centre
of mass, so it hangs level from two points whatever the gripper; and the brick and the gripper
together stay within the payload. The gripper is not designed yet, so its mass is an allowance.
The checks against today's fork are in ``fork_margins``; nothing in the loop calls them, and they
wait for the gripper step (issue #24).
"""

from dataclasses import dataclass
import math

import shapely
from shapely.geometry import Point

from brixzle.families import builders as B
from brixzle.families.interface import BORE, OPENING, Family, SimBodies, grip_bores


@dataclass(frozen=True)
class RuleSettings:
    mu_max: float = 0.4          # PLA-on-PLA upper estimate the design must still slide on
    slide_margin_deg: float = 8  # "steeper than friction suggests" (R6)
    mass_min: float = 1.         # g (R9)
    mass_max: float = 25.        # g (R9)
    payload_max: float = 35.     # g, the drone's payload limit: brick plus gripper (R10; issue #9)
    # g counted for the gripper until it is designed (issue #24). A guess: the fork model in
    # engine/sim/fork.py gives 3.3 to 4.6 g on the three registered families, rounded up.
    gripper_allowance: float = 5.
    min_feature: float = 1.2     # printable wall (3 perimeters @ 0.4 mm)
    hang_margin: float = 1.0     # the carrying points must sit this far above the COM (mm)
    carry_opening_min: float = 5.0  # mm: a carrying opening fits a circle this wide (issue #87)


def shared_margins(family: Family, p, part, bodies: SimBodies, s: RuleSettings = RuleSettings()):
    out = {}
    friction = math.degrees(math.atan(s.mu_max)) + s.slide_margin_deg
    faces = B.section(family, p, part).guide_faces
    if faces:
        # R6: every guide ramp is steeper than friction requires, with margin.
        out["R6.guide_slides"] = min(f.angle_deg for f in faces) - friction
    # R9: mass window in PLA.
    out["R9.mass_min"] = bodies.mass_g - s.mass_min
    out["R9.mass_max"] = s.mass_max - bodies.mass_g
    features = family.pickup(p, part)
    # R10 grippable: at least one feature a passive gripper could hold (how many beyond the one
    # needed), and the brick and the gripper together within the drone's payload. No gripper is
    # assumed (issue #50), so its mass is an allowance.
    out["R10.grippable"] = float(len(features) - 1)
    out["R10.within_payload"] = s.payload_max - (bodies.mass_g + s.gripper_allowance)
    out.update(carrying_margins(features, bodies, s))
    bores = grip_bores(features)
    if bores:
        # R13: printable material around every bore, to the outline and to any lightening hole
        # (measured in the predictor section).
        out["R13.bore_wall"] = min(_bore_wall(B.section(family, p, part), b) for b in bores) - s.min_feature
    return out


def carrying_openings(features, s: RuleSettings = RuleSettings()):
    """The through-Y grip features a ``carry_opening_min`` circle fits in (issue #87): a bore at
    least that wide, or an opening whose outline's largest inscribed circle is at least that wide.
    Each as (centre x, top z) in mm, section frame."""
    out = []
    for f in features:
        if f.kind == BORE and f.size_mm[0] >= s.carry_opening_min:
            out.append((f.x_mm, f.z_mm + f.size_mm[0] / 2))
        elif f.kind == OPENING and f.outline is not None:
            if 2 * shapely.maximum_inscribed_circle(f.outline).length >= s.carry_opening_min:
                c = f.outline.centroid
                out.append((float(c.x), float(f.outline.bounds[3])))
    return out


def carrying_margins(features, bodies: SimBodies, s: RuleSettings = RuleSettings()):
    """R10's carrying-opening margins (issue #87). The brick hangs from two points, whatever the
    gripper:

    - ``R10.two_openings``: carrying openings minus 2;
    - ``R10.hangs_below``: over the two outermost carrying openings in X, the lower of their top
      points, minus the centre-of-mass height, minus ``hang_margin``;
    - ``R10.straddles_com``: how far the centre of mass lies inside those two openings' centres in
      X, minus 1 mm.

    With fewer than two carrying openings, the last two are -1."""
    ops = carrying_openings(features, s)
    out = {"R10.two_openings": float(len(ops) - 2)}
    if len(ops) < 2:
        return {**out, "R10.hangs_below": -1.0, "R10.straddles_com": -1.0}
    left, right = min(ops), max(ops)
    cx, cz = bodies.com_mm[0], bodies.com_mm[2]
    out["R10.hangs_below"] = min(left[1], right[1]) - cz - s.hang_margin
    out["R10.straddles_com"] = min(cx - left[0], right[0] - cx) - 1.0
    return out


def fork_margins(family: Family, p, part, bodies: SimBodies, tine_d_mm=2.0, s: RuleSettings = RuleSettings()):
    """The checks against today's fixed fork, whose tines of diameter ``tine_d_mm`` go through the
    part's bores: the brick hangs below the tines and they straddle its centre of mass.

    Not part of the loop's rule set since issue #50 (``margins`` leaves them out): the gripper is
    designed after the brick. They are kept, with their tests, for that step (issue #24). A part
    with no bores gives {}."""
    bores = grip_bores(family.pickup(p, part))
    if not bores:
        return {}
    cx, cz = bodies.com_mm[0], bodies.com_mm[2]
    xs = [b.x_mm for b in bores]
    # The tines carry the brick from above its centre of mass, and straddle it in X by at least
    # 1 mm, so it hangs level. The tine top (z + d/2) is the contact.
    return {"R10.hangs_stable": min(b.z_mm + tine_d_mm / 2 for b in bores) - cz - s.hang_margin,
            "R10.tines_straddle_com": min(cx - min(xs), max(xs) - cx) - 1.0}


def _bore_wall(section, bore):
    centre = Point(bore.x_mm, bore.z_mm)
    walls = [section.outer.exterior.distance(centre) - bore.radius_mm]
    walls += [centre.buffer(bore.radius_mm).distance(h) for h in section.holes]
    return min(walls)


def margins(family: Family, p, part, bodies: SimBodies, s: RuleSettings = RuleSettings()):
    """Shared margins merged with the family's own ``extra_rules`` (family keys may not override)."""
    out = shared_margins(family, p, part, bodies, s)
    if family.extra_rules is not None:
        extra = family.extra_rules(p, part, bodies)
        clash = set(out) & set(extra)
        if clash:
            raise ValueError(f"extra_rules may not redefine shared margins: {sorted(clash)}")
        out.update(extra)
    return out
