# From LunarSphere/aerial-assembly brixzle/src/brixzle/rules.py @ a649663
# Changed: only the RuleSettings values R6, R9 and R10 need are kept; margins are computed from interface data (guide faces, sim bodies, grip features) for any family instead of from BrickParams, and keyed by requirement id. R9 uses rules.py's formulas (mass_min, mass_max). The fork checks (hangs_stable, tines_straddle_com, with the lowest bore in place of the first) are in fork_margins, out of the loop's rule set since issue #50; R10.grippable and R10.within_payload are new.
"""Shared analytic rules every family is checked against, before any simulation.

Each margin is signed: >= 0 means satisfied. Keys are '<requirement id>.<name>' so a report can
tie every margin to docs/requirements.yaml; a family adds its own through ``extra_rules``.

R10 is "grippable" (issue #50): the loop checks only that a brick offers a grip feature and stays
within the payload. The checks against today's fork are in ``fork_margins``; nothing in the loop
calls them, and they wait for the gripper step (issue #24).
"""

from dataclasses import dataclass
import math

from shapely.geometry import Point

from brixzle.families import builders as B
from brixzle.families.interface import Family, SimBodies, grip_bores


@dataclass(frozen=True)
class RuleSettings:
    mu_max: float = 0.4          # PLA-on-PLA upper estimate the design must still slide on
    slide_margin_deg: float = 8  # "steeper than friction suggests" (R6)
    mass_min: float = 1.         # g (R9)
    mass_max: float = 25.        # g (R9)
    payload_max: float = 35.     # g, the drone's payload limit (R10; settled in issue #9)
    min_feature: float = 1.2     # printable wall (3 perimeters @ 0.4 mm)
    hang_margin: float = 1.0     # fork check: tine contact must sit this far above the COM (mm)


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
    # needed), and a mass within the drone's payload. No gripper is assumed (issue #50).
    out["R10.grippable"] = float(len(features) - 1)
    out["R10.within_payload"] = s.payload_max - bodies.mass_g
    bores = grip_bores(features)
    if bores:
        # R13: printable material around every bore, to the outline and to any lightening hole
        # (measured in the predictor section).
        out["R13.bore_wall"] = min(_bore_wall(B.section(family, p, part), b) for b in bores) - s.min_feature
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
