# From runs/rsi/all-placement-codex-20261122/iter-01/gentle_inboard_sill/family.py.
# Unchanged campaign geometry for the held-placement regression (#191).
"""Gentle's 18 degree hook and 0.4 mm tip, with a pilot at the inboard keel."""
from dataclasses import dataclass, replace
import math

import numpy as np
import shapely

from brixzle.families import builders as B
from brixzle.families import capture
from brixzle.families.fable_gentle.family import BOUNDS, Gentle, GentleParams
from brixzle.families.interface import Section, opening
from brixzle.families.twinhook import base as T


@dataclass(frozen=True)
class Lean18Params(GentleParams):
    hook_lean: float = 18.0
    tip_chamfer: float = 0.4
    early_engagement_mm: float = 0.6


def _early_engagement(p, keel):
    """Narrow extension on the hook-side half of the tooth, in the tooth's stroke frame."""
    d = p.early_engagement_mm
    if not math.isfinite(d) or d < 0:
        raise ValueError("early engagement must be finite and nonnegative")
    # Along the stroke s, the original sloped tip reaches its set-back shoulder
    # at t=w-chamfer. The pilot keeps that same entry-face slope over its own width.
    t_lo = p.hook_w - 1.8
    t_hi = p.hook_w - 0.18
    tip = p.hook_L - p.tip_back * t_lo / (p.hook_w - p.tip_chamfer) + d
    if tip > p.hook_L + p.slot_extra - 0.12:
        raise ValueError("pilot would bottom in the original ring socket")
    heel = p.hook_L - p.tip_back - p.tip_chamfer + d
    st = [(tip,t_lo), (p.hook_L-p.tip_back+d,p.hook_w-p.tip_chamfer),
          (heel,t_hi), (p.hook_L-p.tip_back-p.tip_chamfer-1.0,t_hi),
          (p.hook_L-p.tip_back-p.tip_chamfer-1.0,t_lo)]
    g = T.geom(p)
    xz = np.array([np.asarray(keel) + s*g["a"] + t*g["n"] for s,t in st])
    return np.vstack([np.column_stack((xz[:,0],np.full(len(xz),y),xz[:,1])) for y in (-0.8,0.8)])


class GentleLean18(Gentle):
    name = "gentle_inboard_sill"
    version = "0.4.0"
    description = "Early pilot with a raised internal sill in the negative-X end window."
    params_type = Lean18Params

    def __init__(self):
        super().__init__()
        self.cap = capture.CaptureGeometry(self.section,T.extrusion,self.features)

    @staticmethod
    def slot_params(p):
        # Flare the entrance on the side toward a long +x release. The retaining
        # wall and the last millimetre of the original parallel socket are preserved.
        return replace(p,side_gap=p.side_gap + 0.35*p.early_engagement_mm)

    def layout(self,p):
        lay = T._brick_layout(p)
        inlet = self.slot_params(p)
        x,z = lay["keels"][1]
        lay["slots"][1] = T._mouth(inlet,(x,z+p.H0),up=False)
        return lay

    def section(self,p,part):
        if p.early_engagement_mm == 0:
            section = T.section(p,part)
        else:
            lay = self.layout(p)
            outer = T._outline(p,lay)
            angles = (p.ramp_deg,T.geom(p)["left_deg"],90-p.hook_lean,
                      T.geom(p)["flare_deg"],T.geom(self.slot_params(p))["flare_deg"])
            guides = tuple(face for face in B.inclined_edges(outer)
                           if min(abs(face.angle_deg-a) for a in angles) < T.GUIDE_TOL_DEG
                           and math.dist(face.a,face.b)>1.0)
            section = Section(outer=outer, holes=tuple(T._holes(p,lay,outer)), guide_faces=guides)

        # Raise only the sill of the left end window. The same section is
        # swept through both Y bands, leaving the outside and the two carrying windows intact.
        holes = list(section.holes)
        index = min(range(len(holes)), key=lambda i: holes[i].centroid.x)
        window = holes[index]
        xmin, zmin, xmax, zmax = window.bounds
        sill_z = zmin + 0.6 * (zmax - zmin)
        holes[index] = window.intersection(shapely.box(xmin, sill_z, xmax, zmax))
        return replace(section, holes=tuple(holes))

    def features(self,p,part):
        lay = self.layout(p)
        pieces = T._extras(p,lay)
        if p.early_engagement_mm:
            pieces.append(_early_engagement(p,lay["keels"][0]))
        return pieces

    def pickup(self,p,part):
        holes = self.section(p,part).holes
        wide = [h for h in holes if 2*shapely.maximum_inscribed_circle(h).length >= T.OPENING_MIN_D_MM]
        return tuple(opening(h) for h in sorted(wide,key=lambda h:h.centroid.x))

    def wing_clearance(self,p):
        if p.early_engagement_mm == 0:
            return 0.0
        pilot_low = _early_engagement(p,(0.,0.))[:,2].min()
        return max(0.0,-pilot_low - p.hook_L*T.geom(p)["cos"])



FAMILY = GentleLean18.instance().family()
