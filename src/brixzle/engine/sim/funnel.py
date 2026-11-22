"""Fixed PLA pickup fixture, in the waiting brick's frame. Dimensions are design starting points."""
from dataclasses import dataclass
import math

import numpy as np

from brixzle.engine.sim.scene import MM
from brixzle.families.rules import RuleSettings


@dataclass(frozen=True)
class FunnelStation:
    # Clear width perpendicular to insertion at the entrance, including the fork.
    mouth_width_m: float = .120
    taper_angle_rad: float = math.radians(55)  # guide face from transverse to the insertion stroke
    height_m: float = .012                    # rail above the plate's sliding surface
    clearance_m: float = .00025               # each side of the plate
    insertion_extra_m: float = .002           # short oblique tips need support past mid-depth

    def __post_init__(self):
        for value in (self.mouth_width_m, self.height_m, self.clearance_m, self.insertion_extra_m):
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError("station dimensions must be finite and positive")
        if type(self.taper_angle_rad) not in (float, int) or not 0 < self.taper_angle_rad < math.pi/2:
            raise ValueError("station taper must be between 0 and pi/2")

    def geometry(self, fork, aim, saddles, brick, friction):
        """Convex printable prisms in mm, plus the entry distance along brick Y in metres."""
        slope = fork["tine_dx_dy"]
        norm = math.hypot(1., slope)
        lo, hi = np.asarray(fork["plate"]) + aim/MM
        # u = x - slope*y stays constant on a shaft's insertion line. End the
        # tapered section before either tine can strike the waiting brick.
        ux = [x-slope*y for x in (lo[0], hi[0]) for y in (lo[1], hi[1])]
        left, right = min(ux)-self.clearance_m/MM, max(ux)+self.clearance_m/MM
        flare = (self.mouth_width_m/MM*norm - (right-left))/2
        if flare <= 0:
            raise ValueError("station mouth must be wider than the fork plate")
        # The shear changes the two face angles. Use the shallower face for DR1.
        angle = math.atan(max(friction, RuleSettings.mu_max)) + math.radians(RuleSettings.slide_margin_deg)
        if self.taper_angle_rad <= angle:
            raise ValueError("station guide angle must clear the DR1 friction margin")
        length = flare*(math.tan(self.taper_angle_rad)+abs(slope))/(norm*norm)
        stop = hi[1] + (self.insertion_extra_m/MM if slope else 0.)
        throat = lo[1] - (fork["y_tip"]-fork["y_face"]) - 4
        mouth = throat-length
        wall = 2.  # PLA wall, mm
        top = lo[2]
        bottom = min(float(np.min(p[:, 2])) for p in [*saddles, *brick]) - 2*wall
        height = self.height_m/MM

        def prism(l0, r0, l1, r1, y0, y1, z0, z1):
            return np.array([[u+slope*y, y, z] for z in (z0, z1)
                             for y, a, b in ((y0, l0, r0), (y1, l1, r1)) for u in (a, b)])

        pieces = [prism(left-flare-wall, right+flare+wall, left-flare-wall, right+flare+wall,
                        mouth-.75*self.mouth_width_m/MM, mouth, bottom, top)]
        for y0, y1, l0, r0, l1, r1 in (
                (mouth, throat, left-flare, right+flare, left, right),
                (throat, stop+wall, left, right, left, right)):
            pieces.extend([
                prism(l0-wall, r0+wall, l1-wall, r1+wall, y0, y1, bottom, top),
                prism(l0-wall, l0, l1-wall, l1, y0, y1, top, top+height),
                prism(r0, r0+wall, r1, r1+wall, y0, y1, top, top+height)])
        # Stops catch the plate's outer margins, leaving the shafts free between them.
        for a, b in ((left, left+2.), (right-2., right)):
            pieces.append(prism(a, b, a, b, stop, stop+wall, top, top+height))
        # Fill beneath each saddle down to the common foot. The pocket was
        # cut by a vertical sweep, so material below a saddle clears the brick.
        supported = []
        for saddle in saddles:
            foot = saddle.copy()
            foot[:, 2] = bottom
            supported.append(np.vstack([saddle, foot]))
        stock = np.vstack(saddles)
        x0, y0 = stock[:, :2].min(axis=0)
        x1, y1 = stock[:, :2].max(axis=0)
        bridge = np.array([[x, y, z] for x in (x0, x1) for y in (min(stop, y0), y1)
                           for z in (bottom, bottom+wall)])
        # A short lap under the guide floor joins the rail bed and saddle foot.
        pieces.append(bridge)
        return supported + pieces, (hi[1]-mouth)*MM
