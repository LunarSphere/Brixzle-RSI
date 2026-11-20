# New design for #167 (not from runs/fable): fable_gentle (src/brixzle/families/fable_gentle/family.py) with its
# teeth and horns printed in TPU on the PLA body. Changed: the hooks are a second, compliant material.
"""Gentle with TPU hooks: a mixed-material test of energy dissipation (#167).

The brick, its parameters, bounds and ring kit are fable_gentle's. Only the teeth and horns change
material: they are TPU on the PLA body, so in the sim they collide with their own, softer and more
damped contact (``Family.soft``; the scene gives them their own ``solref`` and friction).

**The TPU values are estimates, not measurements.** PLA keeps the repo's contact (``Physics``:
time constant 0.2, damping ratio 1.0, friction 0.3). For TPU (Shore 95A) the contact is taken
three times softer (time constant 0.6), overdamped (damping ratio 2.0) and grippier (friction 0.6).
These are guesses until the TPU friction and drop tests (docs/physical-tests-167.md) are run.
"""
from brixzle.families.fable_gentle.family import BOUNDS, Gentle, GentleParams  # noqa: F401
from brixzle.families.twinhook.base import _brick_layout, _extras

TPU_CONTACT = {"timeconst": 0.6, "dampratio": 2.0, "friction": 0.6, "estimate": True,
               "source": "guess (#167): 3x the PLA time constant, overdamped, TPU-on-PLA friction 0.6; unmeasured"}


class GentleTpuHook(Gentle):
    name = "fable_gentle_tpuhook"
    version = "0.1.0"
    description = "Gentle with TPU teeth and horns on its PLA body (#167): a mixed-material dissipation test."
    soft_contact = TPU_CONTACT

    def features(self, p, part):
        lay = _brick_layout(p)
        return _extras(p, lay, hooks=False) + self.wings(p, lay["core"]) + self.wing_ribs(p, lay)

    def soft_features(self, p, part):
        lay = _brick_layout(p)
        return _extras(p, lay)[len(_extras(p, lay, hooks=False)):]


FAMILY = GentleTpuHook.instance().family()
