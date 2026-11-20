# New design for #167 (not from runs/fable): fable_gentle (src/brixzle/families/fable_gentle/family.py) with its
# teeth and horns printed in TPU on the PLA body. Changed: the hooks are a second, compliant material.
"""Gentle with TPU hooks: a mixed-material test of energy dissipation (#167).

The brick, its parameters, bounds and ring kit are fable_gentle's. Only the teeth and horns change
material: they are TPU on the PLA body, so in the sim they collide with their own, softer and more
damped contact (``Family.soft``; the scene gives them their own ``solref`` and friction).

The bounce trial's default PLA contact uses a 0.004 s time constant, a 1.0 damping ratio and
friction 0.35 (``TrialConfig``). The Shore 95A TPU contact uses unmeasured estimates of 0.6 s,
2.0 and 0.6, respectively. It uses the PLA impedance curve (solimp) and PLA density for the
mass properties. The TPU friction and drop tests are described in docs/physical-tests-167.md.
"""
from brixzle.families.fable_gentle.family import BOUNDS, Gentle, GentleParams  # noqa: F401
from brixzle.families.twinhook.base import _brick_layout, _extras

TPU_CONTACT = {"material": "TPU (Shore 95A)", "timeconst": 0.6, "dampratio": 2.0, "friction": 0.6, "estimate": True,
               "source": "Unmeasured estimates for Shore 95A TPU against PLA (#167)"}


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
