# New design for #167 (not from runs/fable): fable_gentle (src/brixzle/families/fable_gentle/family.py) with a
# 6 mm hook. Changed: hook_L 5 -> 6 mm (the top of its bounds), for longer engagement before the seat.
"""Gentle with longer hooks: a geometric test of energy dissipation (#167).

A brick that lands hard has to rise out of its seat to bounce out. A longer tooth in a deeper slot
must rise further before it is free, so it should hold more impact energy. Everything else is
fable_gentle's. One change, to compare against gentle in engine/bounce.py.
"""
from dataclasses import dataclass

from brixzle.families.fable_gentle.family import BOUNDS, Gentle, GentleParams  # noqa: F401


@dataclass(frozen=True)
class LongHookParams(GentleParams):
    hook_L: float = 6.0       # hook length along its axis (gentle: 5.0)


class GentleLongHook(Gentle):
    name = "fable_gentle_longhook"
    version = "0.1.0"
    description = "Gentle with 6 mm hooks (#167): longer engagement before the seat, a geometric dissipation test."
    params_type = LongHookParams


FAMILY = GentleLongHook.instance().family()
