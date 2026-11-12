"""The dummy wedge seats in the drop trial (issue #70).

The design agent reads the wedge as its minimal example, so it has to seat where the trial
expects: a course-0 brick at Placement(0, 1), centred at x = 2U. Simulated, uncalibrated contact.
"""

import pytest

from brixzle.engine.sim import trials as T
from brixzle.families.wedge.family import FAMILY as WEDGE


@pytest.mark.parametrize("course", [0, 1])
def test_the_wedge_seats_when_dropped_at_the_aim(course):
    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    rig = T.DropRig(WEDGE, WEDGE.defaults, cfg, course=course)
    row = rig.drop(cfg.error.sample(None), trace=False)
    assert row["outcome"] == "seated", row


def test_the_wedge_base_has_a_valley_under_each_seat():
    p = WEDGE.defaults
    base = WEDGE.base(p, 4, 1)
    top = sorted((x, z) for x, z in base.outer.exterior.coords if z >= 0)
    valleys = sorted({round(x, 6) for x, z in top if abs(z) < 1e-9})
    seat_x = WEDGE.seat_pose(p, __import__("brixzle.families.interface", fromlist=["Placement"]).Placement(0, 1))[0][0]
    assert round(seat_x, 6) in valleys
