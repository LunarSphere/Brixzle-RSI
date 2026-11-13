"""R10 carrying openings (issue #87): two large through-Y openings, above and either side of the
centre of mass, so the brick hangs level. Analytic rules: no simulation."""

from dataclasses import replace

import pytest

from brixzle.families import builders as B
from brixzle.families import rules as R
from brixzle.families.ab.family import FAMILY as AB
from brixzle.families.interface import bore
from brixzle.families.wedge.family import FAMILY as WEDGE

KEYS = ("R10.two_openings", "R10.hangs_below", "R10.straddles_com")


def _margins(family, part="A"):
    p = family.defaults
    return R.margins(family, p, part, B.sim_bodies(family, p, part))


def test_a_b_passes_all_three_and_its_values_are_pinned():
    """A/B: two windows (about 10 x 21 mm) carry it; its 2.6 mm bores are too small to count.
    Values from the code on 2026-11-13 (macOS arm64; mass properties to 1e-3 by platform)."""
    for part in AB.parts:
        m = _margins(AB, part)
        assert all(m[k] >= 0 for k in KEYS), (part, {k: m[k] for k in KEYS})
        assert m["R10.two_openings"] == 0.0
        assert m["R10.grippable"] >= 0 and m["R10.within_payload"] >= 0  # kept
        for k, v in PINNED[part].items():
            assert m[k] == pytest.approx(v, abs=0.02), (part, k)


# mm; the 0.02 mm tolerance covers the platform's CadQuery tessellation (centre of mass).
PINNED = {"A": {"R10.hangs_below": 10.6947, "R10.straddles_com": 2.5927},
          "B": {"R10.hangs_below": 10.7062, "R10.straddles_com": 2.9613}}


def test_a_v_with_one_centre_bore_fails_two_openings():
    one = replace(WEDGE, pickup=lambda p, part: (bore(0.0, 12.0, 6.0),))
    m = _margins(one)
    assert m["R10.two_openings"] == -1.0
    assert m["R10.hangs_below"] == -1.0 and m["R10.straddles_com"] == -1.0  # fewer than two: both -1


def test_two_bores_below_the_centre_of_mass_fail_hangs_below():
    low = replace(WEDGE, pickup=lambda p, part: (bore(-15.0, 6.0, 6.0), bore(15.0, 6.0, 6.0)))
    m = _margins(low)
    assert m["R10.two_openings"] == 0.0 and m["R10.straddles_com"] > 0
    assert m["R10.hangs_below"] < 0


def test_two_openings_on_one_side_fail_straddles_com():
    side = replace(WEDGE, pickup=lambda p, part: (bore(8.0, 14.0, 5.0), bore(18.0, 14.0, 5.0)))
    m = _margins(side)
    assert m["R10.two_openings"] == 0.0 and m["R10.straddles_com"] < 0


def test_small_bores_are_not_carrying_openings():
    small = replace(WEDGE, pickup=lambda p, part: (bore(-15.0, 14.0, 4.9), bore(15.0, 14.0, 4.9)))
    assert _margins(small)["R10.two_openings"] == -2.0
    assert R.RuleSettings().carry_opening_min == 5.0


@pytest.mark.parametrize("name", ["wedge", "pyramid"])
def test_the_dummy_families_carry_on_two_openings(name):
    from brixzle.families import registry

    fam = registry.get(name)
    for part in fam.parts:
        m = _margins(fam, part)
        assert all(m[k] >= 0 for k in KEYS), (name, part, {k: m[k] for k in KEYS})
