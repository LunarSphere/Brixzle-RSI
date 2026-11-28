"""The noisy corbel (#164, #206; reduced to it in #227)."""
import math


from brixzle.engine import objective as OB
from brixzle.families import registry


def test_the_error_at_a_radius_stays_inside_it():
    em = OB.radius_error(10.0)
    rng = __import__("numpy").random.default_rng(0)
    rs = [math.hypot(s["dx"] - s["aim"], s["dy"]) for s in (em.sample(rng) for _ in range(200))]
    assert max(rs) <= 10.0 + 1e-9 and min(rs) >= 0.0 and em.track_supporter
    assert OB.radius_error(0.0).radii_mm == (0.0,)


def test_the_noisy_corbel_is_seeded_and_repeatable():
    fam = registry.get("ab")
    a = OB.noisy_corbel(fam, fam.defaults, radius_mm=2.0, builds=2, steps=2, workers=2)
    b = OB.noisy_corbel(fam, fam.defaults, radius_mm=2.0, builds=2, steps=2, workers=1)
    assert [x["steps"] for x in a["builds"]] == [x["steps"] for x in b["builds"]]
    assert a["median_steps"] == b["median_steps"] and 0 <= a["median_steps"] <= 2

