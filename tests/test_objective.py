"""The build-aware objective (#164): error-tolerant overhang, grip, size and release height."""
import math

import pytest

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


def test_tolerance_is_the_largest_radius_whose_median_reaches_the_steps():
    per = {0.0: 6, 2.0: 4, 5.0: 3, 10.0: 1, 14.8: 0}
    assert OB.tolerance_radius(per, steps=3) == 5.0
    assert OB.tolerance_radius(per, steps=7) is None
    assert OB.tolerance_radius({0.0: 6, 2.0: 1, 5.0: 4}, steps=3) == 0.0  # the first radius that fails ends it


@pytest.mark.parametrize("name, passes", [("ab", True), ("fable_twinhook4", True), ("fable_allround", False)])
def test_the_strict_grip_rule(name, passes):
    fam = registry.get(name)
    r = OB.grip_rule(fam, fam.defaults)
    assert r["passed"] is passes
    if not passes:
        assert OB.grip_rule(fam, fam.defaults, strict=False)["passed"]  # it grips only when relaxed


def test_size_reads_length_pitch_and_release():
    ab, allround = registry.get("ab"), registry.get("fable_allround")
    s_ab, s_ar = OB.size(ab, ab.defaults), OB.size(allround, allround.defaults)
    assert 45 < s_ab["length_mm"] < 55 and 110 < s_ar["length_mm"] < 120
    assert s_ar["pitch_mm"] == 58.0 and s_ar["release_lift_mm"] > 50 > s_ab["release_lift_mm"]


def test_constraints_and_ranking():
    s = OB.Settings()
    good = {"name": "g", "grip": {"rule": True, "lift_mm": 0.7, "lift_deg": 0.3}, "size": {"length_mm": 60, "release_lift_mm": 25},
            "arm": {"overhang": 12, "bridge_riser_mm": 0}, "z": {"mean": 0.97}, "tolerance_mm": 4.0, "overhang_mean_steps": 29 / 6, "z_mu08": 0.9,
            "overhang_n": 11}
    worse = {**good, "name": "w", "tolerance_mm": 5.0, "overhang_mean_steps": 23 / 6}
    big = {**good, "name": "b", "size": {"length_mm": 116, "release_lift_mm": 62}, "tolerance_mm": 10.0}
    assert OB.violations(good, s) == []
    assert set(OB.violations(big, s)) == {"length", "release_lift"}
    ranked = OB.rank([worse, big, good], s)
    assert [r["name"] for r in ranked] == ["g", "w", "b"] and ranked[-1]["feasible"] is False
    assert OB.dominates(good, worse, s) and not OB.dominates(worse, good, s)


def test_the_sweep_scores_all_low_error_radii_even_after_an_early_threshold_failure(monkeypatch):
    built = []
    medians = {0.0: 6, 1.0: 6, 2.0: 3, 3.0: 1, 4.0: 1, 5.0: 6}

    def fake(family, p, r, builds, steps, seed, workers):
        built.append(r)
        return {"median_steps": medians.get(r, 0)}

    monkeypatch.setattr(OB, "noisy_corbel", fake)
    res = OB.overhang_tolerance(None, None, OB.Settings())
    assert built == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert res["tolerance_mm"] == 2.0
    assert res["overhang_mean_steps"] == pytest.approx(23 / 6)
