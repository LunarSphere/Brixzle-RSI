"""Near copies of A/B are refused in an explore round (issue #106)."""

from dataclasses import replace
from pathlib import Path

from brixzle.agents import check_family as CF
from brixzle.families import registry
from brixzle.loop import novelty as NV

AB = registry.get("ab")


def test_a_b_with_a_few_numbers_changed_is_a_near_copy():
    sub = replace(AB, name="ab_sub", params_type=AB.params_type)
    p = AB.defaults.__class__(**{**AB.defaults.to_dict(), "tooth_L": 7.0, "theta": 52.0})
    out = NV.near_copy(sub, p, AB, AB.defaults, share=0.2)
    assert out["near_copy"] and out["share"] < 0.2 and "near copy" in out["reason"]


def test_a_different_profile_is_not_a_near_copy():
    wedge = registry.get("wedge")
    out = NV.near_copy(wedge, wedge.defaults, AB, AB.defaults, share=0.2)
    assert not out["near_copy"] and out["share"] > 0.2


def test_a_family_that_varies_along_y_is_judged_new_by_construction():
    yf = CF.load_family(Path(__file__).parent / "fixtures", "yfunnel")
    out = NV.near_copy(yf, yf.defaults, AB, AB.defaults, share=0.2)
    assert not out["near_copy"] and "varies along Y" in out["reason"]
