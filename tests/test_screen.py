"""The cheap screen (issue #108): its score, the rule that picks what to map, and running several
screens at once. Simulated, uncalibrated contact."""

from pathlib import Path

import pytest

from brixzle.engine import boxes as BX
from brixzle.engine import screen as SC

FIXTURES = Path(__file__).parent / "fixtures"
BOX = BX.default_boxes()["placement"]["mean"]


def _c(s, j, n=60):
    return {"Z": (s - j) / n, "S": s, "J": j, "N": n}


def test_the_screen_score_has_the_form_of_z_over_both_checks():
    out = SC.combine({"outcomes": {"seated": 36, "jammed": 4}, "dropped": 40},
                     {"outcomes": {"seated": 10, "jammed": 6, "fell": 4}, "dropped": 20})
    assert out == {"Z": (46 - 10) / 60, "S": 46, "J": 10, "N": 60, "x": _c(36, 4, 40), "xy": _c(10, 6, 20)}


def test_only_a_clear_loser_is_screened_out_and_the_maps_are_capped_by_rank():
    cands = {"a": _c(47, 6), "b": _c(46, 7), "c": _c(49, 3), "d": _c(17, 19, 37)}  # ab4iter2's designs and yfunnel
    mapped, out = SC.select(cands, max_maps=3)
    assert mapped == ["c", "a", "b"]  # by screen score, best first
    assert set(out) == {"d"} and "below the best" in out["d"]
    mapped, out = SC.select(cands, max_maps=2)
    assert mapped == ["c", "a"] and "cap" in out["b"]


def test_with_one_candidate_it_is_mapped():
    assert SC.select({"only": _c(10, 30)}, max_maps=1) == (["only"], {})


def test_select_refuses_a_cap_below_one():
    with pytest.raises(ValueError):
        SC.select({"a": _c(1, 1)}, max_maps=0)


def test_screens_run_side_by_side_in_subprocesses(tmp_path):
    jobs = [("one", FIXTURES, "yfunnel"), ("two", FIXTURES, "yfunnel"), ("bad", tmp_path, "nosuchpackage")]
    out = SC.screen_many(jobs, "mean", seed=0, n_x=4, n_xy=4, workers=3)
    assert set(out) == {"one", "two", "bad"}
    # The X part stops early once the quick check has failed (yfunnel fails it); the X-Y part always runs.
    one = out["one"]
    assert one["xy"]["N"] == 4 and one["N"] == one["x"]["N"] + 4 and one["check"]["passed"] is False
    same = lambda r: {k: v for k, v in r.items() if k not in ("seconds", "check_full")}  # noqa: E731
    assert same(one) == same(out["two"]) and one["check_full"]["drops"] == out["two"]["check_full"]["drops"]
    assert "error" in out["bad"]
