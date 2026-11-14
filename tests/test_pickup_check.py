"""Cheap pickup check on the kept design (issue #113): a report, not a gate. Simulated, uncalibrated
contact; the gripper is today's fixed fork, a stand-in until #24."""

from pathlib import Path

from brixzle.agents import check_family as CF
from brixzle.engine import pickup_check as PC
from brixzle.families.ab.family import FAMILY as AB

FIXTURES = Path(__file__).parent / "fixtures"


def test_a_b_is_picked_level_released_and_meets_r10_at_ideal_alignment(tmp_path):
    out = PC.pickup_check(AB, AB.defaults, seed=0, n_offsets=2, runs=tmp_path)
    assert out["checked"] is True and out["label"] == PC.LABEL
    assert out["r10"]["R10.two_openings"] >= 0 and "R10.within_payload" in out["r10"]
    assert out["picks"][0]["sample"] == {"dx": 0.0, "dz": 0.0, "psi": 0.0} and out["picks"][0]["outcome"] == "picked"
    assert len(out["picks"]) == 3 and out["release"]["passed"] is True
    assert out["seconds"] > 0 and (Path(out["run"]) / "pickup.json").exists()


def test_a_brick_with_no_usable_opening_is_reported_not_picked(tmp_path):
    stack = CF.load_family(FIXTURES, "stackv")  # 2.6 mm bores, no opening a 5 mm circle fits
    out = PC.pickup_check(stack, stack.defaults, seed=0, n_offsets=0, runs=tmp_path)
    assert out["checked"] is False and "opening" in out["reason"]
    assert out["r10"]["R10.two_openings"] < 0


def test_a_brick_that_varies_along_y_is_reported_not_checked(tmp_path):
    yf = CF.load_family(FIXTURES, "yfunnel")
    out = PC.pickup_check(yf, yf.defaults, seed=0, n_offsets=0, runs=tmp_path)
    assert out["checked"] is False and "Prismatic" in out["reason"]
    assert "R10.within_payload" in out["r10"]
