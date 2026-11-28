"""Objective Z (issue #61): (seated - jammed) / drops inside each target box.

Most tests use made-up drops, so they need no sim. One test drops the A/B baseline at coarse
settings to check that the direct Z (the in-box cells alone) matches the full map's Z.
Sim results are simulated, uncalibrated contact.
"""

import pickle

import pytest

from brixzle.engine import boxes as BX
from brixzle.engine import zscore as ZS

BOXES = BX.default_boxes()["placement"]
MEAN, P95, WORST = BOXES["mean"], BOXES["p95"], BOXES["max"]
C0 = 5.0  # the trial's minimum release clearance (trials.ErrorModel.clearance[0])


def _drop(outcome, x=0.0, z=1.0, t=0.0, **extra):
    return {"outcome": outcome, "dx_mm": x, "clear_mm": C0 + z, "pitch_deg": t, **extra}


# --- counting ---


def test_seated_counts_plus_one_jammed_minus_one_and_the_rest_zero():
    c = ZS.counts(["seated"] * 6 + ["jammed"] * 2 + ["fell", "wrong_valley"])
    assert c == {"Z": (6 - 2) / 10, "S": 6, "J": 2, "N": 10, "fell": 1, "wrong_valley": 1, "other": 0,
                 "passed": False}


def test_z_is_one_only_when_every_drop_seats():
    assert ZS.counts(["seated"] * 5)["Z"] == 1.0 and ZS.counts(["seated"] * 5)["passed"]
    assert ZS.counts(["jammed"] * 5)["Z"] == -1.0
    assert ZS.counts(["fell"] * 5)["Z"] == 0.0


def test_an_outcome_that_is_not_named_counts_as_other_and_zero():
    c = ZS.counts(["seated", "unstable"])
    assert c["other"] == 1 and c["Z"] == 0.5


def test_z_of_no_drops_is_refused():
    with pytest.raises(ValueError, match="no drops"):
        ZS.counts([])


# --- the X-Y disc (issue #86) ---


def test_a_drop_counts_only_inside_the_x_y_disc():
    r = MEAN.half_widths["x_mm"]
    assert not ZS.inside({"x": 0.8 * r, "y": 0.8 * r, "z": 1.0, "t": 0.0}, MEAN)  # 1.13 r from the aim
    assert ZS.inside({"x": 0.6 * r, "y": 0.6 * r, "z": 1.0, "t": 0.0}, MEAN)      # 0.85 r
    assert ZS.inside({"x": r, "z": 1.0, "t": 0.0}, MEAN)                          # no y: on the X axis


# --- the noise band (#110) ---


def test_the_band_is_one_standard_error_of_z_for_its_counts():
    # each drop scores +1, 0 or -1, so var = (S + J) / N - Z^2 and the band is sqrt(var / N)
    c = {"Z": 248 / 356, "S": 279, "J": 31, "N": 356}
    var = (279 + 31) / 356 - (248 / 356) ** 2
    assert ZS.z_band(c) == pytest.approx((var / 356) ** 0.5)
    assert ZS.z_band(c) == pytest.approx(0.0329, abs=5e-4)


def test_the_band_is_zero_when_every_drop_has_the_same_outcome_and_refuses_no_drops():
    assert ZS.z_band({"Z": 1.0, "S": 10, "J": 0, "N": 10}) == 0.0
    assert ZS.z_band({"Z": -1.0, "S": 0, "J": 10, "N": 10}) == 0.0
    with pytest.raises(ValueError):
        ZS.z_band({"Z": 0.0, "S": 0, "J": 0, "N": 0})


def test_a_change_inside_the_band_is_no_change():
    c = {"Z": 0.5, "S": 70, "J": 20, "N": 100}
    band = ZS.z_band(c)
    assert ZS.within_band(0.5 + band * 0.9, 0.5, band)
    assert not ZS.within_band(0.5 - band * 1.1, 0.5, band)
