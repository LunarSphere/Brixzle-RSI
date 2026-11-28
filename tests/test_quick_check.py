"""Quick random check (issue #47): 40 evenly spread sim drops in the target box, 32 seated to pass.

Sim results are simulated, uncalibrated contact. The pose and tally tests need no sim; the A/B
tests drop the V0 baseline (the max box fails within about ten drops, so they stay short).
"""

import json

import numpy as np
import pytest

from brixzle.engine import boxes as BX
from brixzle.engine import box_search as BS
from brixzle.engine import quick_check as QC
from brixzle.engine.sim import trials as T
from brixzle.families.ab.family import FAMILY as AB
from brixzle.families.ab.params import V0

BOXES = BX.default_boxes()["placement"]
CFG = T.TrialConfig(error=T.ErrorModel(ideal=True))


# --- poses ---


def test_the_check_drops_forty_poses_and_needs_thirty_two():
    assert (QC.N_DROPS, QC.NEED_SEATED) == (40, 32)
    assert QC.max_failures(40, 32) == 9  # at the 9th failure 32 is out of reach


@pytest.mark.parametrize("n, need", [(0, 0), (40, 41), (40, -1), (3.5, 2)])
def test_bad_counts_are_rejected(n, need):
    with pytest.raises(ValueError):
        QC.max_failures(n, need)


# --- the pass rule and the early stop (no sim) ---


def _rows(outcomes):
    return ({"outcome": o, "contacts": [{"piece": 0}], "err_mm": [0.0, 0.0, 0.0], "angle_deg": 0.0} for o in outcomes)


def test_thirty_two_of_forty_seated_passes():
    out = QC.tally(_rows(["jammed"] * 8 + ["seated"] * 32), n=40, need=32)
    assert (out["passed"], out["seated"], out["failed"], out["dropped"]) == (True, 32, 8, 40)
    assert out["stopped_early"] is False


def test_thirty_one_of_forty_seated_fails_at_the_ninth_failure():
    outcomes = ["seated"] * 31 + ["wrong_valley"] * 9
    out = QC.tally(_rows(outcomes), n=40, need=32)
    assert (out["passed"], out["seated"], out["failed"], out["dropped"]) == (False, 31, 9, 40)


def test_the_check_stops_at_the_ninth_failure():
    consumed = []

    def rows():
        for i, o in enumerate(["jammed", "seated"] * 20):
            consumed.append(i)
            yield {"outcome": o, "contacts": []}

    out = QC.tally(rows(), n=40, need=32)
    assert out["stopped_early"] is True and out["passed"] is False
    assert (out["failed"], out["seated"], out["dropped"]) == (9, 8, 17)
    assert consumed == list(range(17))  # nothing after the 9th failure is read
    assert len(out["rows"]) == 17


def test_too_few_results_is_an_error():
    with pytest.raises(ValueError):
        QC.tally(_rows(["seated"] * 5), n=40, need=32)


# --- A/B V0 in the sim ---


@pytest.mark.parametrize("workers", [0, -1, 2.5, True])
def test_the_drop_pool_rejects_a_bad_worker_count(workers):
    with pytest.raises(ValueError):
        BS.DropPool(AB, V0, CFG, workers=workers)


# --- output ---


