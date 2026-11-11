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


def test_poses_are_reproducible_from_the_seed():
    a, b, c = QC.check_poses(seed=3), QC.check_poses(seed=3), QC.check_poses(seed=4)
    assert [(k, u.tolist()) for k, u in a] == [(k, u.tolist()) for k, u in b]
    assert [u.tolist() for _, u in a] != [u.tolist() for _, u in c]


@pytest.mark.parametrize("seed", [0, 1, 7])
def test_poses_are_spread_evenly_over_every_axis_and_both_courses(seed):
    poses = QC.check_poses(n=40, seed=seed)
    units = np.array([u for _, u in poses])
    assert units.shape == (40, 3) and np.all(np.abs(units) <= 1.0)
    for axis in range(3):  # Latin hypercube: exactly one pose in each of the 40 equal slices
        slices = np.floor((units[:, axis] + 1.0) / 2.0 * 40).astype(int)
        assert sorted(slices.tolist()) == list(range(40))
    assert sorted(k for k, _ in poses) == [0] * 20 + [1] * 20


def test_poses_lie_inside_the_target_box():
    box = BOXES["p95"]
    w = box.half_widths
    for _, unit in QC.check_poses(seed=0):
        s = BS.pose_sample(box, 1.0, unit, CFG)
        assert abs(s["dx"] - s["aim"]) <= w["x_mm"]
        assert CFG.error.clearance[0] <= s["clear"] <= CFG.error.clearance[0] + 2 * w["z_mm"]
        assert abs(s["pitch"]) <= w["theta_deg"]


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


def test_an_unstable_drop_is_a_failure_and_never_written_as_nan():
    """A simulation that blows up returns NaN. trials.classify calls that jammed (never seated)."""
    sample = {"dx": 4.0, "aim": 3.0, "clear": 6.0, "pitch": 0.1}
    raw = {"outcome": "jammed", "err_mm": [float("nan"), 0.0, 1.0], "angle_deg": float("inf"), "contacts": [],
           "settled": False, "time": 1.5, "f_max": float("nan"), "penetration_mm": 0.0}
    row = QC.drop_row(0, 1, np.zeros(3), sample, raw)
    assert row["outcome"] == "unstable" and row["err_mm"] == [None, 0.0, 1.0] and row["angle_deg"] is None
    assert row["f_max_N"] is None
    json.dumps(row, allow_nan=False)


def test_too_few_results_is_an_error():
    with pytest.raises(ValueError):
        QC.tally(_rows(["seated"] * 5), n=40, need=32)


# --- A/B V0 in the sim ---


@pytest.fixture(scope="module")
def ab_max():
    return QC.quick_check(AB, V0, BOXES["max"], seed=0, workers=1)


def test_ab_v0_fails_the_worst_box_early(ab_max):
    r = ab_max
    assert r["label"] == BS.LABEL == "simulated, uncalibrated contact"
    assert r["passed"] is False and r["stopped_early"] is True
    assert r["failed"] == 9 and r["dropped"] < 40 and r["dropped"] == len(r["drops"])
    assert r["seconds"] > 0 and r["n"] == 40 and r["need"] == 32


def test_failures_go_back_with_pose_outcome_and_contact_trace(ab_max):
    fails = ab_max["failures"]
    assert len(fails) == 9
    for f in fails:
        assert f["outcome"] in ("jammed", "wrong_valley", "fell")
        assert {"index", "course", "dx_mm", "clear_mm", "pitch_deg", "err_mm", "angle_deg"} <= set(f)
        assert f["contacts"], "a failing drop keeps its contact trace"
    seated = [d for d in ab_max["drops"] if d["outcome"] == "seated"]
    assert seated and all("contacts" not in d for d in seated)  # only failures carry traces


def test_the_check_does_not_depend_on_the_worker_count(ab_max):
    again = QC.quick_check(AB, V0, BOXES["max"], seed=0, workers=2)
    key = lambda r: [(d["index"], d["course"], d["outcome"]) for d in r["drops"]]
    assert key(again) == key(ab_max)
    assert (again["passed"], again["seated"], again["failed"]) == (ab_max["passed"], ab_max["seated"], ab_max["failed"])


def test_a_different_seed_drops_different_poses(ab_max):
    other = QC.quick_check(AB, V0, BOXES["max"], seed=1, workers=1)
    assert [d["dx_mm"] for d in other["drops"]] != [d["dx_mm"] for d in ab_max["drops"]]


def test_ab_v0_passes_the_mean_box():
    """On 2026-11-11, seed 0 seated 35 of 40 here (macOS arm64). The count is not pinned exactly:
    a drop at a capture edge can flip between platforms."""
    r = QC.quick_check(AB, V0, BOXES["mean"], seed=0)
    assert r["passed"] is True and r["stopped_early"] is False and r["dropped"] == 40
    assert 32 <= r["seated"] <= 38 and r["seated"] + r["failed"] == 40
    assert r["workers"] == 1  # one process by default: the fastest for this few drops


@pytest.mark.parametrize("workers", [0, -1, 2.5, True])
def test_the_drop_pool_rejects_a_bad_worker_count(workers):
    with pytest.raises(ValueError):
        BS.DropPool(AB, V0, CFG, workers=workers)


def test_only_placement_boxes_are_checked():
    with pytest.raises(ValueError):
        QC.quick_check(AB, V0, BX.default_boxes()["pickup"]["mean"])


# --- output ---


def test_check_json_goes_to_a_new_run_directory(ab_max, tmp_path):
    a = QC.write_check(ab_max, tmp_path)
    b = QC.write_check(ab_max, tmp_path)
    assert a != b and a.parent == b.parent == tmp_path  # never overwrites an earlier run
    assert a.name.startswith("check-") and a.name.endswith("-ab-max")
    saved = json.loads((a / "check.json").read_text(encoding="utf-8"))
    assert saved["label"] == "simulated, uncalibrated contact"
    assert saved["passed"] is False and saved["seed"] == 0 and saved["box"]["statistic"] == "max"
