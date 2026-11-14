"""The drop probe (issue #107): its poses and its drops. Simulated, uncalibrated contact."""

import json
from pathlib import Path

import pytest

from brixzle.engine import boxes as BX
from brixzle.engine import probe as PR
from brixzle.engine import sim_map as SM
from brixzle.engine import zscore as ZS
from brixzle.engine.sim import trials as T
from brixzle.families.ab.family import FAMILY as AB

FIXTURES = Path(__file__).parent / "fixtures"
BOX = BX.default_boxes()["placement"]["mean"]
CLEAR0 = T.TrialConfig(error=T.ErrorModel(ideal=True)).error.clearance[0]


def _drops():
    """Map drops on every grid cell inside the mean box (A/B's pitch), with outcomes cycling
    seated, seated, jammed, wrong valley."""
    cells = ZS.box_cells(BOX, [BOX], 25.0)
    cycle = ("seated", "seated", "jammed", "wrong_valley")
    out = []
    for k, c in enumerate(cells):
        q = c["pose"]
        out.append({"slice": c["slice"], "course": c["course"], "cell": c["cell"], "outcome": cycle[k % 4],
                    "dx_mm": q["x"], "dy_mm": q.get("y", 0.0), "clear_mm": q["z"] + CLEAR0, "pitch_deg": q["t"]})
    return cells, out


def test_probe_poses_from_a_map_are_its_failures_some_controls_and_the_aim_and_repeat_for_a_seed():
    _, drops = _drops()
    poses = PR.probe_poses(BOX, drops, seed=3)
    assert len(poses) == PR.N_PROBE
    sources = [q["source"] for q in poses]
    assert sources.count("aim") == 2 and {q["course"] for q in poses if q["source"] == "aim"} == {0, 1}
    assert sources.count("seated") == PR.N_CONTROL
    assert set(sources) - {"aim", "seated"} <= set(PR.FAILURES) and sources.count("jammed") > 0
    assert PR.probe_poses(BOX, drops, seed=3) == poses
    assert PR.probe_poses(BOX, drops, seed=4) != poses
    for q in poses:
        pose = {"x": q["x_mm"], "y": q["y_mm"], "z": q["sample"]["clear"] - CLEAR0, "t": q["sample"]["pitch"]}
        assert ZS.inside(pose, BOX)


def test_probe_poses_never_include_a_held_out_pose():
    cells, drops = _drops()
    held = ZS.held_out_poses(cells, BOX, 25.0, seed=1)
    key = lambda x, y, z, t: (round(x, 6), round(y, 6), round(z, 6), round(t, 6))  # noqa: E731
    held_keys = {key(h["x"], h.get("y", 0.0), h["z"], h["t"]) for h in held}
    for poses in (PR.probe_poses(BOX, drops, seed=0, n=200), PR.probe_poses(BOX, None, seed=1)):
        for q in poses:
            if q["source"] == "aim":
                continue
            assert key(q["x_mm"], q["y_mm"], q["sample"]["clear"] - CLEAR0, q["sample"]["pitch"]) not in held_keys


def test_without_a_parent_map_the_poses_spread_over_the_box_disc_on_both_courses():
    poses = PR.probe_poses(BOX, None, seed=0)
    assert len(poses) == PR.N_PROBE and {q["course"] for q in poses} == {0, 1}
    r = BOX.half_widths["x_mm"]
    spread = [q for q in poses if q["source"] == "disc"]
    assert all(q["x_mm"] ** 2 + q["y_mm"] ** 2 <= r * r + 1e-6 for q in spread)
    assert max(abs(q["y_mm"]) for q in spread) > r / 2


def test_a_probe_needs_room_for_the_aim():
    with pytest.raises(ValueError):
        PR.probe_poses(BOX, None, n=1)


def test_the_probe_drops_a_package_in_a_subprocess_and_reports_each_pose(tmp_path):
    poses = PR.probe_poses(BOX, None, seed=0, n=4)
    path = PR.write_poses(poses, tmp_path / "poses.json")
    out = PR.probe_in_subprocess(FIXTURES, "yfunnel", path)
    assert out.get("error") is None, out
    assert out["drops"] == 4 and len(out["rows"]) == 4 and out["label"] == PR.LABEL
    assert [r["source"] for r in out["rows"]][:2] == ["aim", "aim"]
    assert out["rows"][0]["outcome"] == "seated"
    assert set(out["outcomes"]) <= {"seated", "jammed", "wrong_valley", "fell", "unstable"}
    assert json.dumps(out, allow_nan=False)


def test_a_probe_of_a_package_that_does_not_import_is_reported(tmp_path):
    path = PR.write_poses(PR.probe_poses(BOX, None, n=2), tmp_path / "poses.json")
    out = PR.probe_in_subprocess(tmp_path, "nosuchpackage", path)
    assert "error" in out and "ModuleNotFoundError" in out["error"]


def test_run_probe_counts_a_parent_failure_that_now_seats():
    poses = [{"source": "jammed", "course": 0, "x_mm": 0.0, "y_mm": 0.0,
              "sample": T.TrialConfig(error=T.ErrorModel(ideal=True)).error.sample(None)}]
    out = PR.run_probe(AB, AB.defaults, poses, workers=1)
    assert out["rows"][0]["outcome"] == "seated" and out["parent_failures_now_seated"] == 1
