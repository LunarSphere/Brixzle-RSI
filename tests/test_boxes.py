"""Target error boxes (issue #7): X measured (exp3-r4), Z / theta / psi estimated (exp3-r2 hover)."""

import json
import math
import re
from pathlib import Path

import pytest

from brixzle.engine import boxes as BX
from brixzle.engine.sim import approach_error as AE

ROOT = Path(__file__).resolve().parents[1]
STATS = ("mean", "p95", "max")
AXES = {"placement": {"x_mm", "z_mm", "theta_deg"}, "pickup": {"x_mm", "z_mm", "psi_deg"}}


@pytest.fixture(scope="module")
def boxes():
    return BX.default_boxes()


@pytest.fixture(scope="module")
def hover():
    return BX.load_hover(BX.HOVER_PATH)


def test_x_side_is_the_full_exp3_r4_radius(boxes):
    summary = AE.load(BX.ERROR_PATH)
    expect = {"mean": 14.82, "p95": 27.78, "max": 38.50}
    for task in AXES:
        for stat in STATS:
            x = boxes[task][stat].half_widths["x_mm"]
            assert x == pytest.approx(summary[f"{stat}_mm"])
            assert x == pytest.approx(expect[stat], abs=0.01)


def test_unmeasured_sides_come_from_the_hover_estimates(boxes, hover):
    for stat in STATS:
        place, pick = boxes["placement"][stat], boxes["pickup"][stat]
        assert set(place.half_widths) == AXES["placement"]
        assert set(pick.half_widths) == AXES["pickup"]
        assert place.half_widths["z_mm"] == pick.half_widths["z_mm"] == hover[f"{stat}_z_mm"]
        assert place.half_widths["theta_deg"] == hover[f"{stat}_pitch_deg"]
        assert pick.half_widths["psi_deg"] == hover[f"{stat}_yaw_deg"]


def test_boxes_grow_from_mean_to_p95_to_max(boxes):
    for task, axes in AXES.items():
        for axis in axes:
            w = [boxes[task][s].half_widths[axis] for s in STATS]
            assert 0 <= w[0] <= w[1] <= w[2], (task, axis, w)


def test_every_side_says_whether_it_is_measured_or_estimated(boxes):
    for task, by_stat in boxes.items():
        for box in by_stat.values():
            assert set(box.sources) == set(box.half_widths)
            assert box.sources["x_mm"].startswith("measured")
            assert all(box.sources[a].startswith("estimated") for a in box.half_widths if a != "x_mm")


def test_boxes_are_json_safe(boxes):
    out = {t: {s: b.to_dict() for s, b in by.items()} for t, by in boxes.items()}
    assert json.loads(json.dumps(out, allow_nan=False)) == out


def test_committed_hover_file_records_its_source(hover):
    src = hover["source"]
    assert src["repo"] == "LunarSphere/aerial-assembly"
    assert src["commit"].startswith("a649663")
    assert src["trials"] and src["commanded_yaw_deg"] == 0
    assert hover["n"] == len(hover["approaches"]) > 0
    assert "estimated" in hover["label"]


def _hover_dict(**overrides):
    d = json.loads(BX.HOVER_PATH.read_text(encoding="utf-8"))
    d.update(overrides)
    return d


@pytest.mark.parametrize("change", [
    {"kind": "approach-error"},
    {"mean_z_mm": math.nan},
    {"max_yaw_deg": -1.0},
    {"n": 0},
])
def test_load_hover_rejects_bad_files(tmp_path, change):
    path = tmp_path / "hover.json"
    d = _hover_dict()
    d.update(change)
    path.write_text(json.dumps(d))  # NaN is written as NaN on purpose here
    with pytest.raises(ValueError):
        BX.load_hover(path)


def test_load_hover_rejects_a_missing_statistic(tmp_path):
    d = _hover_dict()
    del d["p95_pitch_deg"]
    path = tmp_path / "hover.json"
    path.write_text(json.dumps(d))
    with pytest.raises(ValueError, match="p95_pitch_deg"):
        BX.load_hover(path)


def _doc_table():
    """{(task, statistic): (x, z, angle)} from the 'Target boxes' table in docs/requirements.md."""
    text = (ROOT / "docs" / "requirements.md").read_text(encoding="utf-8")
    section = text.split("## Target boxes", 1)[1].split("\n## ", 1)[0]
    rows = {}
    for m in re.finditer(r"^\| (Placement|Pickup) \| (mean|p95|worst) \| ([\d.]+) \| ([\d.]+) \| ([\d.]+) \|",
                         section, re.M):
        task, stat = m.group(1).lower(), {"worst": "max"}.get(m.group(2), m.group(2))
        rows[(task, stat)] = tuple(m.group(i) for i in (3, 4, 5))
    return rows


def test_docs_list_the_same_boxes(boxes):
    rows = _doc_table()
    assert len(rows) == 6
    for task, angle in (("placement", "theta_deg"), ("pickup", "psi_deg")):
        for stat in STATS:
            w = boxes[task][stat].half_widths
            assert rows[(task, stat)] == (f"{w['x_mm']:.1f}", f"{w['z_mm']:.1f}", f"{w[angle]:.1f}"), (task, stat)
