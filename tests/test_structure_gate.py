"""The structure gate (issue #83): a design must build a tower, a running-bond wall and a bridge,
and an overhang of at least ``overhang_floor`` bricks; ``overhang_n`` is the overhang's length.
Each structure is judged on seeded repeats (#81, #98). Simulated, uncalibrated contact."""

import json
from pathlib import Path

import pytest

from brixzle.engine import structure_gate as G
from brixzle.families.ab.family import FAMILY as AB


def test_a_b_passes_the_gate_and_reports_its_overhang(ab_gate):
    """A/B's overhang stopped at 5 of 12 in every repeat (#81: runs/builds-20261113T202519648071Z-ab-overhang+12)."""
    g = ab_gate
    assert g["passed"] is True and g["overhang_n"] == 5 and g["overhang_floor"] == 3
    assert set(g["builds"]) == {"tower", "wall", "bridge", "overhang"}
    for name in ("tower", "wall", "bridge"):
        assert g["builds"][name]["passed"] is True, (name, g["builds"][name]["statuses"])
    o = g["builds"]["overhang"]
    assert o["total"] == 12 and o["stable_bricks"] == [5, 5, 5] and o["passed"] is True
    assert g["repeats"] == 3 and g["label"] == "simulated, uncalibrated contact" and g["seconds"] > 0
    saved = json.loads((Path(g["run"]) / "gate.json").read_text(encoding="utf-8"))
    assert saved["passed"] is True and saved["overhang_n"] == 5 and saved["family"] == "ab"
    assert Path(g["run"]).name.startswith("gate-") and Path(g["run"]).name.endswith("-ab")
    assert "stages" not in json.dumps(saved)  # compact: the full builds sit beside it
    full = json.loads((Path(g["run"]) / "builds-wall.json").read_text(encoding="utf-8"))
    assert len(full["builds"]) == 3 and full["builds"][0]["stages"]


def test_a_stack_only_v_fails_the_gate_naming_the_wall(stack_v, tmp_path):
    g = G.run_gate(stack_v, stack_v.defaults, G.GateSettings(repeats=1), runs=tmp_path)
    assert g["passed"] is False and g["builds"]["wall"]["passed"] is False
    assert "wall" in G.failures_text(g)
    lines = G.failures_for_agent(g)
    # Its course-1 bricks would have to pass through course 0, so the wall cannot even be planned.
    (wall,) = [f for f in lines["failed"] if f["structure"] == "wall"]
    assert "No clear insertion path" in wall["error"] and "b1_" in wall["error"]
    assert g["builds"]["tower"]["passed"] is True  # it stacks


def test_overhang_n_is_the_count_most_repeats_reach():
    assert G.majority_count([5, 7, 3, 9, 1]) == 5
    assert G.majority_count([3, 7, 7]) == 7 and G.majority_count([2]) == 2
    assert G.majority_count([0, 0, 12, 12, 12]) == 12


def test_a_structure_that_raises_counts_as_failed(monkeypatch, tmp_path):
    from brixzle.engine import structures as S

    def boom(*a, **k):
        raise ValueError("not supported for span_voxels = 3")

    for name in ("tower", "wall", "bridge", "overhang"):
        monkeypatch.setattr(S, name, boom)
    g = G.run_gate(AB, AB.defaults, G.GateSettings(repeats=1), runs=tmp_path)
    assert g["passed"] is False and g["overhang_n"] == 0
    assert all(b["passed"] is False and "span_voxels = 3" in b["error"] for b in g["builds"].values())


@pytest.mark.parametrize("bad", [dict(repeats=0), dict(overhang_floor=13), dict(wall=(3,)), dict(tower=0)])
def test_gate_settings_are_checked(bad):
    with pytest.raises(ValueError):
        G.GateSettings(**bad)
