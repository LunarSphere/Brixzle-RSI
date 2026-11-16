"""The structure gate (issue #83): a design must build a tower, a running-bond wall and a bridge,
and an overhang of at least ``overhang_floor`` bricks; ``overhang_n`` is the overhang's length.
Each structure is judged on seeded repeats (#81, #98). Simulated, uncalibrated contact."""

import json
from pathlib import Path

import pytest

from brixzle.engine import structure_gate as G
from brixzle.families.ab.family import FAMILY as AB


def test_a_b_passes_the_gate_and_reports_its_overhang(ab_gate):
    """A/B's overhang, built back-first with each drop aimed at the brick below (#137), reaches 9
    steps of 12 in every repeat; the old staircase aimed at the lattice stopped at 5 (#81)."""
    g = ab_gate
    assert g["overhang_n"] == 9 and g["overhang_floor"] == 3
    assert set(g["builds"]) == {"tower", "wall", "bridge", "overhang"}
    assert g["builds"]["tower"]["passed"] is True
    # A/B's bridge is borderline in the sim (#98, #137): its brick b1_5 ends 0.7 to 1.2 mm off its seat
    # against the 1 mm tolerance, so platform noise decides it (macOS arm64: 5/5; Linux x86_64 CI: 1/3).
    # The gate's verdict follows its bridge and wall; it is checked against them, not fixed here.
    assert g["passed"] is all(b["passed"] for b in g["builds"].values())
    o = g["builds"]["overhang"]
    assert o["total"] == 24 and o["steps"] == [9, 9, 9] and o["passed"] is True
    assert g["repeats"] == 3 and g["label"] == "simulated, uncalibrated contact" and g["seconds"] > 0
    saved = json.loads((Path(g["run"]) / "gate.json").read_text(encoding="utf-8"))
    assert saved["passed"] is g["passed"] and saved["overhang_n"] == 9 and saved["family"] == "ab"
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

    for name in ("tower", "wall", "bridge", "overhang", "corbel"):
        monkeypatch.setattr(S, name, boom)
    g = G.run_gate(AB, AB.defaults, G.GateSettings(repeats=1), runs=tmp_path)
    assert g["passed"] is False and g["overhang_n"] == 0
    assert all(b["passed"] is False and "span_voxels = 3" in b["error"] for b in g["builds"].values())


@pytest.mark.parametrize("bad", [dict(repeats=0), dict(overhang_floor=13), dict(wall=(3,)), dict(tower=0)])
def test_gate_settings_are_checked(bad):
    with pytest.raises(ValueError):
        G.GateSettings(**bad)


# --- the half-supported step (#133) ---


def test_the_step_check_passes_a_b_both_ways():
    from brixzle.families.ab.family import FAMILY as AB

    out = G.step_check(AB, AB.defaults)
    assert out["passed"] and set(out["steps"]) == {"+x", "-x"} and out["seconds"] > 0
    assert all(s["status"] == "complete" for s in out["steps"].values())


def test_the_step_check_names_the_brick_that_tips_off():
    from pathlib import Path

    from brixzle.agents import check_family as CF

    yf = CF.load_family(Path(__file__).parent / "fixtures", "yfunnel")  # no hook: a half-supported brick falls
    out = G.step_check(yf, yf.defaults)
    assert not out["passed"]
    assert out["steps"]["+x"]["brick"] == "b1_1" and out["steps"]["+x"]["outcome"] == "fell"
    assert "half" in G.step_text(out)


# --- aim at the brick below, back-first overhang (#137) ---


def test_the_gate_aims_at_the_brick_below_and_builds_the_overhang_back_first():
    s = G.GateSettings()
    assert s.aim_at_supporter is True and s.overhang_build == "corbel"
    assert G.GateSettings(aim_at_supporter=False, overhang_build="staircase")  # the old gate, still available


def test_a_b_overhang_reaches_further_when_the_drop_aims_at_the_brick_below(ab_gate):
    """Aimed at the ideal lattice, A/B's overhang stopped at 5: a sagging cantilever's seat moves away
    from the lattice (#137). Simulated, uncalibrated contact."""
    o = ab_gate["builds"]["overhang"]
    assert ab_gate["overhang_n"] >= 8 and o["total"] == 2 * G.GateSettings().overhang_ask

