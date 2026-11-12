"""Design log (issue #20): every attempt logged; the agents see past attempts but never held-out results."""

import json

import pytest

from brixzle.agents import critic as CR
from brixzle.agents import runner as RN
from brixzle.loop import design_log as DL


def _entry(**kw):
    base = {"iteration": 1, "attempt": 1, "family": "zig", "module": "rsi_r_i01_a1", "decision": "kept",
            "reason": "first design", "z": {"mean": {"Z": 0.5, "S": 10, "J": 1, "N": 18}},
            "held_out": {"seed 1, friction 0.2": {"Z": 0.4}}, "note": {"summary": "a zigzag with long ramps"},
            "runs": {"map": "runs/map-x"}}
    return {**base, **kw}


def test_entries_are_appended_and_read_back_in_order(tmp_path):
    log = DL.DesignLog.in_dir(tmp_path)
    log.append(_entry())
    log.append(_entry(iteration=2, decision="failed_quick_check", z=None,
                      quick_check={"seated": 20, "dropped": 29, "outcomes": {"jammed": 9}}))
    got = log.entries()
    assert [e["iteration"] for e in got] == [1, 2] and got[1]["decision"] == "failed_quick_check"


@pytest.mark.parametrize("bad", [{"decision": "maybe"}, {"iteration": None}, {"z": {"mean": {"Z": float("nan")}}}])
def test_a_malformed_entry_is_refused(tmp_path, bad):
    entry = _entry(**bad)
    if bad.get("iteration", 1) is None:
        entry.pop("iteration")
    with pytest.raises(ValueError):
        DL.DesignLog.in_dir(tmp_path).append(entry)


def test_the_agents_view_drops_held_out_results(tmp_path):
    log = DL.DesignLog.in_dir(tmp_path)
    log.append(_entry())
    assert "held_out" in log.entries()[0]
    assert all("held_out" not in e for e in log.for_agents())
    text = log.text()
    assert "a zigzag with long ramps" in text and "+0.500" in text and "kept" in text
    assert "seed 1" not in text and "0.4" not in text.replace("+0.400", "")


def test_quick_check_failures_and_grid_overrides_are_in_the_text(tmp_path):
    log = DL.DesignLog.in_dir(tmp_path)
    log.append(_entry(decision="failed_quick_check", z=None, held_out=None,
                      quick_check={"seated": 20, "dropped": 29, "outcomes": {"jammed": 9, "seated": 20}}))
    log.append(_entry(iteration=2, overrides=[{"kind": "zoom", "reason": "look at the +X jam band"}]))
    text = log.text()
    assert "quick check failed: 20 of 29 seated" in text and "jammed 9" in text
    assert "look at the +X jam band" in text


def test_the_critic_sees_past_attempts_in_its_prompt(tmp_path):
    """The issue's 'done when': the critic sees past attempts."""
    log = DL.DesignLog.in_dir(tmp_path)
    log.append(_entry(note={"summary": "steeper ramps at 60 deg"}, decision="rejected",
                      reason="Z on the target did not improve"))
    brief = {"decision": "redesign", "reason": "x", "clusters": [
        {"failure": "f", "diagnosis": "d", "rules": ["DR1"], "change": "c", "scope": "reshape", "expected_effect": "e"}]}
    runner = RN.ScriptedRunner([RN.AgentResult(structured=brief)])
    report = {"statistics": ["mean"], "z": {"mean": {"Z": 0.5, "S": 10, "J": 1, "fell": 0, "wrong_valley": 7,
                                                     "other": 0, "N": 18}}}
    sim_map = {"grids": [], "box": {"half_widths": {"x_mm": 14.8}}}
    CR.critique(runner, report, sim_map, None, "FAMILY = ...", log.text(), "mean")
    prompt = runner.calls[0].prompt
    assert "steeper ramps at 60 deg" in prompt and "rejected" in prompt and "seed 1" not in prompt
