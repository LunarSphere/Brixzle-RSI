"""Lessons carried between iterations and runs (issue #88): the critic proposes them, the loop
attaches the next design's outcome, and both agents read them. No simulation, no API call."""

import json
import math

import pytest

from brixzle.agents import critic as CR
from brixzle.loop import lessons as LS

LESSON = {"lesson": "A wider pitch lets the capture window cover the box.", "rules": ["DR7"], "cluster": 0}


def _rec(i, decision, z, target="mean", family="zigzag_w", map_dir=None):
    return {"iteration": i, "decision": decision, "family": family, "target": target,
            "z": None if z is None else {target: {"Z": z}}, "runs": {"map": map_dir or f"runs/map-{i}"}}


def _brief(*lessons):
    return {"decision": "redesign", "reason": "r", "clusters": [{"failure": "f", "diagnosis": "d", "rules": ["DR7"],
                                                                 "change": "c", "scope": "reshape",
                                                                 "expected_effect": "e"}],
            "lessons": list(lessons)}


def test_a_lesson_followed_by_a_kept_design_is_written_as_supported(tmp_path):
    path = tmp_path / "lessons.jsonl"
    before = {**_rec(1, "kept", 0.909), "brief": _brief(LESSON)}
    written = LS.record(path, "smoke3", before, _rec(2, "kept", 1.0), today="2026-11-13")
    (line,) = [json.loads(x) for x in path.read_text().splitlines()]
    assert written == [line] and line["id"] == "L0001" and line["status"] == "supported"
    assert line["lesson"] == LESSON["lesson"] and line["rules"] == ["DR7"] and line["family"] == "zigzag_w"
    (ev,) = line["evidence"]
    assert ev == {"run": "smoke3", "iteration": 2, "target": "mean", "z_before": 0.909, "z_after": 1.0,
                  "decision": "kept", "map_before": "runs/map-1", "map_after": "runs/map-2"}
    assert line["label"] == "simulated, uncalibrated contact; unreviewed" and line["date"] == "2026-11-13"


def test_a_rejected_design_gives_not_supported_and_no_design_writes_nothing(tmp_path):
    path = tmp_path / "lessons.jsonl"
    before = {**_rec(1, "kept", 0.5), "brief": _brief(LESSON)}
    LS.record(path, "r", before, _rec(2, "rejected", 0.4))
    assert json.loads(path.read_text())["status"] == "not_supported"
    other = tmp_path / "other.jsonl"
    assert LS.record(other, "r", before, _rec(2, "no_design", None)) == [] and not other.exists()


def test_the_same_lesson_again_adds_evidence_and_can_be_mixed(tmp_path):
    path = tmp_path / "lessons.jsonl"
    LS.record(path, "run-a", {**_rec(1, "kept", 0.5), "brief": _brief(LESSON)}, _rec(2, "kept", 0.7))
    again = {**LESSON, "lesson": "  a WIDER pitch lets the capture   window cover the box. "}
    LS.record(path, "run-b", {**_rec(3, "kept", 0.7), "brief": _brief(again)}, _rec(4, "rejected", 0.6))
    (line,) = [json.loads(x) for x in path.read_text().splitlines()]
    assert [e["run"] for e in line["evidence"]] == ["run-a", "run-b"] and line["status"] == "mixed"
    assert LS.status([{"decision": "kept"}] * 2 + [{"decision": "rejected"}]) == "supported"


def test_only_finite_numbers_are_written(tmp_path):
    with pytest.raises(ValueError, match="finite"):
        LS.record(tmp_path / "l.jsonl", "r", {**_rec(1, "kept", math.nan), "brief": _brief(LESSON)},
                  _rec(2, "kept", 0.5))
    assert not (tmp_path / "l.jsonl").exists()


def test_the_text_lists_most_evidence_first_then_newest(tmp_path):
    path = tmp_path / "lessons.jsonl"
    assert LS.lessons_text(path) == ""  # a missing file gives no section
    other = {**LESSON, "lesson": "Steeper flanks clear the jam band."}
    LS.record(path, "a", {**_rec(1, "kept", 0.5), "brief": _brief(LESSON)}, _rec(2, "kept", 0.7))
    LS.record(path, "b", {**_rec(1, "kept", 0.5), "brief": _brief(other)}, _rec(2, "rejected", 0.4))
    LS.record(path, "c", {**_rec(1, "kept", 0.5), "brief": _brief(other)}, _rec(2, "kept", 0.6))
    text = LS.lessons_text(path)
    assert "unreviewed observations from earlier runs" in text and "prefer the design rules" in text
    assert text.index("Steeper flanks") < text.index("A wider pitch")
    assert "kept 1, rejected 1" in text and "[mixed]" in text and "DR7" in text


@pytest.mark.parametrize("bad, match", [
    ({**LESSON, "rules": ["DR99"]}, "do not exist"),
    ({**LESSON, "lesson": "x" * 301}, "300"),
    ({**LESSON, "cluster": 5}, "cluster"),
    ({**LESSON, "lesson": "  "}, "lesson"),
])
def test_validate_brief_refuses_a_bad_lesson(bad, match):
    with pytest.raises(ValueError, match=match):
        CR.validate_brief(_brief(bad))
    assert CR.validate_brief(_brief(LESSON))["lessons"] == [LESSON]
    assert "lessons" in CR.BRIEF_SCHEMA["properties"]


def test_both_agents_read_the_lessons_and_the_design_agent_the_rules(tmp_path):
    from brixzle.agents import design as DA
    from brixzle import rules as RL

    text = "Lessons from earlier runs: X"
    assert text in DA.system_prompt(lessons_text=text) and text in CR.system_prompt(lessons_text=text)
    assert RL.rules_text() in DA.system_prompt()
    assert "Lessons from earlier runs" not in DA.system_prompt() and "at most two" in CR.system_prompt()
