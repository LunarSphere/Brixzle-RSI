"""Critic agent (issue #19): a valid brief that cites the design rules, from a recorded report with jams.

The critic is scripted, so no test calls the API. The recorded inputs are made-up but shaped like
the engine's (a schema-4 report, a map with jam cells, a jam analysis with a band).
"""

import json

import pytest

from brixzle.agents import critic as CR
from brixzle.agents import runner as RN
from brixzle.engine import boxes as BX

BOX = BX.default_boxes()["placement"]["mean"]
REPORT = {"statistics": ["mean", "p95", "max"], "z": {
    "mean": {"Z": 0.848, "S": 154, "J": 3, "fell": 0, "wrong_valley": 21, "other": 0, "N": 178},
    "p95": {"Z": 0.336, "S": 206, "J": 56, "fell": 0, "wrong_valley": 184, "other": 0, "N": 446},
    "max": {"Z": 0.114, "S": 206, "J": 135, "fell": 2, "wrong_valley": 279, "other": 0, "N": 622}}}
GRID = {"slice": "xz", "course": 1, "axes": ["x", "z"], "step": {"x": 1.0, "z": 5.0},
        "index_extent": {"x": [-3, 3], "z": [0, 1]}, "extent": {"x": [-3.0, 3.0], "z": [0.0, 5.0]},
        "rows": ["WSSJSSJ", "WSSSSSJ"], "fixed": {"t": 0.0}}
MAP = {"grids": [GRID], "box": BOX.to_dict()}
JAM = {"family": "ab", "box": BOX.to_dict(), "seconds": 3.0, "one_offs": [],
       "counts": {"inside_failures": 1, "repeatable": 1, "bands": 1, "three_d": 0, "reruns": 12, "capture_limits": 0},
       "bands": [{"id": 0, "slice": "xz", "course": 1, "extent": {"x": [0.0, 0.0], "z": [0.0, 0.0]}, "cells": [[0, 0]],
                  "failure_type": "jammed", "repeat_rate": 0.67, "per_mu": {"0.3": 0.67}, "mechanism":
                  "two contacts, both sticking, wedged (Whitney 1982)", "sites": [], "holding": None, "weight_N": 0.1}],
       "capture_limits": []}
GOOD = {"decision": "redesign", "reason": "a wedging band and wrong valleys at both X edges", "clusters": [
    {"failure": "jam band on course 1 at x 0", "diagnosis": "two-point contact wedges", "rules": ["DR2", "DR10"],
     "change": "lengthen the engaging face", "scope": "reshape", "expected_effect": "J falls by about 3"},
    {"failure": "wrong valley past +-12 mm", "diagnosis": "a stable rest away from the seat", "rules": ["DR5"],
     "change": "slope the outer ledge", "scope": "reshape", "expected_effect": "more of the box seats"}]}


def _run(answer):
    runner = RN.ScriptedRunner([RN.AgentResult(structured=answer, cost_usd=0.03)])
    brief, result = CR.critique(runner, REPORT, MAP, JAM, "FAMILY = Family(...)", "none", "mean")
    return brief, result, runner.calls[0]


def test_a_recorded_report_with_jams_gives_a_valid_brief_that_cites_rules():
    brief, result, call = _run(GOOD)
    assert brief == GOOD and result.cost_usd == 0.03
    assert all(c["rules"] for c in brief["clusters"])


def test_the_critic_reads_z_the_map_the_jam_analysis_the_code_and_the_rules():
    _, _, call = _run(GOOD)
    assert "Z +0.848" in call.prompt and "wrong valley 279" in call.prompt
    assert "WSSJSSJ" in call.prompt                                   # the map grid
    assert "wedged (Whitney 1982)" in call.prompt                      # the jam analysis
    assert "FAMILY = Family(...)" in call.prompt
    assert "## DR2." in call.system and "whitney-1982" in call.system  # the rules and their sources
    assert call.tools == () and call.output_schema == CR.BRIEF_SCHEMA


def test_a_stop_needs_only_a_reason():
    brief, _, _ = _run({"decision": "stop", "reason": "Z is 1 on the target", "clusters": []})
    assert brief["decision"] == "stop"


@pytest.mark.parametrize("bad", [
    {**GOOD, "clusters": [{**GOOD["clusters"][0], "rules": ["DR99"]}]},   # a rule that does not exist
    {**GOOD, "clusters": [{**GOOD["clusters"][0], "rules": []}]},         # no rule cited
    {**GOOD, "clusters": []},                                              # a redesign with nothing to do
    {**GOOD, "clusters": [{**GOOD["clusters"][0], "scope": "tweak"}]},    # an unknown scope
    {**GOOD, "decision": "maybe"},
    {**GOOD, "reason": " "},
])
def test_a_malformed_brief_is_refused(bad):
    with pytest.raises(ValueError):
        _run(bad)


def test_a_brief_given_as_text_is_parsed():
    runner = RN.ScriptedRunner([RN.AgentResult(text=json.dumps(GOOD), structured=None)])
    brief, _ = CR.critique(runner, REPORT, MAP, None, "", "", "mean")
    assert brief["clusters"][0]["rules"] == ["DR2", "DR10"]
