"""Design agent (issue #18): prompt, workspace, tools, and the family it leaves behind.

The agent is scripted: a scripted step writes the family files the way the agent would, so no
test calls the API. One test drops a family loaded by path in two worker processes, to show the
sim can import it. Sim results are simulated, uncalibrated contact.
"""

import json
import re
from pathlib import Path

import pytest

from brixzle.agents import check_family as CF
from brixzle.agents import design as DA
from brixzle.agents import runner as RN

REPO = Path(__file__).resolve().parents[1]
WEDGE = (REPO / "src/brixzle/families/wedge/family.py").read_text(encoding="utf-8")


def _intake(tmp_path, extras=(), fixed=None):
    img = tmp_path / "concept" / "sketch.png"
    img.parent.mkdir(parents=True, exist_ok=True)
    img.write_bytes(b"\x89PNG\r\n\x1a\n")
    return {"concept_images": [{"abs_path": str(img), "name": "sketch.png"}], "extras": list(extras),
            "fixed": fixed or {}}


def write_wedge(name="agentwedge", note=True):
    """A scripted design step: the dummy wedge family under a new name, and a design note."""

    def step(call):
        pkg = Path(call.write_root)
        (pkg / "family.py").write_text(WEDGE.replace('name="wedge"', f'name="{name}"'), encoding="utf-8")
        if note:
            (pkg / DA.NOTE_FILE).write_text(json.dumps({"name": name, "summary": "a chevron", "changes": "first",
                                                        "geometry": "Prismatic"}), encoding="utf-8")
        return RN.AgentResult(text="done", cost_usd=0.05, turns=4)

    return step


def test_the_system_prompt_holds_the_requirements_the_interface_and_the_rules_of_the_game():
    text = DA.system_prompt()
    assert "R11" in text and "deferred" in text            # base requirements, R12 marked deferred
    assert "## Two ways to describe the geometry" in text  # the family interface
    assert "Prismatic" in text and "Solid3D only if" in text and "solid3d_reason" in text
    assert "never copy" in text
    assert "Z = (seated - jammed)" in text


def test_the_task_prompt_holds_the_concept_extras_brief_failures_and_fixed_cell(tmp_path):
    ws = DA.Workspace.make(tmp_path / "a1", "rsi_x_i01_a1")
    extras = [{"id": "X1", "kind": "refines", "text": "under 15 g", "resolution": None},
              {"id": "X2", "kind": "conflicts", "text": "40 g", "resolution": {"choice": "drop_extra", "text": ""}}]
    brief = {"decision": "redesign", "reason": "jams", "clusters": [{"rules": ["DR2"], "change": "widen"}]}
    text = DA.task_prompt(ws, _intake(tmp_path, extras), brief=brief, parent=tmp_path / "best" / "family.py",
                          failures={"seated": 20}, log_text="iter 1: kept", fixed={"U_mm": 25.0})
    assert str(tmp_path / "concept" / "sketch.png") in text and str(ws.pkg) in text
    assert "under 15 g" in text and "40 g" not in text      # a dropped extra is left out
    assert "DR2" in text and "seated" in text and "iter 1: kept" in text
    assert "fixed at intake: 25.0 mm" in text and str(tmp_path / "best" / "family.py") in text


def test_a_designed_family_is_checked_by_the_loop_and_its_note_kept(tmp_path):
    runner = RN.ScriptedRunner([write_wedge()])
    out = DA.design(runner, _intake(tmp_path), tmp_path / "iter-01" / "attempt-1", "rsi_t_i01_a1")
    assert out.ok and out.check["family"] == "agentwedge" and out.check["geometry"] == "Prismatic"
    assert out.check["parts"]["A"]["broken"] == [] and out.note["summary"] == "a chevron"
    call = runner.calls[0]
    assert call.write_root == out.workspace.pkg and set(call.tools) == {"Read", "Glob", "Grep", "Write", "Edit"}
    assert [t.name for t in call.mcp_tools] == [DA.CHECK_TOOL] and call.max_budget_usd == DA.DESIGN_BUDGET_USD
    fam = CF.load_family(out.workspace.root, out.workspace.module)
    assert fam.name == "agentwedge"


def test_the_check_tool_reports_to_the_agent_without_raising(tmp_path):
    ws = DA.Workspace.make(tmp_path / "a1", "rsi_t_i01_a2")
    tool = DA.check_tool(ws)
    first = json.loads(tool.respond({})["content"][0]["text"])  # nothing written yet
    assert first["ok"] is False and "import failed" in first["error"]
    (ws.pkg / "family.py").write_text(WEDGE.replace('name="wedge"', 'name="w2"'), encoding="utf-8")
    second = json.loads(tool.respond({})["content"][0]["text"])
    assert second["ok"] is True and second["violations"] == []


def test_a_broken_family_fails_the_check_with_what_to_fix(tmp_path):
    def step(call):
        Path(call.write_root, "family.py").write_text(
            WEDGE.replace('name="wedge"', 'name="bad"').replace('"U": (18.0, 32.0)', '"U": (30.0, 32.0)'),
            encoding="utf-8")  # the default U (25) now lies outside its bounds
        return RN.AgentResult(cost_usd=0.01)

    out = DA.design(RN.ScriptedRunner([step]), _intake(tmp_path), tmp_path / "a", "rsi_t_i01_a3")
    assert not out.ok and any("bounds" in v for v in out.check["violations"])


def test_a_family_that_crashes_python_is_reported_not_raised(tmp_path):
    def step(call):
        Path(call.write_root, "family.py").write_text("def broken(:\n", encoding="utf-8")
        return RN.AgentResult(cost_usd=0.0)

    out = DA.design(RN.ScriptedRunner([step]), _intake(tmp_path), tmp_path / "a", "rsi_t_i01_a4")
    assert not out.ok and "SyntaxError" in out.check["error"]


def test_a_family_loaded_by_path_can_be_dropped_in_worker_processes(tmp_path):
    from brixzle.engine import boxes as BX
    from brixzle.engine import quick_check as QC

    out = DA.design(RN.ScriptedRunner([write_wedge("wpool")]), _intake(tmp_path), tmp_path / "a", "rsi_t_i01_a5")
    fam = CF.load_family(out.workspace.root, out.workspace.module)
    res = QC.quick_check(fam, fam.defaults, BX.default_boxes()["placement"]["mean"], n=4, need=0, workers=2)
    assert res["dropped"] == 4 and res["workers"] == 2


def test_the_check_drops_the_family_at_the_aim_on_each_course(tmp_path):
    """A design that cannot seat even at the aim shows up in the agent's own check (issue #70)."""
    out = DA.design(RN.ScriptedRunner([write_wedge("waim")]), _intake(tmp_path), tmp_path / "a", "rsi_t_i01_a6")
    aim = out.check["aim_drops"]
    assert [d["course"] for d in aim] == [0, 1]
    assert all(d["outcome"] == "seated" for d in aim), aim
    assert all(len(d["err_mm"]) == 3 for d in aim)


def test_the_prompts_show_only_a_b_as_an_example_family(tmp_path):
    """The agent copied the dummy wedge as its brick (smoke runs of #21), so the dummies are not
    shown to it (issue #73)."""
    ws = DA.Workspace.make(tmp_path / "a1", "rsi_x_i01_a9")
    task = DA.task_prompt(ws, _intake(tmp_path))
    for whole in (DA.system_prompt(), task):
        # The design rules (in the prompt since #88) use "wedge" for the mechanics (Whitney's wedging),
        # not for the dummy family, so the name check reads the text outside them; no family path anywhere.
        assert not re.search(r"families/(wedge|pyramid)", whole)
        text = whole.split("Design rules (rules/design_rules.md", 1)[0]
        assert not re.search(r"\b(wedge|pyramid)\b", text, re.I), re.search(r".{60}\b(wedge|pyramid)\b.{60}", text, re.I)
    assert "families/ab/family.py" in task


def test_the_agent_cannot_read_the_dummy_families(tmp_path):
    runner = RN.ScriptedRunner([write_wedge("readcheck")])
    out = DA.design(runner, _intake(tmp_path), tmp_path / "a", "rsi_t_i01_a7")
    call = runner.calls[0]
    guard = RN.ToolGuard(call.write_root, tuple(call.read_roots), tuple(call.tools))
    fam = REPO / "src" / "brixzle" / "families"
    assert not guard.decide("Read", {"file_path": str(fam / "wedge" / "family.py")})[0]
    assert not guard.decide("Read", {"file_path": str(fam / "pyramid" / "family.py")})[0]
    for ok in (fam / "ab" / "family.py", fam / "interface.py", REPO / "docs" / "family-interface.md",
               out.workspace.pkg / "family.py"):
        assert guard.decide("Read", {"file_path": str(ok)})[0], ok


def test_the_prompts_let_the_agent_build_on_a_b_but_not_copy_papers(tmp_path):
    """The maintainer's own designs may be reused (2026-11-12, issue #85); papers and other groups'
    systems may not."""
    ws = DA.Workspace.make(tmp_path / "a1", "rsi_x_i01_a8")
    system, task = DA.system_prompt(), DA.task_prompt(ws, _intake(tmp_path))
    for text in (system, task):
        assert "not a design to copy" not in text and "do not copy its shape" not in text
    assert "maintainer's own design" in system and "brixzle.families.ab" in system
    assert "never copy a brick, joint or structure from a paper or from another group's system" in system


def test_the_design_prompt_says_the_error_is_a_disc():
    """Issue #86: a Y guide can only centre a brick whose centre of mass is still over the course below."""
    text = DA.system_prompt()
    assert "disc in X-Y" in text and "half the depth" in text and "depth costs mass" in text


def test_the_design_prompt_says_a_brick_must_build_the_structures():
    """Issue #83: seat from a coarse drop and build a wall, a bridge, a ring and an overhang, held by an interlock."""
    text = DA.system_prompt()
    assert "The goal is placement (R11)" not in text
    for want in ("running-bond wall", "a bridge", "a ring", "an overhang", "straight lift cannot free",
                 "is rejected", "longer overhang is better"):
        assert want in text, want
