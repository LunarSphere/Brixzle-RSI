"""The Claude runner (issues #17, #18): spend ledger, credentials, tool guard and the scripted runner.

No test here calls the API or imports the Agent SDK.
"""

import json

import pytest

from brixzle.agents import runner as RN


def _call(**kw):
    base = dict(name="design", system="sys", prompt="hi", max_budget_usd=0.5)
    return RN.AgentCall(**{**base, **kw})


# --- the spend ledger ---


def test_the_ledger_sums_what_was_recorded_and_survives_a_restart(tmp_path):
    path = tmp_path / "spend.jsonl"
    led = RN.SpendLedger(path, cap_usd=1.0)
    assert led.spent() == 0.0 and led.remaining() == 1.0
    led.record("design", "claude-sonnet-5-5", RN.AgentResult(text="x", cost_usd=0.25, turns=3))
    led.record("critic", "claude-sonnet-5-5", RN.AgentResult(text="y", cost_usd=0.10, turns=1))
    again = RN.SpendLedger(path, cap_usd=1.0)
    assert again.spent() == pytest.approx(0.35) and again.remaining() == pytest.approx(0.65)
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert [r["name"] for r in rows] == ["design", "critic"] and rows[0]["model"] == "claude-sonnet-5-5"


def test_a_call_that_could_pass_the_cap_is_refused_before_it_starts(tmp_path):
    led = RN.SpendLedger(tmp_path / "spend.jsonl", cap_usd=1.0)
    led.record("design", "m", RN.AgentResult(text="", cost_usd=0.8))
    led.reserve(0.2)  # exactly up to the cap is allowed
    with pytest.raises(RN.BudgetExceeded, match="cap"):
        led.reserve(0.21)


@pytest.mark.parametrize("cap", [0.0, -1.0, float("nan"), float("inf")])
def test_a_cap_must_be_finite_and_positive(tmp_path, cap):
    with pytest.raises(ValueError):
        RN.SpendLedger(tmp_path / "s.jsonl", cap_usd=cap)


def test_an_unknown_cost_is_recorded_as_the_whole_call_budget(tmp_path):
    """If the SDK reports no cost, the ledger assumes the worst: the call's own cap."""
    led = RN.SpendLedger(tmp_path / "spend.jsonl", cap_usd=1.0)
    led.record("design", "m", RN.AgentResult(text="", cost_usd=None), budget_usd=0.4)
    assert led.spent() == pytest.approx(0.4)


# --- credentials ---


def test_an_api_key_in_the_environment_is_used_as_it_is():
    env = RN.credential_env({"ANTHROPIC_API_KEY": "k"}, ant_token=lambda: pytest.fail("ant asked"))
    assert env == {}


def test_without_a_key_the_ant_profile_token_is_handed_to_claude_code():
    env = RN.credential_env({}, ant_token=lambda: "tok")
    assert env == {"ANTHROPIC_AUTH_TOKEN": "tok", "ANTHROPIC_CUSTOM_HEADERS": RN.OAUTH_BETA}


def test_without_any_credential_the_error_says_what_to_set():
    def none():
        raise FileNotFoundError("ant")

    with pytest.raises(RN.AuthError, match="ANTHROPIC_API_KEY") as e:
        RN.credential_env({}, ant_token=none)
    assert "ant auth login" in str(e.value)


@pytest.mark.parametrize("source, ok", [("ANTHROPIC_API_KEY", True), ("ANTHROPIC_AUTH_TOKEN", True),
                                        ("/login managed key", False), ("none", False)])
def test_claude_code_must_report_a_credential_the_ledger_tracks(source, ok):
    if ok:
        RN.check_source(source)
    else:
        with pytest.raises(RN.AuthError):
            RN.check_source(source)


# --- the tool guard ---


def test_writes_are_allowed_only_inside_the_write_root(tmp_path):
    root = tmp_path / "family"
    guard = RN.ToolGuard(write_root=root, read_roots=(tmp_path,))
    assert guard.decide("Write", {"file_path": str(root / "family.py")})[0]
    assert guard.decide("Edit", {"file_path": str(root / "sub" / "x.py")})[0]
    assert not guard.decide("Write", {"file_path": str(tmp_path / "elsewhere.py")})[0]
    assert not guard.decide("Write", {"file_path": str(root / ".." / "escape.py")})[0]


def test_reads_are_allowed_only_inside_the_read_roots(tmp_path):
    guard = RN.ToolGuard(write_root=tmp_path / "w", read_roots=(tmp_path / "repo",))
    assert guard.decide("Read", {"file_path": str(tmp_path / "repo" / "docs" / "a.md")})[0]
    assert not guard.decide("Read", {"file_path": "/etc/passwd"})[0]
    assert guard.decide("Read", {"file_path": str(tmp_path / "w" / "family.py")})[0]  # its own files


def test_tools_that_are_not_listed_are_refused(tmp_path):
    guard = RN.ToolGuard(write_root=tmp_path, read_roots=(tmp_path,))
    assert not guard.decide("Bash", {"command": "ls"})[0]
    assert not guard.decide("WebFetch", {"url": "https://example.com"})[0]
    assert guard.decide("mcp__brixzle__check_family", {})[0]


# --- the options a call becomes ---


def test_a_call_becomes_isolated_agent_sdk_options(tmp_path):
    spec = RN.ToolSpec("check_family", "check it", {}, lambda args: "ok")
    opts = RN.option_kwargs(_call(tools=("Read", "Write"), cwd=tmp_path, max_turns=7, max_budget_usd=0.6,
                                  mcp_tools=(spec,)),
                            model="claude-sonnet-5-5", remaining_usd=0.4, env={"A": "b"})
    assert opts["model"] == "claude-sonnet-5-5" and opts["max_turns"] == 7
    assert opts["max_budget_usd"] == 0.4          # never more than the ledger has left
    assert opts["setting_sources"] == []          # no user or project settings, no CLAUDE.md
    assert opts["tools"] == ["Read", "Write"] and opts["cwd"] == str(tmp_path)
    assert opts["allowed_tools"] == ["Read", "Write", "mcp__brixzle__check_family"]
    assert opts["env"] == {"A": "b"} and opts["system_prompt"] == "sys"


# --- the scripted runner (what the tests of every agent use) ---


def test_the_scripted_runner_answers_in_order_and_records_spend(tmp_path):
    led = RN.SpendLedger(tmp_path / "spend.jsonl", cap_usd=1.0)
    runner = RN.ScriptedRunner([RN.AgentResult(text="a", cost_usd=0.1), RN.AgentResult(text="b", cost_usd=0.2)],
                               ledger=led)
    assert runner.run(_call()).text == "a" and runner.run(_call(name="critic")).text == "b"
    assert [c.name for c in runner.calls] == ["design", "critic"]
    assert led.spent() == pytest.approx(0.3)
    with pytest.raises(AssertionError, match="no scripted"):
        runner.run(_call())


def test_a_scripted_step_can_act_like_the_agent(tmp_path):
    def step(call):
        (tmp_path / "out.txt").write_text(call.prompt, encoding="utf-8")
        return RN.AgentResult(text="done", cost_usd=0.0)

    RN.ScriptedRunner([step]).run(_call(prompt="written"))
    assert (tmp_path / "out.txt").read_text(encoding="utf-8") == "written"


def test_a_tool_spec_answers_with_text_and_reports_its_errors():
    spec = RN.ToolSpec("t", "d", {"x": "number"}, lambda args: f"got {args['x']}")
    assert spec.respond({"x": 2}) == {"content": [{"type": "text", "text": "got 2"}]}
    broken = RN.ToolSpec("t", "d", {}, lambda args: 1 / 0)
    out = broken.respond({})
    assert out["is_error"] and "ZeroDivisionError" in out["content"][0]["text"]
