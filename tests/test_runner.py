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


@pytest.mark.parametrize("source, token, ok", [
    ("ANTHROPIC_API_KEY", False, True), ("apiKeyHelper", False, True),
    ("/login managed key", False, False), ("/login managed key", True, False),
    # Claude Code reports 'none' (no API key) when it authenticates with ANTHROPIC_AUTH_TOKEN, which it
    # says takes precedence over a claude.ai login. 'none' without a token means that login.
    ("none", True, True), ("none", False, False)])
def test_claude_code_must_report_a_credential_the_ledger_tracks(source, token, ok):
    if ok:
        RN.check_source(source, auth_token=token)
    else:
        with pytest.raises(RN.AuthError):
            RN.check_source(source, auth_token=token)


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


def test_a_call_that_raises_is_still_charged_its_whole_budget(tmp_path, monkeypatch):
    """If the SDK fails mid-call, what it spent is unknown, so the ledger assumes the worst."""
    led = RN.SpendLedger(tmp_path / "spend.jsonl", cap_usd=1.0)
    runner = RN.ClaudeRunner("m", led, environ={"ANTHROPIC_API_KEY": "k"})

    async def boom(call, env):
        raise RuntimeError("connection lost")

    monkeypatch.setattr(runner, "_run", boom)
    with pytest.raises(RuntimeError, match="connection lost"):
        runner.run(_call(max_budget_usd=0.3))
    assert led.spent() == pytest.approx(0.3) and led.rows()[0]["is_error"] is True


def test_every_tool_call_goes_through_the_guard_as_a_pre_tool_use_hook(tmp_path):
    """allowed_tools would approve a tool before a permission callback is asked, so the guard runs
    as a PreToolUse hook, which sees every call, read-only ones included."""
    guard = RN.ToolGuard(write_root=tmp_path / "pkg", read_roots=(tmp_path / "repo",))
    deny = RN.hook_decision(guard, {"tool_name": "Write", "tool_input": {"file_path": "/etc/x"}})
    assert deny["hookSpecificOutput"]["permissionDecision"] == "deny"
    assert "writes are allowed only" in deny["hookSpecificOutput"]["permissionDecisionReason"]
    read = RN.hook_decision(guard, {"tool_name": "Read", "tool_input": {"file_path": "/etc/passwd"}})
    assert read["hookSpecificOutput"]["permissionDecision"] == "deny"
    assert RN.hook_decision(guard, {"tool_name": "Read", "tool_input": {"file_path": str(tmp_path / "repo" / "a")}}) == {}


def test_the_sdks_own_structured_output_tool_is_never_blocked(tmp_path):
    """A structured answer comes back through Claude Code's StructuredOutput tool; the critic's
    wiring check found the guard refusing it (2026-11-12)."""
    guard = RN.ToolGuard(write_root=None, read_roots=(), tools=())
    assert guard.decide("StructuredOutput", {"decision": "stop"})[0]
    assert RN.hook_decision(guard, {"tool_name": "StructuredOutput", "tool_input": {}}) == {}


def test_the_sdk_gets_a_message_buffer_big_enough_for_an_image_result(tmp_path):
    """A concept photo read by the agent came back as a 1.15 MB message, over the SDK's 1 MB default
    (issue #79)."""
    opts = RN.option_kwargs(_call(), model="m", remaining_usd=1.0, env={})
    assert opts["max_buffer_size"] >= 16 * 1024 * 1024


def test_a_reservation_holds_its_budget_until_the_call_is_recorded(tmp_path):
    """Design calls run side by side in an explore round (#106): a call in flight must count against the cap."""
    from brixzle.agents.runner import AgentResult, BudgetExceeded, SpendLedger

    led = SpendLedger(tmp_path / "spend.jsonl", 3.0)
    led.reserve(2.0)
    with pytest.raises(BudgetExceeded):
        led.reserve(2.0)
    led.record("design", "m", AgentResult(cost_usd=0.5), budget_usd=2.0)
    led.reserve(2.0)  # 0.5 spent, 2.0 held: 2.5 of 3.0
    assert led.remaining() == pytest.approx(0.5)


def test_calls_side_by_side_each_get_their_own_budget_as_the_sdk_cap(tmp_path):
    """overhaul-20261114-mean (#123): three explore calls held $4.50 of a $5 cap and each was given
    the $0.42 left as its SDK cap, so all three stopped."""
    led = RN.SpendLedger(tmp_path / "spend.jsonl", 5.0)
    led.record("critic", "m", RN.AgentResult(cost_usd=0.08), budget_usd=0.4)
    runner = RN.ClaudeRunner("m", led)
    calls = [_call(max_budget_usd=1.5) for _ in range(3)]
    for c in calls:
        led.reserve(c.max_budget_usd)
    assert [runner.sdk_cap(c) for c in calls] == [1.5, 1.5, 1.5]


def test_a_call_stopped_at_its_budget_is_charged_what_the_sdk_reported():
    err = "ResultError: Claude Code returned an error result: Reached maximum budget ($0.41813520000000004) (exit code: 1)"
    assert RN.budget_stop_cost(err) == pytest.approx(0.4181352)
    assert RN.budget_stop_cost("RuntimeError: something else") is None


# --- persistent sessions (#125) ---


class ResultMessage:  # named like the SDK's, which the session reads by duck typing
    def __init__(self, k, sid="sess-1"):
        self.total_cost_usd, self.num_turns, self.session_id, self.is_error = 0.01 * (k + 1), 2, sid, False
        self.usage = {"input_tokens": 10, "cache_read_input_tokens": 1000 * (k + 1), "cache_creation_input_tokens": 50}
        self.structured_output, self.errors, self.result = None, [], f"answer {k}"


class AssistantMessage:  # one turn's event, with its own usage, as the SDK's
    def __init__(self, cached, message_id=None):
        self.content, self.usage = [], {"input_tokens": 5, "cache_read_input_tokens": cached,
                                        "cache_creation_input_tokens": 20}
        self.message_id = message_id


class FakeClient:
    """Stands in for ClaudeSDKClient: one connect, many queries, one disconnect."""
    made = []

    def __init__(self, opts):
        self.opts, self.prompts, self.closed = opts, [], False
        FakeClient.made.append(self)

    async def __aenter__(self):
        return self

    async def __aexit__(self, *exc):
        self.closed = True

    async def query(self, prompt):
        self.prompts.append(prompt)

    async def receive_response(self):
        k = len(self.prompts) - 1
        yield AssistantMessage(300 * (k + 1))   # the context grows turn by turn
        yield AssistantMessage(500 * (k + 1))
        yield ResultMessage(k)


def test_a_session_connects_once_answers_every_message_and_records_each_in_the_ledger(tmp_path):
    FakeClient.made.clear()
    led = RN.SpendLedger(tmp_path / "spend.jsonl", 5.0)
    base = _call(tools=("Read", "Write"), write_root=tmp_path / "a", max_budget_usd=1.0)
    s = RN.ClaudeSession("m", led, "designer", base, environ={"ANTHROPIC_API_KEY": "k"}, client_factory=FakeClient)
    r1 = s.send(base)
    r2 = s.send(_call(prompt="again", tools=("Read", "Write"), write_root=tmp_path / "b", max_budget_usd=1.0))
    s.close()
    assert len(FakeClient.made) == 1 and FakeClient.made[0].prompts == ["hi", "again"] and FakeClient.made[0].closed
    assert (r1.text, r2.text) == ("answer 0", "answer 1") and s.session_id == "sess-1" and s.messages == 2
    assert s.guard.write_root == tmp_path / "b"  # the second message's package, not the first's
    # The context is the last turn's input, not the message's usage summed over its turns
    # (sessions-20261115-mean: summed, a 19-turn message read as 1.57 million tokens and the session
    # was replaced before its second message).
    assert s.context_tokens == 5 + 1000 + 20
    rows = led.rows()
    # The SDK's cost is the session's running total (#131): $0.01 then $0.02 means $0.01 each.
    assert [r["session"] for r in rows] == ["designer", "designer"] and [r["cost_usd"] for r in rows] == [0.01, 0.01]
    assert (r1.cost_usd, r2.cost_usd) == (0.01, 0.01)
    assert rows[1]["usage"]["cache_read_input_tokens"] == 2000


def test_a_session_reopens_a_saved_session_by_its_id(tmp_path):
    FakeClient.made.clear()
    led = RN.SpendLedger(tmp_path / "spend.jsonl", 5.0)
    s = RN.ClaudeSession("m", led, "critic", _call(), resume="sess-old", environ={"ANTHROPIC_API_KEY": "k"},
                         client_factory=FakeClient)
    s.send(_call())
    s.close()
    assert FakeClient.made[0].opts["resume"] == "sess-old"


def test_a_scripted_session_marks_its_calls_with_the_session():
    runner = RN.ScriptedRunner([RN.AgentResult(text="a"), RN.AgentResult(text="b")])
    s = runner.session("designer", _call())
    s.send(_call())
    s.send(_call(prompt="again"))
    assert [c.session for c in runner.calls] == ["designer", "designer"] and s.messages == 2
    assert len({c.session_id for c in runner.calls}) == 1


def test_a_session_s_sdk_budget_covers_all_its_messages(tmp_path):
    """long-20261116-mean (#131): the $1.50 of one message capped the whole session, and its third
    message was stopped after one turn."""
    FakeClient.made.clear()
    led = RN.SpendLedger(tmp_path / "spend.jsonl", 9.0)
    s = RN.ClaudeSession("m", led, "designer", _call(max_budget_usd=1.5), environ={"ANTHROPIC_API_KEY": "k"},
                         client_factory=FakeClient, messages=4)
    s.send(_call(max_budget_usd=1.5))
    s.close()
    assert FakeClient.made[0].opts["max_budget_usd"] == 6.0



# --- per-message limits (#139) ---


class LongClient(FakeClient):
    """Keeps taking turns until interrupted (as the explore calls of gate2-20261116-mean, 50 to 55 turns)."""

    def __init__(self, opts):
        super().__init__(opts)
        self.interrupted = False

    async def interrupt(self):
        self.interrupted = True

    async def receive_response(self):
        for k in range(200):
            if self.interrupted:
                break
            yield AssistantMessage(100 * (k + 1))
        yield ResultMessage(0)


def test_a_message_past_its_turns_is_interrupted_and_still_recorded(tmp_path):
    LongClient.made = []
    led = RN.SpendLedger(tmp_path / "spend.jsonl", 9.0)
    s = RN.ClaudeSession("m", led, "designer", _call(max_turns=25), environ={"ANTHROPIC_API_KEY": "k"},
                         client_factory=LongClient, message_seconds=600)
    r = s.send(_call(max_turns=25))
    s.close()
    assert r.interrupted is True and "25 turns" in r.interrupt_reason
    assert len(led.rows()) == 1 and led.rows()[0]["interrupted"] is True


def test_a_message_past_its_time_is_interrupted(tmp_path):
    import asyncio

    class SlowClient(LongClient):
        async def receive_response(self):
            for k in range(200):
                if self.interrupted:
                    break
                await asyncio.sleep(0.02)
                yield AssistantMessage(100)
            yield ResultMessage(0)

    led = RN.SpendLedger(tmp_path / "spend.jsonl", 9.0)
    s = RN.ClaudeSession("m", led, "designer", _call(max_turns=1000), environ={"ANTHROPIC_API_KEY": "k"},
                         client_factory=SlowClient, message_seconds=0.2)
    r = s.send(_call(max_turns=1000))
    s.close()
    assert r.interrupted is True and "s" in r.interrupt_reason


def test_a_message_within_its_limits_is_not_interrupted(tmp_path):
    FakeClient.made.clear()
    led = RN.SpendLedger(tmp_path / "spend.jsonl", 9.0)
    s = RN.ClaudeSession("m", led, "designer", _call(max_turns=25), environ={"ANTHROPIC_API_KEY": "k"},
                         client_factory=FakeClient)
    r = s.send(_call(max_turns=25))
    s.close()
    assert r.interrupted is False


# --- turns as the SDK counts them, and a wrap-up at the limit (#141) ---


class ThreeEventsPerTurn(LongClient):
    """Each turn comes as three events (text and two tool calls) with one message id, as the SDK sends them."""

    async def receive_response(self):
        n = len(self.prompts)
        if n > 1:  # the wrap-up message: one short turn
            yield AssistantMessage(100, message_id="wrap")
            yield ResultMessage(0)
            return
        for k in range(30):
            if self.interrupted:
                break
            for _ in range(3):
                yield AssistantMessage(100, message_id=f"turn-{k}")
        yield ResultMessage(0)


def test_turns_are_counted_by_message_id_and_a_call_at_its_limit_wraps_up(tmp_path):
    FakeClient.made.clear()
    led = RN.SpendLedger(tmp_path / "spend.jsonl", 9.0)
    call = _call(max_turns=20, wrapup="You are at your limit: save design_note.json now and stop.")
    s = RN.ClaudeSession("m", led, "designer", call, environ={"ANTHROPIC_API_KEY": "k"},
                         client_factory=ThreeEventsPerTurn)
    r = s.send(call)
    s.close()
    client = FakeClient.made[-1]
    assert r.interrupted is True and "20 turns" in r.interrupt_reason  # 20 turns, not 7 (= 21 events / 3)
    assert len(client.prompts) == 2 and "design_note.json" in client.prompts[1]
    assert r.wrapped_up is True

