"""The loop driven from Python with Codex roles (#182): no Claude, the same steps and bounds."""
import json
from pathlib import Path

import pytest

from brixzle.director import drive as DR

ROOT = Path(__file__).resolve().parents[1]


def test_each_role_has_a_model_and_an_effort_and_the_file_overrides_them(tmp_path):
    roles = DR.load_roles()
    assert roles["designer"] == {"model": "gpt-6.2-astra", "effort": "high"}
    assert set(roles) == {"designer", "verifier", "critic", "director"}
    f = tmp_path / "roles.json"
    f.write_text(json.dumps({"verifier": {"model": "gpt-6.2-astra", "effort": "low"}}))
    assert DR.load_roles(f)["verifier"] == {"model": "gpt-6.2-astra", "effort": "low"}
    with pytest.raises(ValueError):
        DR.load_roles(tmp_path / "nope.json") if False else DR.check_roles({"designer": {"model": "m", "effort": "huge"}})


def test_the_instructions_come_from_the_role_files():
    text = DR.role_instructions(ROOT, "designer")
    assert text.startswith("You are the **designer**") and "---" not in text.splitlines()[0]


def test_the_codex_command_keeps_the_sandbox_and_the_checkout_read_only(tmp_path):
    run = tmp_path / "runs" / "rsi" / "r"
    cmd = DR.codex_command({"model": "gpt-6.2-astra", "effort": "high"}, workdir=run, add_dirs=[tmp_path / "runs"],
                           out_file=run / "o.txt", prompt="p")
    assert cmd[:3] == ["codex", "exec", "--approve-for-me"]
    assert "-m" in cmd and cmd[cmd.index("-m") + 1] == "gpt-6.2-astra"
    assert 'model_reasoning_effort="high"' in cmd and cmd[cmd.index("-C") + 1] == str(run)
    assert not any("bypass" in c or c == "--sandbox" for c in cmd)
    env = DR.role_env(run)
    assert env["BRIXZLE_DROP_CACHE"].startswith(str(run)) and env["BRIXZLE_STAGE_CACHE"].startswith(str(run))
    assert env["MPLCONFIGDIR"].startswith(str(run))


def test_a_prompt_names_the_role_its_inputs_and_what_it_may_write(tmp_path):
    p = DR.role_prompt(ROOT, "designer", run=tmp_path / "r", iteration=2, refusal="changed outside the brief")
    assert "iter-02/brief.json" in p and "uv run --no-sync" in p and "changed outside the brief" in p
    assert "You are the **designer**" in p
    d = DR.role_prompt(ROOT, "director", run=tmp_path / "r", iteration=0)
    assert "iter-00/decision.json" in d and "STATE.md" in d


class Fake:
    """A scripted director CLI and codex: records calls, answers from a plan."""

    def __init__(self, run, plan):
        self.run, self.plan, self.calls = Path(run), plan, []

    def cli(self, args):
        self.calls.append(("cli", args[0], args[args.index("--iter") + 1] if "--iter" in args else None))
        answers = self.plan.get(args[0], [])
        rc, out = answers.pop(0) if answers else (0, {})
        if args[0] == "decide" and rc == 0:
            n = int(args[args.index("--iter") + 1])
            d = json.loads((self.run / f"iter-{n:02d}" / "decision.json").read_text())
            nxt = {"step": "finish"} if d["action"] == "stop" else {"step": "design", "iteration": n + 1}
            (self.run / "next.json").write_text(json.dumps(nxt))
        if args[0] == "finish":
            (self.run / "next.json").write_text(json.dumps({"step": "finish"}))
        return rc, out

    def role(self, role, iteration, prompt):
        self.calls.append(("role", role, iteration))
        d = self.run / f"iter-{iteration:02d}"
        d.mkdir(parents=True, exist_ok=True)
        if role == "director":
            action = self.plan.get("actions", ["stop"]).pop(0)
            (d / "decision.json").write_text(json.dumps({"action": action}))
        return 0


def _driver(tmp_path, plan, minutes_left=True):
    run = tmp_path / "run"
    run.mkdir()
    (run / "next.json").write_text(json.dumps({"step": "design", "iteration": 0}))
    f = Fake(run, plan)
    plan.setdefault("status", [])
    d = DR.Driver(run, checkout=ROOT, call_cli=f.cli, call_role=f.role,
                  budget=lambda: {"exhausted": not minutes_left})
    return d, f


def test_one_iteration_runs_every_step_in_order(tmp_path):
    d, f = _driver(tmp_path, {"actions": ["push", "stop"]})
    d.run()
    order = [c[1] for c in f.calls]
    assert order == ["baseline", "director", "decide",
                     "designer", "check", "verify", "verifier", "critic", "director", "decide", "guard",
                     "finish"]


def test_a_refused_change_goes_back_to_the_designer_once(tmp_path):
    d, f = _driver(tmp_path, {"actions": ["push", "stop"], "check": [(1, {"reason": "changed outside the brief"}), (0, {})]})
    d.run()
    order = [c[1] for c in f.calls]
    assert order.count("designer") == 2 and order.count("check") == 2


def test_tracked_changes_end_the_run(tmp_path):
    d, f = _driver(tmp_path, {"actions": ["push", "push"], "guard": [(1, {"tracked_changes": ["docs/requirements.md"]})]})
    d.run()
    assert f.calls[-1][1] == "finish" and [c[1] for c in f.calls].count("designer") == 1
    assert "tracked" in d.reason


def test_an_exhausted_budget_stops_before_designing(tmp_path):
    d, f = _driver(tmp_path, {"actions": ["push"]}, minutes_left=False)
    d.run()
    assert "designer" not in [c[1] for c in f.calls] and f.calls[-1][1] == "finish"


def test_the_stage_cache_follows_its_environment_variable(tmp_path, monkeypatch):
    from brixzle.tools import core as TC
    monkeypatch.delenv("BRIXZLE_STAGE_CACHE", raising=False)
    assert TC.stage_cache_dir() == TC.STAGE_CACHE
    monkeypatch.setenv("BRIXZLE_STAGE_CACHE", str(tmp_path / "sc"))
    assert TC.stage_cache_dir() == tmp_path / "sc"
