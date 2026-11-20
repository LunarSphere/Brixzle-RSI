"""The self-steering loop's deterministic parts (#165): the run's files, the bounds, the one-change
check, the verifier and the director's decisions."""
import json
import subprocess
import sys

import pytest

from brixzle.director import archive as AR
from brixzle.director import change as CH
from brixzle.director import decide as DE
from brixzle.director import guard as GU
from brixzle.director import run as RN
from brixzle.director import verify as VE


@pytest.fixture
def repo(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    for f in ("docs/requirements.md", "src/brixzle/rules/design_rules.md", "README.md"):
        (tmp_path / f).parent.mkdir(parents=True, exist_ok=True)
        (tmp_path / f).write_text("x\n")
    (tmp_path / ".gitignore").write_text("runs/\n.rsi-loop-active\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "-A"], check=True)
    subprocess.run(["git", "-C", str(tmp_path), "-c", "user.email=t@t", "-c", "user.name=t", "commit", "-qm", "init"], check=True)
    return tmp_path


def _init(repo, **kw):
    return RN.init_run(repo / "runs" / "rsi" / "r1", goal="Beat A/B on the #164 objective.", baseline="ab", repo=repo, **kw)


# --- the run ---


def test_init_writes_the_run_and_the_lock_and_refuses_a_second(repo):
    run = _init(repo, budget={"iterations": 3})
    for f in ("goal.json", "STATE.md", "decisions.jsonl", "next.json"):
        assert (run / f).exists()
    lock = json.loads((repo / GU.LOCK).read_text())
    assert lock["run"] == str(run) and lock["head"]
    assert json.loads((run / "next.json").read_text())["step"] == "design"
    with pytest.raises(FileExistsError):
        _init(repo)
    RN.finish(run, repo=repo, reason="test")
    assert not (repo / GU.LOCK).exists() and "test" in (run / "summary.json").read_text()


# --- the bounds ---


def _hook(repo, tool, **inp):
    payload = json.dumps({"tool_name": tool, "tool_input": inp, "cwd": str(repo)})
    return subprocess.run([sys.executable, "-m", "brixzle.director.guard", "hook"], input=payload, text=True,
                          capture_output=True, cwd=repo)


def test_the_hook_allows_everything_without_a_loop(repo):
    assert _hook(repo, "Write", file_path=str(repo / "docs/requirements.md")).returncode == 0


def test_in_a_loop_agents_write_only_inside_the_run(repo):
    run = _init(repo, deny_read=["src/brixzle/rules"])
    assert _hook(repo, "Write", file_path=str(run / "iter-01" / "m" / "family.py")).returncode == 0
    for path in ("docs/requirements.md", "src/brixzle/engine/objective.py", ".claude/agents/rsi-director.md",
                 ".cache/drops.sqlite", str(repo / ".rsi-loop-active")):
        r = _hook(repo, "Edit", file_path=str(repo / path) if not path.startswith("/") else path)
        assert r.returncode == 2 and "outside the run" in r.stderr
    assert _hook(repo, "Read", file_path=str(repo / "src/brixzle/rules/design_rules.md")).returncode == 2
    assert _hook(repo, "Read", file_path=str(repo / "README.md")).returncode == 0
    assert _hook(repo, "Bash", command="echo hi > docs/requirements.md").returncode == 2
    assert _hook(repo, "Bash", command="uv run python -m brixzle.tools slices --map runs/x").returncode == 0


def test_the_tracked_tree_must_not_change_during_a_run(repo):
    run = _init(repo)
    assert GU.changed_tracked(run) == []
    (repo / "docs/requirements.md").write_text("changed\n")
    assert GU.changed_tracked(run) == ["docs/requirements.md"]


# --- one change ---


PARENT = '''from dataclasses import dataclass

@dataclass(frozen=True)
class P:
    a: float = 1.0
    b: float = 2.0

def _hook(p):
    """doc"""
    return p.a * 2

def section(p):
    return _hook(p) + p.b
'''


def _module(root, name, text):
    d = root / name
    d.mkdir(parents=True)
    (d / "__init__.py").write_text("")
    (d / "family.py").write_text(text)
    return d / "family.py"


def test_the_change_check_names_what_changed(tmp_path):
    parent = _module(tmp_path, "par", PARENT)
    one_param = _module(tmp_path, "c1", PARENT.replace("b: float = 2.0", "b: float = 2.5"))
    one_def = _module(tmp_path, "c2", PARENT.replace("return p.a * 2", "return p.a * 3").replace('"""doc"""', '"""new doc"""'))
    two = _module(tmp_path, "c3", PARENT.replace("b: float = 2.0", "b: float = 2.5").replace("return p.a * 2", "return p.a * 3"))
    assert CH.changed(parent, one_param) == ["param:b"]
    assert CH.changed(parent, one_def) == ["def:_hook"]
    assert CH.changed(parent, two) == ["def:_hook", "param:b"]
    assert CH.check(parent, one_param, may_change=["param:b"])["ok"]
    bad = CH.check(parent, two, may_change=["param:b"])
    assert not bad["ok"] and bad["outside"] == ["def:_hook"]
    assert CH.check(parent, two, may_change=["def:_hook", "param:b"])["ok"]


# --- the verifier ---


def test_the_verifier_reruns_every_claimed_number(tmp_path):
    measured = {"z": {"mean": {"Z": 0.95, "S": 171}}, "gate": {"overhang_n": 10}, "tolerance_mm": 2.0}
    claim = {"numbers": [{"what": "Z mean", "path": "z.mean.Z", "value": 0.95},
                         {"what": "overhang", "path": "gate.overhang_n", "value": 11},
                         {"what": "tolerance", "path": "tolerance_mm", "value": 2.0},
                         {"what": "made up", "path": "z.p95.Z", "value": 0.5}]}
    res = VE.compare(claim, measured)
    assert [r["match"] for r in res["numbers"]] == [True, False, True, False]
    assert not res["ok"] and res["mismatches"] == 2 and res["numbers"][3]["measured"] is None


# --- the director's decisions ---


def _decision(**kw):
    d = {"action": "push", "design": "iter-01", "term": "tolerance_mm", "brief": "Fix the reaching brick's seat.",
         "may_change": ["def:_mouth"], "goal_change": None, "evidence": []}
    d.update(kw)
    return d


def test_a_valid_decision_is_logged(repo):
    run = _init(repo)
    out = DE.record(run, _decision())
    assert out["ok"]
    (row,) = [json.loads(l) for l in (run / "decisions.jsonl").read_text().splitlines()]
    assert row["action"] == "push" and row["time"]
    assert json.loads((run / "next.json").read_text())["step"] == "design"
    brief = json.loads((run / "iter-01" / "brief.json").read_text())
    assert brief["parent"] == "iter-01" and brief["may_change"] == ["def:_mouth"] and brief["goal"].startswith("Beat")


@pytest.mark.parametrize("bad, why", [
    (dict(action="tune"), "action"),
    (dict(may_change=[]), "may_change"),
    (dict(may_change=["def:a", "def:b", "def:c", "def:d"]), "may_change"),
    (dict(goal_change={"text": "aim for mass"}, evidence=[]), "evidence"),
    (dict(goal_change={"text": "aim for mass"}, evidence=["runs/does-not-exist"]), "evidence"),
    (dict(goal_change={"settings": {"max_length_mm": 200.0}}, evidence=["EXISTS"]), "loosen"),
])
def test_bad_decisions_are_refused(repo, bad, why):
    run = _init(repo)
    (repo / "runs" / "map-x").mkdir(parents=True)
    bad = {k: ([str(repo / "runs" / "map-x")] if v == ["EXISTS"] else v) for k, v in bad.items()}
    out = DE.record(run, _decision(**bad))
    assert not out["ok"] and why in out["reason"]
    assert (run / "decisions.jsonl").read_text() == ""


def test_a_goal_change_with_evidence_may_tighten(repo):
    run = _init(repo)
    (repo / "runs" / "map-x").mkdir(parents=True)
    out = DE.record(run, _decision(goal_change={"text": "Push error tolerance first.", "settings": {"max_length_mm": 60.0}},
                                   evidence=[str(repo / "runs" / "map-x")]))
    assert out["ok"]
    goal = json.loads((run / "goal.json").read_text())
    assert goal["settings"]["max_length_mm"] == 60.0 and goal["history"][-1]["text"] == "Push error tolerance first."


def test_stop_ends_the_loop(repo):
    run = _init(repo)
    assert DE.record(run, _decision(action="stop", may_change=[], brief="No gain in 3 iterations."))["ok"]
    assert json.loads((run / "next.json").read_text())["step"] == "finish"


# --- the archive ---


def _rec(name, z=0.9, tol=2.0, mu=0.5, ovh=10, feasible=True, n=180):
    return {"name": name, "z": {"mean": z}, "tolerance_mm": tol, "z_mu08": mu, "overhang_n": ovh,
            "violations": [] if feasible else ["grip"], "z_counts": {"N": n, "S": int(n * z), "J": 0}}


def test_the_archive_keeps_the_pareto_set_within_noise():
    a = AR.Archive()
    assert a.offer(_rec("ab", z=0.5, tol=4.0))["kept"]
    assert a.offer(_rec("th4", z=0.97, tol=2.0))["kept"]
    assert not a.offer(_rec("worse", z=0.49, tol=4.0, mu=0.4, ovh=8))["kept"]
    assert not a.offer(_rec("bad", z=1.0, tol=9.0, feasible=False))["kept"]
    out = a.offer(_rec("better", z=0.98, tol=4.0, mu=0.6, ovh=11))
    assert out["kept"] and set(out["replaced"]) == {"ab", "th4"}
    assert [r["name"] for r in a.members] == ["better"]


def test_the_command_line_steps(repo, monkeypatch):
    from brixzle.director import __main__ as CLI
    monkeypatch.chdir(repo)
    run = repo / "runs" / "rsi" / "cli"
    assert CLI.main(["init", "--run", str(run), "--goal", "g", "--iterations", "2"]) == 0
    d = run / "iter-01"
    (d / "m").mkdir(parents=True)
    (d / "m" / "__init__.py").write_text("")
    parent = _module(repo / "runs", "par", PARENT)
    (d / "m" / "family.py").write_text(PARENT.replace("b: float = 2.0", "b: float = 3.0"))
    RN.write_json(d / "brief.json", {"parent": f"{repo / 'runs'}:par", "may_change": ["def:_hook"]})
    assert CLI.main(["check", "--run", str(run), "--iter", "1"]) == 1
    assert json.loads((d / "change.json").read_text())["outside"] == ["param:b"]
    RN.write_json(d / "decision.json", _decision(action="stop", may_change=[]))
    assert CLI.main(["decide", "--run", str(run), "--iter", "1"]) == 0
    assert CLI.main(["guard", "--run", str(run)]) == 0
    assert CLI.main(["finish", "--run", str(run), "--reason", "done"]) == 0 and not (repo / GU.LOCK).exists()


ROOT = __import__("pathlib").Path(__file__).resolve().parents[1]


def test_the_project_hook_guards_every_tool_while_a_loop_runs():
    s = json.loads((ROOT / ".claude" / "settings.json").read_text())
    (entry,) = s["hooks"]["PreToolUse"]
    tools = set(entry["matcher"].split("|"))
    assert set(GU.WRITE_TOOLS) | set(GU.READ_TOOLS) | {"Bash"} <= tools
    cmd = entry["hooks"][0]["command"]
    assert GU.LOCK in cmd and "brixzle.director.guard hook" in cmd
    assert GU.LOCK in (ROOT / ".gitignore").read_text()


@pytest.mark.parametrize("role", ["designer", "critic", "director", "verifier"])
def test_each_role_has_its_own_instructions(role):
    text = (ROOT / ".claude" / "agents" / f"rsi-{role}.md").read_text()
    head = text.split("---")[1]
    assert f"name: rsi-{role}" in head and "description:" in head and "tools:" in head
    assert "Write" in head.split("tools:")[1]  # each writes its own file; the guard keeps it inside the run
    assert "rsi-designer" in (ROOT / ".claude" / "commands" / "rsi-loop.md").read_text()
