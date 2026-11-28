"""The self-steering loop's deterministic parts (#165): the run's files, the bounds, the one-change
check, the verifier and the director's decisions, on the error-radius goal (the only goal since #204)."""
import json
import subprocess
import sys

import pytest

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
    """A radius run with forced exploration off and one archive member, so a refinement is allowed."""
    kw.setdefault("settings", {"explore_every": 0, "stall_explore": 0})
    run = RN.init_run(repo / "runs" / "rsi" / "r1", goal="Beat the references on Z.", baseline="none", repo=repo, **kw)
    RN.write_json(run / "archive.json", {"members": [{"name": "iter-01", "z": 0.5, "violations": []}], "offers": []})
    return run


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
    (dict(goal_change={"settings": {"max_length_mm": 200.0}}, evidence=["EXISTS"]), "cannot be changed"),
])
def test_bad_decisions_are_refused(repo, bad, why):
    run = _init(repo)
    (repo / "runs" / "map-x").mkdir(parents=True)
    bad = {k: ([str(repo / "runs" / "map-x")] if v == ["EXISTS"] else v) for k, v in bad.items()}
    out = DE.record(run, _decision(**bad))
    assert not out["ok"] and why in out["reason"]
    assert (run / "decisions.jsonl").read_text() == ""


def test_a_goal_change_with_evidence_may_change_the_text(repo):
    run = _init(repo)
    (repo / "runs" / "map-x").mkdir(parents=True)
    out = DE.record(run, _decision(goal_change={"text": "Push overhang 3 first."}, evidence=[str(repo / "runs" / "map-x")]))
    assert out["ok"]
    goal = json.loads((run / "goal.json").read_text())
    assert goal["history"][-1]["text"] == "Push overhang 3 first." and goal["text"] == "Push overhang 3 first."


def test_init_refuses_a_removed_goal(repo):
    with pytest.raises(ValueError, match="only stage"):
        RN.init_run(repo / "runs" / "rsi" / "old", goal="g", baseline="none", repo=repo, settings={"stage": "held"})


def test_stop_ends_the_loop(repo):
    run = _init(repo)
    assert DE.record(run, _decision(action="stop", may_change=[], brief="No gain in 3 iterations."))["ok"]
    assert json.loads((run / "next.json").read_text())["step"] == "finish"


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


@pytest.mark.parametrize(("role", "model"), [("designer", "fable"), ("critic", "opus"),
                                            ("director", "opus"), ("verifier", "sonnet")])
def test_each_role_has_its_own_instructions(role, model):
    text = (ROOT / ".claude" / "agents" / f"rsi-{role}.md").read_text()
    head = text.split("---")[1]
    assert f"name: rsi-{role}" in head and "description:" in head and "tools:" in head
    assert f"model: {model}" in head.splitlines()
    assert "Write" in head.split("tools:")[1]  # each writes its own file; the guard keeps it inside the run
    assert "rsi-designer" in (ROOT / ".claude" / "commands" / "rsi-loop.md").read_text()


def test_the_loop_never_waits_in_the_background():
    text = (ROOT / ".claude" / "commands" / "rsi-loop.md").read_text()
    assert "run_in_background" in text and "600000" in text


def _git_repo(path, files):
    subprocess.run(["git", "init", "-q", str(path)], check=True)
    for f, text in files.items():
        (path / f).parent.mkdir(parents=True, exist_ok=True)
        (path / f).write_text(text)
    (path / ".gitignore").write_text("runs/\n.rsi-loop-active\n")
    subprocess.run(["git", "-C", str(path), "add", "-A"], check=True)
    subprocess.run(["git", "-C", str(path), "-c", "user.email=t@t", "-c", "user.name=t", "commit", "-qm", "init"], check=True)


def test_guard_and_finish_find_the_run_lock_through_a_runs_link(tmp_path, monkeypatch):
    """#179: runs/ is a link from the loop's worktree into another checkout with its own uncommitted work.
    The guard must compare the loop's checkout, and finish must remove the loop's lock, from any cwd."""
    main, work, elsewhere = tmp_path / "main", tmp_path / "work", tmp_path / "elsewhere"
    _git_repo(main, {"README.md": "x\n"})
    (main / "runs").mkdir()
    (main / "dirty.py").write_text("someone's uncommitted work\n")   # untracked in the main checkout
    _git_repo(work, {"README.md": "x\n"})
    (work / "runs").symlink_to(main / "runs")
    elsewhere.mkdir()
    monkeypatch.chdir(work)
    run = RN.init_run(work / "runs" / "rsi" / "r1", goal="g", baseline="ab", repo=work)
    monkeypatch.chdir(elsewhere)
    assert GU.changed_tracked(run) == []
    (work / "README.md").write_text("changed\n")
    assert GU.changed_tracked(run) == ["README.md"]
    (work / "README.md").write_text("x\n")
    RN.finish(run, reason="done")
    assert not (work / GU.LOCK).exists()


def test_gripper_only_iteration_obeys_the_brief(tmp_path):
    parent = _module(tmp_path, "parent", PARENT)
    child = _module(tmp_path, "child", PARENT)
    RN.write_json(child.with_name("gripper.json"), {"wrist": {"linear_k": [80, 80, 120]}})
    unit = "gripper:wrist.linear_k"
    assert CH.check(parent, child, [unit])["changed"] == [unit]
    assert CH.check(parent, child, [unit])["ok"]
    assert not CH.check(parent, child, ["param:b"])["ok"]
    RN.write_json(parent.with_name("gripper.json"), {"wrist": {"linear_k": [80, 80, 120]}})
    assert CH.changed(parent, child) == []


