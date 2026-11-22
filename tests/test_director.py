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


def test_archive_uses_the_overhang_term_from_the_run_goal(repo):
    from brixzle.director import __main__ as CLI

    run = _init(repo, settings={"scored": ["z_target", "overhang_mean_steps", "z_mu08", "overhang_n"]})
    ab = {**_rec("ab", tol=4.0), "overhang_mean_steps": 29 / 6}
    short = {**_rec("short", tol=5.0), "overhang_mean_steps": 23 / 6}

    assert CLI._offer(run, ab)["kept"]
    assert not CLI._offer(run, short)["kept"]
    assert CLI._offer(run, ab)["kept"]
    archive = RN.read_json(run / "archive.json")
    assert [row["name"] for row in archive["members"]] == ["ab"]
    assert [row["name"] for row in archive["offers"]] == ["ab", "short"]


@pytest.mark.parametrize(
    ("stage", "failure", "violation"),
    [
        ("gate", {"passed": False}, "structure_gate"),
        ("ring", {"status": "placement_jammed"}, "ring"),
        ("ring", {"error": "invalid kit"}, "ring"),
        ("check_family", {"ok": False, "violations": []}, "check_family"),
    ],
)
def test_verify_rejects_infeasible_checkpoint(repo, stage, failure, violation):
    from brixzle.director import __main__ as CLI

    run = _init(repo)
    d = run / "iter-01"
    (d / "candidate").mkdir(parents=True)
    (d / "candidate" / "family.py").touch()
    measured_dir = d / "measured"
    measured_dir.mkdir()
    measured = {
        "gate": {"passed": True},
        "ring": {"status": "complete"},
        "check_family": {"ok": True, "violations": []},
        "objective": _rec("candidate"),
        "violations": [],
        "seconds": 0,
    }
    measured[stage] = failure
    RN.write_json(measured_dir / "measured.json", measured)

    assert CLI.main(["verify", "--run", str(run), "--iter", "1"]) == 0
    verified = RN.read_json(d / "verified.json")
    assert verified["measured"]["violations"] == [violation]
    assert verified["measured"]["objective"]["violations"] == []
    assert not verified["offer"]["kept"]
    assert RN.read_json(run / "archive.json")["members"] == []


def test_baseline_uses_the_same_feasibility_checks(repo):
    from brixzle.director import __main__ as CLI

    run = _init(repo)
    measured_dir = run / "baseline" / "measured"
    measured_dir.mkdir(parents=True)
    RN.write_json(
        measured_dir / "measured.json",
        {"gate": {"passed": True}, "ring": None, "check_family": {"ok": True},
         "objective": _rec("ab", feasible=False), "violations": ["grip"]},
    )

    argv = ["baseline", "--run", str(run)]
    assert CLI.main(argv) == 0
    assert CLI.main(argv) == 0
    verified = RN.read_json(run / "baseline" / "verified.json")
    assert verified["measured"]["violations"] == ["grip", "ring"]
    archive = RN.read_json(run / "archive.json")
    assert archive["members"] == []
    assert len(archive["offers"]) == 1


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


def test_measure_resumes_after_it_was_cut_off(tmp_path, monkeypatch):
    """A foreground step is cut off at 10 minutes; run again, it reuses what it saved (#165)."""
    from brixzle.engine import objective as OB
    from brixzle.tools import core as TC
    calls = {"eval": 0, "obj": 0}

    def fake_eval(spec, out, stages=(), seed=0, cache=True):
        calls["eval"] += 1
        __import__("pathlib").Path(out).mkdir(parents=True)
        rec = {"z": {"mean": {"Z": 0.5, "S": 9, "J": 0, "N": 18}}, "gate": {"overhang_n": 9}, "stages": list(stages),
               "complete": True}
        (__import__("pathlib").Path(out) / "eval.json").write_text(json.dumps(rec))
        return rec

    def fake_obj(fam, **kw):
        calls["obj"] += 1
        if calls["obj"] == 1:
            raise KeyboardInterrupt  # cut off mid-objective
        return {"tolerance_mm": 4.0, "violations": []}

    monkeypatch.setattr(TC, "evaluate", fake_eval)
    monkeypatch.setattr(TC, "load", lambda spec: type("F", (), {"defaults": None})())
    monkeypatch.setattr(TC, "friction", lambda fam, p, mus: {"z": {"mu 0.8": {"Z": 0.3, "S": 5, "J": 1, "N": 10}}})
    monkeypatch.setattr(OB, "evaluate_design", fake_obj)
    with pytest.raises(KeyboardInterrupt):
        VE.measure("ab", tmp_path)
    m = VE.measure("ab", tmp_path)
    assert calls == {"eval": 1, "obj": 2} and m["tolerance_mm"] == 4.0
    again = VE.measure("ab", tmp_path)
    assert calls == {"eval": 1, "obj": 2} and again == m


def test_the_loop_never_waits_in_the_background():
    text = (ROOT / ".claude" / "commands" / "rsi-loop.md").read_text()
    assert "run_in_background" in text and "600000" in text


def test_verify_measures_the_edge_target_on_request_and_checkpoints_it(repo, monkeypatch):
    from brixzle.director import __main__ as CLI
    from brixzle.tools import core as TC

    run = _init(repo)
    d = run / "iter-01"
    (d / "candidate").mkdir(parents=True)
    (d / "candidate" / "family.py").touch()
    measured_dir = d / "measured"
    measured_dir.mkdir()
    RN.write_json(measured_dir / "measured.json",
                  {"gate": {"passed": True}, "ring": {"status": "complete"}, "check_family": {"ok": True},
                   "objective": _rec("candidate"), "seconds": 1.0})
    RN.write_json(d / "brief.json", {"term": "edge_z"})
    RN.write_json(d / "claim.json", {"numbers": [{"what": "weaker edge Z", "path": "edge_z", "value": 0.3}]})
    calls = []
    family = type("FakeFamily", (), {"defaults": None})()
    monkeypatch.setattr(TC, "load", lambda _: family)
    monkeypatch.setattr(TC, "cache_on", lambda: None)
    monkeypatch.setattr(TC, "edgemap", lambda family, params: calls.append(family) or
                        {"edge_z": 0.3, "weaker_bare_edge": "minus_bare", "cases": {}})

    assert CLI.main(["verify", "--run", str(run), "--iter", "1"]) == 0
    assert VE.measure("candidate", measured_dir, measure_edge=True)["edge_z"] == 0.3
    assert calls == [family]
    measured = RN.read_json(d / "verified.json")["measured"]
    assert measured["edge_z"] == 0.3
    assert RN.read_json(measured["edge"]["report"])["edge_z"] == 0.3


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


def test_held_goal_measures_baseline_and_candidate_and_ranks_verified_reach(repo, monkeypatch):
    from brixzle.director import __main__ as cli
    from brixzle.engine import heldmap as hm
    from brixzle.tools import core as tc

    run = _init(repo, settings={"stage": "held", "scored": ["held_reliable_reach_mm"], "min_drop_z": .85})
    calls = []
    def fake(family, p, cfg, settings):
        reach = 100 if cfg.tine_d_m == .003 else 75
        calls.append(reach)
        return dict(scores={"held_reliable_reach_mm": reach}, cases={}, mission_passed=True,
                    violations=[], seconds=.1, wrist_defaults="guess")

    monkeypatch.setattr(hm, "heldmap", fake)
    monkeypatch.setattr(tc, "load", lambda spec: type("Family", (), {"defaults": None})())
    for name, z in (("baseline", .9), ("iter-01", .91), ("iter-02", .8)):
        d = run / name
        m = d / "measured"
        m.mkdir(parents=True)
        RN.write_json(m / "measured.json", dict(check_family={"ok": True}, gate={"passed": True},
                      ring={"status": "complete"}, z={"mean": {"Z": z}}, objective=_rec(name, z=z), seconds=1.))
        if name == "baseline":
            args = ["baseline"]
        else:
            pkg = d / "new"
            pkg.mkdir()
            (pkg / "family.py").touch()
            RN.write_json(pkg / "gripper.json", {"tine_d_m": .003})
            RN.write_json(d / "claim.json", {"numbers": [{"what": "measured reach", "path": "held.scores.held_reliable_reach_mm",
                                                         "value": 100}]})
            args = ["verify", "--iter", name[-2:]]
        assert cli.main(args + ["--run", str(run)]) == 0
        record = RN.read_json(d / "verified.json")
        report = RN.read_json(record["measured"]["held"]["report"])
        assert report["scores"]["held_reliable_reach_mm"] == calls[-1]
        assert record["offer"]["kept"] == (name != "iter-02")
        if name == "iter-02":
            assert "drop_full_z" in record["measured"]["violations"]
    assert calls == [75, 100, 100]
    members = RN.read_json(run / "archive.json")["members"]
    assert [rec["name"] for rec in members] == ["iter-01:new"]
    assert members[0]["held_reliable_reach_mm"] == 100


def test_held_feasibility_survives_resume(tmp_path, monkeypatch):
    from brixzle.engine import heldmap as hm
    from brixzle.tools import core as tc

    run = tmp_path / "measured"
    run.mkdir()
    RN.write_json(run / "measured.json", dict(check_family={"ok": False}, gate={"passed": False},
                  ring={"status": "complete"}, z={"mean": {"Z": 1}}, objective=_rec("test", feasible=False)))
    monkeypatch.setattr(tc, "load", lambda spec: type("Family", (), {"defaults": None})())
    monkeypatch.setattr(hm, "heldmap", lambda *args: dict(scores={}, cases={}, mission_passed=False,
                        violations=["held_r4", "held_payload"], seconds=0))
    m = VE.measure("ab", run, measure_held=True)
    again = VE.measure("ab", run, measure_held=True)
    assert m == again
    assert set(m["violations"]) == {"grip", "check_family", "structure_gate", "held_r4", "held_payload"}


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


def test_init_accepts_campaign_settings_before_the_first_baseline(repo, monkeypatch):
    from brixzle.director import __main__ as cli
    monkeypatch.chdir(repo)
    settings = {"stage": "held", "scored": ["held_reliable_reach_mm", "held_full_z"]}
    cfg = repo / "settings.json"
    RN.write_json(cfg, settings)
    run = repo / "runs" / "stageb"
    assert cli.main(["init", "--run", str(run), "--goal", "Reliable reach", "--settings", str(cfg)]) == 0
    saved = RN.read_json(run / "goal.json")["settings"]
    assert saved["scored"] == settings["scored"]
    assert saved["grip_mm"] > 0  # omitted stage-A settings can still be tightened by the director


def test_fresh_held_verification_keeps_drop_floor_on_resume(tmp_path, monkeypatch):
    from brixzle.engine import heldmap as hm
    from brixzle.tools import core as tc

    def evaluate(spec, directory, **kwargs):
        directory.mkdir()
        return {"check_family": {"ok": True}, "gate": {"passed": True}, "ring": {"status": "complete"},
                "z": {"mean": {"Z": .5}}}

    calls = []
    def measure(family, params, gripper, settings):
        calls.append(settings.seed)
        return {"scores": {}, "cases": {}, "mission_passed": True, "violations": [], "seconds": 0}

    monkeypatch.setattr(tc, "evaluate", evaluate)
    monkeypatch.setattr(tc, "load", lambda spec: type("Family", (), {"defaults": None})())
    monkeypatch.setattr(hm, "heldmap", measure)
    out = VE.measure("ab", tmp_path, objective=False, measure_held=True, min_drop_z=.75)
    assert out["violations"] == ["drop_full_z"]
    assert VE.measure("ab", tmp_path, objective=False, measure_held=True) == out
    assert calls == [hm.HeldMapSettings.seed]
