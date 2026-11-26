"""The error-radius goal (#206): the score, the band archive, the from-scratch start and the explore rule."""
import json
import math
import subprocess

import pytest

from brixzle.director import archive as AR
from brixzle.director import decide as DE
from brixzle.director import run as RN
from brixzle.engine import radius as RA


# --- the score ---


def test_disc_points_cover_the_disc_evenly_and_repeat():
    pts = RA.disc_points(20.0, 30, seed=3)
    assert len(pts) == 30
    assert all(math.hypot(x, y) <= 20.0 + 1e-9 and 0.0 <= h < 1.0 for x, y, h in pts)
    assert pts == RA.disc_points(20.0, 30, seed=3)
    assert pts != RA.disc_points(20.0, 30, seed=4)
    # even cover: about half the points inside the half-area radius
    inner = sum(math.hypot(x, y) <= 20.0 / math.sqrt(2) for x, y, _ in pts)
    assert 13 <= inner <= 17
    with pytest.raises(ValueError):
        RA.disc_points(-1.0, 5)


def test_r90_interpolates_where_the_weakest_rate_crosses_the_level():
    # passes 8 and 16 (rate 1.0, 0.95), fails 22 at 0.85: crosses 0.9 halfway between 16 and 22
    assert RA.r90([(8, 1.0), (16, 0.95), (22, 0.85)]) == pytest.approx(19.0)
    # fails the first radius: interpolated from rate 1 at r = 0, so it can still improve
    assert RA.r90([(8, 0.5)]) == pytest.approx(8 * 0.1 / 0.5)
    # passes every radius: the largest, capped
    assert RA.r90([(8, 1.0), (40, 0.95)], max_mm=40) == 40


def test_radius_settings_refuse_nonsense():
    with pytest.raises(ValueError):
        RA.RadiusSettings(radii_mm=(16.0, 8.0))
    with pytest.raises(ValueError):
        RA.RadiusSettings(drops=0)
    with pytest.raises(ValueError):
        RA.RadiusSettings(placements=("full", "sideways"))


@pytest.mark.slow
def test_radius_search_runs_on_ab():
    from brixzle.families import registry

    fam = registry.get("ab")
    out = RA.radius_search(fam, fam.defaults, RA.RadiusSettings(radii_mm=(8.0,), drops=2), workers=1)
    assert set(out["by_radius"]["8.0"]["placements"]) == set(RA.PLACEMENTS)
    assert 0.0 <= out["r90_mm"] <= 8.0 and out["label"] == RA.LABEL


# --- the band archive ---


def _rec(name, green, steps=3.0, stack=3.0, violations=()):
    return {"name": name, "green_mm": green, "overhang_steps": steps, "stack_courses": stack, "violations": list(violations)}


def test_bands_name_the_error_ranges():
    assert AR.band_of(3.0) == "below 16"
    assert AR.band_of(16.0) == "16-22"
    assert AR.band_of(27.9) == "22-28"
    assert AR.band_of(40.0) == "34+"


def test_the_band_archive_keeps_the_best_per_band():
    arc = AR.BandArchive()
    assert arc.offer(_rec("a", 18.0))["kept"]
    assert arc.offer(_rec("b", 30.0))["kept"]                      # another band: both stay
    assert not arc.offer(_rec("c", 18.5, steps=2.0))["kept"]       # a tie on the green zone, fewer steps
    res = arc.offer(_rec("d", 18.4, steps=5.0))                     # a tie on the green zone, more steps
    assert res["kept"] and res["replaced"] == ["a"]
    assert arc.offer(_rec("e", 21.0))["kept"]                       # clearly more error in the same band
    assert not arc.offer(_rec("f", 39.0, violations=["mvp_gate"]))["kept"]
    assert sorted(m["name"] for m in arc.members) == ["b", "e"]


# --- the run ---


@pytest.fixture
def repo(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    (tmp_path / "README.md").write_text("x\n")
    (tmp_path / ".gitignore").write_text("runs/\n.rsi-loop-active\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "-A"], check=True)
    subprocess.run(["git", "-C", str(tmp_path), "-c", "user.email=t@t", "-c", "user.name=t", "commit", "-qm", "init"],
                   check=True)
    return tmp_path


def _run(repo, baseline="none"):
    return RN.init_run(repo / "runs" / "rsi" / "r1", goal="Largest error radius.", baseline=baseline, repo=repo)


def test_a_new_run_defaults_to_the_radius_goal_and_can_start_from_scratch(repo, monkeypatch):
    from brixzle.director import __main__ as DM

    run = _run(repo)
    goal = json.loads((run / "goal.json").read_text())
    assert goal["settings"]["stage"] == "radius" and goal["settings"]["explore_every"] == 4
    assert json.loads((run / "next.json").read_text())["parent"] == ""
    monkeypatch.chdir(repo)
    assert DM.main(["baseline", "--run", str(run)]) == 0
    v = json.loads((run / "baseline" / "verified.json").read_text())
    assert v["measured"] is None and not v["offer"]["kept"]


def _decision(**kw):
    d = {"action": "push", "design": "ab", "term": "green_mm", "brief": "Lengthen the guide.",
         "may_change": ["param:ramp"], "goal_change": None, "evidence": []}
    d.update(kw)
    return d


def test_an_empty_archive_needs_a_new_family(repo):
    run = _run(repo)
    res = DE.record(run, _decision())
    assert not res["ok"] and "new family" in res["reason"]
    res = DE.record(run, _decision(action="fix", design="", may_change=["new_family"], brief="An original seat."))
    assert res["ok"]
    brief = json.loads((run / "iter-01" / "brief.json").read_text())
    assert brief["parent"] == "" and brief["may_change"] == ["new_family"]


def _archive(run, members, offers):
    RN.write_json(run / "archive.json", {"members": members, "offers": offers})


def test_every_nth_iteration_and_a_stall_need_a_new_family(repo):
    run = _run(repo)
    goal = json.loads((run / "goal.json").read_text())
    _archive(run, [_rec("iter-01:a", 18.0)], [{"name": "iter-01:a", "kept": True}])
    assert DE.explore_due(run, goal, 3) == ""
    assert "every 4th" in DE.explore_due(run, goal, 4)
    _archive(run, [_rec("iter-01:a", 18.0)], [{"name": "iter-01:a", "kept": True}, {"name": "iter-02:b", "kept": False},
                                               {"name": "iter-03:c", "kept": False}, {"name": "iter-04:d", "kept": False}])
    assert "added nothing" in DE.explore_due(run, goal, 5)
    # a legacy goal never forces one
    assert DE.explore_due(run, {"settings": {"stage": "legacy"}}, 3) == ""


def test_each_decided_iteration_is_recorded_once_with_its_mechanism(repo):
    run = _run(repo)
    assert DE.record(run, _decision(action="fix", design="", may_change=["new_family"], brief="An original seat."))["ok"]
    d = run / "iter-01"
    RN.write_json(d / "verified.json", {"measured": {"spec": "x:y", "green_mm": 9.5, "grid": {"weakest": "plus_bare"},
                                                     "overhang_steps": 3.0, "stack_courses": 3.0, "violations": []},
                                        "claim": {"ok": True}, "offer": {"kept": True, "reason": "best in band below 16"}})
    (d / "diagnosis.md").write_text("# Diagnosis\n\nMechanism: the tooth props on the bare end face\n")
    assert DE.record(run, _decision(action="fix", design="", may_change=["new_family"], brief="Another idea."))["ok"]
    lines = [json.loads(x) for x in (run / "designs.jsonl").read_text().splitlines()]
    assert len(lines) == 1 and lines[0]["mechanism"] == "the tooth props on the bare end face"
    assert lines[0]["new_family"] and lines[0]["rules"] == "218"
    assert lines[0]["green_mm"] == 9.5 and lines[0]["weakest"] == "plus_bare"
    shared = run.resolve().parents[1] / "designs.jsonl"
    assert len(shared.read_text().splitlines()) == 1


def test_the_radius_measurement_scores_the_grid_and_builds_at_the_green_zone(tmp_path, monkeypatch):
    from brixzle.agents import check_family as CF
    from brixzle.director import verify as VE
    from brixzle.engine import gridscore as GS
    from brixzle.engine import gripper_spec as GSP
    from brixzle.engine import structure_gate as SG
    from brixzle.tools import core as TC

    monkeypatch.setattr(CF, "check_in_subprocess", lambda root, module: {"ok": True, "violations": [], "mass_g": 9.0})
    monkeypatch.setattr(TC, "load", lambda spec: type("F", (), {"defaults": None, "name": "fake"})())
    monkeypatch.setattr(GSP, "target_check", lambda fam, p: {"violations": []})
    monkeypatch.setattr(SG, "run_gate", lambda *a, **k: {"passed": True, "overhang_n": 3, "builds": [], "seconds": 1,
                                                         "run": "r"})
    fake = GS.search(lambda pts: ["seated" if abs(x) < 18 else "jammed" for _, x, _ in pts],
                     GS.GridSettings(placements=("full", "corbel_reach")))
    monkeypatch.setattr(GS, "grid_search", lambda fam, p, s, workers=None: {**fake, "drops_rows": [], "jams": None})
    seen = {}
    monkeypatch.setattr(RA, "noisy_builds", lambda fam, p, r, bs, workers=None: seen.setdefault(
        "r", {"radius_mm": bs.radius_mm if r is None else r, "overhang_steps": 3.0, "stack_courses": 3.0}))
    m = VE.measure_radius("x:y", tmp_path / "measured", GS.GridSettings(placements=("full", "corbel_reach")))
    assert m["green_mm"] == fake["green_mm"] and seen["r"]["radius_mm"] == 16.0 == m["build_radius_mm"]
    assert m["second_mm"] == max(m["green_by_placement"].values())  # two placements: the other one
    assert m["weakest"] == fake["weakest"] and m["z_rings"]["16"] == 1.0
    assert (tmp_path / "measured" / "grid" / "map-full.png").exists()
    assert m["grid"]["maps"] and "r90_mm" not in m
