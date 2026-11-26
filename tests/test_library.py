"""The design library (#215): one card per past design, an index, filled as iterations are recorded."""
import json
from pathlib import Path

import pytest

from brixzle.director import library as LB
from brixzle.director import run as RN

GREENS = {"corbel_reach": 3.75, "full": 23.78, "plus_bare": 24.04, "minus_bare": 21.25}


def _grid(greens=GREENS):
    return {"green_mm": min(greens.values()), "weakest": min(greens, key=greens.get), "order": sorted(greens, key=greens.get),
            "placements": {pl: {"green_mm": g, "rings": {"16": {"Z": 1.0 if g >= 16 else 0.4},
                                                         "28": {"Z": 0.5}, "40": {"Z": 0.2}}}
                           for pl, g in greens.items()},
            "jams_summary": ["corbel_reach: jammed from 3.8 mm (x -7.5..-5, y -5..+5; 6 cells, repeat 93%): 3D",
                             "    touches A (brick0) at (-14.8, +0.5, +21.9) mm in its own frame, 1.13 of the weight"]}


def _entry(**kw):
    args = dict(name="allround", spec="fable_allround", origin="registry", grid=_grid(),
                builds={"overhang_steps": 0.0, "stack_courses": 2.0, "radius_mm": 16.0},
                idea="A twin hook with wide wings.", diagnosis=None, violations=["gripper_interface"])
    return LB.entry_from(**{**args, **kw})


def test_a_card_names_the_strengths_and_weaknesses_with_numbers():
    card = LB.card_md(_entry())
    assert "# allround" in card and "A twin hook with wide wings." in card
    assert "simulated, uncalibrated contact" in card
    # strengths: placements at or past the 16 mm floor; weaknesses: the rest, weakest first, with the jam lines
    strengths = card[card.index("## Strengths"):card.index("## Weaknesses")]
    weaknesses = card[card.index("## Weaknesses"):]
    assert "plus_bare: 24.04 mm" in strengths and "full: 23.78 mm" in strengths
    assert "corbel_reach: 3.75 mm" in weaknesses and "touches A (brick0)" in weaknesses
    assert "gripper_interface" in card
    assert "overhang 0" in card and "stack 2" in card


def test_the_index_has_one_line_per_design_weakest_placement_first(tmp_path):
    LB.write_entry(tmp_path, _entry(), files={})
    LB.write_entry(tmp_path, _entry(name="wellkey", spec="x:wellkey", origin="radius-scratch-1/iter-02",
                                    idea="A keyed well.", grid=_grid({"corbel_reach": 3.75, "full": 8.75})), files={})
    index = (tmp_path / "INDEX.md").read_text()
    rows = [l for l in index.splitlines() if l.startswith("| [")]
    assert len(rows) == 2
    assert "[allround](allround/card.md)" in index and "corbel_reach 3.75" in index
    scores = json.loads((tmp_path / "wellkey" / "scores.json").read_text())
    assert scores["green_by_placement"]["full"] == 8.75 and scores["rules"] == LB.RULES


def test_an_entry_never_overwrites_another_designs_folder(tmp_path):
    LB.write_entry(tmp_path, _entry(), files={})
    LB.write_entry(tmp_path, _entry(idea="Rescored."), files={})  # the same design rescored: replaced in place
    assert "Rescored." in (tmp_path / "allround" / "card.md").read_text()
    assert len([p for p in tmp_path.iterdir() if p.is_dir()]) == 1


def test_a_recorded_iteration_is_added_from_its_run_files(tmp_path):
    run = tmp_path / "runs" / "rsi" / "r1"
    d = run / "iter-02"
    pkg = d / "wellkey"
    pkg.mkdir(parents=True)
    (pkg / "family.py").write_text("FAMILY = None\n")
    grid_dir = d / "measured" / "grid"
    grid_dir.mkdir(parents=True)
    (grid_dir / "map-full.png").write_bytes(b"png")
    (grid_dir / "jams.json").write_text("{}")
    measured = {"spec": f"{d}:wellkey", "grid": _grid({"corbel_reach": 3.75, "full": 8.75}),
                "builds": {"overhang_steps": 4.0, "stack_courses": 6.0, "radius_mm": 16.0}, "violations": []}
    RN.write_json(d / "verified.json", {"measured": measured, "offer": {"kept": True, "reason": "best in band below 16"}})
    (d / "design_note.md").write_text("# Wellkey\n\nA keyed well that catches the reach.\n\nMore detail.\n")
    (d / "diagnosis.md").write_text("# Diagnosis\n\nMechanism: the reach tips past the socket rim\n")
    lib = tmp_path / "runs" / "library"
    path = LB.add_iteration(run, 2, lib, renders=False)
    card = (path / "card.md").read_text()
    assert path.name == "r1-iter02-wellkey"
    assert "A keyed well that catches the reach." in card and "the reach tips past the socket rim" in card
    assert "kept: best in band below 16" in card
    assert (path / "family.py").read_text() == "FAMILY = None\n"
    assert (path / "map-full.png").exists() and (path / "jams.json").exists()


def test_an_unmeasured_iteration_adds_nothing(tmp_path):
    run = tmp_path / "r1"
    (run / "iter-01").mkdir(parents=True)
    assert LB.add_iteration(run, 1, tmp_path / "library", renders=False) is None


def test_recording_an_iteration_adds_it_to_the_shared_library(tmp_path, monkeypatch):
    from brixzle.director import decide as DE

    run = tmp_path / "runs" / "rsi" / "r1"
    (run / "iter-03").mkdir(parents=True)
    RN.write_json(run / "iter-03" / "verified.json", {"measured": {"spec": "x:y", "green_mm": 3.0, "violations": []},
                                                      "claim": {"ok": True}, "offer": {"kept": False}})
    calls = []
    monkeypatch.setattr(LB, "add_iteration", lambda run, n, lib, **kw: calls.append((Path(run).name, n, Path(lib))))
    DE.log_design(run, 3)
    assert calls == [("r1", 3, (tmp_path / "runs" / "library").resolve())]

    def broken(*a, **kw):
        raise RuntimeError("disk full")

    monkeypatch.setattr(LB, "add_iteration", broken)
    (run / "designs.jsonl").unlink()
    DE.log_design(run, 3)  # never raises
    assert "disk full" in (run / "iter-03" / "library-error.txt").read_text()


@pytest.mark.slow
def test_a_registry_family_is_filed_from_its_grid_run_with_builds_code_and_renders(tmp_path):
    from brixzle.engine import gridscore as GS

    s = GS.GridSettings(placements=("full",))
    res = GS.search(lambda pts: ["seated" if abs(x) < 10 else "jammed" for _, x, _ in pts], s)
    grid_dir = tmp_path / "grid-ab"
    GS.write_grid({**res, "drops_rows": []}, grid_dir, "ab")
    (grid_dir / "builds.json").write_text(json.dumps({"overhang_steps": 2.0, "stack_courses": 3.0, "radius_mm": 16.0}))
    out = LB.add_design("ab", grid_dir, tmp_path / "lib")
    card = (out / "card.md").read_text()
    assert "overhang 2" in card and "stack 3" in card and "full:" in card
    assert (out / "family.py").exists() and (out / "module.png").exists() and list(out.glob("*.stl"))
    assert "A/B" in card or "ab" in card


def test_a_radius_run_starts_without_a_baseline_unless_one_is_named(tmp_path):
    import subprocess

    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    (tmp_path / ".gitignore").write_text("runs/\n.rsi-loop-active\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "-A"], check=True)
    subprocess.run(["git", "-C", str(tmp_path), "-c", "user.email=t@t", "-c", "user.name=t", "commit", "-qm", "i"],
                   check=True)
    run = RN.init_run(tmp_path / "runs" / "rsi" / "a", goal="g", baseline=None, repo=tmp_path)
    assert json.loads((run / "goal.json").read_text())["baseline"] == "none"
    (tmp_path / ".rsi-loop-active").unlink()
    held = RN.init_run(tmp_path / "runs" / "rsi" / "b", goal="g", baseline=None, repo=tmp_path,
                       settings={"stage": "held"})
    assert json.loads((held / "goal.json").read_text())["baseline"] == "ab"  # the held goal reads floors from it
