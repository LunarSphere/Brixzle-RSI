"""Loop improvements before campaign 2 (#214): leximin ranking, gentler exploration and the 3D
contact report."""
import json
import subprocess

import numpy as np
import pytest

from brixzle.director import archive as AR
from brixzle.director import decide as DE
from brixzle.director import run as RN
from brixzle.engine import jam_analysis as JA


# --- leximin ranking ---


def _rec(name, greens, steps=3.0, violations=()):
    return {"name": name, "green_mm": min(greens.values()), "green_by_placement": greens, "overhang_steps": steps,
            "stack_courses": 3.0, "violations": list(violations)}


def test_a_band_ranks_by_every_placements_green_zone_weakest_first():
    arc = AR.BandArchive()
    a = _rec("a", {"corbel_reach": 3.75, "full": 8.75, "plus_bare": 9.0, "minus_bare": 9.0}, steps=5.0)
    b = _rec("b", {"corbel_reach": 3.75, "full": 8.75, "plus_bare": 9.0, "minus_bare": 21.0}, steps=1.0)
    assert arc.offer(a)["kept"]
    res = arc.offer(b)                          # ties on the weakest three, wins on the fourth
    assert res["kept"] and res["replaced"] == ["a"]
    assert AR.green_vector(b) == [3.75, 8.75, 9.0, 21.0]
    # within 1.25 mm everywhere: overhang decides
    c = _rec("c", {"corbel_reach": 4.5, "full": 8.0, "plus_bare": 9.5, "minus_bare": 20.5}, steps=4.0)
    assert arc.offer(c)["kept"]


def test_a_record_without_per_placement_zones_falls_back_to_its_headline():
    assert AR.green_vector({"green_mm": 5.0}) == [5.0]


# --- gentler exploration ---


@pytest.fixture
def repo(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    (tmp_path / "README.md").write_text("x\n")
    (tmp_path / ".gitignore").write_text("runs/\n.rsi-loop-active\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "-A"], check=True)
    subprocess.run(["git", "-C", str(tmp_path), "-c", "user.email=t@t", "-c", "user.name=t", "commit", "-qm", "init"],
                   check=True)
    return tmp_path


def test_new_families_are_forced_every_fourth_iteration_and_after_three_stalls(repo):
    run = RN.init_run(repo / "runs" / "rsi" / "r1", goal="g", baseline="none", repo=repo)
    goal = json.loads((run / "goal.json").read_text())
    assert goal["settings"]["explore_every"] == 4 and goal["settings"]["stall_explore"] == 3
    RN.write_json(run / "archive.json", {"members": [{"name": "iter-01:a"}],
                                         "offers": [{"name": "iter-01:a", "kept": True},
                                                    {"name": "iter-02:b", "kept": False},
                                                    {"name": "iter-03:c", "kept": False}]})
    assert DE.explore_due(run, goal, 3) == ""          # two stalls are not enough now
    assert "every 4th" in DE.explore_due(run, goal, 4)


# --- the 3D contact report ---


def _contact(pos, other, normal=(0.0, 0.0, 1.0), n=0.1):
    return {"pos_mm": list(pos), "normal": list(normal), "force_N": [0.0, 0.0, n], "normal_N": n, "dist_mm": 0.0,
            "piece": "p0", "other": other, "other_geom": f"{other}_g"}


def test_3d_contacts_are_merged_per_body_and_reported_in_the_bricks_frame():
    rest = {"pos_mm": [10.0, 0.0, 20.0], "quat": [1.0, 0.0, 0.0, 0.0], "weight_N": 0.2,
            "placed": [{"brick": "brick0", "part": "A"}],
            "contacts": [_contact((5.0, 1.0, 20.0), "brick0"), _contact((5.5, 1.2, 20.0), "brick0"),
                         _contact((30.0, 0.0, 15.0), "base", normal=(-1.0, 0.0, 0.0), n=0.05),
                         {**_contact((0.0, 0.0, 0.0), "brick0"), "normal_N": 0.0, "force_N": [0.0, 0.0, 0.0]}]}
    sites = JA.contact_sites_3d(rest)
    assert len(sites) == 2
    top = sites[0]
    assert top["other"] == "A (brick0)" and top["points"] == 2
    assert np.allclose(top["at_mm"], [-4.75, 1.1, 0.0])      # in the falling brick's frame
    assert top["weight_share"] == pytest.approx(1.0)
    assert sites[1]["other"] == "base" and sites[1]["normal"] == [-1.0, 0.0, 0.0]
