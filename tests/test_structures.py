"""Structure builds (issue #81): the trial structures, the support graph, assembly order and A/B's
ideal builds. Sim results are simulated, uncalibrated contact, not hardware rates.

The expected placements and bases are the original's (aerial-assembly brixzle/structures.py @
fc4f724, printed with `uv run --frozen` there on 2026-11-12).
"""

from dataclasses import replace

import pytest

from brixzle.engine import structures as S
from brixzle.engine.sim import assemble as A
from brixzle.engine.sim import trials as T
from brixzle.families.ab.family import FAMILY as AB
from brixzle.families.interface import Lattice, Placement


def _tuples(s):
    return [(b.course, b.i0, b.direction) for b in s.bricks]


def test_the_structures_match_the_original():
    assert _tuples(S.overhang(3)) == [(0, 0, 1), (1, 1, 1), (2, 2, 1)] and S.overhang(3).bases == [(-1, 2)]
    assert _tuples(S.overhang(3, -1)) == [(0, 0, -1), (1, -1, -1), (2, -2, -1)]
    assert S.overhang(3, -1).bases == [(0, 3, -1)]
    assert _tuples(S.wall(3, 4)) == [(0, 0, 1), (0, 2, 1), (0, 4, 1), (1, 1, 1), (1, 3, 1), (2, 0, 1), (2, 2, 1),
                                     (2, 4, 1), (3, 1, 1), (3, 3, 1)]
    assert S.wall(3, 4).bases == [(-1, 7)]
    assert _tuples(S.bridge(3)) == [(0, 0, 1), (0, 6, 1), (1, 1, 1), (1, 5, 1), (2, 2, 1), (2, 4, 1), (3, 3, 1)]
    assert S.bridge(3).bases == [(-1, 2), (6, 9)]
    assert _tuples(S.tower(3)) == [(0, 0, 1), (1, 0, 1), (2, 0, 1)] and S.tower(3).bases == [(-1, 3)]
    assert set(S.CATALOG) == {"tower", "overhang+", "overhang-", "overhang+20", "overhang-20", "wall", "bridge"}


def test_connectivity_refuses_a_shared_voxel_and_finds_a_floating_brick():
    clash = S.Structure("clash", [Placement(0, 0), Placement(0, 1)], bases=[(-1, 3)])
    with pytest.raises(ValueError, match="used by"):
        A.connectivity(clash)
    floating = S.Structure("float", [Placement(0, 0), Placement(2, 0)], bases=[(-1, 3)])
    assert A.unsupported(A.connectivity(floating)) == ["b2_0"]
    g = A.connectivity(S.bridge(3))
    assert A.unsupported(g) == [] and g.predecessors("b3_3") == ["b2_2", "b2_4"]


def test_a_structure_for_another_span_raises():
    one = replace(AB, lattice=lambda p: Lattice(U_mm=p.U, D_mm=p.D, H0_mm=p.H0, span_voxels=1))
    for name in ("tower", "wall", "bridge", "overhang+"):
        with pytest.raises(ValueError, match="span_voxels = 1"):
            S.structure_for(one, one.defaults, name)


def test_a_footprint_needs_an_upright_or_turned_seat():
    p = AB.defaults
    from shapely import affinity

    from brixzle.families import builders as BL

    upright = affinity.translate(BL.section(AB, p, "A").outer, 25.0, 0.0)
    assert A.footprint(AB, p, Placement(0, 0)).symmetric_difference(upright).area < 1e-6
    # direction -1 on course 0 is part B turned 180 degrees about Z: its outline mirrored in X
    turned = affinity.translate(affinity.scale(BL.section(AB, p, "B").outer, -1.0, 1.0, origin=(0, 0)), 25.0, 0.0)
    assert A.footprint(AB, p, Placement(0, 0, -1)).symmetric_difference(turned).area < 1e-6
    tilted = replace(AB, seat_pose=lambda p, b: ([25.0, 0.0, 0.0], [0.7071, 0.7071, 0.0, 0.0]))
    with pytest.raises(ValueError, match="180-degree"):
        A.footprint(tilted, p, Placement(0, 0))


def test_the_assembly_order_places_supporters_first():
    order = A.assembly_order(AB, AB.defaults, S.bridge(3))
    names = [S.brick_name(b) for b in order]
    assert names[-1] == "b3_3" and all(names.index(f"b{k}_{k}") < names.index(f"b{k + 1}_{k + 1}") for k in range(2))


REPEATS = 3  # the tests' repeat count: a majority of 3 (the default is 5), to keep the suite fast


@pytest.fixture(scope="module")
def ab_repeats(ab_gate):
    """A/B's ideal builds, each repeated with seeded release jitter: the structure gate's (#83), run
    once per session in conftest.py (the builds are the slow part)."""
    return {"bridge": ab_gate["repeat_results"]["bridge"], "wall": ab_gate["repeat_results"]["wall"],
            "overhang+": ab_gate["repeat_results"]["overhang"]}


def test_a_b_builds_a_bridge_and_a_wall_on_most_repeats(ab_repeats):
    """A symmetric ideal drop can tip either way on numerical noise (issue #98: the bridge's b1_5 flipped
    between 7 of 7 and 3 of 7 under friction +-5e-5), so a build is judged on seeded repeats."""
    for name, n in (("bridge", 7), ("wall", 10)):
        r = ab_repeats[name]
        assert r["n"] == REPEATS and r["structure"] == S.structure_for(AB, AB.defaults, name).name
        assert r["passed"] is True, (name, r["statuses"])
        assert r["complete"] > REPEATS / 2 and r["fraction"] == pytest.approx(r["complete"] / REPEATS)
        for b in r["builds"]:
            if b["status"] == "complete":
                assert b["stable_bricks"] == b["total"] == n


def test_a_b_overhang_stops_before_twelve_on_every_repeat(ab_repeats):
    r = ab_repeats["overhang+"]
    assert r["passed"] is False and r["complete"] == 0
    assert all(b["total"] == 12 and 1 <= b["stable_bricks"] < 12 for b in r["builds"])


def test_repeats_are_seeded_and_jittered(ab_repeats):
    r = ab_repeats["bridge"]
    assert r["seeds"] == [0, 1, 2] and r["jitter_mm"] == A.JITTER_MM > 0
    first = [b["stages"][0]["sample"] for b in r["builds"]]
    offsets = [(s["dx"] - s["aim"], s["dy"]) for s in first]
    assert all(abs(dx) <= A.JITTER_MM and abs(dy) <= A.JITTER_MM for dx, dy in offsets)
    assert len(set(offsets)) == REPEATS  # each repeat has its own jitter
    again = A.assemble(AB, AB.defaults, S.structure_for(AB, AB.defaults, "bridge"),
                       T.TrialConfig(error=T.ErrorModel(ideal=True)), seed=1, jitter_mm=A.JITTER_MM)
    assert again["stages"][0]["sample"] == r["builds"][1]["stages"][0]["sample"]  # repeat 1 is seed 1
    assert again["status"] == r["builds"][1]["status"]


def test_no_jitter_keeps_the_ideal_release():
    r = A.assemble(AB, AB.defaults, S.tower(1), T.TrialConfig(error=T.ErrorModel(ideal=True)))
    s = r["stages"][0]["sample"]
    assert s["dx"] == s["aim"] and s["dy"] == 0.0


def test_repeats_are_written_to_a_new_run_directory(ab_repeats, tmp_path):
    import json

    r = ab_repeats["bridge"]
    one, two = A.write_repeats(r, tmp_path), A.write_repeats(r, tmp_path)
    assert one != two and one.name.startswith("builds-") and one.name.endswith("-ab-bridge3")
    saved = json.loads((one / "builds.json").read_text(encoding="utf-8"))
    assert saved["label"] == "simulated, uncalibrated contact" and saved["family"] == "ab"
    assert saved["n"] == REPEATS and saved["seeds"] == [0, 1, 2] and len(saved["builds"]) == REPEATS
    assert saved["params"]["U"] == 25.0 and "physics" in saved["config"] and saved["seconds"] > 0
    b = A.write_build(r["builds"][0], tmp_path)  # a single build still has its own directory
    assert b.name.startswith("build-") and json.loads((b / "build.json").read_text())["seed"] == 0
    with pytest.raises(ValueError, match="non-finite"):
        A.write_repeats({**r, "seconds": float("nan")}, tmp_path)


def test_a_push_test_pushes_the_top_of_an_ideal_build():
    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    r = A.push_test(AB, AB.defaults, S.tower(2), cfg, rate=20.0, limit=3.0)
    assert r["status"] in ("moved", "held") and 0.0 < r["break_force_N"] <= 3.0
