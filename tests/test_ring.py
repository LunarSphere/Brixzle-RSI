"""The ring (issue #89): a family's ring kit, the cell helper and the ring build.

References from aerial-assembly @ fc4f724 (brixzle/src/brixzle/ring.py, configs/ab.json), printed
with `uv run --frozen` by tests/fixtures/make_ring_reference.py into ring_reference.json.
"""

import json
import math
from pathlib import Path

import pytest

from brixzle.engine import ring_cells as RC
from brixzle.families.ab.family import FAMILY as AB

REF = json.loads((Path(__file__).parent / "fixtures" / "ring_reference.json").read_text(encoding="utf-8"))


@pytest.mark.parametrize("courses", [2, 4])
def test_the_a_b_plan_is_the_original_ring_in_the_original_order(courses):
    ref = REF["plans"][str(courses)]
    plan = AB.ring.plan(AB.defaults, 6, courses)
    assert [b.name for b in plan] == ref["order"]
    yaws = {f"{k}_{c}_{x:g}_{y:g}": yaw for k, c, x, y, yaw in ref["bricks"]}
    U, H0 = AB.defaults.U, AB.defaults.H0
    for b in plan:
        yaw = yaws[b.name]
        assert b.quat == pytest.approx([math.cos(math.pi / 4 * yaw), 0.0, 0.0, math.sin(math.pi / 4 * yaw)])
        kind, c, x, y = b.name.split("_")
        assert b.kind == kind and b.course == int(c)
        assert b.part == (kind + "AB"[b.course % 2] if kind != "S2" else "AB"[b.course % 2])
        assert b.pos_mm == pytest.approx([(float(x) + 0.5) * U, (float(y) + 0.5) * U, b.course * H0])


def test_the_columns_check_and_the_closer_goes_last_on_each_course():
    bricks = RC.ring(6, 4)
    RC.check_columns(bricks)
    order = RC.assembly_order(bricks)
    for k in range(4):
        course = [b for b in order if b.course == k]
        assert course[-1].kind == "K2" and sum(b.kind == "K2" for b in course) == 1


def test_a_ring_needs_an_even_side_of_six_or_more():
    with pytest.raises(ValueError):
        RC.ring(5, 2)
    with pytest.raises(ValueError):
        RC.ring(4, 2)


def test_the_a_b_stroke_travel_matches_the_original():
    from brixzle.families.ab import ring as ABR

    assert RC.stroke_travel(AB, AB.defaults, ABR.stroke_mm(AB.defaults)) == pytest.approx(REF["stroke_travel_mm"])


@pytest.mark.parametrize("part", ["L3A", "K2A", "I3B", "K2B"])
def test_a_b_kit_parts_have_the_original_mass(part):
    from brixzle.families import builders as B

    got = B.ring_bodies(AB, AB.defaults, part).mass_g
    assert got == pytest.approx(REF["masses_g"][part], rel=0.01), (part, got, REF["masses_g"][part])


def test_the_a_b_kit_has_its_parts_and_exact_collision():
    from brixzle.families import builders as B

    assert set(AB.ring.parts) == {"L3A", "L3B", "I3A", "I3B", "K2A", "K2B"}
    b = B.ring_bodies(AB, AB.defaults, "L3A")
    assert b.collision_source == "exact" and len(b.collision) > 0
    assert len(AB.ring.base(AB.defaults, 6)) > 0


def test_a_family_with_no_kit_gives_no_ring_kit(tmp_path):
    from brixzle.engine.sim import ring_build as RB
    from brixzle.families.wedge.family import FAMILY as WEDGE

    assert WEDGE.ring is None
    r = RB.build_ring(WEDGE, WEDGE.defaults, runs=tmp_path)
    assert r["status"] == "no_ring_kit" and r["seated"] == 0 and r["total"] == 0


def test_check_family_reports_the_ring_kit():
    from brixzle.agents import check_family as CF

    assert CF.ring_report(AB) == {"kit": True, "parts": sorted(AB.ring.parts), "built": True, "error": None}
    from brixzle.families.wedge.family import FAMILY as WEDGE

    r = CF.ring_report(WEDGE)
    assert r["kit"] is False and "kit_from_cell" in r["error"]


@pytest.mark.slow
def test_an_a_b_ideal_ring_of_two_courses_seats_every_brick(tmp_path):
    """The only slow test here (about a minute). Simulated, uncalibrated contact."""
    from brixzle.engine.sim import ring_build as RB

    r = RB.build_ring(AB, AB.defaults, n=6, courses=2, runs=tmp_path)
    assert r["status"] == "complete" and r["seated"] == r["total"] == 16, (r["status"], r["stages"][-1])
    saved = json.loads((Path(r["run"]) / "ring.json").read_text(encoding="utf-8"))
    assert saved["label"] == "simulated, uncalibrated contact" and saved["family"] == "ab"


def test_a_family_with_a_kit_can_be_pickled_for_worker_processes():
    """Maps, quick checks and builds send the family to spawned workers (#89: closures broke that)."""
    import pickle

    again = pickle.loads(pickle.dumps(AB))
    assert again == AB  # the tune and Z compare a family after its round trip
    assert [b.name for b in again.ring.plan(AB.defaults, 6, 2)] == REF["plans"]["2"]["order"]
