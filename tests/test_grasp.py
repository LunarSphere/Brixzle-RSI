"""Grasping with the stock SO-101 jaws and the mould pick station (#153)."""
import numpy as np
import pytest

from brixzle.engine import structures as S
from brixzle.engine.sim import arm as A
from brixzle.engine.sim import grasp as G
from brixzle.engine.sim import pickplace as PP
from brixzle.families import builders as B
from brixzle.families import registry

DESIGNS = [("ab", "A"), ("ab", "B"), ("fable_twinhook4", "A"), ("fable_allround", "A")]


def test_jaws_are_measured_from_the_model():
    j = G.jaws()
    assert np.all(np.diff(j.opening_mm) > 0)
    assert j.opening_mm.min() < 20.0 and j.opening_mm.max() > 90.0
    assert j.fixed_x_mm == pytest.approx(-7.35, abs=0.01)    # the fixed jaw's tip spheres, proud of its pad
    assert 5.0 < j.tip_below_mm < 10.0


@pytest.mark.parametrize("name,part", DESIGNS)
def test_grasp_holds_the_outer_side_faces_above_the_centre_of_mass(name, part):
    fam = registry.get(name)
    p = fam.defaults
    g = G.choose_grasp(fam, p, part)
    sb = B.sim_bodies(fam, p, part)
    lo, hi = sb.mesh.bounds
    assert g.width_mm == pytest.approx(2 * min(-lo[1], hi[1]))
    assert g.z_mm >= sb.com_mm[2] + G.ABOVE_COM_MM
    assert abs(g.x_mm - sb.com_mm[0]) <= max(G.COM_X_MM)
    assert g.coverage >= 0.15
    assert g.open_mm > g.width_mm > g.squeeze_mm


def test_the_mould_holds_the_brick_still_close_to_where_it_was_put():
    fam = registry.get("fable_twinhook4")
    rig = PP.station_rig(fam, fam.defaults, "A")
    assert np.linalg.norm(np.array(rig.station[:3]) - np.array(rig.placed[:3])) < 1.5   # settles about 1 mm
    rig.hold(1.0)
    pos, _ = rig.brick_pose()
    assert np.linalg.norm(pos - np.array(rig.station[:3])) < 0.05                       # then stays


@pytest.mark.parametrize("name,part", [("ab", "A"), ("ab", "B"), ("fable_twinhook4", "A")])
def test_pick_and_lift_100mm_keeps_the_brick_in_the_jaws(name, part):
    fam = registry.get(name)
    r = PP.pick(PP.station_rig(fam, fam.defaults, part))
    assert r["lifted_mm"] > 95.0
    assert r["slip_mm"] < 1.0 and r["slip_deg"] < 1.0


@pytest.mark.xfail(strict=True, reason="allround's only top-down grip is its 2 mm wing-tip edges: it lifts but shifts about 1.45 mm in the jaws (#153)")
def test_allround_pick_and_lift_keeps_the_brick_in_the_jaws():
    fam = registry.get("fable_allround")
    r = PP.pick(PP.station_rig(fam, fam.defaults, "A"))
    assert r["lifted_mm"] > 95.0 and r["slip_mm"] < 1.0 and r["slip_deg"] < 1.0


@pytest.mark.parametrize("name,structure", [("ab", S.overhang(4, +1, 2)), ("ab", S.bridge(1, 2)),
                                            ("fable_twinhook4", S.overhang(4, +1, 2)), ("fable_allround", S.overhang(3, +1, 2))])
def test_the_arm_clears_the_structure_at_every_placement(name, structure):
    fam = registry.get(name)
    r = PP.placement_clashes(fam, fam.defaults, structure, A.top_down_envelope(step_mm=20.0))
    assert r["base"] is not None
    assert r["unreached"] == [] and r["clashes"] == []
