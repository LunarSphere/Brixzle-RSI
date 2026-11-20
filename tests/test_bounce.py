"""Bounce-out on impact (#167): a brick dropped from a height onto a half-supported or a full seat."""
import math

import pytest

from brixzle.engine import bounce as BO
from brixzle.families import registry


def test_energy_and_heights():
    assert BO.energy_mJ(20.0, 50.0) == pytest.approx(20e-3 * 9.81 * 50e-3 * 1e3)
    fam = registry.get("ab")
    lift = fam.release_lift_mm(fam.defaults)
    hs = BO.heights(fam, fam.defaults, (10.0, 50.0, 100.0))
    assert hs[0] == pytest.approx(lift) and hs[-1] == 100.0  # never below the family's own lift


def test_classify_names_bounce_out_apart_from_a_jam():
    assert BO.kind("seated") == "seated" and BO.kind("jammed") == "jammed"
    assert BO.kind("fell") == "bounced_out" and BO.kind("wrong_valley") == "bounced_out"


def test_a_low_drop_on_a_full_seat_seats(tmp_path):
    fam = registry.get("ab")
    r = BO.bounce(fam, fam.defaults, heights_mm=(0.0,), seats=("full",), repeats=1)
    (row,) = r["rows"]
    assert row["seat"] == "full" and row["kinds"] == {"seated": 1} and row["energy_mJ"] > 0


def test_a_softer_contact_is_its_own_physics():
    cfg = BO.config(dampratio=0.2)
    assert cfg.physics.contact_dampratio == 0.2
    with pytest.raises(ValueError):
        BO.config(dampratio=0.0)


def test_a_soft_part_gets_its_own_contact_in_the_scene():
    """Mixed materials (#167): the compliant pieces are their own geoms, with their own solref and friction."""
    import mujoco

    from brixzle.engine.sim import trials as T
    from brixzle.families import builders as B
    fam = registry.get("fable_gentle_tpuhook")
    p = fam.defaults
    b = B.sim_bodies(fam, p, "A")
    assert b.soft_collision and b.soft_contact["dampratio"] > 1.0 and b.soft_contact["estimate"]
    rig = T.DropRig(fam, p, T.TrialConfig(error=T.ErrorModel(ideal=True)), course=1, bodies={"A": b})
    soft = [g for g in range(rig.model.ngeom) if rig.model.geom(g).name.startswith("brick") and "_s" in rig.model.geom(g).name]
    assert len(soft) >= len(b.soft_collision)
    g = rig.model.geom(soft[0])
    assert g.priority[0] == 1 and abs(g.solref[1] - b.soft_contact["dampratio"]) < 1e-9
    assert abs(g.friction[0] - b.soft_contact["friction"]) < 1e-9


def test_the_tpu_hook_brick_keeps_gentles_shape():
    from brixzle.families import fingerprint as FP
    a = FP.fingerprint(registry.get("fable_gentle"), solid=False, ring=False)
    b = FP.fingerprint(registry.get("fable_gentle_tpuhook"), solid=False, ring=False)
    assert "params" not in FP.differences(a, b) and "seats:tower" not in FP.differences(a, b)
