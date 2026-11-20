"""Bounce-out on impact (#167): a brick dropped from a height onto a half-supported or a full seat."""
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
    assert g.solimp == pytest.approx([0.95, 0.99, 0.0001, 0.5, 2.0])


def test_the_tpu_hook_brick_keeps_gentles_shape():
    from brixzle.families import fingerprint as FP
    a = FP.fingerprint(registry.get("fable_gentle"), solid=False, ring=False)
    b = FP.fingerprint(registry.get("fable_gentle_tpuhook"), solid=False, ring=False)
    assert "params" not in FP.differences(a, b) and "seats:tower" not in FP.differences(a, b)


def test_half_seat_setup_failures_do_not_count_as_bounce(monkeypatch):
    from brixzle.engine.sim import assemble as A

    fam = registry.get("ab")

    def builds(family, p, structure, cfg, n, seed, jitter_mm, workers):
        assert (seed, n, jitter_mm) == (19, 3, 0.05)
        return {"builds": [
            {"stages": [{"brick": "b0_0", "outcome": "fell"}]},
            {"stages": [{"brick": "b0_0", "outcome": "seated"}, {"brick": "b1_1", "outcome": "fell"}]},
            {"stages": [{"brick": "b0_0", "outcome": "seated"}, {"brick": "b1_1", "outcome": "jammed"}]},
        ]}

    monkeypatch.setattr(A, "repeat_builds", builds)
    report = BO.bounce(fam, heights_mm=(75.0,), seats=("half",), dampratios=(0.3,), repeats=3, seed=19)
    (row,) = report["rows"]
    assert row["outcomes"] == ["before_reach:fell", "fell", "jammed"]
    assert row["kinds"] == {"setup_failed": 1, "bounced_out": 1, "jammed": 1}
    assert (row["n"], row["released"], row["setup_failed"], row["bounced_out"]) == (3, 2, 1, 1)
    assert report["seed"] == 19 and report["repeats"] == 3
    assert report["params"] == fam.defaults.to_dict()
    assert row["config"]["physics"]["contact_dampratio"] == 0.3
    assert row["config"]["error"]["clearance"][0] == pytest.approx(75.0 - fam.release_lift_mm(fam.defaults))


def test_report_makes_effective_tpu_contact_and_its_source_explicit(monkeypatch):
    fam = registry.get("fable_gentle_tpuhook")
    monkeypatch.setattr(BO, "_full", lambda *args: ["seated"])
    report = BO.bounce(fam, heights_mm=(75.0,), seats=("full",), repeats=1)
    assert report["contact"]["pla"]["solimp"] == [0.95, 0.99, 0.0001, 0.5, 2.0]
    assert report["contact"]["soft"]["A"]["solimp"] == report["contact"]["pla"]["solimp"]
    assert report["contact"]["soft"]["A"]["estimate"] is True
    assert "unmeasured" in report["contact"]["soft"]["A"]["source"].lower()


def test_a_contact_change_distinguishes_stage_cache_keys():
    from dataclasses import replace

    from brixzle.tools import core as TC

    fam = registry.get("fable_gentle_tpuhook")
    p = fam.defaults

    def softer(params, part):
        pieces, contact = fam.soft(params, part)
        return pieces, {**contact, "dampratio": 2.1}

    changed = replace(fam, soft=softer)
    assert TC.stage_key(fam, p, "gate", {}) != TC.stage_key(changed, p, "gate", {})


def test_objective_size_includes_both_materials(monkeypatch):
    import numpy as np

    from brixzle.engine import objective as OB
    from brixzle.families import builders as B

    fam = registry.get("fable_gentle_tpuhook")

    def bodies(family, params, part):
        from types import SimpleNamespace

        return SimpleNamespace(collision=[np.array([[0, 0, 0], [20, 10, 5]])],
                               soft_collision=[np.array([[-3, 0, 0], [25, 10, 8]])])

    monkeypatch.setattr(B, "sim_bodies", bodies)
    size = OB.size(fam, fam.defaults)
    assert (size["length_mm"], size["depth_mm"], size["height_mm"]) == (28.0, 10.0, 8.0)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_damping_is_rejected_before_running_any_drops(monkeypatch, bad):
    def unexpected_drop(*args):
        pytest.fail("damping settings should be validated before running drops")

    monkeypatch.setattr(BO, "_full", unexpected_drop)
    fam = registry.get("ab")
    with pytest.raises(ValueError, match="dampratio must be finite"):
        BO.bounce(fam, heights_mm=(0.0,), seats=("full",), dampratios=(1.0, bad), repeats=1)
