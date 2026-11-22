"""Passive station and fork guides (#196), simulated, uncalibrated contact."""
from dataclasses import replace
import json
import math
from pathlib import Path

import numpy as np
import pytest

from brixzle.agents.check_family import load_family
from brixzle.engine.heldmap import gripper_config
from brixzle.engine.sim import held


@pytest.fixture(scope="module")
def family():
    return load_family(Path(__file__).parent / "fixtures", "gentle_inboard_sill")


@pytest.fixture(scope="module")
def rig(family):
    return held.HeldRig(family, family.defaults, 0)


def p95():
    model = held.approach_error
    return model.load(model.DATA_DIR / "error-exp3-r4.json")["p95_mm"] * .001


@pytest.mark.parametrize("xy", [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)])
def test_station_registers_the_fork_before_lift(rig, xy):
    # Retain an empirical hover triplet on the longitudinal approach.
    hover = held.mission_samples(192)[0] if xy == (0, -1) else held.HeldSample()
    error = replace(hover, dx_m=xy[0]*p95(), dy_m=xy[1]*p95())
    row = rig.trial(held.HeldSample(), pickup_sample=error)
    grip = row["phases"]["grip"]
    assert grip["passed"], grip
    assert all(grip["tines_after_lift"])
    if xy == (0, 0):
        assert row["phases"]["transport"]["passed"], row["phases"]["transport"]
    assert grip["station_contact"] and grip["within_force_limit"]


def test_same_p95_misses_without_rails(family):
    rig = held.HeldRig(family, family.defaults, 0, held.HeldConfig(station=None))
    row = rig.trial(held.HeldSample(), pickup_sample=held.HeldSample(dx_m=p95()))
    assert not row["phases"]["grip"]["captured"]


def test_gripper_guides_are_physical_and_charge_payload(tmp_path, family, rig):
    path = tmp_path / "gripper.json"
    guide = dict(material="TPU", size_m=[.004, .006, .012],
                 centre_m=[0, -.026, .035], quat_wxyz=[math.sqrt(.5), 0, 0, math.sqrt(.5)])
    path.write_text(json.dumps(dict(guides=[guide])))
    cfg = gripper_config("unused", path)
    guided = held.HeldRig(family, family.defaults, 0, cfg)
    geom = guided.model.geom("fork_guide0")
    assert geom.bodyid == guided.jaw and geom.contype and geom.conaffinity
    np.testing.assert_allclose(geom.size, np.array(guide["size_m"])/2)
    np.testing.assert_allclose(geom.pos, guide["centre_m"])
    np.testing.assert_allclose(geom.quat, guide["quat_wxyz"])
    assert guided.payload - rig.payload == pytest.approx(np.prod(guide["size_m"])*cfg.guides[0].density_kg_m3)
    assert guided.force_cap < rig.force_cap


@pytest.mark.parametrize("change", [dict(material="PETG"), dict(size_m=[0,.01,.01]),
                                    dict(centre_m=[0,0,float("inf")]), dict(quat_wxyz=[0,0,0,0]),
                                    dict(density=0)])
def test_gripper_rejects_invalid_guides(tmp_path, change):
    path = tmp_path / "gripper.json"
    guide = dict(material="PLA", size_m=[.01]*3, centre_m=[0,0,0])
    path.write_text(json.dumps(dict(guides=[dict(guide, **change)])))
    with pytest.raises(ValueError):
        gripper_config("unused", path)


def test_station_exports_one_printable_solid(rig, tmp_path):
    import cadquery as cq
    import trimesh

    solid = rig.station_solid()
    assert len(solid.Solids()) == 1 and solid.isValid()
    path = tmp_path / "station.stl"
    cq.exporters.export(solid, str(path))
    mesh = trimesh.load_mesh(path)
    assert mesh.is_watertight and mesh.is_volume


def test_station_rejects_a_closed_mouth_or_a_shallow_guide(family):
    from brixzle.engine.sim.funnel import FunnelStation
    for station in (FunnelStation(mouth_width_m=.001), FunnelStation(taper_angle_rad=math.radians(20))):
        with pytest.raises(ValueError, match="station"):
            held.HeldRig(family, family.defaults, 0, held.HeldConfig(station=station))


def test_rail_bed_and_saddle_foot_share_the_print_bed(rig):
    pieces = rig.station_pieces_mm
    zmin = min(p[:, 2].min() for p in pieces)
    # Both ends have material on the first print layer.
    first_layer = np.vstack([p[np.isclose(p[:, 2], zmin)] for p in pieces])
    all_vertices = np.vstack(pieces)
    assert first_layer[:, 1].min() == pytest.approx(all_vertices[:, 1].min())
    assert first_layer[:, 1].max() == pytest.approx(all_vertices[:, 1].max())


def test_guides_must_connect_to_the_printed_fork(family):
    from brixzle.engine.sim.fork import ForkGuide
    guide = ForkGuide("PLA", (.002, .002, .01), (0., .1, .1))
    with pytest.raises(held.ForkFitError, match="attach"):
        held.HeldRig(family, family.defaults, 0, held.HeldConfig(guides=(guide,)))


def test_station_uses_the_design_friction_margin(family):
    from brixzle.engine.sim.funnel import FunnelStation
    from brixzle.families.rules import RuleSettings
    # Between the nominal solver friction threshold and the PLA design threshold.
    angle = math.atan(RuleSettings.mu_max) + math.radians(RuleSettings.slide_margin_deg - 1)
    cfg = held.HeldConfig(station=FunnelStation(taper_angle_rad=angle))
    with pytest.raises(ValueError, match="friction"):
        held.HeldRig(family, family.defaults, 0, cfg)
