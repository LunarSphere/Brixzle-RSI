"""Capture features that vary along Y (issue #105): the Solid3D helper, the contract's depths, the
Y-funnel fixture's drops, its ring kit, and the jam analysis's sim-only report. Simulated drops are
simulated, uncalibrated contact."""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial import ConvexHull

from brixzle.agents import check_family as CF
from brixzle.engine import jam_analysis as JA
from brixzle.engine.sim import trials as T
from brixzle.families import builders as B
from brixzle.families import capture as CP
from brixzle.families import contract as C
from brixzle.families.ab.family import FAMILY as AB
from brixzle.families.interface import Solid3D

FIXTURES = Path(__file__).parent / "fixtures"


@pytest.fixture(scope="module")
def yfunnel():
    return CF.load_family(FIXTURES, "yfunnel")


def test_a_hull_solid_has_the_hull_volume():
    v = CP.prism_x([(0, 0), (4, 0), (6, 3), (0, 5)], -2.0, 3.0)
    assert CP.hull_solid(v).Volume() == pytest.approx(ConvexHull(v).volume, rel=1e-6)


def test_a_prism_needs_a_polygon_and_a_length():
    with pytest.raises(ValueError):
        CP.prism_x([(0, 0), (1, 0)], 0, 1)
    with pytest.raises(ValueError):
        CP.prism_x([(0, 0), (1, 0), (0, 1)], 1, 1)


def test_the_y_funnel_fixture_passes_the_contract_with_exact_collision(yfunnel):
    assert isinstance(yfunnel.geometry, Solid3D)
    assert C.check_family(yfunnel, samples=1) == []
    bodies = B.sim_bodies(yfunnel, yfunnel.defaults, "A")
    assert bodies.collision_source == "exact"
    core = B._pieces(yfunnel.geometry.section(yfunnel.defaults, "A").outer, CP.core_extrusion(yfunnel.geometry)(yfunnel.defaults))
    assert len(bodies.collision) == 3 * len(core)  # the core and a sheared copy on each side


def test_only_a_prismatic_family_counts_as_uniform_along_y(yfunnel):
    assert not B.varies_along_y(AB)
    assert B.varies_along_y(yfunnel)


def test_the_contract_checks_a_solid3d_brick_at_several_depths_not_only_y_0(yfunnel):
    """A Solid3D whose collision leaves out its skirts has no missing face in the y = 0 slice, so
    the old check passed it; sampling depths across Y finds the missing material."""
    g = yfunnel.geometry
    core_only = CP.CaptureGeometry(g.section, CP.core_extrusion(g), lambda p, part: [])
    broken = replace(yfunnel, geometry=Solid3D(solid=g.solid, collision=core_only.collision, section=g.section),
                     ring=None)
    found = [v for v in C.check_family(broken, samples=0) if v.check == "sim_bodies"]
    assert found and "not in the collision" in found[0].message
    inside, _ = B.contact_points(yfunnel, yfunnel.defaults, "A")
    assert len(np.unique(np.round(inside[:, 1], 3))) >= 3  # several depths, not one slice


def _drop(family, course, dy):
    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    sample = {**cfg.error.sample(None), "dy": float(dy)}
    return T.DropRig(family, family.defaults, cfg, course=course).drop(sample, trace=False)["outcome"]


@pytest.mark.parametrize("dy", [-10.0, 10.0])
def test_the_v_across_y_captures_a_drop_10_mm_off_in_y_on_both_courses(yfunnel, dy):
    # Twice the core's half-depth (D / 2 = 5 mm); A/B's map jams or falls at |y| >= 10 mm
    # (runs/map-20261114T002030501064Z-ab-mean, xy slice). Simulated, uncalibrated contact.
    assert _drop(yfunnel, 0, dy) == "seated"
    assert _drop(yfunnel, 1, dy) == "seated"


def test_a_ring_kit_builds_from_the_interlock_cell_of_a_solid3d_family(yfunnel):
    report = CF.ring_report(yfunnel)
    assert report["kit"] and report["built"], report


def test_kit_from_cell_still_refuses_a_solid3d_family_without_a_prismatic_interlock():
    from brixzle.engine import ring_cells
    from brixzle.families.pyramid.family import FAMILY as pyramid

    with pytest.raises(ValueError, match="prismatic interlock"):
        ring_cells.kit_from_cell(pyramid, lambda *a, **k: None)


def test_the_jam_analysis_reports_only_the_sim_for_a_family_that_varies_along_y(yfunnel):
    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    sample = {**cfg.error.sample(None), "dy": 14.0}
    out = JA.analyse_pose(yfunnel, yfunnel.defaults, 0, sample, sim_cfg=cfg)
    assert out["sim_only"] is True
    assert out["sites"] == [] and out["holding"] is None
    assert "varies along Y" in out["mechanism"]
    assert set(out["rest_pose"]) == {"pos_mm", "quat"}
