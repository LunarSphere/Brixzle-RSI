"""Shared family builders checked against hand-worked numbers for the two dummy families.

wedge (Prismatic): a W of constant vertical thickness H0 over two voxels (one V per voxel, #82),
so its section has area 2 U H0 and centroid height U tan(theta)/4 + H0/2 (the underside's mean
height is half the V's rise, U tan(theta)/2).

pyramid (Solid3D): a U x U x H0 block with a square-frustum peg on top and the same frustum as a
pocket underneath, so its volume is the block's and the peg moves the centroid up by
V_frustum H0 / V_block. Both have two Y bores of radius d/2 + clearance through the depth.
"""

import math

import numpy as np
import pytest

from brixzle.families import builders as B
from brixzle.families.pyramid.family import FAMILY as PYRAMID
from brixzle.families.wedge.family import FAMILY as WEDGE

REL = 1e-3  # CadQuery tessellation of the round bores


def _bore_area(p):
    return 2 * math.pi * (p.bore_d / 2 + p.bore_clearance) ** 2


# --- wedge (Prismatic) ---


@pytest.fixture(scope="module")
def wedge():
    p = WEDGE.defaults
    return p, B.sim_bodies(WEDGE, p, "A")


def test_wedge_section_is_a_w_of_constant_thickness():
    p = WEDGE.defaults
    s = B.section(WEDGE, p, "A")
    assert s.outer.area == pytest.approx(2 * p.U * p.H0)
    assert [g.angle_deg for g in s.guide_faces] == pytest.approx([p.groove_deg] * 4)


def test_wedge_mass_properties_match_hand_values(wedge):
    p, bodies = wedge
    t = math.tan(math.radians(p.groove_deg))
    a, a_bore = 2 * p.U * p.H0, _bore_area(p)
    x_bore = p.bore_x_frac * p.U
    z_bore = abs(x_bore - p.U / 2) * t + p.bore_z_frac * p.H0
    volume = (a - a_bore) * p.D
    z_com = (a * (p.U * t / 4 + p.H0 / 2) - a_bore * z_bore) / (a - a_bore)
    assert bodies.volume_mm3 == pytest.approx(volume, rel=REL)
    assert bodies.mass_g == pytest.approx(volume * B.PLA_DENSITY, rel=REL)
    assert bodies.com_mm == pytest.approx([0.0, 0.0, z_com], rel=REL, abs=1e-3)


def test_wedge_collision_is_exact_convex_pieces(wedge):
    _, bodies = wedge
    assert bodies.collision_source == "exact"
    # The W, one band (#82): four convex pieces, the fewest any ring start gives (#98; five before).
    assert len(bodies.collision) == 4
    assert all(B.is_convex_vertex_set(v) for v in bodies.collision)


def test_wedge_inertia_is_symmetric_positive_definite(wedge):
    _, bodies = wedge
    inertia = np.asarray(bodies.inertia_g_mm2)
    assert np.allclose(inertia, inertia.T)
    assert np.all(np.linalg.eigvalsh(inertia) > 0)


# --- pyramid (Solid3D, no section or collision given) ---


@pytest.fixture(scope="module")
def pyramid():
    p = PYRAMID.defaults
    return p, B.sim_bodies(PYRAMID, p, "A")


def _frustum_volume(p):
    a1, a2 = (2 * p.peg_base_half) ** 2, (2 * p.peg_top_half) ** 2
    return p.peg_h / 3 * (a1 + a2 + math.sqrt(a1 * a2))


def test_pyramid_mass_properties_match_hand_values(pyramid):
    p, bodies = pyramid
    block = p.U * p.U * p.H0
    bores = _bore_area(p) * p.U
    z_bore = p.bore_z_frac * p.H0
    volume = block - bores
    z_com = (block * p.H0 / 2 + _frustum_volume(p) * p.H0 - bores * z_bore) / volume
    assert bodies.volume_mm3 == pytest.approx(volume, rel=REL)
    assert bodies.mass_g == pytest.approx(volume * B.PLA_DENSITY, rel=REL)
    assert bodies.com_mm == pytest.approx([0.0, 0.0, z_com], rel=REL, abs=1e-2)


def test_pyramid_section_falls_back_to_the_y0_slice():
    p = PYRAMID.defaults
    s = B.section(PYRAMID, p, "A")
    # The peg trapezoid on top and the pocket notch underneath have the same area.
    assert s.outer.area == pytest.approx(p.U * p.H0, rel=REL)
    side = math.degrees(math.atan2(p.peg_h, p.peg_base_half - p.peg_top_half))
    assert len(s.guide_faces) == 4  # two peg flanks, two pocket flanks
    assert [g.angle_deg for g in s.guide_faces] == pytest.approx([side] * 4, abs=0.01)


def test_pyramid_collision_is_decomposed_and_right_at_the_contact_points(pyramid):
    p, bodies = pyramid
    assert bodies.collision_source == "decomposed"
    inside, outside = B.contact_points(PYRAMID, p, "A")
    assert B.inside_any(inside, bodies.collision, tol=1e-3).all()       # no missing material at a face
    assert not B.inside_any(outside, bodies.collision, tol=-1e-3).any()  # no phantom ledge in the pocket


def test_decomposition_is_reproducible(pyramid):
    p, bodies = pyramid
    again = B.sim_bodies(PYRAMID, p, "A")
    assert len(again.collision) == len(bodies.collision)
    assert all(np.array_equal(a, b) for a, b in zip(again.collision, bodies.collision))


def test_pyramid_base_returns_exact_convex_pieces():
    p = PYRAMID.defaults
    pieces = B.base_pieces(PYRAMID, p, 4, 1)
    assert len(pieces) == 1 + 4  # slab plus one peg per voxel


# --- canonical convex cut (issue #98) ---


def _ab_outline(part):
    from brixzle.families.ab.family import FAMILY as AB

    return AB.geometry.section(AB.defaults, part).outer, AB.geometry.extrusion(AB.defaults)


def _variants(poly):
    """The same outline with its ring started at every vertex, reversed, and with collinear points added."""
    from shapely.geometry import Polygon

    ring = list(poly.exterior.coords)[:-1]
    n = len(ring)
    out = [Polygon(ring[k:] + ring[:k]) for k in range(n)]
    out.append(Polygon(ring[::-1]))
    extra = []
    for i, (a, b) in enumerate(zip(ring, ring[1:] + ring[:1])):
        extra.append(a)
        if i % 3 == 0:  # a point on the edge, as the Linux outline of part B has
            extra.append((a[0] + 0.37 * (b[0] - a[0]), a[1] + 0.37 * (b[1] - a[1])))
    out.append(Polygon(extra[5:] + extra[:5]))
    return out


@pytest.mark.parametrize("part", ["A", "B"])
def test_the_cut_does_not_depend_on_the_ring_start_direction_or_collinear_points(part):
    """Issue #98: part B's outline differs between platforms only in where its ring starts and in two
    points on straight edges, and that changed 6 of its 36 convex pieces."""
    poly, ext = _ab_outline(part)
    want = B._pieces(poly, ext)
    for v in _variants(poly):
        got = B._pieces(v, ext)
        assert len(got) == len(want)
        for g, w in zip(got, want):
            assert g.shape == w.shape and np.allclose(g, w, atol=1e-9)


@pytest.mark.parametrize("part", ["A", "B"])
def test_a_b_pieces_match_the_pinned_cut_on_every_platform(part):
    """Issue #98: A/B's collision pieces, pinned from macOS arm64 (2026-11-13), checked on Linux by CI.
    Written by tests/fixtures/make_ab_pieces_reference.py."""
    import json
    from pathlib import Path

    ref = json.loads((Path(__file__).parent / "fixtures" / "ab_pieces_reference.json").read_text())["parts"][part]
    poly, ext = _ab_outline(part)
    got = B._pieces(poly, ext)
    assert len(got) == len(ref)
    for g, w in zip(got, ref):
        assert np.allclose(g, np.asarray(w), atol=1e-6)
