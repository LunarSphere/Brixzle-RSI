"""Analytic placement predictor (issue #10): hand-worked cases, then families.

Contact-mode tests follow Balkcom & Trinkle 2002 (src/brixzle/rules/papers/Balkcom2002b.pdf), the
stability tests Pang & Trinkle 2000 (Theorem 1), the wedging test Whitney 1982 eq. 49
(quasistatic.pdf). Outputs are predicted, quasi-static 2D.
"""

import json
import math

import numpy as np
import pytest
from shapely.geometry import Polygon

from brixzle.engine.predictor import contact_modes as CM
from brixzle.engine.predictor import planar as PL
from brixzle.engine.predictor import predict as PR

G = np.array([0.0, -1.0, 0.0])  # unit weight, straight down


# --- contact modes (Balkcom & Trinkle 2002) ---


def test_balkcom_trinkle_section_4_disc_example():
    # Unit disc, M = I; fixel 1 touching at (0, -1) (normal +z into the disc), fixel 2 just right of
    # (1, 0) (normal -x), approaching. The paper's generators for mode 'na' at mu = 0.2:
    c1 = CM.Contact(point=(0.0, -1.0), normal=(0.0, 1.0))
    c2 = CM.Contact(point=(1.0, 0.0), normal=(-1.0, 0.0))
    sys = CM.ContactSystem([c1, c2], com=(0.0, 0.0), M=np.eye(3), L=1.0)
    g1, g2, g3 = np.array([1.0, 0.0, -1.0]), np.array([-0.2, -1.0, -0.2]), np.array([0.2, -1.0, 0.2])
    # Generators are boundary solutions (an acceleration is exactly zero, §6), so check them with
    # the strict bound k = 0, and strictly inside the cone with the default k.
    for g in (g1, g2, g3):
        assert sys.feasible("na", g, mu=0.2, k=0.0) is not None, g
    for g in (g1 + g2, g1 + g3, g1 + g2 + g3, 2 * g1 + 0.5 * g3):
        assert sys.feasible("na", g, mu=0.2) is not None, g
    assert sys.feasible("na", np.array([0.0, 1.0, 0.0]), mu=0.2) is None  # pulling off fixel 1
    assert sys.feasible("na", -g1, mu=0.2) is None  # moving away from fixel 2


def _incline(alpha_deg, size=10.0):
    a = math.radians(alpha_deg)
    n = np.array([-math.sin(a), math.cos(a)])   # up the slope's normal
    t = np.array([math.cos(a), math.sin(a)])
    p0, p1 = np.zeros(2), t * size
    com = (p0 + p1) / 2 + n * size / 2
    return CM.ContactSystem([CM.Contact(tuple(p0), tuple(n)), CM.Contact(tuple(p1), tuple(n))],
                            com=tuple(com), M=np.diag([1.0, 1.0, size**2 / 6]), L=size)


def test_block_on_an_incline_sticks_iff_mu_at_least_tan_alpha():
    sys = _incline(30.0)
    tan_a = math.tan(math.radians(30.0))
    for mu in (0.2, 0.5):
        assert not sys.weakly_stable(G, mu)
        mode, acc = sys.moving_mode(G, mu)
        assert set(mode) <= {"+", "-"} and acc[1] < 0  # slides, downhill
    for mu in (0.6, 0.8):
        assert sys.weakly_stable(G, mu)
    assert sys.stick_margin(G, tan_a - 0.02) > 0 > sys.stick_margin(G, tan_a + 0.02)


def _two_wall(d, l):
    # Whitney 1982: a peg of width d touching the left wall and the right wall l higher.
    c1 = CM.Contact(point=(-d / 2, 0.0), normal=(1.0, 0.0))
    c2 = CM.Contact(point=(d / 2, l), normal=(-1.0, 0.0))
    return CM.ContactSystem([c1, c2], com=(0.0, l / 2), M=np.diag([1.0, 1.0, d * d / 6]), L=d)


@pytest.mark.parametrize("l, mu, wedged", [(2.9, 0.3, True), (3.1, 0.3, False), (2.5, 0.2, False)])
def test_whitney_wedging_iff_l_below_mu_d(l, mu, wedged):
    sys = _two_wall(10.0, l)
    assert sys.wedged(mu) == wedged
    assert sys.opposing()
    # Friction alone can hold the peg against gravity exactly when it can wedge.
    assert sys.weakly_stable(G, mu) == wedged


def test_frictionless_stable_seat_has_a_positive_margin():
    # A V seat with both flanks flush (two contacts per 45-degree flank), normals up and inwards.
    s = math.sqrt(0.5)
    flank = [CM.Contact((-5.0, 5.0), (s, s)), CM.Contact((-1.0, 1.0), (s, s)),
             CM.Contact((1.0, 1.0), (-s, s)), CM.Contact((5.0, 5.0), (-s, s))]
    sys = CM.ContactSystem(flank, com=(0.0, 4.0), M=np.diag([1.0, 1.0, 10.0]), L=10.0)
    assert sys.frictionless_stable(G)
    assert sys.strongly_stable(G, 0.5)
    assert sys.seat_margin(G) > 0
    # Two point contacts span only a plane of wrenches: it can rock, so no ball fits (margin 0).
    rock = CM.ContactSystem([flank[0], flank[3]], com=(0.0, 4.0), M=np.diag([1.0, 1.0, 10.0]), L=10.0)
    assert rock.frictionless_stable(G) and rock.seat_margin(G) == 0


# --- geometry ---


def test_first_contact_is_the_vertical_gap():
    # Poses are (x, z, theta) of the body frame origin; theta rotates x toward z about that origin.
    floor = Polygon([(-50, -10), (50, -10), (50, 0), (-50, 0)])
    body = PL.Body(Polygon([(-5, 0), (5, 0), (5, 10), (-5, 10)]), com=(0.0, 5.0), mass=1.0, inertia=100 / 6)
    assert PL.drop_distance(body, (3.0, 7.0, 0.0), [floor]) == pytest.approx(7.0)
    # Tilted 10 degrees counter-clockwise: the bottom-left corner drops by 5 sin(10 deg) and touches first.
    th = math.radians(10.0)
    assert PL.drop_distance(body, (0.0, 7.0, th), [floor]) == pytest.approx(7.0 - 5 * math.sin(th))
    assert PL.drop_distance(body, (80.0, 7.0, 0.0), [floor]) is None  # misses the floor


# --- stepping on hand-worked scenes ---


def _v_scene(flank_deg, peg_depth=6.0):
    t = math.tan(math.radians(flank_deg))
    w = 20.0
    groove = Polygon([(-40, -30), (40, -30), (40, w * t), (w, w * t), (0, 0), (-w, w * t), (-40, w * t)])
    # A V peg with the same flank angle, tip down, origin at its tip.
    h = peg_depth
    peg = Polygon([(0, 0), (h / t, h), (h / t, h + 4), (-h / t, h + 4), (-h / t, h)])
    com = (0.0, float(peg.centroid.y))
    body = PL.Body(peg, com=com, mass=1.0, inertia=float(PL.polygon_inertia(peg, com)))
    return body, [groove]


def test_v_peg_slides_into_a_45_degree_groove_at_every_mu():
    body, env = _v_scene(45.0)
    out = PR.run_pose(body, env, start=(4.0, 10.0, 0.0), seat=(0.0, 0.0, 0.0), U=25.0)
    assert out["outcome"] == "seated", out["per_mu"]
    assert all(r["outcome"] == "seated" for r in out["per_mu"].values())
    assert out["seat_margin"] > 0
    assert out["label"] == "predicted, quasi-static 2D"


def test_seated_only_if_seated_at_every_mu():
    body, env = _v_scene(20.0)  # tan 20 = 0.36: slides at 0.2, sticks on the flank at 0.5
    out = PR.run_pose(body, env, start=(4.0, 10.0, 0.0), seat=(0.0, 0.0, 0.0), U=25.0)
    assert out["per_mu"]["0.2"]["outcome"] == "seated"
    assert out["per_mu"]["0.5"]["outcome"] != "seated"
    assert out["outcome"] != "seated"


def test_a_corner_does_not_balance_on_a_corner():
    # A square dropped with its corner exactly onto the apex of a ridge: a measure-zero balance. It
    # must slip off one side, not count as a (weakly stable) jam.
    ridge = Polygon([(-30, -20), (30, -20), (30, -10), (0, 0), (-30, -10)])
    sq = Polygon([(0, 0), (6, 6), (0, 12), (-6, 6)])  # a square standing on one corner
    body = PL.Body(sq, com=(0.0, 6.0), mass=1.0, inertia=float(PL.polygon_inertia(sq, (0.0, 6.0))))
    r = PR.simulate(body, [ridge], start=(0.0, 3.0, 0.0), seat=(0.0, 0.0, 0.0), mu=0.3, U=25.0)
    assert r["outcome"] != "jammed" or "weakly stable" not in (r["reason"] or ""), r["reason"]
    assert abs(r["end_pose"][0]) > 1.0  # it left the apex


def test_trace_records_contacts_modes_and_margins():
    body, env = _v_scene(45.0)
    out = PR.run_pose(body, env, start=(4.0, 10.0, 0.0), seat=(0.0, 0.0, 0.0), U=25.0, mus=(0.3,))
    trace = out["per_mu"]["0.3"]["trace"]
    assert trace and all({"contacts", "state", "stick_margin", "wedged", "opposing"} <= set(s) for s in trace)
    json.dumps(out, allow_nan=False)


# --- families ---


def test_wedge_family_seats_with_a_small_offset():
    # At i0 = 0: the wedge's base repeats every two voxels, so the engine's default column (i0 = 1,
    # as trials.DropRig, for A/B's one-voxel pattern) would sit on a ridge.
    from brixzle.families.wedge.family import FAMILY as WEDGE

    for course in (0, 1):
        out = PR.predict_pose(WEDGE, WEDGE.defaults, {"dx": 3.0, "pitch": 0.0, "clear": 1.0}, course=course, i0=0)
        assert out["outcome"] == "seated", (course, {m: r["outcome"] for m, r in out["per_mu"].items()})
        assert out["seat_margin"] > 0


def test_ab_aligned_pose_seats_on_both_courses():
    from brixzle.families.ab.family import FAMILY as AB
    from brixzle.families.ab.params import V0

    for course in (0, 1):
        out = PR.predict_pose(AB, V0, {"dx": 3.0, "pitch": 0.0, "clear": 5.0}, course=course)
        assert out["outcome"] == "seated", (course, {m: (r["outcome"], r.get("reason")) for m, r in out["per_mu"].items()})
        json.dumps(out, allow_nan=False)
