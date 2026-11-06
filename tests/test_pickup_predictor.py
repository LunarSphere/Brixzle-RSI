"""Analytic pickup predictor (issue #12): Whitney 1982 peg-in-hole, hand-worked cases, then A/B.

Equation numbers are Whitney, "Quasi-Static Assembly of Compliantly Supported Rigid Parts", J. Dyn.
Sys. Meas. Control 104:65-77, 1982 (src/brixzle/rules/papers/quasistatic.pdf). Outputs are
predicted, quasi-static 2D.
"""

import json
import math

import pytest

from brixzle.engine.predictor import pickup as PP


def _peg(**kw):
    base = dict(r=1.0, R=1.3, w=0.0, alpha=math.radians(45), Lg=16.5, Kx=1.0, Kth=1.0, mu=0.3)
    base.update(kw)
    return PP.Peg(**base)


# --- the jamming diagram (eqs. 36-46, 55-56) ---


def test_one_point_corners_lie_on_the_jamming_parallelogram():
    # Fig. 10(b): Fx/Fz = 1/mu, M/(r Fz) = -1 (eqs. 38-39); Fig. 10(c): Fx/Fz = 1/mu,
    # M/(r Fz) = -(2 lambda + 1) (eqs. 42-43). Both are corners: Fz_req equals Fz exactly.
    r, mu, l = 1.0, 0.3, 4.0
    lam = l / (2 * r * mu)
    for m_over_rfz in (-1.0, -(2 * lam + 1)):
        fz = 1.0
        fx, m = fz / mu, m_over_rfz * r * fz
        assert PP.two_point_fz(fx, m, r, mu, l) == pytest.approx(fz)


def test_no_applied_lateral_force_or_moment_never_jams():
    assert PP.two_point_fz(0.0, 0.0, 1.0, 0.3, 4.0) == 0.0


# --- chamfer crossing (eqs. 7-8) ---


def test_chamfer_crossing_with_a_rigid_angular_support_translates_the_peg():
    # K_theta >> Kx Lg^2: no rotation, the compliance centre follows the chamfer: U = U0 - z/tan(a).
    g = _peg(Kth=1e9)
    th, U = PP.chamfer_crossing(g, e0=2.0, th0=0.01, z=1.0)
    U0 = 2.0 + g.Lg * 0.01
    assert th == pytest.approx(0.01, abs=1e-6)
    assert U == pytest.approx(U0 - 1.0 / math.tan(g.alpha), rel=1e-6)


def test_chamfer_crossing_with_a_free_angular_support_rotates_the_peg():
    # K_theta = 0: the compliance centre stays put (U = U0), the peg turns by z / (Lg tan a).
    g = _peg(Kth=0.0)
    th, U = PP.chamfer_crossing(g, e0=2.0, th0=0.0, z=1.0)
    assert U == pytest.approx(2.0)
    assert th == pytest.approx(1.0 / (g.Lg * math.tan(g.alpha)))


def test_a_chamfer_flatter_than_friction_jams():
    # Crossing needs B = sin a - mu cos a > 0, i.e. tan a > mu (a measured from the bore's end face).
    g = _peg(w=2.0, alpha=math.radians(15), mu=0.3)   # tan 15 deg = 0.27 < 0.3
    r = PP.insert(g, e0=1.0, th0=0.0, depth=25.0)
    assert r["ok"] is False and "chamfer" in r["reason"]


# --- one-point contact (eqs. 9-12) ---


def test_one_point_angle_at_the_chamfer_bottom_is_eq_12():
    # Eq. (10) at l = 0 must equal eq. (12): theta1 = theta0 + Kx(Lg - mu r) e1 / (Kx Lg (Lg - mu r) + K_theta).
    g = _peg(Kx=2.0, Kth=50.0, mu=0.4)
    e1, th0 = 0.5, 0.02
    th, _ = PP.one_point(g, e1 + g.c * g.R, th0, 0.0)
    expected = th0 + g.Kx * (g.Lg - g.mu * g.r) * e1 / (g.Kx * g.Lg * (g.Lg - g.mu * g.r) + g.Kth)
    assert th == pytest.approx(expected)


def test_one_point_with_a_free_angle_pivots_about_the_compliance_centre():
    # Hand case: mu = 0, K_theta = 0, l = 0: theta1 = theta0 + e1 / Lg (eq. 12 with mu = K_theta = 0).
    g = _peg(Kx=1.0, Kth=0.0, mu=0.0)
    th, _ = PP.one_point(g, 0.4 + g.c * g.R, 0.0, 0.0)
    assert th == pytest.approx(0.4 / g.Lg)


# --- two-point contact (eqs. 13-25) and wedging (eq. 49) ---


def test_two_point_onset_satisfies_the_geometry_it_was_solved_from():
    # l2 from the quadratic (21)-(23) must put the one-point angle (10) on l theta = c D (14).
    g = _peg(Kx=1.0, Kth=5.0, mu=0.3)
    e0, th0 = 0.35, 0.05
    l2, l2b = PP.two_point_onset(g, e0, th0)
    th, _ = PP.one_point(g, e0, th0, l2)
    assert l2 * th == pytest.approx(g.c * g.D, rel=1e-9)
    assert l2 < l2b


def test_two_point_onset_approaches_eq_25_for_a_stiff_angular_support():
    # Eq. (25): l2 ~= c D / theta0 when K_theta >> Kx Lg^2 (and the lateral error is small).
    g = _peg(Kx=1e-3, Kth=1e6, mu=0.3)
    th0 = 0.02
    l2, _ = PP.two_point_onset(g, 0.0, th0)
    assert l2 == pytest.approx(g.c * g.D / th0, rel=0.02)


def test_early_two_point_contact_wedges():
    # Eq. (49): wedging is possible when two-point contact starts at l2 <= mu D.
    g = _peg(Kx=1e-3, Kth=1e6, mu=0.5)
    th0 = 0.6   # l2 ~ c D / th0 = 0.6 / 0.6 = 1.0 mm <= mu D = 1.3 mm
    r = PP.insert(g, e0=0.0, th0=th0, depth=25.0)
    assert r["ok"] is False and r["reason"].startswith("wedged")
    r = PP.insert(g, e0=0.0, th0=0.02, depth=25.0, f_avail=lambda l: 1e9)
    assert r["ok"] is True


# --- entry ---


def test_a_tine_outside_the_mouth_lands_on_the_face():
    # The tine edge must land within the mouth: |e| < c R + w (R - r of clearance plus the chamfer).
    g = _peg(w=0.0)
    assert PP.insert(g, e0=0.29, th0=0.0, depth=25.0, f_avail=lambda l: 1e9)["ok"] is True
    r = PP.insert(g, e0=0.31, th0=0.0, depth=25.0, f_avail=lambda l: 1e9)
    assert r["ok"] is False and "face" in r["reason"]
    g = _peg(w=2.0)
    assert PP.insert(g, e0=2.2, th0=0.0, depth=25.0, f_avail=lambda l: 1e9)["ok"] is True


def test_insertion_needs_more_push_than_the_hover_spring_has():
    # A wide chamfer captures a 2 mm error, but if the drone can only push a tiny force it stalls.
    g = _peg(w=3.0, Kx=4e-4, Kth=193.0, mu=0.5)
    assert PP.insert(g, e0=2.0, th0=0.0, depth=25.0, f_avail=lambda l: 1e9)["ok"] is True
    r = PP.insert(g, e0=2.0, th0=0.0, depth=25.0, f_avail=lambda l: 1e-6)
    assert r["ok"] is False and "push" in r["reason"]


# --- the interface ---


def test_bore_chamfer_defaults_to_none_and_is_validated():
    from brixzle.families.interface import Bore

    b = Bore(x_mm=0.0, z_mm=0.0, d_mm=2.0, clearance_mm=0.3)
    assert b.chamfer_w_mm == 0.0
    with pytest.raises(ValueError):
        Bore(x_mm=0.0, z_mm=0.0, d_mm=2.0, clearance_mm=0.3, chamfer_w_mm=-1.0)
    with pytest.raises(ValueError):
        Bore(x_mm=0.0, z_mm=0.0, d_mm=2.0, clearance_mm=0.3, chamfer_w_mm=1.0, chamfer_deg=90.0)


# --- A/B ---


def test_ab_without_lead_ins_captures_only_within_the_clearance():
    # A/B's bores have 0.3 mm clearance and no chamfer: a tine more than 0.3 mm across lands on the
    # face. The sim (#9, runs/box-search-20261106T035007Z-ab-pickup-mean) jams at 0.46 mm across.
    from brixzle.families.ab.family import FAMILY as AB
    from brixzle.families.ab.params import V0

    assert PP.predict_pick(AB, V0, {"dx": 0.2, "dz": 0.0, "psi": 0.0})["outcome"] == "picked"
    r = PP.predict_pick(AB, V0, {"dx": 0.46, "dz": 0.0, "psi": 0.0})
    assert r["outcome"] == "jammed" and "face" in r["reason"]
    json.dumps(r, allow_nan=False)


def test_ab_predicted_pickup_box_is_zero_like_the_sim():
    # Sim (#9): scale 0.000 for the mean, p95 and worst pickup boxes (simulated, uncalibrated contact).
    from brixzle.engine import boxes as BX
    from brixzle.families.ab.family import FAMILY as AB
    from brixzle.families.ab.params import V0

    r = PP.predicted_pickup_box(AB, V0, BX.default_boxes()["pickup"]["mean"])
    assert r["scale"] == 0.0 and r["label"] == PP.LABEL
    assert r["seconds"] < 2.0
    json.dumps(r, allow_nan=False)


def test_a_chamfered_ab_bore_captures_a_larger_error():
    # Predicted only (the sim's bores have no chamfer yet): 3 mm, 45 deg lead-ins capture 2 mm across.
    from dataclasses import replace

    from brixzle.families.ab.family import FAMILY as AB
    from brixzle.families.ab.params import V0

    r = PP.predict_pick(AB, V0, {"dx": 2.0, "dz": 0.0, "psi": 0.0},
                        bores=lambda bs: tuple(replace(b, chamfer_w_mm=3.0, chamfer_deg=45.0) for b in bs))
    assert r["outcome"] == "picked", r["reason"]


def test_two_tines_share_the_drones_lateral_spring():
    # Both tines hang on one hover spring: in the same state they share its deflection, so a
    # two-tine fork needs the same axial push as a one-tine fork, not twice it.
    from dataclasses import replace

    from brixzle.families.ab.family import FAMILY as AB
    from brixzle.families.ab.params import V0

    ch = lambda bs: tuple(replace(b, chamfer_w_mm=6.0) for b in bs)
    one = lambda bs: ch(bs)[:1]
    sample = {"dx": 3.0, "dz": 0.0, "psi": 0.0}
    two_tines = PP.predict_pick(AB, V0, sample, mus=(0.5,), bores=ch)["per_mu"]["0.5"]["peak_push_N"]
    one_tine = PP.predict_pick(AB, V0, sample, mus=(0.5,), bores=one)["per_mu"]["0.5"]["peak_push_N"]
    assert two_tines == pytest.approx(one_tine, rel=1e-3)


def test_cli_records_a_new_run(tmp_path):
    args = ["--family", "ab", "--statistic", "mean", "--tol", "0.5", "--runs", str(tmp_path)]
    PP.main(args)
    PP.main(args)
    runs = sorted(tmp_path.iterdir())
    assert len(runs) == 2  # never overwrites
    assert json.loads((runs[0] / "result.json").read_text())["label"] == PP.LABEL
