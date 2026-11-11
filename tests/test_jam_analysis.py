"""Jam analysis inside the green zone (issue #49): which failures repeat, how they group into
bands, and which contacts hold the brick.

The reduction is tested on hand-built contacts: a block stuck on a shallow slope (one contact),
a block wedged between two walls (two contacts), and duplicate points on one flat patch merging
to its two end points. Two short tests run A/B V0 in the sim.
Sim results are simulated, uncalibrated contact; predictor results are predicted, quasi-static 2D.
"""

import json
import math

import numpy as np
import pytest
from shapely.geometry import Polygon

from brixzle.engine import boxes as BX
from brixzle.engine import jam_analysis as JA
from brixzle.engine.sim import trials as T
from brixzle.families.ab.family import FAMILY as AB
from brixzle.families.ab.params import V0
from brixzle.families.interface import GuideFace

W = 0.1  # N, the hand-built block's weight
CFG = JA.JamConfig()


def _contact(x, y, z, normal, force, other="base"):
    """A resting contact as trials.rest_contacts gives it: the normal points into the brick, and
    the force is the one on the brick."""
    n = np.asarray(normal, float) / np.linalg.norm(normal)
    f = np.asarray(force, float)
    return {"pos_mm": [x, y, z], "normal": n.tolist(), "force_N": f.tolist(), "normal_N": float(f @ n),
            "dist_mm": -0.001, "other": other, "piece": 0, "other_geom": f"{other}_0"}


# --- hand-built case 1: a block stuck on a shallow slope ---

BETA = math.radians(15.0)  # tan 15 deg = 0.268: holds at mu 0.35, slides at mu 0.2
UP = np.array([math.cos(BETA), 0.0, math.sin(BETA)])      # along the slope, uphill
N_SLOPE = np.array([-math.sin(BETA), 0.0, math.cos(BETA)])


def _block_on_slope():
    """A 20 mm block resting on its whole face: MuJoCo gives a point at each corner of the patch
    (and here one of them twice). Normal force W cos(beta), friction W sin(beta) uphill."""
    a, b = np.zeros(3), 20.0 * UP
    f = (W * math.cos(BETA) * N_SLOPE + W * math.sin(BETA) * UP) / 5
    corners = [(a, -10.0), (a, 10.0), (b, -10.0), (b, 10.0), (a, 10.0)]
    contacts = [_contact(c[0], y, c[2], N_SLOPE, f) for c, y in corners]
    com = (a + b) / 2 + 5.0 * N_SLOPE
    return contacts, (com[0], com[2])


def test_duplicate_points_on_the_slope_merge_into_one_face_with_two_end_points():
    contacts, _ = _block_on_slope()
    (site,) = JA.sites(contacts, W, CFG)
    assert site["kind"] == "face" and site["members"] == 5
    ends = sorted(site["points"])
    assert ends[0] == pytest.approx([0.0, 0.0], abs=1e-6)
    assert ends[1] == pytest.approx([20 * math.cos(BETA), 20 * math.sin(BETA)], abs=1e-6)
    assert site["normal"] == pytest.approx([-math.sin(BETA), math.cos(BETA)])
    assert site["normal_N"] == pytest.approx(W * math.cos(BETA))


def test_a_block_on_a_shallow_slope_is_held_by_one_contact():
    contacts, com = _block_on_slope()
    out = JA.reduce_contacts(contacts, com, W, mu=0.35, L=25.0, cfg=CFG)
    assert out["three_d"] is False
    h = out["holding"]
    assert h["size"] == 1 and h["sites"] == [0]
    assert "one contact" in out["mechanism"] and "R6" in out["mechanism"]
    assert h["needs_friction"] is True          # normal forces alone cannot hold it on a slope
    assert h["stick_margin"] < 0                # gravity is inside the sticking cone
    assert h["wedged"] is False


def test_the_same_block_is_not_held_once_friction_is_below_the_slope():
    contacts, com = _block_on_slope()
    out = JA.reduce_contacts(contacts, com, W, mu=0.2, L=25.0, cfg=CFG)
    assert out["holding"] is None and "not explained" in out["mechanism"]
    assert out["all_sites"]["stick_margin"] > 0  # gravity is outside the cone: it should slide


# --- hand-built case 2: a block wedged between two walls ---


def _block_between_walls(mu=0.35):
    """Two walls squeeze a 10 mm block; friction on both carries its weight. Each wall touches at
    one height, seen by MuJoCo as a point at each end of the depth."""
    squeeze = 1.2 * W / (2 * mu)
    left = [_contact(0.0, y, 5.0, (1, 0, 0), (squeeze / 2, 0, W / 4), "base") for y in (-10.0, 10.0)]
    right = [_contact(10.0, y, 5.0, (-1, 0, 0), (-squeeze / 2, 0, W / 4), "brick0") for y in (-10.0, 10.0)]
    return left + right, (5.0, 5.0)


def test_a_block_wedged_between_two_walls_is_held_by_two_contacts():
    contacts, com = _block_between_walls()
    out = JA.reduce_contacts(contacts, com, W, mu=0.35, L=25.0, cfg=CFG)
    assert [s["kind"] for s in out["sites"]] == ["point", "point"]
    h = out["holding"]
    assert h["size"] == 2 and sorted(h["sites"]) == [0, 1]
    assert h["opposing"] is True and h["wedged"] is True  # Whitney 1982: the pair can squeeze it
    assert "two contacts" in out["mechanism"] and "opposite sides" in out["mechanism"]
    assert h["needs_friction"] is True


def test_neither_wall_alone_holds_the_block():
    contacts, com = _block_between_walls()
    one_wall = [c for c in contacts if c["other"] == "base"]
    assert JA.reduce_contacts(one_wall, com, W, mu=0.35, L=25.0, cfg=CFG)["holding"] is None


# --- hand-built case 3: duplicate points on one flat patch ---


def test_duplicate_points_on_a_flat_patch_merge_to_its_two_end_points():
    pts = [(0.0, -10), (0.0, 10), (2.5, -10), (5.0, 0), (5.0, 0), (7.5, 10), (10.0, -10), (10.0, 10)]
    contacts = [_contact(x, y, 0.0, (0, 0, 1), (0, 0, W / len(pts))) for x, y in pts]
    (site,) = JA.sites(contacts, W, CFG)
    assert site["kind"] == "face" and site["members"] == 8
    assert sorted(site["points"]) == [pytest.approx([0.0, 0.0]), pytest.approx([10.0, 0.0])]
    assert site["normal_N"] == pytest.approx(W)
    out = JA.reduce_contacts(contacts, (5.0, 4.0), W, mu=0.35, L=25.0, cfg=CFG)
    assert out["holding"]["size"] == 1 and out["holding"]["needs_friction"] is False  # a flat seat
    assert "without friction" in out["mechanism"]


# --- what is kept, and the 3D check ---


def test_the_force_threshold_applies_to_a_site_not_to_each_point():
    """A seated face spreads the load over many points, each far below 5% of the weight."""
    many = [_contact(x, y, 0.0, (0, 0, 1), (0, 0, W / 100)) for x in np.linspace(0, 10, 50) for y in (-10, 10)]
    stray = [_contact(30.0, 0.0, 8.0, (-1, 0, 0), (-0.01 * W, 0, 0))]  # 1% of the weight, on its own
    sites = JA.sites(many + stray, W, CFG)
    assert len(sites) == 1 and sites[0]["members"] == 100


def test_the_reduction_is_plain_json_data():
    contacts, com = _block_on_slope()
    out = JA.reduce_contacts(contacts, com, W, mu=0.35, L=25.0, cfg=CFG)
    json.dumps(out, allow_nan=False)
    assert out["max_point_share"] == pytest.approx(math.cos(BETA) / 5) and out["contacts_touching"] == 5


def test_contacts_that_do_not_touch_or_carry_no_force_are_dropped():
    lifted = dict(_contact(0, 0, 0, (0, 0, 1), (0, 0, W)), dist_mm=0.2)
    limp = _contact(5, 0, 0, (0, 0, 1), (0, 0, 0.0))
    assert JA.sites([lifted, limp], W, CFG) == []


def test_mirror_contacts_on_a_gutter_cancel_in_y_and_are_not_a_3d_effect():
    """Two faces leaning 30 deg across Y, one each side of the centre line: each carries a large
    Y force, but together they push straight up. Their friction acts on the full normal force."""
    lean = math.radians(30.0)
    fn = W / (2 * math.cos(lean))
    pair = [_contact(5.0, s * 6.0, 0.0, (0, -s * math.sin(lean), math.cos(lean)),
                     fn * np.array([0, -s * math.sin(lean), math.cos(lean)])) for s in (-1, 1)]
    out = JA.reduce_contacts(pair, (5.0, 4.0), W, mu=0.35, L=25.0, cfg=CFG)
    assert out["y_share_raw"] == pytest.approx(math.sin(lean)) and out["y_share"] == pytest.approx(0.0, abs=1e-9)
    assert out["three_d"] is False
    assert out["sites"][0]["mu_scale"] == pytest.approx(1 / math.cos(lean))


def test_a_large_share_of_force_along_y_is_a_3d_effect_and_stops_the_reduction():
    contacts = [_contact(0.0, 5.0, 0.0, (0.6, -0.5, 0.62), (0.03, -0.05, 0.06)),
                _contact(10.0, -12.5, 6.0, (-0.7, 0.3, 0.64), (-0.03, 0.02, 0.04))]
    out = JA.reduce_contacts(contacts, (5.0, 4.0), W, mu=0.35, L=25.0, cfg=CFG)
    assert out["three_d"] is True and out["holding"] is None
    assert out["mechanism"].startswith("3D effect")
    assert len(out["sites"]) == 2  # the contacts are still reported


def test_a_brick_that_ends_out_of_plane_is_a_3d_effect_whatever_its_forces():
    contacts, com = _block_on_slope()
    out = JA.reduce_contacts(contacts, com, W, mu=0.35, L=25.0, cfg=CFG, out_of_plane={"dy_mm": 1.6, "tilt_deg": 3.7})
    assert out["three_d"] is True and out["holding"] is None and "rolled or yawed 3.7 deg" in out["mechanism"]


# --- repeatable or one-off, and bands ---


def _reruns(fails):
    """fails: {mu: number of failing seeds out of 3}."""
    return [{"mu": mu, "seed": s, "outcome": "jammed" if s < fails.get(mu, 0) else "seated"}
            for mu in CFG.mus for s in range(CFG.seeds)]


def test_a_cell_that_fails_at_one_friction_on_every_seed_is_repeatable():
    r = JA.repeatability(_reruns({0.5: 3}), CFG)
    assert r["repeatable"] is True and r["repeat_rate"] == pytest.approx(3 / 12)
    assert r["per_mu"] == {"0.2": 0.0, "0.3": 0.0, "0.4": 0.0, "0.5": 1.0}


def test_a_lone_failure_that_does_not_come_back_is_a_one_off():
    r = JA.repeatability(_reruns({0.3: 1}), CFG)
    assert r["repeatable"] is False and r["repeat_rate"] == pytest.approx(1 / 12)


def test_repeatable_cells_group_into_connected_bands():
    cells = [{"slice": "xz", "course": 1, "cell": c} for c in ([3, 1], [4, 2], [4, 3], [9, 0])]
    cells.append({"slice": "xt", "course": 1, "cell": [4, 2]})
    cells.append({"slice": "xz", "course": 0, "cell": [4, 2]})
    groups = JA.bands(cells)
    assert sorted(sorted(tuple(c["cell"]) for c in g) for g in groups) == [
        [(3, 1), (4, 2), (4, 3)], [(4, 2)], [(4, 2)], [(9, 0)]]  # diagonal neighbours join; slices and courses don't


def test_the_representative_is_the_cell_nearest_the_bands_centre():
    band = [{"cell": [i, 0]} for i in range(5)] + [{"cell": [2, 1]}]
    assert JA.representative(band)["cell"] == [2, 0]


# --- naming the faces ---

SQUARE = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
RAMP = Polygon([(0, 0), (10, 0), (10, 4), (0, 12)])  # a top face falling to the right


def test_a_contact_in_the_middle_of_an_edge_names_the_face():
    f = JA.name_feature(SQUARE, (), (5.0, 10.02))
    assert f["feature"] == "face" and f["angle_deg"] == pytest.approx(0.0) and f["facing"] == "up"
    assert f["from_mm"] in ([0.0, 10.0], [10.0, 10.0]) and "face" in f["label"]
    f = JA.name_feature(SQUARE, (), (10.0, 3.0))
    assert f["facing"] == "right" and f["angle_deg"] == pytest.approx(90.0)


def test_a_contact_near_a_vertex_names_the_corner():
    f = JA.name_feature(SQUARE, (), (0.2, 0.1))
    assert f["feature"] == "corner" and f["at_mm"] == [0.0, 0.0] and f["convex"] is True
    assert "corner" in f["label"]


def test_a_guide_face_is_named_as_one():
    guide = GuideFace((10.0, 4.0), (0.0, 12.0))
    f = JA.name_feature(RAMP, (guide,), (5.0, 8.0))
    assert f["guide"] is True and "guide" in f["label"]
    assert f["angle_deg"] == pytest.approx(math.degrees(math.atan2(8, 10)))
    assert JA.name_feature(RAMP, (guide,), (5.0, 0.0))["guide"] is False


# --- A/B V0 in the sim ---

SIM_CFG = T.TrialConfig(error=T.ErrorModel(ideal=True))
BOX = BX.default_boxes()["placement"]["mean"]


def _sample(dx, h, pitch=0.0):
    e = SIM_CFG.error
    return {"dx": e.aim_bias + dx, "dy": 0.0, "aim": e.aim_bias, "yaw": 0.0, "roll": 0.0, "pitch": pitch,
            "clear": e.clearance[0] + h, "vel": [0.0, 0.0, 0.0]}


def test_resting_contacts_balance_the_bricks_weight():
    rig = T.DropRig(AB, V0, SIM_CFG, course=1)
    row = rig.drop(_sample(0.0, 5.0), trace=False, rest=True)
    rest = row["rest"]
    assert row["outcome"] == "seated" and len(rest["contacts"]) > 50  # a seated face touches at many points
    # ... each carrying a small share of the weight, which is why the force threshold applies to
    # merged sites. The most loaded point carried 3.0% on macOS arm64 and 7.4% on the Linux CI
    # runner (2026-11-11): the split between redundant contacts is not unique.
    assert max(c["normal_N"] for c in rest["contacts"]) < 0.15 * rest["weight_N"]
    total = np.sum([c["force_N"] for c in rest["contacts"]], axis=0)
    assert total == pytest.approx([0.0, 0.0, rest["weight_N"]], abs=0.05 * rest["weight_N"])
    assert {"pos_mm", "normal", "force_N", "normal_N", "dist_mm", "other"} <= set(rest["contacts"][0])
    assert all(c["other"] in ("brick0", "base") for c in rest["contacts"])  # never the floor
    assert "rest" not in rig.drop(_sample(0.0, 5.0), trace=False)  # only on request


@pytest.fixture(scope="module")
def ab_analysis():
    """A map with one grid and the two failures the real A/B mean-box map found inside the green
    zone on 2026-11-11 (runs/map-20261111T194411009874Z-ab-mean): course 1, X-Z slice."""
    steps = {"x": 1.0, "z": 5.0}
    grid = {"slice": "xz", "course": 1, "axes": ["x", "z"], "step": steps, "fixed": {"t": 0.0},
            "inside_failures": [{"cell": [7, 3], "x": 7.0, "z": 15.0, "outcome": "jammed"},
                                {"cell": [10, 4], "x": 10.0, "z": 20.0, "outcome": "jammed"}]}
    sim_map = {"family": "ab", "family_version": AB.version, "params": V0.to_dict(), "box": BOX.to_dict(),
               "aim": {"x": 0.0, "z": BOX.half_widths["z_mm"], "t": 0.0}, "grids": [grid]}
    return JA.analyse(AB, V0, sim_map, cfg=JA.JamConfig(seeds=2, mus=(0.35, 0.5)), sim_cfg=SIM_CFG, workers=1)


def test_the_ab_analysis_lists_its_bands_with_repeat_rate_type_and_contacts(ab_analysis):
    a = ab_analysis
    assert a["labels"] == {"sim": "simulated, uncalibrated contact", "pred": "predicted, quasi-static 2D"}
    assert len(a["cells"]) == 2 and all(len(c["reruns"]) == 4 for c in a["cells"])
    assert len(a["bands"]) + len(a["one_offs"]) >= 1
    for band in a["bands"]:
        assert band["failure_type"] in ("jammed", "wrong_valley", "fell")
        assert 0 < band["repeat_rate"] <= 1 and set(band["per_mu"]) == {"0.35", "0.5"}
        assert isinstance(band["three_d"], bool) and band["mechanism"]
        assert band["sites"], "the contacts at rest are reported, named, even for a 3D effect"
        for s in band["sites"]:
            assert s["on_brick"]["label"] and s["on_support"]["label"]
            assert len(s["normal"]) == 2 and all(len(p) == 2 for p in s["points"])
        if band["three_d"]:
            assert band["holding"] is None
    json.dumps(a, allow_nan=False)


def test_an_ab_jam_at_the_capture_edge_is_reduced_and_its_faces_named():
    """Just past A/B's +X capture edge on course 1 (the map's 'J' column at +14 mm) the brick jams
    about 22 mm off its seat. On macOS arm64 it rests in plane and reduces to a holding set of two
    contacts, both sticking; on the Linux CI runner the same pose ends as a 3D effect (2026-11-11).
    A jam is sensitive to the platform's arithmetic, so the test accepts either and checks that
    each is reported properly."""
    out = JA.analyse_pose(AB, V0, course=1, sample=_sample(14.0, 0.0), sim_cfg=SIM_CFG)
    assert out["outcome"] == "jammed" and out["mu"] == pytest.approx(0.35)
    assert out["sites"], "the contacts at rest are reported either way"
    for s in out["sites"]:
        assert s["on_brick"]["part"] == "B" and s["on_support"]["part"] in ("A", "base")
        assert s["on_brick"]["label"].startswith("B: ") and s["normal_N"] > 0.05 * out["weight_N"]
    h = out["holding"]
    if out["three_d"]:
        assert h is None and out["mechanism"].startswith("3D effect")
    else:
        assert h is not None and 1 <= h["size"] <= len(out["sites"])
        assert isinstance(h["stick_margin"], float) and isinstance(h["wedged"], bool)
        assert out["all_sites"]["weakly_stable"] is True  # every site together holds if a subset does
    json.dumps(out, allow_nan=False)


def test_the_analysis_is_written_beside_the_map_and_never_overwritten(ab_analysis, tmp_path):
    path = JA.write_analysis(ab_analysis, tmp_path)
    assert path == tmp_path / "jam_analysis.json" and json.loads(path.read_text(encoding="utf-8"))["bands"] is not None
    with pytest.raises(FileExistsError):
        JA.write_analysis(ab_analysis, tmp_path)
