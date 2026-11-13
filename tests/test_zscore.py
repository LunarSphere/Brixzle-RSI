"""Objective Z (issue #61): (seated - jammed) / drops inside each target box.

Most tests use made-up drops, so they need no sim. One test drops the A/B baseline at coarse
settings to check that the direct Z (the in-box cells alone) matches the full map's Z.
Sim results are simulated, uncalibrated contact.
"""

import pickle

import pytest

from brixzle.engine import boxes as BX
from brixzle.engine import sim_map as SM
from brixzle.engine import zscore as ZS

BOXES = BX.default_boxes()["placement"]
MEAN, P95, WORST = BOXES["mean"], BOXES["p95"], BOXES["max"]
C0 = 5.0  # the trial's minimum release clearance (trials.ErrorModel.clearance[0])


def _drop(outcome, x=0.0, z=1.0, t=0.0, **extra):
    return {"outcome": outcome, "dx_mm": x, "clear_mm": C0 + z, "pitch_deg": t, **extra}


# --- counting ---


def test_seated_counts_plus_one_jammed_minus_one_and_the_rest_zero():
    c = ZS.counts(["seated"] * 6 + ["jammed"] * 2 + ["fell", "wrong_valley"])
    assert c == {"Z": (6 - 2) / 10, "S": 6, "J": 2, "N": 10, "fell": 1, "wrong_valley": 1, "other": 0,
                 "passed": False}


def test_z_is_one_only_when_every_drop_seats():
    assert ZS.counts(["seated"] * 5)["Z"] == 1.0 and ZS.counts(["seated"] * 5)["passed"]
    assert ZS.counts(["jammed"] * 5)["Z"] == -1.0
    assert ZS.counts(["fell"] * 5)["Z"] == 0.0


def test_an_outcome_that_is_not_named_counts_as_other_and_zero():
    c = ZS.counts(["seated", "unstable"])
    assert c["other"] == 1 and c["Z"] == 0.5


def test_z_of_no_drops_is_refused():
    with pytest.raises(ValueError, match="no drops"):
        ZS.counts([])


def test_only_drops_inside_the_box_count():
    drops = [_drop("seated", x=0.0), _drop("jammed", x=14.0),   # inside the mean box (x +-14.8)
             _drop("jammed", x=20.0),                            # outside it in X
             _drop("jammed", z=7.0),                             # above it (z 0 to 6.2)
             _drop("jammed", t=2.0)]                             # outside it in pitch (+-0.2 deg)
    z = ZS.z_from_drops(drops, {"mean": MEAN}, C0)["mean"]
    assert (z["S"], z["J"], z["N"], z["Z"]) == (1, 1, 2, 0.0)


def test_each_target_box_counts_its_own_drops():
    drops = [_drop("seated", x=0.0), _drop("jammed", x=20.0), _drop("fell", x=35.0)]
    z = ZS.z_from_drops(drops, BOXES, C0)
    assert z["mean"]["N"] == 1 and z["mean"]["Z"] == 1.0
    assert z["p95"]["N"] == 2 and z["p95"]["Z"] == 0.0
    assert z["max"]["N"] == 3 and z["max"]["Z"] == 0.0


def test_zoom_drops_do_not_count():
    """A zoom is an override at a finer step; counting it would let a caller weight Z."""
    drops = [_drop("jammed"), _drop("seated", zoom="look closer at the seat")]
    assert ZS.z_from_drops(drops, {"mean": MEAN}, C0)["mean"]["N"] == 1


def test_a_shrinking_green_zone_cannot_raise_z():
    """Z is over the target box, so failures at its edge count however narrow the zone is."""
    wide = [_drop("seated", x=x) for x in range(-14, 15)]
    narrow = [_drop("seated" if abs(x) < 5 else "fell", x=x) for x in range(-14, 15)]
    assert ZS.z_from_drops(narrow, {"mean": MEAN}, C0)["mean"]["Z"] < ZS.z_from_drops(wide, {"mean": MEAN}, C0)["mean"]["Z"]


# --- the cells inside the boxes ---


def test_box_cells_lie_inside_a_box_and_cover_every_slice_and_course():
    cells = ZS.box_cells(MEAN, {"mean": MEAN}, U=25.0)
    assert {(c["slice"], c["course"]) for c in cells} == {(s, k) for s in SM.SLICES for k in (0, 1)}
    assert all(ZS.inside(c["pose"], MEAN) for c in cells)
    # 1 mm cells in X over +-14.8 mm: 29 columns; 5 mm cells in Z over 0 to 6.2 mm: 2 rows; pitch 0 only.
    # xy (#86): 4 mm X and 2 mm Y cells, 7 x 15 of them in the square, 89 inside the r = 14.8 mm disc.
    per_course = {s: sum(c["slice"] == s and c["course"] == 0 for c in cells) for s in SM.SLICES}
    assert per_course == {"xz": 29 * 2, "xt": 29, "zt": 2, "xy": 89}


def test_the_cells_of_the_largest_box_hold_those_of_the_smaller_ones():
    big = {(c["slice"], c["course"], tuple(c["cell"])) for c in ZS.box_cells(MEAN, BOXES, U=25.0)}
    small = {(c["slice"], c["course"], tuple(c["cell"])) for c in ZS.box_cells(MEAN, {"mean": MEAN}, U=25.0)}
    assert small < big


def test_held_out_poses_are_seeded_inside_the_box_and_differ_from_the_grid():
    cells = ZS.box_cells(MEAN, {"mean": MEAN}, U=25.0)
    a = ZS.held_out_poses(cells, MEAN, U=25.0, seed=1)
    b = ZS.held_out_poses(cells, MEAN, U=25.0, seed=1)
    c = ZS.held_out_poses(cells, MEAN, U=25.0, seed=2)
    assert a == b and a != c and len(a) == len(cells)
    assert all(ZS.inside(p, MEAN) for p in a)
    assert any(p != cell["pose"] for p, cell in zip(a, cells))


# --- against the sim ---


COARSE = SM.MapConfig(x_step_U=0.2, z_step_U=0.5, theta_step_deg=5.0, z_limit_U=0.5, theta_limit_deg=5.0,
                      y_step_U=0.2)


def test_the_direct_z_matches_the_full_maps_z():
    """A drop depends only on its pose, so dropping the in-box cells alone gives the map's Z."""
    from brixzle.families.ab.family import FAMILY as AB
    from brixzle.families.ab.params import V0

    m, drops = SM.sim_map(AB, V0, MEAN, cover=(MEAN, P95), config=COARSE, config_reason="coarse cells for a short test",
                          workers=4)
    from_map = ZS.z_from_drops(drops, {"mean": MEAN, "p95": P95}, m["config"]["error"]["clearance"][0])
    score = ZS.ZScore(AB, MEAN, {"mean": MEAN, "p95": P95}, config=COARSE, workers=4)
    assert pickle.loads(pickle.dumps(score)) == score  # it has to reach another process for the tune
    direct = score(V0)
    assert direct["targets"] == from_map
    assert direct["score"] == from_map["mean"]["Z"]
    assert m["cover"] == ["mean", "p95"]



# --- the X-Y disc (issue #86) ---


def test_a_drop_counts_only_inside_the_x_y_disc():
    r = MEAN.half_widths["x_mm"]
    assert not ZS.inside({"x": 0.8 * r, "y": 0.8 * r, "z": 1.0, "t": 0.0}, MEAN)  # 1.13 r from the aim
    assert ZS.inside({"x": 0.6 * r, "y": 0.6 * r, "z": 1.0, "t": 0.0}, MEAN)      # 0.85 r
    assert ZS.inside({"x": r, "z": 1.0, "t": 0.0}, MEAN)                          # no y: on the X axis


def test_z_x_only_is_the_z_of_the_drops_with_no_y_offset():
    flat = [_drop("seated", x=1.0), _drop("jammed", x=2.0), _drop("seated", x=-3.0), _drop("fell", x=4.0)]
    old = {"Z": (2 - 1) / 4}  # what Z gave before #86: every map drop had y = 0
    off = [_drop("jammed", x=1.0, dy_mm=4.0), _drop("jammed", x=0.0, dy_mm=-6.0)]
    both = ZS.z_from_drops(flat + off, {"mean": MEAN}, C0)
    only = ZS.z_from_drops(flat + off, {"mean": MEAN}, C0, x_only=True)
    assert only["mean"]["Z"] == old["Z"] == ZS.z_from_drops(flat, {"mean": MEAN}, C0)["mean"]["Z"]
    assert both["mean"]["N"] == 6 and both["mean"]["Z"] == pytest.approx((2 - 3) / 6)
    # The xy slice's y = 0 row repeats the xt slice's poses at coarser X: it is not part of the X-only Z.
    row = [_drop("jammed", x=4.0, slice="xy", dy_mm=0.0)]
    assert ZS.z_from_drops(flat + row, {"mean": MEAN}, C0, x_only=True)["mean"]["N"] == 4


def test_a_straight_extruded_v_scores_lower_with_y_than_without(stack_v):
    """The stack-only V has no Y guide: a Y miss jams where an X-only drop seats (simulated, uncalibrated
    contact; the issue's table shows the same for the chevron)."""
    from brixzle.engine import quick_check as QC
    from brixzle.engine.sim import trials as T

    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    rig = T.DropRig(stack_v, stack_v.defaults, cfg, course=0)
    drops = []
    for dy in (0.0, 3.0, -3.0, 6.0, -6.0):
        s = SM.cell_sample(cfg, "xy", (0, int(dy)), {"x": 4.0, "y": 1.0, "z": 5.0, "t": 2.0}, SM.aim_point(MEAN))
        assert s["dy"] == dy  # the Y offset reaches the rig
        drops.append(QC.drop_row(len(drops), 0, (0.0, 0.0, 0.0), s, rig.drop(s, trace=False)))
    z = ZS.z_from_drops(drops, {"mean": MEAN}, C0)["mean"]
    zx = ZS.z_from_drops(drops, {"mean": MEAN}, C0, x_only=True)["mean"]
    assert zx["N"] == 1 and z["N"] == 5 and z["Z"] < zx["Z"]
