"""Full sim map on a fitted grid (issue #48): growing, trimming, the one-row red margin, the
guardrail and the overrides.

The fit is tested on synthetic outcomes (no sim). One coarse A/B V0 map runs in the sim.
Sim results are simulated, uncalibrated contact.
"""

import json

import pytest

from brixzle.engine import boxes as BX
from brixzle.engine import quick_check as QC
from brixzle.engine import sim_map as SM
from brixzle.engine.sim import trials as T
from brixzle.families.ab.family import FAMILY as AB
from brixzle.families.ab.params import V0

BOX = BX.default_boxes()["placement"]["mean"]
WIDE = ((-50, 50), (0, 20))


def _fit(green, start, must, limit=WIDE, grow=2, budget=None):
    """Fit a grid whose cell (i, j) is seated when green(i, j), and jammed otherwise."""
    fit = SM.GridFit(start, must, limit, grow=grow, budget=budget)
    return SM.run_fit(fit, lambda c: "seated" if green(*c) else "jammed")


def _extent(fit):
    return tuple(tuple(e) for e in fit.extent)


# --- growing ---


def test_an_edge_with_green_on_it_grows_until_it_is_red():
    fit = _fit(lambda i, j: abs(i) <= 7, start=((-3, 3), (0, 2)), must=((-3, 3), (0, 2)))
    # One red column past the last green one on each side: where the misses start.
    assert _extent(fit)[0] == (-8, 8)
    assert fit.edges()[0] == {"lo": "closed", "hi": "closed"}


def test_each_edge_grows_on_its_own():
    fit = _fit(lambda i, j: -2 <= i <= 9 and j <= 4, start=((-3, 3), (0, 2)), must=((-1, 1), (0, 1)))
    assert _extent(fit) == ((-3, 10), (0, 5))


def test_growth_stops_at_the_limit_and_says_so():
    fit = _fit(lambda i, j: True, start=((-2, 2), (0, 2)), must=((-2, 2), (0, 2)), limit=((-5, 5), (0, 4)))
    assert _extent(fit) == ((-5, 5), (0, 4))
    assert fit.edges() == [{"lo": "limit", "hi": "limit"}, {"lo": "limit", "hi": "limit"}]


def test_growing_one_axis_fills_in_the_other():
    fit = _fit(lambda i, j: abs(i) <= 6 and j <= 6, start=((-2, 2), (0, 2)), must=((-2, 2), (0, 2)))
    (i0, i1), (j0, j1) = _extent(fit)
    kept = fit.kept()
    assert set(kept) == {(i, j) for i in range(i0, i1 + 1) for j in range(j0, j1 + 1)}  # a full rectangle


def test_the_fit_never_ends_with_green_on_an_edge_short_of_a_limit():
    fit = _fit(lambda i, j: (i - 3) ** 2 + (j - 5) ** 2 <= 30, start=((-1, 1), (0, 1)), must=((-1, 1), (0, 1)))
    assert all(state in ("closed", "limit") for axis in fit.edges() for state in axis.values())
    assert fit.edges()[1]["lo"] == "limit"  # green at j = 0, the floor of the axis


# --- trimming and the one-row red margin ---


def test_all_red_rows_are_trimmed_but_one_is_kept_past_the_boundary():
    fit = _fit(lambda i, j: abs(i) <= 2, start=((-10, 10), (0, 2)), must=((-1, 1), (0, 2)), limit=((-50, 50), (0, 2)))
    assert _extent(fit)[0] == (-3, 3)  # green to +-2, then exactly one red column
    assert len(fit.cells) == 21 * 3    # the trimmed cells were still simulated
    assert len(fit.kept()) == 7 * 3


def test_the_red_margin_is_one_row_however_far_the_grid_grew():
    for grow in (1, 2, 5):
        fit = _fit(lambda i, j: abs(i) <= 7, start=((-3, 3), (0, 1)), must=((-3, 3), (0, 1)), grow=grow)
        assert _extent(fit)[0] == (-8, 8), grow


# --- the guardrail: never shrink inside the target box ---


def test_the_grid_never_shrinks_inside_the_target_box():
    fit = _fit(lambda i, j: abs(i) <= 2, start=((-10, 10), (0, 2)), must=((-6, 6), (0, 2)))
    assert _extent(fit)[0] == (-6, 6)  # red from +-3 outward, but the box reaches +-6


def test_an_all_red_map_keeps_exactly_the_target_box():
    fit = _fit(lambda i, j: False, start=((-9, 9), (0, 5)), must=((-4, 4), (0, 3)))
    assert _extent(fit) == ((-4, 4), (0, 3))
    assert fit.edges() == [{"lo": "closed", "hi": "closed"}, {"lo": "closed", "hi": "closed"}]


def test_a_start_smaller_than_the_target_box_is_widened_to_it():
    fit = SM.GridFit(start=((-1, 1), (0, 1)), must=((-4, 4), (0, 3)), limit=WIDE)
    assert _extent(fit) == ((-4, 4), (0, 3))


def test_the_target_box_wins_over_the_growth_limit():
    fit = _fit(lambda i, j: True, start=((-2, 2), (0, 2)), must=((-8, 8), (0, 2)), limit=((-5, 5), (0, 4)))
    assert _extent(fit)[0] == (-8, 8)


def test_growth_stops_at_the_drop_budget():
    fit = _fit(lambda i, j: True, start=((-2, 2), (0, 2)), must=((-2, 2), (0, 2)), budget=60)
    assert fit.over_budget and len(fit.cells) <= 60
    assert "budget" in {s for axis in fit.edges() for s in axis.values()}
    (i0, i1), (j0, j1) = _extent(fit)
    assert all((i, j) in fit.cells for i in range(i0, i1 + 1) for j in range(j0, j1 + 1))


# --- failures inside the green zone ---

CLOSED = [{"lo": "closed", "hi": "closed"}, {"lo": "closed", "hi": "closed"}]
OPEN_J = [{"lo": "closed", "hi": "closed"}, {"lo": "limit", "hi": "limit"}]  # as an X-Z slice: Z runs on


def _grid(rows):
    """{(i, j): outcome} from strings, the first string being the top row (highest j)."""
    name = {"S": "seated", "J": "jammed", "W": "wrong_valley", "F": "fell"}
    return {(i, len(rows) - 1 - k): name[ch] for k, row in enumerate(rows) for i, ch in enumerate(row)}


def test_a_hole_in_the_zone_is_inside_and_the_failures_beyond_its_edge_are_not():
    grid = _grid(["WSSSSSW",
                  "WSSJSSW",
                  "WSSSSSW"])
    assert SM.inside_green(grid, CLOSED) == [(3, 1)]


def test_a_band_that_cuts_right_across_the_zone_is_inside_it():
    grid = _grid(["WSSJSSW",
                  "WSSJSSW",
                  "WSSJSSW"])
    assert SM.inside_green(grid, CLOSED) == [(3, 0), (3, 1), (3, 2)]  # green both sides, none in its column
    sideways = {(j, i): o for (i, j), o in grid.items()}
    assert SM.inside_green(sideways, CLOSED) == [(0, 3), (1, 3), (2, 3)]


def test_a_ragged_capture_edge_is_not_inside():
    """Green above and below, but only on one side: where the capture limit moves with the row."""
    grid = _grid(["WSSSSSW",
                  "WJSSSSW",
                  "WSSSSSW"])
    assert SM.inside_green(grid, CLOSED) == []


def test_a_band_that_runs_off_an_open_edge_is_inside():
    """A jam band that starts part way up and reaches the top of an axis the zone runs on past."""
    grid = _grid(["WSSJSSW",
                  "WSSJSSW",
                  "WSSSSSW"])
    assert SM.inside_green(grid, CLOSED) == []          # a closed top edge: the band is the zone's edge
    assert SM.inside_green(grid, OPEN_J) == [(3, 1), (3, 2)]  # the zone runs on above: it is inside


def test_without_edges_every_grid_edge_is_closed():
    grid = _grid(["WSSJSSW",
                  "WSSJSSW",
                  "WSSSSSW"])
    assert SM.inside_green(grid) == SM.inside_green(grid, CLOSED) == []


# --- from the box and the check to the grid ---


def test_the_start_covers_the_checks_drops_plus_a_margin():
    cfg, U = SM.MapConfig(), 25.0
    steps = cfg.steps(U)
    check = {"drops": [{"dx_mm": -9.3, "clear_mm": 5.0, "pitch_deg": -0.1}, {"dx_mm": 12.2, "clear_mm": 10.9, "pitch_deg": 0.15}],
             "config": {"error": {"clearance": [5.0, 30.0]}}}
    ranges = SM.check_ranges(check, BOX)
    assert ranges["x"] == (-9.3, 12.2) and ranges["z"] == pytest.approx((0.0, 5.9))
    assert SM.index_extent(ranges, steps, ("x", "z"), pad=1) == ((-11, 14), (0, 3))  # 1 mm and 5 mm cells
    assert SM.check_ranges(None, BOX) == SM.box_ranges(BOX)


def test_cells_and_limits_scale_with_the_voxel_pitch():
    cfg = SM.MapConfig()
    assert cfg.steps(25.0) == {"x": 1.0, "z": 5.0, "t": 2.0}
    assert cfg.steps(50.0)["x"] == 2.0
    assert cfg.limits(25.0) == cfg.limits(50.0) == {"x": (-50, 50), "z": (0, 5), "t": (-5, 5)}


def test_a_cell_maps_to_a_pose_about_the_aim():
    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True))
    steps, aim = SM.MapConfig().steps(25.0), SM.aim_point(BOX)
    s = SM.cell_sample(cfg, "xz", (4, 2), steps, aim)
    assert s["dx"] == cfg.error.aim_bias + 4.0 and s["clear"] == cfg.error.clearance[0] + 10.0 and s["pitch"] == 0.0
    s = SM.cell_sample(cfg, "xt", (-3, 2), steps, aim)
    assert s["dx"] == cfg.error.aim_bias - 3.0 and s["pitch"] == 4.0
    assert s["clear"] == pytest.approx(cfg.error.clearance[0] + BOX.half_widths["z_mm"])  # the box's centre height
    s = SM.cell_sample(cfg, "zt", (1, -1), steps, aim)
    assert s["dx"] == cfg.error.aim_bias and s["clear"] == cfg.error.clearance[0] + 5.0 and s["pitch"] == -2.0


@pytest.mark.parametrize("kw", [{"x_step_U": 0.0}, {"grow": 0}, {"margin": 1.5}, {"theta_limit_deg": float("nan")}])
def test_bad_grid_settings_are_rejected(kw):
    with pytest.raises(ValueError):
        SM.MapConfig(**kw)


# --- A/B V0 in the sim: one coarse map (5 mm, 12.5 mm and 5 deg cells, so it stays short) ---

COARSE = SM.MapConfig(x_step_U=0.2, z_step_U=0.5, theta_step_deg=5.0, z_limit_U=0.5, theta_limit_deg=5.0)
WHY = "coarse cells to keep the test short"


@pytest.fixture(scope="module")
def ab_map():
    zoom = SM.Zoom(slice="xz", course=1, extent={"x": (8.0, 12.0), "z": (0.0, 2.5)}, reason="look at the +X capture edge",
                   step={"x": 2.0, "z": 2.5})
    return SM.sim_map(AB, V0, BOX, config=COARSE, config_reason=WHY, zooms=[zoom], workers=1)


def test_the_map_has_every_slice_on_both_courses_with_traces(ab_map):
    m, drops = ab_map
    assert m["label"] == "simulated, uncalibrated contact"
    assert [(g["slice"], g["course"]) for g in m["grids"]] == [(s, c) for s in ("xz", "xt", "zt") for c in (0, 1)]
    assert m["drops"] == sum(g["simulated"] for g in m["grids"]) == len([d for d in drops if "zoom" not in d])
    assert all("contacts" in d and d["outcome"] in ("seated", "jammed", "wrong_valley", "fell") for d in drops)
    assert any(d["contacts"] for d in drops)
    json.dumps(m, allow_nan=False), json.dumps(drops, allow_nan=False)


def test_the_ab_map_covers_the_target_box_and_ends_on_red_or_a_limit(ab_map):
    m, _ = ab_map
    w = BOX.half_widths
    for g in m["grids"]:
        for ax, lo, hi in (("x", -w["x_mm"], w["x_mm"]), ("z", 0.0, 2 * w["z_mm"]), ("t", -w["theta_deg"], w["theta_deg"])):
            if ax in g["extent"]:
                assert g["extent"][ax][0] <= lo and hi <= g["extent"][ax][1]  # the guardrail
        assert all(state in ("closed", "limit") for e in g["edges"].values() for state in e.values())
        assert len(g["rows"]) * len(g["rows"][0]) == g["cells"]
        if "x" in g["extent_U"]:
            assert g["extent_U"]["x"] == pytest.approx([v / V0.U for v in g["extent"]["x"]])
    assert m["open_edges"] == []
    assert "zt course 0 z hi" in m["limit_edges"]  # at the aim A/B seats from every height tried
    x_edges = [e[ax][side] for g in m["grids"] if "x" in g["edges"] for ax in ("x",) for e in (g["edges"],)
               for side in ("lo", "hi")]
    assert x_edges and set(x_edges) == {"closed"}  # every X edge ends on red: the capture limit is found
    assert m["drops"] < m["fixed_grid_drops"]  # fewer than a fixed +-30 mm grid at these cell sizes


def test_a_zoom_is_logged_with_its_reason_and_leaves_the_fitted_grids_alone(ab_map):
    m, drops = ab_map
    kinds = [o["kind"] for o in m["overrides"]]
    assert kinds == ["config", "zoom"]
    assert m["overrides"][0]["reason"] == WHY and set(m["overrides"][0]["changed"]) == {
        "x_step_U", "z_step_U", "theta_step_deg", "z_limit_U", "theta_limit_deg"}
    z = m["overrides"][1]
    assert z["reason"] == "look at the +X capture edge" and z["slice"] == "xz" and z["course"] == 1
    (g,) = m["zooms"]
    assert g["extent"]["x"] == [8.0, 12.0] and g["cells"] == 3 * 2 == z["cells"]
    assert len([d for d in drops if d.get("zoom")]) == 6
    assert len(m["grids"]) == 6  # the fitted slices are all still there, at their own extent


def test_overrides_need_a_reason():
    with pytest.raises(ValueError, match="reason"):
        SM.sim_map(AB, V0, BOX, config=COARSE)
    zoom = SM.Zoom(slice="xz", course=0, extent={"x": (0.0, 1.0), "z": (0.0, 1.0)}, reason="  ")
    with pytest.raises(ValueError, match="reason"):
        SM.sim_map(AB, V0, BOX, zooms=[zoom])


def test_a_design_that_failed_the_quick_check_is_not_mapped():
    failed = {"passed": False, "seated": 2, "dropped": 11, "seed": 0, "drops": []}
    with pytest.raises(ValueError, match="quick check"):
        SM.sim_map(AB, V0, BOX, check=failed)
    with pytest.raises(ValueError, match="placement"):
        SM.sim_map(AB, V0, BX.default_boxes()["pickup"]["mean"])


def test_map_and_drops_go_to_a_new_run_directory(ab_map, tmp_path):
    m, drops = ab_map
    a, b = SM.write_map(m, drops, tmp_path), SM.write_map(m, drops, tmp_path)
    assert a != b and a.name.startswith("map-") and a.name.endswith("-ab-mean")
    saved = json.loads((a / "map.json").read_text(encoding="utf-8"))
    assert saved["label"] == "simulated, uncalibrated contact" and saved["files"] == {"drops": "drops.json.gz"}
    back = SM.read_drops(a)
    assert [(d["slice"], d["cell"], d["outcome"]) for d in back] == [(d["slice"], d["cell"], d["outcome"]) for d in drops]
    assert (a / "drops.json.gz").stat().st_size < 1_000_000  # compressed: the traces are large
    assert "course 0" in SM.summary(m) and "S" in SM.ascii_grid(m["grids"][0])
