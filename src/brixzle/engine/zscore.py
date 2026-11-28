"""Z = (S - J) / N and the pose helpers the grid score and the tools share (#61, #86).

S seated (+1), J jammed (-1), a fall, a wrong valley or any other outcome 0, so Z runs from -1 to 1.
The grid score (gridscore.py) counts it per ring by cell area (#208) and averages it over the stack
and overhangs 1-3 (#222). The fitted sim map, Z over its target boxes, the end tune's ZScore and the
held-out Z of the #164 goal were removed in #227 (tag ``pre-debloat-2026-11-28``).

Results are simulated, uncalibrated contact.
"""
import math

LABEL = "simulated, uncalibrated contact"
SEATED, JAMMED = "seated", "jammed"
NAMED = ("fell", "wrong_valley")  # count 0; anything else (for example 'unstable') is 'other', also 0
EPS = 1e-9


def box_ranges(box):
    """The target box as (lo, hi) per axis, in mm and degrees about the aim."""
    w = box.half_widths
    y = w.get("y_mm", 0.0)
    return {"x": (-w["x_mm"], w["x_mm"]), "y": (-y, y), "z": (0.0, 2 * w["z_mm"]),
            "t": (-w["theta_deg"], w["theta_deg"])}


def counts(outcomes):
    """Z and the counts behind it, from the outcomes of the drops inside one box."""
    outcomes = list(outcomes)
    if not outcomes:
        raise ValueError("no drops inside the box: Z is undefined (the map must cover the target box)")
    n = len(outcomes)
    s, j = outcomes.count(SEATED), outcomes.count(JAMMED)
    named = {k: outcomes.count(k) for k in NAMED}
    return {"Z": (s - j) / n, "S": s, "J": j, "N": n, **named, "other": n - s - j - sum(named.values()),
            "passed": s == n}


def z_band(c):
    """The noise band of Z (#110): one standard error of Z for the counts ``c`` ({'Z', 'S', 'J', 'N'}).
    Each drop scores +1, 0 or -1, so its variance is (S + J) / N - Z^2, and the band is
    sqrt(variance / N). A change in Z smaller than the band is not a change."""
    n = c.get("N") or 0
    if n <= 0:
        raise ValueError("no drops: the band of Z is undefined")
    var = (c["S"] + c["J"]) / n - c["Z"] ** 2
    return math.sqrt(max(var, 0.0) / n)


def within_band(z_new, z_ref, band):
    """True if ``z_new`` differs from ``z_ref`` by no more than ``band`` (#110)."""
    return abs(z_new - z_ref) <= band + EPS


def inside(pose, box):
    """Whether a pose {x, y, z, t} (mm, mm, mm above the minimum clearance, deg; y defaults to 0)
    lies inside a placement box: the X-Y disc of a disc box (#86), and the box's height and pitch."""
    r = box_ranges(box)
    x, y = pose["x"], pose.get("y", 0.0)
    if getattr(box, "xy_shape", "box") == "disc":
        rad = box.half_widths["x_mm"]
        xy_in = x * x + y * y <= rad * rad + EPS
    else:
        xy_in = r["x"][0] - EPS <= x <= r["x"][1] + EPS and r["y"][0] - EPS <= y <= r["y"][1] + EPS
    return xy_in and all(r[a][0] - EPS <= pose[a] <= r[a][1] + EPS for a in ("z", "t"))


def drop_pose(d, clearance0):
    """The pose of a map drop, about the aim: X and Y offset, release height above the minimum, pitch."""
    return {"x": float(d["dx_mm"]), "y": float(d.get("dy_mm", 0.0)), "z": float(d["clear_mm"]) - clearance0,
            "t": float(d["pitch_deg"])}


def _sample(cfg, pose):
    bias = cfg.error.aim_bias
    return {"dx": bias + pose["x"], "dy": pose.get("y", 0.0) + 0.0, "aim": bias, "yaw": 0.0, "roll": 0.0,
            "pitch": pose["t"] + 0.0, "clear": cfg.error.clearance[0] + pose["z"], "vel": [0.0, 0.0, 0.0]}
