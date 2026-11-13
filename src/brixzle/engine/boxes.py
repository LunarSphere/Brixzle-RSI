"""Target error boxes (R11, R12): the drone error a design must survive, sized to mean, p95 and worst.

A box is a half-width per axis around the aim point:

- placement (R11): X, Z and theta, the brick's profile plane (theta is pitch, rotation about Y), and
  Y across it. X and Y form a disc (issue #86): the measured error is a radius in a random X-Y
  direction, so a pose is inside when x^2 + y^2 <= r^2, with r the X half-width;
- pickup (R12): X across the tines, Z, and psi (yaw).

X is measured: the full Exp 3 positioning radius (exp3-r4, ``error-exp3-r4.json``), so the whole
X-Y error may land along X. Z, theta and psi are not measured yet (aerial-assembly#27); they are
estimates from the exp3-r2 hover logs (``hover-exp3-r2.json``, made by
scripts/make_hover_estimates.py). Each box says which sides are measured and which estimated.
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path

from brixzle.engine.sim import approach_error as AE

DATA_DIR = Path(__file__).resolve().parents[1] / "data"
ERROR_PATH = DATA_DIR / "error-exp3-r4.json"
HOVER_PATH = DATA_DIR / "hover-exp3-r2.json"
STATISTICS = ("mean", "p95", "max")
TASKS = ("placement", "pickup")
_HOVER_KEYS = ("z_mm", "pitch_deg", "yaw_deg")


@dataclass(frozen=True)
class TargetBox:
    task: str          # 'placement' or 'pickup'
    statistic: str     # 'mean', 'p95' or 'max' (worst)
    half_widths: dict  # axis -> half-width; axes x_mm, z_mm and theta_deg (placement) or psi_deg (pickup)
    sources: dict      # axis -> 'measured: ...' or 'estimated: ...'
    xy_shape: str = "box"  # 'disc': x^2 + y^2 <= x_mm^2 (placement, #86); 'box': each axis on its own

    def to_dict(self):
        if not all(math.isfinite(v) and v >= 0 for v in self.half_widths.values()):
            raise ValueError("box half-widths must be finite and >= 0")
        return {"task": self.task, "statistic": self.statistic, "half_widths": dict(self.half_widths),
                "sources": dict(self.sources), "xy_shape": self.xy_shape}


def load_hover(path):
    """A validated hover-deviation file (see scripts/make_hover_estimates.py)."""
    d = json.loads(Path(path).read_text(encoding="utf-8"))
    if d.get("kind") != "hover-deviation":
        raise ValueError("expected a hover-deviation file (kind 'hover-deviation')")
    if not isinstance(d.get("n"), int) or d["n"] < 1 or d["n"] != len(d.get("approaches", [])):
        raise ValueError("n must be >= 1 and match the number of approaches")
    for stat in STATISTICS:
        for key in _HOVER_KEYS:
            name = f"{stat}_{key}"
            if name not in d:
                raise ValueError(f"missing {name}")
            v = d[name]
            if not (isinstance(v, (int, float)) and math.isfinite(v) and v >= 0):
                raise ValueError(f"{name} must be finite and >= 0, got {v!r}")
    return d


def target_boxes(error_summary, hover):
    """{task: {statistic: TargetBox}} from an approach-error summary (X) and hover estimates."""
    session = error_summary.get("source", {}).get("run", "") or "approach-error file"
    hover_session = hover["source"].get("session", "hover file")
    x_src = f"measured: {session} positioning radius (X-Y, applied along X)"
    y_src = "measured: runs/exp3-r4 positioning radius (X-Y disc)"
    est = f"estimated: {hover_session} hover, RMS deviation from the command"
    out = {}
    for task in TASKS:
        out[task] = {}
        for stat in STATISTICS:
            angle = ("theta_deg", hover[f"{stat}_pitch_deg"], f"{est} (pitch)") if task == "placement" else \
                    ("psi_deg", hover[f"{stat}_yaw_deg"], f"{est} (yaw)")
            r = AE.required_tolerance(error_summary, stat)
            half = {"x_mm": r, "z_mm": hover[f"{stat}_z_mm"], angle[0]: angle[1]}
            src = {"x_mm": x_src, "z_mm": f"{est} (Lighthouse height; likely an underestimate)", angle[0]: angle[2]}
            if task == "placement":  # the X-Y disc (issue #86); pickup keeps its own axes
                half["y_mm"], src["y_mm"] = r, y_src
            out[task][stat] = TargetBox(task=task, statistic=stat, half_widths=half, sources=src,
                                        xy_shape="disc" if task == "placement" else "box")
    return out


def default_boxes():
    """Boxes from the committed exp3-r4 error file and exp3-r2 hover estimates."""
    return target_boxes(AE.load(ERROR_PATH), load_hover(HOVER_PATH))
