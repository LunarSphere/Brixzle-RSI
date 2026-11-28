"""The director's archive (#206, ranked on the four-quarter Z since #222): the best verified, feasible
design in each Z band. The older Pareto archive of the #164 goal was removed with that goal (#204).
"""

# --- the archive by Z band (#206; ranked on the four-quarter Z since #222) ---

BAND_EDGES = (0.0, 0.5, 0.8, 0.95)  # Z16: below 0, 0-0.5, 0.5-0.8, 0.8-0.95, 0.95 and above
Z_TIE = 0.02                        # closer Z is a tie (about one fine cell's share of the 16 mm disc)
GREEN_TIE_MM = 1.25                 # half a fine grid cell (#208): closer green zones tie (see better)
RING_ORDER = ("16", "28", "40")     # Z16 first, then the wider discs break ties


def band_of(z, edges=BAND_EDGES):
    """The band's name for a design's Z (#222): 'below 0', '0-0.5', ..., '0.95+'."""
    v = float(z if z is not None else -1.0)
    if v < edges[0]:
        return f"below {edges[0]:g}"
    for lo, hi in zip(edges, edges[1:]):
        if v < hi:
            return f"{lo:g}-{hi:g}"
    return f"{edges[-1]:g}+"


def z_vector(rec):
    """Z over the 16, 28 and 40 mm discs, the mean over the quarters (#222). Z16 is ``rec['z']``."""
    rings = rec.get("z_rings") or {}
    out = [float(rec["z"])]
    out += [float(rings[k]) for k in RING_ORDER[1:] if rings.get(k) is not None]
    return out


def green_vector(rec):
    """The placements' green zones, weakest first (#214): the last tie-break since #222. A record
    without them falls back to its headline, and its second-weakest placement if it has one (#210)."""
    by = rec.get("green_by_placement")
    if by:
        return sorted(float(v) for v in by.values())
    out = [float(rec.get("green_mm") or 0.0)]
    if rec.get("second_mm") is not None:
        out.append(float(rec["second_mm"]))
    return out


def better(a, b, z_tie=Z_TIE, tie_mm=GREEN_TIE_MM):
    """``a`` ranks above ``b`` (#222): Z16, then Z28, then Z40, the first that differs by at least
    ``z_tie`` decides; if all tie, the placements' green zones weakest first (leximin, #214)."""
    for x, y in zip(z_vector(a), z_vector(b)):
        if abs(x - y) >= z_tie:
            return x > y
    for x, y in zip(green_vector(a), green_vector(b)):
        if abs(x - y) >= tie_mm:
            return x > y
    return False


class BandArchive:
    """The best design in each Z band (#206, #222), so a new concept competes in its own band and not
    against a polished design that scores more. A design breaking a hard constraint is not kept, and
    nor is one without a Z (scored before #222): rescore it, never guess its Z."""

    def __init__(self, members=None, edges=BAND_EDGES):
        self.members = list(members or [])
        self.edges = edges

    def offer(self, rec):
        if rec.get("violations"):
            return {"kept": False, "reason": f"infeasible: {rec['violations']}", "replaced": [], "band": None}
        if rec.get("z") is None:
            return {"kept": False, "reason": "no Z (scored before #222): rescore it", "replaced": [], "band": None}
        band = band_of(rec["z"], self.edges)
        rec = {**rec, "band": band}
        same = [m for m in self.members if m.get("band") == band]
        if same and not better(rec, same[0]):
            return {"kept": False, "reason": f"not better than {same[0]['name']} in band {band}", "replaced": [],
                    "band": band}
        gone = [m["name"] for m in same]
        self.members = [m for m in self.members if m.get("band") != band] + [rec]
        return {"kept": True, "reason": f"best in band {band}", "replaced": gone, "band": band}

    def to_json(self):
        return {"members": self.members}
