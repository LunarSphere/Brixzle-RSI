"""The Pareto archive of verified, feasible designs (#164's proposal, used by #165).

A design is a record with Z on the target (``z.mean``), the run's error-tolerant overhang term,
Z at mu 0.8 (``z_mu08``) and the gate's overhang (``overhang_n``), and its
hard-constraint ``violations``. One design beats another beyond noise when it is no worse on every
term by more than that term's noise band and better on one by more than it. The band of a Z term is
one standard error of Z for its counts (zscore.z_band); the overhang terms use a band of 0. A feasible design no member beats is kept, and the members it beats are dropped.
"""
from brixzle.engine import zscore as ZS

LEGACY_TERMS = ("z_target", "tolerance_mm", "z_mu08", "overhang_n")
HELD_Z_TERMS = ("held_full_z", "held_plus_bare_z", "held_minus_bare_z", "held_first_reach_z")
HELD_TERMS = ("z_target", *HELD_Z_TERMS, "held_reliable_reach_mm", "held_mission_rate")
Z_COUNTS = {**{term: f"{term}_counts" for term in HELD_Z_TERMS}, "z_target": "z_counts", "z_mu08": "z_mu08_counts"}


def _value(rec, term):
    v = rec["z"].get("mean") if term == "z_target" else rec.get(term)
    return -1.0 if v is None else float(v)


def _band(rec, counts_key):
    c = rec.get(counts_key) if counts_key else None
    if not c or not c.get("N"):
        return 0.0
    z = (c["S"] - c["J"]) / c["N"]
    return ZS.z_band({**c, "Z": z})


def beats(a, b, terms=LEGACY_TERMS):
    """``a`` beats ``b`` beyond noise (bands are ``b``'s, the design being judged)."""
    better = False
    for term in terms:
        x, y = _value(a, term), _value(b, term)
        band = _band(b, Z_COUNTS.get(term))
        if x < y - band:
            return False
        if x > y + band:
            better = True
    return better


class Archive:
    def __init__(self, members=None, terms=LEGACY_TERMS):
        self.members = list(members or [])
        self.terms = terms

    def offer(self, rec):
        if rec.get("violations"):
            return {"kept": False, "reason": f"infeasible: {rec['violations']}", "replaced": []}
        by = [m["name"] for m in self.members if beats(m, rec, self.terms)]
        if by:
            return {"kept": False, "reason": f"beaten beyond noise by {by}", "replaced": []}
        gone = [m["name"] for m in self.members if beats(rec, m, self.terms)]
        self.members = [m for m in self.members if m["name"] not in gone] + [rec]
        return {"kept": True, "reason": "on the Pareto front", "replaced": gone}

    def to_json(self):
        return {"members": self.members}


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
