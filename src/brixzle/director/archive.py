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


# --- the archive by error band (#206) ---

BAND_EDGES_MM = (16.0, 22.0, 28.0, 34.0)  # below 16, 16-22, 22-28, 28-34, 34 and above
GREEN_TIE_MM = 1.25                       # half a fine grid cell (#208): closer is a tie (see better)


def band_of(green_mm, edges=BAND_EDGES_MM):
    """The band's name for an error radius: 'below 16', '16-22', ..., '34+'."""
    r = float(green_mm or 0.0)
    if r < edges[0]:
        return f"below {edges[0]:g}"
    for lo, hi in zip(edges, edges[1:]):
        if r < hi:
            return f"{lo:g}-{hi:g}"
    return f"{edges[-1]:g}+"


def band_key(rec):
    """What a band ranks its designs on after the green zone: overhang, then stack, then the weakest
    placement's Z within 16 mm, then the green zone itself."""
    g = float(rec.get("green_mm") or 0.0)
    z16 = (rec.get("z_rings") or {}).get("16")
    return (float(rec.get("overhang_steps") or 0.0), float(rec.get("stack_courses") or 0.0),
            -1.0 if z16 is None else float(z16), g)


def green_vector(rec):
    """The placements' green zones, weakest first (#214). A record without them falls back to its
    headline, and its second-weakest placement if it has one (#210)."""
    by = rec.get("green_by_placement")
    if by:
        return sorted(float(v) for v in by.values())
    out = [float(rec.get("green_mm") or 0.0)]
    if rec.get("second_mm") is not None:
        out.append(float(rec["second_mm"]))
    return out


def better(a, b, tie_mm=GREEN_TIE_MM):
    """``a`` ranks above ``b`` in a band (leximin, #214): compare the placements' green zones weakest
    first, and the first that differs by at least ``tie_mm`` decides; if every one ties, a larger
    ``band_key`` (overhang, stack, Z16)."""
    for x, y in zip(green_vector(a), green_vector(b)):
        if abs(x - y) >= tie_mm:
            return x > y
    return band_key(a) > band_key(b)


class BandArchive:
    """The best design in each error band (#206), so a new concept competes in its own band and not
    against a polished design that tolerates more. A design breaking a hard constraint is not kept."""

    def __init__(self, members=None, edges=BAND_EDGES_MM):
        self.members = list(members or [])
        self.edges = edges

    def offer(self, rec):
        if rec.get("violations"):
            return {"kept": False, "reason": f"infeasible: {rec['violations']}", "replaced": [], "band": None}
        band = band_of(rec.get("green_mm"), self.edges)
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
