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
