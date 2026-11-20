"""The Pareto archive of verified, feasible designs (#164's proposal, used by #165).

A design is a record with Z on the target (``z.mean``), the error-tolerant overhang radius
(``tolerance_mm``), Z at mu 0.8 (``z_mu08``) and the gate's overhang (``overhang_n``), and its
hard-constraint ``violations``. One design beats another beyond noise when it is no worse on every
term by more than that term's noise band and better on one by more than it. The band of a Z term is
one standard error of Z for its counts (zscore.z_band); the radius and the overhang are counts, so
their band is 0. A feasible design no member beats is kept, and the members it beats are dropped.
"""
from brixzle.engine import zscore as ZS

TERMS = (("z_target", "z_counts"), ("tolerance_mm", None), ("z_mu08", "z_mu08_counts"), ("overhang_n", None))


def _value(rec, term):
    v = rec["z"].get("mean") if term == "z_target" else rec.get(term)
    return -1.0 if v is None else float(v)


def _band(rec, counts_key):
    c = rec.get(counts_key) if counts_key else None
    if not c or not c.get("N"):
        return 0.0
    z = (c["S"] - c["J"]) / c["N"]
    return ZS.z_band({**c, "Z": z})


def beats(a, b):
    """``a`` beats ``b`` beyond noise (bands are ``b``'s, the design being judged)."""
    better = False
    for term, ck in TERMS:
        x, y, band = _value(a, term), _value(b, term), _band(b, ck)
        if x < y - band:
            return False
        if x > y + band:
            better = True
    return better


class Archive:
    def __init__(self, members=None):
        self.members = list(members or [])

    def offer(self, rec):
        if rec.get("violations"):
            return {"kept": False, "reason": f"infeasible: {rec['violations']}", "replaced": []}
        by = [m["name"] for m in self.members if beats(m, rec)]
        if by:
            return {"kept": False, "reason": f"beaten beyond noise by {by}", "replaced": []}
        gone = [m["name"] for m in self.members if beats(rec, m)]
        self.members = [m for m in self.members if m["name"] not in gone] + [rec]
        return {"kept": True, "reason": "on the Pareto front", "replaced": gone}

    def to_json(self):
        return {"members": self.members}
