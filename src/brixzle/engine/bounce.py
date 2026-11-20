"""Bounce-out on impact (#167): does a brick released from higher than usual stay in its seat?

A brick is released straight above its seat, with no X error, from a set of heights above the seat,
on two seats:

- **half**: the reaching step of an overhang (``structures.overhang(2)``'s second brick, half over
  its supporter's edge), the seat that holds by its hook alone;
- **full**: course 1 across two seated bricks (running bond), the fully supported control.

Each case is built ``repeats`` times with a 0.05 mm seeded release jitter (as the gate does). An
outcome is ``seated``, ``jammed`` or ``bounced_out`` (it fell or came to rest in another valley).
Impact energy is m g h, with h the drop from release to the seat.

**The sim's contact barely bounces.** MuJoCo's contact is a spring-damper set by ``solref`` (time
constant, damping ratio); the repo's default damping ratio is 1.0, critical, so an impact returns
almost no energy. Real PLA on PLA does bounce. So the trial also runs at lower damping ratios
(``dampratios``), as a stand-in for a bouncier contact, until a restitution test calibrates it. Every
result is simulated, uncalibrated contact, and the damping ratio is reported with it.
"""
from dataclasses import replace
import time

LABEL = "simulated, uncalibrated contact"
HEIGHTS_MM = (0.0, 25.0, 50.0, 75.0, 100.0, 150.0)   # drop height above the seat; 0: the family's own lift
SEATS = ("half", "full")
G = 9.81


def energy_mJ(mass_g, height_mm):
    return mass_g * 1e-3 * G * height_mm * 1e-3 * 1e3


def heights(family, p, wanted=HEIGHTS_MM):
    """The drop heights to use: each wanted height, but never below the family's own release lift."""
    lift = float(family.release_lift_mm(p))
    return sorted({max(lift, float(h)) for h in wanted})


def kind(outcome):
    if outcome in ("seated", "jammed"):
        return outcome
    return "bounced_out"


def config(dampratio=None, clear_mm=5.0):
    """An ideal-placement trial config whose release is ``clear_mm`` above the family's lift."""
    from brixzle.engine.sim import trials as T

    cfg = T.TrialConfig(error=T.ErrorModel(ideal=True, clearance=(clear_mm, clear_mm)))
    if dampratio is not None:
        if not dampratio > 0:
            raise ValueError("dampratio must be > 0")
        cfg = replace(cfg, physics=replace(cfg.physics, contact_dampratio=float(dampratio)))
    return cfg


def _half(family, p, cfg, repeats, seed):
    from brixzle.engine import structures as S
    from brixzle.engine.sim import assemble as A

    st = S.overhang(2, +1, family.lattice(p).span_voxels)
    res = A.repeat_builds(family, p, st, cfg, n=repeats, seed=seed, jitter_mm=0.05, workers=min(repeats, 8))
    out = []
    for b in res["builds"]:
        stages = b["stages"]
        last = stages[-1] if stages else {}
        reach = [s_ for s_ in stages if s_["brick"] == "b1_1"]
        out.append(reach[-1]["outcome"] if reach else f"before_reach:{last.get('outcome')}")
    return out


def _full(family, p, cfg, repeats, seed):
    import numpy as np

    from brixzle.engine import box_search as BS

    rng = np.random.default_rng(seed)
    base = cfg.error.sample(None)
    jobs = [(1, dict(base, dx=base["dx"] + float(rng.uniform(-0.05, 0.05))), {"trace": False}) for _ in range(repeats)]
    with BS.DropPool(family, p, cfg, workers=1) as pool:
        return [r["outcome"] for r in pool.imap(jobs)]


def bounce(family, p=None, heights_mm=HEIGHTS_MM, seats=SEATS, dampratios=(None,), repeats=5, seed=0):
    """Rows of (seat, damping ratio, drop height, energy, outcomes). Simulated, uncalibrated contact."""
    from brixzle.families import builders as B

    p = p or family.defaults
    t0 = time.time()
    lift = float(family.release_lift_mm(p))
    mass = float(B.sim_bodies(family, p, family.part_for(1, 1)).mass_g)
    rows = []
    for dr in dampratios:
        for h in heights(family, p, heights_mm):
            cfg = config(dr, clear_mm=h - lift)  # released h above the seat (the lift plus the clearance)
            for seat in seats:
                t = time.time()
                outs = (_half if seat == "half" else _full)(family, p, cfg, repeats, seed)
                kinds = {}
                for o in outs:
                    k = kind(o)
                    kinds[k] = kinds.get(k, 0) + 1
                rows.append({"seat": seat, "dampratio": dr if dr is not None else cfg.physics.contact_dampratio,
                             "height_mm": h, "energy_mJ": round(energy_mJ(mass, h), 3), "outcomes": outs, "kinds": kinds,
                             "bounced_out": kinds.get("bounced_out", 0), "n": len(outs), "seconds": round(time.time() - t, 1)})
    return {"label": LABEL, "family": family.name, "mass_g": mass, "release_lift_mm": lift, "rows": rows,
            "seconds": round(time.time() - t0, 1)}
