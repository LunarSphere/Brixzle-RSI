"""Design log (issue #20): one entry per design attempt, so neither agent repeats an idea.

Every attempt the orchestrator (#21) makes is logged, kept or not:

- a package that broke the family contract or a rule (``failed_contract``);
- a design the quick check turned back (``failed_quick_check``, #47), with its counts by outcome;
- a design that was mapped and scored (Z per target, #61), and then ``kept`` or ``rejected`` by
  the acceptance rule, with the reason;
- a call that failed (``error``);
- the parent family, scored as iteration 0 (``baseline``, #85).

Grid overrides made for the map (#48) are logged with their reasons (``overrides``). Each entry
names the run directories its numbers came from (``runs``).

The log is a JSON-lines file, ``design_log.jsonl``, in the loop's run directory.

**The agents never see held-out results** (#21): ``for_agents`` and ``text`` leave them out.
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path

FILE = "design_log.jsonl"
HIDDEN = ("held_out",)  # keys the agents never see
DECISIONS = ("baseline", "kept", "rejected", "failed_contract", "failed_quick_check", "failed_structure_gate",
             "failed_ring", "error")
REQUIRED = ("iteration", "attempt", "family", "decision")


def _check_finite(v, where):
    if isinstance(v, float) and not math.isfinite(v):
        raise ValueError(f"non-finite number at {where}")
    if isinstance(v, dict):
        for k, x in v.items():
            _check_finite(x, f"{where}.{k}")
    elif isinstance(v, (list, tuple)):
        for i, x in enumerate(v):
            _check_finite(x, f"{where}[{i}]")


@dataclass
class DesignLog:
    path: Path

    @classmethod
    def in_dir(cls, run_dir):
        return cls(Path(run_dir) / FILE)

    def append(self, entry):
        """Add one attempt. Needs 'iteration', 'attempt', 'family' and a 'decision' from DECISIONS."""
        missing = [k for k in REQUIRED if entry.get(k) is None]
        if missing:
            raise ValueError(f"a design log entry needs {missing}")
        if entry["decision"] not in DECISIONS:
            raise ValueError(f"decision must be one of {DECISIONS}, got {entry['decision']!r}")
        _check_finite(entry, "entry")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.path, "a", encoding="utf-8") as f:
            f.write(json.dumps(entry, allow_nan=False, sort_keys=True) + "\n")

    def entries(self):
        if not self.path.exists():
            return []
        return [json.loads(line) for line in self.path.read_text(encoding="utf-8").splitlines() if line.strip()]

    def for_agents(self):
        """Every entry without the held-out results."""
        return [{k: v for k, v in e.items() if k not in HIDDEN} for e in self.entries()]

    def text(self):
        """The log as short text for an agent's prompt (no held-out results)."""
        lines = []
        for e in self.for_agents():
            head = f"- iteration {e['iteration']}, attempt {e['attempt']}: {e['family']} -> {e['decision']}"
            if e.get("reason"):
                head += f" ({e['reason']})"
            lines.append(head)
            note = e.get("note") or {}
            for k in ("summary", "changes"):
                if note.get(k):
                    lines.append(f"  {k}: {note[k]}")
            if e.get("z"):
                lines.append("  Z: " + ", ".join(f"{s} {c['Z']:+.3f} (S {c['S']}, J {c['J']}, N {c['N']})"
                                                 for s, c in e["z"].items()))
            qc = e.get("quick_check")
            if qc and e["decision"] == "failed_quick_check":
                outcomes = ", ".join(f"{k} {v}" for k, v in sorted(qc.get("outcomes", {}).items()) if k != "seated")
                lines.append(f"  quick check failed: {qc['seated']} of {qc['dropped']} seated ({outcomes})")
            if e.get("violations"):
                lines.append("  contract: " + "; ".join(e["violations"][:5]))
            if e.get("brief_followed"):
                lines.append(f"  followed the brief: {e['brief_followed']}")
            for o in e.get("overrides") or []:
                lines.append(f"  map override ({o.get('kind')}): {o.get('reason')}")
        return "\n".join(lines)
