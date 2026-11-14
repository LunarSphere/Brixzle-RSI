"""The loop's settings (issue #21). The runtime model is set here and nowhere else."""

from dataclasses import asdict, dataclass, field
import math

from brixzle.engine import boxes as BX

MODEL = "claude-sonnet-5-5"  # the design agent and the critic (maintainer, 2026-11-12)


@dataclass(frozen=True)
class LoopConfig:
    model: str = MODEL
    target: str = "mean"             # the box the run starts on: the first rung of the ladder (#84)
    ladder: bool = True              # climb mean -> p95 -> max as Z reaches 1 (#84); False: stay on target
    y_error: bool = True             # score the X-Y error disc (#86); False: X offsets only, as before
    lessons: bool = True             # read and write the lessons file (#88); False for repeatable benchmarks
    lessons_file: str = "lessons/lessons.jsonl"  # tracked in git, at the repo root (#88)
    parent: str | None = "ab"        # a registry family scored as iteration 0, the baseline (#85); None: from scratch
    baseline_only: bool = False      # score the parent (iteration 0) and stop: no Claude call (#85)
    max_iterations: int = 6
    patience: int = 2                # stop after this many iterations in a row with no kept design
    budget_usd: float = 5.0          # cap on real API spend, over the ledger
    ledger: str | None = None        # spend ledger file (default: <run>/api-spend.jsonl)
    max_minutes: float = 30.0        # wall-clock cap for the whole run (CLAUDE.md, Speed)
    design_attempts: int = 3         # design-agent tries per iteration (contract or quick-check failures)
    design_budget_usd: float = 1.5   # per design-agent call
    critic_budget_usd: float = 0.4   # per critic call
    design_max_turns: int = 40
    probe_calls: int = 5             # probe_drops calls per design call (#107); 0: no probe tool
    probe_drops: int = 30            # poses per probe, the aim included (#107)
    seed: int = 0                    # quick-check seed; held-out Z uses seed + 1
    workers: int | None = None       # sim drop processes
    held_out: bool = True
    tune: bool = True                # the end tune on the final design (#14)
    videos: int = 5                  # drop videos per iteration, picked at random from the map (#77; 0: none)
    tune_seconds: float = 420.0
    runs: str = "runs"
    results: str = "RSI_RESULTS"

    def validate(self):
        if self.target not in BX.STATISTICS:
            raise ValueError(f"target must be one of {BX.STATISTICS}, got {self.target!r}")
        for name in ("max_iterations", "patience", "design_attempts", "design_max_turns"):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, int) or v < 1:
                raise ValueError(f"{name} must be an integer >= 1, got {v!r}")
        for name in ("budget_usd", "max_minutes", "design_budget_usd", "critic_budget_usd", "tune_seconds"):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
                raise ValueError(f"{name} must be a finite number > 0, got {v!r}")
        for name, lo in (("probe_calls", 0), ("probe_drops", 2)):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, int) or v < lo:
                raise ValueError(f"{name} must be an integer >= {lo}, got {v!r}")
        if isinstance(self.videos, bool) or not isinstance(self.videos, int) or self.videos < 0:
            raise ValueError(f"videos must be an integer >= 0, got {self.videos!r}")
        if self.parent is not None:
            from brixzle.families import registry

            if self.parent not in registry.names():
                raise ValueError(f"parent must be a registry family or None, got {self.parent!r}; "
                                 f"known: {registry.names()}")
        if self.baseline_only and self.parent is None:
            raise ValueError("baseline_only needs a parent family to score")
        if not self.model.strip():
            raise ValueError("model must be named")
        return self

    def to_dict(self):
        return asdict(self)
