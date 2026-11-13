"""The loop's settings (issue #21). The runtime model is set here and nowhere else."""

from dataclasses import asdict, dataclass, field
import math

from brixzle.engine import boxes as BX

MODEL = "claude-sonnet-5-5"  # the design agent and the critic (maintainer, 2026-11-12)


@dataclass(frozen=True)
class LoopConfig:
    model: str = MODEL
    target: str = "mean"             # the run's target box; Z on it decides acceptance and the stop
    max_iterations: int = 6
    patience: int = 2                # stop after this many iterations in a row with no kept design
    budget_usd: float = 5.0          # cap on real API spend, over the ledger
    ledger: str | None = None        # spend ledger file (default: <run>/api-spend.jsonl)
    max_minutes: float = 30.0        # wall-clock cap for the whole run (CLAUDE.md, Speed)
    design_attempts: int = 3         # design-agent tries per iteration (contract or quick-check failures)
    design_budget_usd: float = 1.5   # per design-agent call
    critic_budget_usd: float = 0.4   # per critic call
    design_max_turns: int = 40
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
        if isinstance(self.videos, bool) or not isinstance(self.videos, int) or self.videos < 0:
            raise ValueError(f"videos must be an integer >= 0, got {self.videos!r}")
        if not self.model.strip():
            raise ValueError("model must be named")
        return self

    def to_dict(self):
        return asdict(self)
