# From LunarSphere/aerial-assembly brixzle/src/brixzle/config.py @ a649663
# Changed: write_json dropped (unused here).
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path


def read_json(path):
    return json.loads(Path(path).read_text())


@dataclass(frozen=True)
class Physics:
    timestep: float = 0.1
    friction: float = 0.3
    contact_timeconst: float = 0.2
    contact_dampratio: float = 1.0
    iterations: int = 100

    def __post_init__(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError("Physics settings must be finite")
        if self.timestep <= 0 or self.friction < 0 or self.contact_timeconst < 2*self.timestep:
            raise ValueError("Require dt > 0, friction >= 0, contact_timeconst >= 2*dt")
        if self.contact_dampratio <= 0 or self.iterations < 1:
            raise ValueError("Invalid damping or solver iteration count")
