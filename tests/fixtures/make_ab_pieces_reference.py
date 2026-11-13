"""Write ab_pieces_reference.json: A/B V0's collision pieces from the canonical cut (issue #98).

Run on macOS arm64: `uv run python tests/fixtures/make_ab_pieces_reference.py`. CI checks the same
pieces on Linux x86_64 (tests/test_family_builders.py), so the cut is the same on both platforms.
"""

import json
import platform
from datetime import date
from pathlib import Path

from brixzle.families import builders as B
from brixzle.families.ab.family import FAMILY as AB


def main():
    p = AB.defaults
    ext = AB.geometry.extrusion(p)
    parts = {part: [[[round(float(v), 9) for v in row] for row in piece]
                    for piece in B._pieces(AB.geometry.section(p, part).outer, ext)] for part in AB.parts}
    out = {"source": {"platform": f"{platform.system()} {platform.machine()}".replace("Darwin", "macOS"),
                      "date": date.today().isoformat(), "params": p.to_dict(),
                      "note": "builders._pieces after canonical_ring (issue #98); mm, brick frame"},
           "parts": parts}
    path = Path(__file__).parent / "ab_pieces_reference.json"
    path.write_text(json.dumps(out, separators=(",", ":")) + "\n", encoding="utf-8")
    print(path, {k: len(v) for k, v in parts.items()})


if __name__ == "__main__":
    main()
