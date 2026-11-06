"""Record aerial-assembly's V0 A/B rule margins and CAD mass, reproduced by tests/test_ab_family.py.

Not part of the brixzle package and not run by pytest. It runs under aerial-assembly's own
environment, so the reference comes from the original code:

    ~/Repos/aerial-assembly/brixzle/.venv/bin/python -I \
        tests/fixtures/make_ab_rules_reference.py ~/Repos/aerial-assembly \
        tests/fixtures/ab_rules_reference.json

It refuses to run unless aerial-assembly's brixzle/src matches SOURCE_COMMIT, the commit the A/B
family was ported from. Margins are analytic (rules.py); the mass is from the CadQuery solid.
"""

import json
import platform
import subprocess
import sys
from datetime import date
from pathlib import Path

SOURCE_COMMIT = "a649663"
PATHS = ("brixzle/src",)


def _git(repo, *args):
    return subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True,
                          text=True).stdout.strip()


def main(repo, out):
    repo = Path(repo).resolve()
    out = Path(out)
    if out.exists():
        raise SystemExit(f"{out} exists; remove it first if you mean to regenerate it")
    if _git(repo, "status", "--porcelain", "--", *PATHS):
        raise SystemExit("aerial-assembly has uncommitted changes under brixzle/src")
    if subprocess.run(["git", "-C", str(repo), "diff", "--quiet", SOURCE_COMMIT, "HEAD", "--", *PATHS]).returncode:
        raise SystemExit(f"aerial-assembly brixzle/src differs from {SOURCE_COMMIT}")

    from brixzle import cad, rules as R
    from brixzle.params import V0

    parts = {}
    for name in ("A", "B"):
        bundle = cad.build_brick(V0, part=name)
        parts[name] = {
            "margins": R.rules(V0, bundle, R.RuleSettings()),
            "mass_g": bundle["mass_g"],
            "fragments": bundle["fragments"],
            "channels": [list(map(float, c)) for c in bundle["channels"]],
        }

    reference = {
        "label": "analytic rule margins (aerial-assembly rules.py) and CadQuery mass",
        "source": {
            "repo": "LunarSphere/aerial-assembly",
            "commit": _git(repo, "rev-parse", SOURCE_COMMIT),
            "checked_out_head": _git(repo, "rev-parse", "HEAD"),
            "paths_identical_to_commit": list(PATHS),
            "generator": "tests/fixtures/make_ab_rules_reference.py",
            "date": date.today().isoformat(),
            "platform": platform.platform(),
            "python": platform.python_version(),
        },
        "params": V0.to_dict(),
        "rule_settings": "RuleSettings() defaults",
        "parts": parts,
    }
    out.write_text(json.dumps(reference, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"wrote {out}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    main(sys.argv[1], sys.argv[2])
