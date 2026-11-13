"""Every file borrowed from aerial-assembly names its source and what changed (CLAUDE.md)."""

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
BORROWED = [
    "src/brixzle/engine/sim/config.py",
    "src/brixzle/engine/sim/scene.py",
    "src/brixzle/engine/sim/trials.py",
    "src/brixzle/engine/sim/approach_error.py",
    "src/brixzle/engine/sim/drone.py",
    "src/brixzle/engine/sim/fork.py",
    "src/brixzle/families/ab/params.py",
    "src/brixzle/families/ab/profile.py",
    "src/brixzle/families/convex.py",
    "src/brixzle/families/ab/rules.py",
    "src/brixzle/families/ab/lattice.py",
    "src/brixzle/families/builders.py",
    "src/brixzle/families/rules.py",
    "tests/test_trials.py",
    "tests/test_approach_error.py",
    "src/brixzle/loop/videos.py",
]
HEADER = "# From LunarSphere/aerial-assembly "


@pytest.mark.parametrize("path", BORROWED)
def test_borrowed_file_has_provenance_header(path):
    lines = (ROOT / path).read_text(encoding="utf-8").splitlines()
    assert lines[0].startswith(HEADER) and lines[0].endswith("@ a649663"), lines[0]
    assert lines[1].startswith("# Changed:") or lines[1] == "# Unchanged.", lines[1]


def test_error_file_records_its_source():
    import json

    d = json.loads((ROOT / "src/brixzle/data/error-exp3-r4.json").read_text(encoding="utf-8"))
    assert d["provenance"] == {
        "from": "LunarSphere/aerial-assembly brixzle/configs/error-exp3-r4.json @ a649663",
        "changed": "added this provenance field; values unchanged",
    }
