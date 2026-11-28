"""allround and twinhook4 are registered families (#160), identical to the modules they came from."""
from pathlib import Path

import numpy as np
import pytest

from brixzle.families import registry

FABLE = Path(__file__).resolve().parents[1] / "runs" / "fable"
NAMES = ("fable_allround", "fable_twinhook4")


@pytest.mark.parametrize("name", NAMES)
def test_registered(name):
    fam = registry.get(name)
    assert fam.name == name
    assert fam.parts == ("A",)


@pytest.mark.parametrize("name", NAMES)
def test_matches_the_runs_module_it_was_copied_from(name):
    if not (FABLE / name / "family.py").exists():
        pytest.skip("the source module is only available locally under runs/fable")
    from brixzle.tools.check_family import load_family
    src, reg = load_family(FABLE, name), registry.get(name)
    from dataclasses import asdict
    assert src.version == reg.version and asdict(src.defaults) == asdict(reg.defaults)
    a, b = src.geometry.collision(src.defaults, "A"), reg.geometry.collision(reg.defaults, "A")
    assert len(a) == len(b) and all(np.array_equal(x, y) for x, y in zip(a, b))
