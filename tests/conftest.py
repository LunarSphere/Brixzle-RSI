"""Shared fixtures. Building the A/B CadQuery solids takes a few seconds; the builders cache them."""

import pytest


@pytest.fixture(scope="session")
def ab_bodies():
    """{part: SimBodies} for the V0 A/B family."""
    from brixzle.families import builders as B
    from brixzle.families.ab.family import FAMILY
    from brixzle.families.ab.params import V0

    return {part: B.sim_bodies(FAMILY, V0, part) for part in FAMILY.parts}


@pytest.fixture(scope="session")
def stack_v():
    """A stack-only V family (tests/fixtures/stackv, the dummy wedge before #82): it nests straight
    on top of one brick but cannot lay a running bond."""
    from pathlib import Path

    from brixzle.agents import check_family as CF

    return CF.load_family(Path(__file__).parent / "fixtures", "stackv")


@pytest.fixture(scope="session")
def ab_gate(tmp_path_factory):
    """A/B's structure gate (issue #83) with 3 repeats per structure, run once per session; its
    'repeats' hold every build, which tests/test_structures.py reuses. Simulated, uncalibrated contact."""
    from brixzle.engine import structure_gate as G
    from brixzle.families.ab.family import FAMILY

    return G.run_gate(FAMILY, FAMILY.defaults, G.GateSettings(repeats=3), runs=tmp_path_factory.mktemp("gate"))


_CHECK_CACHE = {}
_REAL_CHECK = []  # the real check_in_subprocess, kept before any test patches it


def cached_check(root, module, **kw):
    """check_family.check_in_subprocess, memoised for the session on the package's source and the
    keyword arguments: the A/B-copy check (#106) takes about 12 s and two test modules use it."""
    from pathlib import Path

    from brixzle.agents import check_family as CF

    if not _REAL_CHECK:
        _REAL_CHECK.append(CF.check_in_subprocess)
    real = _REAL_CHECK[0]
    path = Path(root) / module / "family.py"
    if not path.exists():
        return real(root, module, **kw)
    key = (path.read_text(encoding="utf-8"), tuple(sorted(kw.items())))
    if key not in _CHECK_CACHE:
        _CHECK_CACHE[key] = real(root, module, **kw)
    return {**_CHECK_CACHE[key], "module": module}


def pytest_configure(config):
    from brixzle.agents import check_family as CF

    _REAL_CHECK[:] = [CF.check_in_subprocess]


# A 9-line subclass of A/B's parameters, as the last run's designs were (#106): a near copy.
AB_COPY_SRC = ("from dataclasses import dataclass, replace\n"
               "from brixzle.families.ab.family import FAMILY as AB\n"
               "from brixzle.families.ab.params import BrickParams\n\n"
               "@dataclass(frozen=True)\nclass P(BrickParams):\n    tooth_L: float = 7.0\n\n"
               "FAMILY = replace(AB, name='abcopy', params_type=P)\n")
