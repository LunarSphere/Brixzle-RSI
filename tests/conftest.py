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
