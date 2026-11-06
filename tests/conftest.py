"""Shared fixtures. Building the A/B CadQuery solids takes a few seconds; the builders cache them."""

import pytest


@pytest.fixture(scope="session")
def ab_bodies():
    """{part: SimBodies} for the V0 A/B family."""
    from brixzle.families import builders as B
    from brixzle.families.ab.family import FAMILY
    from brixzle.families.ab.params import V0

    return {part: B.sim_bodies(FAMILY, V0, part) for part in FAMILY.parts}
