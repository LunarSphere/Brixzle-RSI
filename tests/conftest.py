"""Shared fixtures. Building the A/B CadQuery solids takes ~10 s, so it happens once per session."""

import pytest


@pytest.fixture(scope="session")
def ab_bricks():
    from brixzle.families.ab import cad
    from brixzle.families.ab.params import V0

    return cad.build_bricks(V0)
