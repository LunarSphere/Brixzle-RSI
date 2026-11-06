"""The family registry discovers brixzle.families.<pkg>.family modules that export FAMILY."""

from dataclasses import replace

import pytest

from brixzle.families import registry
from brixzle.families.interface import Family
from brixzle.families.wedge.family import FAMILY as WEDGE


def test_discovers_the_dummy_families():
    assert {"wedge", "pyramid"} <= set(registry.names())


def test_registers_the_ab_baseline():
    assert "ab" in registry.names()


def test_get_returns_the_family():
    family = registry.get("wedge")
    assert isinstance(family, Family) and family is WEDGE


def test_get_unknown_name_lists_known_names():
    with pytest.raises(KeyError, match="wedge"):
        registry.get("no-such-family")


def test_collect_rejects_duplicate_names():
    with pytest.raises(ValueError, match="duplicate family name 'wedge'"):
        registry.collect([WEDGE, replace(WEDGE, description="a copy")])


def test_collect_rejects_non_family_objects():
    with pytest.raises(TypeError):
        registry.collect([object()])
