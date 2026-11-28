"""The family registry discovers brixzle.families.<pkg>.family modules that export FAMILY."""

from dataclasses import replace

import pytest

from brixzle.families import registry
from brixzle.families.interface import Family
from wedge.family import FAMILY as WEDGE


def test_registers_only_the_reference_designs():
    # #227: the dummy families moved to tests/fixtures; the registry keeps the reference designs
    assert set(registry.names()) == {"ab", "fable_allround", "fable_twinhook4", "fable_gentle"}


def test_registers_the_ab_baseline():
    assert "ab" in registry.names()


def test_get_returns_the_family():
    from brixzle.families.ab.family import FAMILY as AB
    family = registry.get("ab")
    assert isinstance(family, Family) and family is AB


def test_get_unknown_name_lists_known_names():
    with pytest.raises(KeyError, match="ab"):
        registry.get("no-such-family")


def test_collect_rejects_duplicate_names():
    with pytest.raises(ValueError, match="duplicate family name 'wedge'"):
        registry.collect([WEDGE, replace(WEDGE, description="a copy")])


def test_collect_rejects_non_family_objects():
    with pytest.raises(TypeError):
        registry.collect([object()])
