"""Smoke test: the package imports and its planned layout exists."""

from pathlib import Path

import pytest

SUBPACKAGES = ("intake", "agents", "engine", "families", "rules", "data")


def test_package_imports():
    import brixzle

    assert brixzle.__version__ == "0.1.0"


@pytest.mark.parametrize("name", SUBPACKAGES)
def test_subpackage_imports(name):
    import importlib

    module = importlib.import_module(f"brixzle.{name}")
    assert Path(module.__file__).parent.name == name


def test_repo_map_lists_every_subpackage():
    repo_map = Path(__file__).resolve().parents[1] / "REPO_MAP.md"
    text = repo_map.read_text(encoding="utf-8")
    for name in SUBPACKAGES:
        assert f"src/brixzle/{name}/" in text
