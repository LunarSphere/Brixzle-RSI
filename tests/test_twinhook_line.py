"""The twinhook line as tracked families (#162): registered, sourced, and identical to the runs/fable
modules they were verified as."""
from pathlib import Path

import pytest

from brixzle.families import fingerprint as FP
from brixzle.families import registry

ROOT = Path(__file__).resolve().parents[1]
FABLE = ROOT / "runs" / "fable"
FAMILIES = ROOT / "src" / "brixzle" / "families"
LINE = ("fable_twinhook4", "fable_gentle", "fable_feather", "fable_reach", "fable_wide", "fable_allround")


@pytest.mark.parametrize("name", LINE)
def test_registered(name):
    fam = registry.get(name)
    assert fam.name == name and fam.parts == ("A",) and fam.ring is not None


@pytest.mark.parametrize("name", LINE)
def test_every_file_names_its_source(name):
    _, path = registry.source(name)
    head = path.read_text().splitlines()[:3]
    assert head[0].startswith(f"# From runs/fable/{name}/family.py"), head[0]
    assert any(line.startswith("# Changed:") for line in head), head


_FP = {}


def _fingerprint(where, name):
    if (where, name) not in _FP:
        if where == "runs":
            from brixzle.agents.check_family import load_family
            fam = load_family(FABLE, name)
        else:
            fam = registry.get(name)
        _FP[where, name] = FP.fingerprint(fam)
    return _FP[where, name]


@pytest.mark.parametrize("name", LINE)
def test_identical_to_the_verified_module(name):
    if not (FABLE / name / "family.py").exists():
        pytest.skip("the source module is only available locally under runs/fable")
    a, b = _fingerprint("runs", name), _fingerprint("repo", name)
    assert FP.differences(a, b) == []


def test_fingerprint_sees_a_one_parameter_change():
    from dataclasses import replace
    fam = registry.get("ab")
    p = fam.defaults
    a = FP.fingerprint(fam, p, solid=False)
    b = FP.fingerprint(fam, replace(p, U=p.U + 0.5), solid=False)
    assert "params" in FP.differences(a, b) and any(k.startswith("collision") or k.startswith("section")
                                                    for k in FP.differences(a, b))
    assert FP.differences(a, FP.fingerprint(fam, p, solid=False)) == []
