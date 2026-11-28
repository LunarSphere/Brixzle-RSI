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
            from brixzle.tools.check_family import load_family
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


# --- the class hierarchy (#162) ---

def _line_instance(name):
    from brixzle.families.twinhook.base import TwinHook
    th = registry.get(name).release_lift_mm.__self__
    assert isinstance(th, TwinHook)
    return th


@pytest.mark.parametrize("name", LINE)
def test_each_branch_is_a_twinhook_subclass(name):
    _line_instance(name)


def test_the_branches_share_the_base():
    from brixzle.families.twinhook.base import TwinHook, WebbedTwinHook, WingedTwinHook
    kinds = {name: type(_line_instance(name)) for name in LINE}
    assert kinds["fable_twinhook4"] is TwinHook
    assert issubclass(kinds["fable_reach"], TwinHook) and not issubclass(kinds["fable_reach"], WebbedTwinHook)
    assert all(issubclass(kinds[n], WebbedTwinHook) for n in ("fable_gentle", "fable_feather"))
    assert all(issubclass(kinds[n], WingedTwinHook) for n in ("fable_wide", "fable_allround"))


def _lift_in_worker(family):
    return family.release_lift_mm(family.defaults), len(family.geometry.collision(family.defaults, "A"))


def test_a_branch_family_works_in_a_worker_process():
    import concurrent.futures as cf
    import multiprocessing as mp
    fam = registry.get("fable_gentle")
    with cf.ProcessPoolExecutor(1, mp_context=mp.get_context("spawn")) as ex:
        assert ex.submit(_lift_in_worker, fam).result() == _lift_in_worker(fam)


def test_a_new_design_is_a_small_override():
    """A branch of gentle with thicker walls: a params subclass and a class of a few lines."""
    from dataclasses import dataclass
    from brixzle.families.fable_gentle.family import Gentle, GentleParams

    @dataclass(frozen=True)
    class ThickParams(GentleParams):
        wall: float = 1.6

    class Thick(Gentle):
        name = "test_thick"
        params_type = ThickParams

    fam = Thick().family()
    a = FP.fingerprint(registry.get("fable_gentle"), solid=False, ring=False)
    b = FP.fingerprint(fam, solid=False, ring=False)
    assert fam.defaults.wall == 1.6 and fam.defaults.D == registry.get("fable_gentle").defaults.D
    assert "params" in FP.differences(a, b) and "seats:tower" not in FP.differences(a, b)
