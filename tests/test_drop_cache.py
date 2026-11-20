"""The drop cache (#163): a drop already simulated for the same sim geometry, config and pose is read,
not run again, and gives exactly the same row."""
from dataclasses import replace

import pytest

from brixzle.engine import box_search as BS
from brixzle.engine import drop_cache as DC
from brixzle.engine.sim import trials as T
from brixzle.families import registry


def _cfg():
    return T.TrialConfig(error=T.ErrorModel(ideal=True))


def _jobs(cfg, xs=(-3.0, 0.0, 4.0)):
    base = cfg.error.sample(None)
    return [(c, dict(base, dx=base["aim"] + x), {"trace": False}) for c in (0, 1) for x in xs]


@pytest.fixture(scope="module")
def ab():
    fam = registry.get("ab")
    return fam, fam.defaults


def test_cached_rows_are_identical_and_not_rerun(tmp_path, ab):
    fam, p = ab
    cfg, cache = _cfg(), DC.DropCache(tmp_path / "drops.sqlite")
    with BS.DropPool(fam, p, cfg, workers=1) as pool:
        plain = list(pool.imap(_jobs(cfg)))
    with BS.DropPool(fam, p, cfg, workers=1, cache=cache) as pool:
        first = list(pool.imap(_jobs(cfg)))
        assert pool.cache_stats == {"hits": 0, "misses": 6}
    with BS.DropPool(fam, p, cfg, workers=2, cache=cache) as pool:
        second = list(pool.imap(_jobs(cfg)))
        assert pool.cache_stats == {"hits": 6, "misses": 0}
    assert first == plain and second == plain


def test_partial_hits_keep_job_order(tmp_path, ab):
    fam, p = ab
    cfg, cache = _cfg(), DC.DropCache(tmp_path / "drops.sqlite")
    with BS.DropPool(fam, p, cfg, workers=1, cache=cache) as pool:
        some = list(pool.imap(_jobs(cfg, xs=(0.0,))))
    with BS.DropPool(fam, p, cfg, workers=2, cache=cache) as pool:
        rows = list(pool.imap(_jobs(cfg)))
        assert pool.cache_stats == {"hits": 2, "misses": 4}
    assert [rows[1], rows[4]] == some


def test_the_key_follows_geometry_config_and_pose(ab):
    fam, p = ab
    cfg = _cfg()
    g = DC.geometry_key(fam, p)
    assert g == DC.geometry_key(fam, p)
    assert g != DC.geometry_key(fam, replace(p, U=p.U + 0.5))
    job = _jobs(cfg)[0]
    k = DC.job_key(g, cfg, job)
    assert k == DC.job_key(g, cfg, job)
    assert k != DC.job_key(g, replace(cfg, physics=replace(cfg.physics, friction=cfg.physics.friction + 0.1)), job)
    assert k != DC.job_key(g, cfg, (job[0], dict(job[1], dx=job[1]["dx"] + 0.001), job[2]))
    assert k != DC.job_key(g, cfg, (job[0], job[1], {"trace": True}))
    assert k != DC.job_key(g, cfg, (job[0], job[1], {"trace": False, "friction": 0.5}))


def test_the_same_geometry_under_another_name_shares_the_cache(ab):
    fam, p = ab
    assert DC.geometry_key(fam, p) == DC.geometry_key(replace(fam, name="ab_copy", version="9"), p)


def test_recorded_and_pickup_jobs_are_never_cached():
    assert not DC.cacheable((0, {}, {"record": True}))
    assert not DC.cacheable(("pick", {}))
    assert DC.cacheable((1, {}, {"trace": False}))


def test_the_environment_turns_the_cache_on(tmp_path, monkeypatch, ab):
    monkeypatch.delenv("BRIXZLE_DROP_CACHE", raising=False)
    assert DC.default() is None
    monkeypatch.setenv("BRIXZLE_DROP_CACHE", "off")
    assert DC.default() is None
    monkeypatch.setenv("BRIXZLE_DROP_CACHE", str(tmp_path / "c.sqlite"))
    fam, p = ab
    cfg = _cfg()
    with BS.DropPool(fam, p, cfg, workers=1) as pool:
        list(pool.imap(_jobs(cfg, xs=(0.0,))))
        assert pool.cache is not None and pool.cache_stats["misses"] == 2
    assert len(DC.DropCache(tmp_path / "c.sqlite")) == 2
