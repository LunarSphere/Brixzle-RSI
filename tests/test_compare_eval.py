"""scripts/compare_eval.py finds every difference between two evaluations (#162)."""
import gzip
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("compare_eval", ROOT / "scripts" / "compare_eval.py")
CE = importlib.util.module_from_spec(spec)
spec.loader.exec_module(CE)


def _eval(tmp, name, outcome="seated", tower="complete", ring="complete", S=1):
    d = tmp / name
    (d / "map").mkdir(parents=True)
    (d / "gate").mkdir()
    drop = {"slice": "xz", "course": 0, "cell": [0, 0], "dx_mm": 0.0, "clear_mm": 5.0, "pitch_deg": 0.0,
            "outcome": outcome, "err_mm": [0.1, 0.0, 0.0]}
    (d / "map" / "drops.json.gz").write_bytes(gzip.compress(json.dumps([drop]).encode()))
    (d / "gate" / "gate.json").write_text(json.dumps({"passed": True, "overhang_n": 9, "builds": {
        "tower": {"status": tower, "complete": 5, "n": 5, "statuses": [tower] * 5}}}))
    (d / "eval.json").write_text(json.dumps({
        "map": str(d / "map"), "gate": {"run": str(d / "gate")}, "z": {"mean": {"S": S, "J": 0, "N": 1, "Z": S}},
        "held_out": {}, "ring": {"status": ring, "seated": 16, "total": 16, "failed_brick": None}, "mass_g": 19.2}))
    return d


def test_identical_evaluations(tmp_path):
    r = CE.compare(_eval(tmp_path, "a"), _eval(tmp_path, "b"))
    assert r["identical"] and r["drops_compared"] == 1 and r["differences"] == []


def test_each_kind_of_difference_is_named(tmp_path):
    a = _eval(tmp_path, "a")
    r = CE.compare(a, _eval(tmp_path, "b", outcome="jammed", tower="stopped", ring="stopped", S=0))
    text = " ".join(r["differences"])
    assert not r["identical"]
    assert "1 drops differ (1 in outcome)" in text and "Z mean" in text and "gate tower" in text and "ring" in text
