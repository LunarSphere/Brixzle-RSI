"""Intake (issue #17): concept images and extra requirements, the conflict check, intake.json.

The classifier is a Claude call; here it is scripted, so no test calls the API.
"""

import json

import numpy as np

import pytest

from brixzle.agents import runner as RN
from brixzle.intake import intake as IN

PNG = b"\x89PNG\r\n\x1a\n" + b"\x00" * 32


@pytest.fixture
def image(tmp_path):
    p = tmp_path / "sketch.png"
    p.write_bytes(PNG)
    return p


def _answer(items, cost=0.01):
    return RN.AgentResult(structured={"items": items}, cost_usd=cost, turns=1)


CONFLICT = {"id": "X1", "kind": "conflicts", "against": ["R9"], "explanation": "40 g is above R9's 25 g",
            "question": "Keep R9's 25 g limit, or allow 40 g?"}
OK = {"id": "X2", "kind": "compatible", "against": [], "explanation": "a colour does not touch R1–R13",
      "question": ""}


def test_without_extras_there_is_nothing_to_classify_and_no_call(image):
    runner = RN.ScriptedRunner([])
    d = IN.build_intake([image], [], runner=runner)
    assert runner.calls == [] and d["extras"] == [] and d["ready"] is True
    assert d["concept_images"][0]["sha256"] and d["classifier"] is None
    IN.validate_intake(d)


def test_a_conflicting_requirement_is_caught_and_blocks_the_loop_until_resolved(image):
    runner = RN.ScriptedRunner([_answer([CONFLICT, OK])])
    d = IN.build_intake([image], ["bricks may weigh 40 g", "print them in blue"], runner=runner)
    assert [e["kind"] for e in d["extras"]] == ["conflicts", "compatible"]
    assert d["extras"][0]["against"] == ["R9"] and d["ready"] is False
    with pytest.raises(ValueError, match="X1"):
        IN.validate_intake(d)
    call = runner.calls[0]
    assert "R9" in call.system and "Mass" in call.system          # the base requirements are in the prompt
    assert "bricks may weigh 40 g" in call.prompt and call.output_schema is not None
    assert call.tools == ()                                        # a classifier needs no tools


def test_a_resolved_conflict_makes_the_intake_ready(image):
    d = IN.build_intake([image], ["bricks may weigh 40 g", "print them in blue"],
                        runner=RN.ScriptedRunner([_answer([CONFLICT, OK])]), answers={"X1": ("keep_base", "")})
    assert d["ready"] is True and d["extras"][0]["resolution"]["choice"] == "keep_base"
    IN.validate_intake(d)


def test_a_person_is_asked_about_each_conflict_or_unclear_item(image):
    unclear = {**OK, "id": "X2", "kind": "unclear", "question": "Which axis?"}
    asked = []

    def ask(item):
        asked.append(item["id"])
        return ("clarify", "along X") if item["id"] == "X2" else ("drop_extra", "")

    d = IN.build_intake([image], ["40 g", "tilt it"], runner=RN.ScriptedRunner([_answer([CONFLICT, unclear])]),
                        ask=ask)
    assert asked == ["X1", "X2"] and d["ready"]
    assert d["extras"][1]["resolution"] == {"choice": "clarify", "text": "along X", "by": "person"}


@pytest.mark.parametrize("bad", [
    [{**CONFLICT, "kind": "maybe"}],             # not one of the four kinds
    [{**CONFLICT, "against": ["R99"]}],          # not a base requirement
    [CONFLICT],                                  # one answer for two extras
])
def test_a_malformed_classification_is_refused(image, bad):
    with pytest.raises(ValueError):
        IN.build_intake([image], ["40 g", "blue"], runner=RN.ScriptedRunner([_answer(bad)]))


def test_an_unknown_resolution_is_refused(image):
    with pytest.raises(ValueError, match="choice"):
        IN.build_intake([image], ["40 g"], runner=RN.ScriptedRunner([_answer([CONFLICT])]),
                        answers={"X1": ("whatever", "")})


def test_a_missing_image_is_refused(tmp_path):
    with pytest.raises(FileNotFoundError):
        IN.build_intake([tmp_path / "nope.png"], [])


def test_a_fixed_cell_size_is_recorded(image):
    d = IN.build_intake([image], [], fixed_U_mm=25.0)
    assert d["fixed"] == {"U_mm": 25.0}
    with pytest.raises(ValueError):
        IN.build_intake([image], [], fixed_U_mm=-1.0)


def test_intake_json_and_the_images_go_to_a_new_run_directory(image, tmp_path):
    d = IN.build_intake([image], [])
    out = IN.write_intake(d, runs=tmp_path / "runs")
    again = IN.write_intake(d, runs=tmp_path / "runs")
    assert out != again and out.name.startswith("intake-")
    saved = json.loads((out / "intake.json").read_text(encoding="utf-8"))
    assert saved["schema_version"] == IN.SCHEMA_VERSION
    copied = out / saved["concept_images"][0]["path"]
    assert copied.read_bytes() == PNG
    assert IN.load_intake(out / "intake.json")["ready"] is True


def test_the_command_line_writes_intake_json_with_no_extras(image, tmp_path, capsys):
    IN.main(["--image", str(image), "--runs", str(tmp_path / "runs"), "--fix-U", "25"])
    out = capsys.readouterr().out
    path = out.strip().split("-> ")[-1]
    d = json.loads((tmp_path / "runs" / path.split("/")[-1] / "intake.json").read_text(encoding="utf-8"))
    assert d["ready"] and d["fixed"] == {"U_mm": 25.0}


def test_the_command_line_refuses_unresolved_conflicts_without_a_terminal(image, tmp_path, monkeypatch):
    runner = RN.ScriptedRunner([_answer([CONFLICT])])
    monkeypatch.setattr(IN, "make_runner", lambda args: runner)
    with pytest.raises(SystemExit, match="X1"):
        IN.main(["--image", str(image), "--extra", "40 g", "--runs", str(tmp_path), "--no-input"])
    runner = RN.ScriptedRunner([_answer([CONFLICT])])
    monkeypatch.setattr(IN, "make_runner", lambda args: runner)
    IN.main(["--image", str(image), "--extra", "40 g", "--runs", str(tmp_path), "--no-input",
             "--resolve", "X1=keep_base"])


def _big_image(path, w=3000, h=2000):
    from PIL import Image

    x = np.linspace(0, 255, w, dtype=np.uint8)
    y = np.linspace(0, 255, h, dtype=np.uint8)
    arr = np.stack([np.tile(x, (h, 1)), np.tile(y[:, None], (1, w)), np.full((h, w), 90, dtype=np.uint8)], axis=2)
    Image.fromarray(arr).save(path)
    return path


def test_a_large_concept_image_is_kept_as_a_downscaled_copy(tmp_path):
    """The agent read an 18.7 MB photo and its call failed (issue #79)."""
    import hashlib

    from PIL import Image

    big = _big_image(tmp_path / "photo.png")
    d = IN.build_intake([big], [])
    out = IN.write_intake(d, runs=tmp_path / "runs")
    saved = json.loads((out / "intake.json").read_text(encoding="utf-8"))
    img = saved["concept_images"][0]
    with Image.open(out / img["path"]) as im:
        assert max(im.size) == IN.MAX_IMAGE_EDGE and im.size == (1600, 1067)
    assert img["downscaled"] is True and img["original_size"] == [3000, 2000] and img["size"] == [1600, 1067]
    assert img["sha256"] == hashlib.sha256(big.read_bytes()).hexdigest()  # the original's, not the copy's
    assert (out / img["path"]).stat().st_size < big.stat().st_size


def test_a_small_image_is_copied_unchanged(tmp_path, image):
    out = IN.write_intake(IN.build_intake([image], []), runs=tmp_path / "runs")
    saved = json.loads((out / "intake.json").read_text(encoding="utf-8"))
    assert saved["concept_images"][0].get("downscaled") in (None, False)
    assert (out / saved["concept_images"][0]["path"]).read_bytes() == PNG
