"""Tests for the requirements data file and its loader."""

import re
from pathlib import Path

import pytest

from brixzle.requirements import (
    CHECK_KINDS,
    DEFAULT_PATH,
    Requirement,
    RequirementsError,
    load_requirements,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
REQUIREMENTS_MD = REPO_ROOT / "docs" / "requirements.md"
EXPECTED_IDS = [f"R{n}" for n in range(1, 14)]

VALID_ENTRY = """\
  - id: R{n}
    name: Name {n}
    text: Text {n}.
    checks: [rule]
"""


def write_yaml(tmp_path, body):
    path = tmp_path / "requirements.yaml"
    path.write_text(body, encoding="utf-8")
    return path


def full_yaml(skip=None, extra=""):
    entries = "".join(VALID_ENTRY.format(n=n) for n in range(1, 14) if n != skip)
    return "requirements:\n" + entries + extra


def md_table_rows():
    """Return {id: (name, text, checked_by)} from the table in requirements.md."""
    rows = {}
    pattern = re.compile(r"^\| (R\d+) \| \*\*(.+?)\.\*\* (.*) \| (.+) \|$")
    for line in REQUIREMENTS_MD.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if match:
            rid, name, text, checked_by = match.groups()
            rows[rid] = (name, text, checked_by)
    return rows


def md_check_kinds(checked_by):
    """Map a 'Checked by' cell such as 'Rules: x · Sim: y' to check kinds."""
    labels = {
        "rule": {"rule"},
        "rules": {"rule"},
        "sim": {"sim"},
        "design": {"design"},
        "analytic predictor and sim": {"predictor", "sim"},
    }
    kinds = set()
    for part in checked_by.split("·"):
        label = part.split(":", 1)[0].strip().lower()
        kinds |= labels[label]
    return kinds


# --- The shipped data file ---


def test_default_file_loads_r1_to_r13_in_order():
    requirements = load_requirements()
    assert [r.id for r in requirements] == EXPECTED_IDS
    assert all(isinstance(r, Requirement) for r in requirements)


def test_default_path_is_docs_requirements_yaml():
    assert DEFAULT_PATH == REPO_ROOT / "docs" / "requirements.yaml"


def test_yaml_and_markdown_list_same_ids_and_names():
    rows = md_table_rows()
    yaml_pairs = [(r.id, r.name) for r in load_requirements()]
    md_pairs = [(rid, name) for rid, (name, _, _) in rows.items()]
    assert yaml_pairs == md_pairs


def test_yaml_text_matches_markdown_word_for_word():
    rows = md_table_rows()
    for r in load_requirements():
        assert r.text == rows[r.id][1], r.id


def test_yaml_check_kinds_match_markdown_checked_by_column():
    rows = md_table_rows()
    for r in load_requirements():
        assert set(r.checks) == md_check_kinds(rows[r.id][2]), r.id


def test_check_kinds_are_the_four_agreed_kinds():
    assert CHECK_KINDS == frozenset({"rule", "sim", "design", "predictor"})


# --- Deferred requirements (issue #46) ---


def md_deferred_ids():
    """Ids listed in the table under '## Deferred requirements' in requirements.md."""
    text = REQUIREMENTS_MD.read_text(encoding="utf-8")
    section = text.split("## Deferred requirements", 1)[1].split("\n## ", 1)[0]
    return re.findall(r"^\| (R\d+) \|", section, flags=re.MULTILINE)


def test_r12_is_deferred_and_every_other_requirement_is_active():
    status = {r.id: r.status for r in load_requirements()}
    assert status.pop("R12") == "deferred"
    assert set(status.values()) == {"active"}


def test_markdown_lists_the_same_deferred_requirements():
    assert md_deferred_ids() == [r.id for r in load_requirements() if r.status == "deferred"]


def test_status_defaults_to_active(tmp_path):
    assert {r.status for r in load_requirements(write_yaml(tmp_path, full_yaml()))} == {"active"}


def test_a_requirement_can_be_marked_deferred(tmp_path):
    body = full_yaml().replace("checks: [rule]", "checks: [rule]\n    status: deferred", 1)
    assert load_requirements(write_yaml(tmp_path, body))[0].status == "deferred"


def test_rejects_unknown_status(tmp_path):
    body = full_yaml().replace("checks: [rule]", "checks: [rule]\n    status: someday", 1)
    with pytest.raises(RequirementsError, match="status.*someday"):
        load_requirements(write_yaml(tmp_path, body))


# --- Loader validation ---


def test_valid_file_loads(tmp_path):
    requirements = load_requirements(write_yaml(tmp_path, full_yaml()))
    assert requirements[0] == Requirement(
        id="R1", name="Name 1", text="Text 1.", checks=("rule",)
    )


def test_rejects_missing_id(tmp_path):
    path = write_yaml(tmp_path, full_yaml(skip=7))
    with pytest.raises(RequirementsError, match="missing.*R7"):
        load_requirements(path)


def test_rejects_duplicate_id(tmp_path):
    path = write_yaml(tmp_path, full_yaml(extra=VALID_ENTRY.format(n=3)))
    with pytest.raises(RequirementsError, match="duplicate.*R3"):
        load_requirements(path)


def test_rejects_unknown_id(tmp_path):
    path = write_yaml(tmp_path, full_yaml(extra=VALID_ENTRY.format(n=14)))
    with pytest.raises(RequirementsError, match="unknown id.*R14"):
        load_requirements(path)


def test_rejects_unknown_check_kind(tmp_path):
    body = full_yaml().replace("checks: [rule]", "checks: [rule, vibes]", 1)
    with pytest.raises(RequirementsError, match="unknown check kind.*vibes"):
        load_requirements(write_yaml(tmp_path, body))


def test_rejects_empty_checks(tmp_path):
    body = full_yaml().replace("checks: [rule]", "checks: []", 1)
    with pytest.raises(RequirementsError, match="checks"):
        load_requirements(write_yaml(tmp_path, body))


def test_rejects_repeated_check_kind(tmp_path):
    body = full_yaml().replace("checks: [rule]", "checks: [rule, rule]", 1)
    with pytest.raises(RequirementsError, match="repeated check kind.*rule"):
        load_requirements(write_yaml(tmp_path, body))


@pytest.mark.parametrize("field", ["id", "name", "text", "checks"])
def test_rejects_entry_missing_a_field(tmp_path, field):
    lines = [
        line
        for line in VALID_ENTRY.format(n=1).splitlines(keepends=True)
        if not line.lstrip(" -").startswith(f"{field}:")
    ]
    entry = "".join(lines)
    if not entry.startswith("  - "):
        entry = "  - " + entry[4:]
    body = full_yaml(skip=1, extra=entry)
    with pytest.raises(RequirementsError, match=field):
        load_requirements(write_yaml(tmp_path, body))


def test_rejects_unexpected_field(tmp_path):
    body = full_yaml().replace("checks: [rule]", "checks: [rule]\n    owner: me", 1)
    with pytest.raises(RequirementsError, match="unexpected field.*owner"):
        load_requirements(write_yaml(tmp_path, body))


@pytest.mark.parametrize("field", ["name", "text"])
def test_rejects_blank_string_field(tmp_path, field):
    body = full_yaml().replace(f"{field}: ", f"{field}: '   '  # ", 1)
    with pytest.raises(RequirementsError, match=field):
        load_requirements(write_yaml(tmp_path, body))


@pytest.mark.parametrize(
    "body",
    ["", "- just a list\n", "requirements: not-a-list\n", "requirements:\n  - 5\n"],
)
def test_rejects_malformed_structure(tmp_path, body):
    with pytest.raises(RequirementsError):
        load_requirements(write_yaml(tmp_path, body))
