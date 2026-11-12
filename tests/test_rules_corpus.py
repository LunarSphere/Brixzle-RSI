"""Design rules corpus (issue #16): every rule cites a source, every source has a link, no PDFs."""

from pathlib import Path

import pytest

from brixzle import rules as RL

RULES_DIR = Path(RL.__file__).parent


def test_every_rule_has_a_mechanism_a_design_action_citations_and_evidence():
    rules = RL.load_rules()
    assert len(rules) >= 8
    assert [r.id for r in rules] == [f"DR{i}" for i in range(1, len(rules) + 1)]
    for r in rules:
        assert r.title and r.mechanism and r.in_design, r.id
        assert r.cites, f"{r.id} cites nothing"
        assert r.evidence in RL.EVIDENCE, f"{r.id}: evidence {r.evidence!r}"


def test_every_citation_a_rule_names_exists_and_has_a_link():
    cites = RL.load_citations()
    for r in RL.load_rules():
        for key in r.cites:
            assert key in cites, f"{r.id} cites unknown source {key!r}"
    for key, c in cites.items():
        assert c.citation and c.read, key
        assert c.link.startswith("https://"), f"{key}: link {c.link!r}"


def test_every_listed_source_is_used_by_a_rule():
    used = {k for r in RL.load_rules() for k in r.cites}
    assert set(RL.load_citations()) == used


def test_a_rule_resting_on_an_unread_paper_is_not_labelled_as_a_paper_result():
    cites = RL.load_citations()
    for r in RL.load_rules():
        if r.evidence == "paper":
            assert any(not cites[k].read.lower().startswith("not read") for k in r.cites), r.id


def test_the_repo_holds_no_papers():
    assert not list(RULES_DIR.rglob("*.pdf"))


def test_the_rules_render_as_one_text_for_a_prompt():
    text = RL.rules_text()
    assert "DR1." in text and "whitney-1982" in text and "https://" in text


def test_a_rule_without_a_citation_is_rejected(tmp_path):
    bad = tmp_path / "design_rules.md"
    bad.write_text("## DR1. Something\n\n- **Mechanism.** m\n- **In a design.** d\n- **Evidence:** principle\n",
                   encoding="utf-8")
    with pytest.raises(ValueError, match="Cites"):
        RL.load_rules(bad)
