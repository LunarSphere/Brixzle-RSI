"""The Codex copies of the roles and the repo rules (#229): generated from the Claude sources, so the two
never drift. ``python -m brixzle.director.roles sync`` rewrites them."""
import json
from pathlib import Path
import tomllib

import pytest

from brixzle.director import roles as RO

ROOT = Path(__file__).resolve().parents[1]
ROLES = ("designer", "verifier", "critic", "director")


@pytest.mark.parametrize("role", ROLES)
def test_each_codex_role_says_what_its_claude_role_says(role):
    md = (ROOT / ".claude" / "agents" / f"rsi-{role}.md").read_text()
    head, body = RO.split_frontmatter(md)
    toml = tomllib.loads((ROOT / ".codex" / "agents" / f"rsi-{role}.toml").read_text())
    assert toml["name"] == head["name"] == f"rsi-{role}"
    assert toml["description"] == head["description"]  # whole, not cut at a comma
    assert toml["developer_instructions"] == body.strip()
    assert (ROOT / ".codex" / "agents" / f"rsi-{role}.toml").read_text() == RO.codex_role(md)


def test_the_codex_toml_round_trips_any_instructions():
    md = "---\nname: rsi-x\ndescription: A role (#1, #2). Does things.\nmodel: opus\n---\n\nBody with \"quotes\", \\ and ``` code.\n"
    out = tomllib.loads(RO.codex_role(md))
    assert out == {"name": "rsi-x", "description": "A role (#1, #2). Does things.",
                   "developer_instructions": 'Body with "quotes", \\ and ``` code.'}


def test_agents_md_is_claude_md_for_codex():
    claude = (ROOT / "CLAUDE.md").read_text()
    agents = (ROOT / "AGENTS.md").read_text()
    assert agents == RO.agents_md(claude)
    assert agents.startswith("# AGENTS.md\n\nGuidance for Codex in this repo.")
    assert agents.split("\n", 3)[3] == claude.split("\n", 3)[3]  # the same rules below the title


def test_the_codex_hook_finds_the_repo_without_claude_project_dir():
    hook = json.loads((ROOT / ".codex" / "hooks.json").read_text())
    (entry,) = hook["hooks"]["PreToolUse"]
    cmd = entry["hooks"][0]["command"]
    assert "${CLAUDE_PROJECT_DIR:-$(git rev-parse --show-toplevel)}" in cmd
    assert "brixzle.director.guard hook" in cmd and ".rsi-loop-active" in cmd
