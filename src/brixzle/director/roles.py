"""The Codex copies of the loop's roles and of the repo rules (#229), generated from the Claude sources.

``.claude/agents/rsi-<role>.md`` is the one source of each role: ``director/drive.py`` reads it for
the Codex role calls, and Claude Code reads it as a subagent. A hand-run Codex session reads
``.codex/agents/rsi-<role>.toml`` instead, and ``AGENTS.md`` in place of ``CLAUDE.md``. Those copies
are generated here, so they say exactly what their source says; ``tests/test_codex_roles.py`` fails
when a committed copy drifts.

    uv run python -m brixzle.director.roles sync     # rewrite the copies from their sources
"""
import argparse
from pathlib import Path

ROLES = ("designer", "verifier", "critic", "director")
REPO = Path(__file__).resolve().parents[3]
CLAUDE_TITLE = "# CLAUDE.md\n\nGuidance for Claude Code in this repo."
CODEX_TITLE = "# AGENTS.md\n\nGuidance for Codex in this repo."


def split_frontmatter(md):
    """({key: value}, body) of a role file: ``---`` frontmatter of ``key: value`` lines, then the body."""
    if not md.startswith("---\n"):
        raise ValueError("a role file starts with --- frontmatter")
    head, sep, body = md[4:].partition("\n---\n")
    if not sep:
        raise ValueError("the role file's frontmatter is not closed with ---")
    out = {}
    for line in head.splitlines():
        if line.strip():
            key, colon, value = line.partition(":")
            if not colon:
                raise ValueError(f"frontmatter line without a colon: {line!r}")
            out[key.strip()] = value.strip()
    return out, body


def _basic(s, multiline=False):
    """A TOML basic string: backslashes and quotes escaped, control characters as escapes."""
    out = []
    for ch in s:
        if ch == "\\":
            out.append("\\\\")
        elif ch == '"':
            out.append('\\"')
        elif ch == "\n" and multiline:
            out.append("\n")
        elif ch == "\t":
            out.append("\\t")
        elif ord(ch) < 0x20 or ord(ch) == 0x7F:
            out.append(f"\\u{ord(ch):04x}")
        else:
            out.append(ch)
    text = "".join(out)
    return f'"""\n{text}"""' if multiline else f'"{text}"'


def codex_role(md):
    """The ``.codex/agents`` TOML of a role from its Claude role file: name, description and the body
    as the developer instructions. The model is left to Codex (the drivers choose it per role)."""
    head, body = split_frontmatter(md)
    for key in ("name", "description"):
        if not head.get(key):
            raise ValueError(f"the role file has no {key}")
    return (f"# Generated from .claude/agents/{head['name']}.md by `python -m brixzle.director.roles sync` (#229).\n"
            f"# Edit the .md, not this file.\n"
            f"name = {_basic(head['name'])}\n"
            f"description = {_basic(head['description'])}\n"
            f"developer_instructions = {_basic(body.strip(), multiline=True)}\n")


def agents_md(claude_md):
    """``AGENTS.md`` from ``CLAUDE.md``: the same rules under Codex's title."""
    if not claude_md.startswith(CLAUDE_TITLE):
        raise ValueError(f"CLAUDE.md must start with {CLAUDE_TITLE!r}")
    return CODEX_TITLE + claude_md[len(CLAUDE_TITLE):]


def sync(repo=REPO):
    """Rewrite every Codex copy from its source; returns the paths written."""
    repo = Path(repo)
    written = []
    for role in ROLES:
        out = repo / ".codex" / "agents" / f"rsi-{role}.toml"
        out.write_text(codex_role((repo / ".claude" / "agents" / f"rsi-{role}.md").read_text()))
        written.append(out)
    out = repo / "AGENTS.md"
    out.write_text(agents_md((repo / "CLAUDE.md").read_text()))
    written.append(out)
    return written


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m brixzle.director.roles")
    ap.add_argument("step", choices=("sync",))
    ap.parse_args(argv)
    for path in sync():
        print(path.relative_to(REPO))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
