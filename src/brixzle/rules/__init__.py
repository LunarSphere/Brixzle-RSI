"""Design rules corpus (issue #16): ``design_rules.md`` and ``citations.md``, parsed for the critic.

Each rule in design_rules.md starts with ``## DR<n>. <title>`` and has Mechanism, In a design,
Cites and Evidence lines. Each source in citations.md starts with ``## <key>`` and has Citation,
Link and Read lines. The repo holds links, not papers.
"""

from dataclasses import dataclass
from pathlib import Path
import re

DIR = Path(__file__).resolve().parent
RULES_PATH = DIR / "design_rules.md"
CITATIONS_PATH = DIR / "citations.md"
EVIDENCE = ("paper", "principle", "user observation")

_RULE = re.compile(r"^## (DR\d+)\. (.+)$")
_KEY = re.compile(r"^## ([a-z0-9-]+)$")
_FIELD = re.compile(r"^- \*\*([^*]+?)[.:]\*\*:? ?(.*)$")


@dataclass(frozen=True)
class Rule:
    id: str
    title: str
    mechanism: str
    in_design: str
    cites: tuple
    evidence: str


@dataclass(frozen=True)
class Citation:
    key: str
    citation: str
    link: str
    read: str


def _sections(text, head):
    out, current = [], None
    for line in text.splitlines():
        m = head.match(line)
        if m:
            current = (m.groups(), [])
            out.append(current)
        elif line.startswith("## "):
            current = None
        elif current is not None:
            current[1].append(line)
    return out


def _fields(lines):
    out = {}
    for line in lines:
        m = _FIELD.match(line.strip())
        if m:
            out[m.group(1).strip().lower()] = m.group(2).strip()
    return out


def load_rules(path=RULES_PATH):
    """The rules, in file order; raises ValueError if one lacks a field."""
    rules = []
    for (rid, title), lines in _sections(Path(path).read_text(encoding="utf-8"), _RULE):
        f = _fields(lines)
        for name in ("mechanism", "in a design", "cites", "evidence"):
            if not f.get(name):
                raise ValueError(f"{rid} has no '{name.capitalize()}' line")
        cites = tuple(k.strip() for k in f["cites"].split(",") if k.strip())
        rules.append(Rule(rid, title.strip(), f["mechanism"], f["in a design"], cites, f["evidence"]))
    return rules


def load_citations(path=CITATIONS_PATH):
    """{key: Citation}; raises ValueError if a source lacks a field."""
    out = {}
    for (key,), lines in _sections(Path(path).read_text(encoding="utf-8"), _KEY):
        f = _fields(lines)
        for name in ("citation", "link", "read"):
            if not f.get(name):
                raise ValueError(f"source {key} has no '{name.capitalize()}' line")
        out[key] = Citation(key, f["citation"], f["link"].split()[0], f["read"])
    return out


def rules_text():
    """The rules and the sources they cite, as one text for an agent's prompt."""
    cites = load_citations()
    lines = [RULES_PATH.read_text(encoding="utf-8").strip(), "", "# Sources", ""]
    for c in cites.values():
        lines.append(f"- {c.key}: {c.citation} {c.link} (read in this repo: {c.read})")
    return "\n".join(lines) + "\n"
