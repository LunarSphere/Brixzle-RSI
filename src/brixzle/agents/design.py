"""Design agent (issue #18): from a concept, the requirements and a brief, a brick family package.

The agent runs on the Claude Agent SDK (agents/runner.py). Its prompt holds the base requirements
(R1–R13, with deferred ones marked), any extras from intake, the family interface
(docs/family-interface.md) and two example families to read. Its tools:

- **Read, Glob, Grep** over the repo's docs and family code, and its own package;
- **Write, Edit** inside its own package only (``<attempt>/pkg/<module>/``);
- **check_family**: the family contract and the rule margins, run in a separate process with a
  time limit (agents/check_family.py). The agent keeps fixing the family until it passes.

The output is a package whose ``family.py`` exports ``FAMILY``. It lives in the loop's run
directory, not in ``src/``, and is imported by path (check_family.load_family).

**Prismatic by default** (maintainer, 2026-11-06, on #18): a ``Solid3D`` family is allowed only
when a needed feature cannot be prismatic, and the agent must say which feature and why in its
``design_note.json``, which the design log records. **No copying from papers or other groups**
(CLAUDE.md): papers are for mechanics. The maintainer's own designs (A/B) may be reused (#85).
"""

from dataclasses import dataclass
import json
import re
from pathlib import Path

from brixzle.agents import check_family as CF
from brixzle.agents.runner import AgentCall, ToolSpec
from brixzle.intake.intake import requirements_text

REPO = Path(__file__).resolve().parents[3]
DESIGN_BUDGET_USD = 1.50
MAX_TURNS = 40
NOTE_FILE = "design_note.json"
# A/B only: the agent copied the dummy wedge as its brick in #21's smoke runs (issue #73).
EXAMPLES = ("src/brixzle/families/ab/family.py",)
_SRC = REPO / "src" / "brixzle"
# What the agent may read in the repo: the docs, the engine, the rules, the family interface and its
# shared code, and A/B. Not the dummy families (issue #73).
READ_ROOTS = (REPO / "docs", _SRC / "engine", _SRC / "rules", _SRC / "requirements.py",
              *(_SRC / "families" / f for f in ("interface.py", "builders.py", "rules.py", "contract.py", "convex.py",
                                                "registry.py", "ab")))
DUMMIES = re.compile(r"\b(wedge|pyramid)\b", re.I)
CHECK_TOOL = "check_family"


@dataclass(frozen=True)
class Workspace:
    root: Path      # on sys.path
    module: str     # the package name, unique per attempt
    pkg: Path       # root / module

    @classmethod
    def make(cls, attempt_dir, module):
        root = Path(attempt_dir) / "pkg"
        pkg = root / module
        pkg.mkdir(parents=True, exist_ok=False)
        (pkg / "__init__.py").write_text(f'"""Brick family written by the design agent ({module})."""\n',
                                         encoding="utf-8")
        return cls(root, module, pkg)


def interface_for_agent(text):
    """The family interface without the sentences that name the dummy families (issue #73), so the
    agent is not pointed at a contract-test dummy as a brick to start from."""
    out = []
    for line in text.splitlines():
        if not DUMMIES.search(line):
            out.append(line)
            continue
        kept = [s for s in re.split(r"(?<=[.:])\s+", line) if not DUMMIES.search(s)]
        if kept and kept != ["-"]:
            out.append(" ".join(kept))
    return "\n".join(out)


def system_prompt():
    interface = interface_for_agent((REPO / "docs" / "family-interface.md").read_text(encoding="utf-8"))
    return f"""You are the design agent of Brixzle-RSI, a loop that designs brick families a drone can place imprecisely.
You write one brick family as Python code that follows the family interface below, then check it with the
check_family tool and fix it until the check passes (ok: true). The check also drops your brick once at the
aim on each course (aim_drops): if either is not "seated", the brick does not sit in its own seat, so fix the
seat pose, the base or the profile before anything else. The drop trial seats a course-0 brick at
Placement(0, 1) on the base. A course-1 brick lands across the joint between two seated course-0 bricks,
Placement(0, 1) and Placement(0, 1 + span), at Placement(1, 1 + span // 2): a running bond, as in a wall, a bridge
or an overhang. A brick that only stacks straight on one brick fails this. err_mm is where the brick came to rest
against its seat on the first brick below.

Rules:
- Write only inside your package directory. Its family.py must export FAMILY = Family(...).
- Prismatic (2.5D: an XZ section swept through Y) by default. Use Solid3D only if a feature you need cannot be
  prismatic; then say which feature and why in design_note.json.
- A/B is the maintainer's own design, so reuse or change any part of it; your package may import from
  brixzle.families.ab (for example profile, params). It is also the baseline your design must beat.
  Use papers for mechanics and principles only:
  never copy a brick, joint or structure from a paper or from another group's system.
- Units: mm, degrees, grams; quaternions [w, x, y, z]. Give every tunable parameter a bounds entry.
- The goal is placement (R11): a brick dropped anywhere inside the drone's error box (X up to +-38.5 mm,
  release height and pitch small) slides on its guide faces into its seat with no jam. The loop scores it
  with Z = (seated - jammed) / drops inside the target box. Pickup (R12) is deferred; only be grippable (R10).
- R10: give the brick at least two large through-Y openings (each fits a 5 mm circle), above and either side
  of its centre of mass, so it hangs level from two points.
- The drone's error is a disc in X-Y, not a line along X; a Y guide can only centre a brick whose centre of mass is still over the course below, so Y capture is at most about half the depth D, and depth costs mass.
- When the check passes, write design_note.json in your package:
  {{"name": ..., "summary": "<one paragraph: the idea and how it captures and seats>",
    "changes": "<what you changed from the parent design and why, or 'first design'>",
    "geometry": "Prismatic" | "Solid3D", "solid3d_reason": "<only for Solid3D>"}}
  and stop. Keep your final message to one short paragraph.

Base requirements (R1-R13):
{requirements_text()}

Family interface (docs/family-interface.md):
{interface}
"""


def task_prompt(ws, intake, brief=None, parent=None, failures=None, log_text="", fixed=None):
    """The task: the concept, the extras, the parent design and what to change."""
    images = "\n".join(f"- {img['abs_path']}" for img in intake["concept_images"])
    extras = [e for e in intake.get("extras", []) if (e.get("resolution") or {}).get("choice") != "drop_extra"]
    extra_lines = "\n".join(
        f"- {e['id']} ({e['kind']}): {e['text']}" + (f" -> resolved: {e['resolution']['choice']} "
                                                     f"{e['resolution']['text']}".rstrip() if e.get("resolution") else "")
        for e in extras) or "- none"
    parts = [f"Your package directory: {ws.pkg}\nIts package name: {ws.module} (import siblings relatively).",
             f"Concept images (read them with the Read tool):\n{images}",
             f"Extra requirements from intake:\n{extra_lines}",
             "The A/B family (the maintainer's own design and the baseline; you may reuse or change any part of "
             "it): " + ", ".join(str(REPO / e) for e in EXAMPLES)]
    if (fixed or {}).get("U_mm"):
        parts.append(f"The voxel pitch U is fixed at intake: {fixed['U_mm']} mm. Do not give U a bounds entry.")
    if parent is not None:
        parts.append(f"Parent design (the current best), to reshape or replace: {parent}")
    if brief is not None:
        parts.append("Redesign brief from the critic:\n" + json.dumps(brief, indent=2))
    if failures:
        parts.append("The last attempt failed the quick check (32 of 40 drops must seat; simulated, uncalibrated "
                     "contact). Its failing drops:\n" + json.dumps(failures, indent=2))
    if log_text:
        parts.append("Design log (earlier attempts; do not repeat an idea that failed):\n" + log_text)
    parts.append("Write the family, run check_family until it passes, write design_note.json, and stop.")
    return "\n\n".join(parts)


def check_tool(ws):
    def handler(_args):
        return json.dumps(CF.check_in_subprocess(ws.root, ws.module), indent=1)

    return ToolSpec(CHECK_TOOL, "Check the family in your package: the family contract and the rule margins. "
                    "Returns JSON with ok, violations, broken rules, margins and mass.", {}, handler)


@dataclass
class DesignOutcome:
    workspace: Workspace
    result: object        # AgentResult
    check: dict           # the final check (run by the loop, not trusted from the agent)
    note: dict

    @property
    def ok(self):
        return bool(self.check.get("ok"))


def design(runner, intake, attempt_dir, module, brief=None, parent=None, failures=None, log_text="",
           budget_usd=DESIGN_BUDGET_USD, max_turns=MAX_TURNS):
    """Run the design agent once; returns a DesignOutcome. The loop checks the package itself
    afterwards, whatever the agent said."""
    ws = Workspace.make(attempt_dir, module)
    call = AgentCall(name="design", system=system_prompt(),
                     prompt=task_prompt(ws, intake, brief, parent, failures, log_text, intake.get("fixed")),
                     max_budget_usd=budget_usd, tools=("Read", "Glob", "Grep", "Write", "Edit"), cwd=ws.pkg,
                     write_root=ws.pkg, read_roots=(*READ_ROOTS, Path(attempt_dir).parent.parent,
                                                    *(Path(i["abs_path"]).parent for i in intake["concept_images"])),
                     mcp_tools=(check_tool(ws),), max_turns=max_turns)
    result = runner.run(call)
    final = CF.check_in_subprocess(ws.root, ws.module)
    note_path = ws.pkg / NOTE_FILE
    try:
        note = json.loads(note_path.read_text(encoding="utf-8")) if note_path.exists() else {}
    except json.JSONDecodeError:
        note = {"error": "design_note.json is not valid JSON"}
    return DesignOutcome(ws, result, final, note)
