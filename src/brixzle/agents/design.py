"""Design agent (issue #18): from a concept, the requirements and a brief, a brick family package.

The agent runs on the Claude Agent SDK (agents/runner.py). Its prompt holds the base requirements
(R1–R13, with deferred ones marked), any extras from intake, the family interface
(docs/family-interface.md) and two example families to read. Its tools:

- **Read, Glob, Grep** over the repo's docs and family code, and its own package;
- **Write, Edit** inside its own package only (``<attempt>/pkg/<module>/``);
- **check_family**: the family contract and the rule margins, run in a separate process with a
  time limit (agents/check_family.py). The agent keeps fixing the family until it passes.
- **probe_drops** (issue #107, when the loop gives a prober): the draft dropped at up to 30 poses,
  the parent's failures inside the target box, a few of its seats and the aim (engine/probe.py),
  at most ``PROBE_CALLS`` times per call. Each call's wall-clock is logged.

The output is a package whose ``family.py`` exports ``FAMILY``. It lives in the loop's run
directory, not in ``src/``, and is imported by path (check_family.load_family).

**Capture features may vary along Y** (issue #105, maintainer, 2026-11-13; it replaced "Prismatic by
default" of 2026-11-06): the interlock may stay prismatic, and funnels, crowns, tapered seats and
other capture features may be ``Solid3D``, most simply ``families.capture.with_capture_features``,
with no reason to give. **No copying from papers or other groups**
(CLAUDE.md): papers are for mechanics. The maintainer's own designs (A/B) may be reused (#85).
"""

from dataclasses import dataclass, field
import json
import re
from pathlib import Path
import time

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
                                                "capture.py", "registry.py", "ab")))
DUMMIES = re.compile(r"\b(wedge|pyramid)\b", re.I)
CHECK_TOOL = "check_family"
PROBE_TOOL = "probe_drops"
PROBE_CALLS = 5  # probe calls per design call (issue #107)


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


def system_prompt(lessons_text=""):
    """The system prompt; ``lessons_text`` (loop/lessons.py, #88) is added at the end if given."""
    from brixzle import rules as RL

    interface = interface_for_agent((REPO / "docs" / "family-interface.md").read_text(encoding="utf-8"))
    lessons = f"\n\n{lessons_text}\n" if lessons_text else ""
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
- Geometry: the interlock may stay prismatic (an XZ section swept through Y; Extrusion bands shear it in Z
  across Y, as A/B's V gutter does). Capture features may vary along Y (funnels, crowns, tapered seats, wings,
  rails): use Solid3D, most simply families.capture.with_capture_features(section, extrusion, features), a
  prismatic interlock plus convex features with exact collision; engine.ring_cells.kit_from_cell builds its ring
  kit from the interlock's cell. No reason is needed for Solid3D.
- A/B is the maintainer's own design, so reuse or change any part of it; your package may import from
  brixzle.families.ab (for example profile, params). It is also the baseline your design must beat.
  Use papers for mechanics and principles only:
  never copy a brick, joint or structure from a paper or from another group's system.
- Units: mm, degrees, grams; quaternions [w, x, y, z]. Give every tunable parameter a bounds entry.
- The goal has two parts, and a brick must meet both:
  (a) it seats from a coarse drop (R11): a brick dropped anywhere inside the drone's error box (X up to
  +-38.5 mm, release height and pitch small) slides on its guide faces into its seat with no jam. The loop
  scores it with Z = (seated - jammed) / drops inside the target box;
  (b) it builds a running-bond wall, a bridge, a ring and an overhang, each brick held by its interlock (R4: a
  hook that a straight lift cannot free). The loop builds a tower, the wall, a bridge and an overhang in
  simulation after the quick check (the structure gate; the ring joins later). A design that cannot build them
  is rejected before it is scored, and a longer overhang is better: it is the loop's second objective.
  Pickup (R12) is deferred; only be grippable (R10).
- Give the family a ring kit (Family.ring, a RingKit): a design that cannot close a ring is rejected. The
  simplest is ring=engine.ring_cells.kit_from_cell(family, cell) from one cell: one voxel of a part's profile,
  centred on x = 0, as a Section. cell(p, part, hooks=True, exposed_lo=False, exposed_hi=False, cut_hi=0.0):
  with hooks=False, leave out any feature that seats along a slanted stroke (the turn piece's side arm nests by
  its keel only); trim an exposed end as the straight brick's ends are; cut cut_hi more off the +x end (the
  closer, the last brick of each course). check_family reports whether the kit builds ("ring").
- R10: give the brick at least two large through-Y openings (each fits a 5 mm circle), above and either side
  of its centre of mass, so it hangs level from two points.
- The drone's error is a disc in X-Y, not a line along X (the mean box reaches about +-14.8 mm in Y). A Y guide
  on the course below can only centre a brick whose centre of mass is still over that course, so Y capture is at
  most about D / 2 (12.5 mm at A/B's D = 25 mm). To capture more you may trade depth, a wider lower course or mass,
  within R10's 35 g for brick plus gripper.
- X and Y guides that engage at once can jam each other. A guide in Y must carry the X interlock across it (a shear of
  the section does), or engage only after X has seated. Simulated: wings with no X ridges on them left every
  Y-offset drop 3 mm off in X; skirts that met the brick far below its centre of mass rolled it 13 degrees (tests/fixtures/yfunnel/family.py).
- When the tools offer probe_drops, probe before you finish: it drops your draft at up to 30 poses, the
  parent's jammed and wrong-valley drops inside the target box, a few of its seats as a control and the aim
  (or, with no parent map, a spread over the error disc), and returns each pose's outcome and rest error
  (simulated, uncalibrated contact). Use it to test whether a change fixes the failure you were asked to fix,
  and try another idea if it does not. At most a few calls per design, so change the package between calls.
- When the check passes, write design_note.json in your package:
  {{"name": ..., "summary": "<one paragraph: the idea and how it captures and seats>",
    "changes": "<what you changed from the parent design and why, or 'first design'>",
    "geometry": "Prismatic" | "Solid3D",
    "probe": "<what probe_drops showed: parent failures now seated, seats now failing; or 'not offered'>"}}
  and stop. Keep your final message to one short paragraph.

Base requirements (R1-R13):
{requirements_text()}

Family interface (docs/family-interface.md):
{interface}

Design rules (rules/design_rules.md; the critic cites them by id):
{RL.rules_text()}
{lessons}"""


def repair_text(repair):
    """The repair task (#109): keep what captures, fix what does not build."""
    z = repair.get("z_text") or ""
    return (f"This is a repair. The design below captured well enough to be kept on Z{z}, but it does not build "
            f"yet (its failing builds follow), so it was not kept. Start from its file, keep the capture geometry "
            f"(the guide faces, funnels and seats that gave it its Z) and fix the interlock or the ring kit so it "
            f"builds the tower, the wall, the bridge, the overhang and the ring. This is repair "
            f"{repair.get('attempt', 1)} of {repair.get('of', 2)}; after that it is rejected.\n"
            f"Design to repair: {repair['family_py']}")


def task_prompt(ws, intake, brief=None, parent=None, failures=None, log_text="", fixed=None, explore=None,
                repair=None, pictures=()):
    """The task: the concept, the extras, the parent design and what to change. ``explore`` (#106):
    {'principle', 'tried', 'beat'} for an explore call, which gets no parent to reshape."""
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
    if repair is not None:
        parts.append(repair_text(repair))
    elif explore is not None:
        parts.append(explore_text(explore))
    elif parent is not None:
        parts.append(f"Parent design (the current best), to reshape or replace: {parent}")
    if pictures:
        parts.append("Pictures of the parent design to read with the Read tool before you change it (#112): its jam "
                     "map, its module and where failed drops came to rest (rest-K-xz side on, rest-K-yz end on; "
                     "simulated, uncalibrated contact):\n" + "\n".join(f"- {q}" for q in pictures))
    if brief is not None:
        parts.append("Redesign brief from the critic:\n" + json.dumps(brief, indent=2))
    if failures and "ring" in failures:
        parts.append("The last design was scored and would have been kept, but it could not close a ring (6 x 6 cells, "
                     "2 courses, ideal placement; simulated, uncalibrated contact), so it was rejected. Where the ring "
                     "stopped:\n" + json.dumps(failures["ring"], indent=2))
    elif failures and "structure_gate" in failures:
        parts.append("The last attempt passed the quick check but failed the structure gate: it must build a tower, "
                     "a running-bond wall and a bridge, and an overhang of at least "
                     f"{failures['structure_gate']['overhang_floor']} bricks (each built on seeded repeats; "
                     "simulated, uncalibrated contact). Its builds:\n" + json.dumps(failures["structure_gate"], indent=2))
    elif failures:
        parts.append("The last attempt failed the quick check (32 of 40 drops must seat; simulated, uncalibrated "
                     "contact). Its failing drops:\n" + json.dumps(failures, indent=2))
    if log_text:
        parts.append("Design log (earlier attempts; do not repeat an idea that failed):\n" + log_text)
    parts.append("Write the family, run check_family until it passes, write design_note.json, and stop.")
    return "\n\n".join(parts)


PRINCIPLES = REPO / "src" / "brixzle" / "rules" / "capture_principles.md"


def principles():
    """The capture principles for explore rounds (#106): [{'id', 'title', 'text'}] in file order."""
    out = []
    for block in re.split(r"^## ", PRINCIPLES.read_text(encoding="utf-8"), flags=re.M)[1:]:
        head, _, body = block.partition("\n")
        m = re.match(r"(CP\d+)\.\s*(.+)", head.strip())
        if m:
            out.append({"id": m.group(1), "title": m.group(2).strip(), "text": " ".join(body.split())})
    return out


CONCEPT_KEYS = ("name", "failure", "why", "cost")


def concepts_problem(note):
    """Why an explore note's concepts (#111) are not acceptable, or None."""
    cs = note.get("concepts")
    if not isinstance(cs, list) or len(cs) != 3:
        return "the note must list three concepts (\"concepts\": three mechanisms) before the one built"
    for k, c in enumerate(cs):
        if not isinstance(c, dict) or any(not str(c.get(q, "")).strip() for q in CONCEPT_KEYS):
            return f"concept {k + 1} needs {', '.join(CONCEPT_KEYS)}"
    if not str(note.get("picked", "")).strip():
        return "the note must say which concept was built (\"picked\")"
    return None


def explore_text(explore):
    """The explore task's part of the prompt (#106): the principle, what was tried, and A/B as the
    score to beat."""
    p = explore["principle"]
    tried = ", ".join(explore.get("tried") or []) or "none"
    return (f"This is an explore call: design a NEW brick from the capture principle below. Write your own "
            f"profile and interlock. A/B is the score to beat, not your starting file: the loop refuses a design "
            f"whose section is a near copy of A/B's (a subclass of its parameters with a few numbers changed); "
            f"check_family reports it ('novelty') and is not ok until your section differs enough. "
            f"You may still read A/B for how a family is written.\n"
            f"Score to beat: {explore.get('beat') or 'A/B'}.\n"
            f"Principle {p['id']}. {p['title']}: {p['text']}\n"
            f"Already tried: {tried}. Do not repeat a concept the design log shows failing.\n"
            f"Concepts before code (#111): before you write the family, think of three different mechanisms built on "
            f"this principle. In design_note.json give them as \"concepts\": a list of three objects, each with "
            f"\"name\", \"failure\" (the failure it targets, from the brief or the map), \"why\" (why it should work, "
            f"with design rule ids) and \"cost\" (mass, depth, printability), then \"picked\" (the name you built) "
            f"and \"picked_why\". Then build the one you picked, and probe it with probe_drops before you finish: "
            f"an explore call without three concepts or without a probe is counted as a failed attempt.")


def check_tool(ws, reference=None, share=None):
    def handler(_args):
        return json.dumps(CF.check_in_subprocess(ws.root, ws.module, reference=reference, share=share), indent=1)

    return ToolSpec(CHECK_TOOL, "Check the family in your package: the family contract and the rule margins. "
                    "Returns JSON with ok, violations, broken rules, margins and mass.", {}, handler)


def probe_tool(ws, prober, log, cap=PROBE_CALLS):
    """The probe_drops tool (issue #107): ``prober(root, module)`` drops the package at the loop's
    probe poses. Every call is appended to ``log`` (its number, wall-clock and outcomes); after
    ``cap`` calls the tool answers with an error and runs nothing."""
    def handler(_args):
        if len(log) >= cap:
            return json.dumps({"error": f"probe limit reached: {cap} calls per design; finish the design"})
        t = time.time()
        out = prober(ws.root, ws.module)
        entry = {"call": len(log) + 1, "seconds": round(time.time() - t, 2), "outcomes": out.get("outcomes"),
                 "parent_failures_now_seated": out.get("parent_failures_now_seated"),
                 "parent_seated_now_failing": out.get("parent_seated_now_failing"), "error": out.get("error")}
        log.append(entry)
        return json.dumps({**out, "calls_left": cap - len(log)}, indent=1)

    return ToolSpec(PROBE_TOOL, "Drop the draft in your package at up to 30 poses: the parent's failures inside the "
                    "target box, a few of its seats and the aim (or a spread over the error disc). Returns each "
                    "pose's outcome and rest error. Simulated, uncalibrated contact. Limited calls per design.",
                    {}, handler)


@dataclass
class DesignOutcome:
    workspace: Workspace
    result: object        # AgentResult
    check: dict           # the final check (run by the loop, not trusted from the agent)
    note: dict
    probes: list = field(default_factory=list)  # one entry per probe_drops call (issue #107)

    @property
    def ok(self):
        return bool(self.check.get("ok"))


def design(runner, intake, attempt_dir, module, brief=None, parent=None, failures=None, log_text="",
           budget_usd=DESIGN_BUDGET_USD, max_turns=MAX_TURNS, lessons_text="", prober=None,
           probe_calls=PROBE_CALLS, explore=None, repair=None, pictures=()):
    """Run the design agent once; returns a DesignOutcome. The loop checks the package itself
    afterwards, whatever the agent said. ``prober(root, module)``, if given, is served as the
    probe_drops tool, at most ``probe_calls`` times (issue #107)."""
    ws = Workspace.make(attempt_dir, module)
    probes = []
    ref, share = (explore or {}).get("reference"), (explore or {}).get("share")
    tools = (check_tool(ws, ref, share),) + ((probe_tool(ws, prober, probes, probe_calls),) if prober is not None else ())
    call = AgentCall(name="design", system=system_prompt(lessons_text),
                     prompt=task_prompt(ws, intake, brief, parent, failures, log_text, intake.get("fixed"), explore,
                                        repair, pictures),
                     max_budget_usd=budget_usd, tools=("Read", "Glob", "Grep", "Write", "Edit"), cwd=ws.pkg,
                     write_root=ws.pkg, read_roots=(*READ_ROOTS, Path(attempt_dir).parent.parent,
                                                    *dict.fromkeys(Path(q).parent for q in pictures),
                                                    *(Path(i["abs_path"]).parent for i in intake["concept_images"])),
                     mcp_tools=tools, max_turns=max_turns)
    result = runner.run(call)
    final = CF.check_in_subprocess(ws.root, ws.module, reference=ref, share=share)
    note_path = ws.pkg / NOTE_FILE
    try:
        note = json.loads(note_path.read_text(encoding="utf-8")) if note_path.exists() else {}
    except json.JSONDecodeError:
        note = {"error": "design_note.json is not valid JSON"}
    return DesignOutcome(ws, result, final, note, probes)
