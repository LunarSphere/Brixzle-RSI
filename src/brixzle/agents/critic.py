"""Critic agent (issue #19): from a scored design, a redesign brief that cites the design rules.

The critic reads, for the current design:

- **Z per target** with its counts (report.json, schema 4, #61);
- **the jam analysis** (#49, #56): the repeatable jam bands inside the green zone, each with its
  failure type (from the sim) and mechanism (from the predictor's stability tests), and the
  capture limits at the zone's X edges;
- **the fitted map** (#48), as text grids;
- **the design log** (#20), so it does not suggest an idea that already failed. It never sees the
  held-out results (#21);
- **the design rules** (``rules/design_rules.md``, #16) and the sources they cite;
- **the family's code.**

It answers with a brief (structured output, no tools). For each failure cluster: the diagnosis,
the rules it rests on (by id), the proposed change, whether it reshapes the family or needs a new
one, and the expected effect on Z. Or a decision to stop, with its reason. A brief that cites a
rule that does not exist, or a redesign with no cluster, is refused.
"""

import json
from pathlib import Path

from brixzle import rules as RL
from brixzle.agents.runner import AgentCall
from brixzle.engine import jam_analysis as JA
from brixzle.engine import sim_map as SM

CRITIC_BUDGET_USD = 0.40
MAX_TURNS = 4  # a structured answer takes one tool call; the rest is margin
SCOPES = ("reshape", "new_family")
DECISIONS = ("redesign", "stop")
BRIEF_SCHEMA = {
    "type": "object", "additionalProperties": False, "required": ["decision", "reason", "clusters"],
    "properties": {
        "decision": {"type": "string", "enum": list(DECISIONS)},
        "reason": {"type": "string"},
        "clusters": {"type": "array", "items": {
            "type": "object", "additionalProperties": False,
            "required": ["failure", "diagnosis", "rules", "change", "scope", "expected_effect"],
            "properties": {"failure": {"type": "string"}, "diagnosis": {"type": "string"},
                           "rules": {"type": "array", "items": {"type": "string"}},
                           "change": {"type": "string"}, "scope": {"type": "string", "enum": list(SCOPES)},
                           "expected_effect": {"type": "string"}}}},
        # Lessons a later design could use (issue #88); the loop writes them once the next design is scored.
        "lessons": {"type": "array", "items": {
            "type": "object", "additionalProperties": False, "required": ["lesson", "rules", "cluster"],
            "properties": {"lesson": {"type": "string"}, "rules": {"type": "array", "items": {"type": "string"}},
                           "cluster": {"type": "integer"}}}},
    },
}
LESSON_MAX_CHARS = 300


def system_prompt(lessons_text=""):
    lessons = f"\n\n{lessons_text}" if lessons_text else ""
    return f"""You are the critic of Brixzle-RSI, a loop that designs brick families a drone can place imprecisely.
You read one design's simulation results and write a redesign brief for the design agent.

How to work:
- The objective is Z = (seated - jammed) / drops inside the target box; Z = 1 passes. Seated +1, jammed -1,
  fell and wrong valley 0. Raise Z on the target box without lowering it on the other boxes.
- The drone's error is a disc in X-Y, not a line along X; a Y guide can only centre a brick whose centre of mass is still over the course below, so Y capture is at most about half the depth D, and depth costs mass.
- Group the failures into clusters (a jam band, a capture limit on one side, a wrong valley, falls). For each,
  explain the mechanism and cite the design rules it rests on by id (DR1, DR2, ...). Cite only rules listed
  below. Propose one concrete geometric change, say whether it reshapes this family or needs a new family, and
  say what it should do to Z and why. Do not invent results: sim results are "simulated, uncalibrated contact"
  and mechanisms are "predicted, quasi-static 2D".
- Do not propose an idea the design log shows already failed, unless you say what is different now.
- Do not copy a brick, joint or structure from any paper or system.
- Decide "stop" only if Z is already 1 on the target, or you see no change that could raise it; say why.
- Optionally add at most two lessons: a general statement (not a parameter value) that a later design could
  use, the rule ids it rests on, and the index of the cluster it came from. Only for something worth reusing.

Design rules:
{RL.rules_text()}{lessons}"""


def _z_lines(report):
    out = []
    for s in report["statistics"]:
        c = report["z"][s]
        out.append(f"- {s}: Z {c['Z']:+.3f} (S {c['S']}, J {c['J']}, fell {c['fell']}, wrong valley "
                   f"{c['wrong_valley']}, other {c['other']}, N {c['N']})")
    return "\n".join(out)


def _y_lines(report):
    """Z without Y offsets and the measured Y capture (issue #86), when the report has them."""
    out = []
    if report.get("z_x_only"):
        out.append("Z with X offsets only (y = 0): " + ", ".join(f"{s} {report['z_x_only'][s]['Z']:+.3f}"
                                                                for s in report["statistics"]))
    for course, side in (report.get("y_capture_mm") or {}).items():
        out.append(f"Y capture, course {course} (largest |y| still seated in the column through the aim): "
                   f"-y {side['-y']:.1f} mm, +y {side['+y']:.1f} mm")
    return ("\n" + "\n".join(out)) if out else ""


def task_prompt(report, sim_map, jam, family_source, log_text, target):
    grids = "\n\n".join(SM.ascii_grid(g) for g in sim_map["grids"])
    return "\n\n".join([
        f"Target box: {target}. Z per target box (simulated, uncalibrated contact):\n{_z_lines(report)}"
        + _y_lines(report),
        "Fitted sim map (S seated, J jammed, W wrong valley, F fell, ? other; the target box is "
        f"{json.dumps(sim_map['box']['half_widths'])} about the aim):\n{grids}",
        "Jam analysis (sim: simulated, uncalibrated contact; mechanism: predicted, quasi-static 2D):\n"
        + (JA.summary(jam) if jam else "not run"),
        "Design log (earlier attempts):\n" + (log_text or "none"),
        "The family's code (family.py):\n" + family_source,
        "Write the brief.",
    ])


def validate_brief(brief):
    """Raise ValueError unless the brief is well formed and cites only rules that exist."""
    known = {r.id for r in RL.load_rules()}
    if not isinstance(brief, dict) or brief.get("decision") not in DECISIONS:
        raise ValueError(f"a brief needs a decision from {DECISIONS}")
    if not str(brief.get("reason", "")).strip():
        raise ValueError("a brief needs a reason")
    clusters = brief.get("clusters") or []
    if brief["decision"] == "redesign" and not clusters:
        raise ValueError("a redesign brief needs at least one failure cluster")
    for k, c in enumerate(clusters):
        for f in ("failure", "diagnosis", "change", "expected_effect"):
            if not str(c.get(f, "")).strip():
                raise ValueError(f"cluster {k} has no {f}")
        if c.get("scope") not in SCOPES:
            raise ValueError(f"cluster {k}: scope must be one of {SCOPES}")
        rules = c.get("rules") or []
        if not rules:
            raise ValueError(f"cluster {k} cites no design rule")
        unknown = [r for r in rules if r not in known]
        if unknown:
            raise ValueError(f"cluster {k} cites rules that do not exist: {unknown}")
    for k, item in enumerate(brief.get("lessons") or []):
        text = str(item.get("lesson", "")).strip()
        if not text:
            raise ValueError(f"lesson {k} has no lesson text")
        if len(text) > LESSON_MAX_CHARS:
            raise ValueError(f"lesson {k} runs past {LESSON_MAX_CHARS} characters")
        unknown = [r for r in item.get("rules") or [] if r not in known]
        if not item.get("rules") or unknown:
            raise ValueError(f"lesson {k} cites rules that do not exist: {unknown}" if unknown
                             else f"lesson {k} cites no design rule")
        c = item.get("cluster")
        if isinstance(c, bool) or not isinstance(c, int) or not 0 <= c < len(clusters):
            raise ValueError(f"lesson {k}: cluster must index one of the brief's {len(clusters)} clusters")
    return brief


def critique(runner, report, sim_map, jam, family_source, log_text, target, budget_usd=CRITIC_BUDGET_USD,
             lessons_text=""):
    """Run the critic once; returns (brief, AgentResult). Raises ValueError on a malformed brief.
    ``lessons_text`` (loop/lessons.py, #88) goes into its system prompt."""
    call = AgentCall(name="critic", system=system_prompt(lessons_text),
                     prompt=task_prompt(report, sim_map, jam, family_source, log_text, target),
                     max_budget_usd=budget_usd, output_schema=BRIEF_SCHEMA, max_turns=MAX_TURNS)
    result = runner.run(call)
    brief = result.structured if result.structured is not None else json.loads(result.text)
    return validate_brief(brief), result
