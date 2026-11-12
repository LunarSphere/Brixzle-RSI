"""Intake (issue #17): concept images and extra requirements, the conflict check, intake.json.

A person gives one or more concept images and, optionally, extra requirements. Each extra is
sorted against the base requirements (R1–R13, docs/requirements.yaml) into one of four kinds:

- **compatible:** it adds something R1–R13 do not touch;
- **refines:** it narrows a base requirement (for example a lower mass limit than R9's);
- **conflicts:** it cannot hold together with a base requirement;
- **unclear:** it cannot be judged as written.

The sorting is one Claude call (agents/runner.py) with a structured answer and no tools. Every
conflict and every unclear item must be resolved by the person before the loop may start: keep
the base requirement, use the extra instead, drop the extra, or clarify it in their own words.
Nothing reaches the loop until then (``ready``). With no extras there is nothing to sort and no
call is made.

The output is ``runs/intake-<UTC>/intake.json``, with copies of the images beside it. It is the
only input to the loop's first iteration. This is the command-line intake; a local page that
writes the same intake.json is still planned (#17).

    uv run python -m brixzle.intake --image runs/concept-images/1-zigzag-sketch.png
    uv run --extra agents python -m brixzle.intake --image sketch.png --extra "bricks may weigh 40 g"
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys

from brixzle.requirements import load_requirements

SCHEMA_VERSION = 1
KINDS = ("compatible", "refines", "conflicts", "unclear")
NEEDS_ANSWER = ("conflicts", "unclear")
CHOICES = ("keep_base", "use_extra", "drop_extra", "clarify")
CLASSIFY_BUDGET_USD = 0.25
CLASSIFY_SCHEMA = {
    "type": "object", "additionalProperties": False, "required": ["items"],
    "properties": {"items": {"type": "array", "items": {
        "type": "object", "additionalProperties": False,
        "required": ["id", "kind", "against", "explanation", "question"],
        "properties": {"id": {"type": "string"}, "kind": {"type": "string", "enum": list(KINDS)},
                       "against": {"type": "array", "items": {"type": "string"}},
                       "explanation": {"type": "string"}, "question": {"type": "string"}}}}},
}


def requirements_text():
    """The base requirements, one per line, for the classifier's prompt."""
    lines = []
    for r in load_requirements():
        note = " (deferred: not scored or designed for yet)" if r.status == "deferred" else ""
        lines.append(f"- {r.id} {r.name}{note}: {r.text}")
    return "\n".join(lines)


def classify_system():
    return (
        "You check extra requirements for a brick family that a drone picks up and places imprecisely. "
        "Sort each extra requirement against the base requirements below into exactly one kind:\n"
        "- compatible: it adds something the base requirements do not touch;\n"
        "- refines: it narrows or sharpens a base requirement without breaking it;\n"
        "- conflicts: it cannot hold together with a base requirement;\n"
        "- unclear: it cannot be judged as written.\n"
        "For each, list the ids of the base requirements it touches in 'against' (empty if none), explain in "
        "one sentence, and for conflicts and unclear items write one short question the person can answer "
        "(empty otherwise). Keep each extra's id. Do not invent requirements.\n\n"
        f"Base requirements:\n{requirements_text()}\n")


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def classify(extras, runner):
    """[{id, text, kind, against, explanation, question}] for the extras, from one Claude call."""
    from brixzle.agents.runner import AgentCall

    ids = [f"X{i + 1}" for i in range(len(extras))]
    prompt = "Extra requirements:\n" + "\n".join(f"{i}: {t}" for i, t in zip(ids, extras))
    result = runner.run(AgentCall(name="intake", system=classify_system(), prompt=prompt,
                                  max_budget_usd=CLASSIFY_BUDGET_USD, output_schema=CLASSIFY_SCHEMA, max_turns=3))
    answer = result.structured if result.structured is not None else json.loads(result.text)
    items = answer.get("items") if isinstance(answer, dict) else None
    if not isinstance(items, list) or sorted(i.get("id") for i in items) != sorted(ids):
        raise ValueError(f"the classifier must answer once for each of {ids}, got {answer!r}")
    known = {r.id for r in load_requirements()}
    by_id = {i["id"]: i for i in items}
    out = []
    for rid, text in zip(ids, extras):
        i = by_id[rid]
        if i.get("kind") not in KINDS:
            raise ValueError(f"{rid}: kind must be one of {KINDS}, got {i.get('kind')!r}")
        bad = [a for a in i.get("against", []) if a not in known]
        if bad:
            raise ValueError(f"{rid}: 'against' names unknown requirements {bad}")
        out.append({"id": rid, "text": text, "kind": i["kind"], "against": list(i.get("against", [])),
                    "explanation": str(i.get("explanation", "")), "question": str(i.get("question", "")),
                    "resolution": None})
    return out, {"cost_usd": result.cost_usd, "turns": result.turns}


def _resolution(rid, choice, text, by):
    if choice not in CHOICES:
        raise ValueError(f"{rid}: the choice must be one of {CHOICES}, got {choice!r}")
    if choice == "clarify" and not (text or "").strip():
        raise ValueError(f"{rid}: a clarification needs its text")
    return {"choice": choice, "text": (text or "").strip(), "by": by}


def build_intake(images, extras, runner=None, answers=None, ask=None, fixed_U_mm=None, notes=""):
    """intake.json as a dict. ``answers`` maps an extra's id to (choice, text); ``ask(item)``
    returns (choice, text) for any conflict or unclear item still open (a person at a prompt)."""
    images = [Path(p) for p in images]
    if not images:
        raise ValueError("intake needs at least one concept image")
    for p in images:
        if not p.is_file():
            raise FileNotFoundError(f"no concept image at {p}")
    if fixed_U_mm is not None and not (math.isfinite(fixed_U_mm) and fixed_U_mm > 0):
        raise ValueError(f"a fixed cell size must be a finite length > 0 mm, got {fixed_U_mm!r}")
    extras = [e.strip() for e in extras if e and e.strip()]
    items, classifier = [], None
    if extras:
        if runner is None:
            raise ValueError("extra requirements need a runner to classify them")
        items, classifier = classify(extras, runner)
        classifier["model"] = getattr(runner, "model", None)
    for item in items:
        if item["kind"] not in NEEDS_ANSWER:
            continue
        if answers and item["id"] in answers:
            item["resolution"] = _resolution(item["id"], *answers[item["id"]], by="person")
        elif ask is not None:
            item["resolution"] = _resolution(item["id"], *ask(item), by="person")
    open_ = [i["id"] for i in items if i["kind"] in NEEDS_ANSWER and i["resolution"] is None]
    return {
        "schema_version": SCHEMA_VERSION, "created": datetime.now(timezone.utc).isoformat(),
        "concept_images": [{"source": str(p), "name": p.name, "sha256": _sha256(p)} for p in images],
        "notes": notes, "extras": items, "open": open_, "ready": not open_,
        "fixed": {} if fixed_U_mm is None else {"U_mm": float(fixed_U_mm)},
        "requirements": {"file": "docs/requirements.yaml",
                         "active": [r.id for r in load_requirements() if r.status == "active"],
                         "deferred": [r.id for r in load_requirements() if r.status == "deferred"]},
        "classifier": classifier,
    }


def validate_intake(d):
    """Raise ValueError unless ``d`` is a ready intake: every conflict and unclear item resolved."""
    if d.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"intake schema_version must be {SCHEMA_VERSION}")
    if not d.get("concept_images"):
        raise ValueError("intake has no concept image")
    open_ = [e["id"] for e in d["extras"] if e["kind"] in NEEDS_ANSWER and not e.get("resolution")]
    if open_:
        raise ValueError(f"unresolved extra requirements {open_}: answer each before the loop starts")
    if not d.get("ready"):
        raise ValueError("intake is not ready")


def new_run_dir(root):
    root = Path(root)
    while True:
        path = root / f"intake-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')}"
        try:
            path.mkdir(parents=True, exist_ok=False)
            return path
        except FileExistsError:
            continue


def write_intake(d, runs="runs"):
    """Copy the images and write intake.json to a new runs/intake-<UTC> directory; returns it."""
    text = json.dumps(d, indent=2, allow_nan=False)  # check before making the directory
    out = new_run_dir(runs)
    (out / "concept").mkdir()
    d = json.loads(text)
    for k, img in enumerate(d["concept_images"]):
        dest = out / "concept" / f"{k + 1}-{img['name']}"
        shutil.copyfile(img["source"], dest)
        img["path"] = str(dest.relative_to(out))
    (out / "intake.json").write_text(json.dumps(d, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return out


def load_intake(path):
    """A ready intake.json, with image paths made absolute."""
    path = Path(path)
    d = json.loads(path.read_text(encoding="utf-8"))
    validate_intake(d)
    for img in d["concept_images"]:
        img["abs_path"] = str((path.parent / img["path"]).resolve())
    return d


def make_runner(args):
    from brixzle.agents.runner import ClaudeRunner, SpendLedger

    return ClaudeRunner(args.model, SpendLedger(args.ledger, args.budget_usd))


def _ask_terminal(item):
    print(f"\n{item['id']} ({item['kind']}, against {', '.join(item['against']) or 'none'}): {item['text']}")
    print(f"  {item['explanation']}")
    print(f"  {item['question']}")
    while True:
        choice = input(f"  choose one of {', '.join(CHOICES)}: ").strip()
        if choice in CHOICES:
            text = input("  your words: ").strip() if choice == "clarify" else ""
            return choice, text


def parser():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--image", action="append", required=True, help="a concept image (repeat for several)")
    ap.add_argument("--extra", action="append", default=[], help="an extra requirement (repeat for several)")
    ap.add_argument("--resolve", action="append", default=[], metavar="ID=CHOICE[:TEXT]",
                    help=f"answer a conflict or unclear item; CHOICE is one of {', '.join(CHOICES)}")
    ap.add_argument("--fix-U", type=float, default=None, help="fix the voxel pitch U in mm (R2)")
    ap.add_argument("--notes", default="")
    ap.add_argument("--no-input", action="store_true", help="never prompt; unresolved conflicts are an error")
    ap.add_argument("--model", default="claude-sonnet-5-5", help="the classifier's model")
    ap.add_argument("--ledger", default="runs/api-spend.jsonl", help="spend ledger shared with the loop")
    ap.add_argument("--budget-usd", type=float, default=1.0, help="spend cap for the ledger")
    ap.add_argument("--runs", default="runs")
    return ap


def main(argv=None):
    args = parser().parse_args(argv)
    answers = {}
    for r in args.resolve:
        rid, _, rest = r.partition("=")
        choice, _, text = rest.partition(":")
        answers[rid.strip()] = (choice.strip(), text)
    extras = [e for e in args.extra if e.strip()]
    runner = make_runner(args) if extras else None
    ask = None if args.no_input or not sys.stdin.isatty() else _ask_terminal
    d = build_intake(args.image, extras, runner=runner, answers=answers, ask=ask, fixed_U_mm=args.fix_U,
                     notes=args.notes)
    if not d["ready"]:
        raise SystemExit(f"unresolved extra requirements {d['open']}: answer each with --resolve ID=CHOICE "
                         f"(one of {', '.join(CHOICES)}) or run in a terminal")
    out = write_intake(d, args.runs)
    for e in d["extras"]:
        res = f" -> {e['resolution']['choice']}" if e["resolution"] else ""
        print(f"{e['id']} {e['kind']}{res}: {e['text']}")
    print(f"intake ready -> {out}")


if __name__ == "__main__":
    main()
