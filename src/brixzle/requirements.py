"""Load and validate the base requirements (R1–R13) from docs/requirements.yaml."""

from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_PATH = Path(__file__).resolve().parents[2] / "docs" / "requirements.yaml"
REQUIRED_IDS = tuple(f"R{n}" for n in range(1, 14))
CHECK_KINDS = frozenset({"rule", "sim", "design", "predictor"})
FIELDS = ("id", "name", "text", "checks")
OPTIONAL_FIELDS = ("status",)
# 'deferred': kept in the list, but the loop does not score it and the design agent does not
# design for it yet (R12, issue #46). The reason is in docs/requirements.md.
STATUSES = ("active", "deferred")


class RequirementsError(ValueError):
    """The requirements file is malformed or incomplete."""


@dataclass(frozen=True)
class Requirement:
    id: str
    name: str
    text: str
    checks: tuple[str, ...]
    status: str = "active"


def load_requirements(path: Path | str = DEFAULT_PATH) -> list[Requirement]:
    """Read a requirements file and return its entries in file order.

    Raises RequirementsError if the file is malformed, an entry is missing a
    field or has an extra one, an id is missing, duplicated or unknown, a
    check kind is unknown, repeated or absent, or a status is unknown.
    """
    path = Path(path)
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict) or "requirements" not in data:
        raise RequirementsError(f"{path}: expected a mapping with a 'requirements' key")
    entries = data["requirements"]
    if not isinstance(entries, list):
        raise RequirementsError(f"{path}: 'requirements' must be a list")

    requirements = [_parse_entry(entry, index) for index, entry in enumerate(entries)]

    seen: set[str] = set()
    for r in requirements:
        if r.id in seen:
            raise RequirementsError(f"duplicate id: {r.id}")
        seen.add(r.id)
        if r.id not in REQUIRED_IDS:
            raise RequirementsError(f"unknown id: {r.id} (expected R1–R13)")

    missing = [rid for rid in REQUIRED_IDS if rid not in seen]
    if missing:
        raise RequirementsError(f"missing ids: {', '.join(missing)}")

    return requirements


def _parse_entry(entry: object, index: int) -> Requirement:
    where = f"entry {index}"
    if not isinstance(entry, dict):
        raise RequirementsError(f"{where}: expected a mapping, got {type(entry).__name__}")

    extra = sorted(str(key) for key in entry if key not in FIELDS + OPTIONAL_FIELDS)
    if extra:
        raise RequirementsError(f"{where}: unexpected field(s): {', '.join(extra)}")
    for field in FIELDS:
        if field not in entry:
            raise RequirementsError(f"{where}: missing field '{field}'")

    for field in ("id", "name", "text"):
        value = entry[field]
        if not isinstance(value, str) or not value.strip():
            raise RequirementsError(f"{where}: '{field}' must be a non-empty string")
    rid = entry["id"]
    where = f"{rid} ({where})"

    checks = entry["checks"]
    if not isinstance(checks, list) or not checks:
        raise RequirementsError(f"{where}: 'checks' must be a non-empty list")
    for kind in checks:
        if kind not in CHECK_KINDS:
            raise RequirementsError(
                f"{where}: unknown check kind {kind!r} (expected one of {sorted(CHECK_KINDS)})"
            )
    repeated = sorted({kind for kind in checks if checks.count(kind) > 1})
    if repeated:
        raise RequirementsError(f"{where}: repeated check kind(s): {', '.join(repeated)}")

    status = entry.get("status", "active")
    if status not in STATUSES:
        raise RequirementsError(f"{where}: unknown status {status!r} (expected one of {list(STATUSES)})")

    return Requirement(id=rid, name=entry["name"], text=entry["text"], checks=tuple(checks), status=status)
