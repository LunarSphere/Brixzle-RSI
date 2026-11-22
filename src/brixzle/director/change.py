"""One change per iteration (#165): what a candidate module changed from its parent, by name.

A module is compared definition by definition (``ast``), docstrings and comments ignored:

- ``param:<field>`` for each parameter whose default changed, or that was added or removed (fields
  of any ``@dataclass`` in the module);
- ``def:<name>`` for each function or class (other than a params dataclass) whose body changed, or
  that was added or removed;
- ``const:<name>`` for each module-level assignment that changed.

The family's ``name``, ``version`` and ``description`` may always change. The director's decision
lists what the iteration may change (``may_change``); ``check`` refuses a candidate that changed
anything else, so the rest stays frozen.
"""
import ast
import json
from pathlib import Path

FREE = {"name", "version", "description"}


def _strip_docs(node):
    for n in ast.walk(node):
        body = getattr(n, "body", None)
        if isinstance(body, list) and body and isinstance(body[0], ast.Expr) and isinstance(getattr(body[0], "value", None), ast.Constant) \
                and isinstance(body[0].value.value, str):
            n.body = body[1:] or [ast.Pass()]
    return node


def _is_dataclass(node):
    return isinstance(node, ast.ClassDef) and any("dataclass" in ast.dump(d) for d in node.decorator_list)


def units(path):
    """{unit name: normalised source} of one module."""
    tree = ast.parse(Path(path).read_text())
    out = {}
    for n in tree.body:
        if _is_dataclass(n):
            for f in n.body:
                if isinstance(f, ast.AnnAssign) and isinstance(f.target, ast.Name):
                    out[f"param:{f.target.id}"] = ast.dump(f.value) if f.value is not None else ""
            rest = [f for f in n.body if not isinstance(f, ast.AnnAssign)]
            if rest:
                out[f"def:{n.name}"] = ast.dump(_strip_docs(ast.Module(body=rest, type_ignores=[])))
        elif isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            if isinstance(n, ast.ClassDef):
                n = _strip_docs(n)
                n.body = [b for b in n.body if not (isinstance(b, ast.Assign) and len(b.targets) == 1
                                                    and isinstance(b.targets[0], ast.Name) and b.targets[0].id in FREE)] or [ast.Pass()]
            out[f"def:{n.name}"] = ast.dump(_strip_docs(n))
        elif isinstance(n, (ast.Assign, ast.AnnAssign)):
            targets = n.targets if isinstance(n, ast.Assign) else [n.target]
            names = [t.id for t in targets if isinstance(t, ast.Name)]
            for name in names:
                if name not in FREE:
                    out[f"const:{name}"] = ast.dump(n.value) if n.value is not None else ""
    sidecar = Path(path).with_name("gripper.json")
    if sidecar.exists():
        for key, value in json.loads(sidecar.read_text()).items():
            if key == "wrist":
                out.update({f"gripper:wrist.{axis}": setting for axis, setting in value.items()})
            else:
                out[f"gripper:{key}"] = value
    return out


def changed(parent, candidate):
    a, b = units(parent), units(candidate)
    return sorted(k for k in set(a) | set(b) if a.get(k) != b.get(k))


def check(parent, candidate, may_change):
    ch = changed(parent, candidate)
    outside = [c for c in ch if c not in set(may_change)]
    return {"ok": not outside and bool(ch), "changed": ch, "may_change": list(may_change), "outside": outside,
            "reason": ("no change" if not ch else f"changed outside the brief: {outside}" if outside else "")}
