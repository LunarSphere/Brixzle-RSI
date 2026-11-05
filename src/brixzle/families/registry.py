"""Registry of brick families: every brixzle.families.<pkg>.family module that exports FAMILY.

Discovery means a new family (for example one the design agent writes) needs no edit here. A
package without a ``family`` module, such as ``ab`` until it adopts the interface in #6, is skipped.
"""

import importlib
import importlib.util
import pkgutil

import brixzle.families as families_pkg
from brixzle.families.interface import Family


def collect(families) -> dict[str, Family]:
    out = {}
    for f in families:
        if not isinstance(f, Family):
            raise TypeError(f"expected a Family, got {type(f).__name__}")
        if f.name in out:
            raise ValueError(f"duplicate family name {f.name!r}")
        out[f.name] = f
    return out


def discover() -> dict[str, Family]:
    found = []
    for info in sorted(pkgutil.iter_modules(families_pkg.__path__), key=lambda i: i.name):
        if not info.ispkg:
            continue
        module_name = f"{families_pkg.__name__}.{info.name}.family"
        if importlib.util.find_spec(module_name) is None:
            continue
        module = importlib.import_module(module_name)
        if hasattr(module, "FAMILY"):
            found.append(module.FAMILY)
    return collect(found)


def names() -> list[str]:
    return sorted(discover())


def get(name: str) -> Family:
    registry = discover()
    if name not in registry:
        raise KeyError(f"no family {name!r}; known: {sorted(registry)}")
    return registry[name]
