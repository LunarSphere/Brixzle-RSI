"""The bounds the loop's agents work inside (#165).

While a loop runs, ``LOCK`` (written by run.init_run at the repo root) names its run directory. Then:

- **Writes go only inside the run directory** (and the system's temporary directories). So no
  agent can edit ``docs/requirements.md``, ``src/brixzle/rules/``, the scorer, the sim, the caches,
  the agents' own instructions under ``.claude/`` or the lock itself. ``hook`` enforces it as a
  Claude Code PreToolUse hook on Write, Edit, MultiEdit and NotebookEdit, and refuses a Bash command
  that redirects or copies into a path outside the run. Codex (``.codex/hooks.json``) sends the same
  events, with its edits as ``apply_patch`` (the patch text in ``tool_input.command``, paths relative
  to the event's ``cwd``); every path a patch adds, updates, deletes or moves to must be inside the run
  (#231).
- **Reads** of the lock's ``deny_read`` paths are refused (a replay hides the designs it replays).
- **The tracked tree must not change** (``changed_tracked``): the orchestrator checks after every
  iteration, and a change ends the run. This also catches anything the hook could not see.

Without a lock the hook allows everything, so normal development is unaffected.
"""
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile

LOCK = ".rsi-loop-active"
WRITE_TOOLS = ("Write", "Edit", "MultiEdit", "NotebookEdit")
READ_TOOLS = ("Read", "Grep", "Glob")
TEMP_ROOTS = (Path(tempfile.gettempdir()).resolve(), Path("/tmp").resolve(), Path("/private/tmp"))
# Bash: writes into a path (redirection, tee, cp/mv/rm/touch/sed -i target)
# Codex's apply_patch: the files a patch touches (#231)
_PATCH_PATH = re.compile(r"^\*\*\* (?:Add File|Update File|Delete File|Move to): (.+?)\s*$", re.M)
_BASH_WRITE = re.compile(r"(?:>>?|\btee\b(?:\s+-a)?|\bcp\b|\bmv\b|\brm\b|\btouch\b|\bsed\s+-i\S*|\bmkdir\b)\s+([^\s;&|]+)")


def find_lock(start):
    for d in [Path(start).resolve(), *Path(start).resolve().parents]:
        if (d / LOCK).exists():
            return d / LOCK
    return None


def _inside(path, root):
    try:
        Path(path).resolve().relative_to(Path(root).resolve())
        return True
    except ValueError:
        return False


def write_allowed(path, lock, repo):
    p = Path(path)
    p = p if p.is_absolute() else Path(repo) / p
    if p.resolve() == (Path(repo) / LOCK).resolve():
        return False
    if _inside(p, lock["run"]):
        return True
    if _inside(p, repo):  # inside the checkout only the run directory is writable, wherever the checkout is
        return False
    return any(_inside(p, t) for t in TEMP_ROOTS)


def read_allowed(path, lock, repo):
    p = Path(path)
    p = p if p.is_absolute() else Path(repo) / p
    return not any(_inside(p, Path(repo) / d) for d in lock.get("deny_read", ()))


def decide(event):
    """(allowed, reason) for one PreToolUse event."""
    cwd = event.get("cwd") or "."
    lock_path = find_lock(cwd)
    if lock_path is None:
        return True, ""
    lock, repo = json.loads(lock_path.read_text()), lock_path.parent
    tool, inp = event.get("tool_name", ""), event.get("tool_input") or {}
    if tool in WRITE_TOOLS:
        path = inp.get("file_path") or inp.get("notebook_path") or ""
        if not write_allowed(path, lock, repo):
            return False, f"the loop's agents write only inside the run directory {lock['run']}; {path} is outside the run"
    if tool == "apply_patch":
        for target in _PATCH_PATH.findall(inp.get("command", "") or inp.get("patch", "")):
            path = target if Path(target).is_absolute() else Path(cwd) / target
            if not write_allowed(path, lock, repo):
                return False, f"the loop's agents write only inside the run directory {lock['run']}; {target} is outside the run"
    if tool in READ_TOOLS:
        path = inp.get("file_path") or inp.get("path") or ""
        if path and not read_allowed(path, lock, repo):
            return False, f"{path} is hidden from this run (deny_read in {LOCK})"
    if tool == "Bash":
        cmd = inp.get("command", "")
        for target in _BASH_WRITE.findall(cmd):
            if target.startswith("-") or target.startswith("/dev/"):
                continue
            if not write_allowed(Path(cwd) / target if not target.startswith("/") else target, lock, repo):
                return False, f"this command writes {target}, outside the run directory {lock['run']}"
        for d in lock.get("deny_read", ()):
            if d in cmd:
                return False, f"{d} is hidden from this run (deny_read in {LOCK})"
    return True, ""


def changed_tracked(run):
    """Tracked or untracked files (not git-ignored) of the loop's checkout that changed since the run began.

    The checkout and its status at the start are read from the run's ``run.json`` (#179), because ``runs/``
    may be a link into another checkout. An older run without it falls back to the lock found from the run
    or the working directory; with neither, it raises rather than compare some other repo."""
    run = Path(run)
    where = run / "run.json"
    if where.exists():
        info = json.loads(where.read_text())
        repo, status = Path(info["repo"]), info["status"]
    else:
        lock_path = find_lock(run) or find_lock(Path.cwd())
        if lock_path is None:
            raise FileNotFoundError(f"{run} has no run.json and no loop lock was found: cannot tell which checkout to check")
        lock = json.loads(lock_path.read_text())
        repo, status = Path(lock["repo"]), lock["status"]
    before = {l[3:] for l in status.splitlines() if l}
    out = subprocess.run(["git", "-C", str(repo), "status", "--porcelain"], capture_output=True, text=True, check=True).stdout
    now = {l[3:] for l in out.splitlines() if l}
    return sorted(f for f in now - before if f != LOCK)


def hook():
    event = json.loads(sys.stdin.read() or "{}")
    ok, reason = decide(event)
    if not ok:
        print(reason, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    if sys.argv[1:] == ["hook"]:
        sys.exit(hook())
    sys.exit("usage: python -m brixzle.director.guard hook  (reads a PreToolUse event on stdin)")
