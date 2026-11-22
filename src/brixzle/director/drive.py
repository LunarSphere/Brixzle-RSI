"""The self-steering loop driven from Python, with each role a Codex CLI call (#182).

    uv run python -m brixzle.director.drive --run runs/rsi/<name> --new "<goal>" --baseline <spec>
        [--iterations N] [--minutes M] [--roles roles.json]
    uv run python -m brixzle.director.drive --run runs/rsi/<name>          # resume

The same steps as ``.claude/commands/rsi-loop.md``, with no Claude: the deterministic ones are this
package's CLI (``python -m brixzle.director``), and each AI role (designer, verifier, critic,
director) is ``codex exec`` with its own model and reasoning effort (``load_roles``). A role's
instructions are its ``.claude/agents/rsi-<role>.md``, the one source for both drivers.

**Bounds.** A role's writable folder is the run (``-C <run>``, plus the shared ``runs/`` the tools
write to and uv's cache), so the checkout (requirements, rules, scorer, code) is read-only to it; the
drop, stage and matplotlib caches are pointed into the run (``role_env``). The post-iteration
``guard`` still checks the checkout. Approvals go to Codex's automatic review (``--approve-for-me``):
the account's managed requirements forbid ``approval_policy = never``, and exec mode cannot ask a
person. The sandbox is never bypassed.

Every role call is logged to ``iter-NN/<role>.jsonl`` with its final message, and every step's
wall-clock goes to ``timeline.jsonl``.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROLE_DEFAULTS = {
    "designer": {"model": "gpt-6.2-astra", "effort": "high"},
    "director": {"model": "gpt-6.2-astra", "effort": "high"},
    "critic": {"model": "gpt-6.2-astra", "effort": "medium"},
    "verifier": {"model": "gpt-6.2-sol", "effort": "medium"},
}
EFFORTS = ("minimal", "low", "medium", "high", "xhigh")
UV_CACHE = Path.home() / ".cache" / "uv"
ROLE_TIMEOUT_S = 3600
OUTPUTS = {"designer": "a package <module>/ with family.py and __init__.py, design_note.md and claim.json",
           "verifier": "audit.md", "critic": "diagnosis.md",
           "director": "decision.json, and the run's STATE.md rewritten in full"}


def check_roles(roles):
    for name, cfg in roles.items():
        if name not in ROLE_DEFAULTS:
            raise ValueError(f"unknown role {name!r}")
        if not cfg.get("model") or cfg.get("effort") not in EFFORTS:
            raise ValueError(f"role {name}: needs a model and an effort in {EFFORTS}, got {cfg}")
    return roles


def load_roles(path=None):
    """The roles' models and efforts: the defaults, overridden by a JSON file {role: {model, effort}}."""
    roles = {k: dict(v) for k, v in ROLE_DEFAULTS.items()}
    if path is not None:
        for k, v in json.loads(Path(path).read_text()).items():
            roles.setdefault(k, {}).update(v)
    return check_roles(roles)


def role_instructions(checkout, role):
    text = (Path(checkout) / ".claude" / "agents" / f"rsi-{role}.md").read_text()
    return text.split("---", 2)[2].strip() if text.startswith("---") else text.strip()


def role_prompt(checkout, role, run, iteration, refusal=None):
    run, checkout = Path(run), Path(checkout)
    d = run / f"iter-{iteration:02d}"
    inputs = {
        "designer": f"Your brief: {d}/brief.json. Write in {d}/.",
        "verifier": f"The iteration: {d}/ (design_note.md, verified.json). Write {d}/audit.md.",
        "critic": f"The iteration: {d}/ (verified.json). Write {d}/diagnosis.md.",
        "director": (f"The run: {run}/ (goal.json, STATE.md, archive.json, decisions.jsonl; the iteration just evaluated is "
                     f"{d}/, or {run}/baseline/ at iteration 0). Write {d}/decision.json and rewrite {run}/STATE.md."),
    }[role]
    out = [role_instructions(checkout, role), "",
           "## How this run works",
           f"- You run non-interactively (Codex exec). The run directory is {run}; the repository checkout is {checkout}, "
           "and it is read-only to you: write only inside the run directory.",
           f"- Run the repo's commands from the checkout, without syncing it: `cd {checkout} && uv run --no-sync python -m ...`.",
           "  Each command may take minutes; wait for it. Never start one in the background.",
           f"- {inputs}",
           f"- When you are done, your files must exist: {OUTPUTS[role]}. Reply with a short summary of what you wrote."]
    if refusal:
        out += ["", "## Your last attempt was refused", refusal]
    return "\n".join(out)


def role_env(run):
    run = Path(run)
    env = dict(os.environ)
    env.update(BRIXZLE_DROP_CACHE=str(run / "drops.sqlite"), BRIXZLE_STAGE_CACHE=str(run / "stage-cache"),
               MPLCONFIGDIR=str(run / "mplcache"))
    return env


def codex_command(cfg, workdir, add_dirs, out_file, prompt):
    cmd = ["codex", "exec", "--approve-for-me", "-m", cfg["model"], "-c", f'model_reasoning_effort="{cfg["effort"]}"',
           "--skip-git-repo-check", "-C", str(workdir)]
    for a in add_dirs:
        cmd += ["--add-dir", str(a)]
    return cmd + ["--json", "-o", str(out_file), prompt]


def now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


class Driver:
    """Runs the loop. ``call_cli(args) -> (returncode, json)`` runs ``python -m brixzle.director``;
    ``call_role(role, iteration, prompt) -> returncode`` runs one role; ``budget() -> status['budget']``."""

    def __init__(self, run, checkout, call_cli=None, call_role=None, budget=None, roles=None):
        self.run_dir, self.checkout = Path(run), Path(checkout)
        self.roles = roles or load_roles()
        self.call_cli = call_cli or self._cli
        self.call_role = call_role or self._role
        self.budget = budget or (lambda: self.call_cli(["status", "--run", str(self.run_dir)])[1].get("budget", {}))
        self.reason = ""

    # --- the real calls ---

    def _cli(self, args):
        p = subprocess.run([sys.executable, "-m", "brixzle.director", *args], cwd=self.checkout, capture_output=True,
                           text=True, env=role_env(self.run_dir))
        try:
            out = json.loads(p.stdout[p.stdout.index("{"):]) if "{" in p.stdout else {}
        except ValueError:
            out = {"stdout": p.stdout[-2000:]}
        if p.returncode not in (0, 1):
            out["stderr"] = p.stderr[-2000:]
        return p.returncode, out

    def _role(self, role, iteration, prompt):
        d = self.run_dir / f"iter-{iteration:02d}"
        d.mkdir(parents=True, exist_ok=True)
        runs = self.run_dir.resolve().parents[1]  # <runs>/rsi/<name> -> <runs>, where the tools write
        cmd = codex_command(self.roles[role], self.run_dir.resolve(), [runs, UV_CACHE], d / f"{role}-final.md", prompt)
        with open(d / f"{role}.jsonl", "a") as log:
            p = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, env=role_env(self.run_dir), timeout=ROLE_TIMEOUT_S)
        return p.returncode

    # --- the loop ---

    def _timed(self, step, iteration, fn):
        t0 = time.time()
        try:
            return fn()
        finally:
            with open(self.run_dir / "timeline.jsonl", "a") as f:
                f.write(json.dumps({"step": step, "iteration": iteration, "end": now(), "seconds": round(time.time() - t0, 1),
                                    "driver": "codex", **({"model": self.roles[step]["model"],
                                                           "effort": self.roles[step]["effort"]} if step in self.roles else {})})
                        + "\n")

    def _role_step(self, role, n, refusal=None):
        prompt = role_prompt(self.checkout, role, self.run_dir, n, refusal)
        return self._timed(role, n, lambda: self.call_role(role, n, prompt))

    def _cli_step(self, step, n=None):
        args = [step, "--run", str(self.run_dir)] + (["--iter", str(n)] if n is not None else [])
        return self._timed(step, n, lambda: self.call_cli(args))

    def _direct(self, n):
        self._role_step("director", n)
        rc, out = self._cli_step("decide", n)
        if rc != 0:
            self._role_step("director", n, f"`python -m brixzle.director decide` refused your decision: {out.get('reason')}")
            rc, out = self._cli_step("decide", n)
        return rc == 0

    def _next(self):
        return json.loads((self.run_dir / "next.json").read_text())

    def run(self):
        if not (self.run_dir / "baseline" / "verified.json").exists():
            self._cli_step("baseline")
            if not self._direct(0):
                return self._finish("the director's decision after the baseline was refused twice")
        while True:
            nxt = self._next()
            if nxt.get("step") == "finish":
                return self._finish(nxt.get("reason") or "the director stopped")
            if self.budget().get("exhausted"):
                return self._finish("budget spent")
            n = int(nxt["iteration"])
            self._role_step("designer", n)
            rc, out = self._cli_step("check", n)
            if rc != 0:
                self._role_step("designer", n, f"`python -m brixzle.director check` refused it: {out.get('reason')} "
                                               f"(changed {out.get('changed')}, outside the brief {out.get('outside')}).")
                rc, out = self._cli_step("check", n)
            if rc == 0:
                self._cli_step("verify", n)
                with ThreadPoolExecutor(2) as ex:  # the audit and the critique are independent
                    list(ex.map(lambda r: self._role_step(r, n), ("verifier", "critic")))
            if not self._direct(n):
                return self._finish(f"the director's decision for iteration {n} was refused twice")
            rc, out = self._cli_step("guard")
            if rc != 0:
                return self._finish(f"tracked files changed: {out.get('tracked_changes')}")

    def _finish(self, reason):
        self.reason = reason
        self._timed("finish", None, lambda: self.call_cli(["finish", "--run", str(self.run_dir), "--reason", reason]))
        return reason


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m brixzle.director.drive")
    ap.add_argument("--run", required=True)
    ap.add_argument("--new", help="the goal: start a new run")
    ap.add_argument("--baseline", default="ab")
    ap.add_argument("--iterations", type=int, default=6)
    ap.add_argument("--minutes", type=float, default=240)
    ap.add_argument("--roles", help="JSON {role: {model, effort}} overriding the defaults")
    ap.add_argument("--checkout", default=".")
    a = ap.parse_args(argv)
    checkout = Path(a.checkout).resolve()
    if a.new:
        from brixzle.director import run as RN
        RN.init_run(Path(a.run), a.new, a.baseline, repo=checkout,
                    budget={"iterations": a.iterations, "wall_clock_min": a.minutes})
    d = Driver(Path(a.run).resolve(), checkout, roles=load_roles(a.roles))
    reason = d.run()
    print(json.dumps({"run": str(a.run), "reason": reason}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
