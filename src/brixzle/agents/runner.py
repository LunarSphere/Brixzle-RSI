"""The one place the loop talks to Claude (issues #17, #18, #19): the Agent SDK runner, the spend
ledger and a scripted runner for tests.

- **Model:** the caller passes it. The loop sets it once, in its config (Claude Sonnet 5.5).
- **Credentials:** ``ANTHROPIC_API_KEY`` (or ``ANTHROPIC_AUTH_TOKEN``) if it is set; otherwise the
  short-lived access token of the active ``ant`` CLI profile (``ant auth login``), handed to the
  Agent SDK's Claude Code process as ``ANTHROPIC_AUTH_TOKEN``. The token is never printed or
  written. A call fails if Claude Code reports using some other credential (a claude.ai login,
  say), so spend always lands on the API account the ledger tracks.
- **Spend:** every call reserves its own cap against a ledger (a JSON-lines file) before it
  starts, is given the Agent SDK's per-call cap (``max_budget_usd``, never more than the ledger
  has left), and is recorded after it ends with the cost the SDK reports. ``BudgetExceeded``
  stops a run once the next call could pass the cap.
- **Tools:** a call lists the built-in tools it may use. ``ToolGuard`` refuses any other tool,
  and any write outside the call's ``write_root`` or read outside its ``read_roots``. No settings
  files, CLAUDE.md or skills are loaded (``setting_sources=[]``).
- **Tests** use ``ScriptedRunner``. No test calls the API, and the Agent SDK is imported only when
  ``ClaudeRunner`` runs (it is the optional ``agents`` extra: ``uv sync --extra agents``).
"""

import asyncio
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
from typing import Any, Callable

# Credentials Claude Code may report (SystemMessage init 'apiKeySource') for the spend to be on
# the API account. Anything else, such as a claude.ai login, is refused.
ALLOWED_SOURCES = ("ANTHROPIC_API_KEY", "ANTHROPIC_AUTH_TOKEN", "apiKeyHelper")
OAUTH_BETA = "anthropic-beta: oauth-2025-04-20"  # an OAuth access token needs this header
WRITE_TOOLS = ("Write", "Edit", "MultiEdit", "NotebookEdit")
READ_TOOLS = ("Read", "Glob", "Grep", "LS")
MCP_PREFIX = "mcp__brixzle__"  # the loop's own tools (create_sdk_mcp_server name 'brixzle')
SET_KEY_HELP = ("set ANTHROPIC_API_KEY to an API key, or log in with the ant CLI (`ant auth login`) so its "
                "active profile can be used")


class BudgetExceeded(RuntimeError):
    """The next call could take the spend past the cap."""


class AuthError(RuntimeError):
    """No usable API credential, or Claude Code used one the ledger does not track."""


@dataclass(frozen=True)
class ToolSpec:
    """One of the loop's own tools, served to the agent as ``mcp__brixzle__<name>``. ``handler``
    takes the tool's arguments and returns text; it runs in this process."""

    name: str
    description: str
    input_schema: dict      # {argument: type name} or a JSON schema; {} for no arguments
    handler: Callable[[dict], str]

    def respond(self, args):
        """The MCP answer: the handler's text, or the error it raised (the agent sees it)."""
        try:
            return {"content": [{"type": "text", "text": str(self.handler(args or {}))}]}
        except Exception as e:  # noqa: BLE001 - reported to the agent, not raised
            return {"content": [{"type": "text", "text": f"{type(e).__name__}: {e}"}], "is_error": True}


@dataclass(frozen=True)
class AgentCall:
    """What an agent asks Claude to do."""

    name: str                       # 'design', 'critic', 'intake'
    system: str
    prompt: str
    max_budget_usd: float           # the most this call may cost
    tools: tuple = ()               # built-in tools it may use (Read, Write, Edit, Glob, Grep)
    cwd: Path | None = None
    write_root: Path | None = None  # writes are allowed only under here
    read_roots: tuple = ()          # reads are allowed only under these (and write_root)
    mcp_tools: tuple = ()           # ToolSpec objects, served as mcp__brixzle__<name>
    output_schema: dict | None = None  # JSON schema of a structured answer
    max_turns: int = 10
    effort: str | None = None


@dataclass
class AgentResult:
    text: str = ""
    structured: Any = None
    cost_usd: float | None = 0.0
    turns: int = 0
    usage: dict = field(default_factory=dict)
    api_key_source: str | None = None
    session_id: str | None = None
    is_error: bool = False
    errors: list = field(default_factory=list)
    seconds: float = 0.0


# --- the spend ledger ---


class SpendLedger:
    """Real spend, one JSON line per call, against a cap in US dollars. Several runs (wiring
    checks, a smoke run) can share one ledger file so their total stays under one cap."""

    def __init__(self, path, cap_usd):
        if isinstance(cap_usd, bool) or not isinstance(cap_usd, (int, float)) or not math.isfinite(cap_usd) \
                or cap_usd <= 0:
            raise ValueError(f"the spend cap must be a finite number of dollars > 0, got {cap_usd!r}")
        self.path, self.cap_usd = Path(path), float(cap_usd)

    def rows(self):
        if not self.path.exists():
            return []
        return [json.loads(line) for line in self.path.read_text(encoding="utf-8").splitlines() if line.strip()]

    def spent(self):
        return float(sum(r["cost_usd"] for r in self.rows()))

    def remaining(self):
        return max(0.0, self.cap_usd - self.spent())

    def reserve(self, budget_usd):
        """Raise BudgetExceeded unless a call costing up to ``budget_usd`` fits under the cap."""
        if self.spent() + budget_usd > self.cap_usd + 1e-12:
            raise BudgetExceeded(f"a call of up to ${budget_usd:.2f} could pass the cap: ${self.spent():.4f} spent "
                                 f"of ${self.cap_usd:.2f} ({self.path})")

    def record(self, name, model, result: AgentResult, budget_usd=None):
        """Add a finished call. A call whose cost the SDK did not report counts as its whole budget."""
        cost = result.cost_usd
        if cost is None or not math.isfinite(cost):
            cost = float(budget_usd or 0.0)
        row = {"time": datetime.now(timezone.utc).isoformat(), "name": name, "model": model,
               "cost_usd": float(cost), "reported": result.cost_usd is not None, "turns": result.turns,
               "usage": {k: v for k, v in (result.usage or {}).items() if isinstance(v, (int, float))},
               "api_key_source": result.api_key_source, "is_error": result.is_error}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.path, "a", encoding="utf-8") as f:
            f.write(json.dumps(row, allow_nan=False) + "\n")
        return row


# --- credentials ---


def ant_token():
    """The active ant CLI profile's access token, refreshed if it is near expiry. Never printed."""
    out = subprocess.run(["ant", "auth", "print-credentials", "--access-token"], capture_output=True, text=True,
                         timeout=60, check=True)
    token = out.stdout.strip()
    if not token:
        raise RuntimeError("ant printed no access token")
    return token


def credential_env(environ=None, ant_token: Callable[[], str] = ant_token):
    """Environment entries that give Claude Code an API credential: none if one is already set,
    else the ant profile's token. Raises AuthError, saying what to set, if there is none."""
    environ = os.environ if environ is None else environ
    if environ.get("ANTHROPIC_API_KEY") or environ.get("ANTHROPIC_AUTH_TOKEN"):
        return {}
    try:
        token = ant_token()
    except (OSError, subprocess.SubprocessError, RuntimeError) as e:
        raise AuthError(f"no API credential: ANTHROPIC_API_KEY is not set and the ant CLI gave no token "
                        f"({type(e).__name__}). To fix: {SET_KEY_HELP}.") from None
    return {"ANTHROPIC_AUTH_TOKEN": token, "ANTHROPIC_CUSTOM_HEADERS": OAUTH_BETA}


def check_source(source):
    """Raise AuthError unless Claude Code reported a credential the ledger tracks."""
    if source not in ALLOWED_SOURCES:
        raise AuthError(f"Claude Code used the credential {source!r}, not an API credential "
                        f"({', '.join(ALLOWED_SOURCES)}); refusing so spend is not billed elsewhere. "
                        f"To fix: {SET_KEY_HELP}.")


# --- the tool guard ---


def _inside(path, root):
    try:
        Path(os.path.realpath(path)).relative_to(os.path.realpath(root))
        return True
    except ValueError:
        return False


@dataclass(frozen=True)
class ToolGuard:
    """Which tool uses a call allows: its listed built-in tools, the loop's own MCP tools, writes
    under ``write_root`` and reads under ``read_roots`` (or ``write_root``)."""

    write_root: Path | None
    read_roots: tuple = ()
    tools: tuple = READ_TOOLS + WRITE_TOOLS

    def decide(self, name, args):
        """(allowed, reason)."""
        if name.startswith(MCP_PREFIX):
            return True, ""
        if name not in self.tools:
            return False, f"the tool {name} is not available to this agent"
        path = args.get("file_path") or args.get("path") or args.get("notebook_path")
        if name in WRITE_TOOLS:
            if self.write_root is None or not path or not _inside(path, self.write_root):
                return False, f"writes are allowed only under {self.write_root}"
            return True, ""
        if path is None:  # Glob or Grep with no path searches the working directory
            return True, ""
        roots = [*self.read_roots, *((self.write_root,) if self.write_root else ())]
        if not any(_inside(path, r) for r in roots):
            return False, f"reads are allowed only under {', '.join(str(r) for r in roots)}"
        return True, ""


def option_kwargs(call: AgentCall, model, remaining_usd, env):
    """ClaudeAgentOptions keyword arguments for a call (no SDK import, so tests can check them)."""
    kw = {"model": model, "system_prompt": call.system, "tools": list(call.tools),
          "allowed_tools": [*call.tools, *(f"{MCP_PREFIX}{t.name}" for t in call.mcp_tools)],
          "max_turns": call.max_turns, "max_budget_usd": min(call.max_budget_usd, remaining_usd),
          "setting_sources": [], "env": dict(env), "cwd": str(call.cwd) if call.cwd else None}
    if call.output_schema is not None:
        kw["output_format"] = {"type": "json_schema", "schema": call.output_schema}
    if call.effort:
        kw["effort"] = call.effort
    return kw


# --- runners ---


class ClaudeRunner:
    """Runs calls through the Claude Agent SDK with ``model``, checked against ``ledger``."""

    def __init__(self, model, ledger: SpendLedger, environ=None):
        self.model, self.ledger, self.environ = model, ledger, environ

    def run(self, call: AgentCall) -> AgentResult:
        self.ledger.reserve(call.max_budget_usd)
        env = credential_env(self.environ)
        result = asyncio.run(self._run(call, env))
        self.ledger.record(call.name, self.model, result, budget_usd=call.max_budget_usd)
        check_source(result.api_key_source)
        return result

    async def _run(self, call, env):
        import time

        from claude_agent_sdk import (AssistantMessage, ClaudeAgentOptions, PermissionResultAllow,
                                      PermissionResultDeny, ResultMessage, SystemMessage, TextBlock,
                                      create_sdk_mcp_server, query, tool)

        guard = ToolGuard(call.write_root, tuple(call.read_roots), tuple(call.tools))

        async def can_use_tool(name, args, _context):
            ok, why = guard.decide(name, args)
            return PermissionResultAllow() if ok else PermissionResultDeny(message=why)

        kw = option_kwargs(call, self.model, self.ledger.remaining(), env)
        if call.mcp_tools:
            def serve(spec):
                async def run(args):
                    return await asyncio.to_thread(spec.respond, args)
                return tool(spec.name, spec.description, spec.input_schema)(run)

            kw["mcp_servers"] = {"brixzle": create_sdk_mcp_server("brixzle", tools=[serve(t) for t in call.mcp_tools])}
        options = ClaudeAgentOptions(**kw, can_use_tool=can_use_tool, permission_mode="default")

        async def prompt():  # can_use_tool needs the streaming form of the prompt
            yield {"type": "user", "message": {"role": "user", "content": call.prompt}}

        out, texts, t0 = AgentResult(cost_usd=None), [], time.time()
        async for msg in query(prompt=prompt(), options=options):
            if isinstance(msg, SystemMessage) and msg.subtype == "init":
                out.api_key_source = msg.data.get("apiKeySource")
            elif isinstance(msg, AssistantMessage):
                texts += [b.text for b in msg.content if isinstance(b, TextBlock)]
            elif isinstance(msg, ResultMessage):
                out.cost_usd, out.turns, out.usage = msg.total_cost_usd, msg.num_turns, msg.usage or {}
                out.session_id, out.is_error = msg.session_id, bool(msg.is_error)
                out.structured, out.errors = msg.structured_output, list(msg.errors or [])
                if msg.result:
                    texts.append(msg.result)
        out.text, out.seconds = (texts[-1] if texts else ""), time.time() - t0
        return out


class ScriptedRunner:
    """A runner for tests: answers calls in order from ``script``, each an AgentResult or a
    function call -> AgentResult (to act like the agent, for example by writing files)."""

    def __init__(self, script, ledger: SpendLedger | None = None, model="scripted"):
        self.script, self.ledger, self.model, self.calls = list(script), ledger, model, []

    def run(self, call: AgentCall) -> AgentResult:
        assert self.script, f"no scripted answer left for the {call.name} call"
        if self.ledger is not None:
            self.ledger.reserve(call.max_budget_usd)
        self.calls.append(call)
        step = self.script.pop(0)
        result = step(call) if callable(step) else step
        if self.ledger is not None:
            self.ledger.record(call.name, self.model, result, budget_usd=call.max_budget_usd)
        return result
