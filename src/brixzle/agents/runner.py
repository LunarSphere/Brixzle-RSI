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
  and any write outside the call's ``write_root`` or read outside its ``read_roots``. It runs as a
  PreToolUse hook, which sees every call: a tool in ``allowed_tools`` is approved before a
  permission callback is asked, and read-only tools never ask. No settings files, CLAUDE.md or
  skills are loaded (``setting_sources=[]``).
- **Tests** use ``ScriptedRunner``. No test calls the API, and the Agent SDK is imported only when
  ``ClaudeRunner`` runs (it is the optional ``agents`` extra: ``uv sync --extra agents``).
"""

import asyncio
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
import os
import re
from pathlib import Path
import subprocess
import threading
from typing import Any, Callable

# Credentials Claude Code may report (SystemMessage init 'apiKeySource') for the spend to be on
# the API account. Anything else, such as a claude.ai login, is refused.
ALLOWED_SOURCES = ("ANTHROPIC_API_KEY", "apiKeyHelper")
OAUTH_BETA = "anthropic-beta: oauth-2025-04-20"  # an OAuth access token needs this header
WRITE_TOOLS = ("Write", "Edit", "MultiEdit", "NotebookEdit")
READ_TOOLS = ("Read", "Glob", "Grep", "LS")
# The Agent SDK reads Claude Code's output line by line with a 1 MB limit by default. A concept photo
# read by the agent came back as a 1.15 MB message and killed the call (issue #79).
MAX_BUFFER_BYTES = 16 * 1024 * 1024
MCP_PREFIX = "mcp__brixzle__"  # the loop's own tools (create_sdk_mcp_server name 'brixzle')
# Claude Code's own tool for a structured answer (output_format): never blocked.
SDK_TOOLS = ("StructuredOutput",)
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
    session: str | None = None      # the persistent session that answered it (#125), if any
    session_id: str | None = None


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
    context_tokens: int = 0        # the last turn's input, cached or not: a session's context size (#125)


# --- the spend ledger ---


class SpendLedger:
    """Real spend, one JSON line per call, against a cap in US dollars. Several runs (wiring
    checks, a smoke run) can share one ledger file so their total stays under one cap."""

    def __init__(self, path, cap_usd):
        if isinstance(cap_usd, bool) or not isinstance(cap_usd, (int, float)) or not math.isfinite(cap_usd) \
                or cap_usd <= 0:
            raise ValueError(f"the spend cap must be a finite number of dollars > 0, got {cap_usd!r}")
        self.path, self.cap_usd = Path(path), float(cap_usd)
        # Calls in flight hold their budget until recorded, so calls made side by side (#106) cannot
        # pass the cap together.
        self._held, self._lock = 0.0, threading.Lock()

    def rows(self):
        if not self.path.exists():
            return []
        return [json.loads(line) for line in self.path.read_text(encoding="utf-8").splitlines() if line.strip()]

    def spent(self):
        return float(sum(r["cost_usd"] for r in self.rows()))

    def remaining(self):
        return max(0.0, self.cap_usd - self.spent() - self._held)

    def reserve(self, budget_usd):
        """Hold ``budget_usd`` for a call, or raise BudgetExceeded if it could pass the cap with the
        spend so far and the calls in flight. ``record`` (or ``release``) frees the hold."""
        with self._lock:
            if self.spent() + self._held + budget_usd > self.cap_usd + 1e-12:
                raise BudgetExceeded(f"a call of up to ${budget_usd:.2f} could pass the cap: ${self.spent():.4f} spent "
                                     f"and ${self._held:.2f} held by calls in flight, of ${self.cap_usd:.2f} ({self.path})")
            self._held += float(budget_usd)

    def release(self, budget_usd):
        """Free a hold made by ``reserve``."""
        with self._lock:
            self._held = max(0.0, self._held - float(budget_usd or 0.0))

    def record(self, name, model, result: AgentResult, budget_usd=None, session=None):
        """Add a finished call. A call whose cost the SDK did not report counts as its whole budget."""
        cost = result.cost_usd
        if cost is None or not math.isfinite(cost):
            cost = float(budget_usd or 0.0)
        row = {"time": datetime.now(timezone.utc).isoformat(), "name": name, "model": model,
               "cost_usd": float(cost), "reported": result.cost_usd is not None, "turns": result.turns,
               "usage": {k: v for k, v in (result.usage or {}).items() if isinstance(v, (int, float))},
               "api_key_source": result.api_key_source, "is_error": result.is_error,
               "seconds": float(result.seconds or 0.0) if math.isfinite(float(result.seconds or 0.0)) else 0.0,
               "session": session}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self._lock:
            with open(self.path, "a", encoding="utf-8") as f:
                f.write(json.dumps(row, allow_nan=False) + "\n")
        self.release(budget_usd)
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


def check_source(source, auth_token=False):
    """Raise AuthError unless Claude Code reported a credential the ledger tracks.

    Claude Code reports an API key's source. With ``ANTHROPIC_AUTH_TOKEN`` (the ant profile's
    token) it reports 'none', and says the token takes precedence over a claude.ai login; so 'none'
    is accepted only when such a token is in effect (``auth_token``)."""
    if source == "none" and auth_token:
        return
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
        if name.startswith(MCP_PREFIX) or name in SDK_TOOLS:
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


def hook_decision(guard, hook_input):
    """A PreToolUse hook's answer for one tool call: {} to let it through, or a denial with the
    guard's reason (the agent sees the reason)."""
    ok, why = guard.decide(hook_input.get("tool_name", ""), hook_input.get("tool_input") or {})
    if ok:
        return {}
    return {"hookSpecificOutput": {"hookEventName": "PreToolUse", "permissionDecision": "deny",
                                   "permissionDecisionReason": why}}


def option_kwargs(call: AgentCall, model, remaining_usd, env):
    """ClaudeAgentOptions keyword arguments for a call (no SDK import, so tests can check them)."""
    kw = {"model": model, "system_prompt": call.system, "tools": list(call.tools),
          "allowed_tools": [*call.tools, *(f"{MCP_PREFIX}{t.name}" for t in call.mcp_tools)],
          "max_turns": call.max_turns, "max_budget_usd": min(call.max_budget_usd, remaining_usd),
          "setting_sources": [], "env": dict(env), "cwd": str(call.cwd) if call.cwd else None,
          "max_buffer_size": MAX_BUFFER_BYTES}
    if call.output_schema is not None:
        kw["output_format"] = {"type": "json_schema", "schema": call.output_schema}
    if call.effort:
        kw["effort"] = call.effort
    return kw


# --- runners ---


_BUDGET_STOP = re.compile(r"Reached maximum budget \(\$([0-9]+(?:\.[0-9]+)?)\)")


def budget_stop_cost(error_text):
    """The amount the SDK reports when it stops a call at its budget, or None (#123)."""
    m = _BUDGET_STOP.search(str(error_text))
    return float(m.group(1)) if m else None


class ClaudeRunner:
    """Runs calls through the Claude Agent SDK with ``model``, checked against ``ledger``."""

    def __init__(self, model, ledger: SpendLedger, environ=None):
        self.model, self.ledger, self.environ = model, ledger, environ

    def sdk_cap(self, call):
        """The SDK's budget for a call that holds its own budget in the ledger: its budget, or what
        the ledger has left with this call's hold added back (#123: subtracting every hold gave each
        of three calls side by side only $0.42)."""
        return min(call.max_budget_usd, self.ledger.remaining() + call.max_budget_usd)

    def session(self, name, base: AgentCall, resume=None, messages=4):
        """A persistent session (#125) on this runner's model and ledger, budgeted for ``messages``."""
        return ClaudeSession(self.model, self.ledger, name, base, resume=resume, environ=self.environ,
                             messages=messages)

    def run(self, call: AgentCall) -> AgentResult:
        self.ledger.reserve(call.max_budget_usd)
        env = credential_env(self.environ)
        environ = os.environ if self.environ is None else self.environ
        token = bool(env.get("ANTHROPIC_AUTH_TOKEN") or (environ.get("ANTHROPIC_AUTH_TOKEN")
                                                          and not environ.get("ANTHROPIC_API_KEY")))
        try:
            result = asyncio.run(self._run(call, env))
        except BaseException as e:
            # What a failed call spent is unknown, so it is charged its whole budget, unless the SDK
            # stopped it at its budget and said how much that was (#123).
            err = f"{type(e).__name__}: {e}"
            self.ledger.record(call.name, self.model, AgentResult(cost_usd=budget_stop_cost(err), is_error=True,
                                                                  errors=[err]),
                               budget_usd=call.max_budget_usd)
            raise
        self.ledger.record(call.name, self.model, result, budget_usd=call.max_budget_usd)
        check_source(result.api_key_source, auth_token=token)
        return result

    async def _run(self, call, env):
        import time

        from claude_agent_sdk import (AssistantMessage, ClaudeAgentOptions, HookMatcher, ResultMessage,
                                      SystemMessage, TextBlock, create_sdk_mcp_server, query, tool)

        guard = ToolGuard(call.write_root, tuple(call.read_roots), tuple(call.tools))

        async def pre_tool_use(hook_input, _tool_use_id, _context):
            # Every tool call passes here, before allowed_tools would approve it.
            return hook_decision(guard, hook_input)

        kw = option_kwargs(call, self.model, self.sdk_cap(call), env)
        if call.mcp_tools:
            def serve(spec):
                async def run(args):
                    return await asyncio.to_thread(spec.respond, args)
                return tool(spec.name, spec.description, spec.input_schema)(run)

            kw["mcp_servers"] = {"brixzle": create_sdk_mcp_server("brixzle", tools=[serve(t) for t in call.mcp_tools])}
        options = ClaudeAgentOptions(**kw, permission_mode="default",
                                     hooks={"PreToolUse": [HookMatcher(matcher=None, hooks=[pre_tool_use])]})

        async def prompt():  # hooks need the streaming form of the prompt
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


class ClaudeSession:
    """One persistent Claude Code session (#125): connected once, then one message per task.

    The SDK's client must stay in the async context it connected in, so the session owns an event
    loop in its own thread, and ``send`` hands each message to it. The options (model, system
    prompt, tools, per-message turn and budget limits) come from the first call, ``base``; each
    message's own write root, read roots and loop tools replace the previous ones, because the tool
    guard and the tool handlers are looked up at call time. Each message is charged to the ledger
    with the session's name; the SDK reports cost per message. ``resume`` reopens a saved session."""

    def __init__(self, model, ledger: SpendLedger, name, base: AgentCall, resume=None, environ=None,
                 client_factory=None, messages=4):
        self.model, self.ledger, self.name, self.base = model, ledger, name, base
        # The SDK's budget caps the whole session, and its cost is the session's running total (#131).
        self.session_budget_usd, self.total_usd = base.max_budget_usd * max(1, int(messages)), 0.0
        self.environ, self.resume, self._factory = environ, resume, client_factory
        self.guard = ToolGuard(base.write_root, tuple(base.read_roots), tuple(base.tools))
        self.handlers = {t.name: t for t in base.mcp_tools}
        self.session_id, self.messages, self.context_tokens = resume, 0, 0
        self._loop = self._thread = self._queue = None
        self._token = False

    # --- the owner thread ---

    def _options(self, env):
        kw = option_kwargs(self.base, self.model, self.base.max_budget_usd, env)
        kw["max_budget_usd"] = self.session_budget_usd
        if self.resume:
            kw["resume"] = self.resume
        if self._factory is not None:
            return kw
        from claude_agent_sdk import ClaudeAgentOptions, HookMatcher, create_sdk_mcp_server, tool

        session = self

        async def pre_tool_use(hook_input, _tool_use_id, _context):
            return hook_decision(session.guard, hook_input)

        if self.base.mcp_tools:
            def serve(spec):
                async def run(args):
                    current = session.handlers.get(spec.name)
                    if current is None:
                        return {"content": [{"type": "text", "text": f"{spec.name} is not offered for this task"}],
                                "is_error": True}
                    return await asyncio.to_thread(current.respond, args)
                return tool(spec.name, spec.description, spec.input_schema)(run)

            kw["mcp_servers"] = {"brixzle": create_sdk_mcp_server("brixzle", tools=[serve(t) for t in self.base.mcp_tools])}
        return ClaudeAgentOptions(**kw, permission_mode="default",
                                  hooks={"PreToolUse": [HookMatcher(matcher=None, hooks=[pre_tool_use])]})

    def _client(self, opts):
        if self._factory is not None:
            return self._factory(opts)
        from claude_agent_sdk import ClaudeSDKClient

        return ClaudeSDKClient(options=opts)

    async def _owner(self, ready):
        try:
            env = credential_env(self.environ)
            environ = os.environ if self.environ is None else self.environ
            self._token = bool(env.get("ANTHROPIC_AUTH_TOKEN") or (environ.get("ANTHROPIC_AUTH_TOKEN")
                                                                  and not environ.get("ANTHROPIC_API_KEY")))
            client = self._client(self._options(env))
            self._queue = asyncio.Queue()
            async with client as c:
                ready.set_result(True)
                while True:
                    item = await self._queue.get()
                    if item is None:
                        break
                    prompt, fut = item
                    try:
                        fut.set_result(await self._ask(c, prompt))
                    except BaseException as e:  # noqa: BLE001 - handed to the caller
                        fut.set_exception(e)
        except BaseException as e:  # noqa: BLE001 - a failed connect is the first send's error
            if not ready.done():
                ready.set_exception(e)

    async def _ask(self, c, prompt):
        import time

        out, texts, t0 = AgentResult(cost_usd=None), [], time.time()
        await c.query(prompt)
        async for msg in c.receive_response():
            kind = type(msg).__name__
            if kind == "SystemMessage" and getattr(msg, "subtype", None) == "init":
                out.api_key_source = (getattr(msg, "data", None) or {}).get("apiKeySource")
            elif kind == "AssistantMessage":
                texts += [b.text for b in msg.content if type(b).__name__ == "TextBlock"]
                u = getattr(msg, "usage", None) or {}
                if u:  # the context the model saw on this turn; the last turn's is the session's size
                    out.context_tokens = sum(int(u.get(k) or 0) for k in
                                             ("input_tokens", "cache_read_input_tokens", "cache_creation_input_tokens"))
            elif kind == "ResultMessage":
                out.cost_usd, out.turns, out.usage = msg.total_cost_usd, msg.num_turns, msg.usage or {}
                out.session_id, out.is_error = msg.session_id, bool(msg.is_error)
                out.structured, out.errors = msg.structured_output, list(msg.errors or [])
                if msg.result:
                    texts.append(msg.result)
        out.text, out.seconds = (texts[-1] if texts else ""), time.time() - t0
        return out

    def _start(self):
        import concurrent.futures as cf
        import threading as th

        self._loop = asyncio.new_event_loop()
        self._thread = th.Thread(target=self._loop.run_forever, name=f"session-{self.name}", daemon=True)
        self._thread.start()
        ready = cf.Future()
        asyncio.run_coroutine_threadsafe(self._owner(ready), self._loop)
        ready.result()

    # --- the public side ---

    def send(self, call: AgentCall) -> AgentResult:
        import concurrent.futures as cf

        self.ledger.reserve(call.max_budget_usd)
        self.guard = ToolGuard(call.write_root, tuple(call.read_roots), tuple(self.base.tools))
        self.handlers = {t.name: t for t in call.mcp_tools}
        try:
            if self._loop is None:
                self._start()
            fut = cf.Future()
            self._loop.call_soon_threadsafe(self._queue.put_nowait, (call.prompt, fut))
            result = fut.result()
            if result.is_error and result.errors:
                raise RuntimeError("; ".join(map(str, result.errors)))
        except BaseException as e:
            err = f"{type(e).__name__}: {e}"
            stop = budget_stop_cost(err)  # a running total too, if the SDK stopped the session on its budget
            self.ledger.record(call.name, self.model,
                               AgentResult(cost_usd=None if stop is None else max(0.0, stop - self.total_usd),
                                           is_error=True, errors=[err]),
                               budget_usd=call.max_budget_usd, session=self.name)
            raise
        if result.cost_usd is not None:  # the session's running total: keep this message's share (#131)
            total = float(result.cost_usd)
            result.cost_usd, self.total_usd = max(0.0, total - self.total_usd), total
        self.ledger.record(call.name, self.model, result, budget_usd=call.max_budget_usd, session=self.name)
        if result.api_key_source is not None:  # reported on a session's first message only
            check_source(result.api_key_source, auth_token=self._token)
        self.session_id = result.session_id or self.session_id
        self.messages += 1
        ctx = getattr(result, "context_tokens", None)
        if not ctx:  # no per-turn usage: the message's usage over its turns, per turn
            u = result.usage or {}
            ctx = sum(int(u.get(k) or 0) for k in ("input_tokens", "cache_read_input_tokens",
                                                     "cache_creation_input_tokens")) // max(1, result.turns or 1)
        self.context_tokens = int(ctx)
        return result

    def close(self):
        if self._loop is None:
            return
        try:
            self._loop.call_soon_threadsafe(self._queue.put_nowait, None)
            import time

            for _ in range(100):  # let the client disconnect, up to 10 s
                if not any(not t.done() for t in asyncio.all_tasks(self._loop)):
                    break
                time.sleep(0.1)
        finally:
            self._loop.call_soon_threadsafe(self._loop.stop)
            self._thread.join(timeout=10)
            self._loop = None


class ScriptedSession:
    """A session for tests (#125): answers from its runner's script and marks each call with the
    session's name and id."""

    count = 0

    def __init__(self, runner, name, base, resume=None):
        ScriptedSession.count += 1
        self.runner, self.name, self.base, self.resume = runner, name, base, resume
        self.session_id = resume or f"{name}-{ScriptedSession.count}"
        self.messages, self.context_tokens, self.closed = 0, 0, False

    def send(self, call):
        from dataclasses import replace as _replace

        out = self.runner.run(_replace(call, session=self.name, session_id=self.session_id))
        self.messages += 1
        self.context_tokens = int(getattr(out, "context_tokens", 0) or 0)
        return out

    def close(self):
        self.closed = True


class ScriptedRunner:
    """A runner for tests: answers calls in order from ``script``, each an AgentResult or a
    function call -> AgentResult (to act like the agent, for example by writing files)."""

    def __init__(self, script, ledger: SpendLedger | None = None, model="scripted"):
        self.script, self.ledger, self.model, self.calls = list(script), ledger, model, []

    def session(self, name, base, resume=None, messages=4):
        return ScriptedSession(self, name, base, resume)

    def run(self, call: AgentCall) -> AgentResult:
        assert self.script, f"no scripted answer left for the {call.name} call"
        if self.ledger is not None:
            self.ledger.reserve(call.max_budget_usd)
        self.calls.append(call)
        step = self.script.pop(0)
        try:
            result = step(call) if callable(step) else step
        except BaseException:
            if self.ledger is not None:
                self.ledger.release(call.max_budget_usd)
            raise
        if self.ledger is not None:
            self.ledger.record(call.name, self.model, result, budget_usd=call.max_budget_usd)
        return result
