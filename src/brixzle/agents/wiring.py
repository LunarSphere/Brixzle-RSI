"""Wiring checks (issues #18, #19): one tiny real call per agent, to prove the plumbing.

Each check sends one small request through the agent's real configuration (model, credentials,
spend ledger, tools) and checks what comes back. It does not design or critique anything:

- **design:** the design agent's system prompt and tools, asked only to call ``check_family`` once
  on an empty package and report ``ok``. That proves the credential, the model, the cost the SDK
  reports, the loop's own MCP tool and the tool guard.

These are the only real API calls outside a loop run. They are never run by the tests.

    uv run --extra agents python -m brixzle.agents.wiring --agent design --ledger runs/api-spend.jsonl --budget-usd 5
"""

import argparse
import json
from pathlib import Path
import tempfile

from brixzle.agents import design as DA
from brixzle.agents.runner import AgentCall, ClaudeRunner, SpendLedger
from brixzle.loop.config import MODEL

WIRING_BUDGET_USD = 0.20


def design_check(runner):
    with tempfile.TemporaryDirectory(prefix="brixzle-wiring-") as tmp:
        ws = DA.Workspace.make(Path(tmp) / "a1", "rsi_wiring_design")
        call = AgentCall(name="wiring-design", system=DA.system_prompt(),
                         prompt=("Wiring check, not a design task. Call the check_family tool exactly once, then reply "
                                 "with one line: 'check_family ok=<value of ok>'. Do not write any file."),
                         max_budget_usd=WIRING_BUDGET_USD, tools=("Read", "Glob", "Grep", "Write", "Edit"), cwd=ws.pkg,
                         write_root=ws.pkg, read_roots=(ws.pkg,), mcp_tools=(DA.check_tool(ws),), max_turns=4)
        result = runner.run(call)
    passed = "check_family ok=" in result.text and not result.is_error
    return passed, result


CHECKS = {"design": design_check}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--agent", choices=("design",), required=True)
    ap.add_argument("--model", default=MODEL)
    ap.add_argument("--ledger", default="runs/api-spend.jsonl")
    ap.add_argument("--budget-usd", type=float, default=5.0)
    args = ap.parse_args(argv)
    ledger = SpendLedger(args.ledger, args.budget_usd)
    runner = ClaudeRunner(args.model, ledger)
    passed, result = CHECKS[args.agent](runner)
    print(json.dumps({"agent": args.agent, "passed": passed, "model": args.model, "cost_usd": result.cost_usd,
                      "turns": result.turns, "api_key_source": result.api_key_source, "text": result.text[:300],
                      "spent_usd": ledger.spent(), "cap_usd": ledger.cap_usd}, indent=2))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
