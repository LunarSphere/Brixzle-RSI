---
description: Run the self-steering brick-design loop (#165) - designer, verifier, critic and director subagents, all state in the run directory
argument-hint: <run dir> [--new "<goal>"] [--baseline ab] [--iterations N] [--minutes M] [--replay] [--unattended]
---

You orchestrate Brixzle-RSI's self-steering design loop (issue #165), with no Agent SDK or API calls.

**Arguments:** $ARGUMENTS

**Setup**

1. Read `CLAUDE.md` and `docs/architecture.md`, section "The self-steering loop".
2. **With `--new`:** run
   `uv run python -m brixzle.director init --run <run> --goal "<goal>" --baseline <name> --iterations N --minutes M [--deny-read ...] [--checkpoints ...]`.
   - With `--replay`, add `--deny-read src/brixzle/families/twinhook,src/brixzle/families/fable_twinhook4,src/brixzle/families/fable_gentle,src/brixzle/families/fable_feather,src/brixzle/families/fable_reach,src/brixzle/families/fable_wide,src/brixzle/families/fable_allround,runs/fable,RSI_RESULTS,clean_results,docs/november-16-2026-concise.md,docs/november-16-2026-verbose.md`. This hides the designs being replayed.
3. **Without `--new`:** resume. Read `<run>/next.json` and continue from its step. Every step's output is a file, so nothing else is needed.

**Run every step in the foreground.** Never use `run_in_background` and never stop to wait: in a headless session
(`claude -p`) the session ends when your turn ends. Give each Bash call `timeout: 600000` (10 minutes). `baseline`
and `verify` save their parts as they go, so if one is cut off at 10 minutes, run the same command again: it
resumes, and the drop and stage caches make the redone part fast. Keep each Bash command to one plain command
(no `$(...)` arithmetic), so the permission rules allow it.

**The loop.** After each step, check `<run>/next.json` and `uv run python -m brixzle.director status --run <run>`.

0. **Baseline** (only if `<run>/baseline/` is missing).
   - Run `uv run python -m brixzle.director baseline --run <run>` (about 8 min).
   - Spawn **rsi-director** for iteration 00. Its decision goes to `<run>/iter-00/decision.json`.
   - Then run `uv run python -m brixzle.director decide --run <run> --iter 0`.
1. **Design** iteration N, from `<run>/iter-NN/brief.json`.
   - Spawn the **rsi-designer** subagent.
   - Give it only the brief's path, the run directory and its role file `.claude/agents/rsi-designer.md`.
   - If that agent type isn't available in this session, use a general-purpose subagent with the role file's text as its instructions. Do the same for every role.
2. **Check:** `uv run python -m brixzle.director check --run <run> --iter N`.
   - If it is refused, give the designer the refusal once.
   - If it is refused again, write `{"action": "fix", ...}` back to the director (step 6) with the refusal as evidence.
3. **Verify:** `uv run python -m brixzle.director verify --run <run> --iter N`.
   - It measures the candidate (about 6–9 min), compares the claim and offers the design to the archive.
4. **Audit:** spawn **rsi-verifier** on `<run>/iter-NN/`.
5. **Critique:** spawn **rsi-critic** on `<run>/iter-NN/`.
6. **Direct:** spawn **rsi-director**. It writes `STATE.md` and `<run>/iter-NN/decision.json`.
   - Then run `uv run python -m brixzle.director decide --run <run> --iter N`.
   - If it is refused, give the director the reason once.
7. **Guard:** `uv run python -m brixzle.director guard --run <run>`.
   - If any tracked file changed, stop the run with `finish --reason "tracked files changed: ..."` and report it.
8. **Checkpoint.** If N is one of `goal.json` → `human_checkpoints` and `--unattended` was not given, stop here and summarise `STATE.md` for the maintainer.
   - They resume with `/rsi-loop <run>`.
   - With `--unattended`, never wait.
9. **Stop** when `next.json` says `finish` or the budget is exhausted:
   - run `uv run python -m brixzle.director finish --run <run> --reason "..."`;
   - summarise `STATE.md` and `summary.json`.

**Rules for you, the orchestrator**

- Pass each subagent only what its role needs: the paths above, not your own conclusions.
- Never edit a subagent's files.
- Never write numbers yourself. They come from the director CLI's output files.
- Record each step's wall-clock in `<run>/timeline.jsonl`, one line per step: step, iteration, start, end, seconds.
- Results are "simulated, uncalibrated contact".
