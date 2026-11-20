"""The self-steering design loop's deterministic parts (#165): the run's files, the bounds the agents
work inside, the one-change check, the verifier, the director's decisions and the Pareto archive.

The loop itself runs inside Claude Code (``.claude/commands/rsi-loop.md``): one session orchestrates
and spawns a subagent per role (designer, verifier, critic, director); every step's state is a file
in the run directory, so a stopped session resumes from it. No Agent SDK and no API calls.
"""
