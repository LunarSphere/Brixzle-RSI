# The first error-radius campaign (#206)

How to start the first campaign of the error-radius goal from scratch, on Codex roles, from a clean worktree.
All results it produces are simulated, uncalibrated contact.

## Setup (once)

From your main checkout, make a worktree of `main` that shares the existing `runs/` folder (the drop cache, earlier
runs and `runs/designs.jsonl` live there):

```sh
cd ~/Repos/Brixzle-RSI
git fetch origin
git worktree add ../Brixzle-RSI-206 origin/main
cd ../Brixzle-RSI-206
ln -s ../Brixzle-RSI/runs runs
uv sync
codex --version     # the roles run on Codex: it must be on PATH and signed in
```

The loop refuses to start if another loop's lock (`.rsi-loop-active`) is in this checkout, and ends the run if a
tracked file changes, so keep this worktree clean while it runs.

## Start

```sh
uv run python -m brixzle.director.drive --run runs/rsi/radius-scratch-1 \
  --new "$(cat docs/campaign-206-goal.txt)" --baseline none --iterations 12 --minutes 480
```

Resume a stopped run with the same command without `--new`:

```sh
uv run python -m brixzle.director.drive --run runs/rsi/radius-scratch-1
```

From Claude Code instead, `/rsi-loop runs/rsi/radius-scratch-1 --new "..." --baseline none --unattended` runs the
same steps with Claude subagents (#205 will make it call Codex roles).

## What to expect

- Iteration 1 (when there is nothing to refine), every 4th iteration and any iteration after three that added nothing must be a new family (#214; 3rd and two before)
  (`decide` refuses anything else).
- Each iteration: a designer call (8 to 15 min in earlier Codex runs), the check, the verifier's measurement
  (the grid score of A/B took 45 s on 14 workers, `runs/grid208-20261126-090821-ab-all`; the MVP gate and the noisy
  builds add to it), the audit and the critique
  in parallel, and the director.
- Each iteration's maps are in `iter-NN/measured/grid/map-<placement>.png` and its jam analysis in `jams.json`.
- Watch `runs/rsi/radius-scratch-1/STATE.md`, `archive.json` (the best design per error band) and `designs.jsonl`.
  `python -m brixzle.director status --run runs/rsi/radius-scratch-1` prints the next step and the budget.

## Known limits

- The gripper values (`src/brixzle/engine/gripper_spec.py`) are guesses until the bench tests.
- The yaw that releases the magnet is not simulated; design rule DR18 asks for seats that resist it.
- Held release holds the brick's attitude rigidly; a real magnet mount gives some compliance.
- Joint gaps of at least 0.2 mm are a design rule (DR19), not a check.
