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

## The design library

Every measured design gets a card in `runs/library/` (#215). `decide` adds each iteration on its own. To add a registry family, score it with the grid (Z on the four quarters, #222; add `--builds` for a finalist's noisy builds), then file it:

```sh
uv run python -m brixzle.engine.gridscore fable_allround --out runs/grid-<stamp>-fable_allround
uv run python -m brixzle.director.library add fable_allround --grid runs/grid-<stamp>-fable_allround
```

`python -m brixzle.director.library add-run runs/rsi/<name>` files every measured iteration of an earlier run.

## Start

The radius goal starts without a baseline by default (#215): the design library is how a run knows past designs, and nothing in it is a parent. `--baseline <spec>` still works, and the older goals default to `ab`.


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
  (the check, the MVP gate, the quarter screen and the grid score on four placements, #222; noisy builds only when the
  campaign sets `run_builds`), the audit and the critique
  in parallel, and the director.
- Each iteration's maps are in `iter-NN/measured/grid/map-<placement>.png` and its jam analysis in `jams.json`.
- Watch `runs/rsi/radius-scratch-1/STATE.md`, `archive.json` (the best design per Z band) and `designs.jsonl`.
  `python -m brixzle.director status --run runs/rsi/radius-scratch-1` prints the next step and the budget.

## Known limits

- The gripper values (`src/brixzle/engine/gripper_spec.py`) are guesses until the bench tests.
- The yaw that releases the magnet is not simulated; design rule DR18 asks for seats that resist it.
- The release is a free drop from 11.2 mm (#218). A real release (a yaw shear) may give the brick a little spin, which is not simulated.
- Joint gaps of at least 0.2 mm are a design rule (DR19), not a check.
