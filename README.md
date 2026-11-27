# Brixzle-RSI

**Brixzle recursive self-improvement.** A self-steering loop for designing families of bricks that a drone can place imprecisely.

The drone is a Crazyflie 2.1 Brushless with prop guards. It carries a brick with a passive magnet, hovers over the target with 16 to 40 mm of placement error, and drops it. The brick's own geometry has to catch that error as it falls and guide itself into its seat. The brick's own geometry has to catch that error and guide itself into its seat. The loop designs bricks only: the gripper and the landing pad are designed by hand (`src/brixzle/engine/gripper_spec.py`, `docs/requirements.md`).

> **Status:** the director loop (#165) runs the error-radius goal (#206), scored on the self-alignment grid (#208, #210). The older Agent SDK loop (`intake/`, `agents/`, `loop/`) is being removed (#204).

## Run it

From a clean worktree of `main` that shares the `runs/` folder (full steps in [`docs/campaign-206.md`](docs/campaign-206.md)):

    uv sync
    uv run python -m brixzle.director.drive --run runs/rsi/<name> \
      --new "$(cat docs/campaign-206-goal.txt)" --baseline none --iterations 12 --minutes 480
    uv run python -m brixzle.director status --run runs/rsi/<name>      # next step, budget, archive

The roles run on Codex (`codex exec`; `codex` must be on PATH and signed in). In Claude Code, `/rsi-loop runs/rsi/<name> --new "<goal>"` runs the same steps with Claude subagents. A stopped run resumes with the same command without `--new`.

Every design measured, in any run, also gets a card in the shared **design library**, `runs/library/` (`INDEX.md` lists them): its code, maps, scores per placement, strengths and weaknesses, and why it failed. The roles read it; nothing in it is ever a parent.

Everything a run produces lands in `runs/rsi/<name>/` (git-ignored): `STATE.md`, `archive.json`, `designs.jsonl`, `timeline.jsonl`, and one `iter-NN/` folder per iteration with the design, its measurement and the maps.

## How it works

Each iteration has four roles, each with its own instructions in `.claude/agents/rsi-<role>.md`, and deterministic steps in between (`python -m brixzle.director`):

1. **Director** reads the run's state and decides the next design: a new family, or one change (1 to 3 units: a parameter, a function or a constant) to a member of the archive. It must try a new family when there is nothing to refine, every fourth iteration, and after three iterations that added nothing.
2. **Designer** writes the brick family as a Python package, checks it, measures it with the repo's tools and claims only numbers it measured.
3. **Check:** the candidate changed only what the brief allowed.
4. **Verify** measures the candidate itself (`director/verify.measure_radius`), cheapest first:
   - the family contract, the rule margins and the magnet-target interface;
   - the MVP gate: a stack of 3 and an overhang of at least 3 with ideal placement and a free drop;
   - the **grid score**: per placement type, free drops on an X–Y grid out to 40 mm, refined where the outcome changes; the **green zone** is the largest error radius inside which every release seats;
   - a **jam analysis** of the failures that bound each green zone: reruns at other frictions, bands of repeatable failures, and the contacts at rest reduced and named;
   - **noisy builds** of a tower and a back-first corbel at 16 mm of error, with each failed corbel brick replayed and its contacts named.
5. **Verifier** audits the designer's note against the measurement; **critic** names the mechanisms behind the failures from the maps, the jam analysis and the contacts.
6. The design is offered to the **band archive**, which keeps the best design per error band (below 16, 16–22, 22–28, 28–34, 34+ mm).

The bounds are enforced by code, not by the prompts: while a run is active, a hook refuses writes outside the run directory, the tracked tree must not change, and every claimed number is rerun.

### The score

- **`green_mm`**, the headline: the largest placement error, as a disc radius about the aim, inside which every free drop seats with no jam, wrong valley or fall, on every placement type. The weakest placement sets it. The placement types are the full seat (stacking), both bare edges, and three bricks of a back-first corbel (the first reach, a later back brick and the second reach). Only a failure that repeats counts: each failure nearest the aim is rerun at other frictions with small shifts, and one that does not come back is solver noise.
- Within a band, designs rank by every placement's green zone, weakest first (ties within 1.25 mm), then by the overhang and stack height that the noisy builds reach at 16 mm, then by **Z**.
- **Z = (seated − jammed) / N**, per ring at 16, 28 and 40 mm, counted by cell area. It runs from −1 to 1, and shows how much of the error disc works beyond the green zone.
- The minimum worth printing is a green zone of 16 mm, a stack of 3 and an overhang of 3.

Each iteration's maps are `iter-NN/measured/grid/map-<placement>.png`: one cell per release offset, coloured by outcome, with rings at 16, 28 and 40 mm and the green zone dashed. All sim results are simulated, uncalibrated contact.

See [`docs/requirements.md`](docs/requirements.md) for R1–R13 and the hard constraints, and [`docs/architecture.md`](docs/architecture.md#the-error-radius-goal-206) for the components.

## Relationship to aerial-assembly

This repo starts from the brick and simulation work in [LunarSphere/aerial-assembly](https://github.com/LunarSphere/aerial-assembly) and borrows from it:

- the MuJoCo drop trials and outcome tolerances;
- the measured drone error model (Exp 3);
- the A/B brick, the first family and the reference baseline.

Code is copied in, not imported. Each borrowed file names the aerial-assembly path and commit it came from. Flight, calibration and hardware experiments stay in aerial-assembly; this repo has no flight code.

## Layout

The package lives in `src/brixzle/`. [`REPO_MAP.md`](REPO_MAP.md) is the full map.

| Path | What it holds |
|---|---|
| `src/brixzle/director/` | The loop's deterministic steps and its Codex driver (`drive.py`): run files, the one-change check, verify, the band archive, decisions and the write guard |
| `src/brixzle/engine/` | The measurement: grid score, jam analysis, structure gate, noisy builds, the drop cache and the analytic predictor |
| `src/brixzle/engine/sim/` | The MuJoCo core: drop rigs (course and edge layouts), the held release of older goals, structure builds |
| `src/brixzle/tools/` | Deterministic tools the roles call: `python -m brixzle.tools` evaluate, contacts, redrop, gate and more |
| `src/brixzle/families/<name>/` | One brick family each: parameters and bounds, section, CAD solid, seats and rules |
| `src/brixzle/rules/` | Design rules (`design_rules.md`) and citations (`citations.md`) |
| `.claude/agents/`, `.claude/commands/` | The four roles' instructions and the `/rsi-loop` command |
| `docs/` | Requirements, architecture, the campaign runbook and decision records |
| `runs/` | Git-ignored. One folder per run |
