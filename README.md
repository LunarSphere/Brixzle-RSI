# Brixzle-RSI

**Brixzle recursive self-improvement.** A self-steering loop for designing families of bricks that a drone can place imprecisely.

The drone is a Crazyflie 2.1 Brushless with prop guards. It carries a brick with a passive magnet, hovers over the target with 16 to 40 mm of placement error, and drops it. The brick's own geometry has to catch that error as it falls and guide itself into its seat. The loop designs bricks only: the gripper and the landing pad are designed by hand (`src/brixzle/engine/gripper_spec.py`, `docs/requirements.md`).

> **Status:** the director loop (#165) runs the error-radius goal (#206), scored as Z on the grid over four placements: a stack and the first three reaching bricks of an overhang (#208, #222).

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
   - the **screen**: 40 random free drops within 16 mm, 10 on each of the four placements; at least 32 must seat, or the design is not gridded;
   - the **grid score**: per placement, free drops on an X–Y grid out to 40 mm, refined where the outcome changes, and **Z** from the cells within 16 mm;
   - a **jam analysis** of the failures nearest the aim: reruns at other frictions, bands of repeatable failures, and the contacts at rest reduced and named;
   - for finalists only (`run_builds`), **noisy builds** of a tower and a back-first corbel at 16 mm of error.
5. **Verifier** audits the designer's note against the measurement; **critic** names the mechanisms behind the failures from the maps, the jam analysis and the contacts.
6. The design is offered to the **band archive**, which keeps the best design per Z band (below 0, 0–0.5, 0.5–0.8, 0.8–0.95, 0.95+).

The bounds are enforced by code, not by the prompts: while a run is active, a hook refuses writes outside the run directory, the tracked tree must not change, and every claimed number is rerun.

### The score

- **Z**, the headline (#222): **Z = ¼ Z_stack + ¼ Z_overhang1 + ¼ Z_overhang2 + ¼ Z_overhang3**. Each term is (seated − jammed) / N over that placement's grid cells within 16 mm of the aim, counted by cell area, so it runs from −1 to 1 and a jam costs more than a fall.
- **Stack** is a brick dropped across two seated bricks. **Overhang n** is the reaching brick of course n of a back-first corbel, n voxels past the base's face, dropped onto an ideally built corbel below it. If one brick stacks, many do; if each reach seats, the corbel builds.
- Z = 1 means every release within 16 mm seats on all four: the minimum worth printing, with the MVP gate's stack of 3 and overhang of 3.
- Ties (within 0.02) are broken by Z over 28 mm, then over 40 mm, then by each placement's **green zone** (`green_mm`, the largest error radius inside which every release seats), weakest first. Only a failure that repeats at other frictions counts; one that does not is solver noise.

Each iteration's maps are `iter-NN/measured/grid/map-<placement>.png`: one cell per release offset, coloured by outcome, with rings at 16, 28 and 40 mm, the green zone dashed and the placement's Z16 in the title. All sim results are simulated, uncalibrated contact.

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
| `src/brixzle/engine/` | The measurement: the screen, grid score and Z, jam analysis, structure gate, noisy builds, the drop cache and the analytic predictor |
| `src/brixzle/engine/sim/` | The MuJoCo core: drop rigs (course, edge and corbel layouts), structure and ring builds |
| `src/brixzle/tools/` | Deterministic tools the roles call: `python -m brixzle.tools` contacts, redrop, gate and more, and `tools/check_family.py` |
| `src/brixzle/families/<name>/` | One brick family each: parameters and bounds, section, CAD solid, seats and rules |
| `src/brixzle/rules/` | Design rules (`design_rules.md`) and citations (`citations.md`) |
| `.claude/agents/`, `.claude/commands/` | The four roles' instructions and the `/rsi-loop` command |
| `docs/` | Requirements, architecture, the campaign runbook and decision records |
| `runs/` | Git-ignored. One folder per run |
