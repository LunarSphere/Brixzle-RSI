# Brixzle-RSI

**Brixzle recursive self-improvement.** A recursive, self-improving loop for designing families of bricks that a drone can pick up and place imprecisely.

Given a concept sketch and a set of requirements, two AI agents keep redesigning the brick. The goal is to make the region of drone error it tolerates when placing a brick as large as possible, with no jams inside it. Pickup is deferred until a gripper is designed for the chosen brick.

> **Status:** the loop runs end to end: intake (command line), design agent, quick check, full sim map, jam analysis, Z, critic, acceptance and the end tune. The intake page (#65) and the first full benchmark run (#22) are still to come.

To run it (it spends real API money; the cap is `--budget-usd`):

    uv sync --extra agents
    uv run python -m brixzle.intake --image path/to/sketch.png
    uv run --extra agents python -m brixzle.loop --intake runs/intake-<stamp>/intake.json --target mean --budget-usd 5

Results land in `RSI_RESULTS/<run>/`. The run first scores the A/B baseline as iteration 0, with no Claude call, and every design must beat it (`--parent none` starts from scratch; `--baseline-only` scores A/B and stops).

## How it works

Each iteration of the loop redesigns the brick's shape and tests it in simulation:

1. The design agent builds a brick family from the concept sketch, the requirements and the last redesign brief.
2. A quick check drops the brick 40 times inside the target error box in MuJoCo. A weak design goes straight back to step 1.
3. A structure gate builds a tower, a running-bond wall, a bridge and an overhang in simulation, each on seeded repeats. A design that cannot build them goes back to step 1. A design good enough to keep must also close a ring (issue #89).
4. A full map of drops shows where the brick seats, jams, lands in the wrong valley or falls.
5. A jam analysis explains each jam inside the zone where the brick seats, and what stops capture at the zone's edges.
6. A critic agent reads the map and the analysis, explains the failures with design rules drawn from the literature, and writes a redesign brief. The brief can reshape the current family or invent a new one.

The loop does not tune numbers inside an iteration. When Z reaches 1 on the last box or stops improving, one short gradient-free search (Py-BOBYQA) tunes the final family's parameters, such as ramp angles and clearances.

The target climbs a ladder (issue #84): a run starts on `--target`, and once the best design reaches Z = 1 there it is judged on the next box (mean, then p95, then max). A design is kept only if, on Z on the current box and the overhang's length (issue #83), neither is lower than the best's and one is higher, without lowering Z on a box below. `--no-ladder` stays on `--target`.

A design's objective is **Z** (issue #61). For a target error box in X, release height and θ, take every drop of the full sim map inside the box: Z = (seated − jammed) / drops, so it runs from −1 to 1, and Z = 1 (every drop seated) passes. Each design is scored on three target boxes, sized to the drone's mean, 95th-percentile and worst measured error. Z is counted over the box, not the green zone, so a design cannot raise it by narrowing where it seats.

See [`docs/architecture.md`](docs/architecture.md) for the components and [`docs/requirements.md`](docs/requirements.md) for R1–R13.

## Relationship to aerial-assembly

This repo starts from the brick and simulation work in [LunarSphere/aerial-assembly](https://github.com/LunarSphere/aerial-assembly) and borrows from it:

- the MuJoCo drop trials and outcome tolerances;
- the measured drone error model (Exp 3);
- the A/B brick, used as the first family and the baseline.

Code is copied in, not imported. Each borrowed file names the aerial-assembly path and commit it came from.

Flight, calibration and hardware experiments stay in aerial-assembly.

## Layout

The package lives in `src/brixzle/`. [`REPO_MAP.md`](REPO_MAP.md) is the full map.

| Path | What it holds |
|---|---|
| `src/brixzle/intake/` | Local web page: concept sketch and extra requirements, with a conflict check against R1–R13 |
| `src/brixzle/agents/` | Design agent and critic agent (Claude Agent SDK) |
| `src/brixzle/engine/` | Quick check, sim map, jam analysis, objective Z, scorer, end tune and the analytic predictor |
| `src/brixzle/families/<name>/` | One brick family each: parameters and bounds, 2D section, CAD solid, rules |
| `src/brixzle/rules/` | Distilled design rules (`design_rules.md`) and a citation list with links (`citations.md`) |
| `src/brixzle/data/` | Measured drone error files |
| `runs/` | Git-ignored. One folder per loop run, one subfolder per iteration |
