# Brixzle-RSI

**Brixzle recursive self-improvement.** A recursive, self-improving loop for designing families of bricks that a drone can pick up and place imprecisely.

Given a concept sketch and a set of requirements, two AI agents keep redesigning the brick. The goal is to make the region of drone error it tolerates when placing a brick as large as possible, with no jams inside it. Pickup is deferred until a gripper is designed for the chosen brick.

> **Status:** the evaluation engine is built. The agents, the intake page and the orchestrator are not.

## How it works

Each iteration of the loop redesigns the brick's shape and tests it in simulation:

1. The design agent builds a brick family from the concept sketch, the requirements and the last redesign brief.
2. A quick check drops the brick 40 times inside the target error box in MuJoCo. A weak design goes straight back to step 1.
3. A full map of drops shows where the brick seats, jams, lands in the wrong valley or falls.
4. A jam analysis explains each jam inside the zone where the brick seats, and what stops capture at the zone's edges.
5. A critic agent reads the map and the analysis, explains the failures with design rules drawn from the literature, and writes a redesign brief. The brief can reshape the current family or invent a new one.

The loop does not tune numbers inside an iteration. When the score reaches 1 or stops improving, one short gradient-free search (Py-BOBYQA) tunes the final family's parameters, such as ramp angles and clearances.

A design's **score** is the largest scaling of a target error box (in X, Z and θ) that the brick handles with every drop seated. A score of 1 or more passes. Each design is run against three target boxes, sized to the drone's mean, 95th-percentile and worst measured error.

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
| `src/brixzle/engine/` | Analytic predictor, tuner, sim verifier, scorer and diagnoser |
| `src/brixzle/families/<name>/` | One brick family each: parameters and bounds, 2D section, CAD solid, rules |
| `src/brixzle/rules/` | Distilled design rules (`design_rules.md`) and the papers they cite |
| `src/brixzle/data/` | Measured drone error files |
| `runs/` | Git-ignored. One folder per loop run, one subfolder per iteration |
