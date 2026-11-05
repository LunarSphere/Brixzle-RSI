# Brixzle-RSI

**Brixzle recursive self-improvement.** A recursive, self-improving loop for designing families of bricks that a drone can pick up and place imprecisely.

Given a concept sketch and a set of requirements, two AI agents keep redesigning the brick. The goal is to make the region of drone error it tolerates as large as possible, with no jams inside it, for both pickup and placement.

> **Status:** planning. The architecture and requirements are written down; no loop code exists yet.

## How it works

The loop has two nested parts.

- **Inner loop: tune the numbers.** The brick's shape is fixed; only its parameters move (ramp angles, clearances and so on).
  1. A 2D analytic model in X, Z and θ follows each release pose contact by contact. It predicts whether the brick seats, jams or falls.
  2. A fast gradient-free search tunes the parameters on that prediction.
  3. MuJoCo then checks only the best few designs, with drops at the edges and corners of the target box.
  4. A short BOBYQA run fine-tunes them.
- **Outer loop: redesign the shape.** A critic agent reads the drop maps and jam traces and explains each jam using design rules drawn from the literature. It then writes a redesign brief, which can reshape the current brick family or invent a new one. The design agent builds the new shape, and the loop runs again.

A design's **score** is the largest scaling of a target error box (in X, Z and θ) that the brick handles with every drop seated. A score of 1 or more passes. Each design is run against three target boxes, sized to the drone's mean, 95th-percentile and worst measured error.

See [`docs/architecture.md`](docs/architecture.md) for the components and [`docs/requirements.md`](docs/requirements.md) for R1–R12.

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
| `src/brixzle/intake/` | Local web page: concept sketch and extra requirements, with a conflict check against R1–R12 |
| `src/brixzle/agents/` | Design agent and critic agent (Claude Agent SDK) |
| `src/brixzle/engine/` | Analytic predictor, tuner, sim verifier, scorer and diagnoser |
| `src/brixzle/families/<name>/` | One brick family each: parameters and bounds, 2D section, CAD solid, rules |
| `src/brixzle/rules/` | Distilled design rules (`design_rules.md`) and the papers they cite |
| `src/brixzle/data/` | Measured drone error files |
| `runs/` | Git-ignored. One folder per loop run, one subfolder per iteration |
