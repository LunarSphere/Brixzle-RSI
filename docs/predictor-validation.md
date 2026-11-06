# Predictor validation (issue #11)

Where the analytic placement predictor (`engine/predictor/`, issue #10) can stand in for the MuJoCo drop trial (`engine/box_search.py`, issue #8).

- Predictor results are **predicted, quasi-static 2D**.
- Sim results are **simulated, uncalibrated contact**.
- Neither is a hardware rate.

## How it was run

`uv run python -m brixzle.engine.validate --family ab --statistic max` (`src/brixzle/engine/validate.py`).

- **Run:** `runs/validate-20261106T061226397001Z-ab-placement-max/` (`result.json`, `report.md`). It was produced on 2026-11-06 in the issue #11 session. The grid is deterministic, so it has no seed.
- **Design:** the A/B V0 baseline, on both courses (course 0 onto the base, course 1 onto a seated A).
- **Grid:** 41 X × 3 Z × 3 θ, regular, at scale 1 of the worst (max) placement box:
  - X: ±38.5 mm about the aim, in 1.925 mm steps;
  - Z: 0 to 14.5 mm above the minimum release clearance;
  - θ: ±0.35° pitch.

  The worst box is the widest in every axis, so it contains the mean and p95 boxes.
- **Sim:** the default `TrialConfig` (friction 0.35, timestep 1 ms, contact time constant 4 ms, 50 iterations), ideal pose except for the grid offset.
- **Predictor:** compared two ways:
  - **every μ:** seated only if it seats at every μ in {0.2, 0.3, 0.4, 0.5}. This is what the loop scores.
  - **sim μ:** at μ = 0.35 only. This is the like-for-like check of the model against the sim.
- **Capture edges:** per course, (Z, θ) row and side, the grid brackets the edge of the seated band around the aim. Five bisections then refine it to 0.06 mm, for each model.
- **Wall-clock:** 59.1 s on 4 workers (grid 48.8 s, edges 10.3 s), for 738 grid poses on each model.

## Results

### Seated or not (what the score uses)

| Course | Predictor | Agreement | Only the sim seats (conservative) | Only the predictor seats (optimistic) |
|---|---|---|---|---|
| 0 | every μ | 99.2 % (366 / 369) | 3 | 0 |
| 0 | sim μ | 100 % | 0 | 0 |
| 1 | every μ | 100 % | 0 | 0 |
| 1 | sim μ | 100 % | 0 | 0 |

- **No optimistic poses anywhere.** The predictor never calls a pose seated when the sim does not.
- **The agreement rate flatters the predictor.** About two thirds of the grid is far outside the capture band, where both models fail easily. The edge comparison below is the stronger test.

### Capture edges, in mm about the aim

Ranges are over the 9 (Z, θ) rows. The bisection resolution is 0.06 mm.

| Course | Side | Sim edge | Predictor, sim μ | Predictor, every μ |
|---|---|---|---|---|
| 0 | dx < 0 | 10.98–11.04 | 11.04–11.10 (max diff 0.12) | 11.04–11.10 (max diff 0.12) |
| 0 | dx > 0 | 13.93–14.05 | 13.93–13.99 (max diff 0.06) | 11.52–13.93 (max diff **2.53**) |
| 1 | dx < 0 | 11.76–11.88 | 11.82–11.88 (max diff 0.12) | 11.82–11.88 (max diff 0.12) |
| 1 | dx > 0 | 13.08–13.14 | 12.90–13.02 (max diff 0.24) | 12.30–12.66 (max diff 0.84) |

- **At the sim's friction, the edges agree to within 0.24 mm,** which is four bisection steps.
- **Z and θ have no visible effect** on either model inside this box. Every row's edge sits within 0.12 mm of the others, except where the μ = 0.2 artefact below applies.

### Failure class (what the critic reads)

When both models fail, they often disagree on why.

| Course | Predictor | Same class | Main mismatches (sim → predictor) |
|---|---|---|---|
| 0 | every μ | 94.0 % | wrong valley → jammed: 18 |
| 0 | sim μ | 99.7 % | jammed → wrong valley: 1 |
| 1 | every μ | 80.8 % | wrong valley → jammed: 57; fell → jammed: 8; jammed → wrong valley: 6 |
| 1 | sim μ | 85.4 % | jammed → wrong valley: 26; wrong valley → jammed: 20; fell → jammed: 8 |

## The one predictor artefact

All three conservative poses sit on course 0, at dx = +11.55 mm and θ = +0.35°, at every Z.

- **What fails:** the predictor fails only at μ = 0.2, with the reason `blocked: every step penetrates`. Its last state is a slide on two contacts whose normals differ by about 0.1°. This is a stepping failure, not a physical jam.
- **The sim seats all three at μ = 0.2 too.** This was checked in this session with `DropRig` at friction 0.2. So the predictor is simply wrong there.
- **It also explains #10's mean-box gap.** The recorded mean box (`runs/predict-20261106T053155Z-ab-placement-mean`) stops at 0.594 against the sim's 0.719 because one corner at scale 0.625 fails with the same reason at μ = 0.2. The other two μ values and the sim seat it.
- **It makes the predictor non-monotone in X:** a hole at 11.55 mm with a seat at 13.48 mm. A bisected box score can therefore stop early on it.

## Where the predictor can be trusted

On A/B V0, inside the worst placement box:

1. **Trust it for seated or not, and for the capture edge, at a fixed μ.**
   - It matches the sim's edges to within 0.24 mm.
   - It never calls a pose seated that the sim rejects.
   - It is safe as the tuner's first stage (#14), because it will not promote a design the sim then fails.
2. **The every-μ score is conservative, and partly for the wrong reason.**
   - On course 1, dx > 0, the every-μ edge sits up to 0.84 mm inside the sim's. This may be real μ dependence, but the sim was only run at μ = 0.35, so that is unverified.
   - On course 0, dx > 0, the up-to-2.53 mm gap is the μ = 0.2 stepping artefact above.
   - Until that is fixed, expect box scores to be low by up to one bisection step or so. Check any box result whose limiting pose failed with `blocked: every step penetrates`.
3. **Don't trust its failure class.**
   - It confuses jammed and wrong valley often, especially on course 1.
   - It calls the sim's 8 course-1 falls jams: it has no momentum, so a brick that topples off in the sim stops in the predictor.
   - The critic (#19) should take the failure class and contact traces from the sim's drops, not from the predictor.
4. **Not covered.** This validation says nothing about the following:
   - pitch beyond ±0.35°, or release heights beyond 14.5 mm above the minimum clearance;
   - roll, yaw, Y error and release velocity (the predictor is 2D and at rest);
   - other families or other A/B parameters. A tuned design should be spot-checked again with this tool, which takes about 1 minute.
