# Predictor validation (issue #11)

Where the analytic placement predictor (`engine/predictor/`, issue #10) can stand in for the MuJoCo drop trial (`engine/box_search.py`, issue #8).

- Predictor results are **predicted, quasi-static 2D**.
- Sim results are **simulated, uncalibrated contact**.
- Neither is a hardware rate.

This page describes the predictor on `main` after #37, #39 and #41. What the first validation found, and how it was fixed, is under [History](#history).

## How it was run

`uv run python -m brixzle.engine.validate --family ab --statistic max` (`src/brixzle/engine/validate.py`).

- **Run:** `runs/validate-20261106T152702911428Z-ab-placement-max/` (`result.json`, `report.md`), on `main` at `ba6e3ef`, produced 2026-11-06. The grid is deterministic, so it has no seed.
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
- **Wall-clock:** 47.3 s on 4 workers (grid 39.4 s, edges 7.9 s). Summed per pose, the predictor took 38.7 s and the sim 83.4 s.

## Results

### Seated or not (what the score uses)

| Course | Predictor | Agreement | Only the sim seats (conservative) | Only the predictor seats (optimistic) |
|---|---|---|---|---|
| 0 | every μ | 100 % | 0 | 0 |
| 0 | sim μ | 100 % | 0 | 0 |
| 1 | every μ | 100 % | 0 | 0 |
| 1 | sim μ | 100 % | 0 | 0 |

- **No optimistic poses anywhere.** The predictor never calls a pose seated when the sim does not.
- **The agreement rate flatters the predictor.** About two thirds of the grid is far outside the capture band, where both models fail easily. The edge comparison below is the stronger test.

### Capture edges, in mm about the aim

Ranges are over the 9 (Z, θ) rows. The bisection resolution is 0.06 mm. Both predictor verdicts give the same edges.

| Course | Side | Sim edge | Predictor edge | Max diff |
|---|---|---|---|---|
| 0 | dx < 0 | 10.98–11.04 | 11.04–11.10 | 0.12 |
| 0 | dx > 0 | 13.93–14.05 | 13.93–13.99 | 0.06 |
| 1 | dx < 0 | 11.76–11.88 | 11.82–11.88 | 0.12 |
| 1 | dx > 0 | 13.08–13.14 | 12.90–13.02 | 0.24 |

- **The edges agree to within 0.24 mm,** which is four bisection steps.
- **Z and θ have no visible effect** on either model inside this box.

### Predicted boxes against the sim

Seed 0, tol 0.05; course 0 / course 1.

| Box | Predicted | Sim |
|---|---|---|
| mean | 0.719 / 0.781 | 0.719 / 0.781 (#8) |
| p95 | 0.375 / 0.406 | 0.375 / 0.406 (`runs/box-search-20261106T151148Z-ab-placement-p95`) |
| worst | 0.281 / 0.281 | 0.281 / 0.281 (#8) |

- **Predicted runs:** `runs/predict-20261106T151159Z-ab-placement-mean`, `…151208Z-…-p95`, `…151216Z-…-max` (#41).
- **They match exactly,** and they are the same at step sizes of 0.25, 0.5 and 1 mm (#41, recorded on `predict.DS_MM`).
- **The p95 box is pinned by a test,** `test_ab_p95_box_matches_the_sim`.

### Failure class (what the critic reads)

When both models fail, they often disagree on why.

| Course | Predictor | Same class | Main mismatches (sim → predictor) |
|---|---|---|---|
| 0 | every μ | 94.9 % | wrong valley → jammed: 18 |
| 0 | sim μ | 99.7 % | jammed → wrong valley: 1 |
| 1 | every μ | 83.7 % | wrong valley → jammed: 47; fell → jammed: 8; jammed → wrong valley: 5 |
| 1 | sim μ | 88.3 % | jammed → wrong valley: 28; fell → jammed: 8; wrong valley → jammed: 7 |

No run ends `blocked` any more. Every predictor failure ends in one of two ways:
- "weakly stable away from the seat": 783 of 3,690 (pose, μ) runs;
- "strongly stable away from the seat": 171.

Both count as jams by rule, which is where most of the jammed / wrong-valley confusion comes from.

## The seat rule (changed in #41)

A brick at the seat can be **indeterminate**: resting and moving are both mechanically consistent (Pang & Trinkle: weakly but not strongly stable).

- **Old rule (#10):** that counted as a possible jam, a failure.
- **New rule (#41):** the predictor follows the consistent motion for at most `predict.MAX_FOLLOW` = 20 steps.
  - If it ends in a strongly stable seat, the pose is **seated**.
  - If it is still indeterminate after 20 steps, it stays a **possible jam**.
- **Why:** under the old rule, the outcome depended on the exact arrival pose. One mean-box pose stopped 0.017 mm short of full seat at μ = 0.4, where sliding on was consistent. It counted as a jam there, but it seated at 0.2, 0.3 and 0.5. That made the boxes jitter by up to two bisection steps with step size.
- **Tests:** `test_an_indeterminate_seat_is_followed_into_the_seat` pins the rule, and checks that `MAX_FOLLOW = 0` gives the old rule back.

## Where the predictor can be trusted

On A/B V0, inside the worst placement box:

1. **Trust it for seated or not, the capture edge and the box score.**
   - Its edges match the sim's to within 0.24 mm, and all three boxes match exactly.
   - It never calls a pose seated that the sim rejects.
   - It is safe as the tuner's first stage (#14), because it will not promote a design the sim then fails.
2. **Don't trust its failure class.**
   - It often confuses jammed and wrong valley, especially on course 1.
   - It calls the sim's 8 course-1 falls jams: it has no momentum, so a brick that topples off in the sim stops in the predictor.
   - The critic (#19) should take the failure class and contact traces from the sim's drops, not from the predictor.
3. **Momentum catches.** The predictor can stop in a catch that momentum carries the sim past. One example: a p95 corner pose at μ = 0.5 caught in a concave corner of the seated brick, 5.4 mm above the seat (#39). It did not affect any box score here, but a new design with deeper catches could score lower in the predictor than in the sim.
4. **Not covered.** This validation says nothing about the following:
   - pitch beyond ±0.35°, or release heights beyond 14.5 mm above the minimum clearance;
   - roll, yaw, Y error and release velocity (the predictor is 2D and at rest);
   - `Solid3D` families. The predictor steps the y = 0 slice, so it is only sound for `Prismatic` (2.5D) families. Mirror symmetry is not enough. `Solid3D` designs are a sim-only escape hatch (decision on #18).
   - other families or other A/B parameters. A tuned design should be spot-checked again with this tool, which takes about 1 minute.

## History

The first validation (#11, PR #35, `runs/validate-20261106T061226397001Z-ab-placement-max`) found three predictor faults. All three are now fixed.

| Found | Effect | Fixed in |
|---|---|---|
| A contact merge dropped a distinct corner contact. The corner then sank into the brick and every step blocked. | 3 conservative poses at μ = 0.2 (course 0, dx +11.55 mm); a 2.53 mm edge gap | #37 (issue #36): retry a blocked step with unmerged contacts |
| A detection gap at edge ends let the brick drop through a corner. | #10's mean box 0.594 against the sim's 0.719 | #37: corner pairs reach √2 × the contact tolerance; a penetrating drop is reported |
| Penetration drift when pivoting on one corner: first-order steps crept to the tolerance and blocked. | 345 `blocked` runs; a 0.84 mm course-1 edge gap; wrong failure classes | #39 (issue #38): depenetrate after each step |
