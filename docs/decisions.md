# Decisions

Choices made where an issue or the docs left something open, each with its reason. One section per issue, in the order the work merged.

- Add to this file in the same PR that makes the choice.
- A decision the maintainer made on an issue is linked from here, not repeated.
- A choice here can be reversed in a later PR; say so in that PR and update the entry.

## #13 Scorer and diagnoser (PR #45)

- **A target's overall score is the smaller of its placement and pickup scores.**
  A design has to do both, and R11 and R12 use the same box score, so the weaker task caps the target.
- **A failed clean-release check scores pickup 0 at every target.**
  The score is the largest box in which every trial succeeds. The release does not depend on the box, so if it fails, no such box exists.
- **The release check runs once per design, not once per target.**
  It is the same trial for every box. `box_search(..., release=False)` lets the scorer run it once.
- **Failure counts are counts of the drops the bisection ran, not rates.**
  Bisection samples scales unevenly, so a rate over those drops would mean nothing. The report says so in its `flags`.
- **Limiting failures are kept per course.**
  They are the failing drops at the smallest failing scale at or above that course's score, with their contact traces. Each row names its course and scale, so a reader can tell which course caps the target.
- **The predictor's failure class is recorded but marked untrusted (`class_trusted: false`).**
  `docs/predictor-validation.md` shows that it confuses jammed with wrong valley and has no falls. Classes and traces come from the sim.
- **Sim-only scoring for `Solid3D` families is left to #51.**
  The review found that the scorer always runs the predictor, so it cannot score a `Solid3D` family (the decision on #18 says those are scored by the sim only). #13 asks for the baseline's report, which is Prismatic, so the fix is a separate issue.

## #14 End tune

Numbers are for A/B V0 on the mean box, seed 0, on macOS arm64 (simulated, uncalibrated contact). Each comes from the `tune.json` of the run directory named with it. The section sits here, out of date order, so that it does not collide with the section another open PR adds at the end of the file.

- **Result: the tuned A/B scores 0.781 against 0.719 untuned, in 175.5 s.**
  `runs/tune-20261112T012144551435Z-ab-mean/`: 13 candidates scored of 18 tried, search 151.9 s, held-out check 23.5 s. Only U moved, from 25 to 26.4 mm. Both held-out scores also went from 0.719 to 0.781.
- **The tune raises the target box only, and can lower another.**
  The scorer on the tuned design (`runs/report-20261112T012155641634Z-ab/`, seed 0) gives 0.781, 0.344 and 0.281 on the mean, p95 and worst boxes. The untuned scores were 0.719, 0.375 and 0.281 (`runs/report-20261111T191022016036Z-ab/`, not produced in this session). So p95 dropped by one bisection step. The tune does not check the other boxes; a run tunes for its own target.
- **There is no aerial-assembly NSGA-III run to compare the time with.**
  The issue asks for that run's config and directory. At aerial-assembly `fc4f724`, `brixzle/runs/` holds only `v0-bridge`, `v0-ring` and `v0-wall`, with no optimiser history or timing. The config `brixzle/configs/nsga-ab.json` asks for a population of 35 over up to 40 generations, so up to 1,400 evaluations against 13 here. The only time on record is the 45–90 minutes per family in this repo's `CLAUDE.md`, which is for the whole old process, not a measured optimiser run.
- **Stage 1 on the predictor is gone, so every family is tuned the same way.**
  The issue was rescoped on 2026-11-11 to one short tune at the end. The earlier comment that routed `Prismatic` families through the predictor no longer applies, and `Solid3D` families need no special case.
- **The variables are scaled to 0–1, and the search starts with a step of 10 % of each range.**
  Py-BOBYQA uses one trust-region radius for every variable, and A/B's ranges run from 0.5 mm (clearance) to 35° (theta).
- **n + 1 interpolation points, the fewest Py-BOBYQA allows.**
  A/B has 12 bounded parameters, so the first model costs 13 candidates. A full quadratic would need 91.
- **A candidate that cannot be built, or breaks a rule, is not simulated.**
  It gets a fixed objective (1.0 and 0.5) worse than any score, since a score is never below 0. On A/B, 5 of the 18 candidates broke a rule (`R11.nest_cone`, `R5.capture_x`, `R4.seated_play`, `R13.bore_wall`) and cost about 4 s each for the solids, not about 20 s.
- **The first probes are scored side by side.**
  The search starts by stepping each parameter once, and those candidates don't depend on each other. Scored in 6 processes, A/B's 12 probes took 42 s of wall-clock (`runs/tune-20261112T010919892394Z-ab-mean/`: search 313.6 s, 21 scored, solver converged). One after another, the same 12 took 130 s of sim time alone, plus rule checks that run did not time (`runs/tune-20261112T005631361934Z-ab-mean/`, made before this change: search 410.7 s, stopped by the 420 s cap with 19 scored).
- **Patience: stop after 4 scored candidates in a row with no gain, counted after the first probes.**
  In `runs/tune-20261112T010919892394Z-ab-mean/` the 12 candidates scored after the probes took 251 s and none beat the U probe. With patience the search took 151.9 s and gave the same design.
- **The score's tolerance stays at the scorer's 0.05.**
  The box scale moves in steps of 1/32, so a probe that changes the capture edge a little shows no gain. A finer tolerance (0.0125) was tried in `runs/tune-20261112T011724313282Z-ab-mean/`: 0.742 to 0.797 on that finer scale, in 461.5 s against 382.1 s for the same code at 0.05. At the scorer's tolerance that tuned design is still 0.781, so the finer scale bought nothing the scorer sees.
- **Most of A/B's parameters do not move the score at all.**
  Stepping H0, gutter, gutter_flat, tip_chamfer, tooth_taper or mouth_chamfer by 10 % of its range left the score at 0.719. Only U raised it, and tooth_L lowered it (0.688). The limiting failure is a wrong valley on course 0 in every scored candidate, which the capture-limit analysis (#56) also found at the zone's X edge. This is a finding for the critic: on A/B the box is capped by the zone's width, which is a shape question more than a tuning one.
- **The held-out check uses the next seed's interior points at friction 0.2 and 0.5.**
  The search runs at the trial's default friction (0.35) and seed. The corners and face centres of the box are the same for every seed, so a new seed alone changes only 4 of the 19 points; the friction ends are what make the check held out. A tuned design is kept only if neither score got worse. The four box searches run side by side.
- **The time of the rule check counts.**
  Building the solids for the rules takes about 4 s a candidate on A/B, so it is inside each candidate's `seconds` and inside the wall-clock cap.
- **Known cost, not fixed here: every box search starts its own worker processes.**
  A candidate takes about 20 s on A/B however it is run. A pool that outlives one design would cut that for the tune and for the scorer, and needs its own issue.

## #46 Placement only; pickup deferred

The maintainer answered the issue's open question on 2026-11-11: bricks do not have to keep the large pickup openings. They only have to be grippable, which is #50.

- **A task that was not scored is absent from the report, not null.**
  `report.json` gains `tasks`, the list of tasks it scored. `targets`, `score` and the counts hold only those tasks, and `overall` is the smallest of their scores. The schema version goes from 1 to 2, because a reader must now check `tasks` before it looks for `pickup`.
- **A task that is not scored is not run.**
  With placement only, the pickup predictor, sim, release check and cross-check are skipped, which is where the time goes. On A/B the report took 55.7 s with placement only (`runs/report-20261111T185545672167Z-ab`) against 174.1 s with both tasks (`runs/report-20261106T170808361622Z-ab`); simulated, uncalibrated contact.
- **Deferral is data: `status: deferred` in `docs/requirements.yaml`.**
  The design agent reads the requirements as data, so a note in the prose would not reach it. `status` is optional and defaults to `active`. `docs/requirements.md` lists the deferred requirements with the reason, and a test keeps the two files in step.
- **R12's wording is unchanged.**
  Only its status changed. Its text and "R12 in detail" describe the pickup trial as built, and they will be revisited with the gripper (#24).

## #47 Quick random check

- **The poses come from one Latin hypercube in four dimensions: X, Z, θ and the course.**
  The issue allows Sobol or a Latin hypercube. 40 is not a power of two, so a Sobol sequence would lose its balance. A Latin hypercube puts exactly one pose in each of the 40 equal slices of every axis.
- **40 drops in total, 20 on each course.**
  The issue asks for 40 poses and both courses, and its binomial numbers assume 40 drops. The fourth dimension of the hypercube picks the course, so the split is exact and each course still covers the box evenly.
- **Poses are judged in order, and the check stops at the 9th failure in that order.**
  The verdict and the drops in `check.json` are then the same for any number of worker processes.
- **One process by default.**
  Measured on A/B V0 on 2026-11-11: the 40 drops of the mean box took 5.2 s in one process and 8.6 s on 4 workers, because each worker takes several seconds to start (`runs/check-20261111T191401341107Z-ab-mean`, `runs/check-20261111T191303900392Z-ab-mean`).
- **Only failing drops keep their contact traces.**
  The design agent needs the traces of the failures. Leaving out the seated ones keeps `check.json` small.
- **A non-finite sim result is a failure called `unstable`.**
  A design whose simulation blows up should go back to the design agent with that reason. It should not stop the loop, and NaN is never written to JSON.
- **The poses use the same mapping as the box search, at scale 1.**
  X is about the aim, Z is the release height above the minimum clearance, and θ is pitch (`box_search.pose_sample`). The error model is otherwise ideal, as it is for the score.

## #48 Full sim map on a fitted grid

All numbers here are simulated, uncalibrated contact, on A/B V0 with the mean target box.

- **Three slices through the aim, on both courses, each fitted on its own.**
  The issue asks for X–Z, X–θ and Z–θ slices around the aim. Each slice holds the third coordinate at the aim: zero pitch, the box's centre height, or zero X offset.
- **For fitting, "red" means any cell that did not seat.**
  Jammed and wrong valley stop an edge growing just as fell does. What matters to the fit is where the green zone ends.
- **Cells are U/25 in X, U/5 in Z and 2° in θ.**
  X is the measured error and where the capture edges and jam bands are, so it keeps the old map's 1 mm at U = 25 mm. Z and θ are coarser because A/B's outcomes barely change along them.
- **Release height stops at 1 U and pitch at ±10°, and an edge still green there is marked `limit`.**
  The issue says to grow an edge "until it is red". On A/B the green zone still reached 2 U and ±15°, the largest values tried, on every slice (`runs/map-20261111T193029503482Z-ab-mean`: 4,734 drops, 169 s on 4 workers, 104 MB of traces). Growing those axes further only spends drops. The worst target box is 7.2 mm and 0.4°, so the limits are still far outside it. In X every edge closes on red.
- **The target box wins over a limit.**
  If a box is wider than a limit, the grid covers the box. The guardrail comes first.
- **An override is either a zoom or a change of grid settings, and both need a reason.**
  A zoom is an extra grid, so it cannot shrink a fitted slice inside the target box. Both kinds are listed in `overrides` in `map.json`.
- **A failing cell is "inside the green zone" if the zone lies on both sides of it along its row and its column, or along one of them when the other has no green at all.**
  That keeps holes and bands that cut across the zone, and leaves out a ragged capture edge (green above and below, but only on one side). A grid edge marked `limit` counts as green, because the zone runs on past it.
- **Every drop's contact trace is kept, in `drops.json.gz` with floats rounded to 4 decimals.**
  The issue asks for every trace. As plain JSON the first map's traces took 104 MB; rounded and compressed, the fitted map's take 1.8 MB (`runs/map-20261111T193806246752Z-ab-mean`).
- **Six worker processes by default.**
  The same 1,254-drop map took 48.8 s on 4 workers, 40.1 s on 6 and 37.8 s on 8 (`runs/map-20261111T193311241213Z-ab-mean`, `…193402487600Z…`, `…193451515332Z…`). The grids are identical for any count.
- **The fixed-grid comparison uses the same rows and cell sizes, with X spanning ±30 mm.**
  The old grid had no Z or θ axis, so the Z–θ slice counts as it is.
- **The map refuses a design that failed the quick check, unless the caller says otherwise.**
  The issue says the map runs only for designs that pass. The command line has `--even-if-check-fails` for looking at a baseline.

## #49 Jam analysis inside the green zone

Numbers are from A/B V0's mean-box analysis, `runs/map-20261111T200204588290Z-ab-mean/jam_analysis.json` (sim: simulated, uncalibrated contact; mechanism: predicted, quasi-static 2D). They were measured on macOS arm64. Contact details differ between platforms: on the Linux CI runner the seated brick's most loaded point carried 7.4% of the weight, and the hand-picked edge jam below came out as a 3D effect.

- **A "seed" is a small seeded shift of the pose: up to 5% of a map cell on each axis.**
  The sim gives the same result for the same pose, so re-running a pose unchanged tests nothing. A shift that small asks whether the failure survives noise, without moving off a thin band.
- **A cell is repeatable if it fails at least two thirds of the seeds at some friction coefficient.**
  A design has to seat at every friction, so a failure that is reliable at one of them is real. A/B's one repeatable cell fails 2 of 3 seeds at μ 0.3 and none at 0.2, 0.4 or 0.5, so an overall rate (17%) would have hidden it.
- **The 5% force threshold applies to a site, after merging.**
  The issue's step 1 puts it on each contact. A seated A/B brick touches at 193 points, and the most loaded one carries 3.0% of its weight (`seated_reference`), so a per-point threshold would leave a seated face with no contacts at all.
- **Contacts are flattened to X–Z before they are merged, and the Y share is measured on the merged sites.**
  A/B's faces lean across Y (its V gutter), so every contact on them has a Y force, and mirror-image contacts cancel. Measured raw, a clean seat has 22% of its force along Y, more than the real jam (15%). Measured on merged sites, the seat has 1%.
- **"A large share" along Y is more than a quarter.**
  A/B's in-plane jam at its +X capture edge has 10% on macOS arm64 (the pose of test `test_an_ab_jam_at_the_capture_edge_is_reduced_and_its_faces_named`), and its 2D reduction is sound. A quarter leaves room above that.
- **A brick resting out of plane is also a 3D effect: more than 1 mm in Y, or rolled or yawed more than 2°.**
  The issue only names the force share. A/B's repeatable jam ends rolled 5.5° and 2.2 mm off its plane with a Y share of only 15%, so the share alone would have sent a clearly 3D jam into the 2D analysis.
- **A leaning face gets more friction in plane: `mu_scale`.**
  Two mirror-image contacts press harder than their in-plane resultant shows. The holding-set test scales each site's friction by the summed normal force over its in-plane part (1.00 to 1.07 on a seated A/B brick).
- **Among holding sets of the same size, the one the sim loads most is reported,** with a count of the alternatives.
  The smallest set is not always unique, and the sim's forces are the best hint about which one is acting.
- **`needs_friction` separates a friction jam from a stable rest.**
  If the set's normal forces alone balance gravity, steeper faces or less friction will not free the brick; the geometry has to change. A wrong valley is the usual case.
- **A 3D-effect band still lists its named contact sites.**
  The reduction stops, as the issue says, but the critic still needs to see what touches what.
- **Face names are built from the geometry:** the part, corner or face, its angle and which way it faces, whether it is a guide face, and its end points in the section frame.
  The family interface has no names for faces, and the names have to work for a brick the design agent has just invented.
- **`jam_analysis.json` is written into the map's run directory and never overwrites one.**
  The issue asks for it "beside the map". A second analysis of the same map needs a new map run.
- **Capture limits are left to #56.**
  This issue covers the failures inside the green zone. What stops capture at the zone's edge needs the same reduction on the first failing cell past each edge.

## #56 Capture-limit analysis

Numbers are from `runs/map-20261111T201408241278Z-ab-mean/jam_analysis.json`, A/B V0 on the mean box, measured on macOS arm64 (sim: simulated, uncalibrated contact; mechanism: predicted, quasi-static 2D).

- **A limit is taken in the row through the aim, from the green band that holds the aim.**
  That is the row the target box is centred on. A stray green cell further out does not move the limit.
- **Only the X edges are analysed.**
  X is the measured error and the axis the zone is too narrow on. In release height and pitch A/B's zone runs past the map's limits, so there is no edge to explain.
- **A capture limit is not re-run with seeds.**
  The edge is by definition where the outcome changes, so a small shift of the pose would flip it and say nothing. It is dropped once at each friction instead. On A/B all eight limits give the same outcome at μ 0.2, 0.3, 0.4 and 0.5.
- **If the aim itself fails, there is one entry, at the aim.**
  There is then no band to have edges.
- **The X–Z and X–θ slices both report their limits, even where they nearly coincide.**
  Their rows through the aim differ only in release height (5 mm against 3.1 mm on A/B's mean box). Reporting both keeps the rule simple, and it costs eight drops.
