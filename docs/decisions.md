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

## #51 Sim-only report for Solid3D families

The section sits here, out of date order, so that it does not collide with sections other open PRs add elsewhere in the file.

- **`report.json` gains `predictor: {used, reason}`, and the schema version goes from 2 to 3.**
  Every report has the key, so a reader never has to infer a sim-only report from what is missing. A/B's report keeps every key and value it had; it gains `predictor: {"used": true, "reason": null}`.
- **A sim-only report leaves the predictor's results out; it does not null them.**
  A target holds `box` and `sim` only. `disagreements` is empty, `disagreement_counts` and `class_mismatches` are `{}`, and `labels` names the sim alone. This follows #46, where a task that was not scored is absent, not null.
- **The predictor is skipped by geometry type, not by trying it.**
  `predictor_skip_reason(family)` is set for any `Solid3D` family, including one that gives a `section`. The decision on #18 (2026-11-06) says those are scored by the sim only, and `docs/predictor-validation.md` says the predictor is only sound for `Prismatic` families.
- **The flag about the predictor's failure class is dropped from a sim-only report.**
  There is no predicted class to distrust. A flag naming the skip and its reason takes its place.
- **A run that raises removes its run directory.**
  Results are now written only after every stage has run (the release check used to be written early), so the directory is empty or holds partial files when something raises. It is this call's own new directory, so nothing earlier can be lost.
- **Evidence.**
  `uv run python -m brixzle.engine.report --family pyramid --seed 0` wrote `runs/report-20261112T012938315613Z-pyramid/`: the dummy pyramid scores 0 on all three boxes (jams at the aim; simulated, uncalibrated contact), in 12.1 s. It is a contract-test dummy, so the score says nothing about a design. The A/B report after the change is `runs/report-20261112T012959361342Z-ab/` (0.719, 0.375 and 0.281; 56.1 s). Against `runs/report-20261111T191022016036Z-ab/` from before it, only `predictor`, `schema_version` and `timing` differ.

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

## #50 R10 grippable

The maintainer reviewed the R10 wording on 2026-11-11 and changed the mass sentence: the combined mass of the brick and the gripper must not be more than 35 g. The rest of the text in `docs/requirements.md` and `docs/requirements.yaml` is as the issue proposed it.

- **`pickup(p, part)` keeps its name and returns a tuple of `GripFeature(kind, x_mm, z_mm, size_mm, outline=None)`.**
  The issue asks for "a list of generic grip features (type, position, size)". `Bore` and `PickupFeatures` are gone. Keeping the field name avoids touching every family and the `Family` contract for no gain.
- **`kind` is free text, with one reserved meaning: `"bore"`.**
  A bore is a round hole straight through Y that the builders cut, as before. Every other kind is already in the family's geometry. A fixed list of kinds would stop the design agent from offering a feature nobody thought of, which is the point of dropping the fork.
- **A bore carries its hole diameter, not a tine diameter and a clearance.**
  Those two belong to a gripper. A/B's bores are still 2.6 mm (`fork_d + 2 * fork_clearance`), so its solid, mass and collision are unchanged.
- **The loop's R10 rules are `R10.grippable` (features − 1) and `R10.within_payload` (35 g − brick mass − gripper allowance).**
  They state the first two sentences of the wording as margins. The contract also fails a family with no grip feature.
- **The gripper counts as 5 g until it is designed (`RuleSettings.gripper_allowance`).**
  R10 limits the brick and the gripper together, but the gripper comes after the brick (#24). The 5 g is a guess, not a measurement: the fork model in `engine/sim/fork.py` gives 3.3 g (pyramid), 4.1 g (A/B) and 4.6 g (wedge) at each family's defaults (predicted from its PLA box model, computed in this session, not weighed), rounded up. #24 replaces it with the real gripper's mass.
- **R9 is still the tighter mass rule.**
  R9 caps a brick at 25 g, which leaves 10 g of the payload for the gripper. So `R10.within_payload` can only fail if the allowance is set above 10 g. It is kept as its own margin because R10 states the limit.
- **The fork checks moved to `rules.fork_margins`, with the tine diameter as an argument.**
  `R10.hangs_stable` and `R10.tines_straddle_com` keep their formulas and still match the aerial-assembly reference for A/B. Nothing in the loop calls them.
- **The contract no longer asks whether a straight 2 mm tine clears each opening.**
  That check assumed the fork. It lives on in `engine/predictor/pickup.py` (`geometry`), which the pickup trial uses.
- **A feature with no outline is checked against the brick's bounding extent, not its outline.**
  A lip sits on the outline's edge, so "inside the outline" would reject it. The extent check still catches a feature placed off the brick.
- **A/B's own `R10.fork_fits` stays in its `extra_rules`.**
  It only says that A/B found room for its two bores, which are its grip features.

## #61 Objective Z

The maintainer set the definition on 2026-11-12: for each target box, Z = (S − J) / N over every drop of the full sim map inside the box, across all slices and both courses; seated +1, jammed −1, fell and wrong valley 0; Z = 1 passes. Z replaces the box scale in the report, the loop's stop rule, the acceptance rule (#21) and the end tune. The choices below are where the definition left something open.

Numbers are for A/B V0 on macOS arm64 (simulated, uncalibrated contact), each from the run directory named with it.

- **Map limits accepted (maintainer, 2026-11-12):** 1 U of release height and ±10° of pitch, as merged in #55.
- **R10's 5 g gripper allowance stays (maintainer, 2026-11-12),** marked in `docs/requirements.md` as an estimate until #24.
- **One map covers all three target boxes.** Z is counted per box, and the map's grid only promises to cover the box it was run for. So `sim_map(..., cover=...)` widens the guardrail to every box the scorer counts, and the scorer passes the mean, p95 and worst boxes. The slices still cross at the run's target's aim, so a held coordinate (the X–θ slice's release height) is the target box's centre. A report refuses a box the map did not cover, because Z would miss drops.
- **Every drop counts, duplicates included.** The X–Z and Z–θ slices share their cells at zero offset and zero pitch. The definition says every drop, so both count. On A/B's mean box that is 2 of 89 drops per course.
- **Zoom drops do not count.** A zoom is an override at a finer step; counting it would let a caller weight Z.
- **Pitch: only the zero row is inside a box.** Map cells are 2° apart and the boxes are ±0.2–0.4°, so each box holds the zero-pitch row of the X–θ and Z–θ slices. The cell sizes are #48's and were not changed here.
- **The report lives beside the map.** `report.json` (schema 4) is written into the map's run directory, as the jam analysis is. It holds Z per target with S, J, N, fell, wrong valley and other. The pickup report (`--tasks pickup`), the predictor, the cross-check and the box scale are gone from the report. The box search, the pickup trial and the predictor validation stay in the code with their tests.
- **The tune and the acceptance rule get Z without the full map.** A drop depends only on its pose, so dropping only the map's cells inside the boxes (`zscore.ZScore`) gives the map's Z exactly (test `test_the_direct_z_matches_the_full_maps_z`). On A/B that is 622 drops for the three boxes, against 2,000 to 3,000 for a fitted map.
- **The tune refuses a candidate that lowers Z on any other target box** below the untuned design's (`lowers_other`, a penalty worse than any score), so it can never be the result. This reverses the #14 entry "The tune raises the target box only, and can lower another".
- **Penalties sit above 1,** because Z can be negative: −Z lies in [−1, 1], so lowering another box is 1.5, breaking a rule 2.0 and failing to build 3.0.
- **Held-out Z.** "Different seeds" is read as different poses: each in-box cell is shifted by a seeded amount, up to half a cell or the box's half-width if that is smaller (pitch), clipped to the box, and dropped at friction 0.2 and at 0.5. The seed is the run seed + 1. Without the shift, a new seed would change nothing, because the map's cells have no seed.
- **A/B V0's Z,** `runs/map-20261112T202420460668Z-ab-mean/report.json` (quick check seed 0): mean +0.848 (S 154, J 3, wrong valley 21, N 178), p95 +0.336 (S 206, J 56, wrong valley 184, N 446), worst +0.114 (S 206, J 135, fell 2, wrong valley 279, N 622). Check 7.6 s, map 65.5 s, total 74.4 s.
- **A/B V0's end tune on Z found nothing better,** `runs/tune-20261112T202915149550Z-ab-mean/` (seed 0, default settings): Z on the mean box stayed +0.848 in 269.3 s of search; 19 candidates, 7 scored, 7 broke a rule, and 5 were refused for lowering Z on p95 or worst (for example p95 0.336 to 0.323) while holding the mean at +0.848. The U step that raised the old box scale (25 to 26.4 mm) lowered Z on the mean box (+0.843). Each of the first probes took about 70 s, scored serially in six processes side by side; later candidates took about 28 s on six workers.
- **R11 is checked by the sim alone** in `docs/requirements.md` and `docs/requirements.yaml` (`checks: [sim]`), since the predictor is no longer part of the score.

## #16 Design rules corpus

- **The PDFs are gone from the repo; `citations.md` holds a citation and a link for each source.** The repo is public, and the issue asks for links rather than PDFs. The papers stay in git history. Links were checked by web search on 2026-11-12: a DOI where one was found, otherwise the authors', publisher's or a library's record. Whitney 1982 has no confirmed DOI, so it links to a library record.
- **Two files, parsed in code.** `design_rules.md` holds the rules (`## DR<n>.` with Mechanism, In a design, Cites and Evidence lines). `citations.md` holds the sources (`## <key>` with Citation, Link and Read lines). `brixzle.rules` parses both, and a test checks that every rule cites a listed source and every source has an https link and is used.
- **Every rule says what its evidence is:** "paper" (stated in a source read in this repo), "principle" (standard mechanics the source sets out, not checked against its text here) or "user observation". A test keeps a rule from being labelled "paper" when all its sources are unread. The labels follow the "Read" notes from #10.
- **The maintainer's rod-lengthening finding is DR10, labelled a user observation.** It is not in any paper (stated 2026-11-06).
- **Whitney, Gustavson & Hennessey 1983 is not cited.** The maintainer dropped it on 2026-11-06. It is listed under "Removed from the folder, not cited", with `AIM-759.pdf`, whose title was never confirmed.
- **Robotic-construction papers are context only (DR11).** They motivate R5 and R11; the rule says not to copy their parts. This follows CLAUDE.md.

## #17 Intake

- **A command first, the page later.** The maintainer said a command-line intake that writes the same `intake.json` is enough to unblock the loop (2026-11-12). `python -m brixzle.intake` asks at a terminal, and takes `--resolve ID=CHOICE` otherwise. The page stays planned under this issue's scope.
- **One Claude call sorts the extras, with a structured answer and no tools.** Sorting needs judgement, so the design agent's model does it, through the same runner as the agents. With no extras, no call is made.
- **Four answers to a conflict or an unclear item:** keep the base requirement, use the extra, drop the extra, or clarify it in the person's words. A clarification needs its text. Nothing else is accepted.
- **The runner lands here, with its first user.** `agents/runner.py` is shared with the design agent (#18) and the critic (#19). It holds the credential logic, the spend ledger and the tool guard, so they are written and tested once.
- **The Agent SDK is an optional extra (`agents`).** Only real runs need it. The tests use a scripted runner, CI does not download the SDK (about 300 MB, as it bundles Claude Code), and no test can reach the API.
- **Credentials: an API key, else the `ant` profile.** The maintainer logged in with the `ant` CLI. Claude Code does not read that profile itself, so the runner passes the profile's short-lived access token to it as `ANTHROPIC_AUTH_TOKEN`, with the OAuth beta header. The token is never printed or written. Claude Code reports the credential it used. A call fails on anything but an API credential, so spend cannot land on a claude.ai subscription that the ledger does not track.
- **A call whose cost the SDK does not report counts as its whole budget** in the ledger, so an unknown cost errs toward stopping early.
- **`fixed.U_mm`** records a voxel pitch fixed at intake (R2), which the end tune then leaves alone.

## #18 Design agent

- **The runtime model is Claude Sonnet 5.5 (`claude-sonnet-5-5`), set once in `brixzle/loop/config.py` (`MODEL`).** The maintainer chose it on 2026-11-12. It replaces the earlier brief's Opus default.
- **Agent-written families live in the run directory, not in `src/`.** Each attempt gets a package with a unique module name, `rsi_<run>_iNN_aK`, imported with its root on `sys.path`. Worker processes started with spawn inherit `sys.path`, so the sim imports it too. Test `test_a_family_loaded_by_path_can_be_dropped_in_worker_processes` drops one in two workers. Writing into `src/` would mix generated code into the package and the registry.
- **The agent's tools are Read, Glob, Grep, Write, Edit and `check_family`.** It gets no shell. Writes are allowed only inside its package. Reads are allowed only in the repo's `docs/` and `src/`, the run directory and the concept images' folder.
- **`check_family` runs in a separate process with a 240 s limit.** A family's code runs when it is imported, so a hang or a crash must not take the loop down. The loop runs the same check again after the agent stops, and only a package that passes goes to the quick check.
- **A design note, `design_note.json`, comes with every package.** It holds the idea, the change from the parent, the geometry type and, for a `Solid3D` family, which feature could not be prismatic and why (the decision on #18). The design log records it.
- **One design call is capped at $1.50 and 40 turns.** Those are the loop's defaults and can be changed in its config.
- **The wiring check (one real call, 2026-11-12) found two plumbing faults, both fixed here.** It cost $0.0255 (2 turns, Sonnet 5.5; `runs/api-spend-session-20261112.jsonl`), and the API accepted the `ant` profile's token.
  - **The tool guard was never asked.** The Agent SDK warned that a tool in `allowed_tools` is approved before `can_use_tool` is consulted, and read-only tools never ask at all. The guard now runs as a PreToolUse hook, which sees every tool call. `test_every_tool_call_goes_through_the_guard_as_a_pre_tool_use_hook` covers the decision. The hook itself first runs for real in the smoke run, because the one wiring call allowed for this agent was spent.
  - **Claude Code reports `apiKeySource: "none"` when it authenticates with `ANTHROPIC_AUTH_TOKEN`.** It printed that this source "takes precedence over your claude.ai login". The runner therefore accepts "none" only when an auth token is in effect. Without a token, "none" would mean a claude.ai login, which the ledger does not track, and is still refused.

## #19 Critic agent

- **The critic has no tools; its inputs are in the prompt.** Z per target, the fitted map's grids as text, the jam analysis summary (`jam_analysis.summary`), the design log and the family's `family.py`. One structured answer keeps the call small and cheap: a capped call ($0.40 by default), at most 4 turns. Its contact traces are already reduced by the jam analysis, so the critic does not need to read them raw.
- **The brief is validated in code.** Each cluster must cite at least one rule id that exists in `design_rules.md`, and give a failure, a diagnosis, a change, a scope (`reshape` or `new_family`) and an expected effect. A redesign needs at least one cluster. A malformed brief is refused, and the loop logs the error instead of passing it on.
- **The critic optimises Z on the target and is told not to lower it on the other boxes,** to match the end tune (#61).
- **The design log, not the held-out check, is its memory.** The held-out results never reach its prompt (#21 tests this).
- **The wiring check (one real call, 2026-11-12) found the tool guard blocking Claude Code's own `StructuredOutput` tool,** which is how a structured answer is returned. The model wrote a sensible one-cluster brief twice, the PreToolUse hook refused it both times, and the call stopped at its 3-turn cap with an error. The guard now always allows `StructuredOutput` (test `test_the_sdks_own_structured_output_tool_is_never_blocked`), and the cap is 4 turns. The SDK raised instead of reporting a cost, so the ledger charged the call's whole budget of $0.20 (`runs/api-spend-session-20261112.jsonl`). From the session's token counts (about 9.0k cache-write, 16.4k cache-read and 1.4k output tokens on Sonnet 5.5), the real cost was nearer $0.04. That figure is an estimate, not a reported cost. The fixed path first runs for real in the smoke run, because the one wiring call allowed for this agent was spent.

## #20 Design log

- **One JSON line per attempt, in the loop's run directory** (`design_log.jsonl`), appended and never rewritten. A run that stops part-way keeps every attempt it made.
- **Every attempt is logged, not only scored ones.** The decisions are `failed_contract`, `failed_quick_check`, `kept`, `rejected` and `error`. The 2026-11-11 comment asks for quick-check failures, so the design agent does not repeat a design that failed in seconds.
- **Grid overrides are logged with their reasons,** from `map.json`'s `overrides` (the 2026-11-11 comment).
- **The agents read a short text version (`DesignLog.text`)**: the decision and reason, the design note's idea and change, Z per target, quick-check counts, contract problems, and overrides. It is a few lines per attempt, so the prompt stays small as the log grows.
- **Held-out results are kept in the log but never shown to the agents.** `for_agents` and `text` drop them. The acceptance rule (#21) needs them, and the critic must not see them.
- **Non-finite numbers are refused,** as everywhere else that writes JSON.

## #15 Report renders

The maintainer added `RSI_RESULTS/` to this issue on 2026-11-12 (comment on #15).

- **Wrong valley gets its own colour, purple.** The figure style is aerial-assembly's drop grid: seated green, jammed orange, fell red. That figure had no wrong valley, but on A/B it is the most common failure (wrong valley 279 of 622 drops in the worst box), so folding it into red would hide what the critic most needs.
- **The jam bands are outlined, and the target box is drawn dashed.** A box thinner than one map cell in pitch (±0.2° against 2° cells) is drawn one cell tall so it can be seen.
- **The jam map is drawn from map.json alone.** It does not need the 2 MB of traces in `drops.json.gz`, so the render takes a second or two.
- **`summary.md` includes held-out Z.** It is for people. The critic reads the design log, never `RSI_RESULTS/`, so it still never sees held-out results.
- **matplotlib is a dependency.** Renders run in every loop iteration, and the tests draw them.
- **`RSI_RESULTS/` is git-ignored,** like `runs/`. Figures for the repo go through #25.

## #70 Seats in the contract and the drop trial

Found by the first smoke run of #21. The design agent copied the dummy `wedge` as its template, and every design seated 0 of 9 in the quick check (`runs/check-20261112T213715471022Z-zigzag_chevron-mean/`; simulated, uncalibrated contact).

- **Cause: the contract and the drop trial used different seats.** The contract placed course 0 at `Placement(0, 0)` (centre U). The drop trial uses `Placement(0, 1)` (centre 2U) on a 4-voxel base. A/B's base repeats every voxel, so it fit both. The wedge's repeats every two voxels, and it was written to the contract's convention, so in the sim every brick landed a voxel off (wrong valley, error 25 mm = U).
- **The contract now uses the drop trial's placements,** not the other way round. Voxel 1 leaves base on both sides of the brick for drops that land off-centre; voxel 0 would put the brick at the base's left end.
- **The wedge's base now puts its grooves at even multiples of U.** It seats at the aim on both courses (`tests/test_wedge_seats.py`), and 40 of 40 in the quick check on the mean box (seed 0; simulated, uncalibrated contact). It is still a contract dummy, not a design.
- **`check_family` drops the brick once at the aim on each course** and reports the outcome, the rest error and the tilt (`aim_drops`). The prompt tells the agent to make both seat before anything else. Each drop takes about a second, so a brick that cannot sit in its own seat is caught in the agent's loop, not after a quick check and a new call.
- **The aim drops are reported, not required for `ok`.** The quick check stays the gate, and a design that seats at the aim but nowhere else still has to pass it.

## #21 Orchestrator

The maintainer set the loop's order, acceptance rule and stop rule on 2026-11-12: intake, design agent, quick check (back to the design agent on a fail), full sim map, jam analysis, Z, critic, and back to the design agent. A redesign replaces the best only if Z on the target improves and a held-out check (the same target, different seeds, μ 0.2 and 0.5) does not get worse. The critic never sees the held-out results. Stop at Z ≥ 1, no improvement for a set number of iterations, or a spent budget, then run the end tune.

- **The first scored design becomes the best.** There is nothing to compare it with. Its held-out Z is computed and kept as the baseline for the next one.
- **"Improves" means strictly higher Z on the target box. "Not worse" means neither held-out Z (μ 0.2, μ 0.5) is below the current best's.** Held-out Z is `zscore.HeldOutZ` with seed + 1 (#61).
- **Up to 3 design attempts per iteration.** A package that fails the contract or the rules goes back with the check's output. A design that fails the quick check goes back with up to 9 of its failing drops (pose, outcome, where it came to rest). If all three fail, the iteration ends with no scored design and counts as one with no improvement.
- **Patience is 2 iterations with no kept design, and the default cap is 6 iterations.** The 30-minute wall-clock budget (CLAUDE.md) is checked before each iteration. One A/B-sized iteration costs about 95 s of sim (check 7.6 s, map 65.5 s, jam analysis 12 s, held-out about 10 s) plus the agents.
- **The API budget stops the loop before it could be passed.** An iteration starts only if the ledger can cover one more design call and one critic call ($1.50 + $0.40 by default). Each call is also capped by the SDK and charged in full if it fails.
- **The critic is not called when it cannot matter:** when Z is already 1, or on the last allowed iteration.
- **An SDK or API error in an agent call is logged and survived.** A failed design call is a failed attempt (`error`). A failed critic call leaves the next design without a brief. The wiring checks (#18, #19) showed both kinds of failure happen.
- **The end tune runs on the best design whatever stopped the loop,** with U fixed if intake fixed it (R2). It is the end tune from #14 on Z (#61).
- **State is saved after every iteration (`state.json`), and `--resume` carries on.** An interrupted iteration runs again in a new folder (`iter-NN-2`), so no run directory is reused.
- **One run, one target box.** The three target runs are three invocations with `--target mean`, `p95` and `max`.
- **Smoke runs (real API, 2026-11-12; zigzag concept `runs/intake-20261112T211711302350Z/`, mean target, at most 2 iterations, the session ledger `runs/api-spend-session-20261112.jsonl` capped at $5).**
  - **Run 1** (`runs/loop/smoke-20261112-mean/`) was stopped by hand during iteration 1. The design agent copied the dummy `wedge`, which never seated in the sim (#70). Two design calls ($0.0738 and $0.0917) and a third that was killed (about $0.098, estimated from its transcript; it was charged to the ledger by hand). A Claude Code process survived the kill, so the loop now turns SIGTERM into an interrupt that unwinds the SDK.
  - **Run 2** (`runs/loop/smoke2-20261112-mean/`, after #70) passed in one iteration. The design agent wrote `zigzag_chevron` in one call ($0.0848, 8 turns). It seated 40 of 40 in the quick check and scored Z mean +1.000 (178 drops), p95 +0.901 (fell 44) and worst +0.646 (fell 220), with held-out Z +1.000 at μ 0.2 and 0.5 (simulated, uncalibrated contact). The loop stopped on Z = 1 without calling the critic. The end tune found nothing better (`runs/tune-20261112T220148160314Z-zigzag_chevron-mean/`). Run time was 1.0 min; renders are in `RSI_RESULTS/smoke2-20261112-mean/`.
  - **What the smoke run does not show:** design skill. The design is the dummy wedge with a 45° groove and wider bores, adapted rather than taken from the concept sketch, although the prompt says the examples are not designs to copy. The critic's real call was not exercised (Z was already 1).

## #73 No dummy example for the design agent

The maintainer decided on 2026-11-12 to remove the dummy `wedge` from the design agent's examples. In #21's smoke runs the agent copied it as its brick both times.

- **A/B is the only example family the agent is shown,** and its prompt says not to copy A/B's shape.
- **The agent's copy of `family-interface.md` drops the three sentences that name the dummies** (`design.interface_for_agent`). The doc itself is unchanged, because people still need to know the dummies exist.
- **The agent can no longer read the dummy families.** Its read roots are now the docs, the engine, the rules, the family interface and its shared modules, and A/B, not the whole of `src/`. Without this, Glob would find the wedge again.

## #74 Orthographic views of each design

The maintainer decided on 2026-11-12 that, for now, each iteration reports the jamming graph and a render of the latest design as three orthographic views. The report keeps Z per target (schema 4). The pickup report, box scale and predictor cross-check stay out.

- **The views are front (X–Z, looking along +Y), side (Y–Z, from +X) and top (X–Y, looking down),** one row per part, to scale. Each title gives the view's size in mm.
- **They are drawn from the printed solid's mesh,** with its bores and windows, not from the 2D section. Each view keeps the faces that face the viewer and draws them farthest first, so nearer surfaces cover farther ones. Faces are shaded by a light set off the view axis, so faces at different angles read differently.
- **`module.png` replaces its section-and-3D render** with these views. The file name is kept, so the orchestrator and `summary.md` are unchanged.

## #77 Drop videos and STL in the results

The maintainer asked on 2026-11-12 for five random drop videos per iteration, with their positions marked, and the STL of the design.

- **The five drops are picked from the full map's own drops** (the fitted grids, not zooms), with a seed of run seed × 1000 + iteration. A run is then repeatable, and two iterations do not show the same cells. Any outcome can be picked; the user asked for a random pick, not a selection of failures.
- **Each pick is dropped again with recording on, not replayed.** The map keeps contact traces, not frames, and a drop depends only on its pose. The summary gives the re-run's outcome if it ever differs from the map's.
- **Recording is at 240 frames per second of sim time, played at 30 fps (8 times slower), with the last frame held 0.75 s.** At the trial's old 60 fps a drop that settles in 0.27 s was 16 frames, too short to see. `DropRig` gained a `record_fps` setting for this. On the dummy wedge, a video is 65 frames and 2.9 s.
- **The camera looks along +Y at the X–Z profile, the plane the map is drawn in,** pulled back so the falling brick stays in frame. The caption gives the drop number, slice, course, X, release height, pitch and outcome, and the label.
- **Videos are MP4, written through the ffmpeg binary.** Without ffmpeg, a GIF is written instead (Pillow), so no new Python dependency is needed. The renderer is aerial-assembly's (`brixzle/src/brixzle/render.py` @ a649663), adapted.
- **A drop that cannot be rendered keeps its pick with the error,** and the loop carries on. A headless machine without a GL context gets no videos rather than a failed run. Its test is skipped there.
- **The STL is the printed solid** (with bores) from the builders, one file per part, in mm.
- **`videos` in the loop's config (default 5; `--videos 0` turns them off).** They add the re-drops and the rendering to each iteration's time; see the PR for measurements.
