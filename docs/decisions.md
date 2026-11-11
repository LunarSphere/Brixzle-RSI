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

## #50 R10 grippable

The R10 wording in `docs/requirements.md` and `docs/requirements.yaml` is the text proposed in the issue, word for word. The issue asks the maintainer to review it, so the PR waits for that.

- **`pickup(p, part)` keeps its name and returns a tuple of `GripFeature(kind, x_mm, z_mm, size_mm, outline=None)`.**
  The issue asks for "a list of generic grip features (type, position, size)". `Bore` and `PickupFeatures` are gone. Keeping the field name avoids touching every family and the `Family` contract for no gain.
- **`kind` is free text, with one reserved meaning: `"bore"`.**
  A bore is a round hole straight through Y that the builders cut, as before. Every other kind is already in the family's geometry. A fixed list of kinds would stop the design agent from offering a feature nobody thought of, which is the point of dropping the fork.
- **A bore carries its hole diameter, not a tine diameter and a clearance.**
  Those two belong to a gripper. A/B's bores are still 2.6 mm (`fork_d + 2 * fork_clearance`), so its solid, mass and collision are unchanged.
- **The loop's R10 rules are `R10.grippable` (features − 1) and `R10.within_payload` (35 g − mass).**
  They state the two sentences of the new wording as margins. The contract also fails a family with no grip feature.
- **The fork checks moved to `rules.fork_margins`, with the tine diameter as an argument.**
  `R10.hangs_stable` and `R10.tines_straddle_com` keep their formulas and still match the aerial-assembly reference for A/B. Nothing in the loop calls them.
- **The contract no longer asks whether a straight 2 mm tine clears each opening.**
  That check assumed the fork. It lives on in `engine/predictor/pickup.py` (`geometry`), which the pickup trial uses.
- **A feature with no outline is checked against the brick's bounding extent, not its outline.**
  A lip sits on the outline's edge, so "inside the outline" would reject it. The extent check still catches a feature placed off the brick.
- **A/B's own `R10.fork_fits` stays in its `extra_rules`.**
  It only says that A/B found room for its two bores, which are its grip features.
