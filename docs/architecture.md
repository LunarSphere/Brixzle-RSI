# Architecture

The evaluation engine is built. The agents, the intake page and the orchestrator are planned.

## Loop order

Agreed on 2026-11-11 (epic #1). Each iteration runs these steps:

1. **Design.** The design agent makes a new CAD model as a brick family (issue #18).
2. **Quick random check.** 40 evenly spread sim drops inside the target box. Fewer than 32 seated sends the design straight back to step 1 (issue #47, built).
3. **Full sim map** on a grid fitted to the brick (issue #48, built).
4. **Jam analysis** inside the green zone (issue #49, built).
5. **Critic.** The critic agent writes a redesign brief, and the loop returns to step 1 (issue #19).

The loop's score is the scorer's box scale for the target (see [Evaluation engine](#evaluation-engine-no-llm)). When it reaches 1, or stops improving, a short tune finishes the design (issue #14).

## Orchestrator

The orchestrator runs each iteration without a person prompting it, and enforces the budget and stop rules. Every iteration is written to `runs/loop/<run>/iter-N/`.

The loop stops when either of these holds:

- the design's placement box covers the target error with no jams;
- the score stops improving, or the budget runs out.

The loop scores placement (R11) only for now. Pickup (R12) is deferred until the gripper is designed for the chosen brick (issues #46 and #24).

## Intake page

A local web page. A person uploads one or more concept images and adds extra requirements. The design agent sorts each extra requirement into one of four kinds:

- compatible;
- refines a base requirement;
- conflicts with a base requirement;
- unclear.

For each conflict or unclear item, it asks the person to choose. Nothing reaches the loop until every conflict is resolved. The output is `intake.json`, the only input to the first iteration.

## Agents (Claude Agent SDK)

**Design agent.**

- **Prompt:** the base requirements (R1–R13), any extras from intake, and the brick family interface (`docs/family-interface.md`).
- **Tools:** write family code, run the rule checks and `contract.check_family`, render CAD.
- **Output:** a family package in `src/brixzle/families/<name>/`.

**Critic agent.**

- **Prompt:** the distilled design rules in `src/brixzle/rules/design_rules.md`. Each rule cites its paper.
- **Tools:** read the report and jam traces, look up the cited papers, read the design log.
- **Output:** a redesign brief, which may reshape the current family or invent a new one; or a decision to stop.

## Evaluation engine (no LLM)

It runs once for each target box: the drone's mean, 95th-percentile and worst error. For now it scores placement (R11) only. The pickup trial and the pickup predictor (R12) stay in the code with their tests, but the loop does not run them (issue #46).

1. **Analytic predictor.** It takes a 2D section of the brick in X, Z and θ. For each pose, it steps through the contacts:
   - find the first contact (configuration-space geometry);
   - decide slide or stick, from the friction cone;
   - find the next contact;
   - at two or more contacts, run a jam test, which asks whether friction can hold the part still.

   It repeats this across μ ≈ 0.2–0.5, and returns a predicted box and contact traces in seconds.
2. **Tuner.** One short, gradient-free Py-BOBYQA run on the simulated box, at the end of the loop and not inside an iteration. See [End tune](#end-tune).
3. **Sim verifier.** MuJoCo drops (and pickups) at the box's corners and faces, plus a few interior points. Bisection on the box's scale finds the largest box in which every trial succeeds. Every contact is logged.
4. **Scorer and diagnoser.**
   - **Score:** the sim box scale for each target, where 1 or more passes. A target's overall score is the smallest over the tasks that were scored. By default that is placement alone (`tasks`, issue #46). With `--tasks placement pickup` it is the smaller of the two, and a failed clean-release check scores pickup 0.
   - **Failures:** placement failures are sorted into jam, fall and wrong valley. Pickup failures, when pickup is scored, are sorted into jam, pickup miss, off-centre and forceful, plus a failed release. The limiting failures, which cap each box, keep their contact traces.
   - **Cross-check:** the predictor is run on the poses the sim dropped, and the report flags where they disagree. *Optimistic* means only the predictor succeeds; *conservative* means only the sim does. The scale gap is recorded for each target. When both fail, their failure classes are compared but not trusted.

   `engine/report.py` writes `report.json` to a new run directory. The report names the tasks it scored. The full sim and predictor results are written beside it.

### Quick random check

`engine/quick_check.py` is the go/no-go gate before the full map. It exists so that a weak design is turned back in seconds.

- **Poses:** 40 poses inside the target box, from one seeded Latin hypercube over X, Z, θ and the course. Each axis has one pose in each of its 40 equal slices, and each course gets 20 poses.
- **Physics:** the MuJoCo drop trial, not the predictor.
- **Pass rule:** at least 32 of 40 seated. The check stops at the 9th failure, because 32 is then out of reach. Poses are judged in order, so the result does not depend on how many processes ran it.
- **On a fail:** the failing poses go back to the design agent with their outcomes and contact traces.
- **Output:** `runs/check-<UTC>-<family>-<target>/check.json`, labelled "simulated, uncalibrated contact".

The check is not the score. A design that seats every drop in the box always passes it, so it never turns back a design that would score 1.

### Full sim map

`engine/sim_map.py` maps a design that passed the quick check. It drops the brick over three slices through the aim point, on both courses, and records seated, jammed, wrong valley and fell for every cell.

| Slice | Axes | Held at the aim |
|---|---|---|
| `xz` | X offset against release height | zero pitch |
| `xt` | X offset against pitch | the target box's centre height |
| `zt` | release height against pitch | zero X offset |

- **Coordinates:** X is the offset about the aim. Z is the release height above the minimum release clearance, so it never goes below 0. θ is pitch.
- **Cells:** U/25 in X (1 mm for a 25 mm voxel), U/5 in Z and 2° in θ. Sizes in X and Z follow the voxel pitch U, so bricks of different sizes get maps with about the same number of cells. Extents are reported in U as well as in mm.
- **Fitting,** in code (`GridFit`):
  1. start from the quick check's drops, widened to the target box, plus a one-cell margin;
  2. grow any edge that still has a seated cell on it, until it has none or reaches the axis limit;
  3. trim outer rows and columns with no seated cell, keeping one such row past the last green one. Where the misses start is the capture limit.
- **Limits:** ±2 U in X, 1 U of release height and ±10° of pitch. An edge that is still green at a limit is marked `limit` in the map, not left looking closed.
- **Guardrail:** the grid never shrinks inside the target box, and the target box wins over a limit. The score stays the scorer's box scale over the target box, whatever the map covers, so cropping the map cannot raise a score.
- **Overrides:** a caller may add a zoom (an extra, finer grid over part of a slice) or change the grid settings. Either needs a reason, and both are written to `overrides` in `map.json` for the design log (issue #20). A zoom never replaces a fitted slice.
- **Inside the green zone:** each grid lists its failing cells that lie inside the zone, for the jam analysis (issue #49). A failure beyond the zone's edge is the capture limit, not a jam inside it.
- **Output:** `runs/map-<UTC>-<family>-<target>/map.json` holds the grids. `drops.json.gz` beside it holds every drop with its contact trace. Both are labelled "simulated, uncalibrated contact".

### Jam analysis

`engine/jam_analysis.py` takes the failing cells inside the map's green zone and tells the critic which are real and what holds the brick. The failure type (jammed, wrong valley, fell) comes from the sim. The mechanism comes from the predictor's stability tests, applied to the sim's contacts.

1. **Repeatable or one-off.** Each cell is re-run with 3 seeds at each friction coefficient (0.2, 0.3, 0.4, 0.5). A seed shifts the pose by up to 5% of a map cell. A cell is repeatable if it fails at least two thirds of the seeds at some friction; otherwise it is a one-off (solver noise).
2. **Bands.** Repeatable cells that touch, on one slice and course, form a band.
3. **Contacts.** The drop nearest each band's centre is run again, and its contacts at rest are reduced:
   - keep the contacts between the brick and what it rests on that still touch;
   - flatten them to the X–Z plane and merge points that lie together into sites. A flat patch becomes its two end points, the rule the predictor uses. A site must carry more than 5% of the brick's weight;
   - if the brick rests out of plane (more than 1 mm in Y, or rolled or yawed more than 2°), or more than a quarter of the sites' force is along Y, the band is a **3D effect** and the reduction stops;
   - otherwise find the smallest set of sites whose friction can balance gravity, with the predictor's weak-stability test at the sim's friction. MuJoCo's own split of the load is not trusted;
   - name each contact: which face or corner of the falling brick touches which face or corner below, with the predictor's stick margin and the Whitney (1982) wedging test.
4. **Reading the result:**
   - one contact: stuck on a face too shallow for its friction (R6);
   - two contacts: both sticking (Balkcom & Trinkle 2002), and wedged if they push from opposite sides with the line between them inside both friction cones (Whitney 1982);
   - three or more: reported as found;
   - a set that holds without friction is a stable rest away from the seat, not a friction jam.

**Capture limits (issue #56).** A zone with no jam inside it can still be too narrow for the target box. So on each slice with an X axis, in the row through the aim, the first failing cell past each end of the green band is dropped again and reduced the same way. It is also dropped once at each friction coefficient, to show whether the edge moves with friction. The result is under `capture_limits`, one entry per slice, course and side.

**Output:** `jam_analysis.json` in the map's run directory. Sim results are labelled "simulated, uncalibrated contact" and the mechanism "predicted, quasi-static 2D".

### End tune

`engine/tune.py` runs once, on the family the loop settled on (issue #14). The loop itself changes shapes, not numbers; this last step moves the numbers.

- **Variables:** the parameters in the family's `bounds`, each scaled to 0–1 over its range. A caller can fix any of them, for example the cell size U when it was set at intake (R2).
- **Objective:** the scorer's sim box scale for the run's target box, placement only. The untuned design is scored first, and no point is scored twice.
- **Search:** Py-BOBYQA, which fits a model through n + 1 points for n parameters. It starts with a step of 10 % of each range and stops at 1 %.
- **Budget:** 24 scored candidates and 7 minutes of search by default. A candidate is not started if it would run past the time cap.
- **Infeasible candidates:** one that cannot be built, or that breaks a rule margin (R6, R9, R10, R13 or the family's own), is not simulated and can never be the result.
- **Held-out check:** a tuned design that scores higher is scored again, with the untuned one, on the next seed's interior points at friction 0.2 and 0.5. The search itself runs at the trial's default friction. The tuned design is accepted only if neither score got worse; otherwise the untuned design is kept.
- **Not used:** the analytic predictor. A first stage on the predictor (CMA-ES or random search) was planned and then dropped on 2026-11-11, so `Prismatic` and `Solid3D` families are tuned the same way.
- **Output:** `runs/tune-<UTC>-<family>-<target>/tune.json`: every candidate with its score or the reason it was skipped, the untuned and tuned parameters, the held-out scores, and the wall-clock of the search and the held-out check. Labelled "simulated, uncalibrated contact".

## Files

| Path | Written by | Read by |
|---|---|---|
| `docs/requirements.md` (later a data file) | people | design agent |
| `src/brixzle/rules/design_rules.md`, `src/brixzle/rules/papers/` | people, with Claude's help | critic agent |
| `src/brixzle/data/error-*.json` | copied from aerial-assembly | evaluation engine |
| `src/brixzle/families/<name>/` | design agent | evaluation engine |
| `runs/loop/<run>/iter-N/` | orchestrator and engine | critic agent (with the design log) |

## Open questions

- The pickup trial (R12) doesn't exist in aerial-assembly yet. Its success test is defined in `docs/requirements.md` under "R12 in detail": capture, self-centring, gentle contact and clean release.
- The Z and θ sides of the target box need real measurements of the drone's height and tilt error.
- The analytic predictor is quasi-static and 2D. Issue #11 checked it against the sim on the A/B baseline; [`predictor-validation.md`](predictor-validation.md) says where it can be trusted (seated or not and the capture edge, yes; the failure class, no).

## Future work

- **Co-design the gripper.** Let the loop redesign the passive gripper together with the brick. This covers forks, hooks or any other non-actuated type, and the pick fixture's guides. For now the fork is fixed and only the brick is redesigned.
