# Architecture

The evaluation engine, the agents, the command-line intake and the orchestrator are built. The intake page (#65) is planned.

## Loop order

Agreed on 2026-11-11 (epic #1). Each iteration runs these steps:

1. **Design.** The design agent makes a new CAD model as a brick family (issue #18).
2. **Quick random check.** 40 evenly spread sim drops inside the target box. Fewer than 32 seated sends the design straight back to step 1 (issue #47, built).
3. **Full sim map** on a grid fitted to the brick (issue #48, built).
4. **Jam analysis** inside the green zone (issue #49, built).
5. **Critic.** The critic agent writes a redesign brief, and the loop returns to step 1 (issue #19).

The loop's one objective is **Z** (issue #61; see [Objective Z](#objective-z)). A redesign replaces the best only if Z on the target improves and a held-out check is not worse. When Z reaches 1 on the target box, or stops improving, a short tune finishes the design (issue #14). The [orchestrator](#orchestrator) runs it all.

## Orchestrator

`loop/orchestrator.py` (issue #21) runs the loop without a person prompting it, and enforces the budget and stop rules:

    uv run --extra agents python -m brixzle.loop --intake runs/intake-<stamp>/intake.json --target mean

Each iteration:

1. **Design.** The design agent writes a package. The loop checks it (contract and rules); a package that fails goes back to the agent with the check's output.
2. **Quick check.** Below 32 of 40 seated, the design goes back to the agent with its failing drops. An iteration gets up to `design_attempts` tries (3).
3. **Full sim map** covering the mean, p95 and worst boxes, and **Z** per target.
4. **Jam analysis.**
5. **Acceptance.** The design replaces the current best only if Z on the target box improves and held-out Z is not worse at either friction. Held-out Z uses the target box's poses shifted by seed + 1, at μ 0.2 and 0.5. The first scored design becomes the best. The critic never sees the held-out results.
6. **Critic.** It writes a redesign brief, and the loop returns to step 1. It is not called when Z is already 1 or on the last iteration.

Every attempt goes in the design log. Every iteration writes `RSI_RESULTS/<run>/iter-NN/` and updates `progress.png`.

**Stop** when the first of these holds, checked before each iteration:

- Z reaches 1 on the target box;
- the critic says stop;
- `max_iterations` have run;
- `patience` iterations in a row have kept no design;
- the API budget could not cover one more design and critic call;
- the wall-clock budget (30 min) is spent.

Then the end tune runs on the best design, with U fixed if intake fixed it.

**Files:**

- `runs/loop/<run>/`: `state.json` (written after every iteration), `design_log.jsonl`, `api-spend.jsonl` (unless `--ledger` names a shared one), and `iter-NN/attempt-K/pkg/<module>/`, each attempt's package;
- `RSI_RESULTS/<run>/`: the renders and summaries;
- `runs/check-…`, `runs/map-…`, `runs/tune-…`: the engine's usual run directories.

**Config** (`loop/config.py`, `LoopConfig`) holds the runtime model (Claude Sonnet 5.5), the target, the iteration, patience, API and wall-clock budgets, the per-call caps, the seed and the worker count. The model is set there and nowhere else.

**Resume.** `--resume <run>` reads `state.json` and carries on from the next iteration. An iteration interrupted part-way is run again in a new folder (`iter-NN-2`), so nothing is overwritten.

**Errors.** An SDK or API error in a design call is logged (`error`) and that attempt is retried. One in the critic leaves the next design without a brief. The spend cap (`BudgetExceeded`) ends the run cleanly, and the end tune still runs.

The loop scores placement (R11) only for now. Pickup (R12) is deferred until the gripper is designed for the chosen brick (issues #46 and #24).

## Per-iteration results

Every outer iteration writes `RSI_RESULTS/<run>/iter-NN/` (issue #15; `loop/render.py`):

- **`jam_map.png`:** the full sim map, one panel per slice and course. Colours follow aerial-assembly's `figures/biasbrick-drop-grid`: seated green, jammed orange, fell red, plus wrong valley in purple, since it is A/B's main failure and would otherwise be hidden. Each legend gives counts. The target box and the jam bands are drawn, Z per target is in the title, and it is labelled "simulated, uncalibrated contact".
- **`module.png`:** the latest design as three orthographic views of its printed solid: front (X–Z), side (Y–Z) and top (X–Y), one row per part, to scale in mm (issue #74).
- **`summary.md`:** Z per target with its counts (and the held-out Z, which the critic never sees); what changed and why (the brief followed, the design agent's note, every attempt); the decision and its reason; the critic's next brief; and the run directories, spend and wall-clock behind every number.

`RSI_RESULTS/<run>/progress.png` shows Z per iteration, with kept designs filled, rejected ones hollow and iterations with no scored design crossed. `RSI_RESULTS/<run>/summary.md` sums up the run. `RSI_RESULTS/` is git-ignored; figures for the repo still go through #25.

## Intake

A person gives one or more concept images and adds extra requirements. Today this is a command (`python -m brixzle.intake`, issue #17); a local web page that writes the same `intake.json` is still planned. One Claude call sorts each extra requirement into one of four kinds:

- compatible;
- refines a base requirement;
- conflicts with a base requirement;
- unclear.

For each conflict or unclear item, the person chooses: keep the base requirement, use the extra instead, drop the extra, or clarify it in their own words. At a terminal the command asks; otherwise each answer is given with `--resolve ID=CHOICE`. Nothing reaches the loop until every conflict is resolved. With no extras there is nothing to sort and no call is made. The output is `runs/intake-<UTC>/intake.json`, with copies of the images beside it. It is the only input to the first iteration, and it may fix the voxel pitch U (R2).

## Talking to Claude

Every Claude call goes through `agents/runner.py`:

- **Model:** set once, by the caller (the loop's config); Claude Sonnet 5.5 by default.
- **Credentials:** `ANTHROPIC_API_KEY` if it is set, otherwise the active `ant` CLI profile's access token, handed to the Agent SDK's Claude Code process. The token is never printed. A call fails if Claude Code reports using any other credential, such as a claude.ai login.
- **Spend:** each call reserves its own cap against a spend ledger (a JSON-lines file with a cap in dollars) before it starts. It gets the Agent SDK's per-call cap and is recorded with the cost the SDK reports.
- **Tools:** each call lists the built-in tools it may use, and a guard refuses writes outside the call's own folder and reads outside its read roots. No settings files or CLAUDE.md are loaded.
- **Tests** use a scripted runner. The Agent SDK is an optional extra (`uv sync --extra agents`), so CI never installs or imports it.

## Design log

`loop/design_log.py` (issue #20) keeps one JSON line per design attempt in `runs/loop/<run>/design_log.jsonl`. That covers a package that broke the contract, a design the quick check turned back (with its counts by outcome), and a design that was scored and kept or rejected (with Z per target and the reason). Map overrides are logged with their reasons. Each entry names the run directories behind its numbers. Both agents read it as short text, so neither repeats an idea that failed. That text never holds the held-out results.

## Agents (Claude Agent SDK)

**Design agent** (`agents/design.py`, issue #18).

- **Prompt:** the base requirements (R1–R13, deferred ones marked), any extras from intake, the brick family interface (`docs/family-interface.md`), the concept images, the critic's last brief, the current best design as a parent, the quick check's failing drops when the last attempt failed, and the design log.
- **Tools:** Read, Glob and Grep over the repo's docs, the engine, the rules, the family interface and the A/B family (not the dummy families, #73); Write and Edit inside its own package only; and `check_family`, which runs the family contract and the rule margins in a separate process with a time limit.
- **Output:** a family package `runs/loop/<run>/iter-NN/attempt-K/pkg/<module>/` with `family.py` and `design_note.json` (the idea, the change from the parent, the geometry type and, for `Solid3D`, the reason). The loop checks the package itself afterwards; it does not take the agent's word.

**Critic agent** (`agents/critic.py`, issue #19).

- **Prompt:** the design rules (`rules/design_rules.md`) and the sources they cite (`rules/citations.md`), with Z per target, the fitted map as text grids, the jam analysis (bands with failure types and mechanisms, and the capture limits), the design log and the family's code. It never sees held-out results.
- **Tools:** none. Everything it needs is in the prompt, and it answers with a structured brief.
- **Output:** a redesign brief. For each failure cluster it gives the diagnosis, the rules it rests on (by id), the proposed change, whether it reshapes the family or needs a new one, and the expected effect on Z. Or it gives a decision to stop, with its reason. A brief that cites a rule that does not exist, or a redesign with no cluster, is refused.

## Evaluation engine (no LLM)

It scores each target box: the drone's mean, 95th-percentile and worst error. For now it scores placement (R11) only. The pickup trial and the pickup predictor (R12) stay in the code with their tests, but the loop does not run them (issue #46).

1. **Analytic predictor.** It is not part of the score (issue #61). The jam analysis uses its stability tests to explain jams, and `engine/validate.py` checks it against the sim. It takes a 2D section of the brick in X, Z and θ. For each pose, it steps through the contacts:
   - find the first contact (configuration-space geometry);
   - decide slide or stick, from the friction cone;
   - find the next contact;
   - at two or more contacts, run a jam test, which asks whether friction can hold the part still.

   It repeats this across μ ≈ 0.2–0.5, and returns a predicted box and contact traces in seconds.
2. **Tuner.** One short, gradient-free Py-BOBYQA run on Z, at the end of the loop and not inside an iteration. See [End tune](#end-tune).
3. **Sim verifier.** MuJoCo drops through the full sim map (below). The older box search (`engine/box_search.py`: bisection on the scale of a box until every drop at its corners, faces and a few interior points succeeds) is kept for the pickup trial and the predictor validation; it is no longer the score.
4. **Scorer (`engine/report.py`).** It runs the quick check, then the full sim map with its grid covering all three target boxes (aimed at the run's target), and counts Z on each box. It writes `map.json`, `drops.json.gz` and `report.json` (schema 4) to one new `runs/map-<UTC>-<family>-<target>/` directory; the jam analysis is written beside them. The report holds Z per target with S, J, N and the fell, wrong-valley and other counts behind it. It holds no box scale and no predictor term.

### Objective Z

Issue #61. For each target box (mean, p95, worst), every drop of the full sim map whose pose lies inside the box counts, across all slices and both courses:

- S = seated, J = jammed, N = every drop in the box;
- **Z = (S − J) / N**: a seated drop counts +1, a jammed one −1, and a fall, a wrong valley or any other outcome 0;
- Z runs from −1 to 1, and **Z = 1 passes**: every drop in the box seated.

Z is counted over the target box, not over the green zone. The box is fixed by the drone's measured error, so a design cannot raise Z by shrinking its green zone. Zoom drops, which are overrides at a finer step, are not counted.

- **Cells in a box.** The map's cells are 1 mm in X, 5 mm in release height and 2° in pitch at U = 25 mm, so the target boxes' ±0.2–0.4° of pitch hold only the zero-pitch row. On A/B the mean box holds 29 X columns: 89 drops per course over the three slices, 178 in all; p95 holds 446 and the worst box 622.
- **Without the full map.** A drop depends only on its pose, so dropping the map's cells inside the boxes alone gives the same Z (`zscore.ZScore`), in about a third of the drops and with no grid fitting. The end tune and the loop's acceptance rule use it.
- **Held-out Z.** Each in-box pose is shifted by a seeded amount (up to half a cell, or the box's half-width if smaller) and kept inside the box, and dropped at friction 0.2 and 0.5 (`zscore.HeldOutZ`). The critic never sees it.

### Quick random check

`engine/quick_check.py` is the go/no-go gate before the full map. It exists so that a weak design is turned back in seconds.

- **Poses:** 40 poses inside the target box, from one seeded Latin hypercube over X, Z, θ and the course. Each axis has one pose in each of its 40 equal slices, and each course gets 20 poses.
- **Physics:** the MuJoCo drop trial, not the predictor.
- **Pass rule:** at least 32 of 40 seated. The check stops at the 9th failure, because 32 is then out of reach. Poses are judged in order, so the result does not depend on how many processes ran it.
- **On a fail:** the failing poses go back to the design agent with their outcomes and contact traces.
- **Output:** `runs/check-<UTC>-<family>-<target>/check.json`, labelled "simulated, uncalibrated contact".

The check is not the score. A design that seats every drop in the box always passes it, so it never turns back a design with Z = 1.

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
- **Guardrail:** the grid never shrinks inside the target boxes it covers (`cover`; the scorer passes all three), and a target box wins over a limit. Z is counted over the target box, whatever else the map covers, so cropping the map cannot raise it.
- **Limits accepted:** the maintainer accepted 1 U of release height and ±10° of pitch on 2026-11-12 (#61).
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
- **Objective:** Z on the run's target box, placement only, from the map's cells inside the boxes (`zscore.ZScore`). The untuned design is scored first, and no point is scored twice.
- **Other targets:** a candidate that lowers Z on any other target box below the untuned design's is refused (`lowers_other`) and can never be the result.
- **Search:** Py-BOBYQA, which fits a model through n + 1 points for n parameters. It starts with a step of 10 % of each range and stops at 1 %.
- **Budget:** 24 scored candidates and 7 minutes of search by default. A candidate is not started if it would run past the time cap. The search also stops once 4 scored candidates in a row, after the first probes, bring no gain.
- **First probes together:** the search begins by stepping each parameter once. Those candidates are scored side by side in 6 processes, and the search then reuses the results.
- **Infeasible candidates:** one that cannot be built, or that breaks a rule margin (R6, R9, R10, R13 or the family's own), is not simulated and can never be the result.
- **Held-out check:** a tuned design that scores higher is scored again, with the untuned one, on held-out Z: the target box's poses shifted by the next seed, at friction 0.2 and 0.5. The search itself runs on the map's own cells at the trial's default friction. The tuned design is accepted only if neither held-out Z got worse; otherwise the untuned design is kept.
- **Not used:** the analytic predictor. A first stage on the predictor (CMA-ES or random search) was planned and then dropped on 2026-11-11, so `Prismatic` and `Solid3D` families are tuned the same way.
- **Output:** `runs/tune-<UTC>-<family>-<target>/tune.json`: every candidate with its score or the reason it was skipped, the untuned and tuned parameters, the held-out scores, why the search stopped, and the wall-clock of the search and the held-out check. Labelled "simulated, uncalibrated contact".

## Files

| Path | Written by | Read by |
|---|---|---|
| `docs/requirements.md` (later a data file) | people | design agent |
| `src/brixzle/rules/design_rules.md`, `src/brixzle/rules/citations.md` | people, with Claude's help | critic agent |
| `src/brixzle/data/error-*.json` | copied from aerial-assembly | evaluation engine |
| `src/brixzle/families/<name>/` | design agent | evaluation engine |
| `runs/loop/<run>/iter-N/` | orchestrator and engine | critic agent (with the design log) |

## Open questions

- The pickup trial (R12) doesn't exist in aerial-assembly yet. Its success test is defined in `docs/requirements.md` under "R12 in detail": capture, self-centring, gentle contact and clean release.
- The Z and θ sides of the target box need real measurements of the drone's height and tilt error.
- The analytic predictor is quasi-static and 2D. Issue #11 checked it against the sim on the A/B baseline; [`predictor-validation.md`](predictor-validation.md) says where it can be trusted (seated or not and the capture edge, yes; the failure class, no).

## Future work

- **Design the gripper for the chosen brick (issue #24).** The gripper is designed after the loop has settled on a brick, not together with it. It can be any non-actuated type (a fork, a hook or something else), and it uses the brick's grip features (R10, issue #50). The pickup trial, the pickup predictor and the checks against today's fork are kept in the code for that step.
