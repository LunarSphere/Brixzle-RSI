# Architecture

The evaluation engine is built. The agents, the intake page and the orchestrator are planned.

## Loop order

Agreed on 2026-11-11 (epic #1). Each iteration runs these steps:

1. **Design.** The design agent makes a new CAD model as a brick family (issue #18).
2. **Quick random check.** 40 evenly spread sim drops inside the target box. Fewer than 32 seated sends the design straight back to step 1 (issue #47, built).
3. **Full sim map** on a grid fitted to the brick (issue #48).
4. **Jam analysis** inside the green zone (issue #49).
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
2. **Tuner.** Gradient-free, in two stages:
   - CMA-ES or random search on the predicted box, over thousands of candidates;
   - then a short Py-BOBYQA run on the simulated box.
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
