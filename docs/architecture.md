# Architecture

The director loop (#165) and the evaluation engine it measures with. The Agent SDK loop, its agents, the intake and the orchestrator were removed in #204 (tag `pre-debloat-2026-11-28` has them).

## The error-radius goal (#206)

Since #206 a new director run (`python -m brixzle.director init`, `director.drive`) defaults to this goal; the `held` and #164 goals remain for old runs (`settings.stage`). The gripper and the landing pad are designed by hand outside the loop; the loop designs bricks only.

- **Score** (`engine/gridscore.py`, #222; the grid since #208): **Z = ¼ Z_stack + ¼ Z_overhang1 + ¼ Z_overhang2 + ¼ Z_overhang3**, each Z = (S - J) / N over that placement's grid cells whose centre lies within 16 mm (`GridSettings.z_ring_mm`), by cell area: seated +1, jammed -1, wrong valley and fell 0. The four placements are `radius.QUARTERS`: the full seat (`full`, stacking) and the reaching brick of a back-first corbel's courses 1, 2 and 3 (`corbel_reach`, `corbel_reach2`, `corbel_reach3`; trials layouts `plus_anchored`, `plus_cantilever`, `plus_cantilever3`), each dropped with every brick below it, and its course's back brick, ideally placed. The result carries `z`, `z_quarters` and `z_rings` (the same mean over the 16, 28 and 40 mm discs). `green_mm`, the largest disc radius about the aim inside which every release seats on every gridded placement, is kept as a diagnostic; the bare edges and the corbel's back brick can still be gridded with `--placements`. Drops on an X-Y grid out to 40 mm: 7.5 mm coarse cells, then any coarse cell touching a different outcome splits into its 3 x 3 fine cells (2.5 mm) around its own centre, so its drop is reused and every drop lies on one 2.5 mm lattice; this repeats until nothing is left to split. Plains of one outcome stay coarse. Once a placement fails within 16 mm, cells past its nearest failure are refined only if they reach inside the 16 mm disc Z is counted over (early stop, #222). The zone ends at the near edge of the nearest failing cell. The nearest failure is rerun at friction 0.2-0.5 with 3 seeded shifts; if it does not repeat (jam_analysis's rule) it is tolerated as solver noise, even if its neighbours failed too, and the search continues with the next nearest until one repeats (maintainer, 2026-11-26; #218 widened it from lone cells). Z = (S - J) / N per ring (16, 28, 40 mm, by cell area). All placements share one `DropPool` with a worker per core and the drop cache. On A/B under #208's six placements and held release (`runs/grid208-20261126-090821-ab-all`, 14 workers, simulated, uncalibrated contact): 1023 drops (362 from the cache) in 44.9 s, of which coarse 26.1 s, refinement 2.4 s and jam analysis 12.1 s; the green zone is 3.75 mm (corbel reach and both bare edges) and 6.25 mm (full seat). `python -m brixzle.engine.gridscore <family>` runs it alone and writes `grid.json`, `jams.json`, `drops.json.gz` (for `tools contacts`) and `map-<placement>.png`.
- **Jam analysis on the grid** (`gridscore.analyse_jams`, reusing `engine/jam_analysis.py`, #49): per placement, the failing fine cells within two fine cells of the green zone's edge (the nearest six) are rerun at each friction with seeded shifts, the repeatable ones are grouped into bands of touching cells, and each band's representative drop is reduced to its contact sites, named on both bodies, with the holding set and the mechanism. The frames now handle the bare-edge and corbel layouts (`trials.edge_layout`, with the base shifted v0 voxels). A brick resting out of plane is a "3D effect": the 2D holding-set reduction stops, but its sites are still named.
- **3D contacts (#214):** for a family that varies along Y, the jam analysis cannot reduce contacts to an X-Z slice, so `jam_analysis.contact_sites_3d` reports the touching contacts at rest instead: per body touched, points within 2 mm merged into a site, its position in the falling brick's own frame, its mean normal and its share of the weight. They are in `jams.json` (`contacts_3d`), the grid's jam summary and the build-failure replays.
- **The design library (#215):** `runs/library/` (git-ignored, shared by every worktree through the `runs/` link) holds one folder per design measured: its `family.py` (and the shared base a thin subclass needs), `card.md` (the idea, Z, Z and green zone per placement, strengths and weaknesses with numbers, the jam bands with their named contacts, the critic's diagnosis and the archive's verdict), `scores.json`, the maps, `jams.json`, three orthographic views and the STL. `INDEX.md` lists them all, best Z first (#222; an entry scored before #222 has no Z and is listed last until it is rescored). `decide` adds each iteration when it records it (`director/library.add_iteration`); `python -m brixzle.director.library add <spec> --grid <dir>` adds a registry family or any grid run. The roles read it; no entry is ever a parent, so a campaign starts from scratch knowing everything tried so far without a lineage to grind on.
- **The whole corbel (#210):** two more grid placements (since #222 the second reach is in the score, the back brick a diagnostic, and the third reach `corbel_reach3` was added) cover the corbel past its first reach: `corbel_back`, a back brick from course 2 up, landing across the back brick below and the reach that cantilevers past it (`trials` layout `back_cantilever`), and `corbel_reach2`, the second reach (`plus_cantilever`). The noisy builds run at a fixed 16 mm (`BuildSettings.radius_mm`), not at the design's green zone, so overhang and stack mean the same for every design. Each corbel build's failed brick keeps its release error and outcome, and verify replays it on the single-brick layout for its place in the corbel (`radius.corbel_layout`, `radius.replay_failures`): contacts at rest reduced and named as in the jam analysis, and whether the failure repeats on the ideal layout or needed the build's accumulated error. The band archive breaks a green-zone tie on the second-weakest placement's green zone (`second_mm`) before overhang. On A/B (`runs/grid210-20261126-104850-ab`, simulated, uncalibrated contact): the second reach is now the weakest placement at 1.77 mm, the back brick 6.25 mm; six placements took 117.8 s with a cold cache while a campaign shared the 14 cores; at 16 mm all 7 corbel builds failed on a course-0 or course-1 brick and every failure repeated on its ideal layout. Adding the layout changed the drop cache's geometry key, so earlier cached drops miss once.
- **r90** (`engine/radius.radius_search`, #206) is no longer part of verify: random disc samples miss jam pockets, and the ring Z values carry its seat-rate signal. It stays as a standalone probe, and `radius.py` still holds the placements and the noisy builds.
- **Hover and drop (#218):** the score, the noisy builds and the MVP gate release every brick in a free drop (`GridSettings.held`, `BuildSettings.held` and `MVP.held` False) from 11.2 mm, the top of the drone's hover wobble (`GridSettings.z_frac` 1.0). The held release's descent speed, stop threshold and dwell were guesses, and the reaching placements turned on them: for `allround`, a 0.05 s versus a 0.5 s dwell moved plus_bare from 24.0 to 7.3 mm and corbel_reach from 3.75 to 8.8 mm (`runs/grid210-20261126-111949-fable_allround`, `runs/dwell-20261126-121511`). With a free drop, `allround` keeps 23.8 mm on the full seat and 23.3 mm on the corbel back brick, but its bare edges and corbel reaches fall to 1.25 mm (`runs/freedrop-20261126-122146`, simulated, uncalibrated contact). On the bench a printed `allround` dropped onto a full seat seats almost always (maintainer, 2026-11-26); reaching bricks have not been drop-tested yet.
- **Held release** (`trials.TrialConfig.held`, the #206 default, kept for comparison): the gripper holds the brick's attitude and lowers it at no more than 0.1 m/s until it stops sinking, then lets go; sideways, contacts steer it, as a compliant mount would. It was the default for the score, the noisy builds and the MVP gate until #218. The yaw that shears the magnet off is not simulated (design rule DR18). On A/B at 8 mm (8 drops each, simulated, uncalibrated contact) the bare edges and the corbel reach seated 3/8, 3/8 and 2/8 by free drop and 5/8, 7/8 and 4/8 held; the full seat 8/8 both ways.
- **Verify** (`director/verify.measure_radius`), cheapest first, each part saved so a cut-off step resumes: the family check and the gripper interface (`engine/gripper_spec.py`, guesses until the bench tests); the MVP gate (`structure_gate.MVP`: a stack of 3 and a back-first overhang of at least 3, ideal placement, free drop); the quarter screen (`quick_check.quarter_screen`, #222: 40 seeded random free drops within 16 mm, 10 per quarter interleaved, at least 32 seated, stopping at the 9th failure; a design that fails it gets the violation `screen`, keeps the screen's Z and is not gridded); the grid score on the four quarters, with its jam analysis and maps in `measured/grid/`; and, only when the campaign sets `run_builds` (finalists, #222), noisy builds at 16 mm of error for a brick that passed the gate (`engine/radius.noisy_builds`: the median stack height and overhang length). Each stage's wall-clock is in `measured.stage_seconds`.
- **Archive by Z band** (`director/archive.BandArchive`, #222): the best design in each band of Z (below 0, 0-0.5, 0.5-0.8, 0.8-0.95, 0.95+). Within a band Z16 decides; a difference under 0.02 (`Z_TIE`, about one fine cell's share of the 16 mm disc) is a tie, broken by Z28, then Z40, then each placement's green zone weakest first (leximin, #214, ties within 1.25 mm). A record without Z (scored before #222) is refused for a rescore. A new idea competes in its own band. Before #222 the bands were green-zone bands (below 16, 16-22, 22-28, 28-34, 34+ mm) and overhang and stack from the noisy builds broke ties.
- **New families** (`director/decide.explore_due`): `decide` refuses a decision that is not a new family (`design: ""`, `may_change: ["new_family"]`) when the archive is empty, every `explore_every`-th iteration (4 since #214; 3 before), and after `stall_explore` (3; 2 before) iterations in a row that added nothing to the archive. The first campaign (`runs/rsi/radius-scratch-1`) forced a new family in 5 of 6 briefs under the old values.
- **From scratch:** `init --baseline none` scores nothing at iteration 0; the first iteration is a new family.
- **The record of designs tried** (`decide.log_design`): when the director decides on an iteration, its design, score, violations, the archive's verdict and the critic's `Mechanism:` line are appended to `<run>/designs.jsonl` and to the shared, git-ignored `runs/designs.jsonl`, which later runs' directors read. Each line names the rules it was judged under (`"rules": "206"`).

## The self-steering loop

Issue #165. The twinhook line beat the SDK loop because the maintainer steered between rounds: they changed the goal as evidence came in, kept each step to one change, and had every claim rerun (`docs/november-16-2026-concise.md`). This loop automates that steering. It runs inside Claude Code, with no Agent SDK and no API calls:

    /rsi-loop runs/rsi/<name> --new "<goal>" [--baseline ab] [--iterations N] [--minutes M] [--replay] [--unattended]
    claude -p "/rsi-loop runs/rsi/<name> --new '<goal>' --unattended"       # headless, same command

**Subagents and checkpoint files.** One Claude Code session runs `.claude/commands/rsi-loop.md`. It spawns one subagent per role, each with its own instructions in `.claude/agents/` and only the paths it needs:

| Role | Reads | Writes |
|---|---|---|
| designer | `iter-NN/brief.json`, the parent's `family.py`, the tools | `iter-NN/<module>/`, `design_note.md`, `claim.json` |
| verifier | the note, `verified.json`, cited runs | `audit.md` |
| critic | `verified.json`, the grid's maps and `jams.json`, and the tools' contacts | `diagnosis.md` |
| director | `goal.json`, `STATE.md`, `archive.json`, `decisions.jsonl`, the iteration's files | `STATE.md`, `iter-NN/decision.json` |

Separate subagents give each role a clean context, so the critic never grades work it wrote.

Every step's output is a file in the run directory, and `next.json` names the next step. A stopped or context-full session resumes with `/rsi-loop <run>` and loses at most the step in flight.

**Deterministic steps** (`python -m brixzle.director`, `src/brixzle/director/`):

- `init` writes the run (`goal.json`, `STATE.md`, `decisions.jsonl`, `archive.json`, `next.json`) and the lock.
- `baseline` scores the baseline and seeds the archive.
- `check` is the one-change check.
- `verify` measures the candidate itself with `tools.evaluate` and the #164 objective, compares the designer's claim number by number, and offers the design to the Pareto archive.
- `decide` validates and logs the director's decision and writes the next brief.
- `guard` checks the tracked tree.
- `finish` ends the run.

**Bounds**, enforced by code:

- **Writes only inside the run.** While `.rsi-loop-active` (the lock) exists, a PreToolUse hook (`.claude/settings.json`, `director/guard.py`) refuses any write outside the run directory. That covers `docs/requirements.md`, `rules/`, the scorer, the sim, the caches, `.claude/` and the lock. It also refuses a Bash command that writes outside the run, and reads of the lock's `deny_read` paths (a replay hides the designs it replays). Without a lock, the hook does nothing.
- **The tracked tree must not change.** `guard` runs after every iteration and ends the run on any change.
- **One change per iteration.** The director's decision names what the next design may change, as 1 to 3 units (`param:<field>`, `def:<name>`, `const:<name>`). `check` compares the candidate's module with its parent's, definition by definition, and refuses any other change. The rest stays frozen.
- **Verified before acted on.** The designer's numbers are rerun by `verify`, and its prose is audited by the verifier. The director is told not to build on a design whose claim did not match.
- **Goal changes only tighten, with evidence.** A goal change needs run directories that exist, and may only tighten an objective setting. Each goes in `decisions.jsonl` and `goal.json`'s history.
- **Budgets.** Iterations and wall-clock are set at `init`. The director stops after three iterations that add nothing to the archive.
- **Human checkpoints are optional.** At the iterations given by `--checkpoints`, the loop stops and summarises, unless it was started `--unattended`.

**Acceptance** is the band archive (`director/archive.BandArchive`; see [The error-radius goal](#the-error-radius-goal-206)): the best verified, feasible design per Z band. The #164 goal and its Pareto archive were removed in #204.

## Tools for the mechanical steps

Issue #163. The hand-run twinhook line spent LLM turns on steps that are mechanical: re-dropping failing poses, reading rest contacts, splitting Z by slice, comparing two maps, sweeping one parameter, and running the stages by hand-written script. `python -m brixzle.tools <tool>` does each one deterministically. Each tool writes `runs/<tool>-<UTC>-<family>/result.json` and prints a short JSON summary for an agent to read.

| Tool | Does | Sim |
|---|---|---|
| `redrop` | Chosen drops of a grid run (`drops.json.gz`, by outcome, placement, a seeded sample), at a friction | yes, cached |
| `edgemap` | Fixed drop grids for both half-supported edges and the first two reaching corbel seats (#172) | yes, cached |
| `contacts` | The same, with each drop's rest pose and its contacts grouped by the pair touching | yes, cached |
| `sweep` | One parameter over a list of values, on a grid run's drops; a value the family refuses is reported | yes, cached |
| `diffdesign` | Changed parameters and fingerprint items between two designs (#162) | no |
| `gate`, `ring` | The stages alone | yes, cached |

A family is a registry name or `<root>:<module>`.

**Half-supported seats (#172).** `edgemap` measures both bare edges on the same 87 poses and the anchored and cantilevered positive-X corbel seats on 13 poses each. The bare map resolves offsets near the aim at its nominal height, with sparse outer cells and lower and higher release heights. Each row keeps the layout, error about aim, signed world-space release displacement and exact input sample. Tipped jams count toward J. To diagnose an edge, use `contacts <family> --map <edgemap-run> --edge-case <layout> --outcomes jammed`. The intended push target is the lower of the two bare-edge Z values. Keep the sequential noisy corbel and the structure gate as build evidence.

The earlier allround campaign shifted the centre of mass toward opposite sides to help the two edges individually. That trade-off is family-specific (`runs/opus/campaign-log.md`). Treat a change that improves one edge as a hypothesis until the other edge has been measured as well.


**The drop cache** (`engine/drop_cache.py`). A drop depends only on what the rigs build and on its pose. So `DropPool` reads a drop that was already simulated for the same key and stores new ones, when `BRIXZLE_DROP_CACHE` is set (the tools set it). The key covers:

- the sim geometry: each part's collision pieces, mass, centre of mass and inertia, the lattice, the release lift, both rigs' seats, and their bases;
- the source of `engine/sim/`, `families/builders.py` and `engine/box_search.py`;
- the trial config;
- the job: course, pose and options.

Names and parameter values are not in the key. Two designs that build the same scene share drops, for example the #162 port and refactor.

Gate and ring results are kept whole, by the family's fingerprint without parameter values, bounds or rule margins, together with the engine's code. A result read back is marked `cached_from`.

Unset, as in the loop and the tests, every drop is simulated. A cached result is an earlier sim result (simulated, uncalibrated contact), not a new one.

## Evaluation engine (no LLM)

The score is the grid (see [The error-radius goal](#the-error-radius-goal-206)): free drops on an X-Y grid per placement, Z within 16 mm averaged over the stack and overhangs 1-3 (#222), with the quarter screen before it (`engine/quick_check.py`). Placement only (R11): pickup is the landing pad's job (R12, not planned), and the pickup trial was removed in #227.

1. **Analytic predictor.** It is not part of the score (issue #61). The jam analysis uses its stability tests to explain jams. It takes a 2D section of the brick in X, Z and θ. For each pose, it steps through
   - find the first contact (configuration-space geometry);
   - decide slide or stick, from the friction cone;
   - find the next contact;
   - at two or more contacts, run a jam test, which asks whether friction can hold the part still.

   It repeats this across μ ≈ 0.2–0.5 in seconds. Issue #11 checked it against the sim ([`predictor-validation.md`](predictor-validation.md); the check's code, `engine/validate.py`, was removed in #227 and is in the tag `pre-debloat-2026-11-28`).
2. **Sim.** MuJoCo drops (`engine/sim/trials.DropRig`) in parallel worker processes (`box_search.DropPool`), with the drop cache. The older bisection on a box's scale (`box_search.box_search`) is kept; the fitted sim map and Z over target boxes of the first loop were removed in #227.

### Jam analysis

`engine/jam_analysis.py` takes the grid's failing cells nearest each placement's aim (`gridscore.jam_candidates`) and tells the critic which are real and what holds the brick. The failure type (jammed, wrong valley, fell) comes from the sim. The mechanism comes from the predictor's stability tests, applied to the sim's contacts.

A family that varies along Y (any `Solid3D`, issue #105) gets steps 1 and 2 as usual, but step 3 stops at the sim: each band reports the failure type, the rest pose (`rest_pose`), the contact count and the share of force along Y, with no sites, no holding set and no mechanism (`sim_only`). A 2D reduction of one XZ slice does not describe such a brick, and the summary says so.

1. **Repeatable or one-off.** Each cell is re-run with 3 seeds at each friction coefficient (0.2, 0.3, 0.4, 0.5). A seed shifts the pose by up to 5% of a fine grid cell. A cell is repeatable if it fails at least two thirds of the seeds at some friction; otherwise it is a one-off (solver noise).
2. **Bands.** Repeatable cells that touch, on one placement, form a band.
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


**Output:** `jams.json` beside the grid (`gridscore.analyse_jams`). Sim results are labelled "simulated, uncalibrated contact" and the mechanism "predicted, quasi-static 2D".

### How the gate builds (#137)

Every gate build aims each drop at its supporter's measured pose (`GateSettings.aim_at_supporter`, default on; the drone is assumed to place relative to the brick it builds on), and the overhang is built back-first, as the maintainer's lab builds: in course k a back brick at voxel k − 2, then the reaching-out brick at voxel k (`structures.corbel`, placed in that order). `overhang_n` is the number of reaching-out bricks seated from course 0 up (the majority over repeats). `overhang_build = "staircase"` and `aim_at_supporter = False` give the old gate. A/B's gate overhang went from 5 to 9 and its bridge from 3/5 to 5/5; the gate takes about 60 s for A/B, against about 27 s before.

### Ring build

Issue #89. `engine/sim/ring_build.py` builds the ring a family's `RingKit` plans (see `docs/family-interface.md`), course by course, with the scene's bodies from `builders.ring_bodies` and the kit's base as static pieces. Each brick is released `release_lift_mm` above its seat, judged against its seat carried on the brick below, and the build stops at the first brick that does not seat or at a collapse (#81's test). A family with no kit gives `no_ring_kit`. `engine/ring_cells.py` ports aerial-assembly's ring (turn piece, corner straight, closer, the column check and the assembly order) as `kit_from_cell`. Output: `runs/ring-<UTC>-<family>/ring.json`, labelled "simulated, uncalibrated contact". `python -m brixzle.engine.sim.ring_build --family ab --courses 2`.

### Structure builds

Issue #81. `engine/structures.py` and `engine/sim/assemble.py` port aerial-assembly's trial structures and its sequential assembly trial to the family interface.

- **Structures** (`structures.py`):
  - `tower(height)`: a stack bond.
  - `overhang(n, direction)`: one brick per course, stepping one voxel past a two-voxel base.
  - `wall(width, height)`: a running bond.
  - `bridge(arm_courses)`: two corbelled arms closed by a key brick.

  Each lists its bricks as `Placement`s and its anchored bases as voxel ranges. They are laid out for a two-voxel brick; `structure_for` raises `ValueError` for any other `span_voxels`.
- **Order** (`assembly_order`): supporters first, lowest course first, and a brick only when its nominal approach is clear of the bricks already placed. The approach is a vertical drop, then the final stroke of length `release_lift_mm` along `insertion_axis`. The check uses the brick's section outline at its seat, which must be upright or turned 180° about Z.
- **Assembly** (`assemble`): each brick is released above its seat, with the error model's sample, and judged against its seat on its actual supporter. The build stops at the first brick that does not seat, or at a collapse: a placed brick leaves its seat on its supporter (judged against the supporter's actual pose), or a brick touches the floor (#135; before, a placed brick moving more than 2 mm in one stage or sag beyond half a course also counted). Sag and stage motion are reported, not judged. It returns the status, the bricks placed and stable, the largest sag, each stage and the wall-clock.
- **Repeats** (`repeat_builds`, `repeat_structures`): a structure is judged on 5 seeded builds, each brick's release moved by a seeded offset of up to 0.05 mm in X and Y, and passes when most complete. An ideal drop onto a symmetric seat can tip either way on numerical noise (issue #98: A/B's bridge flipped between 7 of 7 and 3 of 7 when friction moved by 5e-5), so one deterministic build is not a reliable verdict. The repeats run in parallel processes.
- **Push** (`push_test`): an ideal build, then a ramped sideways force on the top brick until it moves (R3).
- **Output:** `runs/builds-<UTC>-<family>-<structure>/builds.json` (every repeat) or, for one build, `runs/build-…/build.json`, with the family, parameters, seeds and config, labelled "simulated, uncalibrated contact". `python -m brixzle.engine.sim.assemble <structure> --family <name> --ideal` runs the repeats (`--repeats 1` builds once, without jitter).

Only ideal placement is reported so far.

## Files

| Path | Written by | Read by |
|---|---|---|
| `docs/requirements.md`, `requirements.yaml` | people | the roles |
| `src/brixzle/rules/design_rules.md`, `src/brixzle/rules/citations.md` | people, with Claude's help | designer and critic |
| `src/brixzle/data/error-*.json` | copied from aerial-assembly | evaluation engine |
| `src/brixzle/families/<name>/` | people (the reference designs) | evaluation engine; a run's designs live in `runs/rsi/<run>/iter-NN/` |
| `runs/rsi/<run>/iter-NN/` | the director loop's roles and steps | the next roles, and the design library |

## Open questions

- The Z and θ sides of the target box need real measurements of the drone's height and tilt error.
- The analytic predictor is quasi-static and 2D. Issue #11 checked it against the sim on the A/B baseline; [`predictor-validation.md`](predictor-validation.md) says where it can be trusted (seated or not and the capture edge, yes; the failure class, no).

## Future work

- **Design the gripper for the chosen brick (issue #24).** The gripper is designed after the loop has settled on a brick, not together with it. It can be any non-actuated type (a fork, a hook or something else), and it uses the brick's grip features (R10, issue #50). The pickup trial, the pickup predictor and the checks against today's fork are kept in the code for that step.
