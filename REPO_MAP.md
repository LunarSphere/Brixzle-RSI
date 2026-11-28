# Repo map

What lives where. Update this file in the same PR that adds or moves a top-level path or a module the loop uses.

## The loop (director, #165; error-radius goal, #206)

| Path | What it holds |
|---|---|
| `src/brixzle/director/` | The loop's deterministic steps, `python -m brixzle.director` (`__main__.py`: init, status, baseline, check, verify, decide, guard, finish); the error-radius goal is the only goal (#204); `novelty.py`: the near-copy check `check_family` runs |
| `src/brixzle/director/drive.py` | The loop driven from Python with each role a `codex exec` call (#182): `python -m brixzle.director.drive --run runs/rsi/<name>`; per-role model and effort, role logs and `timeline.jsonl` |
| `src/brixzle/director/run.py`, `guard.py`, `change.py` | Run files and goal settings (`RADIUS_DEFAULTS`); the write/read guard and its PreToolUse hook; the one-change check against the parent |
| `src/brixzle/director/verify.py` | The verifier's own measurement. `measure_radius` (the error-radius goal): check and gripper interface, MVP gate, the quarter screen, the grid score and Z (#222), noisy builds for finalists only, each part saved so a cut-off step resumes |
| `src/brixzle/director/archive.py` | `BandArchive` (best design per Z band, ranked by Z16, then Z28 and Z40, then each placement's green zone weakest first; #206, #222). The #164 Pareto archive went with its goal (#204) |
| `src/brixzle/director/library.py` | The design library (#215): one card per measured design in `runs/library/` (code, maps, scores, strengths and weaknesses, contacts, diagnosis) and `INDEX.md`; `python -m brixzle.director.library add <spec> --grid <dir>` |
| `src/brixzle/director/decide.py` | The director's decisions, the forced-new-family rule and the record of designs tried (`designs.jsonl`) |
| `.claude/agents/rsi-*.md`, `.claude/commands/rsi-loop.md`, `.claude/settings.json` | The four roles' instructions (designer, verifier, critic, director; read by both drivers), the `/rsi-loop` command for Claude Code, and the hook that bounds a run |
| `.codex/agents/` | Codex helper agents for working on this repo (`brick-cad`, `brick-coding`); not loop roles |
| `CLAUDE.md`, `AGENTS.md` | Working rules for Claude Code and Codex in this repo (the same rules) |

## The measurement (`src/brixzle/engine/`)

| Path | What it holds |
|---|---|
| `engine/gridscore.py` | The grid score (#208, #222): per placement (default the four quarters: stack and overhangs 1–3), free drops on an X–Y grid to 40 mm (7.5 mm cells split to 2.5 mm at outcome changes), the score `z` (the mean of each quarter's Z within 16 mm), Z per ring, the green zone `green_mm` as a diagnostic, solver-noise tolerance, the jam analysis of the failures that bound each zone (`jams.json`) and the maps (`map-<placement>.png`); `python -m brixzle.engine.gridscore <family>` |
| `engine/radius.py` | The placement types (`PLACEMENTS`: full, both bare edges, corbel reach, corbel back brick, second and third reach) and the four the score uses (`QUARTERS`, #222), the noisy tower and corbel builds at 16 mm with failure replay (`noisy_builds`, `replay_failures`), and the older r90 probe (#206) |
| `engine/jam_analysis.py` | Jam analysis (#49): reruns at friction 0.2–0.5 with seeded shifts, bands of repeatable failures, contacts at rest reduced to named sites, a 3D check and the smallest holding set; works on course and edge layouts (#208) |
| `engine/gripper_spec.py` | The magnet-target interface every brick designs against (#206); values are guesses until the bench tests (#24) |
| `engine/structure_gate.py`, `structures.py` | The structure gate (#83) and its MVP setting (`MVP`: stack 3, overhang at least 3, held); structure plans (tower, corbel, wall, bridge, ring) |
| `engine/objective.py` | Noisy corbel builds by error radius and their records (#164, #206), used by the finalists' noisy builds; the rest of the #164 objective goes in the strip issue |
| `engine/box_search.py`, `boxes.py`, `drop_cache.py` | `DropPool` (parallel drops, one rig per course per worker), target error boxes, and the drop cache keyed by sim geometry, code, config and pose (#163) |
| `engine/heldmap.py`, `bounce.py` | Held-placement scores for the `held` goal (#192) and bounce-out on impact (#167) |
| `engine/predictor/` | Analytic placement predictor: quasi-static 2D contact stepping and stability tests (Balkcom & Trinkle 2002, Pang & Trinkle 2000); its contact modes power the jam analysis's reduction. `predictor/pickup.py`: the opening pickup predictor (#12) |
| `engine/sim/trials.py` | Drop rigs: course 0 and 1 (`rig_layout`), the half-supported and corbel layouts (`edge_layout`: bare edges, `plus_anchored`, `plus_cantilever`, `plus_cantilever3`, `back_cantilever`), held release, outcome classification |
| `engine/sim/held.py`, `assemble.py`, `ring_build.py`, `scene.py` | The held fork mission (#191), brick-by-brick structure builds, ring builds, and the MuJoCo scene |
| `engine/sim/guards.py`, `drone.py`, `approach_error.py`, `config.py` | Prop-guard collision solids, drone mass and payload, the measured Exp 3 error model, sim constants |
| `engine/sim/pickup.py`, `fork.py`, `funnel.py` | Pickup trial (R12, deferred): pick fixture, the fork, the funnel fixture |
| `engine/sim/arm.py`, `grasp.py`, `pickplace.py` | SO-101 arm, stock-jaw grasp and pick-and-place (#151–#154). Not used by the loop; the SO-101 work has stopped |
| `src/brixzle/tools/` | Deterministic tools for the roles (#163): `python -m brixzle.tools` redrop, contacts, sweep, slices, mapdiff, diffdesign, gate, ring, friction, edgemap, heldmap; `tools/check_family.py`: the contract-and-rules check the designer and verify run; `tools/render.py`, `videos.py`: the module pictures, STLs and drop videos the design library uses; each writes a run directory and prints a short JSON summary |

## Families and rules

| Path | What it holds |
|---|---|
| `src/brixzle/families/interface.py`, `builders.py`, `rules.py`, `contract.py`, `registry.py`, `fingerprint.py` | The family contract, shared builders (solid, collision, mass, base), shared R6/R9/R10 rules, contract checks, family discovery, and geometry digests for checking refactors |
| `src/brixzle/families/capture.py`, `convex.py` | Capture features that vary along Y (Solid3D, #105); convex decomposition |
| `src/brixzle/families/ab/` | The A/B baseline ported from aerial-assembly |
| `src/brixzle/families/twinhook/`, `fable_*/` | The twinhook line (#160, #162): the shared joint and the Fable designs (twinhook4, gentle, gentle_longhook, gentle_tpuhook, feather, reach, wide, allround), each a thin subclass |
| `src/brixzle/families/prim/` | The primitive family the staged search samples (#166) |
| `src/brixzle/families/wedge/`, `pyramid/` | Dummy families for the contract tests (one Prismatic, one Solid3D) |
| `src/brixzle/rules/` | Design rules (`design_rules.md`), each citing its sources, and the citation list (`citations.md`); `__init__.py` parses both. No PDFs: the repo is public |
| `src/brixzle/data/` | Drone error files (`error-exp3-r4.json`, `hover-exp3-r2.json`); `so101/`: the SO-101 model from mujoco_menagerie |

## More of the measurement

| Path | What it holds |
|---|---|
| `engine/quick_check.py` | The quarter screen the director runs before the grid (`quarter_screen`, #222: 40 random free drops within 16 mm, 10 per quarter, at least 32 seated; `python -m brixzle.engine.quick_check --quarters --family <spec>`) and the pre-#206 quick check |
| `engine/sim_map.py`, `zscore.py`, `validate.py`, `pickup_check.py`, `ring_cells.py` | The pre-#206 scoring that `tools/` and the jam analysis still use (fitted sim map, Z over target boxes, predictor validation, the pickup check) and the ring cells. The SDK loop's tune, report, probe and screen were removed (#204) |

## Everything else

| Path | What it holds |
|---|---|
| `docs/` | Requirements (R1–R13) as prose (`requirements.md`) and data (`requirements.yaml`), architecture, the family interface, the campaign runbook and goal (`campaign-206.md`, `campaign-206-goal.txt`), held placement, physical test protocols (#167), predictor validation, decision records, and the 2026-11-16 design-workflow record |
| `clean_results/` | Curated results of earlier designs (A/B to allround), with STL and STEP files and the scripts that made them |
| `scripts/` | One-off generators and baselines: hover estimates, bounce baselines, SO-101 reach, grasp and video (#152, #153) |
| `tests/`, `tests/fixtures/` | pytest suite (`uv run pytest -n auto`, in parallel with pytest-xdist; `-m "not slow"` skips the long sim runs, as CI does on pull requests, #224) and reference results from aerial-assembly |
| `.github/workflows/ci.yml` | CI: the tests on every PR and on pushes to `main` |
| `runs/` | Git-ignored. One folder per run: `runs/rsi/<name>/` for director runs, one `iter-NN/` per iteration |
