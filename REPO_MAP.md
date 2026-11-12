# Repo map

What lives where. Update this file in the same PR that adds or moves a top-level path.

| Path | What it holds |
|---|---|
| `src/brixzle/` | The `brixzle` package |
| `src/brixzle/intake/` | Local web page: concept sketch and extra requirements, with a conflict check against R1–R13 |
| `src/brixzle/agents/` | Design agent and critic agent (Claude Agent SDK) |
| `src/brixzle/engine/` | Analytic predictor, tuner, sim verifier, scorer and diagnoser |
| `src/brixzle/engine/sim/` | MuJoCo sim core ported from aerial-assembly: scene, drop trial, Tolerance and classify, Exp 3 error model |
| `src/brixzle/engine/sim/pickup.py`, `fork.py`, `drone.py` | Pickup trial (R12): pick fixture, bored brick collision, the fixed fork, and the drone's mass, thrust margin and hover spring |
| `src/brixzle/engine/boxes.py`, `box_search.py` | Target error boxes (R11, R12) and the box-scale search for placement and pickup |
| `src/brixzle/engine/quick_check.py` | Quick random check (issue #47): 40 evenly spread sim drops inside a target box, 32 seated to pass, stopping at the 9th failure; writes `check.json` with the failing poses and their contact traces |
| `src/brixzle/engine/sim_map.py` | Full sim map on a fitted grid (issue #48): seated, jammed, wrong valley and fell over X-Z, X-theta and Z-theta slices through the aim, on a grid that grows along green edges, trims red rows and never shrinks inside the target box; writes `map.json` and `drops.json.gz` |
| `src/brixzle/engine/jam_analysis.py` | Jam analysis inside the green zone (issue #49): re-runs each failing cell of the map with seeds and friction 0.2 to 0.5, groups the repeatable ones into bands, and reduces each band's contacts at rest to named sites, a 3D check and the smallest holding set. It reduces the first failure past each X edge of the zone the same way (capture limits, issue #56). Writes `jam_analysis.json` beside the map |
| `src/brixzle/engine/predictor/` | Analytic placement predictor (R11): quasi-static 2D contact stepping, contact-mode LPs and stability tests (Balkcom & Trinkle 2002, Pang & Trinkle 2000), predicted box and map |
| `src/brixzle/engine/predictor/pickup.py` | Opening pickup predictor (R12, issue #12): capture through the fork openings along the whole depth (band shear and yaw), and self-centring by sliding up the opening's ceiling; the predicted pickup box |
| `src/brixzle/engine/validate.py` | Predictor-vs-sim validation (issue #11): both models on an X, Z, theta grid over a placement box, agreement, capture edges and a disagreement map |
| `src/brixzle/engine/report.py` | Scorer and diagnoser (issue #13): runs the predictor, the sim box search and a cross-check for every target, and writes `report.json` (scores, failure categories, limiting failures with traces, prediction-versus-sim disagreements). Scores placement only by default (`tasks`, issue #46) |
| `src/brixzle/families/` | One brick family per subpackage: parameters and bounds, 2D section, CAD solid, rules |
| `src/brixzle/families/interface.py`, `builders.py`, `rules.py`, `contract.py`, `registry.py` | The family contract, shared builders (solid, collision, mass, base), shared R6/R9/R10 rules, contract checks, and family discovery |
| `src/brixzle/families/wedge/`, `pyramid/` | Dummy families for the contract tests: one Prismatic (2.5D), one Solid3D (full 3D) |
| `src/brixzle/families/ab/` | The A/B baseline family ported from aerial-assembly: parameters and bounds, profile, lattice seats, rules, and `family.py` (the interface) |
| `src/brixzle/rules/` | Distilled design rules (`design_rules.md`) and the papers they cite |
| `src/brixzle/rules/papers/` | The papers (PDFs) behind the rules and the predictor, indexed with citations and takeaways in `README.md` |
| `src/brixzle/data/` | Drone error files: measured X–Y error (`error-exp3-r4.json`) and hover estimates (`hover-exp3-r2.json`) |
| `scripts/` | One-off data generators, e.g. `make_hover_estimates.py` (Z, pitch and yaw estimates from aerial-assembly's Exp 3 logs) |
| `tests/` | pytest suite (`uv run pytest`) |
| `tests/fixtures/` | Reference results recorded from aerial-assembly, and the scripts that recorded them |
| `docs/` | Requirements (R1–R13) as prose (`requirements.md`) and data (`requirements.yaml`), the family interface (`family-interface.md`), architecture, the predictor validation report (`predictor-validation.md`), and the decisions made where an issue left something open (`decisions.md`) |
| `.github/workflows/ci.yml` | CI: runs the tests on every PR and on pushes to `main` |
| `runs/` | Git-ignored. One folder per loop run, one subfolder per iteration |
