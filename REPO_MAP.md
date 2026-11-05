# Repo map

What lives where. Update this file in the same PR that adds or moves a top-level path.

| Path | What it holds |
|---|---|
| `src/brixzle/` | The `brixzle` package |
| `src/brixzle/intake/` | Local web page: concept sketch and extra requirements, with a conflict check against R1–R12 |
| `src/brixzle/agents/` | Design agent and critic agent (Claude Agent SDK) |
| `src/brixzle/engine/` | Analytic predictor, tuner, sim verifier, scorer and diagnoser |
| `src/brixzle/engine/sim/` | MuJoCo sim core ported from aerial-assembly: scene, drop trial, Tolerance and classify, Exp 3 error model |
| `src/brixzle/families/` | One brick family per subpackage: parameters and bounds, 2D section, CAD solid, rules |
| `src/brixzle/families/ab/` | The A/B family ported from aerial-assembly: parameters, profile, CadQuery solid, collision pieces, lattice seats |
| `src/brixzle/rules/` | Distilled design rules (`design_rules.md`) and the papers they cite |
| `src/brixzle/data/` | Measured drone error files |
| `tests/` | pytest suite (`uv run pytest`) |
| `tests/fixtures/` | Reference results recorded from aerial-assembly, and the scripts that recorded them |
| `docs/` | Requirements (R1–R12) as prose (`requirements.md`) and data (`requirements.yaml`), and architecture |
| `.github/workflows/ci.yml` | CI: runs the tests on every PR and on pushes to `main` |
| `runs/` | Git-ignored. One folder per loop run, one subfolder per iteration |
