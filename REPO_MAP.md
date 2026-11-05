# Repo map

What lives where. Update this file in the same PR that adds or moves a top-level path.

| Path | What it holds |
|---|---|
| `src/brixzle/` | The `brixzle` package |
| `src/brixzle/intake/` | Local web page: concept sketch and extra requirements, with a conflict check against R1–R12 |
| `src/brixzle/agents/` | Design agent and critic agent (Claude Agent SDK) |
| `src/brixzle/engine/` | Analytic predictor, tuner, sim verifier, scorer and diagnoser |
| `src/brixzle/families/` | One brick family per subpackage: parameters and bounds, 2D section, CAD solid, rules |
| `src/brixzle/rules/` | Distilled design rules (`design_rules.md`) and the papers they cite |
| `src/brixzle/data/` | Measured drone error files |
| `tests/` | pytest suite (`uv run pytest`) |
| `docs/` | Requirements (R1–R12) and architecture |
| `.github/workflows/ci.yml` | CI: runs the tests on every PR and on pushes to `main` |
| `runs/` | Git-ignored. One folder per loop run, one subfolder per iteration |
