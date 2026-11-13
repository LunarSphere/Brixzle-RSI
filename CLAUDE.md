# CLAUDE.md

Guidance for Claude Code in this repo.

Brixzle-RSI is a recursive self-improving loop for designing brick families that a drone can pick up and place imprecisely. Read `README.md`, `docs/requirements.md` and `docs/architecture.md` before starting work.

## Status

Planning. Don't write loop code until the issue for it exists and has been agreed.

## Speed

Rapid prototyping beats exhaustive sim verification, because the sim-to-real gap is real: printing and testing a brick tells us more than over-verifying it in sim. Design iterations must be as quick as possible while staying accurate.

- **Target:** go from the initial design to the final brick family as fast as possible, and in no more than 30 minutes of wall-clock. Faster is always better. The old aerial-assembly process took 45–90 minutes per family.
- **The bar** is enough evidence to know whether printing the brick is a good idea, not exhaustive robustness.
- **Check key points, not volumes:** a few key contact points (guide faces, seats, bores) instead of densely sampling the object, and small seeded sample counts.
- **Measure time:** record wall-clock for each stage, and flag anything that would push a family past about 30 minutes. Ask before starting a long sweep.
- **Keep the test suite fast:** prefer memoising and targeted checks to brute force.

## Workflow

1. Find the GitHub issue (`gh issue view N`). No issue, no branch: ask for one or offer to draft it.
2. Branch from an up-to-date `main` as `<issue#>-short-name`. One issue, one branch, one PR.
3. TDD: write a failing test, run it and see it fail, then make it pass. New behaviour needs a new test.
4. Run the tests; they must pass before the PR.
5. Open a PR whose body says `Closes #N`, lists the tests run, and cites evidence for any result. The maintainer reviews every PR; merge only when they ask, once CI is green.

## Borrowing from aerial-assembly

- Copy code in; don't import aerial-assembly packages. brixzle depends on the flight stack by relative path, and its trials are changing here.
- Every borrowed file starts with a comment naming its source: `# From LunarSphere/aerial-assembly <path> @ <commit>`, plus a line on what changed.
- Borrow only what the current issue needs.

## Original designs

Don't copy a brick, joint, gripper or structure design from an existing assembly paper or system (for example the construction papers listed in `src/brixzle/rules/citations.md`). Use papers for mechanics and principles, cite them, and design originally. The maintainer's own designs in aerial-assembly (A/B, biasbrick, dfbrick) may be reused and changed (maintainer, 2026-11-12).

## Hardware safety

This repo has no flight code. Never arm a drone, open a radio link, or run anything with `--arm` from here. Real flights happen in aerial-assembly, and only a human flies.

## Reporting results (anti-hallucination)

- Simulated results are not hardware rates. Label them "simulated, uncalibrated contact".
- Analytic predictions are not sim results. Label them "predicted, quasi-static 2D".
- Never state a number, success rate or timing without a source to cite: a run directory with its seed and config, a test, or a file in `docs/`.
- Say when a result was not produced in this session.
- If you did not run it, do not say it passed. Report failing tests with their output.
- Don't invent paper results or API behaviour. Quote the paper or read the package source, and say when a value is a guess.

## Conventions

- **Python and style:** Python 3.12 with uv. Four-space indent, `snake_case`, `PascalCase` classes, dataclasses for settings. Validate inputs explicitly, and write only finite numbers to JSON.
- **Units:** SI.
- **Quaternions:** `[w, x, y, z]`.
- **Runs:** every run writes to a new directory under `runs/` and never overwrites an earlier one. `runs/` is git-ignored.
- **Requirements:** the files in `docs/` and `rules/` define what the agents see. Change them only in a reviewed PR.
- **Commits:** a short imperative subject, one coherent change.
