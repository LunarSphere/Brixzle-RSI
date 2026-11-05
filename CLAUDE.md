# CLAUDE.md

Guidance for Claude Code in this repo.

Brixzle-RSI is a recursive self-improving loop for designing brick families that a drone can pick up and place imprecisely. Read `README.md`, `docs/requirements.md` and `docs/architecture.md` before starting work.

## Status

Planning. Don't write loop code until the issue for it exists and has been agreed.

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
