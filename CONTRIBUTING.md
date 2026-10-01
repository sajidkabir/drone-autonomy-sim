# Contributing

Contributions are welcome: bug reports, planning or decision-logic
corrections, new behaviors, documentation, and examples. This project
aims to stay small, readable, and honest about its assumptions.

## Getting set up

```bash
git clone https://github.com/sajidkabir/drone-autonomy-sim.git
cd drone-autonomy-sim
python -m venv .venv
source .venv/bin/activate
pip install -e . pytest
pytest -q
```

All 16 tests should pass before you change anything.

## Making a change

1. Fork the repository and create a branch from `main`
   (`git checkout -b feature/short-name`).
2. Keep the change focused. One behavior, one fix, or one feature per
   pull request.
3. Add or update tests. Behavior changes need a scenario test that shows
   the decision log doing the right thing, and the test should say why
   that behavior is correct.
4. Run `pytest -q` and make sure it is green.
5. Update the README, the docstrings, and `CHANGELOG.md` (Unreleased
   section) if behavior or the decision log changes.
6. Open a pull request against `main` describing what changed, why, and
   what it does to the decision log on the reference missions.

## Ground rules

- No silent behavior changes. If a fix alters the outcome of a reference
  mission, say so in the pull request and in the changelog.
- Prefer explicit decisions over clever code. A reviewer should be able
  to trace every state transition to a logged reason.
- New assumptions go in the README limitations list until they are
  modeled.

## Reporting issues

Open an issue with the world configuration, the mission parameters, the
decision log you got, and the behavior you expected. A minimal reproducing
scenario is worth more than a long description.
