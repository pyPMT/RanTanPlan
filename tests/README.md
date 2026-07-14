# RantanPlan test suites

Run everything from the repo root:

```bash
python test.py            # PDDL suite + API-only suite
python test.py --quick    # fast PDDL subset + API-only suite
python test.py --api-only # only the API-only feature tests
python test.py --pddl-only
python test.py -v         # planner output / failure tracebacks
python -m tests           # equivalent to `python test.py`
```

`test.py` at the repo root is a thin shim into this package.

## Layout

| Module | What it is |
|--------|-----------|
| `tests/cli.py` | Argument parsing; runs both suites and prints one combined summary (non-zero exit on any failure). |
| `tests/pddl_suite.py` | The PDDL-file harness: discovers `domain`/`problem` pairs under `pddl/test/` and runs every strategy, validating each plan. Parallel by default (`-j`). |
| `tests/api/` | API-only feature tests — one module per feature (below). |
| `tests/harness.py` | Shared helpers for API tests: `solve()`, `assert_valid()`, `int_args()`. |

## Why two suites

Some features have **no PDDL syntax** and cannot be expressed in a `pddl/test/`
file (e.g. bounded-int action parameters — `PDDLReader` cannot parse them). Those
are built via the UP model API and live in `tests/api/`. Everything expressible in
PDDL stays in the PDDL suite.

## Adding tests for a new API-only feature

Drop a new module `tests/api/<feature>.py` that exposes a `CASES` list:

```python
from ..harness import solve, assert_valid, int_args

def test_something():
    problem = ...            # build a UP problem via the API
    r = solve(problem)       # or solve(problem, optimal=True)
    assert_valid(problem, r.plan)   # gold standard: UP re-executes the plan

CASES = [test_something]
```

Each entry in `CASES` is a zero-argument callable that raises (e.g. via `assert`)
on failure. `tests/api/__init__.py` discovers the module automatically — no wiring
needed. Prefer `assert_valid` (UP's `PlanValidator`) over hand-checking plans; it
re-executes the plan against the original problem and catches round-trip bugs.
