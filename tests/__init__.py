"""RantanPlan test suites.

Two complementary suites, run together by ``tests.cli`` (entry point: the
root ``test.py`` shim, or ``python -m tests``):

- ``tests.pddl_suite`` — the PDDL-file harness: discovers domain/problem pairs
  under ``pddl/test/`` and runs every strategy, validating each plan.
- ``tests.api`` — API-only feature tests for things that have *no PDDL syntax*
  (e.g. bounded-int action parameters) and so must be built via the UP model
  API. Auto-discovers one module per feature; see ``tests/api/__init__.py``.
"""


class C:
    """ANSI colour helpers (shared across the suites)."""
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BOLD = '\033[1m'
    END = '\033[0m'
