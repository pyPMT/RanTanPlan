"""Shared helpers for API-only feature tests.

These features (e.g. bounded-int action parameters) have no PDDL syntax, so the
tests build UP problems via the model API and solve them with the engine. The
gold-standard assertion is UP's own ``PlanValidator``, which re-executes the plan
against the original problem — catching substitution/round-trip bugs, not just
"a plan came back".
"""
import rantanplan  # noqa: F401 — registers the UP engines

from unified_planning.shortcuts import OneshotPlanner, PlanValidator
from unified_planning.engines import ValidationResultStatus


def solve(problem, optimal=False, strategy=None):
    """Solve with the RantanPlan engine (silent). ``optimal=True`` selects the
    separately-registered optimal engine (name-based selection ignores the
    optimality guarantee for engine choice). ``strategy`` pins a specific
    strategy (default: the engine default) — used by regression tests that must
    exercise a particular encoder/preprocessor path."""
    name = "rantanplan-optimal" if optimal else "rantanplan"
    params = {"verbosity": "silent"}
    if strategy is not None:
        params["strategy"] = strategy
    with OneshotPlanner(name=name, params=params) as planner:
        return planner.solve(problem)


def assert_valid(problem, plan):
    """Assert UP's PlanValidator accepts the plan against the original problem."""
    val = PlanValidator()
    vr = val.validate(problem, plan)
    assert vr.status == ValidationResultStatus.VALID, \
        f"UP PlanValidator rejected the plan: {vr.status}"


def int_args(action_instance):
    """The integer-constant arguments of a ground action instance, in order."""
    return [p.int_constant_value()
            for p in action_instance.actual_parameters if p.is_int_constant()]
