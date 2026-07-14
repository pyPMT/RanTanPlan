"""Targeted regression guards for specific bugs that were fixed and must not
resurface. Unlike the other modules here, these are not a *feature* — each case
reproduces one historical bug in the smallest form that fails without the fix.
They live in the API suite so they run on every invocation, including
``--quick`` (the full PDDL sweep that also covers them is skipped there).
"""
from unified_planning.shortcuts import (
    Fluent, InstantaneousAction, Problem, BoolType, UserType, Object,
)
from unified_planning.engines import PlanGenerationResultStatus

from ..harness import solve, assert_valid


def _conditional_effect_delivery_problem():
    """Minimal encoder-independent reproducer of the conditional-effect-condition
    pruning bug: a goal reachable only through a conditional effect whose guard
    must first be established by another action.

        setflag        : effect  flag := true
        condmove(o, t) : effect  at(o, t) := true  WHEN flag
        init  at(pkg, locA), flag = false;  goal  at(pkg, locB)

    ``[setflag, condmove(pkg, locB)]`` is valid, but before the fix
    goal-relevance pruning dropped ``setflag`` (the guard ``flag`` was never a
    needed sub-goal), leaving the problem UNSAT at every horizon. See
    docs/specs/conditional-effect-condition-pruning.md.
    """
    Loc, Obj = UserType("Loc"), UserType("Obj")
    at = Fluent("at", BoolType(), o=Obj, l=Loc)
    flag = Fluent("flag", BoolType())
    locA, locB = Object("locA", Loc), Object("locB", Loc)
    pkg = Object("pkg", Obj)

    setflag = InstantaneousAction("setflag")
    setflag.add_effect(flag, True)
    condmove = InstantaneousAction("condmove", o=Obj, t=Loc)
    condmove.add_effect(at(condmove.parameter("o"), condmove.parameter("t")),
                        True, condition=flag)

    problem = Problem("cond_effect_delivery")
    problem.add_objects([locA, locB, pkg])
    problem.add_fluent(at, default_initial_value=False)
    problem.add_fluent(flag, default_initial_value=False)
    problem.set_initial_value(at(pkg, locA), True)
    problem.add_action(setflag)
    problem.add_action(condmove)
    problem.add_goal(at(pkg, locB))
    return problem


def test_conditional_effect_guard_not_pruned():
    """The guard-establishing action must survive goal-relevance pruning, so the
    problem stays solvable — checked on a sequential and an exists-step path
    (both run the achievers/ActionPruning preprocessor pass)."""
    for strategy in ("seq", "elsc"):
        problem = _conditional_effect_delivery_problem()
        r = solve(problem, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{strategy}: expected SOLVED, got {r.status} " \
            f"(conditional-effect guard likely pruned again)"
        assert_valid(problem, r.plan)


CASES = [
    test_conditional_effect_guard_not_pruned,
]
