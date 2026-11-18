"""Finite-domain (bounded-int) action parameters — end-to-end tests.

Integer action parameters are an x-unified-planning model extension with no PDDL
syntax (PDDLReader cannot parse them), so each case builds a UP problem via the
API and asserts UP's PlanValidator accepts the plan. See
docs/specs/numeric-action-parameters.md.
"""
from unified_planning.shortcuts import (
    Fluent, InstantaneousAction, Problem, IntType, RealType, UserType, Object,
    BoolType, GE, LE, LT,
)
from unified_planning.engines import PlanGenerationResultStatus
from unified_planning.exceptions import UPException

from ..harness import solve, assert_valid, int_args

# Bounded types need the Grounded encoder: the default strategy (elsc) refuses
# declared bounds, so the cases that declare them name a Grounded strategy.
GROUNDED = "e"


def test_single_bounded_int_param():
    """Grounder enumerates [1,3]; plan carries the chosen int; validates."""
    level = Fluent("level", IntType(0, 10))
    pour = InstantaneousAction("pour", amount=IntType(1, 3))
    amount = pour.parameter("amount")
    pour.add_precondition(GE(level, amount))
    pour.add_decrease_effect(level, amount)

    problem = Problem("single")
    problem.add_fluent(level, default_initial_value=0)
    problem.set_initial_value(level, 5)
    problem.add_action(pour)
    problem.add_goal(LE(level, 1))

    r = solve(problem, strategy=GROUNDED)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    assert_valid(problem, r.plan)
    for ai in r.plan.actions:
        assert ai.actual_parameters[0].is_int_constant(), ai
        assert 1 <= ai.actual_parameters[0].int_constant_value() <= 3, ai


def test_interleaved_param_order():
    """object / int / object signature must round-trip in signature order."""
    Loc, Tag = UserType("Loc"), UserType("Tag")
    counter = Fluent("counter", IntType(0, 5), loc=Loc)
    marked = Fluent("marked", BoolType(), loc=Loc, tag=Tag)
    bump = InstantaneousAction("bump", loc=Loc, amount=IntType(1, 2), tag=Tag)
    loc, amount, tag = (bump.parameter(n) for n in ("loc", "amount", "tag"))
    bump.add_precondition(LT(counter(loc), 5))
    bump.add_increase_effect(counter(loc), amount)
    bump.add_effect(marked(loc, tag), True)

    l1, t1 = Object("l1", Loc), Object("t1", Tag)
    problem = Problem("interleave")
    problem.add_objects([l1, t1])
    problem.add_fluent(counter, default_initial_value=0)
    problem.add_fluent(marked, default_initial_value=False)
    problem.add_action(bump)
    problem.add_goal(marked(l1, t1))
    problem.add_goal(GE(counter(l1), 2))

    r = solve(problem, strategy=GROUNDED)
    assert r.plan is not None and r.plan.actions, "no plan"
    assert_valid(problem, r.plan)
    ps = r.plan.actions[0].actual_parameters
    assert ps[0].object() == l1 and ps[1].is_int_constant() and ps[2].object() == t1, ps


def test_two_int_params_cartesian():
    """Two int parameters => Cartesian enumeration of both intervals."""
    x = Fluent("x", IntType(0, 20))
    setxy = InstantaneousAction("setxy", a=IntType(1, 3), b=IntType(1, 3))
    a, b = setxy.parameter("a"), setxy.parameter("b")
    setxy.add_increase_effect(x, a + b)

    problem = Problem("cartesian")
    problem.add_fluent(x, default_initial_value=0)
    problem.set_initial_value(x, 0)
    problem.add_action(setxy)
    problem.add_goal(GE(x, 5))          # e.g. a=3,b=2 in one step

    r = solve(problem, strategy=GROUNDED)
    assert r.plan is not None, "no plan"
    assert_valid(problem, r.plan)
    assert all(p.is_int_constant() for p in r.plan.actions[0].actual_parameters)


def test_numeric_pruning_keeps_feasible():
    """Infeasible values are pruned, but a feasible grounding still solves."""
    level = Fluent("level", IntType(0, 10))
    # amount ranges 1..5 but level starts at 2, so amount>2 is infeasible via
    # the precondition; grounding/numeric-RPG must not drop the feasible ones.
    pour = InstantaneousAction("pour", amount=IntType(1, 5))
    amount = pour.parameter("amount")
    pour.add_precondition(GE(level, amount))
    pour.add_decrease_effect(level, amount)

    problem = Problem("pruning")
    problem.add_fluent(level, default_initial_value=0)
    problem.set_initial_value(level, 2)
    problem.add_action(pour)
    problem.add_goal(LE(level, 0))

    r = solve(problem, strategy=GROUNDED)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    assert_valid(problem, r.plan)
    assert all(v >= 1 for ai in r.plan.actions for v in int_args(ai))


def test_cost_depends_on_int_param_optimal():
    """Action cost references the int parameter; optimal mode substitutes it."""
    from unified_planning.model.metrics import MinimizeActionCosts
    level = Fluent("level", IntType(0, 10))
    pour = InstantaneousAction("pour", amount=IntType(1, 3))
    amount = pour.parameter("amount")
    pour.add_precondition(GE(level, amount))
    pour.add_decrease_effect(level, amount)

    problem = Problem("cost")
    problem.add_fluent(level, default_initial_value=0)
    problem.set_initial_value(level, 5)
    problem.add_action(pour)
    problem.add_goal(LE(level, 1))
    problem.add_quality_metric(MinimizeActionCosts({pour: amount}))

    r = solve(problem, optimal=True, strategy=GROUNDED)
    assert r.status == PlanGenerationResultStatus.SOLVED_OPTIMALLY, r.status
    assert_valid(problem, r.plan)
    # Minimum total cost to drive 5 -> <=1 with cost==amount is 4.
    total = sum(v for ai in r.plan.actions for v in int_args(ai))
    assert total == 4, f"expected optimal cost 4, got {total}"


def test_reject_unbounded_and_real():
    """Unsupported parameter types are rejected, not falsely solved. Rejection
    may surface either as a clear conversion error (UPException) or, when the
    unsupported type shows up in the problem kind, as a clean UNSUPPORTED_PROBLEM
    result from the wrapper's allow-list gate — never as a plan."""
    level = Fluent("level", IntType(0, 10))

    def build(ptype):
        act = InstantaneousAction("pour", amount=ptype)
        act.add_decrease_effect(level, act.parameter("amount"))
        p = Problem("reject")
        p.add_fluent(level, default_initial_value=0)
        p.set_initial_value(level, 5)
        p.add_action(act)
        p.add_goal(LE(level, 1))
        return p

    for ptype, why in [(IntType(), "fully unbounded"),
                       (IntType(0), "lower-only"),
                       (IntType(None, 3), "upper-only"),
                       (RealType(0, 3), "real")]:
        try:
            r = solve(build(ptype))
        except UPException:
            continue  # rejected during conversion
        assert r.status not in (PlanGenerationResultStatus.SOLVED_SATISFICING,
                                PlanGenerationResultStatus.SOLVED_OPTIMALLY), \
            f"expected rejection for {why} parameter, got {r.status}"


CASES = [
    test_single_bounded_int_param,
    test_interleaved_param_order,
    test_two_int_params_cartesian,
    test_numeric_pruning_keeps_feasible,
    test_cost_depends_on_int_param_optimal,
    test_reject_unbounded_and_real,
]
