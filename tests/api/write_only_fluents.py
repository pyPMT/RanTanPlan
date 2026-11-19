"""WriteOnlyFluentPass: the written fluents that nothing a plan must satisfy
reads.

Each case builds a small problem, checks how many write-only fluents the pass
reports (the stat ``write_only_fluent.fluents``), and validates the plan. The
benchmarks never need the closure over writes or the bounds rule, so these
cases are the only check of either.
"""
from unified_planning.shortcuts import (
    Fluent, InstantaneousAction, Problem, IntType, GE, Times,
)

from ..harness import solve, assert_valid

# Bounded types need an encoder that asserts declared bounds: the Chained one
# refuses them. The default strategy (e-lin) is Grounded, but the cases that
# declare them name a Grounded strategy, so they do not depend on the default.
GROUNDED = "e"


def _stat(result, key):
    raw = (result.metrics or {}).get(key)
    assert raw is not None, f"no {key} stat: did the pass run?"
    return int(float(raw))


def _write_only_count(result):
    return _stat(result, "write_only_fluent.fluents")


def _actions_left_without_effect(result):
    """Actions whose effects were all write-only. EffectNormalizationPass,
    which runs next, drops them."""
    return _stat(result, "effect_normalization.actions_removed_no_effect")


def _action(name, *increases, condition=None):
    """An action with no precondition that increases each (fluent, value)."""
    action = InstantaneousAction(name)
    for fluent, value in increases:
        if condition is None:
            action.add_increase_effect(fluent, value)
        else:
            action.add_increase_effect(fluent, value, condition=condition)
    return action


def _closure_problem():
    """The example from close_over_writes' comment, plus a conditional effect:

        grow_base: base += 1             wear_out: wear += 1
        grow_rate: rate += 2 * base      spend:    spent += wear
        add:       total += rate         rain:     puddle += 1
        drip:      total += 1  when puddle >= 1
        goal: total >= 10

    The goal reads total. The writes to total read rate (a value) and puddle
    (inside a condition); the write to rate reads base, which only a second
    round finds, inside the compound value 2 * base. Nothing reads spent, so
    its write is never followed: spent and wear are write-only.

    wear_out and spend write only write-only fluents, so they are left with
    no effect and dropped.

    Each wrong closure gives another count: none 5, one round 3, the value
    and condition expressions instead of the fluents inside them 4 (base and
    puddle missed), every effect's value and condition counted as read 1.
    """
    total, rate, base, spent, wear, puddle = (
        Fluent(n, IntType()) for n in ("total", "rate", "base", "spent", "wear", "puddle"))
    problem = Problem("write_only_closure")
    for f in (total, rate, base, spent, wear, puddle):
        problem.add_fluent(f, default_initial_value=0)
    for a in (_action("grow_base", (base, 1)),
              _action("grow_rate", (rate, Times(2, base))),
              _action("add", (total, rate)),
              _action("wear_out", (wear, 1)),
              _action("spend", (spent, wear)),
              _action("rain", (puddle, 1)),
              _action("drip", (total, 1), condition=GE(puddle, 1))):
        problem.add_action(a)
    problem.add_goal(GE(total, 10))
    return problem


def _bounded_problem():
    """A bounded fluent that nothing reads still restricts plans:

        cheap:     visits += 1, fuel_used += 10, distance += 1
        expensive: visits += 1, fuel_used += 60, distance += 5
        fuel_used: integer[0, 100];  goal: visits >= 2

    The bound rules out two expensive visits (120 > 100), so fuel_used counts
    as read. distance, unbounded, is write-only.
    """
    visits = Fluent("visits", IntType())
    fuel_used = Fluent("fuel_used", IntType(0, 100))
    distance = Fluent("distance", IntType())
    problem = Problem("write_only_bounded")
    for f in (visits, fuel_used, distance):
        problem.add_fluent(f, default_initial_value=0)
    problem.add_action(_action("cheap", (visits, 1), (fuel_used, 10), (distance, 1)))
    problem.add_action(_action("expensive", (visits, 1), (fuel_used, 60), (distance, 5)))
    problem.add_goal(GE(visits, 2))
    return problem


def test_closure_follows_values_and_conditions_of_read_writes():
    problem = _closure_problem()
    result = solve(problem)
    assert _write_only_count(result) == 2
    assert _actions_left_without_effect(result) == 2
    assert_valid(problem, result.plan)


def test_bounded_fluent_counts_as_read():
    problem = _bounded_problem()
    result = solve(problem, strategy=GROUNDED)
    assert _write_only_count(result) == 1
    assert _actions_left_without_effect(result) == 0
    assert_valid(problem, result.plan)


CASES = [
    test_closure_follows_values_and_conditions_of_read_writes,
    test_bounded_fluent_counts_as_read,
]
