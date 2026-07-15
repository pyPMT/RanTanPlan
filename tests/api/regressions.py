"""Targeted regression guards for specific bugs that were fixed and must not
resurface. Unlike the other modules here, these are not a *feature* — each case
reproduces one historical bug in the smallest form that fails without the fix.
They live in the API suite so they run on every invocation, including
``--quick`` (the full PDDL sweep that also covers them is skipped there).
"""
from unified_planning.shortcuts import (
    Fluent, InstantaneousAction, Problem, BoolType, IntType, UserType, Object,
    Equals,
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


def test_goal_relevance_numeric_value_chaining():
    """Goal-relevance pruning must not drop an action whose contribution flows
    through the *value* of a numeric fluent rather than through a boolean
    sub-goal. Smallest failing form:

        level : Int(0, 20), initially 0
        inc   : level += 5
        dec   : level -= 2
        goal  : level == 3          (solvable: inc -> 5, dec -> 3)

    Every valid plan must overshoot the target and come back down (3 = 5a - 2b
    needs a >= 1 and b >= 1), so BOTH actions are required.

    The historical bug: AchieversAnalysis asserts RPG-derived bounds on the
    pre- and post-state of its per-(action, condition) satisfiability check,
    so `inc` is *correctly* judged unable to directly achieve `level == 3`
    (that would need pre-state level == -2, outside [0, 20]); only `dec` is an
    achiever (from level == 5). The backward relevance walk then enqueues only
    the achiever's boolean needed-conditions (preconditions and effect
    guards). `dec` has none, so the walk stops, `inc` is pruned as
    goal-irrelevant, and the next reachability pass "proves" the goal
    unreachable: a solvable problem returns UNSOLVABLE_PROVEN.

    All three ingredients are needed to expose it: declared bounds (they make
    the direct-achiever test sharp enough to reject `inc`), an equality goal
    (directional), and a goal reachable only through the decrease
    (non-monotone trajectory). Dropping any one masks the bug — an unbounded
    type readmits `inc` via the spurious pre-state -2, and `level >= 3` is
    achieved by `inc` directly.

    The fix (the numeric sibling of the conditional-effect-guard fix pinned by
    test_conditional_effect_guard_not_pruned): a numeric condition is achieved
    *from* a numeric pre-state (`level == 3` via `-= 2` regresses to
    `level == 5`), which no boolean sub-goal can represent, so the closure
    conservatively marks every writer of a numeric fluent occurring in a
    relevant condition as relevant."""
    lv = Fluent("level", IntType(0, 20))
    p = Problem("numeric_value_chaining")
    p.add_fluent(lv, default_initial_value=0)
    inc = InstantaneousAction("inc")
    inc.add_increase_effect(lv, 5)
    dec = InstantaneousAction("dec")
    dec.add_decrease_effect(lv, 2)
    p.add_action(inc)
    p.add_action(dec)
    p.add_goal(Equals(lv, 3))

    r = solve(p)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {r.status} " \
        f"(numeric-value-chained action likely pruned again)"
    assert_valid(p, r.plan)


def test_goal_relevance_numeric_rhs_chaining():
    """Transitive variant of test_goal_relevance_numeric_value_chaining: the
    needed action writes a fluent that only appears on the *right-hand side*
    of the achiever's effect, one step further from the goal.

        level : Int(0, 20) = 0        step : Int(0, 5) = 0
        inc     : level += 5
        dec     : level -= step
        setstep : step  := 2
        goal    : level == 3          (solvable: inc, setstep, dec)

    `dec` achieves `level == 3` (from level == 5 with step == 2), but neither
    `inc` nor `setstep` establishes any boolean condition of `dec` — `inc`
    matters through the value of `level` and `setstep` through the value of
    `step`, which `dec` merely *reads* in its effect expression. The relevance
    closure must therefore chain twice: writers of `level` (goal fluent) are
    relevant, and numeric fluents read by those writers' effect RHS (`step`)
    recursively pull in their own writers (`setstep`). Before the fix both
    `inc` and `setstep` were pruned and the problem returned
    UNSOLVABLE_PROVEN."""
    lv = Fluent("level", IntType(0, 20))
    st = Fluent("step", IntType(0, 5))
    p = Problem("numeric_rhs_chaining")
    p.add_fluent(lv, default_initial_value=0)
    p.add_fluent(st, default_initial_value=0)
    inc = InstantaneousAction("inc")
    inc.add_increase_effect(lv, 5)
    dec = InstantaneousAction("dec")
    dec.add_decrease_effect(lv, st)
    setstep = InstantaneousAction("setstep")
    setstep.add_effect(st, 2)
    p.add_action(inc)
    p.add_action(dec)
    p.add_action(setstep)
    p.add_goal(Equals(lv, 3))

    r = solve(p)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {r.status} " \
        f"(RHS-chained numeric writer likely pruned again)"
    assert_valid(p, r.plan)


def test_boolean_assign_from_fluent():
    """Expression-valued BOOLEAN assignment (`y := x`) must work end to end.
    Smallest failing form (no arrays involved, though boolean array cell
    copies/swaps — `a[1] := a[0]` — are how real models hit it):

        x, y : bool, initially false
        setx : x := true
        copy : y := x
        goal : y                      (solvable: setx; copy)

    Historically, several preprocessing components classified boolean
    assignments purely by their constant value (true = add, false = delete)
    and silently treated an expression-valued assign as a no-op:

      * the ReachabilityGrounder's delete-relaxation counted only
        constant-true assigns as adds, so `y` was never reachable-true and
        grounding "proved" the problem unsolvable (UNSOLVABLE_PROVEN);
      * the h2 mutex checker counted it as neither add nor delete, deriving
        unsound mutexes/invariants from the missed achiever;
      * the exactly-one invariant upgrades counted it as neither, accepting
        groups whose cardinality the assign can silently reduce;
      * goal-relevance pruning saw `copy` achieve the goal but never learned
        that its effect *reads* `x` — a value dependency, not a boolean
        sub-goal — so `setx` was pruned as irrelevant and the search failed
        at every horizon (UNSOLVABLE_INCOMPLETELY once the grounder was
        fixed).

    The fixes: grounder and h2 classify a non-constant boolean assign as a
    may-add (never a guaranteed delete); the exactly-one checks treat it as a
    may-delete; and the relevance closure marks every fluent read by a
    relevant action's effect RHS as value-relevant, pulling in its writers
    transitively — the boolean face of the same value-chaining rule pinned by
    test_goal_relevance_numeric_value_chaining."""
    x = Fluent("x", BoolType())
    y = Fluent("y", BoolType())
    p = Problem("boolean_assign_from_fluent")
    p.add_fluent(x, default_initial_value=False)
    p.add_fluent(y, default_initial_value=False)
    setx = InstantaneousAction("setx")
    setx.add_effect(x, True)
    copy = InstantaneousAction("copy")
    copy.add_effect(y, x)
    p.add_action(setx)
    p.add_action(copy)
    p.add_goal(y)

    r = solve(p)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {r.status} " \
        f"(expression-valued boolean assign mishandled again)"
    assert_valid(p, r.plan)


CASES = [
    test_conditional_effect_guard_not_pruned,
    test_goal_relevance_numeric_value_chaining,
    test_goal_relevance_numeric_rhs_chaining,
    test_boolean_assign_from_fluent,
]
