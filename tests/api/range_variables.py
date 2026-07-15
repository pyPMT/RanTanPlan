"""Numeric range-variable quantifiers, expanded by the C++ QuantifierExpansionPass.

Int variables have no PDDL syntax, and UP has no validator for INT_VARIABLES
problems. So each case solves P (the range-var problem, via the C++ expansion
path) and validates the produced plan against a hand-written range-var-FREE
semantic equivalent P_ref that UP CAN validate. A wrong expansion yields a plan
that fails P_ref's validation. See docs/specs/numeric-action-parameters.md (§6).
"""
from unified_planning.shortcuts import (
    Fluent, InstantaneousAction, Problem, IntType, UserType, Object, BoolType,
    GE, Equals, Or, And, Forall, Exists,
)
from unified_planning.engines import ValidationResultStatus
from unified_planning.plans import SequentialPlan, ActionInstance
from unified_planning.model.int_variable import IntVariable

from ..harness import solve
from unified_planning.shortcuts import PlanValidator


def _check(p, p_ref):
    """Solve P (range-var, C++ path); validate its plan against range-var-free P_ref."""
    r = solve(p)
    assert r.plan is not None, f"no plan for {p.name}"
    mapped = SequentialPlan([
        ActionInstance(p_ref.action(ai.action.name), ai.actual_parameters)
        for ai in r.plan.actions])
    with PlanValidator(problem_kind=p_ref.kind) as pv:
        vr = pv.validate(p_ref, mapped)
    assert vr.status == ValidationResultStatus.VALID, \
        f"expansion wrong: {p.name} plan invalid under equivalent ({vr.status})"


def test_forall_constant_bound_goal():
    # forall i in [1,3]: level >= i   ==   level >= 3
    level = Fluent("level", IntType(0, 10))

    def build(goal):
        inc = InstantaneousAction("inc"); inc.add_increase_effect(level, 1)
        p = Problem("f_goal"); p.add_fluent(level, default_initial_value=0)
        p.set_initial_value(level, 0); p.add_action(inc); p.add_goal(goal)
        return p

    i = IntVariable("i", 1, 3)
    _check(build(Forall(GE(level, i), i)), build(GE(level, 3)))


def test_forall_constant_bound_precondition():
    # precond forall i in [1,2]: level >= i   ==   level >= 2
    level = Fluent("level", IntType(0, 10))

    def build(precond):
        inc = InstantaneousAction("inc"); inc.add_increase_effect(level, 1)
        fire = InstantaneousAction("fire"); fire.add_precondition(precond)
        fire.add_decrease_effect(level, 2)
        p = Problem("f_pre"); p.add_fluent(level, default_initial_value=0)
        p.set_initial_value(level, 0); p.add_action(inc); p.add_action(fire)
        p.add_goal(Equals(level, 0))  # inc,inc -> fire (needs level>=2) -> 0
        return p

    i = IntVariable("i", 1, 2)
    _check(build(Forall(GE(level, i), i)), build(GE(level, 2)))


def test_forall_parameter_dependent_bound():
    # precond forall j in [1,n]: level >= j   ==   level >= n   (n an int param)
    mark = Fluent("mark", IntType(0, 1))
    level = Fluent("level", IntType(0, 10))

    def build(precond_fn):
        need = InstantaneousAction("need", n=IntType(1, 3))
        n = need.parameter("n")
        need.add_precondition(precond_fn(n)); need.add_increase_effect(mark, 1)
        p = Problem("f_param"); p.add_fluent(mark, default_initial_value=0)
        p.add_fluent(level, default_initial_value=0)
        p.set_initial_value(level, 2); p.set_initial_value(mark, 0)
        p.add_action(need); p.add_goal(GE(mark, 1))
        return p

    _check(build(lambda n: Forall(GE(level, IntVariable("j", 1, n)),
                                  IntVariable("j", 1, n))),
           build(lambda n: GE(level, n)))


def test_exists_constant_bound_goal():
    # exists k in [1,3]: level == k   ==   level==1 or level==2 or level==3
    level = Fluent("level", IntType(0, 10))

    def build(goal):
        inc = InstantaneousAction("inc"); inc.add_increase_effect(level, 1)
        p = Problem("e_goal"); p.add_fluent(level, default_initial_value=0)
        p.set_initial_value(level, 0); p.add_action(inc); p.add_goal(goal)
        return p

    k = IntVariable("k", 1, 3)
    _check(build(Exists(Equals(level, k), k)),
           build(Or(Equals(level, 1), Equals(level, 2), Equals(level, 3))))


def test_object_quantifier_in_range_problem():
    # A range variable makes UP skip QuantifiersRemover, so the C++ pass must
    # also expand the OBJECT forall in the goal. P_ref keeps the object forall
    # (UP validates UNIVERSAL_CONDITIONS) and only drops the range var.
    from unified_planning.model.variable import Variable
    Loc = UserType("Loc")
    l1, l2 = Object("l1", Loc), Object("l2", Loc)
    visited = Fluent("visited", BoolType(), loc=Loc)
    level = Fluent("level", IntType(0, 10))

    def build(range_goal):
        visit = InstantaneousAction("visit", loc=Loc)
        visit.add_effect(visited(visit.parameter("loc")), True)
        inc = InstantaneousAction("inc"); inc.add_increase_effect(level, 1)
        p = Problem("obj_in_range")
        p.add_objects([l1, l2])
        p.add_fluent(visited, default_initial_value=False)
        p.add_fluent(level, default_initial_value=0)
        p.set_initial_value(level, 0)
        p.add_action(visit); p.add_action(inc)
        v = Variable("l", Loc)
        # goal: (forall ?l - Loc: visited(?l)) AND <range goal>
        p.add_goal(And(Forall(visited(v), v), range_goal))
        return p

    i = IntVariable("i", 1, 2)
    _check(build(Forall(GE(level, i), i)), build(GE(level, 2)))


CASES = [
    test_forall_constant_bound_goal,
    test_forall_constant_bound_precondition,
    test_forall_parameter_dependent_bound,
    test_exists_constant_bound_goal,
    test_object_quantifier_in_range_problem,
]
