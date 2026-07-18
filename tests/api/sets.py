"""Native set-fluent support (SetLoweringPass — docs/native-sets-plan.md).

Set fluents have no PDDL syntax and UP ships no plan validator for SET_FLUENTS,
so each case builds the UP problem via the model API, solves it through
RantanPlan (which lowers set operations to boolean membership cells before
grounding), and validates the returned plan against a **set-semantics
simulator** (below) that executes the plan under real set semantics and checks
every precondition and the goal. This is the sets analogue of the array tests'
scalar-reference / simulator oracle (UP's PlanValidator can't evaluate sets).

Error cases assert the ill-formed model is rejected at conversion (a clear
UPException) or, for --up-grounding, a hard C++ rejection.
"""
import warnings
warnings.filterwarnings("ignore")  # UP prints a no-set-validator notice

from unified_planning.shortcuts import (
    Fluent, InstantaneousAction, Problem, IntType, BoolType, UserType, Object,
    And, Not, Equals, GE, LT, Int,
)
from unified_planning.model.operators import OperatorKind as OK
from unified_planning.exceptions import UPException

from ..harness import solve

_ENV = None


def _em():
    global _ENV
    if _ENV is None:
        from unified_planning.environment import get_environment
        _ENV = get_environment()
    return _ENV.expression_manager


def _st(elem):
    """A SetType over `elem` (SetType lives on the type manager)."""
    _em()  # ensure _ENV initialised
    return _ENV.type_manager.SetType(elem)


# ── set-semantics simulator (the oracle) ─────────────────────────────────────

def _const(node):
    """A constant/object/set literal FNode → a Python value (name / bool / int /
    frozenset of names)."""
    if node.is_object_exp():
        return node.object().name
    if node.is_bool_constant():
        return node.bool_constant_value()
    if node.is_int_constant():
        return node.int_constant_value()
    if node.node_type == OK.SET_CONSTANT:
        return frozenset(_const(e) for e in node._content.payload)
    raise AssertionError(f"not a constant literal: {node}")


def _eval(node, state, subs):
    """Evaluate an FNode against a ground `state` and a param/variable→name
    substitution `subs`."""
    nt = node.node_type
    if node.is_object_exp():
        return node.object().name
    if node.is_parameter_exp():
        return subs[node.parameter().name]
    if node.is_variable_exp():
        return subs[node.variable().name]
    if node.is_bool_constant():
        return node.bool_constant_value()
    if node.is_int_constant():
        return node.int_constant_value()
    if node.node_type == OK.SET_CONSTANT:
        return frozenset(_eval(e, state, subs) for e in node._content.payload)
    if node.is_fluent_exp():
        key = (node.fluent().name, tuple(_eval(a, state, subs) for a in node.args))
        return state[key]
    # set operators
    if nt == OK.SET_MEMBER:
        return _eval(node.arg(0), state, subs) in _eval(node.arg(1), state, subs)
    if nt == OK.SET_ADD:
        return _eval(node.arg(1), state, subs) | {_eval(node.arg(0), state, subs)}
    if nt == OK.SET_REMOVE:
        return _eval(node.arg(1), state, subs) - {_eval(node.arg(0), state, subs)}
    if nt == OK.SET_UNION:
        return _eval(node.arg(0), state, subs) | _eval(node.arg(1), state, subs)
    if nt == OK.SET_INTERSECT:
        return _eval(node.arg(0), state, subs) & _eval(node.arg(1), state, subs)
    if nt == OK.SET_DIFFERENCE:
        return _eval(node.arg(0), state, subs) - _eval(node.arg(1), state, subs)
    if nt == OK.SET_CARDINALITY:
        return len(_eval(node.arg(0), state, subs))
    if nt == OK.SET_SUBSETEQ:
        return _eval(node.arg(0), state, subs) <= _eval(node.arg(1), state, subs)
    if nt == OK.SET_DISJOINT:
        return not (_eval(node.arg(0), state, subs) & _eval(node.arg(1), state, subs))
    # logic / relations / arithmetic
    if node.is_and():
        return all(_eval(a, state, subs) for a in node.args)
    if node.is_or():
        return any(_eval(a, state, subs) for a in node.args)
    if node.is_not():
        return not _eval(node.arg(0), state, subs)
    if node.is_equals():
        return _eval(node.arg(0), state, subs) == _eval(node.arg(1), state, subs)
    if node.is_le():
        return _eval(node.arg(0), state, subs) <= _eval(node.arg(1), state, subs)
    if node.is_lt():
        return _eval(node.arg(0), state, subs) < _eval(node.arg(1), state, subs)
    if node.is_plus():
        return sum(_eval(a, state, subs) for a in node.args)
    if node.is_minus():
        return _eval(node.arg(0), state, subs) - _eval(node.arg(1), state, subs)
    raise AssertionError(f"simulator: unhandled node {node} ({nt})")


def _initial_state(problem):
    state = {}
    for fexp, val in problem.initial_values.items():
        key = (fexp.fluent().name, tuple(_const(a) for a in fexp.args))
        state[key] = _const(val)
    return state


def _forall_bindings(variables, problem, subs):
    """Yield subs extended with every combination of object bindings for the
    given forall variables."""
    if not variables:
        yield subs
        return
    v, rest = variables[0], variables[1:]
    for obj in problem.objects(v.type):
        s2 = dict(subs)
        s2[v.name] = obj.name
        yield from _forall_bindings(rest, problem, s2)


def _apply(action_instance, state, problem):
    """Apply one ground action: check preconditions against the pre-state, stage
    all effects (evaluated against the pre-state), then commit. Returns False if
    a precondition is violated."""
    action = action_instance.action
    subs = {prm.name: _const(o) for prm, o in zip(action.parameters,
                                                  action_instance.actual_parameters)}
    for pre in action.preconditions:
        if not _eval(pre, state, subs):
            return False
    staged = {}
    for eff in action.effects:
        for s in _forall_bindings(list(eff.forall), problem, subs):
            if eff.condition is not None and not _eval(eff.condition, state, s):
                continue
            key = (eff.fluent.fluent().name, tuple(_eval(a, state, s) for a in eff.fluent.args))
            staged[key] = _eval(eff.value, state, s)
    state.update(staged)
    return True


def _simulate_ok(problem, plan):
    """True iff every action's preconditions hold in turn and all goals hold at
    the end — the plan is valid under set semantics."""
    state = _initial_state(problem)
    for ai in plan.actions:
        if not _apply(ai, state, problem):
            return False
    return all(_eval(g, state, {}) for g in problem.goals)


def _solve_ok(problem, strategy="seq"):
    r = solve(problem, strategy=strategy)
    assert r.plan is not None, f"{problem.name}: no plan ({r.status})"
    assert _simulate_ok(problem, r.plan), \
        f"{problem.name}: plan {r.plan} fails set-semantics simulation"
    return r.plan


def _assert_rejected(build):
    try:
        from rantanplan.converter import convert_problem
        convert_problem(build())
    except UPException:
        return
    raise AssertionError("expected a UPException, but conversion succeeded")


# ── shared objects ───────────────────────────────────────────────────────────

Truck, Loc, Pkg = UserType("Truck"), UserType("Loc"), UserType("Pkg")


def _pkgs(p, names):
    objs = [Object(n, Pkg) for n in names]
    p.add_objects(objs)
    return objs


# ── semantics ────────────────────────────────────────────────────────────────

def test_member_precondition_and_add():
    """load adds a package (add-onto-self); a member precondition forbids
    re-loading; goal asks two members."""
    p = Problem("member_add")
    held = Fluent("held", _st(Pkg), t=Truck)
    p.add_fluent(held, default_initial_value=_em().Set([]))
    t1 = Object("t1", Truck); p.add_object(t1)
    a, b = _pkgs(p, ["a", "b"])
    load = InstantaneousAction("load", t=Truck, pk=Pkg)
    t, pk = load.parameter("t"), load.parameter("pk")
    load.add_precondition(Not(_em().SetMember(pk, held(t))))
    load.add_effect(held(t), _em().SetAdd(pk, held(t)))
    p.add_action(load)
    p.add_goal(And(_em().SetMember(_em().ObjectExp(a), held(t1)),
                   _em().SetMember(_em().ObjectExp(b), held(t1))))
    _solve_ok(p)


def test_remove_effect():
    """remove takes an element out; goal asks it is no longer a member."""
    p = Problem("remove")
    held = Fluent("held", _st(Pkg), t=Truck)
    p.add_fluent(held, default_initial_value=_em().Set([]))
    t1 = Object("t1", Truck); p.add_object(t1)
    a, b = _pkgs(p, ["a", "b"])
    p.set_initial_value(held(t1), _em().Set([_em().ObjectExp(a), _em().ObjectExp(b)]))
    unload = InstantaneousAction("unload", t=Truck, pk=Pkg)
    t, pk = unload.parameter("t"), unload.parameter("pk")
    unload.add_precondition(_em().SetMember(pk, held(t)))
    unload.add_effect(held(t), _em().SetRemove(pk, held(t)))
    p.add_action(unload)
    p.add_goal(Not(_em().SetMember(_em().ObjectExp(a), held(t1))))
    _solve_ok(p)


def test_cardinality_vs_constant_pb():
    """card(held) >= 2 (the pseudo-boolean path)."""
    p = Problem("card_const")
    held = Fluent("held", _st(Pkg), t=Truck)
    p.add_fluent(held, default_initial_value=_em().Set([]))
    t1 = Object("t1", Truck); p.add_object(t1)
    _pkgs(p, ["a", "b", "c"])
    load = InstantaneousAction("load", t=Truck, pk=Pkg)
    t, pk = load.parameter("t"), load.parameter("pk")
    load.add_precondition(Not(_em().SetMember(pk, held(t))))
    load.add_effect(held(t), _em().SetAdd(pk, held(t)))
    p.add_action(load)
    p.add_goal(GE(_em().SetCardinality(held(t1)), Int(2)))
    _solve_ok(p)


def test_cardinality_vs_cardinality_sum():
    """card(held(t1)) < card(held(t2)) (the sum-comparison path)."""
    p = Problem("card_card")
    held = Fluent("held", _st(Pkg), t=Truck)
    p.add_fluent(held, default_initial_value=_em().Set([]))
    t1, t2 = Object("t1", Truck), Object("t2", Truck)
    p.add_objects([t1, t2])
    a, b = _pkgs(p, ["a", "b"])
    p.set_initial_value(held(t2), _em().Set([_em().ObjectExp(a), _em().ObjectExp(b)]))
    load = InstantaneousAction("load", t=Truck, pk=Pkg)
    t, pk = load.parameter("t"), load.parameter("pk")
    load.add_effect(held(t), _em().SetAdd(pk, held(t)))
    p.add_action(load)
    # t1 starts empty (0) < t2 (2) already; require it stays a strict inequality
    p.add_goal(LT(_em().SetCardinality(held(t1)), _em().SetCardinality(held(t2))))
    _solve_ok(p)


def test_union_onto_self_and_clear():
    """The dump_trucks core: pat(l) := pat(l) ∪ pin(t) and pin(t) := {} in one
    action, with an initial constant set — exercises the union-onto-self and
    assign-empty effect shapes together (and the mutex-invariant fix)."""
    p = Problem("dump")
    pin = Fluent("pin", _st(Pkg), t=Truck)
    pat = Fluent("pat", _st(Pkg), l=Loc)
    p.add_fluent(pin, default_initial_value=_em().Set([]))
    p.add_fluent(pat, default_initial_value=_em().Set([]))
    t1, l1 = Object("t1", Truck), Object("l1", Loc)
    p.add_objects([t1, l1])
    a, b = _pkgs(p, ["a", "b"])
    dump = InstantaneousAction("dump", t=Truck, l=Loc)
    t, l = dump.parameter("t"), dump.parameter("l")
    dump.add_effect(pat(l), _em().SetUnion(pat(l), pin(t)))
    dump.add_effect(pin(t), _em().Set([]))
    p.add_action(dump)
    p.set_initial_value(pin(t1), _em().Set([_em().ObjectExp(a), _em().ObjectExp(b)]))
    p.add_goal(GE(_em().SetCardinality(pat(l1)), Int(2)))
    _solve_ok(p, strategy="seq")
    _solve_ok(p, strategy="els")


def test_assign_constant_set():
    """s := {a, b} (general assignment from a constant set)."""
    p = Problem("assign_const")
    held = Fluent("held", _st(Pkg), t=Truck)
    p.add_fluent(held, default_initial_value=_em().Set([]))
    t1 = Object("t1", Truck); p.add_object(t1)
    a, b = _pkgs(p, ["a", "b"])
    fill = InstantaneousAction("fill", t=Truck)
    t = fill.parameter("t")
    fill.add_effect(held(t), _em().Set([_em().ObjectExp(a), _em().ObjectExp(b)]))
    p.add_action(fill)
    p.add_goal(_em().SetMember(_em().ObjectExp(b), held(t1)))
    _solve_ok(p)


def test_subseteq_precondition():
    """A subseteq precondition (fluent ⊆ fluent), general lowering."""
    p = Problem("subseteq")
    need = Fluent("need", _st(Pkg), l=Loc)
    have = Fluent("have", _st(Pkg), t=Truck)
    p.add_fluent(need, default_initial_value=_em().Set([]))
    p.add_fluent(have, default_initial_value=_em().Set([]))
    t1, l1 = Object("t1", Truck), Object("l1", Loc)
    p.add_objects([t1, l1])
    a, b = _pkgs(p, ["a", "b"])
    p.set_initial_value(need(l1), _em().Set([_em().ObjectExp(a)]))
    done = Fluent("done", BoolType())
    p.add_fluent(done, default_initial_value=False)
    load = InstantaneousAction("load", t=Truck, pk=Pkg)
    t, pk = load.parameter("t"), load.parameter("pk")
    load.add_effect(have(t), _em().SetAdd(pk, have(t)))
    p.add_action(load)
    deliver = InstantaneousAction("deliver", t=Truck, l=Loc)
    t, l = deliver.parameter("t"), deliver.parameter("l")
    deliver.add_precondition(_em().SetSubseteq(need(l), have(t)))  # need ⊆ have
    deliver.add_effect(done, True)
    p.add_action(deliver)
    p.add_goal(done)
    _solve_ok(p)


def test_disjoint_and_equality_goal():
    """disjoint precondition and a set-equality goal in one problem."""
    p = Problem("disj_eq")
    s = Fluent("s", _st(Pkg), t=Truck)
    goalset = Fluent("goalset", _st(Pkg), t=Truck)
    p.add_fluent(s, default_initial_value=_em().Set([]))
    p.add_fluent(goalset, default_initial_value=_em().Set([]))
    t1 = Object("t1", Truck); p.add_object(t1)
    a, b = _pkgs(p, ["a", "b"])
    p.set_initial_value(goalset(t1), _em().Set([_em().ObjectExp(a), _em().ObjectExp(b)]))
    load = InstantaneousAction("load", t=Truck, pk=Pkg)
    t, pk = load.parameter("t"), load.parameter("pk")
    load.add_precondition(Not(_em().SetMember(pk, s(t))))            # disjoint-ish guard
    load.add_effect(s(t), _em().SetAdd(pk, s(t)))
    p.add_action(load)
    p.add_goal(Equals(s(t1), goalset(t1)))   # s == {a, b}
    _solve_ok(p)


def test_card_of_union_goal():
    """card(pin(t1) ∪ pin(t2)) >= 3 — cardinality of a composed set expression."""
    p = Problem("card_union")
    pin = Fluent("pin", _st(Pkg), t=Truck)
    p.add_fluent(pin, default_initial_value=_em().Set([]))
    t1, t2 = Object("t1", Truck), Object("t2", Truck)
    p.add_objects([t1, t2])
    a, b, c = _pkgs(p, ["a", "b", "c"])
    p.set_initial_value(pin(t1), _em().Set([_em().ObjectExp(a)]))
    p.set_initial_value(pin(t2), _em().Set([_em().ObjectExp(b)]))
    load = InstantaneousAction("load", t=Truck, pk=Pkg)
    t, pk = load.parameter("t"), load.parameter("pk")
    load.add_effect(pin(t), _em().SetAdd(pk, pin(t)))
    p.add_action(load)
    p.add_goal(GE(_em().SetCardinality(_em().SetUnion(pin(t1), pin(t2))), Int(3)))
    _solve_ok(p)


def test_difference_effect_third_set():
    """s3 := s1 \\ s2 onto a *third* set (the srm-unsound general-assignment
    case): stale bits must be cleared, not left add-only."""
    p = Problem("difference")
    s1 = Fluent("s1", _st(Pkg), t=Truck)
    s2 = Fluent("s2", _st(Pkg), t=Truck)
    s3 = Fluent("s3", _st(Pkg), t=Truck)
    for f in (s1, s2, s3):
        p.add_fluent(f, default_initial_value=_em().Set([]))
    t1 = Object("t1", Truck); p.add_object(t1)
    a, b, c = _pkgs(p, ["a", "b", "c"])
    p.set_initial_value(s1(t1), _em().Set([_em().ObjectExp(a), _em().ObjectExp(b)]))
    p.set_initial_value(s2(t1), _em().Set([_em().ObjectExp(b)]))
    # pre-populate s3 with a stale member c that the difference must drop
    p.set_initial_value(s3(t1), _em().Set([_em().ObjectExp(c)]))
    diff = InstantaneousAction("diff", t=Truck)
    t = diff.parameter("t")
    diff.add_effect(s3(t), _em().SetDifference(s1(t), s2(t)))  # s3 := {a,b}\{b} = {a}
    p.add_action(diff)
    # goal: a is in s3, but b and c are not (stale c cleared) — total card 1
    p.add_goal(And(_em().SetMember(_em().ObjectExp(a), s3(t1)),
                   Not(_em().SetMember(_em().ObjectExp(c), s3(t1))),
                   Equals(_em().SetCardinality(s3(t1)), Int(1))))
    _solve_ok(p)


def test_static_set_fluent():
    """A static set fluent (never assigned) used in a member precondition:
    becomes static boolean cells and is folded by StaticFluentPass."""
    p = Problem("static_set")
    connects = Fluent("connects", _st(Loc), l=Loc)   # static
    at = Fluent("at", BoolType(), l=Loc)
    p.add_fluent(connects, default_initial_value=_em().Set([]))
    p.add_fluent(at, default_initial_value=False)
    l1, l2 = Object("l1", Loc), Object("l2", Loc)
    p.add_objects([l1, l2])
    p.set_initial_value(connects(l1), _em().Set([_em().ObjectExp(l2)]))
    p.set_initial_value(at(l1), True)
    move = InstantaneousAction("move", frm=Loc, to=Loc)
    frm, to = move.parameter("frm"), move.parameter("to")
    move.add_precondition(at(frm))
    move.add_precondition(_em().SetMember(to, connects(frm)))   # l2 ∈ connects(l1)
    move.add_effect(at(frm), False)
    move.add_effect(at(to), True)
    p.add_action(move)
    p.add_goal(at(l2))
    _solve_ok(p)


def test_initial_constant_set_horizon0():
    """An initial constant set already satisfies a cardinality goal (no action
    needed) — checks initial-state normalisation."""
    p = Problem("init0")
    pin = Fluent("pin", _st(Pkg), t=Truck)
    p.add_fluent(pin, default_initial_value=_em().Set([]))
    t1 = Object("t1", Truck); p.add_object(t1)
    a, b = _pkgs(p, ["a", "b"])
    p.set_initial_value(pin(t1), _em().Set([_em().ObjectExp(a), _em().ObjectExp(b)]))
    nop = InstantaneousAction("noop")
    nop.add_effect(Fluent("flag", BoolType())(), True)
    p.add_fluent(Fluent("flag", BoolType()), default_initial_value=False)
    p.add_action(nop)
    p.add_goal(GE(_em().SetCardinality(pin(t1)), Int(2)))
    plan = _solve_ok(p)
    assert len(plan.actions) == 0, f"expected empty plan, got {plan}"


# ── parallelism ──────────────────────────────────────────────────────────────

def test_parallel_different_element_adds_exists_step():
    """Two loads of DIFFERENT packages co-fire in one exists-step: the cells are
    independent booleans, so the plan validates (sets analogue of arrays'
    test_parallel_different_cells_exists_step)."""
    p = Problem("par")
    held = Fluent("held", _st(Pkg), t=Truck)
    p.add_fluent(held, default_initial_value=_em().Set([]))
    t1 = Object("t1", Truck); p.add_object(t1)
    a, b = _pkgs(p, ["a", "b"])
    load = InstantaneousAction("load", t=Truck, pk=Pkg)
    t, pk = load.parameter("t"), load.parameter("pk")
    load.add_effect(held(t), _em().SetAdd(pk, held(t)))
    p.add_action(load)
    p.add_goal(And(_em().SetMember(_em().ObjectExp(a), held(t1)),
                   _em().SetMember(_em().ObjectExp(b), held(t1))))
    plan = _solve_ok(p, strategy="els")
    assert len(plan.actions) == 2


# ── ill-formed models → clean rejection ──────────────────────────────────────

def test_reject_set_typed_parameter():
    def build():
        p = Problem("sp")
        p.add_fluent(Fluent("f", BoolType()), default_initial_value=False)
        a = InstantaneousAction("a", s=_st(Pkg))   # set-typed param
        a.add_effect(Fluent("f", BoolType())(), True)
        p.add_action(a)
        return p
    _assert_rejected(build)


def test_reject_int_element_set():
    def build():
        p = Problem("ie")
        p.add_fluent(Fluent("s", _st(IntType(0, 5))),
                     default_initial_value=_em().Set([]))
        return p
    _assert_rejected(build)


def test_reject_nested_set_type():
    def build():
        p = Problem("ns")
        p.add_fluent(Fluent("s", _st(_st(Pkg))),
                     default_initial_value=_em().Set([]))
        return p
    _assert_rejected(build)


def test_fluent_valued_element():
    """`member(locof(t), S)` — a fluent-valued element, formerly rejected at
    conversion — is resolved by the C++ TermResolutionPass into a ghost
    parameter pinned by an equality precondition before set lowering
    (docs/nesting-status-and-gaps.md §4.2), so it now solves. The simulator
    oracle does not evaluate object fluents, so this asserts plan shape
    directly (the ghost machinery itself is UP-validated in
    tests/api/nested_terms.py)."""
    p = Problem("fe")
    locof = Fluent("locof", Pkg, t=Truck)
    S = Fluent("S", _st(Pkg))
    g = Fluent("g", BoolType())
    p.add_fluent(locof)
    p.add_fluent(S, default_initial_value=_em().Set([]))
    p.add_fluent(g, default_initial_value=False)
    t1 = Object("t1", Truck); p.add_object(t1)
    pa, = _pkgs(p, ["a"])
    p.set_initial_value(locof(t1), pa)
    p.set_initial_value(S(), _em().Set([_em().ObjectExp(pa)]))
    pick = InstantaneousAction("pick", t=Truck)
    pick.add_precondition(_em().SetMember(locof(pick.parameter("t")), S()))
    pick.add_effect(g(), True)
    p.add_action(pick)
    p.add_goal(g())
    r = solve(p)
    assert r.plan is not None, f"fluent-valued element did not solve ({r.status})"
    assert "pick" in [ai.action.name for ai in r.plan.actions]


def test_reject_array_of_set():
    def build():
        from unified_planning.shortcuts import ArrayType
        p = Problem("aos")
        p.add_fluent(Fluent("s", ArrayType(2, _st(Pkg))))
        return p
    _assert_rejected(build)


def test_symmetries_with_set_fluents():
    """`--symmetries` used to crash on set problems: the detection snapshot
    was taken before SetLoweringPass, so its Z3 equivalence formula met raw
    SET_* nodes ("Wrong number of arguments ... set_constant"). The snapshot
    is now taken after the canonicalize lowerings (all-scalar, still lifted,
    pre-StaticFluent), so detection runs and sees the interchangeable
    packages. Pins solve + simulator validity with symmetries enabled."""
    from unified_planning.shortcuts import OneshotPlanner
    p = Problem("sym_sets")
    held = Fluent("held", _st(Pkg), t=Truck)
    p.add_fluent(held, default_initial_value=_em().Set([]))
    t1 = Object("t1", Truck); p.add_object(t1)
    _pkgs(p, ["a", "b"])
    load = InstantaneousAction("load", t=Truck, pk=Pkg)
    t, pk = load.parameter("t"), load.parameter("pk")
    load.add_precondition(Not(_em().SetMember(pk, held(t))))
    load.add_effect(held(t), _em().SetAdd(pk, held(t)))
    p.add_action(load)
    p.add_goal(GE(_em().SetCardinality(held(t1)), Int(2)))
    with OneshotPlanner(name="rantanplan",
                        params={"verbosity": "silent",
                                "symmetries": True}) as pl:
        r = pl.solve(p)
    assert r.plan is not None, f"symmetries+sets did not solve ({r.status})"
    assert _simulate_ok(p, r.plan), "plan fails set-semantics simulation"


def test_up_grounding_rejected():
    """--up-grounding is rejected for set problems (membership cells are only
    validated through native grounding)."""
    from rantanplan.converter import convert_problem
    from rantanplan import _rantanplan
    p = Problem("ug")
    held = Fluent("held", _st(Pkg), t=Truck)
    p.add_fluent(held, default_initial_value=_em().Set([]))
    t1 = Object("t1", Truck); p.add_object(t1)
    _pkgs(p, ["a"])
    load = InstantaneousAction("load", t=Truck, pk=Pkg)
    load.add_effect(held(load.parameter("t")),
                    _em().SetAdd(load.parameter("pk"), held(load.parameter("t"))))
    p.add_action(load)
    p.add_goal(_em().SetMember(_em().ObjectExp(p.object("a")), held(t1)))
    ir = convert_problem(p)
    try:
        _rantanplan.preprocess(ir, ["--strategy", "seq", "--verbosity", "silent",
                                    "--up-grounding"])
    except Exception:
        return
    raise AssertionError("expected --up-grounding to be rejected for set problems")


CASES = [
    # semantics
    test_member_precondition_and_add,
    test_remove_effect,
    test_cardinality_vs_constant_pb,
    test_cardinality_vs_cardinality_sum,
    test_union_onto_self_and_clear,
    test_assign_constant_set,
    test_subseteq_precondition,
    test_disjoint_and_equality_goal,
    test_card_of_union_goal,
    test_difference_effect_third_set,
    test_static_set_fluent,
    test_initial_constant_set_horizon0,
    # parallelism
    test_parallel_different_element_adds_exists_step,
    # ill-formed models → clean rejection
    test_reject_set_typed_parameter,
    test_reject_int_element_set,
    test_reject_nested_set_type,
    test_fluent_valued_element,
    test_reject_array_of_set,
    test_symmetries_with_set_fluents,
    test_up_grounding_rejected,
]
