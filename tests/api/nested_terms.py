"""Nested fluent terms (FSTRIPS composition — TermResolutionPass,
docs/nesting-status-and-gaps.md §4.2).

A fluent application used as an *argument* of another fluent application
(`safe(loc(r))`, `loc(owner(p)) := l`) is compiled away in C++ by ghost
parameters: a fresh action parameter per distinct inner term, pinned by an
equality precondition, enumerated by the grounder's equality join over the
term's reachable values. Ghost parameters ride ground instances as trailing
parameters and are dropped by the plan mapping, so returned plans carry the
original signatures — and UP's PlanValidator evaluates object-fluent problems
natively, so plans here are validated against the original problem (the
gold-standard oracle; the array/set interaction cases assert plan shape only,
since UP ships no validator for those).

Rejection cases pin the v1 line: goals cannot host a ghost parameter, and
inner terms must be object-valued (state-dependent array indices stay
rejected — that is the N2 axis, covered in tests/api/arrays.py).
"""
from unified_planning.shortcuts import (
    Fluent, InstantaneousAction, Problem, BoolType, UserType, Object, Equals,
    And,
)
from unified_planning.exceptions import UPException
from unified_planning.engines.results import PlanGenerationResultStatus

from ..harness import solve, assert_valid

# Bounded types need the Grounded encoder: the default strategy (elsc) refuses
# declared bounds, so the cases that declare them name a Grounded strategy.
GROUNDED = "e"


Room, Robot, Pkg = UserType("Room"), UserType("Robot"), UserType("Pkg")


def _rooms(p, names):
    objs = [Object(n, Room) for n in names]
    p.add_objects(objs)
    return objs


def test_nested_precondition():
    """`safe(loc(r))` as a precondition: the inner object-fluent read becomes
    a ghost parameter; the plan must still be UP-valid on the original
    problem."""
    p = Problem("nested_pre")
    r1 = Object("r1", Robot)
    p.add_object(r1)
    ra, rb = _rooms(p, ["ra", "rb"])
    loc = Fluent("loc", Room, r=Robot)
    safe = Fluent("safe", BoolType(), m=Room)
    done = Fluent("done", BoolType())
    p.add_fluent(loc)
    p.add_fluent(safe, default_initial_value=False)
    p.add_fluent(done, default_initial_value=False)
    p.set_initial_value(loc(r1), ra)
    p.set_initial_value(safe(ra), True)

    work = InstantaneousAction("work", r=Robot)
    work.add_precondition(safe(loc(work.parameter("r"))))  # nested term
    work.add_effect(done(), True)
    p.add_action(work)
    p.add_goal(done())

    res = solve(p)
    assert res.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {res.status}"
    assert "work" in [ai.action.name for ai in res.plan.actions]
    assert_valid(p, res.plan)


def test_nested_precondition_blocks():
    """The mirror case: the robot's room is NOT safe, so the ghost equality
    pins the inner term to an unsafe room and the action is inapplicable —
    the problem is proven unsolvable (no false positive from the ghost
    ranging over other rooms)."""
    p = Problem("nested_pre_blocks")
    r1 = Object("r1", Robot)
    p.add_object(r1)
    ra, rb = _rooms(p, ["ra", "rb"])
    loc = Fluent("loc", Room, r=Robot)
    safe = Fluent("safe", BoolType(), m=Room)
    done = Fluent("done", BoolType())
    p.add_fluent(loc)
    p.add_fluent(safe, default_initial_value=False)
    p.add_fluent(done, default_initial_value=False)
    p.set_initial_value(loc(r1), ra)
    p.set_initial_value(safe(rb), True)  # safe room != robot's room

    work = InstantaneousAction("work", r=Robot)
    work.add_precondition(safe(loc(work.parameter("r"))))
    work.add_effect(done(), True)
    p.add_action(work)
    p.add_goal(done())

    res = solve(p)
    assert res.status == PlanGenerationResultStatus.UNSOLVABLE_PROVEN, \
        f"expected UNSOLVABLE_PROVEN, got {res.status}"


def test_nested_write_target_up_rejected():
    """`loc(owner(pk)) := m` — a write through a computed cell. The C++ pass
    supports this shape, but the x-unified-planning *model layer* refuses to
    construct it (UPProblemDefinitionError: 'contains other fluents in his
    arguments'), so it is unreachable through the UP API today. This pins the
    upstream behaviour: if the fork ever starts accepting nested effect
    targets, this test flips and tells us to add the end-to-end coverage."""
    from unified_planning.exceptions import UPProblemDefinitionError
    loc = Fluent("loc", Room, r=Robot)
    owner = Fluent("owner", Robot, pk=Pkg)
    recall = InstantaneousAction("recall", pk=Pkg, m=Room)
    try:
        recall.add_effect(loc(owner(recall.parameter("pk"))),
                          recall.parameter("m"))
    except UPProblemDefinitionError:
        return
    raise AssertionError(
        "UP now accepts nested effect targets — wire them through "
        "(TermResolutionPass already resolves nested targets) and replace "
        "this test with an end-to-end solve+validate case.")


def test_nested_effect_value():
    """A nested term inside an effect *value*: `loc(r) := home(owner(pk))`."""
    p = Problem("nested_value")
    r1 = Object("r1", Robot)
    p.add_object(r1)
    ra, rb = _rooms(p, ["ra", "rb"])
    k1 = Object("k1", Pkg)
    p.add_object(k1)
    loc = Fluent("loc", Room, r=Robot)
    owner = Fluent("owner", Robot, pk=Pkg)
    home = Fluent("home", Room, r=Robot)
    p.add_fluent(loc)
    p.add_fluent(owner)
    p.add_fluent(home)
    p.set_initial_value(loc(r1), ra)
    p.set_initial_value(owner(k1), r1)
    p.set_initial_value(home(r1), rb)

    ret = InstantaneousAction("return_home", r=Robot, pk=Pkg)
    ret.add_effect(loc(ret.parameter("r")),
                   home(owner(ret.parameter("pk"))))  # nested value
    p.add_action(ret)
    p.add_goal(Equals(loc(r1), rb))

    res = solve(p)
    assert res.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {res.status}"
    assert_valid(p, res.plan)


def test_two_level_nesting():
    """`safe(loc(owner(pk)))` — innermost-first resolution chains two ghost
    equalities (owner(pk) = g0, loc(g0) = g1, safe(g1))."""
    p = Problem("nested_two_levels")
    r1 = Object("r1", Robot)
    p.add_object(r1)
    ra, = _rooms(p, ["ra"])
    k1 = Object("k1", Pkg)
    p.add_object(k1)
    loc = Fluent("loc", Room, r=Robot)
    owner = Fluent("owner", Robot, pk=Pkg)
    safe = Fluent("safe", BoolType(), m=Room)
    done = Fluent("done", BoolType())
    p.add_fluent(loc)
    p.add_fluent(owner)
    p.add_fluent(safe, default_initial_value=False)
    p.add_fluent(done, default_initial_value=False)
    p.set_initial_value(loc(r1), ra)
    p.set_initial_value(owner(k1), r1)
    p.set_initial_value(safe(ra), True)

    check = InstantaneousAction("check", pk=Pkg)
    check.add_precondition(safe(loc(owner(check.parameter("pk")))))
    check.add_effect(done(), True)
    p.add_action(check)
    p.add_goal(done())

    res = solve(p)
    assert res.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {res.status}"
    assert_valid(p, res.plan)


def test_shared_inner_term():
    """Two occurrences of the same inner term in one action share one ghost
    (observable only as 'it still solves and validates', but it also pins the
    memoization path)."""
    p = Problem("nested_shared")
    r1 = Object("r1", Robot)
    p.add_object(r1)
    ra, = _rooms(p, ["ra"])
    loc = Fluent("loc", Room, r=Robot)
    safe = Fluent("safe", BoolType(), m=Room)
    lit = Fluent("lit", BoolType(), m=Room)
    done = Fluent("done", BoolType())
    p.add_fluent(loc)
    p.add_fluent(safe, default_initial_value=False)
    p.add_fluent(lit, default_initial_value=False)
    p.add_fluent(done, default_initial_value=False)
    p.set_initial_value(loc(r1), ra)
    p.set_initial_value(safe(ra), True)
    p.set_initial_value(lit(ra), True)

    work = InstantaneousAction("work", r=Robot)
    r = work.parameter("r")
    work.add_precondition(safe(loc(r)))
    work.add_precondition(lit(loc(r)))  # same inner term again
    work.add_effect(done(), True)
    p.add_action(work)
    p.add_goal(done())

    res = solve(p)
    assert res.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {res.status}"
    assert_valid(p, res.plan)


def test_nested_after_move():
    """The ghost join must track *reachable* values through state change: the
    robot starts in an unsafe room and must move first — a two-step plan whose
    second step's ghost value only becomes reachable via the first."""
    p = Problem("nested_after_move")
    r1 = Object("r1", Robot)
    p.add_object(r1)
    ra, rb = _rooms(p, ["ra", "rb"])
    loc = Fluent("loc", Room, r=Robot)
    safe = Fluent("safe", BoolType(), m=Room)
    done = Fluent("done", BoolType())
    p.add_fluent(loc)
    p.add_fluent(safe, default_initial_value=False)
    p.add_fluent(done, default_initial_value=False)
    p.set_initial_value(loc(r1), ra)
    p.set_initial_value(safe(rb), True)

    move = InstantaneousAction("move", r=Robot, to=Room)
    move.add_effect(loc(move.parameter("r")), move.parameter("to"))
    p.add_action(move)

    work = InstantaneousAction("work", r=Robot)
    work.add_precondition(safe(loc(work.parameter("r"))))
    work.add_effect(done(), True)
    p.add_action(work)
    p.add_goal(done())

    res = solve(p)
    assert res.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {res.status}"
    names = [ai.action.name for ai in res.plan.actions]
    assert "work" in names and "move" in names, f"unexpected plan: {names}"
    assert_valid(p, res.plan)


def test_array_cell_inner_term():
    """The labyrinth shape: an array cell as inner term —
    `connections(card_at[0], d)`. The ghost equality lands on the array cell
    and is rewritten by array lowering. UP ships no array validator, so this
    asserts plan shape."""
    from unified_planning.shortcuts import ArrayType
    Card, Dir = UserType("Card"), UserType("Dir")

    p = Problem("nested_array_cell")
    c1, c2 = Object("c1", Card), Object("c2", Card)
    north = Object("north", Dir)
    p.add_objects([c1, c2, north])
    card_at = Fluent("card_at", ArrayType(2, Card))
    connections = Fluent("connections", BoolType(), c=Card, d=Dir)
    done = Fluent("done", BoolType())
    p.add_fluent(card_at)
    p.add_fluent(connections, default_initial_value=False)
    p.add_fluent(done, default_initial_value=False)
    p.set_initial_value(card_at, [c1, c2])  # whole-array literal init
    p.set_initial_value(connections(c1, north), True)

    go = InstantaneousAction("go", d=Dir)
    go.add_precondition(connections(card_at[0], go.parameter("d")))
    go.add_effect(done(), True)
    p.add_action(go)
    p.add_goal(done())

    res = solve(p)
    assert res.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {res.status}"
    assert "go" in [ai.action.name for ai in res.plan.actions]


def test_goal_nested_term_expanded():
    """A nested term in a *goal* has no action to host a ghost — it is
    expanded by value-universe disjunction instead
    (`⋁_o (loc(r1) = o ∧ safe(o))`). The robot starts in an unsafe room, so
    achieving the goal needs a move — pinning that the expansion tracks
    state, not just the initial value."""
    p = Problem("nested_goal")
    r1 = Object("r1", Robot)
    p.add_object(r1)
    ra, rb = _rooms(p, ["ra", "rb"])
    loc = Fluent("loc", Room, r=Robot)
    safe = Fluent("safe", BoolType(), m=Room)
    p.add_fluent(loc)
    p.add_fluent(safe, default_initial_value=False)
    p.set_initial_value(loc(r1), ra)
    p.set_initial_value(safe(rb), True)

    move = InstantaneousAction("move", r=Robot, to=Room)
    move.add_effect(loc(move.parameter("r")), move.parameter("to"))
    p.add_action(move)
    p.add_goal(safe(loc(r1)))  # nested in goal — expanded, not rejected

    res = solve(p)
    assert res.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {res.status}"
    assert "move" in [ai.action.name for ai in res.plan.actions]
    assert_valid(p, res.plan)


# ── N2: state-dependent array indices (per-position instance splitting,
# docs/specs/undefinedness-semantics.md regime R2) ──────────────────────────


def _int_array_problem(name, values, ptr_init, ptr_hi=None):
    """A 3-cell int array `arr`, an int pointer fluent `ptr`, and a `done`
    flag. `ptr_hi` widens ptr's declared range beyond the array (OOB states
    expressible)."""
    from unified_planning.shortcuts import ArrayType, IntType
    em = _em()
    p = Problem(name)
    arr = Fluent("arr", ArrayType(3, IntType(0, 9)))
    ptr = Fluent("ptr", IntType(0, ptr_hi if ptr_hi is not None else 2))
    done = Fluent("done", BoolType())
    p.add_fluent(arr, default_initial_value=0)
    p.add_fluent(ptr, default_initial_value=0)
    p.add_fluent(done, default_initial_value=False)
    p.set_initial_value(arr, values)
    p.set_initial_value(ptr, ptr_init)
    return p, arr, ptr, done, em


def _em():
    from unified_planning.environment import get_environment
    return get_environment().expression_manager


def test_state_dependent_index_read():
    """`arr[ptr] = 7` as a precondition: ptr=1 and arr[1]=7 initially, so the
    split instance `ptr = 1 ∧ arr[1] = 7` fires."""
    from unified_planning.shortcuts import Int
    p, arr, ptr, done, em = _int_array_problem("n2_read", [3, 7, 5], 1)
    a = InstantaneousAction("probe")
    a.add_precondition(Equals(em.ArrayIndex(arr(), ptr()), Int(7)))
    a.add_effect(done(), True)
    p.add_action(a)
    p.add_goal(done())
    res = solve(p, strategy=GROUNDED)
    assert res.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {res.status}"
    assert "probe" in [ai.action.name for ai in res.plan.actions]


def test_state_dependent_index_read_blocks():
    """The mirror case: ptr points at a cell that does NOT hold 7, so no
    in-range instance is applicable and the problem is unsolvable (the split
    must not let the ghost position drift from ptr's actual value)."""
    from unified_planning.shortcuts import Int
    p, arr, ptr, done, em = _int_array_problem("n2_read_blocks", [3, 7, 5], 2)
    a = InstantaneousAction("probe")
    a.add_precondition(Equals(em.ArrayIndex(arr(), ptr()), Int(7)))
    a.add_effect(done(), True)
    p.add_action(a)
    p.add_goal(done())
    res = solve(p)
    assert res.status == PlanGenerationResultStatus.UNSOLVABLE_PROVEN, \
        f"expected UNSOLVABLE_PROVEN, got {res.status}"


def test_state_dependent_index_write_up_rejected():
    """`arr[ptr] := 9` — a write through a state-dependent index. The C++
    split resolves this shape (same machinery as reads), but the
    x-unified-planning model layer refuses to construct a fluent-containing
    effect target (`UPProblemDefinitionError`), exactly as for nested-fluent
    write targets. Pins the upstream behaviour; flips when the fork starts
    accepting it."""
    from unified_planning.shortcuts import ArrayType, IntType, Int
    from unified_planning.exceptions import UPProblemDefinitionError
    em = _em()
    arr = Fluent("arr", ArrayType(3, IntType(0, 9)))
    ptr = Fluent("ptr", IntType(0, 2))
    a = InstantaneousAction("stamp")
    try:
        a.add_effect(em.ArrayIndex(arr(), ptr()), Int(9))
    except UPProblemDefinitionError:
        return
    raise AssertionError(
        "UP now accepts state-dependent index write targets — add the "
        "end-to-end solve coverage (the split already handles targets).")


def test_state_dependent_index_oob_inapplicable():
    """Non-strict read-of-⊥: ptr's declared range exceeds the array and the
    initial state pins it out of bounds (ptr = 5, array size 3). The
    unconditional read makes every in-range instance inapplicable and the
    boundary instance is discarded (unconditional ⊥) — unsolvable, not an
    error."""
    from unified_planning.shortcuts import Int
    p, arr, ptr, done, em = _int_array_problem(
        "n2_oob", [3, 7, 5], 5, ptr_hi=9)
    a = InstantaneousAction("probe")
    a.add_precondition(Equals(em.ArrayIndex(arr(), ptr()), Int(7)))
    a.add_effect(done(), True)
    p.add_action(a)
    p.add_goal(done())
    res = solve(p)
    assert res.status == PlanGenerationResultStatus.UNSOLVABLE_PROVEN, \
        f"expected UNSOLVABLE_PROVEN, got {res.status}"


def test_boundary_or_absorption():
    """Def-9 OR absorption at runtime: precondition `arr[ptr] = 7 ∨ done0`
    with ptr out of bounds. The boundary instance rewrites the ⊥ read per
    Def. 9, keeping the other disjunct — the action stays applicable via
    `done0` even though the read is undefined."""
    from unified_planning.shortcuts import Int, Or
    p, arr, ptr, done, em = _int_array_problem(
        "n2_or", [3, 7, 5], 5, ptr_hi=9)
    done0 = Fluent("done0", BoolType())
    p.add_fluent(done0, default_initial_value=False)
    p.set_initial_value(done0(), True)
    a = InstantaneousAction("probe")
    a.add_precondition(Or(Equals(em.ArrayIndex(arr(), ptr()), Int(7)),
                          done0()))
    a.add_effect(done(), True)
    p.add_action(a)
    p.add_goal(done())
    res = solve(p, strategy=GROUNDED)
    assert res.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED (OR absorbs the ⊥ read), got {res.status}"


def test_inner_state_dependent_index():
    """`arr[idx[0]]` — the index is itself an array read: the inner constant
    access resolves through the split equality (`idx[0] = p`)."""
    from unified_planning.shortcuts import ArrayType, IntType, Int
    em = _em()
    p = Problem("n2_inner")
    arr = Fluent("arr", ArrayType(3, IntType(0, 9)))
    idx = Fluent("idx", ArrayType(2, IntType(0, 2)))
    done = Fluent("done", BoolType())
    p.add_fluent(arr, default_initial_value=0)
    p.add_fluent(idx, default_initial_value=0)
    p.add_fluent(done, default_initial_value=False)
    p.set_initial_value(arr, [3, 7, 5])
    p.set_initial_value(idx, [2, 0])
    a = InstantaneousAction("probe")
    a.add_precondition(Equals(
        em.ArrayIndex(arr(), em.ArrayIndex(idx(), Int(0))), Int(5)))
    a.add_effect(done(), True)
    p.add_action(a)
    p.add_goal(done())
    res = solve(p, strategy=GROUNDED)
    assert res.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {res.status}"


def test_index_after_pointer_move():
    """Two-step: an action moves ptr onto the 7-cell, then the probe fires —
    the split equality must track state change, not just the initial value."""
    from unified_planning.shortcuts import Int
    p, arr, ptr, done, em = _int_array_problem("n2_move", [3, 7, 5], 0)
    step = InstantaneousAction("advance")
    step.add_effect(ptr(), Int(1))
    p.add_action(step)
    a = InstantaneousAction("probe")
    a.add_precondition(Equals(em.ArrayIndex(arr(), ptr()), Int(7)))
    a.add_effect(done(), True)
    p.add_action(a)
    p.add_goal(done())
    res = solve(p, strategy=GROUNDED)
    assert res.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {res.status}"
    names = [ai.action.name for ai in res.plan.actions]
    assert "advance" in names and "probe" in names, f"unexpected plan: {names}"


CASES = [
    test_nested_precondition,
    test_nested_precondition_blocks,
    test_nested_write_target_up_rejected,
    test_nested_effect_value,
    test_two_level_nesting,
    test_shared_inner_term,
    test_nested_after_move,
    test_array_cell_inner_term,
    test_goal_nested_term_expanded,
    test_state_dependent_index_read,
    test_state_dependent_index_read_blocks,
    test_state_dependent_index_write_up_rejected,
    test_state_dependent_index_oob_inapplicable,
    test_boundary_or_absorption,
    test_inner_state_dependent_index,
    test_index_after_pointer_move,
]
