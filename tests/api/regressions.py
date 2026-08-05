"""Targeted regression guards for specific bugs that were fixed and must not
resurface. Unlike the other modules here, these are not a *feature* — each case
reproduces one historical bug in the smallest form that fails without the fix.
They live in the API suite so they run on every invocation, including
``--quick`` (the full PDDL sweep that also covers them is skipped there).
"""
from unified_planning.shortcuts import (
    Fluent, InstantaneousAction, Problem, BoolType, IntType, UserType, Object,
    Equals, GE, Not,
)
from unified_planning.engines import PlanGenerationResultStatus

from ..harness import solve, assert_valid


def _metric(result, key):
    """A stats value off a PlanGenerationResult, as float, or None. Every stat
    reaches result.metrics (stringified) on every solve — no --stats-file
    needed — so these are assertable without touching the filesystem."""
    if not result.metrics:
        return None
    raw = result.metrics.get(key)
    return None if raw is None else float(raw)


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


def test_axioms_rejected_as_unsupported():
    """RantanPlan has no axiom / derived-predicate support and must reject such a
    problem cleanly (UNSUPPORTED_PROBLEM naming DERIVED_FLUENTS) rather than
    crashing deep in conversion (historically an `AttributeError: '_DerivedBoolType'
    object has no attribute 'name'`). supported_kind() omits DERIVED_FLUENTS, so
    the deny-list gate in the wrapper rejects before conversion."""
    from unified_planning.shortcuts import DerivedBoolType
    from unified_planning.model import Axiom

    flag = Fluent("flag", BoolType())
    derived = Fluent("derived", DerivedBoolType())
    p = Problem("has_axiom")
    p.add_fluent(flag, default_initial_value=False)
    p.add_fluent(derived, default_initial_value=False)
    ax = Axiom("ax")
    ax.set_head(derived)
    ax.add_body_condition(flag)
    p.add_axiom(ax)
    setf = InstantaneousAction("setf")
    setf.add_effect(flag, True)
    p.add_action(setf)
    p.add_goal(derived)

    r = solve(p)
    assert r.status == PlanGenerationResultStatus.UNSUPPORTED_PROBLEM, r.status
    assert any("DERIVED_FLUENTS" in m.message for m in (r.log_messages or [])), \
        "the rejection should name the unsupported DERIVED_FLUENTS feature"


def test_object_fluent_default_initial_value():
    """An object (UserType) fluent's ``default_initial_value`` must be honoured
    for instances the initial state leaves unset — and be visible to the
    *grounder*, not only the encoder.

        hand : Colour, default_initial_value = W   (no explicit initial value)
        paint : precondition hand == W;  effect hand := R
        goal  : hand == R                          (solvable: paint)

    Two historical faults, both here:
      * the converter dropped object-fluent defaults entirely, so CWA raised
        "object fluents have no default"; and
      * once carried, the default was materialised only by the CWA pass, which
        runs *after* grounding — so the reachability grounder never saw
        ``hand == W``, pruned ``paint`` (its precondition unmatched), and
        "proved" the problem unsolvable (UNSOLVABLE_PROVEN).

    Fixes: the converter carries the object default onto the fluent's declared
    default, and the ObjectFluentIndex seeds declared object-fluent defaults for
    every ground instance the initial state leaves unset, before the grounding
    fixpoint runs."""
    Colour = UserType("Colour")
    R, W = Object("R", Colour), Object("W", Colour)
    hand = Fluent("hand", Colour)
    p = Problem("object_fluent_default")
    p.add_fluent(hand, default_initial_value=W)  # object default, no explicit init
    p.add_object(R)
    p.add_object(W)
    paint = InstantaneousAction("paint")
    paint.add_precondition(Equals(hand(), W))
    paint.add_effect(hand(), R)
    p.add_action(paint)
    p.add_goal(Equals(hand(), R))

    r = solve(p)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {r.status} (object-fluent default mishandled again)"
    assert [ai.action.name for ai in r.plan.actions] == ["paint"]
    assert_valid(p, r.plan)


def test_object_fluent_assign_from_fluent_reachability():
    """A fluent-valued object assign (``b := a``) must propagate the source's
    reachable values in the grounding relaxation. Smallest failing form:

        a, b : Obj, init a = o2, b = o1
        copy   : b := a
        finish : precondition b == o2;  effect done := true
        goal   : done                    (solvable: copy; finish)

    Historically the ObjectFluentIndex accumulated only *constant* assign
    values (``b := o2``); a fluent RHS was silently skipped, so ``b`` never
    gained ``a``'s values, the equality join found no tuple for ``finish``'s
    precondition, ``finish`` was never instantiated, and a trivially solvable
    problem returned UNSOLVABLE_PROVEN — despite
    FLUENTS_IN_OBJECT_ASSIGNMENTS being declared in supported_kind(). The
    object-fluent face of the value-chaining family pinned by
    test_boolean_assign_from_fluent.

    The chained variant (``c := b`` declared *before* ``b := a``) additionally
    requires re-applying object-fluent effects of all ground actions to
    fixpoint each grounding iteration (like numeric effects): the copy is
    processed before its source gains values, so a single collection at
    instantiation time misses the propagation.

    Fixes: collect_object_fluent_effects resolves a fluent RHS to the source's
    reachable value set (whole-type-universe fallback for any other
    non-constant shape), and ground() re-applies object-fluent effects across
    all accepted actions until the value sets stabilise."""
    Obj = UserType("Obj")
    o1, o2 = Object("o1", Obj), Object("o2", Obj)

    def build(chain: bool) -> Problem:
        a = Fluent("a", Obj)
        b = Fluent("b", Obj)
        done = Fluent("done", BoolType())
        p = Problem("objfluent_chain" if chain else "objfluent_copy")
        p.add_objects([o1, o2])
        p.add_fluent(a)
        p.add_fluent(b)
        p.add_fluent(done, default_initial_value=False)
        tested = b
        if chain:
            c = Fluent("c", Obj)
            p.add_fluent(c)
            p.set_initial_value(c(), o1)
            # Declared first on purpose: its source (b) only gains o2 after
            # copy_ba is processed later in the same grounding iteration.
            copy_cb = InstantaneousAction("copy_cb")
            copy_cb.add_effect(c(), b())
            p.add_action(copy_cb)
            tested = c
        copy_ba = InstantaneousAction("copy_ba")
        copy_ba.add_effect(b(), a())
        p.add_action(copy_ba)
        finish = InstantaneousAction("finish")
        finish.add_precondition(Equals(tested(), o2))
        finish.add_effect(done(), True)
        p.add_action(finish)
        p.set_initial_value(a(), o2)
        p.set_initial_value(b(), o1)
        p.add_goal(done())
        return p

    for chain in (False, True):
        p = build(chain)
        r = solve(p)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"chain={chain}: expected SOLVED, got {r.status} " \
            f"(fluent-valued object assign invisible to grounding again)"
        assert_valid(p, r.plan)


def _two_writes_to_one_fluent_problem():
    """One action writing ONE fluent twice, both writes guarded by the same
    (satisfiable) condition, so both must fire and their deltas must accumulate.

        both : WHEN flag: g += 1
               WHEN flag: g += 2
               done := true
        clear: PRE done   effect flag := false      (keeps `flag` dynamic)
        init  flag, g = 0;  goal  done AND g = 3

    `g = 3` is reachable only if BOTH writes are applied. The guard reads a
    fluent another action writes so neither StaticFluentPass nor the numeric
    bounds analysis can fold it to a constant — a folded guard becomes
    unconditional and changes which code path is exercised.
    """
    flag = Fluent("flag", BoolType())
    done = Fluent("done", BoolType())
    g = Fluent("g", IntType())

    both = InstantaneousAction("both")
    both.add_precondition(Not(done()))
    both.add_increase_effect(g(), 1, condition=flag())
    both.add_increase_effect(g(), 2, condition=flag())
    both.add_effect(done(), True)

    clear = InstantaneousAction("clear")
    clear.add_precondition(done())
    clear.add_effect(flag(), False)

    p = Problem("two_writes_one_fluent")
    # Declared False and set True: a boolean default of True is rejected (it
    # inverts the closed-world assumption).
    p.add_fluent(flag, default_initial_value=False)
    p.set_initial_value(flag(), True)
    p.add_fluent(done, default_initial_value=False)
    p.add_fluent(g, default_initial_value=0)
    p.add_action(both)
    p.add_action(clear)
    p.add_goal(done())
    p.add_goal(Equals(g(), 3))
    return p


def test_chained_encoder_composes_every_write_to_one_fluent():
    """ChainedGroundedEncoder must give each EFFECT its own chain link.

    It used to index writes per ACTION and take the first matching effect,
    silently ignoring the rest (the deleted get_effect_for_variable). With one
    action writing `g` twice, the per-action index also collapsed to size 1, so
    the variable did not look multi-modified and fell through to the plain
    conjoined-equalities path — which emits `g' = g+1 AND g' = g+2`, an
    unsatisfiable constraint that makes the action unselectable and the goal
    unreachable.

    Chained encoders only: under the non-chained GroundedEncoder this shape is
    still encoded as that unsatisfiable conjunction, which is a separate known
    gap (see DuplicateEffectPass's header).
    """
    for strategy in ("elsc", "flsc", "elsc-fp"):
        p = _two_writes_to_one_fluent_problem()
        r = solve(p, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{strategy}: expected SOLVED, got {r.status} — a duplicate write " \
            f"to one fluent is being dropped or made unsatisfiable again"
        assert_valid(p, r.plan)


def test_duplicate_writes_do_not_fabricate_interference():
    """Writing one fluent twice must not make an action interfere with
    everything.

    are_simply_commuting used to return "not commuting" the moment it saw a
    repeated fluent, before it had looked at the second action at all — so one
    action's internal shape became a verdict about every pair it appeared in,
    including actions it shares no fluent with. Those edges were invisible to
    the search (which only asks about pairs sharing a fluent) but visible to
    plan extraction (which asks about all n^2), and being symmetric they formed
    2-cycles that Graph::topological_sort silently emitted in the wrong order.

    `both` writes only `g`/`done`; `p_act` and `q_act` touch neither, nor each
    other. The eager semantic graph must therefore be empty.
    """
    p = _two_writes_to_one_fluent_problem()
    h = Fluent("h", IntType())
    k = Fluent("k", IntType())
    p.add_fluent(h, default_initial_value=0)
    p.add_fluent(k, default_initial_value=0)
    for name, fluent in (("p_act", h), ("q_act", k)):
        act = InstantaneousAction(name)
        act.add_increase_effect(fluent(), 1)
        p.add_action(act)

    # egsc, not egs: the eager graph is built the same way either side, but the
    # non-chained encoder still turns this action's two writes into an
    # unsatisfiable conjunction, so only the chained variant reaches a plan.
    r = solve(p, strategy="egsc")
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {r.status}"
    edges = _metric(r, "interference.semantic.edges")
    assert edges is not None, "interference.semantic.edges not reported"
    assert edges == 0, \
        f"expected an empty interference graph, got {edges} edge(s) — a unary " \
        f"property is being reported as a verdict about a pair again"


def test_effect_rhs_read_is_not_hidden_from_interference():
    """A fluent read only on an effect's RIGHT-HAND SIDE is part of the
    footprint.

    compute_action_footprint feeds both the search prefilter
    (potential_interferers) and the oracle's own early-out. It used to collect
    written fluents and PRECONDITION fluents only, so this pair looked disjoint:

        accelerate : v += 1
        move       : x += v          <- reads v, but only in the effect value

    They genuinely interfere (CHECK2_HAPPEN: the parallel happening evaluates
    `v` at the pre-state, a serialisation lets accelerate's write flow into
    move's delta), and a debug assertion found the same shape in five
    pddl/test domains. Narrowing the footprint again makes the oracle answer
    NONE here and empties the graph.
    """
    v = Fluent("v", IntType())
    x = Fluent("x", IntType())

    accelerate = InstantaneousAction("accelerate")
    accelerate.add_increase_effect(v(), 1)
    move = InstantaneousAction("move")
    move.add_increase_effect(x(), v())

    p = Problem("effect_rhs_read")
    p.add_fluent(v, default_initial_value=1)
    p.add_fluent(x, default_initial_value=0)
    p.add_action(accelerate)
    p.add_action(move)
    p.add_goal(GE(x(), 1))
    p.add_goal(GE(v(), 2))

    r = solve(p, strategy="egs")
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
        f"expected SOLVED, got {r.status}"
    edges = _metric(r, "interference.semantic.edges")
    assert edges is not None, "interference.semantic.edges not reported"
    assert edges > 0, \
        "accelerate and move must interfere: move reads `v` on its effect's " \
        "right-hand side and accelerate writes it. An empty graph means the " \
        "footprint no longer collects effect-RHS reads, which lets the search " \
        "co-schedule them"


def test_ground_atoms_have_exactly_one_expr_id():
    """One ground atom, one ExprID.

    Interning distinguishes nodes by type_id; the Z3 variable name is built from
    payload strings only. So two ids can denote one atom, and everything that
    groups by fluent id then mis-groups SILENTLY — the chained encoder builds two
    chain links for one fluent, DuplicateEffectPass counts one write as two, the
    RPG keeps two reachability entries. Measured 0 across all 66 pddl/test
    domains; the PDDL suite asserts this per instance, this covers the API path.
    """
    p = _two_writes_to_one_fluent_problem()
    r = solve(p, strategy="elsc")
    aliased = _metric(r, "pool.aliased_state_variables")
    assert aliased is not None, "pool.aliased_state_variables not reported"
    assert aliased == 0, \
        f"{aliased} ground state variable(s) have more than one ExprID"


CASES = [
    test_conditional_effect_guard_not_pruned,
    test_chained_encoder_composes_every_write_to_one_fluent,
    test_duplicate_writes_do_not_fabricate_interference,
    test_effect_rhs_read_is_not_hidden_from_interference,
    test_ground_atoms_have_exactly_one_expr_id,
    test_goal_relevance_numeric_value_chaining,
    test_goal_relevance_numeric_rhs_chaining,
    test_boolean_assign_from_fluent,
    test_object_fluent_default_initial_value,
    test_object_fluent_assign_from_fluent_reachability,
    test_axioms_rejected_as_unsupported,
]
