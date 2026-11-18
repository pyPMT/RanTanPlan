"""Targeted regression guards for specific bugs that were fixed and must not
resurface. Unlike the other modules here, these are not a *feature* — each case
reproduces one historical bug in the smallest form that fails without the fix.
They live in the API suite so they run on every invocation, including
``--quick`` (the full PDDL sweep that also covers them is skipped there).
"""
from fractions import Fraction

from unified_planning.shortcuts import (
    Fluent, InstantaneousAction, Problem, ArrayType, BoolType, IntType, RealType,
    UserType, Object, And, Count, Equals, GE, GT, Iff, Int, LE, LT, Not, Or, Plus, Times,
)
from unified_planning.model.metrics import MinimizeActionCosts
from unified_planning.engines import PlanGenerationResultStatus

from rantanplan import _rantanplan
from rantanplan.converter import convert_problem

from ..harness import solve, assert_valid

# Bounded types need the Grounded encoder: the default strategy (elsc) refuses
# declared bounds, so the cases that declare them name a Grounded strategy.
GROUNDED = "e"


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

    r = solve(p, strategy=GROUNDED)
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

    r = solve(p, strategy=GROUNDED)
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

    EffectNormalizationPass now sums the two writes into one before any encoder
    sees them, so this checks the end-to-end result, not the chained encoder's
    own composition.
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

    # EffectNormalizationPass merges `both`'s two writes into one first, so this
    # checks the end-to-end result.
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

    # egsc: semantic interference is registered only with the chained encoder.
    r = solve(p, strategy="egsc")
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
    chain links for one fluent, EffectNormalizationPass takes two writes to one
    atom for writes to two fluents, the RPG keeps two reachability entries. Measured 0 across all 66 pddl/test
    domains; the PDDL suite asserts this per instance, this covers the API path.
    """
    p = _two_writes_to_one_fluent_problem()
    r = solve(p, strategy="elsc")
    # Preprocessing stops with an error on an alias, and then no stat arrives.
    assert r.status != PlanGenerationResultStatus.INTERNAL_ERROR, \
        "; ".join(m.message for m in r.log_messages or [])
    aliased = _metric(r, "pool.aliased_state_variables")
    assert aliased is not None, "pool.aliased_state_variables not reported"
    assert aliased == 0, \
        f"{aliased} ground state variable(s) have more than one ExprID"


def test_a_non_constant_amount_does_not_bound_its_fluent():
    """tighten_bounds_syntactically (bound_tightening.cpp) took an increase by
    an amount that is not a constant as never lowering its fluent, so a fluent
    that starts at 0 got the lower bound 0. On sailing-wind-sat a move
    increases y by (* (v ?b) -0.71): y got the lower bound 0, goal relevance
    dropped the only action that saves the person at y = -71, and two solvable
    problems came out UNSOLVABLE_PROVEN. verify_upper_bound had the mirror
    image: every decrease was taken as never raising its fluent.

        y : Int, initially 0        v : Int, initially 1
        speed : v += 1               (so v is not static and the amount is not folded)
        move  : y += v * -1          (or y -= v * -1)
        save  : y <= -2 -> saved     (or y >= 2)
        goal  : saved                (solvable: move twice, save)"""
    for direction in ("increase", "decrease"):
        y, v = Fluent("y", IntType()), Fluent("v", IntType())
        saved = Fluent("saved", BoolType())
        p = Problem(f"non_constant_{direction}")
        p.add_fluent(y, default_initial_value=0)
        p.add_fluent(v, default_initial_value=1)
        p.add_fluent(saved, default_initial_value=False)
        speed = InstantaneousAction("speed")
        speed.add_increase_effect(v(), 1)
        move = InstantaneousAction("move")
        save = InstantaneousAction("save")
        if direction == "increase":
            move.add_increase_effect(y(), Times(v(), -1))
            save.add_precondition(LE(y(), -2))
        else:
            move.add_decrease_effect(y(), Times(v(), -1))
            save.add_precondition(GE(y(), 2))
        save.add_effect(saved(), True)
        for a in (speed, move, save):
            p.add_action(a)
        p.add_goal(saved())

        r = solve(p)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{direction}: expected SOLVED, got {r.status} (a bound from a " \
            f"non-constant amount again?)"
        assert_valid(p, r.plan)


def _solve_with(problem, **params):
    """Solve with the default engine and extra planner parameters."""
    from unified_planning.shortcuts import OneshotPlanner
    with OneshotPlanner(name="rantanplan",
                        params={"verbosity": "silent", **params}) as planner:
        return planner.solve(problem, timeout=20)


def _chain_problem():
    """A walk along l0 - l1 - ... - l5; the goal at(l5) needs five steps."""
    Location = UserType("Location")
    at = Fluent("at", BoolType(), l=Location)
    link = Fluent("link", BoolType(), a=Location, b=Location)
    move = InstantaneousAction("move", a=Location, b=Location)
    a, b = move.parameters
    move.add_precondition(at(a))
    move.add_precondition(link(a, b))
    move.add_effect(at(a), False)
    move.add_effect(at(b), True)
    p = Problem("chain")
    p.add_fluent(at, default_initial_value=False)
    p.add_fluent(link, default_initial_value=False)
    p.add_action(move)
    locations = [Object(f"l{i}", Location) for i in range(6)]
    p.add_objects(locations)
    for x, y in zip(locations, locations[1:]):
        p.set_initial_value(link(x, y), True)
    p.set_initial_value(at(locations[0]), True)
    p.add_goal(at(locations[-1]))
    return p


def _capped_problem():
    """x only ever holds 0 or 3, and the goal is x >= 10: unsolvable."""
    x = Fluent("x", IntType())
    set_x = InstantaneousAction("set_x")
    set_x.add_effect(x, 3)
    p = Problem("capped")
    p.add_fluent(x, default_initial_value=0)
    p.add_action(set_x)
    p.add_goal(GE(x, 10))
    return p


def test_step_limit_is_not_a_proof_of_unsolvability():
    """ActionPruning's planning graph stops at max_steps layers. Goals it has
    not reached by then may be reached later, yet a solvable problem came
    back UNSOLVABLE_PROVEN under a small max_steps. That case is now
    UNSOLVABLE_INCOMPLETELY, the status a search that runs out of steps
    returns. Unreachable at the graph's fixpoint stays a proof."""
    INCOMPLETELY = PlanGenerationResultStatus.UNSOLVABLE_INCOMPLETELY
    PROVEN = PlanGenerationResultStatus.UNSOLVABLE_PROVEN

    chain = _chain_problem()
    assert _solve_with(chain, max_steps=4).status == INCOMPLETELY
    solved = _solve_with(chain, max_steps=5)
    assert solved.status == PlanGenerationResultStatus.SOLVED_SATISFICING
    assert_valid(chain, solved.plan)

    capped = _capped_problem()
    # Grounding proves it at its own fixpoint, which no step limit cuts short.
    assert _solve_with(capped, max_steps=1).status == PROVEN
    # Without numeric grounding the planning graph has to: it sees its
    # fixpoint at the second layer, so a limit of 1 proves nothing.
    result = _solve_with(capped, numeric_grounding=False)
    assert result.status == PROVEN
    assert any("action-pruning" in m.message for m in result.log_messages), \
        result.log_messages
    assert _solve_with(capped, numeric_grounding=False, max_steps=1).status == INCOMPLETELY


def _residual_contributor_problem():
    """x must reach 10 from 0, and an action that is not an achiever of
    x >= 10 supplies most of it:

        big   : PRE x <= 0   effect ready := true, x += 9
        small : PRE ready    effect x += 1
        init  x = 0;  goal  x >= 10          (solvable in two steps: big, small)

    Only `small` can make x >= 10 true: `big` needs x <= 0, so it reaches 9
    at most. `big` also makes `ready` true, the precondition of `small`, so
    goal-relevance pruning keeps it."""
    ready = Fluent("ready", BoolType())
    x = Fluent("x", IntType())
    big = InstantaneousAction("big")
    big.add_precondition(LE(x(), 0))
    big.add_effect(ready(), True)
    big.add_increase_effect(x(), 9)
    small = InstantaneousAction("small")
    small.add_precondition(ready())
    small.add_increase_effect(x(), 1)
    p = Problem("residual_contributor")
    p.add_fluent(ready, default_initial_value=False)
    p.add_fluent(x, default_initial_value=0)
    p.add_action(big)
    p.add_action(small)
    p.add_goal(GE(x(), 10))
    return p


def test_landmark_residual_counts_every_contributor():
    """A numeric landmark's deficit is filled by every action that adds to
    its quantity, not only by its achievers. LandmarkAnalysisPass took the
    mass per step from the achievers of x >= 10 alone (`small`, 1 a step), so
    its depth bound was 10 and, under max_steps 5, a problem with a two-step
    plan came back UNSOLVABLE_INCOMPLETELY. Counting `big` gives 10 a step and
    a bound of 1."""
    p = _residual_contributor_problem()
    for params in ({}, {"strategy": "elsc"}):
        r = _solve_with(p, max_steps=5, **params)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{params}: {r.status}"
        assert_valid(p, r.plan)
        assert _metric(r, "landmarks.depth_bound") == 1, params


def _breaker_and_needer_problem(case):
    """One step can run `breaker` and `needer` only as needer-then-breaker:
    breaker changes something needer depends on. breaker is declared first,
    so it has the lower id, and with no interference edge between the two
    the step is extracted in id order, breaker first. Each case hides the
    dependency from InterferenceOracle in a different way:

      computed         breaker: p := q        needer requires p
      iff              breaker: p := true     needer requires p <-> q
      both_polarities  breaker: p := false    needer requires (p or r) and (not p or s)
      value_read       breaker: q := true     needer: p := q
      count            breaker: p := true     needer requires count(p, q) <= 1

    `computed` and `value_read` are a boolean write whose value is not a
    constant, which the analysis did not record as making its fluent true or
    false, nor its value's fluents as read. `iff` (UP's Iff arrives as =) and
    `count` hid the fluents under an operator the polarity collector took as
    positive, and in `both_polarities` the second occurrence of p overwrote
    the first. Each fluent breaker does not write gets a writer, so that it is
    not static and folded away."""
    p, q, r, s, g = (Fluent(name) for name in "pqrsg")
    breaker = InstantaneousAction("breaker")
    needer = InstantaneousAction("needer")
    needer.add_effect(g(), True)
    initially_true = []
    goal = [g()]
    if case == "computed":
        initially_true = [p]
        breaker.add_effect(p(), q())
        needer.add_precondition(p())
        goal.append(Not(p()))
    elif case == "iff":
        breaker.add_effect(p(), True)
        needer.add_precondition(Iff(p(), q()))
        goal.append(p())
    elif case == "both_polarities":
        initially_true = [p, s]
        breaker.add_effect(p(), False)
        needer.add_precondition(Or(p(), r()))
        needer.add_precondition(Or(Not(p()), s()))
        goal.append(Not(p()))
    elif case == "value_read":
        initially_true = [p]
        breaker.add_effect(q(), True)
        needer.add_effect(p(), q())
        goal += [Not(p()), q()]
    elif case == "count":
        initially_true = [q]
        breaker.add_effect(p(), True)
        needer.add_precondition(LE(Count(p(), q()), 1))
        goal.append(p())
    problem = Problem(f"breaker_and_needer_{case}")
    for f in (p, q, r, s, g):
        problem.add_fluent(f, default_initial_value=False)
    for f in initially_true:
        problem.set_initial_value(f(), True)
    problem.add_action(breaker)
    problem.add_action(needer)
    for f in (q, r, s):
        writer = InstantaneousAction(f"write_{f.name}")
        writer.add_effect(f(), f not in initially_true)
        problem.add_action(writer)
    problem.add_goal(And(goal))
    return problem


def test_interference_sees_every_dependency():
    """An effect that can change what another action reads makes the two
    interfere, however the read is written. Each case gave the strategies
    with a syntactic interference analysis a one-step plan, extracted as
    breaker-then-needer, which is invalid (see _breaker_and_needer_problem)."""
    from rantanplan import _rantanplan
    registered = set(_rantanplan.list_strategies())
    for strategy in ("e", "e-p", "f", "f-p", "el", "fl", "e-dt", "f-dt"):
        if strategy not in registered:
            continue
        for case in ("computed", "iff", "both_polarities", "value_read", "count"):
            problem = _breaker_and_needer_problem(case)
            r = solve(problem, strategy=strategy)
            assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
                f"{strategy}/{case}: {r.status}"
            names = [a.action.name for a in r.plan.actions]
            if case == "count":
                # UP's validator does not support count. The plan is valid if
                # and only if needer runs while count(p, q) <= 1: before
                # breaker (p still false), or after write_q (q made false).
                p_true, q_true = False, True
                for name in names:
                    if name == "breaker":
                        p_true = True
                    elif name == "write_q":
                        q_true = False
                    elif name == "needer":
                        assert p_true + q_true <= 1, f"{strategy}/{case}: {names}"
                assert "needer" in names and "breaker" in names, \
                    f"{strategy}/{case}: {names}"
            else:
                try:
                    assert_valid(problem, r.plan)
                except AssertionError as e:
                    raise AssertionError(f"{strategy}/{case}: {names}: {e}")


def test_declared_bounds_on_the_chained_encoder_are_unsupported():
    """The Chained encoder does not assert declared numeric bounds on its
    intermediate values. A strategy on it must refuse the problem as
    unsupported and say which strategies take it, not fail with an internal
    error. The Grounded and R2E encoders take it.

        x : Int[0, 10], initially 0;  inc : x += 1;  goal  x >= 1"""
    x = Fluent("x", IntType(0, 10))
    inc = InstantaneousAction("inc")
    inc.add_increase_effect(x(), 1)
    p = Problem("declared_bounds")
    p.add_fluent(x, default_initial_value=0)
    p.add_action(inc)
    p.add_goal(GE(x(), 1))

    r = solve(p, strategy="elsc")
    assert r.status == PlanGenerationResultStatus.UNSUPPORTED_PROBLEM, r.status
    assert r.plan is None
    messages = [m.message for m in r.log_messages or []]
    assert any("Declared numeric bounds" in m and "Chained" in m for m in messages), messages

    for strategy in ("e", "r2e"):
        r = solve(p, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, (strategy, r.status)
        assert_valid(p, r.plan)


def _two_actions_increase_one_fluent_problem():
    """Two actions that increase one fluent by the same amount, and a shortest
    plan that would be valid only if the two increases counted as one:

        inc1   : PRE not done1   effect done1 := true, x += 1
        inc2   : PRE not done2   effect done2 := true, x += 1
        reset  : PRE x >= 2      effect x := 0
        finish : PRE done1 AND done2 AND x <= 1   effect finished := true
        init  x = 0;  goal  finished      (solvable: inc1, inc2, reset, finish)

    inc1 and inc2 in one step leave x = 2: two increases add up, in Unified
    Planning and in PDDL 2.1 alike. An encoder that states each write as its
    own equation, x' = x + 1 twice, records x = 1 instead, and then finish
    looks applicable without reset."""
    done1, done2, finished = (Fluent(n, BoolType()) for n in ("done1", "done2", "finished"))
    x = Fluent("x", IntType())
    inc1 = InstantaneousAction("inc1")
    inc1.add_precondition(Not(done1()))
    inc1.add_effect(done1(), True)
    inc1.add_increase_effect(x(), 1)
    inc2 = InstantaneousAction("inc2")
    inc2.add_precondition(Not(done2()))
    inc2.add_effect(done2(), True)
    inc2.add_increase_effect(x(), 1)
    reset = InstantaneousAction("reset")
    reset.add_precondition(GE(x(), 2))
    reset.add_effect(x(), 0)
    finish = InstantaneousAction("finish")
    finish.add_precondition(And(done1(), done2(), LE(x(), 1)))
    finish.add_effect(finished(), True)
    p = Problem("two_actions_increase_one_fluent")
    for f in (done1, done2, finished):
        p.add_fluent(f, default_initial_value=False)
    p.add_fluent(x, default_initial_value=0)
    for a in (inc1, inc2, reset, finish):
        p.add_action(a)
    p.add_goal(finished())
    return p


# Semantic interference lets inc1 and inc2 share a step, because their
# increases commute. That is sound only with an encoder that adds them up, as
# the chained one does. With the Grounded encoder, els, fls, egs and fgs
# returned [inc1, inc2, finish], which UP rejects, so StrategyFactory::validate
# refuses that pairing. e and f keep two writers of one numeric fluent in
# separate steps.
def test_increases_of_one_fluent_in_one_step_add_up():
    """Two increases of one fluent in one step must not be counted as one: the
    plan must be valid, on every registered strategy."""
    from rantanplan import _rantanplan
    for strategy in _rantanplan.list_strategies():
        p = _two_actions_increase_one_fluent_problem()
        r = solve(p, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{strategy}: expected SOLVED, got {r.status}"
        try:
            assert_valid(p, r.plan)
        except AssertionError as e:
            raise AssertionError(f"{strategy}: {[a.action.name for a in r.plan.actions]}: {e}")


def test_suite_strategies_are_registered():
    """Every strategy the PDDL suite names (quick, optimal, skip and regression
    lists) is registered, so no list silently tests less than it says. Every
    solve runs StrategyFactory::validate, so the quick tests also check that
    no registered strategy is a pairing it refuses (such as semantic
    interference with the Grounded encoder)."""
    from rantanplan import _rantanplan
    from .. import pddl_suite
    registered = set(_rantanplan.list_strategies())
    named = set(pddl_suite.QUICK_STRATEGIES)
    named |= set(pddl_suite.OPTIMAL_MODE_STRATEGIES)
    named |= set(pddl_suite.QUICK_OPTIMAL_MODE_STRATEGIES)
    for strategies in pddl_suite.SKIP_CONFIGS.values():
        named |= set(strategies)
    for _, strategies in pddl_suite.REGRESSION_CONFIGS:
        named |= set(strategies)
    assert named <= registered, f"not registered: {sorted(named - registered)}"


# Strategies the effect-normalization tests below run on (those registered):
# every encoder and parallelism path, without the semantic-interference
# strategies on the Grounded encoder.
NORMALIZATION_STRATEGIES = ("seq", "e", "f", "r2e", "egsc", "fgsc",
                            "elsc", "flsc", "elsc-dt")


def _registered(*names):
    """Those of `names` that are registered strategies in this build."""
    from rantanplan import _rantanplan
    registered = set(_rantanplan.list_strategies())
    return [n for n in names if n in registered]


def _rolling_ball_problem():
    """Case 1. A ball that grows as it rolls on snow, as in the Snowman game
   :

        roll : WHEN small:  small := false, medium := true
               WHEN medium: medium := false, large := true
        init  small;  goal  large           (solvable: roll; roll)

    The two conditional effects on `medium` set and clear it under guards
    that cannot both hold in a reachable state (a ball has one size), but
    nothing in the action says so."""
    small, medium, large = (Fluent(n, BoolType()) for n in ("small", "medium", "large"))
    roll = InstantaneousAction("roll")
    roll.add_effect(small(), False, condition=small())
    roll.add_effect(medium(), True, condition=small())
    roll.add_effect(medium(), False, condition=medium())
    roll.add_effect(large(), True, condition=medium())
    p = Problem("rolling_ball")
    for f in (small, medium, large):
        p.add_fluent(f, default_initial_value=False)
    p.set_initial_value(small(), True)
    p.add_action(roll)
    p.add_goal(large())
    return p


def test_boolean_set_and_clear_is_add_after_delete():
    """An action that sets and clears one Boolean atom is well defined: PDDL
    and Unified Planning apply the clear before the set, so the atom ends up
    true (add after delete).

    Dropping such an action (as the planner once did whenever the set and the
    clear could both hold given the precondition alone) removes `roll`, and the
    problem is reported PROVEN unsolvable, as every Snowman level was.
    EffectNormalizationPass makes each clear apply only when no set of the
    same atom does, and keeps the action."""
    for strategy in _registered(*NORMALIZATION_STRATEGIES):
        p = _rolling_ball_problem()
        r = solve(p, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{strategy}: expected SOLVED, got {r.status} (an action that sets " \
            f"and clears one atom was dropped again)"
        assert_valid(p, r.plan)


def test_unconditional_set_and_clear_keeps_the_action():
    """Case 1, unconditional, as a reflexive grounding produces it (a move
    from a place to itself deletes and adds the same `at`): the atom stays
    true, and the action's other effects still happen.

        stay : (not (at)), (at), done := true
        init  at;  goal  done AND at        (solvable: stay)"""
    for strategy in _registered(*NORMALIZATION_STRATEGIES):
        at = Fluent("at", BoolType())
        done = Fluent("done", BoolType())
        stay = InstantaneousAction("stay")
        stay.add_precondition(at())
        stay.add_effect(at(), False)
        stay.add_effect(at(), True)
        stay.add_effect(done(), True)
        p = Problem("stay_in_place")
        p.add_fluent(at, default_initial_value=False)
        p.add_fluent(done, default_initial_value=False)
        p.set_initial_value(at(), True)
        p.add_action(stay)
        p.add_goal(done())
        p.add_goal(at())
        r = solve(p, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{strategy}: expected SOLVED, got {r.status} (the reflexive action " \
            f"was dropped again)"
        assert_valid(p, r.plan)


def _complex_set_and_clear_problem(case):
    """Case 1 with complex guards. One action that sets and clears the atom
    `f`, twice each, under conditions with disjunction, negation and numeric
    comparisons:

        bump : WHEN (p OR n > 2)          f := true
               WHEN r                     f := true
               WHEN (NOT q AND n < 4)     f := false
               WHEN (q AND NOT r)         f := false
               n += 1
        p, q, r false throughout; n an unbounded Int, kept below 5 by bump's
        precondition n < 5

    Case "both": init n = 3, f false; at n = 3 the first set and the first
    clear both hold, so add after delete makes f TRUE. Goal f AND n = 4:
    only `bump` from n = 3 reaches it.
    Case "clear": init n = 0, f true; only the first clear holds, so f
    becomes FALSE. Goal (NOT f) AND n = 1."""
    p, q, r, f = (Fluent(name, BoolType()) for name in ("p", "q", "r", "f"))
    n = Fluent("n", IntType())
    bump = InstantaneousAction("bump")
    bump.add_precondition(LT(n(), 5))
    bump.add_effect(f(), True, condition=Or(p(), GT(n(), 2)))
    bump.add_effect(f(), True, condition=r())
    bump.add_effect(f(), False, condition=And(Not(q()), LT(n(), 4)))
    bump.add_effect(f(), False, condition=And(q(), Not(r())))
    bump.add_increase_effect(n(), 1)
    prob = Problem(f"complex_set_and_clear_{case}")
    for fl in (p, q, r, f):
        prob.add_fluent(fl, default_initial_value=False)
    prob.add_fluent(n, default_initial_value=0)
    prob.add_action(bump)
    if case == "both":
        prob.set_initial_value(n(), 3)
        prob.add_goal(f())
        prob.add_goal(Equals(n(), 4))
    else:
        prob.set_initial_value(f(), True)
        prob.add_goal(Not(f()))
        prob.add_goal(Equals(n(), 1))
    return prob


def test_complex_set_and_clear_conditions():
    """Add after delete with arbitrary conditions: in the state where a set
    and a clear of one atom both apply, the atom ends up true ("both"), and a
    clear still applies where no set does ("clear"). The plan must be the one
    `bump`, and UP must accept it."""
    for strategy in _registered(*NORMALIZATION_STRATEGIES):
        for case in ("both", "clear"):
            prob = _complex_set_and_clear_problem(case)
            r = solve(prob, strategy=strategy)
            assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
                f"{strategy}/{case}: expected SOLVED, got {r.status}"
            assert [ai.action.name for ai in r.plan.actions] == ["bump"], r.plan
            assert_valid(prob, r.plan)


# ---------------------------------------------------------------------------
# Several writes to one fluent in one action (EffectNormalizationPass). Each
# problem's docstring names its case, numbered as in the pass's class comment,
# which has an example of each; case 1 is tested above, from
# _rolling_ball_problem on. Every test runs on NORMALIZATION_STRATEGIES and
# checks the plan with Unified Planning's validator.
# ---------------------------------------------------------------------------

def _computed_boolean_writes_problem():
    """Case 2. `poll` assigns `alarm` two computed values; Unified Planning
    makes it true if either is true (here `heat`).

        poll      : alarm := smoke, alarm := heat
        ventilate : smoke := false, heat := false   (keeps both dynamic)
        init  heat;  goal  alarm            (solvable: poll)

    Dropping `poll`, as the pass once did, left the goal unreachable."""
    alarm, smoke, heat = (Fluent(n, BoolType()) for n in ("alarm", "smoke", "heat"))
    poll = InstantaneousAction("poll")
    poll.add_effect(alarm(), smoke())
    poll.add_effect(alarm(), heat())
    ventilate = InstantaneousAction("ventilate")
    ventilate.add_effect(smoke(), False)
    ventilate.add_effect(heat(), False)
    p = Problem("computed_boolean_writes")
    for f in (alarm, smoke, heat):
        p.add_fluent(f, default_initial_value=False)
    p.set_initial_value(heat(), True)
    p.add_action(poll)
    p.add_action(ventilate)
    p.add_goal(alarm())
    return p


def test_computed_boolean_writes_are_merged():
    """Case 2: `poll`'s two writes of `alarm` become one."""
    for strategy in _registered(*NORMALIZATION_STRATEGIES):
        p = _computed_boolean_writes_problem()
        r = solve(p, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{strategy}: expected SOLVED, got {r.status}"
        assert_valid(p, r.plan)
        assert _metric(r, "effect_normalization.boolean_computed_merged") == 1, strategy


def _cancelling_writes_problem():
    """Case 3, as in sugar's reflexive sugar-cane-mills(r, m, m): `act`
    increases and decreases `x` by 1, which cancel, but it also sets `done`.
    `wiggle` does nothing else, so it changes nothing.

        act    : PRE not done   effect x += 1, x -= 1, done := true
        wiggle : x += 1, x -= 1
        init  x = 0;  goal  done AND x = 0      (solvable: act)

    Dropping `act`, as the pass once did, left the goal unreachable."""
    x = Fluent("x", IntType())
    done = Fluent("done", BoolType())
    act = InstantaneousAction("act")
    act.add_precondition(Not(done()))
    act.add_increase_effect(x(), 1)
    act.add_decrease_effect(x(), 1)
    act.add_effect(done(), True)
    wiggle = InstantaneousAction("wiggle")
    wiggle.add_increase_effect(x(), 1)
    wiggle.add_decrease_effect(x(), 1)
    p = Problem("cancelling_writes")
    p.add_fluent(x, default_initial_value=0)
    p.add_fluent(done, default_initial_value=False)
    p.add_action(act)
    p.add_action(wiggle)
    p.add_goal(done())
    p.add_goal(Equals(x(), 0))
    return p


def test_cancelling_writes_keep_the_action():
    """Case 3: `act` keeps its other effect; `wiggle`, left with no effect,
    is removed."""
    for strategy in _registered(*NORMALIZATION_STRATEGIES):
        p = _cancelling_writes_problem()
        r = solve(p, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{strategy}: expected SOLVED, got {r.status}"
        assert_valid(p, r.plan)
        assert _metric(r, "effect_normalization.additive_merged") == 2, strategy
        assert _metric(r, "effect_normalization.actions_removed_no_effect") == 1, strategy


def _harvest_problem():
    """Case 4 with different amounts: two guarded increases of `yield`.

        water     : PRE not watered     effect watered := true
        fertilise : PRE not fertilised  effect fertilised := true
        harvest   : PRE not harvested   effect harvested := true,
                    WHEN watered: yield += 2,  WHEN fertilised: yield += 3
        init  yield = 0;  goal  harvested AND yield = 5
        (solvable: water, fertilise, harvest)

    Both increases apply together; one equation per write made them
    contradict, so `harvest` was unusable exactly where the goal needs it."""
    watered, fertilised, harvested = (
        Fluent(n, BoolType()) for n in ("watered", "fertilised", "harvested"))
    yld = Fluent("yield", IntType())
    water = InstantaneousAction("water")
    water.add_precondition(Not(watered()))
    water.add_effect(watered(), True)
    fertilise = InstantaneousAction("fertilise")
    fertilise.add_precondition(Not(fertilised()))
    fertilise.add_effect(fertilised(), True)
    harvest = InstantaneousAction("harvest")
    harvest.add_precondition(Not(harvested()))
    harvest.add_effect(harvested(), True)
    harvest.add_increase_effect(yld(), 2, condition=watered())
    harvest.add_increase_effect(yld(), 3, condition=fertilised())
    p = Problem("harvest")
    for f in (watered, fertilised, harvested):
        p.add_fluent(f, default_initial_value=False)
    p.add_fluent(yld, default_initial_value=0)
    for a in (water, fertilise, harvest):
        p.add_action(a)
    p.add_goal(harvested())
    p.add_goal(Equals(yld(), 5))
    return p


def _equal_guarded_increases_problem():
    """Case 4 with equal amounts, and a shortest plan that is valid only if
    the two increases counted as one:

        act    : PRE not done   effect done := true,
                 WHEN p: x += 1,  WHEN q: x += 1
        calm   : p := false, q := false         (keeps p and q dynamic)
        reset  : PRE x >= 2     effect x := 0
        finish : PRE done AND x <= 1   effect finished := true
        init  p, q, x = 0;  goal  finished
        (solvable in three steps: act, reset, finish or calm, act, finish)

    One equation per write recorded x = 1 after `act`, so [act, finish]
    looked valid; Unified Planning rejects it (x = 2)."""
    p_, q_, done, finished = (Fluent(n, BoolType()) for n in ("p", "q", "done", "finished"))
    x = Fluent("x", IntType())
    act = InstantaneousAction("act")
    act.add_precondition(Not(done()))
    act.add_effect(done(), True)
    act.add_increase_effect(x(), 1, condition=p_())
    act.add_increase_effect(x(), 1, condition=q_())
    calm = InstantaneousAction("calm")
    calm.add_effect(p_(), False)
    calm.add_effect(q_(), False)
    reset = InstantaneousAction("reset")
    reset.add_precondition(GE(x(), 2))
    reset.add_effect(x(), 0)
    finish = InstantaneousAction("finish")
    finish.add_precondition(And(done(), LE(x(), 1)))
    finish.add_effect(finished(), True)
    prob = Problem("equal_guarded_increases")
    for f in (p_, q_, done, finished):
        prob.add_fluent(f, default_initial_value=False)
    prob.set_initial_value(p_(), True)
    prob.set_initial_value(q_(), True)
    prob.add_fluent(x, default_initial_value=0)
    for a in (act, calm, reset, finish):
        prob.add_action(a)
    prob.add_goal(finished())
    return prob


def test_guarded_increases_are_split():
    """Case 4: two guards that can hold together give three writes (one, the
    other, both)."""
    for strategy in _registered(*NORMALIZATION_STRATEGIES):
        for build in (_harvest_problem, _equal_guarded_increases_problem):
            p = build()
            r = solve(p, strategy=strategy)
            assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
                f"{strategy}: expected SOLVED, got {r.status}"
            assert_valid(p, r.plan)
            assert _metric(r, "effect_normalization.additive_split_max_writes") == 3, strategy


def test_increases_with_the_same_guard_are_summed():
    """Case 4, first step: _two_writes_to_one_fluent_problem guards both
    increases of `g` with `flag`, so they become one write, `g += 3`, and
    every strategy solves it."""
    for strategy in _registered(*NORMALIZATION_STRATEGIES):
        p = _two_writes_to_one_fluent_problem()
        r = solve(p, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{strategy}: expected SOLVED, got {r.status}"
        assert_valid(p, r.plan)
        assert _metric(r, "effect_normalization.additive_merged") == 1, strategy


def _five_guarded_increases_problem():
    """Case 4 above the warning: five guarded increases of `x`, whose guards
    can all hold together, give 2^5 - 1 = 31 writes.

        bump : PRE not done   effect done := true,
               WHEN c1: x += 1, ..., WHEN c5: x += 5
        drop : c1 := false, ..., c5 := false    (keeps the guards dynamic)
        init  c1 ... c5, x = 0;  goal  done AND x = 15      (solvable: bump)"""
    cs = [Fluent(f"c{i}", BoolType()) for i in range(1, 6)]
    done = Fluent("done", BoolType())
    x = Fluent("x", IntType())
    bump = InstantaneousAction("bump")
    bump.add_precondition(Not(done()))
    bump.add_effect(done(), True)
    for i, c in enumerate(cs, start=1):
        bump.add_increase_effect(x(), i, condition=c())
    drop = InstantaneousAction("drop")
    for c in cs:
        drop.add_effect(c(), False)
    p = Problem("five_guarded_increases")
    for c in cs:
        p.add_fluent(c, default_initial_value=False)
        p.set_initial_value(c(), True)
    p.add_fluent(done, default_initial_value=False)
    p.add_fluent(x, default_initial_value=0)
    p.add_action(bump)
    p.add_action(drop)
    p.add_goal(done())
    p.add_goal(Equals(x(), 15))
    return p


def test_many_guarded_increases_are_split():
    """Case 4 above the warning (more than 4 guards): all 31 writes, with no
    limit."""
    for strategy in _registered(*NORMALIZATION_STRATEGIES):
        p = _five_guarded_increases_problem()
        r = solve(p, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{strategy}: expected SOLVED, got {r.status}"
        assert_valid(p, r.plan)
        assert _metric(r, "effect_normalization.additive_split_max_writes") == 31, strategy


def _conflicting_assignments_problem():
    """Case 6. `calibrate` assigns `temp` two different values; Unified
    Planning rejects it where it is both hot and humid.

        calibrate : WHEN hot: temp := 50,  WHEN humid: temp := 40
        dry       : humid := false
        cool      : hot := false
        init  hot, humid, temp = 0;  goal  temp = 50   (solvable: dry, calibrate)

    [calibrate] alone would be invalid. Dropping `calibrate`, as the pass
    once did, left the goal unreachable."""
    hot, humid = Fluent("hot", BoolType()), Fluent("humid", BoolType())
    temp = Fluent("temp", IntType())
    calibrate = InstantaneousAction("calibrate")
    calibrate.add_effect(temp(), 50, condition=hot())
    calibrate.add_effect(temp(), 40, condition=humid())
    dry = InstantaneousAction("dry")
    dry.add_effect(humid(), False)
    cool = InstantaneousAction("cool")
    cool.add_effect(hot(), False)
    p = Problem("conflicting_assignments")
    for f in (hot, humid):
        p.add_fluent(f, default_initial_value=False)
        p.set_initial_value(f(), True)
    p.add_fluent(temp, default_initial_value=0)
    for a in (calibrate, dry, cool):
        p.add_action(a)
    p.add_goal(Equals(temp(), 50))
    return p


def test_conflicting_assignments_restrict_the_precondition():
    """Case 6: `calibrate` stays usable where only one of its guards holds."""
    for strategy in _registered(*NORMALIZATION_STRATEGIES):
        p = _conflicting_assignments_problem()
        r = solve(p, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{strategy}: expected SOLVED, got {r.status}"
        assert_valid(p, r.plan)
        assert _metric(r, "effect_normalization.assign_conflicts_restricted") == 1, strategy


def _reflexive_assignments_problem():
    """Case 6, unconditional, from a reflexive grounding: set_levels(a, b)
    assigns level(a) := 3 and level(b) := 5, so set_levels(t, t) assigns
    level(t) two values and Unified Planning can never execute it.

        init  level(t1) = level(t2) = 0;  goal  level(t1) = 3 AND level(t2) = 5
        (solvable: set_levels(t1, t2))"""
    Tank = UserType("Tank")
    t1, t2 = Object("t1", Tank), Object("t2", Tank)
    level = Fluent("level", IntType(), t=Tank)
    set_levels = InstantaneousAction("set_levels", a=Tank, b=Tank)
    set_levels.add_effect(level(set_levels.parameter("a")), 3)
    set_levels.add_effect(level(set_levels.parameter("b")), 5)
    p = Problem("reflexive_assignments")
    p.add_objects([t1, t2])
    p.add_fluent(level, default_initial_value=0)
    p.add_action(set_levels)
    p.add_goal(Equals(level(t1), 3))
    p.add_goal(Equals(level(t2), 5))
    return p


def test_reflexive_conflicting_assignments_are_removed():
    """Case 6, unconditional: set_levels(t1, t1) and set_levels(t2, t2) can
    never apply, and are removed."""
    for strategy in _registered(*NORMALIZATION_STRATEGIES):
        p = _reflexive_assignments_problem()
        r = solve(p, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{strategy}: expected SOLVED, got {r.status}"
        assert_valid(p, r.plan)
        assert _metric(r, "effect_normalization.actions_removed_never_applicable") == 2, strategy


def _assignment_and_increase_problem():
    """Case 7. A refuel idiom with >= where > was meant: at fuel = 0 both
    writes apply, and Unified Planning rejects `refuel` there.

        refuel : WHEN fuel + 200 <= 200: fuel += 200,
                 WHEN fuel + 200 >= 200: fuel := 200
        add10  : PRE fuel <= 0   effect fuel += 10
        init  fuel = 0;  goal  fuel = 200      (solvable: add10, refuel)

    One equation per write accepted [refuel] (both give 200), which Unified
    Planning rejects."""
    fuel = Fluent("fuel", IntType())
    refuel = InstantaneousAction("refuel")
    refuel.add_increase_effect(fuel(), 200, condition=LE(Plus(fuel(), 200), 200))
    refuel.add_effect(fuel(), 200, condition=GE(Plus(fuel(), 200), 200))
    add10 = InstantaneousAction("add10")
    add10.add_precondition(LE(fuel(), 0))
    add10.add_increase_effect(fuel(), 10)
    p = Problem("assignment_and_increase")
    p.add_fluent(fuel, default_initial_value=0)
    p.add_action(refuel)
    p.add_action(add10)
    p.add_goal(Equals(fuel(), 200))
    return p


def test_assignment_and_increase_restrict_the_precondition():
    """Case 7: `refuel` is inapplicable at fuel = 0, where both writes
    apply."""
    for strategy in _registered(*NORMALIZATION_STRATEGIES):
        p = _assignment_and_increase_problem()
        r = solve(p, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{strategy}: expected SOLVED, got {r.status}"
        assert_valid(p, r.plan)
        assert _metric(r, "effect_normalization.assign_conflicts_restricted") == 1, strategy


def _same_assignment_twice_problem():
    """Case 5. `reset` assigns `x` the same value under two guards that hold
    together; Unified Planning accepts that, and the pass leaves it alone.

        reset : PRE not done   effect done := true,
                WHEN a: x := 5,  WHEN b: x := 5
        calm  : a := false, b := false          (keeps a and b dynamic)
        init  a, b, x = 0;  goal  done AND x = 5      (solvable: reset)

    Treating the two writes as a conflict (case 6) would make `reset`
    inapplicable where a and b both hold, and after `calm` it would assign
    nothing, so the goal would be unreachable."""
    a, b, done = (Fluent(n, BoolType()) for n in ("a", "b", "done"))
    x = Fluent("x", IntType())
    reset = InstantaneousAction("reset")
    reset.add_precondition(Not(done()))
    reset.add_effect(done(), True)
    reset.add_effect(x(), 5, condition=a())
    reset.add_effect(x(), 5, condition=b())
    calm = InstantaneousAction("calm")
    calm.add_effect(a(), False)
    calm.add_effect(b(), False)
    p = Problem("same_assignment_twice")
    for f in (a, b, done):
        p.add_fluent(f, default_initial_value=False)
    p.set_initial_value(a(), True)
    p.set_initial_value(b(), True)
    p.add_fluent(x, default_initial_value=0)
    p.add_action(reset)
    p.add_action(calm)
    p.add_goal(done())
    p.add_goal(Equals(x(), 5))
    return p


def test_same_assignment_twice_is_left_alone():
    """Case 5: the precondition of `reset` is not restricted."""
    for strategy in _registered(*NORMALIZATION_STRATEGIES):
        p = _same_assignment_twice_problem()
        r = solve(p, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{strategy}: expected SOLVED, got {r.status}"
        assert_valid(p, r.plan)
        assert _metric(r, "effect_normalization.assign_conflicts_restricted") == 0, strategy


def _exclusive_guards_problem():
    """Cases 4 and 7 with guards that exclude each other, which the pass must
    leave alone: no state satisfies both guards, so the writes never apply
    together.

        sail   : PRE not sailed   effect sailed := true,
                 WHEN load = 0: fuel -= 1,  WHEN not (load = 0): fuel -= 2
        refuel : WHEN fuel + 5 <= 10: fuel += 5,
                 WHEN fuel + 5 > 10:  fuel := 10
        stow   : load := 1                      (keeps load dynamic)
        init  fuel = 7, load = 0;  goal  sailed AND fuel = 9
        (solvable: refuel, sail)

    `sail` has petrobras's shape, a condition and its negation, which the
    overlap check answers without the solver. `refuel` is the clip-to-capacity
    idiom, whose guards the solver proves exclusive."""
    sailed = Fluent("sailed", BoolType())
    fuel, load = Fluent("fuel", IntType()), Fluent("load", IntType())
    sail = InstantaneousAction("sail")
    sail.add_precondition(Not(sailed()))
    sail.add_effect(sailed(), True)
    sail.add_decrease_effect(fuel(), 1, condition=Equals(load(), 0))
    sail.add_decrease_effect(fuel(), 2, condition=Not(Equals(load(), 0)))
    refuel = InstantaneousAction("refuel")
    refuel.add_increase_effect(fuel(), 5, condition=LE(Plus(fuel(), 5), 10))
    refuel.add_effect(fuel(), 10, condition=GT(Plus(fuel(), 5), 10))
    stow = InstantaneousAction("stow")
    stow.add_effect(load(), 1)
    p = Problem("exclusive_guards")
    p.add_fluent(sailed, default_initial_value=False)
    p.add_fluent(fuel, default_initial_value=7)
    p.add_fluent(load, default_initial_value=0)
    for a in (sail, refuel, stow):
        p.add_action(a)
    p.add_goal(sailed())
    p.add_goal(Equals(fuel(), 9))
    return p


def test_exclusive_guards_are_left_alone():
    """Nothing is split, merged or restricted, and only `refuel`'s guards
    take a solver query."""
    for strategy in _registered(*NORMALIZATION_STRATEGIES):
        p = _exclusive_guards_problem()
        r = solve(p, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
            f"{strategy}: expected SOLVED, got {r.status}"
        assert_valid(p, r.plan)
        assert _metric(r, "effect_normalization.additive_split") == 0, strategy
        assert _metric(r, "effect_normalization.additive_merged") == 0, strategy
        assert _metric(r, "effect_normalization.assign_conflicts_restricted") == 0, strategy
        assert _metric(r, "effect_normalization.overlap_queries") == 1, strategy


_KIND = _rantanplan.ExprKind


def _pool_nodes(problem):
    """Every node of the preprocessed problem's expression pool, as
    (ExprKind, type name, text). The pool keeps every node ever interned, so
    this shows the converter's nodes beside those the passes made."""
    pre = _rantanplan.preprocess(convert_problem(problem),
                                 ["--strategy", "seq", "--verbosity", "silent"])
    return pre.pool_nodes()


def test_integer_declarations_and_nodes_have_one_type():
    """The type table has one int type, so a type taken from a declaration is
    the type of the converter's nodes.

    It had two, the planner's own up:int beside UP's up:integer: the
    converter's nodes resolved "up:integer" to up:int, but declarations were
    looked up by name, so an int fluent's Fluent::type() was up:integer. Array
    lowering mints each cell from the declaration, so the cells of `a` were
    typed up:integer while `x` and the literals were up:int. Now the one entry
    is UP's up:integer. A pass that made an atom the converter also
    makes, with its declaration's type, would have made an alias."""
    x = Fluent("x", IntType(0, 9))
    a = Fluent("a", ArrayType(2, IntType(0, 9)))
    p = Problem("int_types")
    p.add_fluent(x, default_initial_value=0)
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0, 0])
    em = p.environment.expression_manager
    w = InstantaneousAction("w")
    w.add_effect(em.ArrayIndex(a(), Int(0)), 1)
    w.add_effect(x(), 2)
    p.add_action(w)
    p.add_goal(And(Equals(em.ArrayIndex(a(), Int(0)), 1), Equals(x(), 2)))

    nodes = _pool_nodes(p)
    atoms = {text: type_name for kind, type_name, text in nodes
             if kind == _KIND.STATE_VARIABLE and text in ("(x)", "(a[0])", "(a[1])")}
    literals = {type_name for kind, type_name, text in nodes
                if kind == _KIND.CONSTANT and text.isdigit()}
    assert len(atoms) == 3, f"expected atoms x, a[0], a[1]; got {atoms}"
    assert set(atoms.values()) | literals == {"up:integer"}, \
        f"int atoms {atoms} and int literals {literals} should share one type"


def test_boolean_constants_have_one_node_each():
    """`true` is one node, typed bool, whoever interns it.

    The converter types its `true` (the precondition of an action without
    one) as up:bool, while intern_bool_constant(), behind every pass's
    ExprBuilder::true_id(), gave no type. So most problems had two `true`
    nodes, and Problem::is_bool_type() said no to the passes' one."""
    b, c = Fluent("b"), Fluent("c")
    p = Problem("bool_constants")
    p.add_fluent(b, default_initial_value=False)
    p.add_fluent(c, default_initial_value=False)
    s = InstantaneousAction("s")
    s.add_effect(b(), True)
    t = InstantaneousAction("t")
    t.add_precondition(b())
    t.add_effect(c(), True)
    p.add_action(s)
    p.add_action(t)
    p.add_goal(c())

    bools = [(type_name, text) for kind, type_name, text in _pool_nodes(p)
             if kind == _KIND.CONSTANT and text in ("true", "false")]
    assert ("up:bool", "true") in bools, f"no typed true among {bools}"
    assert len(bools) == len(set(text for _, text in bools)), \
        f"a boolean value has more than one node: {bools}"
    assert all(type_name == "up:bool" for type_name, _ in bools), bools


def test_folded_integers_are_integer_constants():
    """Folding integer arithmetic gives an integer constant: the node of the
    literal with that value.

    Folding computes in double and stored the double: static-fluent folding
    turned `k + 2` (k static, 3) into an untyped 5.000000, and effect
    normalisation summed `y += 3` and `y += 4` into a 7.000000 typed int. Each
    sat beside the goal's literal 5 or 7 as a second node for one value."""
    x = Fluent("x", IntType(0, 20))
    y = Fluent("y", IntType(0, 20))
    k = Fluent("k", IntType(0, 9))
    p = Problem("folded_integers")
    p.add_fluent(x, default_initial_value=0)
    p.add_fluent(y, default_initial_value=0)
    p.add_fluent(k, default_initial_value=3)
    set_x = InstantaneousAction("set_x")
    set_x.add_effect(x(), Plus(k(), 2))
    inc_y = InstantaneousAction("inc_y")
    inc_y.add_increase_effect(y(), 3)
    inc_y.add_increase_effect(y(), 4)
    p.add_action(set_x)
    p.add_action(inc_y)
    p.add_goal(And(Equals(x(), 5), Equals(y(), 7)))

    constants = [(type_name, text) for kind, type_name, text in _pool_nodes(p)
                 if kind == _KIND.CONSTANT]
    for value in (5, 7):
        nodes = [(t, s) for t, s in constants if s.startswith(str(value))]
        assert nodes == [("up:integer", str(value))], \
            f"{value} should be one integer constant, got {nodes}"

    r = solve(p, strategy=GROUNDED)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    assert_valid(p, r.plan)


def test_unsupported_expression_kinds_are_rejected():
    """A node's kind in the IR is an ExprKind, so a number with no kind, such
    as 8 (UP's CONTAINER_ID) or 9, cannot be set, and the loader refuses a node
    left UNKNOWN. Before, the kind was a bare int and only 8 was refused: any
    other number became a node of no kind."""
    for number in (8, 9):
        try:
            _rantanplan.ExprNodeIR().kind = number
        except TypeError:
            pass
        else:
            raise AssertionError(f"kind {number} was accepted")
    ir = convert_problem(_two_writes_to_one_fluent_problem())
    nodes = ir.nodes
    nodes[-1].kind = _KIND.UNKNOWN
    ir.nodes = nodes
    try:
        _rantanplan.preprocess(ir, ["--strategy", "seq", "--verbosity", "silent"])
    except RuntimeError as e:
        assert "Unsupported expression kind 0 " in str(e), str(e)
    else:
        raise AssertionError("a node of UNKNOWN kind was accepted")


def test_a_conjunction_is_one_node_whoever_builds_it():
    """An action's list of preconditions and UP's And of the same conditions
    are one node, with one `and` symbol.

    The converter built the conjunction of a list itself, from an `and` symbol
    and an application both typed up:bool, while UP's And is the untyped
    `up:and`. So the pool held two `and` symbols, and the action `two` and the
    action `one` below had two nodes for one formula. The converter now walks
    UP's own And of the list."""
    a, b, g, h = Fluent("a"), Fluent("b"), Fluent("g"), Fluent("h")
    two = InstantaneousAction("two")
    two.add_precondition(a())
    two.add_precondition(b())
    two.add_effect(g(), True)
    one = InstantaneousAction("one")
    one.add_precondition(And(a(), b()))
    one.add_effect(h(), True)
    p = Problem("conjunctions")
    for f in (a, b, g, h):
        p.add_fluent(f, default_initial_value=False)
    p.add_action(two)
    p.add_action(one)
    for f in (a, b):  # not static, so the preconditions are not folded away
        set_f = InstantaneousAction(f"set_{f.name}")
        set_f.add_effect(f(), True)
        p.add_action(set_f)
    p.add_goal(And(g(), h()))

    ands = [(kind, type_name) for kind, type_name, text in _pool_nodes(p)
            if text in ("and", "(and (a) (b))")]
    assert ands == [(_KIND.FUNCTION_SYMBOL, ""), (_KIND.FUNCTION_APPLICATION, "")], ands


def test_fluents_named_like_operators_keep_their_names():
    """Only an operator symbol takes the standard operator spelling. The loader
    gave it to fluent names too, so a fluent called `==` became `=` in the
    expressions while its declaration kept `==`, and preprocessing stopped
    with "Fluent definition not found for: =". With the name kept, the
    declared default 2 holds and one increment reaches the goal 3."""
    for name in ("==", "up:plus"):
        f = Fluent(name, IntType(0, 5))
        inc = InstantaneousAction("inc")
        inc.add_increase_effect(f(), 1)
        p = Problem("operator_names")
        p.add_fluent(f, default_initial_value=2)
        p.add_action(inc)
        p.add_goal(Equals(f(), 3))

        heads = [text for kind, _, text in _pool_nodes(p) if kind == _KIND.FLUENT_SYMBOL]
        assert heads == [name], heads
        r = solve(p, strategy=GROUNDED)
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
        assert len(r.plan.actions) == 1, r.plan
        assert_valid(p, r.plan)


def test_function_symbols_carry_an_operator():
    """A FUNCTION_SYMBOL names its operator with an Operator, not a string,
    and the loader refuses a FUNCTION_SYMBOL without one."""
    try:
        _rantanplan.ExprNodeIR().op = "up:plus"
    except TypeError:
        pass
    else:
        raise AssertionError("a string was accepted as an operator")
    ir = convert_problem(_breaker_and_needer_problem("iff"))
    nodes = ir.nodes
    head = next(n for n in nodes if n.kind == _KIND.FUNCTION_SYMBOL)
    assert isinstance(head.op, _rantanplan.Operator), head.op
    head.op = None
    ir.nodes = nodes
    try:
        _rantanplan.preprocess(ir, ["--strategy", "seq", "--verbosity", "silent"])
    except RuntimeError as e:
        assert "Function symbol without an operator" in str(e), str(e)
    else:
        raise AssertionError("a function symbol without an operator was accepted")


def _real_problem(name, fluents, actions, goal):
    p = Problem(name)
    for f, initial in fluents:
        p.add_fluent(f, default_initial_value=initial)
    for a in actions:
        p.add_action(a)
    p.add_goal(goal)
    return p


def test_reals_are_exact():
    """A real constant reaches the solver as the exact fraction UP gives. A
    third added three times is 1, and 10^-7 is not 0. Through a double and a
    6-decimal string both would be unsolvable: 0.333333 * 3 < 1, and
    y = 0.000000 is not above 0."""
    x = Fluent("x", RealType())
    inc = InstantaneousAction("inc")
    inc.add_increase_effect(x(), Fraction(1, 3))
    thirds = _real_problem("thirds", [(x, 0)], [inc], Equals(x(), 1))
    r = solve(thirds)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    assert len(r.plan.actions) == 3, r.plan
    assert_valid(thirds, r.plan)

    y = Fluent("y", RealType())
    set_y = InstantaneousAction("set_y")
    set_y.add_effect(y(), Fraction(1, 10**7))
    small = _real_problem("small", [(y, 0)], [set_y], GT(y(), 0))
    r = solve(small)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    assert_valid(small, r.plan)


def test_reals_equal_to_six_decimals_are_two_nodes():
    """The interning key holds a real exactly, so two reals that agree to 6
    decimals are two nodes, and z := 2 * 10^-7 is above y := 10^-7."""
    y, z = Fluent("y", RealType()), Fluent("z", RealType())
    a = InstantaneousAction("a")
    a.add_effect(y(), Fraction(1, 10**7))
    a.add_effect(z(), Fraction(2, 10**7))
    p = _real_problem("close_reals", [(y, 0), (z, 0)], [a], GT(z(), y()))
    reals = sorted(text for kind, type_name, text in _pool_nodes(p)
                   if kind == _KIND.CONSTANT and type_name == "up:real")
    assert reals == ["1/10000000", "1/5000000"], reals
    r = solve(p)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    assert_valid(p, r.plan)


def test_declared_real_bounds_keep_their_decimals():
    """A declared bound reaches the solver as its shortest decimal: the upper
    bound 10^-7 of x is 0.0000001, not 0.000000, so x can take it."""
    x = Fluent("x", RealType(0, Fraction(1, 10**7)))
    set_x = InstantaneousAction("set_x")
    set_x.add_effect(x(), Fraction(1, 10**7))
    p = _real_problem("small_bound", [(x, 0)], [set_x], GT(x(), 0))
    r = solve(p, strategy=GROUNDED)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    assert_valid(p, r.plan)


def test_real_folding_is_exact():
    """Folding computes with fractions: static a + b with a = 1/10 and
    b = 2/10 is 3/10, and so are y += 1/10 and y += 2/10 merged into one
    write. In double, 0.1 + 0.2 is 0.30000000000000004."""
    a, b = Fluent("a", RealType()), Fluent("b", RealType())
    x, y = Fluent("x", RealType()), Fluent("y", RealType())
    set_x = InstantaneousAction("set_x")
    set_x.add_effect(x(), Plus(a(), b()))
    inc_y = InstantaneousAction("inc_y")
    inc_y.add_increase_effect(y(), Fraction(1, 10))
    inc_y.add_increase_effect(y(), Fraction(2, 10))
    p = _real_problem("real_folding",
                      [(a, Fraction(1, 10)), (b, Fraction(2, 10)), (x, 0), (y, 0)],
                      [set_x, inc_y],
                      And(Equals(x(), Fraction(3, 10)), Equals(y(), Fraction(3, 10))))
    reals = {text for kind, type_name, text in _pool_nodes(p)
             if kind == _KIND.CONSTANT and type_name == "up:real"}
    assert "3/10" in reals, reals
    r = solve(p)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    assert_valid(p, r.plan)


def test_declared_bounds_hold_inside_a_step():
    """x is declared in [0, 10] and starts at 5; up adds 8, down takes 5, and
    the goal is x = 8. Down then up stays inside the bounds (5, 0, 8); up then
    down passes through 13, which UP's validator rejects. R2E chains the writers
    of one step through intermediate values, so the declared bounds must hold
    on those too, whichever action its fixed ordering puts first (declaring the
    actions in both orders covers both). R2E used to refuse declared bounds."""
    for strategy in ("e", "r2e"):
        for up_first in (True, False):
            x = Fluent("x", IntType(0, 10))
            up = InstantaneousAction("up")
            up.add_increase_effect(x(), 8)
            down = InstantaneousAction("down")
            down.add_decrease_effect(x(), 5)
            p = _real_problem("trap", [(x, 5)], [up, down] if up_first else [down, up],
                              Equals(x(), 8))
            r = solve(p, strategy=strategy)
            assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
                f"{strategy}, up_first={up_first}: {r.status}"
            try:
                assert_valid(p, r.plan)
            except AssertionError as e:
                raise AssertionError(f"{strategy}, up_first={up_first}: "
                                     f"{[a.action.name for a in r.plan.actions]}: {e}")


def test_bounds_admit_values_no_double_holds():
    """The only plan moves x from 0 to exactly q (or -q), and the nearest
    double to 1/3, 2/3 or 1/7 lies inside it. A bound the preprocessor derives
    (x <= 0 + q under the guard x <= 0) or the type declares (x in [-q, q])
    must still admit q. Derived bounds were computed in doubles rounded to
    nearest, and declared ones read as doubles, so both cut q off."""
    for strategy in ("e", "r2e"):
        for declared in (False, True):
            for up in (True, False):
                for q in (Fraction(1, 3), Fraction(2, 3), Fraction(1, 7)):
                    x = Fluent("x", RealType(-q, q) if declared else RealType())
                    move = InstantaneousAction("move")
                    if not declared:
                        move.add_precondition(LE(x(), 0) if up else GE(x(), 0))
                    if up:
                        move.add_increase_effect(x(), q)
                    else:
                        move.add_decrease_effect(x(), q)
                    p = _real_problem("no_double", [(x, 0)], [move],
                                      Equals(x(), q if up else -q))
                    r = solve(p, strategy=strategy)
                    case = f"{strategy}, declared={declared}, up={up}, q={q}"
                    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
                        f"{case}: {r.status}"
                    assert_valid(p, r.plan)


def test_derived_bound_reaches_the_solver_as_its_exact_double():
    """x <= 0 before inc adds 14/15, so x stays at most 14/15: rounded up, the
    double nearest 14/15. Its shortest decimal, 0.9333333333333333, is below
    14/15, and asserting that cut off the only plan; the double's exact value
    does not."""
    q = Fraction(14, 15)
    x = Fluent("x", RealType())
    inc = InstantaneousAction("inc")
    inc.add_precondition(LE(x(), 0))
    inc.add_increase_effect(x(), q)
    p = _real_problem("fourteen_fifteenths", [(x, 0)], [inc], Equals(x(), q))
    r = solve(p, strategy="e")
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    assert_valid(p, r.plan)


def test_conservation_constant_is_exact():
    """move takes a third from f and gives it to g, both starting at 1/3, so
    f + g = 2/3 in every state. The law's constant was a double, asserted as
    0.6666666666666666, which the initial state already broke."""
    third = Fraction(1, 3)
    f, g = Fluent("f", RealType()), Fluent("g", RealType())
    move = InstantaneousAction("move")
    move.add_precondition(GE(f(), third))
    move.add_decrease_effect(f(), third)
    move.add_increase_effect(g(), third)
    p = _real_problem("thirds", [(f, third), (g, third)], [move], Equals(g(), 2 * third))
    r = solve(p, strategy="e")
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    assert_valid(p, r.plan)


def _one_way_pair_problem(declared_first):
    """`spoil` affects `use` (it makes p false, which use needs) and `use`
    does not affect `spoil`, so one step can hold both only as use-then-spoil:

        use    : PRE p   effect q := true
        spoil  :         effect p := false, x += 1
        link   :         effect q := false, x += 1
        init  p, not q, x = 0;  goal  q and not p and x >= 1

    `link` writes q and x as well, so it conflicts with both and puts the
    three in one strongly connected component of the interference graph,
    where the step order decides which pairs are forbidden. The actions are
    declared with `declared_first` first, so the two cases give `use` the
    lower id in one and the higher id in the other."""
    p, q = Fluent("p"), Fluent("q")
    x = Fluent("x", IntType())
    use = InstantaneousAction("use")
    use.add_precondition(p())
    use.add_effect(q(), True)
    spoil = InstantaneousAction("spoil")
    spoil.add_effect(p(), False)
    spoil.add_increase_effect(x(), 1)
    link = InstantaneousAction("link")
    link.add_effect(q(), False)
    link.add_increase_effect(x(), 1)
    problem = Problem(f"one_way_pair_{declared_first}_first")
    problem.add_fluent(p, default_initial_value=False)
    problem.add_fluent(q, default_initial_value=False)
    problem.add_fluent(x, default_initial_value=0)
    problem.set_initial_value(p(), True)
    pair = [use, spoil] if declared_first == "use" else [spoil, use]
    for a in pair + [link]:
        problem.add_action(a)
    problem.add_goal(And(q(), Not(p()), GE(x(), 1)))
    return problem


def test_exists_step_runs_the_affected_action_first():
    """Exists-step semantics orders each step so that an action runs before
    the actions that affect it, whatever their ids: use and spoil share the
    one step of the plan."""
    for strategy in ("e-p", "elsc"):
        for declared_first in ("use", "spoil"):
            problem = _one_way_pair_problem(declared_first)
            r = solve(problem, strategy=strategy)
            case = f"{strategy}, {declared_first} first"
            assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, \
                f"{case}: {r.status}"
            assert_valid(problem, r.plan)
            assert _metric(r, "planner.solution_horizon") == 1, \
                f"{case}: {[a.action.name for a in r.plan.actions]}"


def _plan_cost(problem, plan):
    """Total cost of a plan under the problem's MinimizeActionCosts metric
    (costs may read static fluents, which fold through the initial state)."""
    metric = problem.quality_metrics[0]
    total = 0
    for ai in plan.actions:
        e = metric.get_action_cost(ai.action)
        e = e.substitute(dict(zip(ai.action.parameters, ai.actual_parameters)))
        total += e.substitute(problem.initial_values).simplify().constant_value()
    return total


def _assert_optimal(problem, strategies, expected_cost, why):
    for strategy in strategies:
        r = solve(problem, optimal=True, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_OPTIMALLY, \
            f"{strategy}: expected SOLVED_OPTIMALLY, got {r.status}"
        assert_valid(problem, r.plan)
        cost = _plan_cost(problem, r.plan)
        assert cost == expected_cost, \
            f"{strategy}: claimed optimal at cost {cost}, but a plan of cost " \
            f"{expected_cost} exists ({why}); plan: {[str(a) for a in r.plan.actions]}"


def _longer_cheaper_plan_problem():
    """`direct` reaches g in one step at cost 10; step(s1,s2), finish reaches
    it in two steps at cost 2. B&B finds `direct` at horizon 1 and must not
    stop there."""
    Spot = UserType("Spot")
    s1, s2 = Object("s1", Spot), Object("s2", Spot)
    done = Fluent("done", BoolType(), x=Spot)
    g = Fluent("g", BoolType())
    step = InstantaneousAction("step", x=Spot, y=Spot)
    x, y = step.parameters
    step.add_precondition(done(x))
    step.add_effect(done(y), True)
    finish = InstantaneousAction("finish")
    finish.add_precondition(done(s2))
    finish.add_effect(g, True)
    direct = InstantaneousAction("direct")
    direct.add_effect(g, True)
    p = Problem("bb_longer_cheaper_plan")
    p.add_fluent(done, default_initial_value=False)
    p.add_fluent(g, default_initial_value=False)
    p.add_objects([s1, s2])
    p.set_initial_value(done(s1), True)
    for a in (step, finish, direct):
        p.add_action(a)
    p.add_goal(g)
    p.add_quality_metric(MinimizeActionCosts({direct: Int(10), finish: Int(1), step: Int(1)}))
    return p


def test_bb_optimality_proof_covers_longer_plans():
    """The bounded query that proves optimality assumed goal_at_h, i.e. it only
    asked for plans that END at horizon h; the abstract suffix could then never
    stand in for a longer, cheaper continuation, and the first UNSAT was taken
    as a global optimality proof."""
    _assert_optimal(_longer_cheaper_plan_problem(), ("seq", "f", "e", "elsc", "r2e"), 2,
                    "step(s1,s2), finish")


def _shared_schema_problem(variant):
    """prep, a(p), finish (cost 3, three steps) vs direct (cost 10, one step).
    'pre':  a(q) needs ok(q), which only `restore` (cost 100) provides.
    'cost': twelve usable groundings of a, cost 1 each.
    With one abstract variable per SCHEMA, using a(p) in the suffix also
    imposes a(q)'s relaxed precondition ('pre') or charges every grounding's
    cost ('cost'), so the suffix overstates the cheapest continuation."""
    Obj = UserType("Obj")
    names = ["p", "q"] if variant == "pre" else [f"o{i}" for i in range(12)]
    objs = [Object(n, Obj) for n in names]
    prepped, mid, g = (Fluent(n, BoolType()) for n in ("prepped", "mid", "g"))
    flag = Fluent("flag", BoolType(), x=Obj)  # 'pre': ok(x); 'cost': used(x)
    prep = InstantaneousAction("prep")
    prep.add_effect(prepped, True)
    a = InstantaneousAction("a", x=Obj)
    x = a.parameter("x")
    a.add_precondition(prepped)
    a.add_effect(mid, True)
    if variant == "pre":
        a.add_precondition(flag(x))
    else:
        a.add_precondition(Not(flag(x)))
        a.add_effect(flag(x), True)
    finish = InstantaneousAction("finish")
    finish.add_precondition(mid)
    finish.add_effect(g, True)
    direct = InstantaneousAction("direct")
    direct.add_effect(g, True)
    actions = [prep, a, finish, direct]
    costs = {prep: Int(1), a: Int(1), finish: Int(1), direct: Int(10)}
    if variant == "pre":
        restore = InstantaneousAction("restore")
        restore.add_effect(flag(objs[1]), True)
        actions.append(restore)
        costs[restore] = Int(100)
    p = Problem(f"bb_shared_schema_{variant}")
    for f in (prepped, mid, g, flag):
        p.add_fluent(f, default_initial_value=False)
    p.add_objects(objs)
    if variant == "pre":
        p.set_initial_value(flag(objs[0]), True)
    for act in actions:
        p.add_action(act)
    p.add_goal(g)
    p.add_quality_metric(MinimizeActionCosts(costs))
    return p


def test_bb_abstract_actions_are_per_grounding():
    """Every grounding of a schema got one abstract Bool `abs_<schema>` (Z3
    interns constants by name), so the suffix could not use one grounding
    without the relaxed preconditions and the costs of all of them."""
    for variant in ("pre", "cost"):
        _assert_optimal(_shared_schema_problem(variant), ("seq", "f", "e", "elsc"), 3,
                        "prep, a(...), finish")


def _guarded_setter_problem():
    """set1 needs (not b) and sets b: {mod_b, [not b]} is a strongly connected
    component that nothing outside points into. Cheap: prep, set1 (cost 2,
    two steps); direct costs 10 in one step."""
    prepped, b = Fluent("prepped", BoolType()), Fluent("b", BoolType())
    prep = InstantaneousAction("prep")
    prep.add_effect(prepped, True)
    set1 = InstantaneousAction("set1")
    set1.add_precondition(prepped)
    set1.add_precondition(Not(b))
    set1.add_effect(b, True)
    direct = InstantaneousAction("direct")
    direct.add_effect(b, True)
    p = Problem("bb_guarded_setter")
    p.add_fluent(prepped, default_initial_value=False)
    p.add_fluent(b, default_initial_value=False)
    for a in (prep, set1, direct):
        p.add_action(a)
    p.add_goal(b)
    p.add_quality_metric(MinimizeActionCosts({prep: Int(1), set1: Int(1), direct: Int(10)}))
    return p


def _counter_problem():
    """inc (needs c < 5) twice costs 2; direct costs 10. Needs the SAME action
    twice, so not even r2e can fit the cheap plan into horizon 1."""
    c = Fluent("c", IntType(0, 5))
    inc = InstantaneousAction("inc")
    inc.add_precondition(LT(c, 5))
    inc.add_increase_effect(c, 1)
    direct = InstantaneousAction("direct")
    direct.add_effect(c, 2)
    p = Problem("bb_counter")
    p.add_fluent(c, default_initial_value=0)
    p.add_action(inc)
    p.add_action(direct)
    p.add_goal(GE(c, 2))
    p.add_quality_metric(MinimizeActionCosts({direct: Int(10), inc: Int(1)}))
    return p


def test_bb_loop_formulas_keep_external_support():
    """encode_loop_formulas() took the SCC's PREDECESSORS as its external
    support; an SCC with none had all its mod_v forced false, so the suffix
    could not touch those fluents at all, even through an action whose
    precondition holds at state h."""
    _assert_optimal(_guarded_setter_problem(), ("seq", "f", "e", "elsc"), 2, "prep, set1")
    _assert_optimal(_counter_problem(), ("seq", "f", "e", "r2e"), 2, "inc, inc")


def _sdac_wait_problem():
    """cost(a) = price; b (cost 1) drops price from 10 to 1. Cheapest: b ; a
    (cost 2). Under forall-step, axiom 6 forbids a at step 1 unless an action
    at step 0 interferes with it, and interference ignores cost fluents."""
    price = Fluent("price", IntType(1, 10))
    g = Fluent("g", BoolType())
    b = InstantaneousAction("b")
    b.add_effect(price, 1)
    a = InstantaneousAction("a")
    a.add_effect(g, True)
    p = Problem("bb_sdac_wait")
    p.add_fluent(price, default_initial_value=10)
    p.add_fluent(g, default_initial_value=False)
    p.add_action(b)
    p.add_action(a)
    p.add_goal(g)
    p.add_quality_metric(MinimizeActionCosts({b: Int(1), a: price}))
    return p


def test_bb_sdac_actions_may_wait_for_a_cheaper_state():
    """State-dependent costs: moving an action earlier changes the state its
    cost is read in, so axiom 6's front-loading normal form is not
    cost-preserving under forall-step semantics."""
    p = _sdac_wait_problem()
    for strategy in ("seq", "f", "r2e"):
        r = solve(p, optimal=True, strategy=strategy)
        assert r.status == PlanGenerationResultStatus.SOLVED_OPTIMALLY, \
            f"{strategy}: expected SOLVED_OPTIMALLY, got {r.status}"
        assert_valid(p, r.plan)
        assert [str(a) for a in r.plan.actions] == ["b", "a"], \
            f"{strategy}: claimed optimal {[str(a) for a in r.plan.actions]}, but b ; a costs 2"


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
    test_a_non_constant_amount_does_not_bound_its_fluent,
    test_step_limit_is_not_a_proof_of_unsolvability,
    test_interference_sees_every_dependency,
    test_landmark_residual_counts_every_contributor,
    test_declared_bounds_on_the_chained_encoder_are_unsupported,
    test_increases_of_one_fluent_in_one_step_add_up,
    test_suite_strategies_are_registered,
    test_boolean_set_and_clear_is_add_after_delete,
    test_unconditional_set_and_clear_keeps_the_action,
    test_complex_set_and_clear_conditions,
    test_computed_boolean_writes_are_merged,
    test_cancelling_writes_keep_the_action,
    test_guarded_increases_are_split,
    test_increases_with_the_same_guard_are_summed,
    test_many_guarded_increases_are_split,
    test_conflicting_assignments_restrict_the_precondition,
    test_reflexive_conflicting_assignments_are_removed,
    test_assignment_and_increase_restrict_the_precondition,
    test_same_assignment_twice_is_left_alone,
    test_exclusive_guards_are_left_alone,
    test_integer_declarations_and_nodes_have_one_type,
    test_boolean_constants_have_one_node_each,
    test_folded_integers_are_integer_constants,
    test_unsupported_expression_kinds_are_rejected,
    test_a_conjunction_is_one_node_whoever_builds_it,
    test_fluents_named_like_operators_keep_their_names,
    test_function_symbols_carry_an_operator,
    test_reals_are_exact,
    test_reals_equal_to_six_decimals_are_two_nodes,
    test_declared_real_bounds_keep_their_decimals,
    test_real_folding_is_exact,
    test_declared_bounds_hold_inside_a_step,
    test_bounds_admit_values_no_double_holds,
    test_derived_bound_reaches_the_solver_as_its_exact_double,
    test_conservation_constant_is_exact,
    test_exists_step_runs_the_affected_action_first,
    test_bb_optimality_proof_covers_longer_plans,
    test_bb_abstract_actions_are_per_grounding,
    test_bb_loop_formulas_keep_external_support,
    test_bb_sdac_actions_may_wait_for_a_cheaper_state,
]
