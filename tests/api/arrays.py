"""Native array-fluent support (Phase 1: constant-index lowering L1/L3/L5).

Array fluents have no PDDL syntax and UP ships no plan validator for
ARRAY_FLUENTS problems, so each case builds the UP problem via the model API,
solves it through RantanPlan (which lowers array accesses to scalar cells in the
grounder), and validates the plan through one of two array-free surrogates that
UP *can* validate:

  * AR oracle (constant-literal-index problems): compile the problem with UP's
    ``ArrayFluentsRemover`` to a scalar equivalent P'. Both the native plan
    (mapped onto P' by action name + parameters — AR preserves action
    signatures) and RantanPlan's own solve of P' must validate against P'.
  * Scalar reference (int-parameter / range-variable indices, which AR refuses):
    a hand-written array-free equivalent with the same action signatures and
    conditional effects, validated exactly like tests/api/range_variables.py.

Error cases assert the ill-formed model is rejected (a hard C++ error surfaces
as INTERNAL_ERROR, or an unsatisfiable bound as UNSOLVABLE_PROVEN). The 2x2
sliding-puzzle integration test replays the plan through a tiny domain-specific
simulator (the paper's running example). See docs/native-arrays-plan.md.
"""
from unified_planning.shortcuts import (
    Fluent, InstantaneousAction, Problem, IntType, BoolType, UserType, Object,
    ArrayType, Compiler, PlanValidator, And, Or, Not, Equals, GE, LT, Plus, Minus, Int,
)
from unified_planning.engines import (
    CompilationKind, ValidationResultStatus, PlanGenerationResultStatus,
)
from unified_planning.exceptions import UPException
from unified_planning.plans import SequentialPlan, ActionInstance
from unified_planning.model.int_variable import IntVariable

from ..harness import solve

_ENV_EM = None


def _em():
    global _ENV_EM
    if _ENV_EM is None:
        from unified_planning.environment import get_environment
        _ENV_EM = get_environment().expression_manager
    return _ENV_EM


# ── validation surrogates ────────────────────────────────────────────────────

def _ar_compile(p):
    """Compile arrays away with UP's ArrayFluentsRemover (constant indices only)."""
    with Compiler(problem_kind=p.kind,
                  compilation_kind=CompilationKind.ARRAY_FLUENTS_REMOVING) as c:
        return c.compile(p, CompilationKind.ARRAY_FLUENTS_REMOVING).problem


def _map_plan(p_ref, plan):
    """Re-anchor a plan's action instances onto p_ref (same names + params)."""
    return SequentialPlan([
        ActionInstance(p_ref.action(ai.action.name), ai.actual_parameters)
        for ai in plan.actions])


def _assert_valid(p_ref, plan, label):
    with PlanValidator(problem_kind=p_ref.kind) as pv:
        vr = pv.validate(p_ref, plan)
    assert vr.status == ValidationResultStatus.VALID, \
        f"{label}: plan invalid under array-free equivalent ({vr.status})"


def _ar_check(p):
    """AR oracle: native plan and RantanPlan's AR-solve both validate vs P'.
    Returns the native PlanGenerationResult for extra assertions."""
    p_ar = _ar_compile(p)
    r = solve(p)
    assert r.plan is not None, f"{p.name}: no native plan"
    _assert_valid(p_ar, _map_plan(p_ar, r.plan), f"{p.name} native")

    r_ar = solve(p_ar)
    assert r_ar.plan is not None, f"{p.name}: no AR-oracle plan"
    _assert_valid(p_ar, r_ar.plan, f"{p.name} AR-oracle")
    return r


def _ref_check(p_native, p_ref):
    """Solve the array problem, validate its plan against a hand-written
    array-free reference (same action signatures). Returns the native result."""
    r = solve(p_native)
    assert r.plan is not None, f"{p_native.name}: no native plan"
    _assert_valid(p_ref, _map_plan(p_ref, r.plan), f"{p_native.name} vs reference")
    return r


def _assert_rejected(build):
    """The model is ill-formed: solving raises, or returns no plan with a
    non-SOLVED status."""
    try:
        r = solve(build())
    except UPException:
        return
    assert r.plan is None, "expected no plan for an ill-formed array model"
    assert r.status not in (PlanGenerationResultStatus.SOLVED_SATISFICING,
                            PlanGenerationResultStatus.SOLVED_OPTIMALLY), r.status


# ═══════════════════════ base semantics (AR oracle) ══════════════════════════

def test_1d_int_read_write_goal():
    """1-D int array: constant-index read + write + goal on cells."""
    em = _em()
    a = Fluent("a", ArrayType(3, IntType(0, 9)))
    p = Problem("arr1d")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [1, 2, 3])
    # write a[0]:=5, and a read: a[2]:=a[0] (copies the pre-state a[0]=1).
    w = InstantaneousAction("w")
    w.add_effect(em.ArrayIndex(a(), Int(0)), 5)
    p.add_action(w)
    cp = InstantaneousAction("cp")
    cp.add_effect(em.ArrayIndex(a(), Int(2)), em.ArrayIndex(a(), Int(0)))
    p.add_action(cp)
    p.add_goal(And(Equals(em.ArrayIndex(a(), Int(0)), 5),
                   Equals(em.ArrayIndex(a(), Int(2)), 1)))  # cp must fire before w
    _ar_check(p)


def test_swap_one_action_reads_prestate():
    """a[0]:=a[1]; a[1]:=a[0] in one action: both RHS see the pre-state, so the
    values swap rather than duplicate."""
    em = _em()
    a = Fluent("a", ArrayType(2, IntType(0, 9)))
    p = Problem("swap")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [3, 7])
    sw = InstantaneousAction("swap")
    sw.add_effect(em.ArrayIndex(a(), Int(0)), em.ArrayIndex(a(), Int(1)))
    sw.add_effect(em.ArrayIndex(a(), Int(1)), em.ArrayIndex(a(), Int(0)))
    p.add_action(sw)
    # A read-post-state bug would give [7,7]; the swap must give [7,3].
    p.add_goal(And(Equals(em.ArrayIndex(a(), Int(0)), 7),
                   Equals(em.ArrayIndex(a(), Int(1)), 3)))
    r = _ar_check(p)
    assert [ai.action.name for ai in r.plan.actions] == ["swap"]


def test_increase_decrease_cell():
    """INCREASE / DECREASE on a cell accumulate correctly. (The bounded-int
    interior-target case that once misbehaved was the goal-relevance
    value-chaining bug, since fixed and guarded by
    regressions.py::test_goal_relevance_numeric_value_chaining.)"""
    em = _em()
    a = Fluent("inc_a", ArrayType(2, IntType()))
    p = Problem("incdec")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0, 0])
    inc = InstantaneousAction("inc")
    inc.add_increase_effect(em.ArrayIndex(a(), Int(0)), 5)
    dec = InstantaneousAction("dec")
    dec.add_decrease_effect(em.ArrayIndex(a(), Int(0)), 2)
    p.add_action(inc)
    p.add_action(dec)
    p.add_goal(Equals(em.ArrayIndex(a(), Int(0)), 3))  # 0 +5 -2 = 3
    _ar_check(p)


def test_conditional_effect_on_cell():
    """Conditional effect `when p: a[1]:=5` fires only when the guard holds."""
    em = _em()
    a = Fluent("a", ArrayType(2, IntType(0, 9)))
    p_ = Fluent("p", BoolType())
    p = Problem("cond")
    p.add_fluent(a, default_initial_value=0)
    p.add_fluent(p_, default_initial_value=False)
    p.set_initial_value(a, [0, 0])
    p.set_initial_value(p_, True)
    act = InstantaneousAction("cset")
    act.add_effect(em.ArrayIndex(a(), Int(1)), 5, condition=p_())
    p.add_action(act)
    p.add_goal(Equals(em.ArrayIndex(a(), Int(1)), 5))
    _ar_check(p)


def test_boolean_array_precond_and_cwa():
    """Boolean array: a boolean cell in a precondition, true/false cells, and
    CWA-defaulting of unspecified boolean cells to false."""
    em = _em()
    b = Fluent("b", ArrayType(3, BoolType()))
    p = Problem("boolarr")
    p.add_fluent(b, default_initial_value=False)
    p.set_initial_value(em.ArrayIndex(b(), Int(0)), True)  # b[1], b[2] unset -> CWA false

    act = InstantaneousAction("setb1")
    act.add_precondition(em.ArrayIndex(b(), Int(0)))       # boolean cell precondition
    act.add_effect(em.ArrayIndex(b(), Int(1)), True)
    p.add_action(act)
    # b[2] must be false by CWA for this goal to hold.
    p.add_goal(And(em.ArrayIndex(b(), Int(1)), Not(em.ArrayIndex(b(), Int(2)))))

    # Scalar reference (booleans validate directly under UP).
    b0, b1, b2 = (Fluent(n, BoolType()) for n in ("b0", "b1", "b2"))
    ref = Problem("boolref")
    for f in (b0, b1, b2):
        ref.add_fluent(f, default_initial_value=False)
    ref.set_initial_value(b0(), True)
    ra = InstantaneousAction("setb1")
    ra.add_precondition(b0())
    ra.add_effect(b1(), True)
    ref.add_action(ra)
    ref.add_goal(And(b1(), Not(b2())))
    _ref_check(p, ref)


def test_multidim_2d():
    """2-D array: nested constant access, row-major cells, per-dim sizes."""
    em = _em()
    g = Fluent("g", ArrayType(2, ArrayType(2, IntType(0, 9))))

    def cell(r, c):
        return em.ArrayIndex(em.ArrayIndex(g(), Int(r)), Int(c))

    p = Problem("md2d")
    p.add_fluent(g, default_initial_value=0)
    p.set_initial_value(g, [[1, 2], [3, 4]])
    act = InstantaneousAction("setg")
    act.add_effect(cell(0, 1), 7)
    p.add_action(act)
    # g[1][0] must still read the initial 3 (row-major independence).
    p.add_goal(And(Equals(cell(0, 1), 7), Equals(cell(1, 0), 3)))
    _ar_check(p)


def test_initial_state_both_shapes():
    """Whole-array nested-list init AND per-cell ARRAY_INDEX-keyed init."""
    em = _em()
    x = Fluent("x", ArrayType(2, IntType(0, 9)))
    y = Fluent("y", ArrayType(2, IntType(0, 9)))
    p = Problem("initshapes")
    p.add_fluent(x, default_initial_value=0)
    p.add_fluent(y, default_initial_value=0)
    p.set_initial_value(x, [1, 2])                       # whole-array
    p.set_initial_value(em.ArrayIndex(y(), Int(0)), 3)   # per-cell
    p.set_initial_value(em.ArrayIndex(y(), Int(1)), 4)   # per-cell
    act = InstantaneousAction("bump")
    act.add_increase_effect(em.ArrayIndex(x(), Int(0)), em.ArrayIndex(y(), Int(1)))  # x0 += y1
    p.add_action(act)
    p.add_goal(And(Equals(em.ArrayIndex(x(), Int(0)), 5),   # 1 + 4
                   Equals(em.ArrayIndex(y(), Int(0)), 3)))
    _ar_check(p)


def test_default_initial_value_applies_to_cells():
    """default_initial_value on an array fluent pins every cell without an
    explicit value at t=0. With a[1] defaulted to 0, no writer of a[1], and goal
    a[1] == 7, the problem is unsolvable — if the default were dropped, a[1]
    would be a free variable and the solver could 'satisfy' the goal with an
    empty plan."""
    em = _em()
    a = Fluent("dflt_a", ArrayType(2, IntType(0, 9)))
    p = Problem("dfltcells")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(em.ArrayIndex(a(), Int(0)), 1)  # only a[0] explicit
    w = InstantaneousAction("w0")
    w.add_effect(em.ArrayIndex(a(), Int(0)), 5)          # nothing writes a[1]
    p.add_action(w)
    p.add_goal(Equals(em.ArrayIndex(a(), Int(1)), 7))
    r = solve(p)
    assert r.plan is None, f"free-variable cell: got a plan ({r.plan})"
    assert r.status not in (PlanGenerationResultStatus.SOLVED_SATISFICING,
                            PlanGenerationResultStatus.SOLVED_OPTIMALLY), r.status


def test_default_with_partial_explicit_override():
    """Explicit per-cell values take precedence over the array default; the
    remaining cells get the default."""
    em = _em()
    a = Fluent("dflt_b", ArrayType(3, IntType(0, 9)))
    p = Problem("dfltoverride")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(em.ArrayIndex(a(), Int(1)), 4)  # explicit beats default
    w = InstantaneousAction("w")
    w.add_effect(em.ArrayIndex(a(), Int(0)), 2)
    p.add_action(w)
    # a[0] written, a[1] explicit 4, a[2] defaulted 0 — all three checked.
    p.add_goal(And(Equals(em.ArrayIndex(a(), Int(0)), 2),
                   Equals(em.ArrayIndex(a(), Int(1)), 4),
                   Equals(em.ArrayIndex(a(), Int(2)), 0)))
    r = solve(p)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status


def test_boolean_array_cwa_without_default():
    """Boolean cells with NO declared default and no explicit value are still
    false at t=0 (same closed-world treatment as an uninitialized scalar)."""
    em = _em()
    b = Fluent("cwa_nodefault", ArrayType(2, BoolType()))
    p = Problem("cwanodflt")
    p.add_fluent(b)  # no default_initial_value at all
    p.set_initial_value(em.ArrayIndex(b(), Int(0)), True)
    act = InstantaneousAction("noopish")
    act.add_effect(em.ArrayIndex(b(), Int(0)), True)
    p.add_action(act)
    p.add_goal(And(em.ArrayIndex(b(), Int(0)), Not(em.ArrayIndex(b(), Int(1)))))
    r = solve(p)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status


def test_nonzero_default_honoured_and_grounding_sound():
    """A non-zero array default must be honoured at t=0 *and* seen by the
    grounder's numeric pruning. a[1] defaults to 5 (no explicit value, no
    writer); `use` needs a[1] >= 5 and is the only writer of the goal `done`.
    Two failure modes are ruled out at once: if CWA dropped the default, a[1]
    would be 0 and `use`'s precondition would fail at solve time; if the grounder
    seeded the absent cell at [0,0] instead of [5,5], it would prune `use`
    before solving. Solvability with `use` in the plan pins both."""
    em = _em()
    a = Fluent("nzdflt", ArrayType(2, IntType(0, 9)))
    done = Fluent("nz_done", BoolType())
    p = Problem("nzdefault")
    p.add_fluent(a, default_initial_value=5)
    p.add_fluent(done, default_initial_value=False)
    p.set_initial_value(em.ArrayIndex(a(), Int(0)), 0)   # a[1] defaults to 5
    use = InstantaneousAction("use")
    use.add_precondition(GE(em.ArrayIndex(a(), Int(1)), 5))
    use.add_effect(done(), True)
    p.add_action(use)
    p.add_goal(done())
    r = solve(p)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    assert r.plan is not None and len(r.plan.actions) == 1, r.plan


def test_boolean_cell_copy():
    """Expression-valued BOOLEAN cell assignment (`a[1] := a[0]`) with a
    dynamic source: the copied value must chain through both the reachability
    fixpoint and goal-relevance pruning. Guards the array face of the
    boolean-assign-from-fluent fixes (the scalar face and the full failure
    story live in regressions.py::test_boolean_assign_from_fluent): the
    grounder/h2 must classify the copy as a may-add, and the relevance closure
    must keep `seta0` — the copy's effect *reads* a[0], a value dependency no
    boolean sub-goal represents."""
    em = _em()
    a = Fluent("bcopy", ArrayType(2, BoolType()))
    p = Problem("boolcopy")
    p.add_fluent(a, default_initial_value=False)
    seta = InstantaneousAction("seta0")
    seta.add_effect(em.ArrayIndex(a(), Int(0)), True)
    p.add_action(seta)
    cp = InstantaneousAction("copy01")
    cp.add_effect(em.ArrayIndex(a(), Int(1)), em.ArrayIndex(a(), Int(0)))
    p.add_action(cp)
    p.add_goal(em.ArrayIndex(a(), Int(1)))
    r = _ar_check(p)
    names = [ai.action.name for ai in r.plan.actions]
    assert names.index("seta0") < names.index("copy01"), names


def test_boolean_cell_swap():
    """Boolean swap in one action (`a[0]:=a[1]; a[1]:=a[0]`): both RHS read
    the pre-state, so [true,false] becomes [false,true] in a single step —
    the boolean twin of test_swap_one_action_reads_prestate."""
    em = _em()
    b = Fluent("bswap", ArrayType(2, BoolType()))
    p = Problem("boolswap")
    p.add_fluent(b, default_initial_value=False)
    p.set_initial_value(em.ArrayIndex(b(), Int(0)), True)
    sw = InstantaneousAction("swap")
    sw.add_effect(em.ArrayIndex(b(), Int(0)), em.ArrayIndex(b(), Int(1)))
    sw.add_effect(em.ArrayIndex(b(), Int(1)), em.ArrayIndex(b(), Int(0)))
    p.add_action(sw)
    p.add_goal(And(Not(em.ArrayIndex(b(), Int(0))), em.ArrayIndex(b(), Int(1))))
    r = _ar_check(p)
    assert [ai.action.name for ai in r.plan.actions] == ["swap"]


def test_size1_array_degenerates_to_scalar():
    """A size-1 array behaves as an ordinary scalar cell."""
    em = _em()
    a = Fluent("a", ArrayType(1, IntType(0, 9)))
    p = Problem("size1")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [3])
    act = InstantaneousAction("set")
    act.add_effect(em.ArrayIndex(a(), Int(0)), 8)
    p.add_action(act)
    p.add_goal(Equals(em.ArrayIndex(a(), Int(0)), 8))
    _ar_check(p)


# ═══════════════════════ whole-array literals (AR oracle) ════════════════════

def test_whole_array_goal_equality_literal():
    """Whole-array equality against a 2-D literal in the GOAL — n_puzzle's
    `Equals(puzzle, goal_blocks)`. The converter expands it element-wise into
    per-cell equalities; UP's ArrayFluentsRemover expands the same literal, so
    the native solve and the AR oracle agree."""
    em = _em()
    g = Fluent("wg", ArrayType(2, ArrayType(2, IntType(0, 9))))

    def cell(r, c):
        return em.ArrayIndex(em.ArrayIndex(g(), Int(r)), Int(c))

    p = Problem("wholegoal")
    p.add_fluent(g, default_initial_value=0)
    p.set_initial_value(g, [[1, 2], [3, 4]])
    act = InstantaneousAction("seta")
    act.add_effect(cell(0, 0), 5)
    act.add_effect(cell(1, 1), 6)
    p.add_action(act)
    p.add_goal(Equals(g(), [[5, 2], [3, 6]]))   # whole-array literal goal
    r = _ar_check(p)
    assert [ai.action.name for ai in r.plan.actions] == ["seta"]


def test_whole_array_goal_disequality_literal():
    """`Not(Equals(array, literal))` in the goal: the whole-array equality
    expands element-wise and the surrounding Not turns it into a disjunction of
    per-cell disequalities. Initially the array *equals* the literal (goal
    false), so an action must perturb a cell — proving the expansion is
    load-bearing, not vacuously satisfied by the empty plan."""
    em = _em()
    a = Fluent("wd", ArrayType(2, IntType(0, 9)))
    p = Problem("wholediseq")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [1, 2])
    bump = InstantaneousAction("bump")
    bump.add_effect(em.ArrayIndex(a(), Int(0)), 7)
    p.add_action(bump)
    p.add_goal(Not(Equals(a(), [1, 2])))        # must differ from [1, 2]
    r = _ar_check(p)
    assert [ai.action.name for ai in r.plan.actions] == ["bump"]


def test_whole_array_assignment_effect_literal():
    """A whole-array assignment effect from a literal (`a := [3, 4]`) expands to
    one per-cell assignment sharing the effect's kind/condition/forall. Validated
    against a hand-written scalar reference: UP's ArrayFluentsRemover leaves a
    whole-array assignment effect un-expanded (it only rewrites init/goal/reads),
    so it is not a valid oracle for this construct — the reference is."""
    em = _em()
    a = Fluent("wae", ArrayType(2, IntType(0, 9)))
    p = Problem("wholeassign")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0, 0])
    setall = InstantaneousAction("setall")
    # A bare list value would be flattened by add_effect's auto_promote; wrap it
    # in an explicit ARRAY_CONSTANT (em.Array) so it reaches the converter whole.
    setall.add_effect(a(), em.Array([3, 4]))    # whole-array literal assignment
    p.add_action(setall)
    p.add_goal(And(Equals(em.ArrayIndex(a(), Int(0)), 3),
                   Equals(em.ArrayIndex(a(), Int(1)), 4)))

    a0, a1 = (Fluent(n, IntType(0, 9)) for n in ("wae0", "wae1"))
    ref = Problem("wholeassignref")
    for f in (a0, a1):
        ref.add_fluent(f, default_initial_value=0)
        ref.set_initial_value(f, 0)
    rset = InstantaneousAction("setall")
    rset.add_effect(a0(), 3)
    rset.add_effect(a1(), 4)
    ref.add_action(rset)
    ref.add_goal(And(Equals(a0(), 3), Equals(a1(), 4)))

    r = _ref_check(p, ref)
    assert [ai.action.name for ai in r.plan.actions] == ["setall"]


# ═══════════════ int-parameter / range-variable indices (reference) ══════════

def test_index_arithmetic_int_param():
    """a[c+1] := a[c] with an int parameter c: instantiation must produce the
    right cells (a[c+1] and a[c])."""
    em = _em()
    a = Fluent("a", ArrayType(3, IntType(0, 9)))
    p = Problem("shift")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [1, 2, 3])
    sh = InstantaneousAction("shift", c=IntType(0, 1))
    c = sh.parameter("c")
    sh.add_effect(em.ArrayIndex(a(), Plus(c, 1)), em.ArrayIndex(a(), c))
    p.add_action(sh)
    p.add_goal(Equals(em.ArrayIndex(a(), Int(1)), 1))  # shift(0): a[1] := a[0] = 1

    a0, a1, a2 = (Fluent(n, IntType(0, 9)) for n in ("a0", "a1", "a2"))
    ref = Problem("shiftref")
    for f, v in ((a0, 1), (a1, 2), (a2, 3)):
        ref.add_fluent(f, default_initial_value=0)
        ref.set_initial_value(f, v)
    rsh = InstantaneousAction("shift", c=IntType(0, 1))
    rc = rsh.parameter("c")
    rsh.add_effect(a1(), a0(), condition=Equals(rc, 0))
    rsh.add_effect(a2(), a1(), condition=Equals(rc, 1))
    ref.add_action(rsh)
    ref.add_goal(Equals(a1(), 1))

    r = _ref_check(p, ref)
    assert [ai.actual_parameters[0].int_constant_value()
            for ai in r.plan.actions] == [0]


def test_forall_effect_range_variable_reverse():
    """Regression: a forall EFFECT over a range variable (`forall b in [0,2]:
    a[b] := a[2-b]`, an in-place reverse) must fire. The effect-forall variable
    used to reach C++ as a bare leaf without its [lo,hi] bounds, so the
    QuantifierExpansionPass read it as an object variable, enumerated an empty
    domain, and silently dropped the whole effect — turning the action into a
    no-op and 'proving' this solvable problem UNSOLVABLE."""
    em = _em()
    a = Fluent("a", ArrayType(3, IntType(0, 2)))
    p = Problem("reverse")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [2, 1, 0])
    rev = InstantaneousAction("rev")
    b = IntVariable("b", 0, 2)
    rev.add_effect(em.ArrayIndex(a(), b), em.ArrayIndex(a(), Minus(Int(2), b)), forall=[b])
    p.add_action(rev)
    p.add_goal(And(Equals(em.ArrayIndex(a(), Int(0)), 0),
                   Equals(em.ArrayIndex(a(), Int(1)), 1),
                   Equals(em.ArrayIndex(a(), Int(2)), 2)))

    a0, a1, a2 = (Fluent(n, IntType(0, 2)) for n in ("a0", "a1", "a2"))
    ref = Problem("reverseref")
    for f, v in ((a0, 2), (a1, 1), (a2, 0)):
        ref.add_fluent(f, default_initial_value=0)
        ref.set_initial_value(f, v)
    rrev = InstantaneousAction("rev")
    rrev.add_effect(a0(), a2())  # b=0: a[0]:=a[2]
    rrev.add_effect(a1(), a1())  # b=1: a[1]:=a[1]
    rrev.add_effect(a2(), a0())  # b=2: a[2]:=a[0]
    ref.add_action(rrev)
    ref.add_goal(And(Equals(a0(), 0), Equals(a1(), 1), Equals(a2(), 2)))

    r = _ref_check(p, ref)
    assert [ai.action.name for ai in r.plan.actions] == ["rev"]


def test_forall_effect_parameter_dependent_bound():
    """Regression companion: a forall EFFECT whose bound depends on an int
    action parameter (`flip(f): forall b in [0,f]: a[b] := a[f-b]`, the pancake
    flip). Instantiation must fix f, then expand b over [0,f]. Solve the two-cell
    case where flip(1) swaps the pair."""
    em = _em()
    a = Fluent("a", ArrayType(2, IntType(0, 1)))
    p = Problem("flip2")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [1, 0])
    flip = InstantaneousAction("flip", f=IntType(1, 1))
    f = flip.parameter("f")
    b = IntVariable("b", 0, f)
    flip.add_effect(em.ArrayIndex(a(), b), em.ArrayIndex(a(), Minus(f, b)), forall=[b])
    p.add_action(flip)
    p.add_goal(And(Equals(em.ArrayIndex(a(), Int(0)), 0),
                   Equals(em.ArrayIndex(a(), Int(1)), 1)))

    a0, a1 = (Fluent(n, IntType(0, 1)) for n in ("a0", "a1"))
    ref = Problem("flip2ref")
    for fl, v in ((a0, 1), (a1, 0)):
        ref.add_fluent(fl, default_initial_value=0)
        ref.set_initial_value(fl, v)
    rflip = InstantaneousAction("flip", f=IntType(1, 1))
    rflip.add_effect(a0(), a1())  # f=1, b=0: a[0]:=a[1]
    rflip.add_effect(a1(), a0())  # f=1, b=1: a[1]:=a[0]
    ref.add_action(rflip)
    ref.add_goal(And(Equals(a0(), 0), Equals(a1(), 1)))

    _ref_check(p, ref)


def test_forall_effect_arithmetic_range_bound():
    """Regression: a range-variable bound that is *arithmetic* over an int
    parameter (`forall b in [0, r-1]: a[b] := 9`). The bound `r-1` reaches the
    converter as an FNode (not a bare int or parameter), so `_bound_node` must
    walk it as an expression; C++ `eval_bound` must then fold the
    parameter-substituted constant arithmetic (`(- 3 1)` -> 2). Previously the
    converter crashed with `'FNode' object has no attribute 'name'`."""
    em = _em()
    a = Fluent("a", ArrayType(3, IntType(0, 9)))
    p = Problem("arith")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0, 0, 0])
    fill = InstantaneousAction("fill", r=IntType(1, 3))
    r = fill.parameter("r")
    b = IntVariable("b", 0, Minus(r, 1))
    fill.add_effect(em.ArrayIndex(a(), b), 9, forall=[b])
    p.add_action(fill)
    p.add_goal(And(Equals(em.ArrayIndex(a(), Int(0)), 9),
                   Equals(em.ArrayIndex(a(), Int(1)), 9),
                   Equals(em.ArrayIndex(a(), Int(2)), 9)))  # forces r=3

    a0, a1, a2 = (Fluent(n, IntType(0, 9)) for n in ("a0", "a1", "a2"))
    ref = Problem("arithref")
    for f in (a0, a1, a2):
        ref.add_fluent(f, default_initial_value=0)
        ref.set_initial_value(f, 0)
    rf = InstantaneousAction("fill", r=IntType(1, 3))
    rr = rf.parameter("r")
    rf.add_effect(a0(), 9, condition=GE(rr, 1))  # cell i is filled iff i <= r-1
    rf.add_effect(a1(), 9, condition=GE(rr, 2))
    rf.add_effect(a2(), 9, condition=GE(rr, 3))
    ref.add_action(rf)
    ref.add_goal(And(Equals(a0(), 9), Equals(a1(), 9), Equals(a2(), 9)))

    r_res = _ref_check(p, ref)
    assert [ai.actual_parameters[0].int_constant_value()
            for ai in r_res.plan.actions] == [3]


def test_quantified_forall_goal_index():
    """forall i in 0..2: a[i] == 0 (range var used as index) — expanded by the
    C++ QuantifierExpansionPass into three constant-index cell goals."""
    em = _em()
    a = Fluent("a", ArrayType(3, IntType(0, 9)))
    p = Problem("qforall")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [1, 1, 1])
    z = InstantaneousAction("zero", k=IntType(0, 2))
    z.add_effect(em.ArrayIndex(a(), z.parameter("k")), 0)
    p.add_action(z)
    i = IntVariable("i", 0, 2)
    p.add_goal(em.Forall(Equals(em.ArrayIndex(a(), i), 0), i))

    a0, a1, a2 = (Fluent(n, IntType(0, 9)) for n in ("a0", "a1", "a2"))
    ref = Problem("qforallref")
    for f in (a0, a1, a2):
        ref.add_fluent(f, default_initial_value=0)
        ref.set_initial_value(f, 1)
    rz = InstantaneousAction("zero", k=IntType(0, 2))
    rk = rz.parameter("k")
    rz.add_effect(a0(), 0, condition=Equals(rk, 0))
    rz.add_effect(a1(), 0, condition=Equals(rk, 1))
    rz.add_effect(a2(), 0, condition=Equals(rk, 2))
    ref.add_action(rz)
    ref.add_goal(And(Equals(a0(), 0), Equals(a1(), 0), Equals(a2(), 0)))
    _ref_check(p, ref)


def test_quantified_exists_goal_index():
    """exists i in 0..2: a[i] == 5 (range var index, OR expansion)."""
    em = _em()
    a = Fluent("a", ArrayType(3, IntType(0, 9)))
    p = Problem("qexists")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0, 0, 0])
    s = InstantaneousAction("set", k=IntType(0, 2))
    s.add_effect(em.ArrayIndex(a(), s.parameter("k")), 5)
    p.add_action(s)
    i = IntVariable("i", 0, 2)
    p.add_goal(em.Exists(Equals(em.ArrayIndex(a(), i), 5), i))

    a0, a1, a2 = (Fluent(n, IntType(0, 9)) for n in ("a0", "a1", "a2"))
    ref = Problem("qexistsref")
    for f in (a0, a1, a2):
        ref.add_fluent(f, default_initial_value=0)
        ref.set_initial_value(f, 0)
    rs = InstantaneousAction("set", k=IntType(0, 2))
    rk = rs.parameter("k")
    rs.add_effect(a0(), 5, condition=Equals(rk, 0))
    rs.add_effect(a1(), 5, condition=Equals(rk, 1))
    rs.add_effect(a2(), 5, condition=Equals(rk, 2))
    ref.add_action(rs)
    ref.add_goal(Or(Equals(a0(), 5), Equals(a1(), 5), Equals(a2(), 5)))
    _ref_check(p, ref)


# ═══════════════════════════ edge cases ══════════════════════════════════════

def test_oob_by_parameter_enumeration():
    """A parameter range that includes out-of-bounds indices: the OOB instances
    are discarded, the problem still solves via the in-bounds ones."""
    em = _em()
    a = Fluent("a", ArrayType(2, IntType(0, 9)))
    p = Problem("oobparam")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0, 0])
    st = InstantaneousAction("setc", c=IntType(0, 3))  # c=2,3 land OOB and drop
    st.add_effect(em.ArrayIndex(a(), st.parameter("c")), 1)
    p.add_action(st)
    p.add_goal(Equals(em.ArrayIndex(a(), Int(1)), 1))

    a0, a1 = Fluent("a0", IntType(0, 9)), Fluent("a1", IntType(0, 9))
    ref = Problem("oobref")
    for f in (a0, a1):
        ref.add_fluent(f, default_initial_value=0)
        ref.set_initial_value(f, 0)
    rst = InstantaneousAction("setc", c=IntType(0, 3))
    rc = rst.parameter("c")
    rst.add_effect(a0(), 1, condition=Equals(rc, 0))
    rst.add_effect(a1(), 1, condition=Equals(rc, 1))
    ref.add_action(rst)
    ref.add_goal(Equals(a1(), 1))

    r = _ref_check(p, ref)
    for ai in r.plan.actions:
        assert 0 <= ai.actual_parameters[0].int_constant_value() <= 1, \
            "an out-of-bounds instance leaked into the plan"


def test_undefined_positions_discard_and_neighbours():
    """Undefined-position cells are never minted; instances touching them are
    discarded, while in-bounds neighbours are unaffected."""
    em = _em()
    # Unique fluent name: UP caches fluents by name+type in the global
    # environment, and re-using a name from another test drops this fluent's
    # undefined_positions (a test-isolation artifact; a single real problem is
    # unaffected).
    a = Fluent("sparse_d", ArrayType(3, IntType(0, 9)), undefined_positions=[(1,)])
    p = Problem("undef")
    p.add_fluent(a)
    p.set_initial_value(a, [1, 2, 3])  # whole-array init skips the undefined cell
    st = InstantaneousAction("setc", c=IntType(0, 2))  # c=1 hits undefined and drops
    st.add_effect(em.ArrayIndex(a(), st.parameter("c")), 9)
    p.add_action(st)
    p.add_goal(Equals(em.ArrayIndex(a(), Int(2)), 9))
    r = solve(p)
    assert r.plan is not None and r.status == PlanGenerationResultStatus.SOLVED_SATISFICING
    for ai in r.plan.actions:
        assert ai.actual_parameters[0].int_constant_value() in (0, 2), \
            "an undefined-position instance leaked into the plan"


def test_oob_precondition_read_discard():
    """A constant-folded OOB read in a PRECONDITION discards that instance;
    the in-bounds instance still solves the problem."""
    em = _em()
    a = Fluent("preoob_a", ArrayType(3, IntType(0, 9)))
    flag = Fluent("preoob_flag", BoolType())
    p = Problem("preoob")
    p.add_fluent(a, default_initial_value=0)
    p.add_fluent(flag, default_initial_value=False)
    p.set_initial_value(a, [0, 0, 3])
    act = InstantaneousAction("need", k=IntType(0, 4))  # k=3,4 read OOB -> dropped
    act.add_precondition(GE(em.ArrayIndex(a(), act.parameter("k")), Int(3)))
    act.add_effect(flag(), True)
    p.add_action(act)
    p.add_goal(flag())
    r = solve(p)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    for ai in r.plan.actions:
        assert ai.actual_parameters[0].int_constant_value() == 2, \
            "only k=2 satisfies a[k]>=3 in bounds"


def test_oob_effect_rhs_read_discard():
    """A constant-folded OOB read in an EFFECT RHS discards that instance;
    the in-bounds instance still solves the problem."""
    em = _em()
    a = Fluent("rhsoob_a", ArrayType(3, IntType(0, 9)))
    p = Problem("rhsoob")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0, 0, 3])
    cp = InstantaneousAction("cp", k=IntType(0, 4))  # k=3,4 read OOB -> dropped
    cp.add_effect(em.ArrayIndex(a(), Int(0)), em.ArrayIndex(a(), cp.parameter("k")))
    p.add_action(cp)
    p.add_goal(Equals(em.ArrayIndex(a(), Int(0)), 3))
    r = solve(p)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    for ai in r.plan.actions:
        assert ai.actual_parameters[0].int_constant_value() == 2, \
            "only k=2 copies the value 3"


def test_guarded_oob_permissive_survives():
    """Permissive undefinedness (paper Def. 9): a *guarded* out-of-bounds access
    must not discard the whole action — the boundary instance survives with the
    offending part removed. This is the plotting `shoot` pattern.

        a : array[2] of int[0,9], init [0, 0]
        step(k), k in 0..1:
          pre : Or(k == 1, a[k+1] >= 0)         # k=1: a[2] is OOB, but the OR's
                                                 #   other disjunct is true → kept
          eff : a[k+1] := 5  WHEN k < 1          # k=1: OOB target, guard false → dropped
          eff : a[k] := 5                        # unconditional, always in-bounds
        goal : a[1] == 5 AND a[0] == 0

    Only ``step(1)`` reaches the goal (``step(0)`` also writes ``a[0]``). Before
    permissive lowering, ``step(1)``'s OOB accesses (in the OR-precondition and
    the guarded conditional effect) discarded the entire instance, leaving the
    problem UNSOLVABLE. Now the OR drops the undefined disjunct and the guarded
    conditional effect is dropped, so ``step(1)`` survives and solves it."""
    em = _em()
    a = Fluent("guard_a", ArrayType(2, IntType(0, 9)))
    p = Problem("guardoob")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0, 0])
    step = InstantaneousAction("step", k=IntType(0, 1))
    k = step.parameter("k")
    step.add_precondition(Or(Equals(k, 1),
                             GE(em.ArrayIndex(a(), Plus(k, 1)), Int(0))))
    step.add_effect(em.ArrayIndex(a(), Plus(k, 1)), 5, condition=LT(k, Int(1)))
    step.add_effect(em.ArrayIndex(a(), k), 5)
    p.add_action(step)
    p.add_goal(And(Equals(em.ArrayIndex(a(), Int(1)), 5),
                   Equals(em.ArrayIndex(a(), Int(0)), 0)))

    r = solve(p)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    assert [ai.actual_parameters[0].int_constant_value() for ai in r.plan.actions] == [1], \
        "only the guarded-OOB boundary instance step(1) reaches the goal"


def _guarded_undefined_effect(side, with_lower):
    """The problem of the two tests below: poke(2)'s conditional effect has an
    out-of-bounds target (side='target') or value (side='value')."""
    em = _em()
    a = Fluent(f"gue_{side}_a", ArrayType(2, IntType(0, 9)))
    x = Fluent(f"gue_{side}_x", IntType(0, 9))
    flag = Fluent(f"gue_{side}_flag", BoolType())
    done = Fluent(f"gue_{side}_done", BoolType())
    p = Problem(f"gue_{side}")
    p.add_fluent(a, default_initial_value=0)
    p.add_fluent(x, default_initial_value=0)
    p.add_fluent(flag, default_initial_value=False)
    p.add_fluent(done, default_initial_value=False)
    p.set_initial_value(flag(), True)
    poke = InstantaneousAction("poke", k=IntType(2, 2))   # k = 2 is out of bounds
    k = poke.parameter("k")
    if side == "target":
        poke.add_effect(em.ArrayIndex(a(), k), 1, condition=flag())
    else:
        poke.add_effect(x(), em.ArrayIndex(a(), k), condition=flag())
    poke.add_effect(done(), True)
    p.add_action(poke)
    if with_lower:
        lower = InstantaneousAction("lower")
        lower.add_effect(flag(), False)
        p.add_action(lower)
    p.add_goal(done())
    return p


def test_guarded_undefined_effect_blocks_where_condition_holds():
    """A conditional effect ⟨C ▷ l := r⟩ whose target l or
    value r is undefined is removed and ¬C joins the precondition, so the action
    is inapplicable wherever C holds (not applicable with the effect skipped).

        a : array[2] of int[0,9];  flag (init true), done (init false)
        lower      : flag := false
        poke(k), k in 2..2 (a[2] is out of bounds)
          eff : a[k] := 1   WHEN flag      (side = target)
                x := a[k]   WHEN flag      (side = value)
          eff : done := true
        goal : done

    poke(2) needs flag false, so every plan lowers the flag first. Dropping the
    effect without ¬flag let poke(2) reach the goal in one step."""
    for side in ("target", "value"):
        r = solve(_guarded_undefined_effect(side, with_lower=True))
        assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, (side, r.status)
        names = [ai.action.name for ai in r.plan.actions]
        assert "lower" in names and "poke" in names, (side, names)
        assert names.index("lower") < names.index("poke"), (side, names)


def test_guarded_undefined_effect_never_applicable_when_condition_static():
    """The same action without `lower`: flag is always true, so poke(2) is never
    applicable and the goal is unreachable."""
    for side in ("target", "value"):
        r = solve(_guarded_undefined_effect(side, with_lower=False))
        assert r.plan is None, (side, [ai.action.name for ai in r.plan.actions])
        assert r.status in (PlanGenerationResultStatus.UNSOLVABLE_PROVEN,
                            PlanGenerationResultStatus.UNSOLVABLE_INCOMPLETELY), (side, r.status)


def test_state_invariant_with_cell_read():
    """A declared state invariant containing a cell read is lowered and
    enforced: the action violating it cannot be used."""
    from unified_planning.shortcuts import Always, LE
    em = _em()
    a = Fluent("inv_a", ArrayType(2, IntType(0, 9)))
    p = Problem("invcell")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0, 0])
    good = InstantaneousAction("good")
    good.add_effect(em.ArrayIndex(a(), Int(0)), 1)
    bad = InstantaneousAction("bad")
    bad.add_effect(em.ArrayIndex(a(), Int(0)), 1)
    bad.add_effect(em.ArrayIndex(a(), Int(1)), 9)   # violates the invariant
    p.add_action(good)
    p.add_action(bad)
    p.add_trajectory_constraint(Always(LE(em.ArrayIndex(a(), Int(1)), Int(5))))
    p.add_goal(Equals(em.ArrayIndex(a(), Int(0)), 1))
    r = solve(p)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    assert [ai.action.name for ai in r.plan.actions] == ["good"], \
        f"invariant not enforced: {r.plan}"


def test_action_cost_with_cell_read():
    """An action-cost expression containing a cell read is lowered; optimal
    mode solves with the state-dependent cost."""
    from unified_planning.model.metrics import MinimizeActionCosts
    em = _em()
    a = Fluent("cost_a", ArrayType(2, IntType(0, 9)))
    p = Problem("costcell")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [2, 0])
    w = InstantaneousAction("w")
    w.add_effect(em.ArrayIndex(a(), Int(1)), 1)
    p.add_action(w)
    p.add_goal(Equals(em.ArrayIndex(a(), Int(1)), 1))
    p.add_quality_metric(MinimizeActionCosts({w: em.Plus(em.ArrayIndex(a(), Int(0)), 1)}))
    r = solve(p, optimal=True)
    assert r.status == PlanGenerationResultStatus.SOLVED_OPTIMALLY, r.status


def test_same_cell_collision_instance_dependent():
    """a[i]:=5; a[j]:=6 with int params: the i==j instances assign one cell two
    values, which UP can never execute, so EffectNormalizationPass removes them
    (case 6); distinct-cell instances solve the problem."""
    em = _em()
    a = Fluent("a", ArrayType(2, IntType(0, 9)))
    p = Problem("collinst")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0, 0])
    wp = InstantaneousAction("writepair", i=IntType(0, 1), j=IntType(0, 1))
    i, j = wp.parameter("i"), wp.parameter("j")
    wp.add_effect(em.ArrayIndex(a(), i), 5)
    wp.add_effect(em.ArrayIndex(a(), j), 6)
    p.add_action(wp)
    p.add_goal(And(Equals(em.ArrayIndex(a(), Int(0)), 5),
                   Equals(em.ArrayIndex(a(), Int(1)), 6)))  # needs i=0, j=1

    a0, a1 = Fluent("a0", IntType(0, 9)), Fluent("a1", IntType(0, 9))
    ref = Problem("collref")
    for f in (a0, a1):
        ref.add_fluent(f, default_initial_value=0)
        ref.set_initial_value(f, 0)
    rwp = InstantaneousAction("writepair", i=IntType(0, 1), j=IntType(0, 1))
    ri, rj = rwp.parameter("i"), rwp.parameter("j")
    rwp.add_effect(a1(), 5, condition=Equals(ri, 1))
    rwp.add_effect(a0(), 6, condition=Equals(rj, 0))
    rwp.add_effect(a1(), 6, condition=Equals(rj, 1))
    rwp.add_effect(a0(), 5, condition=Equals(ri, 0))
    ref.add_action(rwp)
    ref.add_goal(And(Equals(a0(), 5), Equals(a1(), 6)))

    r = _ref_check(p, ref)
    for ai in r.plan.actions:
        vs = [x.int_constant_value() for x in ai.actual_parameters]
        assert vs[0] != vs[1], "an instance assigning one cell two values leaked into the plan"


def test_same_cell_increases_add_up():
    """a[i] += 1; a[j] += 1 with int params: at i == j both increases hit one
    cell and add up, as for any fluent (EffectNormalizationPass, case 3). The
    goal a = [2, 0] needs exactly that instance, bump(0, 0)."""
    em = _em()
    a = Fluent("twice_inc_a", ArrayType(2, IntType(0, 9)))
    done = Fluent("twice_inc_done", BoolType())
    p = Problem("twiceinc")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0, 0])
    p.add_fluent(done, default_initial_value=False)
    bump = InstantaneousAction("bump", i=IntType(0, 1), j=IntType(0, 1))
    i, j = bump.parameter("i"), bump.parameter("j")
    bump.add_precondition(Not(done()))
    bump.add_increase_effect(em.ArrayIndex(a(), i), 1)
    bump.add_increase_effect(em.ArrayIndex(a(), j), 1)
    bump.add_effect(done(), True)
    p.add_action(bump)
    p.add_goal(And(Equals(em.ArrayIndex(a(), Int(0)), 2),
                   Equals(em.ArrayIndex(a(), Int(1)), 0)))

    a0, a1 = Fluent("a0", IntType(0, 9)), Fluent("a1", IntType(0, 9))
    ref = Problem("twiceincref")
    for f in (a0, a1):
        ref.add_fluent(f, default_initial_value=0)
    ref.add_fluent(done, default_initial_value=False)
    rbump = InstantaneousAction("bump", i=IntType(0, 1), j=IntType(0, 1))
    ri, rj = rbump.parameter("i"), rbump.parameter("j")
    rbump.add_precondition(Not(done()))
    for par in (ri, rj):
        rbump.add_increase_effect(a0(), 1, condition=Equals(par, 0))
        rbump.add_increase_effect(a1(), 1, condition=Equals(par, 1))
    rbump.add_effect(done(), True)
    ref.add_action(rbump)
    ref.add_goal(And(Equals(a0(), 2), Equals(a1(), 0)))

    r = _ref_check(p, ref)
    assert [[x.int_constant_value() for x in ai.actual_parameters]
            for ai in r.plan.actions] == [[0, 0]], r.plan


def test_same_cell_set_and_clear():
    """occ[i] := false; occ[j] := true on a Boolean array: at i == j the set
    wins (add after delete, EffectNormalizationPass case 1), so a move to the
    same cell leaves it occupied. The goal needs move(0, 0)."""
    em = _em()
    occ = Fluent("twice_occ", ArrayType(2, BoolType()))
    moved = Fluent("twice_moved", BoolType())
    p = Problem("twiceocc")
    p.add_fluent(occ, default_initial_value=False)
    p.set_initial_value(occ, [True, False])
    p.add_fluent(moved, default_initial_value=False)
    move = InstantaneousAction("move", i=IntType(0, 1), j=IntType(0, 1))
    i, j = move.parameter("i"), move.parameter("j")
    move.add_precondition(And(em.ArrayIndex(occ(), i), Not(moved())))
    move.add_effect(em.ArrayIndex(occ(), i), False)
    move.add_effect(em.ArrayIndex(occ(), j), True)
    move.add_effect(moved(), True)
    p.add_action(move)
    p.add_goal(And(moved(), em.ArrayIndex(occ(), Int(0)),
                   Not(em.ArrayIndex(occ(), Int(1)))))

    o0, o1 = Fluent("o0", BoolType()), Fluent("o1", BoolType())
    ref = Problem("twiceoccref")
    for f in (o0, o1, moved):
        ref.add_fluent(f, default_initial_value=False)
    ref.set_initial_value(o0(), True)
    rmove = InstantaneousAction("move", i=IntType(0, 1), j=IntType(0, 1))
    ri, rj = rmove.parameter("i"), rmove.parameter("j")
    rmove.add_precondition(And(Or(And(Equals(ri, 0), o0()), And(Equals(ri, 1), o1())),
                               Not(moved())))
    rmove.add_effect(o0(), False, condition=Equals(ri, 0))
    rmove.add_effect(o1(), False, condition=Equals(ri, 1))
    rmove.add_effect(o0(), True, condition=Equals(rj, 0))
    rmove.add_effect(o1(), True, condition=Equals(rj, 1))
    rmove.add_effect(moved(), True)
    ref.add_action(rmove)
    ref.add_goal(And(moved(), o0(), Not(o1())))

    r = _ref_check(p, ref)
    assert [[x.int_constant_value() for x in ai.actual_parameters]
            for ai in r.plan.actions] == [[0, 0]], r.plan


def test_same_cell_exclusive_guards():
    """WHEN c: a[i] := 1; WHEN not c: a[i] := 2. Every instance writes one cell
    twice, but the guards exclude each other, so the two writes never apply
    together and the action is kept as it is. Goal a[0] = 2: flip, put(0)."""
    em = _em()
    a = Fluent("twice_excl_a", ArrayType(2, IntType(0, 9)))
    c = Fluent("twice_excl_c", BoolType())
    p = Problem("twiceexcl")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0, 0])
    p.add_fluent(c, default_initial_value=False)
    p.set_initial_value(c(), True)
    put = InstantaneousAction("put", i=IntType(0, 1))
    i = put.parameter("i")
    put.add_effect(em.ArrayIndex(a(), i), 1, condition=c())
    put.add_effect(em.ArrayIndex(a(), i), 2, condition=Not(c()))
    flip = InstantaneousAction("flip")
    flip.add_effect(c(), False)
    p.add_action(put)
    p.add_action(flip)
    p.add_goal(Equals(em.ArrayIndex(a(), Int(0)), 2))

    a0, a1 = Fluent("a0", IntType(0, 9)), Fluent("a1", IntType(0, 9))
    ref = Problem("twiceexclref")
    for f in (a0, a1):
        ref.add_fluent(f, default_initial_value=0)
    ref.add_fluent(c, default_initial_value=False)
    ref.set_initial_value(c(), True)
    rput = InstantaneousAction("put", i=IntType(0, 1))
    ri = rput.parameter("i")
    rput.add_effect(a0(), 1, condition=And(c(), Equals(ri, 0)))
    rput.add_effect(a1(), 1, condition=And(c(), Equals(ri, 1)))
    rput.add_effect(a0(), 2, condition=And(Not(c()), Equals(ri, 0)))
    rput.add_effect(a1(), 2, condition=And(Not(c()), Equals(ri, 1)))
    rflip = InstantaneousAction("flip")
    rflip.add_effect(c(), False)
    ref.add_action(rput)
    ref.add_action(rflip)
    ref.add_goal(Equals(a0(), 2))

    _ref_check(p, ref)


def test_same_cell_folded_increases_add_up():
    """a[0] += 1; a[1-1] += 2: the two targets fold to one cell in every
    instance, and the increases add up (a[0] = 3), as in UP's model of the same
    action on a scalar fluent. (UP itself blocks syntactically identical
    targets, hence the folding.)"""
    em = _em()
    a = Fluent("twice_fold_a", ArrayType(2, IntType(0, 9)))
    p = Problem("twicefold")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0, 0])
    add3 = InstantaneousAction("add3")
    add3.add_increase_effect(em.ArrayIndex(a(), Int(0)), 1)
    add3.add_increase_effect(em.ArrayIndex(a(), Minus(Int(1), Int(1))), 2)  # folds to a[0]
    p.add_action(add3)
    p.add_goal(Equals(em.ArrayIndex(a(), Int(0)), 3))

    a0 = Fluent("a0", IntType(0, 9))
    ref = Problem("twicefoldref")
    ref.add_fluent(a0, default_initial_value=0)
    radd3 = InstantaneousAction("add3")
    radd3.add_increase_effect(a0(), 1)
    radd3.add_increase_effect(a0(), 2)
    ref.add_action(radd3)
    ref.add_goal(Equals(a0(), 3))

    r = _ref_check(p, ref)
    assert [ai.action.name for ai in r.plan.actions] == ["add3"]


def test_element_bounds_respected():
    """Cell values respect the declared Int<lb,ub> element bounds: with ub=5 and
    `a[0] += 3` from 0, the goal a[0] >= 6 is REACHABLE in two steps if the
    bound were dropped, so this is load-bearing on the bound actually holding."""
    em = _em()
    a = Fluent("bounded_cell", ArrayType(1, IntType(0, 5)))
    p = Problem("elembounds")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0])
    inc = InstantaneousAction("inc3")
    inc.add_increase_effect(em.ArrayIndex(a(), Int(0)), 3)
    p.add_action(inc)
    p.add_goal(GE(em.ArrayIndex(a(), Int(0)), Int(6)))  # 3+3=6 without the bound
    r = solve(p)
    assert r.plan is None, r.status
    assert r.status not in (PlanGenerationResultStatus.SOLVED_SATISFICING,
                            PlanGenerationResultStatus.SOLVED_OPTIMALLY), r.status


def test_non_array_regression():
    """A problem with no array fluents is entirely unaffected by array lowering
    (and UP validates it directly)."""
    from ..harness import assert_valid
    level = Fluent("level", IntType(0, 10))
    p = Problem("noarray")
    p.add_fluent(level, default_initial_value=0)
    p.set_initial_value(level, 5)
    dec = InstantaneousAction("dec")
    dec.add_decrease_effect(level, 2)
    p.add_action(dec)
    p.add_goal(Equals(level, 1))
    r = solve(p)
    assert r.plan is not None, r.status
    assert_valid(p, r.plan)


def test_parallel_different_cells_exists_step():
    """Two actions writing DIFFERENT cells of the same array co-fire under an
    exists-step strategy; cells are independent scalars, so the plan validates."""
    em = _em()
    a = Fluent("a", ArrayType(2, IntType(0, 9)))
    p = Problem("parallel")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0, 0])
    sa = InstantaneousAction("sA")
    sa.add_effect(em.ArrayIndex(a(), Int(0)), 1)
    sb = InstantaneousAction("sB")
    sb.add_effect(em.ArrayIndex(a(), Int(1)), 2)
    p.add_action(sa)
    p.add_action(sb)
    p.add_goal(And(Equals(em.ArrayIndex(a(), Int(0)), 1),
                   Equals(em.ArrayIndex(a(), Int(1)), 2)))

    p_ar = _ar_compile(p)
    r = solve(p, strategy="els")   # exists-step, grounded encoder
    assert r.plan is not None, f"parallel: no plan ({r.status})"
    _assert_valid(p_ar, _map_plan(p_ar, r.plan), "parallel native (els)")


# ═══════════════════ ill-formed models → clean rejection ═════════════════════

def test_static_oob_goal_error():
    """A statically out-of-bounds goal access is a hard error."""
    def build():
        em = _em()
        a = Fluent("a", ArrayType(3, IntType(0, 9)))
        p = Problem("oobgoal")
        p.add_fluent(a, default_initial_value=0)
        p.set_initial_value(a, [1, 2, 3])
        nop = InstantaneousAction("nop")
        nop.add_effect(em.ArrayIndex(a(), Int(0)), 1)
        p.add_action(nop)
        p.add_goal(Equals(em.ArrayIndex(a(), Int(5)), 0))  # size 3 -> OOB
        return p
    _assert_rejected(build)


def test_undefined_position_goal_error():
    """A goal on an undefined position is a hard error."""
    def build():
        em = _em()
        a = Fluent("sparse_g", ArrayType(3, IntType(0, 9)), undefined_positions=[(1,)])
        p = Problem("undefgoal")
        p.add_fluent(a)
        p.set_initial_value(a, [1, 2, 3])
        nop = InstantaneousAction("nop")
        nop.add_effect(em.ArrayIndex(a(), Int(0)), 1)
        p.add_action(nop)
        p.add_goal(Equals(em.ArrayIndex(a(), Int(1)), 0))  # a[1] is undefined
        return p
    _assert_rejected(build)


def test_same_cell_conflicting_assignments_never_apply():
    """Two assignments of different values to one literal cell (a[0] and the
    folded a[1-1]): UP can never execute the action, so EffectNormalizationPass
    removes it (case 6), with a warning, and the goal is unreachable. (UP
    itself blocks syntactically identical targets, so the conflict is reached
    via constant folding.)"""
    def build():
        em = _em()
        a = Fluent("a", ArrayType(2, IntType(0, 9)))
        p = Problem("collconst")
        p.add_fluent(a, default_initial_value=0)
        p.set_initial_value(a, [0, 0])
        bad = InstantaneousAction("bad")
        bad.add_effect(em.ArrayIndex(a(), Int(0)), 1)
        bad.add_effect(em.ArrayIndex(a(), Minus(Int(1), Int(1))), 2)  # folds to a[0]
        p.add_action(bad)
        p.add_goal(Equals(em.ArrayIndex(a(), Int(0)), 1))
        return p
    _assert_rejected(build)


def test_whole_array_equality_rejected():
    """eq(a, b) between two whole array fluents (an L5 op) is rejected cleanly."""
    def build():
        em = _em()
        a = Fluent("a", ArrayType(2, IntType(0, 9)))
        b = Fluent("b", ArrayType(2, IntType(0, 9)))
        p = Problem("wholeeq")
        p.add_fluent(a, default_initial_value=0)
        p.add_fluent(b, default_initial_value=0)
        p.set_initial_value(a, [1, 2])
        p.set_initial_value(b, [1, 2])
        nop = InstantaneousAction("nop")
        nop.add_effect(em.ArrayIndex(a(), Int(0)), 1)
        p.add_action(nop)
        p.add_goal(Equals(a(), b()))  # whole-array equality
        return p
    _assert_rejected(build)


def test_all_instances_conflicting_never_apply():
    """The same conflict (a[0] and the folded a[1-1]) in an action that also
    has an unrelated int parameter: every instance assigns one cell two values,
    so EffectNormalizationPass removes every instance, with a warning naming
    them, and the goal is unreachable."""
    def build():
        em = _em()
        a = Fluent("allcoll_a", ArrayType(2, IntType(0, 9)))
        b = Fluent("allcoll_b", ArrayType(4, IntType(0, 9)))
        p = Problem("allcoll")
        p.add_fluent(a, default_initial_value=0)
        p.add_fluent(b, default_initial_value=0)
        p.set_initial_value(a, [0, 0])
        p.set_initial_value(b, [0, 0, 0, 0])
        bad = InstantaneousAction("bad", k=IntType(0, 3))
        bad.add_effect(em.ArrayIndex(a(), Int(0)), 1)
        bad.add_effect(em.ArrayIndex(a(), Minus(Int(1), Int(1))), 2)  # folds to a[0]
        bad.add_effect(em.ArrayIndex(b(), bad.parameter("k")), 3)
        p.add_action(bad)
        p.add_goal(Equals(em.ArrayIndex(a(), Int(0)), 1))
        return p
    _assert_rejected(build)


def test_oob_initial_state_assignment_dropped():
    """A per-cell initial-state assignment targeting an out-of-bounds/undefined
    position is permissively dropped, not a hard error (paper Def. 9:
    compile-time undefinedness is simply removed; the fork's permissive AR does
    the same, with a warning). The non-existent cell keeps no value and the rest
    of the problem still solves. This is what lets domains that unconditionally
    seed a corner cell (e.g. sokoban's `grid(P)[0][0]`) work on irregular grids
    where that corner is an undefined hole."""
    em = _em()
    a = Fluent("oobinit_a", ArrayType(2, IntType(0, 9)))
    p = Problem("oobinit")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(em.ArrayIndex(a(), Int(5)), 1)  # size 2 -> OOB, dropped
    w = InstantaneousAction("w")
    w.add_effect(em.ArrayIndex(a(), Int(0)), 1)
    p.add_action(w)
    p.add_goal(Equals(em.ArrayIndex(a(), Int(0)), 1))
    r = solve(p)
    assert r.status == PlanGenerationResultStatus.SOLVED_SATISFICING, r.status
    assert [ai.action.name for ai in r.plan.actions] == ["w"]


def test_whole_array_assignment_rejected():
    """A whole-array assignment effect (`a := b`, an L5 op) is rejected cleanly."""
    def build():
        em = _em()
        a = Fluent("wassign_a", ArrayType(2, IntType(0, 9)))
        b = Fluent("wassign_b", ArrayType(2, IntType(0, 9)))
        p = Problem("wassign")
        p.add_fluent(a, default_initial_value=0)
        p.add_fluent(b, default_initial_value=0)
        p.set_initial_value(a, [0, 0])
        p.set_initial_value(b, [1, 2])
        cpy = InstantaneousAction("copyall")
        cpy.add_effect(a(), b())
        p.add_action(cpy)
        p.add_goal(Equals(em.ArrayIndex(a(), Int(0)), 1))
        return p
    _assert_rejected(build)


def test_fluent_index_in_goal_rejected():
    """A state-dependent index in a *goal* (`a[f]` — goals are strict-⊥ and
    cannot host the per-position split, docs/specs/undefinedness-semantics.md)
    is rejected by the C++ TermResolutionPass with a rewrite hint. Action-side
    state-dependent indices now solve — covered in tests/api/nested_terms.py."""
    em = _em()
    a = Fluent("flidx_a", ArrayType(3, IntType(0, 9)))
    f = Fluent("flidx_f", IntType(0, 2))
    p = Problem("flidx")
    p.add_fluent(a, default_initial_value=0)
    p.add_fluent(f, default_initial_value=0)
    p.set_initial_value(a, [1, 2, 3])
    p.set_initial_value(f, 1)
    w = InstantaneousAction("w")
    w.add_effect(em.ArrayIndex(a(), Int(0)), 5)
    p.add_action(w)
    p.add_goal(Equals(em.ArrayIndex(a(), f()), Int(2)))  # fluent as index
    r = solve(p)
    assert r.plan is None, "a state-dependent goal index must not solve"
    assert r.status == PlanGenerationResultStatus.INTERNAL_ERROR, \
        f"expected the C++ rejection to surface as INTERNAL_ERROR, got {r.status}"


def test_up_grounding_flag_rejected():
    """`--up-grounding` disables the native grounder, where the array lowering
    lives; the preprocess() guard must reject array problems with that flag
    instead of leaking ARRAY_INDEX nodes into un-audited passes."""
    from rantanplan import _rantanplan
    from rantanplan.converter import convert_problem
    em = _em()
    a = Fluent("upg_a", ArrayType(2, IntType(0, 9)))
    p = Problem("upground")
    p.add_fluent(a, default_initial_value=0)
    p.set_initial_value(a, [0, 0])
    w = InstantaneousAction("w")
    w.add_effect(em.ArrayIndex(a(), Int(0)), 1)
    p.add_action(w)
    p.add_goal(Equals(em.ArrayIndex(a(), Int(0)), 1))
    ir = convert_problem(p)
    try:
        _rantanplan.preprocess(ir, ["--strategy", "seq", "--verbosity", "silent",
                                    "--up-grounding"])
    except Exception as e:
        assert "native grounding" in str(e), str(e)
        return
    raise AssertionError("preprocess accepted an array problem with --up-grounding")


# ═══════════════════════ mini end-to-end integration ═════════════════════════

def test_sliding_puzzle_2x2():
    """The paper's 15-Puzzle scaled to 2x2: grid : array[2, array[2, int]] with a
    single int-parameter `slide` move. Solved natively and replayed through a
    domain-specific simulator (UP cannot validate array problems, and int
    parameters make it non-AR-compilable)."""
    em = _em()
    grid = Fluent("grid", ArrayType(2, ArrayType(2, IntType(0, 3))))
    p = Problem("puzzle2x2")
    p.add_fluent(grid, default_initial_value=0)
    start = [[1, 2], [3, 0]]           # blank at (1,1)
    goal = [[0, 2], [1, 3]]
    p.set_initial_value(grid, start)

    def cell(r, c):
        return em.ArrayIndex(em.ArrayIndex(grid(), r), c)

    slide = InstantaneousAction("slide", fr=IntType(0, 1), fc=IntType(0, 1),
                                tr=IntType(0, 1), tc=IntType(0, 1))
    fr, fc, tr, tc = (slide.parameter(n) for n in ("fr", "fc", "tr", "tc"))
    slide.add_precondition(Equals(cell(tr, tc), 0))  # target is the blank
    slide.add_precondition(Or(                        # Manhattan-adjacent
        And(Equals(fr, tr), Equals(Plus(fc, 1), tc)),
        And(Equals(fr, tr), Equals(fc, Plus(tc, 1))),
        And(Equals(fc, tc), Equals(Plus(fr, 1), tr)),
        And(Equals(fc, tc), Equals(fr, Plus(tr, 1)))))
    slide.add_effect(cell(tr, tc), cell(fr, fc))      # tile moves into the blank
    slide.add_effect(cell(fr, fc), 0)                 # source becomes blank
    p.add_action(slide)
    p.add_goal(And(*[Equals(cell(r, c), goal[r][c]) for r in range(2) for c in range(2)]))

    r = solve(p)
    assert r.plan is not None, f"puzzle: no plan ({r.status})"

    # Replay the plan through a faithful simulator (checks preconditions + swap).
    state = [row[:] for row in start]
    for ai in r.plan.actions:
        assert ai.action.name == "slide"
        fr_, fc_, tr_, tc_ = (x.int_constant_value() for x in ai.actual_parameters)
        assert state[tr_][tc_] == 0, "slide target was not the blank"
        assert abs(fr_ - tr_) + abs(fc_ - tc_) == 1, "slide was not to an adjacent cell"
        state[tr_][tc_], state[fr_][fc_] = state[fr_][fc_], 0
    assert state == goal, f"replayed state {state} != goal {goal}"


CASES = [
    # base semantics (AR oracle)
    test_1d_int_read_write_goal,
    test_swap_one_action_reads_prestate,
    test_increase_decrease_cell,
    test_conditional_effect_on_cell,
    test_boolean_array_precond_and_cwa,
    test_multidim_2d,
    test_initial_state_both_shapes,
    test_default_initial_value_applies_to_cells,
    test_default_with_partial_explicit_override,
    test_boolean_array_cwa_without_default,
    test_boolean_cell_copy,
    test_boolean_cell_swap,
    test_size1_array_degenerates_to_scalar,
    # whole-array literals (AR oracle)
    test_whole_array_goal_equality_literal,
    test_whole_array_goal_disequality_literal,
    test_whole_array_assignment_effect_literal,
    # int-parameter / range-variable indices (scalar reference)
    test_index_arithmetic_int_param,
    test_forall_effect_range_variable_reverse,
    test_forall_effect_parameter_dependent_bound,
    test_forall_effect_arithmetic_range_bound,
    test_quantified_forall_goal_index,
    test_quantified_exists_goal_index,
    # edge cases
    test_oob_by_parameter_enumeration,
    test_undefined_positions_discard_and_neighbours,
    test_oob_precondition_read_discard,
    test_oob_effect_rhs_read_discard,
    test_guarded_oob_permissive_survives,
    test_guarded_undefined_effect_blocks_where_condition_holds,
    test_guarded_undefined_effect_never_applicable_when_condition_static,
    test_state_invariant_with_cell_read,
    test_action_cost_with_cell_read,
    test_same_cell_collision_instance_dependent,
    test_same_cell_increases_add_up,
    test_same_cell_set_and_clear,
    test_same_cell_exclusive_guards,
    test_same_cell_folded_increases_add_up,
    test_element_bounds_respected,
    test_non_array_regression,
    test_parallel_different_cells_exists_step,
    # ill-formed models → clean rejection
    test_static_oob_goal_error,
    test_undefined_position_goal_error,
    test_same_cell_conflicting_assignments_never_apply,
    test_whole_array_equality_rejected,
    test_all_instances_conflicting_never_apply,
    test_oob_initial_state_assignment_dropped,
    test_whole_array_assignment_rejected,
    test_fluent_index_in_goal_rejected,
    test_up_grounding_flag_rejected,
    # mini end-to-end
    test_sliding_puzzle_2x2,
]
