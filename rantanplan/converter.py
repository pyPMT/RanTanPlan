"""UP Problem -> ProblemIR converter (the Python side of the C++ contract).

Replaces the protobuf transport: a hand-rolled memoized DFS (see _FNodeToIR)
flattens every expression into one shared node pool
(children precede parents, shared subexpressions stored once), and the
problem structure references nodes by index. The C++ loader (`ir_io.cpp`)
interns the pool in a single forward pass.

The node shapes deliberately mirror what UP's own ``FNode2Protobuf``
produced, so the C++ loader stayed a port of the protobuf loader. The
operator/type naming helpers below are small local ports of
``unified_planning.grpc.proto_writer`` (`map_operator`, `proto_type`) --
copied rather than imported so this module does not pull in protobuf.
"""

from fractions import Fraction

from unified_planning import model
from unified_planning.exceptions import UPException
from unified_planning.model import InstantaneousAction
from unified_planning.model.int_variable import IntVariable
from unified_planning.model.metrics import (
    MinimizeActionCosts,
    MinimizeSequentialPlanLength,
)
from unified_planning.model.operators import OperatorKind

from . import _rantanplan as _ir

# Node kinds (ir.hpp's ExprKindIR, UP's ExpressionKind numbers)
_CONSTANT = _ir.ExprKind.CONSTANT
_PARAMETER = _ir.ExprKind.PARAMETER
_FLUENT_SYMBOL = _ir.ExprKind.FLUENT_SYMBOL
_FUNCTION_SYMBOL = _ir.ExprKind.FUNCTION_SYMBOL
_STATE_VARIABLE = _ir.ExprKind.STATE_VARIABLE
_FUNCTION_APPLICATION = _ir.ExprKind.FUNCTION_APPLICATION
_VARIABLE = _ir.ExprKind.VARIABLE

# The operators the planner supports: ir.hpp's OperatorIR, whose members are
# named after UP's OperatorKind.
_OPERATORS = {OperatorKind[name]: op for name, op in _ir.Operator.__members__.items()}


def _up_type_str(tpe) -> str:
    """Port of proto_writer.proto_type."""
    if tpe.is_bool_type():
        return "up:bool"
    if tpe.is_time_type():
        return "up:time"
    # Bounds live on FluentIR.lower/upper_bound; the type string stays the base
    # name so bounded types (e.g. integer[0, 15]) resolve like plain int/real.
    if tpe.is_int_type():
        return "up:integer"
    if tpe.is_real_type():
        return "up:real"
    if tpe.is_array_type():
        # Array type name, constructed from the nested structure (size +
        # element). Needed as the type-reference key for array-fluent
        # *expression* head symbols (a bare FLUENT_EXP inside ARRAY_INDEX); it
        # must match the name the C++ loader generates from the same structured
        # shape (ir_io.cpp array_type_name), and a mismatch surfaces as a hard
        # "could not find type" at load. The fluent *declaration* no longer
        # carries this string -- C++ derives the fluent's value type from
        # element_type + array_sizes.
        return f"array[{tpe.size}, {_up_type_str(tpe.elements_type)}]"
    if tpe.is_set_type():
        # Set type name from the element type. Needed as the type-reference key
        # for set-fluent *expression* head symbols (a bare FLUENT_EXP referring
        # to a set fluent); it must match the name the C++ loader generates
        # (ir_io.cpp set_type_name), and a mismatch surfaces as a hard "could not
        # find type" at load. The fluent *declaration* does not carry this string
        # -- C++ derives the value type from element_type + shape=SET.
        return f"set{{{_up_type_str(tpe.elements_type)}}}"
    return str(tpe.name)


def _array_shape(tpe):
    """(element_type_str, [per-dim sizes], leaf_type) for an ArrayType.

    Descends nested ArrayTypes: array[2, array[3, integer[0,5]]] ->
    ("up:integer", [2, 3], integer[0,5])."""
    sizes = []
    leaf = tpe
    while leaf.is_array_type():
        sizes.append(int(leaf.size))
        leaf = leaf.elements_type
    return _up_type_str(leaf), sizes, leaf


def _fraction(value) -> tuple:
    """A declared bound (an int, or a Fraction for a real) as an exact (numerator, denominator)."""
    value = Fraction(value)
    return (value.numerator, value.denominator)


class _FNodeToIR:
    """Flattens FNode DAGs into the shared node pool.

    A hand-rolled memoized DFS: children must precede parents in the pool
    (the C++ loader interns it in a single forward pass), and several node
    kinds need custom control flow a generic walker complicates
    (payload-carrying ARRAY_CONSTANT/SET_CONSTANT, range-bound folding,
    the literal-equality expansions).

    Head symbols (fluent/function names) are memoized separately so
    repeated applications share one head node.
    """

    def __init__(self):
        self.nodes = []
        self._memo = {}
        self._head_memo = {}

    def walk(self, expression):
        idx = self._memo.get(expression)
        if idx is None:
            idx = self._convert(expression)
            self._memo[expression] = idx
        return idx

    # ── node construction ────────────────────────────────────────────────

    def _add(self, kind, type_name="", op=None, symbol=None, int_value=None,
             real_value=None, bool_value=None, children=()):
        node = _ir.ExprNodeIR()
        node.kind = kind
        node.type_name = type_name
        if op is not None:
            node.op = op
        if symbol is not None:
            node.symbol = symbol
        if int_value is not None:
            node.int_value = int_value
        if real_value is not None:
            node.real_value = real_value
        if bool_value is not None:
            node.bool_value = bool_value
        if children:
            node.children = list(children)
        self.nodes.append(node)
        return len(self.nodes) - 1

    def _head(self, kind, symbol, type_name):
        key = (kind, symbol, type_name)
        idx = self._head_memo.get(key)
        if idx is None:
            idx = self._add(kind, type_name=type_name, symbol=symbol)
            self._head_memo[key] = idx
        return idx

    def _operator(self, op):
        """The FUNCTION_SYMBOL head of an application of `op`."""
        key = (_FUNCTION_SYMBOL, op)
        idx = self._head_memo.get(key)
        if idx is None:
            idx = self._add(_FUNCTION_SYMBOL, op=op)
            self._head_memo[key] = idx
        return idx

    # ── FNode conversion (shapes mirror UP's FNode2Protobuf) ─────────────

    def _convert(self, expression):
        if expression.is_bool_constant():
            return self._add(_CONSTANT, "up:bool",
                             bool_value=expression.bool_constant_value())
        if expression.is_int_constant():
            return self._add(_CONSTANT, "up:integer",
                             int_value=expression.int_constant_value())
        if expression.is_real_constant():
            value: Fraction = expression.real_constant_value()
            return self._add(_CONSTANT, "up:real",
                             real_value=(value.numerator, value.denominator))
        if expression.is_parameter_exp():
            param = expression.parameter()
            return self._add(_PARAMETER, _up_type_str(param.type), symbol=param.name)
        if expression.is_variable_exp():
            variable = expression.variable()
            return self._head(_VARIABLE, variable.name, _up_type_str(variable.type))
        if expression.is_int_variable_exp():
            # Reference to a numeric int variable in a quantifier body. Emit it
            # as an ordinary VARIABLE node (name + integer type) -- same shape as
            # an object-variable reference -- so the C++ QuantifierExpansionPass
            # substitutes it by name. The [initial, last] bounds travel on the
            # variable's *declaration* inside the enclosing forall/exists.
            rv = expression.int_variable()
            return self._head(_VARIABLE, rv.name, _up_type_str(rv.type))
        if expression.is_object_exp():
            obj = expression.object()
            return self._head(_CONSTANT, obj.name, _up_type_str(obj.type))
        if expression.is_fluent_exp():
            fluent_type = _up_type_str(expression.fluent().type)
            head = self._head(_FLUENT_SYMBOL, expression.fluent().name, fluent_type)
            children = [head] + [self.walk(arg) for arg in expression.args]
            return self._add(_STATE_VARIABLE, fluent_type, children=children)

        if expression.node_type == OperatorKind.ARRAY_CONSTANT:
            raise UPException(
                "Array constants outside the initial state are not supported yet.")
        # A state-dependent array index (`a[ptr]` — arg(1) reads a fluent) is
        # allowed: the C++ TermResolutionPass compiles it away by per-position
        # instance splitting before array lowering
        # (docs/specs/undefinedness-semantics.md, regime R2).

        if expression.node_type == OperatorKind.SET_CONSTANT:
            # A set literal `{c1..ck}` (empty for `{}`). Its elements live in the
            # payload (an ordered tuple) -- `args` is empty and `constant_value()`
            # is an *unordered* set -- so they are walked from the payload.
            # Emitted as an ordinary SET_CONSTANT application (element-constant
            # children, no payload), which the SetLoweringPass rewrites away.
            # Handled here rather than by the generic path below, which reads
            # `args`.
            head = self._operator(_ir.Operator.SET_CONSTANT)
            children = [head] + [self.walk(e) for e in expression._content.payload]
            return self._add(_FUNCTION_APPLICATION, "", children=children)

        # A fluent-valued element of member/add/remove (`member(loc_of(t), S)`)
        # is allowed: the C++ TermResolutionPass replaces it with a ghost
        # parameter (pinned by an equality precondition) before set lowering,
        # so the lowered cell coordinate is an ordinary parameter.

        # Whole-array equality against a literal (an ARRAY_CONSTANT operand,
        # e.g. n_puzzle's `Equals(puzzle, goal_blocks)` goal). Expand it
        # element-wise into a conjunction of scalar per-cell equalities, which
        # the constant-index array lowering already handles. Handled only when
        # exactly one side is the literal (the other is the array term to index).
        if expression.is_equals():
            a, b = expression.arg(0), expression.arg(1)
            a_arr, b_arr = a.is_array_constant(), b.is_array_constant()
            if a_arr != b_arr:
                em = expression.environment.expression_manager
                term, const = (b, a) if a_arr else (a, b)
                conjuncts = [em.Equals(t, v)
                             for t, v in _array_literal_cells(em, term, const)]
                return self.walk(em.And(*conjuncts))

        # Operator application (and/or/not/implies/iff, arithmetic, relations,
        # array_index)
        op = _OPERATORS.get(expression.node_type)
        if op is None:
            raise UPException(f"Unsupported operator: {expression.node_type}")
        head = self._operator(op)
        children = [head]
        # forall/exists: declared variables precede the arguments
        if expression.is_exists() or expression.is_forall():
            for variable in expression.variables():
                children.append(self._variable_declaration(variable))
        children.extend(self.walk(arg) for arg in expression.args)
        return self._add(_FUNCTION_APPLICATION, "", children=children)

    def _variable_declaration(self, variable):
        """A quantifier's bound-variable declaration. Object variables are leaf
        VARIABLE nodes; a numeric IntVariable additionally carries its
        [initial, last] bounds as two child nodes (an int constant or a
        parameter reference), so C++ grounding substitutes any parameter in a
        bound before the QuantifierExpansionPass reads it."""
        if isinstance(variable, IntVariable):
            lo = self._bound_node(variable._initial)
            hi = self._bound_node(variable._last)
            return self._add(_VARIABLE, _up_type_str(variable.type),
                             symbol=variable.name, children=[lo, hi])
        return self._head(_VARIABLE, variable.name, _up_type_str(variable.type))

    def _bound_node(self, bound):
        """An int-variable bound (the raw IntVariable._initial / _last value):
        an int constant, an int parameter, or an arithmetic expression over
        them (e.g. `lc - 1`)."""
        if isinstance(bound, bool):
            raise UPException(f"Unexpected boolean range bound: {bound}")
        if isinstance(bound, int):
            return self._add(_CONSTANT, "up:integer", int_value=bound)
        if isinstance(bound, model.Parameter):
            return self._add(_PARAMETER, _up_type_str(bound.type), symbol=bound.name)
        # An arithmetic bound over parameters/constants arrives as an FNode.
        # Walk it into an expression tree; C++ int-parameter instantiation
        # substitutes the parameters and eval_bound folds it to a constant.
        return self.walk(bound)


def _convert_parameters(parameters):
    result = []
    for param in parameters:
        if param.type.is_set_type():
            raise UPException(
                f"Set-typed parameter '{param.name}' is not supported "
                f"(a set-typed action/fluent parameter would enumerate 2^|D| "
                f"instances).")
        ir_param = _ir.ParameterIR()
        ir_param.name = param.name
        ir_param.type_name = _up_type_str(param.type)
        # Bounded-int parameters carry their [lower, upper] interval so the C++
        # grounder can enumerate it (the type string stays "up:integer"). Real
        # and unbounded-int parameters are unsupported and rejected upstream.
        ptype = param.type
        if ptype.is_int_type():
            if ptype.lower_bound is not None:
                ir_param.lower_bound = int(ptype.lower_bound)
            if ptype.upper_bound is not None:
                ir_param.upper_bound = int(ptype.upper_bound)
        result.append(ir_param)
    return result


def _array_literal_cells(expr_manager, term, array_const):
    """Yield (indexed_term, scalar_element) for every leaf of an array constant.

    Pairs each leaf of a (possibly nested) ARRAY_CONSTANT with the corresponding
    constant-index ARRAY_INDEX access into `term`, in row-major order. Used to
    expand a whole-array literal appearing in a goal/precondition equality or an
    assignment effect into per-cell scalar operations (the same shape the C++
    array lowering already handles). Unlike `_emit_array_cells` there is no
    undefined-position handling: a literal in an expression is fully defined."""
    for i, element in enumerate(array_const.constant_value()):
        cell_term = expr_manager.ArrayIndex(term, expr_manager.Int(i))
        if element.is_array_constant():
            yield from _array_literal_cells(expr_manager, cell_term, element)
        else:
            yield cell_term, element


def _emit_array_cells(walker, expr_manager, array_expr, array_const,
                      prefix, undefined, out):
    """Normalise a whole-array initial value into one per-cell AssignmentIR.

    Recurses through a (possibly nested) ARRAY_CONSTANT; for each defined leaf
    cell it emits an assignment whose fluent target is ARRAY_INDEX(...) built
    with constant indices (walked through the ordinary operator path, so the
    up:array_index head is interned like any other) and whose value is the
    scalar element. Positions in `undefined` (full coordinate tuples) are
    skipped -- their cells are never assigned."""
    for i, element in enumerate(array_const.constant_value()):
        coord = prefix + (i,)
        if element.is_array_constant():
            _emit_array_cells(walker, expr_manager, array_expr, element,
                              coord, undefined, out)
            continue
        if coord in undefined:
            continue
        index_expr = array_expr
        for c in coord:
            index_expr = expr_manager.ArrayIndex(index_expr, expr_manager.Int(c))
        ir_assignment = _ir.AssignmentIR()
        ir_assignment.fluent = walker.walk(index_expr)
        ir_assignment.value = walker.walk(element)
        out.append(ir_assignment)


def convert_problem(problem: model.Problem) -> "_ir.ProblemIR":
    """Convert a UP Problem into the ProblemIR the C++ loader consumes.

    Walk order matches the C++ loader's interning order (actions, then
    initial state, goals, metric costs), so the resulting ExprPool is
    identical to the one the protobuf path produced.
    """
    walker = _FNodeToIR()
    out = _ir.ProblemIR()

    # --- Types ---
    types = []
    for tpe in problem.user_types:
        ir_type = _ir.TypeIR()
        ir_type.name = str(tpe.name)
        ir_type.parent = "" if tpe.father is None else _up_type_str(tpe.father)
        types.append(ir_type)
    out.types = types

    # --- Fluents ---
    fluents = []
    for fluent in problem.fluents:
        ir_fluent = _ir.FluentIR()
        ir_fluent.name = fluent.name
        ir_fluent.parameters = _convert_parameters(fluent.signature)
        ftype = fluent.type
        if ftype.is_array_type():
            # An array fluent sends only its structured shape; the C++ loader
            # generates the canonical array type name from element_type +
            # array_sizes (the single renderer -- ir_io.cpp). value_type is left
            # empty, so there is no second, hand-rendered string to keep in sync.
            element_type, sizes, leaf = _array_shape(ftype)
            if leaf.is_set_type():
                raise UPException(
                    f"Array fluent '{fluent.name}' has set-typed elements; "
                    f"set/array mixing is not supported.")
            ir_fluent.shape = _ir.FluentShape.ARRAY
            ir_fluent.element_type = element_type
            ir_fluent.array_sizes = sizes
            # Element bounds ride on lower/upper_bound (same convention as scalar
            # bounded fluents); the grounder mints cells of the element type.
            if leaf.is_int_type() or leaf.is_real_type():
                if leaf.lower_bound is not None:
                    ir_fluent.lower_bound = _fraction(leaf.lower_bound)
                if leaf.upper_bound is not None:
                    ir_fluent.upper_bound = _fraction(leaf.upper_bound)
            if fluent.undefined_positions:
                ir_fluent.undefined_positions = [
                    [int(c) for c in pos] for pos in fluent.undefined_positions]
        elif ftype.is_set_type():
            # A set fluent sends only its structured shape (shape=SET + element
            # type); the C++ loader generates the `set{T}` value type and the
            # SetLoweringPass rewrites the fluent into boolean membership cells.
            # value_type is left empty (no second hand-rendered string to sync).
            elem = ftype.elements_type
            if not elem.is_user_type():
                raise UPException(
                    f"Set fluent '{fluent.name}' has element type '{elem}': only "
                    f"user-type set elements are supported (int elements, nested "
                    f"set types, and set/array mixing are out of scope for v1).")
            ir_fluent.shape = _ir.FluentShape.SET
            ir_fluent.element_type = _up_type_str(elem)
        else:
            ir_fluent.value_type = _up_type_str(ftype)
            if ftype.is_int_type() or ftype.is_real_type():
                if ftype.lower_bound is not None:
                    ir_fluent.lower_bound = _fraction(ftype.lower_bound)
                if ftype.upper_bound is not None:
                    ir_fluent.upper_bound = _fraction(ftype.upper_bound)
        fluents.append(ir_fluent)
    # out.fluents assigned last (with out.actions/out.nodes): nanobind copies
    # vector fields on assignment, and the declared-defaults loop below still
    # mutates these FluentIR entries.

    # --- Objects ---
    objects = []
    for obj in problem.all_objects:
        ir_object = _ir.ObjectIR()
        ir_object.name = obj.name
        ir_object.type_name = _up_type_str(obj.type)
        objects.append(ir_object)
    out.objects = objects

    # --- Actions ---
    expr_manager = problem.environment.expression_manager
    actions = []
    for action in problem.actions:
        if not isinstance(action, InstantaneousAction):
            raise UPException(
                f"Unsupported action type: {type(action).__name__} "
                f"(only instantaneous actions are supported)")
        ir_action = _ir.ActionIR()
        ir_action.name = action.name
        ir_action.parameters = _convert_parameters(action.parameters)
        # UP's own conjunction (true for none, the condition itself for one),
        # so it is the same node as any other And of these conditions.
        ir_action.precondition = walker.walk(expr_manager.And(action.preconditions))

        effects = []
        for effect in action.effects:
            if effect.is_assignment():
                kind = _ir.EffectKind.ASSIGN
            elif effect.is_increase():
                kind = _ir.EffectKind.INCREASE
            elif effect.is_decrease():
                kind = _ir.EffectKind.DECREASE
            else:
                raise UPException(f"Unsupported effect: {effect}")

            # Emit each forall variable as a *declaration* (an IntVariable carries
            # its [initial, last] bounds as children) -- the same node a quantifier
            # condition uses -- so the QuantifierExpansionPass enumerates a range
            # variable's integer interval. A bare walk() reference would drop the
            # bounds, leaving C++ to misread the range variable as an (object-less)
            # object variable and silently expand the effect to nothing.
            forall = [walker._variable_declaration(v) for v in effect.forall]
            condition = walker.walk(effect.condition)

            # Whole-array assignment from a literal (e.g. `puzzle := goal_blocks`):
            # expand into one per-cell effect, sharing kind/condition/forall. The
            # per-cell target is a constant-index access the array lowering
            # handles like any scalar cell.
            if effect.value.is_array_constant():
                for cell_fluent, cell_value in _array_literal_cells(
                        expr_manager, effect.fluent, effect.value):
                    ir_effect = _ir.EffectIR()
                    ir_effect.kind = kind
                    ir_effect.fluent = walker.walk(cell_fluent)
                    ir_effect.value = walker.walk(cell_value)
                    ir_effect.condition = condition
                    ir_effect.forall = forall
                    effects.append(ir_effect)
                continue

            ir_effect = _ir.EffectIR()
            ir_effect.kind = kind
            ir_effect.fluent = walker.walk(effect.fluent)
            ir_effect.value = walker.walk(effect.value)
            ir_effect.condition = condition
            ir_effect.forall = forall
            effects.append(ir_effect)
        ir_action.effects = effects
        actions.append(ir_action)

    # --- Initial state ---
    # Two shapes (see docs/native-arrays-plan.md): a whole-array ARRAY_CONSTANT
    # value is normalised to one per-cell assignment per defined cell (avoids an
    # ARRAY_CONSTANT node kind in the ExprPool); everything else (per-cell
    # ARRAY_INDEX keys and scalar assignments) passes through unchanged.
    initial_state = []
    for state_var, value in problem.explicit_initial_values.items():
        if value.is_array_constant():
            undefined = set()
            if state_var.is_fluent_exp():
                positions = state_var.fluent().undefined_positions
                if positions:
                    undefined = {tuple(p) for p in positions}
            _emit_array_cells(walker, expr_manager, state_var, value, (),
                              undefined, initial_state)
        else:
            ir_assignment = _ir.AssignmentIR()
            ir_assignment.fluent = walker.walk(state_var)
            ir_assignment.value = walker.walk(value)
            initial_state.append(ir_assignment)
    out.initial_state = initial_state

    # --- Goals ---
    out.goals = [walker.walk(goal) for goal in problem.goals]

    # --- Declared state invariants: bodies of `(always f)` constraints ---
    # (The planner declares STATE_INVARIANTS support only, so UP rejects
    # other trajectory-constraint kinds upstream; this check is defensive.)
    state_invariants = []
    for tc in problem.trajectory_constraints:
        if not tc.is_always():
            raise UPException(
                f"Unsupported trajectory constraint (only 'always' state "
                f"invariants are supported): {tc}")
        state_invariants.append(walker.walk(tc.arg(0)))
    out.state_invariants = state_invariants

    # --- Metric (first supported one wins, like the protobuf loader) ---
    metric_kind = _ir.MetricKind.NONE
    for metric in problem.quality_metrics:
        if isinstance(metric, MinimizeActionCosts):
            metric_kind = _ir.MetricKind.MINIMIZE_ACTION_COSTS
            costs_by_name = {a.name: cost for a, cost in metric.costs.items()}
            for action, ir_action in zip(problem.actions, actions):
                cost = costs_by_name.get(action.name, metric.default)
                if cost is not None:
                    ir_action.cost = walker.walk(cost)
            break
        if isinstance(metric, MinimizeSequentialPlanLength):
            metric_kind = _ir.MetricKind.MINIMIZE_PLAN_LENGTH
            break
    out.metric_kind = metric_kind

    # --- Declared fluent defaults (UP fluents_defaults) ---
    # Walked last, after every other expression: a re-walked constant dedups
    # to its existing id, a genuinely new one appends. A
    # declared default is authoritative for instances/cells left uninitialised
    # (the C++ CWA pass materialises it). UP stores an array's default as one
    # scalar, so one value covers every cell. Only numeric and boolean-false
    # defaults are carried: a boolean-true default would invert the closed-world
    # assumption unsoundly (the grounder assumes false), and object fluents must
    # be initialised explicitly (the CWA pass errors otherwise).
    for fluent, ir_fluent in zip(problem.fluents, fluents):
        default = problem.fluents_defaults.get(fluent)
        if default is None:
            continue
        # _array_shape returns leaf == fluent.type for a scalar fluent, so this
        # covers scalar and array defaults with one descent.
        _, _, leaf = _array_shape(fluent.type)
        if leaf.is_bool_type():
            if default.is_bool_constant() and default.bool_constant_value():
                raise UPException(
                    f"Boolean fluent '{fluent.name}' declares "
                    f"default_initial_value=True, which is not supported "
                    f"(it inverts the closed-world assumption).")
            ir_fluent.default_value = walker.walk(default)
        elif leaf.is_int_type() or leaf.is_real_type():
            ir_fluent.default_value = walker.walk(default)
        elif leaf.is_user_type():
            # Object (UserType) fluents: carry the declared default so the C++
            # CWA pass materialises it for uninitialised instances/cells. Unlike
            # a boolean-true default there is no closed-world assumption to
            # invert -- an object fluent must hold some object, and the default
            # supplies it for positions the initial state leaves unset.
            ir_fluent.default_value = walker.walk(default)
        elif leaf.is_set_type():
            # An empty-set default is already the closed-world default (every
            # membership cell false), so it needs no materialisation and carries
            # no default_value. A non-empty set default would need per-cell trues
            # and is rejected in v1 (mirrors the boolean-true rejection).
            if default.node_type == OperatorKind.SET_CONSTANT and \
                    len(default._content.payload) > 0:
                raise UPException(
                    f"Set fluent '{fluent.name}' declares a non-empty "
                    f"default_initial_value, which is not supported (only the "
                    f"empty-set default, i.e. the closed-world default, is).")

    # Vector fields copy on assignment, so these land after all mutation
    out.fluents = fluents
    out.actions = actions
    out.nodes = walker.nodes
    return out
