"""UP Problem -> ProblemIR converter (the Python side of the C++ contract).

Replaces the protobuf transport: a hand-rolled memoized DFS (see _FNodeToIR
for why not a DagWalker) flattens every expression into one shared node pool
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

# unified_planning ExpressionKind values (must match ir.hpp's ExprKindIR)
_CONSTANT = 1
_PARAMETER = 2
_FLUENT_SYMBOL = 3
_FUNCTION_SYMBOL = 4
_STATE_VARIABLE = 5
_FUNCTION_APPLICATION = 6
_VARIABLE = 7

_OPERATOR_NAMES = {
    OperatorKind.PLUS: "up:plus",
    OperatorKind.MINUS: "up:minus",
    OperatorKind.TIMES: "up:times",
    OperatorKind.DIV: "up:div",
    OperatorKind.LE: "up:le",
    OperatorKind.LT: "up:lt",
    OperatorKind.EQUALS: "up:equals",
    OperatorKind.AND: "up:and",
    OperatorKind.OR: "up:or",
    OperatorKind.NOT: "up:not",
    OperatorKind.IMPLIES: "up:implies",
    OperatorKind.IFF: "up:iff",
    OperatorKind.COUNT: "up:count",
    OperatorKind.EXISTS: "up:exists",
    OperatorKind.FORALL: "up:forall",
    OperatorKind.ARRAY_INDEX: "up:array_index",
}


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
        # Mirror UP's array[size, elements_type] form, but with the element type
        # rendered through _up_type_str so bounds stay off the string (they ride
        # on FluentIR.lower/upper_bound, like scalar bounded types). Nested for
        # multi-dim: array[2, array[3, up:integer]].
        return f"array[{tpe.size}, {_up_type_str(tpe.elements_type)}]"
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


def _index_contains_fluent(node) -> bool:
    """True if an array-index expression reads a fluent (state variable).

    Scope guard (docs/native-arrays-plan.md): array indices must be built from
    constants, parameters, quantified variables, and arithmetic over them --
    never fluents. `a[f]` / `a[b[i]]` would make the index state-dependent."""
    if node.is_fluent_exp():
        return True
    return any(_index_contains_fluent(arg) for arg in node.args)


class _FNodeToIR:
    """Flattens FNode DAGs into the shared node pool.

    A hand-rolled memoized DFS rather than a DagWalker: the node creation
    order must match the C++ ExprPool's insertion order exactly (head
    symbol first, then arguments, then the application — the recursion
    order of the old protobuf loader), and DagWalker visits arguments
    before the parent handler runs, which permutes head symbols.

    Head symbols (fluent/function names) are memoized separately so
    repeated applications share one head node.
    """

    def __init__(self):
        self.nodes = []
        self._memo = {}
        self._head_memo = {}
        self._true_idx = None

    def walk(self, expression):
        idx = self._memo.get(expression)
        if idx is None:
            idx = self._convert(expression)
            self._memo[expression] = idx
        return idx

    # ── node construction ────────────────────────────────────────────────

    def _add(self, kind, type_name="", symbol=None, int_value=None,
             real_value=None, bool_value=None, children=()):
        node = _ir.ExprNodeIR()
        node.kind = kind
        node.type_name = type_name
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

    def true_constant(self):
        """The synthesized `true` used for empty preconditions (same shape
        the protobuf loader synthesized)."""
        if self._true_idx is None:
            self._true_idx = self._add(_CONSTANT, "up:bool", bool_value=True)
        return self._true_idx

    def precondition(self, conditions):
        """Synthesize the precondition conjunction exactly like the protobuf
        loader's intern_precondition did (note: plain "and", type up:bool).
        Takes unwalked FNodes: the "and" head node must be created *before*
        the condition subtrees to preserve the pool's insertion order."""
        if not conditions:
            return self.true_constant()
        if len(conditions) == 1:
            return self.walk(conditions[0])
        head = self._head(_FUNCTION_SYMBOL, "and", "up:bool")
        children = [head] + [self.walk(cond) for cond in conditions]
        return self._add(_FUNCTION_APPLICATION, "up:bool", children=children)

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
        if expression.is_array_index():
            # arg(0) is the array (a fluent / nested access); arg(1) is the index,
            # which must not depend on state (see _index_contains_fluent).
            index_expr = expression.arg(1)
            if _index_contains_fluent(index_expr):
                raise UPException(
                    f"State-dependent array indices are not supported: index "
                    f"'{index_expr}' reads a fluent. Array indices may only use "
                    f"constants, parameters, quantified variables, and "
                    f"arithmetic over them.")

        # Operator application (and/or/not/implies/iff, arithmetic, relations,
        # array_index)
        operator_name = _OPERATOR_NAMES.get(expression.node_type)
        if operator_name is None:
            raise UPException(f"Unsupported operator: {expression.node_type}")
        head = self._head(_FUNCTION_SYMBOL, operator_name, "up:operator")
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
        """An int-variable bound: an int constant or a reference to an int
        parameter (the raw IntVariable._initial / _last value)."""
        if isinstance(bound, bool):
            raise UPException(f"Unexpected boolean range bound: {bound}")
        if isinstance(bound, int):
            return self._add(_CONSTANT, "up:integer", int_value=bound)
        return self._add(_PARAMETER, _up_type_str(bound.type), symbol=bound.name)


def _convert_parameters(parameters):
    result = []
    for param in parameters:
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
        ir_fluent.value_type = _up_type_str(fluent.type)
        ir_fluent.parameters = _convert_parameters(fluent.signature)
        ftype = fluent.type
        if ftype.is_int_type() or ftype.is_real_type():
            if ftype.lower_bound is not None:
                ir_fluent.lower_bound = float(ftype.lower_bound)
            if ftype.upper_bound is not None:
                ir_fluent.upper_bound = float(ftype.upper_bound)
        elif ftype.is_array_type():
            element_type, sizes, leaf = _array_shape(ftype)
            ir_fluent.element_type = element_type
            ir_fluent.array_sizes = sizes
            # Element bounds ride on lower/upper_bound (same convention as scalar
            # bounded fluents); the grounder mints cells of the element type.
            if leaf.is_int_type() or leaf.is_real_type():
                if leaf.lower_bound is not None:
                    ir_fluent.lower_bound = float(leaf.lower_bound)
                if leaf.upper_bound is not None:
                    ir_fluent.upper_bound = float(leaf.upper_bound)
            if fluent.undefined_positions:
                ir_fluent.undefined_positions = [
                    [int(c) for c in pos] for pos in fluent.undefined_positions]
        fluents.append(ir_fluent)
    out.fluents = fluents

    # --- Objects ---
    objects = []
    for obj in problem.all_objects:
        ir_object = _ir.ObjectIR()
        ir_object.name = obj.name
        ir_object.type_name = _up_type_str(obj.type)
        objects.append(ir_object)
    out.objects = objects

    # --- Actions ---
    actions = []
    for action in problem.actions:
        if not isinstance(action, InstantaneousAction):
            raise UPException(
                f"Unsupported action type: {type(action).__name__} "
                f"(only instantaneous actions are supported)")
        ir_action = _ir.ActionIR()
        ir_action.name = action.name
        ir_action.parameters = _convert_parameters(action.parameters)
        ir_action.precondition = walker.precondition(action.preconditions)

        effects = []
        for effect in action.effects:
            ir_effect = _ir.EffectIR()
            if effect.is_assignment():
                ir_effect.kind = 0
            elif effect.is_increase():
                ir_effect.kind = 1
            elif effect.is_decrease():
                ir_effect.kind = 2
            else:
                raise UPException(f"Unsupported effect: {effect}")
            ir_effect.fluent = walker.walk(effect.fluent)
            ir_effect.value = walker.walk(effect.value)
            ir_effect.condition = walker.walk(effect.condition)
            auto_promote = effect.environment.expression_manager.auto_promote
            ir_effect.forall = [walker.walk(v) for v in auto_promote(effect.forall)]
            effects.append(ir_effect)
        ir_action.effects = effects
        actions.append(ir_action)

    # --- Initial state ---
    # Two shapes (see docs/native-arrays-plan.md): a whole-array ARRAY_CONSTANT
    # value is normalised to one per-cell assignment per defined cell (avoids an
    # ARRAY_CONSTANT node kind in the ExprPool); everything else (per-cell
    # ARRAY_INDEX keys and scalar assignments) passes through unchanged.
    expr_manager = problem.environment.expression_manager
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
    metric_kind = 0
    for metric in problem.quality_metrics:
        if isinstance(metric, MinimizeActionCosts):
            metric_kind = 1
            costs_by_name = {a.name: cost for a, cost in metric.costs.items()}
            for action, ir_action in zip(problem.actions, actions):
                cost = costs_by_name.get(action.name, metric.default)
                if cost is not None:
                    ir_action.cost = walker.walk(cost)
            break
        if isinstance(metric, MinimizeSequentialPlanLength):
            metric_kind = 2
            break
    out.metric_kind = metric_kind

    # Vector fields copy on assignment, so these land after all mutation
    out.actions = actions
    out.nodes = walker.nodes
    return out
