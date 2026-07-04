"""UP Problem -> ProblemIR converter (the Python side of the C++ contract).

Replaces the protobuf transport: a DagWalker flattens every expression into
one shared node pool (children precede parents, shared subexpressions stored
once), and the problem structure references nodes by index. The C++ loader
(`ir_io.cpp`) interns the pool in a single forward pass.

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
    OperatorKind.EXISTS: "up:exists",
    OperatorKind.FORALL: "up:forall",
}


def _up_type_str(tpe) -> str:
    """Port of proto_writer.proto_type."""
    if tpe.is_bool_type():
        return "up:bool"
    if tpe.is_time_type():
        return "up:time"
    if tpe.is_int_type() or tpe.is_real_type():
        return f"up:{tpe}"
    return str(tpe.name)


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
        if expression.is_object_exp():
            obj = expression.object()
            return self._head(_CONSTANT, obj.name, _up_type_str(obj.type))
        if expression.is_fluent_exp():
            fluent_type = _up_type_str(expression.fluent().type)
            head = self._head(_FLUENT_SYMBOL, expression.fluent().name, fluent_type)
            children = [head] + [self.walk(arg) for arg in expression.args]
            return self._add(_STATE_VARIABLE, fluent_type, children=children)

        # Operator application (and/or/not/implies/iff, arithmetic, relations)
        operator_name = _OPERATOR_NAMES.get(expression.node_type)
        if operator_name is None:
            raise UPException(f"Unsupported operator: {expression.node_type}")
        head = self._head(_FUNCTION_SYMBOL, operator_name, "up:operator")
        children = [head]
        # forall/exists: declared variables precede the arguments
        if expression.is_exists() or expression.is_forall():
            for variable in expression.variables():
                children.append(self._head(_VARIABLE, variable.name,
                                           _up_type_str(variable.type)))
        children.extend(self.walk(arg) for arg in expression.args)
        return self._add(_FUNCTION_APPLICATION, "", children=children)


def _convert_parameters(parameters):
    result = []
    for param in parameters:
        ir_param = _ir.ParameterIR()
        ir_param.name = param.name
        ir_param.type_name = _up_type_str(param.type)
        result.append(ir_param)
    return result


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
    initial_state = []
    for state_var, value in problem.explicit_initial_values.items():
        ir_assignment = _ir.AssignmentIR()
        ir_assignment.fluent = walker.walk(state_var)
        ir_assignment.value = walker.walk(value)
        initial_state.append(ir_assignment)
    out.initial_state = initial_state

    # --- Goals ---
    out.goals = [walker.walk(goal) for goal in problem.goals]

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
