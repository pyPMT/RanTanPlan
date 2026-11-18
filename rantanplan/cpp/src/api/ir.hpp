#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

/**
 * @file ir.hpp
 * @brief Plain-data IR: the Python <-> C++ problem/result contract.
 *
 * Produced by rantanplan/converter.py and consumed by
 * load_problem_from_ir (ir_io.cpp). The expression pool is one flat node
 * array with index children; children always precede their parents, so the
 * loader interns the array in a single forward pass.
 *
 * Node `kind` values are UP's ExpressionKind numbers, and an application's
 * head symbol (its operator or fluent) is its first child, as in UP.
 *
 * Every enumerated field is one of the enums below, which bindings.cpp
 * exports to Python, so the numbers are written only here. load_problem_from_ir
 * maps them onto the planner's own enums.
 */
namespace rantanplan::ir {

/// unified_planning expression kinds (subset the loader understands; UNKNOWN
/// is an unset node, which the loader rejects)
enum class ExprKindIR : int {
    UNKNOWN = 0,
    CONSTANT = 1,
    PARAMETER = 2,
    FLUENT_SYMBOL = 3,
    FUNCTION_SYMBOL = 4,
    STATE_VARIABLE = 5,
    FUNCTION_APPLICATION = 6,
    VARIABLE = 7,
};

/// The UP operators the converter sends, under their OperatorKind names (the
/// converter maps each OperatorKind to the member of the same name).
enum class OperatorIR : int {
    PLUS, MINUS, TIMES, DIV,
    LE, LT, EQUALS,
    AND, OR, NOT, IMPLIES, IFF,
    COUNT, EXISTS, FORALL, ARRAY_INDEX,
    SET_MEMBER, SET_SUBSETEQ, SET_DISJOINT, SET_CARDINALITY,
    SET_ADD, SET_REMOVE, SET_UNION, SET_INTERSECT, SET_DIFFERENCE, SET_CONSTANT,
};

struct ExprNodeIR {
    ExprKindIR kind = ExprKindIR::UNKNOWN;
    /// UP type string ("up:bool", "up:integer", user type name) or ""
    std::string type_name;
    // Payload: at most one is set (atoms only; applications have children).
    // A FUNCTION_SYMBOL has `op`; other named atoms have `symbol`.
    std::optional<OperatorIR> op;
    std::optional<std::string> symbol;
    std::optional<int64_t> int_value;
    std::optional<std::pair<int64_t, int64_t>> real_value;  ///< numerator/denominator
    std::optional<bool> bool_value;
    /// Indices into ProblemIR::nodes; always lower than this node's index
    std::vector<uint32_t> children;
};

struct TypeIR {
    std::string name;
    std::string parent;  ///< "" for root types
};

struct ObjectIR {
    std::string name;
    std::string type_name;
};

struct ParameterIR {
    std::string name;
    std::string type_name;
    /// Declared integer bounds for a bounded-int action parameter (e.g.
    /// `amount: integer[1, 3]`); unset for object/bool/real parameters and for
    /// unbounded ints. Grounding enumerates the closed interval [lower, upper].
    std::optional<int64_t> lower_bound;
    std::optional<int64_t> upper_bound;
};

/// A fluent's value shape — the single positive discriminator the loader
/// dispatches on (mirrors the C++ `TypeKind`). SCALAR uses `value_type`; ARRAY
/// uses `element_type` + `array_sizes`; SET uses `element_type`.
enum class FluentShapeIR : int { SCALAR = 0, ARRAY = 1, SET = 2 };

struct FluentIR {
    std::string name;
    std::string value_type;
    std::vector<ParameterIR> parameters;
    /// Declared numeric bounds for bounded int/real value types (e.g.
    /// `integer[0, 15]`); unset for unbounded numeric or non-numeric fluents.
    /// For array fluents these carry the *element* type's bounds (the grounder
    /// mints cells of that element type), same as scalar bounded fluents.
    /// Exact, as numerator/denominator (like ExprNodeIR::real_value).
    std::optional<std::pair<int64_t, int64_t>> lower_bound;
    std::optional<std::pair<int64_t, int64_t>> upper_bound;
    /// Structured-value data, interpreted per `shape`. For an ARRAY fluent (from
    /// an ArrayType value, e.g. `array[3, integer[0, 15]]`): `element_type` is
    /// the leaf scalar type string ("up:integer", …), `array_sizes` the
    /// per-dimension size list, `undefined_positions` full coordinate tuples the
    /// grounder must never mint as cells. For a SET fluent (from a SetType
    /// value): `element_type` is the set's element type and `array_sizes` is
    /// empty. For a SCALAR fluent both are empty and `value_type` is used.
    std::string element_type;
    std::vector<int64_t> array_sizes;
    std::vector<std::vector<int64_t>> undefined_positions;
    /// The value shape (scalar / array / set). ARRAY and SET are mutually
    /// exclusive (set-of-array / array-of-set are rejected at conversion); the
    /// loader resolves the value type as `set{element_type}` for SET, and the
    /// SetLoweringPass rewrites set fluents into boolean membership cells before
    /// grounding.
    FluentShapeIR shape = FluentShapeIR::SCALAR;
    /// Declared default_initial_value (UP `fluents_defaults`), as an index into
    /// ProblemIR::nodes pointing to a CONSTANT node; unset when the fluent
    /// declares no default. The CWA pass treats it as authoritative for cells
    /// missing an explicit initial value (else it falls back to clamped-0 /
    /// false). Array fluents carry one scalar default (UP stores array defaults
    /// as scalars); the lowering stamps it onto every cell. Only numeric and
    /// boolean-`false` defaults are carried — a boolean-`true` default is
    /// rejected at conversion, and object defaults are ignored (object fluents
    /// require explicit initialization).
    std::optional<uint32_t> default_value;
};

enum class EffectKindIR : int { ASSIGN = 0, INCREASE = 1, DECREASE = 2 };

struct EffectIR {
    EffectKindIR kind = EffectKindIR::ASSIGN;
    uint32_t fluent = 0;
    uint32_t value = 0;
    std::optional<uint32_t> condition;
    std::vector<uint32_t> forall;
};

struct ActionIR {
    std::string name;
    std::vector<ParameterIR> parameters;
    /// The conjunction of the action's preconditions, as UP's And builds it
    /// (the true constant for none, the condition itself for one)
    uint32_t precondition = 0;
    std::vector<EffectIR> effects;
    /// Metric cost, already resolved per action (explicit or metric default);
    /// unset means unit cost
    std::optional<uint32_t> cost;
};

struct AssignmentIR {
    uint32_t fluent = 0;
    uint32_t value = 0;
};

enum class MetricKindIR : int {
    NONE = 0,
    MINIMIZE_ACTION_COSTS = 1,
    MINIMIZE_PLAN_LENGTH = 2,
};

struct ProblemIR {
    std::vector<ExprNodeIR> nodes;
    std::vector<TypeIR> types;
    std::vector<ObjectIR> objects;
    std::vector<FluentIR> fluents;
    std::vector<ActionIR> actions;
    std::vector<AssignmentIR> initial_state;
    std::vector<uint32_t> goals;
    std::vector<uint32_t> state_invariants;  ///< bodies of `(always f)` constraints
    MetricKindIR metric_kind = MetricKindIR::NONE;
};

// ── Results ─────────────────────────────────────────────────────────────

enum class SolveStatusIR : int {
    SOLVED_SATISFICING = 0,
    SOLVED_OPTIMALLY = 1,
    UNSOLVABLE_PROVEN = 2,
    UNSOLVABLE_INCOMPLETELY = 3,
    TIMEOUT = 4,
    /// The problem asks for something the planner does not handle (an
    /// UnsupportedProblemError); the reason is an ERROR log message.
    UNSUPPORTED_PROBLEM = 5,
};

struct ActionInstanceIR {
    std::string action_name;
    std::vector<std::string> parameters;
};

enum class LogLevelIR : int { DEBUG = 0, INFO = 1, WARNING = 2, ERROR = 3 };

struct LogMessageIR {
    LogLevelIR level = LogLevelIR::INFO;
    std::string message;
};

struct ResultIR {
    SolveStatusIR status = SolveStatusIR::SOLVED_SATISFICING;
    bool has_plan = false;
    std::vector<ActionInstanceIR> plan;
    std::vector<LogMessageIR> log_messages;
    /// Per-solve statistics (Stats snapshot taken at the end of the solve)
    std::vector<std::pair<std::string, double>> stats;
};

} // namespace rantanplan::ir
