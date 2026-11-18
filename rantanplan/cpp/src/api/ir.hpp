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
 * Node `kind` values and payload shapes deliberately mirror the
 * unified_planning protobuf ExpressionKind encoding this IR replaced, so
 * the loader stays a port of the protobuf loader, not a redesign.
 */
namespace rantanplan::ir {

/// unified_planning expression kinds (subset the loader understands)
enum ExprKindIR : int {
    CONSTANT = 1,
    PARAMETER = 2,
    FLUENT_SYMBOL = 3,
    FUNCTION_SYMBOL = 4,
    STATE_VARIABLE = 5,
    FUNCTION_APPLICATION = 6,
    VARIABLE = 7,
};

struct ExprNodeIR {
    int kind = 0;
    /// UP type string ("up:bool", "up:integer", user type name) or ""
    std::string type_name;
    // Payload: at most one is set (atoms only; applications have children)
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
    std::optional<double> lower_bound;
    std::optional<double> upper_bound;
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

struct EffectIR {
    int kind = 0;  ///< 0 ASSIGN, 1 INCREASE, 2 DECREASE
    uint32_t fluent = 0;
    uint32_t value = 0;
    std::optional<uint32_t> condition;
    std::vector<uint32_t> forall;
};

struct ActionIR {
    std::string name;
    std::vector<ParameterIR> parameters;
    /// The converter synthesizes the precondition conjunction (true constant
    /// for no conditions, AND application for several)
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
    int metric_kind = 0;  ///< MetricKindIR value
};

// ── Results ─────────────────────────────────────────────────────────────

enum class SolveStatusIR : int {
    SOLVED_SATISFICING = 0,
    SOLVED_OPTIMALLY = 1,
    UNSOLVABLE_PROVEN = 2,
    UNSOLVABLE_INCOMPLETELY = 3,
    TIMEOUT = 4,
    UNSUPPORTED_PROBLEM = 5,
};

struct ActionInstanceIR {
    std::string action_name;
    std::vector<std::string> parameters;
};

struct LogMessageIR {
    int level = 1;  ///< 0 DEBUG, 1 INFO, 2 WARNING, 3 ERROR
    std::string message;
};

struct ResultIR {
    int status = 0;  ///< SolveStatusIR value
    bool has_plan = false;
    std::vector<ActionInstanceIR> plan;
    std::vector<LogMessageIR> log_messages;
    /// Per-solve statistics (Stats snapshot taken at the end of the solve)
    std::vector<std::pair<std::string, double>> stats;
};

} // namespace rantanplan::ir
