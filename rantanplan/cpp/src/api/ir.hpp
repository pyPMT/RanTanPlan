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
 * Produced by rantanplan/converter.py (a UP DagWalker) and consumed by
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
};

struct FluentIR {
    std::string name;
    std::string value_type;
    std::vector<ParameterIR> parameters;
    /// Declared numeric bounds for bounded int/real value types (e.g.
    /// `integer[0, 15]`); unset for unbounded numeric or non-numeric fluents.
    std::optional<double> lower_bound;
    std::optional<double> upper_bound;
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
