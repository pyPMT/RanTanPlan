#include "static_fluent_pass.hpp"
#include "../problem/expr_builder.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"
#include "../util/stats.hpp"
#include <cmath>
#include <optional>
#include <unordered_map>
#include <unordered_set>

namespace rantanplan {

// ============================================================================
// Helpers: extract numeric value from a CONSTANT ExprID
// ============================================================================

static std::optional<double> try_get_numeric(const ExprPool& pool, ExprID id) {
    if (!id.valid() || !pool.is_constant(id)) return std::nullopt;
    if (pool.payload_is_double(id)) return pool.payload_double(id);
    if (pool.payload_is_int(id))    return static_cast<double>(pool.payload_int(id));
    return std::nullopt;
}

// ============================================================================
// Constant folding for arithmetic operations
// ============================================================================

// Try to fold a FUNCTION_APPLICATION whose arguments (children[1..]) are all
// numeric constants. Returns a folded CONSTANT ExprID or EXPR_NULL. The result
// is an integer if every argument is one and the operation keeps integers (all
// but division).
static ExprID try_fold_arithmetic(ExprPool& pool, ExprOperator op,
                                   const std::vector<ExprID>& children) {
    // children[0] = head symbol; children[1..] = arguments
    if (children.size() < 2) return EXPR_NULL;

    // Collect numeric values from arguments
    std::vector<double> vals;
    bool integer = op != ExprOperator::DIVIDE;
    for (size_t i = 1; i < children.size(); ++i) {
        auto v = try_get_numeric(pool, children[i]);
        if (!v) return EXPR_NULL;
        vals.push_back(*v);
        integer = integer && pool.payload_is_int(children[i]);
    }

    double result;
    switch (op) {
        case ExprOperator::PLUS:
            result = 0.0;
            for (double v : vals) result += v;
            break;
        case ExprOperator::MINUS:
            if (vals.size() == 1) { result = -vals[0]; break; }
            result = vals[0] - vals[1];
            break;
        case ExprOperator::MULTIPLY:
            result = 1.0;
            for (double v : vals) result *= v;
            break;
        case ExprOperator::DIVIDE:
            if (vals.size() < 2 || vals[1] == 0.0) return EXPR_NULL;
            result = vals[0] / vals[1];
            break;
        case ExprOperator::MODULO:
            if (vals.size() < 2 || vals[1] == 0.0) return EXPR_NULL;
            result = std::fmod(vals[0], vals[1]);
            break;
        case ExprOperator::ABSOLUTE:
            result = std::abs(vals[0]);
            break;
        case ExprOperator::MAXIMUM:
            result = vals[0];
            for (size_t i = 1; i < vals.size(); ++i) result = std::max(result, vals[i]);
            break;
        case ExprOperator::MINIMUM:
            result = vals[0];
            for (size_t i = 1; i < vals.size(); ++i) result = std::min(result, vals[i]);
            break;
        default:
            return EXPR_NULL;
    }

    return pool.intern_folded_constant(result, integer);
}

// ============================================================================
// Constant folding for comparison operations
// ============================================================================

static ExprID try_fold_comparison(ExprPool& pool, ExprOperator op,
                                   const std::vector<ExprID>& children) {
    if (children.size() != 3) return EXPR_NULL;  // head + 2 args

    auto lhs = try_get_numeric(pool, children[1]);
    auto rhs = try_get_numeric(pool, children[2]);
    if (!lhs || !rhs) return EXPR_NULL;

    bool result;
    switch (op) {
        case ExprOperator::EQUALS:        result = (*lhs == *rhs); break;
        case ExprOperator::LESS_EQUAL:    result = (*lhs <= *rhs); break;
        case ExprOperator::LESS_THAN:     result = (*lhs <  *rhs); break;
        case ExprOperator::GREATER_EQUAL: result = (*lhs >= *rhs); break;
        case ExprOperator::GREATER_THAN:  result = (*lhs >  *rhs); break;
        default: return EXPR_NULL;
    }

    return pool.intern_bool_constant(result);
}

// ============================================================================
// Constant folding for boolean operations (full and partial)
// ============================================================================

// Returns a simplified ExprID, or EXPR_NULL if no simplification possible.
// Handles both full folding (all constant args) and partial folding
// (e.g., AND(true, X) → X).
static ExprID try_fold_boolean(ExprPool& pool, ExprBuilder& builder, ExprOperator op,
                                const std::vector<ExprID>& children, int type_id) {
    if (children.size() < 2) return EXPR_NULL;

    if (op == ExprOperator::NOT && children.size() == 2) {
        if (pool.is_true_constant(children[1]))  return pool.intern_bool_constant(false);
        if (pool.is_false_constant(children[1])) return pool.intern_bool_constant(true);
        return EXPR_NULL;
    }

    if (op == ExprOperator::AND) {
        // Filter out true args, short-circuit on false
        std::vector<ExprID> kept;
        for (size_t i = 1; i < children.size(); ++i) {
            if (pool.is_true_constant(children[i])) continue;  // skip true
            if (pool.is_false_constant(children[i])) return pool.intern_bool_constant(false);
            kept.push_back(children[i]);
        }
        if (kept.empty()) return pool.intern_bool_constant(true);
        if (kept.size() == 1) return kept[0];
        if (kept.size() < children.size() - 1)
            return builder.make_app(ExprOperator::AND, kept, type_id);
        return EXPR_NULL;
    }

    if (op == ExprOperator::OR) {
        // Filter out false args, short-circuit on true
        std::vector<ExprID> kept;
        for (size_t i = 1; i < children.size(); ++i) {
            if (pool.is_false_constant(children[i])) continue;  // skip false
            if (pool.is_true_constant(children[i])) return pool.intern_bool_constant(true);
            kept.push_back(children[i]);
        }
        if (kept.empty()) return pool.intern_bool_constant(false);
        if (kept.size() == 1) return kept[0];
        if (kept.size() < children.size() - 1)
            return builder.make_app(ExprOperator::OR, kept, type_id);
        return EXPR_NULL;
    }

    if (op == ExprOperator::IMPLIES && children.size() == 3) {
        if (pool.is_false_constant(children[1])) return pool.intern_bool_constant(true);
        if (pool.is_true_constant(children[1]))  return children[2];
        if (pool.is_true_constant(children[2]))  return pool.intern_bool_constant(true);
    }

    return EXPR_NULL;
}

// ============================================================================
// Combined substitute + constant fold walker
// ============================================================================

static ExprID simplify_expr(ExprPool& pool, ExprBuilder& builder, ExprID expr,
                             const std::unordered_map<ExprID, ExprID>& static_values) {
    if (!expr.valid()) return expr;

    // Direct substitution: if this is a static fluent → return its constant value
    if (auto it = static_values.find(expr); it != static_values.end()) {
        return it->second;
    }

    const ExprNode& node_ref = pool.get(expr);

    // Leaf → unchanged
    if (node_ref.children.empty()) return expr;

    // Copy needed fields before recursing (pool.intern may reallocate nodes_)
    auto orig_children = node_ref.children;
    ExprKind orig_kind = node_ref.kind;
    ExprOperator orig_op = node_ref.op;
    int orig_type_id = node_ref.type_id;
    // node_ref may be dangling after this point

    // Recurse on all children
    bool any_changed = false;
    std::vector<ExprID> new_children;
    new_children.reserve(orig_children.size());
    for (ExprID child : orig_children) {
        ExprID nc = simplify_expr(pool, builder, child, static_values);
        new_children.push_back(nc);
        if (nc != child) any_changed = true;
    }

    if (!any_changed) return expr;

    // Try constant folding on FUNCTION_APPLICATION nodes
    if (orig_kind == ExprKind::FUNCTION_APPLICATION) {
        ExprOperator op = orig_op;

        if (is_arithmetic_operator(op)) {
            ExprID folded = try_fold_arithmetic(pool, op, new_children);
            if (folded.valid()) return folded;
        } else if (is_comparison_operator(op)) {
            ExprID folded = try_fold_comparison(pool, op, new_children);
            if (folded.valid()) return folded;
        } else if (is_logical_operator(op)) {
            ExprID folded = try_fold_boolean(pool, builder, op, new_children, orig_type_id);
            if (folded.valid()) return folded;
        }
    }

    // Rebuild the node with substituted children (kind/op/type_id/payload
    // preserved from the original — read fresh, as the content at `expr` is
    // stable across the appends above).
    return builder.reintern(expr, std::move(new_children));
}

// ============================================================================
// Pass implementation
// ============================================================================

void StaticFluentPass::apply(PreprocessedProblem& result) const {
    ScopedTimer timer;  // measuring-only; the pipeline records the stats key
    const auto& problem = result.problem;
    ExprPool& pool = *problem.pool_ptr();
    ExprBuilder builder(pool);

    // Step 1: Identify fluents modified by any effect
    std::unordered_set<ExprID> modified_fluents;
    for (const auto& action : problem.actions()) {
        for (const auto& effect : action.effects()) {
            modified_fluents.insert(effect.effect_expression().fluent_id());
        }
    }

    // Step 2: Static fluents = grounded fluents NOT in modified set
    std::unordered_set<ExprID> static_fluent_set;
    for (ExprID gf : problem.grounded_fluents()) {
        if (!modified_fluents.count(gf)) {
            static_fluent_set.insert(gf);
        }
    }

    if (static_fluent_set.empty()) {
        Logger::instance().component(VerbosityLevel::INFO, name(), {
            {"status", "no static fluents found"}
        });
        return;
    }

    // Step 3: Build value map from initial state
    std::unordered_map<ExprID, ExprID> static_values;
    for (const auto& assignment : problem.initial_state()) {
        if (static_fluent_set.count(assignment.fluent_id())) {
            static_values[assignment.fluent_id()] = assignment.value_id();
        }
    }

    // Step 4: Simplify all action expressions
    std::vector<Action> new_actions;
    new_actions.reserve(problem.actions().size());

    for (const auto& action : problem.actions()) {
        Action new_action = action;

        // Simplify precondition
        if (new_action.has_precondition()) {
            ExprID new_pre = simplify_expr(pool, builder, action.precondition_id(), static_values);
            if (new_pre != action.precondition_id()) {
                new_action.set_precondition_id(new_pre);
            }
        }

        // Simplify effects
        for (auto& effect : new_action.mutable_effects()) {
            const auto& ee = effect.effect_expression();
            ExprID new_value = simplify_expr(pool, builder, ee.value_id(), static_values);
            ExprID new_cond = ee.is_conditional()
                ? simplify_expr(pool, builder, ee.condition_id(), static_values)
                : ee.condition_id();

            if (new_value != ee.value_id() || new_cond != ee.condition_id()) {
                EffectExpression new_ee(ee.kind(), ee.fluent_id(), new_value, new_cond, &pool);
                effect.set_effect_expression(new_ee);
            }
        }

        // Simplify cost expression
        if (new_action.has_explicit_cost()) {
            ExprID new_cost = simplify_expr(pool, builder, action.cost_id(), static_values);
            if (new_cost != action.cost_id()) {
                new_action.set_cost_id(new_cost);
            }
        }

        new_actions.push_back(std::move(new_action));
    }

    // Step 5: Simplify goals
    std::vector<Goal> new_goals;
    new_goals.reserve(problem.goals().size());
    for (const auto& goal : problem.goals()) {
        Goal new_goal = goal;
        ExprID new_gid = simplify_expr(pool, builder, goal.goal_id(), static_values);
        if (new_gid != goal.goal_id()) {
            new_goal.set_goal_id(new_gid);
        }
        new_goals.push_back(std::move(new_goal));
    }

    // Step 6: Filter initial state (remove static fluent assignments)
    std::vector<Assignment> new_initial_state;
    new_initial_state.reserve(problem.initial_state().size());
    for (const auto& assignment : problem.initial_state()) {
        if (!static_fluent_set.count(assignment.fluent_id())) {
            new_initial_state.push_back(assignment);
        }
    }

    // Step 6b: Simplify declared state invariants (same substitution as goals —
    // an invariant mentioning a folded static fluent would dangle otherwise)
    std::vector<ExprID> new_invariants;
    new_invariants.reserve(problem.state_invariants().size());
    for (ExprID inv : problem.state_invariants()) {
        new_invariants.push_back(simplify_expr(pool, builder, inv, static_values));
    }

    // Step 7: Build new problem (re-collects grounded fluents automatically)
    size_t old_fluent_count = problem.grounded_fluent_count();
    result.problem = problem.with_simplified(
        std::move(new_actions), std::move(new_goals), std::move(new_initial_state),
        std::move(new_invariants));
    size_t new_fluent_count = result.problem.grounded_fluent_count();

    Stats::instance().set("static_fluent.removed",
                           static_cast<double>(static_values.size()));

    Logger::instance().component(VerbosityLevel::INFO, name(), {
        {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
        {"static fluents", std::to_string(static_values.size())},
        {"grounded fluents", std::to_string(old_fluent_count) + " -> " +
                             std::to_string(new_fluent_count)}
    });
}

} // namespace rantanplan
