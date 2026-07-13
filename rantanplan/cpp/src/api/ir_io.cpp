#include "ir_io.hpp"

#include "../problem/problem.hpp"
#include "../problem/plan.hpp"
#include "../problem/expr_enums.hpp"
#include "../util/logger.hpp"

#include <iostream>
#include <stdexcept>

namespace rantanplan {

// ============================================================================
// File-local helpers (ports of the former protobuf loader's logic)
// ============================================================================

/// Resolve a UP type string to an index into the problem's type table,
/// handling the same aliases the protobuf loader handled.
static int resolve_type_id(const std::string& type_str,
                           const std::vector<Type>& types,
                           const std::unordered_map<std::string, const Type*>& type_map) {
    if (type_str.empty()) return -1;

    std::string resolved = type_str;
    if (type_str == "up:integer") resolved = "up:int";
    else if (type_str == "up:boolean") resolved = "up:bool";

    auto it = type_map.find(resolved);
    if (it == type_map.end() && resolved != type_str) {
        it = type_map.find(type_str);
    }
    if (it != type_map.end()) {
        const Type* found_type = it->second;
        for (size_t i = 0; i < types.size(); ++i) {
            if (&types[i] == found_type) {
                return static_cast<int>(i);
            }
        }
    }
    return -1;
}

/// Context shared by the loading helpers.
struct IRLoadContext {
    const ir::ProblemIR& in;
    const std::vector<Type>& types;
    const std::unordered_map<std::string, const Type*>& type_map;
    ExprPool& pool;
    /// IR node index -> interned ExprID (filled by intern_nodes)
    std::vector<ExprID> expr_ids;

    ExprID id(uint32_t ir_index) const { return expr_ids.at(ir_index); }

    const Type* find_type(const std::string& name) const {
        auto it = type_map.find(name);
        return (it != type_map.end()) ? it->second : nullptr;
    }
};

/// Intern the flat node pool in one forward pass. Children precede parents
/// by construction, so every child ExprID is available when needed.
static void intern_nodes(IRLoadContext& ctx) {
    ctx.expr_ids.reserve(ctx.in.nodes.size());

    for (size_t index = 0; index < ctx.in.nodes.size(); ++index) {
        const ir::ExprNodeIR& ir_node = ctx.in.nodes[index];
        if (ir_node.kind == 8) {
            throw std::runtime_error("Unsupported expression kind CONTAINER_ID (8) in IR input");
        }

        ExprNode node;
        node.kind = ir_node.kind;
        node.type_id = resolve_type_id(ir_node.type_name, ctx.types, ctx.type_map);

        auto kind = static_cast<ExprKind>(ir_node.kind);
        if (ir_node.symbol) {
            std::string symbol = *ir_node.symbol;
            if (kind == ExprKind::FUNCTION_SYMBOL || kind == ExprKind::FLUENT_SYMBOL) {
                symbol = map_up_operator(symbol);
            }
            node.payload = symbol;
            if (kind == ExprKind::FUNCTION_SYMBOL) {
                node.op = static_cast<int>(string_to_expr_operator(symbol));
            }
        } else if (ir_node.int_value) {
            node.payload = *ir_node.int_value;
        } else if (ir_node.real_value) {
            double val = static_cast<double>(ir_node.real_value->first) /
                         static_cast<double>(ir_node.real_value->second);
            node.payload = val;
        } else if (ir_node.bool_value) {
            node.payload = *ir_node.bool_value;
        }

        node.children.reserve(ir_node.children.size());
        for (size_t i = 0; i < ir_node.children.size(); ++i) {
            uint32_t child_index = ir_node.children[i];
            if (child_index >= index) {
                throw std::runtime_error("IR node pool is not topologically ordered");
            }
            node.children.push_back(ctx.expr_ids[child_index]);

            // A function application takes its operator from its head symbol
            // (same deduction the protobuf loader made on the child atom).
            if (i == 0 && kind == ExprKind::FUNCTION_APPLICATION) {
                const ir::ExprNodeIR& head = ctx.in.nodes[child_index];
                if (head.kind == static_cast<int>(ExprKind::FUNCTION_SYMBOL) && head.symbol) {
                    node.op = static_cast<int>(
                        string_to_expr_operator(map_up_operator(*head.symbol)));
                }
            }
        }

        ctx.expr_ids.push_back(ctx.pool.intern(std::move(node)));
    }
}

static Action convert_action(IRLoadContext& ctx, const ir::ActionIR& ir_action) {
    std::vector<Parameter> params;
    for (const auto& ir_param : ir_action.parameters) {
        const Type* param_type = ctx.find_type(ir_param.type_name);
        params.emplace_back(ir_param.name, param_type);
    }

    std::vector<Effect> effects;
    for (const auto& ir_effect : ir_action.effects) {
        auto kind = static_cast<EffectExpression::Kind>(ir_effect.kind);
        ExprID fluent_id = ctx.id(ir_effect.fluent);
        ExprID value_id = ctx.id(ir_effect.value);

        ExprID condition_id = EXPR_NULL;
        if (ir_effect.condition) {
            condition_id = ctx.id(*ir_effect.condition);
        }

        std::vector<ExprID> forall_ids;
        for (uint32_t var : ir_effect.forall) {
            forall_ids.push_back(ctx.id(var));
        }

        effects.emplace_back(EffectExpression(kind, fluent_id, value_id, condition_id,
                                              std::move(forall_ids), &ctx.pool));
    }

    return Action(ir_action.name, std::move(params), ctx.id(ir_action.precondition),
                  std::move(effects), &ctx.pool);
}

// ============================================================================
// Public API
// ============================================================================

Problem load_problem_from_ir(const ir::ProblemIR& ir_problem) {
    Problem problem;

    // --- Load types ---
    problem.types_->clear();
    problem.type_name_to_ptr_.clear();
    problem.types_->emplace_back("up:bool");
    problem.types_->emplace_back("up:int");
    problem.types_->emplace_back("up:integer");
    problem.types_->emplace_back("up:real");
    for (const auto& ir_type : ir_problem.types) {
        problem.types_->emplace_back(ir_type.name);
        problem.types_->back().set_parent_name(ir_type.parent);
    }
    for (auto& type : *problem.types_) {
        problem.type_name_to_ptr_[type.name()] = &type;
    }
    problem.resolve_type_hierarchy();

    IRLoadContext ctx{ir_problem, *problem.types_, problem.type_name_to_ptr_,
                      *problem.pool_, {}};

    // --- Intern the expression pool (children precede parents) ---
    intern_nodes(ctx);

    // --- Load objects ---
    for (const auto& ir_obj : ir_problem.objects) {
        const Type* type = ctx.find_type(ir_obj.type_name);
        problem.objects_.emplace_back(ir_obj.name, type);
    }
    problem.build_object_mappings();

    // --- Load fluents ---
    for (const auto& ir_fluent : ir_problem.fluents) {
        std::vector<Parameter> params;
        for (const auto& ir_param : ir_fluent.parameters) {
            const Type* param_type = ctx.find_type(ir_param.type_name);
            params.emplace_back(ir_param.name, param_type);
        }
        const Type* value_type = ctx.find_type(ir_fluent.value_type);
        if (!value_type) {
            Logger::instance().error("Could not find type '" + ir_fluent.value_type +
                                     "' for fluent '" + ir_fluent.name + "'");
        }
        problem.fluents_.emplace_back(ir_fluent.name, value_type, params);
        if (ir_fluent.lower_bound || ir_fluent.upper_bound) {
            problem.fluents_.back().set_declared_bounds(ir_fluent.lower_bound,
                                                        ir_fluent.upper_bound);
        }
    }
    problem.build_fluent_mappings();

    // --- Load actions ---
    for (const auto& ir_action : ir_problem.actions) {
        problem.actions_.push_back(convert_action(ctx, ir_action));
        problem.actions_.back().set_id(problem.actions_.size() - 1);
    }
    problem.build_action_mappings();

    // --- Load initial state ---
    for (const auto& ir_assignment : ir_problem.initial_state) {
        problem.initial_state_.emplace_back(ctx.id(ir_assignment.fluent),
                                            ctx.id(ir_assignment.value),
                                            &problem.pool());
    }

    // --- Load goals ---
    for (uint32_t ir_goal : ir_problem.goals) {
        problem.goals_.emplace_back(ctx.id(ir_goal), &problem.pool());
    }

    // --- Load declared state invariants (bodies of `always` constraints) ---
    for (uint32_t ir_inv : ir_problem.state_invariants) {
        problem.state_invariants_.push_back(ctx.id(ir_inv));
    }

    // --- Load metric (costs already resolved per action by the converter) ---
    auto metric_kind = static_cast<ir::MetricKindIR>(ir_problem.metric_kind);
    if (metric_kind == ir::MetricKindIR::MINIMIZE_ACTION_COSTS) {
        problem.set_metric_kind(MetricKind::MINIMIZE_ACTION_COSTS);
        for (size_t i = 0; i < problem.actions_.size(); ++i) {
            if (ir_problem.actions[i].cost) {
                problem.actions_[i].set_cost_id(ctx.id(*ir_problem.actions[i].cost));
            }
        }
    } else if (metric_kind == ir::MetricKindIR::MINIMIZE_PLAN_LENGTH) {
        problem.set_metric_kind(MetricKind::MINIMIZE_PLAN_LENGTH);
    }

    // Ensure every action has a cost_id. Actions without explicit costs
    // (e.g., MINIMIZE_PLAN_LENGTH or no metric) default to unit cost = 1.
    {
        ExprID unit_cost = problem.pool_->intern_int_constant(1);
        for (auto& action : problem.actions_) {
            if (!action.has_explicit_cost()) {
                action.set_cost_id(unit_cost);
            }
        }
    }

    problem.collect_grounded_fluents();
    return problem;
}

std::vector<ir::ActionInstanceIR> plan_to_ir(const Plan& plan) {
    std::vector<ir::ActionInstanceIR> actions;
    actions.reserve(plan.actions().size());

    for (const Action* action : plan.actions()) {
        ir::ActionInstanceIR instance;
        instance.action_name = action->name();
        for (const Parameter& param : action->parameters()) {
            instance.parameters.push_back(param.name());
        }
        actions.push_back(std::move(instance));
    }

    return actions;
}

} // namespace rantanplan
