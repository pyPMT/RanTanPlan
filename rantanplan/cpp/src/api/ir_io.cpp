#include "ir_io.hpp"

#include "../problem/problem.hpp"
#include "../problem/plan.hpp"
#include "../problem/expr_enums.hpp"
#include "../util/logger.hpp"

#include <stdexcept>
#include <unordered_set>

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
        Parameter param(ir_param.name, param_type);
        if (ir_param.lower_bound) param.set_lower_bound(*ir_param.lower_bound);
        if (ir_param.upper_bound) param.set_upper_bound(*ir_param.upper_bound);
        params.emplace_back(std::move(param));
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
// Array type registration (two-phase; see load_problem_from_ir)
// ============================================================================

/// The single renderer for an array type's table name, from one dimension's
/// element name + size. C++ is the sole source of truth for these names: the
/// converter sends only the structured shape (element_type + array_sizes), and
/// both type registration below and per-fluent value-type resolution generate
/// the name through this one function — so there is no cross-language string to
/// keep in agreement.
static std::string array_type_name(const std::string& element, int64_t size) {
    return "array[" + std::to_string(size) + ", " + element + "]";
}

/// The whole-fluent (outermost) array type name, built innermost-first from the
/// structured shape. Matches the outer name minted in register_array_type_names
/// by construction (same renderer, same fields).
static std::string array_value_type_name(const ir::FluentIR& ir_fluent) {
    std::string element = ir_fluent.element_type;
    for (auto it = ir_fluent.array_sizes.rbegin();
         it != ir_fluent.array_sizes.rend(); ++it) {
        element = array_type_name(element, *it);
    }
    return element;
}

/// One first-class array type per dimension. Array types are entries in the
/// problem's type table, named via array_type_name and nested innermost-first.
/// Registration is two-phase because the table is a `std::vector<Type>`: every
/// name must be emplaced *before* any pointer into the table is taken (phase 1),
/// and element links are wired only once those pointers are stable (phase 2).
/// Each entry keeps the *index* of its type — indices survive vector growth,
/// pointers do not — plus the element name to resolve against the map in phase 2.
struct ArrayTypeEntry {
    size_t type_index;
    std::string element_name;
    int64_t size;
};

/// Phase 1: emplace a nested array type per dimension for every array fluent,
/// deduplicated by name. Must run before the name→pointer map is built; returns
/// the entries to link in phase 2.
static std::vector<ArrayTypeEntry> register_array_type_names(
        std::vector<Type>& types, const ir::ProblemIR& ir_problem) {
    std::vector<ArrayTypeEntry> array_types;
    std::unordered_set<std::string> seen;
    for (const auto& ir_fluent : ir_problem.fluents) {
        if (ir_fluent.array_sizes.empty()) continue;
        std::string element = ir_fluent.element_type;
        for (auto it = ir_fluent.array_sizes.rbegin();
             it != ir_fluent.array_sizes.rend(); ++it) {
            std::string tname = array_type_name(element, *it);
            if (seen.insert(tname).second) {
                types.emplace_back(tname);
                array_types.push_back({types.size() - 1, element, *it});
            }
            element = std::move(tname);
        }
    }
    return array_types;
}

/// Phase 2: link each registered array type to its element type. Runs after the
/// name→pointer map is built, so both the lookup and the stored pointer are
/// stable.
static void link_array_element_types(
        std::vector<Type>& types,
        const std::unordered_map<std::string, const Type*>& name_to_ptr,
        const std::vector<ArrayTypeEntry>& array_types) {
    for (const auto& entry : array_types) {
        types[entry.type_index].set_array(name_to_ptr.at(entry.element_name),
                                          entry.size);
    }
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
    // Array types are first-class, one (nested) entry per dimension. Register
    // all names before the name→pointer map is built (the table is a vector and
    // cannot grow once pointers into it exist), then link element types once
    // those pointers are stable — see register_array_type_names / link_...
    std::vector<ArrayTypeEntry> array_types =
        register_array_type_names(*problem.types_, ir_problem);
    for (auto& type : *problem.types_) {
        problem.type_name_to_ptr_[type.name()] = &type;
    }
    problem.resolve_type_hierarchy();
    link_array_element_types(*problem.types_, problem.type_name_to_ptr_, array_types);

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
        // An array fluent carries no value_type string; its type is the
        // generated (registered) array type name derived from its structured
        // shape. Scalar fluents resolve by their value_type string as before.
        const std::string value_type_name = ir_fluent.array_sizes.empty()
            ? ir_fluent.value_type
            : array_value_type_name(ir_fluent);
        const Type* value_type = ctx.find_type(value_type_name);
        if (!value_type) {
            Logger::instance().error("Could not find type '" + value_type_name +
                                     "' for fluent '" + ir_fluent.name + "'");
        }
        problem.fluents_.emplace_back(ir_fluent.name, value_type, params);
        if (ir_fluent.lower_bound || ir_fluent.upper_bound) {
            problem.fluents_.back().set_declared_bounds(ir_fluent.lower_bound,
                                                        ir_fluent.upper_bound);
        }
        // Undefined positions are per-fluent data, not part of the array type
        // (two fluents of one array type may exclude different cells).
        if (!ir_fluent.undefined_positions.empty()) {
            problem.fluents_.back().set_array_undefined_positions(
                ir_fluent.undefined_positions);
        }
        // Declared default_initial_value (constant node); consumed by the CWA
        // pass (authoritative over the clamped-0/false fallback) and by the
        // grounder's numeric bounds seeding.
        if (ir_fluent.default_value) {
            problem.fluents_.back().set_declared_default(
                ctx.id(*ir_fluent.default_value));
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
