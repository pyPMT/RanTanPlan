#include "ir_io.hpp"

#include "../problem/problem.hpp"
#include "../problem/plan.hpp"
#include "../problem/expr_enums.hpp"
#include "../util/logger.hpp"

#include <stdexcept>
#include <unordered_set>

namespace rantanplan {

// ============================================================================
// File-local helpers
// ============================================================================

/// The planner's kind for an IR node kind. UNKNOWN is a node the converter left
/// unset.
static ExprKind to_expr_kind(ir::ExprKindIR kind) {
    switch (kind) {
        case ir::ExprKindIR::CONSTANT: return ExprKind::CONSTANT;
        case ir::ExprKindIR::PARAMETER: return ExprKind::PARAMETER;
        case ir::ExprKindIR::FLUENT_SYMBOL: return ExprKind::FLUENT_SYMBOL;
        case ir::ExprKindIR::FUNCTION_SYMBOL: return ExprKind::FUNCTION_SYMBOL;
        case ir::ExprKindIR::STATE_VARIABLE: return ExprKind::STATE_VARIABLE;
        case ir::ExprKindIR::FUNCTION_APPLICATION: return ExprKind::FUNCTION_APPLICATION;
        case ir::ExprKindIR::VARIABLE: return ExprKind::VARIABLE;
        case ir::ExprKindIR::UNKNOWN: break;
    }
    throw std::runtime_error("Unsupported expression kind " +
                             std::to_string(static_cast<int>(kind)) + " in IR input");
}

ir::ExprKindIR to_ir_kind(ExprKind kind) {
    switch (kind) {
        case ExprKind::CONSTANT: return ir::ExprKindIR::CONSTANT;
        case ExprKind::PARAMETER: return ir::ExprKindIR::PARAMETER;
        case ExprKind::FLUENT_SYMBOL: return ir::ExprKindIR::FLUENT_SYMBOL;
        case ExprKind::FUNCTION_SYMBOL: return ir::ExprKindIR::FUNCTION_SYMBOL;
        case ExprKind::STATE_VARIABLE: return ir::ExprKindIR::STATE_VARIABLE;
        case ExprKind::FUNCTION_APPLICATION: return ir::ExprKindIR::FUNCTION_APPLICATION;
        case ExprKind::VARIABLE: return ir::ExprKindIR::VARIABLE;
        case ExprKind::UNKNOWN: break;
    }
    return ir::ExprKindIR::UNKNOWN;
}

/// The planner's operator for an IR one. IFF becomes EQUALS: = on two
/// booleans is iff, as in SMT-LIB, which has no iff of its own.
static ExprOperator to_expr_operator(ir::OperatorIR op) {
    switch (op) {
        case ir::OperatorIR::PLUS: return ExprOperator::PLUS;
        case ir::OperatorIR::MINUS: return ExprOperator::MINUS;
        case ir::OperatorIR::TIMES: return ExprOperator::MULTIPLY;
        case ir::OperatorIR::DIV: return ExprOperator::DIVIDE;
        case ir::OperatorIR::LE: return ExprOperator::LESS_EQUAL;
        case ir::OperatorIR::LT: return ExprOperator::LESS_THAN;
        case ir::OperatorIR::EQUALS: return ExprOperator::EQUALS;
        case ir::OperatorIR::AND: return ExprOperator::AND;
        case ir::OperatorIR::OR: return ExprOperator::OR;
        case ir::OperatorIR::NOT: return ExprOperator::NOT;
        case ir::OperatorIR::IMPLIES: return ExprOperator::IMPLIES;
        case ir::OperatorIR::IFF: return ExprOperator::EQUALS;
        case ir::OperatorIR::COUNT: return ExprOperator::COUNT;
        case ir::OperatorIR::EXISTS: return ExprOperator::EXISTS;
        case ir::OperatorIR::FORALL: return ExprOperator::FORALL;
        case ir::OperatorIR::ARRAY_INDEX: return ExprOperator::ARRAY_INDEX;
        case ir::OperatorIR::SET_MEMBER: return ExprOperator::SET_MEMBER;
        case ir::OperatorIR::SET_SUBSETEQ: return ExprOperator::SET_SUBSETEQ;
        case ir::OperatorIR::SET_DISJOINT: return ExprOperator::SET_DISJOINT;
        case ir::OperatorIR::SET_CARDINALITY: return ExprOperator::SET_CARDINALITY;
        case ir::OperatorIR::SET_ADD: return ExprOperator::SET_ADD;
        case ir::OperatorIR::SET_REMOVE: return ExprOperator::SET_REMOVE;
        case ir::OperatorIR::SET_UNION: return ExprOperator::SET_UNION;
        case ir::OperatorIR::SET_INTERSECT: return ExprOperator::SET_INTERSECT;
        case ir::OperatorIR::SET_DIFFERENCE: return ExprOperator::SET_DIFFERENCE;
        case ir::OperatorIR::SET_CONSTANT: return ExprOperator::SET_CONSTANT;
    }
    throw std::runtime_error("Unsupported operator " + std::to_string(static_cast<int>(op)) +
                             " in IR input");
}

static EffectExpression::Kind to_effect_kind(ir::EffectKindIR kind) {
    switch (kind) {
        case ir::EffectKindIR::ASSIGN: return EffectExpression::Kind::ASSIGN;
        case ir::EffectKindIR::INCREASE: return EffectExpression::Kind::INCREASE;
        case ir::EffectKindIR::DECREASE: return EffectExpression::Kind::DECREASE;
    }
    throw std::runtime_error("Unsupported effect kind " +
                             std::to_string(static_cast<int>(kind)) + " in IR input");
}

/// Resolve a UP type string to an index into the problem's type table, or -1
/// for "" or an unknown name.
static int resolve_type_id(const std::string& type_str,
                           const std::vector<Type>& types,
                           const std::unordered_map<std::string, const Type*>& type_map) {
    auto it = type_map.find(type_str);
    return it == type_map.end() ? -1 : static_cast<int>(it->second - types.data());
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
        const ExprKind kind = to_expr_kind(ir_node.kind);

        ExprNode node;
        node.kind = kind;
        node.type_id = resolve_type_id(ir_node.type_name, ctx.types, ctx.type_map);

        if (kind == ExprKind::FUNCTION_SYMBOL) {
            // An operator, under its standard spelling (the one ExprBuilder's
            // heads use); intern() gives its applications the same op.
            if (!ir_node.op) {
                throw std::runtime_error("Function symbol without an operator in IR input");
            }
            node.op = to_expr_operator(*ir_node.op);
            node.payload = expr_operator_to_string(node.op);
        } else if (ir_node.symbol) {
            node.payload = *ir_node.symbol;  // a name: fluent, object, parameter or variable
        } else if (ir_node.int_value) {
            node.payload = *ir_node.int_value;
        } else if (ir_node.real_value) {
            auto [num, den] = *ir_node.real_value;
            auto value = Rational::make(num, den);
            if (!value) {
                throw std::runtime_error("Real constant " + std::to_string(num) + "/" +
                                         std::to_string(den) + " in IR input is not a number");
            }
            node.payload = *value;
        } else if (ir_node.bool_value) {
            node.payload = *ir_node.bool_value;
        }

        node.children.reserve(ir_node.children.size());
        for (uint32_t child_index : ir_node.children) {
            if (child_index >= index) {
                throw std::runtime_error("IR node pool is not topologically ordered");
            }
            node.children.push_back(ctx.expr_ids[child_index]);
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
        EffectExpression::Kind kind = to_effect_kind(ir_effect.kind);
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

/// The single renderer for a set type's table name, from its element type name.
/// As with arrays, C++ is the sole source of truth: the converter sends only
/// the structured shape (element_type + shape=SET), and both set-type registration
/// and per-fluent value-type resolution go through this one function.
static std::string set_type_name(const std::string& element) {
    return "set{" + element + "}";
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
        if (ir_fluent.shape != ir::FluentShapeIR::ARRAY) continue;
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

/// A registered set type, kept by *index* (indices survive vector growth,
/// pointers do not) plus its element name to resolve against the map in phase 2.
struct SetTypeEntry {
    size_t type_index;
    std::string element_name;
};

/// Phase 1: emplace one `set{<element>}` type per distinct set fluent's element
/// type, deduplicated by name. Sets have no nesting (nested set types are
/// rejected at conversion), so this is the flat analogue of
/// register_array_type_names. Must run before the name→pointer map is built.
static std::vector<SetTypeEntry> register_set_type_names(
        std::vector<Type>& types, const ir::ProblemIR& ir_problem) {
    std::vector<SetTypeEntry> set_types;
    std::unordered_set<std::string> seen;
    for (const auto& ir_fluent : ir_problem.fluents) {
        if (ir_fluent.shape != ir::FluentShapeIR::SET) continue;
        std::string tname = set_type_name(ir_fluent.element_type);
        if (seen.insert(tname).second) {
            types.emplace_back(tname);
            set_types.push_back({types.size() - 1, ir_fluent.element_type});
        }
    }
    return set_types;
}

/// Phase 2: link each registered set type to its element type, once the
/// name→pointer map is stable.
static void link_set_element_types(
        std::vector<Type>& types,
        const std::unordered_map<std::string, const Type*>& name_to_ptr,
        const std::vector<SetTypeEntry>& set_types) {
    for (const auto& entry : set_types) {
        types[entry.type_index].set_set(name_to_ptr.at(entry.element_name));
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
    // The built-in types, at BOOL_TYPE_ID, INT_TYPE_ID and REAL_TYPE_ID.
    problem.types_->emplace_back("up:bool");
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
    // Set types are likewise first-class table entries, one `set{<element>}` per
    // distinct set fluent. Registered in the same before-the-map window as array
    // types so vector growth never dangles a Type*, and linked in the same
    // post-map phase (see register_set_type_names / link_set_element_types).
    std::vector<SetTypeEntry> set_types =
        register_set_type_names(*problem.types_, ir_problem);
    for (auto& type : *problem.types_) {
        problem.type_name_to_ptr_[type.name()] = &type;
    }
    problem.resolve_type_hierarchy();
    link_array_element_types(*problem.types_, problem.type_name_to_ptr_, array_types);
    link_set_element_types(*problem.types_, problem.type_name_to_ptr_, set_types);

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
        // Structured fluents (array / set) carry no value_type string; their
        // type is the generated (registered) type name derived from the
        // structured shape. The loader dispatches on the single `shape`
        // discriminator (mutually-exclusive kinds — see FluentShapeIR).
        const std::string value_type_name =
            ir_fluent.shape == ir::FluentShapeIR::ARRAY ? array_value_type_name(ir_fluent)
            : ir_fluent.shape == ir::FluentShapeIR::SET ? set_type_name(ir_fluent.element_type)
            :                                             ir_fluent.value_type;
        const Type* value_type = ctx.find_type(value_type_name);
        if (!value_type) {
            Logger::instance().error("Could not find type '" + value_type_name +
                                     "' for fluent '" + ir_fluent.name + "'");
        }
        problem.fluents_.emplace_back(ir_fluent.name, value_type, params);
        if (ir_fluent.lower_bound || ir_fluent.upper_bound) {
            auto bound = [&](const auto& b) -> std::optional<Rational> {
                if (!b) return std::nullopt;
                auto value = Rational::make(b->first, b->second);
                if (!value) {
                    throw std::runtime_error("A bound of fluent '" + ir_fluent.name +
                                             "' in IR input is not a number");
                }
                return value;
            };
            problem.fluents_.back().set_declared_bounds(bound(ir_fluent.lower_bound),
                                                        bound(ir_fluent.upper_bound));
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
    const ir::MetricKindIR metric_kind = ir_problem.metric_kind;
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
