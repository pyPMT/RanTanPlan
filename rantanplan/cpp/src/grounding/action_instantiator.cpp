#include "action_instantiator.hpp"
#include "../problem/substitution.hpp"

namespace rantanplan {

// ---------------------------------------------------------------------------
// Helper: build an object-constant ExprID from an object index.
// ---------------------------------------------------------------------------

static ExprID make_object_constant(ExprPool& pool,
                                    const Problem& problem,
                                    int obj_idx) {
    const Object& obj = problem.object(obj_idx);

    ExprNode node;
    node.kind = static_cast<int>(ExprKind::CONSTANT);
    // NOTE: Leave node.op as default (-1) to match intern_nodes() in ir_io.cpp,
    // which does NOT set op for CONSTANT nodes.

    // Derive type_id via pointer arithmetic (Type* points into types_ vector).
    const Type* obj_type = obj.type();
    if (obj_type) {
        node.type_id = static_cast<int>(obj_type - &problem.types()[0]);
    }

    node.payload = obj.name();
    return pool.intern(node);
}

// ---------------------------------------------------------------------------
// Helper: build an integer-constant ExprID (mirrors an int literal loaded by
// intern_nodes() — kind CONSTANT with an int64_t payload). Used to substitute
// a bounded-int action parameter with its chosen value during grounding.
// ---------------------------------------------------------------------------

static ExprID make_int_constant(ExprPool& pool,
                                const Problem& problem,
                                int64_t value) {
    ExprNode node;
    node.kind = static_cast<int>(ExprKind::CONSTANT);
    const Type* int_type = problem.find_type("up:integer");
    if (int_type) {
        node.type_id = static_cast<int>(int_type - &problem.types()[0]);
    }
    node.payload = value;
    return pool.intern(node);
}

// ---------------------------------------------------------------------------
// Helper: build a Substitution (param_name → constant ExprID).
// ---------------------------------------------------------------------------

static Substitution build_substitution(ExprPool& pool,
                                        const Problem& problem,
                                        const Action& lifted_action,
                                        const PartialBinding& binding) {
    Substitution subst;
    for (const auto& [param_idx, value] : binding) {
        const Parameter& param = lifted_action.parameter(param_idx);
        // A binding slot holds an object index for object parameters, or a
        // literal integer value for bounded-int parameters (see
        // BindingMatcher::fill_unbound_parameters).
        ExprID constant_id = param.is_numeric()
            ? make_int_constant(pool, problem, static_cast<int64_t>(value))
            : make_object_constant(pool, problem, value);
        subst[param.name()] = constant_id;
    }
    return subst;
}

// ---------------------------------------------------------------------------
// Main entry point
// ---------------------------------------------------------------------------

Action instantiate_action(ExprPool& pool,
                          const Problem& problem,
                          const Action& lifted_action,
                          const PartialBinding& binding) {
    Substitution subst = build_substitution(pool, problem, lifted_action, binding);

    // Create ground action with base name (e.g., "board") and parameters
    // holding the bound object names (e.g., "person2", "plane1", "city0").
    // This ensures plan_to_ir() emits action instances the Python side can
    // resolve against the original lifted problem.
    Action ground(lifted_action.name());
    ground.set_pool(&pool);

    for (size_t i = 0; i < lifted_action.parameter_count(); ++i) {
        auto it = binding.find(static_cast<int>(i));
        if (it != binding.end()) {
            const Parameter& lparam = lifted_action.parameter(i);
            if (lparam.is_numeric()) {
                // Ground bounded-int parameter: record the chosen value as the
                // parameter name (e.g. "2") so plan_to_ir() emits it and the
                // Python side rebuilds Int(2) against the lifted signature.
                ground.add_parameter(
                    Parameter(std::to_string(it->second), lparam.type()));
            } else {
                const Object& obj = problem.object(it->second);
                ground.add_parameter(Parameter(obj.name(), obj.type()));
            }
        }
    }

    // Substitute precondition.
    if (lifted_action.has_precondition()) {
        ExprID ground_pre = substitute(pool, lifted_action.precondition_id(), subst);
        ground.set_precondition_id(ground_pre);
    }

    // Substitute cost expression.
    if (lifted_action.has_explicit_cost()) {
        ExprID ground_cost = substitute(pool, lifted_action.cost_id(), subst);
        ground.set_cost_id(ground_cost);
    }

    // Substitute each effect.
    for (const auto& eff : lifted_action.effects()) {
        const auto& ee = eff.effect_expression();

        ExprID ground_fluent = substitute(pool, ee.fluent_id(), subst);
        ExprID ground_value  = substitute(pool, ee.value_id(), subst);
        ExprID ground_cond   = ee.is_conditional()
            ? substitute(pool, ee.condition_id(), subst)
            : EXPR_NULL;

        EffectExpression ground_ee(ee.kind(), ground_fluent, ground_value,
                                    ground_cond, &pool);
        ground.add_effect(Effect(ground_ee));
    }

    return ground;
}

} // namespace rantanplan

