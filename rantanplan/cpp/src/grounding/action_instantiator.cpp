#include "action_instantiator.hpp"
#include "../problem/expr_builder.hpp"
#include "../problem/substitution.hpp"

namespace rantanplan {

// ---------------------------------------------------------------------------
// Helper: build a Substitution (param_name → object-constant ExprID).
//
// Only object parameters appear in a binding. Integer parameters were already
// compiled to constants by IntParameterInstantiationPass, so they are absent
// here and their references in the expressions are already substituted.
// ---------------------------------------------------------------------------

static Substitution build_substitution(ExprPool& pool,
                                        const Problem& problem,
                                        const Action& lifted_action,
                                        const PartialBinding& binding) {
    ExprBuilder builder(pool);
    Substitution subst;
    for (const auto& [param_idx, obj_idx] : binding) {
        const Object& obj = problem.object(obj_idx);
        subst[lifted_action.parameter(param_idx).name()] =
            builder.object_constant(obj.name(), problem.type_id_of(obj.type()));
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

    // Create the ground action with the base name (e.g. "board") and parameters
    // in the original order, so plan_to_ir() emits instances the Python side can
    // resolve against the lifted problem.
    Action ground(lifted_action.name());
    ground.set_pool(&pool);

    for (size_t i = 0; i < lifted_action.parameter_count(); ++i) {
        const Parameter& lparam = lifted_action.parameter(i);
        auto it = binding.find(static_cast<int>(i));
        if (it != binding.end()) {
            // Object parameter: use the bound object's name.
            const Object& obj = problem.object(it->second);
            ground.add_parameter(Parameter(obj.name(), obj.type()));
        } else if (lparam.is_numeric()) {
            // Integer parameter already compiled to a constant-named parameter
            // (e.g. "2") by IntParameterInstantiationPass — carry it through so
            // the Python side rebuilds Int(2) against the lifted signature.
            ground.add_parameter(lparam);
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

