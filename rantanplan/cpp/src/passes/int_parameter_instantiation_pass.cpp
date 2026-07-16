#include "int_parameter_instantiation_pass.hpp"
#include "../problem/substitution.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"
#include "../util/stats.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace rantanplan {

namespace {

/// Substitute a name→constant map into every expression of `action`, producing a
/// copy whose integer parameters are the given constant-named parameters.
Action instantiate(const Action& action, ExprPool& pool,
                   const std::vector<Parameter>& new_params,
                   const Substitution& subst) {
    Action copy = action;
    copy.set_parameters(new_params);

    if (action.has_precondition())
        copy.set_precondition_id(substitute(pool, action.precondition_id(), subst));
    if (action.has_explicit_cost())
        copy.set_cost_id(substitute(pool, action.cost_id(), subst));

    std::vector<Effect> effects;
    effects.reserve(action.effects().size());
    for (const auto& eff : action.effects()) {
        const auto& ee = eff.effect_expression();
        std::vector<ExprID> foralls;
        foralls.reserve(ee.forall_variable_ids().size());
        for (ExprID fv : ee.forall_variable_ids())
            foralls.push_back(substitute(pool, fv, subst));
        EffectExpression new_ee(ee.kind(),
                                substitute(pool, ee.fluent_id(), subst),
                                substitute(pool, ee.value_id(), subst),
                                substitute(pool, ee.condition_id(), subst),
                                std::move(foralls), &pool);
        effects.push_back(Effect(new_ee));
    }
    copy.mutable_effects() = std::move(effects);
    return copy;
}

/// Emit one instantiated action per tuple in the Cartesian product of the int
/// parameters' [lower, upper] intervals (enumerated as an odometer).
void expand_action(const Action& action, ExprPool& pool,
                   const std::vector<size_t>& int_indices, std::vector<Action>& out) {
    // Read each integer parameter's interval; cur[] is the current tuple.
    std::vector<int64_t> lo(int_indices.size()), hi(int_indices.size()), cur(int_indices.size());
    for (size_t k = 0; k < int_indices.size(); ++k) {
        const Parameter& p = action.parameter(int_indices[k]);
        if (!p.lower_bound() || !p.upper_bound()) {
            throw std::invalid_argument(
                "Unbounded integer action parameter '" + p.name() + "' in action '" +
                action.name() + "' cannot be instantiated (needs both bounds).");
        }
        lo[k] = *p.lower_bound();
        hi[k] = *p.upper_bound();
        cur[k] = lo[k];
    }

    while (true) {
        // Substitute the current integer values, and record them as constant-named
        // parameters so the grounder carries them through for the plan round-trip.
        Substitution subst;
        std::vector<Parameter> new_params = action.parameters();
        for (size_t k = 0; k < int_indices.size(); ++k) {
            size_t idx = int_indices[k];
            subst[action.parameter(idx).name()] = pool.intern_int_constant(cur[k]);
            new_params[idx] = Parameter(std::to_string(cur[k]), action.parameter(idx).type());
        }
        out.push_back(instantiate(action, pool, new_params, subst));

        // Advance the odometer; stop once the most-significant digit wraps.
        size_t k = int_indices.size();
        while (k > 0) {
            --k;
            if (++cur[k] <= hi[k]) break;
            cur[k] = lo[k];
            if (k == 0) return;
        }
    }
}

}  // namespace

void IntParameterInstantiationPass::apply(PreprocessedProblem& result) const {
    ScopedTimer timer;  // measuring-only; the pipeline records the stats key
    const auto& problem = result.problem;
    ExprPool& pool = *problem.pool_ptr();

    bool any = false;
    std::vector<Action> new_actions;
    new_actions.reserve(problem.actions().size());

    for (const auto& action : problem.actions()) {
        // Which parameters are bounded integers?
        std::vector<size_t> int_indices;
        for (size_t i = 0; i < action.parameter_count(); ++i)
            if (action.parameter(i).is_numeric()) int_indices.push_back(i);

        if (int_indices.empty()) {
            new_actions.push_back(action);  // nothing to compile away
            continue;
        }
        any = true;
        expand_action(action, pool, int_indices, new_actions);
    }

    if (!any) {
        Logger::instance().component(VerbosityLevel::INFO, name(), {
            {"status", "no integer parameters"}
        });
        return;
    }

    Stats::instance().set("int_parameter_instantiation.actions",
                          static_cast<double>(new_actions.size()));

    result.problem = problem.with_simplified(
        std::move(new_actions),
        std::vector<Goal>(problem.goals()),
        std::vector<Assignment>(problem.initial_state()),
        std::vector<ExprID>(problem.state_invariants()));

    Logger::instance().component(VerbosityLevel::INFO, name(), {
        {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
        {"actions", std::to_string(new_actions.size())}
    });
}

} // namespace rantanplan
