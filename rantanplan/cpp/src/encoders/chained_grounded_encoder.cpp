#include "chained_grounded_encoder.hpp"
#include "../util/stats.hpp"
#include <iostream>
#include <algorithm>
#include <cassert>

namespace rantanplan {

ChainedGroundedEncoder::ChainedGroundedEncoder(const Problem& problem, cvc5::TermManager& ctx,
                                               std::unique_ptr<ParallelismStrategy> parallelism)
    : GroundedEncoder(problem, ctx, std::move(parallelism)) {
    build_variable_modifiers_index();
}

Term ChainedGroundedEncoder::get_or_create_chain_variable(const std::string& name,
                                                          const Type* var_type) {
    // See chain_vars_ in the header: the map is what makes a repeated request
    // for the same ζ name return the same variable, which Z3 gave for free.
    auto it = chain_vars_.find(name);
    if (it != chain_vars_.end()) {
        return it->second;
    }
    Term var = variable_factory_.create_symbol_variable(name, var_type);
    chain_vars_.emplace(name, var);
    return var;
}

void ChainedGroundedEncoder::build_variable_modifiers_index() {
    variable_modifiers_.clear();

    // One entry per EFFECT. Actions are visited in problem order and their
    // effects in declaration order, so each action's writes to one fluent stay
    // adjacent and in PDDL order — which is the order the chain composes them
    // in, and the only defensible one when they do not commute (two assigns).
    //
    // This list is deliberately NOT deduplicated by action. An action with two
    // writes to one fluent needs two chain links: with one link it would
    // contribute only its first write (see the deleted get_effect_for_variable),
    // so `drink(lamb,lamb)` — (decrease (locale ?from) 1) and (increase (locale
    // ?to) 1) ground at ?from = ?to — applied -1 where PDDL says 0, and the
    // clip-to-capacity idiom `(when (<= .. cap) (increase ..)) (when (> .. cap)
    // (assign .. cap))` lost whichever arm came second.
    for (const Action& action : problem_.actions()) {
        for (const Effect& effect : action.effects()) {
            variable_modifiers_[effect.effect_expression().fluent_id()]
                .emplace_back(&action, &effect);
        }
    }

    auto& stats = Stats::instance();
    size_t chained_vars = 0, cumulative_vars = 0;
    for (const auto& [eid, writes] : variable_modifiers_) {
        if (!is_chained(eid)) continue;
        chained_vars++;
        // The subset that genuinely needs the chain for cumulative semantics.
        // The rest are single-writer fluents chained only so that they, too,
        // need no frame axiom; tracking both keeps that cost visible.
        if (writes.size() > 1) cumulative_vars++;
    }
    stats.set("encoder.chained_variables", chained_vars);
    stats.set("encoder.cumulative_variables", cumulative_vars);
}


Term ChainedGroundedEncoder::encode_actions(int t) {
    TermVec all_constraints;
    auto& stats = Stats::instance();

    // Process each action with modified effect constraints
    for (const Action& action : problem_.actions()) {
        Term action_var = variable_factory_.get_action_variable(action, t);

        if (action.effects().empty()) continue;

        // Create precondition constraints (unchanged)
        if (action.has_precondition()) {
            Term z3_precond = convert_expr_id_to_z3(action.precondition_id(), t);
            all_constraints.push_back(tb().mk_implies(action_var, z3_precond));
        }

        // Create effect constraints, filtering out multi-written non-Boolean variables
        TermVec filtered_effects;

        for (const Effect& effect : action.effects()) {
            if (!is_chained(effect.effect_expression().fluent_id())) {
                // Boolean fluents only: every modified numeric one is chained.
                Term z3_effect = convert_effect_to_z3(effect.effect_expression(), t);
                filtered_effects.push_back(z3_effect);
            }
            // Chained variables are handled by encode_chained_effects
        }

        // Add filtered effect constraints for this action
        if (!filtered_effects.empty()) {
            all_constraints.push_back(tb().mk_implies(action_var, tb().mk_and(filtered_effects)));
        }
    }

    // Add chained effect constraints for multi-modified non-Boolean variables
    all_constraints.push_back(encode_chained_effects(t));

    stats.add("encoder.action_constraints", all_constraints.size());

    if (all_constraints.empty()) {
        return tb().bool_val(true);
    }

    return tb().mk_and(all_constraints);
}

Term ChainedGroundedEncoder::encode_chained_effects(int t) {
    TermVec constraints;
    auto& stats = Stats::instance();

    // Section 5.2 chained encoding, applied to EVERY modified numeric variable
    // rather than only the multi-written ones (see is_chained for why).
    // Key insight: conditions are evaluated using current chain position (input),
    // while effects produce the next chain position (output).
    //
    // Equations (7) and (9) are fused into the loop: the chain runs
    //     n^t = ζ_0  ->  ζ_1  ->  ...  ->  ζ_{k-1}  ->  n^{t+1}
    // with the LAST link writing n^{t+1} directly instead of minting a final ζ
    // and equating it.  So k writes cost k-1 chain variables, and the common
    // single-write case costs none at all — it collapses to the one constraint
    //     n^{t+1} = ite(a [AND c], eff(n^t), n^t)
    // which is exactly what the old effect implication plus its frame axiom
    // said between them.
    for (const auto& [var_eid, writes] : variable_modifiers_) {
        if (!is_chained(var_eid)) continue;

        const std::string var_str = problem_.pool().to_string(var_eid);
        const Type* var_type = problem_.type_for_id(var_eid);

        Term chain_prev = convert_expr_id_to_z3(var_eid, t);   // ζ_0 = n^t

        // Walk the writes in order. Two writes by the SAME action share one
        // action variable, so they either both apply or both pass through —
        // which is what composes them correctly.
        for (size_t i = 0; i < writes.size(); ++i) {
            const Action* action = writes[i].first;
            const Effect* full_effect = writes[i].second;
            Term action_var = variable_factory_.get_action_variable(*action, t);

            // The last link writes n^{t+1} directly (eq (9) fused in); the rest
            // mint ζ. Minting goes through the cache, not straight to the
            // factory: see chain_vars_ in the header — cvc5's mkConst is fresh
            // per call, so a second encode for the same t would otherwise build
            // a second, disconnected chain.
            const bool is_last = (i + 1 == writes.size());
            Term chain_next =
                is_last ? convert_expr_id_to_z3(var_eid, t + 1)
                        : get_or_create_chain_variable(
                              "zeta_" + var_str + "_t" + std::to_string(t) +
                                  "_i" + std::to_string(i + 1),
                              var_type);

            // Equation (8): Apply effect conditionally based on action execution.
            // The effect VALUE is converted at timestep t (the pre-state) while
            // the operator is applied to the running chain value, so additive
            // deltas accumulate while their expressions read the pre-state —
            // which is what PDDL 2.1 specifies.
            Term effect_applied =
                apply_effect_to_expression(full_effect->effect_expression(), chain_prev, t);

            // Unconditional:  action^t          -> next = eff(prev), else next = prev
            // Conditional:   (action^t AND c^t) -> next = eff(prev), else next = prev
            // The else-branch is what makes the successor TOTAL, and hence what
            // licenses effects_determine_successor to drop the frame axiom.
            Term apply_effect = action_var;
            if (full_effect->is_conditional()) {
                apply_effect = tb().mk_and({action_var,
                    convert_expr_id_to_z3(full_effect->effect_expression().condition_id(), t)});
            }
            constraints.push_back(tb().mk_ite(apply_effect,
                                              tb().mk_eq(chain_next, effect_applied),
                                              tb().mk_eq(chain_next, chain_prev)));

            chain_prev = chain_next;
        }
    }

    stats.add("encoder.chained_constraints", constraints.size());

    if (constraints.empty()) {
        return tb().bool_val(true);
    }

    return tb().mk_and(constraints);
}

// NOTE: get_effect_for_variable is gone. It carried a CONTRACT that at most one
// effect per (action, variable) reached this encoder, taking the first match and
// silently ignoring the rest — which is precisely the bug the per-effect
// variable_modifiers_ index fixes, so lowering passes no longer have to avoid
// emitting several same-target effects in one action. The array-lowering
// same-cell collision handling stays as it is: folding to a single
// expression-valued assign is still the clearer encoding, it is just no longer
// load-bearing for correctness here.
Term ChainedGroundedEncoder::apply_effect_to_expression(
    const EffectExpression& effect, const Term& base_expr, int timestep) const {

    // Convert the effect value to a term - cast away const for this method
    auto* non_const_this = const_cast<ChainedGroundedEncoder*>(this);
    Term value_z3 = non_const_this->convert_expr_id_to_z3(effect.value_id(), timestep);

    // Apply the effect operation to the base expression
    // Note: We can't reuse convert_effect_to_z3() from base class because that creates
    // a constraint (fluent_next == result), but we need the result expression itself
    switch (effect.kind()) {
        case EffectExpression::Kind::ASSIGN:
            return value_z3;

        case EffectExpression::Kind::INCREASE:
            return tb().add(base_expr, value_z3);

        case EffectExpression::Kind::DECREASE:
            return tb().sub(base_expr, value_z3);
    }

    // Fallback - should never reach here
    return base_expr;
}

std::optional<Term> ChainedGroundedEncoder::encode_non_cumulative_effects(
        const Action& action, int t) {
    if (action.effects().empty()) return std::nullopt;

    Term action_var = variable_factory_.get_action_variable(action, t);

    // Filter out effects on multi-modified non-Boolean variables — those are
    // handled by encode_chained_timestep_effects (chain variables ζ).
    TermVec filtered_effects;
    for (const Effect& effect : action.effects()) {
        if (!is_chained(effect.effect_expression().fluent_id())) {
            filtered_effects.push_back(convert_effect_to_z3(effect.effect_expression(), t));
        }
    }

    if (filtered_effects.empty()) return std::nullopt;
    return tb().mk_implies(action_var, tb().mk_and(filtered_effects));
}

Term ChainedGroundedEncoder::encode_cumulative_effects(int t) {
    return encode_chained_effects(t);
}

} // namespace rantanplan
