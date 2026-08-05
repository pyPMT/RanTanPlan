#include "semantic_interference_predicate.hpp"
#include "action_footprint.hpp"
#include <cassert>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#ifndef NDEBUG
#include "../util/logger.hpp"
#endif

namespace rantanplan {

SemanticInterferencePredicate::SemanticInterferencePredicate(const Problem& problem) {
    z3_context_ = std::make_unique<z3::context>();
    z3_solver_ = std::make_unique<z3::solver>(*z3_context_);
    z3_variable_factory_ = std::make_unique<Z3VariableFactory>(*z3_context_);
    z3_variable_factory_->set_problem(&problem);
    grounded_visitor_ = std::make_unique<GroundedEncodingVisitor>(
        *z3_context_, &problem, z3_variable_factory_.get());
    for (const Action& action : problem.actions()) {
        footprints_[action.id()] = compute_action_footprint(problem, action);
    }
}

bool SemanticInterferencePredicate::shares_fluent(const Action& a1,
                                                  const Action& a2) const {
    auto it1 = footprints_.find(a1.id());
    auto it2 = footprints_.find(a2.id());
    // An action the constructor never saw (id from a different Problem) must not
    // be filtered out — fall through to the full check rather than answer NONE.
    if (it1 == footprints_.end() || it2 == footprints_.end()) return true;

    // Iterate the smaller set.
    const auto& small = it1->second.size() <= it2->second.size() ? it1->second : it2->second;
    const auto& large = it1->second.size() <= it2->second.size() ? it2->second : it1->second;
    for (ExprID fid : small) {
        if (large.count(fid)) return true;
    }
    return false;
}

#ifndef NDEBUG
void SemanticInterferencePredicate::assert_filter_hides_nothing(
    const Action& a1, const Action& a2) const {
    InterferenceSource hidden = check1(a1, a2)
        ? InterferenceSource::CHECK1
        : check2_source(a1, a2);
    if (hidden == InterferenceSource::NONE) return;

    Logger::instance().info(
        "INVARIANT VIOLATION: the footprint filter would hide a real "
        "interference — '" + a1.label() + "' vs '" + a2.label() +
        "' (source=" + std::to_string(static_cast<int>(hidden)) +
        "). compute_action_footprint is too narrow for this pair.");
    assert(false && "footprint filter must never hide a real interference");
}
#endif

InterferenceSource SemanticInterferencePredicate::affects_source(const Action& a1,
                                                                 const Action& a2) const {
    // Definition 3.9: a1 affects a2 if it can prevent a2's execution (check1) or
    // its effects don't compose commutatively with a2's (check2).
    //
    // There used to be a third branch here: if a1 had two conditional effects on
    // one fluent whose guards could both hold, report CONFLICTING_COND_EFFECTS.
    // That was not a property of the pair at all — it was a conservative screen
    // around a bug in apply_action_effects_substitution, which built every arm of
    // its ite chain on the PRE-STATE fluent instead of on the running value and so
    // dropped all but the last write when several guards held. The composition is
    // correct now, so the screen is gone; see the note there.
    // Necessary condition, checked first: actions touching no common fluent
    // cannot interfere. Answering here rather than in the callers is what makes
    // the search and plan extraction agree — see footprints_ in the header.
    if (!shares_fluent(a1, a2)) {
#ifndef NDEBUG
        assert_filter_hides_nothing(a1, a2);
#endif
        return InterferenceSource::NONE;
    }

    if (check1(a1, a2)) {
        return InterferenceSource::CHECK1;
    }
    return check2_source(a1, a2);  // CHECK2_UNCOMM / CHECK2_HAPPEN / NONE
}

z3::expr SemanticInterferencePredicate::apply_action_effects_substitution(
    const Action& action, const z3::expr& target_expr) const {
    // σ(action, expr): apply the action's effects to target_expr by substitution.
    // Multiple conditional effects on the same fluent are composed into a nested
    // ite chain (Z3's substitute() would otherwise drop duplicate keys).
    if (action.effects().empty()) {
        return target_expr;
    }

    // Group effects by fluent ExprID to compose multiple effects on one fluent.
    std::unordered_map<ExprID, std::vector<const Effect*>> effects_by_fluent;
    std::vector<ExprID> fluent_order;  // preserve first-seen order
    for (const Effect& effect : action.effects()) {
        ExprID fid = effect.effect_expression().fluent_id();
        if (effects_by_fluent.find(fid) == effects_by_fluent.end()) {
            fluent_order.push_back(fid);
        }
        effects_by_fluent[fid].push_back(&effect);
    }

    z3::expr_vector from_vector(*z3_context_);
    z3::expr_vector to_vector(*z3_context_);

    for (ExprID fid : fluent_order) {
        z3::expr fluent_z3 = grounded_visitor_->convert_from_pool(fid, -1);
        z3::expr composed = fluent_z3;  // start with the unchanged fluent

        // Nested ite chain, each arm built on the RUNNING value rather than on
        // the pre-state fluent. That distinction is the whole correctness of the
        // composition when several writes to one fluent can fire together:
        //
        //   base = fluent_z3  ->  ite(c2, f+d2, ite(c1, f+d1, f))   yields f+d2
        //                         when both guards hold: d1 is dropped.
        //   base = composed   ->  ite(c2, ite(c1,f+d1,f)+d2, ...)   yields
        //                         f+d1+d2, the true cumulative effect.
        //
        // The delta expressions and the guards are still converted at pseudo-
        // timestep -1, so they read the PRE-state while the accumulation is
        // sequential — which is exactly PDDL 2.1, and matches both
        // happening_value() here and the chain in ChainedGroundedEncoder.
        // Mutual exclusivity of the guards is no longer required.
        //
        // ASSIGN ignores the base and overrides; INCREASE/DECREASE add to it. So
        // the unconditional branch below is correct for either kind without
        // special-casing.
        for (const Effect* effect : effects_by_fluent[fid]) {
            const EffectExpression& eff_expr = effect->effect_expression();
            z3::expr new_value_z3 = convert_effect_to_z3(eff_expr, composed);

            if (effect->is_conditional()) {
                z3::expr condition_z3 = grounded_visitor_->convert_from_pool(
                    effect->effect_expression().condition_id(), -1);
                composed = z3::ite(condition_z3, new_value_z3, composed);
            } else {
                composed = new_value_z3;
            }
        }

        from_vector.push_back(fluent_z3);
        to_vector.push_back(composed);
    }

    z3::expr result = target_expr;
    return result.substitute(from_vector, to_vector);
}

bool SemanticInterferencePredicate::check1(const Action& a1, const Action& a2) const {
    // Condition 1 of Def 3.9: pre(a1) ∧ pre(a2) ∧ ¬(pre(a2) σ_a1) satisfiable —
    // can a1's effects falsify a2's precondition? (composition of functions is
    // correct here: a precondition IS evaluated after a1's full effects.)
    if (!a2.has_precondition()) return false;

    z3::expr a1_pre = convert_precondition_to_z3(a1);
    z3::expr a2_pre = convert_precondition_to_z3(a2);

    // Apply a1's effects to a2's precondition.
    z3::expr substituted_expr = apply_action_effects_substitution(a1, a2_pre);

    // If a1 doesn't touch a2's precondition variables, it cannot prevent it.
    if (z3::eq(a2_pre, substituted_expr)) {
        return false;
    }

    z3::expr check1 = a1_pre && a2_pre && !substituted_expr;

    z3_solver_->push();
    z3_solver_->add(check1);
    z3::check_result result = z3_solver_->check();
    z3_solver_->pop();

    return (result == z3::sat);
}

InterferenceSource SemanticInterferencePredicate::check2_source(const Action& a1,
                                                               const Action& a2) const {
    // Condition 2 of Def 3.9:
    //   2a. not simply commuting            → unconditional interference
    //   2b. simply commuting but happening ≠ sequential → state-dependent
    if (!are_simply_commuting(a1, a2)) {
        return InterferenceSource::CHECK2_UNCOMM;
    }

    z3::expr a1_pre = convert_precondition_to_z3(a1);
    z3::expr a2_pre = convert_precondition_to_z3(a2);

    // Every fluent touched by either action.
    std::unordered_set<ExprID> affected_var_eids;
    for (const Effect& effect : a1.effects()) {
        affected_var_eids.insert(effect.effect_expression().fluent_id());
    }
    for (const Effect& effect : a2.effects()) {
        affected_var_eids.insert(effect.effect_expression().fluent_id());
    }

    for (ExprID var_eid : affected_var_eids) {
        // Happening x·σ_h ("function of compositions"): per-variable parallel
        // composition, read-only variables kept at their pre-state.
        z3::expr var_after_happening = happening_value(var_eid, a1, a2);

        // Sequential x·σ_a2·σ_a1 ("composition of functions"): a2 then a1.
        z3::expr var_z3 = grounded_visitor_->convert_from_pool(var_eid, -1);
        z3::expr var_after_sequential = var_z3;
        var_after_sequential = apply_action_effects_substitution(a2, var_after_sequential);
        var_after_sequential = apply_action_effects_substitution(a1, var_after_sequential);

        if (z3::eq(var_after_happening, var_after_sequential)) {
            continue;
        }

        z3::expr non_commutativity = a1_pre && a2_pre &&
                                     (var_after_happening != var_after_sequential);

        z3_solver_->push();
        z3_solver_->add(non_commutativity);
        z3::check_result result = z3_solver_->check();
        z3_solver_->pop();

        if (result == z3::sat) {
            return InterferenceSource::CHECK2_HAPPEN;
        }
    }

    return InterferenceSource::NONE;
}

z3::expr SemanticInterferencePredicate::happening_value(ExprID var_eid,
                                                        const Action& a1,
                                                        const Action& a2) const {
    // Function of compositions (Def 5 of Bofill et al. 2016): the value of var_eid
    // after the *parallel* happening of {a1, a2}. Compose ONLY the assignments to
    // var_eid, so any other variable an assignment reads (e.g. a velocity fluent)
    // stays at its pre-state value — unlike apply_action_effects_substitution
    // (composition of functions), which would let one action's write flow into the
    // other's read. Actions are simply commuting here, so the order is irrelevant.
    // Conditional effects compose as ite chains over the running value, which
    // needs no exclusivity assumption — this function always did that correctly,
    // and apply_action_effects_substitution now matches it.
    z3::expr value = grounded_visitor_->convert_from_pool(var_eid, -1);
    for (const Action* act : {&a2, &a1}) {
        for (const Effect& effect : act->effects()) {
            const EffectExpression& ee = effect.effect_expression();
            if (ee.fluent_id() != var_eid) continue;
            z3::expr new_value = convert_effect_to_z3(ee, value);
            if (ee.is_conditional()) {
                z3::expr condition = grounded_visitor_->convert_from_pool(ee.condition_id(), -1);
                value = z3::ite(condition, new_value, value);
            } else {
                value = new_value;
            }
        }
    }
    return value;
}

bool SemanticInterferencePredicate::are_simply_commuting(const Action& a1,
                                                         const Action& a2) const {
    // Simply commuting (Def 3.5): for every variable assigned by BOTH actions, the
    // two assignments commute.
    //
    // A fluent may be written more than once by ONE action — a reflexive
    // grounding of move(?from,?to), or the two arms of a clip-to-capacity idiom
    // — so group the writes per fluent and test every cross pair. Testing pairs
    // is sufficient: writing a1's composed effect on the fluent as
    // F1 = f1_k ∘ … ∘ f1_1 and a2's as F2 = f2_m ∘ … ∘ f2_1, every adjacent swap
    // in F2∘F1 = f2_m…f2_1 f1_k…f1_1 is licensed by one pairwise result, and
    // bubbling every f2 past every f1 yields F1∘F2. So all pairs commuting
    // implies the compositions commute, in both orders — which is what
    // check2_source relies on when it compares the happening against a single
    // serialization.
    //
    // It is conservative where the composition would have cancelled:
    // {increase f 1, decrease f 1} is the identity and does commute with
    // {assign f 5}, though neither individual pair does. That costs
    // parallelism, not soundness.
    //
    // ONLY fluents written by BOTH actions are examined, and that is load-bearing.
    // This function used to return false on sight of a duplicate, before it had
    // looked at a2 at all — reporting one action's internal shape as a verdict
    // about every pair it appeared in, and breaking the unwritten invariant
    //     interference(a, b)  IMPLIES  a and b share a fluent.
    // The bogus edges were invisible to the search, which only asks about pairs
    // in PropagatorSharedState::potential_interferers, but visible to plan
    // extraction, which asks about all n² ordered pairs. Being symmetric they
    // formed 2-cycles, and Graph::topological_sort has no cycle detection, so it
    // silently emitted a non-topological order — an invalid plan out of a sound
    // model. See DuplicateEffectPass's header for the full chain.
    std::unordered_map<ExprID, std::vector<const EffectExpression*>> a1_effects;
    std::unordered_map<ExprID, std::vector<const EffectExpression*>> a2_effects;

    for (const Effect& effect : a1.effects()) {
        a1_effects[effect.effect_expression().fluent_id()]
            .push_back(&effect.effect_expression());
    }
    for (const Effect& effect : a2.effects()) {
        a2_effects[effect.effect_expression().fluent_id()]
            .push_back(&effect.effect_expression());
    }

    for (const auto& [var_eid, a1_writes] : a1_effects) {
        auto a2_it = a2_effects.find(var_eid);
        if (a2_it == a2_effects.end()) continue;  // not shared: irrelevant here
        for (const EffectExpression* e1 : a1_writes) {
            for (const EffectExpression* e2 : a2_it->second) {
                if (!assignments_commute(*e1, *e2, var_eid)) {
                    return false;
                }
            }
        }
    }

    return true;
}

bool SemanticInterferencePredicate::assignments_commute(const EffectExpression& eff1,
                                                        const EffectExpression& eff2,
                                                        ExprID var_eid) const {
    // Def 3.3: two assignments to x commute iff T ⊨ (exp2{x→exp1} = exp1{x→exp2}).
    z3::expr var_z3 = grounded_visitor_->convert_from_pool(var_eid, -1);

    z3::expr exp1 = convert_effect_to_z3(eff1, var_z3);
    z3::expr exp2 = convert_effect_to_z3(eff2, var_z3);

    z3::expr_vector from_vec(*z3_context_);
    z3::expr_vector to_vec1(*z3_context_);
    z3::expr_vector to_vec2(*z3_context_);

    from_vec.push_back(var_z3);
    to_vec1.push_back(exp1);
    to_vec2.push_back(exp2);

    z3::expr exp2_substituted = exp2.substitute(from_vec, to_vec1);  // exp2{x→exp1}
    z3::expr exp1_substituted = exp1.substitute(from_vec, to_vec2);  // exp1{x→exp2}

    z3::expr non_commute = (exp2_substituted != exp1_substituted);

    z3_solver_->push();
    z3_solver_->add(non_commute);
    z3::check_result result = z3_solver_->check();
    z3_solver_->pop();

    return (result == z3::unsat);  // commute iff non-commutativity is unsatisfiable
}

z3::expr SemanticInterferencePredicate::convert_precondition_to_z3(const Action& action) const {
    if (!action.has_precondition()) {
        return z3_context_->bool_val(true);
    }
    return grounded_visitor_->convert_from_pool(action.precondition_id(), -1);
}

z3::expr SemanticInterferencePredicate::convert_effect_to_z3(const EffectExpression& effect,
                                                            const z3::expr& base_var_z3) const {
    switch (effect.kind()) {
        case EffectExpression::Kind::ASSIGN:
            return grounded_visitor_->convert_from_pool(effect.value_id(), -1);
        case EffectExpression::Kind::INCREASE:
            return base_var_z3 + grounded_visitor_->convert_from_pool(effect.value_id(), -1);
        case EffectExpression::Kind::DECREASE:
            return base_var_z3 - grounded_visitor_->convert_from_pool(effect.value_id(), -1);
    }
    throw std::runtime_error("Unknown effect kind");
}

} // namespace rantanplan
