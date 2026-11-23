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
        // sequential — which is exactly PDDL 2.1 for the writes of ONE action.
        // The result is that action's assignment to the fluent, as one term.
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

    return result != z3::unsat;  // an undecided check counts as interference
}

InterferenceSource SemanticInterferencePredicate::check2_source(const Action& a1,
                                                               const Action& a2) const {
    // Condition 2, corrected: a1 and a2 are not simply commuting, or a1 can change
    // the value that the term of an assignment of a2 to x has, for SOME value of x.
    // x is replaced by a fresh constant ξ, which no precondition constrains.
    //
    // The published condition (Bofill et al. 2016, Def 7) compared the pair's
    // happening with one serialization, i.e. the term at the single value x has
    // after a1. In a step the term of a2 is read after the other writes to x as
    // well, and a precondition of a1 can hide a dependence at that one value:
    // with a1 = <z = 0, y := y+1> and two writers of z := z(1+y)+1, each pair
    // passes the published check and the step of the three reaches z = 3 where
    // the happening says 2. With ξ, a1 affects both writers. On terms whose
    // dependence on the other variables does not change with x (every linear
    // term, every term free of x) the two conditions coincide.
    if (!are_simply_commuting(a1, a2)) {
        return InterferenceSource::CHECK2_UNCOMM;
    }

    z3::expr a1_pre = convert_precondition_to_z3(a1);
    z3::expr a2_pre = convert_precondition_to_z3(a2);

    std::unordered_set<ExprID> written_by_a2;
    for (const Effect& effect : a2.effects()) {
        written_by_a2.insert(effect.effect_expression().fluent_id());
    }

    for (ExprID var_eid : written_by_a2) {
        z3::expr var_z3 = grounded_visitor_->convert_from_pool(var_eid, -1);
        z3::expr_vector from(*z3_context_), to(*z3_context_);
        from.push_back(var_z3);
        to.push_back(z3_context_->constant("rtp!xi", var_z3.get_sort()));

        // a2's assignment to x as one term, read at x = ξ; then the same after a1.
        z3::expr term = apply_action_effects_substitution(a2, var_z3).substitute(from, to);
        z3::expr term_after_a1 = apply_action_effects_substitution(a1, term);
        if (z3::eq(term, term_after_a1)) {
            continue;  // a1 writes nothing that the term reads
        }

        z3_solver_->push();
        z3_solver_->add(a1_pre && a2_pre && term != term_after_a1);
        z3::check_result result = z3_solver_->check();
        z3_solver_->pop();

        if (result != z3::unsat) {  // an undecided check counts as interference
            return InterferenceSource::CHECK2_HAPPEN;
        }
    }

    return InterferenceSource::NONE;
}

bool SemanticInterferencePredicate::are_simply_commuting(const Action& a1,
                                                         const Action& a2) const {
    // Simply commuting (Def 4): for every variable assigned by BOTH actions, the
    // two assignments commute.
    //
    // Each action's assignment to x is taken as ONE term: its writes to x
    // composed as apply_action_effects_substitution does (several writes per
    // fluent — a reflexive grounding of move(?from,?to), the two arms of a
    // clip-to-capacity idiom — and conditional writes as ite terms). Testing the
    // two terms is Def 3 itself, guards included, so it also covers a guard that
    // reads x, which a test on the bare effects would not see.
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
    // model.
    std::unordered_set<ExprID> written_by_a1;
    for (const Effect& effect : a1.effects()) {
        written_by_a1.insert(effect.effect_expression().fluent_id());
    }

    std::unordered_set<ExprID> checked;
    for (const Effect& effect : a2.effects()) {
        ExprID var_eid = effect.effect_expression().fluent_id();
        if (!written_by_a1.count(var_eid) || !checked.insert(var_eid).second) continue;
        z3::expr var_z3 = grounded_visitor_->convert_from_pool(var_eid, -1);
        if (!terms_commute(apply_action_effects_substitution(a1, var_z3),
                           apply_action_effects_substitution(a2, var_z3), var_z3)) {
            return false;
        }
    }

    return true;
}

bool SemanticInterferencePredicate::terms_commute(const z3::expr& exp1, const z3::expr& exp2,
                                                  const z3::expr& var_z3) const {
    // Def 3: two assignments to x commute iff T ⊨ (exp2{x→exp1} = exp1{x→exp2}).
    z3::expr_vector from_vec(*z3_context_);
    z3::expr_vector to_vec1(*z3_context_);
    z3::expr_vector to_vec2(*z3_context_);

    from_vec.push_back(var_z3);
    to_vec1.push_back(exp1);
    to_vec2.push_back(exp2);

    z3::expr exp2_substituted = z3::expr(exp2).substitute(from_vec, to_vec1);  // exp2{x→exp1}
    z3::expr exp1_substituted = z3::expr(exp1).substitute(from_vec, to_vec2);  // exp1{x→exp2}
    if (z3::eq(exp2_substituted, exp1_substituted)) return true;

    z3_solver_->push();
    z3_solver_->add(exp2_substituted != exp1_substituted);
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
