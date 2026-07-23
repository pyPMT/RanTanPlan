#include "semantic_interference_predicate.hpp"
#include "../util/logger.hpp"
#include <stdexcept>
#include <unordered_set>

namespace rantanplan {

SemanticInterferencePredicate::SemanticInterferencePredicate(const Problem& problem) {
    z3_context_ = std::make_unique<cvc5::TermManager>();
    z3_solver_ = std::make_unique<cvc5::Solver>(*z3_context_);
    tb_ = std::make_unique<TermBuilder>(*z3_context_);
    tb_->set_problem(&problem);
    z3_variable_factory_ = std::make_unique<SmtVariableFactory>(*z3_context_);
    z3_variable_factory_->set_problem(&problem);
    grounded_visitor_ = std::make_unique<GroundedEncodingVisitor>(
        *z3_context_, &problem, z3_variable_factory_.get());
}

InterferenceSource SemanticInterferencePredicate::affects_source(const Action& a1,
                                                                 const Action& a2) const {
    // Definition 3.9: a1 affects a2 if it can prevent a2's execution (check1) or
    // its effects don't compose commutatively with a2's (check2). If a1 has
    // non-exclusive conditional effects, the ite substitution may misrepresent the
    // combined effect, so report interference conservatively.
    if (has_conflicting_conditional_effects(a1)) {
        return InterferenceSource::CONFLICTING_COND_EFFECTS;
    }
    if (check1(a1, a2)) {
        return InterferenceSource::CHECK1;
    }
    return check2_source(a1, a2);  // CHECK2_UNCOMM / CHECK2_HAPPEN / NONE
}

Term SemanticInterferencePredicate::apply_action_effects_substitution(
    const Action& action, const Term& target_expr) const {
    // σ(action, expr): apply the action's effects to target_expr by substitution.
    // Multiple conditional effects on the same fluent are composed into a nested
    // ite chain (Term::substitute() would otherwise drop duplicate keys).
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

    TermVec from_vector;
    TermVec to_vector;

    for (ExprID fid : fluent_order) {
        Term fluent_z3 = grounded_visitor_->convert_from_pool(fid, -1);
        Term composed = fluent_z3;  // start with the unchanged fluent

        // Nested ite chain — correct when conditions are mutually exclusive (the
        // common PDDL pattern); non-exclusive cases are screened out upstream by
        // has_conflicting_conditional_effects().
        for (const Effect* effect : effects_by_fluent[fid]) {
            const EffectExpression& eff_expr = effect->effect_expression();
            Term new_value_z3 = convert_effect_to_z3(eff_expr, fluent_z3);

            if (effect->is_conditional()) {
                Term condition_z3 = grounded_visitor_->convert_from_pool(
                    effect->effect_expression().condition_id(), -1);
                composed = tb_->mk_ite(condition_z3, new_value_z3, composed);
            } else {
                composed = new_value_z3;  // unconditional overrides all
            }
        }

        from_vector.push_back(fluent_z3);
        to_vector.push_back(composed);
    }

    Term result = target_expr;
    return result.substitute(from_vector, to_vector);
}

bool SemanticInterferencePredicate::check1(const Action& a1, const Action& a2) const {
    // Condition 1 of Def 3.9: pre(a1) ∧ pre(a2) ∧ ¬(pre(a2) σ_a1) satisfiable —
    // can a1's effects falsify a2's precondition? (composition of functions is
    // correct here: a precondition IS evaluated after a1's full effects.)
    if (!a2.has_precondition()) return false;

    Term a1_pre = convert_precondition_to_z3(a1);
    Term a2_pre = convert_precondition_to_z3(a2);

    // Apply a1's effects to a2's precondition.
    Term substituted_expr = apply_action_effects_substitution(a1, a2_pre);

    // If a1 doesn't touch a2's precondition variables, it cannot prevent it.
    // (`Term::operator==` is syntactic equality on hash-consed nodes,
    // returning bool directly — not a formula constructor. Formulas are
    // built via TermBuilder instead.)
    if (a2_pre == substituted_expr) {
        return false;
    }

    Term check1 = tb_->mk_and({a1_pre, a2_pre, tb_->mk_not(substituted_expr)});

    z3_solver_->push();
    z3_solver_->assertFormula(check1);
    cvc5::Result result = z3_solver_->checkSat();
    z3_solver_->pop();

    return result.isSat();
}

InterferenceSource SemanticInterferencePredicate::check2_source(const Action& a1,
                                                               const Action& a2) const {
    // Condition 2 of Def 3.9:
    //   2a. not simply commuting            → unconditional interference
    //   2b. simply commuting but happening ≠ sequential → state-dependent
    if (!are_simply_commuting(a1, a2)) {
        return InterferenceSource::CHECK2_UNCOMM;
    }

    Term a1_pre = convert_precondition_to_z3(a1);
    Term a2_pre = convert_precondition_to_z3(a2);

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
        Term var_after_happening = happening_value(var_eid, a1, a2);

        // Sequential x·σ_a2·σ_a1 ("composition of functions"): a2 then a1.
        Term var_z3 = grounded_visitor_->convert_from_pool(var_eid, -1);
        Term var_after_sequential = var_z3;
        var_after_sequential = apply_action_effects_substitution(a2, var_after_sequential);
        var_after_sequential = apply_action_effects_substitution(a1, var_after_sequential);

        if (var_after_happening == var_after_sequential) {
            continue;
        }

        Term non_commutativity = tb_->mk_and(
            {a1_pre, a2_pre, tb_->mk_distinct(var_after_happening, var_after_sequential)});

        z3_solver_->push();
        z3_solver_->assertFormula(non_commutativity);
        cvc5::Result result = z3_solver_->checkSat();
        z3_solver_->pop();

        if (result.isSat()) {
            return InterferenceSource::CHECK2_HAPPEN;
        }
    }

    return InterferenceSource::NONE;
}

Term SemanticInterferencePredicate::happening_value(ExprID var_eid,
                                                    const Action& a1,
                                                    const Action& a2) const {
    // Function of compositions (Def 5 of Bofill et al. 2016): the value of var_eid
    // after the *parallel* happening of {a1, a2}. Compose ONLY the assignments to
    // var_eid, so any other variable an assignment reads (e.g. a velocity fluent)
    // stays at its pre-state value — unlike apply_action_effects_substitution
    // (composition of functions), which would let one action's write flow into the
    // other's read. Actions are simply commuting here, so the order is irrelevant.
    // Conditional effects compose as ite chains (conflicting ones were screened
    // out by has_conflicting_conditional_effects).
    Term value = grounded_visitor_->convert_from_pool(var_eid, -1);
    for (const Action* act : {&a2, &a1}) {
        for (const Effect& effect : act->effects()) {
            const EffectExpression& ee = effect.effect_expression();
            if (ee.fluent_id() != var_eid) continue;
            Term new_value = convert_effect_to_z3(ee, value);
            if (ee.is_conditional()) {
                Term condition = grounded_visitor_->convert_from_pool(ee.condition_id(), -1);
                value = tb_->mk_ite(condition, new_value, value);
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
    // two assignments commute. If a fluent has multiple effects (paired conditional
    // effects), simple commutativity doesn't apply — report not-simply-commuting.
    std::unordered_map<ExprID, const EffectExpression*> a1_effects;
    std::unordered_map<ExprID, const EffectExpression*> a2_effects;

    for (const Effect& effect : a1.effects()) {
        ExprID fid = effect.effect_expression().fluent_id();
        if (a1_effects.count(fid)) return false;
        a1_effects[fid] = &effect.effect_expression();
    }
    for (const Effect& effect : a2.effects()) {
        ExprID fid = effect.effect_expression().fluent_id();
        if (a2_effects.count(fid)) return false;
        a2_effects[fid] = &effect.effect_expression();
    }

    for (const auto& [var_eid, a1_effect] : a1_effects) {
        auto a2_it = a2_effects.find(var_eid);
        if (a2_it != a2_effects.end()) {
            if (!assignments_commute(*a1_effect, *a2_it->second, var_eid)) {
                return false;
            }
        }
    }

    return true;
}

bool SemanticInterferencePredicate::assignments_commute(const EffectExpression& eff1,
                                                        const EffectExpression& eff2,
                                                        ExprID var_eid) const {
    // Def 3.3: two assignments to x commute iff T ⊨ (exp2{x→exp1} = exp1{x→exp2}).
    Term var_z3 = grounded_visitor_->convert_from_pool(var_eid, -1);

    Term exp1 = convert_effect_to_z3(eff1, var_z3);
    Term exp2 = convert_effect_to_z3(eff2, var_z3);

    TermVec from_vec;
    TermVec to_vec1;
    TermVec to_vec2;

    from_vec.push_back(var_z3);
    to_vec1.push_back(exp1);
    to_vec2.push_back(exp2);

    Term exp2_substituted = exp2.substitute(from_vec, to_vec1);  // exp2{x→exp1}
    Term exp1_substituted = exp1.substitute(from_vec, to_vec2);  // exp1{x→exp2}

    Term non_commute = tb_->mk_distinct(exp2_substituted, exp1_substituted);

    z3_solver_->push();
    z3_solver_->assertFormula(non_commute);
    cvc5::Result result = z3_solver_->checkSat();
    z3_solver_->pop();

    return result.isUnsat();  // commute iff non-commutativity is unsatisfiable
}

Term SemanticInterferencePredicate::convert_precondition_to_z3(const Action& action) const {
    if (!action.has_precondition()) {
        return tb_->bool_val(true);
    }
    return grounded_visitor_->convert_from_pool(action.precondition_id(), -1);
}

Term SemanticInterferencePredicate::convert_effect_to_z3(const EffectExpression& effect,
                                                         const Term& base_var_z3) const {
    switch (effect.kind()) {
        case EffectExpression::Kind::ASSIGN:
            return grounded_visitor_->convert_from_pool(effect.value_id(), -1);
        case EffectExpression::Kind::INCREASE:
            return tb_->add(base_var_z3, grounded_visitor_->convert_from_pool(effect.value_id(), -1));
        case EffectExpression::Kind::DECREASE:
            return tb_->sub(base_var_z3, grounded_visitor_->convert_from_pool(effect.value_id(), -1));
    }
    throw std::runtime_error("Unknown effect kind");
}

bool SemanticInterferencePredicate::has_conflicting_conditional_effects(const Action& action) const {
    auto cache_it = conflicting_effects_cache_.find(action.id());
    if (cache_it != conflicting_effects_cache_.end()) return cache_it->second;

    bool has_conflict = false;

    std::unordered_map<ExprID, std::vector<const Effect*>> cond_by_fluent;
    for (const Effect& effect : action.effects()) {
        if (effect.is_conditional()) {
            cond_by_fluent[effect.effect_expression().fluent_id()].push_back(&effect);
        }
    }

    Term pre = convert_precondition_to_z3(action);

    for (const auto& [fid, effects] : cond_by_fluent) {
        if (effects.size() < 2) continue;
        for (size_t i = 0; i < effects.size() && !has_conflict; ++i) {
            for (size_t j = i + 1; j < effects.size() && !has_conflict; ++j) {
                Term ci = grounded_visitor_->convert_from_pool(
                    effects[i]->effect_expression().condition_id(), -1);
                Term cj = grounded_visitor_->convert_from_pool(
                    effects[j]->effect_expression().condition_id(), -1);

                z3_solver_->push();
                z3_solver_->assertFormula(tb_->mk_and({pre, ci, cj}));
                has_conflict = z3_solver_->checkSat().isSat();
                z3_solver_->pop();
            }
        }
        if (has_conflict) break;
    }

    if (has_conflict) {
        Logger::instance().verbose("Action '" + action.name() +
            "' (id=" + std::to_string(action.id()) +
            ") has non-exclusive conditional effects — forcing interference");
    }

    conflicting_effects_cache_[action.id()] = has_conflict;
    return has_conflict;
}

} // namespace rantanplan
