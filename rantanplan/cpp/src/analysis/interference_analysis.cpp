#include "interference_analysis.hpp"
#include <map>
#include <utility>

namespace rantanplan {


void InterferenceOracle::analyze_all_actions() {
    if (!problem_) {
        return;
    }

    // Clear any existing analysis
    action_analysis_.clear();

    // Analyze each action and store the results
    for (const Action& action : problem_->actions()) {
        action_analysis_[action] = analyze_action(action);
    }
}

InterferenceOracle::ActionAnalysis InterferenceOracle::analyze_action(const Action& action) const {
    ActionAnalysis analysis;
    // Analyze preconditions and effects
    analyze_preconditions(action, analysis);
    analyze_effects(action, analysis);
    return analysis;
}

void InterferenceOracle::analyze_preconditions(const Action& action, ActionAnalysis& analysis) const {
    if (!action.has_precondition()) {
        return;
    }

    FluentPolarityCollector collector(*problem_);
    collector.collect_from_id(action.precondition_id());

    using Polarity = FluentPolarityCollector::Polarity;
    for (const auto& [eid, polarity] : collector.get_boolean_fluents()) {
        if (polarity != Polarity::NEGATIVE) analysis.positive_boolean_preconditions.insert(eid);
        if (polarity != Polarity::POSITIVE) analysis.negative_boolean_preconditions.insert(eid);
    }
    for (ExprID eid : collector.get_numeric_fluents()) {
        analysis.numeric_preconditions.insert(eid);
    }
}

void InterferenceOracle::analyze_effects(const Action& action, ActionAnalysis& analysis) const {
    const ExprPool& pool = problem_->pool();

    // Every fluent `expr` reads, boolean and numeric alike, into `out`.
    auto add_reads = [&](ExprID expr, std::unordered_set<ExprID>& out) {
        FluentPolarityCollector collector(*problem_);
        collector.collect_from_id(expr);
        for (const auto& [eid, _] : collector.get_boolean_fluents()) out.insert(eid);
        for (ExprID eid : collector.get_numeric_fluents()) out.insert(eid);
    };

    for (const Effect& effect : action.effects()) {
        const EffectExpression& eff_expr = effect.effect_expression();
        ExprID fluent_eid = eff_expr.fluent_id();
        ExprID value_eid = eff_expr.value_id();
        analysis.all_effects.insert(fluent_eid);

        if (effect.is_conditional()) {
            add_reads(eff_expr.condition_id(), analysis.conditional_effect_fluents);
        }

        if (effect.kind() == EffectExpression::Kind::ASSIGN && problem_->is_bool_type(fluent_eid)) {
            if (pool.is_constant(value_eid) && pool.payload_is_bool(value_eid)) {
                if (pool.payload_bool(value_eid)) {
                    analysis.positive_boolean_effects.insert(fluent_eid);
                } else {
                    analysis.negative_boolean_effects.insert(fluent_eid);
                }
            } else {
                // A computed value (p := q) can make the fluent true or false.
                analysis.positive_boolean_effects.insert(fluent_eid);
                analysis.negative_boolean_effects.insert(fluent_eid);
                add_reads(value_eid, analysis.effect_value_dependencies);
            }
        } else {
            analysis.numeric_effects.insert(fluent_eid);
            add_reads(value_eid, analysis.effect_value_dependencies);
        }
    }
}

bool InterferenceOracle::actions_interfere(const Action& a1, const Action& a2) const {
    // Get analysis for both actions
    auto it1 = action_analysis_.find(a1);
    auto it2 = action_analysis_.find(a2);

    if (it1 == action_analysis_.end() || it2 == action_analysis_.end()) {
        return false;
    }

    const ActionAnalysis& analysis_a1 = it1->second;
    const ActionAnalysis& analysis_a2 = it2->second;

    // Helper lambda to check if two sets have non-empty intersection
    auto has_intersection = [](const std::unordered_set<ExprID>& set1,
                              const std::unordered_set<ExprID>& set2) -> bool {
        for (const auto& elem : set1) {
            if (set2.find(elem) != set2.end()) {
                return true;
            }
        }
        return false;
    };

    // 1. Check if effects of a1 can prevent execution of a2
    if (has_intersection(analysis_a1.positive_boolean_effects, analysis_a2.negative_boolean_preconditions) ||
        has_intersection(analysis_a1.negative_boolean_effects, analysis_a2.positive_boolean_preconditions) ||
        has_intersection(analysis_a1.numeric_effects, analysis_a2.numeric_preconditions)) {
        return true;
    }

    // 2. Check if effects of a1 can affect the conditions of a2's conditional effects
    if (has_intersection(analysis_a1.all_effects, analysis_a2.conditional_effect_fluents)) {
        return true;
    }

    // 3. Check if effects of a1 can affect the value one of a2's effects writes
    if (has_intersection(analysis_a1.all_effects, analysis_a2.effect_value_dependencies)) {
        return true;
    }

    // 4. Check if effects of a1 and a2 interfere (same fluents modified differently or both modified)
    if (has_intersection(analysis_a1.positive_boolean_effects, analysis_a2.negative_boolean_effects) ||
        has_intersection(analysis_a1.negative_boolean_effects, analysis_a2.positive_boolean_effects) ||
        has_intersection(analysis_a1.numeric_effects, analysis_a2.numeric_effects)) {
        return true;
    }

    return false;
}

std::vector<InterferenceOracle::InterferenceUnit> InterferenceOracle::interference_units() const {
    // The units follow the tests of actions_interfere. A write of each kind
    // affects the preconditions it can falsify (test 1) and the writes it
    // conflicts with (test 4). A write of any kind affects the fluent's reads
    // in effect conditions and effect values (tests 2 and 3), as all_effects
    // is the union of the three effect sets. For a fluent p:
    //
    //     kind          writers               affected
    //     made true     write p true          need ~p, write p false, read p
    //     made false    write p false         need p,  write p true,  read p
    //     numeric       write p               have p in a precondition, write p,
    //                                         read p
    //
    // "Read p": p occurs in an effect condition or an effect value. A
    // computed write p := q writes p both true and false, so it is in both
    // Boolean units.
    enum Kind { MadeTrue, MadeFalse, Numeric };
    // Ordered, so that the units, and with them the clauses and the chain
    // variables, come in the same order on every run.
    std::map<std::pair<ExprID, Kind>, InterferenceUnit> units;

    for (const Action& action : problem_->actions()) {
        const ActionAnalysis& a = action_analysis_.at(action);
        const int id = action.id();
        // Each set holds a fluent once, so an action is a unit's writer once.
        for (ExprID f : a.positive_boolean_effects) units[{f, MadeTrue}].writers.push_back(id);
        for (ExprID f : a.negative_boolean_effects) units[{f, MadeFalse}].writers.push_back(id);
        for (ExprID f : a.numeric_effects) units[{f, Numeric}].writers.push_back(id);

        // An action can be affected through several of its sets. All of
        // them are added here, one action after another, so a repeat is the
        // last entry of the list.
        auto affected_by = [&](ExprID f, Kind kind) {
            std::vector<int>& affected = units[{f, kind}].affected;
            if (affected.empty() || affected.back() != id) affected.push_back(id);
        };
        for (ExprID f : a.negative_boolean_preconditions) affected_by(f, MadeTrue);
        for (ExprID f : a.positive_boolean_preconditions) affected_by(f, MadeFalse);
        for (ExprID f : a.numeric_preconditions) affected_by(f, Numeric);
        for (const auto* reads : {&a.conditional_effect_fluents, &a.effect_value_dependencies}) {
            for (ExprID f : *reads) {
                for (Kind kind : {MadeTrue, MadeFalse, Numeric}) affected_by(f, kind);
            }
        }
        for (ExprID f : a.negative_boolean_effects) affected_by(f, MadeTrue);
        for (ExprID f : a.positive_boolean_effects) affected_by(f, MadeFalse);
        for (ExprID f : a.numeric_effects) affected_by(f, Numeric);
    }

    // A unit without writers or without affected actions forbids nothing.
    std::vector<InterferenceUnit> result;
    for (auto& [key, unit] : units) {
        unit.fluent = key.first;
        unit.numeric = key.second == Numeric;
        if (!unit.writers.empty() && !unit.affected.empty()) result.push_back(std::move(unit));
    }
    return result;
}

std::unordered_set<ExprID> InterferenceOracle::plain_boolean_fluents() const {
    // Over a plain fluent p, a write that makes p false (true) affects an
    // action exactly when that action needs p (not p) or writes p true
    // (false), whatever the state.
    const ExprPool& pool = problem_->pool();
    std::unordered_set<ExprID> written, not_plain;
    auto mark_reads = [&](ExprID e) {
        if (!e.valid()) return;
        FluentPolarityCollector collector(*problem_);
        collector.collect_from_id(e);
        for (const auto& [f, _] : collector.get_boolean_fluents()) not_plain.insert(f);
    };
    auto is_atom = [&](ExprID e) {
        const ExprKind k = pool.kind(e);
        return (k == ExprKind::FLUENT_SYMBOL || k == ExprKind::STATE_VARIABLE) && problem_->is_bool_type(e);
    };
    for (const Action& action : problem_->actions()) {
        if (action.has_precondition()) {
            std::vector<ExprID> conjuncts{action.precondition_id()};
            while (!conjuncts.empty()) {
                const ExprID e = conjuncts.back();
                conjuncts.pop_back();
                if (is_atom(e)) continue;
                if (pool.is_function_application(e)) {
                    auto args = pool.arguments(e);
                    if (pool.is_and(e)) {
                        for (ExprID a : args) conjuncts.push_back(a);
                        continue;
                    }
                    if (pool.is_not(e) && args.size() == 1 && is_atom(args[0])) continue;
                }
                mark_reads(e);
            }
        }
        for (const Effect& effect : action.effects()) {
            const EffectExpression& expr = effect.effect_expression();
            if (effect.is_conditional()) mark_reads(expr.condition_id());
            mark_reads(expr.value_id());  // a constant reads nothing
            const ExprID f = expr.fluent_id();
            if (!problem_->is_bool_type(f)) continue;
            written.insert(f);
            const ExprID v = expr.value_id();
            if (effect.is_conditional() || effect.kind() != EffectExpression::Kind::ASSIGN ||
                !(pool.is_constant(v) && pool.payload_is_bool(v))) {
                not_plain.insert(f);
            }
        }
    }
    std::unordered_set<ExprID> plain;
    for (ExprID f : written) {
        if (!not_plain.count(f)) plain.insert(f);
    }
    return plain;
}

} // namespace rantanplan
