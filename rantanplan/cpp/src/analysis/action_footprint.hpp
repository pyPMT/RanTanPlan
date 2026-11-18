#pragma once

#include "../problem/problem.hpp"
#include "../problem/visitors/fluent_collector.hpp"

#include <unordered_set>

namespace rantanplan {

/**
 * @brief Every fluent an action touches — the single definition of "footprint".
 *
 * WHY THIS EXISTS AS ONE FUNCTION. Two places need it and they must not drift:
 *
 *   - PropagatorSharedState::build_footprint_index, which turns it into the
 *     `potential_interferers` adjacency used by ExistsCycleModule's DFS.
 *   - SemanticInterferencePredicate::affects_source, which uses it to answer
 *     NONE without an SMT query for pairs that cannot possibly interfere.
 *
 * They used to disagree, and the second did not exist at all, so the filter was
 * a caller-side optimisation whose soundness nobody checked. It rests on
 *
 *     interference(a, b)  IMPLIES  a and b share a fluent
 *
 * which is only true if "share a fluent" means everything either action reads
 * OR writes. An earlier version collected written fluents and PRECONDITION
 * fluents only, and that is unsound: a fluent read on an effect's right-hand
 * side, or in a conditional guard, was invisible. Measured consequence, from
 * pddl/test/fo-sailing:
 *
 *     (:action accelerate     :effect (increase (v ?b) 1.0))
 *     (:action go_north_east  :effect (increase (x ?b) (* (v ?b) 1.5))
 *                                     (increase (y ?b) (* (v ?b) 1.5)))
 *
 * accelerate writes v; go_north_east READS v, but only inside its effect
 * values. Under the old definition the footprints were {v} and {x, y, dummy} —
 * disjoint — so ExistsCycleModule never asked the oracle about the pair and was
 * free to co-schedule them, even though the oracle reports CHECK2_HAPPEN: the
 * parallel happening evaluates v at the pre-state while a serialisation lets
 * accelerate's write flow into go_north_east's delta. A debug assertion over
 * pddl/test found the same shape in five domains (fo-sailing, refinery,
 * obj-logistics, incremental-cascade, incremental-cascade-3).
 *
 * Widening is the safe direction: a larger footprint means more pairs reach the
 * oracle and none are skipped, so it can cost time but not soundness.
 *
 * NOTE the asymmetry with the interference DEFINITION itself. This is a
 * necessary condition only — sharing a fluent does not imply interfering; that
 * is what the oracle decides. Reads and writes are deliberately not
 * distinguished here, because a read/write pair in either direction is exactly
 * the hazard the filter must not hide.
 */
inline std::unordered_set<ExprID> compute_action_footprint(const Problem& problem,
                                                           const Action& action) {
    std::unordered_set<ExprID> footprint;
    FluentCollector collector(problem);
    // Footprints are compared by intersection, keyed by fluent id: one ground
    // atom has one id (enforced in preprocess(); see ExprPool's "Ground-atom
    // identity" comment).

    for (const Effect& effect : action.effects()) {
        const EffectExpression& e = effect.effect_expression();
        footprint.insert(e.fluent_id());              // written
        collector.collect_from_id(e.value_id());      // read on the RHS
        // Deliberately NOT gated on is_conditional(). collect_from_id ignores an
        // invalid id, and a guard normalised away (EXPR_NULL, or the TRUE
        // constant — see EffectExpression's four-arg constructor) contains no
        // state variables, so collecting unconditionally is free. Gating here
        // would make the footprint depend on that normalisation, and a filter
        // this load-bearing should not have a coupling that subtle.
        collector.collect_from_id(e.condition_id());  // read in the guard
    }
    if (action.has_precondition()) {
        collector.collect_from_id(action.precondition_id());
    }

    for (ExprID fid : collector.get_fluents()) footprint.insert(fid);
    return footprint;
}

} // namespace rantanplan
