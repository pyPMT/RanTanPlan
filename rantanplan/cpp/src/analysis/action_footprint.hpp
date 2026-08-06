#pragma once

#include "../problem/problem.hpp"
#include "../problem/visitors/fluent_collector.hpp"

#include <unordered_set>

namespace rantanplan {

/**
 * @brief Every fluent an action touches — the single definition of "footprint".
 *
 * WHY THIS EXISTS AS ONE FUNCTION. It has exactly one caller today:
 *
 *   - SemanticInterferencePredicate::affects_source, which uses it to answer
 *     NONE without an SMT query for pairs that cannot possibly interfere.
 *
 * It stays a free function rather than folding into that class because it had a
 * second caller and will again. PropagatorSharedState::build_footprint_index
 * built the `potential_interferers` adjacency for ExistsCycleModule's DFS from
 * the same notion; those sources were deleted at M4 (see strategy_registry.cpp)
 * and the THEORY_PLAN port that replaces them needs the same predicate. Whoever
 * writes it MUST call this rather than recompute a footprint — that is the exact
 * drift this function exists to prevent, and the paragraphs below are why.
 *
 * The two used to disagree, and the interference-side one did not exist at all,
 * so the filter was a caller-side optimisation whose soundness nobody checked.
 * It rests on
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
    // Footprints are compared by intersection, so they must be keyed by GROUND
    // ATOM, not by node: interning distinguishes a type_id the Z3 variable name
    // does not, so two ids can denote one atom. Comparing raw ids would then see
    // two actions as disjoint when they touch the same fluent, and the filter
    // would skip a check it should have run. See ExprPool's "Ground-atom
    // identity" comment; pool.aliased_state_variables reports whether any
    // aliasing exists in practice.
    const ExprPool& pool = problem.pool();
    auto atom = [&pool](ExprID id) { return pool.canonical_state_variable(id); };

    for (const Effect& effect : action.effects()) {
        const EffectExpression& e = effect.effect_expression();
        footprint.insert(atom(e.fluent_id()));        // written
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

    for (ExprID fid : collector.get_fluents()) footprint.insert(atom(fid));
    return footprint;
}

} // namespace rantanplan
