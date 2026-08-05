#pragma once

#include "../problem/problem.hpp"
#include "../z3core/grounded_encoding_visitor.hpp"
#include "../z3core/z3_variable_factory.hpp"
#include <z3++.h>
#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace rantanplan {

/// Source of interference detected by the semantic definition (Bofill, Espasa,
/// Villaret, "A Semantic Notion of Interference for Planning Modulo Theories",
/// ICAPS 2016; code cites thesis numbering 3.3/3.5/3.9 = paper Defs 3/4/7/8):
///
///   CHECK1:                    Pre_a ∧ Pre_b ∧ ¬(Pre_b σ_a) satisfiable — a can
///                              prevent b's execution (state-dependent).
///   CHECK2_UNCOMM:             not simply commuting — assignments don't commute
///                              (state-independent).
///   CHECK2_HAPPEN:             simply commuting but the parallel happening σ_h
///                              differs from a serialization (state-dependent).
///
/// A fifth source, CONFLICTING_COND_EFFECTS, used to fire when one action had
/// two conditional effects on a fluent whose guards could both hold. It was a
/// screen around a composition bug rather than a property of the pair — and,
/// being a property of a1 alone, it reported interference against every action
/// in the problem, including fluent-disjoint ones. The composition is fixed
/// (see apply_action_effects_substitution) and the source is gone.
enum class InterferenceSource {
    NONE,
    CHECK1,
    CHECK2_UNCOMM,
    CHECK2_HAPPEN
};

/**
 * @brief The semantic interference *definition* (Bofill et al. 2016).
 *
 * Computes the directional "affects" relation between two actions via Z3 SMT
 * queries. This is the single home of that logic; it is wrapped by the two
 * *materialization* strategies:
 *   - SemanticInterferenceAnalysis     (lazy oracle: on-demand + cache)
 *   - EagerSemanticInterferenceAnalysis (eager: precompute the whole graph)
 *
 * The predicate owns the Z3 infrastructure and is otherwise stateless apart from
 * the per-action footprint index it filters on (footprints_, below). Result
 * caching / graph building is the wrappers' responsibility.
 *
 * CONTRACT (interference sees what the visitor sees): preconditions and
 * effects are converted into one shared context at pseudo-timestep -1 via the
 * same GroundedEncodingVisitor the encoders use — whatever that visitor
 * cannot convert, interference cannot analyse either. Corollaries: (a) a
 * structured feature lowered to ground scalar cells before grounding is
 * automatically analysed soundly here, with no new code; (b) any future
 * symbolic residue (per-action decision variables) needs action-scoped
 * variable naming in this shared context, or two actions' variables unify at
 * timestep -1 and CHECK1/CHECK2 silently miss interference.
 */
class SemanticInterferencePredicate {
public:
    explicit SemanticInterferencePredicate(const Problem& problem);

    /// Directional (Def 3.9): does a1 affect a2? NOT symmetric.
    bool affects(const Action& a1, const Action& a2) const {
        return affects_source(a1, a2) != InterferenceSource::NONE;
    }

    /// The affects result together with which check produced it (NONE = no
    /// interference). Computed fresh on each call; wrappers add their own caching.
    InterferenceSource affects_source(const Action& a1, const Action& a2) const;

private:
    mutable std::unique_ptr<z3::context> z3_context_;
    mutable std::unique_ptr<z3::solver> z3_solver_;
    mutable std::unique_ptr<Z3VariableFactory> z3_variable_factory_;
    mutable std::unique_ptr<GroundedEncodingVisitor> grounded_visitor_;

    /// Per-action footprint (compute_action_footprint), built once at
    /// construction. affects_source answers NONE without any SMT query when two
    /// actions' footprints are disjoint, since sharing a fluent is a necessary
    /// condition for interfering.
    ///
    /// This lives HERE, in the oracle, rather than in the callers. It used to be
    /// a caller-side filter in ExistsCycleModule alone, so the search saw a
    /// filtered relation while plan extraction — SemanticInterferenceAnalysis::
    /// topological_sort_actions, which sweeps all n² ordered pairs — saw an
    /// unfiltered one. The two halves of the planner then reasoned about
    /// different graphs: an edge visible only to extraction became a symmetric
    /// 2-cycle, and Graph::topological_sort has no cycle detection, so it
    /// emitted a silent non-order. One filtered oracle makes them agree by
    /// construction, and makes extraction's n² sweep cheap.
    std::unordered_map<int, std::unordered_set<ExprID>> footprints_;
    bool shares_fluent(const Action& a1, const Action& a2) const;

#ifndef NDEBUG
    /// Debug-only guard on the FOOTPRINT DEFINITION, not on the filter.
    ///
    /// Once the filter is inside the oracle, "a positive verdict implies a
    /// shared fluent" is a tautology and checks nothing. The property still
    /// worth testing is the one the filter rests on: that skipping is safe.
    /// So when the footprints are disjoint, debug builds compute the verdict
    /// ANYWAY and assert it is NONE. A firing assertion means
    /// compute_action_footprint is too narrow — which is exactly how the
    /// effect-right-hand-side omission was found.
    void assert_filter_hides_nothing(const Action& a1, const Action& a2) const;
#endif

    // Definition helpers (see the .cpp for the correspondence to the paper).
    z3::expr convert_precondition_to_z3(const Action& action) const;
    z3::expr apply_action_effects_substitution(const Action& action, const z3::expr& target_expr) const;
    bool check1(const Action& a1, const Action& a2) const;
    InterferenceSource check2_source(const Action& a1, const Action& a2) const;
    bool are_simply_commuting(const Action& a1, const Action& a2) const;
    bool assignments_commute(const EffectExpression& eff1, const EffectExpression& eff2, ExprID var_eid) const;
    z3::expr happening_value(ExprID var_eid, const Action& a1, const Action& a2) const;
    z3::expr convert_effect_to_z3(const EffectExpression& effect, const z3::expr& base_var_z3) const;
};

} // namespace rantanplan
