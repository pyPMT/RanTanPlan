#pragma once

#include "../problem/problem.hpp"
#include "../smtcore/grounded_encoding_visitor.hpp"
#include "../smtcore/smt_types.hpp"
#include "../smtcore/smt_variable_factory.hpp"
#include "../smtcore/term_builder.hpp"
#include <memory>
#include <unordered_map>

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
///   CONFLICTING_COND_EFFECTS:  conservative fallback for non-exclusive
///                              conditional effects on the same fluent.
enum class InterferenceSource {
    NONE,
    CHECK1,
    CHECK2_UNCOMM,
    CHECK2_HAPPEN,
    CONFLICTING_COND_EFFECTS
};

/**
 * @brief The semantic interference *definition* (Bofill et al. 2016).
 *
 * Computes the directional "affects" relation between two actions via cvc5 SMT
 * queries. This is the single home of that logic; it is wrapped by the
 * *materialization* strategy:
 *   - SemanticInterferenceAnalysis (lazy oracle: on-demand + cache)
 *
 * The predicate owns the SMT infrastructure and is otherwise stateless apart
 * from a per-action cache for the (cheap) conflicting-conditional-effects
 * screen. Result caching / graph building is the wrappers' responsibility.
 *
 * CONTRACT (interference sees what the visitor sees): preconditions and
 * effects are converted into one shared TermManager at pseudo-timestep -1 via
 * the same GroundedEncodingVisitor the encoders use — whatever that visitor
 * cannot convert, interference cannot analyse either. Corollaries: (a) a
 * structured feature lowered to ground scalar cells before grounding is
 * automatically analysed soundly here, with no new code; (b) any future
 * symbolic residue (per-action decision variables) needs action-scoped
 * variable naming in this shared TermManager, or two actions' variables unify
 * at timestep -1 and CHECK1/CHECK2 silently miss interference.
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
    // Declaration order is destruction order reversed: the TermManager must
    // outlive everything built from it.
    mutable std::unique_ptr<cvc5::TermManager> z3_context_;
    mutable std::unique_ptr<cvc5::Solver> z3_solver_;
    mutable std::unique_ptr<TermBuilder> tb_;
    mutable std::unique_ptr<SmtVariableFactory> z3_variable_factory_;
    mutable std::unique_ptr<GroundedEncodingVisitor> grounded_visitor_;

    // Per-action cache for the conflicting-conditional-effects screen.
    mutable std::unordered_map<int, bool> conflicting_effects_cache_;

    // Definition helpers (see the .cpp for the correspondence to the paper).
    Term convert_precondition_to_z3(const Action& action) const;
    Term apply_action_effects_substitution(const Action& action, const Term& target_expr) const;
    bool check1(const Action& a1, const Action& a2) const;
    InterferenceSource check2_source(const Action& a1, const Action& a2) const;
    bool are_simply_commuting(const Action& a1, const Action& a2) const;
    bool assignments_commute(const EffectExpression& eff1, const EffectExpression& eff2, ExprID var_eid) const;
    Term happening_value(ExprID var_eid, const Action& a1, const Action& a2) const;
    Term convert_effect_to_z3(const EffectExpression& effect, const Term& base_var_z3) const;
    bool has_conflicting_conditional_effects(const Action& action) const;
};

} // namespace rantanplan
