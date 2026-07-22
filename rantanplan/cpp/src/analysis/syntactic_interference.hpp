#pragma once

#include "../problem/problem.hpp"
#include "../problem/action.hpp"
#include <unordered_map>
#include <unordered_set>

/**
 * @file syntactic_interference.hpp
 * @brief Standalone syntactic action-interference check (sound overapproximation).
 */
namespace rantanplan {

/**
 * @brief Syntactic action-interference check (sound overapproximation).
 *
 * Precomputes per-action fluent read/write sets at construction and answers
 * the directional "a1 interferes with a2" question by set intersection.
 * May return true for pairs that don't interfere semantically, but never
 * returns false for pairs that do.
 *
 * This is a plain value type, deliberately outside the InterferenceOracle
 * hierarchy. It has two consumers:
 *   - EagerInterferenceAnalysis materializes it into the interference graph
 *     (the "eager syntactic" oracle behind strategy `e`)
 *   - AbstractSuffixEncoder uses it directly for the axiom-6 deferral sets,
 *     which need a cheap sound overapproximation regardless of how precise
 *     the strategy's own interference oracle is
 */
class SyntacticInterference {
public:
    explicit SyntacticInterference(const Problem& problem);

    /**
     * @brief Directional check: does a1 interfere with a2?
     *
     * IMPORTANT: NOT symmetric — interferes(A, B) != interferes(B, A) in general.
     */
    bool interferes(const Action& a1, const Action& a2) const;

private:
    // Per-action fluent read/write sets, computed once at construction.
    struct ActionAnalysis {
        // Precondition analysis
        std::unordered_set<ExprID> positive_boolean_preconditions;  // Boolean fluents required to be true
        std::unordered_set<ExprID> negative_boolean_preconditions;  // Boolean fluents required to be false
        std::unordered_set<ExprID> numeric_preconditions;           // Numeric fluents in preconditions

        // Effect analysis
        std::unordered_set<ExprID> positive_boolean_effects;  // Boolean fluents made true
        std::unordered_set<ExprID> negative_boolean_effects;  // Boolean fluents made false
        std::unordered_set<ExprID> numeric_effects;           // Numeric fluents modified

        // A set of all fluents modified by this action's effects (the union of the previous 3 sets)
        std::unordered_set<ExprID> all_effects;

        // Fluents appearing in the conditions of conditional effects
        std::unordered_set<ExprID> conditional_effect_fluents;

        // Fluents appearing on the RHS of numeric assignments.
        // For example, an effect x' = x + y + y + z^2 would have y and z in this set.
        std::unordered_set<ExprID> numeric_effect_dependencies;
    };

    const Problem& problem_;
    std::unordered_map<Action, ActionAnalysis> action_analysis_;

    ActionAnalysis analyze_action(const Action& action) const;
    void analyze_preconditions(const Action& action, ActionAnalysis& analysis) const;
    void analyze_effects(const Action& action, ActionAnalysis& analysis) const;
};

} // namespace rantanplan
