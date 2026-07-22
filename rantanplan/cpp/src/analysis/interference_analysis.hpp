#pragma once

#include "../problem/problem.hpp"
#include "../problem/action.hpp"
#include <vector>

/**
 * @file interference_analysis.hpp
 * @brief InterferenceOracle: the single runtime interface for interference
 *        queries. Consumers (propagator modules, plan extraction) stay
 *        ignorant of how the answers are computed — precomputed syntactic
 *        graph (EagerInterferenceAnalysis) or on-demand solver-checked
 *        queries (SemanticInterferenceAnalysis).
 *
 * The syntactic check itself is NOT part of this interface: consumers that
 * specifically need the syntactic overapproximation (the abstract suffix
 * encoder) take a SyntacticInterference value directly.
 */
namespace rantanplan {

class InterferenceOracle {
public:
    virtual ~InterferenceOracle() = default;

    /**
     * @brief Check if first action interferes with second action (DIRECTIONAL)
     *
     * IMPORTANT: This checks if a1 interferes with a2, NOT if they interfere with each other.
     * Interference is directional: has_interference(A, B) != has_interference(B, A) in general.
     *
     * Returns true if executing a1 would prevent a2 from executing correctly, or if
     * a1's effects would interfere with a2's preconditions or effects.
     *
     * @param a1 The potentially interfering action
     * @param a2 The potentially interfered-with action
     * @return True if a1 interferes with a2, false otherwise
     */
    virtual bool has_interference(const Action& a1, const Action& a2) const = 0;

    /**
     * @brief Check if first action interferes with second action by node IDs (DIRECTIONAL)
     *
     * Optimized version that works directly with node IDs, avoiding Action object lookups.
     */
    virtual bool has_interference(int node_id1, int node_id2) const = 0;

    /**
     * @brief Sort actions for safe execution order (reverse topological order)
     *
     * IMPORTANT: This returns actions in EXECUTION ORDER, not standard topological order.
     * If action A interferes with action B (A -> B), then B will appear BEFORE A in the result.
     * This ensures that when actions are executed in the returned order, no interference occurs.
     *
     * Example: If we have edges A->B and B->C, the result will be [C, B, A]
     *
     * This method only requires the subgraph between the provided actions,
     * making it compatible with lazy analysis approaches.
     *
     * @param actions The actions to sort
     * @return Actions sorted in safe execution order (reverse topological order)
     */
    virtual std::vector<const Action*> topological_sort_actions(const std::vector<const Action*>& actions) const = 0;
};

} // namespace rantanplan
