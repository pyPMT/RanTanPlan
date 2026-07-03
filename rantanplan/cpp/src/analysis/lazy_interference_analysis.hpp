#pragma once

#include "interference_analysis.hpp"
#include "../util/hash_utils.hpp"
#include <utility>

namespace rantanplan {

/**
 * @brief Lazy interference analysis implementation
 *
 * Computes action interferences on-demand and caches results. Provides memory-efficient
 * analysis suitable for use with exists and lazy forall propagators that only need
 * edge-based interference checking. Being an oracle (not a graph analysis),
 * it has no whole-graph query surface.
 */
class LazyInterferenceAnalysis : public InterferenceOracle {
public:
    /**
     * @brief Construct and initialize the analyzer with a planning problem
     * @param problem The planning problem to analyze
     */
    explicit LazyInterferenceAnalysis(const Problem& problem);
    
    // Default constructor for cases where problem isn't available at construction time
    LazyInterferenceAnalysis() = default;
    
    void initialize(const Problem& problem) override;
    
    // Direct interference checking methods (fully supported)
    bool has_interference(const Action& a1, const Action& a2) const override;
    bool has_interference(int node_id1, int node_id2) const override;
    std::vector<const Action*> topological_sort_actions(const std::vector<const Action*>& actions) const override;

private:
    
    // Cache for interference results to avoid recomputation
    // Keyed by (action_id_1, action_id_2) pair for correct per-instance caching
    mutable std::unordered_map<std::pair<int,int>, bool, PairHash> interference_cache_;
    
    /**
     * @brief Check if two actions interfere with each other (internal implementation with caching)
     * @param a1 First action
     * @param a2 Second action
     * @return True if actions interfere, false otherwise
     */
    bool compute_interference(const Action& a1, const Action& a2) const;
    
    
};

} // namespace rantanplan