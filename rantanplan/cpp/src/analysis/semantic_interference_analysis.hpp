#pragma once

#include "interference_analysis.hpp"
#include "semantic_interference_predicate.hpp"
#include "../util/hash_utils.hpp"
#include <memory>
#include <unordered_map>

namespace rantanplan {

/**
 * @brief Lazy semantic interference oracle.
 *
 * On-demand materialization of the semantic interference definition
 * (SemanticInterferencePredicate): computes `has_interference` per pair when
 * asked and caches the result. Being an oracle (not a graph analysis), it has no
 * whole-graph query surface — see EagerSemanticInterferenceAnalysis for that.
 */
class SemanticInterferenceAnalysis : public InterferenceOracle {
public:
    explicit SemanticInterferenceAnalysis(const Problem& problem);
    SemanticInterferenceAnalysis() = default;

    void initialize(const Problem& problem) override;

    bool has_interference(const Action& a1, const Action& a2) const override;
    bool has_interference(int node_id1, int node_id2) const override;
    std::vector<const Action*> topological_sort_actions(const std::vector<const Action*>& actions) const override;

    /// Source of interference for a previously-queried pair (NONE if not queried).
    InterferenceSource get_interference_source(int source_id, int target_id) const;

private:
    std::unique_ptr<SemanticInterferencePredicate> predicate_;

    // Cache of the directional result-with-source, keyed by (a1.id(), a2.id()).
    mutable std::unordered_map<std::pair<int, int>, InterferenceSource, PairHash> source_cache_;
};

} // namespace rantanplan
