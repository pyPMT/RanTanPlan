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
 * asked and caches the result.
 */
class SemanticInterferenceAnalysis : public InterferenceOracle {
public:
    explicit SemanticInterferenceAnalysis(const Problem& problem);

    bool has_interference(const Action& a1, const Action& a2) const override;
    bool has_interference(int node_id1, int node_id2) const override;
    std::vector<const Action*> topological_sort_actions(const std::vector<const Action*>& actions) const override;

private:
    const Problem& problem_;
    std::unique_ptr<SemanticInterferencePredicate> predicate_;

    // Cache of the directional result, keyed by (a1.id(), a2.id()).
    mutable std::unordered_map<std::pair<int, int>, bool, PairHash> cache_;
};

} // namespace rantanplan
