#include "semantic_interference_analysis.hpp"
#include <algorithm>

namespace rantanplan {

SemanticInterferenceAnalysis::SemanticInterferenceAnalysis(const Problem& problem) {
    initialize(problem);
}

void SemanticInterferenceAnalysis::initialize(const Problem& problem) {
    problem_ = &problem;
    action_analysis_.clear();
    source_cache_.clear();
    commute_cache_.clear();
    analyze_all_actions();  // base-class per-action metadata
    predicate_ = std::make_unique<SemanticInterferencePredicate>(problem);
}

bool SemanticInterferenceAnalysis::has_interference(const Action& a1, const Action& a2) const {
    // Directional "affects" (Def 3.9), computed on demand and cached.
    auto key = std::make_pair(a1.id(), a2.id());
    auto it = source_cache_.find(key);
    if (it != source_cache_.end()) {
        return it->second != InterferenceSource::NONE;
    }
    InterferenceSource source = predicate_->affects_source(a1, a2);
    source_cache_[key] = source;
    return source != InterferenceSource::NONE;
}

bool SemanticInterferenceAnalysis::has_interference(int node_id1, int node_id2) const {
    return has_interference(problem_->action(node_id1), problem_->action(node_id2));
}

InterferenceSource SemanticInterferenceAnalysis::get_interference_source(int source_id,
                                                                        int target_id) const {
    auto it = source_cache_.find({source_id, target_id});
    return it != source_cache_.end() ? it->second : InterferenceSource::NONE;
}

bool SemanticInterferenceAnalysis::effects_commute(int u, int v) const {
    auto key = std::make_pair(std::min(u, v), std::max(u, v));
    auto it = commute_cache_.find(key);
    if (it != commute_cache_.end()) return it->second;
    return commute_cache_[key] = predicate_->effects_commute(problem_->action(u), problem_->action(v));
}

std::vector<const Action*> SemanticInterferenceAnalysis::topological_sort_actions(
    const std::vector<const Action*>& actions) const {
    if (actions.size() <= 1) {
        return actions;
    }

    // Collect valid node IDs.
    std::vector<int> node_ids;
    for (const Action* action : actions) {
        if (action->id() >= 0) node_ids.push_back(action->id());
    }
    if (node_ids.empty()) {
        return actions;
    }

    // Build a temporary subgraph from the (cached) directional interferences.
    Graph temp_subgraph(problem_->action_count());
    for (size_t i = 0; i < actions.size(); ++i) {
        for (size_t j = 0; j < actions.size(); ++j) {
            if (i == j) continue;
            if (has_interference(*actions[i], *actions[j])) {
                int node1 = actions[i]->id();
                int node2 = actions[j]->id();
                if (node1 >= 0 && node2 >= 0) {
                    temp_subgraph.add_edge(node1, node2);
                }
            }
        }
    }

    // Topologically sort (execution order) and map node IDs back to actions.
    std::vector<int> sorted_node_ids = temp_subgraph.topological_sort(node_ids);
    std::vector<const Action*> result;
    result.reserve(sorted_node_ids.size());
    for (int node_id : sorted_node_ids) {
        result.push_back(&problem_->action(node_id));
    }
    return result;
}

} // namespace rantanplan
