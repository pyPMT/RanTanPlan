#pragma once

#include "interference_analysis.hpp"
#include "semantic_interference_predicate.hpp"

namespace rantanplan {

/**
 * @brief Eager semantic interference analysis.
 *
 * Precomputes the full interference graph up front using the semantic definition
 * (SemanticInterferencePredicate), then discards the Z3 infrastructure. Provides
 * O(1) graph queries (get_neighbours / get_interference_graph) at the cost of an
 * O(n²) build. Same definition as the lazy SemanticInterferenceAnalysis — only the
 * materialization strategy differs.
 */
class EagerSemanticInterferenceAnalysis : public InterferenceGraphAnalysis {
public:
    explicit EagerSemanticInterferenceAnalysis(const Problem& problem);
    EagerSemanticInterferenceAnalysis() = default;

    void initialize(const Problem& problem) override;

    bool has_interference(const Action& a1, const Action& a2) const override;
    bool has_interference(int node_id1, int node_id2) const override;
    std::vector<const Action*> topological_sort_actions(const std::vector<const Action*>& actions) const override;

    const Graph& get_interference_graph() const override;
    const std::vector<int>& get_neighbours(int node_id) const override;
    void output_interference_graph_dot(const std::string& filename = "interference.dot") const override;

private:
    Graph interference_graph_;  // pre-computed directional interferences

    /// Populate interference_graph_ by querying the predicate over all ordered pairs.
    void build_interference_graph(const SemanticInterferencePredicate& predicate);
};

} // namespace rantanplan
