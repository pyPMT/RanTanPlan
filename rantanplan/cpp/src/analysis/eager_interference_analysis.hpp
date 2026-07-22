#pragma once

#include "interference_analysis.hpp"
#include "syntactic_interference.hpp"
#include "graph.hpp"

namespace rantanplan {

/**
 * @brief Eager syntactic interference oracle.
 *
 * Materializes SyntacticInterference into a complete interference graph at
 * construction: O(n²) pairwise checks up front, O(1) lookups per query.
 */
class EagerInterferenceAnalysis : public InterferenceOracle {
public:
    /**
     * @brief Construct and initialize the analyzer with a planning problem
     * @param problem The planning problem to analyze
     */
    explicit EagerInterferenceAnalysis(const Problem& problem);

    bool has_interference(const Action& a1, const Action& a2) const override;
    bool has_interference(int node_id1, int node_id2) const override;
    std::vector<const Action*> topological_sort_actions(const std::vector<const Action*>& actions) const override;

    /// The materialized interference graph. Concrete accessor (not part of
    /// InterferenceOracle): the encoded exists-step parallelism (`e`) derives
    /// its a-priori fixed-order/SCC mutex clauses from the whole graph.
    const Graph& interference_graph() const { return interference_graph_; }

private:
    const Problem& problem_;
    Graph interference_graph_;

    /// Run the pairwise syntactic checks and record edges in the graph.
    void build_interference_graph(const SyntacticInterference& syntactic);
};

} // namespace rantanplan
