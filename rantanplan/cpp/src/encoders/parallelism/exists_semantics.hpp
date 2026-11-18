#pragma once

#include "parallelism_strategy.hpp"
#include "../../analysis/interference_analysis.hpp"
#include <memory>
#include <utility>
#include <vector>

namespace rantanplan {

/**
 * @brief The fixed step order of exists-step semantics
 *
 * The order is fixed in advance, the same at every step (Section 3.4.4 of the
 * paper cited on ExistsSemantics). An edge u -> v of the interference graph
 * (u affects v) is forbidden when u runs before v. The order runs the
 * strongly connected components one after another, the affected component
 * first for every edge across two of them, so such a pair is never
 * forbidden. A pair with edges both ways is forbidden whatever the order. So
 * the order matters only for the one-way pairs inside a component, and the
 * constructor picks one that runs the affected action first in as many of
 * them as it can.
 */
class ExistsStepOrder {
public:
    explicit ExistsStepOrder(const Graph& graph);

    /// For an edge u -> v (u affects v): whether u and v cannot share a
    /// step, as u runs before v.
    bool forbids(int u, int v) const { return position_[u] < position_[v]; }

private:
    std::vector<int> position_;  ///< action id -> place in the step order
};

/**
 * @brief Exists semantics
 *  Set of actions in a timestep can execute if there exists
 *  at least one order where actions do not interfere with actions further
 *  down the line.
 *
 * Jussi Rintanen, Keijo Heljanko, Ilkka Niemelä,
 * Planning as satisfiability: parallel plans and algorithms for plan search,
 * Artificial Intelligence, Volume 170, Issues 12–13, 2006,
 *
 * encode_parallelism writes one clause for each pair the ExistsStepOrder
 * forbids.
 */
class ExistsSemantics : public ParallelismStrategy {
public:
    /// The interference analyzer is mandatory for exists semantics —
    /// injected at construction so a half-wired strategy cannot exist.
    /// `graph` is the graph-bearing view of the same analyzer: non-null for
    /// eager analyzers, null for lazy ones (where encode_parallelism is never
    /// called because a propagator module manages parallelism instead).
    explicit ExistsSemantics(std::unique_ptr<InterferenceOracle> analyzer,
                             const InterferenceGraphAnalysis* graph = nullptr)
        : interference_analyzer_(std::move(analyzer)), graph_(graph) {}

    z3::expr encode_parallelism(int timestep) override;

    std::string get_name() const override { return "ExistsSemantics"; }

    const InterferenceOracle* get_interference_analyzer() const override;
    const InterferenceGraphAnalysis* get_interference_graph_analysis() const override { return graph_; }

    bool allows_concurrent_actions() const override { return true; }

    /// The step order, built on first use from the interference graph, so
    /// it needs an eager (graph-bearing) analyzer.
    const ExistsStepOrder& step_order() const;

private:
    std::unique_ptr<InterferenceOracle> interference_analyzer_;
    const InterferenceGraphAnalysis* graph_ = nullptr;

    mutable std::unique_ptr<ExistsStepOrder> step_order_;

    bool prepared_ = false;
    std::vector<std::pair<int, int>> forbidden_;  ///< the pairs u -> v the order forbids

    /// Once, before the first step: the forbidden pairs, in the order of the
    /// graph.
    void prepare_pairs();
};

} // namespace rantanplan
