#pragma once

#include "parallelism_strategy.hpp"
#include "../../analysis/interference_analysis.hpp"
#include <memory>

namespace rantanplan {

/**
 * @brief Forall semantics: actions can execute in parallel if they satisfy forall constraints
 * 
 * This strategy allows multiple actions to execute simultaneously in the same timestep
 * as long as none of the actions in a step affect any other one
 * 
 * Jussi Rintanen, Keijo Heljanko, Ilkka Niemelä,
 * Planning as satisfiability: parallel plans and algorithms for plan search,
 * Artificial Intelligence, Volume 170, Issues 12–13, 2006,
 */
class ForallSemantics : public ParallelismStrategy {
public:
    /// The interference analyzer is mandatory for forall semantics —
    /// injected at construction so a half-wired strategy cannot exist.
    /// `graph` is the graph-bearing view of the same analyzer: non-null for
    /// eager analyzers, null for lazy ones (where encode_parallelism is never
    /// called because a propagator module manages parallelism instead).
    explicit ForallSemantics(std::unique_ptr<InterferenceOracle> analyzer,
                             const InterferenceGraphAnalysis* graph = nullptr)
        : interference_analyzer_(std::move(analyzer)), graph_(graph) {}

    z3::expr encode_parallelism(int timestep) override;

    std::string get_name() const override { return "ForallSemantics"; }

    const InterferenceOracle* get_interference_analyzer() const override;
    const InterferenceGraphAnalysis* get_interference_graph_analysis() const override { return graph_; }

    bool allows_concurrent_actions() const override { return true; }

private:
    std::unique_ptr<InterferenceOracle> interference_analyzer_;
    const InterferenceGraphAnalysis* graph_ = nullptr;
};

} // namespace rantanplan
