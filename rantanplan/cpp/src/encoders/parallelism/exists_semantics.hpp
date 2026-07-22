#pragma once

#include "parallelism_strategy.hpp"
#include "../../analysis/interference_analysis.hpp"
#include "../../analysis/eager_interference_analysis.hpp"
#include <memory>

namespace rantanplan {

/**
 * @brief Exists semantics
 *  Set of actions in a timestep can execute if there exists
 *  at least one order where actions do not interfere with actions further
 *  down the line.
 *
 * The ∃-step condition is enforced one of two ways, depending on the strategy:
 *
 *  - Encoded (`e`): a-priori mutex clauses from the eager interference graph
 *    — the fixed-order approximation of Rintanen et al. 2006. Pick the
 *    action-id ordering as the fixed serialization order; a step is accepted
 *    iff all interference among its actions points forward in that order.
 *    Only interference edges that go backward (id(a1) > id(a2)) AND stay
 *    within one SCC can break the order, so exactly those pairs get a mutex.
 *    Sound but stricter than exact ∃-step. Requires the eager analyzer.
 *
 *  - Propagator (`els`, `els-fp`): exact cycle detection over the actions
 *    fixed true, done lazily during search by ExistsCycleModule. In that
 *    case encode_parallelism is never called (the module manages
 *    parallelism) and no graph is needed.
 *
 * Jussi Rintanen, Keijo Heljanko, Ilkka Niemelä,
 * Planning as satisfiability: parallel plans and algorithms for plan search,
 * Artificial Intelligence, Volume 170, Issues 12–13, 2006,
 */
class ExistsSemantics : public ParallelismStrategy {
public:
    /// The interference analyzer is mandatory for exists semantics —
    /// injected at construction so a half-wired strategy cannot exist.
    /// `eager_graph` is the graph-bearing view of the same analyzer, set
    /// exactly when the analyzer is eager; it is required iff the strategy
    /// has no parallelism-managing propagator (encoded ∃-step).
    explicit ExistsSemantics(std::unique_ptr<InterferenceOracle> analyzer,
                             const EagerInterferenceAnalysis* eager_graph = nullptr)
        : interference_analyzer_(std::move(analyzer)), eager_graph_(eager_graph) {}

    /// Encoded fixed-order ∃-step mutexes (throws without an eager analyzer;
    /// only reachable for propagator-free exists strategies).
    z3::expr encode_parallelism(int timestep) override;

    std::string get_name() const override { return "ExistsSemantics"; }

    const InterferenceOracle* get_interference_analyzer() const override;

    bool allows_concurrent_actions() const override { return true; }

private:
    std::unique_ptr<InterferenceOracle> interference_analyzer_;
    const EagerInterferenceAnalysis* eager_graph_ = nullptr;
};

} // namespace rantanplan
