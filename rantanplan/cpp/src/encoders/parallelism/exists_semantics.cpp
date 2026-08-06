#include "exists_semantics.hpp"
#include "../../problem/action.hpp"
#include "../../util/stats.hpp"
#include <stdexcept>

namespace rantanplan {

Term ExistsSemantics::encode_parallelism(int timestep) {
    // Get the interference graph from the graph-bearing analyzer view.
    // graph_ is non-null whenever encode_parallelism is reachable: lazy
    // analyzers require a propagator module that manages parallelism.
    if (!graph_) {
        throw std::logic_error(
            "ExistsSemantics::encode_parallelism requires an eager "
            "(graph-bearing) interference analyzer");
    }
    const Graph& interference_graph = graph_->get_interference_graph();

    // Get SCC mapping for optimization: only add mutex within same SCC
    std::vector<int> scc_mapping = interference_graph.get_scc_mapping();

    const TermBuilder& tb = variable_factory_->builder();
    TermVec mutex_constraints;

    // Iterate through all nodes (actions) in the interference graph
    for (size_t node_id = 0; node_id < interference_graph.num_nodes(); ++node_id) {
        // Get the action corresponding to this node
        const Action* action1 = &problem_->action(node_id);
        if (!action1) continue;

        // Get all neighbors (actions that this action interferes with)
        const std::vector<int>& neighbors = interference_graph.get_neighbours(node_id);

        for (int neighbor_id : neighbors) {
            // Enhanced conditions for ExistsSemantics:
            // 1. Only create mutex if source node ID > target node ID
            // 2. Only create mutex if both nodes are in the same SCC
            if (static_cast<int>(node_id) > neighbor_id &&
                scc_mapping[node_id] == scc_mapping[neighbor_id]) {

                // Get the second action
                const Action* action2 = &problem_->action(neighbor_id);
                if (!action2) continue;

                // Create SMT variables for both actions at the given timestep
                Term action1_var = variable_factory_->get_action_variable(*action1, timestep);
                Term action2_var = variable_factory_->get_action_variable(*action2, timestep);

                // Create mutex constraint: ¬(action1_t ∧ action2_t) ≡ (¬action1_t ∨ ¬action2_t)
                Term mutex = tb.mk_or({tb.mk_not(action1_var), tb.mk_not(action2_var)});
                mutex_constraints.push_back(mutex);
            }
        }
    }

    // Combine all mutex constraints with logical AND
    if (mutex_constraints.empty()) {
        return tb.bool_val(true);
    }

    auto& stats = Stats::instance();
    stats.add("encoder.mutex_constraints", mutex_constraints.size());

    Term mutex_formula = tb.mk_and(mutex_constraints);
    return mutex_formula;
}

const InterferenceOracle* ExistsSemantics::get_interference_analyzer() const {
    return interference_analyzer_.get();
}

} // namespace rantanplan
