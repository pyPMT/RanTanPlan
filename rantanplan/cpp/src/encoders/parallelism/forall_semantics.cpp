#include "forall_semantics.hpp"
#include "../../problem/action.hpp"
#include "../../util/stats.hpp"
#include <stdexcept>

namespace rantanplan {

Term ForallSemantics::encode_parallelism(int timestep) {
    // Get the interference graph from the graph-bearing analyzer view.
    // graph_ is non-null whenever encode_parallelism is reachable: lazy
    // analyzers require a propagator module that manages parallelism.
    if (!graph_) {
        throw std::logic_error(
            "ForallSemantics::encode_parallelism requires an eager "
            "(graph-bearing) interference analyzer");
    }
    const Graph& interference_graph = graph_->get_interference_graph();

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
            // Get the second action
            const Action* action2 = &problem_->action(neighbor_id);
            if (!action2) continue;

            // Create SMT variables for both actions at the given timestep
            Term action1_var = variable_factory_->get_action_variable(*action1, timestep);
            Term action2_var = variable_factory_->get_action_variable(*action2, timestep);

            // Add mutex constraint: not both actions can be true at the same time
            Term mutex_constraint = tb.mk_not(tb.mk_and({action1_var, action2_var}));
            mutex_constraints.push_back(mutex_constraint);
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

const InterferenceOracle* ForallSemantics::get_interference_analyzer() const {
    return interference_analyzer_.get();
}

} // namespace rantanplan
