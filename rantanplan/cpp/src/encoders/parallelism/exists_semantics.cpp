#include "exists_semantics.hpp"
#include "../../problem/action.hpp"
#include "../../util/stats.hpp"
#include <stdexcept>

namespace rantanplan {

z3::expr ExistsSemantics::encode_parallelism(int timestep) {
    // Only reachable for propagator-free exists strategies (`e`), which
    // validate() restricts to eager interference — the graph is available.
    if (!eager_graph_) {
        throw std::logic_error(
            "ExistsSemantics::encode_parallelism requires an eager "
            "(graph-bearing) interference analyzer; propagator-based exists "
            "strategies never call it");
    }
    const Graph& interference_graph = eager_graph_->interference_graph();

    // Fixed-order ∃-step (Rintanen et al. 2006): with the action-id ordering
    // as the fixed serialization order, only interference edges that go
    // backward in the order AND stay within one SCC can break it — exactly
    // those pairs get an a-priori mutex.
    std::vector<int> scc_mapping = interference_graph.get_scc_mapping();

    z3::expr_vector mutex_constraints(*ctx_);

    for (size_t node_id = 0; node_id < interference_graph.num_nodes(); ++node_id) {
        const Action* action1 = &problem_->action(node_id);
        if (!action1) continue;

        const std::vector<int>& neighbors = interference_graph.get_neighbours(node_id);

        for (int neighbor_id : neighbors) {
            if (static_cast<int>(node_id) > neighbor_id &&
                scc_mapping[node_id] == scc_mapping[neighbor_id]) {

                const Action* action2 = &problem_->action(neighbor_id);
                if (!action2) continue;

                z3::expr action1_var = variable_factory_->get_action_variable(*action1, timestep);
                z3::expr action2_var = variable_factory_->get_action_variable(*action2, timestep);

                // Mutex constraint: ¬(action1_t ∧ action2_t)
                mutex_constraints.push_back(!action1_var || !action2_var);
            }
        }
    }

    if (mutex_constraints.empty()) {
        return ctx_->bool_val(true);
    }

    Stats::instance().add("encoder.mutex_constraints", mutex_constraints.size());

    return z3::mk_and(mutex_constraints);
}

const InterferenceOracle* ExistsSemantics::get_interference_analyzer() const {
    return interference_analyzer_.get();
}

} // namespace rantanplan
