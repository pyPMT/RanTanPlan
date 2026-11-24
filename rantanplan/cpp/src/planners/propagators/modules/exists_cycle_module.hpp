#pragma once

#include "../propagator_module.hpp"
#include "../propagator_strategy.hpp"

namespace rantanplan {

/**
 * @brief Exists-step cycle detection module.
 *
 * When an action is fixed to true, performs DFS on the interference graph
 * among active actions at that timestep, from that action: the active actions
 * had no cycle before it, so a new cycle passes through it. If a cycle is
 * found, reports a conflict with all actions in the cycle. The final check
 * searches every step from every active action, so a conflict that Z3 drops
 * is raised again there. With --full-cycle-search every search starts from
 * every active action.
 *
 * Reads from shared state: active_actions_per_timestep, potential_interferers,
 * interference analyzer, variable_factory, problem.
 * No private trail needed — cycle detection is stateless between callbacks.
 */
class ExistsCycleModule : public PropagatorModule {
public:
    std::string get_name() const override { return "ExistsCycle"; }
    bool manages_parallelism_constraints() const override { return true; }

    void on_fixed(const z3::expr& ast, const z3::expr& value) override;
    void on_final() override;
    void cleanup() override;

private:
    int cycle_count_ = 0;

    void perform_exists_propagation(const Action& action, int timestep,
                                    const z3::expr& action_var);
    /// A cycle among the active actions, through `start` when it is >= 0.
    bool find_cycle_in_active_actions(
        const std::unordered_set<int>& active_node_ids,
        std::vector<int>& cycle, int start = -1);
    void report_cycle(const std::vector<int>& cycle, int timestep);
};

} // namespace rantanplan
