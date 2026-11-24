#pragma once

#include "../propagator_module.hpp"
#include "../propagator_strategy.hpp"
#include <functional>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace rantanplan {

class ChainedGroundedEncoder;

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
 * With robust actions (--robust-actions), the removable edges u -> v of the
 * encoder are left out of the search while r_v is true. A cycle through
 * removable edges into actions whose r is not true raises a conflict with
 * those r literals, all false, or else propagates that one of them is true.
 *
 * Reads from shared state: active_actions_per_timestep, potential_interferers,
 * interference analyzer, variable_factory, problem.
 * Without robust actions no private trail is needed: cycle detection is
 * stateless between callbacks.
 */
class ExistsCycleModule : public PropagatorModule {
public:
    std::string get_name() const override { return "ExistsCycle"; }
    /// False with plain chains (--plain-chains), which the planner posts.
    bool manages_parallelism_constraints() const override;

    void register_timestep_variables(int timestep) override;
    void on_fixed(const z3::expr& ast, const z3::expr& value) override;
    void on_final() override;
    void on_push() override;
    void on_pop(unsigned num_scopes) override;
    void cleanup() override;

private:
    int cycle_count_ = 0;

    void perform_exists_propagation(const Action& action, int timestep,
                                    const z3::expr& action_var);
    /// A cycle among the active actions, through `start` when it is >= 0,
    /// without the edges u -> v for which skip(u, v) holds.
    bool find_cycle_in_active_actions(
        const std::unordered_set<int>& active_node_ids,
        std::vector<int>& cycle, int start = -1,
        const std::function<bool(int, int)>& skip = nullptr);
    void report_cycle(const std::vector<int>& cycle, int timestep);

    // Robust actions: the encoder (null without them), the value of each r
    // fixed so far, and the cycles propagated, each undone on pop.
    ChainedGroundedEncoder* robust_ = nullptr;
    bool robust_ready_ = false;
    std::map<std::pair<int, int>, bool> robust_value_;  ///< (action, step) -> r
    std::vector<std::pair<int, int>> robust_trail_;
    std::vector<size_t> robust_levels_;
    std::set<std::pair<int, std::vector<int>>> robust_issued_;
    std::vector<std::pair<int, std::vector<int>>> robust_issued_trail_;
    std::vector<size_t> robust_issued_levels_;
    ChainedGroundedEncoder* robust_encoder();
    int robust_state(int v, int t) const;  ///< -1 while unassigned
    /// The cycle check of step t with robust actions, from `changed` (the
    /// action or the r just fixed) unless in_final; true if it raised a conflict.
    bool robust_step(int t, bool in_final, int changed);
};

} // namespace rantanplan
