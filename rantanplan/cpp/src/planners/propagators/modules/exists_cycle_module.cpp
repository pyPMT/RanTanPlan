#include "exists_cycle_module.hpp"
#include "../../../config/config.hpp"
#include "../../../encoders/chained_grounded_encoder.hpp"
#include "../../../encoders/parallelism/exists_semantics.hpp"
#include "../../../util/stats.hpp"
#include <algorithm>
#include <functional>

namespace rantanplan {

bool ExistsCycleModule::manages_parallelism_constraints() const {
    const auto* exists = dynamic_cast<const ExistsSemantics*>(shared_ ? shared_->parallelism : nullptr);
    return !(exists && exists->plain_chains());
}

ChainedGroundedEncoder* ExistsCycleModule::robust_encoder() {
    if (!robust_ready_) {
        robust_ready_ = true;
        auto* chained = dynamic_cast<ChainedGroundedEncoder*>(shared_->encoder);
        if (chained && chained->robust_on()) robust_ = chained;
    }
    return robust_;
}

int ExistsCycleModule::robust_state(int v, int t) const {
    auto it = robust_value_.find({v, t});
    return it == robust_value_.end() ? -1 : it->second;
}

void ExistsCycleModule::register_timestep_variables(int timestep) {
    if (ChainedGroundedEncoder* robust = robust_encoder(); robust && timestep > 0)
        for (const z3::expr& r : robust->robust_vars_at(timestep - 1)) host_->module_add(r);
}

void ExistsCycleModule::on_fixed(const z3::expr& ast, const z3::expr& value) {
    if (ChainedGroundedEncoder* robust = robust_encoder()) {
        auto it = robust->robust_ids().find(ast.id());
        if (it != robust->robust_ids().end()) {
            const auto [v, t] = it->second;
            robust_value_[{v, t}] = value.is_true();
            robust_trail_.push_back({v, t});
            if (value.is_false()) robust_step(t, false, v);  // the edges into v count again
            return;
        }
    }
    if (!value.is_true()) return;
    auto action_info = shared_->variable_factory->get_action_from_variable(ast);
    if (!action_info) return;

    perform_exists_propagation(action_info->first, action_info->second, ast);
}

void ExistsCycleModule::perform_exists_propagation(const Action& action,
                                                   int timestep,
                                                   const z3::expr& action_var) {
    shared_->build_footprint_index();
    if (robust_encoder()) {
        robust_step(timestep, false, action.id());
        return;
    }
    const auto& active = shared_->active_actions_per_timestep[timestep];
    const int start = Config::instance().exists.full_cycle_search ? -1 : action.id();

    std::vector<int> cycle;
    if (find_cycle_in_active_actions(active, cycle, start)) report_cycle(cycle, timestep);
}

bool ExistsCycleModule::robust_step(int t, bool in_final, int changed) {
    shared_->build_footprint_index();
    auto ait = shared_->active_actions_per_timestep.find(t);
    if (ait == shared_->active_actions_per_timestep.end() || ait->second.size() < 2) return false;
    const auto& active = ait->second;
    const int start = in_final || Config::instance().exists.full_cycle_search ? -1 : changed;
    if (start >= 0 && !active.contains(start)) return false;  // its edges matter only while it is active
    auto skip = [&](int u, int v) { return robust_state(v, t) == 1 && robust_->robust_removable(u, v); };
    std::vector<int> cycle;
    if (!find_cycle_in_active_actions(active, cycle, start, skip)) return false;

    // The cycle's removable edges u -> v whose r_v is false join the conflict;
    // with some r_v open, one of them must become true instead.
    z3::expr_vector lits(host_->z3_ctx()), open(host_->z3_ctx());
    for (int a : cycle) lits.push_back(shared_->variable_factory->get_action_variable(shared_->problem->action(a), t));
    for (size_t i = 0; i < cycle.size(); ++i) {
        const int u = cycle[i], v = cycle[(i + 1) % cycle.size()];  // u -> v: u may disable v
        const z3::expr* r = robust_->robust_var(v, t);
        if (!r || !robust_->robust_removable(u, v)) continue;
        const int state = robust_state(v, t);
        if (state == 0) lits.push_back(*r);
        else if (state < 0) open.push_back(*r);
    }
    if (open.empty()) {
        cycle_count_++;
        host_->module_conflict(lits);
        return true;
    }
    // A cycle already propagated on this branch issues nothing.
    std::vector<int> key(cycle);
    std::sort(key.begin(), key.end());
    if (!robust_issued_.insert({t, key}).second) return false;
    robust_issued_trail_.push_back({t, std::move(key)});
    host_->module_propagate(lits, z3::mk_or(open));
    Stats::instance().add("robust.propagations");
    return false;
}

void ExistsCycleModule::on_push() {
    if (!robust_encoder()) return;
    robust_levels_.push_back(robust_trail_.size());
    robust_issued_levels_.push_back(robust_issued_trail_.size());
}

void ExistsCycleModule::on_pop(unsigned num_scopes) {
    if (!robust_encoder()) return;
    for (unsigned i = 0; i < num_scopes && !robust_levels_.empty(); ++i) {
        const size_t keep = robust_levels_.back();
        robust_levels_.pop_back();
        while (robust_trail_.size() > keep) {
            robust_value_.erase(robust_trail_.back());
            robust_trail_.pop_back();
        }
        const size_t keep_issued = robust_issued_levels_.back();
        robust_issued_levels_.pop_back();
        while (robust_issued_trail_.size() > keep_issued) {
            robust_issued_.erase(robust_issued_trail_.back());
            robust_issued_trail_.pop_back();
        }
    }
}

void ExistsCycleModule::on_final() {
    if (robust_encoder()) {
        // A propagation that Z3 dropped leaves a cycle that no later search
        // from a fixed variable passes through.
        for (const auto& [timestep, active] : shared_->active_actions_per_timestep)
            if (robust_step(timestep, true, -1)) return;
        return;
    }
    if (Config::instance().exists.full_cycle_search) return;
    shared_->build_footprint_index();
    for (const auto& [timestep, active] : shared_->active_actions_per_timestep) {
        std::vector<int> cycle;
        if (find_cycle_in_active_actions(active, cycle)) {
            report_cycle(cycle, timestep);
            return;
        }
    }
}

void ExistsCycleModule::report_cycle(const std::vector<int>& cycle, int timestep) {
    cycle_count_++;

    z3::expr_vector conflict_actions(host_->z3_ctx());
    for (int nid : cycle) {
        z3::expr var = shared_->variable_factory->get_action_variable(
            shared_->problem->action(nid), timestep);
        conflict_actions.push_back(var);
    }
    host_->module_conflict(conflict_actions);
}

bool ExistsCycleModule::find_cycle_in_active_actions(
        const std::unordered_set<int>& active_node_ids,
        std::vector<int>& cycle, int start,
        const std::function<bool(int, int)>& skip) {
    if (active_node_ids.size() < 2) return false;

    std::unordered_set<int> visited;
    std::unordered_set<int> recursion_stack;
    std::vector<int> path;

    std::function<bool(int)> dfs = [&](int current) -> bool {
        visited.insert(current);
        recursion_stack.insert(current);
        path.push_back(current);

        auto fp_it = shared_->potential_interferers.find(current);
        if (fp_it != shared_->potential_interferers.end()) {
            for (int other_node : fp_it->second) {
                if (!active_node_ids.contains(other_node)) continue;
                if (skip && skip(current, other_node)) continue;
                if (shared_->interference->has_interference(current, other_node)) {
                    if (recursion_stack.contains(other_node)) {
                        auto cycle_start = std::find(path.begin(), path.end(), other_node);
                        cycle.assign(cycle_start, path.end());
                        return true;
                    }
                    if (!visited.contains(other_node)) {
                        if (dfs(other_node)) return true;
                    }
                }
            }
        }

        recursion_stack.erase(current);
        path.pop_back();
        return false;
    };

    if (start >= 0) return dfs(start);
    for (int start_node : active_node_ids) {
        if (!visited.count(start_node)) {
            if (dfs(start_node)) return true;
        }
    }
    return false;
}

void ExistsCycleModule::cleanup() {
    Stats::instance().set("propagator.exists_total_cycles", cycle_count_);
}

} // namespace rantanplan
