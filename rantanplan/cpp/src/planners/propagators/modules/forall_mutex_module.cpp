#include "forall_mutex_module.hpp"
#include "../../../util/stats.hpp"

namespace rantanplan {

void LazyForallMutexModule::on_push() {
    // No private trail needed — shared action trail is managed by composite.
}

void LazyForallMutexModule::on_pop(unsigned /*num_scopes*/) {
    // Shared action trail is restored by composite.
}

void LazyForallMutexModule::on_fixed(const z3::expr& ast, const z3::expr& value) {
    if (!value.is_true()) return;
    auto action_info = shared_->variable_factory->get_action_from_variable(ast);
    if (!action_info) return;

    int current_id = action_info->first.id();
    int timestep = action_info->second;

    z3::expr_vector conflicting(host_->z3_ctx());
    for (int other_id : shared_->active_actions_per_timestep[timestep]) {
        if (other_id == current_id) continue;
        if (shared_->interference->has_interference(current_id, other_id) ||
            shared_->interference->has_interference(other_id, current_id)) {
            z3::expr var = shared_->variable_factory->get_action_variable(
                shared_->problem->action(other_id), timestep);
            conflicting.push_back(var);
        }
    }

    if (!conflicting.empty()) {
        conflict_count_++;
        z3::expr_vector justification(host_->z3_ctx());
        justification.push_back(ast);
        for (unsigned i = 0; i < conflicting.size(); ++i)
            justification.push_back(conflicting[i]);
        host_->module_conflict(justification);
    }
}

void LazyForallMutexModule::cleanup() {
    Stats::instance().set("propagator.lazy_forall_total_conflicts", conflict_count_);
}

} // namespace rantanplan
