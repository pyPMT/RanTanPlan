#pragma once

#include "../propagator_module.hpp"
#include "../propagator_strategy.hpp"

namespace rantanplan {

/**
 * @brief Lazy forall mutex module.
 *
 * Maintains an active action set. When a new action is fixed to true,
 * checks for interference with all currently active actions and raises
 * a conflict if interference is found.
 */
class LazyForallMutexModule : public PropagatorModule {
public:
    std::string get_name() const override { return "LazyForallMutex"; }
    bool manages_parallelism_constraints() const override { return true; }

    void on_push() override;
    void on_pop(unsigned num_scopes) override;
    void on_fixed(const z3::expr& ast, const z3::expr& value) override;
    void cleanup() override;

private:
    // Private trail (tracks which actions we added, for undo on backtrack)
    // Note: the shared action trail handles exists-style tracking.
    // For lazy-forall, we use the shared active_actions_per_timestep for reads,
    // but the shared trail already tracks push/pop correctly.
    int conflict_count_ = 0;
};

} // namespace rantanplan
