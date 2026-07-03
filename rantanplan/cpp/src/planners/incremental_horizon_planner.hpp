#pragma once

#include "../problem/problem.hpp"
#include "../problem/plan.hpp"
#include "../encoders/base_encoder.hpp"
#include "base_planner.hpp"
#include "propagators/propagator_strategy.hpp"
#include <z3++.h>
#include <memory>

// Incremental-horizon search: grow the horizon step by step, re-checking
// satisfiability until a plan is found (any parallelism semantics).
namespace rantanplan {

class IncrementalHorizonPlanner : public BasePlanner {
public:
    // Constructor
    IncrementalHorizonPlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx);

    // Constructor with propagator support
    IncrementalHorizonPlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx,
                     std::unique_ptr<PropagatorStrategy> propagator);

    // BasePlanner interface implementation
    Plan search() override;

private:
    // Debug method to output constraints for a given timestep
    void debug_output_constraints();

    // Add constraints for a given timestep (delegates to BasePlanner::add_timestep_constraints)
    void add_timestep_constraints(int timestep);
};

} // namespace rantanplan
