#pragma once

#include "../problem/problem.hpp"
#include "../problem/plan.hpp"
#include "../encoders/grounded_encoder.hpp"
#include "base_planner.hpp"
#include "../smtcore/smt_types.hpp"
#include <memory>

// Incremental-horizon search: grow the horizon step by step, re-checking
// satisfiability until a plan is found (any parallelism semantics).
namespace rantanplan {

class IncrementalHorizonPlanner : public BasePlanner {
public:
    // Constructor
    IncrementalHorizonPlanner(const Problem& problem, GroundedEncoder& encoder, cvc5::TermManager& tm);

    // BasePlanner interface implementation
    Plan search() override;

private:
    // Add constraints for a given timestep (delegates to BasePlanner::add_timestep_constraints)
    void add_timestep_constraints(int timestep);
};

} // namespace rantanplan
