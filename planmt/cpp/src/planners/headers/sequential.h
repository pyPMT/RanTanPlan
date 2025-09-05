#pragma once

#include <z3++.h>
#include <memory>
#include <vector>

#include "problem/headers/problem.h"
#include "problem/headers/plan.h"
#include "encoders/headers/base_encoder.h"
#include "propagators/propagator_strategy.h"
#include "propagators/propagator_factory.h"
#include "util/headers/stats.h"

// This class is able to search for a plan in a sequential manner by using an encoder.
namespace planmt {

enum class PlanningCategory {
    SINGLE  = 0,
    TOP_K   = 1, 
    TOP_Q   = 2,
    DIVERSE = 3
};


class SequentialPlanner {
public:
    // Constructor
    SequentialPlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx);
    
    // Constructor with propagator support
    SequentialPlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx, 
                     std::unique_ptr<PropagatorStrategy> propagator);

    // Search for a plan and return it
    Plan search();

    // Search for k plans 
    std::vector<Plan> search(int k, PlanningCategory category);
    
    // Check if the last search found a solution (even if empty plan)
    bool solution_found() const { return solution_found_; }
    
    // Strategy management
    void set_propagator_strategy(PropagatorType type);
    void set_propagator_strategy(const std::string& strategy_name);
    std::string get_propagator_strategy_name() const;
    PropagatorType get_propagator_type() const;

private:
    int formula_length{0}; // Number of steps currently encoded in the formula
    const Problem& problem_;
    BaseEncoder& encoder_;
    z3::context& ctx_;
    z3::solver solver_;  // Solver to maintain current constraints state
    bool solution_found_ = false;  // Track if last search found a solution
    
    // Propagator strategy for custom propagation (defaults to null propagator)
    std::unique_ptr<PropagatorStrategy> propagator_strategy_;
    
    // Debug method to output constraints for a given timestep
    void debug_output_constraints();
    
    // Collect comprehensive statistics
    void collect_statistics();

    // Construct a step in the formula
    void construct_step(int timestep);

    // Construct a plan from the current model
    Plan construct_plan(int timestep);

    z3::expr convert_plan_to_expr(const Plan& plan) const;

    // Forbid-Plan loop
    std::vector<Plan> forbid_loop(const std::vector<Plan>& previous_plans, int k);
};

} // namespace planmt
