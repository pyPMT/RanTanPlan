#pragma once

#include <memory>
#include <z3++.h>

#include "problem/problem.hpp"
#include "problem/plan.hpp"
#include "encoders/base_encoder.hpp"
#include "propagators/propagator_strategy.hpp"
#include "planners/base_planner.hpp"

namespace rantanplan {

class ForbidBehaviourIterativePlanner : public BasePlanner {
public:
    // Constructor
    ForbidBehaviourIterativePlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx);

    // Constructor with propagator support
    ForbidBehaviourIterativePlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx,
                     std::unique_ptr<PropagatorStrategy> propagator);

    // BasePlanner interface implementation
    Plan search(float q = 1.0, int k = 1, const std::vector<std::string> dimensions = {}) override;

    // Check if the last search found a solution (even if empty plan)
    bool solution_found() const override { return solution_found_; }

    // Strategy management
    void set_propagator_strategy(std::unique_ptr<PropagatorStrategy> propagator) override;
    std::string get_propagator_strategy_name() const override;

    // Get solver reference (for creating propagators)
    z3::solver& get_solver() override { return solver_; }

private:
    const Problem& problem_;
    BaseEncoder& encoder_;
    z3::context& ctx_;
    z3::solver solver_;  // Solver to maintain current constraints state
    bool solution_found_ = false;  // Track if last search found a solution

    // Propagator strategy for custom propagation (defaults to null propagator)
    std::unique_ptr<PropagatorStrategy> propagator_strategy_;
};

} // namespace rantanplan
