
#include <fstream>
#include <iostream>
#include <chrono>
#include <unordered_map>

#include "config/config.hpp"
#include "encoders/parallelism/graph.hpp"
#include "encoders/parallelism/interference_analysis.hpp"
#include "propagators/null_propagator.hpp"
#include "util/memory_tracker.hpp"
#include "util/stats.hpp"
#include "util/logger.hpp"

#include "planners/forbid_behaviour_iterative.hpp"


namespace rantanplan {

ForbidBehaviourIterativePlanner::ForbidBehaviourIterativePlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx)
    : problem_(problem), encoder_(encoder), ctx_(ctx), solver_(ctx),
        propagator_strategy_(std::make_unique<NullPropagator>()) {
    // Initialize planner with the given problem, encoder, and Z3 context
    // Uses null propagator by default (no propagation)
}

ForbidBehaviourIterativePlanner::ForbidBehaviourIterativePlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx,
                                    std::unique_ptr<PropagatorStrategy> propagator)
    : problem_(problem), encoder_(encoder), ctx_(ctx), solver_(ctx),
        propagator_strategy_(propagator ? std::move(propagator) : std::make_unique<NullPropagator>()) {
    // Initialize planner with the given problem, encoder, Z3 context, and propagator strategy
}

Plan ForbidBehaviourIterativePlanner::search(float q, int k, const std::vector<std::string> dimensions) {
    return Plan(); // Return empty plan
}

void ForbidBehaviourIterativePlanner::set_propagator_strategy(std::unique_ptr<PropagatorStrategy> propagator) {
    propagator_strategy_ = std::move(propagator);
}

std::string ForbidBehaviourIterativePlanner::get_propagator_strategy_name() const {
    return propagator_strategy_->get_name();
}


} // namespace rantanplan
