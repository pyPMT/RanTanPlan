#pragma once

#include <string>
#include <vector>

#include "problem/headers/problem.h"
#include "problem/headers/plan.h"
#include "encoders/headers/base_encoder.h"
#include "propagators/propagator_strategy.h"
#include "propagators/propagator_factory.h"
#include "util/headers/stats.h"

#include "theories/headers/diversity/diversity_base_dimension.h"

namespace planmt {

enum ForbiddingMode {
    BEHAVIOUR,
    PLAN,
    NONE
};

class DiversePlannerCfg {
public:
    DiversePlannerCfg(std::string propagator_strategy, bool detect_symmetries, size_t max_steps) {
        propagator_strategy = propagator_strategy;
        detect_symmetries   = detect_symmetries;
        max_steps           = max_steps;
    }
    
    std::string get_propagator_strategy() { return propagator_strategy; }
    bool break_symmetry() { return detect_symmetries; }
    size_t get_max_step() { return max_steps; }
    
private:
    std::string propagator_strategy{"null"}; // TODO: I need to check this value.
    bool detect_symmetries{false};
    size_t max_steps;
};

class DiversePlanner {
public:
    // Constructor with propagator support
    DiversePlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx, DiversePlannerCfg &cfg, std::vector<DiversityDimension> dims);
    
    std::vector<Plan> search(size_t k, float q);
    
protected:
    const Problem& problem_;
    BaseEncoder& encoder_;
    z3::context& ctx_;
    z3::solver solver_;
    std::unique_ptr<PropagatorStrategy> propagator_strategy_;
    std::unique_ptr<DiversePlannerCfg> cfg_;
    int formula_length{0};
    
    // Search for plans based on selected forbidding mode.
    std::vector<Plan> produce_plans(size_t k, std::vector<Plan> &planlist, ForbiddingMode mode);
    
    // Convert a plan to a conjunction of actions.
    z3::expr convert_plan_to_expr(const Plan& plan) const;
    
    // Extract plan from model.
    Plan extract_plan(size_t length);
    
    // Find formula length
    void construct_formula(void);
    
    // Increment formula length
    void increment_formula(void);
};

}

