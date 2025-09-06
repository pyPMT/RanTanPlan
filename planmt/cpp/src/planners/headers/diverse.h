#pragma once

#include <string>
#include <vector>
#include <set>
#include <map>

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

struct formula_t {
    std::unordered_map<size_t, std::vector<z3::expr>> initial;
    std::unordered_map<size_t, std::vector<z3::expr>> transition;
    std::unordered_map<size_t, std::vector<z3::expr>> goal;
    size_t length;
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
    DiversePlanner(const Problem& problem, BaseEncoder& encoder, DiversePlannerCfg &cfg, std::vector<DiversityDimension> dims);
    
    std::vector<Plan> search(size_t k, float q);
    
protected:
    const Problem& problem_;
    BaseEncoder& encoder_;
    z3::context& ctx_;
    z3::solver solver_;
    z3::expr horizon_;
    std::unique_ptr<PropagatorStrategy> propagator_strategy_;
    std::unique_ptr<DiversePlannerCfg> cfg_;

    // Search for plans based on selected forbidding mode.
    std::vector<Plan> produce_plans(formula_t phi, size_t k, std::vector<Plan> &planlist, ForbiddingMode mode);
    
    // Convert a plan to a conjunction of actions.
    z3::expr convert_plan_to_expr(const Plan& plan) const;
    
    // Extract plan from model.
    Plan extract_plan(const z3::solver & solver);
    
    // Contruct the full formula I(0) /\ T_0 .. T_n /\ (\/ G_0 .. G_n)
    formula_t construct_formula(void);
    
    // Add formula_t elements to z3 solver.
    z3::solver create_solver_from(formula_t f);
    
    // Increment formula length and return the goal literal
    z3::expr increment_formula(formula_t &f, size_t timestep);
};

}

