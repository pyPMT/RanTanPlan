
#include <iostream>

#include "planners/headers/propagators/null_propagator.h"

#include "planners/headers/diverse.h"

//



namespace planmt {

DiversePlanner::DiversePlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx, DiversePlannerCfg &cfg, std::vector<DiversityDimension> dims):
    problem_(problem),
    encoder_(encoder),
    ctx_(ctx),
    solver_(ctx) {
        
        // Initialise the propagator strategy
        cfg_ = std::make_unique<DiversePlannerCfg>(cfg);
        propagator_strategy_ = PropagatorFactory::create_strategy(cfg_->get_propagator_strategy(), solver_, problem_, encoder_);
}


std::vector<Plan> DiversePlanner::search(size_t k, float q) {
    std::vector<Plan> ret_plans(k, Plan());
    
    // First infer the formula length.
    construct_formula();
    
    // TODO: Get the initial plan to infer the optimal plan lenght?
    ret_plans[0] = extract_plan(formula_length);
    
    
    // Increase the formula length based on the quality bound q.
    for (size_t i = static_cast<size_t>(formula_length * q) - formula_length; i > 0; i--) {
        increment_formula();
    }
    
    // TODO: Construct behaviour space dimensions and append it to the solver.
    
    // TODO: Now it is time to start generating plans.
    auto generated_plans = produce_plans(3, ret_plans, ForbiddingMode::PLAN);
    
    // Remove plans with empty actions.
    ret_plans.erase(std::remove_if(ret_plans.begin(), ret_plans.end(), [](const Plan& p) { return p.length() == 0; }), ret_plans.end());
    return ret_plans;
}


std::vector<Plan> produce_plans(size_t k, std::vector<Plan> &planlist, ForbiddingMode mode) {
    std::vector<Plan> ret_plans(k, Plan());
    
    switch (mode) {
        case ForbiddingMode::BEHAVIOUR: {
            // TODO: We need to work on this.
            break;
        }
        
        case ForbiddingMode::PLAN: {
            
            break;
        }
            
        default: {
            break;
        }
    }
    
    ret_plans.erase(std::remove_if(ret_plans.begin(), ret_plans.end(), [](const Plan& p) { return p.length() == 0; }), ret_plans.end());
    return ret_plans;
}

Plan DiversePlanner::extract_plan(size_t length) {
    z3::model model = solver_.get_model();
    // Extract plan from model using encoder
    return encoder_.extract_plan(model, length);
}

void DiversePlanner::construct_formula(void) {
    // Add initial state constraints (these are invariant)
    solver_.add(*encoder_.encode_initial_state());
    
    // Register variables for timestep 0 (initial state)
    propagator_strategy_->register_timestep_variables(0);
    
    for (size_t timestep = 0; timestep <= cfg_->get_max_step(); ++timestep) {
        
        // Add constraints for this timestep (incremental)
        increment_formula();
        
        // Make the goal literal imply the goal at this timestep
        std::string goal_var_name = "goal_at_" + std::to_string(timestep);
        z3::expr goal_literal = ctx_.bool_const(goal_var_name.c_str());
        // goal_at_3 -> at_package1_location_3 /\ at_package2_location_3
        solver_.add(z3::implies(goal_literal, *encoder_.encode_goal(timestep)));
        
        // Check satisfiability by assuming a reified goal literal
        z3::expr_vector assumptions(ctx_);
        assumptions.push_back(goal_literal);
        z3::check_result result = solver_.check(assumptions);
        
        // Formula constructed once satisfied.
        if (result == z3::sat) {
            break;
        }
    }
}

void DiversePlanner::increment_formula(void) {
    
    // Add actions and frame axioms.
    solver_.add(*encoder_.encode_actions(formula_length));
    solver_.add(*encoder_.encode_frames(formula_length));
    
    // Add symmetry breaking constraints if enabled
    if (cfg_->break_symmetry()) {
        solver_.add(*encoder_.encode_symmetries(formula_length));
    }
    
    // Add propagatotion stategy.
    if (propagator_strategy_->get_type() == PropagatorType::NULL_PROPAGATOR) {
        solver_.add(*encoder_.encode_parallelism(formula_length));
    }
    
    // Register variables for timestep after constraints are added
    propagator_strategy_->register_timestep_variables(formula_length);
    
    // Increment formula length.
    formula_length += 1;
}


}
