
#include <iostream>
#include <set>

#include "planners/headers/propagators/null_propagator.h"
#include "planners/headers/diverse.h"


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
        std::vector<Plan> ret_plans;
        
        // First infer the formula length.
        construct_formula();
        
        // TODO: Get the initial plan to infer the optimal plan lenght?
        ret_plans.push_back(extract_plan(formula_length));
        
        
        // Increase the formula length based on the quality bound q.
        for (size_t i = static_cast<size_t>(formula_length * q) - formula_length; i > 0; i--) {
            increment_formula();
        }
        
        // TODO: Construct behaviour space dimensions and append it to the solver.
        
        // TODO: Now it is time to start generating plans.
        auto generated_behaviours = produce_plans(k, ret_plans, ForbiddingMode::BEHAVIOUR);
        ret_plans.insert(ret_plans.end(), generated_behaviours.begin(), generated_behaviours.end());
        
        // Start forbidding plans since the required number of plans is less than the generated behaviours.
        if(ret_plans.size() < k) {
            auto generated_plans = produce_plans(k-ret_plans.size(), ret_plans, ForbiddingMode::PLAN);
            ret_plans.insert(ret_plans.end(), generated_plans.begin(), generated_plans.end());
        }
        

        return ret_plans;
    }


    std::vector<Plan> DiversePlanner::produce_plans(size_t k, std::vector<Plan> &planlist, ForbiddingMode mode) {
        
        std::vector<Plan> ret_plans;
        
        // Forbid plans in planlist.
        z3::expr_vector assumptions(ctx_);
        for (const auto& plan : planlist) {
            if (plan.length() == 0) { continue; }
            assumptions.push_back(!convert_plan_to_expr(plan));
        }
        
        // TODO: Collect behaviours in planlist.
        
        switch (mode) {
            case ForbiddingMode::BEHAVIOUR: {
                // TODO: We need to work on this.
                break;
            }
            
            case ForbiddingMode::PLAN: {
                // TODO: Use the behaviours generated before to help the search.
                
                // Start generating the remaming plans.
                for (size_t i = k; i > 0; i--) {
                    // Check satisfiability.
                    z3::check_result result = solver_.check(assumptions);
                    if (result != z3::sat) { break; }
                    auto plan = extract_plan(formula_length);
                    assumptions.push_back(!convert_plan_to_expr(plan));
                    ret_plans.push_back(plan);
                }
                break;
            }
                
            default: {
                break;
            }
        }
        
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


    z3::expr DiversePlanner::convert_plan_to_expr(const Plan& plan) const {
        auto &var_factory = encoder_.get_variable_factory();
        auto actions = plan.actions();
        z3::expr_vector literals(ctx_);
        for (size_t t = 0; t < plan.length(); ++t) {
            literals.push_back(var_factory.get_action_variable(*actions[t], static_cast<int>(t)));
        }
        return z3::mk_and(literals);
    }
}
