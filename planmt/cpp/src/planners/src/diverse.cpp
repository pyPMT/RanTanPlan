
#include <iostream>
#include <set>

#include "encoders/headers/z3_variable_factory.h"
#include "planners/headers/propagators/null_propagator.h"
#include "planners/headers/diverse.h"


namespace planmt {

DiversePlanner::DiversePlanner(const Problem& problem, BaseEncoder& encoder, DiversePlannerCfg &cfg, std::vector<DiversityDimension> dims):
problem_(problem),
encoder_(encoder),
ctx_(encoder_.get_variable_factory().get_context()),
solver_(ctx_),
horizon_(encoder_.get_variable_factory().create_int_variable("Horizon")) {
    // Initialise the propagator strategy
    cfg_ = std::make_unique<DiversePlannerCfg>(cfg);
    propagator_strategy_ = PropagatorFactory::create_strategy(cfg_->get_propagator_strategy(), solver_, problem_, encoder_);
}


std::vector<Plan> DiversePlanner::search(size_t k, float q) {
    std::vector<Plan> ret_plans;
    
    // First infer the formula length.
    auto formula_ = construct_formula();
    
    // TODO: Construct behaviour space dimensions and append it to the solver.
    
    auto solver = create_solver_from(formula_);
    auto result = solver.check();
    
    if (result != z3::sat) {return ret_plans;}
    
    // Get the initial plan to infer the optimal plan lenght?
    ret_plans.push_back(extract_plan(solver));
    
    // Compute upperbound formula length.
    size_t upper_bound = static_cast<size_t>(formula_.length * q);
    
    // Generate the remaning plans, and increment the formula in case there are
    // no plans available in the current plan lenght.
    for(size_t i = formula_.length; i < upper_bound; i++) {
        std::cout << "Check i: " << i << std::endl;
        increment_formula(formula_, i);
    }
    
    auto generated_behaviours = produce_plans(formula_, k-ret_plans.size(), ret_plans, ForbiddingMode::BEHAVIOUR);
    ret_plans.insert(ret_plans.end(), generated_behaviours.begin(), generated_behaviours.end());
    
    // Start forbidding plans since the required number of plans is less than the generated behaviours.
    if(ret_plans.size() < k) {
        auto generated_plans = produce_plans(formula_, k-ret_plans.size(), ret_plans, ForbiddingMode::PLAN);
        ret_plans.insert(ret_plans.end(), generated_plans.begin(), generated_plans.end());
    }
    
    return ret_plans;
}

std::vector<Plan> DiversePlanner::produce_plans(formula_t phi, size_t k, std::vector<Plan> &planlist, ForbiddingMode mode) {
    
    std::vector<Plan> ret_plans;
    
    // Clear solver
    solver_ = create_solver_from(phi);
    
    // Forbid plans in planlist.
    z3::expr_vector assumptions(ctx_);
    for (const auto& plan : planlist) {
        if (plan.length() == 0) { continue; }
        assumptions.push_back(!convert_plan_to_expr(plan));
    }
    
    // TODO: Collect behaviours in planlist.
    
    for (size_t i = k; i > 0; i--) {
        // Check satisfiability.
        z3::check_result result = solver_.check(assumptions);
        if (result != z3::sat) { break; }
        auto plan = extract_plan(solver_);
        assumptions.push_back(!convert_plan_to_expr(plan));
        // TODO: Check if we are in behaviour forbidding or not.
        if (mode == ForbiddingMode::BEHAVIOUR) {
            // TODO: Infer the extracted plan's behaviour and forbid it.
        }
        
        ret_plans.push_back(plan);
    }
    
    return ret_plans;
}

Plan DiversePlanner::extract_plan(const z3::solver & solver) {
    // Extract plan from model using encoder
    z3::model model = solver.get_model();
    return encoder_.extract_plan(model, model.eval(horizon_, true).get_numeral_int());
}

formula_t DiversePlanner::construct_formula(void) {
    
    formula_t phi;
    auto &z3factory = encoder_.get_variable_factory();
    
    auto make_pair = [](size_t t, z3::expr expr) {
        return std::pair<size_t, z3::expr>(t, expr);
    };
    
    // Add initial state constraints (these are invariant)
    phi.initial[0].push_back(*encoder_.encode_initial_state());
    phi.initial[0].push_back(horizon_ >= ctx_.int_val(0));
    phi.initial[0].push_back(horizon_ <= static_cast<int>(cfg_->get_max_step()));
    
    // Register variables for timestep 0 (initial state)
    propagator_strategy_->register_timestep_variables(0);
    
    for (size_t timestep = 0; timestep <= cfg_->get_max_step(); ++timestep) {
        
        // clear solver.
        solver_.reset();
        
        // increment the formula
        auto g = increment_formula(phi, timestep);
        
        // check if formula is satisfiable
        solver_ = create_solver_from(phi);
        z3::expr_vector assumptions(ctx_);
        assumptions.push_back(g);
        z3::check_result result = solver_.check(assumptions);
        // break loop if a plan is found.
        if (result == z3::sat) { break; }
    }
    
    return phi;
}

z3::expr DiversePlanner::increment_formula(formula_t &f, size_t timestep) {
    
    // Make the goal literal imply the goal at this timestep
    std::string goal_var_name = "goal_at_" + std::to_string(timestep);
    z3::expr goal_literal = ctx_.bool_const(goal_var_name.c_str());
    
    // Add the goal literal to the solver.
    f.transition[timestep].push_back(z3::implies(goal_literal, *encoder_.encode_goal(timestep)));
    f.goal[timestep].push_back(goal_literal);
    
    // TODO: Add behaviour space dimensions encodings.
    
    if (timestep < 1) { return goal_literal; }
    
    // Add actions and frame axioms.
    f.transition[timestep - 1].push_back(*encoder_.encode_actions(timestep - 1));
    f.transition[timestep - 1].push_back(*encoder_.encode_frames(timestep - 1));
    
    // Add symmetry breaking constraints if enabled
    if (cfg_->break_symmetry()) {
        f.transition[timestep - 1].push_back(*encoder_.encode_symmetries(timestep - 1));
    }
    
    // Add propagatotion stategy.
    if (propagator_strategy_->get_type() == PropagatorType::NULL_PROPAGATOR) {
        f.transition[timestep - 1].push_back(*encoder_.encode_parallelism(timestep - 1));
    }
    
    // Register variables for timestep after constraints are added
    propagator_strategy_->register_timestep_variables(timestep - 1);
    
    // Update the horizon variable.
    // The value of horizon variable is used to help the planner decide the number
    // of actions should be extracted from the formula.
    z3::expr_vector previous_goal_literals(ctx_);
    for(size_t i = 0; i < timestep; i++) {
        previous_goal_literals.push_back(!f.goal[i].back());
    }
    previous_goal_literals.push_back(f.goal[timestep].back());
    f.transition[timestep-1].push_back(z3::mk_and(previous_goal_literals) == (horizon_ == ctx_.int_val(static_cast<int>(timestep))));
    
    
    // Lambda functions:
    auto make_actions_vector = [](auto& encoder, int t, z3::context& ctx) {
        z3::expr_vector actions(ctx);
        for (auto& a : encoder.get_variable_factory().get_all_action_variables(t)) {
            actions.push_back(a);
        }
        return actions;
    };
    
    // Force no actions to be taken after the first goal state, which means that on
    // the goal state there should be no actions selected.
    // The following constraint that denys any empty intermediate steps.
    auto actions_vector = make_actions_vector(encoder_, timestep-1, ctx_);
    f.transition[timestep-1].push_back(f.goal[timestep-1].back() == !z3::mk_or(actions_vector));
    
    // To deny any empty steps, we just confirm that at least one action is selected
    if (timestep > 1) {
        auto actions_step_minus_1 = make_actions_vector(encoder_, timestep-1, ctx_);
        auto actions_step_minus_2 = make_actions_vector(encoder_, timestep-2, ctx_);
        size_t actions_count_per_step = encoder_.get_variable_factory().get_all_action_variables(timestep-2).size();
        std::vector<int> coeffs(actions_count_per_step, 1);
        f.transition[timestep-1].push_back(z3::mk_or(actions_step_minus_1) == z3::pbge(actions_step_minus_2, &coeffs[0], 1));
    }
    
    f.length = timestep + 1;
    
    return goal_literal;
}


z3::solver DiversePlanner::create_solver_from(formula_t f) {
    z3::solver ret_solver(ctx_);
    
    for (auto &expr : f.initial[0]) {
        ret_solver.add(expr);
    }
    
    for (auto &[t, exprs] : f.transition) {
        for (auto &expr : exprs) {
            ret_solver.add(expr);
        }
    }
    
    z3::expr_vector goal_disjunction(ctx_);
    for (auto &[t, exprs] : f.goal) {
        for (auto &expr : exprs) {
            goal_disjunction.push_back(expr);
        }
    }
    
    ret_solver.add(z3::mk_or(goal_disjunction));
    return ret_solver;
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
