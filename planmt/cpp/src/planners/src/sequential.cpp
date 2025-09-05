#include <fstream>
#include <iostream>
#include <chrono>
#include <unordered_map>
#include <iomanip>

#include "config/headers/config.h"
#include "encoders/headers/parallelism/graph.h"
#include "encoders/headers/parallelism/interference_analysis.h"
#include "planners/headers/propagators/null_propagator.h"
#include "planners/headers/propagators/propagator_factory.h"
#include "util/headers/memory_tracker.h"
#include "util/headers/stats.h"

#include "planners/headers/sequential.h"


namespace planmt {

SequentialPlanner::SequentialPlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx): 
    problem_(problem), 
    encoder_(encoder), 
    ctx_(ctx), 
    solver_(ctx),
    propagator_strategy_(std::make_unique<NullPropagator>()) {
    // Initialize planner with the given problem, encoder, and Z3 context
    // Uses null propagator by default (no propagation)
}

SequentialPlanner::SequentialPlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx, 
    std::unique_ptr<PropagatorStrategy> propagator): 
    problem_(problem), 
    encoder_(encoder), 
    ctx_(ctx), 
    solver_(ctx),
    propagator_strategy_(propagator ? std::move(propagator) : std::make_unique<NullPropagator>()) {
    // Initialize planner with the given problem, encoder, Z3 context, and propagator strategy
}

void SequentialPlanner::debug_output_constraints() {
    // Output current constraints from the solver to SMT2 file
    std::ofstream smt2_file("output.smt2");
    if (smt2_file.is_open()) {
        smt2_file << ";; Current solver constraints from planMT" << std::endl;
        smt2_file << solver_.to_smt2() << std::endl;
        smt2_file.close();
    } else {
        std::cerr << "Error: Could not open output.smt2 for writing" << std::endl;
    }
}

void SequentialPlanner::collect_statistics() {
    auto& stats = Stats::instance();
    
    // Collect key Z3 solver statistics (just the important numbers)
    z3::stats z3_stats = solver_.statistics();
    for (unsigned i = 0; i < z3_stats.size(); ++i) {
        std::string key = "z3." + z3_stats.key(i);
        
        if (z3_stats.is_uint(i)) {
            stats.set(key, static_cast<double>(z3_stats.uint_value(i)));
        } else if (z3_stats.is_double(i)) {
            stats.set(key, z3_stats.double_value(i));
        }
    }
    
    // Add memory info
    stats.set("memory.current_mb", MemoryTracker::instance().get_current_memory_mb());
}

Plan SequentialPlanner::search() {
    auto& config = Config::instance();
    auto& stats  = Stats::instance();
    
    if (config.is_info()) {
        std::cout << "Starting search with propagator: " << propagator_strategy_->get_name() << std::endl;
    }
    
    // Reset solution found flag and statistics
    solution_found_ = false;

    // Add initial state constraints (these are invariant)
    solver_.add(*encoder_.encode_initial_state());
    
    // Register variables for timestep 0 (initial state)
    propagator_strategy_->register_timestep_variables(0);
    
    // Iterate from 0 to max_steps timesteps
    double total_time = 0.0; // Track total time used
    auto start_time = std::chrono::high_resolution_clock::now(); // Track total search time for timeout
    
    for (int timestep = 0; timestep <= config.planner.max_steps; ++timestep) {
        // Check timeout
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds = std::chrono::duration<double>(current_time - start_time).count();
        if (elapsed_seconds >= config.global.timeout) {
            if (config.is_info()) {
                std::cout << "\n*** TIMEOUT reached after " << elapsed_seconds << "s ***" << std::endl;
            }
            break;
        }

        auto step_start = std::chrono::high_resolution_clock::now();
        
        // Time formula creation
        auto formula_start = std::chrono::high_resolution_clock::now();
        
        // Add constraints for this timestep (incremental)
        construct_step(timestep);

        // Make the goal literal imply the goal at this timestep
        std::string goal_var_name = "goal_at_" + std::to_string(timestep);
        z3::expr goal_literal = ctx_.bool_const(goal_var_name.c_str());
        // goal_at_3 -> at_package1_location_3 /\ at_package2_location_3
        solver_.add(z3::implies(goal_literal, *encoder_.encode_goal(timestep)));
        
        // Check satisfiability by assuming a reified goal literal
        z3::expr_vector assumptions(ctx_);
        assumptions.push_back(goal_literal);
        
        auto formula_end = std::chrono::high_resolution_clock::now();
        auto formula_time = std::chrono::duration<double>(formula_end - formula_start).count();

        if (config.is_info()) {
            std::cout << "T" << timestep;
        }

        // Time solving
        auto solve_start = std::chrono::high_resolution_clock::now();
        z3::check_result result = solver_.check(assumptions);
        auto solve_end = std::chrono::high_resolution_clock::now();
        auto solve_time = std::chrono::duration<double>(solve_end - solve_start).count();
        
        auto step_end = std::chrono::high_resolution_clock::now();
        auto step_time = std::chrono::duration<double>(step_end - step_start).count();

        total_time += step_time; // Accumulate total time
        
        // Collect basic statistics
        stats.add("planner.timesteps_explored");
        stats.add("planner.formula_time", formula_time);
        stats.add("planner.solve_time", solve_time);
        stats.add("planner.total_time", step_time);
        
        // Print timing in compact format at INFO level
        if (config.is_info()) {
            double current_memory = MemoryTracker::instance().get_current_memory_mb();
            std::cout << " timing: formula=" << formula_time << "s, solve=" << solve_time << "s, step=" << step_time << "s";
            std::cout << ", memory=" << current_memory << "MB";
            std::cout << std::endl;
        }
        
        switch (result) {
            case z3::sat: {
                    if (config.is_info()) {
                        std::cout << "\n*** PLAN FOUND at timestep " << timestep << " (total time: " << total_time << "s) ***" << std::endl;
                    }
                    
                    // Mark that we found a solution
                    solution_found_ = true;
                    return construct_plan(timestep); // Extract and return the plan
                    break;
                }
            case z3::unsat:
                break; // Continue to next timestep
            default: {
                if (config.is_info()) {
                    std::cout << "Solver returned unknown result at timestep " << timestep << std::endl;
                }
                break;
            }
        }
    }
    
    if (config.is_info()) {
        std::cout << "\n*** NO PLAN FOUND within " << config.planner.max_steps << " timesteps, aborting ***" << std::endl;
        std::cout << "No plan found within " << config.planner.max_steps << " timesteps." << std::endl;
    }
    
    // Collect statistics after unsuccessful search
    collect_statistics();
    
    // Clean up propagator before returning
    propagator_strategy_->cleanup();
    return Plan(); // Return empty plan
}


std::vector<Plan> SequentialPlanner::search(int k, PlanningCategory category) {
    // Search for the initial plan and construct the basic formula.
    std::vector<Plan> ret_plans(k, Plan());
    auto initial_plan = search();

    // Return the plan if the category is SINGLE
    switch (category) {
        case PlanningCategory::TOP_K: {
            ret_plans = forbid_loop(std::vector<Plan>(1, initial_plan), k);
            break;
        }

        case PlanningCategory::TOP_Q:
        case PlanningCategory::DIVERSE:
            break;

        default:
        case PlanningCategory::SINGLE:
            ret_plans.assign(0, initial_plan);
    }
    
    // TODO: Remove empty plans from the return vector.

    // Now we need need to think about how to find multiple plans.
    return ret_plans; // Return empty plan
}


void SequentialPlanner::set_propagator_strategy(PropagatorType type) {
    propagator_strategy_ = PropagatorFactory::create_strategy(type, solver_, problem_, encoder_);
}

void SequentialPlanner::set_propagator_strategy(const std::string& strategy_name) {
    propagator_strategy_ = PropagatorFactory::create_strategy(strategy_name, solver_, problem_, encoder_);
}

std::string SequentialPlanner::get_propagator_strategy_name() const {
    return propagator_strategy_->get_name();
}

PropagatorType SequentialPlanner::get_propagator_type() const {
    return propagator_strategy_->get_type();
}

void SequentialPlanner::construct_step(int timestep) {
    auto& config = Config::instance();
    
    // Add constraints for this timestep (incremental)
    if (timestep <= 0) { return; }

    solver_.add(*encoder_.encode_actions(timestep-1));
    solver_.add(*encoder_.encode_frames(timestep-1));
    
    // Add symmetry breaking constraints if enabled
    if (config.symmetry.detect_symmetries) {
        solver_.add(*encoder_.encode_symmetries(timestep-1));
    }
    
    // Only add parallelism constraints if not using ForallPropagator or LazyForallPropagator
    // (ForallPropagator and LazyForallPropagator handle interference dynamically via propagation)
    if (get_propagator_type()    != PropagatorType::FORALL
        && get_propagator_type() != PropagatorType::LAZY_FORALL
        && get_propagator_type() != PropagatorType::HEURISTIC
        && get_propagator_type() != PropagatorType::EXISTS) {
        solver_.add(*encoder_.encode_parallelism(timestep-1));
    }
    
    // Register variables for timestep after constraints are added
    propagator_strategy_->register_timestep_variables(timestep);

    formula_length = timestep + 1; // Update formula length
}

Plan SequentialPlanner::construct_plan(int timestep) {
    auto& config = Config::instance();
    auto& stats  = Stats::instance();
    
    Plan plan = Plan();
    try {
        // Get and output the model to a file
        z3::model model = solver_.get_model();
        // Extract plan from model using encoder
        plan = encoder_.extract_plan(model, timestep);
        // Record successful solve
        stats.set("planner.plan_length", plan.length());
        stats.set("planner.solution_timestep", timestep);    
    } catch (const std::exception& e) {
        if (config.is_info()) {
            std::cout << "ERROR during plan extraction: " << e.what() << std::endl;
            std::cout << "Returning empty plan." << std::endl;
        }
    } 

    collect_statistics();
    
    // Clean up propagator before returning
    propagator_strategy_->cleanup();
    
    // Return plan
    return plan;
}

z3::expr SequentialPlanner::convert_plan_to_expr(const Plan& plan) const {
    auto &var_factory = encoder_.get_variable_factory();
    auto actions = plan.actions();
    z3::expr_vector literals(ctx_);
    for (size_t t = 0; t < plan.length(); ++t) {
        literals.push_back(var_factory.get_action_variable(*actions[t], static_cast<int>(t)));
    }
    return z3::mk_and(literals);
}


std::vector<Plan> SequentialPlanner::forbid_loop(const std::vector<Plan>& previous_plans, int k) {
    auto& config = Config::instance();
    
    assert(k > static_cast<int>(previous_plans.size()) && "k must be greater than the number of previous plans");

    // std::vector<Plan> ret_plans(k - static_cast<int>(previous_plans.size()), Plan());
    std::vector<Plan> ret_plans; // I hate the push back since it keeps reallocating and doubling the size.
    
    z3::expr_vector assumptions(ctx_);
    for (const auto& plan : previous_plans) {
        assumptions.push_back(!convert_plan_to_expr(plan));
        ret_plans.push_back(plan);
    }

    for (auto i = previous_plans.size(); i < k - static_cast<int>(previous_plans.size()); ++i) {
        z3::check_result result = solver_.check(assumptions);

        if (result != z3::sat) {
            break;
        }

        if (config.is_info()) {
            std::cout << "Found " << (i + 1) << " plans till now, remaining: " << (k - (i + 1)) << std::endl;
        }

        // Extract the plan.
        ret_plans.push_back(construct_plan(formula_length));
        assumptions.push_back(!convert_plan_to_expr(ret_plans[i]));
    }
    
    // remove plans with zero length
    ret_plans.erase(std::remove_if(ret_plans.begin(), ret_plans.end(), [](const Plan& p) { return p.length() == 0; }), ret_plans.end());
    
    return ret_plans;
}

} // namespace planmt
