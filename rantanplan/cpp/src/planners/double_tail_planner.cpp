#include "double_tail_planner.hpp"
#include "../config/config.hpp"
#include "../analysis/graph.hpp"
#include "../analysis/interference_analysis.hpp"
#include "../util/memory_tracker.hpp"
#include "../util/stats.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"
#include <chrono>
#include <iostream>

namespace rantanplan {

DoubleTailPlanner::DoubleTailPlanner(const Problem& problem, BaseEncoder& encoder, cvc5::TermManager& tm)
    : BasePlanner(problem, encoder, tm) {
    auto& config = Config::instance();
    max_horizon_ = config.planner.max_steps;
}

Term DoubleTailPlanner::create_link_constraints(int forward_t, int backward_t) {
    // Create equivalence constraints for all fluents: fluent@forward_t ⟺ fluent@backward_t
    TermVec links;

    for (ExprID eid : problem_.grounded_fluents()) {
        Term forward_var = encoder_.convert_expr_id_to_z3(eid, forward_t);
        Term backward_var = encoder_.convert_expr_id_to_z3(eid, backward_t);
        // mk_eq is bi-implication for Boolean fluents, equality otherwise —
        // Term::operator== is syntactic equality in cvc5, not a formula.
        links.push_back(tb_.mk_eq(forward_var, backward_var));
    }

    return tb_.mk_and(links);
}

void DoubleTailPlanner::add_timestep_constraints(int t) {
    // DoubleTail omits prefix_monotone (not compatible with bidirectional search)
    BasePlanner::add_timestep_constraints(solver_, encoder_, t,
                                          /*prefix_monotone=*/false);
}

/**
 * Double-tail incremental SAT planning algorithm
 * Based on Gocht & Balyo (ICAPS 2017)
 *
 * Algorithm overview:
 * - Builds SAT formula from BOTH initial state (forward) and goal state (backward)
 * - Alternates between adding forward and backward transitions
 * - Uses link constraints to connect the two stacks
 * - Iteration i tests for plans of exactly length i
 *
 * Key invariants:
 * - Initial state always at timestep 0: I(0)
 * - Goal state always at timestep max_horizon_: G(max_horizon_)
 * - Forward stack grows from timestep 0 upward on odd iterations
 * - Backward stack grows from timestep max_horizon_ downward on even iterations
 *
 * Iteration structure:
 * - Iteration 0: No transitions, just link I(0) with G(max_horizon_)
 *   Tests if initial state satisfies goal (empty plan)
 *
 * - Iteration 1: Add one forward transition T(0)
 *   Formula: I(0) ∧ T(0) ∧ G(max_horizon_) ∧ L(1, max_horizon_)
 *   States: 0 --T(0)--> 1 <==link==> max_horizon_
 *
 * - Iteration 2: Add one backward transition T(max_horizon_-1)
 *   Formula: I(0) ∧ T(0) ∧ T(max_horizon_-1) ∧ G(max_horizon_) ∧ L(1, max_horizon_-1)
 *   States: 0 --T(0)--> 1 <==link==> max_horizon_-1 --T(max_horizon_-1)--> max_horizon_
 *
 * - General pattern for iteration i:
 *   - forward_depth = ceil(i/2) = number of forward actions
 *   - backward_depth = floor(i/2) = number of backward actions
 *   - Forward stack: states 0, 1, ..., forward_depth with transitions T(0), ..., T(forward_depth-1)
 *   - Backward stack: states backward_start, ..., max_horizon_ with transitions T(backward_start), ..., T(max_horizon_-1)
 *     where backward_start = max_horizon_ - backward_depth
 *   - Link constraint: L(forward_depth, backward_start) forces state equivalence
 *
 * Link constraints with activation literals:
 * - Each iteration creates a new activation literal: link_at_i
 * - Formula includes: link_at_i => L(forward_depth, backward_start)
 * - Solver is called with assumption [link_at_i = true]
 * - This allows incremental solving while testing different link positions
 */
Plan DoubleTailPlanner::search() {
    auto& config = Config::instance();
    auto& stats = Stats::instance();

    std::string search_msg = "Starting double-tail search";
    search_msg += ", max horizon: " + std::to_string(max_horizon_);
    search_msg += ", timeout: " + format_timeout_string();
    Logger::instance().info(search_msg);

    solution_found_ = false;
    timed_out_ = false;
    init_deadline();

    // Add invariant constraints: initial state at t=0, goal state at t=max_horizon_
    solver_.assertFormula(encoder_.encode_initial_state());

    solver_.assertFormula(encoder_.encode_goal(max_horizon_));

    double total_time = 0.0;
    auto start_time = std::chrono::high_resolution_clock::now();

    // Main iteration loop: iteration i tests for plan of exactly length i
    // - Iteration 0: no transitions, just link states 0 and max_horizon_
    // - Odd iterations (1,3,5,...): add one forward transition
    // - Even iterations (2,4,6,...): add one backward transition
    for (int iteration = 0; iteration <= max_horizon_; ++iteration) {
        // Apply the solver timeout for the remaining budget
        if (!apply_solver_timeout(solver_)) break;

        auto step_start = std::chrono::high_resolution_clock::now();

        // Calculate stack depths using helper methods
        int forward_depth = calculate_forward_depth(iteration);
        int backward_depth = calculate_backward_depth(iteration);

        // Calculate timestep boundaries
        // - Forward stack ends at timestep forward_depth
        // - Backward stack starts at timestep (max_horizon_ - backward_depth)
        int forward_end = forward_depth;
        int backward_start = max_horizon_ - backward_depth;

        // Build formula (add transition if iteration > 0)
        auto formula_start = std::chrono::high_resolution_clock::now();

        if (iteration > 0) {
            bool is_odd_iteration = (iteration % 2 == 1);
            if (is_odd_iteration) {
                // Odd: add forward transition at timestep (forward_end - 1)
                add_timestep_constraints(forward_end - 1);
            } else {
                // Even: add backward transition at timestep backward_start
                add_timestep_constraints(backward_start);
            }
        }

        // Create link with activation literal: link_literal → L(forward_end, backward_start)
        std::string link_var_name = "link_at_" + std::to_string(iteration);
        Term link_literal = tm_.mkConst(tm_.getBooleanSort(), link_var_name);
        solver_.assertFormula(tb_.mk_implies(link_literal,
                                             create_link_constraints(forward_end, backward_start)));

        auto formula_end = std::chrono::high_resolution_clock::now();
        double formula_time = std::chrono::duration<double>(formula_end - formula_start).count();

        // Solve with link activated
        TermVec assumptions;
        assumptions.push_back(link_literal);

        auto solve_start = std::chrono::high_resolution_clock::now();
        cvc5::Result result = solver_.checkSatAssuming(assumptions);
        auto solve_end = std::chrono::high_resolution_clock::now();
        double solve_time = std::chrono::duration<double>(solve_end - solve_start).count();

        auto step_end = std::chrono::high_resolution_clock::now();
        double step_time = std::chrono::duration<double>(step_end - step_start).count();
        total_time += step_time;

        // Statistics
        stats.add("planner.timesteps_explored");
        stats.add("planner.formula_time", formula_time);
        stats.add("planner.solve_time", solve_time);
        stats.add("planner.total_time", step_time);

        // Log iteration progress
        double current_memory = MemoryTracker::instance().get_current_memory_mb();
        std::string direction = (iteration == 0) ? "[Init]" : ((iteration % 2 == 1) ? "[Forward]" : "[Backward]");
        Logger::instance().timestep_solving(VerbosityLevel::INFO, iteration, {
            {direction, ""},
            {"formula", std::to_string(formula_time) + "s"},
            {"solve", std::to_string(solve_time) + "s"},
            {"step", std::to_string(step_time) + "s"},
            {"mem", std::to_string(static_cast<int>(current_memory)) + "MB"},
            {"forward", "[0.." + std::to_string(forward_end) + "]"},
            {"backward", "[" + std::to_string(backward_start) + ".." + std::to_string(max_horizon_) + "]"},
            {"link", "t" + std::to_string(forward_end) + " <=> t" + std::to_string(backward_start)}
        });

        if (result.isSat()) {
            Logger::instance().info("\n*** PLAN FOUND at iteration " + std::to_string(iteration) +
                                   " (plan length " + std::to_string(iteration) +
                                   ", total time: " + std::to_string(total_time) + "s) ***");

            solution_found_ = true;

            if (iteration == 0) {
                // Empty plan
                stats.set("planner.plan_length", 0.0);
                stats.set("planner.solution_timestep", 0.0);
                collect_statistics();
                return Plan();
            }

            try {
                Plan plan = extract_plan(solver_, iteration);

                plan.write_ipc(config.planner.output_plan, 1,
                               -1.0, true,
                               config.planner.strategy, config.planner.mode, total_time);
                notify_intermediate_plan(plan);

                stats.set("planner.plan_length", static_cast<double>(plan.length()));
                stats.set("planner.solution_timestep", static_cast<double>(iteration));
                collect_statistics();
                return plan;
            } catch (const std::exception& e) {
                Logger::instance().error("ERROR during plan extraction: " + std::string(e.what()));
                collect_statistics();
                return Plan();
            }
        } else if (result.isUnsat()) {
            // Continue to next iteration
        } else {
            if (handle_unknown_result(result, "iteration " + std::to_string(iteration))) break;
        }
    }

    if (timed_out_) {
        Logger::instance().info("No plan found (timeout).");
    } else {
        Logger::instance().info("\n*** NO PLAN FOUND within " + std::to_string(max_horizon_) + " iterations ***");
    }
    collect_statistics();
    return Plan();
}

Plan DoubleTailPlanner::extract_plan(const cvc5::Solver& solver, int plan_length) {
    Plan plan;

    // Calculate stack depths using helper methods
    int forward_depth = calculate_forward_depth(plan_length);
    int backward_depth = calculate_backward_depth(plan_length);
    int backward_start = max_horizon_ - backward_depth;

    Logger::instance().debug("Extracting double-tail plan: " + std::to_string(forward_depth) +
                             " forward actions (timesteps 0.." + std::to_string(forward_depth - 1) + "), " +
                             std::to_string(backward_depth) + " backward actions (timesteps " +
                             std::to_string(backward_start) + ".." + std::to_string(max_horizon_ - 1) + ")");

    // Extract forward actions (from timestep 0 up to forward_depth-1)
    // Then extract backward actions (from timestep backward_start up to max_horizon_-1)
    extract_actions_in_range(plan, solver, 0, forward_depth);
    extract_actions_in_range(plan, solver, backward_start, max_horizon_);

    return plan;  // Total actions: forward_depth + backward_depth = plan_length
}

void DoubleTailPlanner::extract_actions_in_range(Plan& plan, const cvc5::Solver& solver,
                                                   int start_timestep, int end_timestep) {
    for (int t = start_timestep; t < end_timestep; ++t) {
        std::vector<const Action*> actions_at_t = extract_parallel_actions_at_timestep(solver, t);

        if (encoder_.get_parallelism_strategy()->allows_concurrent_actions()) {
            // Parallel: topologically sort and add all actions
            std::vector<const Action*> ordered = topologically_sort_actions(actions_at_t);
            for (const Action* action : ordered) {
                plan.add_action(action);
            }
        } else {
            // Sequential: add at most one action
            if (!actions_at_t.empty()) {
                plan.add_action(actions_at_t[0]);
            }
        }
    }
}

std::vector<const Action*> DoubleTailPlanner::extract_parallel_actions_at_timestep(
    const cvc5::Solver& solver, int timestep) const {

    std::vector<const Action*> parallel_actions;

    for (const Action& grounded_action : problem_.actions()) {
        try {
            const Term& action_var = encoder_.get_variable_factory()
                .get_action_variable(grounded_action, timestep);
            Term action_value = solver.getValue(action_var);

            if (action_value.isBooleanValue() && action_value.getBooleanValue()) {
                parallel_actions.push_back(&grounded_action);
            }
        } catch (const std::exception&) {
            // Skip actions whose variables don't exist
        }
    }

    return parallel_actions;
}

std::vector<const Action*> DoubleTailPlanner::topologically_sort_actions(
    const std::vector<const Action*>& actions) const {

    if (actions.size() <= 1) {
        return {actions.begin(), actions.end()};
    }

    const ParallelismStrategy* strategy = encoder_.get_parallelism_strategy();
    const InterferenceOracle* analyzer = strategy->get_interference_analyzer();

    return analyzer->topological_sort_actions(actions);
}

int DoubleTailPlanner::calculate_forward_depth(int iteration) const {
    // Forward stack grows on odd iterations: ceil(iteration/2)
    return (iteration + 1) / 2;
}

int DoubleTailPlanner::calculate_backward_depth(int iteration) const {
    // Backward stack grows on even iterations: floor(iteration/2)
    return iteration / 2;
}

} // namespace rantanplan
