#include "incremental_horizon_planner.hpp"
#include "../config/config.hpp"
#include "../analysis/graph.hpp"
#include "../analysis/interference_analysis.hpp"
#include "../util/memory_tracker.hpp"
#include "../util/stats.hpp"
#include "../util/logger.hpp"
#include <iostream>
#include <chrono>
#include <unordered_map>

namespace rantanplan {

    IncrementalHorizonPlanner::IncrementalHorizonPlanner(const Problem& problem, GroundedEncoder& encoder, cvc5::TermManager& tm)
        : BasePlanner(problem, encoder, tm) {
    }

void IncrementalHorizonPlanner::add_timestep_constraints(int timestep) {
    BasePlanner::add_timestep_constraints(solver_, encoder_, timestep);
}

Plan IncrementalHorizonPlanner::search() {
    auto& config = Config::instance();
    auto& stats = Stats::instance();

    int start_timestep = std::max(0, config.planner.start_timestep);

    // Initialise the horizon schedule from config.
    schedule_ = HorizonSchedule::from_string(config.planner.horizon_schedule);

    std::string search_msg = "Starting search";
    if (start_timestep > 0) {
        search_msg += " from timestep " + std::to_string(start_timestep);
    }
    if (config.planner.horizon_schedule != "linear") {
        search_msg += ", schedule: " + config.planner.horizon_schedule;
    }
    search_msg += ", timeout: " + format_timeout_string();
    Logger::instance().info(search_msg);

    solution_found_ = false;
    timed_out_ = false;
    init_deadline();

    // Add initial state constraints (invariant across all horizons).
    solver_.assertFormula(encoder_.encode_initial_state());

    // Pre-fill all transitions for [0, start_timestep - 1] so the first SAT call
    // begins from the RPG lower bound without redundant checks.
    for (int t = 0; t < start_timestep; ++t) {
        add_timestep_constraints(t);
    }

    // -------------------------------------------------------------------------
    // Schedule-driven search loop
    //
    // Key invariant: before each SAT call at horizon h, ALL transitions for
    // timesteps [0, h-1] are in the solver (encoding states 0..h).
    // The schedule determines *which* horizons receive a goal literal and a SAT
    // call; the inner loop fills every skipped transition unconditionally.
    //
    // Completeness (why one goal literal at h suffices for the whole batch):
    //   encode_prefix_monotone ensures actions are front-loaded in the model:
    //   if a plan of length k <= h exists, the solver represents it with actions
    //   at steps 0..k-1 and empty steps at k..h-1. Frame axioms propagate the
    //   goal state through the empty suffix so goal@h is satisfied. The single
    //   literal goal_at_h therefore witnesses any plan length in [prev_h+1, h].
    //
    // Plan length recovery after SAT:
    //   Scan forward from t=0 for the first step where all action variables are
    //   false. Because of front-loading, this is the exact plan boundary and
    //   requires no extra solver calls.
    // -------------------------------------------------------------------------

    double total_time = 0.0;
    auto start_time = std::chrono::high_resolution_clock::now();

    int filled_up_to = start_timestep;  // transitions [0, filled_up_to-1] are encoded
    int h             = start_timestep;  // first horizon to check

    while (h <= config.planner.max_steps) {
        // Check timeout before building formula
        if (!apply_solver_timeout(solver_)) break;

        auto step_start   = std::chrono::high_resolution_clock::now();
        auto formula_start = std::chrono::high_resolution_clock::now();

        // 1. Fill all skipped transitions [filled_up_to, h).
        //    When h == start_timestep (first iteration) or h == 0, the loop is
        //    empty — no transitions added, mirroring the original empty-plan check.
        for (int t = filled_up_to; t < h; ++t) {
            add_timestep_constraints(t);
        }
        filled_up_to = h;

        // 2. Place ONE goal literal at the scheduled horizon h.
        //    Intermediate horizons in the batch do NOT get goal literals; the
        //    prefix-monotone + frame-axiom argument above covers them.
        std::string goal_var_name = "goal_at_" + std::to_string(h);
        Term goal_literal = tb_.fresh_bool(goal_var_name);
        solver_.assertFormula(tb_.mk_implies(goal_literal, encoder_.encode_goal(h)));

        TermVec assumptions;
        assumptions.push_back(goal_literal);

        auto formula_end  = std::chrono::high_resolution_clock::now();
        auto formula_time = std::chrono::duration<double>(formula_end - formula_start).count();

        auto solve_start = std::chrono::high_resolution_clock::now();
        cvc5::Result result = solver_.checkSatAssuming(assumptions);
        auto solve_end  = std::chrono::high_resolution_clock::now();
        auto solve_time = std::chrono::duration<double>(solve_end - solve_start).count();

        auto step_end  = std::chrono::high_resolution_clock::now();
        auto step_time = std::chrono::duration<double>(step_end - step_start).count();

        total_time += step_time;

        stats.add("planner.timesteps_explored");
        stats.add("planner.formula_time", formula_time);
        stats.add("planner.solve_time", solve_time);
        stats.add("planner.total_time", step_time);

        double current_memory = MemoryTracker::instance().get_current_memory_mb();
        Logger::instance().timestep_solving(VerbosityLevel::INFO, h, {
            {"formula", std::to_string(formula_time) + "s"},
            {"solve",   std::to_string(solve_time) + "s"},
            {"step",    std::to_string(step_time) + "s"},
            {"mem",     std::to_string(static_cast<int>(current_memory)) + "MB"}
        });

        if (result.isSat()) {
            solution_found_ = true;

            try {
                // cvc5 has no separate model object: extract_plan reads values
                // straight off the solver with getValue().
                Plan plan = encoder_.extract_plan(solver_, h);

                Logger::instance().info("\n*** PLAN FOUND: horizon=" + std::to_string(h) +
                    ", actions=" + std::to_string(plan.length()) +
                    " (total time: " + std::to_string(total_time) + "s) ***");

                plan.write_ipc(config.planner.output_plan, 1,
                               -1.0, true,
                               config.planner.strategy, config.planner.mode, total_time);
                notify_intermediate_plan(plan);

                stats.set("planner.plan_length",      static_cast<double>(plan.length()));
                stats.set("planner.solution_horizon", static_cast<double>(h));
                collect_statistics();
                return plan;
            } catch (const std::exception& e) {
                Logger::instance().error("ERROR during plan extraction: " + std::string(e.what()));
                collect_statistics();
                return Plan();
            }

        } else if (result.isUnsat()) {
            // UNSAT at this horizon; advance to the next scheduled horizon.
        } else {
            if (handle_unknown_result(result, "horizon " + std::to_string(h))) break;
        }

        // Advance to next scheduled horizon.
        int next_h = schedule_.next(h, config.planner.max_steps);
        if (next_h <= h) break;  // capped at max_steps — no further progress possible
        h = next_h;
    }

    if (timed_out_) {
        Logger::instance().info("No plan found (timeout).");
    } else {
        Logger::instance().info("\n*** NO PLAN FOUND within " +
            std::to_string(config.planner.max_steps) + " timesteps ***");
    }

    collect_statistics();
    return Plan();
}


} // namespace rantanplan
