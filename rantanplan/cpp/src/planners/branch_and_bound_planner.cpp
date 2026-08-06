#include "branch_and_bound_planner.hpp"
#include "../config/config.hpp"
#include "../encoders/base_encoder.hpp"
#include "../preprocessing/preprocessed_problem.hpp"
#include "../util/memory_tracker.hpp"
#include "../util/stats.hpp"
#include "../util/logger.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>

namespace rantanplan {

// ============================================================================
// Construction and utilities
// ============================================================================

void BranchAndBoundPlanner::consume_artifacts(PreprocessedProblem& artifacts) {
    if (artifacts.sdac_cost_lower_bounds.empty()) {
        return;
    }

    const auto& cost_bounds = artifacts.sdac_cost_lower_bounds;
    bool all_positive = std::all_of(cost_bounds.begin(), cost_bounds.end(),
                                     [](double lb) { return lb > 0.0; });
    if (!all_positive) {
        Logger::instance().info(
            "WARNING: Some SDAC action cost expressions have a lower bound "
            "of 0 (or could not be bounded). The abstract suffix will use 0 "
            "for these actions, reducing branch-and-bound pruning power.");
    } else {
        Logger::instance().info(
            "SDAC cost lower bounds computed via numeric RPG fixpoint — "
            "all action costs have positive lower bounds.");
    }

    set_cost_lower_bounds(std::vector<double>(cost_bounds.begin(), cost_bounds.end()));
}

BranchAndBoundPlanner::BranchAndBoundPlanner(
    const Problem& problem, BaseEncoder& encoder, cvc5::TermManager& tm,
    const InterferenceOracle* interference, SemanticsKind semantics)
    : BasePlanner(problem, encoder, tm),
      abstract_(problem, tm, encoder, interference, semantics) {
}

void BranchAndBoundPlanner::add_timestep_constraints(int timestep) {
    BasePlanner::add_timestep_constraints(solver_, encoder_, timestep);
}

/// Extract the numeric cost value from the model. cvc5 returns values as exact
/// rationals rather than Z3's approximate decimal string, so this reads the
/// rational directly; non-numeric values still yield 0.0, as before.
double BranchAndBoundPlanner::extract_cost_from_model(
    const cvc5::Solver& solver, const Term& cost_expr) {
    try {
        Term val = solver.getValue(cost_expr);

        if (val.isIntegerValue()) {
            return std::stod(val.getIntegerValue());
        }
        if (val.isReal64Value()) {
            auto [num, den] = val.getReal64Value();
            if (den == 0) return 0.0;
            return static_cast<double>(num) / static_cast<double>(den);
        }
        // PORT-QUESTION: fallback for rationals that do not fit in 64 bits.
        // cvc5 documents getRealValue() only as "the representation of a
        // rational value as a (rational) string"; observed output is "n/d"
        // (or a plain integer), which is what is parsed here. If a build
        // emits some other spelling this silently returns 0.0, exactly as the
        // Z3 version did for an unparsable decimal string.
        if (val.isRealValue()) {
            std::string rational = val.getRealValue();
            size_t slash = rational.find('/');
            if (slash == std::string::npos) return std::stod(rational);
            double num = std::stod(rational.substr(0, slash));
            double den = std::stod(rational.substr(slash + 1));
            if (den == 0.0) return 0.0;
            return num / den;
        }
    } catch (...) {
        return 0.0;
    }
    return 0.0;
}

// ============================================================================
// Main search loop
//
// The B&B search interleaves two levels:
//   - Outer loop: iterates over horizons (h = number of concrete steps).
//     Concrete transition constraints are added permanently as h grows.
//   - Inner loop: at each horizon, repeatedly tightens the cost bound until
//     either a concrete solution is found (update upper bound, continue) or
//     no solution with cost < C* exists at this horizon (advance).
//
// The abstract suffix constraints are split into three categories following
// the same per-timestep pattern as the grounded encoder:
//
//   Permanent constraints (added once after initialize):
//     - mod_v equivalences (Eq. 4B) — horizon-independent
//     - Loop formulas (Eq. 5) — horizon-independent, cached
//
//   Horizon-scoped constraints (inside push/pop per horizon):
//     - Axiom 6 (Eq. 6) — no arbitrary deferral, per step
//     - Axiom 7 step (Eq. 7) — full concrete prefix, per step
//     - Relaxed preconditions (Eq. 4A) — evaluated at state h
//     - Abstract goal (Eq. 3) — evaluated at state h
//     - Cost bound and goal implications from the inner B&B loop
//
// Compared to the naive approach of push/pop around ALL suffix constraints,
// this avoids re-adding the permanent mod_v equivalences and loop formulas
// each horizon. The decomposed AbstractSuffixEncoder API also allows the
// planner to control exactly which constraints are permanent vs scoped.
// ============================================================================

Plan BranchAndBoundPlanner::search() {
    auto& config = Config::instance();
    auto& stats = Stats::instance();

    int start_timestep = std::max(0, config.planner.start_timestep);
    schedule_ = HorizonSchedule::from_string(config.planner.horizon_schedule);

    std::string search_msg = "B&B search";
    if (start_timestep > 0) {
        search_msg += " from timestep " + std::to_string(start_timestep);
    }
    search_msg += ", timeout: " + format_timeout_string();
    Logger::instance().info(search_msg);

    solution_found_ = false;
    optimality_proven_ = false;
    timed_out_ = false;
    init_deadline();

    // Initialize
    solver_.assertFormula(encoder_.encode_initial_state());
    abstract_.initialize();

    // Permanent abstract constraints (add once, like encode_actions/frames)
    solver_.assertFormula(abstract_.encode_mod_v_equivalences());
    solver_.assertFormula(abstract_.encode_loop_formulas());

    Plan best_plan;
    double upper_bound = std::numeric_limits<double>::infinity();
    int cost_bound_counter = 0;

    auto start_time = std::chrono::high_resolution_clock::now();
    double total_time = 0.0;

    // h = horizon = goal state index (same convention as IncrementalHorizonPlanner).
    // At horizon h we encode transitions [0, h-1], producing states 0..h,
    // and check the goal at state h with the abstract suffix starting from h.
    int filled_up_to = 0;
    int h = start_timestep;

    // Pre-fill transitions up to start_timestep
    for (int t = 0; t < h; ++t) {
        add_timestep_constraints(t);
        filled_up_to = t + 1;
    }

    while (h <= config.planner.max_steps) {
        // Check timeout before building formula
        if (!apply_solver_timeout(solver_)) break;

        auto step_start = std::chrono::high_resolution_clock::now();

        // 1. Fill concrete transitions up to h-1 (producing states 0..h)
        for (int t = filled_up_to; t < h; ++t) {
            add_timestep_constraints(t);
        }
        filled_up_to = h;

        // 2. Push scope for horizon-dependent constraints. Permanent constraints
        //    (mod_v equivalences, loop formulas) remain outside and are not re-added.
        solver_.push();

        // Add axioms for all steps up to h (inside scope — they get popped
        // and re-added each horizon, same as original, but now using the
        // decomposed per-step methods)
        for (int i = 0; i < h; ++i) {
            if (i > 0) {
                solver_.assertFormula(abstract_.encode_axiom_6(i));
            }
            solver_.assertFormula(abstract_.encode_axiom_7_step(i));
        }

        solver_.assertFormula(abstract_.encode_relaxed_preconditions(h));
        solver_.assertFormula(abstract_.encode_abstract_goal(h));

        // 4. Build total cost expression and goal for this horizon
        Term total_cost = abstract_.build_total_cost(h);

        std::string goal_name = "goal_at_" + std::to_string(h);
        Term goal_lit = tm_.mkConst(tm_.getBooleanSort(), goal_name);
        solver_.assertFormula(tb_.mk_implies(goal_lit, encoder_.encode_goal(h)));

        // 5. Inner B&B loop at this horizon
        bool advance_horizon = false;
        while (!advance_horizon) {
            // Apply the solver timeout for the inner loop check too
            if (!apply_solver_timeout(solver_)) {
                advance_horizon = true;
                break;
            }

            // Create cost bound assumption
            std::string cost_name = "cost_lt_" + std::to_string(cost_bound_counter++);
            Term cost_bound_lit = tm_.mkConst(tm_.getBooleanSort(), cost_name);
            bool have_bound = !std::isinf(upper_bound);
            if (have_bound) {
                Term bound_val = encoder_.get_variable_factory().make_numeric_val(upper_bound);
                solver_.assertFormula(
                    tb_.mk_implies(cost_bound_lit, tb_.lt(total_cost, bound_val)));
            }

            TermVec assumptions;
            assumptions.push_back(goal_lit);
            if (have_bound) {
                assumptions.push_back(cost_bound_lit);
            }

            // Ask for a CONCRETE solution first, by assuming every abstract
            // action false, and only fall back to the relaxed query if that is
            // unsatisfiable.
            //
            // The original code ran the relaxed query alone and then *tested*
            // the model with is_concrete_solution(). That silently depended on
            // the solver preferring a model with no abstract actions: the
            // relaxed formula admits both, and nothing in it favours either.
            // Z3 happened to return concrete models; cvc5 returns abstract ones,
            // so every horizon was classified "abstract -> advance" and the
            // bound stayed inf forever -- the search ran to 500 horizons without
            // ever recognising the 3-step plan it was being handed.
            //
            // A model is not part of a formula's specification. Concreteness has
            // to be *asserted*, not hoped for.
            TermVec concrete_assumptions = assumptions;
            for (const Term& a_abs : abstract_.abstract_action_vars()) {
                concrete_assumptions.push_back(tb_.mk_not(a_abs));
            }

            auto solve_start = std::chrono::high_resolution_clock::now();
            cvc5::Result result = solver_.checkSatAssuming(concrete_assumptions);
            if (!result.isSat()) {
                // No concrete plan at this horizon under the current bound. The
                // relaxed query then distinguishes "horizon exhausted, keep
                // going" from "globally unsatisfiable".
                result = solver_.checkSatAssuming(assumptions);
            }
            auto solve_end = std::chrono::high_resolution_clock::now();
            auto solve_time = std::chrono::duration<double>(solve_end - solve_start).count();

            stats.add("planner.timesteps_explored");
            stats.add("planner.solve_time", solve_time);

            double current_memory = MemoryTracker::instance().get_current_memory_mb();
            Logger::instance().timestep_solving(VerbosityLevel::INFO, h, {
                {"solve", std::to_string(solve_time) + "s"},
                {"bound", have_bound ? std::to_string(upper_bound) : "inf"},
                {"mem",   std::to_string(static_cast<int>(current_memory)) + "MB"}
            });

            if (result.isUnsat()) {
                if (std::isinf(upper_bound)) {
                    if (h == 0) {
                        Logger::instance().info(
                            "\n*** UNSOLVABLE_PROVEN: no plan exists ***");
                        collect_statistics();
                        return Plan();
                    }
                    advance_horizon = true;
                } else {
                    // GLOBALLY OPTIMAL
                    optimality_proven_ = true;
                    best_cost_ = upper_bound;
                    auto end_time = std::chrono::high_resolution_clock::now();
                    total_time = std::chrono::duration<double>(end_time - start_time).count();
                    Logger::instance().info(
                        "\n*** OPTIMAL PLAN FOUND: horizon=" + std::to_string(h) +
                        ", actions=" + std::to_string(best_plan.length()) +
                        ", cost=" + std::to_string(upper_bound) +
                        " (total time: " + std::to_string(total_time) + "s) ***");
                    collect_statistics();
                    return best_plan;
                }
            } else if (result.isSat()) {
                // PORT-QUESTION: cvc5 has no model object — values are read
                // with solver.getValue(). This assumes both
                // AbstractSuffixEncoder::is_concrete_solution and
                // BaseEncoder::extract_plan are re-typed to take the Solver in
                // place of the z3::model (encoders layer, ported separately).
                if (abstract_.is_concrete_solution(solver_)) {
                    double plan_cost = extract_cost_from_model(solver_, total_cost);
                    upper_bound = plan_cost;
                    best_plan = encoder_.extract_plan(solver_, h);
                    solution_found_ = true;
                    solution_count_++;

                    auto now = std::chrono::high_resolution_clock::now();
                    double elapsed = std::chrono::duration<double>(now - start_time).count();
                    bool unit = (problem_.metric_kind() != MetricKind::MINIMIZE_ACTION_COSTS);
                    best_plan.write_ipc(config.planner.output_plan, solution_count_,
                                        upper_bound, unit,
                                        config.planner.strategy, config.planner.mode, elapsed);
                    notify_intermediate_plan(best_plan);

                    Logger::instance().info(
                        "  Improved plan: cost=" + std::to_string(upper_bound) +
                        ", horizon=" + std::to_string(h) +
                        ", actions=" + std::to_string(best_plan.length()));
                    // Continue inner loop to find cheaper plan at same horizon
                } else {
                    // Abstract solution — this horizon exhausted
                    advance_horizon = true;
                }
            } else {
                handle_unknown_result(result, "B&B horizon " + std::to_string(h));
                advance_horizon = true;
            }
        }

        // 6. Pop horizon-dependent scope, advance horizon
        solver_.pop();
        auto step_end = std::chrono::high_resolution_clock::now();
        auto step_time = std::chrono::duration<double>(step_end - step_start).count();
        total_time += step_time;
        stats.add("planner.total_time", step_time);

        if (timed_out_) break;

        int next_h = schedule_.next(h, config.planner.max_steps);
        if (next_h <= h) break;
        h = next_h;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    total_time = std::chrono::duration<double>(end_time - start_time).count();

    if (solution_found_) {
        best_cost_ = upper_bound;
        std::string reason = timed_out_ ? "timeout" : "search limit";
        Logger::instance().info(
            "\n*** BEST PLAN FOUND: horizon=" + std::to_string(h) +
            ", actions=" + std::to_string(best_plan.length()) +
            ", cost=" + std::to_string(upper_bound) +
            " (optimality not proven — " + reason +
            ", total time: " + std::to_string(total_time) + "s) ***");
    } else if (timed_out_) {
        Logger::instance().info(
            "\n*** NO PLAN FOUND (timeout after " +
            std::to_string(total_time) + "s) ***");
    } else {
        Logger::instance().info(
            "\n*** NO PLAN FOUND within " +
            std::to_string(config.planner.max_steps) +
            " timesteps (total time: " + std::to_string(total_time) + "s) ***");
    }

    collect_statistics();
    return best_plan;
}

} // namespace rantanplan
