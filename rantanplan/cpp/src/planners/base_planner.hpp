#pragma once

#include "../analysis/arithmetic_profile.hpp"
#include "../problem/problem.hpp"
#include "../problem/plan.hpp"
#include "../encoders/grounded_encoder.hpp"
#include "../smtcore/smt_types.hpp"
#include "../smtcore/term_builder.hpp"
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

namespace rantanplan {

struct PreprocessedProblem;

// ---------------------------------------------------------------------------
// HorizonSchedule
// ---------------------------------------------------------------------------
// Controls which horizons receive a SAT call during planning. Between any two
// consecutive scheduled horizons, ALL transitions are still added to the solver
// (correctness requires a complete transition chain), but only one goal literal
// is placed at the scheduled horizon. Completeness is preserved via the
// prefix-monotone encoding (encode_prefix_monotone) which ensures actions are
// front-loaded so frame axioms propagate shorter plans to the batch horizon.

enum class ScheduleType { LINEAR, ARITHMETIC, GEOMETRIC, DOUBLING };

struct HorizonSchedule {
    ScheduleType type = ScheduleType::LINEAR;

    /**
     * Return the next horizon after `current`, always >= current+1, capped at `max`.
     * Guaranteeing current+1 as a minimum prevents infinite loops when current is 0
     * with doubling (0*2 = 0) or when the factor/step would not advance the index.
     *
     * Per-type hardcoded step/factor:
     *   linear:     +1 per step
     *   arithmetic: +5 per step
     *   geometric:  *1.5 per step
     *   doubling:   *2  per step
     */
    int next(int current, int max) const {
        int n;
        switch (type) {
            case ScheduleType::LINEAR:     n = current + 1;                                      break;
            case ScheduleType::ARITHMETIC: n = current + 5;                                      break;
            case ScheduleType::GEOMETRIC:  n = std::max(current + 1,
                                               static_cast<int>(current * 1.5));                 break;
            case ScheduleType::DOUBLING:   n = std::max(current + 1, current * 2);               break;
            default:                       n = current + 1;                                      break;
        }
        return std::min(n, max);
    }

    static HorizonSchedule from_string(const std::string& name) {
        HorizonSchedule s;
        if      (name == "linear")     s.type = ScheduleType::LINEAR;
        else if (name == "arithmetic") s.type = ScheduleType::ARITHMETIC;
        else if (name == "geometric")  s.type = ScheduleType::GEOMETRIC;
        else if (name == "doubling")   s.type = ScheduleType::DOUBLING;
        else throw std::invalid_argument("Unknown horizon schedule: '" + name +
                                         "'. Valid values: linear, arithmetic, geometric, doubling");
        return s;
    }
};

// ---------------------------------------------------------------------------
// BasePlanner
// ---------------------------------------------------------------------------

/**
 * @brief Base interface for all planner implementations
 *
 * This interface abstracts different planning strategies (e.g., incremental-horizon
 * satisficing, branch-and-bound optimal). The planner type is part of the strategy
 * configuration, allowing strategies to control both the encoding and the search approach.
 */
class BasePlanner {
public:
    /// Joins the deadline watchdog if one is running.
    virtual ~BasePlanner();

    /**
     * @brief Main search method - runs the planning algorithm
     * @return Plan object (may be empty if no solution found)
     */
    virtual Plan search() = 0;

    /**
     * @brief Check if the last search found a solution
     * @return true if a valid plan was found, false otherwise
     */
    virtual bool solution_found() const { return solution_found_; }

    /// Whether the planner proved optimality. Default: false (satisficing planners).
    virtual bool optimality_proven() const { return false; }

    /// Best cost found (only meaningful for cost-optimal planners).
    virtual double best_cost() const { return 0.0; }

    /// Whether the search ended due to timeout.
    bool timed_out() const { return timed_out_; }

    /// Set the arithmetic profile for cvc5 solver tuning.
    void set_arithmetic_profile(ArithmeticProfile profile) { arithmetic_profile_ = profile; }

    /// Apply solver-wide options (fixed seed, logic). MUST run before the first
    /// assertion — cvc5 accepts options only before the solver is initialised —
    /// and after set_arithmetic_profile, which the logic choice depends on.
    /// StrategyFactory::create_planner calls it; nothing else should.
    void configure_solver();

    /// Take ownership of the preprocessing artifacts this planner consumes.
    /// Default: consume nothing. Overrides move artifacts out of the bundle
    /// (single-consumer — the bundle must not be reused for a second solve).
    virtual void consume_artifacts(PreprocessedProblem& artifacts) { (void)artifacts; }

    /// Install a callback invoked with each plan the search reports (every
    /// improving plan for branch-and-bound, the final plan otherwise) — the
    /// in-process counterpart of the plan.txt.N files written at the same
    /// points. Runs on the solver thread; must be cheap.
    void set_intermediate_plan_callback(std::function<void(const Plan&)> callback) {
        on_intermediate_plan_ = std::move(callback);
    }


    /// Access to the underlying cvc5 solver.
    cvc5::Solver& get_solver() { return solver_; }

    /**
     * @brief Set the horizon schedule used during search
     */
    void set_horizon_schedule(HorizonSchedule s) { schedule_ = s; }
    const HorizonSchedule& get_horizon_schedule() const { return schedule_; }

protected:
    /// Construct with shared planner dependencies.
    BasePlanner(const Problem& problem, GroundedEncoder& encoder, cvc5::TermManager& tm);

    // Shared state across all planners (declaration order matters for init).
    const Problem& problem_;
    GroundedEncoder& encoder_;
    cvc5::TermManager& tm_;
    /// Term construction for the planners themselves (goal/link/blocking
    /// literals and the formulas around them). cvc5 has no operator
    /// overloading, so every connective goes through this.
    TermBuilder tb_;
    cvc5::Solver solver_;

    /// Horizon schedule; concrete planners initialise this from Config in their constructor.
    HorizonSchedule schedule_;

    ArithmeticProfile arithmetic_profile_ = ArithmeticProfile::LINEAR;

    bool solution_found_ = false;
    bool timed_out_ = false;

    /// Deadline watchdog. cvc5 accepts options only before the first assertion,
    /// so the Z3 idiom of re-setting a timeout before every check() cannot be
    /// reproduced with setOption. Instead one thread sleeps until the deadline
    /// and calls Solver::interrupt() (the fork patch added at M1, thread-safe
    /// by construction), which is the faithful equivalent: each check is cut off
    /// at the wall-clock deadline rather than after a stale per-check budget.
    /// Started lazily on the first timed check and joined in the destructor.
    std::thread watchdog_;
    std::mutex watchdog_mutex_;
    std::condition_variable watchdog_cv_;
    bool watchdog_stop_ = false;
    void start_watchdog(cvc5::Solver& solver);
    void stop_watchdog();

    /// Optional plan hook (set_intermediate_plan_callback). Concrete planners
    /// call notify_intermediate_plan() wherever they write plan.txt.N.
    std::function<void(const Plan&)> on_intermediate_plan_;

    void notify_intermediate_plan(const Plan& plan) const {
        if (on_intermediate_plan_) on_intermediate_plan_(plan);
    }

    /// Wall-clock deadline for the search. Set from Config::global::timeout in
    /// init_deadline(), then queried via remaining_ms() before each solver.check().
    std::chrono::steady_clock::time_point deadline_;

    /// Initialise the deadline from Config::global::timeout.
    void init_deadline();

    /// Milliseconds remaining until deadline. Returns 0 if already past.
    unsigned remaining_ms() const;

    /// Apply the remaining timeout to a cvc5 solver so that the next checkSat()
    /// call is interrupted if the wall-clock budget runs out.
    /// Returns false if the deadline has already passed (caller should break).
    /// See the PORT-QUESTION in the .cpp: cvc5 only accepts options before the
    /// first assertion, so only the first call can take effect.
    bool apply_solver_timeout(cvc5::Solver& solver);

    /// Check if an unknown result was caused by timeout. If so, sets
    /// timed_out_ and logs the interruption. Returns true if it was a timeout.
    /// Takes the Result (not the solver): in cvc5 the explanation belongs to
    /// the result of the check, not to the solver.
    bool handle_unknown_result(const cvc5::Result& result, const std::string& context);

    /// Format the timeout value for log messages (e.g. "120s" or "none").
    static std::string format_timeout_string();

    /// Collect cvc5 solver statistics and memory usage into the global Stats singleton.
    /// Uses get_solver() to access the derived class's solver.
    void collect_statistics();

    /// Add standard timestep constraints: actions, frames, prefix-monotone,
    /// symmetries, parallelism. The propagator variable-registration hook was
    /// removed pending the in-solver reimplementation.
    static void add_timestep_constraints(cvc5::Solver& solver, GroundedEncoder& encoder,
                                         int timestep);
};

} // namespace rantanplan
