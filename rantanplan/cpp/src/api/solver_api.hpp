#pragma once

#include "ir.hpp"
#include "preprocessing/preprocessed_problem.hpp"
#include "preprocessing/preprocessor_options.hpp"
#include "problem/problem.hpp"

#include <functional>
#include <mutex>
#include <string>

namespace z3 {
class context;
}

namespace rantanplan {
class Plan;
}

/**
 * @file solver_api.hpp
 * @brief The two-layer entry points: preprocess() and solve(), plus the
 *        cross-thread cancellation and plan-callback plumbing they share.
 */
namespace rantanplan::api {

/// Callback invoked with each plan the planner reports (improving plans in
/// anytime mode, the final plan otherwise) — the in-process replacement for
/// polling the plan.txt.N files that --output-plan still writes.
using PlanCallback = std::function<void(const Plan&)>;

/// Cross-thread cancellation slot. solve() publishes its z3::context here
/// for exactly the duration of the search; interrupt() flips Z3's interrupt
/// flag on the published context (documented safe from another thread).
/// Mutex-guarded rather than a bare atomic: unpublishing (and thus the
/// context's destruction) waits for any in-flight interrupt() call, closing
/// the load-then-use-after-free race. Outside the search window interrupt()
/// is a no-op — preprocessing and non-Z3 phases are not interruptible.
class InterruptSlot {
public:
    void interrupt();

private:
    friend class ScopedContextPublisher;
    std::mutex mutex_;
    z3::context* ctx_ = nullptr;
};

/// RAII publish/unpublish of a z3::context into an InterruptSlot. Declare it
/// after the context (destruction order then unpublishes before the context
/// dies). A null slot makes it a no-op.
class ScopedContextPublisher {
public:
    ScopedContextPublisher(InterruptSlot* slot, z3::context& ctx);
    ~ScopedContextPublisher();
    ScopedContextPublisher(const ScopedContextPublisher&) = delete;
    ScopedContextPublisher& operator=(const ScopedContextPublisher&) = delete;

private:
    InterruptSlot* slot_;
};

/**
 * @brief Run the full preprocessing pipeline on a problem.
 *
 * Strategy-independent and Config-free: all knobs arrive through the
 * options (driver builds them via PreprocessorOptions::from_config).
 * Owns pass-list construction, the pre-grounding snapshot for symmetry
 * detection, and the pipeline run. If the result has proven_unsolvable
 * set, the caller must not call solve().
 */
PreprocessedProblem preprocess(Problem problem, const PreprocessorOptions& options);

/**
 * @brief Resolve the configured strategy, assemble its components, and search.
 *
 * Owns spec resolution (registry lookup + problem-specific adjustment +
 * validation) and interference construction. Consumes (moves out of) parts
 * of the preprocessed problem: state constraints, achievers.
 *
 * If on_plan is set, it is installed on the planner and invoked from the
 * solver thread with each plan found during the search.
 *
 * If interrupt_slot is set, the solve's z3::context is published there for
 * the duration of the search so another thread can cancel it (the canceled
 * check surfaces through the planners' timeout path as TIMEOUT).
 */
ir::ResultIR solve(PreprocessedProblem& pipeline_result,
                   const PlanCallback& on_plan = {},
                   InterruptSlot* interrupt_slot = nullptr);

/**
 * @brief Export the SMT formula for a fixed timestep instead of solving.
 *
 * Bypasses preprocessing entirely — operates on the raw loaded problem.
 * Timestep and strategy come from Config's formula-export section.
 * Returns the formula text; throws std::invalid_argument on bad options.
 */
std::string export_formula(const Problem& problem);

} // namespace rantanplan::api
