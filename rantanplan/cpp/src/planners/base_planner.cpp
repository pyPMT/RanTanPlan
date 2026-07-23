#include "base_planner.hpp"
#include "../config/config.hpp"
#include "../util/memory_tracker.hpp"
#include "../util/stats.hpp"
#include "../util/logger.hpp"
// UINT_MAX (remaining_ms) needs an explicit include now that nothing else
// pulls it in transitively.
#include <climits>
#include <string>

namespace rantanplan {

BasePlanner::BasePlanner(const Problem& problem, GroundedEncoder& encoder, cvc5::TermManager& tm)
    : problem_(problem), encoder_(encoder), tm_(tm), tb_(tm), solver_(tm) {
    tb_.set_problem(&problem_);

    // Model extraction is not on by default in cvc5, and options are only
    // accepted before the first assertion — so it is enabled here, at the one
    // point that is guaranteed to precede every assertFormula on this solver.
    solver_.setOption("produce-models", "true");
}

BasePlanner::~BasePlanner() { stop_watchdog(); }

void BasePlanner::configure_solver() {
    // ── cvc5 solver tuning parameters ──────────────────────────────────
    //
    // cvc5 accepts options only before the first assertion, so this must run
    // before search() — the factory calls it right after construction.
    //
    // Tuning is deliberately minimal: a fixed seed and the logic string.
    // No search-tuning options (restarts, phase selection, ...) have been
    // chosen yet; anything added here must be picked against cvc5's option
    // set and measured, not assumed.
    solver_.setOption("seed", "42");  // Fixed seed for reproducibility

    // Set the logic based on numeric constraint analysis.
    // When the problem is all-integer, use integer logics (QF_IDL/QF_LIA)
    // for tighter integer-specific reasoning.
    //
    // cvc5's setLogic is BINDING — asserting a term outside the declared
    // logic is an error, not a missed tuning opportunity. Difference-logic
    // profiles are therefore widened to LINEAR before mapping: QF_IDL/QF_RDL
    // admit only `x - y ~ c` and `x ~ c`, while the encoders freely emit
    // general linear terms and ite-sums (the cardinality helper's Int-sum
    // path alone puts `(+ (ite b 1 0) ...)` into any problem with a COUNT
    // bound > 1), so a binding QF_IDL would throw at assert time. Widening is
    // strictly more permissive — it can only cost tuning, never soundness.
    ArithmeticProfile profile = arithmetic_profile_;
    if (profile == ArithmeticProfile::DIFFERENCE_LOGIC) {
        profile = ArithmeticProfile::LINEAR;
    }
    const char* logic = recommended_logic(profile, problem_.all_integer());
    if (logic) {
        // The logic string must ALSO name the plan theory. cvc5's setLogic is
        // binding and setLogicString starts from all-theories-false, so a bare
        // "QF_LIA" *disables* THEORY_PLAN and the first PLAN_* atom throws
        // "the logic was specified as QF_LIA, which doesn't include
        // THEORY_PLAN". The fork's logic_info.cpp hunks (M3) add the "PLAN"
        // token for exactly this; splice it in after the "QF_" prefix, giving
        // e.g. QF_PLANLIA.
        //
        // "UF" is included unconditionally. The lifted visitor emits genuine
        // uninterpreted functions, and cvc5's own preprocessing introduces UF
        // terms (r2e fails with "doesn't include THEORY_UF, but got a
        // preprocessing-time term" otherwise). A wider logic can only cost
        // tuning, never soundness, so this is the safe direction.
        //
        // Token order follows logic_info.cpp's emitter: UF (:354) ... PLAN
        // (:388) ... arithmetic (:404), so QF_ + UF + PLAN + LIA parses.
        std::string s(logic);
        const std::string qf = "QF_";
        if (s.rfind(qf, 0) == 0) {
            s.insert(qf.size(), "UFPLAN");
        } else {
            s = "UFPLAN" + s;
        }
        solver_.setLogic(s);
    }
    // NOTE ArithmeticProfile::NONE yields no logic string at all, so cvc5
    // defaults to ALL -- which does include THEORY_PLAN, hence correct but
    // untuned. §6.7's Boolean-only-plus-PLAN logic is the follow-up.
}

void BasePlanner::init_deadline() {
    auto& config = Config::instance();
    if (config.global.timeout > 0) {
        deadline_ = std::chrono::steady_clock::now() +
                    std::chrono::seconds(config.global.timeout);
    } else {
        // No timeout — set deadline to max so remaining_ms() always returns large value
        deadline_ = std::chrono::steady_clock::time_point::max();
    }
}

unsigned BasePlanner::remaining_ms() const {
    if (deadline_ == std::chrono::steady_clock::time_point::max()) {
        return UINT_MAX;  // No timeout
    }
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline_) return 0;
    return static_cast<unsigned>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline_ - now).count());
}

void BasePlanner::start_watchdog(cvc5::Solver& solver) {
    if (watchdog_.joinable()) return;  // already running
    watchdog_ = std::thread([this, &solver] {
        std::unique_lock<std::mutex> lock(watchdog_mutex_);
        // Wake at the deadline, or earlier if stop_watchdog() is called.
        watchdog_cv_.wait_until(lock, deadline_, [this] { return watchdog_stop_; });
        if (!watchdog_stop_) {
            solver.interrupt();  // fork patch (M1): safe from another thread
        }
    });
}

void BasePlanner::stop_watchdog() {
    {
        std::lock_guard<std::mutex> lock(watchdog_mutex_);
        watchdog_stop_ = true;
    }
    watchdog_cv_.notify_all();
    if (watchdog_.joinable()) watchdog_.join();
}

bool BasePlanner::apply_solver_timeout(cvc5::Solver& solver) {
    unsigned ms = remaining_ms();
    if (ms == 0) {
        timed_out_ = true;
        Logger::instance().info("\n*** TIMEOUT ***");
        return false;
    }
    if (ms != UINT_MAX) {
        // cvc5 locks options once the solver is initialised, so a shrinking
        // per-check budget cannot be expressed with setOption ("tlimit-per"
        // would freeze the first check's budget for the whole search). A
        // watchdog thread expresses it faithfully: it interrupts at the
        // absolute deadline, so no single check can overrun the global budget.
        // handle_unknown_result maps the resulting
        // UnknownExplanation::INTERRUPTED onto timed_out_, exactly as it maps
        // TIMEOUT.
        start_watchdog(solver);
    }
    return true;
}

bool BasePlanner::handle_unknown_result(const cvc5::Result& result, const std::string& context) {
    // cvc5 reports the reason as an enum on the Result, so an exact
    // comparison suffices.
    cvc5::UnknownExplanation reason = result.getUnknownExplanation();
    if (reason == cvc5::UnknownExplanation::TIMEOUT ||
        reason == cvc5::UnknownExplanation::INTERRUPTED) {
        timed_out_ = true;
        Logger::instance().info("\n*** TIMEOUT during solve at " + context + " ***");
        return true;
    }
    // cvc5 declares the UnknownExplanation to_string overload in namespace std.
    Logger::instance().info("Solver returned unknown at " + context +
                            " (reason: " + std::to_string(reason) + ")");
    return false;
}

std::string BasePlanner::format_timeout_string() {
    auto& config = Config::instance();
    return config.global.timeout > 0
        ? std::to_string(config.global.timeout) + "s" : "none";
}

void BasePlanner::collect_statistics() {
    auto& stats = Stats::instance();

    cvc5::Statistics cvc5_stats = get_solver().getStatistics();
    for (auto it = cvc5_stats.begin(); it != cvc5_stats.end(); ++it) {
        const std::string& name = (*it).first;
        const cvc5::Stat& stat = (*it).second;
        std::string key = "cvc5." + name;

        if (stat.isInt()) {
            stats.set(key, static_cast<double>(stat.getInt()));
        } else if (stat.isDouble()) {
            stats.set(key, stat.getDouble());
        }
    }

    stats.set("memory.current_mb", MemoryTracker::instance().get_current_memory_mb());
}

void BasePlanner::add_timestep_constraints(cvc5::Solver& solver, GroundedEncoder& encoder,
                                           int timestep) {
    solver.assertFormula(encoder.encode_actions(timestep));
    solver.assertFormula(encoder.encode_frames(timestep));
    solver.assertFormula(encoder.encode_prefix_monotone(timestep));
    solver.assertFormula(encoder.encode_symmetries(timestep));
    solver.assertFormula(encoder.encode_state_constraints(timestep));

    // Parallelism is asserted unconditionally: the propagator hook that used to
    // gate this (if !propagator.manages_parallelism_constraints()) was removed
    // pending the in-solver reimplementation. Every strategy that remains
    // registered uses encoder-side parallelism and EAGER interference analysis
    // (strategy_registry.cpp), so eager clause emission is sound.
    solver.assertFormula(encoder.encode_parallelism(timestep));
}

} // namespace rantanplan
