#pragma once

#include "../config/strategy_spec.hpp"
#include "../problem/problem.hpp"
#include "../encoders/base_encoder.hpp"
#include "../planners/base_planner.hpp"
#include "../planners/propagators/propagator_strategy.hpp"
#include "../encoders/parallelism/parallelism_strategy.hpp"
#include "../preprocessing/preprocessed_problem.hpp"
#include <z3++.h>
#include <memory>
#include <string>
#include <vector>

/**
 * @file strategy_factory.hpp
 * @brief Assembles fully wired encoder/planner/propagator components from a
 *        StrategySpec at solve time.
 */
namespace rantanplan {

/// The problem uses a feature the selected strategy cannot handle soundly,
/// though another strategy can. api::solve reports it as UNSUPPORTED_PROBLEM
/// with this message; any other exception is an internal error.
class UnsupportedProblemError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};

/// Strategy component factory — handles validation, problem-specific
/// adjustments, and fully wired component assembly at solve time.
///
/// Wiring discipline: mandatory dependencies are constructor-injected
/// (interference → parallelism → encoder), optional artifacts (symmetry
/// data, state constraints) are applied via setters here, and planners
/// declare what they consume via BasePlanner::consume_artifacts. The one
/// ordering Z3 imposes — the propagator must attach to the planner's
/// solver, so create_propagator runs after create_planner — lives with
/// the caller (api::solve).
class StrategyFactory {
public:
    /// Validate that a spec is internally consistent and compatible with
    /// the given configuration options.
    /// Throws std::invalid_argument with a clear message on failure.
    static void validate(const StrategySpec& spec,
                         const std::string& strategy_name,
                         const std::string& horizon_schedule);

    /// Adjust a strategy spec for problem-specific constraints (e.g. SDAC +
    /// exists-step downgrade). Mutates spec in place and logs adjustments.
    /// Throws UnsupportedProblemError if the strategy cannot take the problem.
    static void adjust_spec(StrategySpec& spec, const Problem& problem);

    /// Build a fully wired encoder: interference analyzer (timed as
    /// backend.interference_ms) into parallelism strategy into encoder.
    static std::unique_ptr<BaseEncoder> create_encoder(
        const StrategySpec& spec, const Problem& problem, z3::context& ctx);

    /// As above, plus routes the preprocessing artifacts the encoder
    /// consumes: symmetry data (copied) and state constraints (moved).
    static std::unique_ptr<BaseEncoder> create_encoder(
        const StrategySpec& spec, PreprocessedProblem& pre, z3::context& ctx);

    /// The propagator hosting the spec's modules; null when it has none.
    static std::unique_ptr<PropagatorStrategy> create_propagator(
        const StrategySpec& spec, z3::solver& solver,
        const Problem& problem, BaseEncoder& encoder);

    /// Create the planner and route its artifacts: arithmetic profile,
    /// then BasePlanner::consume_artifacts (B&B takes SDAC cost bounds).
    static std::unique_ptr<BasePlanner> create_planner(
        const StrategySpec& spec, PreprocessedProblem& pre,
        BaseEncoder& encoder, z3::context& ctx);
};

} // namespace rantanplan
