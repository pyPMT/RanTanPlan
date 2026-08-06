#pragma once

#include "../config/strategy_spec.hpp"
#include "../problem/problem.hpp"
#include "../encoders/base_encoder.hpp"
#include "../planners/base_planner.hpp"
#include "../encoders/parallelism/parallelism_strategy.hpp"
#include "../preprocessing/preprocessed_problem.hpp"
#include "../smtcore/smt_types.hpp"
#include <memory>
#include <string>
#include <vector>

/**
 * @file strategy_factory.hpp
 * @brief Assembles fully wired encoder/planner/propagator components from a
 *        StrategySpec at solve time.
 */
namespace rantanplan {

/// Strategy component factory — handles validation, problem-specific
/// adjustments, and fully wired component assembly at solve time.
///
/// Wiring discipline: mandatory dependencies are constructor-injected
/// (interference → parallelism → encoder), optional artifacts (symmetry
/// data, state constraints) are applied via setters here, and planners
/// declare what they consume via BasePlanner::consume_artifacts. The one
/// ordering Z3 imposes — the propagator must attach to the planner's
/// solver, so the propagator ran after create_planner — lives with
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
    static void adjust_spec(StrategySpec& spec, const Problem& problem);

    /// Build a fully wired encoder: interference analyzer (timed as
    /// backend.interference_ms) into parallelism strategy into encoder.
    static std::unique_ptr<BaseEncoder> create_encoder(
        const StrategySpec& spec, const Problem& problem, cvc5::TermManager& ctx);

    /// As above, plus routes the preprocessing artifacts the encoder
    /// consumes: symmetry data (copied) and state constraints (moved).
    static std::unique_ptr<BaseEncoder> create_encoder(
        const StrategySpec& spec, PreprocessedProblem& pre, cvc5::TermManager& ctx);


    /// Create the planner and route its artifacts: arithmetic profile,
    /// then BasePlanner::consume_artifacts (CGB takes achievers, B&B
    /// takes SDAC cost bounds).
    static std::unique_ptr<BasePlanner> create_planner(
        const StrategySpec& spec, PreprocessedProblem& pre,
        BaseEncoder& encoder, cvc5::TermManager& ctx);
};

} // namespace rantanplan
