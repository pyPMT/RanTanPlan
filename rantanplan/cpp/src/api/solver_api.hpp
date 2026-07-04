#pragma once

#include "ir.hpp"
#include "preprocessing/preprocessed_problem.hpp"
#include "preprocessing/preprocessor_options.hpp"
#include "problem/problem.hpp"

#include <functional>
#include <string>

namespace rantanplan {
class Plan;
}

namespace rantanplan::api {

/// Callback invoked with each plan the planner reports (improving plans in
/// anytime mode, the final plan otherwise) — the in-process replacement for
/// polling the plan.txt.N files that --output-plan still writes.
using PlanCallback = std::function<void(const Plan&)>;

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
 */
ir::ResultIR solve(PreprocessedProblem& pipeline_result,
                   const PlanCallback& on_plan = {});

/**
 * @brief Export the SMT formula for a fixed timestep instead of solving.
 *
 * Bypasses preprocessing entirely — operates on the raw loaded problem.
 * Timestep and strategy come from Config's formula-export section.
 * Returns the formula text; throws std::invalid_argument on bad options.
 */
std::string export_formula(const Problem& problem);

} // namespace rantanplan::api
