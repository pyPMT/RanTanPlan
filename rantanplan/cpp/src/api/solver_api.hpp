#pragma once

#include "ir.hpp"
#include "preprocessing/preprocessed_problem.hpp"
#include "preprocessing/preprocessor_options.hpp"
#include "problem/problem.hpp"

#include <string>

namespace rantanplan::api {

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
 */
ir::ResultIR solve(PreprocessedProblem& pipeline_result);

/**
 * @brief Export the SMT formula for a fixed timestep instead of solving.
 *
 * Bypasses preprocessing entirely — operates on the raw loaded problem.
 * Timestep and strategy come from Config's formula-export section.
 * Returns the formula text; throws std::invalid_argument on bad options.
 */
std::string export_formula(const Problem& problem);

} // namespace rantanplan::api
