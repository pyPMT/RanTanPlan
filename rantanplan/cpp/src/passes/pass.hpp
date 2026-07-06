#pragma once

#include <string>
#include "../preprocessing/preprocessed_problem.hpp"

/**
 * @file pass.hpp
 * @brief The Pass interface every preprocessing pass implements; see the
 *        pipeline order enforced in api::preprocess (api/solver_api.cpp).
 */
namespace rantanplan {

/**
 * @brief Abstract base class for preprocessing passes.
 *
 * A pass reads and updates a PreprocessedProblem in place. It may transform
 * the problem, set a lower bound, or prove unsolvability.
 *
 * Passes are stateless — all state flows through PreprocessedProblem.
 */
class Pass {
public:
    /// Read and update the pipeline result. Called exactly once per pipeline run.
    virtual void apply(PreprocessedProblem& result) const = 0;

    /// Human-readable name for logging and diagnostics.
    virtual std::string name() const = 0;

    virtual ~Pass() = default;
};

} // namespace rantanplan
