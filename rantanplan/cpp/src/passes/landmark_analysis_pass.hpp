#pragma once

#include "pass.hpp"

/**
 * @file landmark_analysis_pass.hpp
 * @brief Extracts disjunctive action landmarks (with count structure where
 *        sound) from the achiever graph built by ActionPruningPass.
 */
namespace rantanplan {

/**
 * @brief Landmark extraction over the achiever graph.
 *
 * AND/OR closure: goal conditions not satisfied initially are landmarks; a
 * condition required by ALL achievers of a landmark is itself a landmark
 * (with a natural ordering).  For landmarks whose condition is a linear
 * numeric threshold (fluent >= c and variants) and whose fluents every
 * action changes by a constant additive amount, the pass records the
 * contribution lambda(a,c) of every action that moves the threshold towards
 * satisfaction (achiever or not: an action that can never make the condition
 * true can still add most of the deficit) and the initial-state deficit
 * t(c), and derives a horizon lower bound ceil(t/Lambda) that is sound under
 * parallel cumulative semantics (all contributors firing every step).  An
 * assignment, state-dependent, nonlinear, or quantified effect on those
 * fluents, by any action, invalidates the count structure of the landmark
 * (never just that action — dropping a contributor would understate Lambda
 * and overstate the bound).
 *
 * Contributes max(depth bounds) to PreprocessedProblem::lower_bound and
 * stores the full artifact in PreprocessedProblem::landmark_data.
 *
 * Requires: result.achievers (from ActionPruningPass), which only exists
 * when goal-relevance analysis is on — api::preprocess gates this pass on
 * that same option.  No-op without achievers.
 */
class LandmarkAnalysisPass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "landmark-analysis"; }
};

} // namespace rantanplan
