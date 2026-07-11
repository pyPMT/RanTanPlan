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
 * (with a natural ordering).  For landmarks whose condition is a simple
 * numeric threshold (fluent >= c and variants) and whose achievers all have
 * constant additive effects on that fluent, the pass records per-achiever
 * contributions lambda(a,c) and the initial-state deficit t(c), and derives
 * a horizon lower bound ceil(t/Lambda) that is sound under parallel
 * cumulative semantics (all achievers firing every step).  Effects that are
 * assignments, state-dependent, nonlinear, or quantified invalidate the
 * count structure of the affected landmark (never just the achiever —
 * dropping one achiever would understate Lambda and overstate the bound).
 *
 * Contributes max(depth bounds) to PreprocessedProblem::lower_bound and
 * stores the full artifact in PreprocessedProblem::landmark_data.
 *
 * Requires: result.achievers (from ActionPruningPass).  No-op without it.
 */
class LandmarkAnalysisPass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "LandmarkAnalysis"; }
};

} // namespace rantanplan
