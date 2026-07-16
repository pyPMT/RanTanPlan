#pragma once

#include "pass.hpp"

namespace rantanplan {

/**
 * @brief Reachability grounding pass.
 *
 * Runs the delete-relaxation fixpoint grounder on a lifted problem,
 * producing a ground problem where every action is fully instantiated.
 * Because the fixpoint is a reachability analysis, grounding doubles as
 * pruning (only reachable instantiations are emitted) and as a deduction
 * step (layer depth, goal reachability).
 *
 * Results:
 *   - problem is replaced with the grounded version
 *   - lower_bound is set from the fixpoint layer depth
 *   - proven_unsolvable is set if goal facts are unreachable
 */
class GroundingPass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "grounding"; }
};

} // namespace rantanplan
