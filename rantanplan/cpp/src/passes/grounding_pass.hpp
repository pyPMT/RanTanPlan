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
 *
 * INVARIANT ESTABLISHED (ground-atom shape): from this pass onward, every
 * STATE_VARIABLE's arguments are object constants (string-payload CONSTANT
 * nodes). Every downstream consumer silently assumes this — FactIndex
 * decomposition, ObjectFluentIndex, h2 mutexes, achievers, the RPG, the
 * encoders' frame/writer indices, interference analysis; the only runtime
 * check is the encoding visitor's "Function composition" throw
 * (grounded_encoding_visitor.cpp). A feature that would introduce
 * non-constant arguments (e.g. nested fluent terms) must compile them away
 * *before* this pass, not teach the consumers about them.
 */
class GroundingPass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "grounding"; }
};

} // namespace rantanplan
