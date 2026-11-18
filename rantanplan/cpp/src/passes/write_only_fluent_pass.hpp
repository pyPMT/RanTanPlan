#pragma once

#include "pass.hpp"

namespace rantanplan {

/**
 * @brief Removes write-only fluents: the written fluents that nothing a plan
 *        must satisfy or pay for reads.
 *
 * A fluent is read if it occurs in a precondition, a goal, a declared state
 * invariant or an action cost, if its type declares bounds (the bounds are
 * checked in every state), or if it occurs in the value or the condition of
 * an effect on a read fluent. Every other written fluent is write-only, such
 * as an accumulator like (fuel-used) that only a final-value metric
 * mentions. The pass:
 *   1. Finds the read fluents (the read analysis)
 *   2. Removes the effects on write-only fluents and their initial values
 *   3. Leaves the actions with no effect left to EffectNormalizationPass,
 *      which runs next and drops them
 *
 * Sound in every search mode: no plan condition mentions a write-only fluent,
 * so a plan of the reduced problem is a plan of the original. The metrics the
 * planner optimises (action costs, plan length) read no fluent outside the
 * action costs, which count as read; final-value metrics, which would, are
 * rejected outside satisficing search (rantanplan/planner_wrapper.py). See
 * MetricKind for what a metric expression will need here.
 *
 * The gain is parallelism as much as size: two writes to one numeric fluent
 * cannot share a step (the Grounded encoder does not add them up), so a
 * global accumulator makes every pair of its writers interfere.
 *
 * Runs after StaticFluentPass (the problem is ground, so reads are per atom,
 * and folded conditions no longer read what they folded away) and before
 * EffectNormalizationPass and ActionPruningPass, which then never see these
 * fluents.
 */
class WriteOnlyFluentPass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "write-only-fluent"; }
};

} // namespace rantanplan
