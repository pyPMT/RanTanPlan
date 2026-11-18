#pragma once

#include "pass.hpp"
#include "../preprocessing/preprocessor_options.hpp"

namespace rantanplan {

/**
 * @brief Unified action pruning pass (merges NumericRPG + GoalRelevance).
 *
 * Runs a fixpoint loop that alternates forward pruning (RPG-unreachable
 * actions) and backward pruning (goal-irrelevant actions via achiever
 * analysis). The RPG always runs to fixpoint for tightest possible bounds;
 * the final RPG stays local to the pass.
 *
 * Produces:
 *   - state_constraints.bounds: numeric bounds (RPG + syntactic +
 *     conservation-derived), encoder-ready
 *   - state_constraints.conservations: discovered f + g == C invariants
 *   - state_constraints.domains: reachable object-fluent value sets
 *     (MutexAnalysisPass later adds state_constraints.mutexes)
 *   - achievers: goal-relevant achiever analysis (for the CGB planner and
 *     LandmarkAnalysisPass)
 *   - sdac_cost_lower_bounds: cost bounds on the FINAL action set
 *
 * Always runs: the published bounds/domains are unconditionally injected at
 * every timestep downstream, so the RPG is always built here to produce them.
 *
 * The last pass that changes the action set, so action ids are final after
 * it (see Problem's action access). Both pruning steps renumber the actions
 * they keep (Problem::without_actions); the RPG's first layers, taken before
 * the forward step and handed to AchieversAnalysis after it, are re-keyed to
 * the new ids there. Everything else the loop keeps is rebuilt from the
 * pruned problem.
 */
class ActionPruningPass : public Pass {
public:
    explicit ActionPruningPass(const PreprocessorOptions& opts) : opts_(&opts) {}

    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "action-pruning"; }

private:
    const PreprocessorOptions* opts_;
};

} // namespace rantanplan
