#pragma once

#include "pass.hpp"
#include "../preprocessing/preprocessor_options.hpp"

namespace rantanplan {

/**
 * @brief Unified action pruning pass (merges NumericRPG + GoalRelevance).
 *
 * Runs a fixpoint loop that alternates forward pruning (RPG-unreachable
 * actions) and backward pruning (goal-irrelevant actions via achiever
 * analysis). The RPG always runs to fixpoint for tightest possible bounds.
 *
 * Produces:
 *   - fixpoint_rpg: final RPG on the pruned problem
 *   - tightened_bounds: numeric bounds (RPG + syntactic + conservation-derived)
 *   - conservation_laws: discovered f + g == C invariants
 *   - achievers: goal-relevant achiever analysis (for the CGB planner)
 *   - sdac_cost_lower_bounds: cost bounds on the FINAL action set
 *
 * Runs when any of rpg_enabled, goal_relevance, or
 * (local_reasoning_enabled && rpg_bounds) is set in the options.
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
