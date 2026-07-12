#pragma once

#include "../config/search_mode.hpp"

/**
 * @file preprocessor_options.hpp
 * @brief Feature-level knobs for preprocess() — see PreprocessorOptions.
 */
namespace rantanplan {

class Config;

/**
 * @brief Explicit options for preprocess() — the preprocessor's only input
 *        besides the problem itself.
 *
 * The vocabulary is feature-level ("run symmetry detection", "cost fluents
 * in the RPG"); it never mentions encoders, semantics, or planner kinds.
 * The spec may inform options (the driver knows both), but options never
 * contain a spec.
 *
 * Single construction site: from_config(), called by the driver. Hand-built
 * instances are for library/embedding use.
 */
struct PreprocessorOptions {
    // --- Pass-list construction ---
    bool enable_grounding = true;    ///< Reachability grounding pass
    bool enable_symmetries = false;  ///< Symmetry detection + completion
    bool enable_cnf_normalization = true;  ///< CNF-normalize conditions after static folding

    // --- Action pruning (RPG + goal relevance) ---
    bool rpg_enabled = true;         ///< Run NumericRPG (lower bound + reachability)
    bool rpg_action_removal = true;  ///< Prune RPG-unreachable actions
    bool goal_relevance = true;      ///< Backward-prune goal-irrelevant actions
    int goal_relevance_max_iterations = 10; ///< Fixpoint limit for the prune loop

    // --- Invariant oracle (local reasoning) ---
    bool local_reasoning_enabled = true; ///< Master switch for state constraints
    bool rpg_bounds = true;          ///< Inject RPG fixpoint bounds per timestep
    bool mutex_discovery = true;     ///< Tier 1 syntactic mutex detection

    // --- The one semantic input ---
    /// Optimal/anytime search needs cost lower bounds, so the RPG and
    /// goal-relevance analysis must keep cost fluents alive.
    SearchMode mode = SearchMode::Satisficing;

    /// The single Config -> options mapping (anti-drift: no other
    /// construction site may read Config).
    static PreprocessorOptions from_config(const Config& config);
};

} // namespace rantanplan
