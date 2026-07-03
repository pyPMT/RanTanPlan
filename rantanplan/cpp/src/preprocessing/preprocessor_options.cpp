#include "preprocessor_options.hpp"

#include "../config/config.hpp"

namespace rantanplan {

PreprocessorOptions PreprocessorOptions::from_config(const Config& config) {
    PreprocessorOptions opts;
    opts.enable_grounding = config.global.reachability_grounding;
    opts.enable_symmetries = config.symmetry.enable_symmetries;
    opts.rpg_enabled = config.rpg.enabled;
    opts.rpg_action_removal = config.rpg.action_removal;
    opts.goal_relevance = config.rpg.goal_relevance;
    opts.goal_relevance_max_iterations = config.rpg.goal_relevance_max_iterations;
    opts.local_reasoning_enabled = config.local_reasoning.enabled;
    opts.rpg_bounds = config.local_reasoning.rpg_bounds;
    opts.mutex_discovery = config.local_reasoning.mutex_discovery;
    opts.mode = parse_search_mode(config.planner.mode);
    return opts;
}

} // namespace rantanplan
