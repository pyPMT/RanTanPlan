#include "preprocessor_options.hpp"

#include "../config/config.hpp"

namespace rantanplan {

PreprocessorOptions PreprocessorOptions::from_config(const Config& config) {
    PreprocessorOptions opts;
    opts.enable_grounding = config.global.reachability_grounding;
    opts.enable_symmetries = config.symmetry.enable_symmetries;
    opts.enable_cnf_normalization = config.global.cnf_normalization;
    opts.rpg_action_removal = config.rpg.action_removal;
    opts.goal_relevance = config.rpg.goal_relevance;
    opts.goal_relevance_max_iterations = config.rpg.goal_relevance_max_iterations;
    opts.mode = parse_search_mode(config.planner.mode);
    return opts;
}

} // namespace rantanplan
