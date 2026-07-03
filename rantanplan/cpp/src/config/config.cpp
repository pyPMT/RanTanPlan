#include "config.hpp"
#include "cli_parser.hpp"
#include "strategy_registry.hpp"
#include "strategy_spec.hpp"
#include <stdexcept>

namespace rantanplan {

std::once_flag Config::initialized_flag_;
std::unique_ptr<Config> Config::instance_;

Config& Config::instance() {
    std::call_once(initialized_flag_, []() {
        instance_ = std::unique_ptr<Config>(new Config());
    });
    return *instance_;
}

void Config::initialize(int argc, char* argv[]) {
    CLIParser parser;
    parser.parse(*this, argc, argv);
    validate();
}

void Config::validate() const {
    // Validate strategy exists
    if (!StrategyRegistry::exists(planner.strategy)) {
        throw std::invalid_argument("Unknown strategy: " + planner.strategy +
                                   ". Use --list-strategies to see available options.");
    }

    // Validate horizon schedule name (catches typos before search begins)
    const auto& sched = planner.horizon_schedule;
    if (sched != "linear" && sched != "arithmetic" && sched != "geometric" && sched != "doubling") {
        throw std::invalid_argument(
            "Unknown horizon schedule: '" + sched +
            "'. Valid values: linear, arithmetic, geometric, doubling");
    }

    // Validate search mode against the strategy (throws on bad mode strings
    // and on incompatible combos, e.g. optimal + double-tail). Deeper spec
    // validation (StrategyFactory::validate) runs in the driver — the
    // factory lives in the api layer, which config/ must not depend on.
    const auto& spec = StrategyRegistry::get(planner.strategy);
    resolve_planner_kind(parse_search_mode(planner.mode), spec);

    // Validate global settings (0 = no timeout, any positive value is valid)
    if (global.timeout < 0) {
        throw std::invalid_argument("Timeout must be non-negative (0 = no timeout)");
    }

    if (planner.max_steps <= 0) {
        throw std::invalid_argument("Max steps must be positive");
    }
}

} // namespace rantanplan
