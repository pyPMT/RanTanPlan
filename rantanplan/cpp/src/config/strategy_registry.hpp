#pragma once

#include "strategy_spec.hpp"
#include <map>
#include <vector>
#include <string>

/**
 * @file strategy_registry.hpp
 * @brief Maps string strategy names (CLI --strategy values) to StrategySpec
 *        instances. Adding a strategy means adding one entry here — see
 *        strategy_registry.cpp for the full table.
 */
namespace rantanplan {

/// One named, described strategy: the spec plus a human-readable summary
/// shown by --list-strategies.
struct StrategyEntry {
    StrategySpec spec;
    std::string description;
};

class StrategyRegistry {
public:
    /// Look up a strategy spec by name.
    /// Throws std::invalid_argument if the name is unknown.
    static const StrategySpec& get(const std::string& name);

    /// Returns true if `name` is a registered strategy.
    static bool exists(const std::string& name);

    /// Names of all registered strategies (the source test.py and the
    /// nanobind module's list_strategies() both discover strategies from).
    static std::vector<std::string> list_strategies();

    /// Return the full registry (name -> entry with spec + description).
    static const std::map<std::string, StrategyEntry>& get_registry();
};

} // namespace rantanplan
