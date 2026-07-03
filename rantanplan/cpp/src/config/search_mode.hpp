#pragma once

#include <stdexcept>
#include <string>

namespace rantanplan {

/// Search mode — orthogonal to strategy (encoding/semantics/interference/propagator).
///   Satisficing: find first valid plan, return immediately.
///   Optimal:     find cost-optimal plan with proof (BranchAndBound).
///   Anytime:     keep finding improving plans via B&B, return best at end.
enum class SearchMode { Satisficing, Optimal, Anytime };

inline SearchMode parse_search_mode(const std::string& mode) {
    if (mode.empty() || mode == "satisficing") return SearchMode::Satisficing;
    if (mode == "optimal")  return SearchMode::Optimal;
    if (mode == "anytime")  return SearchMode::Anytime;
    throw std::invalid_argument(
        "Unknown mode: '" + mode + "'. Valid values: satisficing, optimal, anytime");
}

} // namespace rantanplan
