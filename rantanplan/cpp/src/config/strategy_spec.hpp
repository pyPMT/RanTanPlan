#pragma once

#include <set>

#include "search_mode.hpp"

/**
 * @file strategy_spec.hpp
 * @brief The strategy taxonomy: the enums a StrategySpec is built from, and
 *        pure derived queries over a spec (no virtuals needed).
 */
namespace rantanplan {

/// Parallelism semantics for the encoding: Sequential (one action per
/// timestep), Forall (all non-interfering actions may fire), Exists (any
/// consistent execution order of the chosen actions is a valid step).
/// Forall is kept as the sound fallback for SDAC cost evaluation (the
/// exists->forall downgrade in StrategyFactory::adjust_spec).
enum class SemanticsKind { Sequential, Forall, Exists };

/// How action interference is computed: EagerSyntactic (whole graph built
/// once, fluent-set overapproximation), LazySemantic (queried on demand,
/// solver-checked, precise). None for Sequential semantics.
enum class InterferenceKind { None, EagerSyntactic, LazySemantic };

/// Planner backend: Sequential drives satisficing search with a growing
/// horizon; BranchAndBound is the cost-optimal/anytime search.
enum class PlannerKind { Sequential, BranchAndBound };

/// Composable propagator module kinds.
enum class PropagatorModuleKind { ForallLazy, ExistsCycle, FrameAxiom };

using PropagatorFlags = std::set<PropagatorModuleKind>;

/// A named, validated combination of semantics/interference/propagator/
/// planner — the contributor unit for adding a new backend configuration
/// (see StrategyRegistry). All strategies use the grounded encoder.
struct StrategySpec {
    SemanticsKind semantics;
    InterferenceKind interference;
    PropagatorFlags propagator;
    PlannerKind planner;
};

// ---------------------------------------------------------------------------
// SearchMode helpers (SearchMode + parse_search_mode live in search_mode.hpp)
// ---------------------------------------------------------------------------

/// Resolve the final PlannerKind from the SearchMode and the StrategySpec.
///   satisficing → use the strategy's default planner.
///   optimal/anytime → always BranchAndBound.
inline PlannerKind resolve_planner_kind(SearchMode mode, const StrategySpec& spec) {
    if (mode == SearchMode::Satisficing) return spec.planner;
    return PlannerKind::BranchAndBound;
}

// ---------------------------------------------------------------------------
// Derived queries — pure functions of the spec, no virtuals needed.
// ---------------------------------------------------------------------------

/// Returns true if the spec's own planner field is BranchAndBound. Does not
/// account for a SearchMode override — see the overload below for that.
inline bool uses_branch_and_bound(const StrategySpec& spec) {
    return spec.planner == PlannerKind::BranchAndBound;
}

/// Overload that accounts for the search mode override.
inline bool uses_branch_and_bound(SearchMode mode, const StrategySpec& spec) {
    return resolve_planner_kind(mode, spec) == PlannerKind::BranchAndBound;
}

/// Returns true if SDAC cost evaluation is unsound with this strategy.
inline bool sdac_unsafe(const StrategySpec& spec) {
    return spec.semantics == SemanticsKind::Exists;
}

} // namespace rantanplan
