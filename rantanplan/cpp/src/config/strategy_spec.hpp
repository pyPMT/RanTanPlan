#pragma once

#include <set>
#include <string>
#include <stdexcept>

#include "search_mode.hpp"

/**
 * @file strategy_spec.hpp
 * @brief The strategy taxonomy: the enums a StrategySpec is built from, and
 *        pure derived queries over a spec (no virtuals needed).
 */
namespace rantanplan {

/// SAT encoder family: Grounded (one clause set per ground action), Chained
/// (frame-axiom chaining, reused across elsc/elsc-dt/cgb), R2E (relaxed-to-exact).
enum class EncoderFamily { Grounded, Chained, R2E };

/// Parallelism semantics for the encoding: Sequential (one action per
/// timestep), Forall (all non-interfering actions may fire), Exists (any
/// consistent execution order of the chosen actions is a valid step).
enum class SemanticsKind { Sequential, Forall, Exists };

/// How action interference is computed: eager (built once, whole graph)
/// vs. lazy (queried on demand), syntactic (fluent-set overapproximation)
/// vs. semantic (Z3-checked, precise). None for Sequential semantics.
enum class InterferenceKind { None, EagerSyntactic, EagerSemantic, LazySyntactic, LazySemantic };

/// Planner backend: Sequential/DoubleTail drive satisficing search with a
/// growing horizon; BranchAndBound is the cost-optimal/anytime search;
/// CGB activates ground actions lazily under Exists semantics, with
/// growth named by unsat cores; WindowedCommitment solves short windows
/// from concrete anchors and commits stepping stones under assumptions,
/// falling back to the monolithic incremental search on exhaustion.
enum class PlannerKind { Sequential, DoubleTail, BranchAndBound, CGB, WindowedCommitment };

/// Composable propagator module kinds.
enum class PropagatorModuleKind { ForallEager, ForallLazy, ExistsCycle, FrameAxiom };

using PropagatorFlags = std::set<PropagatorModuleKind>;

/// A named, validated combination of encoder/semantics/interference/
/// propagator/planner — the contributor unit for adding a new backend
/// configuration (see StrategyRegistry).
struct StrategySpec {
    EncoderFamily encoder;
    SemanticsKind semantics;
    InterferenceKind interference;
    PropagatorFlags propagator;
    PlannerKind planner;
};

// ---------------------------------------------------------------------------
// SearchMode helpers (SearchMode + parse_search_mode live in search_mode.hpp)
// ---------------------------------------------------------------------------

/// Resolve the final PlannerKind from the SearchMode and the StrategySpec.
///   satisficing → use the strategy's default planner (Sequential or DoubleTail).
///   optimal/anytime → always BranchAndBound; error if strategy is DoubleTail.
inline PlannerKind resolve_planner_kind(SearchMode mode, const StrategySpec& spec) {
    if (mode == SearchMode::Satisficing) return spec.planner;
    if (spec.planner == PlannerKind::DoubleTail) {
        std::string mode_str = (mode == SearchMode::Optimal) ? "optimal" : "anytime";
        throw std::invalid_argument(
            "Mode '" + mode_str + "' is incompatible with double-tail strategies. "
            "Use a non-dt strategy (e.g., 'fl' instead of 'fl-dt').");
    }
    if (spec.planner == PlannerKind::CGB) {
        std::string mode_str = (mode == SearchMode::Optimal) ? "optimal" : "anytime";
        throw std::invalid_argument(
            "Mode '" + mode_str + "' is incompatible with the CGB strategy.");
    }
    if (spec.planner == PlannerKind::WindowedCommitment) {
        // Structurally satisficing-only: committed prefixes are not improving
        // plans, so the anytime/optimal callback contract cannot be honored.
        std::string mode_str = (mode == SearchMode::Optimal) ? "optimal" : "anytime";
        throw std::invalid_argument(
            "Mode '" + mode_str + "' is incompatible with windowed-commitment "
            "strategies. Use --mode satisficing (the default).");
    }
    return PlannerKind::BranchAndBound;
}

// ---------------------------------------------------------------------------
// Derived queries — pure functions of the spec, no virtuals needed.
// ---------------------------------------------------------------------------

/// Returns true if the strategy can export its SMT formula for a fixed
/// timestep instead of solving (no propagator, plain sequential planner).
inline bool supports_formula_export(const StrategySpec& spec) {
    return spec.propagator.empty() &&
           spec.planner == PlannerKind::Sequential;
}

/// Returns true if the strategy uses the double-tail planner (incompatible
/// with non-linear horizon schedules and with optimal/anytime search modes).
inline bool uses_double_tail(const StrategySpec& spec) {
    return spec.planner == PlannerKind::DoubleTail;
}

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
