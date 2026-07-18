#pragma once

#include "pass.hpp"
#include "../problem/problem.hpp"

namespace rantanplan {

/// True if any fluent schema in the problem is a set fluent.
bool has_set_fluents(const Problem& problem);

/// Lower set fluents into boolean membership cells (docs/native-sets-plan.md,
/// encoding A). Each `s(params): set{T}` becomes a boolean membership fluent
/// `s(?e: T, params): bool` (the element type prepended to the signature), and
/// every set operation is rewritten through the one compositional membership
/// formula ⟦S⟧_e:
///   * boolean/int-valued operators in conditions/goals/invariants/costs —
///     `member` → the cell atom, `subseteq`/`disjoint`/set-equality → a
///     conjunction over the element domain, `cardinality` → a COUNT of the
///     membership formulas;
///   * set-valued effect RHS by shape — an add/remove chain on the target set
///     itself lowers to single-cell `s(x,args):=true/false` (no enumeration); a
///     monotone union onto the target to per-element conditional adds; anything
///     else (intersect/difference/third-set union, `s:={}`, `s:={c}`) to one
///     expression-valued boolean assign per element;
///   * a `SET_CONSTANT` initial value to the true membership cells (CWA falses
///     the rest).
/// The set fluent schemas are dropped from the returned problem.
///
/// Runs after `QuantifierExpansionPass` (quantified variables over element
/// types are already enumerated) and before `GroundingPass` (the grounder then
/// grounds the prepended element parameter like any object parameter). No-op
/// when the problem declares no set fluents.
///
/// Precondition: only invoked when the problem actually has set fluents.
Problem lower_sets(const Problem& problem);

/// Validation backstop: throw std::runtime_error if any expression referenced by
/// the problem still contains a SET_* node or a set-typed state variable. Called
/// at the end of SetLoweringPass so later work cannot silently regress the
/// "sets are fully lowered" invariant.
void assert_no_set_nodes(const Problem& problem);

/**
 * @brief Lower set fluents to boolean membership cells before grounding.
 *
 * Thin pipeline wrapper over lower_sets(); a no-op without set fluents. Ends
 * by asserting no SET_* node survives.
 *
 * Note: `api::preprocess` rejects set problems when native grounding is
 * disabled (--up-grounding) before any pass runs.
 */
class SetLoweringPass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "set-lowering"; }
};

} // namespace rantanplan
