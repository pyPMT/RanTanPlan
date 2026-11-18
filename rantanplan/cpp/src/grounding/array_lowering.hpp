#pragma once

#include "../problem/problem.hpp"

namespace rantanplan {

/// Phase 1 array lowering (constant-index subset — rules L1/L3/L5 of
/// docs/native-arrays-plan.md).
///
/// Invoked by ArrayLoweringPass (passes/array_lowering_pass.hpp), which runs
/// immediately before GroundingPass — i.e. *before* the reachability
/// fixpoint, on the already int-instantiated, quantifier-expanded lifted
/// problem. By that point every array index is ground constant arithmetic
/// (object parameters never appear in indices), so each array access folds
/// to a single scalar cell. The implementation lives next to the grounder
/// because Phase 2 (symbolic indices) interleaves lowering with the
/// reachability fixpoint itself.
///
/// The transform:
///   * mints one ordinary scalar cell fluent per defined array position, in
///     row-major position order;
///   * rewrites every `ARRAY_INDEX(a, c)` (L1 read / L3 write target) into its
///     cell fluent, in preconditions, effect conditions, effect RHS, costs,
///     goals, state invariants, and initial-state targets;
///   * discards a ground action whose folded index lands out of bounds or on
///     an undefined position (strict-⊥ inapplicability), except under OR/COUNT
///     and in conditional effects: one with an undefined target or value is
///     dropped and its condition negated into the precondition;
///     two effects writing one cell are kept, for EffectNormalizationPass to
///     combine;
///   * hard-errors on an out-of-bounds / undefined goal or initial-state
///     access (the model itself is ill-formed);
///   * rejects whole-array operations (`eq(a,b)`, whole-array assign — L5)
///     with a clear message.
///
/// After it runs no `ARRAY_INDEX` node is referenced anywhere: cells are
/// ordinary scalar grounded fluents and every downstream pass/encoder is
/// unchanged. The array fluent schemas themselves are dropped from the
/// returned problem.
///
/// Precondition: the caller only invokes this when the problem actually has
/// array fluents (see has_array_fluents).
Problem lower_arrays(const Problem& problem);

/// True if any fluent schema in the problem is an array fluent.
bool has_array_fluents(const Problem& problem);

/// Validation backstop: throw std::runtime_error if any expression referenced
/// by the problem (actions, goals, invariants, initial state) still contains
/// an ARRAY_INDEX node. Called at the end of ArrayLoweringPass so later work
/// cannot silently regress the "arrays are fully lowered" invariant.
void assert_no_array_index(const Problem& problem);

} // namespace rantanplan
