#pragma once

#include "pass.hpp"
#include <vector>
#include <unordered_set>

namespace rantanplan {

// ============================================================================
// MutexAnalysisPass
// ============================================================================
//
// Discovers mutex groups (at-most-one / exactly-one invariants over boolean
// fluents) from structural analysis of the grounded problem. These are
// injected into the SMT solver at every timestep to prune infeasible
// assignments. The numeric bounds, conservation laws, and object-fluent
// domains that share StateConstraints with them are published earlier, by
// ActionPruningPass.
//
// --- Mutex Groups (Tier 1 Syntactic Check) ---
//
// For boolean predicates with arity >= 2, we group ground instances by each
// argument position. For example, `located(plane1, city0..city5)` forms a
// group of 6 members sharing the entity `plane1`.
//
// A group is AT-MOST-ONE if no reachable state can have two members TRUE
// simultaneously. The syntactic check verifies this WITHOUT any SMT call
// by inspecting action effects:
//
//   Sufficient conditions for at-most-one:
//   1. No action has conditional effects on any group member.
//   2. Every action that sets a member TRUE also sets exactly one member FALSE.
//   3. No action sets more than one member TRUE simultaneously.
//
// These conditions guarantee the group cardinality never increases beyond 1.
// If the initial state has at most one TRUE member, the invariant holds.
// This is the same pattern Fast Downward's translator uses for SAS+ variables:
// movement actions like `drive(truck, from, to)` delete `located(truck, from)`
// and add `located(truck, to)` — a delete-add pair.
//
// A group is upgraded to EXACTLY-ONE if additionally:
//   - The initial state has exactly one TRUE member.
//   - Every action that deletes a member also adds one (cardinality never
//     decreases).
//
// Exactly-one groups give Z3 the equivalent of finite-domain variables
// (encode_atmost + encode_atleast). This is the primary driver of solver speedups.
//
// Candidates that fail Tier 1 get a second chance via the h² pairwise
// reachability fixpoint (H2MutexChecker), and h²-verified groups over
// different predicates of the same entity are merged into larger
// exactly-one groups.
//
// ============================================================================

/**
 * @brief Preprocessing pass that discovers mutex groups (no SMT).
 *
 * Results are appended to PreprocessedProblem::state_constraints.mutexes for
 * the encoder to inject at every timestep.
 */
class MutexAnalysisPass : public Pass {
public:
    MutexAnalysisPass() = default;

    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "mutex-analysis"; }

private:
    /// Generate mutex candidates by grouping boolean fluent instances
    /// by predicate and argument position.
    struct MutexCandidate {
        std::vector<ExprID> members;
        std::string entity;
        std::string predicate;
    };
    std::vector<MutexCandidate> generate_mutex_candidates(const Problem& problem) const;

    /// Tier 1 syntactic check: verify at-most-one by checking that every action
    /// that adds a member also deletes exactly one member, with no conditional effects.
    /// Returns true if at-most-one is verified.
    bool syntactic_mutex_check(const MutexCandidate& candidate,
                               const Problem& problem) const;

    /// Check if the group is exactly-one (syntactic): initial state has exactly
    /// one TRUE member, and every action that deletes a member also adds one.
    bool syntactic_exactly_one_check(const MutexCandidate& candidate,
                                     const Problem& problem,
                                     const std::unordered_set<ExprID>& initially_true) const;

};

} // namespace rantanplan
