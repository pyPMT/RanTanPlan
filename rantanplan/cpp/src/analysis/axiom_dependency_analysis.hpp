#pragma once

#include <vector>
#include "../problem/problem.hpp"

namespace rantanplan {

/**
 * @file axiom_dependency_analysis.hpp
 * @brief Truth-Dependency-Graph analysis of a problem's (ground) axioms.
 *
 * TDG nodes are the ground derived atoms; an edge D -> H means a derived atom D
 * appears in the body of an axiom defining H. SCCs of this graph split the
 * derived predicates into strata handled two ways (see
 * docs/specs/axioms-derived-predicates.md §4):
 *   - acyclic  (singleton SCC, no self-loop) -> eager `head <-> body` completion
 *   - recursive (SCC size > 1, or a self-loop) -> lazy propagator evaluation
 *
 * Plain data only (no Z3), so AxiomInfo can live on the PreprocessedProblem
 * contract.
 */

/// Classification of the derived-predicate strata. `atoms` are the TDG nodes;
/// `scc_id`, `recursive` are parallel to it.
struct AxiomInfo {
    std::vector<ExprID> atoms;     ///< derived atoms (TDG nodes), sorted by ExprID
    std::vector<int> scc_id;       ///< scc_id[i] = SCC index of atoms[i]
    std::vector<bool> recursive;   ///< recursive[i] = atoms[i] sits in a recursive SCC
    int num_sccs = 0;
    bool stratified = true;        ///< false if a negated derived atom shares its head's SCC

    bool empty() const { return atoms.empty(); }
    /// Index of a derived atom in `atoms`, or -1 if it is not a TDG node.
    int index_of(ExprID atom) const;
    /// Whether `atom` is a derived atom in a recursive SCC (false if not a node).
    bool is_recursive(ExprID atom) const;
};

/// Build the TDG from `problem.axioms()`, compute SCCs, classify each derived
/// atom recursive/acyclic, and check stratification. Empty if no axioms.
AxiomInfo analyze_axioms(const Problem& problem);

} // namespace rantanplan
