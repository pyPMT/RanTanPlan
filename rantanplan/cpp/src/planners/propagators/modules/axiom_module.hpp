#pragma once

#include "../propagator_module.hpp"

#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rantanplan {

/**
 * @brief Phase-A axiom propagator for RECURSIVE derived-predicate strata.
 *
 * The encoder completes only acyclic strata (head <-> body); recursive strata
 * (SCC size > 1, or a self-loop) are left free and enforced here. On each full
 * candidate model this module checks, per layer, that the recursive derived
 * atoms equal the least fixpoint of their axioms over the assigned leaf atoms
 * (basic fluents, numeric-guard atoms, lower-stratum derived atoms), and blocks
 * any model that disagrees. See docs/specs/axioms-derived-predicates.md §6.1.
 *
 * No-op when the task has no recursive axioms.
 */
class AxiomModule : public PropagatorModule {
public:
    std::string get_name() const override { return "Axiom"; }

    void initialize(PropagatorSharedState& shared, PropagatorStrategy& host) override;
    void register_timestep_variables(int timestep) override;
    void on_fixed(const z3::expr& ast, const z3::expr& value) override;
    void on_final() override;

private:
    GroundedEncoder* encoder_ = nullptr;

    // Recursive heads -> their ground axiom bodies (disjuncts).
    std::map<ExprID, std::vector<ExprID>> recursive_by_head_;
    // Leaf atoms read from the model: recursive heads + every state-variable /
    // guard-comparison atom appearing in a recursive body. Sorted for determinism.
    std::vector<ExprID> read_atoms_;

    std::vector<int> layers_;                          // registered layers
    std::unordered_map<long long, int8_t> value_;      // (layer,atom) -> -1/0/1
    std::unordered_map<unsigned, long long> varid_key_; // z3 var id -> (layer,atom) key
    std::unordered_map<long long, z3::expr> key_var_;   // (layer,atom) -> z3 var (for conflict)

    static long long key(int layer, ExprID atom) {
        return (static_cast<long long>(layer) << 32) |
               static_cast<unsigned>(atom.id);
    }
    int8_t read(int layer, ExprID atom) const;
    void collect_read_atoms(ExprID e, std::unordered_set<ExprID>& out) const;
    bool eval(ExprID e, int layer, const std::unordered_map<ExprID, bool>& rec) const;
};

} // namespace rantanplan
