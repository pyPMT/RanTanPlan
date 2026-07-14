#pragma once

#include "grounded_encoder.hpp"
#include "../analysis/axiom_dependency_analysis.hpp"

namespace rantanplan {

/**
 * @brief GroundedEncoder extended with PDDL axioms (derived predicates).
 *
 * Acyclic derived-predicate strata are encoded eagerly as per-layer completions
 * (head@t <-> OR of its axiom bodies@t). Recursive strata are left to the axiom
 * propagator (not yet wired). Derived atoms receive no frame axioms (they are
 * recomputed from basic fluents every layer). See
 * docs/specs/axioms-derived-predicates.md §7.
 */
class AxiomGroundedEncoder : public GroundedEncoder {
public:
    AxiomGroundedEncoder(const Problem& problem, z3::context& ctx,
                         std::unique_ptr<ParallelismStrategy> parallelism);

    void set_axiom_info(const AxiomInfo& info) override { axiom_info_ = info; }

    // Base action encoding plus the per-layer axiom-support constraints.
    z3::expr encode_actions(int t) override;

protected:
    bool gets_frame_axiom(ExprID eid) const override;

private:
    AxiomInfo axiom_info_;

    /// Completion clauses for acyclic derived predicates at layer t:
    /// head@t <-> (body_1@t OR ...). Recursive heads are skipped (propagator).
    z3::expr encode_axiom_support(int t);
};

} // namespace rantanplan
