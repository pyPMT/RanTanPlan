#include "axiom_grounded_encoder.hpp"

#include <map>
#include <vector>

namespace rantanplan {

AxiomGroundedEncoder::AxiomGroundedEncoder(const Problem& problem, z3::context& ctx,
                                           std::unique_ptr<ParallelismStrategy> parallelism)
    : GroundedEncoder(problem, ctx, std::move(parallelism)) {}

bool AxiomGroundedEncoder::gets_frame_axiom(ExprID eid) const {
    const Type* t = problem_.type_for_id(eid);
    return !(t && t->is_derived_bool());
}

z3::expr AxiomGroundedEncoder::encode_actions(int t) {
    return GroundedEncoder::encode_actions(t) && encode_axiom_support(t);
}

z3::expr AxiomGroundedEncoder::encode_axiom_support(int t) {
    // Group axiom bodies by head; std::map keeps a deterministic (ExprID) order
    // so clause emission — and thus the plan Z3 finds — is stable across runs.
    std::map<ExprID, std::vector<ExprID>> by_head;
    for (const auto& axiom : problem_.axioms()) {
        by_head[axiom.head].push_back(axiom.body);
    }

    z3::expr_vector clauses(ctx_);
    for (const auto& [head, bodies] : by_head) {
        if (axiom_info_.is_recursive(head)) continue;  // recursive strata: propagator
        z3::expr head_z3 = convert_expr_id_to_z3(head, t);
        z3::expr_vector body_terms(ctx_);
        for (ExprID body : bodies) {
            body_terms.push_back(convert_expr_id_to_z3(body, t));
        }
        z3::expr body_or =
            body_terms.empty() ? ctx_.bool_val(false) : z3::mk_or(body_terms);
        clauses.push_back(head_z3 == body_or);  // completion: head <-> body
    }

    if (clauses.empty()) return ctx_.bool_val(true);
    return z3::mk_and(clauses);
}

} // namespace rantanplan
