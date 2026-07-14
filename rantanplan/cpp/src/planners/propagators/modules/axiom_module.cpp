#include "axiom_module.hpp"

#include "../propagator_strategy.hpp"
#include "../../../analysis/axiom_dependency_analysis.hpp"
#include "../../../problem/expr_enums.hpp"

#include <algorithm>

namespace rantanplan {

void AxiomModule::initialize(PropagatorSharedState& shared, PropagatorStrategy& host) {
    PropagatorModule::initialize(shared, host);
    encoder_ = dynamic_cast<GroundedEncoder*>(shared.encoder);

    AxiomInfo info = analyze_axioms(*shared.problem);
    std::unordered_set<ExprID> read_set;
    for (const auto& axiom : shared.problem->axioms()) {
        if (!info.is_recursive(axiom.head)) continue;  // acyclic strata: encoder
        recursive_by_head_[axiom.head].push_back(axiom.body);
        read_set.insert(axiom.head);
        collect_read_atoms(axiom.body, read_set);
    }
    read_atoms_.assign(read_set.begin(), read_set.end());
    std::sort(read_atoms_.begin(), read_atoms_.end());
}

void AxiomModule::collect_read_atoms(ExprID e, std::unordered_set<ExprID>& out) const {
    const ExprNode& node = shared_->problem->pool().get(e);
    auto kind = static_cast<ExprKind>(node.kind);
    if (kind == ExprKind::STATE_VARIABLE) { out.insert(e); return; }
    if (kind != ExprKind::FUNCTION_APPLICATION) return;  // constant, etc.
    auto op = static_cast<ExprOperator>(node.op);
    if (is_comparison_operator(op)) { out.insert(e); return; }  // numeric-guard leaf
    for (size_t i = 1; i < node.children.size(); ++i)  // child[0] is the head symbol
        collect_read_atoms(node.children[i], out);
}

void AxiomModule::register_timestep_variables(int timestep) {
    if (recursive_by_head_.empty() || !encoder_) return;
    layers_.push_back(timestep);
    for (ExprID atom : read_atoms_) {
        z3::expr v = encoder_->convert_expr_id_to_z3(atom, timestep);
        long long k = key(timestep, atom);
        varid_key_[v.id()] = k;
        key_var_.emplace(k, v);
        value_[k] = -1;
        host_->module_add(v);
    }
}

void AxiomModule::on_fixed(const z3::expr& ast, const z3::expr& value) {
    auto it = varid_key_.find(ast.id());
    if (it == varid_key_.end()) return;
    value_[it->second] = value.is_true() ? 1 : 0;
}

int8_t AxiomModule::read(int layer, ExprID atom) const {
    auto it = value_.find(key(layer, atom));
    return it == value_.end() ? int8_t(-1) : it->second;
}

bool AxiomModule::eval(ExprID e, int layer,
                       const std::unordered_map<ExprID, bool>& rec) const {
    const ExprNode& node = shared_->problem->pool().get(e);
    auto kind = static_cast<ExprKind>(node.kind);
    if (kind == ExprKind::STATE_VARIABLE) {
        auto rit = rec.find(e);            // recursive atom -> fixpoint value
        if (rit != rec.end()) return rit->second;
        return read(layer, e) == 1;        // leaf -> model value
    }
    if (kind == ExprKind::CONSTANT)
        return std::holds_alternative<bool>(node.payload) && std::get<bool>(node.payload);
    if (kind != ExprKind::FUNCTION_APPLICATION) return false;

    auto op = static_cast<ExprOperator>(node.op);
    if (is_comparison_operator(op)) return read(layer, e) == 1;  // guard -> model

    const auto& c = node.children;  // c[0] is the head symbol
    switch (op) {
        case ExprOperator::NOT:
            return c.size() >= 2 ? !eval(c[1], layer, rec) : false;
        case ExprOperator::AND:
            for (size_t i = 1; i < c.size(); ++i)
                if (!eval(c[i], layer, rec)) return false;
            return true;
        case ExprOperator::OR:
            for (size_t i = 1; i < c.size(); ++i)
                if (eval(c[i], layer, rec)) return true;
            return false;
        case ExprOperator::IMPLIES:
            return c.size() >= 3 ? (!eval(c[1], layer, rec) || eval(c[2], layer, rec)) : true;
        case ExprOperator::IFF:
            return c.size() >= 3 ? (eval(c[1], layer, rec) == eval(c[2], layer, rec)) : true;
        default:
            // Quantifiers (lifted axioms, deferred to milestone 9): fall back to
            // the whole node's model value if it happens to be a registered atom.
            return read(layer, e) == 1;
    }
}

void AxiomModule::on_final() {
    if (recursive_by_head_.empty()) return;
    for (int t : layers_) {
        // Least fixpoint of the recursive atoms over the layer's leaf reads.
        std::unordered_map<ExprID, bool> rec;
        for (const auto& [head, bodies] : recursive_by_head_) rec[head] = false;
        bool changed = true;
        while (changed) {
            changed = false;
            for (const auto& [head, bodies] : recursive_by_head_) {
                if (rec[head]) continue;
                for (ExprID body : bodies) {
                    if (eval(body, t, rec)) { rec[head] = true; changed = true; break; }
                }
            }
        }
        // Block the layer if any assigned recursive head disagrees with the fixpoint.
        for (const auto& [head, bodies] : recursive_by_head_) {
            int8_t assigned = read(t, head);
            if (assigned < 0) continue;                 // free (unreferenced) head
            if ((assigned == 1) == rec[head]) continue; // matches fixpoint
            z3::expr_vector fixed(host_->z3_ctx());
            for (ExprID atom : read_atoms_) fixed.push_back(key_var_.at(key(t, atom)));
            host_->module_conflict(fixed);
            return;
        }
    }
}

} // namespace rantanplan
