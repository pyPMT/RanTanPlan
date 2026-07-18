#pragma once

#include "expr_pool.hpp"

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rantanplan {

/// A small, stateful helper for building interned expression nodes in one
/// ExprPool. Centralises the boolean/relational/count construction that several
/// passes (quantifier expansion, set lowering, …) each used to re-implement:
/// cached operator head symbols, the and/or/not/implies/iff/eq/count builders
/// with constant folding, object constants, and child reinterning.
///
/// Head symbols use the standard operator spelling (`expr_operator_to_string`),
/// which is the form the IR loader normalises to (ir_io map_up_operator) — so
/// nodes built here dedup with the loaded formula's nodes. The op *field* is the
/// authoritative operator; the head string is display/dedup only.
///
/// Note: the pool interns append-only, so node-creation *order* is load-bearing
/// for downstream determinism (CLAUDE.md pitfall #7). This helper does not
/// change ordering — it interns in the same order the hand-written builders did.
class ExprBuilder {
public:
    explicit ExprBuilder(ExprPool& pool) : pool_(pool) {}

    ExprPool& pool() const { return pool_; }
    ExprID true_id() { return pool_.intern_bool_constant(true); }
    ExprID false_id() { return pool_.intern_bool_constant(false); }

    // ── boolean connectives (with constant folding) ─────────────────────────

    ExprID make_and(std::vector<ExprID> parts) {
        std::vector<ExprID> kept;
        kept.reserve(parts.size());
        for (ExprID p : parts) {
            if (pool_.is_false_constant(p)) return false_id();
            if (pool_.is_true_constant(p)) continue;
            kept.push_back(p);
        }
        if (kept.empty()) return true_id();
        if (kept.size() == 1) return kept.front();
        return make_app(ExprOperator::AND, std::move(kept));
    }

    ExprID make_or(std::vector<ExprID> parts) {
        std::vector<ExprID> kept;
        kept.reserve(parts.size());
        for (ExprID p : parts) {
            if (pool_.is_true_constant(p)) return true_id();
            if (pool_.is_false_constant(p)) continue;
            kept.push_back(p);
        }
        if (kept.empty()) return false_id();
        if (kept.size() == 1) return kept.front();
        return make_app(ExprOperator::OR, std::move(kept));
    }

    ExprID make_not(ExprID a) {
        if (pool_.is_true_constant(a)) return false_id();
        if (pool_.is_false_constant(a)) return true_id();
        return make_app(ExprOperator::NOT, {a});
    }

    ExprID make_implies(ExprID a, ExprID b) { return make_or({make_not(a), b}); }
    ExprID make_iff(ExprID a, ExprID b) {
        return make_and({make_implies(a, b), make_implies(b, a)});
    }

    /// Equality, folding two structurally-equal operands and two constant
    /// operands (object constants have a string payload; equal payloads are the
    /// same object) to a boolean.
    ExprID make_eq(ExprID a, ExprID b) {
        if (a == b) return true_id();
        if (pool_.is_constant(a) && pool_.is_constant(b) &&
            pool_.payload_is_string(a) && pool_.payload_is_string(b))
            return pool_.payload_string(a) == pool_.payload_string(b)
                       ? true_id() : false_id();
        return make_app(ExprOperator::EQUALS, {a, b});
    }

    /// Pseudo-boolean count of boolean arguments (no folding).
    ExprID make_count(std::vector<ExprID> parts) {
        return make_app(ExprOperator::COUNT, std::move(parts));
    }

    // ── generic construction ────────────────────────────────────────────────

    /// A function application: FUNCTION_APPLICATION[head, args...] with a cached
    /// operator head symbol.
    ExprID make_app(ExprOperator op, std::vector<ExprID> args, int type_id = -1) {
        ExprNode node;
        node.kind = static_cast<int>(ExprKind::FUNCTION_APPLICATION);
        node.op = static_cast<int>(op);
        node.type_id = type_id;
        node.children.reserve(args.size() + 1);
        node.children.push_back(op_head(op));
        for (ExprID a : args) node.children.push_back(a);
        return pool_.intern(std::move(node));
    }

    /// An object constant (CONSTANT kind, string payload = object name).
    ExprID object_constant(const std::string& name, int type_id) {
        ExprNode node;
        node.kind = static_cast<int>(ExprKind::CONSTANT);
        node.type_id = type_id;
        node.payload = name;
        return pool_.intern(std::move(node));
    }

    /// Rebuild `id` with replacement children, preserving kind/op/type_id/payload
    /// (the substitute()/structural-rewrite idiom).
    ExprID reintern(ExprID id, std::vector<ExprID> children) {
        ExprNode node;
        node.kind = static_cast<int>(pool_.kind(id));
        node.op = static_cast<int>(pool_.op(id));
        node.type_id = pool_.type_id(id);
        node.payload = pool_.payload(id);
        node.children = std::move(children);
        return pool_.intern(std::move(node));
    }

private:
    ExprID op_head(ExprOperator op) {
        auto it = op_heads_.find(static_cast<int>(op));
        if (it != op_heads_.end()) return it->second;
        ExprNode node;
        node.kind = static_cast<int>(ExprKind::FUNCTION_SYMBOL);
        node.op = static_cast<int>(op);
        node.payload = expr_operator_to_string(op);
        ExprID id = pool_.intern(std::move(node));
        op_heads_.emplace(static_cast<int>(op), id);
        return id;
    }

    ExprPool& pool_;
    std::unordered_map<int, ExprID> op_heads_;
};

} // namespace rantanplan
