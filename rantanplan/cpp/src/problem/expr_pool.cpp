#include "expr_pool.hpp"
#include "type.hpp"
#include <sstream>

namespace rantanplan {

std::string ExprNode::canonical_key() const {
    // Format: "kind:op:type_id|payload_tag:payload_val|c0,c1,..."
    std::string key;
    key += std::to_string(static_cast<int>(kind));
    key += ':';
    key += std::to_string(static_cast<int>(op));
    key += ':';
    key += std::to_string(type_id);
    key += '|';

    std::visit([&](auto&& val) {
        using T = std::decay_t<decltype(val)>;
        if constexpr (std::is_same_v<T, std::monostate>) {
            key += "n:";
        } else if constexpr (std::is_same_v<T, bool>) {
            key += "b:";
            key += val ? '1' : '0';
        } else if constexpr (std::is_same_v<T, int64_t>) {
            key += "i:";
            key += std::to_string(val);
        } else if constexpr (std::is_same_v<T, double>) {
            key += "d:";
            key += std::to_string(val);
        } else if constexpr (std::is_same_v<T, std::string>) {
            key += "s:";
            key += std::to_string(val.size());
            key += ':';
            key += val;
        }
    }, payload);

    key += '|';
    for (size_t i = 0; i < children.size(); ++i) {
        if (i > 0) key += ',';
        key += std::to_string(children[i].id);
    }
    return key;
}

std::string ExprPool::state_variable_signature(const ExprNode& node) const {
    if (node.kind != ExprKind::STATE_VARIABLE) return {};
    if (node.children.empty()) return {};

    // GROUND atoms only: children[0] a fluent symbol, children[1..] all
    // CONSTANTs (objects or integers). A lifted node (PARAMETER or VARIABLE arguments) is excluded
    // deliberately — it never becomes a Z3 variable, since grounding replaces it
    // first, so it cannot participate in the id-vs-name mismatch this guards
    // against. Without that restriction the signature also over-merges: a schema
    // `next(?t1, ?t2)` and the ground `next(t1, t2)` have identical payloads and
    // would be reported as aliases of each other, which is noise, not a hazard.
    if (nodes_[node.children[0].id].kind != ExprKind::FLUENT_SYMBOL) {
        return {};
    }
    std::string signature;
    for (size_t i = 0; i < node.children.size(); ++i) {
        ExprID child = node.children[i];
        if (!child.valid() || static_cast<size_t>(child.id) >= nodes_.size()) return {};
        const ExprNode& arg = nodes_[child.id];
        if (i > 0 && arg.kind != ExprKind::CONSTANT) return {};
        if (std::holds_alternative<std::string>(arg.payload)) {
            signature += std::get<std::string>(arg.payload);
        } else if (i > 0 && std::holds_alternative<int64_t>(arg.payload)) {
            // An integer argument by its digits, whatever the constant's type.
            signature += std::to_string(std::get<int64_t>(arg.payload));
        } else {
            return {};
        }
        signature += '\0';  // unambiguous separator: object names cannot contain it
    }
    return signature;
}

ExprID ExprPool::intern(ExprNode node) {
    assert(node.kind != ExprKind::UNKNOWN);

    // These fields follow from the node's head or value, not from the caller,
    // so that no producer can give one atom or value a second type, or an
    // application an operator other than its head's.
    const ExprNode* head = nullptr;
    if (!node.children.empty() && node.children[0].valid() &&
        static_cast<size_t>(node.children[0].id) < nodes_.size()) {
        head = &nodes_[node.children[0].id];
    }
    // A state variable has its head's type: the fluent's value type.
    if (node.kind == ExprKind::STATE_VARIABLE && head && head->kind == ExprKind::FLUENT_SYMBOL) {
        node.type_id = head->type_id;
    }
    // An application has its head's operator.
    if (node.kind == ExprKind::FUNCTION_APPLICATION && head &&
        head->kind == ExprKind::FUNCTION_SYMBOL) {
        node.op = head->op;
    }
    // A constant has its value's type. An object constant (a string payload)
    // keeps the object's type, which only the caller knows.
    if (node.kind == ExprKind::CONSTANT) {
        if (std::holds_alternative<bool>(node.payload)) node.type_id = BOOL_TYPE_ID;
        else if (std::holds_alternative<int64_t>(node.payload)) node.type_id = INT_TYPE_ID;
        else if (std::holds_alternative<double>(node.payload)) node.type_id = REAL_TYPE_ID;
    }

    std::string key = node.canonical_key();
    auto it = intern_.find(key);
    if (it != intern_.end()) {
        return it->second;
    }
    ExprID new_id{static_cast<int32_t>(nodes_.size())};

    // Record which ground atom this node denotes, before the node is moved from.
    // Done here rather than in a cache built later because passes keep interning
    // throughout preprocessing, so anything built once would go stale.
    std::string signature = state_variable_signature(node);
    canonical_sv_.resize(static_cast<size_t>(new_id.id) + 1, EXPR_NULL);
    if (!signature.empty()) {
        auto sig_it = sv_signature_.find(signature);
        if (sig_it == sv_signature_.end()) {
            sv_signature_.emplace(std::move(signature), new_id);  // first of its kind
        } else {
            // Same ground atom, different node — differing only in a type_id
            // somewhere. Point at the first one; see ExprPool's "Ground-atom
            // identity" comment for who depends on this.
            canonical_sv_[new_id.id] = sig_it->second;
            ++aliased_sv_count_;
        }
    }

    nodes_.push_back(std::move(node));
    intern_[std::move(key)] = new_id;
    return new_id;
}

std::string ExprPool::to_string(ExprID id) const {
    if (!id.valid()) return "<null>";

    const ExprNode& node = get(id);

    // Leaf: payload is the display value
    if (node.children.empty()) {
        return std::visit([](auto&& val) -> std::string {
            using T = std::decay_t<decltype(val)>;
            if constexpr (std::is_same_v<T, std::monostate>) {
                return "()";
            } else if constexpr (std::is_same_v<T, bool>) {
                return val ? "true" : "false";
            } else if constexpr (std::is_same_v<T, int64_t>) {
                return std::to_string(val);
            } else if constexpr (std::is_same_v<T, double>) {
                return std::to_string(val);
            } else if constexpr (std::is_same_v<T, std::string>) {
                return val;
            }
        }, node.payload);
    }

    // Compound: (child0 child1 ...)
    std::ostringstream oss;
    oss << "(";
    for (size_t i = 0; i < node.children.size(); ++i) {
        if (i > 0) oss << " ";
        oss << to_string(node.children[i]);
    }
    oss << ")";
    return oss.str();
}

} // namespace rantanplan
