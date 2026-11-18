#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include <variant>
#include <unordered_map>
#include <memory>
#include <functional>
#include <cassert>
#include <span>
#include "expr_enums.hpp"

// Optional Improvements
// ---------------------
// - Replace `ExprPool::canonical_key()` string hashing with structural hash
//   (hash `kind`, `op`, `type_id`, children IDs, payload directly into `size_t`).
//   Currently string-based — only affects construction time, not runtime.
// - Add `ExprPool::freeze()` to drop the `intern_` map after construction, freeing memory.

/**
 * @file expr_pool.hpp
 * @brief ExprID/ExprPool: the interned, structurally-shared representation
 *        of every precondition, effect, and goal in the problem.
 */
namespace rantanplan {

/// Lightweight handle to an interned expression. O(1) equality and hashing.
/// Only valid within the ExprPool that created it.
struct ExprID {
    int32_t id = -1;

    bool valid() const { return id >= 0; }
    bool operator==(ExprID other) const { return id == other.id; }
    bool operator!=(ExprID other) const { return id != other.id; }
    bool operator<(ExprID other) const { return id < other.id; }
};

inline constexpr ExprID EXPR_NULL{-1};

} // namespace rantanplan

// Hash specialisation — delegates to the underlying int32_t hash
namespace std {
template <>
struct hash<rantanplan::ExprID> : hash<int32_t> {
    size_t operator()(rantanplan::ExprID eid) const noexcept {
        return hash<int32_t>::operator()(eid.id);
    }
};
} // namespace std

namespace rantanplan {

/// Payload carried by leaf nodes in the expression pool.
using ExprPayload = std::variant<std::monostate, bool, int64_t, double, std::string>;

/// A single node in the expression pool (internal representation). Two
/// kinds (FUNCTION_APPLICATION, STATE_VARIABLE) use a head+arguments child
/// layout — see the ExprPool class comment below for the convention.
struct ExprNode {
    ExprKind kind = ExprKind::UNKNOWN;
    ExprOperator op = ExprOperator::NONE;  // an application's is its head's (intern())
    int type_id = -1; // index into Problem's types_ vector, -1 if null

    std::vector<ExprID> children;
    ExprPayload payload; // leaf value (string for symbols, int64 for ints, etc.)

    /// Build a canonical string key for interning (used as hash map key).
    std::string canonical_key() const;
};

/// Append-only, structurally-interning expression pool.
///
/// Every expression is stored exactly once. Structurally identical expressions
/// share the same ExprID, making equality comparison O(1).
///
/// The pool is designed to be shared (via shared_ptr) across immutable
/// Problem instances in a transformation pipeline.
class ExprPool {
public:
    // ------------------------------------------------------------------------
    // Expression child-layout conventions
    //
    // Most expression kinds expose their full child list directly via
    // children()/child()/child_count().
    //
    // Two kinds use a "head+arguments" layout:
    //   - ExprKind::FUNCTION_APPLICATION
    //   - ExprKind::STATE_VARIABLE
    //
    // For these, children[0] is the head symbol (operator/fluent symbol), and
    // children[1..] are semantic arguments.
    //
    // Use the application helpers when you mean semantic arguments/head:
    //   - has_head_and_arguments(id)
    //   - argument_count(id), argument(id, i), arguments(id)
    //   - head_symbol_id(id), head_symbol_node(id)
    //
    // Use raw child access when you need the exact structural tree layout
    // including the head symbol (e.g., generic traversals, serialization,
    // structural formatting/debug output).
    // ------------------------------------------------------------------------
    ExprPool() = default;

    /// Intern a fully-constructed ExprNode. Returns existing ID if structurally
    /// identical node already exists, otherwise appends and returns new ID.
    ExprID intern(ExprNode node);

    // ========================================================================
    // Ground-atom identity
    // ========================================================================
    //
    // Interning distinguishes nodes by kind:op:TYPE_ID and children, but the
    // Z3 variable a state variable becomes is named from PAYLOAD STRINGS ONLY
    // — GroundedEncodingVisitor reads the head symbol's payload and each
    // argument's payload, and Z3VariableFactory::get_fluent_var_name uses only
    // those. So interning is FINER than the identity the solver sees: two ids
    // for `at(truck1, depot)` that differ only in an argument's type_id both
    // become the variable `at_truck1_depot`.
    //
    // Anything that groups by fluent ExprID assumes one ground atom has one id:
    // WriteOnlyFluentPass's reads and writes, SemanticInterferencePredicate's
    // effects_by_fluent, ChainedGroundedEncoder's variable_modifiers_,
    // EffectNormalizationPass's write counting, NumericRelaxedPlanningGraph's
    // reachability/bounds tables and the footprint filter in
    // action_footprint.hpp. Each would mis-group, not fail, if it broke. So
    // the assumption is enforced: preprocess() (api/solver_api.cpp) stops with
    // an error when aliased_state_variable_count() is nonzero.
    //
    // An alias needs two producers that build one atom with a different type
    // somewhere. Most types are fixed in one place so that this cannot happen:
    //   - intern() gives a state variable its head's type, and a bool, int or
    //     real constant its value's type. There is one int type,
    //     INT_TYPE_ID. Otherwise the goal visited(3) and the effect
    //     visited(?i) with ?i = 3 (an integer parameter instantiated by
    //     IntParameterInstantiationPass) could be two nodes;
    //   - object constants are typed by the object's own type, by the
    //     converter and the grounder alike, and minted cells (array and set
    //     lowering) are interned once and reused.
    // What is left is a producer that breaks one of these: typing an object by
    // a parameter's declared (super)type, or a head symbol differently (say,
    // int against real). Bounds cannot do it: a bounded integer is the base
    // int type, its bounds are data on the Fluent (see Type).
    //
    // The signature covers atoms whose arguments are object or integer
    // constants. No atom with integer arguments exists yet (integer fluent
    // parameters are unsupported, set elements must be user types, and the
    // encoder rejects non-string arguments), but the check sees them already.
    //
    // The interning key itself stays finer (set_lowering_pass.cpp:147 and
    // array_lowering.cpp rely on type_id keeping nodes distinct). Instead each
    // state variable is mapped onto a representative sharing its payload
    // signature, maintained incrementally in intern() because passes keep
    // interning throughout preprocessing (a cache built once would go stale).

    /// Representative id for a ground state variable: the first one interned
    /// with the same head symbol and argument payloads. Identity on any other
    /// kind, and on an invalid id.
    ExprID canonical_state_variable(ExprID id) const {
        if (!id.valid() || static_cast<size_t>(id.id) >= canonical_sv_.size()) return id;
        ExprID rep = canonical_sv_[id.id];
        return rep.valid() ? rep : id;
    }

    /// State variables ever interned that are not their own representative.
    /// Only grows. preprocess() refuses a problem when it is nonzero.
    size_t aliased_state_variable_count() const { return aliased_sv_count_; }

    /// Access a node by ID.
    const ExprNode& get(ExprID id) const {
        assert(id.valid() && static_cast<size_t>(id.id) < nodes_.size());
        return nodes_[id.id];
    }

    /// Number of interned nodes.
    size_t size() const { return nodes_.size(); }

    /// Build a human-readable string for an expression (for debugging/display).
    std::string to_string(ExprID id) const;

    // ========================================================================
    // Kind / Operator / Type accessors
    // ========================================================================

    ExprKind kind(ExprID id) const { return get(id).kind; }
    ExprOperator op(ExprID id) const { return get(id).op; }
    int type_id(ExprID id) const { return get(id).type_id; }

    // ========================================================================
    // Structural checks (based on kind)
    // ========================================================================

    bool is_leaf(ExprID id) const { return get(id).children.empty(); }
    bool is_constant(ExprID id) const { return kind(id) == ExprKind::CONSTANT; }
    bool is_parameter(ExprID id) const { return kind(id) == ExprKind::PARAMETER; }
    bool is_variable(ExprID id) const { return kind(id) == ExprKind::VARIABLE; }
    bool is_fluent_symbol(ExprID id) const { return kind(id) == ExprKind::FLUENT_SYMBOL; }
    bool is_function_symbol(ExprID id) const { return kind(id) == ExprKind::FUNCTION_SYMBOL; }
    bool is_state_variable(ExprID id) const { return kind(id) == ExprKind::STATE_VARIABLE; }
    bool is_function_application(ExprID id) const { return kind(id) == ExprKind::FUNCTION_APPLICATION; }

    // ========================================================================
    // Operator checks (based on op field)
    // ========================================================================

    bool is_and(ExprID id) const { return op(id) == ExprOperator::AND; }
    bool is_or(ExprID id) const { return op(id) == ExprOperator::OR; }
    bool is_not(ExprID id) const { return op(id) == ExprOperator::NOT; }
    bool is_implies(ExprID id) const { return op(id) == ExprOperator::IMPLIES; }
    bool is_iff(ExprID id) const { return op(id) == ExprOperator::IFF; }
    bool is_count(ExprID id) const { return op(id) == ExprOperator::COUNT; }
    bool is_forall(ExprID id) const { return op(id) == ExprOperator::FORALL; }
    bool is_exists(ExprID id) const { return op(id) == ExprOperator::EXISTS; }
    /// A quantifier application: FUNCTION_APPLICATION[head, decl..., body].
    /// The declarations are the VARIABLE children (children[1..n-2]); the body
    /// is the single last child. See QuantifierExpansionPass.
    bool is_quantifier(ExprID id) const { return is_forall(id) || is_exists(id); }
    bool is_equals(ExprID id) const { return op(id) == ExprOperator::EQUALS; }
    bool is_plus(ExprID id) const { return op(id) == ExprOperator::PLUS; }
    bool is_minus(ExprID id) const { return op(id) == ExprOperator::MINUS; }
    bool is_multiply(ExprID id) const { return op(id) == ExprOperator::MULTIPLY; }
    bool is_divide(ExprID id) const { return op(id) == ExprOperator::DIVIDE; }
    bool is_less_than(ExprID id) const { return op(id) == ExprOperator::LESS_THAN; }
    bool is_less_equal(ExprID id) const { return op(id) == ExprOperator::LESS_EQUAL; }
    bool is_greater_than(ExprID id) const { return op(id) == ExprOperator::GREATER_THAN; }
    bool is_greater_equal(ExprID id) const { return op(id) == ExprOperator::GREATER_EQUAL; }

    // ========================================================================
    // Children access
    // ========================================================================

    const std::vector<ExprID>& children(ExprID id) const { return get(id).children; }
    size_t child_count(ExprID id) const { return get(id).children.size(); }
    ExprID child(ExprID id, size_t index) const {
        const auto& c = get(id).children;
        assert(index < c.size());
        return c[index];
    }

    // ========================================================================
    // Head+arguments accessors
    //
    // In this codebase, both FUNCTION_APPLICATION and STATE_VARIABLE encode a
    // head symbol at children[0] and arguments at children[1..].
    // ========================================================================

    bool has_head_and_arguments(ExprID id) const {
        ExprKind k = kind(id);
        return k == ExprKind::FUNCTION_APPLICATION || k == ExprKind::STATE_VARIABLE;
    }

    size_t argument_count(ExprID id) const {
        assert(has_head_and_arguments(id));
        size_t n = child_count(id);
        assert(n > 0);
        return n - 1;
    }

    ExprID argument(ExprID id, size_t index) const {
        assert(index < argument_count(id));
        return child(id, index + 1);
    }

    std::span<const ExprID> arguments(ExprID id) const {
        assert(has_head_and_arguments(id));
        const auto& kids = children(id);
        assert(!kids.empty());
        return std::span<const ExprID>(kids.data() + 1, kids.size() - 1);
    }

    ExprID head_symbol_id(ExprID id) const {
        assert(has_head_and_arguments(id));
        assert(child_count(id) > 0);
        return child(id, 0);
    }

    const ExprNode& head_symbol_node(ExprID id) const {
        return get(head_symbol_id(id));
    }

    // ========================================================================
    // Payload access
    // ========================================================================

    const ExprPayload& payload(ExprID id) const { return get(id).payload; }

    bool payload_is_bool(ExprID id) const {
        return std::holds_alternative<bool>(get(id).payload);
    }
    bool payload_bool(ExprID id) const {
        return std::get<bool>(get(id).payload);
    }
    bool payload_is_int(ExprID id) const {
        return std::holds_alternative<int64_t>(get(id).payload);
    }
    int64_t payload_int(ExprID id) const {
        return std::get<int64_t>(get(id).payload);
    }
    bool payload_is_double(ExprID id) const {
        return std::holds_alternative<double>(get(id).payload);
    }
    double payload_double(ExprID id) const {
        return std::get<double>(get(id).payload);
    }
    bool payload_is_string(ExprID id) const {
        return std::holds_alternative<std::string>(get(id).payload);
    }
    const std::string& payload_string(ExprID id) const {
        return std::get<std::string>(get(id).payload);
    }

    // ========================================================================
    // Semantic helpers
    // ========================================================================

    // A constant's type follows from its value, set by intern(), so these
    // helpers take none.

    /// Intern an integer constant and return its ExprID.
    ExprID intern_int_constant(int64_t value) {
        ExprNode node;
        node.kind = ExprKind::CONSTANT;
        node.payload = value;
        return intern(std::move(node));
    }

    /// Intern a real constant and return its ExprID.
    ExprID intern_double_constant(double value) {
        ExprNode node;
        node.kind = ExprKind::CONSTANT;
        node.payload = value;
        return intern(std::move(node));
    }

    /// Intern the result of folding arithmetic, which is computed in double:
    /// an integer constant if `integer` (every operand was an integer and the
    /// operation keeps integers) and the value fits one, else a real one. So
    /// folded 2 + 3 is the node of the literal 5.
    ExprID intern_folded_constant(double value, bool integer) {
        // Whole and below 2^63 in magnitude, so the cast is exact.
        if (integer && std::trunc(value) == value && std::fabs(value) < 0x1p63) {
            return intern_int_constant(static_cast<int64_t>(value));
        }
        return intern_double_constant(value);
    }

    /// Intern a boolean constant and return its ExprID.
    ExprID intern_bool_constant(bool value) {
        ExprNode node;
        node.kind = ExprKind::CONSTANT;
        node.payload = value;
        return intern(std::move(node));
    }

    /// Intern a STATE_VARIABLE node: a FLUENT_SYMBOL head (payload = fluent
    /// name) followed by its argument children. `type_id` is the fluent's value
    /// type, used for both the head symbol and the state-variable node. `args`
    /// defaults to empty for a nullary fluent (e.g. an array cell minted by
    /// array lowering). This is the single place the state-variable node layout
    /// (children[0] = FLUENT_SYMBOL, children[1..] = arguments) is constructed.
    ExprID intern_state_variable(const std::string& fluent_name, int type_id,
                                 const std::vector<ExprID>& args = {}) {
        ExprNode head;
        head.kind = ExprKind::FLUENT_SYMBOL;
        head.type_id = type_id;
        head.payload = fluent_name;
        ExprID head_id = intern(std::move(head));

        ExprNode sv;
        sv.kind = ExprKind::STATE_VARIABLE;
        sv.type_id = type_id;
        sv.children.reserve(1 + args.size());
        sv.children.push_back(head_id);
        for (ExprID a : args) sv.children.push_back(a);
        return intern(std::move(sv));
    }

    /// Check if this is a boolean constant with value true.
    bool is_true_constant(ExprID id) const {
        if (!id.valid()) return false;
        const auto& n = get(id);
        return n.children.empty() && n.kind == ExprKind::CONSTANT &&
               std::holds_alternative<bool>(n.payload) &&
               std::get<bool>(n.payload) == true;
    }

    /// Check if this is a boolean constant with value false.
    bool is_false_constant(ExprID id) const {
        if (!id.valid()) return false;
        const auto& n = get(id);
        return n.children.empty() && n.kind == ExprKind::CONSTANT &&
               std::holds_alternative<bool>(n.payload) &&
               std::get<bool>(n.payload) == false;
    }

private:
    /// Payload signature of a state variable — head symbol payload plus each
    /// argument payload, which is exactly what the Z3 variable name is built
    /// from. Empty unless every argument is an object or integer constant
    /// (nested fluent arguments have no name and are rejected by the encoding
    /// visitor anyway), which disables aliasing for it.
    std::string state_variable_signature(const ExprNode& node) const;

    std::vector<ExprNode> nodes_;
    std::unordered_map<std::string, ExprID> intern_;

    // See "Ground-atom identity" above.
    std::vector<ExprID> canonical_sv_;                     // indexed by ExprID
    std::unordered_map<std::string, ExprID> sv_signature_;
    size_t aliased_sv_count_ = 0;
};

} // namespace rantanplan
