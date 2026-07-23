#pragma once

#include "smt_types.hpp"
#include "symbol_table.hpp"
#include "term_builder.hpp"
#include "../problem/expr_pool.hpp"
#include "../problem/fluent.hpp"
#include "../problem/action.hpp"

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace rantanplan {

class Problem;

/**
 * @brief Creates and caches the SMT variables of the encoding.
 *
 * ATOM IDENTITY. Every encoding entity is an entity of the fork's
 * `THEORY_PLAN`, one kind per role:
 *
 *   - an action at a timestep  ->  `PLAN_DOES(action.id(), t)`         Bool
 *   - a fluent at a timestep   ->  `PLAN_FLUENT(ground_fluent_id, t)`  Bool|Int|Real
 *
 * Identity *is* the `(kind, index, timestep)` triple: PLAN atoms are
 * operator-indexed terms, structurally hash-consed like any compound term
 * (unlike `mkConst` constants, which are fresh on every call), so the same
 * triple yields the identical term from any call site and the reverse lookup
 * is a structural payload read with no string map anywhere.
 *
 * Role and sort are orthogonal, which is why there is one fluent kind and not
 * two: `PLAN_FLUENT` is Boolean for a predicate and Int/Real for a numeric or
 * object fluent, and the sort travels in the operator payload. A separate
 * "value" kind would have been distinguishable from a "holds" kind only by
 * that sort, and so would have carried no information.
 *
 * Both indices come from the *same* authoritative source. `action.id()` is the
 * dense ground-action id (`problem.cpp:171`); the fluent index is
 * `Problem::find_grounded_fluent_index(ExprID)`, the dense ground-fluent id
 * from the registry documented at `problem.hpp:96-100`. Note `Fluent::id()` is
 * **not** that -- it indexes lifted *declarations*, not ground instances.
 *
 * Numeric fluents follow a single-sort policy -- Int when the whole problem
 * is proven integer-safe, else Real -- carried in the payload's sort tag.
 *
 * CONTRACT (static atom identity): a ground fluent *is* its interned ExprID,
 * and its state variable is keyed by dense ground id x timestep, so which cell
 * an access touches is decided at compile time, never by the model. Everything
 * structured (arrays, sets) is lowered to ground scalar cells before grounding
 * to preserve this.
 */
class SmtVariableFactory {
public:
    /// The problem decides Int-vs-Real for numeric sorts and supplies the
    /// dense ground ids; bound at construction.
    SmtVariableFactory(cvc5::TermManager& tm, const Problem* problem);

    cvc5::TermManager& tm() const { return tm_; }
    const TermBuilder& builder() const { return builder_; }

    // ── symbol variables (interned) ─────────────────────────────────────────
    /// Get-or-create the variable for a non-fluent symbol leaf, keyed by name.
    /// Interning is mandatory, not an optimization: mkConst mints a fresh
    /// constant on every call, so re-deriving a symbol by name would silently
    /// split one symbol into many unrelated variables.
    Term create_symbol_variable(const std::string& name, const Type* type);

    // ── cached, timestep-aware accessors ────────────────────────────────────
    /// @param fluent_eid the ground fluent's interned ExprID, which is what
    ///        the ground-fluent registry is keyed by; it resolves to the dense
    ///        ground-fluent id that keys the cache and the atom payload.
    const Term& get_fluent_variable(const Fluent& fluent, int timestep, ExprID fluent_eid);
    const Term& get_action_variable(const Action& action, int timestep);
    /// Lookup-only (plan extraction runs on a const encoder); throws if the
    /// variable has not been created.
    const Term& get_action_variable(const Action& action, int timestep) const;

    // ── naming (still used for numeric fluents and for diagnostics) ─────────
    std::string get_fluent_var_name(const Fluent& fluent) const;
    std::string get_fluent_var_name(const Fluent& fluent, int timestep) const;
    std::string get_action_var_name(const Action& action) const;
    std::string get_action_var_name(const Action& action, int timestep) const;

    // ── reverse lookup ──────────────────────────────────────────────────────
    // One structural payload read, then a direct index into the problem's own
    // registries -- the payload index IS the dense id those registries are
    // positionally keyed by, and the atom's kind says which one to consult.
    // No string rendering, no map, no side table.
    /// PLAN_FLUENT atom -> (ground fluent ExprID, timestep). The ExprID is the
    /// ground fluent's canonical identity (`grounded_fluents()[idx]`).
    std::optional<std::pair<ExprID, int>> get_fluent_from_variable(const Term& var) const;
    /// PLAN_DOES atom -> (action, timestep), via `actions()[id]`.
    std::optional<std::pair<const Action&, int>> get_action_from_variable(const Term& var) const;
    bool is_fluent_variable(const Term& var) const;
    bool is_action_variable(const Term& var) const;

    // ── numeric constants ───────────────────────────────────────────────────
    Term make_numeric_val(int64_t v) const { return builder_.numeric_val(v); }
    Term make_numeric_val(double v) const { return builder_.numeric_val(v); }
    Term make_zero() const { return builder_.zero(); }

private:
    cvc5::TermManager& tm_;
    TermBuilder builder_;
    const Problem* problem_;

    // [timestep][dense ground id] -> atom; a null Term is "not created yet".
    // Inner vectors are preallocated to the full dense range (both id spaces
    // are fixed once the problem is grounded), so returned references stay
    // stable for the factory's lifetime.
    std::vector<std::vector<Term>> state_vars_;
    std::vector<std::vector<Term>> action_vars_;

    // Interned symbol-leaf variables, keyed by name.
    SymbolTable symbols_;

    void ensure_timestep_capacity(int timestep);
    Term create_new_fluent_variable(const Fluent& fluent, int timestep, uint32_t idx);
    uint32_t ground_fluent_index(const Fluent& fluent, ExprID fluent_eid) const;
};

}  // namespace rantanplan
