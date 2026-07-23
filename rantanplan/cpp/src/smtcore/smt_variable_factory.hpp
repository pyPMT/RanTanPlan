#pragma once

#include "smt_types.hpp"
#include "term_builder.hpp"
#include "../problem/expr_pool.hpp"
#include "../problem/fluent.hpp"
#include "../problem/action.hpp"

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace rantanplan {

class Problem;

/**
 * @brief Creates and caches the SMT variables of the encoding.
 *
 * Ported from Z3VariableFactory. One thing changed, and it is the reason M3
 * landed first: **atom identity**.
 *
 * Under Z3 every variable was a plain Boolean/Int/Real constant named by a
 * rendered string, and the reverse lookup a propagator needed was
 * `var.decl().name().str()` followed by a hash-map probe, on every callback
 * (`z3core/z3_variable_factory.cpp:159-175`). Here **every** encoding entity is
 * an entity of the fork's `THEORY_PLAN`, one kind per role:
 *
 *   - an action at a timestep  ->  `PLAN_DOES(action.id(), t)`         Bool
 *   - a fluent at a timestep   ->  `PLAN_FLUENT(ground_fluent_id, t)`  Bool|Int|Real
 *
 * so identity *is* the `(kind, index, timestep)` triple and the reverse lookup
 * is structural, with no string map anywhere.
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
 * Numeric fluents keep the Int-vs-Real single-sort policy the Z3 factory
 * enforced; it now rides in the payload's sort tag.
 *
 * CONTRACT (static atom identity) -- unchanged from the Z3 factory: a ground
 * fluent *is* its interned ExprID, and its state variable is keyed by rendered
 * name x timestep, so which cell an access touches is decided at compile time,
 * never by the model. Everything structured (arrays, sets) is lowered to ground
 * scalar cells before grounding to preserve this.
 */
class SmtVariableFactory {
public:
    explicit SmtVariableFactory(cvc5::TermManager& tm);

    cvc5::TermManager& tm() const { return tm_; }
    const TermBuilder& builder() const { return builder_; }

    /// Set the problem pointer; decides Int-vs-Real for numeric sorts.
    void set_problem(const Problem* p);

    // ── raw variable creation (uncached) ────────────────────────────────────
    Term create_bool_variable(const std::string& name) const;
    Term create_int_variable(const std::string& name) const;
    Term create_real_variable(const std::string& name) const;
    Term create_object_variable(const std::string& name) const;
    Term create_symbol_variable(const std::string& name, const Type* type) const;

    // ── cached, timestep-aware accessors ────────────────────────────────────
    /// @param fluent_eid the ground fluent's interned ExprID, which is what the
    ///        ground-fluent registry is keyed by. Callers that hold one (the
    ///        encoding visitors always do) must pass it; the overload without
    ///        it exists only for call sites that have lost the ExprID, and
    ///        resolves the index by rendered name.
    const Term& get_fluent_variable(const Fluent& fluent, int timestep, ExprID fluent_eid);
    const Term& get_fluent_variable(const Fluent& fluent, int timestep);
    const Term& get_action_variable(const Action& action, int timestep);
    const Term& get_fluent_variable(const Fluent& fluent, int timestep) const;
    const Term& get_action_variable(const Action& action, int timestep) const;

    // ── naming (still used for numeric fluents and for diagnostics) ─────────
    std::string get_fluent_var_name(const Fluent& fluent) const;
    std::string get_fluent_var_name(const Fluent& fluent, int timestep) const;
    std::string get_action_var_name(const Action& action) const;
    std::string get_action_var_name(const Action& action, int timestep) const;

    // ── reverse lookup ──────────────────────────────────────────────────────
    // One structural payload read; no string rendering, no map probe on the
    // hot path. This is what the theory-kind design buys.
    std::optional<std::pair<const Fluent&, int>> get_fluent_from_variable(const Term& var) const;
    std::optional<std::pair<const Action&, int>> get_action_from_variable(const Term& var) const;
    bool is_fluent_variable(const Term& var) const;
    bool is_action_variable(const Term& var) const;

    // ── enumeration (for propagator/theory registration) ────────────────────
    std::vector<std::shared_ptr<Term>> get_all_fluent_variables(int timestep) const;
    std::vector<std::shared_ptr<Term>> get_all_action_variables(int timestep) const;
    std::vector<std::pair<std::shared_ptr<Term>, int>> get_all_fluent_variables() const;
    std::vector<std::pair<std::shared_ptr<Term>, int>> get_all_action_variables() const;

    // ── numeric constants ───────────────────────────────────────────────────
    Term make_numeric_val(int64_t v) const { return builder_.numeric_val(v); }
    Term make_numeric_val(double v) const { return builder_.numeric_val(v); }
    Term make_zero() const { return builder_.zero(); }

    // ── timestep queries ────────────────────────────────────────────────────
    int get_max_timestep() const;
    bool has_variables_for_timestep(int timestep) const;

private:
    cvc5::TermManager& tm_;
    TermBuilder builder_;
    const Problem* problem_ = nullptr;

    // Indexed by timestep; keyed by rendered variable name.
    std::vector<std::unordered_map<std::string, std::shared_ptr<Term>>> state_vars_;
    std::vector<std::unordered_map<std::string, std::shared_ptr<Term>>> action_vars_;

    // Name-keyed maps, used only for diagnostics and the no-ExprID overload --
    // never on the reverse-lookup hot path.
    std::unordered_map<std::string, std::pair<std::shared_ptr<Fluent>, int>> state_var_name_to_fluent_;
    std::unordered_map<std::string, std::pair<std::shared_ptr<Action>, int>> action_var_name_to_action_;

    // Payload index -> entity. Both are the problem's own dense ground ids, so
    // these are recovery tables, not identity assignment.
    std::unordered_map<uint32_t, std::shared_ptr<Action>> id_to_action_;
    std::unordered_map<uint32_t, std::shared_ptr<Fluent>> id_to_fluent_;

    void ensure_timestep_capacity(int timestep);
    Term create_new_fluent_variable(const Fluent& fluent, int timestep, ExprID fluent_eid);
    uint32_t ground_fluent_index(const Fluent& fluent, ExprID fluent_eid) const;
};

}  // namespace rantanplan
