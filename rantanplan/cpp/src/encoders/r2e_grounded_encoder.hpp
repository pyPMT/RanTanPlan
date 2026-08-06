#pragma once

#include "grounded_encoder.hpp"
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rantanplan {

/**
 * @brief R2E (Relaxed ∃-step) SMT encoder
 *
 * Implements the R2∃-step semantics using chain variables and global action ordering.
 * Generates constraints for preconditions, effects, carry-forward, and linking.
 */
class R2EGroundedEncoder : public GroundedEncoder {
public:
    enum class ActionOrdering {
        DEC,  // Declaration order (as actions appear in input)
        RPG   // NumericRPG layer-based ordering
    };

    R2EGroundedEncoder(const Problem& problem, cvc5::TermManager& ctx,
                       std::unique_ptr<ParallelismStrategy> parallelism,
                       ActionOrdering ordering = ActionOrdering::RPG);

    // Override encoding methods to implement R2E semantics
    Term encode_actions(int t) override;
    Term encode_frames(int t) override;
    Term encode_parallelism(int t) override;

    // TODO: R2E uses chain-substituted intermediate variables (ζ) where each
    // action sees a different "state" based on its position in the fixed ordering.
    // State constraints (mutexes, bounds) apply to the TRUE state at timestep
    // boundaries (x_t and x_{t+1}), not to intermediate chain variables.
    // To support state constraints here, we would need to assert them only on
    // the first chain variable (= x_t) and the last (= x_{t+1}), not on
    // intermediate ζ values. For now, disabled.
    Term encode_state_constraints(int t) override {
        return tb().bool_val(true);
    }

    /// Evaluate cost at the chain-substituted state σ^t_{prev(i)} that action i
    /// actually sees under R2E's fixed ordering, making SDAC costs sound.
    Term convert_cost_to_z3(const Action& action, int timestep) override;

private:
    ActionOrdering action_ordering_;

    // Global action ordering L = [a1, a2, ..., a|A|]
    std::vector<const Action*> global_action_order_;

    // For each variable x (ExprID), Ax = set of actions that modify x
    std::unordered_map<ExprID, std::vector<const Action*>> variable_modifiers_;

    // Mapping ρx: {0, ..., |Ax|} → {0, ..., |A|} for each variable x
    std::unordered_map<ExprID, std::vector<int>> rho_x_;

    // Mapping prevx: {1, ..., |A|} → {0, ..., |A|} for each variable x
    std::unordered_map<ExprID, std::vector<int>> prev_x_;

    /// Chain variables x^t_i, keyed by their rendered name.
    ///
    /// PORT NOTE (cvc5): this cache is what holds the whole R2E encoding
    /// together. `TermManager::mkConst` returns a **fresh** variable on every
    /// call ("even if the same arguments were provided on a previous call",
    /// cvc5.h:4388), whereas Z3's `ctx.int_const`/`bool_const` were interned by
    /// name. get_chain_variable() is called many times for the same
    /// (variable, timestep, action index) — from create_prev_substitution,
    /// create_modi_substitution, encode_linking_constraints and
    /// get_prev_variable_or_chain — and every one of those occurrences MUST be
    /// the same variable, otherwise the chain equations link nothing and the
    /// encoding is silently wrong.
    std::unordered_map<std::string, Term> chain_variables_;

    // All state variables for closed-world assumption
    std::unordered_set<ExprID> all_state_variables_;

    // Setup methods
    void build_action_ordering();
    void build_variable_modifiers();
    void build_rho_mappings();
    void build_prev_mappings();
    void collect_all_state_variables();

    // RPG-based ordering
    std::vector<const Action*> extract_rpg_ordering();

    // Constraint generation methods
    Term encode_precondition_constraints(int t);
    Term encode_effect_constraints(int t);
    Term encode_linking_constraints(int t);

    // Helper methods
    std::unordered_map<ExprID, Term> create_prev_substitution(int action_index, int timestep);
    std::unordered_map<ExprID, Term> create_modi_substitution(int action_index, int timestep);
    Term apply_substitution(const Term& expr, const std::unordered_map<ExprID, Term>& substitution, int timestep);
    Term create_effect_value_z3(const EffectExpression& eff_expr, const Term& fluent_z3,
                                   const std::unordered_map<ExprID, Term>& prev_substitution, int timestep);

    // Chain variable management
    std::string get_chain_variable_name(ExprID var_eid, int timestep, int action_index) const;
    Term get_chain_variable(ExprID var_eid, int timestep, int action_index);
    Term get_prev_variable_or_chain(ExprID var_eid, int timestep, int action_index);

    // Action index management
    int get_global_action_index(const Action* action) const;

    // Plan extraction for R2E semantics
    Plan extract_plan(const cvc5::Solver& solver, int max_timestep) const override;

    // Debug method
    void debug_print_structures() const;
};

} // namespace rantanplan
