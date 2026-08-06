#pragma once

#include "grounded_encoder.hpp"

namespace rantanplan {

/**
 * @brief Chained SMT encoder implementing Section 5.2 from the semantic interference paper
 *
 * This encoder extends GroundedEncoder to support cumulative effects in parallel plans
 * by implementing the "chained SMT encoding" described in Section 5.2 of the paper
 * "Relaxing non-interference requirements in parallel plans".
 *
 * For variables modified by multiple actions (|A_n| > 1), it creates intermediate
 * chain variables ζ^t_{n,i} and applies effects sequentially through the chain.
 * This allows proper handling of cumulative numeric effects from parallel actions.
 */
class ChainedGroundedEncoder : public GroundedEncoder {
public:
    ChainedGroundedEncoder(const Problem& problem, cvc5::TermManager& ctx,
                           std::unique_ptr<ParallelismStrategy> parallelism);

    // Override action encoding to implement chained effects
    Term encode_actions(int t) override;

    std::optional<Term> encode_non_cumulative_effects(const Action& action, int t) override;
    Term encode_cumulative_effects(int t) override;

private:
    // Map from non-Boolean variable (ExprID) to actions that modify it
    std::unordered_map<ExprID, std::vector<const Action*>> variable_modifiers_;

    /// Chain variables ζ^t_{n,i}, keyed by their rendered name.
    ///
    /// PORT NOTE (cvc5): `TermManager::mkConst` is fresh on every call, unlike
    /// Z3's name-interned `ctx.bool_const`/`int_const`. Under Z3 two calls with
    /// the same ζ name gave the *same* variable, so encode_chained_effects(t)
    /// could be called twice for one timestep and still build one chain. This
    /// cache preserves that: without it a second call for the same t would mint
    /// a second, independent chain of variables.
    std::unordered_map<std::string, Term> chain_vars_;

    /// Get (creating on first use) the chain variable named @p name.
    Term get_or_create_chain_variable(const std::string& name, const Type* var_type);

    /**
     * @brief Build index of which actions modify which variables
     */
    void build_variable_modifiers_index();


    /**
     * @brief Encode chained effects for multi-modified non-Boolean variables
     * Implements equations (7)-(9) from Section 5.2
     */
    Term encode_chained_effects(int t);

    /**
     * @brief Get the effect expression for a specific variable from an action
     */
    std::optional<const EffectExpression*> get_effect_for_variable(
        const Action& action, ExprID var_eid) const;

    /**
     * @brief Apply an effect to an expression (for chaining)
     * @param effect The effect expression to apply
     * @param base_expr The base expression to apply the effect to
     * @param timestep The current timestep for variable lookups
     * @return Term representing effect applied to base_expr
     */
    Term apply_effect_to_expression(const EffectExpression& effect,
                                      const Term& base_expr,
                                      int timestep) const;
};

} // namespace rantanplan
