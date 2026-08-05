#pragma once

#include "grounded_encoder.hpp"

#include <utility>

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
    ChainedGroundedEncoder(const Problem& problem, z3::context& ctx,
                           std::unique_ptr<ParallelismStrategy> parallelism);
    
    // Override action encoding to implement chained effects
    z3::expr encode_actions(int t) override;

    std::optional<z3::expr> encode_non_cumulative_effects(const Action& action, int t) override;
    z3::expr encode_cumulative_effects(int t) override;

private:
    // Map from variable (ExprID) to the writes targeting it, as (action, effect)
    // pairs — one entry per EFFECT, not per action. An action that writes one
    // fluent twice (a reflexive grounding of move(?from,?to); the two arms of a
    // clip-to-capacity idiom) therefore contributes two chain links, and the
    // chain composes them in effects() order. Deduplicating to one entry per
    // action, as this index used to, silently dropped every write but the first.
    std::unordered_map<ExprID, std::vector<std::pair<const Action*, const Effect*>>>
        variable_modifiers_;

    /**
     * @brief Build index of which effects write which variables
     */
    void build_variable_modifiers_index();


    /**
     * @brief Encode chained effects for multi-modified non-Boolean variables
     * Implements equations (7)-(9) from Section 5.2
     */
    z3::expr encode_chained_effects(int t);
    
    /**
     * @brief Apply an effect to an expression (for chaining)
     * @param effect The effect expression to apply
     * @param base_expr The base expression to apply the effect to  
     * @param timestep The current timestep for variable lookups
     * @return Z3 expression representing effect applied to base_expr
     */
    z3::expr apply_effect_to_expression(const EffectExpression& effect, 
                                      const z3::expr& base_expr, 
                                      int timestep) const;
};

} // namespace rantanplan