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
    ChainedGroundedEncoder(const Problem& problem, cvc5::TermManager& ctx,
                           std::unique_ptr<ParallelismStrategy> parallelism);

    // Override action encoding to implement chained effects
    Term encode_actions(int t) override;

    std::optional<Term> encode_non_cumulative_effects(const Action& action, int t) override;
    Term encode_cumulative_effects(int t) override;

    /// The chain defines f^{t+1} totally for exactly the fluents it covers, so
    /// this is is_chained().  See GroundedEncoder for what the caller does with
    /// it, and encode_chained_effects for why the definition is total.
    bool effects_determine_successor(ExprID fluent) const override {
        return is_chained(fluent);
    }

private:
    /// Whether this fluent's successor is built by the chain rather than by
    /// per-action effect implications.  THE definition, consulted by every site
    /// that has to agree on the split — the effect filters, the chain builder,
    /// and the frame-axiom hook.  They must not drift: a fluent the filters
    /// exclude but the chain skips would lose its effects entirely.
    ///
    /// Every modified non-Boolean fluent qualifies, including single-writer
    /// ones.  A single write needs no chain to be CORRECT (one implication plus
    /// a frame axiom says the same thing), but chaining it anyway is what lets
    /// numeric fluents drop out of encode_frames uniformly, and it trades the
    /// frame axiom's `f^t != f^{t+1}` — an arithmetic atom the theory solver
    /// has to decide — for an ite term that states the same thing directly.
    ///
    /// Booleans stay out: their frame axioms are cheap (no theory atom) and the
    /// chain's ite would be boolean-sorted, which the solver internalizes as an
    /// iff node — a fresh literal plus four gate clauses per write.
    ///
    /// UNMEASURED ON CVC5. Chaining single-writer fluents to drop their frame
    /// axioms arrived from the z3 branch flagged "need to test if the implied
    /// constraint is worth it", and the boolean rationale above was reasoned
    /// about z3's internalizer. Soundness does not depend on the choice — only
    /// on all three sites agreeing — so narrowing this predicate is a safe knob
    /// if an A/B says the frame axioms were cheaper.
    bool is_chained(ExprID fluent) const {
        auto it = variable_modifiers_.find(fluent);
        return it != variable_modifiers_.end() && !it->second.empty() &&
               !problem_.is_bool_type(fluent);
    }

    // Map from variable (ExprID) to the writes targeting it, as (action, effect)
    // pairs — one entry per EFFECT, not per action. An action that writes one
    // fluent twice (a reflexive grounding of move(?from,?to); the two arms of a
    // clip-to-capacity idiom) therefore contributes two chain links, and the
    // chain composes them in effects() order. Deduplicating to one entry per
    // action, as this index used to, silently dropped every write but the first.
    std::unordered_map<ExprID, std::vector<std::pair<const Action*, const Effect*>>>
        variable_modifiers_;

    /// Chain variables ζ^t_{n,i}, keyed by their rendered name.
    ///
    /// PORT NOTE (cvc5): `TermManager::mkConst` is fresh on every call, unlike
    /// Z3's name-interned `ctx.bool_const`/`int_const`. Under Z3 two calls with
    /// the same ζ name gave the *same* variable, so encode_chained_effects(t)
    /// could be called twice for one timestep and still build one chain. This
    /// cache preserves that: without it a second call for the same t would mint
    /// a second, independent chain of variables. The double call is real, not
    /// hypothetical — encode_actions builds the chain, and CGBPlanner calls
    /// encode_cumulative_effects for the same t as well.
    std::unordered_map<std::string, Term> chain_vars_;

    /// Get (creating on first use) the chain variable named @p name.
    Term get_or_create_chain_variable(const std::string& name, const Type* var_type);

    /**
     * @brief Build index of which effects write which variables
     */
    void build_variable_modifiers_index();


    /**
     * @brief Encode chained effects for multi-modified non-Boolean variables
     * Implements equations (7)-(9) from Section 5.2
     */
    Term encode_chained_effects(int t);

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
