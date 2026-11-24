#pragma once

#include "grounded_encoder.hpp"

#include <map>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

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

    /// The chain defines f^{t+1} totally for exactly the fluents it covers, so
    /// this is is_chained().  See GroundedEncoder for what the caller does with
    /// it, and encode_chained_effects for why the definition is total.
    bool effects_determine_successor(ExprID fluent) const override {
        return is_chained(fluent);
    }

    /// --robust-actions. A linear condition c.x >= c0 (or > c0) of an action's
    /// precondition whose written variables are written only by unconditional
    /// increases and decreases whose terms do not read the variable they change
    /// is a threshold condition. An action v with one that another action can
    /// lower is a candidate: it gets per step t a Boolean r_v^t and, up front,
    ///     r_v^t AND v^t -> c.x^t + sum_{u != v} ite(u^t, min(0, c.d_u), 0) >= c0
    /// per threshold condition, which then holds after the decreases of every
    /// other writer active in the step, in any order. With r_v^t true the
    /// ExistsCycle module leaves out the removable edges into v.
    bool robust_on() const { return robust_actions_; }
    /// r_v^t, or null when v is no candidate or step t is not encoded yet.
    const z3::expr* robust_var(int action, int t) const;
    const std::vector<z3::expr>& robust_vars_at(int t) const;
    /// Z3 ast id of an r variable -> (action, step).
    const std::unordered_map<unsigned, std::pair<int, int>>& robust_ids() const { return robust_ids_; }
    /// The edge u -> v (u may disable v) is removable: v is a candidate, the
    /// edge comes from the first condition of the semantic relation, the
    /// effects of u and v commute, and u writes no variable of v's other
    /// conditions, so u can disable v only through its threshold conditions,
    /// whose rows count u. Cached.
    bool robust_removable(int u, int v) const;
    /// The order of the oracle without the removable edges into the step's
    /// robust actions.
    std::vector<const Action*> order_step(const z3::model& model, int t,
                                          const std::vector<const Action*>& actions) const override;

    /// --rolling. An action whose effects are unconditional increases and
    /// decreases by constants of numeric fluents with exact summed links, and
    /// constant Boolean assignments to fluents its precondition does not
    /// mention, runs n_a^t >= 0 times in step t (a^t <-> n_a^t >= 1): the
    /// summed links add n_a d_a, the rows count n_u for every other writer and
    /// n_v - 1 for v itself, and n_v >= 2 needs r_v when v's own run lowers one
    /// of its threshold conditions. Whether some action rolls.
    bool rolling_applies() {
        if (robust_actions_ && !robust_built_) build_robust(0);
        return rolling_ && !roll_set_.empty();
    }

private:
    struct RobustConj {
        size_t index = 0;                                 ///< among the precondition's top-level conjuncts
        bool strict = false;                              ///< e > 0 rather than e >= 0
        std::vector<std::pair<ExprID, z3::expr>> coef;    ///< written fluents with their coefficient in e
        std::vector<int> writers;                         ///< every action writing one of them
        std::string key;                                  ///< the coefficients, for the shared sum
        bool harmless = true;                             ///< no writer can lower e: no row needed
    };
    struct RobustInfo {
        std::vector<RobustConj> conj;
        std::unordered_set<ExprID> other_fluents;         ///< fluents of the other conjuncts and literals
        bool self_decrease = false;                       ///< rolled: its own run lowers a threshold condition
    };
    const bool rolling_;
    std::unordered_set<int> roll_set_;
    std::map<std::pair<int, int>, z3::expr> roll_n_;
    bool rollable(const Action& a, const RobustInfo* info, int t);
    const bool robust_actions_;
    bool robust_built_ = false;
    std::unordered_map<int, RobustInfo> robust_info_;
    std::map<std::pair<int, int>, z3::expr> robust_r_;
    std::unordered_map<int, std::vector<z3::expr>> robust_by_t_;
    std::unordered_map<unsigned, std::pair<int, int>> robust_ids_;
    std::map<std::pair<std::string, int>, z3::expr> robust_sum_;
    mutable std::map<std::pair<int, int>, bool> robust_removable_;
    void build_robust(int t);
    void encode_robust_rows(int t, z3::expr_vector& constraints);
    /// c.d_u for the condition, at step t (before the minimum with 0).
    z3::expr robust_shift(int u, const RobustConj& c, int t);
    z3::expr robust_weight(int u, const RobustConj& c, int t);
    /// ite(u^t, w, 0), or n_u^t * w for a rolled action u.
    z3::expr robust_term(int u, const z3::expr& w, const z3::expr& zero, int t);

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
    /// chain's ite would be boolean-sorted, which Z3 internalizes as an iff
    /// node — a fresh literal plus four gate clauses per write. The exception
    /// is a Boolean whose own write reads it (self_reading_bools_).
    bool is_chained(ExprID fluent) const {
        auto it = variable_modifiers_.find(fluent);
        return it != variable_modifiers_.end() && !it->second.empty() &&
               (!problem_.is_bool_type(fluent) || self_reading_bools_.count(fluent));
    }

    /// Writes whose guard or value reads the fluent they write, and the Boolean
    /// fluents with such a write, which are chained (build_variable_modifiers_index).
    std::unordered_set<const EffectExpression*> self_reading_effects_;
    std::unordered_set<ExprID> self_reading_bools_;

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

    /// --summed-links: x^{t+1} = ite(c_1, e_1, ... ite(c_m, e_m, x^t + sum of
    /// the active increases)), over the plain assignments c_i -> x := e_i of
    /// the writers of x, instead of x's chain. Two writers of x share a step
    /// only if their writes commute, so the active plain assignments have one
    /// value and leave no increase, and increases add up in any order. Exact
    /// when no action writes x twice and no write of x reads x; otherwise
    /// x keeps its chain and this returns false.
    bool encode_summed_link(ExprID var_eid, int t, z3::expr_vector& constraints);
    bool summed_link_exact(ExprID var_eid) const;
    const bool summed_links_;
    
    /**
     * @brief Apply an effect to an expression (for chaining)
     * @param effect The effect expression to apply
     * @param base_expr The base expression to apply the effect to
     * @param self_value The value at which the effect's terms read the fluent itself
     * @param timestep The current timestep for variable lookups
     * @return Z3 expression representing effect applied to base_expr
     */
    z3::expr apply_effect_to_expression(const EffectExpression& effect,
                                      const z3::expr& base_expr,
                                      const z3::expr& self_value,
                                      int timestep) const;

    /// expr, a term or guard of effect converted at timestep, with the fluent
    /// that effect writes read as fluent_value; expr itself unless the effect
    /// reads that fluent.
    z3::expr read_at(const EffectExpression& effect, const z3::expr& expr,
                     const z3::expr& fluent_value, int timestep) const;
};

} // namespace rantanplan