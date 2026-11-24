#include "chained_grounded_encoder.hpp"
#include "../analysis/semantic_interference_analysis.hpp"
#include "../config/config.hpp"
#include "../problem/visitors/fluent_collector.hpp"
#include "../util/stats.hpp"
#include <iostream>
#include <algorithm>
#include <cassert>
#include <set>

namespace rantanplan {

ChainedGroundedEncoder::ChainedGroundedEncoder(const Problem& problem, z3::context& ctx,
                                               std::unique_ptr<ParallelismStrategy> parallelism)
    : GroundedEncoder(problem, ctx, std::move(parallelism)),
      robust_actions_(Config::instance().exists.robust_actions),
      rolling_(Config::instance().exists.rolling),
      summed_links_(Config::instance().exists.summed_links) {
    build_variable_modifiers_index();
}

void ChainedGroundedEncoder::build_variable_modifiers_index() {
    variable_modifiers_.clear();

    // One entry per EFFECT. Actions are visited in problem order and their
    // effects in declaration order, so each action's writes to one fluent stay
    // adjacent and in PDDL order — which is the order the chain composes them
    // in, and the only defensible one when they do not commute (two assigns).
    //
    // This list is deliberately NOT deduplicated by action. An action with two
    // writes to one fluent needs two chain links: with one link it would
    // contribute only its first write (see the deleted get_effect_for_variable),
    // so `drink(lamb,lamb)` — (decrease (locale ?from) 1) and (increase (locale
    // ?to) 1) ground at ?from = ?to — applied -1 where PDDL says 0, and the
    // clip-to-capacity idiom `(when (<= .. cap) (increase ..)) (when (> .. cap)
    // (assign .. cap))` lost whichever arm came second.
    for (const Action& action : problem_.actions()) {
        for (const Effect& effect : action.effects()) {
            variable_modifiers_[effect.effect_expression().fluent_id()]
                .emplace_back(&action, &effect);
        }
    }

    // Writes whose guard or value reads the fluent they write. Only their
    // terms need the running value of the fluent (see encode_chained_effects);
    // every other term is converted exactly as before. A Boolean fluent with
    // such a write is chained too: only the chain lets that read see the value
    // left by the writers before it in the step. Every other Boolean keeps its
    // implications and frame axiom, which say the same as the chain for writes
    // whose guards and values do not read the fluent.
    self_reading_effects_.clear();
    self_reading_bools_.clear();
    for (const auto& [eid, writes] : variable_modifiers_) {
        for (const auto& [action, effect] : writes) {
            const EffectExpression& ee = effect->effect_expression();
            FluentCollector collector(problem_);
            collector.collect_from_id(ee.value_id());
            if (effect->is_conditional()) collector.collect_from_id(ee.condition_id());
            if (collector.get_fluents().count(eid)) {
                self_reading_effects_.insert(&ee);
                if (problem_.is_bool_type(eid)) self_reading_bools_.insert(eid);
            }
        }
    }

    auto& stats = Stats::instance();
    size_t chained_vars = 0, cumulative_vars = 0;
    for (const auto& [eid, writes] : variable_modifiers_) {
        if (!is_chained(eid)) continue;
        chained_vars++;
        // The subset that genuinely needs the chain for cumulative semantics.
        // The rest are single-writer fluents chained only so that they, too,
        // need no frame axiom; tracking both keeps that cost visible.
        if (writes.size() > 1) cumulative_vars++;
    }
    stats.set("encoder.chained_variables", chained_vars);
    stats.set("encoder.cumulative_variables", cumulative_vars);
}


z3::expr ChainedGroundedEncoder::encode_actions(int t) {
    z3::expr_vector all_constraints(ctx_);
    auto& stats = Stats::instance();

    // Process each action with modified effect constraints
    for (const Action& action : problem_.actions()) {
        z3::expr action_var = variable_factory_.get_action_variable(action, t);

        if (action.effects().empty()) continue;

        // Create precondition constraints (unchanged)
        if (action.has_precondition()) {
            z3::expr z3_precond = convert_expr_id_to_z3(action.precondition_id(), t);
            all_constraints.push_back(z3::implies(action_var, z3_precond));
        }

        // Create effect constraints, filtering out multi-written non-Boolean variables
        z3::expr_vector filtered_effects(ctx_);

        for (const Effect& effect : action.effects()) {
            if (!is_chained(effect.effect_expression().fluent_id())) {
                // Boolean fluents only: every modified numeric one is chained.
                z3::expr z3_effect = convert_effect_to_z3(effect.effect_expression(), t);
                filtered_effects.push_back(z3_effect);
            }
            // Chained variables are handled by encode_chained_effects
        }

        // Add filtered effect constraints for this action
        if (!filtered_effects.empty()) {
            all_constraints.push_back(z3::implies(action_var, z3::mk_and(filtered_effects)));
        }
    }

    // The run counts of step t, which the summed links read.
    if (robust_actions_ && !robust_built_) build_robust(t);
    for (int a : roll_set_) {
        const z3::expr n = variable_factory_.create_int_variable("roll_n_" + std::to_string(a) + "_t" + std::to_string(t));
        const z3::expr av = variable_factory_.get_action_variable(problem_.action(a), t);
        roll_n_.emplace(std::make_pair(a, t), n);
        all_constraints.push_back(z3::implies(av, n >= 1));
        all_constraints.push_back(z3::implies(!av, n == 0));
    }

    // Add chained effect constraints for multi-modified non-Boolean variables
    all_constraints.push_back(encode_chained_effects(t));

    if (robust_actions_) encode_robust_rows(t, all_constraints);

    stats.add("encoder.action_constraints", all_constraints.size());

    if (all_constraints.empty()) {
        return ctx_.bool_val(true);
    }

    return z3::mk_and(all_constraints);
}

z3::expr ChainedGroundedEncoder::encode_chained_effects(int t) {
    z3::expr_vector constraints(ctx_);
    auto& stats = Stats::instance();

    // Section 5.2 chained encoding, applied to EVERY modified numeric variable
    // rather than only the multi-written ones (see is_chained for why).
    // Key insight: conditions are evaluated using current chain position (input),
    // while effects produce the next chain position (output).
    //
    // Equations (7) and (9) are fused into the loop: the chain runs
    //     n^t = ζ_0  ->  ζ_1  ->  ...  ->  ζ_{k-1}  ->  n^{t+1}
    // with the LAST link writing n^{t+1} directly instead of minting a final ζ
    // and equating it.  So k writes cost k-1 chain variables, and the common
    // single-write case costs none at all — it collapses to the one constraint
    //     n^{t+1} = ite(a [AND c], eff(n^t), n^t)
    // which is exactly what the old effect implication plus its frame axiom
    // said between them.
    for (const auto& [var_eid, writes] : variable_modifiers_) {
        if (!is_chained(var_eid)) continue;
        if (summed_links_ && encode_summed_link(var_eid, t, constraints)) continue;

        // ζ is named after the atom and its id: a UP name may hold a space,
        // and n(a) and a 0-ary "n a" both render as (n a).
        const std::string var_str = problem_.pool().to_string(var_eid);
        const Type* var_type = problem_.type_for_id(var_eid);

        z3::expr chain_prev = convert_expr_id_to_z3(var_eid, t);   // ζ_0 = n^t
        z3::expr action_base = chain_prev;

        // Walk the writes in order. Two writes by the SAME action share one
        // action variable, so they either both apply or both pass through —
        // which is what composes them correctly.
        for (size_t i = 0; i < writes.size(); ++i) {
            const Action* action = writes[i].first;
            const Effect* full_effect = writes[i].second;
            z3::expr action_var = variable_factory_.get_action_variable(*action, t);

            // The value the variable has when this action's turn comes: the
            // terms of an action read it there (function of compositions,
            // Bofill et al. 2016, Def 5), and every other variable at t. All
            // the writes of one action read the same value, the action's own
            // pre-state. Without this a term that mentions its own variable
            // read n^t, and the chain depended on the order of the writers.
            if (i == 0 || writes[i - 1].first != action) action_base = chain_prev;

            const bool is_last = (i + 1 == writes.size());
            z3::expr chain_next =
                is_last ? convert_expr_id_to_z3(var_eid, t + 1)
                        : variable_factory_.create_symbol_variable(
                              "zeta_" + var_str + Z3VariableFactory::atom_tag(var_eid) +
                                  "_t" + std::to_string(t) + "_i" + std::to_string(i + 1),
                              var_type);

            // Equation (8): Apply effect conditionally based on action execution.
            // The effect VALUE is converted at timestep t (the pre-state), but for
            // the fluent itself, which it reads at action_base, while the
            // operator is applied to the running chain value, so additive
            // deltas accumulate while their expressions read the pre-state.
            z3::expr effect_applied = apply_effect_to_expression(
                full_effect->effect_expression(), chain_prev, action_base, t);

            // Unconditional:  action^t          -> next = eff(prev), else next = prev
            // Conditional:   (action^t AND c^t) -> next = eff(prev), else next = prev
            // The else-branch is what makes the successor TOTAL, and hence what
            // licenses effects_determine_successor to drop the frame axiom.
            z3::expr apply_effect = action_var;
            if (full_effect->is_conditional()) {
                apply_effect = action_var &&
                    read_at(full_effect->effect_expression(),
                            convert_expr_id_to_z3(full_effect->effect_expression().condition_id(), t),
                            action_base, t);
            }
            constraints.push_back(z3::ite(apply_effect,
                                          chain_next == effect_applied,
                                          chain_next == chain_prev));

            chain_prev = chain_next;
        }
    }

    stats.add("encoder.chained_constraints", constraints.size());

    if (constraints.empty()) {
        return ctx_.bool_val(true);
    }

    return z3::mk_and(constraints);
}

bool ChainedGroundedEncoder::summed_link_exact(ExprID var) const {
    std::unordered_set<const Action*> writers;
    for (const auto& [action, effect] : variable_modifiers_.at(var)) {
        if (!writers.insert(action).second || self_reading_effects_.count(&effect->effect_expression())) return false;
    }
    return true;
}

bool ChainedGroundedEncoder::encode_summed_link(ExprID var, int t, z3::expr_vector& constraints) {
    if (!summed_link_exact(var)) return false;
    const auto& writes = variable_modifiers_.at(var);
    const z3::expr x_t = convert_expr_id_to_z3(var, t);
    z3::expr_vector terms(ctx_);
    std::vector<std::pair<z3::expr, z3::expr>> assigns;  // (guard, value), the first one wins
    for (const auto& [action, effect] : writes) {
        const auto& ee = effect->effect_expression();
        z3::expr guard = variable_factory_.get_action_variable(*action, t);
        if (effect->is_conditional()) guard = guard && convert_expr_id_to_z3(ee.condition_id(), t);
        z3::expr value = convert_expr_id_to_z3(ee.value_id(), t);
        if (ee.kind() == EffectExpression::Kind::ASSIGN) {
            assigns.emplace_back(guard, value);
            continue;
        }
        if (ee.kind() == EffectExpression::Kind::DECREASE) value = -value;
        auto rn = roll_n_.find({action->id(), t});
        if (rn != roll_n_.end())
            terms.push_back((value.is_real() ? z3::to_real(rn->second) : rn->second) * value);
        else
            terms.push_back(z3::ite(guard, value, value.is_int() ? ctx_.int_val(0) : ctx_.real_val(0)));
    }
    z3::expr next = terms.empty() ? x_t : x_t + z3::sum(terms);
    for (auto it = assigns.rbegin(); it != assigns.rend(); ++it) next = z3::ite(it->first, it->second, next);
    constraints.push_back(convert_expr_id_to_z3(var, t + 1) == next);
    Stats::instance().add("encoder.summed_links");
    return true;
}

z3::expr ChainedGroundedEncoder::read_at(const EffectExpression& effect, const z3::expr& expr,
                                         const z3::expr& fluent_value, int timestep) const {
    if (!self_reading_effects_.count(&effect)) return expr;
    auto* non_const_this = const_cast<ChainedGroundedEncoder*>(this);
    z3::expr fluent_t = non_const_this->convert_expr_id_to_z3(effect.fluent_id(), timestep);
    if (z3::eq(fluent_value, fluent_t)) return expr;
    z3::expr_vector from(ctx_), to(ctx_);
    from.push_back(fluent_t);
    to.push_back(fluent_value);
    z3::expr result = expr;
    return result.substitute(from, to);
}

z3::expr ChainedGroundedEncoder::apply_effect_to_expression(
    const EffectExpression& effect, const z3::expr& base_expr, const z3::expr& self_value,
    int timestep) const {

    // Convert the effect value to Z3 - cast away const for this method; the
    // fluent itself is read as self_value (see encode_chained_effects).
    auto* non_const_this = const_cast<ChainedGroundedEncoder*>(this);
    z3::expr value_z3 = read_at(effect, non_const_this->convert_expr_id_to_z3(effect.value_id(), timestep),
                                self_value, timestep);

    // Apply the effect operation to the base expression
    // Note: We can't reuse convert_effect_to_z3() from base class because that creates
    // a constraint (fluent_next == result), but we need the result expression itself
    switch (effect.kind()) {
        case EffectExpression::Kind::ASSIGN:
            return value_z3;

        case EffectExpression::Kind::INCREASE:
            return base_expr + value_z3;

        case EffectExpression::Kind::DECREASE:
            return base_expr - value_z3;
    }

    // Fallback - should never reach here
    return base_expr;
}

// ---------------------------------------------------------------------------
// Robust actions (--robust-actions)
// ---------------------------------------------------------------------------

namespace {

void flatten_and(const z3::expr& e, std::vector<z3::expr>& out) {
    if (e.is_app() && e.decl().decl_kind() == Z3_OP_AND) {
        for (unsigned i = 0; i < e.num_args(); ++i) flatten_and(e.arg(i), out);
    } else {
        out.push_back(e);
    }
}

bool has_const(const z3::expr& e) {
    if (e.is_numeral()) return false;
    if (e.is_const()) return true;
    if (!e.is_app()) return true;
    for (unsigned i = 0; i < e.num_args(); ++i)
        if (has_const(e.arg(i))) return true;
    return false;
}

/// Linear arithmetic in uninterpreted constants: sums, numeral multiples, numeral divisors.
bool is_linear(const z3::expr& e) {
    if (e.is_numeral()) return true;
    if (!e.is_app() || !e.is_arith()) return false;
    if (e.is_const()) return e.decl().decl_kind() == Z3_OP_UNINTERPRETED;
    switch (e.decl().decl_kind()) {
        case Z3_OP_ADD: case Z3_OP_SUB: case Z3_OP_UMINUS: case Z3_OP_TO_REAL:
            for (unsigned i = 0; i < e.num_args(); ++i)
                if (!is_linear(e.arg(i))) return false;
            return true;
        case Z3_OP_MUL: {
            int with_const = 0;
            for (unsigned i = 0; i < e.num_args(); ++i) {
                if (!is_linear(e.arg(i))) return false;
                if (has_const(e.arg(i))) ++with_const;
            }
            return with_const <= 1;
        }
        case Z3_OP_DIV:
            return e.num_args() == 2 && is_linear(e.arg(0)) && e.arg(1).is_numeral();
        default:
            return false;
    }
}

void collect_consts(const z3::expr& e, std::vector<z3::expr>& out, std::set<unsigned>& seen) {
    if (e.is_numeral() || !e.is_app()) return;
    if (e.is_const()) {
        if (e.decl().decl_kind() == Z3_OP_UNINTERPRETED && seen.insert(e.id()).second) out.push_back(e);
        return;
    }
    for (unsigned i = 0; i < e.num_args(); ++i) collect_consts(e.arg(i), out, seen);
}

/// An arithmetic comparison as e >= 0 (strict: e > 0); false for anything else.
bool normalize_atom(const z3::expr& atom, z3::expr& e, bool& strict) {
    z3::expr a = atom;
    bool neg = false;
    if (a.is_app() && a.decl().decl_kind() == Z3_OP_NOT) {
        neg = true;
        a = a.arg(0);
    }
    if (!a.is_app() || a.num_args() != 2 || !a.arg(0).is_arith() || !a.arg(1).is_arith()) return false;
    const z3::expr l = a.arg(0), r = a.arg(1);
    switch (a.decl().decl_kind()) {
        case Z3_OP_GE: e = neg ? r - l : l - r; strict = neg; return true;   // not(l >= r): r - l > 0
        case Z3_OP_GT: e = neg ? r - l : l - r; strict = !neg; return true;  // not(l > r): r - l >= 0
        case Z3_OP_LE: e = neg ? l - r : r - l; strict = neg; return true;
        case Z3_OP_LT: e = neg ? l - r : r - l; strict = !neg; return true;
        default: return false;
    }
}

bool is_zero(const z3::expr& e) { return e.is_numeral() && (e == 0).simplify().is_true(); }

z3::expr same_sort(const z3::expr& w, const z3::expr& like) {
    if (like.is_real() && w.is_int()) return z3::to_real(w);
    return w;
}

}  // namespace

void ChainedGroundedEncoder::build_robust(int t) {
    robust_built_ = true;
    std::unordered_map<unsigned, ExprID> num_at, bool_at;
    for (const auto& [f, writes] : variable_modifiers_) {
        if (writes.empty()) continue;
        const z3::expr x = convert_expr_id_to_z3(f, t);
        (problem_.is_bool_type(f) ? bool_at : num_at)[x.id()] = f;
    }
    // The variables of a threshold condition are written only by unconditional
    // increases and decreases whose terms do not read the variable they change,
    // so their changes add up in any order.
    auto additive = [&](ExprID f) {
        for (const auto& [a, eff] : variable_modifiers_.at(f))
            if (eff->is_conditional() || eff->effect_expression().kind() == EffectExpression::Kind::ASSIGN ||
                self_reading_effects_.count(&eff->effect_expression()))
                return false;
        return true;
    };
    std::map<int, RobustInfo> all_info;
    for (const Action& v : problem_.actions()) {
        if (!v.has_precondition() || v.effects().empty()) continue;
        RobustInfo info;
        std::vector<z3::expr> conj;
        flatten_and(convert_expr_id_to_z3(v.precondition_id(), t), conj);
        for (size_t j = 0; j < conj.size(); ++j) {
            const z3::expr& a = conj[j];
            if (a.is_bool() && a.is_const() && bool_at.contains(a.id())) {
                info.other_fluents.insert(bool_at.at(a.id()));
                continue;
            }
            if (a.is_app() && a.decl().decl_kind() == Z3_OP_NOT && a.arg(0).is_const() &&
                bool_at.contains(a.arg(0).id())) {
                info.other_fluents.insert(bool_at.at(a.arg(0).id()));
                continue;
            }
            std::vector<z3::expr> vars;
            std::set<unsigned> seen;
            collect_consts(a, vars, seen);
            z3::expr e(ctx_);
            RobustConj rc;
            rc.index = j;
            bool ok = normalize_atom(a, e, rc.strict) && is_linear(e);
            if (ok) {
                for (const z3::expr& x : vars) {
                    auto it = num_at.find(x.id());
                    if (it == num_at.end()) continue;  // not written: constant within the step
                    z3::expr_vector from(ctx_), to(ctx_);
                    from.push_back(x);
                    to.push_back(x + 1);
                    z3::expr shifted = e;
                    z3::expr c = (shifted.substitute(from, to) - e).simplify();
                    if (is_zero(c)) continue;
                    if (!c.is_numeral() || !additive(it->second)) {
                        ok = false;
                        break;
                    }
                    rc.coef.emplace_back(it->second, c);
                }
            }
            if (!ok || rc.coef.empty()) {
                for (const z3::expr& x : vars) {
                    if (auto it = num_at.find(x.id()); it != num_at.end()) info.other_fluents.insert(it->second);
                    if (auto it = bool_at.find(x.id()); it != bool_at.end()) info.other_fluents.insert(it->second);
                }
                continue;
            }
            std::set<int> writers;
            for (const auto& [f, c] : rc.coef) {
                rc.key += std::to_string(f.id) + ":" + c.to_string() + ";";
                for (const auto& [a2, eff] : variable_modifiers_.at(f)) writers.insert(a2->id());
            }
            rc.writers.assign(writers.begin(), writers.end());
            for (int u : rc.writers) {
                if (u == v.id()) continue;
                const z3::expr sh = robust_shift(u, rc, t);
                if (!(sh.is_numeral() && (sh >= 0).simplify().is_true())) rc.harmless = false;
            }
            info.conj.push_back(std::move(rc));
        }
        all_info.emplace(v.id(), std::move(info));
    }
    if (rolling_) {
        for (const Action& a : problem_.actions()) {
            auto it = all_info.find(a.id());
            if (rollable(a, it == all_info.end() ? nullptr : &it->second, t)) roll_set_.insert(a.id());
        }
        Stats::instance().set("roll.rollable_actions", static_cast<double>(roll_set_.size()));
    }
    for (auto& [vid, info] : all_info) {
        if (roll_set_.contains(vid)) {
            for (RobustConj& rc : info.conj) {
                if (std::find(rc.writers.begin(), rc.writers.end(), vid) == rc.writers.end()) continue;
                const z3::expr sh = robust_shift(vid, rc, t);
                if (sh.is_numeral() && (sh < 0).simplify().is_true()) {
                    rc.harmless = false;
                    info.self_decrease = true;
                }
            }
        }
        bool candidate = false;
        for (const RobustConj& rc : info.conj) candidate = candidate || !rc.harmless;
        if (candidate) robust_info_.emplace(vid, std::move(info));
    }
    Stats::instance().set("robust.candidates", static_cast<double>(robust_info_.size()));
}

z3::expr ChainedGroundedEncoder::robust_shift(int u, const RobustConj& c, int t) {
    z3::expr sum = ctx_.int_val(0);
    bool any = false;
    for (const Effect& eff : problem_.action(u).effects()) {
        const auto& ee = eff.effect_expression();
        for (const auto& [f, coef] : c.coef) {
            if (ee.fluent_id() != f) continue;
            z3::expr d = convert_expr_id_to_z3(ee.value_id(), t);
            if (ee.kind() == EffectExpression::Kind::DECREASE) d = -d;
            sum = any ? sum + coef * d : coef * d;
            any = true;
        }
    }
    return sum.simplify();
}

z3::expr ChainedGroundedEncoder::robust_weight(int u, const RobustConj& c, int t) {
    const z3::expr sh = robust_shift(u, c, t);
    const z3::expr zero = sh.is_real() ? ctx_.real_val(0) : ctx_.int_val(0);
    if (sh.is_numeral()) return (sh < 0).simplify().is_true() ? sh : zero;
    return z3::ite(sh < 0, sh, zero);
}

bool ChainedGroundedEncoder::rollable(const Action& a, const RobustInfo* info, int t) {
    if (a.effects().empty()) return false;
    for (const Effect& eff : a.effects()) {
        if (eff.is_conditional()) return false;
        const auto& ee = eff.effect_expression();
        const ExprID f = ee.fluent_id();
        // Every condition of a that mentions a variable a writes is a threshold
        // condition: other_fluents holds the variables of a's literals and of
        // its other conjuncts.
        if (info && info->other_fluents.contains(f)) return false;
        const z3::expr val = convert_expr_id_to_z3(ee.value_id(), t).simplify();
        if (problem_.is_bool_type(f)) {
            if (!val.is_true() && !val.is_false()) return false;
            continue;
        }
        if (ee.kind() == EffectExpression::Kind::ASSIGN || !val.is_numeral()) return false;
        if (!summed_link_exact(f)) return false;  // the runs add up through the summed link of f
    }
    return true;
}

z3::expr ChainedGroundedEncoder::robust_term(int u, const z3::expr& w, const z3::expr& zero, int t) {
    auto rn = roll_n_.find({u, t});
    if (rn != roll_n_.end()) return (w.is_real() ? z3::to_real(rn->second) : rn->second) * w;
    return z3::ite(variable_factory_.get_action_variable(problem_.action(u), t), w, zero);
}

const z3::expr* ChainedGroundedEncoder::robust_var(int action, int t) const {
    auto it = robust_r_.find({action, t});
    return it == robust_r_.end() ? nullptr : &it->second;
}

const std::vector<z3::expr>& ChainedGroundedEncoder::robust_vars_at(int t) const {
    static const std::vector<z3::expr> none;
    auto it = robust_by_t_.find(t);
    return it == robust_by_t_.end() ? none : it->second;
}

void ChainedGroundedEncoder::encode_robust_rows(int t, z3::expr_vector& constraints) {
    if (!robust_built_) build_robust(t);
    for (const auto& [vid, info] : robust_info_) {
        const Action& v = problem_.action(vid);
        const z3::expr r = variable_factory_.create_bool_variable(
            "robust_" + std::to_string(vid) + "_t" + std::to_string(t));
        robust_r_.emplace(std::make_pair(vid, t), r);
        robust_by_t_[t].push_back(r);
        robust_ids_[r.id()] = {vid, t};
        std::vector<z3::expr> conj;
        flatten_and(convert_expr_id_to_z3(v.precondition_id(), t), conj);
        z3::expr_vector rows(ctx_);
        for (const RobustConj& rc : info.conj) {
            if (rc.harmless) continue;
            z3::expr e(ctx_);
            bool strict = false;
            normalize_atom(conj[rc.index], e, strict);
            const z3::expr zero = e.is_real() ? ctx_.real_val(0) : ctx_.int_val(0);
            // The sum over every writer, shared by the actions with the same condition.
            auto key = std::make_pair(rc.key, t);
            auto it = robust_sum_.find(key);
            if (it == robust_sum_.end()) {
                z3::expr_vector terms(ctx_);
                for (int u : rc.writers) {
                    const z3::expr w = same_sort(robust_weight(u, rc, t), e);
                    if (is_zero(w)) continue;
                    terms.push_back(robust_term(u, w, zero, t));
                }
                it = robust_sum_.emplace(key, terms.empty() ? zero : z3::sum(terms)).first;
            }
            z3::expr lhs = e + it->second;
            if (std::find(rc.writers.begin(), rc.writers.end(), vid) != rc.writers.end())
                lhs = lhs - same_sort(robust_weight(vid, rc, t), e);  // v's own term: v runs once
            rows.push_back(strict ? lhs > zero : lhs >= zero);
        }
        if (rows.empty()) continue;
        constraints.push_back(z3::implies(r && variable_factory_.get_action_variable(v, t), z3::mk_and(rows)));
        // A rolled v that lowers its own threshold condition counts itself n_v - 1 times in its row.
        if (info.self_decrease) constraints.push_back(z3::implies(roll_n_.at({vid, t}) >= 2, r));
    }
}

bool ChainedGroundedEncoder::robust_removable(int u, int v) const {
    if (auto it = robust_removable_.find({u, v}); it != robust_removable_.end()) return it->second;
    bool ok = false;
    auto info = robust_info_.find(v);
    const auto* oracle = dynamic_cast<const SemanticInterferenceAnalysis*>(
        get_parallelism_strategy()->get_interference_analyzer());
    if (info != robust_info_.end() && oracle && oracle->has_interference(u, v) &&
        oracle->get_interference_source(u, v) == InterferenceSource::CHECK1 && oracle->effects_commute(u, v)) {
        ok = true;
        for (const Effect& eff : problem_.action(u).effects())
            if (info->second.other_fluents.contains(eff.effect_expression().fluent_id())) {
                ok = false;
                break;
            }
    }
    return robust_removable_[{u, v}] = ok;
}

std::vector<const Action*> ChainedGroundedEncoder::order_step(const z3::model& model, int t,
                                                             const std::vector<const Action*>& actions) const {
    if (!robust_actions_) return GroundedEncoder::order_step(model, t, actions);
    const InterferenceOracle* oracle = get_parallelism_strategy()->get_interference_analyzer();
    std::unordered_set<int> robust;
    for (const Action* a : actions) {
        const z3::expr* r = robust_var(a->id(), t);
        if (r && model.eval(*r, true).is_true()) robust.insert(a->id());
    }
    // Kahn's algorithm in the given order; an edge u -> v (u may disable v) runs v first.
    const size_t n = actions.size();
    std::vector<std::vector<size_t>> after(n);
    std::vector<int> before(n, 0);
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < n; ++j) {
            const int u = actions[i]->id(), v = actions[j]->id();
            if (i == j || !oracle->has_interference(u, v)) continue;
            if (robust.contains(v) && robust_removable(u, v)) continue;
            after[j].push_back(i);
            ++before[i];
        }
    std::vector<const Action*> out;
    std::vector<char> done(n, 0);
    for (size_t round = 0; round < n; ++round) {
        size_t pick = n;
        for (size_t i = 0; i < n && pick == n; ++i)
            if (!done[i] && before[i] == 0) pick = i;
        if (pick == n) return GroundedEncoder::order_step(model, t, actions);  // a cycle: not an exists-step
        done[pick] = 1;
        out.push_back(actions[pick]);
        for (size_t k : after[pick]) --before[k];
    }
    Stats::instance().add("robust.robust_in_plan", static_cast<double>(robust.size()));
    if (roll_n_.empty()) return out;
    std::vector<const Action*> runs;
    for (const Action* a : out) {
        int64_t n = 1;
        if (auto rn = roll_n_.find({a->id(), t}); rn != roll_n_.end()) {
            const z3::expr value = model.eval(rn->second, true);
            if (value.is_numeral()) n = std::max<int64_t>(1, value.get_numeral_int64());
        }
        runs.insert(runs.end(), n, a);
    }
    return runs;
}

std::optional<z3::expr> ChainedGroundedEncoder::encode_non_cumulative_effects(
        const Action& action, int t) {
    if (action.effects().empty()) return std::nullopt;

    z3::expr action_var = variable_factory_.get_action_variable(action, t);

    // Filter out effects on multi-modified non-Boolean variables — those are
    // handled by encode_chained_timestep_effects (chain variables ζ).
    z3::expr_vector filtered_effects(ctx_);
    for (const Effect& effect : action.effects()) {
        if (!is_chained(effect.effect_expression().fluent_id())) {
            filtered_effects.push_back(convert_effect_to_z3(effect.effect_expression(), t));
        }
    }

    if (filtered_effects.empty()) return std::nullopt;
    return z3::implies(action_var, z3::mk_and(filtered_effects));
}

z3::expr ChainedGroundedEncoder::encode_cumulative_effects(int t) {
    return encode_chained_effects(t);
}

} // namespace rantanplan
