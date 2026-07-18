#include "set_lowering_pass.hpp"

#include "../problem/expr_builder.hpp"
#include "../problem/expr_enums.hpp"
#include "../problem/expr_walk.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace rantanplan {

namespace {

// ---------------------------------------------------------------------------
// The recursive worker. Constructed once over the (const) source problem, whose
// shared ExprPool it interns new nodes into. Mints one boolean membership fluent
// per set fluent and rewrites every set operation through ⟦·⟧_e.
// ---------------------------------------------------------------------------
class SetLowerer {
public:
    explicit SetLowerer(const Problem& problem)
        : problem_(problem), pool_(*problem.pool_ptr()), builder_(pool_) {
        bool_type_ = problem_.find_type("up:bool");
        if (!bool_type_) throw std::runtime_error("Set lowering: no bool type.");
        bool_type_id_ = problem_.type_id_of(bool_type_);
        true_ = builder_.true_id();
        false_ = builder_.false_id();
        mint_membership_fluents();
    }

    const std::vector<Fluent>& membership_fluents() const { return membership_fluents_; }

    // Rewrite a boolean/numeric expression, expanding the boolean/int-valued set
    // operators (member/subseteq/disjoint/cardinality/set-equality) and
    // recursing structurally through everything else.
    ExprID rewrite_expr(ExprID eid, const std::string& ctx) {
        if (!eid.valid()) return eid;

        if (pool_.is_function_application(eid)) {
            switch (pool_.op(eid)) {
                case ExprOperator::SET_MEMBER:  // (element, set)
                    return membership(pool_.argument(eid, 1), pool_.argument(eid, 0), ctx);
                case ExprOperator::SET_CARDINALITY:  // (set)
                    return lower_cardinality(pool_.argument(eid, 0), ctx);
                case ExprOperator::SET_SUBSETEQ:  // (s1, s2)
                    return lower_subseteq(pool_.argument(eid, 0), pool_.argument(eid, 1), ctx);
                case ExprOperator::SET_DISJOINT:  // (s1, s2)
                    return lower_disjoint(pool_.argument(eid, 0), pool_.argument(eid, 1), ctx);
                case ExprOperator::EQUALS: {
                    ExprID a = pool_.argument(eid, 0), b = pool_.argument(eid, 1);
                    if (is_set_expr(a) || is_set_expr(b))
                        return lower_set_equality(a, b, ctx);
                    break;  // ordinary (object/numeric) equality → generic recurse
                }
                default:
                    break;
            }
        }

        // Generic structural rewrite: recurse into children, re-intern if any
        // changed (mirrors substitute()).
        const auto& kids = pool_.children(eid);
        if (kids.empty()) return eid;
        std::vector<ExprID> new_kids;
        new_kids.reserve(kids.size());
        bool changed = false;
        for (ExprID c : kids) {
            ExprID nc = rewrite_expr(c, ctx);
            new_kids.push_back(nc);
            changed |= (nc != c);
        }
        if (!changed) return eid;
        return builder_.reintern(eid, std::move(new_kids));
    }

    // Rewrite one action: precondition/cost via rewrite_expr, effects via the
    // set-effect case split. Forall-effects must already be expanded.
    Action rewrite_action(const Action& a) {
        const std::string ctx = "action '" + a.name() + "'";
        Action out(a.name());
        out.set_pool(&pool_);
        out.set_parameters(a.parameters());
        if (a.has_precondition())
            out.set_precondition_id(rewrite_expr(a.precondition_id(), ctx));
        if (a.has_explicit_cost())
            out.set_cost_id(rewrite_expr(a.cost_id(), ctx));
        for (const Effect& eff : a.effects()) {
            const EffectExpression& ee = eff.effect_expression();
            if (!ee.forall_variable_ids().empty())
                throw std::runtime_error(
                    "Set lowering ran on an unexpanded forall-effect in " + ctx +
                    " — QuantifierExpansionPass must run before it.");
            lower_effect(ee, out, ctx);
        }
        return out;
    }

    // Normalise an initial-state assignment. A set-fluent target with a
    // SET_CONSTANT value becomes one true membership cell per element (CWA
    // falses the rest); everything else passes through (values rewritten).
    void rewrite_initial(const Assignment& asn, std::vector<Assignment>& out) {
        ExprID target = asn.fluent_id();
        const Fluent* f = set_fluent_of(target);
        if (!f) {
            out.emplace_back(rewrite_expr(target, "initial state"),
                             rewrite_expr(asn.value_id(), "initial state"), &pool_);
            return;
        }
        ExprID value = asn.value_id();
        if (!(pool_.is_function_application(value) &&
              pool_.op(value) == ExprOperator::SET_CONSTANT)) {
            throw std::runtime_error(
                "Initial value of set fluent '" + f->name() +
                "' is not a set constant (" + pool_.to_string(value) + ").");
        }
        std::vector<ExprID> base_args(pool_.arguments(target).begin(),
                                      pool_.arguments(target).end());
        for (ExprID elem : pool_.arguments(value)) {
            out.emplace_back(membership_cell(*f, elem, base_args), true_, &pool_);
        }
    }

private:
    const Problem& problem_;
    ExprPool& pool_;
    ExprBuilder builder_;
    const Type* bool_type_ = nullptr;
    int bool_type_id_ = -1;
    ExprID true_ = EXPR_NULL;
    ExprID false_ = EXPR_NULL;
    std::vector<Fluent> membership_fluents_;

    // ── fluent minting ───────────────────────────────────────────────────────

    void mint_membership_fluents() {
        for (const Fluent& f : problem_.fluents()) {
            if (!f.is_set_fluent()) continue;
            const Type* et = f.set_element_type();
            if (!et)
                throw std::runtime_error("Set fluent '" + f.name() +
                                         "' has no resolved element type.");
            // The membership fluent keeps the set fluent's name (decision #3):
            // its FLUENT_SYMBOL is bool-typed, so it never collides with the
            // dropped set head (canonical_key includes type_id).
            std::vector<Parameter> params;
            params.reserve(f.parameter_count() + 1);
            params.emplace_back("?_set_elem", et);
            for (const Parameter& p : f.parameters()) params.push_back(p);
            membership_fluents_.emplace_back(f.name(), bool_type_, params);
        }
    }

    // ── the compositional membership formula ⟦S⟧_e ───────────────────────────

    // `e` is any ExprID valid in the element position: an object constant (the
    // enumerated cases) or an action parameter / quantified variable (the
    // member-with-parameter case). Constant/constant equalities fold away.
    ExprID membership(ExprID S, ExprID e, const std::string& ctx) {
        if (const Fluent* f = set_fluent_of(S)) {
            std::vector<ExprID> args;
            args.reserve(pool_.argument_count(S) + 1);
            args.push_back(e);
            for (ExprID a : pool_.arguments(S)) args.push_back(a);
            return membership_cell(*f, e, {args.begin() + 1, args.end()});
        }
        if (pool_.is_function_application(S)) {
            switch (pool_.op(S)) {
                case ExprOperator::SET_CONSTANT: {
                    std::vector<ExprID> parts;
                    for (ExprID c : pool_.arguments(S)) parts.push_back(builder_.make_eq(e, c));
                    return builder_.make_or(std::move(parts));  // empty set → false
                }
                case ExprOperator::SET_ADD:  // (x, S')
                    return builder_.make_or({membership(pool_.argument(S, 1), e, ctx),
                                    builder_.make_eq(e, pool_.argument(S, 0))});
                case ExprOperator::SET_REMOVE:
                    return builder_.make_and({membership(pool_.argument(S, 1), e, ctx),
                                     builder_.make_not(builder_.make_eq(e, pool_.argument(S, 0)))});
                case ExprOperator::SET_UNION:
                    return builder_.make_or({membership(pool_.argument(S, 0), e, ctx),
                                    membership(pool_.argument(S, 1), e, ctx)});
                case ExprOperator::SET_INTERSECT:
                    return builder_.make_and({membership(pool_.argument(S, 0), e, ctx),
                                     membership(pool_.argument(S, 1), e, ctx)});
                case ExprOperator::SET_DIFFERENCE:
                    return builder_.make_and({membership(pool_.argument(S, 0), e, ctx),
                                     builder_.make_not(membership(pool_.argument(S, 1), e, ctx))});
                default:
                    break;
            }
        }
        throw std::runtime_error("Set lowering: expected a set expression in " +
                                 ctx + " (" + pool_.to_string(S) + ").");
    }

    // ── boolean/int-valued operator lowerings ────────────────────────────────

    ExprID lower_cardinality(ExprID S, const std::string& ctx) {
        const Type* et = set_element_type_of(S);
        if (!et) return pool_.intern_int_constant(0);  // card of an empty literal
        std::vector<ExprID> parts;
        for (int oi : problem_.objects_of_type(et))
            parts.push_back(membership(S, object_const(oi), ctx));
        return builder_.make_count(std::move(parts));
    }

    ExprID lower_subseteq(ExprID S1, ExprID S2, const std::string& ctx) {
        const Type* et = element_type_of_pair(S1, S2);
        if (!et) return true_;  // {} ⊆ {}
        std::vector<ExprID> parts;
        for (int oi : problem_.objects_of_type(et)) {
            ExprID e = object_const(oi);
            // ⟦S1⟧_e → ⟦S2⟧_e
            parts.push_back(builder_.make_or({builder_.make_not(membership(S1, e, ctx)),
                                     membership(S2, e, ctx)}));
        }
        return builder_.make_and(std::move(parts));
    }

    ExprID lower_disjoint(ExprID S1, ExprID S2, const std::string& ctx) {
        const Type* et = element_type_of_pair(S1, S2);
        if (!et) return true_;
        std::vector<ExprID> parts;
        for (int oi : problem_.objects_of_type(et)) {
            ExprID e = object_const(oi);
            parts.push_back(builder_.make_not(builder_.make_and({membership(S1, e, ctx),
                                               membership(S2, e, ctx)})));
        }
        return builder_.make_and(std::move(parts));
    }

    ExprID lower_set_equality(ExprID S1, ExprID S2, const std::string& ctx) {
        const Type* et = element_type_of_pair(S1, S2);
        if (!et) return true_;
        std::vector<ExprID> parts;
        for (int oi : problem_.objects_of_type(et)) {
            ExprID e = object_const(oi);
            ExprID a = membership(S1, e, ctx), b = membership(S2, e, ctx);
            // a ↔ b  ≡  (a → b) ∧ (b → a)
            parts.push_back(builder_.make_and({builder_.make_or({builder_.make_not(a), b}),
                                      builder_.make_or({builder_.make_not(b), a})}));
        }
        return builder_.make_and(std::move(parts));
    }

    // ── effect case split ────────────────────────────────────────────────────

    void lower_effect(const EffectExpression& ee, Action& out, const std::string& ctx) {
        ExprID cond = EXPR_NULL;
        if (ee.is_conditional()) {
            cond = rewrite_expr(ee.condition_id(), ctx);
            if (pool_.is_false_constant(cond)) return;         // never fires — drop
            if (pool_.is_true_constant(cond)) cond = EXPR_NULL; // trivially true
        }

        const Fluent* f = set_fluent_of(ee.fluent_id());
        if (!f) {
            // Non-set target: rewrite the value (it may read set expressions,
            // e.g. `n := card(S)`); the target itself has no set nodes.
            out.add_effect(Effect(EffectExpression(
                ee.kind(),
                rewrite_expr(ee.fluent_id(), ctx),
                rewrite_expr(ee.value_id(), ctx),
                cond, &pool_)));
            return;
        }
        if (ee.kind() != EffectExpression::Kind::ASSIGN)
            throw std::runtime_error("Set fluent '" + f->name() +
                                     "' has a non-assign (increase/decrease) effect in " + ctx);
        lower_set_effect(*f, ee.fluent_id(), ee.value_id(), cond, out, ctx);
    }

    void lower_set_effect(const Fluent& f, ExprID target, ExprID rhs, ExprID base_cond,
                          Action& out, const std::string& ctx) {
        std::vector<ExprID> args(pool_.arguments(target).begin(),
                                 pool_.arguments(target).end());

        // (a) add/remove chain on the target itself → single-cell effects.
        std::vector<std::pair<ExprID, bool>> chain;  // (element, is_add)
        if (collect_self_chain(rhs, target, chain)) {
            bool has_add = false, has_remove = false;
            for (const auto& op : chain) (op.second ? has_add : has_remove) = true;
            if (has_add && has_remove)
                throw std::runtime_error(
                    "Mixed add/remove chain onto set fluent '" + f.name() +
                    "' is not supported in v1 (" + ctx + ").");
            for (const auto& [elem, is_add] : chain)
                emit(out, membership_cell(f, elem, args), is_add ? true_ : false_, base_cond);
            return;
        }

        // (b) monotone union onto the target → per-element conditional add.
        ExprID other;
        if (is_union_onto_target(rhs, target, other)) {
            const Type* et = f.set_element_type();
            for (int oi : problem_.objects_of_type(et)) {
                ExprID e = object_const(oi);
                ExprID q = membership(other, e, ctx);  // ⟦Q⟧_e
                if (pool_.is_false_constant(q)) continue;  // e never added
                ExprID cell_cond = combine_and(base_cond,
                                               pool_.is_true_constant(q) ? EXPR_NULL : q);
                emit(out, membership_cell(f, e, args), true_, cell_cond);
            }
            return;
        }

        // (c) general assignment → one expression-valued boolean assign per cell.
        const Type* et = f.set_element_type();
        for (int oi : problem_.objects_of_type(et)) {
            ExprID e = object_const(oi);
            emit(out, membership_cell(f, e, args), membership(rhs, e, ctx), base_cond);
        }
    }

    // Walk a SET_ADD/SET_REMOVE chain; succeeds iff its innermost base is the
    // effect target set variable itself. `chain` is filled outermost-first
    // (order is irrelevant for a pure add- or remove-only chain).
    bool collect_self_chain(ExprID rhs, ExprID target,
                            std::vector<std::pair<ExprID, bool>>& chain) {
        ExprID cur = rhs;
        while (pool_.is_function_application(cur)) {
            ExprOperator op = pool_.op(cur);
            if (op == ExprOperator::SET_ADD)
                chain.emplace_back(pool_.argument(cur, 0), true);
            else if (op == ExprOperator::SET_REMOVE)
                chain.emplace_back(pool_.argument(cur, 0), false);
            else
                return false;
            cur = pool_.argument(cur, 1);
        }
        return cur == target;  // innermost base must be the target itself
    }

    bool is_union_onto_target(ExprID rhs, ExprID target, ExprID& other) {
        if (!(pool_.is_function_application(rhs) &&
              pool_.op(rhs) == ExprOperator::SET_UNION))
            return false;
        ExprID a = pool_.argument(rhs, 0), b = pool_.argument(rhs, 1);
        if (a == target) { other = b; return true; }
        if (b == target) { other = a; return true; }
        return false;
    }

    void emit(Action& out, ExprID cell, ExprID value, ExprID cond) {
        out.add_effect(Effect(EffectExpression(EffectExpression::Kind::ASSIGN,
                                               cell, value, cond, &pool_)));
    }

    // ── small helpers ────────────────────────────────────────────────────────

    // The set fluent named by a set-typed state variable, or nullptr.
    const Fluent* set_fluent_of(ExprID id) const {
        if (!id.valid() || !pool_.is_state_variable(id)) return nullptr;
        const Fluent* f = problem_.find_fluent(pool_.payload_string(pool_.head_symbol_id(id)));
        return (f && f->is_set_fluent()) ? f : nullptr;
    }

    bool is_set_expr(ExprID id) const {
        if (set_fluent_of(id)) return true;
        return pool_.is_function_application(id) && is_set_operator(pool_.op(id));
    }

    // The element type of a set expression, found by descending to a set fluent
    // or a typed constant. Null only for a context-free empty literal.
    const Type* set_element_type_of(ExprID S) const {
        if (const Fluent* f = set_fluent_of(S)) return f->set_element_type();
        if (pool_.is_function_application(S)) {
            switch (pool_.op(S)) {
                case ExprOperator::SET_ADD:
                case ExprOperator::SET_REMOVE:
                    if (const Type* t = set_element_type_of(pool_.argument(S, 1))) return t;
                    return problem_.type_for_id(pool_.argument(S, 0));
                case ExprOperator::SET_UNION:
                case ExprOperator::SET_INTERSECT:
                case ExprOperator::SET_DIFFERENCE:
                    if (const Type* t = set_element_type_of(pool_.argument(S, 0))) return t;
                    return set_element_type_of(pool_.argument(S, 1));
                case ExprOperator::SET_CONSTANT:
                    for (ExprID c : pool_.arguments(S))
                        if (const Type* t = problem_.type_for_id(c)) return t;
                    return nullptr;
                default:
                    break;
            }
        }
        return nullptr;
    }

    const Type* element_type_of_pair(ExprID S1, ExprID S2) const {
        if (const Type* t = set_element_type_of(S1)) return t;
        return set_element_type_of(S2);
    }

    // Build the boolean membership state variable s(e, args...).
    ExprID membership_cell(const Fluent& f, ExprID e, const std::vector<ExprID>& args) {
        std::vector<ExprID> cell_args;
        cell_args.reserve(args.size() + 1);
        cell_args.push_back(e);
        for (ExprID a : args) cell_args.push_back(a);
        return pool_.intern_state_variable(f.name(), bool_type_id_, cell_args);
    }

    // An object constant for the enumerated element; the node construction is
    // delegated to the shared builder (this only resolves index → name+type_id).
    ExprID object_const(int obj_idx) {
        const Object& obj = problem_.object(obj_idx);
        return builder_.object_constant(obj.name(), problem_.type_id_of(obj.type()));
    }

    // AND of two optional conditions (an unset/EXPR_NULL side is "no condition").
    ExprID combine_and(ExprID a, ExprID b) {
        if (!a.valid()) return b;
        if (!b.valid()) return a;
        return builder_.make_and({a, b});
    }
};

} // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool has_set_fluents(const Problem& problem) {
    for (const Fluent& f : problem.fluents())
        if (f.is_set_fluent()) return true;
    return false;
}

Problem lower_sets(const Problem& problem) {
    ExprPool& pool = *problem.pool_ptr();
    SetLowerer lowerer(problem);

    std::vector<Action> new_actions;
    new_actions.reserve(problem.actions().size());
    for (const Action& a : problem.actions())
        new_actions.push_back(lowerer.rewrite_action(a));

    std::vector<Goal> new_goals;
    new_goals.reserve(problem.goals().size());
    for (const Goal& g : problem.goals())
        new_goals.emplace_back(lowerer.rewrite_expr(g.goal_id(), "goal"), &pool);

    std::vector<ExprID> new_invariants;
    new_invariants.reserve(problem.state_invariants().size());
    for (ExprID inv : problem.state_invariants())
        new_invariants.push_back(lowerer.rewrite_expr(inv, "state invariant"));

    std::vector<Assignment> new_initial;
    new_initial.reserve(problem.initial_state().size());
    for (const Assignment& asn : problem.initial_state())
        lowerer.rewrite_initial(asn, new_initial);

    // Assemble: drop set fluent schemas, append boolean membership fluents.
    Problem result = problem.clone_base();
    std::vector<Fluent> new_fluents;
    new_fluents.reserve(problem.fluent_count() + lowerer.membership_fluents().size());
    for (const Fluent& f : problem.fluents())
        if (!f.is_set_fluent()) new_fluents.push_back(f);
    for (const Fluent& m : lowerer.membership_fluents()) new_fluents.push_back(m);
    result.fluents_ = std::move(new_fluents);

    result.actions_ = std::move(new_actions);
    for (size_t i = 0; i < result.actions_.size(); ++i)
        result.actions_[i].set_id(static_cast<int>(i));

    result.goals_ = std::move(new_goals);
    result.initial_state_ = std::move(new_initial);
    result.state_invariants_ = std::move(new_invariants);

    result.rebuild_derived_indices();
    return result;
}

void assert_no_set_nodes(const Problem& problem) {
    const ExprPool& pool = problem.pool();
    for_each_expression(problem, [&](ExprID eid) {
        if (pool.is_function_application(eid) && is_set_operator(pool.op(eid)))
            throw std::runtime_error(
                "Set lowering invariant violated: a SET_* node survived (" +
                pool.to_string(eid) + ").");
        if (pool.is_state_variable(eid)) {
            const Type* t = problem.type_for_id(eid);
            if (t && t->is_set_type())
                throw std::runtime_error(
                    "Set lowering invariant violated: a set-typed state variable "
                    "survived (" + pool.to_string(eid) + ").");
        }
    });
}

void SetLoweringPass::apply(PreprocessedProblem& result) const {
    if (!has_set_fluents(result.problem)) return;

    ScopedTimer timer;
    result.problem = lower_sets(result.problem);
    assert_no_set_nodes(result.problem);

    Logger::instance().component(VerbosityLevel::INFO, "SetLowering", {
        {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
        {"status", "sets lowered to boolean membership cells"}
    });
}

} // namespace rantanplan
