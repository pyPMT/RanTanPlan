#include "term_resolution_pass.hpp"

#include "../problem/expr_builder.hpp"
#include "../problem/expr_enums.hpp"
#include "../problem/problem.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace rantanplan {

namespace {

// A state-reading term: a fluent application or an array access.
bool is_state_reading(const ExprPool& pool, ExprID id) {
    if (pool.is_state_variable(id)) return true;
    return pool.is_function_application(id) &&
           pool.op(id) == ExprOperator::ARRAY_INDEX;
}

// Does `id`'s subtree contain a state-reading node?
bool contains_state_read(const ExprPool& pool, ExprID id) {
    if (!id.valid()) return false;
    if (is_state_reading(pool, id)) return true;
    for (ExprID c : pool.get(id).children)
        if (contains_state_read(pool, c)) return true;
    return false;
}

// Is the element position (argument 0) of this operator a cell coordinate
// after set lowering (so a state-reading element there is a nested term)?
bool has_element_position(ExprOperator op) {
    return op == ExprOperator::SET_MEMBER || op == ExprOperator::SET_ADD ||
           op == ExprOperator::SET_REMOVE;
}

// Does `id` contain a nested fluent term (state-reading node in an argument
// position of a fluent application, or in a set element position)?
bool expr_has_nested_term(const ExprPool& pool, ExprID id) {
    if (!id.valid()) return false;
    if (pool.is_state_variable(id)) {
        for (size_t i = 0; i < pool.argument_count(id); ++i) {
            ExprID arg = pool.argument(id, i);
            if (contains_state_read(pool, arg)) return true;
        }
        return false;
    }
    if (pool.is_function_application(id)) {
        if (has_element_position(pool.op(id)) &&
            contains_state_read(pool, pool.argument(id, 0)))
            return true;
        for (size_t i = 0; i < pool.argument_count(id); ++i)
            if (expr_has_nested_term(pool, pool.argument(id, i))) return true;
        return false;
    }
    for (ExprID c : pool.get(id).children)
        if (expr_has_nested_term(pool, c)) return true;
    return false;
}

// ---------------------------------------------------------------------------
// The per-action resolver. Walks expressions bottom-up; a state-reading term
// found in an argument/element position (whose own arguments have already
// been resolved, so it is flat) is replaced by a ghost parameter, memoized
// per distinct term so repeated occurrences share one ghost.
// ---------------------------------------------------------------------------
class TermResolver {
public:
    explicit TermResolver(const Problem& problem)
        : problem_(problem), pool_(*problem.pool_ptr()), builder_(pool_) {}

    Action resolve_action(const Action& a) {
        ghost_memo_.clear();
        ghosts_.clear();
        counter_ = 0;
        action_ = &a;
        ctx_ = "action '" + a.name() + "'";

        ExprID pre = a.has_precondition()
                         ? resolve(a.precondition_id(), false) : EXPR_NULL;
        ExprID cost = a.has_explicit_cost()
                          ? resolve(a.cost_id(), false) : EXPR_NULL;

        struct ResolvedEffect {
            EffectExpression::Kind kind;
            ExprID fluent, value, condition;
        };
        std::vector<ResolvedEffect> effects;
        effects.reserve(a.effects().size());
        for (const Effect& eff : a.effects()) {
            const EffectExpression& ee = eff.effect_expression();
            if (!ee.forall_variable_ids().empty())
                throw std::runtime_error(
                    "Term resolution ran on an unexpanded forall-effect in " +
                    ctx_ + " — QuantifierExpansionPass must run before it.");
            effects.push_back({ee.kind(),
                               resolve(ee.fluent_id(), false),
                               resolve(ee.value_id(), false),
                               resolve(ee.condition_id(), false)});
        }

        if (ghosts_.empty()) return a;  // untouched action passes through

        Action out(a.name());
        out.set_pool(&pool_);
        std::vector<Parameter> params = a.parameters();
        std::vector<ExprID> pre_parts;
        if (pre.valid()) pre_parts.push_back(pre);
        for (const Ghost& g : ghosts_) {
            params.emplace_back(g.name, g.type);
            pre_parts.push_back(builder_.make_eq(g.term, g.param));
        }
        out.set_parameters(params);
        out.set_precondition_id(builder_.make_and(std::move(pre_parts)));
        if (cost.valid()) out.set_cost_id(cost);
        for (const ResolvedEffect& r : effects)
            out.add_effect(Effect(EffectExpression(
                r.kind, r.fluent, r.value, r.condition, &pool_)));
        return out;
    }

private:
    struct Ghost {
        std::string name;
        const Type* type;
        ExprID term;   // the resolved (flat) inner term the ghost stands for
        ExprID param;  // the PARAMETER node substituted for it
    };

    // Bottom-up rewrite. `in_arg` is true exactly when `id` sits in a fluent
    // argument / set element position, i.e. where a state-reading term must
    // become a ghost parameter.
    ExprID resolve(ExprID id, bool in_arg) {
        if (!id.valid()) return id;

        if (pool_.is_state_variable(id)) {
            ExprID out = resolve_arguments(id, /*first_in_arg=*/true,
                                           /*rest_in_arg=*/true);
            return in_arg ? ghost_for(out) : out;
        }

        if (pool_.is_function_application(id)) {
            ExprOperator op = pool_.op(id);
            if (op == ExprOperator::ARRAY_INDEX) {
                // The access chain is one state-reading term: recurse into the
                // base (its own arguments may nest) and the index without
                // entering argument context, then ghost the whole chain if it
                // sits in one.
                ExprID out = resolve_arguments(id, false, false);
                return in_arg ? ghost_for(out) : out;
            }
            if (has_element_position(op))
                return resolve_arguments(id, /*first_in_arg=*/true,
                                         /*rest_in_arg=*/false);
            return resolve_arguments(id, false, false);
        }

        return id;  // leaves: constants, parameters, symbols, variables
    }

    // Rebuild a head+arguments node with resolved arguments; argument 0 is
    // resolved in argument context iff `first_in_arg`, the rest iff
    // `rest_in_arg`. Returns the original id when nothing changed.
    ExprID resolve_arguments(ExprID id, bool first_in_arg, bool rest_in_arg) {
        std::vector<ExprID> children;
        children.reserve(pool_.argument_count(id) + 1);
        children.push_back(pool_.head_symbol_id(id));
        bool changed = false;
        for (size_t i = 0; i < pool_.argument_count(id); ++i) {
            ExprID arg = pool_.argument(id, i);
            ExprID resolved = resolve(arg, i == 0 ? first_in_arg : rest_in_arg);
            changed |= (resolved != arg);
            children.push_back(resolved);
        }
        return changed ? builder_.reintern(id, std::move(children)) : id;
    }

    ExprID ghost_for(ExprID term) {
        auto it = ghost_memo_.find(term);
        if (it != ghost_memo_.end()) return it->second;

        const Type* vt = value_type_of(term);
        if (!vt || !vt->is_object())
            throw std::invalid_argument(
                "Nested fluent term '" + pool_.to_string(term) + "' in " +
                ctx_ + " is not object-valued; only object-valued nested "
                "terms are supported (int-valued inner terms are the "
                "state-dependent-index axis, not yet implemented).");

        std::string name;
        do {
            name = "_nested" + std::to_string(counter_++);
        } while (action_->has_parameter(name));

        ExprNode node;
        node.kind = static_cast<int>(ExprKind::PARAMETER);
        node.type_id = problem_.type_id_of(vt);
        node.payload = name;
        ExprID param = pool_.intern(std::move(node));

        ghosts_.push_back({name, vt, term, param});
        ghost_memo_.emplace(term, param);
        return param;
    }

    // The value type of a state-reading term: a fluent application carries its
    // fluent's value type; an array access chain must fully index its array
    // fluent and yields the leaf element type.
    const Type* value_type_of(ExprID term) const {
        if (pool_.is_state_variable(term)) return problem_.type_for_id(term);
        size_t depth = 0;
        ExprID cur = term;
        while (pool_.is_function_application(cur) &&
               pool_.op(cur) == ExprOperator::ARRAY_INDEX) {
            ++depth;
            cur = pool_.argument(cur, 0);
        }
        if (!pool_.is_state_variable(cur)) return nullptr;
        const Fluent* f = problem_.find_fluent(
            pool_.payload_string(pool_.head_symbol_id(cur)));
        if (!f) return nullptr;
        if (f->array_sizes().size() != depth) return nullptr;  // partial access
        return f->array_leaf_element_type();
    }

    const Problem& problem_;
    ExprPool& pool_;
    ExprBuilder builder_;
    const Action* action_ = nullptr;
    std::string ctx_;
    std::unordered_map<ExprID, ExprID> ghost_memo_;
    std::vector<Ghost> ghosts_;
    int counter_ = 0;
};

void reject_nested_outside_actions(const Problem& problem) {
    const ExprPool& pool = problem.pool();
    auto reject = [&](ExprID id, const std::string& where) {
        if (expr_has_nested_term(pool, id))
            throw std::invalid_argument(
                "Nested fluent term in " + where + " is not supported: only "
                "action expressions can host a ghost parameter. Rewrite with "
                "an explicit existential quantifier "
                "(Exists(v, And(Equals(<inner term>, v), ...))).");
    };
    for (const Goal& g : problem.goals()) reject(g.goal_id(), "a goal");
    for (ExprID inv : problem.state_invariants())
        reject(inv, "a state invariant");
    for (const Assignment& asn : problem.initial_state()) {
        reject(asn.fluent_id(), "the initial state");
        reject(asn.value_id(), "the initial state");
    }
}

} // namespace

bool has_nested_fluent_terms(const Problem& problem) {
    const ExprPool& pool = problem.pool();
    for (const Action& a : problem.actions()) {
        if (a.has_precondition() &&
            expr_has_nested_term(pool, a.precondition_id()))
            return true;
        if (a.has_explicit_cost() && expr_has_nested_term(pool, a.cost_id()))
            return true;
        for (const Effect& eff : a.effects()) {
            const EffectExpression& ee = eff.effect_expression();
            if (expr_has_nested_term(pool, ee.fluent_id()) ||
                expr_has_nested_term(pool, ee.value_id()) ||
                expr_has_nested_term(pool, ee.condition_id()))
                return true;
        }
    }
    return false;
}

Problem resolve_nested_terms(const Problem& problem) {
    TermResolver resolver(problem);

    std::vector<Action> new_actions;
    new_actions.reserve(problem.actions().size());
    for (const Action& a : problem.actions())
        new_actions.push_back(resolver.resolve_action(a));

    Problem result = problem;  // cheap copy (shared ExprPool)
    result.actions_ = std::move(new_actions);
    for (size_t i = 0; i < result.actions_.size(); ++i)
        result.actions_[i].set_id(static_cast<int>(i));
    result.rebuild_derived_indices();
    return result;
}

void TermResolutionPass::apply(PreprocessedProblem& result) const {
    // Goals/invariants/init have no action to hang a ghost on — always check,
    // even when no action nests (clear error beats the encoder's backstop).
    reject_nested_outside_actions(result.problem);

    if (!has_nested_fluent_terms(result.problem)) return;

    ScopedTimer timer;
    result.problem = resolve_nested_terms(result.problem);
    if (has_nested_fluent_terms(result.problem))
        throw std::runtime_error(
            "Term resolution invariant violated: a nested fluent term "
            "survived the pass.");

    Logger::instance().component(VerbosityLevel::INFO, "TermResolution", {
        {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
        {"status", "nested fluent terms resolved via ghost parameters"}
    });
}

} // namespace rantanplan
