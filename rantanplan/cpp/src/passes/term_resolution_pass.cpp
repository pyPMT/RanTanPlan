#include "term_resolution_pass.hpp"

#include "../problem/expr_builder.hpp"
#include "../problem/expr_enums.hpp"
#include "../problem/problem.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"

#include <deque>
#include <optional>
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

public:
    // Goal-side mechanism: goals cannot host a ghost parameter, so an
    // object-valued inner term `t` is expanded by value-universe disjunction
    // instead — `φ` becomes `⋁_{o ∈ objects(T)} (t = o ∧ φ[t → o])`, the
    // Exists-rewrite done mechanically (object fluents are total under CWA,
    // so the disjunction is exhaustive). Innermost terms expand first;
    // distinct terms expand by iteration.
    ExprID expand_condition(ExprID expr, const std::string& ctx) {
        while (true) {
            std::optional<ExprID> term = find_inner_term(expr, false);
            if (!term) return expr;
            const Type* vt = value_type_of(*term);
            if (!vt || !vt->is_object())
                throw std::invalid_argument(
                    "Nested fluent term '" + pool_.to_string(*term) + "' in " +
                    ctx + " is not object-valued; only object-valued nested "
                    "terms can be expanded there.");
            std::vector<ExprID> disjuncts;
            for (int obj_idx : problem_.objects_of_type(vt)) {
                const Object& obj = problem_.object(obj_idx);
                ExprID o = builder_.object_constant(
                    obj.name(), problem_.type_id_of(obj.type()));
                disjuncts.push_back(builder_.make_and(
                    {builder_.make_eq(*term, o),
                     replace_subterm(expr, *term, o)}));
            }
            expr = builder_.make_or(std::move(disjuncts));
        }
    }

private:
    struct Ghost {
        std::string name;
        const Type* type;
        ExprID term;   // the resolved (flat) inner term the ghost stands for
        ExprID param;  // the PARAMETER node substituted for it
    };

    // The innermost state-reading term in a structured position (fluent
    // argument / set element), or nullopt. Mirrors resolve()'s traversal.
    std::optional<ExprID> find_inner_term(ExprID id, bool in_arg) const {
        if (!id.valid()) return std::nullopt;
        if (pool_.is_state_variable(id)) {
            for (size_t i = 0; i < pool_.argument_count(id); ++i)
                if (auto t = find_inner_term(pool_.argument(id, i), true))
                    return t;
            if (in_arg) return id;
            return std::nullopt;
        }
        if (pool_.is_function_application(id)) {
            ExprOperator op = pool_.op(id);
            if (op == ExprOperator::ARRAY_INDEX) {
                for (size_t i = 0; i < pool_.argument_count(id); ++i)
                    if (auto t = find_inner_term(pool_.argument(id, i), false))
                        return t;
                if (in_arg) return id;
                return std::nullopt;
            }
            bool element_pos0 = has_element_position(op);
            for (size_t i = 0; i < pool_.argument_count(id); ++i)
                if (auto t = find_inner_term(pool_.argument(id, i),
                                             element_pos0 && i == 0))
                    return t;
            return std::nullopt;
        }
        return std::nullopt;
    }

    // Replace every occurrence of the exact subterm `from` with `to`.
    ExprID replace_subterm(ExprID id, ExprID from, ExprID to) {
        if (!id.valid()) return id;
        if (id == from) return to;
        const ExprNode& node = pool_.get(id);
        if (node.children.empty()) return id;
        std::vector<ExprID> children = node.children;  // copy before interning
        bool changed = false;
        for (ExprID& c : children) {
            ExprID r = replace_subterm(c, from, to);
            changed |= (r != c);
            c = r;
        }
        return changed ? builder_.reintern(id, std::move(children)) : id;
    }

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
        node.kind = ExprKind::PARAMETER;
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

// ---------------------------------------------------------------------------
// State-dependent index splitting (N2 — undefinedness-semantics.md, R2).
// ---------------------------------------------------------------------------

// Does `id` contain a state-dependent array index anywhere?
bool expr_has_state_dep_index(const ExprPool& pool, ExprID id) {
    if (!id.valid()) return false;
    if (pool.is_function_application(id) &&
        pool.op(id) == ExprOperator::ARRAY_INDEX &&
        contains_state_read(pool, pool.argument(id, 1)))
        return true;
    for (ExprID c : pool.get(id).children)
        if (expr_has_state_dep_index(pool, c)) return true;
    return false;
}

// One split step: the state-dependent index expression to enumerate, and the
// size of the array dimension it indexes.
struct SplitTarget {
    ExprID index;
    int64_t dim_size;
};

class IndexSplitter {
public:
    explicit IndexSplitter(const Problem& problem)
        : problem_(problem), pool_(*problem.pool_ptr()), builder_(pool_) {}

    // Expand one action into its fully split instances (identity vector for
    // an action with no state-dependent index). Instances keep the action's
    // name and signature — like int-parameter instantiation, several lifted
    // instances of one name are fine; they differ only in preconditions.
    std::vector<Action> split_action(const Action& a) {
        std::vector<Action> done;
        std::deque<Action> work;
        work.push_back(a);
        while (!work.empty()) {
            Action cur = std::move(work.front());
            work.pop_front();
            std::optional<SplitTarget> target = find_target(cur);
            if (!target) {
                done.push_back(std::move(cur));
                continue;
            }
            // In-range instances: `e = p`, index -> constant p.
            for (int64_t p = 0; p < target->dim_size; ++p)
                work.push_back(instance(cur, target->index,
                                        pool_.intern_int_constant(p),
                                        builder_.make_eq(
                                            target->index,
                                            pool_.intern_int_constant(p))));
            // Boundary instance: `e < 0 ∨ e ≥ n`, index -> the statically
            // out-of-bounds constant n. ArrayLoweringPass then applies the
            // permissive policy at compile time: a guarded
            // write is dropped with ¬guard added to the precondition,
            // OR/COUNT absorb the ⊥ read, and an unconditional ⊥
            // discards the instance (see undefinedness-semantics.md).
            ExprID zero = pool_.intern_int_constant(0);
            ExprID n = pool_.intern_int_constant(target->dim_size);
            ExprID oob = builder_.make_or(
                {builder_.make_app(ExprOperator::LESS_THAN,
                                   {target->index, zero}),
                 builder_.make_app(ExprOperator::GREATER_EQUAL,
                                   {target->index, n})});
            work.push_back(instance(cur, target->index, n, oob));
        }
        return done;
    }

private:
    // Innermost-first: recurse into children before testing the node itself,
    // so `a[b[ptr]]` splits on `ptr` (over b's dimension) before `b[p]`
    // (over a's dimension).
    std::optional<SplitTarget> find_target(ExprID id) const {
        if (!id.valid()) return std::nullopt;
        for (ExprID c : pool_.get(id).children)
            if (auto t = find_target(c)) return t;
        if (pool_.is_function_application(id) &&
            pool_.op(id) == ExprOperator::ARRAY_INDEX) {
            ExprID idx = pool_.argument(id, 1);
            if (contains_state_read(pool_, idx))
                return SplitTarget{idx, dim_size_of(id)};
        }
        return std::nullopt;
    }

    std::optional<SplitTarget> find_target(const Action& a) const {
        if (a.has_precondition())
            if (auto t = find_target(a.precondition_id())) return t;
        if (a.has_explicit_cost())
            if (auto t = find_target(a.cost_id())) return t;
        for (const Effect& eff : a.effects()) {
            const EffectExpression& ee = eff.effect_expression();
            if (auto t = find_target(ee.fluent_id())) return t;
            if (auto t = find_target(ee.value_id())) return t;
            if (auto t = find_target(ee.condition_id())) return t;
        }
        return std::nullopt;
    }

    // The size of the dimension this ARRAY_INDEX node indexes: the number of
    // ARRAY_INDEX layers below it selects the dimension (outermost dimension
    // first in Fluent::array_sizes(), matching the access chain order).
    int64_t dim_size_of(ExprID access) const {
        size_t below = 0;
        ExprID cur = pool_.argument(access, 0);
        while (pool_.is_function_application(cur) &&
               pool_.op(cur) == ExprOperator::ARRAY_INDEX) {
            ++below;
            cur = pool_.argument(cur, 0);
        }
        if (!pool_.is_state_variable(cur))
            throw std::runtime_error(
                "State-dependent index on a non-fluent array base: " +
                pool_.to_string(access));
        const Fluent* f = problem_.find_fluent(
            pool_.payload_string(pool_.head_symbol_id(cur)));
        if (!f || f->array_sizes().size() <= below)
            throw std::runtime_error(
                "Malformed array access for state-dependent index: " +
                pool_.to_string(access));
        return f->array_sizes()[below];
    }

    // Copy `a` with every ARRAY_INDEX whose index expression *is* `from`
    // rewritten to `to`, and `extra_pre` conjoined to the precondition.
    // Replacement is exact-match on the index slot only — other occurrences
    // of the expression (e.g. in unrelated arithmetic) keep their meaning
    // through the `e = p` equality.
    Action instance(const Action& a, ExprID from, ExprID to, ExprID extra_pre) {
        Action out(a.name());
        out.set_pool(&pool_);
        out.set_parameters(a.parameters());
        ExprID pre = a.has_precondition()
                         ? replace_index(a.precondition_id(), from, to)
                         : EXPR_NULL;
        out.set_precondition_id(pre.valid()
                                    ? builder_.make_and({pre, extra_pre})
                                    : extra_pre);
        if (a.has_explicit_cost())
            out.set_cost_id(replace_index(a.cost_id(), from, to));
        for (const Effect& eff : a.effects()) {
            const EffectExpression& ee = eff.effect_expression();
            out.add_effect(Effect(EffectExpression(
                ee.kind(),
                replace_index(ee.fluent_id(), from, to),
                replace_index(ee.value_id(), from, to),
                replace_index(ee.condition_id(), from, to), &pool_)));
        }
        return out;
    }

    ExprID replace_index(ExprID id, ExprID from, ExprID to) {
        if (!id.valid()) return id;
        const ExprNode& node = pool_.get(id);
        if (node.children.empty()) return id;
        bool is_access = pool_.is_function_application(id) &&
                         pool_.op(id) == ExprOperator::ARRAY_INDEX;
        std::vector<ExprID> children = node.children;  // copy before interning
        bool changed = false;
        for (size_t i = 0; i < children.size(); ++i) {
            // children[0] is the head symbol; children[2] of an ARRAY_INDEX
            // is the index slot.
            ExprID replaced = (is_access && i == 2 && children[i] == from)
                                  ? to
                                  : replace_index(children[i], from, to);
            changed |= (replaced != children[i]);
            children[i] = replaced;
        }
        return changed ? builder_.reintern(id, std::move(children)) : id;
    }

    const Problem& problem_;
    ExprPool& pool_;
    ExprBuilder builder_;
};

void reject_nested_outside_actions(const Problem& problem) {
    const ExprPool& pool = problem.pool();
    auto reject_nested = [&](ExprID id, const std::string& where) {
        if (expr_has_nested_term(pool, id))
            throw std::invalid_argument(
                "Nested fluent term in " + where + " is not supported: only "
                "action expressions can host a ghost parameter, and only "
                "goals get the value-universe expansion. Rewrite with an "
                "explicit existential quantifier "
                "(Exists(v, And(Equals(<inner term>, v), ...))).");
    };
    auto reject_index = [&](ExprID id, const std::string& where) {
        if (expr_has_state_dep_index(pool, id))
            throw std::invalid_argument(
                "State-dependent array index in " + where + " is not "
                "supported: only action expressions can host the per-position "
                "split (goals/invariants/initial state are strict-⊥ — see "
                "docs/specs/undefinedness-semantics.md). Rewrite with an "
                "explicit existential quantifier over the index.");
    };
    // Goals: nested object-valued terms are *expanded* (expand_condition),
    // so only state-dependent indices are rejected here.
    for (const Goal& g : problem.goals()) reject_index(g.goal_id(), "a goal");
    for (ExprID inv : problem.state_invariants()) {
        reject_nested(inv, "a state invariant");
        reject_index(inv, "a state invariant");
    }
    for (const Assignment& asn : problem.initial_state()) {
        reject_nested(asn.fluent_id(), "the initial state");
        reject_nested(asn.value_id(), "the initial state");
        reject_index(asn.fluent_id(), "the initial state");
        reject_index(asn.value_id(), "the initial state");
    }
}

bool goals_have_nested_terms(const Problem& problem) {
    const ExprPool& pool = problem.pool();
    for (const Goal& g : problem.goals())
        if (expr_has_nested_term(pool, g.goal_id())) return true;
    return false;
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

bool has_state_dependent_indices(const Problem& problem) {
    const ExprPool& pool = problem.pool();
    for (const Action& a : problem.actions()) {
        if (a.has_precondition() &&
            expr_has_state_dep_index(pool, a.precondition_id()))
            return true;
        if (a.has_explicit_cost() &&
            expr_has_state_dep_index(pool, a.cost_id()))
            return true;
        for (const Effect& eff : a.effects()) {
            const EffectExpression& ee = eff.effect_expression();
            if (expr_has_state_dep_index(pool, ee.fluent_id()) ||
                expr_has_state_dep_index(pool, ee.value_id()) ||
                expr_has_state_dep_index(pool, ee.condition_id()))
                return true;
        }
    }
    return false;
}

Problem resolve_nested_terms(const Problem& problem) {
    TermResolver resolver(problem);
    ExprPool& pool = *problem.pool_ptr();

    std::vector<Action> new_actions;
    new_actions.reserve(problem.actions().size());
    for (const Action& a : problem.actions())
        new_actions.push_back(resolver.resolve_action(a));

    std::vector<Goal> new_goals;
    new_goals.reserve(problem.goals().size());
    for (const Goal& g : problem.goals())
        new_goals.emplace_back(resolver.expand_condition(g.goal_id(), "a goal"),
                               &pool);

    Problem result = problem;  // cheap copy (shared ExprPool)
    result.actions_ = std::move(new_actions);
    for (size_t i = 0; i < result.actions_.size(); ++i)
        result.actions_[i].set_id(static_cast<int>(i));
    result.goals_ = std::move(new_goals);
    result.rebuild_derived_indices();
    return result;
}

Problem split_state_dependent_indices(const Problem& problem) {
    IndexSplitter splitter(problem);

    std::vector<Action> new_actions;
    for (const Action& a : problem.actions())
        for (Action& inst : splitter.split_action(a))
            new_actions.push_back(std::move(inst));

    Problem result = problem;  // cheap copy (shared ExprPool)
    result.actions_ = std::move(new_actions);
    for (size_t i = 0; i < result.actions_.size(); ++i)
        result.actions_[i].set_id(static_cast<int>(i));
    result.rebuild_derived_indices();
    return result;
}

void TermResolutionPass::apply(PreprocessedProblem& result) const {
    // Invariants/init can host neither mechanism, and goals cannot host the
    // per-position split — always check, even when no action nests (a clear
    // error beats the encoder's backstop).
    reject_nested_outside_actions(result.problem);

    bool split = has_state_dependent_indices(result.problem);
    bool nested = has_nested_fluent_terms(result.problem) ||
                  goals_have_nested_terms(result.problem);
    if (!split && !nested) return;

    ScopedTimer timer;
    // Index splitting first: it makes every array index constant, so the
    // ghost phase (and its pinning equalities) only ever sees constant-index
    // accesses. Splitting can surface no *new* nested terms, but a split
    // equality `b[0] = p` may still contain an object-fluent inner term that
    // the ghost phase resolves.
    if (split) {
        result.problem = split_state_dependent_indices(result.problem);
        if (has_state_dependent_indices(result.problem))
            throw std::runtime_error(
                "Term resolution invariant violated: a state-dependent "
                "index survived the split.");
    }
    if (has_nested_fluent_terms(result.problem) ||
        goals_have_nested_terms(result.problem)) {
        result.problem = resolve_nested_terms(result.problem);
        if (has_nested_fluent_terms(result.problem) ||
            goals_have_nested_terms(result.problem))
            throw std::runtime_error(
                "Term resolution invariant violated: a nested fluent term "
                "survived the pass.");
    }

    Logger::instance().component(VerbosityLevel::INFO, "TermResolution", {
        {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
        {"status", split && nested
                       ? "state-dependent indices split; nested terms ghosted"
                       : (split ? "state-dependent indices split"
                                : "nested fluent terms resolved via ghost "
                                  "parameters")}
    });
}

} // namespace rantanplan
