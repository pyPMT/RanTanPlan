#include "quantifier_expansion_pass.hpp"
#include "../problem/const_fold.hpp"
#include "../problem/expr_builder.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"
#include "../util/stats.hpp"
#include "../util/unsupported_problem_error.hpp"

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace rantanplan {

namespace {

using VarAssignment = std::unordered_map<std::string, ExprID>;

/// Rewrites forall/exists condition nodes into AND/OR, and expands forall-effects
/// into one effect per binding, over each variable's finite domain. Object
/// variables enumerate the type's objects; range variables enumerate their
/// (already-constant) integer interval.
class QuantifierExpander {
public:
    QuantifierExpander(const Problem& problem, ExprPool& pool)
        : problem_(problem), pool_(pool), builder_(pool) {}

    /// Expand every quantifier in a condition `id`; returns `id` unchanged when
    /// there is nothing to expand.
    ExprID expand(ExprID id) {
        if (!id.valid()) return id;
        if (pool_.is_quantifier(id)) return expand_quantifier(id);
        if (pool_.is_leaf(id)) return id;
        return map_children(id, [&](ExprID c) { return expand(c); });
    }

    /// Expand a (possibly forall-quantified) effect into one or more effects, with
    /// each effect's condition quantifier-expanded. Non-quantified effects yield a
    /// single effect (condition still expanded).
    void expand_effect(const Effect& eff, std::vector<Effect>& out) {
        const auto& ee = eff.effect_expression();
        if (ee.forall_variable_ids().empty()) {
            out.push_back(finalize_effect(ee, ee.fluent_id(), ee.value_id(),
                                          ee.condition_id()));
            return;
        }
        ++effects_expanded_;
        VarAssignment assignment;
        for_each_binding(ee.forall_variable_ids(), 0, assignment, [&]() {
            out.push_back(finalize_effect(ee,
                                          substitute(ee.fluent_id(), assignment),
                                          substitute(ee.value_id(), assignment),
                                          substitute(ee.condition_id(), assignment)));
        });
    }

    size_t expanded() const { return expanded_; }
    size_t effects_expanded() const { return effects_expanded_; }

private:
    // ── Condition quantifier expansion ───────────────────────────────────

    ExprID expand_quantifier(ExprID id) {
        ++expanded_;
        size_t n = pool_.child_count(id);
        std::vector<ExprID> decls;
        for (size_t i = 1; i + 1 < n; ++i) decls.push_back(pool_.child(id, i));
        ExprID body = pool_.child(id, n - 1);
        bool forall = pool_.is_forall(id);

        // One expanded body per binding (nested quantifiers handled by the inner
        // expand()); combine as AND (forall) / OR (exists).
        std::vector<ExprID> parts;
        VarAssignment assignment;
        for_each_binding(decls, 0, assignment, [&]() {
            parts.push_back(expand(substitute(body, assignment)));
        });
        return combine(forall, parts);
    }

    // ── Forall-effect expansion ──────────────────────────────────────────

    /// Build an effect with no forall variables, its condition quantifier-expanded.
    Effect finalize_effect(const EffectExpression& ee, ExprID fluent, ExprID value,
                           ExprID condition) {
        return Effect(EffectExpression(ee.kind(), fluent, value, expand(condition),
                                       {}, &pool_));
    }

    // ── Shared: enumeration and structural recursion ─────────────────────

    /// Enumerate every binding of `decls` — the Cartesian product over each
    /// variable's finite domain — calling `emit` with `assignment` populated for
    /// each. Shared by condition and forall-effect expansion. `var_name` is copied,
    /// not referenced: domain_of() interns constants and may reallocate the pool.
    template <class Emit>
    void for_each_binding(const std::vector<ExprID>& decls, size_t idx,
                          VarAssignment& assignment, const Emit& emit) {
        if (idx == decls.size()) {
            emit();
            return;
        }
        std::string var_name = pool_.payload_string(decls[idx]);
        for (ExprID value : domain_of(decls[idx])) {
            assignment[var_name] = value;
            for_each_binding(decls, idx + 1, assignment, emit);
        }
        assignment.erase(var_name);
    }

    /// Apply `fn` to each child of `id`, reinterning only if a child changed
    /// (otherwise `id` is returned as-is — no needless pool churn).
    template <class Fn>
    ExprID map_children(ExprID id, const Fn& fn) {
        std::vector<ExprID> kids(pool_.children(id).begin(), pool_.children(id).end());
        bool changed = false;
        for (auto& c : kids) {
            ExprID nc = fn(c);
            changed |= (nc != c);
            c = nc;
        }
        return changed ? builder_.reintern(id, std::move(kids)) : id;
    }

    // ── Shared: domains and substitution ─────────────────────────────────

    /// The finite domain of a bound variable, as constant ExprIDs.
    std::vector<ExprID> domain_of(ExprID decl) {
        const std::string& var_name = pool_.payload_string(decl);
        std::vector<ExprID> values;

        // Range variable: a VARIABLE node carrying [initial, last] as children.
        if (!pool_.is_leaf(decl)) {
            int64_t lo = eval_bound(pool_.child(decl, 0), var_name);
            int64_t hi = eval_bound(pool_.child(decl, 1), var_name);
            for (int64_t v = lo; v <= hi; ++v)
                values.push_back(pool_.intern_int_constant(v));
            return values;
        }

        // Object variable: enumerate the type's objects.
        int tid = pool_.type_id(decl);
        const Type* type = (tid >= 0 && tid < static_cast<int>(problem_.types().size()))
                               ? &problem_.types()[tid] : nullptr;
        for (int oidx : problem_.objects_of_type(type))
            values.push_back(make_object_constant(oidx));
        return values;
    }

    int64_t eval_bound(ExprID id, const std::string& var_name) {
        // After IntParameterInstantiation the bound is ground, but it may still
        // be constant arithmetic (e.g. `lc - 1` with lc substituted) — fold it.
        if (auto value = fold_const_int(id, pool_)) return *value;
        throw UnsupportedProblemError(
            "Range variable '" + var_name + "' has a non-constant bound (" +
            pool_.to_string(id) + "); integer parameters must be instantiated first.");
    }

    /// Replace each VARIABLE-reference leaf named in `sub` with the mapped constant.
    ExprID substitute(ExprID id, const VarAssignment& sub) {
        if (!id.valid()) return id;
        if (pool_.is_variable(id) && pool_.is_leaf(id)) {
            auto it = sub.find(pool_.payload_string(id));
            return it != sub.end() ? it->second : id;
        }
        if (pool_.is_leaf(id)) return id;
        return map_children(id, [&](ExprID c) { return substitute(c, sub); });
    }

    // ── Node construction ────────────────────────────────────────────────

    ExprID combine(bool forall, const std::vector<ExprID>& parts) {
        // Empty domain → forall vacuously true / exists false; a single part
        // stands alone — all handled by the builder's constant folding.
        return forall ? builder_.make_and(parts) : builder_.make_or(parts);
    }

    ExprID make_object_constant(int obj_idx) {
        const Object& obj = problem_.object(obj_idx);
        return builder_.object_constant(obj.name(), problem_.type_id_of(obj.type()));
    }

    const Problem& problem_;
    ExprPool& pool_;
    ExprBuilder builder_;
    size_t expanded_ = 0;
    size_t effects_expanded_ = 0;
};

}  // namespace

void QuantifierExpansionPass::apply(PreprocessedProblem& result) const {
    ScopedTimer timer;  // measuring-only; the pipeline records the stats key
    const auto& problem = result.problem;
    ExprPool& pool = *problem.pool_ptr();
    QuantifierExpander expander(problem, pool);

    bool any_changed = false;

    std::vector<Action> new_actions;
    new_actions.reserve(problem.actions().size());
    for (const auto& action : problem.actions()) {
        Action new_action = action;

        if (new_action.has_precondition()) {
            ExprID np = expander.expand(action.precondition_id());
            if (np != action.precondition_id()) {
                new_action.set_precondition_id(np);
                any_changed = true;
            }
        }

        // Expand forall-effects into concrete effects and remove quantifiers
        // from each effect's condition.
        std::vector<Effect> new_effects;
        new_effects.reserve(action.effects().size());
        for (const auto& effect : action.effects())
            expander.expand_effect(effect, new_effects);

        // Changed if the count differs or any effect expression differs.
        bool effects_changed = new_effects.size() != action.effects().size();
        for (size_t i = 0; !effects_changed && i < new_effects.size(); ++i)
            effects_changed = new_effects[i].effect_expression() !=
                              action.effects()[i].effect_expression();
        any_changed |= effects_changed;
        new_action.mutable_effects() = std::move(new_effects);

        new_actions.push_back(std::move(new_action));
    }

    std::vector<Goal> new_goals;
    new_goals.reserve(problem.goals().size());
    for (const auto& goal : problem.goals()) {
        Goal new_goal = goal;
        ExprID ng = expander.expand(goal.goal_id());
        if (ng != goal.goal_id()) {
            new_goal.set_goal_id(ng);
            any_changed = true;
        }
        new_goals.push_back(std::move(new_goal));
    }

    std::vector<ExprID> new_invariants;
    new_invariants.reserve(problem.state_invariants().size());
    for (ExprID inv : problem.state_invariants()) {
        ExprID ni = expander.expand(inv);
        if (ni != inv) any_changed = true;
        new_invariants.push_back(ni);
    }

    Stats::instance().set("quantifier_expansion.conditions",
                          static_cast<double>(expander.expanded()));
    Stats::instance().set("quantifier_expansion.effects",
                          static_cast<double>(expander.effects_expanded()));

    if (!any_changed) {
        Logger::instance().component(VerbosityLevel::INFO, name(), {
            {"status", "no quantifiers"}
        });
        return;
    }

    result.problem = problem.with_simplified(
        std::move(new_actions), std::move(new_goals),
        std::vector<Assignment>(problem.initial_state()),
        std::move(new_invariants));

    Logger::instance().component(VerbosityLevel::INFO, name(), {
        {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
        {"conditions", std::to_string(expander.expanded())},
        {"forall-effects", std::to_string(expander.effects_expanded())}
    });
}

} // namespace rantanplan
