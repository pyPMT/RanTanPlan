#include "effect_normalization_pass.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"
#include "../util/stats.hpp"
#include "../problem/effect_expression.hpp"
#include "../problem/expr_builder.hpp"
#include "../z3core/grounded_encoding_visitor.hpp"
#include "../z3core/z3_variable_factory.hpp"

#include <z3++.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rantanplan {
namespace {

/// Above this many distinct guards on the increases and decreases of one
/// fluent (more than 2^4 = 16 split writes), case 4 warns before splitting.
constexpr size_t kSplitWarningGuards = 4;

/// A guard, and the increases and decreases of one fluent under it.
using GuardedWrites = std::pair<ExprID, std::vector<const EffectExpression*>>;

/// Two assignments of the same value (case 5).
bool same_assignment(const EffectExpression& a, const EffectExpression& b) {
    return a.kind() == EffectExpression::Kind::ASSIGN &&
           b.kind() == EffectExpression::Kind::ASSIGN &&
           a.fluent_id() == b.fluent_id() && a.value_id() == b.value_id();
}

/// The value of a numeric constant, or nothing.
std::optional<double> numeric_constant(const ExprPool& pool, ExprID id) {
    if (!id.valid() || !pool.is_constant(id)) return std::nullopt;
    if (pool.payload_is_int(id)) return static_cast<double>(pool.payload_int(id));
    if (pool.payload_is_double(id)) return pool.payload_double(id);
    return std::nullopt;
}

/// The conjuncts of `parts`, with nested conjunctions flattened and the
/// constant true left out; nothing if they plainly cannot all hold: one is the
/// constant false, or one is the negation of another, as in (= load 0) and
/// (not (= load 0)).
std::optional<std::vector<ExprID>> conjuncts(const ExprPool& pool,
                                             const std::vector<ExprID>& parts) {
    std::vector<ExprID> open;
    std::vector<ExprID> stack(parts.rbegin(), parts.rend());
    while (!stack.empty()) {
        ExprID p = stack.back();
        stack.pop_back();
        if (pool.is_false_constant(p)) return std::nullopt;
        if (pool.is_true_constant(p)) continue;
        if (pool.is_function_application(p) && pool.is_and(p)) {
            auto args = pool.arguments(p);
            stack.insert(stack.end(), args.rbegin(), args.rend());
            continue;
        }
        open.push_back(p);
    }
    for (ExprID p : open) {
        if (pool.is_function_application(p) && pool.is_not(p) &&
            pool.argument_count(p) == 1 &&
            std::find(open.begin(), open.end(), pool.argument(p, 0)) != open.end())
            return std::nullopt;
    }
    return open;
}

/// `effects` with the writes of each fluent in `replacement` replaced by its
/// new writes, which take the place of the fluent's first write.
std::vector<Effect> replace_writes(
        const std::vector<Effect>& effects,
        const std::unordered_map<ExprID, std::vector<Effect>>& replacement) {
    std::vector<Effect> rebuilt;
    std::unordered_map<ExprID, bool> placed;
    for (const Effect& e : effects) {
        ExprID fid = e.effect_expression().fluent_id();
        auto it = replacement.find(fid);
        if (it == replacement.end()) { rebuilt.push_back(e); continue; }
        if (placed[fid]) continue;
        placed[fid] = true;
        rebuilt.insert(rebuilt.end(), it->second.begin(), it->second.end());
    }
    return rebuilt;
}

/// Rewrites the actions of one problem, one at a time, as the class comment
/// of EffectNormalizationPass describes (cases 1 to 7), and counts what it
/// does. apply() builds one per run: a pass holds no state of its own.
class EffectNormalizer {
public:
    explicit EffectNormalizer(const Problem& problem)
        : problem_(problem), pool_(*problem.pool_ptr()), builder_(pool_) {}

    /// Rewrites `action` in place. Returns false if the action is to be
    /// removed: it can never apply (after case 6 or 7), or it has no effect.
    bool normalize(Action& action) {
        const std::vector<Effect> original = action.effects();
        const ExprID original_pre = precondition_of(action);
        ExprID pre = original_pre;
        std::string conflict_fluent;  // the first fluent of case 6 or 7

        // The writes of each fluent, fluents in order of first write.
        std::vector<ExprID> fluents;
        std::unordered_map<ExprID, std::vector<const Effect*>> writes;
        for (const Effect& e : original) {
            ExprID fid = e.effect_expression().fluent_id();
            if (writes[fid].empty()) fluents.push_back(fid);
            writes[fid].push_back(&e);
        }

        // Cases 2 to 7, for each fluent written more than once.
        std::unordered_map<ExprID, std::vector<Effect>> replacement;
        for (ExprID fid : fluents) {
            const std::vector<const Effect*>& fluent_writes = writes[fid];
            if (fluent_writes.size() < 2) continue;
            // Grounding leaves no quantified write; never rewrite one.
            if (std::any_of(fluent_writes.begin(), fluent_writes.end(),
                            [](const Effect* w) { return w->is_quantified(); }))
                continue;

            if (problem_.is_bool_type(fid)) {
                if (auto merged = merge_computed_booleans(fid, fluent_writes))
                    replacement[fid] = {*merged};
                continue;
            }
            ExprID restricted = restrict_conflicts(fluent_writes, pre);
            if (restricted != pre && conflict_fluent.empty())
                conflict_fluent = pool_.to_string(fid);
            pre = restricted;
            if (auto merged = merge_additive(action, fid, fluent_writes, pre))
                replacement[fid] = std::move(*merged);
        }
        if (!replacement.empty()) {
            action.mutable_effects() = replace_writes(original, replacement);
            changed_ = true;
        }
        if (pre != original_pre) {
            action.set_precondition_id(pre);
            changed_ = true;
        }
        add_after_delete(action);  // case 1

        if (!conflict_fluent.empty()) {
            conflicts_.push_back(action.label() + " on " + conflict_fluent);
            if (!may_hold({pre})) {
                ++removed_never_applicable_;
                changed_ = true;
                return false;
            }
        }
        // An action that changes nothing is never needed by a plan. (The
        // Grounded encoders would assert nothing for it, not even its
        // precondition.)
        if (action.effects().empty()) {
            ++removed_no_effect_;
            changed_ = true;
            return false;
        }
        return true;
    }

    /// Whether any action was rewritten or is to be removed.
    bool changed() const { return changed_; }

    /// Records the statistics and warns about the write conflicts of cases 6
    /// and 7.
    void report() const {
        auto& stats = Stats::instance();
        stats.set("effect_normalization.boolean_add_after_delete",
                  static_cast<double>(add_after_delete_));
        stats.set("effect_normalization.boolean_computed_merged",
                  static_cast<double>(computed_merged_));
        stats.set("effect_normalization.additive_merged", static_cast<double>(additive_merged_));
        stats.set("effect_normalization.additive_split", static_cast<double>(additive_split_));
        stats.set("effect_normalization.additive_split_max_writes",
                  static_cast<double>(max_split_writes_));
        stats.set("effect_normalization.assign_conflicts_restricted",
                  static_cast<double>(conflicts_.size()));
        stats.set("effect_normalization.actions_removed_never_applicable",
                  static_cast<double>(removed_never_applicable_));
        stats.set("effect_normalization.actions_removed_no_effect",
                  static_cast<double>(removed_no_effect_));
        stats.set("effect_normalization.overlap_queries", static_cast<double>(queries_));
        stats.set("effect_normalization.overlap_ms", query_ms_);
        stats.set("effect_normalization.overlap_setup_ms", setup_ms_);

        if (conflicts_.empty()) return;
        // Loud on purpose: the model has a write conflict that Unified Planning
        // and PDDL 2.1 treat as an error. The planner handles it exactly as
        // Unified Planning does, so no plan is lost.
        // Logger has no warning level; info() is visible at the default
        // verbosity and silent under the test suite.
        Logger::instance().info(
            "WARNING: " + std::to_string(conflicts_.size()) + " ground action(s) "
            "write one fluent twice in a way Unified Planning and PDDL 2.1 treat "
            "as an error where both writes apply: two different assignments, or "
            "an assignment and an increase or decrease (e.g. " + conflicts_[0] +
            "). Each is now inapplicable in the states where both writes apply, "
            "which is where Unified Planning would reject it; " +
            std::to_string(removed_never_applicable_) +
            " of them can never apply and were removed. (-vv names every one)");
        for (const std::string& line : conflicts_)
            Logger::instance().debug("  write conflict: " + line);
    }

private:
    // ── The cases ───────────────────────────────────────────────────────

    /// Case 1: a Boolean set and cleared, with the constants true and false.
    /// Where both apply the atom ends up true (add after delete), so each
    /// clear also requires that no set applies:
    ///     when small: medium := true;  when medium: medium := false
    ///     -> the clear becomes  when (medium and not small): medium := false
    /// A clear is removed when a set is unconditional. Runs after
    /// merge_computed_booleans, which leaves one write for any atom that has a
    /// computed value.
    void add_after_delete(Action& action) {
        std::unordered_map<ExprID, std::vector<size_t>> sets, clears;
        const std::vector<Effect>& effects = action.effects();
        for (size_t k = 0; k < effects.size(); ++k) {
            const EffectExpression& e = effects[k].effect_expression();
            if (e.kind() != EffectExpression::Kind::ASSIGN || e.is_quantified()) continue;
            if (pool_.is_true_constant(e.value_id())) sets[e.fluent_id()].push_back(k);
            else if (pool_.is_false_constant(e.value_id())) clears[e.fluent_id()].push_back(k);
        }

        size_t rewritten = 0;
        std::vector<bool> removed(effects.size(), false);
        std::vector<Effect> rebuilt = effects;
        for (const auto& [fid, clear_idx] : clears) {
            auto it = sets.find(fid);
            if (it == sets.end()) continue;
            std::vector<ExprID> set_guards;
            for (size_t k : it->second)
                set_guards.push_back(guard_of(effects[k].effect_expression()));
            ExprID no_set_applies = builder_.make_not(builder_.make_or(set_guards));
            for (size_t k : clear_idx) {
                const EffectExpression& e = effects[k].effect_expression();
                ExprID guard = builder_.make_and({guard_of(e), no_set_applies});
                ++rewritten;
                if (pool_.is_false_constant(guard)) { removed[k] = true; continue; }
                rebuilt[k].set_effect_expression(
                    EffectExpression(e.kind(), fid, e.value_id(), guard, &pool_));
            }
        }
        if (rewritten == 0) return;

        std::vector<Effect>& target = action.mutable_effects();
        target.clear();
        for (size_t k = 0; k < rebuilt.size(); ++k)
            if (!removed[k]) target.push_back(std::move(rebuilt[k]));
        add_after_delete_ += rewritten;
        changed_ = true;
    }

    /// Case 2: a Boolean with at least one write of a computed value. The atom
    /// ends up true if any applying write gives true, so the writes become one:
    ///     alarm := smoke;  alarm := heat   ->  alarm := smoke or heat
    /// In general  when (g1 or ... or gn): p := (g1 and v1) or ... or (gn and vn).
    /// Nothing if every value is a constant: that is case 1.
    std::optional<Effect> merge_computed_booleans(ExprID fid,
                                                  const std::vector<const Effect*>& writes) {
        bool computed = false;
        for (const Effect* w : writes) {
            ExprID v = w->effect_expression().value_id();
            computed |= !pool_.is_true_constant(v) && !pool_.is_false_constant(v);
        }
        if (!computed) return std::nullopt;

        std::vector<ExprID> guards, values;
        for (const Effect* w : writes) {
            const EffectExpression& e = w->effect_expression();
            guards.push_back(guard_of(e));
            values.push_back(builder_.make_and({guard_of(e), e.value_id()}));
        }
        ++computed_merged_;
        return Effect(EffectExpression(EffectExpression::Kind::ASSIGN, fid,
                                       builder_.make_or(values), builder_.make_or(guards),
                                       &pool_));
    }

    /// Cases 6 and 7: two different assignments, or an assignment and an
    /// increase or decrease, that can apply together. Unified Planning cannot
    /// execute the action where both apply, so the precondition excludes
    /// exactly those states:
    ///     when hot: temp := 50;  when humid: temp := 40
    ///     -> precondition and not (hot and humid)
    /// For assignments of computed values the excluded states also need
    /// v1 != v2. Two assignments of the same value are case 5 and left alone.
    /// Returns the new precondition, `pre` itself if nothing conflicts.
    ExprID restrict_conflicts(const std::vector<const Effect*>& writes, ExprID pre) {
        std::vector<const EffectExpression*> assigns, additive;
        for (const Effect* w : writes) {
            const EffectExpression& e = w->effect_expression();
            (e.is_assign() ? assigns : additive).push_back(&e);
        }

        std::vector<ExprID> conflicts;
        for (size_t x = 0; x < assigns.size(); ++x) {
            const EffectExpression& e1 = *assigns[x];
            for (size_t y = x + 1; y < assigns.size(); ++y) {
                const EffectExpression& e2 = *assigns[y];
                if (same_assignment(e1, e2)) continue;
                auto c1 = numeric_constant(pool_, e1.value_id());
                auto c2 = numeric_constant(pool_, e2.value_id());
                if (c1 && c2 && *c1 == *c2) continue;  // same value, spelt differently
                ExprID differ = (c1 && c2)
                    ? builder_.true_id()
                    : builder_.make_not(builder_.make_eq(e1.value_id(), e2.value_id()));
                conflicts.push_back(builder_.make_and({guard_of(e1), guard_of(e2), differ}));
            }
            for (const EffectExpression* a : additive)
                conflicts.push_back(builder_.make_and({guard_of(e1), guard_of(*a)}));
        }

        for (ExprID c : conflicts)
            if (may_hold({pre, c})) pre = builder_.make_and({pre, builder_.make_not(c)});
        return pre;
    }

    /// Cases 3 and 4: two or more increases and decreases of `fid`. They add
    /// up, so they become writes of which at most one applies:
    ///     x += 1;  x -= 1                     ->  nothing (case 3: net 0)
    ///     when f: g += 1;  when f: g += 2     ->  when f: g += 3
    ///     when (= load 0): fuel -= 1;  when (not (= load 0)): fuel -= 2
    ///                                         ->  unchanged (exclusive guards)
    ///     when w: y += 2;  when f: y += 3     ->  split (case 4)
    /// Returns the fluent's new writes, or nothing if they do not change. Its
    /// assignments come first, unchanged: restrict_conflicts has made them
    /// exclusive with every increase and decrease.
    std::optional<std::vector<Effect>> merge_additive(const Action& action, ExprID fid,
                                                      const std::vector<const Effect*>& writes,
                                                      ExprID pre) {
        std::vector<Effect> rewritten;  // the assignments, then the new writes
        std::vector<const EffectExpression*> unconditional;
        std::vector<GuardedWrites> guarded;  // one entry per distinct guard
        size_t additive_count = 0;
        for (const Effect* w : writes) {
            const EffectExpression& e = w->effect_expression();
            if (e.is_assign()) { rewritten.push_back(*w); continue; }
            ++additive_count;
            if (!e.is_conditional()) { unconditional.push_back(&e); continue; }
            auto it = std::find_if(guarded.begin(), guarded.end(),
                [&](const GuardedWrites& g) { return g.first == e.condition_id(); });
            if (it == guarded.end()) guarded.push_back({e.condition_id(), {&e}});
            else it->second.push_back(&e);
        }
        if (additive_count < 2) return std::nullopt;

        auto add = [&](const std::vector<const EffectExpression*>& group, ExprID guard) {
            if (auto w = additive_write(fid, group, guard)) rewritten.push_back(*w);
        };
        const size_t k = guarded.size();
        if (k == 0 || (k == 1 && unconditional.empty())) {  // one guard, or none
            if (k == 0) add(unconditional, builder_.true_id());
            else add(guarded[0].second, guarded[0].first);
            ++additive_merged_;
            return rewritten;
        }

        std::vector<std::vector<bool>> overlap(k, std::vector<bool>(k, true));
        bool any_overlap = !unconditional.empty();
        for (size_t i = 0; i < k; ++i)
            for (size_t j = i + 1; j < k; ++j) {
                overlap[i][j] = overlap[j][i] =
                    may_hold({pre, guarded[i].first, guarded[j].first});
                any_overlap |= overlap[i][j];
            }
        if (!any_overlap) {  // exclusive guards: one write per guard
            if (k == additive_count) return std::nullopt;  // already so
            for (const auto& [guard, group] : guarded) add(group, guard);
            ++additive_merged_;
            return rewritten;
        }
        std::vector<Effect> combinations = split(action, fid, unconditional, guarded, overlap);
        ++additive_split_;
        max_split_writes_ = std::max(max_split_writes_, combinations.size());
        rewritten.insert(rewritten.end(), combinations.begin(), combinations.end());
        return rewritten;
    }

    /// Case 4: one write per combination of the guards that can hold together,
    /// adding the unconditional writes and those of the guards that hold:
    ///     when w: y += 2;  when f: y += 3
    ///     -> when (w and not f): y += 2;  when (not w and f): y += 3;
    ///        when (w and f): y += 5
    /// `overlap[i][j]` says whether guards i and j can hold together. A guard
    /// that cannot hold with one that holds is false there anyway, so it is
    /// left out of the combination's guard, and so are the combinations with
    /// two such guards.
    std::vector<Effect> split(const Action& action, ExprID fid,
                              const std::vector<const EffectExpression*>& unconditional,
                              const std::vector<GuardedWrites>& guarded,
                              const std::vector<std::vector<bool>>& overlap) {
        const size_t k = guarded.size();
        if (k > kSplitWarningGuards) {
            // Before the split, so the reason is on screen if the planner then
            // runs out of time or memory.
            Logger::instance().info(
                "WARNING: " + action.label() + " increases or decreases " +
                pool_.to_string(fid) + " under " + std::to_string(k) +
                " different conditions that can hold together. The planner "
                "writes this as one effect per combination of them, up to " +
                (k < 63 ? std::to_string(1ULL << k) : std::string("2^") + std::to_string(k)) +
                " effects, which can take a lot of time and memory. Keeping the "
                "total up to date as it changes, in the domain, avoids this.");
        }

        std::vector<Effect> writes;
        std::vector<size_t> chosen;  // the guards that hold in this combination
        std::function<void(size_t)> visit = [&](size_t i) {
            if (i == k) {
                if (chosen.empty() && unconditional.empty()) return;  // no write applies
                std::vector<ExprID> parts;
                std::vector<const EffectExpression*> total = unconditional;
                for (size_t c : chosen) {
                    parts.push_back(guarded[c].first);
                    total.insert(total.end(), guarded[c].second.begin(),
                                 guarded[c].second.end());
                }
                for (size_t j = 0; j < k; ++j) {
                    if (std::find(chosen.begin(), chosen.end(), j) != chosen.end()) continue;
                    bool could_hold = std::all_of(chosen.begin(), chosen.end(),
                        [&](size_t c) { return overlap[j][c]; });
                    if (could_hold) parts.push_back(builder_.make_not(guarded[j].first));
                }
                if (auto w = additive_write(fid, total, builder_.make_and(parts)))
                    writes.push_back(*w);
                return;
            }
            visit(i + 1);  // guard i does not hold
            bool fits = std::all_of(chosen.begin(), chosen.end(),
                [&](size_t c) { return overlap[i][c]; });
            if (fits) {  // guard i holds
                chosen.push_back(i);
                visit(i + 1);
                chosen.pop_back();
            }
        };
        visit(0);
        return writes;
    }

    /// The write "fid changes by the sum of `writes`" (increases added,
    /// decreases subtracted) under `guard`, with the constants folded; nothing
    /// if the change is 0. Sums are nested binary nodes, the form the interval
    /// evaluators read.
    std::optional<Effect> additive_write(ExprID fid,
                                         const std::vector<const EffectExpression*>& writes,
                                         ExprID guard) {
        const int type_id = pool_.type_id(fid);
        double constant = 0.0;
        std::vector<ExprID> added, subtracted;
        for (const EffectExpression* e : writes) {
            const bool increase = e->kind() == EffectExpression::Kind::INCREASE;
            if (auto c = numeric_constant(pool_, e->value_id())) constant += increase ? *c : -*c;
            else (increase ? added : subtracted).push_back(e->value_id());
        }
        if (constant > 0) added.push_back(pool_.intern_double_constant(constant, type_id));
        if (constant < 0) subtracted.push_back(pool_.intern_double_constant(-constant, type_id));
        if (added.empty() && subtracted.empty()) return std::nullopt;

        auto sum = [&](const std::vector<ExprID>& terms) {
            ExprID s = terms[0];
            for (size_t i = 1; i < terms.size(); ++i)
                s = builder_.make_app(ExprOperator::PLUS, {s, terms[i]}, type_id);
            return s;
        };
        EffectExpression::Kind kind = EffectExpression::Kind::INCREASE;
        ExprID value;
        if (subtracted.empty()) {
            value = sum(added);
        } else if (added.empty()) {
            kind = EffectExpression::Kind::DECREASE;
            value = sum(subtracted);
        } else {
            value = builder_.make_app(ExprOperator::MINUS, {sum(added), sum(subtracted)}, type_id);
        }
        return Effect(EffectExpression(kind, fid, value, guard, &pool_));
    }

    // ── The overlap check ───────────────────────────────────────────────

    /// Can all of `parts` hold in one state? Asked over a single state, with
    /// no actions and no reachability. A no is exact. A yes may be
    /// pessimistic, and so are an unknown answer and a formula the solver
    /// cannot take, which count as yes; that is safe, because every rewrite above is
    /// exact in every state, so a needless yes only costs a needless rewrite.
    bool may_hold(const std::vector<ExprID>& parts) {
        std::optional<std::vector<ExprID>> open = conjuncts(pool_, parts);
        if (!open) return false;
        if (open->empty()) return true;
        return solver_may_hold(*open);
    }

    /// The Z3 part of may_hold, at pseudo-timestep -1 as in the
    /// interference predicate.
    bool solver_may_hold(const std::vector<ExprID>& formulas) {
        ScopedTimer timer;  // measuring-only; summed into query_ms_
        if (!solver_) start_solver();
        ++queries_;
        z3::expr_vector terms(*ctx_);
        try {
            for (ExprID f : formulas) terms.push_back(visitor_->convert_from_pool(f, -1));
        } catch (const std::exception&) {
            query_ms_ += timer.elapsed_ms();
            return true;
        }
        bool unsat = false;
        solver_->push();
        try {
            solver_->add(z3::mk_and(terms));
            unsat = (solver_->check() == z3::unsat);
        } catch (const std::exception&) {
            unsat = false;
        }
        solver_->pop();
        query_ms_ += timer.elapsed_ms();
        return !unsat;
    }

    /// Built on the first query, so a problem that needs none builds nothing.
    void start_solver() {
        ScopedTimer setup;  // measuring-only
        ctx_ = std::make_unique<z3::context>();
        factory_ = std::make_unique<Z3VariableFactory>(*ctx_);
        factory_->set_problem(&problem_);
        visitor_ = std::make_unique<GroundedEncodingVisitor>(*ctx_, &problem_, factory_.get());
        solver_ = std::make_unique<z3::solver>(*ctx_);
        setup_ms_ = setup.elapsed_ms();
    }

    // ── Helpers and state ───────────────────────────────────────────────

    ExprID guard_of(const EffectExpression& e) {
        return e.is_conditional() ? e.condition_id() : builder_.true_id();
    }

    ExprID precondition_of(const Action& action) {
        return action.has_precondition() ? action.precondition_id() : builder_.true_id();
    }

    const Problem& problem_;
    ExprPool& pool_;
    ExprBuilder builder_;

    // Declaration order is destruction order reversed: the context is
    // declared first, so it outlives every term and solver built from it.
    std::unique_ptr<z3::context> ctx_;
    std::unique_ptr<Z3VariableFactory> factory_;
    std::unique_ptr<GroundedEncodingVisitor> visitor_;
    std::unique_ptr<z3::solver> solver_;

    bool changed_ = false;
    size_t add_after_delete_ = 0;       ///< case 1, clears rewritten or removed
    size_t computed_merged_ = 0;        ///< case 2, fluents
    size_t additive_merged_ = 0;        ///< cases 3 and 4 without a split, fluents
    size_t additive_split_ = 0;         ///< case 4, fluents
    size_t max_split_writes_ = 0;       ///< case 4, most writes from one split
    std::vector<std::string> conflicts_;  ///< cases 6 and 7, "action on fluent"
    size_t removed_never_applicable_ = 0;
    size_t removed_no_effect_ = 0;
    size_t queries_ = 0;                ///< overlap check, solver queries
    double query_ms_ = 0.0;
    double setup_ms_ = 0.0;
};

} // namespace

void EffectNormalizationPass::apply(PreprocessedProblem& result) const {
    EffectNormalizer normalizer(result.problem);
    std::vector<Action> kept;
    for (Action action : result.problem.actions())
        if (normalizer.normalize(action)) kept.push_back(std::move(action));
    normalizer.report();
    if (normalizer.changed()) result.problem = result.problem.with_actions(std::move(kept));
}

} // namespace rantanplan
