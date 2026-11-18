#include "write_only_fluent_pass.hpp"
#include "../problem/visitors/fluent_collector.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"
#include "../util/stats.hpp"
#include <algorithm>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rantanplan {

// ============================================================================
// Read analysis
// ============================================================================

/// The effects that write each written fluent.
///
/// Every fluent set in this pass holds canonical ids
/// (ExprPool::canonical_state_variable()): one ground atom can be interned
/// under two ids, and a read under one id must keep a write under the other.
using WritesByFluent = std::unordered_map<ExprID, std::vector<const EffectExpression*>>;

/// Every effect of every action, keyed by the fluent it writes.
static WritesByFluent collect_writes(const Problem& problem) {
    const ExprPool& pool = problem.pool();
    WritesByFluent writes;
    for (const Action& action : problem.actions()) {
        for (const Effect& effect : action.effects()) {
            // A conditional effect whose condition can never hold writes
            // nothing, but telling which ones those are is as hard as the
            // planning problem. Counting every effect as a write is safe: in
            // close_over_writes it can only add reads, never remove one.
            const EffectExpression& write = effect.effect_expression();
            writes[pool.canonical_state_variable(write.fluent_id())].push_back(&write);
        }
    }
    return writes;
}

/// The fluents read directly: those in action preconditions and explicit
/// action costs, goals and declared state invariants (FluentCollector walks
/// an expression; canonicalise what it returns). Also every written fluent
/// whose type declares bounds, since the bounds are checked in every state:
/// resolve its schema through its head symbol, as CWAInitialStatePass does,
/// and ask Fluent::has_declared_bounds().
static std::unordered_set<ExprID> collect_direct_reads(const Problem& problem,
                                                       const WritesByFluent& writes) {
    std::unordered_set<ExprID> read;
    // we need to consider all possible places where a fluent can be read
    FluentCollector collector(problem);

    // - goals
    for (const Goal& goal : problem.goals()) {
        collector.collect_from_id(goal.goal_id());
    }

    for (const Action& action : problem.actions()) {
        // - preconditions
        collector.collect_from_id(action.precondition_id());
        // - action costs
        if (action.cost_id().valid()) collector.collect_from_id(action.cost_id());
    }
    // Effect values and conditions are not direct reads: close_over_writes
    // adds them, for the writes to read fluents only.

    // - declared state invariants
    for (const ExprID& invariant : problem.state_invariants()) {
        collector.collect_from_id(invariant);
    }

    const ExprPool& pool = problem.pool();
    for (ExprID atom : collector.get_fluents()) read.insert(pool.canonical_state_variable(atom));

    // - fluents with declared bounds
    for (const auto& [atom, _] : writes) {
        // An effect target is a state variable headed by its FLUENT_SYMBOL
        // (ExprPool::intern_state_variable). If no schema has that name, keep
        // the atom: it might be bounded.
        const Fluent* fluent = problem.find_fluent(pool.payload_string(pool.head_symbol_id(atom)));
        if (!fluent || fluent->has_declared_bounds()) read.insert(atom);
    }

    return read;
}

/// Adds to @p read the fluents that read fluents depend on through their
/// writes. If an effect writes a read fluent, the fluents in its value are
/// read too, and so are those in its condition when the effect is
/// conditional, since the condition decides whether the write happens. Those
/// fluents can have writes of their own, so this repeats (a worklist) until
/// nothing is added. Writes to unread fluents are never followed. Ids are
/// canonical, as everywhere in this pass.
///
/// Example: the goal is (>= (total) 10) and the effects are
///   (increase (total) (rate))   (increase (rate) (base))   (increase (base) 1)
///   (increase (spent) (wear))   (increase (wear) 1)
/// The goal reads total; the write to total makes rate read; the write to
/// rate makes base read. Stopping after one round would remove base and leave
/// rate += base pointing at nothing, hence the loop. Nothing reads spent, so
/// its write is never followed, and wear, which only that write reads, is
/// write-only too. Treating every effect's value as read would also be sound,
/// but would keep wear (nlnp-sugar's (wear ?m) feeds the write-only
/// mill-cost the same way). Likewise, (when (raining) (increase (total) 1))
/// would make raining read.
static void close_over_writes(const Problem& problem, const WritesByFluent& writes,
                              std::unordered_set<ExprID>& read) {
    const ExprPool& pool = problem.pool();
    std::vector<ExprID> worklist(read.begin(), read.end());
    while (!worklist.empty()) {
        ExprID f = worklist.back();
        worklist.pop_back();
        auto it = writes.find(f);
        if (it == writes.end()) continue;  // read, but no action writes it

        // The fluents inside each write's value and condition, not the
        // expressions themselves.
        FluentCollector deps(problem);
        for (const EffectExpression* e : it->second) {
            deps.collect_from_id(e->value_id());
            if (e->is_conditional()) deps.collect_from_id(e->condition_id());
        }
        for (ExprID d : deps.get_fluents()) {
            ExprID atom = pool.canonical_state_variable(d);
            if (read.insert(atom).second) worklist.push_back(atom);  // new: follow its writes too
        }
    }
}

// ============================================================================
// Removal
// ============================================================================

/// @p problem without the effects on @p write_only fluents and without their
/// initial values; adds the number of effects removed to @p removed_effects.
/// Actions left with no effect stay: EffectNormalizationPass, which runs next,
/// drops them (the Grounded encoder would assert nothing for them, not even
/// their precondition).
static Problem without_write_only(const Problem& problem,
                                  const std::unordered_set<ExprID>& write_only,
                                  size_t& removed_effects) {
    const ExprPool& pool = problem.pool();
    auto removed = [&](ExprID id) { return write_only.count(pool.canonical_state_variable(id)) > 0; };

    std::vector<Action> actions = problem.actions();
    for (Action& action : actions) {
        std::vector<Effect>& effects = action.mutable_effects();
        const size_t before = effects.size();
        effects.erase(std::remove_if(effects.begin(), effects.end(), [&](const Effect& e) {
                          return removed(e.effect_expression().fluent_id());
                      }),
                      effects.end());
        removed_effects += before - effects.size();
    }

    std::vector<Assignment> initial_state;
    for (const Assignment& assignment : problem.initial_state()) {
        if (!removed(assignment.fluent_id())) initial_state.push_back(assignment);
    }

    // Also rebuilds the grounded fluents, which lose the write-only ones.
    return problem.with_simplified(std::move(actions), problem.goals(),
                                   std::move(initial_state), problem.state_invariants());
}

// ============================================================================
// Pass
// ============================================================================

void WriteOnlyFluentPass::apply(PreprocessedProblem& result) const {
    ScopedTimer timer;  // measuring-only; the pipeline records the stats key
    const Problem& problem = result.problem;

    const WritesByFluent writes = collect_writes(problem);
    std::unordered_set<ExprID> read = collect_direct_reads(problem, writes);
    close_over_writes(problem, writes, read);

    std::unordered_set<ExprID> write_only;
    for (const auto& [fluent, _] : writes) {
        if (!read.count(fluent)) write_only.insert(fluent);
    }
    if (Logger::instance().should_print(VerbosityLevel::DEBUG)) {
        std::vector<std::string> names;
        for (ExprID fluent : write_only) names.push_back(problem.pool().to_string(fluent));
        std::sort(names.begin(), names.end());
        for (const std::string& n : names) Logger::instance().debug("  write-only: " + n);
    }

    const size_t fluents_before = problem.grounded_fluent_count();
    size_t removed_effects = 0;
    if (!write_only.empty()) {
        // problem refers to result.problem: from here on it is the reduced
        // problem, and the pointers in writes point into the old one.
        result.problem = without_write_only(problem, write_only, removed_effects);
    }
    const size_t fluents_after = result.problem.grounded_fluent_count();

    auto& stats = Stats::instance();
    stats.set("write_only_fluent.fluents", static_cast<double>(write_only.size()));
    stats.set("write_only_fluent.removed_effects", static_cast<double>(removed_effects));
    Logger::instance().component(VerbosityLevel::INFO, name(), {
        {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
        {"written fluents", std::to_string(writes.size())},
        {"read", std::to_string(read.size())},
        {"write-only", std::to_string(write_only.size())},
        {"effects removed", std::to_string(removed_effects)},
        {"grounded fluents", std::to_string(fluents_before) + " -> " + std::to_string(fluents_after)}
    });
    // Nothing kept mentions a write-only fluent, so each one leaves the
    // grounded fluents. If fewer left, some kept expression still mentions
    // one, which means the read analysis missed a read.
    if (fluents_before - fluents_after != write_only.size()) {
        Logger::instance().info(
            "WARNING: write-only-fluent removed " + std::to_string(write_only.size()) +
            " fluent(s) but the grounded fluents went from " + std::to_string(fluents_before) +
            " to " + std::to_string(fluents_after) + "; a kept expression may still "
            "mention a removed fluent.");
    }
}

} // namespace rantanplan
