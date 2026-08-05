#include "duplicate_effect_pass.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"
#include "../util/stats.hpp"
#include "../problem/effect_expression.hpp"
#include "../z3core/grounded_encoding_visitor.hpp"
#include "../z3core/z3_variable_factory.hpp"

#include <z3++.h>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

namespace rantanplan {
namespace {

/// One (action, fluent) pair with two or more ASSIGN writes that are not all
/// identical — the only shape whose well-formedness needs an SMT query.
struct AssignCandidate {
    size_t action_idx;
    ExprID fluent_id;
    std::vector<const Effect*> assigns;  // in effects() order
};

} // namespace

void DuplicateEffectPass::apply(PreprocessedProblem& result) const {
    const std::vector<Action>& actions = result.problem.actions();

    std::vector<size_t> offending;
    std::vector<std::string> report;
    std::vector<AssignCandidate> assign_candidates;
    size_t kept_conditional = 0;

    for (size_t i = 0; i < actions.size(); ++i) {
        // Count writes per fluent, and how many of them are unconditional.
        std::unordered_map<ExprID, int> writes;
        std::unordered_map<ExprID, int> unconditional;
        for (const Effect& effect : actions[i].effects()) {
            ExprID fid = effect.effect_expression().fluent_id();
            writes[fid]++;
            if (!effect.is_conditional()) unconditional[fid]++;
        }

        // ONLY fluents with TWO OR MORE UNCONDITIONAL writes.  Not "every write
        // unconditional": an action with (increase f 1), (decrease f 1) and a
        // third conditional write on f is exactly as broken as drink(lamb,lamb) —
        // the Grounded encoders still conjoin all three equalities into something
        // unsatisfiable, and Chained still applies only the first — but an
        // every-write-unconditional test spares it.  Requiring >= 2 unconditional
        // writes catches it and still spares both legitimate idioms below, which
        // have 1 and 0 unconditional writes respectively.
        //
        // That distinction is empirical, and it is what separates the degenerate
        // shapes from the legitimate idioms.  Measured over pddl/test:
        //
        //   ALL-UNCONDITIONAL (degenerate — no sensible reading, and no encoder
        //   handles them):
        //     drink(x,x)      two additive updates that cancel  (reflexive
        //                     grounding of drink(?from,?to))
        //     feast(v,x,x)    (not (craves v x)) AND (craves v x) — a boolean
        //                     add and delete of one atom, undefined in PDDL
        //
        //   INVOLVING A CONDITIONAL (legitimate, and dropping them lost plans on
        //   cave-diving and petrobras — they must survive):
        //     hire-diver      an unconditional (not (available ?d1)) plus the
        //                     ?d2 = ?d1 case of a forall-when that also deletes
        //                     it.  Both writes AGREE; the action is well defined
        //                     and merely redundantly represented.
        //     refuel_at_port  two conditional effects on current_fuel with
        //                     MUTUALLY EXCLUSIVE guards (<= capacity -> increase,
        //                     > capacity -> clip).  The standard clip-to-capacity
        //                     idiom; exactly one arm ever applies.
        int duplicated = 0;
        for (const auto& [fid, n] : writes) {
            (void)n;
            if (unconditional[fid] < 2) continue;
            // Repeated IDENTICAL assignments are the one duplicate shape with a
            // canonical composition — itself.  `(and (at ?x) (at ?x))` after
            // grounding is ordinary redundancy, the Grounded encoders already
            // encode it correctly (f' = v AND f' = v is satisfiable), so dropping
            // it would be a pure completeness loss with nothing to offset it.
            // Additive writes are NOT idempotent: two identical (increase f 1)
            // encode as f' = f+1 twice, which yields +1 where PDDL says +2, so
            // they stay in scope.
            bool all_identical_assign = true;
            const EffectExpression* first = nullptr;
            for (const Effect& effect : actions[i].effects()) {
                const EffectExpression& e = effect.effect_expression();
                if (effect.is_conditional() || e.fluent_id() != fid) continue;
                if (e.kind() != EffectExpression::Kind::ASSIGN) {
                    all_identical_assign = false; break;
                }
                if (!first) first = &e;
                else if (!(*first == e)) { all_identical_assign = false; break; }
            }
            if (all_identical_assign) continue;
            duplicated++;
        }
        if (duplicated == 0) {
            for (const auto& [fid, n] : writes)
                if (n > 1) { kept_conditional++; break; }

            // Syntactic screen for the ill-formed ASSIGN case, cheap and always
            // run; the SMT query below only sees what survives it. Two writes to
            // one fluent are fine when they are additive (they compose to a sum)
            // or when they are the same assignment twice (idempotent). Two
            // DIFFERENT assignments that can both apply are ill-formed — PDDL 2.1
            // forbids two assignments to one primitive numeric expression, and
            // there is no canonical composition of two overwrites. Whether they
            // CAN both apply is a satisfiability question, deferred to phase 2.
            for (const auto& [fid, n] : writes) {
                if (n < 2) continue;
                std::vector<const Effect*> assigns;
                const EffectExpression* first = nullptr;
                bool all_identical = true;
                for (const Effect& effect : actions[i].effects()) {
                    const EffectExpression& e = effect.effect_expression();
                    if (e.fluent_id() != fid) continue;
                    if (e.kind() != EffectExpression::Kind::ASSIGN) continue;
                    assigns.push_back(&effect);
                    if (!first) first = &e;
                    else if (!(*first == e)) all_identical = false;
                }
                if (assigns.size() < 2 || all_identical) continue;
                assign_candidates.push_back({i, fid, std::move(assigns)});
            }
            continue;
        }

        offending.push_back(i);
        // label(), not name(): name() is the LIFTED name, so every grounding of
        // drink would log identically and the ?from = ?to collapse this pass
        // exists to catch would be invisible.
        report.push_back(actions[i].label() + " (" + std::to_string(duplicated) +
                         " fluent(s) written more than once, unconditionally)");
    }

    // ---------------------------------------------------------------------
    // Phase 2: are the surviving ASSIGN duplicates actually able to fire
    // together? Only reachable for actions the syntactic screen flagged, so on
    // a problem without that shape this costs one comparison against zero and
    // no Z3 context is ever built. See duplicate_effect.assign_* in the stats.
    //
    // This moved here from SemanticInterferencePredicate, which used to run the
    // same pre ∧ ci ∧ cj query per action pair and, on a hit, report a1 as
    // interfering with EVERY action in the problem — a unary property returned
    // as a verdict about a pair. The predicate now composes such effects
    // correctly and no longer needs the screen; what remains is the genuine
    // ill-formedness, which is a property of one action and belongs in a
    // preprocessing diagnostic, computed once per action rather than per pair.
    // ---------------------------------------------------------------------
    size_t assign_queries = 0;
    size_t assign_conflicts = 0;
    {
        ScopedTimer assign_timer("duplicate_effect.assign_conflict_ms");
        if (!assign_candidates.empty()) {
            // Split out separately: the fixed cost of standing up a Z3 context
            // and an encoding visitor dwarfs the queries themselves, so the
            // interesting number is how often we pay it at all, not how many
            // queries we run once we have.
            ScopedTimer setup_timer;  // measuring-only, recorded by hand below
            z3::context ctx;
            Z3VariableFactory factory(ctx);
            factory.set_problem(&result.problem);
            GroundedEncodingVisitor visitor(ctx, &result.problem, &factory);
            z3::solver solver(ctx);
            Stats::instance().set("duplicate_effect.assign_conflict_setup_ms",
                                  setup_timer.elapsed_ms());

            // Pseudo-timestep -1, the same convention the interference predicate
            // uses: guards are evaluated against the pre-state, so a guard is a
            // formula over the state before the action runs.
            auto guard_of = [&](const Effect* e) {
                return e->is_conditional()
                    ? visitor.convert_from_pool(e->effect_expression().condition_id(), -1)
                    : ctx.bool_val(true);
            };

            for (const AssignCandidate& cand : assign_candidates) {
                const Action& action = actions[cand.action_idx];
                if (std::find(offending.begin(), offending.end(), cand.action_idx)
                        != offending.end()) {
                    continue;  // already dropped for the unconditional reason
                }
                z3::expr pre = action.has_precondition()
                    ? visitor.convert_from_pool(action.precondition_id(), -1)
                    : ctx.bool_val(true);

                bool conflict = false;
                for (size_t x = 0; x < cand.assigns.size() && !conflict; ++x) {
                    for (size_t y = x + 1; y < cand.assigns.size() && !conflict; ++y) {
                        // Identical assignments compose to themselves, so only a
                        // pair that actually differs can be ill-formed.
                        if (cand.assigns[x]->effect_expression() ==
                            cand.assigns[y]->effect_expression()) continue;
                        solver.push();
                        solver.add(pre && guard_of(cand.assigns[x]) &&
                                   guard_of(cand.assigns[y]));
                        ++assign_queries;
                        conflict = (solver.check() == z3::sat);
                        solver.pop();
                    }
                }
                if (!conflict) continue;

                ++assign_conflicts;
                offending.push_back(cand.action_idx);
                report.push_back(
                    action.label() + " (two assignments to '" +
                    result.problem.pool().to_string(cand.fluent_id) +
                    "' can apply together)");
            }
        }
    }

    Stats::instance().set("duplicate_effect.assign_conflict_candidates",
                          static_cast<double>(assign_candidates.size()));
    Stats::instance().set("duplicate_effect.assign_conflict_queries",
                          static_cast<double>(assign_queries));
    Stats::instance().set("duplicate_effect.actions_dropped_assign_conflict",
                          static_cast<double>(assign_conflicts));
    Stats::instance().set("duplicate_effect.actions_dropped",
                          static_cast<double>(offending.size()));
    // The number that matters for the invariant is the one NOT dropped: an action
    // with a duplicated fluent that survives because a conditional write is
    // involved still poisons are_simply_commuting, which is conditionality-blind.
    Stats::instance().set("duplicate_effect.actions_kept_conditional",
                          static_cast<double>(kept_conditional));
    if (offending.empty()) return;

    // Loud on purpose: this is a lossy interim measure, not a normalisation.
    // See the header for what it costs and what it should become.
    // Logger has no warning level; info() is visible at the default verbosity and
    // silent under the test suite, which is the behaviour we want.
    Logger::instance().info(
        "WARNING: dropping " + std::to_string(offending.size()) + " of " +
        std::to_string(actions.size()) + " ground actions that write one fluent "
        "more than once in a way nothing downstream can compose. " +
        std::to_string(offending.size() - assign_conflicts) + " have two or more "
        "UNCONDITIONAL writes to one fluent, which the Grounded encoders turn "
        "into a conjunction of equalities that is unsatisfiable, so the action "
        "is already unselectable there. " + std::to_string(assign_conflicts) +
        " have two ASSIGNMENTS to one fluent whose guards can hold together, "
        "which PDDL 2.1 forbids — two assignments to one primitive numeric "
        "expression have no canonical composition, and the chained encoders "
        "would silently pick the last one. Dropping is sound but may lose plans "
        "where the writes would have cancelled.");
    for (const std::string& line : report)
        Logger::instance().info("  dropped: " + line);

    result.problem = result.problem.without_actions(offending);
}

} // namespace rantanplan
