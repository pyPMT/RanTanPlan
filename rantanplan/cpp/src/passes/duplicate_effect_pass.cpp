#include "duplicate_effect_pass.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"
#include "../util/stats.hpp"
#include "../problem/effect_expression.hpp"
#include "../smtcore/grounded_encoding_visitor.hpp"
#include "../smtcore/smt_types.hpp"
#include "../smtcore/smt_variable_factory.hpp"
#include "../smtcore/term_builder.hpp"

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
    // Two report streams, kept separate because the two cases mean different
    // things. A drop whose duplicated fluents are ALL BOOLEAN is an atom
    // asserted both true and false by one action: no sensible reading, no
    // encoder ever handled it, and nothing is lost by dropping it. A drop
    // involving a NUMERIC fluent may have had a real net effect, so it is a
    // genuine completeness risk (see "WHAT IT COSTS" in the header) and the two
    // counts should never be read as one number.
    //
    // Either way the per-action names are DEBUG-only. Reflexive groundings
    // produce these wholesale — a Zenotravel instance drops one fly-slow(p,c,c)
    // per plane x city — and at default verbosity the list buries the summary
    // that actually tells you something.
    std::vector<std::string> report;       // numeric  — counted at INFO, listed at -vv
    std::vector<std::string> bool_report;  // boolean  — counted at INFO, listed at -vv
    std::string bool_example;              // first label of each kind, shown in its
    std::string num_example;               // count line so the shape is recognisable
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
        int duplicated_bool = 0;
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
            if (result.problem.is_bool_type(fid)) duplicated_bool++;
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
        std::string line = actions[i].label() + " (" + std::to_string(duplicated) +
                           " fluent(s) written more than once, unconditionally)";
        // ALL duplicated fluents boolean -> the atom is asserted both true and
        // false. Undefined in PDDL, unsatisfiable on every encoder (booleans
        // never chain), nothing lost by dropping it. A MIXED action counts as
        // numeric: one of its duplicates can still have had a real net effect.
        if (duplicated_bool == duplicated) {
            if (bool_example.empty()) bool_example = actions[i].label();
            bool_report.push_back(std::move(line));
        } else {
            if (num_example.empty()) num_example = actions[i].label();
            report.push_back(std::move(line));
        }
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
            // Split out separately: the fixed cost of standing up a TermManager
            // and an encoding visitor dwarfs the queries themselves, so the
            // interesting number is how often we pay it at all, not how many
            // queries we run once we have.
            //
            // Declaration order is destruction order reversed, so the
            // TermManager is declared FIRST and therefore outlives every term
            // and solver built from it — same discipline as
            // SemanticInterferencePredicate's members.
            ScopedTimer setup_timer;  // measuring-only, recorded by hand below
            cvc5::TermManager tm;
            cvc5::Solver solver(tm);
            TermBuilder tb(tm);
            tb.set_problem(&result.problem);
            SmtVariableFactory factory(tm);
            factory.set_problem(&result.problem);
            GroundedEncodingVisitor visitor(tm, &result.problem, &factory);
            Stats::instance().set("duplicate_effect.assign_conflict_setup_ms",
                                  setup_timer.elapsed_ms());

            // Pseudo-timestep -1, the same convention the interference predicate
            // uses: guards are evaluated against the pre-state, so a guard is a
            // formula over the state before the action runs.
            auto guard_of = [&](const Effect* e) -> Term {
                return e->is_conditional()
                    ? visitor.convert_from_pool(e->effect_expression().condition_id(), -1)
                    : tb.bool_val(true);
            };

            for (const AssignCandidate& cand : assign_candidates) {
                const Action& action = actions[cand.action_idx];
                if (std::find(offending.begin(), offending.end(), cand.action_idx)
                        != offending.end()) {
                    continue;  // already dropped for the unconditional reason
                }
                Term pre = action.has_precondition()
                    ? visitor.convert_from_pool(action.precondition_id(), -1)
                    : tb.bool_val(true);

                bool conflict = false;
                for (size_t x = 0; x < cand.assigns.size() && !conflict; ++x) {
                    for (size_t y = x + 1; y < cand.assigns.size() && !conflict; ++y) {
                        // Identical assignments compose to themselves, so only a
                        // pair that actually differs can be ill-formed.
                        if (cand.assigns[x]->effect_expression() ==
                            cand.assigns[y]->effect_expression()) continue;
                        solver.push();
                        solver.assertFormula(tb.mk_and({pre,
                                                        guard_of(cand.assigns[x]),
                                                        guard_of(cand.assigns[y])}));
                        ++assign_queries;
                        conflict = solver.checkSat().isSat();
                        solver.pop();
                    }
                }
                if (!conflict) continue;

                ++assign_conflicts;
                offending.push_back(cand.action_idx);
                std::string line =
                    action.label() + " (two assignments to '" +
                    result.problem.pool().to_string(cand.fluent_id) +
                    "' can apply together)";
                // Same split as phase 1: a boolean fluent assigned true and
                // false under guards that can both hold is the conditional form
                // of the same degenerate shape, and costs nothing to drop.
                if (result.problem.is_bool_type(cand.fluent_id)) {
                    if (bool_example.empty()) bool_example = action.label();
                    bool_report.push_back(std::move(line));
                } else {
                    if (num_example.empty()) num_example = action.label();
                    report.push_back(std::move(line));
                }
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
    Stats::instance().set("duplicate_effect.actions_dropped_boolean",
                          static_cast<double>(bool_report.size()));
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
    // One counted line per kind at INFO, every name at DEBUG. The split is the
    // point: the boolean count is reassurance (those actions were unusable
    // anyway), the numeric count is the one to worry about.
    if (!bool_report.empty()) {
        Logger::instance().info(
            "  dropped: " + std::to_string(bool_report.size()) +
            " on BOOLEAN fluents only (one atom asserted both true and false, "
            "e.g. " + bool_example + ") — degenerate on every encoder, so "
            "nothing is lost.");
    }
    if (!report.empty()) {
        Logger::instance().info(
            "  dropped: " + std::to_string(report.size()) +
            " involving a NUMERIC fluent (e.g. " + num_example + ") — these had "
            "a real net effect under the chained encoders, so a plan may be "
            "lost.");
    }
    Logger::instance().info("  (-vv names every dropped action)");
    for (const std::string& line : bool_report)
        Logger::instance().debug("  dropped: " + line);
    for (const std::string& line : report)
        Logger::instance().debug("  dropped: " + line);

    result.problem = result.problem.without_actions(offending);
}

} // namespace rantanplan
