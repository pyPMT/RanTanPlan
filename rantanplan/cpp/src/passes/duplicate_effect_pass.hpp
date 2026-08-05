#pragma once

#include "pass.hpp"

namespace rantanplan {

/**
 * @brief Drops ground actions that write the same fluent more than once.
 *
 * INTERIM MEASURE, not the intended end state — see "What this should become".
 *
 * Two categories, and only the second needs a solver:
 *
 *   (1) two or more UNCONDITIONAL writes to one fluent — purely syntactic.
 *   (2) two ASSIGNMENTS to one fluent whose guards can hold together — a
 *       satisfiability question, so a syntactic screen collects candidates and
 *       phase 2 of apply() asks Z3 only about those. A problem with no such
 *       action never builds a Z3 context. Cost is reported as
 *       duplicate_effect.assign_conflict_{candidates,queries,ms}.
 *
 * ---------------------------------------------------------------------------
 * WHAT IT DETECTS
 * ---------------------------------------------------------------------------
 * A ground action with two or more effects on one fluent. The usual source is
 * not a modelling error but GROUNDING A LIFTED ACTION AT A REFLEXIVE
 * INSTANTIATION:
 *
 *     drink(?from, ?to):  (decrease (locale ?from) 1)
 *                         (increase (locale ?to)   1)
 *
 * writes two DIFFERENT fluents when lifted. Ground it with ?from = ?to = lamb
 * and both land on locale(lamb). PDDL does not assume parameters are distinct
 * unless the precondition says so, so drink(lamb,lamb) is a legal ground action
 * — and a well-defined no-op on locale(lamb), since the two additive updates
 * cancel. The same happens for every move(from,to) / transfer(a,b) / swap(x,y)
 * at a reflexive instantiation.
 *
 * ---------------------------------------------------------------------------
 * WHY DROPPING THEM IS AN IMPROVEMENT ON THE STATUS QUO
 * ---------------------------------------------------------------------------
 * Nothing downstream handles this shape correctly today. All of the following
 * was established while chasing an invalid plan on pddl/test/mprime under
 * `elsc-fp` (2026-08-04); each item is verified in the source or measured.
 *
 * >>> THE MECHANISM IS NOW MOSTLY CLOSED, BUT NOT ENTIRELY.  `are_simply_commuting`
 * >>> used to bail on any repeated fluent whatsoever, so `hire-diver` in
 * >>> cave-diving and `refuel_at_port` in petrobras reported interference against
 * >>> every action in the problem.  It is scoped to fluents BOTH actions write
 * >>> now, which removed 144 of 3246 edges on cave-diving and 29 of 50 on
 * >>> petrobras (measured, `interference.semantic.edges` under egs/egsc).  The
 * >>> remaining route to the same failure is independent of this pass:
 * >>> PropagatorSharedState::build_footprint_index collects only WRITTEN fluents
 * >>> and precondition fluents, never the fluents an effect reads on its
 * >>> right-hand side or in a conditional guard.  So a genuine read/write hazard
 * >>> between two actions that share only such a fluent is invisible to the
 * >>> search's prefilter and visible to extraction — the same asymmetry that
 * >>> produced the invalid plan below.  UNCONFIRMED on a real instance.
 * >>> `duplicate_effect.actions_kept_conditional` still counts the actions this
 * >>> pass deliberately spares.
 *
 *   - GroundedEncoder::encode_non_cumulative_effects emits one equality per
 *     effect and conjoins them: `action -> (f' = f-1) AND (f' = f+1)`, which is
 *     UNSATISFIABLE, so the action is already unselectable. encode_cumulative_
 *     effects(), the hook that would aggregate additive effects, is a `return
 *     true;` stub in GroundedEncoder — only the Chained encoder overrides it.
 *     Dropping changes nothing for the Grounded encoders; it makes a silent
 *     fact visible.  NOTE this is a COMPLETENESS BUG in its own right: an
 *     action like `(decrease (energy) 3) (increase (energy) 1)` has a real net
 *     effect of -2 and is likewise unselectable, so the planner can report no
 *     plan for a solvable problem.
 *   - The Chained encoders USED TO apply half the action: the chain built one
 *     link per action and took the first matching effect, silently ignoring the
 *     rest. Measured on mprime: locale_lamb went 3 -> 2 across t=0 where the
 *     true net effect is 0. FIXED — variable_modifiers_ now holds one entry per
 *     EFFECT, so duplicate writes get one chain link each and compose correctly
 *     (net 0 for a reflexive grounding, net -2 for `(decrease f 3) (increase f
 *     1)`, exactly one arm for a clip-to-capacity pair). So for NUMERIC fluents
 *     under the Chained encoders this pass is now pure completeness loss, not a
 *     repair. It still matters for the Grounded encoders (previous item) and
 *     for BOOLEAN duplicates, which never chain (`is_bool_type` is excluded)
 *     and so still hit the unsatisfiable conjunction on every encoder.
 *   - SemanticInterferencePredicate::are_simply_commuting USED TO bail out with
 *     "not commuting" the moment it saw the duplicate, before it had even looked
 *     at the other action (see the next section). FIXED — it groups the writes
 *     per fluent and tests every cross pair, examining only fluents both actions
 *     write. Testing pairs suffices because commutation bubbles: if every write
 *     of a1 commutes with every write of a2, the two compositions commute.
 *   - epc_index_ lists the action twice as an explainer of its own fluent, so
 *     the frame axiom repeats it — harmless, but it is why FrameAxiomModule's
 *     header warns that one action can own two explainers of one shape.
 *
 * ---------------------------------------------------------------------------
 * THE FAILURE THIS CAME FROM, IN FULL — the interesting part
 * ---------------------------------------------------------------------------
 * `mprime` under `elsc-fp` produced a plan VAL rejects: the precondition
 * `craves(rest,pork)` of `overcome(abrasion,rest,pork)` was unsatisfied. The
 * Z3 MODEL was sound — the co-scheduled action set at t=0 is serialisable
 * exactly as chosen, and simulating it with `overcome` before `feast` validates
 * and reaches the goal. Plan EXTRACTION emitted the wrong order. Chain:
 *
 *  1. `drink(?from,?to)` moves liquid: (decrease (locale ?from) 1) and
 *     (increase (locale ?to) 1) — two DIFFERENT fluents when lifted. Ground at
 *     ?from = ?to = lamb and they collapse onto locale(lamb).
 *  2. are_simply_commuting sees the duplicate and returns false — a property of
 *     ONE action reported as a verdict about the PAIR, and the check is
 *     symmetric, so `drink(lamb,lamb)` is declared mutually interfering with
 *     EVERY action in the problem, including ones it shares no state with.
 *  3. That breaks an unwritten invariant the rest of the system relies on:
 *         interference(a,b)  IMPLIES  a and b share a fluent.
 *     PropagatorSharedState::build_footprint_index builds `potential_interferers`
 *     from shared fluents, and ExistsCycleModule only asks the oracle about
 *     pairs in that index. So the bogus edges are INVISIBLE during search: the
 *     module sees an acyclic graph and permits the step (correctly — the set IS
 *     serialisable).
 *  4. SemanticInterferenceAnalysis::topological_sort_actions asks about ALL
 *     n^2 ordered pairs, with no prefilter. It sees the bogus edges, and since
 *     they are symmetric it gets immediate 2-cycles: {feast, overcome,
 *     drink(lamb,lamb)} is one SCC.
 *  5. Graph::topological_sort (analysis/graph.cpp) is a post-order DFS with a
 *     `visited` array and NO CYCLE DETECTION. On cyclic input it silently
 *     returns a non-topological order. It emitted `feast` before `overcome`;
 *     feast deletes craves(rest,pork); the plan is invalid.
 *
 * So the two halves of the planner disagreed about whether a step was legal,
 * and the half that cannot cope is the one that found out. Note ExistsCycleModule
 * DOES have correct cycle detection (a recursion stack) — had it seen those
 * edges it would simply have forbidden the co-scheduling: sound, just less
 * parallel. Only the Grounded encoders are shielded, because there
 * `drink(lamb,lamb)` is unselectable (item 1 above), which is why `els-fp` never
 * reproduced this and `elsc-fp` did.
 *
 * ---------------------------------------------------------------------------
 * THE SAME ERROR APPEARS AT LEAST THREE TIMES — a unary property returned as
 * a verdict about a pair
 * ---------------------------------------------------------------------------
 *  (a) are_simply_commuting's duplicate-fluent guard, above. FIXED: scoped to
 *      fluents both actions write.
 *  (b) affects_source's `if (has_conflicting_conditional_effects(a1)) return
 *      CONFLICTING_COND_EFFECTS;` — again a property of a1 alone, returned
 *      before a2 is examined. FIXED, by deletion: it was a conservative screen
 *      around the composition bug in (c), and once that is fixed it guards
 *      nothing. What remained of it — the genuine ill-formedness of two
 *      simultaneously-applicable assignments — moved into phase 2 of this pass,
 *      where it is a unary property of a unary thing.
 *  (c) check2_source's CHECK2_HAPPEN loop ranges over the UNION of both actions'
 *      fluents, so it compares happening-vs-sequential for a fluent only ONE
 *      action touches. That is deliberate — it is how read/write hazards are
 *      caught — but the two sides used to compose duplicate writes DIFFERENTLY:
 *      happening_value built each arm on the running value (correct) while
 *      apply_action_effects_substitution built every arm on the pre-state fluent
 *      (dropping all but the last write). For a fluent written twice by a1 and
 *      not at all by a2, happening gave f+d1+d2 and sequential gave f+d2, so the
 *      query was satisfiable and interference was reported for a pair sharing
 *      nothing. FIXED: both now compose on the running value.
 *
 *      This is very likely the third path that made the debug assertion fire on
 *      mprime in the earlier attempt — the mechanism matches exactly, and
 *      mprime's `drink(lamb,lamb)` is precisely a fluent written twice by one
 *      action. NOT CONFIRMED: that attempt was reverted before the composition
 *      was fixed, so the two were never separated. Re-adding the assertion on a
 *      Debug build would settle it.
 *
 * The assertion is worth re-adding when the invariant is enforced rather than
 * merely hoped for; it caught (c) instantly and at the point of failure, where
 * the original symptom took a full debugging session to localise.
 *
 * ---------------------------------------------------------------------------
 * WHAT IT COSTS — read before relying on this
 * ---------------------------------------------------------------------------
 * Dropping is SOUND (removing an action cannot make an invalid plan valid) but
 * NOT COMPLETENESS-PRESERVING in general. It is free only when the duplicate
 * writes cancel, as they do for a reflexive grounding. An action like
 *
 *     (decrease (energy) 3)  (increase (energy) 1)
 *
 * has a real net effect of -2, and dropping it removes a legitimate action —
 * the planner may then report no plan for a solvable problem. Under the
 * Grounded encoders that action is already unusable (unsatisfiable effect
 * constraint), so nothing is lost there. Under the Chained encoders it is now
 * encoded CORRECTLY, so for a numeric fluent this pass is a pure loss there and
 * the right next step is to scope the drop to the cases that are still broken:
 * boolean duplicates, and every duplicate under a non-chained encoder. The
 * WARNING is deliberately loud for that reason.
 *
 * It reports the two kinds SEPARATELY, because only one of them is a loss:
 *
 *   BOOLEAN-only drops are free. The action asserts one atom both true and
 *   false, which is undefined in PDDL and unsatisfiable on every encoder
 *   (booleans never chain), so it was unusable before this pass touched it.
 *   Reflexive groundings emit them wholesale — Zenotravel drops a
 *   fly-slow(p,c,c) for every plane x city.
 *
 *   NUMERIC drops are the ones that can cost a plan, per the paragraph above.
 *   An action counts as numeric as soon as ONE duplicated fluent is.
 *
 * Each kind gets a count and one example at INFO and is counted separately in
 * duplicate_effect.actions_dropped{,_boolean}. The per-action names are DEBUG
 * (-vv) for both kinds: on a reflexively-grounded domain the list is hundreds
 * of lines long and buries the summary.
 *
 * ---------------------------------------------------------------------------
 * WHAT THIS SHOULD BECOME
 * ---------------------------------------------------------------------------
 * Normalise instead of drop, establishing the invariant "at most one effect per
 * (action, fluent)" that the rest of the pipeline already assumes:
 *
 *   - all effects on f unconditional additive  -> one net effect; drop the
 *     effect entirely when the net is syntactically zero. Exact even when the
 *     deltas are state-dependent, because PDDL evaluates every effect
 *     expression against the PRE-state, so merging is a symbolic sum.
 *   - all effects on f conditional additive    -> one guarded-delta sum,
 *     f + ite(c1, d1, 0) + ite(c2, d2, 0). Needs no mutual exclusivity. DONE
 *     in the two consumers that compose effects themselves: the chained
 *     encoder and SemanticInterferencePredicate both now build their ite chain
 *     on the RUNNING value, so several guarded writes accumulate instead of the
 *     last one winning. Not done in the IR — ExprPool has no ITE operator, so
 *     the group cannot be folded into a single EffectExpression.
 *   - anything involving assign (including a boolean add and delete of one
 *     atom, which are assigns to true/false) -> genuinely ill-formed, PDDL 2.1
 *     forbids two assignments to the same primitive numeric expression; there
 *     is no canonical composition of two overwrites. DETECTED HERE: phase 2 of
 *     apply() runs the `pre AND ci AND cj` satisfiability query per (action,
 *     fluent) that survives the syntactic screen, and drops the action if two
 *     different assignments can apply together. That query used to live in
 *     SemanticInterferencePredicate, where it was evaluated per action PAIR and
 *     on a hit reported a1 as interfering with every action in the problem.
 *
 * Once that invariant is enforced AND asserted at the consumers, the duplicate
 * guard in are_simply_commuting becomes dead code and should be replaced by the
 * assertion rather than kept as logic. (Its sibling in the predicate,
 * has_conflicting_conditional_effects, is already gone.)
 */
class DuplicateEffectPass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "duplicate-effect"; }
};

} // namespace rantanplan
