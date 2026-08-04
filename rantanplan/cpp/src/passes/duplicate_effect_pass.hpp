#pragma once

#include "pass.hpp"

namespace rantanplan {

/**
 * @brief Drops ground actions that write the same fluent more than once.
 *
 * INTERIM MEASURE, not the intended end state — see "What this should become".
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
 *   - ChainedGroundedEncoder::get_effect_for_variable takes the FIRST matching
 *     effect and silently ignores the rest — its own comment documents this as
 *     a contract that callers must respect, and this action shape violates it.
 *     So the Chained encoders apply half the action. Measured on mprime:
 *     locale_lamb went 3 -> 2 across t=0 where the true net effect is 0.
 *     Dropping is strictly better than mis-applying.
 *   - SemanticInterferencePredicate::are_simply_commuting bails out with
 *     "not commuting" the moment it sees the duplicate, before it has even
 *     looked at the other action (see the next section).
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
 *  (a) are_simply_commuting's duplicate-fluent guard, above.
 *  (b) affects_source's `if (has_conflicting_conditional_effects(a1)) return
 *      CONFLICTING_COND_EFFECTS;` — again a property of a1 alone, returned
 *      before a2 is examined.
 *  (c) At least one more, still unidentified. An earlier attempt scoped (a) and
 *      (b) to shared fluents and added the invariant of item 3 as a debug
 *      assertion; on a Debug build the assertion STILL fired on mprime, so a
 *      third path reports interference for fluent-disjoint actions. The prime
 *      suspect is check2_source's CHECK2_HAPPEN loop, which ranges over the
 *      UNION of both actions' fluents rather than the intersection, and so
 *      compares happening-vs-sequential values for a fluent only one action
 *      touches — where the two compositions can disagree precisely because of
 *      the duplicate-effect handling in apply_action_effects_substitution.
 *      NOT CONFIRMED. That attempt was reverted in favour of this pass.
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
 * constraint), so nothing is lost there; under the Chained encoders it is
 * currently mis-encoded rather than lost, so this pass does trade one wrong
 * behaviour for another. The WARNING is deliberately loud for that reason, and
 * names every action it drops.
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
 *     f + ite(c1, d1, 0) + ite(c2, d2, 0). Needs no mutual exclusivity, which
 *     would also retire the CONFLICTING_COND_EFFECTS fallback for additive
 *     effects.
 *   - anything involving assign (including a boolean add and delete of one
 *     atom, which are assigns to true/false) -> genuinely ill-formed, PDDL 2.1
 *     forbids two assignments to the same primitive numeric expression. Reject
 *     with a diagnostic; there is no canonical composition of two overwrites.
 *
 * Once that invariant is enforced AND asserted at the consumers, the duplicate
 * guards in are_simply_commuting and has_conflicting_conditional_effects become
 * dead code and should be replaced by the assertion rather than kept as logic.
 */
class DuplicateEffectPass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "duplicate-effect"; }
};

} // namespace rantanplan
