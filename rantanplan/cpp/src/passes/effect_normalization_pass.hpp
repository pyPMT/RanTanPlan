#pragma once

#include "pass.hpp"

namespace rantanplan {

/**
 * @brief Makes every ground action write each fluent at most once in any
 * state, without changing what the action does.
 *
 * One ground action can write a fluent several times: conditional effects,
 * a forall expanded into one guarded write per object, or a reflexive
 * grounding such as drink(rice, rice), which decreases and increases
 * locale(rice). The Grounded encoders give every write its own equation on
 * the fluent's next value, so two writes that apply together must agree:
 * y' = y + 2 and y' = y + 3 have no model, and the action cannot be used
 * there. After this pass, wherever an action can be executed, at most one of
 * its writes to each fluent applies, or all that apply assign the same value.
 *
 * What an action does is what Unified Planning, whose validator checks every
 * plan, does with it: guards and values are read before the action,
 * increases and decreases add up, a Boolean is true if any applying write
 * says so, and two different assignments (or an assignment and an increase)
 * make the action inapplicable where both apply. The cases, for each fluent
 * an action writes more than once:
 *
 *   1. A Boolean set and cleared (the set wins: add after delete)
 *        when small: medium := true;  when medium: medium := false
 *        -> the clear becomes  when (medium and not small): medium := false
 *   2. A Boolean given computed values (true if any of them is)
 *        alarm := smoke;  alarm := heat
 *        -> alarm := smoke or heat
 *   3. Unconditional increases and decreases (they add up)
 *        x -= 1;  x += 1
 *        -> one net write; here none
 *   4. Guarded increases and decreases whose guards can hold together
 *        when w: y += 2;  when f: y += 3
 *        -> when (w and not f): y += 2;  when (not w and f): y += 3;
 *           when (w and f): y += 5
 *        Writes with the same guard are summed first. k guards give up to
 *        2^k writes: there is no limit, but above 4 the pass warns first.
 *   5. The same assignment twice (allowed)
 *        when a: x := 5;  when b: x := 5
 *        -> unchanged
 *   6. Two different assignments (an error where both apply)
 *        when hot: t := 50;  when humid: t := 40
 *        -> the precondition gains  not (hot and humid)
 *   7. An assignment and an increase or decrease (the same error)
 *        when a: x += 200;  when b: x := 200
 *        -> the precondition gains  not (a and b)
 *
 * Writes whose guards cannot hold together are left alone. Whether they can
 * is asked of Z3, over a single state; a pessimistic "yes" is safe, since
 * every rewrite above is exact in every state.
 *
 * An action that can never apply after case 6 or 7, or has no effect, is
 * removed; nothing else is, so "proven unsolvable" stays a proof. Cases 6
 * and 7 also print a warning (the model has a write conflict, an error in
 * Unified Planning and PDDL 2.1). Every case is counted in the statistics
 * effect_normalization.*.
 *
 * Runs after StaticFluentPass, which folds a guard that reads only static
 * fluents to true, making its write unconditional.
 *
 * Tests: tests/api/regressions.py, where each test problem's docstring
 * names its case.
 */
class EffectNormalizationPass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "effect-normalization"; }
};

} // namespace rantanplan
