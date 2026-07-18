#pragma once

#include "problem.hpp"

#include <unordered_set>

namespace rantanplan {

/// Call `fn(ExprID)` on every *root* expression a problem references, once per
/// occurrence: each action's precondition and explicit cost, every effect's
/// fluent / value / (conditional) condition, then every goal, state invariant,
/// and initial-state fluent / value. Roots only — it does not recurse into
/// subexpressions (use for_each_expression for that). This is the single place
/// the "which expressions does a problem hold" enumeration lives.
template <class Fn>
void for_each_root_expression(const Problem& problem, Fn&& fn) {
    for (const Action& a : problem.actions()) {
        if (a.has_precondition()) fn(a.precondition_id());
        if (a.has_explicit_cost()) fn(a.cost_id());
        for (const Effect& eff : a.effects()) {
            const EffectExpression& ee = eff.effect_expression();
            fn(ee.fluent_id());
            fn(ee.value_id());
            if (ee.is_conditional()) fn(ee.condition_id());
        }
    }
    for (const Goal& g : problem.goals()) fn(g.goal_id());
    for (ExprID inv : problem.state_invariants()) fn(inv);
    for (const Assignment& asn : problem.initial_state()) {
        fn(asn.fluent_id());
        fn(asn.value_id());
    }
}

namespace detail {
template <class Fn>
void visit_subexpressions(const ExprPool& pool, ExprID eid,
                          std::unordered_set<ExprID>& seen, Fn& fn) {
    if (!eid.valid() || !seen.insert(eid).second) return;
    fn(eid);
    for (ExprID c : pool.children(eid)) visit_subexpressions(pool, c, seen, fn);
}
}  // namespace detail

/// Call `fn(ExprID)` once on every distinct subexpression reachable from any
/// root expression of the problem (each root and its transitive children). A
/// single shared visited-set means interned/shared nodes are visited exactly
/// once across the whole problem.
///
/// Example — a lowering-pass survival check that no ARRAY_INDEX node leaked
/// past grounding:
///
///     void assert_no_array_index(const Problem& problem) {
///         const ExprPool& pool = problem.pool();
///         for_each_expression(problem, [&](ExprID eid) {
///             if (pool.is_function_application(eid) &&
///                 pool.op(eid) == ExprOperator::ARRAY_INDEX)
///                 throw std::runtime_error(
///                     "ARRAY_INDEX node survived lowering: " + pool.to_string(eid));
///         });
///     }
///
template <class Fn>
void for_each_expression(const Problem& problem, Fn&& fn) {
    const ExprPool& pool = problem.pool();
    std::unordered_set<ExprID> seen;
    for_each_root_expression(problem, [&](ExprID root) {
        detail::visit_subexpressions(pool, root, seen, fn);
    });
}

}  // namespace rantanplan
