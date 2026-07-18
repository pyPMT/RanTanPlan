#pragma once

#include "pass.hpp"

namespace rantanplan {

class Problem;

/// True if any action expression contains a nested fluent term: a
/// state-reading term (fluent application or array access) in an argument
/// position of another fluent application, or in the element position of a
/// set member/add/remove.
bool has_nested_fluent_terms(const Problem& problem);

/// Rewrite every action so no nested fluent term remains (ghost parameters +
/// pinning equality preconditions). Returns the rewritten problem (shared
/// ExprPool); no-op inputs come back unchanged action-by-action.
Problem resolve_nested_terms(const Problem& problem);

/**
 * @brief Resolve nested fluent terms (FSTRIPS composition) via ghost
 * parameters, before the structured lowerings and grounding.
 *
 * A nested fluent term is a state-reading term used as an *argument* of
 * another fluent application — `connections(card_at[r][c], N)`,
 * `loc(owner(x)) := l`, `member(loc(t), S)` — which violates the ground-atom
 * shape every post-grounding consumer assumes (a state variable's arguments
 * are object constants; see grounding_pass.hpp). This pass restores that
 * shape by compilation: for each distinct inner term in an action it appends
 * a fresh "ghost" action parameter of the term's value type, pins it with an
 * equality precondition (`card_at[r][c] = ?g`), and substitutes the parameter
 * for the term. The equality is exactly the object-fluent pattern the
 * grounder's BindingMatcher joins, so the ghost is enumerated only over the
 * term's *reachable* values, and everything downstream sees stock shapes —
 * no encoder, analysis, or interference change.
 *
 * Runs after `QuantifierExpansionPass` (quantified variables are already
 * enumerated) and before `ArrayLoweringPass`/`SetLoweringPass` (a ghost
 * equality on an array cell is lowered like any other access; a ghosted set
 * element is an ordinary parameter). Inner terms resolve innermost-first, so
 * `f(g(h(x)))` yields chained ghost equalities. v1 restrictions: inner terms
 * must be object-valued (int-valued terms are the N2/N3 axes —
 * docs/nesting-status-and-gaps.md §4.3/§4.4), and goals / state invariants /
 * initial state cannot host a ghost parameter, so nested terms there are
 * rejected with a clear error.
 *
 * Ghost parameters are appended after the declared parameters; ground
 * instances therefore carry their ghost values as trailing parameters, which
 * the Python plan mapping ignores (it zips against the lifted UP signature).
 */
class TermResolutionPass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "term-resolution"; }
};

} // namespace rantanplan
