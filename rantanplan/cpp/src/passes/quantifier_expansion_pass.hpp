#pragma once

#include "pass.hpp"

namespace rantanplan {

/**
 * @brief Remove all quantifiers, before grounding.
 *
 * Runs once, after `IntParameterInstantiationPass` (so every integer parameter is
 * already a constant and every range-variable bound is concrete) and before
 * `GroundingPass`. Two jobs:
 *
 *  1. **Conditions** — rewrite every `forall`/`exists` node in preconditions,
 *     conditional-effect conditions, goals, and state invariants into a ground
 *     conjunction (forall) or disjunction (exists) over its variables' domains:
 *       - object variable → the type's objects (`objects_of_type`);
 *       - range variable → its integer interval `[initial, last]`.
 *  2. **Forall-effects** — expand an effect carrying `forall_variable_ids` into one
 *     effect per binding of those variables. This must happen before grounding: the
 *     reachability grounder drops facts from effects with free variables.
 *
 * Object and range variables differ only in how the domain is computed; the
 * enumerate-substitute-combine machinery is shared. Nested quantifiers fall out of
 * the recursion. Because int parameters are already constants, there is no
 * symbolic-bound case here (a lifted backend that keeps int parameters would extend
 * this pass with the guarded expansion — the pass is otherwise independent of how
 * int parameters are handled).
 *
 * This replaces the Python `QuantifiersRemover`: RantanPlan owns all quantifier
 * removal in C++.
 */
class QuantifierExpansionPass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "quantifier-expansion"; }
};

} // namespace rantanplan
