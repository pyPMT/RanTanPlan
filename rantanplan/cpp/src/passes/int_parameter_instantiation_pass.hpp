#pragma once

#include "pass.hpp"

namespace rantanplan {

/**
 * @brief Compile bounded-integer action parameters away, before grounding.
 *
 * A purely syntactic pass: for each action with integer parameters, it takes the
 * Cartesian product of their `[lower, upper]` intervals and emits one action per
 * value tuple, substituting the chosen `Int` constant everywhere the parameter
 * appears — preconditions, effect fluents/values/conditions, effect `forall`
 * binders (so a range-variable bound `[1, n]` becomes `[1, 2]`), and cost.
 *
 * The instantiated integer parameter is kept in the signature as a constant-named
 * parameter (name = the value, e.g. `"2"`), so grounding carries it through and
 * the existing plan round-trip rebuilds `Int(2)` from it. Object parameters are
 * untouched and remain for normal grounding.
 *
 * Running this first means that by the time the QuantifierExpansionPass and the
 * grounder run, every range-variable bound is a constant and no parameter is
 * numeric-and-unbound — so quantifier removal is a single pre-grounding pass and
 * grounding is ordinary object grounding.
 *
 * No pruning here (that would duplicate the grounder / numeric RPG); this pass is
 * blind Cartesian expansion. Unbounded/partial int parameters are rejected earlier
 * (planner_wrapper._check_supported_action_parameters); this pass throws as a
 * backstop if one reaches it.
 */
class IntParameterInstantiationPass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "int-parameter-instantiation"; }
};

} // namespace rantanplan
