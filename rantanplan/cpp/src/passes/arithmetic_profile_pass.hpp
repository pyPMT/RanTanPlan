#pragma once

#include "pass.hpp"

namespace rantanplan {

/**
 * @brief Classifies the numeric constraint structure of the final problem.
 *
 * Runs NumericConstraintAnalyzer on the fully preprocessed problem, stores
 * the resulting ArithmeticProfile in the result, and marks the problem
 * all-integer when every numeric fluent is integer-valued (drives Int vs
 * Real sort selection in the Z3 translation layer).
 *
 * Must run last: it reads the final expressions (post grounding, folding,
 * pruning) and mutates the problem's all_integer flag, which every later
 * Z3 translation consults.
 */
class ArithmeticProfilePass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "arithmetic-profile"; }
};

} // namespace rantanplan
