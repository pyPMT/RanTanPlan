#pragma once

#include "pass.hpp"

namespace rantanplan {

/**
 * @brief Closed-World Assumption initial state completion pass.
 *
 * Ensures every grounded fluent has an explicit initial state assignment.
 * This pass is the sole materialiser of default initial values (the Python
 * wrapper no longer fills them in). For a fluent without an assignment:
 *   - a declared default_initial_value is authoritative;
 *   - otherwise booleans are closed-world false and numerics are 0 clamped
 *     into the declared bounds;
 *   - an object fluent has no sensible default and is a hard error.
 */
class CWAInitialStatePass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "cwa-initial-state"; }
};

} // namespace rantanplan
