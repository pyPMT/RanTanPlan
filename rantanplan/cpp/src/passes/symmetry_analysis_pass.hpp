#pragma once

#include "pass.hpp"

namespace rantanplan {

/**
 * @brief Unified symmetry analysis pass (merges detection + completion).
 *
 * Detection: uses the original (pre-grounding) problem snapshot to find
 * symmetric object pairs via SMT equivalence checking.
 *
 * Completion: uses the current (grounded) problem to compute variable
 * pairs and action pairs for each detected swap.
 *
 * The intermediate detected_object_swaps stays local to this pass.
 */
class SymmetryAnalysisPass : public Pass {
public:
    /// @param original_problem  Optional pre-grounding snapshot for detection
    ///                          (cheaper Z3 equivalence check); may be null.
    explicit SymmetryAnalysisPass(const Problem* original_problem = nullptr)
        : original_problem_(original_problem) {}

    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "symmetry-analysis"; }

private:
    const Problem* original_problem_;
};

} // namespace rantanplan
