#pragma once

#include "pass.hpp"

namespace rantanplan {

/**
 * @brief Classifies the problem's axioms (TDG / SCC / stratification) into
 *        PreprocessedProblem::axiom_info.
 *
 * Runs *before* Grounding so the artifact exists even when Grounding (not yet
 * axiom-aware) would declare the task unsolvable and short-circuit the pipeline,
 * and so the future reachability-through-axioms fix can consume it. A no-op for
 * problems without axioms.
 */
class AxiomAnalysisPass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "AxiomAnalysis"; }
};

} // namespace rantanplan
