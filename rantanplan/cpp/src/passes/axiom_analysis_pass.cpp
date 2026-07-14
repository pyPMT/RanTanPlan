#include "axiom_analysis_pass.hpp"

#include "../analysis/axiom_dependency_analysis.hpp"
#include "../preprocessing/preprocessed_problem.hpp"

namespace rantanplan {

void AxiomAnalysisPass::apply(PreprocessedProblem& result) const {
    result.axiom_info = analyze_axioms(result.problem);
}

} // namespace rantanplan
