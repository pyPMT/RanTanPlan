#include "preprocessed_problem.hpp"

#include "../analysis/achievers_analysis.hpp"
#include "../analysis/numeric_relaxed_planning_graph.hpp"

namespace rantanplan {

PreprocessedProblem::PreprocessedProblem() = default;
PreprocessedProblem::~PreprocessedProblem() = default;
PreprocessedProblem::PreprocessedProblem(PreprocessedProblem&&) noexcept = default;
PreprocessedProblem& PreprocessedProblem::operator=(PreprocessedProblem&&) noexcept = default;

} // namespace rantanplan
