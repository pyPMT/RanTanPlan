#include "arithmetic_profile_pass.hpp"
#include "../analysis/numeric_constraint_analyzer.hpp"
#include "../util/logger.hpp"

namespace rantanplan {

void ArithmeticProfilePass::apply(PreprocessedProblem& result) const {
    auto constraint_analysis = NumericConstraintAnalyzer::analyze(result.problem);
    result.arithmetic_profile = constraint_analysis.profile;
    result.problem.set_all_integer(constraint_analysis.all_integer);

    const char* logic_hint = recommended_logic(constraint_analysis.profile, constraint_analysis.all_integer);
    Logger::instance().component(VerbosityLevel::INFO, "ArithProfile", {
        {"class", arithmetic_profile_to_string(constraint_analysis.profile)},
        {"numeric fluents", std::to_string(constraint_analysis.num_numeric_fluents)},
        {"integer", constraint_analysis.num_numeric_fluents > 0
            ? (constraint_analysis.all_integer ? "yes" : "no") : "n/a"},
        {"logic", logic_hint ? logic_hint : "default"}
    });
}

} // namespace rantanplan
