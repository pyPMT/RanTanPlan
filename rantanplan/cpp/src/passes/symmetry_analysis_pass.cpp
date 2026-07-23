#include "symmetry_analysis_pass.hpp"
#include "../symmetries/smt_symmetry_checker.hpp"
#include "../util/logger.hpp"
#include "../util/stats.hpp"
#include "../smtcore/smt_types.hpp"

namespace rantanplan {

void SymmetryAnalysisPass::apply(PreprocessedProblem& result) const {
    // Step 1: Detect symmetric object pairs using SMT.
    // Use the original (pre-grounding) problem if available — it has a smaller
    // initial state, making the SMT equivalence check cheaper. Falls back to
    // the current problem when grounding is disabled (problem is still lifted).
    const Problem& detection_problem = original_problem_
        ? *original_problem_ : result.problem;

    cvc5::TermManager detect_ctx;
    SMTSymmetryChecker detector(&detection_problem, detect_ctx);
    auto swaps = detector.detect_all_object_swaps();

    Logger::instance().info("Symmetry detection: " +
                            std::to_string(swaps.size()) + " symmetric pairs");
    Stats::instance().set("symmetries.pairs",
                          static_cast<double>(swaps.size()));

    if (swaps.empty()) return;

    // Step 2: Compute variable pairs and action pairs on the current
    // (grounded) problem.
    cvc5::TermManager completion_ctx;
    SMTSymmetryChecker completer(&result.problem, completion_ctx);
    completer.compute_symmetry_pairs(swaps);
    result.symmetry_data = completer.get_all_symmetries();
}

} // namespace rantanplan
