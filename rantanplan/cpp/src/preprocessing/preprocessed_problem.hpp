#pragma once

#include <memory>
#include <string>
#include <vector>
#include "../analysis/arithmetic_profile.hpp"
#include "../problem/problem.hpp"
#include "landmark_data.hpp"
#include "local_invariants.hpp"
#include "symmetry_info.hpp"

/**
 * @file preprocessed_problem.hpp
 * @brief The layer contract between the preprocessor and any backend. Must
 *        stay compilable with no solver (cvc5) include path (see
 *        PreprocessedProblem).
 */
namespace rantanplan {

// Solver-using artifact classes appear only as forward declarations
class AchieversAnalysis;

/**
 * @brief Output contract of the preprocessor — input to any backend.
 *
 * Accumulated by the pass pipeline (each pass reads and updates it; the
 * pipeline short-circuits if any pass sets proven_unsolvable), then handed
 * to solve() as the single typed boundary between preprocessing and search.
 * All artifacts are strategy-independent with ExprID/plain-data query
 * surfaces.
 */
struct PreprocessedProblem {
    // Canonical task: grounded, CWA-complete initial state, statics folded, pruned.
    Problem problem;

    // Verdict
    bool proven_unsolvable = false;     ///< True if any pass proved unsolvability
    std::string unsolvable_reason;      ///< Name of the pass that proved unsolvability
    int lower_bound = 0;                ///< Max horizon lower bound across all passes (drives start_timestep)

    // Artifacts
    std::vector<SymmetryInfo> symmetry_data; ///< From SymmetryAnalysisPass
    std::vector<double> sdac_cost_lower_bounds; ///< SDAC cost lower bounds (from ActionPruningPass); empty if not SDAC
    ArithmeticProfile arithmetic_profile = ArithmeticProfile::LINEAR; ///< Numeric constraint class (from ArithmeticProfilePass)
    std::unique_ptr<AchieversAnalysis> achievers; ///< Pre-built achiever analysis (from ActionPruningPass); consumed by move
    StateConstraints state_constraints; ///< Bounds/conservations/domains from ActionPruningPass, mutexes from MutexAnalysisPass
    LandmarkData landmark_data; ///< Disjunctive action landmarks (from LandmarkAnalysisPass)

    // Out-of-line special members so the forward-declared artifact types
    // never need to be complete for users of this header.
    PreprocessedProblem();
    ~PreprocessedProblem();
    PreprocessedProblem(PreprocessedProblem&&) noexcept;
    PreprocessedProblem& operator=(PreprocessedProblem&&) noexcept;
    PreprocessedProblem(const PreprocessedProblem&) = delete;
    PreprocessedProblem& operator=(const PreprocessedProblem&) = delete;
};

} // namespace rantanplan
