#pragma once

#include <utility>
#include <vector>
#include "../problem/expr_pool.hpp"

/**
 * @file landmark_data.hpp
 * @brief Plain-data landmark artifact produced by LandmarkAnalysisPass.
 *        Included by value in preprocessed_problem.hpp, which must stay
 *        compilable without a solver (cvc5) include path — hence no solver
 *        types HERE; passes and pointer-held artifacts use the solver freely.
 */
namespace rantanplan {

/// A disjunctive action landmark with a uniform count semantics: achiever
/// firings must accumulate at least `residual` mass, where achiever
/// `achiever_action_ids[i]` contributes `contributions[i]` per firing
/// (parallel vectors).  Numeric threshold landmarks whose achievers all
/// have constant additive effects on the tracked fluent get real values
/// (non-helping achievers contribute 0).  Everything else — boolean
/// landmarks, and any landmark with an assignment, state-dependent,
/// nonlinear, or quantified achiever effect — takes the sound default:
/// residual 1 and every contribution 1, i.e. plain "some achiever fires".
struct LandmarkInfo {
    ExprID condition = EXPR_NULL;
    std::vector<int> achiever_action_ids;
    std::vector<double> contributions;  ///< per-firing mass, parallel to ids
    double residual = 1.0;              ///< required total mass (> 0)
};

struct LandmarkData {
    std::vector<LandmarkInfo> landmarks;

    /// Natural orderings from the closure: (earlier, later) indices into
    /// `landmarks` — the earlier landmark is a common precondition of EVERY
    /// achiever of the later one, and false at the initial state.  That is
    /// strong enough for per-occurrence enforcement (any later-achiever
    /// firing at step t needs an earlier-achiever firing in [0, t]; on
    /// chained encodings t is inclusive — same-step achievement earlier in
    /// the chain is possible).  Recorded for downstream consumers; note that
    /// on exists-step semantics ordering CHAINS collapse into single
    /// timesteps (a linear enabling chain is exactly what a step permits),
    /// so chains give no horizon bound there.  Unused by the horizon
    /// arithmetic.
    std::vector<std::pair<int, int>> orderings;

    /// max over landmarks of ceil(residual / sum of contributions): no plan
    /// has fewer steps, even with every achiever firing cumulatively in one
    /// step.  Default landmarks contribute the trivial bound 1.
    int depth_lower_bound = 0;
};

} // namespace rantanplan
