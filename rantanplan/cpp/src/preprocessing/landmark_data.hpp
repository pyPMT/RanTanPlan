#pragma once

#include <utility>
#include <vector>
#include "../problem/expr_pool.hpp"

/**
 * @file landmark_data.hpp
 * @brief Plain-data landmark artifact produced by LandmarkAnalysisPass.
 *        Included by value in preprocessed_problem.hpp, which must stay
 *        compilable without a Z3 include path — hence no Z3 types HERE;
 *        passes and pointer-held artifacts use Z3 freely.
 */
namespace rantanplan {

/// A disjunctive action landmark: some action of `achiever_action_ids`
/// (the actions that can make `condition` true) fires in every plan.
///
/// It also has a uniform count semantics: firings of the contributors must
/// accumulate at least `residual` mass, where `contributor_action_ids[i]`
/// adds `contributions[i]` per firing (parallel vectors).  For a linear
/// numeric threshold whose fluents every action changes by a constant
/// additive amount, the contributors are all the actions that move it
/// towards the threshold, achievers or not (an action that can never make
/// the condition true can still add most of the deficit), and `residual`
/// is the deficit at the initial state.  Everything else — boolean
/// landmarks, and any threshold whose fluents some action assigns, or
/// changes by a state-dependent, nonlinear or quantified amount — takes the
/// sound default: the achievers are the contributors, each adding 1, and
/// the residual is 1, i.e. plain "some achiever fires".
struct LandmarkInfo {
    ExprID condition = EXPR_NULL;
    std::vector<int> achiever_action_ids;
    std::vector<int> contributor_action_ids;
    std::vector<double> contributions;  ///< per-firing mass, parallel to contributor_action_ids
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
    /// has fewer steps, even with every contributor firing cumulatively in
    /// one step.  Default landmarks contribute the trivial bound 1.
    int depth_lower_bound = 0;
};

} // namespace rantanplan
