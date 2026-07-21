#pragma once

#include "base_planner.hpp"
#include "gse_classification.hpp"
#include "../preprocessing/landmark_data.hpp"

#include <unordered_map>

namespace rantanplan {

/**
 * @brief GSE planner — Generalised Support Encoding (partial-order causal-link).
 *
 * A horizon-free SMT encoding for numeric planning.  Variables are action
 * *instances* (occurrence flag + integer rank + seen values), not per-timestep
 * state copies.  The single support rule "the value an instance sees is the
 * output of its last writer plus the net additive change since" generalises the
 * classical causal link (Boolean case) and the Petri-net state equation
 * (pure-additive case).  The outer loop iterates an *instance budget* instead of
 * a horizon.  Design: ideas/pocl/idea.md; didactic companion:
 * ideas/pocl/gse-encoding.tex; staging: ideas/pocl/implementation-plan.md.
 *
 * Implementation notes:
 *  - The inherited encoder_/solver_ are accepted and ignored (F9); GSE builds
 *    its own formula on a fresh z3::solver each budget round (refines D3).
 *  - Every extracted plan is checked by an exact simulator before return (D7);
 *    a simulator/encoding mismatch is a hard error, never a warning.
 */
class GsePlanner : public BasePlanner {
public:
    GsePlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx);
    ~GsePlanner() override = default;

    /// Consume the landmark artifact (budget init + C8 cuts).  Copies the
    /// plain-data LandmarkData; leaves achievers to other planners.
    void consume_artifacts(PreprocessedProblem& artifacts) override;

    Plan search() override;

private:
    GseClassification cls_;
    LandmarkData landmark_data_;

    /// k_a — copies per ground action id (grows across budget rounds).
    std::unordered_map<int, int> copies_;

    /// M3: raise initial k_a from single-achiever count landmarks.
    void initialise_budget_from_landmarks();
};

} // namespace rantanplan
