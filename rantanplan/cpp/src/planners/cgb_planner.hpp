#pragma once

#include "../problem/problem.hpp"
#include "../problem/plan.hpp"
#include "../encoders/base_encoder.hpp"
#include "../analysis/achievers_analysis.hpp"
#include "../preprocessing/landmark_data.hpp"
#include "base_planner.hpp"
#include <z3++.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rantanplan {

/**
 * @brief CGB planner — Core-Guided Budgets
 *
 * Implements the core loop of "Numeric Planning as SMT with Optional
 * Occurrences" on the grounded step-indexed encoding, deliberately stripped
 * of PDLA's heuristic machinery (obligation queue, action scoring, spread
 * classification, tracked preconditions, assumption shuffling) so the two
 * designs can be compared side by side on the same encoding stack.
 *
 * Mechanism:
 *   - Existence is a literal: each ground action starts dormant, guarded by
 *     a stable blocking assumption blk_a -> !a_t (all timesteps); dormant
 *     actions' preconditions/effects are not encoded.
 *   - Loop: solve under {blocking literals} + {goal literal}.  SAT: extract
 *     the plan.  UNSAT: blocking literals in the core name exactly which
 *     actions to activate (encode + stop assuming); a core free of blocking
 *     literals proves the horizon binds, so extend it by one timestep.
 *   - Seeding: actions in an h^ff-style relaxed plan are activated up
 *     front; the horizon starts at the RPG lower bound.
 *
 * Design notes from measurement (see docs/pdla-optional-occurrence-roadmap.md):
 * per-schema occurrence budgets with cardinality caps — the paper-faithful
 * first cut — were dropped.  In a step-indexed encoding the horizon already
 * bounds multiplicity, and caps only made intermediate refutations harder;
 * per-schema activation granularity over-activated on domains with few,
 * large schemas.  What remains is the paper's existence boundary and
 * assumption/core outer loop at per-ground-action granularity.
 */
class CGBPlanner : public BasePlanner {
public:
    CGBPlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx);
    ~CGBPlanner() override = default;

    /// Consumes (moves) the achievers analysis built by ActionPruningPass.
    void consume_artifacts(PreprocessedProblem& artifacts) override;

    Plan search() override;

protected:
    // ---- Existence guards (one blocking literal per ground action) ----

    std::unordered_map<const Action*, z3::expr> block_lit_;
    std::unordered_set<const Action*> blocked_;
    std::unordered_set<const Action*> activated_;
    std::unordered_map<unsigned, const Action*> blk_id_to_action_;

    int current_horizon_ = -1;
    z3::expr goal_assumption_;
    int next_goal_version_ = 0;
    std::unordered_map<const Action*, std::unordered_set<int>> action_encoded_at_;

    class GroundedEncoder* grounded_encoder_ = nullptr;
    GroundedEncoder& grounded_encoder();

    // ---- Achiever data (relaxed-plan seeding) ----

    std::unique_ptr<AchieversAnalysis> achievers_;
    std::unordered_map<int, const Action*> action_id_to_ptr_;
    std::unordered_map<ExprID, std::vector<const Action*>> condition_achievers_;
    std::unordered_set<ExprID> init_satisfied_conditions_;
    std::vector<ExprID> goal_condition_ids_;
    std::unordered_map<const Action*, std::vector<ExprID>> action_precondition_ids_;

    void initialize_achievers();
    std::unordered_set<const Action*> extract_relaxed_plan();
    void seed_activations();

    // Removed after measurement (designs and numbers in
    // docs/pdla-optional-occurrence-roadmap.md section 5): the branching
    // prior (M4 — free delivery, signal loses to Z3's phase caching), the
    // landmark watcher propagator with ordering enforcement and
    // thin-triggered discovery (M8/M9 — free when silent, firings perturb
    // satisfiable queries more often than they pay), and per-round
    // conditional landmark recomputation (M6 — provably vacuous: activation
    // is monotone, so the all-achievers-blocked hole set is maximal at
    // round 0 and only shrinks).

    // ---- Landmarks (paper section 7: guidance compiled into the solver) ----
    //
    // Disjunctive action landmarks over the achiever graph, asserted as one
    // guarded clause per landmark per horizon: goal -> some achiever fires
    // at some step.  Unit propagation against the blocking assumptions puts
    // the blocking literals of needed-but-dormant achievers straight into
    // the core, without search.  Deliberately clause-only: an equivalent
    // latch-chain encoding (auxiliary lm_c_t variables) slowed large
    // satisfiable queries measurably.

    std::vector<ExprID> landmarks_;
    void adopt_landmarks();

    // ---- Count landmarks: deficit budgets and horizon jumps ----
    //
    // From the LandmarkAnalysisPass artifact, used through the two channels
    // measurement showed to be safe (roadmap section 5):
    //
    //  - Deficit budgets over presence literals, per goal version: within
    //    t steps an unblocked contributor adds at most lambda*t toward a
    //    count landmark's residual, so goal -> enough unblocked mass.
    //    Root-level arithmetic over blk literals only — decided by the
    //    assumptions before search begins, so it cannot perturb; a conflict
    //    puts a SET of contributors into the core (activation by deficit
    //    breadth rather than one presence-clause achiever per round).
    //
    //  - Horizon jumps: when a budget-free core forces an extension, jump
    //    to the smallest horizon the count landmarks do not refute under
    //    current activations, ceil(residual / sum of active lambdas).
    //    Complete — steps may be empty, so any later plan still fits.

    LandmarkData landmark_data_;
    int count_bound_target() const;
    void assert_deficit_budgets(int t);

    // ---- Encoding ----

    void add_timestep(int t);
    void refresh_goal(int t);
    void encode_action_at(const Action* action, int timestep);

    // ---- Search helpers ----

    void activate_action(const Action* action);
    z3::expr_vector build_assumptions();

    Plan extract_plan(const z3::model& model);
};

} // namespace rantanplan
