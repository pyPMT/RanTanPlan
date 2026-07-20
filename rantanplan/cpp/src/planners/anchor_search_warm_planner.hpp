#pragma once

#include "../problem/problem.hpp"
#include "../problem/plan.hpp"
#include "../problem/state_valuation.hpp"
#include "../preprocessing/landmark_data.hpp"
#include "../analysis/achievers_analysis.hpp"
#include "../encoders/base_encoder.hpp"
#include "base_planner.hpp"
#include <z3++.h>
#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rantanplan {

/**
 * @brief els-wc3 — anchor search on ONE warm SMT solver, used in two modes.
 *
 * This is the single-solver redesign of anchor search (paper:
 * ideas/new_planner/anchor_search_planning.tex).  Where els-wc2 rebuilds a
 * cold solver per dive, els-wc3 keeps a single incremental solver (the
 * inherited `solver_`) holding a *rolling* unrolling of the transition
 * relation with step 0 LEFT FREE.  Every query pins step 0 to a chosen
 * anchor state via retractable check-assumptions, so the same warm solver
 * serves every anchor and every horizon without a rebuild.  The design bets
 * that scoping (assumptions, not permanent per-anchor clauses) avoids the
 * clause-ledger thrash the predecessor measured on a shared solver — a bet
 * this implementation exists to test, which is why it is heavily
 * instrumented (see the wc3_* stats and RANTANPLAN_WC3_TRACE).
 *
 * Two modes, both `solver_.check(assumptions)`:
 *   DISCOVER(N,w): assume {x^0 == sigma_N} + goal@w.  SAT -> a plan
 *                  (prefix ++ window); UNSAT -> no plan of length <= w from N.
 *   WIDEN(N):      DISCOVER at w=1 read the model — harvest up to C forward
 *                  children (step-1 states), scored by the FF heuristic.
 * The backbone is DISCOVER from the initial anchor at a growing horizon H,
 * the completeness guarantee.  A signal-driven control law (Section 6 of the
 * paper) widens where a heuristic gradient exists and dives when an anchor's
 * estimate fits an affordable window.
 *
 * Milestone 1 scope: WIDEN + DISCOVER + backbone + control law + full
 * signals + replay-validation.  Nogood learning / inductive promotion
 * (Channels A/B, frame content) is deferred to milestone 2.
 */
class AnchorSearchWarmPlanner : public BasePlanner {
public:
    AnchorSearchWarmPlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx);

    void consume_artifacts(PreprocessedProblem& artifacts) override;

    Plan search() override;

private:
    // -----------------------------------------------------------------------
    // Anchor: one concrete reachable state and its remembered history.
    // -----------------------------------------------------------------------
    struct Anchor {
        int id = 0;
        int parent = -1;

        std::vector<z3::expr> sigma_values;   // dynamic-fluent z3 values (step-0 pin)
        size_t sigma_hash = 0;

        Plan prefix;                          // serialized actions s0 -> sigma
        int t = 0;                            // depth (steps in prefix)

        // FF relaxed-plan estimates (recomputed at creation).
        int h_ff = 0;                         // remaining size (actions)
        int d_ff = 0;                         // remaining steps  (the dive estimate)
        double lb = 0.0;                      // RPG min-steps lower bound
        std::unordered_map<int, int> first_layers;
        int rpg_action_layers = 0;            // RPG layer count (for absent-action masks)
        bool rpg_saw_all_actions = false;

        // Landmarks of the remaining task, recomputed at this anchor: frontier
        // steering, ordering, and d_lm = lm->depth_lower_bound.
        std::shared_ptr<const LandmarkData> lm;
        int d_lm = 0;                         // landmark depth lower bound
        int u = 0;                            // open-landmark count

        int D = 0;                            // demotions (failed dives sink here)
        int U = 0;                            // refuted window: no plan <= U from here
        int window = 1;                       // suggested dive window
    };

    /// Total order over anchor ids.  `mode` selects the primary greedy signal:
    /// 0 = d_ff (relaxed-plan depth), 1 = u (open-landmark count).  D is always
    /// primary (demoted anchors sink behind undemoted), id the final tiebreak.
    /// Two queues, one per mode, popped in alternation, give both signals
    /// expansion turns (the els-wc2 keep-heuristics-separate design).  Reads
    /// scores at compare time, so an anchor's keys must not change while queued.
    struct Order {
        const std::vector<Anchor>* anchors = nullptr;
        int mode = 0;
        bool operator()(int a, int b) const;
    };

    struct Closed { int min_depth = 0; bool dead = false; };

    /// A lifted nogood (Channel A): the UNSAT core of a failed dive, kept as a
    /// SOFTWARE filter (partial state assignment + horizon), NOT a solver
    /// clause — so it prunes future dives (Channel B) without salting the warm
    /// solver's ledger.  "No state agreeing on `core` has a plan to the goal
    /// within `w` steps"; sound to reuse for any dive of window <= w.
    struct Nogood {
        std::vector<std::pair<int, z3::expr>> core;  // (dynamic-fluent index, value)
        int w = 0;
    };

    // ---- helpers (state-generic; adapted from els-wc2) --------------------
    void initialize_achievers();
    StateValuation extract_state(const z3::model& model, int t,
                                 std::vector<z3::expr>& dynamic_values) const;
    /// Individual step-0 fluent-equality atoms, for check-assumptions.
    void state_equal_atoms(int t, const std::vector<z3::expr>& values,
                           z3::expr_vector& out) const;
    /// Full conjunction form (used to forbid staying put during WIDEN).
    z3::expr state_equals(int t, const std::vector<z3::expr>& values) const;
    bool score_anchor(const StateValuation& sigma, Anchor& out) const;
    int relaxed_plan_size(const z3::model& model, int t,
                          const std::unordered_map<int, int>& first_layers,
                          int* depth_out) const;

    // ---- landmark guidance (M2; adapted from els-wc2) ---------------------
    std::vector<ExprID> satisfied_conditions(const z3::model& model, int t) const;
    std::shared_ptr<const LandmarkData> recompute_landmarks(
        const StateValuation& sigma, const std::vector<ExprID>& sat_conditions) const;
    std::vector<int> landmark_frontier(const std::vector<int>& open_idx,
                                       const LandmarkData& data) const;
    z3::expr steering_disjunction(const std::vector<int>& frontier_idx,
                                  int t_begin, int t_end, const LandmarkData& data) const;
    int informed_window(const std::unordered_map<int, int>& first_layers,
                        const LandmarkData& lm, int d_ff) const;
    /// RPG masks as retractable assumptions: append `!a^off` for every
    /// (action, offset) the anchor's RPG proves cannot fire yet.  Sound
    /// (delete-relaxation over-approximates), salt-free (per-check only).
    void append_masks(const std::unordered_map<int, int>& first_layers,
                      int rpg_action_layers, bool saw_all_actions, int w,
                      z3::expr_vector& out) const;

    // ---- the one warm solver ----------------------------------------------
    void ensure_started();          // register step-0 vars; step 0 stays free
    void ensure_depth(int w);       // extend the rolling unrolling to depth w
    z3::expr goal_lit(int h);       // cached goal_at_h literal (implies goal@h)
    bool extract_root(Anchor& root);// one-shot init-state extraction (throwaway)

    // ---- the two modes -----------------------------------------------------
    enum class DiveResult { PlanFound, Unsat, Unknown, Timeout };
    DiveResult discover(const Anchor& n, int w, const char* engine);
    // ---- nogood channel (M4) ----
    void note_nogood(const z3::expr_vector& atoms, const z3::expr_vector& core, int w);
    bool nogood_blocks(const std::vector<z3::expr>& sigma_values, int w) const;
    void widen(int anchor_id, int w);  // emit up to C frontier-crossing children w steps ahead

    // ---- control ----------------------------------------------------------
    int make_anchor(int parent, Plan prefix, int t, const StateValuation& sigma,
                    std::vector<z3::expr> sigma_values,
                    const z3::model& model, int model_t);
    void push_open(int id);
    void erase_open(int id);
    int pop_open();                 // pop from the alternating d_ff / u queues
    int best_open() const;          // peek d_ff-best (trace only)

    enum class Step { PlanFound, Progress, Retire, Timeout };
    Step greedy_step();             // one widen-or-dive on the best anchor
    Step backbone_step();           // one DISCOVER(root, H) at the current H

    bool replay_validate(const Plan& flat) const;
    Plan finish(const Plan& flat, int horizon, const char* engine);
    void record_stats() const;
    std::string summary_line() const;  // always-on end-of-search signal digest
    void trace(const char* event);  // RANTANPLAN_WC3_TRACE live line

    // -----------------------------------------------------------------------
    // Static problem data
    // -----------------------------------------------------------------------
    std::vector<ExprID> dynamic_fluent_eids_;
    std::vector<int> dynamic_fluent_ids_;
    StateValuation base_valuation_;
    std::unordered_map<int, const Action*> action_by_id_;
    std::unique_ptr<AchieversAnalysis> achievers_;
    std::vector<ExprID> goal_condition_ids_;
    std::unordered_map<ExprID, std::vector<const Action*>> condition_achievers_;
    std::unordered_map<const Action*, std::vector<ExprID>> action_preconditions_;
    LandmarkData landmark_data_;
    std::vector<ExprID> novelty_feature_ids_;   // achiever + goal conditions

    // -----------------------------------------------------------------------
    // Search state
    // -----------------------------------------------------------------------
    std::vector<Anchor> anchors_;
    std::set<int, Order> open_d_{Order{&anchors_, 0}};   // d_ff-primary
    std::set<int, Order> open_u_{Order{&anchors_, 1}};   // u-primary (open landmarks)
    int pop_turn_ = 0;                                    // alternates the two queues
    std::unordered_map<size_t, Closed> closed_;
    std::vector<Nogood> nogoods_;                 // Channel A store (software filter)

    int root_id_ = -1;              // the initial-state anchor (backbone origin)
    bool started_ = false;
    int filled_ = 0;                // transitions [0, filled_) present on solver_
    std::map<int, z3::expr> goal_lits_;

    int backbone_h_ = 0;            // current backbone horizon H
    int H_unsat_ = 0;              // deepest proven "no plan of length <= H"
    bool backbone_exhausted_ = false;
    bool greedy_alive_ = false;

    // Tunables (env-guarded; deterministic, no clocks).
    int window_cap_ = 12;           // RANTANPLAN_WC3_WINDOW_CAP: max dive window
    int greedy_ratio_ = 4;          // RANTANPLAN_WC3_GREEDY_RATIO: greedy steps / backbone
    unsigned dive_rlimit_ = 0;      // RANTANPLAN_WC3_DIVE_RLIMIT: 0 = unbounded
    bool trace_on_ = false;         // RANTANPLAN_WC3_TRACE
    bool backbone_only_ = false;    // RANTANPLAN_WC3_BACKBONE_ONLY
    // RANTANPLAN_WC3_MASKS: 0=off (default), 1=full, 2=unreachable-only.
    // Default OFF: full masks fix overshoot/plan-quality on some domains (depots
    // 119->43 actions) but their per-check assumption overhead regresses
    // action-heavy domains (zenotravel -> TO); an opt-in per-domain lever.
    int masks_on_ = 0;
    bool uorder_ = true;            // RANTANPLAN_WC3_UORDER: alternate d_ff/u queues (default on)
    // RANTANPLAN_WC3_DYNRATIO: Tier 1 dynamic budget split.  When the backbone
    // horizon has caught up to the root relaxed-plan depth estimate, the greedy
    // exploration is unamortized (the monolithic solve is in its closing zone,
    // OR the estimate is a gross under-estimate the greedy layer is overshooting
    // on), so the effective greedy/backbone ratio is throttled to 1 and the
    // budget goes to the backbone.  A logical (no-clock) regime predicate over
    // signals already tracked; default off pending the A/B.
    bool dyn_ratio_ = false;

    Plan found_plan_;
    std::chrono::steady_clock::time_point search_start_;

    // ---- signals ----------------------------------------------------------
    long widens_ = 0, widen_children_ = 0, dead_children_ = 0, closed_hits_ = 0;
    long dives_ = 0, dive_sat_ = 0, dive_unsat_ = 0, dive_unknown_ = 0;
    long backbone_checks_ = 0, backbone_unsat_ = 0, demotions_ = 0;
    long nogoods_learned_ = 0, nogood_prunes_ = 0;
    long throttled_iters_ = 0;      // Tier 1: iterations the greedy ratio was throttled
    long iters_ = 0;
    int max_depth_ = 0;
    size_t peak_open_ = 0;
    double widen_secs_ = 0.0, dive_secs_ = 0.0, backbone_secs_ = 0.0;
    long dive_window_sum_ = 0;      // for average dive window
};

} // namespace rantanplan
