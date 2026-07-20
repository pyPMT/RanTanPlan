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
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace rantanplan {

/**
 * @brief Anchor search (els-wc2) — greedy best-first search over concrete
 *        states, redesign of the windowed-commitment planner.
 *
 * The search tree is an ordinary tree over concrete states (anchors).  A
 * node's successor function is an SMT oracle: a bounded els encoding whose
 * step-0 state is the node's anchor (the INIT OVERRIDE, the one genuinely
 * new piece of plumbing) solved under a landmark-frontier steering target,
 * returning up to C diverse, frontier-crossing, non-dead states w steps
 * ahead.  Live children enter two LAMA-style alternating open queues keyed
 * (demotions, h_ff|u, u|h_ff, t, novelty); expansion order IS the
 * greediness.  Completeness comes from an interleaved bounded-horizon sweep
 * run from the initial state (the root lane); goal direction from the same
 * sweep run from the best anchor as a hard initial state (the suffix lane,
 * which replaces the deleted phase-hint machinery).
 *
 * Nothing is ever retracted: prefixes are concrete states.  There are no
 * commitment guards, no pool, no retreat/jump machinery — the retreat
 * semantics are reproduced by the demotion count D, the primary queue key.
 * No search decision is driven by clocks: alternation and escalation go by
 * events and logical signals only; a wall deadline merely truncates.
 *
 * Design record: ideas/new_planner/anchor-search-design.md.
 */
class AnchorSearchPlanner : public BasePlanner {
public:
    AnchorSearchPlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx);

    /// Copies the landmark artifact and takes the achievers analysis.
    void consume_artifacts(PreprocessedProblem& artifacts) override;

    Plan search() override;

private:
    // -----------------------------------------------------------------------
    // Node: one concrete anchor state and everything the search hangs on it.
    // -----------------------------------------------------------------------
    struct Node {
        int id = 0;
        int parent = -1;                        // -1 for the root

        // Anchor state.  We keep the dynamic-fluent Z3 model values (context-
        // stable numerals) for the init override and blocking clauses, plus
        // the full-valuation hash for the global closed set.  The full
        // StateValuation is consumed at creation (RPG + landmark recompute)
        // and not retained.
        std::vector<z3::expr> sigma_values;     // aligned with dynamic_fluent_eids_
        size_t sigma_hash = 0;

        Plan prefix;                            // serialized action list s0 -> sigma
        int t = 0;                              // anchor timestep (steps in prefix)

        // Landmarks of the REMAINING task, recomputed from this anchor.  Every
        // landmark in it is open by construction; drives frontier/steering/
        // ordering/window, and lm->depth_lower_bound is d_LM(sigma).
        std::shared_ptr<const LandmarkData> lm;

        // Scores.
        int h_ff = 0;                           // FF relaxed-plan action count
        int d_ff = 0;                           // relaxed-plan parallel depth (steps)
        int u = 0;                              // open-landmark count
        double lb = 0.0;                        // RPG min-steps lower bound
        std::unordered_map<int, int> first_layers;  // action id -> first RPG layer
        int rpg_action_layers = 0;
        bool rpg_saw_all_actions = false;
        bool novel = false;                     // novelty flag (queue tie-break)

        // Search bookkeeping.
        int D = 0;                              // demotions (primary queue key)
        int U = 0;                              // node-local unsat bound
        int window = 0;                         // current window w
        std::vector<std::vector<z3::expr>> served;  // window-end states already generated
    };

    /// Global closed set entry (keyed on the full-valuation hash).
    struct Closed {
        int min_depth = 0;   // shallowest anchor timestep at which the state was seen
        bool dead = false;   // relaxed-dead: dead at any depth
    };

    /// LAMA-style total order over node ids.  `mode` selects the primary
    /// signal: 0 = h_ff, 1 = open-landmarks u, 2 = novelty (novel-first).
    /// D is always primary; node id is the final total-order tiebreak.  Reads
    /// scores from *nodes at compare time, so a node's keys must not change
    /// while it is in a queue (we always erase before mutating, reinsert
    /// after).
    struct QueueOrder {
        const std::vector<Node>* nodes = nullptr;
        int mode = 0;  // 0: h_ff primary, 1: u primary, 2: novelty primary
        bool operator()(int a, int b) const;
    };

    // ---- Reused analysis helpers (clean rewrites of the els-wc versions) ----

    void initialize_achievers();
    StateValuation extract_state(const z3::model& model, int t,
                                 std::vector<z3::expr>& dynamic_values) const;
    z3::expr state_equals(int t, const std::vector<z3::expr>& values) const;
    bool score_anchor(const StateValuation& sigma, Node& out) const;
    int relaxed_plan_size(const z3::model& model, int t,
                          const std::unordered_map<int, int>& first_layers,
                          int* depth_out = nullptr) const;
    std::vector<int> unachieved_landmark_indices(const z3::model& model, int t,
                                                 const LandmarkData& data) const;
    std::shared_ptr<const LandmarkData> recompute_landmarks(
        const StateValuation& sigma,
        const std::vector<ExprID>& sat_conditions) const;
    std::vector<int> landmark_frontier(const std::vector<int>& open_idx,
                                       const LandmarkData& data) const;
    z3::expr steering_disjunction(const std::vector<int>& frontier_idx,
                                  int t_begin, int t_end,
                                  const LandmarkData& data) const;
    void assert_masks(z3::solver& s, int t_anchor, int begin_off, int end_off,
                      const std::unordered_map<int, int>& first_layers,
                      int rpg_action_layers, bool saw_all_actions) const;
    void assert_ordering_clauses(z3::solver& s, int t_anchor, int begin_off,
                                 int end_off, const std::vector<int>& open_idx,
                                 const LandmarkData& data) const;

    // ---- Novelty (Panino, tie-break only; wired at M6) ----
    void extract_novelty_features(const z3::model& model, int t,
                                  std::vector<ExprID>& psi,
                                  std::vector<std::pair<ExprID, double>>& delta,
                                  std::vector<ExprID>& sat_conditions) const;
    bool is_novel(int h_ff, int u, const std::vector<ExprID>& psi,
                  const std::vector<std::pair<ExprID, double>>& delta) const;
    void update_novelty_tables(int h_ff, int u, const std::vector<ExprID>& psi,
                               const std::vector<std::pair<ExprID, double>>& delta);

    // ---- Oracle: a fresh bounded els encoding with the init override ----
    struct Oracle {
        std::unique_ptr<z3::solver> solver;
        std::unique_ptr<PropagatorStrategy> prop;
        int filled = 0;   // transitions [0, filled) present
    };
    Oracle build_oracle(const Node& n, int w, bool with_guidance = true) const;
    void extend_oracle(Oracle& oracle, const Node& n, int from_w, int to_w,
                       bool with_guidance = true) const;

    // Init-override self-test (M2), gated by RANTANPLAN_WC2_SELFTEST.
    void selftest_init_override();
    /// Sound internal replay check: force the exact serialized action list
    /// into a fresh native-init encoding and check the goal holds at its end.
    bool replay_validate(const Plan& flat) const;

    // ---- Greedy layer ----
    int informed_window(const std::unordered_map<int, int>& first_layers,
                        const LandmarkData& lm, int d_ff) const;
    int make_node(int parent, Plan prefix, int t, const StateValuation& sigma,
                  std::vector<z3::expr> sigma_values,
                  const z3::model& model, int model_t, int U0);
    void push_open(int id);
    void erase_open(int id);
    int pop_open();                 // -1 if both queues empty
    int best_estimate_node() const; // open node minimising t + d_ff (suffix target)
    int best_horizon_target() const;// max over open nodes of t + d_ff (root jump)

    enum class TurnResult { PlanFound, Progress, Retire, Timeout };
    TurnResult greedy_turn();       // pop one node, Expand it
    TurnResult expand(int node_id); // §6

    // ---- Sweep lanes (§7) ----
    TurnResult sweep_turn();        // alternate root / suffix
    TurnResult root_sweep_turn();   // §7.1 completeness lane (on solver_)
    TurnResult suffix_sweep_turn(); // §7.2 goal-direction lane (fresh solver)
    void ensure_root_started();

    // ---- Plan finalisation ----
    void record_final_stats();
    Plan finish_with_plan_flat(const Plan& plan, int reported_horizon,
                               const char* engine);

    // -----------------------------------------------------------------------
    // Static problem data (constructor)
    // -----------------------------------------------------------------------
    std::vector<ExprID> dynamic_fluent_eids_;
    std::vector<int> dynamic_fluent_ids_;
    StateValuation base_valuation_;
    LandmarkData landmark_data_;
    std::unordered_map<int, const Action*> action_by_id_;

    // Achiever data for h_ff / steering (state-independent structure).
    std::unique_ptr<AchieversAnalysis> achievers_;
    std::unordered_map<ExprID, std::vector<const Action*>> condition_achievers_;
    std::vector<ExprID> goal_condition_ids_;
    std::unordered_map<const Action*, std::vector<ExprID>> action_preconditions_;
    std::vector<ExprID> novelty_feature_ids_;

    // -----------------------------------------------------------------------
    // Search state
    // -----------------------------------------------------------------------
    std::vector<Node> nodes_;
    std::set<int, QueueOrder> open_h_{QueueOrder{&nodes_, /*mode=*/0}};
    std::set<int, QueueOrder> open_u_{QueueOrder{&nodes_, /*mode=*/1}};
    // Optional third queue, novelty-primary (RANTANPLAN_WC2_NOVELTY_QUEUE).
    // Surfaces novel states when h_ff and u are both flat (uninformative).
    std::set<int, QueueOrder> open_n_{QueueOrder{&nodes_, /*mode=*/2}};
    int pop_turn_ = 0;   // rotates over the active queues (2 or 3)
    std::unordered_map<size_t, Closed> closed_;

    // A/B feature flags (env-guarded; default off — baseline unchanged).
    bool novelty_queue_ = false;  // RANTANPLAN_WC2_NOVELTY_QUEUE
    bool dyn_window_ = false;      // RANTANPLAN_WC2_DYN_WINDOW
    // Deterministic resource bound on the greedy goal probe (design §10:
    // rlimit, not a clock).  0 = unbounded (default).  When set, an
    // rlimit-exhausted probe falls through to generation instead of eating
    // the whole budget — the fix for the long-plan starvation (§6.1).
    unsigned probe_rlimit_ = 0;    // RANTANPLAN_WC2_PROBE_RLIMIT
    // Greedy:sweep turn ratio (count-based, not a clock).  k greedy
    // expansions per sweep check, so the bounded dive gets room to search
    // between the expensive uncapped root-sweep checks.  1 = strict
    // alternation (default).
    int greedy_ratio_ = 1;         // RANTANPLAN_WC2_GREEDY_RATIO

    // Panino novelty tables (partition key = (h_ff, open landmarks)).
    using NoveltyPartition = std::pair<int, int>;
    std::map<NoveltyPartition, std::unordered_set<ExprID>> psi_seen_;
    std::map<NoveltyPartition, std::unordered_map<ExprID, double>> delta_min_;

    int H_unsat_ = 0;               // deepest T proven "no plan of length <= T"
    bool greedy_alive_ = false;     // enabled once the root node is scored (M3)
    bool sweeps_alive_ = true;
    bool sweep_root_turn_ = true;   // root/suffix alternation

    // Root sweep lane (on the primary solver_/propagator_strategy_).
    bool root_started_ = false;
    bool root_exhausted_ = false;
    int root_h_ = 0;
    int root_filled_ = 0;
    std::map<int, z3::expr> root_goal_lits_;

    // Suffix sweep lane (fresh persistent solver, retargeted on incumbent change).
    std::unique_ptr<z3::solver> suffix_solver_;
    std::unique_ptr<PropagatorStrategy> suffix_prop_;
    std::map<int, z3::expr> suffix_goal_lits_;
    int suffix_target_ = -1;
    int suffix_h_ = 0;
    int suffix_filled_ = 0;
    bool suffix_exhausted_ = false;
    std::unordered_set<int> suffix_done_;  // node ids whose suffix reached the cap

    Plan found_plan_;
    std::chrono::steady_clock::time_point search_start_;

    // Stats accumulators.
    size_t peak_open_ = 0;
    double expansion_build_secs_ = 0.0;
    bool greedy_only_ = false;      // RANTANPLAN_WC2_GREEDY_ONLY
};

} // namespace rantanplan
