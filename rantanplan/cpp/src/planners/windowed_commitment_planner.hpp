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
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace rantanplan {

class PhaseHintModule;
class OrderHintModule;

/**
 * @brief Windowed commitment planner v2 — budgeted greedy best-first
 *        search over stepping stones, in one loop over one formula.
 *
 * The greedy search is an EHC-style dive over a TREE of commitments.
 * Windows are solved from concrete anchor states with the full encoding;
 * candidate frontier states are scored by h^FF / open landmarks / RPG
 * bound from the candidate's own relaxed planning graph.  An improving
 * candidate is committed immediately (a child node under a fresh guard
 * pair: the frame guard c_k fixes the window's actions, the prefix guard
 * p_k <-> c_k /\ p_{k-1} carries the anchor-derived facts — masks,
 * landmark orderings — that are consequences of the whole prefix).
 * Non-improving viable candidates go into a single GLOBAL best-first open
 * pool.  When the dive is stuck — the core names the prefix, or the
 * lookahead ladder hits the horizon room — the search jumps to the best
 * pool entry and materialises it under its own prefix; because everything
 * is assumptions over one incremental formula, the jump is just a change
 * of assumption set.
 *
 * The CENTRAL GUIDING MECHANISM is the landmark ordering FRONTIER: at
 * each anchor, the ordering-minimal open landmarks form the antichain of
 * "what can come next" (the 1UIP picture — demand crossing a cut that
 * dominates the goal, not the goal itself).  Every generation query
 * requires some frontier landmark's achiever to fire in the window, so
 * every candidate makes ordered progress by construction, and the query
 * is expect-SAT.  When it is UNSAT and the core names no commitment
 * guard, no frontier crossing fits in w steps from this anchor — the
 * sound signal to WIDEN.  A core naming commitment guards jumps instead.
 * No search decision is driven by clocks: alternation and escalation go
 * by events and logical signals only; a wall deadline, when the user set
 * one, merely truncates.
 *
 * Complete search: a cold els sweep (own solver, prefix-monotone,
 * resumable) advanced ONE horizon check per greedy rung — event-driven
 * round-robin.  Its UNSAT at h proves no plan of length <= h and mutes
 * greedy goal probes at frontiers <= h.  Completeness = els's within
 * max_steps.  Design record: docs/windowed-commitment-v2.md.
 */
class WindowedCommitmentPlanner : public BasePlanner {
public:
    WindowedCommitmentPlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx);

    /// Copies the landmark artifact (non-exclusive: achievers/landmarks stay
    /// available to other consumers).
    void consume_artifacts(PreprocessedProblem& artifacts) override;

    Plan search() override;

private:
    // ---- Candidate frontiers and the commitment tree ----

    struct Candidate {
        std::vector<std::vector<const Action*>> segment;  // actions fired per window step
        StateValuation frontier;
        std::vector<z3::expr> frontier_values;  // aligned with dynamic_fluent_eids_
        double lb = std::numeric_limits<double>::infinity();  // +inf = vetoed dead end
        int h_ff = 0;  // FF-style relaxed-plan action count from the frontier
        int h_depth = 0;  // relaxed plan's parallel depth (steps, 1-based)
        std::vector<int> unachieved_idx;  // landmark indices unsatisfied at the frontier
        int goals_satisfied = 0;
        int nonempty_steps = 0;
        std::unordered_map<int, int> first_layers;  // action id -> first RPG layer at frontier
        int rpg_action_layers = 0;
        bool rpg_saw_all_actions = false;

        // Panino novelty features at the frontier (§5.4): boolean subgoals
        // satisfied, and numeric subgoal margins (satisfied margins go
        // negative — widening past the goal line still counts as novel).
        std::vector<ExprID> novel_psi;
        std::vector<std::pair<ExprID, double>> novel_delta;
        // ALL subgoal conditions satisfied at the frontier (superset of
        // novel_psi: includes satisfied numeric comparisons) — the state
        // oracle for the per-commit landmark recomputation.
        std::vector<ExprID> sat_conditions;

        int unachieved_landmarks() const { return static_cast<int>(unachieved_idx.size()); }
    };

    /// A materialised commitment: one node of the search tree.  The node's
    /// anchor is its committed frontier; its ladder state (window, masks
    /// high-water, per-frontier guards, generated-state store) lives here so
    /// a paused episode resumes exactly where it left off.
    struct Node {
        int parent;        // node index; -1 for the root
        int depth;         // commit edges from the root
        z3::expr guard;    // c_k — fixes this window's actions (root: true)
        z3::expr prefix;   // p_k <-> c_k /\ p_parent (root: true)
        int t_begin;       // committed window [t_begin, t_end) (root: [0, 0))
        int t_end;         // frontier position == this node's anchor timestep
        Candidate anchor;

        // Ladder state at this anchor.
        int window;
        int masked_upto;   // high-water mark of asserted mask offsets
        std::map<int, z3::expr> gen_guards;    // per-frontier generation guards
        std::map<int, z3::expr> steer_guards;  // per-frontier landmark-frontier steering

        // Landmarks of the REMAINING task, recomputed from this node's
        // anchor state at commit time (root: the preprocessing artifact).
        // Every landmark in it is open at the anchor by construction; its
        // orderings drive this node's frontier/steering/ordering clauses,
        // and its depth bound mutes goal probes the counts refute.
        std::shared_ptr<const LandmarkData> lm;
    };

    /// An unexplored stepping stone: a scored viable candidate that was not
    /// committed when generated.  Materialised into a Node if and when the
    /// best-first jump selects it.
    struct OpenEntry {
        int parent;  // node whose anchor generated it
        int t_f;
        Candidate cand;
    };

    /// Strict "a scores better than b" under one of two lexicographic
    /// orders over (h^FF, open landmarks) — LAMA-style, the pool alternates
    /// them per jump; after the pair: the admissible RPG bound, tiebreaks.
    static bool better(const Candidate& a, const Candidate& b, bool lms_first);

    // ---- Loop stages (see the design record, §3–§6) ----

    /// Add transitions for [filled_up_to_, upto) — permanent, no prefix-monotone.
    void fill_transitions(int upto);

    /// Assumption-gated goal literal for horizon h (asserted once, reused).
    z3::expr goal_literal(int h);

    /// Frame guards along the path from node `id` to the root (root excluded).
    z3::expr_vector path_assumptions(int id) const;

    /// path_assumptions(current_): the live prefix.
    z3::expr_vector commit_assumptions() const;

    /// Read the state at timestep t from the model: dynamic fluents from the
    /// model (model_completion), statics from the initial state.  Also
    /// returns the dynamic fluents' model value exprs for blocking clauses.
    StateValuation extract_state(const z3::model& model, int t,
                                 std::vector<z3::expr>& dynamic_values) const;

    /// Read the fired actions per step over [t_begin, t_end) from the model.
    std::vector<std::vector<const Action*>> extract_segment(
        const z3::model& model, int t_begin, int t_end, int& nonempty_steps) const;

    /// Build the anchor-state RPG for a candidate: fills lb (or +inf veto),
    /// first_layers, rpg layer info.  Returns true if the candidate is viable.
    bool score_candidate(Candidate& cand) const;

    /// state_eq(t, values) over the dynamic fluents (for blocking clauses).
    z3::expr state_equals(int t, const std::vector<z3::expr>& values);

    /// Landmark indices (into `data`) whose condition is unsatisfied in the
    /// model at t (exact: conditions are static-free after StaticFluentPass).
    std::vector<int> unachieved_landmark_indices(const z3::model& model, int t,
                                                 const LandmarkData& data) const;

    /// Re-run the landmark closure from a committed anchor state (§5.4c of
    /// the design doc; "landmarks of the remaining task").  State oracle:
    /// cand.sat_conditions for condition satisfaction, cand.frontier for
    /// numeric fluent values (statics carry their initial values).
    std::shared_ptr<const LandmarkData> recompute_landmarks(
        const Candidate& cand) const;

    // ---- Partitioned numeric novelty, à la Panino (§5.4).  Features are
    // the subgoal conditions the h^FF plumbing already walks (achiever
    // conditions + goal conditions); numeric comparisons contribute a
    // margin δ (satisfied margins negative), everything else a boolean ψ.
    // Novelty-1 within the partition keyed by the score pair
    // (h^FF, open landmarks): some ψ achieved for the first time, or some
    // δ strictly below the running minimum.  Pure bookkeeping — never
    // expressed as solver assumptions. ----

    /// Fill cand.novel_psi / cand.novel_delta from the model at t.
    void extract_novelty_features(const z3::model& model, int t,
                                  Candidate& cand) const;

    /// Novelty-1 test against the current tables (no table update).
    bool is_novel(const Candidate& cand) const;

    /// Fold the candidate's features into the partition tables.
    void update_novelty_tables(const Candidate& cand);

    /// Achiever plumbing for h^FF (mirrors CGBPlanner::initialize_achievers).
    void initialize_achievers();

    /// FF-style relaxed-plan action count from the state in `model` at t,
    /// with reachability layers from the *anchor-state* RPG (the candidate's
    /// first_layers).  The heuristic-search h^FF: inadmissible, but far more
    /// discriminating than the RPG's h_max-style depth bound.  If depth_out
    /// is non-null it receives the relaxed plan's parallel DEPTH (max first
    /// layer among its actions, 1-based) — the step-count estimate of the
    /// remaining task, the right unit under exists-step parallelism.
    int relaxed_plan_size(const z3::model& model, int t,
                          const std::unordered_map<int, int>& first_layers,
                          int* depth_out = nullptr) const;

    /// Ordering-minimal open landmarks: the antichain of "what can come
    /// next".  A landmark is on the frontier iff no OPEN landmark is
    /// ordered before it (orderings are sound — §4.2c asserts them as hard
    /// guarded clauses).  Indices are relative to `data`.
    std::vector<int> landmark_frontier(const std::vector<int>& open_idx,
                                       const LandmarkData& data) const;

    /// The frontier target: "some achiever of some frontier landmark fires
    /// in [t_begin, t_end)".  Empty frontier -> true (no steering; plain
    /// diversity, e.g. near the goal or landmark-free problems).
    z3::expr steering_disjunction(const std::vector<int>& frontier_idx,
                                  int t_begin, int t_end,
                                  const LandmarkData& data);

    /// Assert anchor-RPG action masks for window offsets [begin_off, end_off)
    /// from the anchor at t_anchor, under `guard` (nullptr = permanent, root
    /// anchor): an action first relaxed-reachable at offset f cannot fire at
    /// offsets < f; actions the RPG never reached are masked everywhere when
    /// the RPG ran to fixpoint.  The offset range makes WIDEN extensions
    /// incremental.
    void assert_masks(const z3::expr* guard, int t_anchor, int begin_off, int end_off,
                      const std::unordered_map<int, int>& first_layers,
                      int rpg_action_layers, bool saw_all_actions);

    /// §4.2c landmark ordering clauses for window offsets [begin_off,
    /// end_off): a later-landmark achiever firing at t needs an earlier-
    /// landmark achiever in [t_anchor, t] (inclusive — exists-step allows
    /// same-step chains), for ordering pairs whose earlier landmark is open
    /// at the anchor.  Same guard discipline and extension pattern as masks.
    void assert_ordering_clauses(const z3::expr* guard, int t_anchor,
                                 int begin_off, int end_off,
                                 const std::vector<int>& unachieved_idx,
                                 const LandmarkData& data);

    /// Materialise a commitment: fix `cand`'s window [parent.t_end, t_f)
    /// under a fresh guard pair, assert the new anchor's masks/orderings
    /// under the prefix guard, push the node.  Returns the new node's index
    /// (the caller re-points current_).
    int make_node(int parent, int t_f, Candidate cand);

    /// Pop the globally best open candidate (skipping entries dominated by
    /// a strictly shallower closed-list twin) and materialise it as the new
    /// current node.  Returns false if no eligible entry exists.
    bool pop_pool();

    /// Deepest (max-depth) node on the current path named by the unsat
    /// core, or -1 if the core names no commitment guard.
    int deepest_core_node() const;

    /// Log + write + notify a found plan; sets solution_found_.
    Plan finish_with_plan(const z3::model& model, int horizon);

    // ---- The single loop (docs/windowed-commitment-v2.md §3–§4) ----

    enum class RungResult {
        PlanFound,  // found_plan_ set
        Progress,   // committed, widened, or jumped: loop
        Dead,       // no candidates, no room, empty pool: greedy exhausted
        Timeout     // wall deadline hit
    };

    /// One rung at current_: PROBE the goal at the frontier (skipped when
    /// the complete sweep already proved t_f unsolvable), then one GENERATE
    /// round under the landmark-frontier target, then the escalation ladder
    /// (EHC commit > core-directed pool jump > widen > pool jump).  A
    /// generation UNSAT whose core names no commitment guard means no
    /// frontier crossing fits in w steps — the widen signal.
    RungResult expand_rung();

    /// One els check at cp_horizon_ on the COLD side solver (prefix-
    /// monotone, own propagator, resumable across calls with learned
    /// clauses retained).  Cold, not on the shared ledger: measured E4 —
    /// the same query on the greedy-filled formula lost 3 small-suite
    /// instances to clause/depth poison (sugar h=12: ~14 s shared vs
    /// 0.35 s cold).  UNSAT at h proves no plan of length <= h
    /// (prefix-monotone preserves satisfiability), advances the schedule
    /// and raises unsat_horizon_, muting greedy probes at <= h.  Called
    /// once per loop iteration — event-driven round-robin, no clocks.
    /// Returns true iff a plan was found.
    bool run_complete_probe();

    // ---- Static problem data ----

    std::vector<ExprID> dynamic_fluent_eids_;  // targeted by some effect
    std::vector<int> dynamic_fluent_ids_;      // parallel grounded ids
    StateValuation base_valuation_;            // full valuation of the initial state
    LandmarkData landmark_data_;               // copied artifact (may be empty)
    std::unordered_map<int, const Action*> action_by_id_;  // achiever ids -> actions

    // ---- Achiever data for h^FF (state-independent structure; per-anchor
    // layers come from each candidate's RPG) ----
    std::unique_ptr<AchieversAnalysis> achievers_;
    std::unordered_map<ExprID, std::vector<const Action*>> condition_achievers_;
    std::vector<ExprID> goal_condition_ids_;
    std::unordered_map<const Action*, std::vector<ExprID>> action_preconditions_;

    // ---- Search state ----

    std::vector<Node> nodes_;    // the commitment tree; nodes_[0] is the root
    int current_ = 0;            // the node whose anchor the dive extends
    std::vector<OpenEntry> open_;  // global best-first pool of stepping stones
    std::unordered_map<unsigned, int> guard_id_to_node_;  // z3 expr id -> node

    std::map<int, z3::expr> goal_literals_;
    int filled_up_to_ = 0;
    int next_guard_ = 0;   // fresh-name counters (guards are never reused)
    int next_round_ = 0;
    int pool_jumps_ = 0;

    // Two independent EHC ratchets (LAMA-style): each signal tracks its own
    // best-ever committed value; a candidate improving EITHER commits.
    // Measured against the best ever, not the current anchor — otherwise
    // plateau commits degrade the anchor and "beating" the degraded anchor
    // keeps resetting the plateau (measured on sugar/pfile02: endless churn).
    int best_h_ff_ = 0;
    int best_unachieved_ = 0;
    bool pool_lms_turn_ = true;  // alternation state for pop_pool()

    // Greedy-informed sweep target: current anchor's t_end + its relaxed
    // plan's parallel depth — the dive's own estimate of total plan length
    // in steps.  The complete sweep jumps forward to it (prefix-monotone:
    // one check at H covers every horizon <= H), converting its workload
    // from the UNSAT wall to the greedy's plausibly-SAT question.
    int greedy_estimate_ = 0;

    // Closed list (M1 + cross-anchor re-blocking): shallowest frontier depth
    // at which each state was generated, plus its dynamic-fluent model values
    // so new generation guards can block it BEFORE the SAT check (a pre-check
    // block is free; the post-check dup skip below costs one hard query).  A
    // state re-reached at >= its recorded depth is a duplicate — its first
    // occurrence owns the subtree.
    struct SeenState {
        int depth;
        std::vector<z3::expr> values;  // aligned with dynamic_fluent_eids_
        bool dead = false;  // vetoed relaxed-dead: dead at ANY depth
    };
    std::unordered_map<size_t, SeenState> seen_states_;

    // Panino novelty tables (§5.4): per-partition seen ψ features and
    // running δ minima.  Partition key = (h^FF, open landmarks).
    using NoveltyPartition = std::pair<int, int>;
    std::map<NoveltyPartition, std::unordered_set<ExprID>> psi_seen_;
    std::map<NoveltyPartition, std::unordered_map<ExprID, double>> delta_min_;
    std::vector<ExprID> novelty_feature_ids_;  // achiever conditions + goals

    Plan found_plan_;  // set on PlanFound

    int cp_horizon_ = 0;        // next horizon for the complete sweep
    int unsat_horizon_ = -1;    // deepest T proven "no plan of length <= T"
    bool greedy_exhausted_ = false;
    bool complete_exhausted_ = false;

    // Cold complete-side solver (lazily built on first use).
    std::unique_ptr<z3::solver> c_solver_;
    std::unique_ptr<PropagatorStrategy> c_propagator_;
    std::map<int, z3::expr> c_goal_literals_;
    int c_filled_ = 0;
    bool c_started_ = false;

    // Phase hints (warm start, §5.2): the dive's committed prefix fed to
    // the cold solver as decide-phase hints.  Arm A (absolute-time prefix
    // hints, RANTANPLAN_WC_PHASE_ARM=prefix) or arm C (order-based dynamic
    // hints, the default).  Owned by c_propagator_; raw pointers for hint
    // refreshes.  Disable entirely via RANTANPLAN_WC_NO_PHASE_HINTS.
    PhaseHintModule* c_phase_hints_ = nullptr;
    OrderHintModule* c_order_hints_ = nullptr;
    int c_hint_stamp_ = -1;  // current_ value the installed hints reflect

    std::chrono::steady_clock::time_point search_start_;

    /// check() under the live commitments plus `extra`.  Uncapped except by
    /// the user's wall deadline, which only truncates (never a search
    /// decision).
    z3::check_result commit_check(const z3::expr_vector& extra);
};

} // namespace rantanplan
