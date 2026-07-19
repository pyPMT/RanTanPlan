#include "windowed_commitment_planner.hpp"
#include "../analysis/numeric_relaxed_planning_graph.hpp"
#include "../passes/landmark_analysis_pass.hpp"
#include "../preprocessing/preprocessed_problem.hpp"
#include "propagators/modules/phase_hint_module.hpp"
#include <cstdlib>
#include "../config/config.hpp"
#include "../util/logger.hpp"
#include "../util/stats.hpp"
#include "../util/memory_tracker.hpp"
#include <algorithm>
#include <cassert>
#include <climits>
#include <cmath>
#include <numeric>
#include <iterator>
#include <string>
#include <unordered_set>

namespace rantanplan {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

/// Model numeral -> double (mirrors NumericRelaxedPlanningGraph's private
/// extract_numeric_value: int fast path, then "num/den" rationals, then
/// decimal strings).
double numeral_to_double(const z3::expr& v) {
    int64_t i64 = 0;
    if (v.is_numeral_i64(i64)) {
        return static_cast<double>(i64);
    }
    std::string s = v.to_string();
    size_t slash = s.find('/');
    try {
        if (slash != std::string::npos) {
            double num = std::stod(s.substr(0, slash));
            double den = std::stod(s.substr(slash + 1));
            return den != 0.0 ? num / den : 0.0;
        }
        return std::stod(s);
    } catch (const std::exception&) {
        return 0.0;
    }
}

} // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

WindowedCommitmentPlanner::WindowedCommitmentPlanner(const Problem& problem,
                                                     BaseEncoder& encoder,
                                                     z3::context& ctx)
    : BasePlanner(problem, encoder, ctx) {
    // Dynamic ground fluents: those targeted by some action effect.  Only
    // these have Z3 state variables at every timestep (frame axioms create
    // them); statics are read from the initial state instead.
    std::unordered_set<int> dynamic_ids;
    for (const Action& action : problem_.actions()) {
        for (const Effect& effect : action.effects()) {
            int fid = problem_.find_grounded_fluent_index(
                effect.effect_expression().fluent_id());
            if (fid >= 0) {
                dynamic_ids.insert(fid);
            }
        }
    }
    dynamic_fluent_ids_.assign(dynamic_ids.begin(), dynamic_ids.end());
    std::sort(dynamic_fluent_ids_.begin(), dynamic_fluent_ids_.end());
    dynamic_fluent_eids_.reserve(dynamic_fluent_ids_.size());
    for (int fid : dynamic_fluent_ids_) {
        dynamic_fluent_eids_.push_back(problem_.grounded_fluent(fid));
    }

    // Full valuation of the (CWA-completed) initial state, the source for
    // static fluent values in every extracted state.
    const auto& pool = problem_.pool();
    base_valuation_.values.assign(problem_.grounded_fluent_count(),
                                  StateValuation::FluentValue{});
    for (const auto& assignment : problem_.initial_state()) {
        ExprID fluent_eid = assignment.fluent_id();
        int fid = problem_.find_grounded_fluent_index(fluent_eid);
        if (fid < 0) continue;
        ExprID val_eid = assignment.value_id();
        if (problem_.is_bool_type(fluent_eid)) {
            bool v = pool.is_constant(val_eid) && pool.payload_is_bool(val_eid) &&
                     pool.payload_bool(val_eid);
            base_valuation_.values[fid] = StateValuation::FluentValue::make_bool(v);
        } else if (problem_.is_object_type(fluent_eid)) {
            base_valuation_.values[fid] = StateValuation::FluentValue::make_object(
                problem_.object_constant_index(val_eid));
        } else if (problem_.is_numeric_type(fluent_eid)) {
            double v = 0.0;
            if (pool.is_constant(val_eid)) {
                if (pool.payload_is_int(val_eid)) {
                    v = static_cast<double>(pool.payload_int(val_eid));
                } else if (pool.payload_is_double(val_eid)) {
                    v = pool.payload_double(val_eid);
                }
            }
            base_valuation_.values[fid] = StateValuation::FluentValue::make_numeric(v);
        }
    }

    for (const Action& action : problem_.actions()) {
        action_by_id_[action.id()] = &action;
    }
}

void WindowedCommitmentPlanner::consume_artifacts(PreprocessedProblem& artifacts) {
    // Landmarks by copy; the achievers analysis by move (the CGB pattern —
    // one planner per solve, and it is rebuilt locally when absent).
    landmark_data_ = artifacts.landmark_data;
    if (artifacts.achievers) {
        achievers_ = std::move(artifacts.achievers);
    }
}

void WindowedCommitmentPlanner::initialize_achievers() {
    if (!achievers_) {
        achievers_ = std::make_unique<AchieversAnalysis>(problem_);
    }
    for (ExprID cond : achievers_->get_all_conditions()) {
        auto& ptrs = condition_achievers_[cond];
        for (const Action& a : achievers_->get_achievers(cond)) {
            auto it = action_by_id_.find(a.id());
            if (it != action_by_id_.end()) {
                ptrs.push_back(it->second);
            }
        }
    }
    for (ExprID cond : achievers_->get_goal_conditions()) {
        goal_condition_ids_.push_back(cond);
    }
    for (const Action& a : problem_.actions()) {
        const auto& precs = achievers_->get_preconditions(a);
        if (!precs.empty()) {
            action_preconditions_[action_by_id_.at(a.id())]
                .assign(precs.begin(), precs.end());
        }
    }

    // Panino novelty feature space (§5.4): the subgoal conditions —
    // achiever conditions plus goal conditions, deduplicated.
    std::unordered_set<ExprID> feature_set;
    for (const auto& [cond, ptrs] : condition_achievers_) {
        feature_set.insert(cond);
    }
    for (ExprID cond : goal_condition_ids_) {
        feature_set.insert(cond);
    }
    novelty_feature_ids_.assign(feature_set.begin(), feature_set.end());
}

// ---------------------------------------------------------------------------
// Partitioned numeric novelty (Panino, §5.4)
// ---------------------------------------------------------------------------

void WindowedCommitmentPlanner::extract_novelty_features(
    const z3::model& model, int t, Candidate& cand) const {
    cand.novel_psi.clear();
    cand.novel_delta.clear();
    cand.sat_conditions.clear();
    for (ExprID g : novelty_feature_ids_) {
        z3::expr cond = encoder_.convert_expr_id_to_z3(g, t);
        Z3_decl_kind kind =
            cond.is_app() ? cond.decl().decl_kind() : Z3_OP_UNINTERPRETED;
        bool numeric_cmp =
            (kind == Z3_OP_GE || kind == Z3_OP_GT || kind == Z3_OP_LE ||
             kind == Z3_OP_LT ||
             (kind == Z3_OP_EQ && cond.arg(0).is_arith()));
        if (numeric_cmp) {
            double lhs = numeral_to_double(model.eval(cond.arg(0), true));
            double rhs = numeral_to_double(model.eval(cond.arg(1), true));
            // Distance-to-satisfaction, satisfied margins negative: widening
            // the margin past the goal line still registers as progress.
            double delta = (kind == Z3_OP_GE || kind == Z3_OP_GT)
                               ? rhs - lhs
                               : (kind == Z3_OP_EQ ? std::abs(lhs - rhs)
                                                   : lhs - rhs);
            cand.novel_delta.emplace_back(g, delta);
            if (model.eval(cond, /*model_completion=*/true).is_true()) {
                cand.sat_conditions.push_back(g);
            }
        } else if (model.eval(cond, /*model_completion=*/true).is_true()) {
            cand.novel_psi.push_back(g);
            cand.sat_conditions.push_back(g);
        }
    }
}

std::shared_ptr<const LandmarkData> WindowedCommitmentPlanner::recompute_landmarks(
    const Candidate& cand) const {
    std::unordered_set<ExprID> sat(cand.sat_conditions.begin(),
                                   cand.sat_conditions.end());
    const StateValuation& state = cand.frontier;
    auto data = std::make_shared<LandmarkData>(LandmarkAnalysisPass::compute(
        problem_, *achievers_,
        [&](ExprID cond) { return sat.count(cond) > 0; },
        [&](ExprID fluent) -> std::optional<double> {
            int fid = problem_.find_grounded_fluent_index(fluent);
            if (fid < 0 || fid >= static_cast<int>(state.values.size())) {
                return std::nullopt;
            }
            const auto& v = state.values[static_cast<size_t>(fid)];
            if (v.kind != StateValuation::Kind::Numeric) return std::nullopt;
            return v.numeric_value;
        }));
    return data;
}

bool WindowedCommitmentPlanner::is_novel(const Candidate& cand) const {
    NoveltyPartition key{cand.h_ff, cand.unachieved_landmarks()};
    auto psi_it = psi_seen_.find(key);
    if (psi_it == psi_seen_.end()) {
        // First sample of this partition is novel by definition.
        return !cand.novel_psi.empty() || !cand.novel_delta.empty();
    }
    for (ExprID g : cand.novel_psi) {
        if (!psi_it->second.count(g)) return true;
    }
    auto delta_it = delta_min_.find(key);
    for (const auto& [g, delta] : cand.novel_delta) {
        if (delta_it == delta_min_.end()) return true;
        auto min_it = delta_it->second.find(g);
        if (min_it == delta_it->second.end() || delta < min_it->second) {
            return true;
        }
    }
    return false;
}

void WindowedCommitmentPlanner::update_novelty_tables(const Candidate& cand) {
    NoveltyPartition key{cand.h_ff, cand.unachieved_landmarks()};
    auto& psi = psi_seen_[key];
    for (ExprID g : cand.novel_psi) {
        psi.insert(g);
    }
    auto& mins = delta_min_[key];
    for (const auto& [g, delta] : cand.novel_delta) {
        auto [it, inserted] = mins.emplace(g, delta);
        if (!inserted && delta < it->second) {
            it->second = delta;
        }
    }
}

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

bool WindowedCommitmentPlanner::better(const Candidate& a, const Candidate& b,
                                       bool lms_first) {
    // Two lexicographic orders over the same signal pair, LAMA-style: the
    // A/B (2026-07-19) showed each is load-bearing in a different region —
    // the landmark count where goals decompose into many enumerable
    // landmarks (transport: the count IS the todo list), h^FF where the
    // landmark structure is coarse (plotting, factory-robot: the count is a
    // step function and only the relaxed-plan size discriminates).  Neither
    // fixed order is right for both, so the pool alternates them per jump.
    // After the pair: the admissible RPG bound (which the sound veto still
    // owns), then tiebreaks.  The spec's original LB-first order measured
    // as flat noise dominating both informative signals (§4.3 correction).
    int a1 = lms_first ? a.unachieved_landmarks() : a.h_ff;
    int b1 = lms_first ? b.unachieved_landmarks() : b.h_ff;
    int a2 = lms_first ? a.h_ff : a.unachieved_landmarks();
    int b2 = lms_first ? b.h_ff : b.unachieved_landmarks();
    if (a1 != b1) return a1 < b1;
    if (a2 != b2) return a2 < b2;
    if (a.lb != b.lb) return a.lb < b.lb;
    if (a.goals_satisfied != b.goals_satisfied) return a.goals_satisfied > b.goals_satisfied;
    return a.nonempty_steps < b.nonempty_steps;
}

void WindowedCommitmentPlanner::fill_transitions(int upto) {
    for (int t = filled_up_to_; t < upto; ++t) {
        // No prefix-monotone in the greedy lane: the padding argument (one
        // probe at the frontier witnesses all shorter extensions via empty
        // steps) needs empties anywhere, and front-loading would contradict
        // commitments.  Lane B asserts it on its own solver.
        BasePlanner::add_timestep_constraints(solver_, encoder_, *propagator_strategy_,
                                              t, /*prefix_monotone=*/false);
    }
    if (upto > filled_up_to_) {
        filled_up_to_ = upto;
    }
}

z3::expr WindowedCommitmentPlanner::goal_literal(int h) {
    auto it = goal_literals_.find(h);
    if (it != goal_literals_.end()) {
        return it->second;
    }
    std::string name = "wc_goal_at_" + std::to_string(h);
    z3::expr lit = ctx_.bool_const(name.c_str());
    solver_.add(z3::implies(lit, encoder_.encode_goal(h)));
    goal_literals_.emplace(h, lit);
    return lit;
}

z3::expr_vector WindowedCommitmentPlanner::path_assumptions(int id) const {
    z3::expr_vector assumptions(ctx_);
    for (int n = id; n > 0; n = nodes_[n].parent) {
        assumptions.push_back(nodes_[n].guard);
    }
    return assumptions;
}

z3::expr_vector WindowedCommitmentPlanner::commit_assumptions() const {
    return path_assumptions(current_);
}

z3::check_result WindowedCommitmentPlanner::commit_check(
    const z3::expr_vector& extra) {
    if (remaining_ms() == 0) {
        timed_out_ = true;
        return z3::unknown;
    }
    apply_solver_timeout(solver_);  // the user's wall deadline; truncation only

    z3::expr_vector assumptions = commit_assumptions();
    for (unsigned i = 0; i < extra.size(); ++i) {
        assumptions.push_back(extra[i]);
    }
    z3::check_result result = solver_.check(assumptions);
    if (result == z3::unknown && remaining_ms() == 0) {
        timed_out_ = true;
    }
    return result;
}

// ---------------------------------------------------------------------------
// State extraction and scoring
// ---------------------------------------------------------------------------

StateValuation WindowedCommitmentPlanner::extract_state(
    const z3::model& model, int t, std::vector<z3::expr>& dynamic_values) const {
    StateValuation state = base_valuation_;  // statics from the initial state
    dynamic_values.clear();
    dynamic_values.reserve(dynamic_fluent_eids_.size());
    for (size_t i = 0; i < dynamic_fluent_eids_.size(); ++i) {
        ExprID eid = dynamic_fluent_eids_[i];
        int fid = dynamic_fluent_ids_[i];
        z3::expr var = encoder_.convert_expr_id_to_z3(eid, t);
        // Frame axioms functionally determine every dynamic fluent at t;
        // completion is belt-and-braces for irrelevant ones.
        z3::expr val = model.eval(var, /*model_completion=*/true);
        if (problem_.is_bool_type(eid)) {
            state.values[fid] = StateValuation::FluentValue::make_bool(val.is_true());
        } else if (problem_.is_object_type(eid)) {
            int64_t idx = -1;
            if (!val.is_numeral_i64(idx)) idx = -1;
            state.values[fid] =
                StateValuation::FluentValue::make_object(static_cast<int>(idx));
        } else {
            state.values[fid] =
                StateValuation::FluentValue::make_numeric(numeral_to_double(val));
        }
        dynamic_values.push_back(val);
    }
    return state;
}

std::vector<std::vector<const Action*>> WindowedCommitmentPlanner::extract_segment(
    const z3::model& model, int t_begin, int t_end, int& nonempty_steps) const {
    auto& factory = encoder_.get_variable_factory();
    std::vector<std::vector<const Action*>> segment(t_end - t_begin);
    nonempty_steps = 0;
    for (int t = t_begin; t < t_end; ++t) {
        auto& step = segment[t - t_begin];
        for (const Action& action : problem_.actions()) {
            const z3::expr& av = factory.get_action_variable(action, t);
            if (model.eval(av, /*model_completion=*/true).is_true()) {
                step.push_back(&action);
            }
        }
        if (!step.empty()) {
            nonempty_steps++;
        }
    }
    return segment;
}

bool WindowedCommitmentPlanner::score_candidate(Candidate& cand) const {
    NumericRelaxedPlanningGraph rpg(problem_, cand.frontier);
    bool goals_reachable = rpg.build();
    cand.first_layers = rpg.get_action_first_layers();
    cand.rpg_action_layers = static_cast<int>(rpg.get_layer_count()) - 1;
    cand.rpg_saw_all_actions = cand.first_layers.size() == problem_.actions().size();
    if (!goals_reachable) {
        cand.lb = kInf;  // sound dead-end veto: relaxed-unreachable => unreachable
        return false;
    }
    int lb = rpg.get_minimum_steps_lower_bound();
    cand.lb = lb < 0 ? kInf : static_cast<double>(lb);
    return cand.lb < kInf;
}

z3::expr WindowedCommitmentPlanner::state_equals(int t,
                                                 const std::vector<z3::expr>& values) {
    z3::expr_vector equalities(ctx_);
    for (size_t i = 0; i < dynamic_fluent_eids_.size(); ++i) {
        z3::expr var = encoder_.convert_expr_id_to_z3(dynamic_fluent_eids_[i], t);
        equalities.push_back(var == values[i]);
    }
    if (equalities.empty()) {
        return ctx_.bool_val(true);
    }
    return z3::mk_and(equalities);
}

std::vector<int> WindowedCommitmentPlanner::unachieved_landmark_indices(
    const z3::model& model, int t, const LandmarkData& data) const {
    std::vector<int> indices;
    for (size_t i = 0; i < data.landmarks.size(); ++i) {
        const LandmarkInfo& lm = data.landmarks[i];
        if (lm.condition == EXPR_NULL) continue;
        z3::expr cond = encoder_.convert_expr_id_to_z3(lm.condition, t);
        if (!model.eval(cond, /*model_completion=*/true).is_true()) {
            indices.push_back(static_cast<int>(i));
        }
    }
    return indices;
}

int WindowedCommitmentPlanner::relaxed_plan_size(
    const z3::model& model, int t,
    const std::unordered_map<int, int>& first_layers, int* depth_out) const {
    // FF-style backward extraction, mirroring CGBPlanner::extract_relaxed_plan
    // with two substitutions: reachability layers come from the anchor-state
    // RPG (the candidate's first_layers) instead of the init-state ARPG, and
    // "satisfied initially" is evaluated in the candidate's model at t.
    std::unordered_map<ExprID, bool> satisfied_cache;
    auto satisfied_at_state = [&](ExprID cond) {
        auto it = satisfied_cache.find(cond);
        if (it != satisfied_cache.end()) return it->second;
        z3::expr ce = encoder_.convert_expr_id_to_z3(cond, t);
        bool sat = model.eval(ce, /*model_completion=*/true).is_true();
        satisfied_cache.emplace(cond, sat);
        return sat;
    };

    std::unordered_set<const Action*> plan;
    std::unordered_set<ExprID> unsatisfied;
    for (ExprID g : goal_condition_ids_) {
        if (!satisfied_at_state(g)) unsatisfied.insert(g);
    }
    int max_layer = 0;
    for (const auto& [action_id, layer] : first_layers) {
        max_layer = std::max(max_layer, layer);
    }
    for (int layer = max_layer; layer >= 0 && !unsatisfied.empty(); --layer) {
        std::vector<ExprID> resolved;
        std::vector<ExprID> new_subgoals;
        for (ExprID cond : unsatisfied) {
            auto ach_it = condition_achievers_.find(cond);
            if (ach_it == condition_achievers_.end()) continue;
            const Action* best = nullptr;
            int best_layer = -1;
            for (const Action* a : ach_it->second) {
                auto fl = first_layers.find(a->id());
                if (fl == first_layers.end()) continue;  // unreachable from anchor
                if (fl->second <= layer && fl->second > best_layer) {
                    best = a;
                    best_layer = fl->second;
                }
            }
            if (!best) continue;
            resolved.push_back(cond);
            if (plan.insert(best).second) {
                auto pre_it = action_preconditions_.find(best);
                if (pre_it != action_preconditions_.end()) {
                    for (ExprID pre : pre_it->second) {
                        if (!satisfied_at_state(pre)) new_subgoals.push_back(pre);
                    }
                }
            }
        }
        for (ExprID cond : resolved) unsatisfied.erase(cond);
        for (ExprID pre : new_subgoals) unsatisfied.insert(pre);
    }
    if (depth_out) {
        int depth = 0;
        for (const Action* a : plan) {
            auto fl = first_layers.find(a->id());
            if (fl != first_layers.end()) depth = std::max(depth, fl->second);
        }
        *depth_out = depth;
    }
    return static_cast<int>(plan.size());
}

std::vector<int> WindowedCommitmentPlanner::landmark_frontier(
    const std::vector<int>& open_idx, const LandmarkData& data) const {
    // The 1UIP picture: the ordering-minimal open landmarks form the
    // antichain of "what can come next" — a cut dominating the goal.  A
    // landmark is on the frontier iff no OPEN landmark is ordered before
    // it.  (Achieved predecessors do not block: their obligation is met.)
    std::unordered_set<int> open(open_idx.begin(), open_idx.end());
    std::unordered_set<int> blocked;
    for (const auto& [earlier, later] : data.orderings) {
        if (open.count(earlier) && open.count(later)) {
            blocked.insert(later);
        }
    }
    std::vector<int> frontier;
    for (int idx : open_idx) {
        if (!blocked.count(idx)) {
            frontier.push_back(idx);
        }
    }
    return frontier;
}

z3::expr WindowedCommitmentPlanner::steering_disjunction(
    const std::vector<int>& frontier_idx, int t_begin, int t_end,
    const LandmarkData& data) {
    auto& factory = encoder_.get_variable_factory();
    z3::expr_vector fires(ctx_);
    for (int idx : frontier_idx) {
        const LandmarkInfo& lm =
            data.landmarks[static_cast<size_t>(idx)];
        for (int action_id : lm.achiever_action_ids) {
            auto it = action_by_id_.find(action_id);
            if (it == action_by_id_.end()) continue;
            for (int t = t_begin; t < t_end; ++t) {
                fires.push_back(factory.get_action_variable(*it->second, t));
            }
        }
    }
    if (fires.empty()) {
        return ctx_.bool_val(true);
    }
    return z3::mk_or(fires);
}

// ---------------------------------------------------------------------------
// Masks, commits, backjumps
// ---------------------------------------------------------------------------

void WindowedCommitmentPlanner::assert_masks(
    const z3::expr* guard, int t_anchor, int begin_off, int end_off,
    const std::unordered_map<int, int>& first_layers, int rpg_action_layers,
    bool saw_all_actions) {
    // Delete-relaxation over-approximates reachability, so an action not
    // relaxed-reachable within f layers of the anchor RPG cannot fire at
    // offsets < f in any real execution.  first_layers values are 1-based
    // (action_layers_[0] is the empty pre-anchor entry), so an action first
    // applicable at layer f fires earliest at offset f-1.  Actions absent
    // from the map were unreachable in every built layer; when the RPG ran
    // past goal achievement to fixpoint (early_goal_termination off, the
    // default) that means unreachable outright — mask the whole window.
    bool absent_means_unreachable =
        saw_all_actions || !Config::instance().rpg.early_goal_termination;
    auto& factory = encoder_.get_variable_factory();
    int masked = 0;
    for (const Action& action : problem_.actions()) {
        auto it = first_layers.find(action.id());
        int first_off;
        if (it != first_layers.end()) {
            first_off = std::max(0, it->second - 1);
        } else if (absent_means_unreachable) {
            first_off = end_off;
        } else {
            first_off = std::max(0, rpg_action_layers - 1);
        }
        int stop = std::min(first_off, end_off);
        for (int off = begin_off; off < stop; ++off) {
            const z3::expr& av = factory.get_action_variable(action, t_anchor + off);
            solver_.add(guard ? z3::implies(*guard, !av) : !av);
            masked++;
        }
    }
    if (masked > 0) {
        Stats::instance().add("planner.wc_masked_literals", static_cast<double>(masked));
        Logger::instance().verbose("  masks: " + std::to_string(masked) +
                                   " action literals over offsets [" +
                                   std::to_string(begin_off) + ", " +
                                   std::to_string(end_off) + ")");
    }
}

void WindowedCommitmentPlanner::assert_ordering_clauses(
    const z3::expr* guard, int t_anchor, int begin_off, int end_off,
    const std::vector<int>& unachieved_idx, const LandmarkData& data) {
    if (data.orderings.empty() || unachieved_idx.empty()) return;
    std::unordered_set<int> open(unachieved_idx.begin(), unachieved_idx.end());
    auto& factory = encoder_.get_variable_factory();
    int asserted = 0;
    for (const auto& [earlier, later] : data.orderings) {
        if (!open.count(earlier)) continue;  // achieved at the anchor: vacuous
        const LandmarkInfo& lm_a =
            data.landmarks[static_cast<size_t>(earlier)];
        const LandmarkInfo& lm_b =
            data.landmarks[static_cast<size_t>(later)];
        // Cumulative disjunction of A-achiever firings over [t_anchor, t],
        // built incrementally; inclusive of t (same-step chains are legal
        // under exists-step semantics).
        z3::expr_vector a_fires(ctx_);
        for (int off = 0; off < end_off; ++off) {
            int t = t_anchor + off;
            for (int aid : lm_a.achiever_action_ids) {
                auto it = action_by_id_.find(aid);
                if (it != action_by_id_.end()) {
                    a_fires.push_back(factory.get_action_variable(*it->second, t));
                }
            }
            if (off < begin_off) continue;  // accumulated, already asserted
            z3::expr a_before =
                a_fires.empty() ? ctx_.bool_val(false) : z3::mk_or(a_fires);
            for (int bid : lm_b.achiever_action_ids) {
                auto it = action_by_id_.find(bid);
                if (it == action_by_id_.end()) continue;
                z3::expr clause = z3::implies(
                    factory.get_action_variable(*it->second, t), a_before);
                solver_.add(guard ? z3::implies(*guard, clause) : clause);
                asserted++;
            }
        }
    }
    if (asserted > 0) {
        Stats::instance().add("planner.wc_ordering_clauses",
                              static_cast<double>(asserted));
    }
}

int WindowedCommitmentPlanner::make_node(int parent, int t_f, Candidate cand) {
    auto& config = Config::instance();
    // Copies, not references: nodes_.push_back below may reallocate.
    int t_begin = nodes_[parent].t_end;
    int depth = nodes_[parent].depth + 1;
    z3::expr parent_prefix = nodes_[parent].prefix;

    std::string guard_name = "wc_c" + std::to_string(next_guard_);
    z3::expr guard = ctx_.bool_const(guard_name.c_str());
    // M2 prefix guard: p_k <-> c_k /\ p_{k-1}.  Assuming the path guards
    // c_1..c_k unit-propagates every p_j true, so facts hung on p_j activate;
    // a retired p_j is freely settable false, so its facts can never prune.
    // Anchor-derived facts (masks, orderings) go on p_k because they are
    // consequences of the WHOLE prefix — the anchor state is a function of
    // every commitment below, not of c_k alone.  This is what lets nodes
    // from different branches coexist soundly in one formula.
    std::string prefix_name = "wc_p" + std::to_string(next_guard_++);
    z3::expr prefix = ctx_.bool_const(prefix_name.c_str());
    solver_.add(prefix == (guard && parent_prefix));
    auto& factory = encoder_.get_variable_factory();

    // Total per-step fixing: every action variable in the window is a unit
    // under the guard, so the frontier is uniquely determined.
    for (int t = t_begin; t < t_f; ++t) {
        std::unordered_set<int> fired;
        for (const Action* action : cand.segment[t - t_begin]) {
            fired.insert(action->id());
        }
        for (const Action& action : problem_.actions()) {
            const z3::expr& av = factory.get_action_variable(action, t);
            solver_.add(z3::implies(guard, fired.count(action.id()) ? av : !av));
        }
    }

    // Landmarks of the REMAINING task, recomputed from the new anchor state
    // (informativeness gain: conditions that only became necessary under
    // this prefix surface as landmarks, with fresh orderings, deficits from
    // the anchor's numeric values, and a depth bound that mutes refuted
    // goal probes).  By construction every landmark is open at the anchor.
    std::shared_ptr<const LandmarkData> lm = recompute_landmarks(cand);

    // Ratchet update uses the GENERATING node's landmark frame (the one the
    // improving/plateau decision was made in) before re-indexing.
    // Two independent ratchets (LAMA-style): each signal tracks its own
    // best-ever, component-wise.
    best_h_ff_ = std::min(best_h_ff_, cand.h_ff);
    best_unachieved_ = std::min(best_unachieved_, cand.unachieved_landmarks());
    cand.unachieved_idx.resize(lm->landmarks.size());
    std::iota(cand.unachieved_idx.begin(), cand.unachieved_idx.end(), 0);

    // Greedy-informed sweep target: this dive's own estimate of total plan
    // length in steps (anchor timestep + relaxed-plan parallel depth).
    if (cand.h_depth > 0) {
        greedy_estimate_ = t_f + cand.h_depth;
    }

    // Informed window (instead of the blind fixed w0): the smallest window
    // at which the frontier steering can possibly fire is the minimum
    // first-RPG-layer over the frontier landmarks' achievers — asking a
    // smaller window is a solver query the relaxation already refutes.
    // Optimistic (relaxed), so widening still covers any shortfall;
    // wc_window stays as the floor stride.
    int window = config.planner.wc_window;
    {
        std::vector<int> front = landmark_frontier(cand.unachieved_idx, *lm);
        int informed = INT_MAX;
        for (int idx : front) {
            for (int aid :
                 lm->landmarks[static_cast<size_t>(idx)].achiever_action_ids) {
                auto it = cand.first_layers.find(aid);
                if (it != cand.first_layers.end()) {
                    informed = std::min(informed, it->second);
                }
            }
        }
        if (informed != INT_MAX) window = std::max(window, informed);
    }

    // Anchor-conditional facts for the NEXT window (the one this frontier
    // anchors) live under the prefix guard: they are consequences of the
    // entire committed prefix and retract with any part of it.
    assert_masks(&prefix, t_f, 0, window, cand.first_layers,
                 cand.rpg_action_layers, cand.rpg_saw_all_actions);
    assert_ordering_clauses(&prefix, t_f, 0, window,
                            cand.unachieved_idx, *lm);

#ifndef NDEBUG
    // §7.1 mask soundness: no action masked for this window (asserted under
    // the establishing anchor's data) fires in the committed segment.
    for (int t = t_begin; t < t_f; ++t) {
        for (const Action* action : cand.segment[t - t_begin]) {
            auto it = nodes_[parent].anchor.first_layers.find(action->id());
            if (it != nodes_[parent].anchor.first_layers.end()) {
                assert(t - t_begin >= std::max(0, it->second - 1) &&
                       "masked action in committed segment");
            }
        }
    }
#endif

    Logger::instance().info(
        "  COMMIT [" + std::to_string(t_begin) + ", " + std::to_string(t_f) +
        "): h_ff=" + std::to_string(cand.h_ff) +
        ", lb=" + std::to_string(cand.lb) +
        ", lms=" + std::to_string(lm->landmarks.size()) +
        "(o=" + std::to_string(lm->orderings.size()) +
        ",d=" + std::to_string(lm->depth_lower_bound) + ")" +
        ", nonempty=" + std::to_string(cand.nonempty_steps) +
        ", depth=" + std::to_string(depth) +
        ", pool=" + std::to_string(open_.size()));
    Stats::instance().add("planner.wc_commits");

    int id = static_cast<int>(nodes_.size());
    guard_id_to_node_[guard.id()] = id;
    nodes_.push_back(Node{parent, depth, guard, prefix, t_begin, t_f,
                          std::move(cand), window,
                          /*masked_upto=*/window,
                          {}, {}, std::move(lm)});

#ifndef NDEBUG
    // §7.1 frontier determinism: the total fixing plus frames uniquely
    // determine the frontier — a fresh model under the new path must
    // reproduce the recorded valuation.  Debug builds only.
    z3::params p(ctx_);
    p.set("timeout", std::max(1u, remaining_ms()));
    solver_.set(p);
    if (solver_.check(path_assumptions(id)) == z3::sat) {
        std::vector<z3::expr> replay_values;
        StateValuation replayed =
            extract_state(solver_.get_model(), t_f, replay_values);
        assert(replayed == nodes_[id].anchor.frontier &&
               "committed frontier is not a function of the fixed actions");
    }
#endif
    return id;
}

int WindowedCommitmentPlanner::deepest_core_node() const {
    int deepest = -1;
    int deepest_depth = -1;
    z3::expr_vector core = solver_.unsat_core();
    for (unsigned i = 0; i < core.size(); ++i) {
        auto it = guard_id_to_node_.find(core[i].id());
        if (it != guard_id_to_node_.end() &&
            nodes_[it->second].depth > deepest_depth) {
            deepest = it->second;
            deepest_depth = nodes_[it->second].depth;
        }
    }
    return deepest;
}

bool WindowedCommitmentPlanner::pop_pool() {
    // LAMA-style alternation: successive jumps pick the pool's best under
    // alternating orders (lms-first, then h^FF-first, ...) so both signals
    // stay load-bearing in the region where each discriminates.
    bool lms_first = pool_lms_turn_;
    while (true) {
        // §5.4a: among equal-scored entries prefer the still-novel one,
        // re-evaluated against the CURRENT tables — a jump target in
        // territory the search has since covered stops attracting jumps.
        auto best = open_.end();
        bool best_novel = false;
        for (auto it = open_.begin(); it != open_.end(); ++it) {
            bool it_novel = is_novel(it->cand);
            if (best == open_.end() || better(it->cand, best->cand, lms_first) ||
                (!better(best->cand, it->cand, lms_first) &&
                 (it_novel > best_novel ||
                  (it_novel == best_novel && it->t_f < best->t_f)))) {
                best = it;
                best_novel = it_novel;
            }
        }
        if (best == open_.end()) {
            return false;
        }
        OpenEntry entry = std::move(*best);
        open_.erase(best);

        // Dominated: a strictly shallower twin of this state was generated
        // later elsewhere; per the closed-list doctrine the shallower
        // occurrence owns the subtree (same state => interchangeable
        // futures).
        auto seen = seen_states_.find(entry.cand.frontier.hash());
        if (seen != seen_states_.end() && seen->second.depth < entry.t_f) {
            Stats::instance().add("planner.wc_pool_dominated");
            continue;
        }

        pool_jumps_++;
        pool_lms_turn_ = !pool_lms_turn_;
        Stats::instance().add("planner.wc_pool_jumps");
        Logger::instance().info(
            "  JUMP to pool best (" + std::string(lms_first ? "lms" : "h_ff") +
            "-turn): parent node " + std::to_string(entry.parent) +
            " (depth " + std::to_string(nodes_[entry.parent].depth + 1) +
            "), t_f=" + std::to_string(entry.t_f) +
            ", h_ff=" + std::to_string(entry.cand.h_ff) +
            ", lms=" + std::to_string(entry.cand.unachieved_landmarks()) +
            ", pool=" + std::to_string(open_.size()));
        current_ = make_node(entry.parent, entry.t_f, std::move(entry.cand));
        return true;
    }
}

// ---------------------------------------------------------------------------
// Terminal paths
// ---------------------------------------------------------------------------

Plan WindowedCommitmentPlanner::finish_with_plan(const z3::model& model, int horizon) {
    auto& config = Config::instance();
    auto& stats = Stats::instance();
    double total_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - search_start_).count();
    solution_found_ = true;
    try {
        Plan plan = encoder_.extract_plan(model, horizon);
        Logger::instance().info("\n*** PLAN FOUND: horizon=" + std::to_string(horizon) +
            ", actions=" + std::to_string(plan.length()) +
            ", depth=" + std::to_string(nodes_.empty() ? 0 : nodes_[current_].depth) +
            ", pool_jumps=" + std::to_string(pool_jumps_) +
            " (total time: " + std::to_string(total_time) + "s) ***");

        plan.write_ipc(config.planner.output_plan, 1, -1.0, true,
                       config.planner.strategy, config.planner.mode, total_time);
        notify_intermediate_plan(plan);

        stats.set("planner.plan_length", static_cast<double>(plan.length()));
        stats.set("planner.solution_horizon", static_cast<double>(horizon));
        stats.set("planner.wc_commit_depth",
                  static_cast<double>(nodes_.empty() ? 0 : nodes_[current_].depth));
        stats.set("planner.total_time", total_time);
        collect_statistics();
        propagator_strategy_->cleanup();
        return plan;
    } catch (const std::exception& e) {
        Logger::instance().error("ERROR during plan extraction: " + std::string(e.what()));
        solution_found_ = false;
        collect_statistics();
        propagator_strategy_->cleanup();
        return Plan();
    }
}

// ---------------------------------------------------------------------------
// One greedy rung
// ---------------------------------------------------------------------------

WindowedCommitmentPlanner::RungResult WindowedCommitmentPlanner::expand_rung() {
    auto& config = Config::instance();
    auto& stats = Stats::instance();

    // Double the lookahead at the current anchor (clamped by the room below
    // max_steps).  Returns false when it cannot grow — the ladder's only
    // ceilings are the horizon room and the rung caps.
    auto double_lookahead = [&]() -> bool {
        Node& n = nodes_[current_];
        int room = config.planner.max_steps - n.t_end;
        if (n.window >= room) return false;
        n.window = std::min(n.window * 2, room);
        Logger::instance().info(
            "  LOOKAHEAD w=" + std::to_string(n.window) + " at anchor " +
            std::to_string(n.t_end));
        stats.add("planner.wc_widens");
        if (n.window > n.masked_upto) {
            const z3::expr* guard = current_ == 0 ? nullptr : &n.prefix;
            assert_masks(guard, n.t_end, n.masked_upto, n.window,
                         n.anchor.first_layers, n.anchor.rpg_action_layers,
                         n.anchor.rpg_saw_all_actions);
            assert_ordering_clauses(guard, n.t_end, n.masked_upto,
                                    n.window, n.anchor.unachieved_idx, *n.lm);
            n.masked_upto = n.window;
        }
        return true;
    };

    {
        if (remaining_ms() == 0) {
            timed_out_ = true;
            return RungResult::Timeout;
        }

        int anchor_t = nodes_[current_].t_end;
        int t_f = anchor_t + nodes_[current_].window;
        if (t_f > config.planner.max_steps) {
            t_f = config.planner.max_steps;
            if (t_f <= anchor_t) {
                // This anchor sits at the horizon limit; only the pool can
                // continue the greedy search.
                if (pop_pool()) return RungResult::Progress;
                Logger::instance().info("Greedy: frontier at max_steps.");
                return RungResult::Dead;
            }
            nodes_[current_].window = t_f - anchor_t;
        }
        fill_transitions(t_f);

        // ---- PROBE: one goal check at the frontier covers every horizon in
        // (anchor_t, t_f] via empty-step padding.  Skipped when the complete
        // sweep already proved no plan of length <= t_f exists, or when the
        // anchor's recomputed count landmarks refute the suffix room (the
        // depth bound is a sound "no goal within < d steps of the anchor").
        bool lm_muted =
            t_f - anchor_t < nodes_[current_].lm->depth_lower_bound;
        if (lm_muted) {
            Logger::instance().verbose(
                "  DBG probe t_f=" + std::to_string(t_f) +
                " muted by anchor landmark depth bound " +
                std::to_string(nodes_[current_].lm->depth_lower_bound));
        }
        if (!lm_muted && t_f >= config.planner.start_timestep &&
            t_f > unsat_horizon_) {
            z3::expr_vector extra(ctx_);
            extra.push_back(goal_literal(t_f));
            auto probe_start = std::chrono::steady_clock::now();
            z3::check_result result = commit_check(extra);
            Logger::instance().verbose(
                "  DBG probe t_f=" + std::to_string(t_f) + " result=" +
                (result == z3::sat ? "sat" : result == z3::unsat ? "unsat" : "unknown") +
                " time=" + std::to_string(std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - probe_start).count()) + "s");
            if (timed_out_) return RungResult::Timeout;
            if (result == z3::sat) {
                found_plan_ = finish_with_plan(solver_.get_model(), t_f);
                return RungResult::PlanFound;
            }
            // UNSAT: generate stepping stones instead.
        }

        // ---- GENERATE: distinct frontier states under the cached
        // per-frontier guard (revisits continue where the last round left
        // off — earlier diversity blocks still apply). ----
        auto rg_it = nodes_[current_].gen_guards.find(t_f);
        if (rg_it == nodes_[current_].gen_guards.end()) {
            std::string name = "wc_r" + std::to_string(next_round_++);
            z3::expr guard = ctx_.bool_const(name.c_str());
            // Cross-ANCHOR re-blocking (E1): every closed-list state whose
            // shallowest occurrence is <= t_f is blocked at this frontier —
            // padding makes each reachable here too, its first occurrence
            // owns the subtree, and blocking by clause is free where the
            // post-check dup skip costs one SAT query per re-serve (measured
            // 26–49% of generation on sokoban under M3).  Subsumes both the
            // old per-anchor cross-rung store and the anchor no-op block
            // (every anchor frontier is itself a closed-list entry).
            int blocked = 0;
            for (const auto& [hash, seen] : seen_states_) {
                // Dead states are dead at ANY depth; live ones only own
                // frontiers at or below their shallowest occurrence.
                if (seen.depth <= t_f || seen.dead) {
                    solver_.add(z3::implies(guard, !state_equals(t_f, seen.values)));
                    blocked++;
                }
            }
            stats.add("planner.wc_preblock_clauses", static_cast<double>(blocked));
            rg_it = nodes_[current_].gen_guards.emplace(t_f, guard).first;
        }
        z3::expr round_guard = rg_it->second;

        // The landmark-frontier target: every candidate must cross the cut —
        // some achiever of some ordering-minimal open landmark fires in the
        // window.  Central guiding mechanism; expect-SAT.
        const LandmarkData& node_lm = *nodes_[current_].lm;
        std::vector<int> frontier =
            landmark_frontier(nodes_[current_].anchor.unachieved_idx, node_lm);
        bool steering_on = !frontier.empty();
        if (steering_on &&
            nodes_[current_].steer_guards.find(t_f) ==
                nodes_[current_].steer_guards.end()) {
            std::string name = "wc_s" + std::to_string(next_round_ - 1) +
                               "_" + std::to_string(t_f);
            z3::expr sg = ctx_.bool_const(name.c_str());
            solver_.add(z3::implies(
                sg, steering_disjunction(frontier, anchor_t, t_f, node_lm)));
            nodes_[current_].steer_guards.emplace(t_f, sg);
            Stats::instance().set("planner.wc_frontier_size",
                                  static_cast<double>(frontier.size()));
            for (int idx : frontier) {
                const LandmarkInfo& lm =
                    node_lm.landmarks[static_cast<size_t>(idx)];
                Logger::instance().verbose(
                    "  DBG frontier lm[" + std::to_string(idx) + "] " +
                    encoder_.convert_expr_id_to_z3(lm.condition, t_f).to_string() +
                    " achievers=" + std::to_string(lm.achiever_action_ids.size()));
            }
        }

        int viable_this_round = 0;
        bool round_unsat = false;
        bool committed_this_round = false;
        int deepest_named = -1;
        int vetoed = 0;
        int candidates_cap = std::max(1, config.planner.wc_candidates);
        // Debug: numeric slice of a frontier state as "var=val, ..." pairs.
        auto describe_numerics = [&](const std::vector<z3::expr>& values) {
            std::string out;
            for (size_t i = 0; i < dynamic_fluent_eids_.size(); ++i) {
                ExprID eid = dynamic_fluent_eids_[i];
                if (problem_.is_bool_type(eid)) continue;
                z3::expr var = encoder_.convert_expr_id_to_z3(eid, 0);
                if (!out.empty()) out += ", ";
                out += var.to_string() + "=" + values[i].to_string();
            }
            return out;
        };

        for (int i = 0; i < candidates_cap; ++i) {
            z3::expr_vector extra(ctx_);
            extra.push_back(round_guard);
            if (steering_on) extra.push_back(nodes_[current_].steer_guards.at(t_f));
            auto check_start = std::chrono::steady_clock::now();
            z3::check_result result = commit_check(extra);
            Logger::instance().verbose(
                "  DBG cand#" + std::to_string(i) + " t_f=" + std::to_string(t_f) +
                " check=" +
                (result == z3::sat ? "sat" : result == z3::unsat ? "unsat" : "unknown") +
                " time=" + std::to_string(std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - check_start).count()) + "s");
            if (timed_out_) return RungResult::Timeout;
            if (result == z3::sat) {
                z3::model model = solver_.get_model();
                Candidate cand;
                cand.frontier = extract_state(model, t_f, cand.frontier_values);
                // State-level blocking (persists for this frontier's guard;
                // future guards re-block it from the closed list).
                solver_.add(z3::implies(round_guard,
                                        !state_equals(t_f, cand.frontier_values)));
                stats.add("planner.wc_candidates_generated");

                // Closed list (M1): a state already generated at <= this
                // depth is a duplicate — its first occurrence owns the
                // subtree; skip the RPG/h^FF scoring entirely.  (Reachable
                // only when the state entered the list after this frontier's
                // guard was created — earlier entries are pre-blocked.)
                size_t state_hash = cand.frontier.hash();
                auto seen_it = seen_states_.find(state_hash);
                if (seen_it != seen_states_.end() &&
                    (seen_it->second.depth <= t_f || seen_it->second.dead)) {
                    stats.add("planner.wc_frontier_dups");
                    continue;
                }
                if (seen_it == seen_states_.end()) {
                    seen_states_.emplace(state_hash,
                                         SeenState{t_f, cand.frontier_values});
                } else {
                    seen_it->second.depth = t_f;  // re-reached shallower: update
                }

                cand.segment =
                    extract_segment(model, anchor_t, t_f, cand.nonempty_steps);
                cand.unachieved_idx =
                    unachieved_landmark_indices(model, t_f, node_lm);
                for (const Goal& goal : problem_.goals()) {
                    z3::expr ge = encoder_.convert_expr_id_to_z3(goal.goal_id(), t_f);
                    if (model.eval(ge, true).is_true()) {
                        cand.goals_satisfied++;
                    }
                }
                bool ok = score_candidate(cand);
                bool novel = false;
                if (ok) {
                    cand.h_ff = relaxed_plan_size(model, t_f, cand.first_layers,
                                                  &cand.h_depth);
                    // Novelty (§5.4): evaluate against the tables, then fold
                    // in.  Dead candidates are skipped — a discarded state
                    // must not claim novelty credit from live achievers.
                    extract_novelty_features(model, t_f, cand);
                    novel = is_novel(cand);
                    update_novelty_tables(cand);
                }
                {
                    std::string crossed;
                    std::unordered_set<int> cand_open_dbg(
                        cand.unachieved_idx.begin(), cand.unachieved_idx.end());
                    for (int idx : frontier) {
                        if (!cand_open_dbg.count(idx)) {
                            if (!crossed.empty()) crossed += ",";
                            crossed += std::to_string(idx);
                        }
                    }
                    Logger::instance().verbose(
                        "  DBG cand#" + std::to_string(i) +
                        (ok ? " VIABLE h_ff=" + std::to_string(cand.h_ff) +
                                  " lb=" + std::to_string(cand.lb) +
                                  (novel ? " NOVEL" : "")
                            : " VETO-dead") +
                        " open_lms=" + std::to_string(cand.unachieved_idx.size()) +
                        " crossed_frontier=[" + crossed + "]" +
                        " goals_sat=" + std::to_string(cand.goals_satisfied) +
                        " nonempty=" + std::to_string(cand.nonempty_steps) +
                        " | " + describe_numerics(cand.frontier_values));
                }

                // Feature-class blocking (behaviour-space dedup, §4.2/OD 4):
                // the candidate's class is the truth vector of the anchor's
                // open landmark conditions at the frontier — the next
                // candidate must differ in some landmark, not merely in some
                // fluent.  Rung-local (deeper rungs get a fresh shot at the
                // class) and only when the partition is non-degenerate.
                const auto& anchor_open = nodes_[current_].anchor.unachieved_idx;
                if (anchor_open.size() >= 3) {
                    std::unordered_set<int> cand_open(cand.unachieved_idx.begin(),
                                                      cand.unachieved_idx.end());
                    z3::expr_vector cls(ctx_);
                    for (int idx : anchor_open) {
                        z3::expr c = encoder_.convert_expr_id_to_z3(
                            node_lm.landmarks[static_cast<size_t>(idx)].condition,
                            t_f);
                        cls.push_back(cand_open.count(idx) ? !c : c);
                    }
                    solver_.add(z3::implies(round_guard, !z3::mk_and(cls)));
                    stats.add("planner.wc_class_blocks");
                }
                if (ok) {
                    viable_this_round++;
                    // EHC move: commit the FIRST improving successor — at
                    // deep ladder rungs every further candidate costs a hard
                    // query, and enumerate-then-sort was measured to burn
                    // the budget on rush_hour.  Everything else joins the
                    // global open pool of stepping stones.
                    // LAMA-style: a candidate improving EITHER signal's own
                    // ratchet commits — each heuristic rescues the states
                    // the other misjudges.
                    bool improving = cand.h_ff < best_h_ff_ ||
                                     cand.unachieved_landmarks() < best_unachieved_;
                    // Plateau commit (§5.4b): at the ratchet's h^FF level a
                    // novel candidate is exploration worth standing on —
                    // commit it instead of pooling.  Restricted to the
                    // ratchet level so fresh-but-worse partitions (which are
                    // trivially novel) cannot hijack the dive.
                    bool plateau_novel =
                        novel && (cand.h_ff == best_h_ff_ ||
                                  cand.unachieved_landmarks() == best_unachieved_);
                    if (improving || plateau_novel) {
                        if (!improving) {
                            stats.add("planner.wc_novelty_commits");
                        }
                        current_ = make_node(current_, t_f, std::move(cand));
                        committed_this_round = true;
                        break;
                    }
                    open_.push_back(OpenEntry{current_, t_f, std::move(cand)});
                } else {
                    vetoed++;
                    stats.add("planner.wc_candidates_vetoed");
                    // Relaxed-dead is depth-independent: mark the closed-list
                    // entry so every future generation guard blocks this
                    // state at EVERY frontier, not only at depth <= t_f.
                    auto dead_it = seen_states_.find(state_hash);
                    if (dead_it != seen_states_.end()) {
                        dead_it->second.dead = true;
                    }
                }
            } else if (result == z3::unsat) {
                // A core naming commitment guards implicates the prefix
                // (jump); a core without them — steering alone, or the
                // exhausted diversity blocks — means no NEW frontier
                // crossing fits in w steps from this anchor: the widen
                // signal.  Steering is never dropped for unguided
                // diversity; the frontier target IS the search.
                round_unsat = true;
                deepest_named = deepest_core_node();
                break;
            } else {
                // Wall-deadline truncation is handled above; a genuine
                // solver 'unknown' (e.g. arithmetic incompleteness) ends
                // the round and falls through to the ladder.
                break;
            }
        }

        if (committed_this_round) {
            return RungResult::Progress;
        }

        Logger::instance().timestep_solving(VerbosityLevel::INFO, t_f, {
            {"anchor", std::to_string(anchor_t)},
            {"w", std::to_string(nodes_[current_].window)},
            {"viable", std::to_string(viable_this_round)},
            {"vetoed", std::to_string(vetoed)},
            {"anchor_hff", std::to_string(nodes_[current_].anchor.h_ff)},
            {"anchor_lms", std::to_string(nodes_[current_].anchor.unachieved_landmarks())},
            {"pool", std::to_string(open_.size())},
            {"depth", std::to_string(nodes_[current_].depth)}
        }, "W");

        // ---- No commit this round: escalate. ----
        if (round_unsat) {
            // A core naming the committed prefix: this line is dead under
            // its commitments — best-first backjump to the globally best
            // unexplored stepping stone.
            if (deepest_named >= 0 && pop_pool()) return RungResult::Progress;
            // Steering/blocks-only core: no NEW frontier crossing fits in w
            // steps from this anchor.  This is the ONLY widen signal — the
            // frontier space here is exhausted, so reach further.
            if (double_lookahead()) return RungResult::Progress;
            // Ladder at the room limit: the pool is all that is left.
            if (pop_pool()) return RungResult::Progress;
            Logger::instance().info("Greedy: no lookahead room and an empty pool.");
            return RungResult::Dead;
        }
        // SAT round with nothing worth standing on (all vetoed as dead, or
        // duplicates, or a plateau that pooled everything): crossings still
        // exist here, so widening is the WRONG reading — measured on rover
        // pfile16, it doubles the query size while the sampler keeps serving
        // dead states.  Retreat instead: best pool entry, else re-open the
        // parent anchor (its frontier keeps its guards and its blocks), else
        // at the root just sample again next rung — every served state is
        // blocked, so the frontier space strictly shrinks toward the honest
        // UNSAT/widen signal.
        if (pop_pool()) return RungResult::Progress;
        if (current_ != 0) {
            int parent = nodes_[current_].parent;
            Logger::instance().info(
                "  RETREAT to depth " + std::to_string(nodes_[parent].depth) +
                " anchor " + std::to_string(nodes_[parent].t_end) +
                " (barren round at depth " +
                std::to_string(nodes_[current_].depth) + ")");
            stats.add("planner.wc_retreats");
            current_ = parent;
        }
        return RungResult::Progress;
    }
}

// ---------------------------------------------------------------------------
// The unconstrained complete probe (same solver, same formula)
// ---------------------------------------------------------------------------

bool WindowedCommitmentPlanner::run_complete_probe() {
    auto& config = Config::instance();
    auto& stats = Stats::instance();

    if (complete_exhausted_) return false;
    if (remaining_ms() == 0) {
        timed_out_ = true;
        return false;
    }
    if (cp_horizon_ > config.planner.max_steps) {
        complete_exhausted_ = true;
        return false;
    }

    if (!c_started_) {
        if (!propagator_factory_) {
            Logger::instance().info(
                "WARNING: no propagator factory — complete side unavailable, "
                "search is greedy-only (incomplete).");
            complete_exhausted_ = true;
            return false;
        }
        c_solver_ = std::make_unique<z3::solver>(ctx_);
        // Propagator first, then SMT-level params (see the ordering note in
        // BasePlanner::set_propagator_strategy).
        c_propagator_ = propagator_factory_(*c_solver_);
        // Phase hints (§5.2): warm-start the sweep with the dive's
        // committed prefix.  Default arm C (order-based, insertion-proof);
        // RANTANPLAN_WC_PHASE_ARM=prefix selects arm A (absolute-time);
        // RANTANPLAN_WC_NO_PHASE_HINTS disables (the measured control).
        if (!std::getenv("RANTANPLAN_WC_NO_PHASE_HINTS")) {
            const char* arm = std::getenv("RANTANPLAN_WC_PHASE_ARM");
            if (arm && std::string(arm) == "prefix") {
                auto hints = std::make_unique<PhaseHintModule>();
                c_phase_hints_ = hints.get();
                c_propagator_->add_module(std::move(hints));
            } else {
                auto hints = std::make_unique<OrderHintModule>();
                c_order_hints_ = hints.get();
                c_propagator_->add_module(std::move(hints));
            }
        }
        z3::params p(ctx_);
        p.set("random_seed", (unsigned)42);
        const char* logic =
            recommended_logic(arithmetic_profile_, problem_.all_integer());
        if (logic) p.set("logic", logic);
        c_solver_->set(p);
        c_solver_->add(encoder_.encode_initial_state());
        c_propagator_->register_timestep_variables(0);
        c_filled_ = 0;
        c_started_ = true;
        Logger::instance().info(
            "Complete side: cold els sweep from h=" +
            std::to_string(cp_horizon_) +
            " (prefix-monotone, own solver, matched-time installments).");
    }

    // Greedy-informed jump: prefix-monotonicity makes one check at H cover
    // every horizon <= H, so the sweep may skip ahead to the dive's own
    // total-length estimate instead of grinding the UNSAT wall below it.
    // Only ever forward; the schedule keeps advancing from the jumped
    // horizon on UNSAT as before.
    {
        int target = std::min(greedy_estimate_, config.planner.max_steps);
        if (target > cp_horizon_) {
            Logger::instance().info(
                "Complete probe: jump to greedy estimate h=" +
                std::to_string(target) + " (was " +
                std::to_string(cp_horizon_) + ")");
            stats.add("planner.wc_horizon_jumps");
            cp_horizon_ = target;
        }
    }

    for (int t = c_filled_; t < cp_horizon_; ++t) {
        BasePlanner::add_timestep_constraints(*c_solver_, encoder_,
                                              *c_propagator_, t,
                                              /*prefix_monotone=*/true);
    }
    if (cp_horizon_ > c_filled_) c_filled_ = cp_horizon_;

    auto c_goal_literal = [&](int h) -> z3::expr {
        auto it = c_goal_literals_.find(h);
        if (it != c_goal_literals_.end()) return it->second;
        std::string name = "wcc_goal_at_" + std::to_string(h);
        z3::expr lit = ctx_.bool_const(name.c_str());
        c_solver_->add(z3::implies(lit, encoder_.encode_goal(h)));
        c_goal_literals_.emplace(h, lit);
        return lit;
    };

    // Refresh the phase hints when the dive has moved.
    if (c_phase_hints_ && c_hint_stamp_ != current_) {
        // Arm A: absolute-time hints — the committed prefix's action
        // variables (true for segment actions, false for the rest of each
        // committed step) become the sweep's preferred phases.  Same z3
        // context and variable factory on both sides, so ast ids match.
        std::unordered_map<unsigned, bool> hints;
        auto& factory = encoder_.get_variable_factory();
        for (int id = current_; id > 0; id = nodes_[id].parent) {
            const Node& n = nodes_[id];
            for (int t = n.t_begin; t < n.t_end; ++t) {
                std::unordered_set<int> fired;
                for (const Action* a : n.anchor.segment[t - n.t_begin]) {
                    fired.insert(a->id());
                }
                for (const Action& action : problem_.actions()) {
                    const z3::expr& av = factory.get_action_variable(action, t);
                    hints[av.id()] = fired.count(action.id()) > 0;
                }
            }
        }
        Stats::instance().set("planner.wc_phase_hints",
                              static_cast<double>(hints.size()));
        c_phase_hints_->set_hints(std::move(hints));
        c_hint_stamp_ = current_;
    }
    if (c_order_hints_ && c_hint_stamp_ != current_) {
        // Arm C: order hints — the dive's committed steps as LEVELS of
        // action ids, root first, empty steps skipped.  Absolute times are
        // deliberately not transferred; the module places the trace at
        // whatever steps the sweep's own search leaves room for.
        std::vector<std::vector<int>> levels;
        std::vector<int> path;
        for (int id = current_; id > 0; id = nodes_[id].parent) {
            path.push_back(id);
        }
        for (auto it = path.rbegin(); it != path.rend(); ++it) {
            const Node& n = nodes_[*it];
            for (int t = n.t_begin; t < n.t_end; ++t) {
                const auto& step = n.anchor.segment[t - n.t_begin];
                if (step.empty()) continue;
                std::vector<int> level;
                level.reserve(step.size());
                for (const Action* a : step) {
                    level.push_back(a->id());
                }
                levels.push_back(std::move(level));
            }
        }
        Stats::instance().set("planner.wc_order_hint_levels",
                              static_cast<double>(levels.size()));
        c_order_hints_->set_trace(std::move(levels));
        c_hint_stamp_ = current_;
    }

    apply_solver_timeout(*c_solver_);  // wall deadline; truncation only

    z3::expr_vector assumptions(ctx_);
    assumptions.push_back(c_goal_literal(cp_horizon_));
    auto solve_start = std::chrono::steady_clock::now();
    z3::check_result result = c_solver_->check(assumptions);
    double solve_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - solve_start).count();
    stats.add("planner.wc_complete_probes");

    Logger::instance().timestep_solving(VerbosityLevel::INFO, cp_horizon_, {
        {"solve", std::to_string(solve_time) + "s"},
        {"mem", std::to_string(static_cast<int>(
             MemoryTracker::instance().get_current_memory_mb())) + "MB"}
    }, "C");

    if (result == z3::sat) {
        found_plan_ = finish_with_plan(c_solver_->get_model(), cp_horizon_);
        return true;
    }
    if (result == z3::unknown) {
        if (remaining_ms() == 0) {
            timed_out_ = true;
        }
        return false;
    }
    // UNSAT: a sound proof that no plan of length <= cp_horizon_ exists
    // (prefix-monotone preserves satisfiability, so the proof transfers).
    // Greedy goal probes at frontiers <= cp_horizon_ ask a strictly more
    // constrained question and are muted via unsat_horizon_.
    unsat_horizon_ = std::max(unsat_horizon_, cp_horizon_);
    int next_h = schedule_.next(cp_horizon_, config.planner.max_steps);
    if (next_h <= cp_horizon_) {
        complete_exhausted_ = true;
        Logger::instance().info("Complete probe: max_steps exhausted.");
    } else {
        cp_horizon_ = next_h;
    }
    return false;
}

// ---------------------------------------------------------------------------
// The single loop
// ---------------------------------------------------------------------------

Plan WindowedCommitmentPlanner::search() {
    auto& config = Config::instance();

    schedule_ = HorizonSchedule::from_string(config.planner.horizon_schedule);
    solution_found_ = false;
    timed_out_ = false;
    init_deadline();
    search_start_ = std::chrono::steady_clock::now();

    int window0 = std::max(1, config.planner.wc_window);

    Logger::instance().info(
        "Starting windowed-commitment search (landmark-frontier guided): w0=" +
        std::to_string(window0) +
        ", C=" + std::to_string(config.planner.wc_candidates) +
        ", propagator: " + propagator_strategy_->get_name() +
        ", timeout: " + format_timeout_string());

    solver_.add(encoder_.encode_initial_state());
    propagator_strategy_->register_timestep_variables(0);

    // Goal already satisfied at t=0?  (Skipped when the preprocessor's RPG
    // bound proves plans shorter than start_timestep impossible.)
    if (config.planner.start_timestep <= 0) {
        if (!apply_solver_timeout(solver_)) return Plan();
        z3::expr_vector assumptions(ctx_);
        assumptions.push_back(goal_literal(0));
        z3::check_result result = solver_.check(assumptions);
        if (result == z3::sat) {
            return finish_with_plan(solver_.get_model(), 0);
        }
        if (result == z3::unknown && handle_unknown_result(solver_, "goal@0")) {
            collect_statistics();
            propagator_strategy_->cleanup();
            return Plan();
        }
    }

    // Root anchor: the initial state, read back from a model of state 0 so
    // frontier values and landmark/h^FF evaluation use the exact solver-side
    // representation (only the initial state is asserted yet).
    {
        if (!apply_solver_timeout(solver_)) return Plan();
        if (solver_.check() != z3::sat) {
            Logger::instance().error(
                "Initial state unsatisfiable — inconsistent input.");
            collect_statistics();
            propagator_strategy_->cleanup();
            return Plan();
        }
        z3::model init_model = solver_.get_model();
        initialize_achievers();
        Candidate root;
        root.frontier = extract_state(init_model, 0, root.frontier_values);
        root.unachieved_idx =
            unachieved_landmark_indices(init_model, 0, landmark_data_);
        score_candidate(root);
        if (root.lb == kInf) {
            // Relaxed-unreachable from init (normally caught by the
            // preprocessor); the complete probe will exhaust and report
            // honestly.
            Logger::instance().info(
                "Goals relaxed-unreachable from the initial state — greedy "
                "search disabled.");
            greedy_exhausted_ = true;
        } else {
            root.h_ff = relaxed_plan_size(init_model, 0, root.first_layers,
                                          &root.h_depth);
            best_h_ff_ = root.h_ff;
            best_unachieved_ = root.unachieved_landmarks();
            if (root.h_depth > 0) greedy_estimate_ = root.h_depth;
            extract_novelty_features(init_model, 0, root);
            update_novelty_tables(root);
            seen_states_.emplace(root.frontier.hash(),
                                 SeenState{0, root.frontier_values});
            // Informed root window: same rule as make_node — the smallest
            // window at which the frontier steering can possibly fire.
            {
                std::vector<int> front =
                    landmark_frontier(root.unachieved_idx, landmark_data_);
                int informed = INT_MAX;
                for (int idx : front) {
                    for (int aid : landmark_data_
                             .landmarks[static_cast<size_t>(idx)]
                             .achiever_action_ids) {
                        auto it = root.first_layers.find(aid);
                        if (it != root.first_layers.end()) {
                            informed = std::min(informed, it->second);
                        }
                    }
                }
                if (informed != INT_MAX) window0 = std::max(window0, informed);
            }
            assert_masks(nullptr, 0, 0, window0, root.first_layers,
                         root.rpg_action_layers, root.rpg_saw_all_actions);
            assert_ordering_clauses(nullptr, 0, 0, window0, root.unachieved_idx,
                                    landmark_data_);

            // Root landmark frame: the preprocessing artifact (already
            // computed from the initial state — no recompute needed).
            nodes_.push_back(Node{-1, 0, ctx_.bool_val(true), ctx_.bool_val(true),
                                  0, 0, std::move(root), window0,
                                  /*masked_upto=*/window0, {}, {},
                                  std::make_shared<LandmarkData>(landmark_data_)});
            current_ = 0;
        }
    }

    cp_horizon_ = std::max(0, config.planner.start_timestep);

    // Event-driven round-robin, no clocks: each iteration runs one greedy
    // rung (goal probe + one frontier-targeted generation round) and then
    // one complete-side horizon check.  Escalation inside the rung is by
    // logical signals only (cores, frontier unreachability, structural
    // exhaustion); the user's wall deadline, when set, merely truncates.
    while (!greedy_exhausted_ || !complete_exhausted_) {
        if (remaining_ms() == 0) {
            timed_out_ = true;
            break;
        }

        if (!greedy_exhausted_) {
            RungResult result = expand_rung();
            if (result == RungResult::PlanFound) {
                return found_plan_;
            }
            if (result == RungResult::Timeout) {
                timed_out_ = true;
                break;
            }
            if (result == RungResult::Dead) {
                greedy_exhausted_ = true;
                Logger::instance().info(
                    "Greedy search retired (structural exhaustion).");
            }
        }

        if (!complete_exhausted_) {
            if (run_complete_probe()) {
                return found_plan_;
            }
            if (timed_out_) break;
        }
    }

    if (timed_out_) {
        Logger::instance().info("No plan found (timeout).");
    } else {
        Logger::instance().info("\n*** NO PLAN FOUND within " +
            std::to_string(config.planner.max_steps) + " timesteps ***");
    }
    collect_statistics();
    propagator_strategy_->cleanup();
    if (c_propagator_) c_propagator_->cleanup();
    return Plan();
}

} // namespace rantanplan
