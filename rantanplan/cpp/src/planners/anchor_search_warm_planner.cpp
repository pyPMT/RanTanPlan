#include "anchor_search_warm_planner.hpp"
#include "../analysis/numeric_relaxed_planning_graph.hpp"
#include "../passes/landmark_analysis_pass.hpp"
#include "../preprocessing/preprocessed_problem.hpp"
#include "../config/config.hpp"
#include "../util/logger.hpp"
#include "../util/stats.hpp"
#include "../util/memory_tracker.hpp"
#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <numeric>
#include <string>

namespace rantanplan {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

/// Model numeral -> double (mirrors the els-wc2 extractor).
double numeral_to_double(const z3::expr& v) {
    int64_t i64 = 0;
    if (v.is_numeral_i64(i64)) return static_cast<double>(i64);
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

// ===========================================================================
// Construction
// ===========================================================================

AnchorSearchWarmPlanner::AnchorSearchWarmPlanner(const Problem& problem,
                                                 BaseEncoder& encoder,
                                                 z3::context& ctx)
    : BasePlanner(problem, encoder, ctx) {
    if (const char* c = std::getenv("RANTANPLAN_WC3_WINDOW_CAP"))
        window_cap_ = std::max(1, static_cast<int>(std::atol(c)));
    if (const char* g = std::getenv("RANTANPLAN_WC3_GREEDY_RATIO"))
        greedy_ratio_ = std::max(1, static_cast<int>(std::atol(g)));
    if (const char* r = std::getenv("RANTANPLAN_WC3_DIVE_RLIMIT"))
        dive_rlimit_ = static_cast<unsigned>(std::max(0L, std::atol(r)));
    trace_on_ = std::getenv("RANTANPLAN_WC3_TRACE") != nullptr;
    backbone_only_ = std::getenv("RANTANPLAN_WC3_BACKBONE_ONLY") != nullptr;
    if (const char* m = std::getenv("RANTANPLAN_WC3_MASKS")) masks_on_ = static_cast<int>(std::atol(m));
    if (const char* u = std::getenv("RANTANPLAN_WC3_UORDER")) uorder_ = std::atol(u) != 0;
    if (const char* d = std::getenv("RANTANPLAN_WC3_DYNRATIO")) dyn_ratio_ = std::atol(d) != 0;

    // Dynamic ground fluents: those targeted by some action effect.  Only
    // these get Z3 state variables at every timestep; statics come from s0.
    std::unordered_set<int> dynamic_ids;
    for (const Action& action : problem_.actions())
        for (const Effect& effect : action.effects()) {
            int fid = problem_.find_grounded_fluent_index(
                effect.effect_expression().fluent_id());
            if (fid >= 0) dynamic_ids.insert(fid);
        }
    dynamic_fluent_ids_.assign(dynamic_ids.begin(), dynamic_ids.end());
    std::sort(dynamic_fluent_ids_.begin(), dynamic_fluent_ids_.end());
    dynamic_fluent_eids_.reserve(dynamic_fluent_ids_.size());
    for (int fid : dynamic_fluent_ids_)
        dynamic_fluent_eids_.push_back(problem_.grounded_fluent(fid));

    // Full valuation of the initial state: static-fluent source for every
    // extracted state.
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
                if (pool.payload_is_int(val_eid)) v = static_cast<double>(pool.payload_int(val_eid));
                else if (pool.payload_is_double(val_eid)) v = pool.payload_double(val_eid);
            }
            base_valuation_.values[fid] = StateValuation::FluentValue::make_numeric(v);
        }
    }

    for (const Action& action : problem_.actions())
        action_by_id_[action.id()] = &action;
}

void AnchorSearchWarmPlanner::consume_artifacts(PreprocessedProblem& artifacts) {
    landmark_data_ = artifacts.landmark_data;
    if (artifacts.achievers) achievers_ = std::move(artifacts.achievers);
}

void AnchorSearchWarmPlanner::initialize_achievers() {
    if (!achievers_) achievers_ = std::make_unique<AchieversAnalysis>(problem_);
    for (ExprID cond : achievers_->get_all_conditions()) {
        auto& ptrs = condition_achievers_[cond];
        for (const Action& a : achievers_->get_achievers(cond)) {
            auto it = action_by_id_.find(a.id());
            if (it != action_by_id_.end()) ptrs.push_back(it->second);
        }
    }
    for (ExprID cond : achievers_->get_goal_conditions())
        goal_condition_ids_.push_back(cond);
    for (const Action& a : problem_.actions()) {
        const auto& precs = achievers_->get_preconditions(a);
        if (!precs.empty())
            action_preconditions_[action_by_id_.at(a.id())].assign(precs.begin(), precs.end());
    }
    // Feature space for satisfied-condition extraction (landmark recompute):
    // achiever conditions + goal conditions.
    std::unordered_set<ExprID> features;
    for (const auto& [cond, ptrs] : condition_achievers_) features.insert(cond);
    for (ExprID cond : goal_condition_ids_) features.insert(cond);
    novelty_feature_ids_.assign(features.begin(), features.end());
}

// ===========================================================================
// Open-set order
// ===========================================================================

bool AnchorSearchWarmPlanner::Order::operator()(int a, int b) const {
    const Anchor& na = (*anchors)[a];
    const Anchor& nb = (*anchors)[b];
    if (na.D != nb.D) return na.D < nb.D;         // demotions primary
    if (mode == 1) {                              // u-primary queue
        if (na.u != nb.u) return na.u < nb.u;
        if (na.d_ff != nb.d_ff) return na.d_ff < nb.d_ff;
    } else {                                      // d_ff-primary queue
        if (na.d_ff != nb.d_ff) return na.d_ff < nb.d_ff;
        if (na.u != nb.u) return na.u < nb.u;
    }
    if (na.h_ff != nb.h_ff) return na.h_ff < nb.h_ff;
    if (na.t != nb.t) return na.t < nb.t;
    return a < b;                                 // total-order tiebreak
}

// ===========================================================================
// State extraction and step-0 pins
// ===========================================================================

StateValuation AnchorSearchWarmPlanner::extract_state(
    const z3::model& model, int t, std::vector<z3::expr>& dynamic_values) const {
    StateValuation state = base_valuation_;
    dynamic_values.clear();
    dynamic_values.reserve(dynamic_fluent_eids_.size());
    for (size_t i = 0; i < dynamic_fluent_eids_.size(); ++i) {
        ExprID eid = dynamic_fluent_eids_[i];
        int fid = dynamic_fluent_ids_[i];
        z3::expr var = encoder_.convert_expr_id_to_z3(eid, t);
        z3::expr val = model.eval(var, /*model_completion=*/true);
        if (problem_.is_bool_type(eid)) {
            state.values[fid] = StateValuation::FluentValue::make_bool(val.is_true());
        } else if (problem_.is_object_type(eid)) {
            int64_t idx = -1;
            if (!val.is_numeral_i64(idx)) idx = -1;
            state.values[fid] = StateValuation::FluentValue::make_object(static_cast<int>(idx));
        } else {
            state.values[fid] = StateValuation::FluentValue::make_numeric(numeral_to_double(val));
        }
        dynamic_values.push_back(val);
    }
    return state;
}

void AnchorSearchWarmPlanner::state_equal_atoms(
    int t, const std::vector<z3::expr>& values, z3::expr_vector& out) const {
    for (size_t i = 0; i < dynamic_fluent_eids_.size(); ++i) {
        z3::expr var = encoder_.convert_expr_id_to_z3(dynamic_fluent_eids_[i], t);
        out.push_back(var == values[i]);
    }
}

z3::expr AnchorSearchWarmPlanner::state_equals(
    int t, const std::vector<z3::expr>& values) const {
    z3::expr_vector eqs(ctx_);
    state_equal_atoms(t, values, eqs);
    if (eqs.empty()) return ctx_.bool_val(true);
    return z3::mk_and(eqs);
}

// ===========================================================================
// Scoring (RPG + FF relaxed plan) — adapted from els-wc2
// ===========================================================================

bool AnchorSearchWarmPlanner::score_anchor(const StateValuation& sigma, Anchor& out) const {
    NumericRelaxedPlanningGraph rpg(problem_, sigma);
    bool goals_reachable = rpg.build();
    out.first_layers = rpg.get_action_first_layers();
    out.rpg_action_layers = static_cast<int>(rpg.get_layer_count()) - 1;
    out.rpg_saw_all_actions = out.first_layers.size() == problem_.actions().size();
    if (!goals_reachable) { out.lb = kInf; return false; }  // sound dead-end veto
    int lb = rpg.get_minimum_steps_lower_bound();
    out.lb = lb < 0 ? kInf : static_cast<double>(lb);
    return out.lb < kInf;
}

int AnchorSearchWarmPlanner::relaxed_plan_size(
    const z3::model& model, int t,
    const std::unordered_map<int, int>& first_layers, int* depth_out) const {
    std::unordered_map<ExprID, bool> satisfied_cache;
    auto satisfied = [&](ExprID cond) {
        auto it = satisfied_cache.find(cond);
        if (it != satisfied_cache.end()) return it->second;
        z3::expr ce = encoder_.convert_expr_id_to_z3(cond, t);
        bool s = model.eval(ce, /*model_completion=*/true).is_true();
        satisfied_cache.emplace(cond, s);
        return s;
    };
    std::unordered_set<const Action*> plan;
    std::unordered_set<ExprID> unsatisfied;
    for (ExprID g : goal_condition_ids_)
        if (!satisfied(g)) unsatisfied.insert(g);
    int max_layer = 0;
    for (const auto& [aid, layer] : first_layers) max_layer = std::max(max_layer, layer);
    for (int layer = max_layer; layer >= 0 && !unsatisfied.empty(); --layer) {
        std::vector<ExprID> resolved, new_subgoals;
        for (ExprID cond : unsatisfied) {
            auto ach = condition_achievers_.find(cond);
            if (ach == condition_achievers_.end()) continue;
            const Action* best = nullptr; int best_layer = -1;
            for (const Action* a : ach->second) {
                auto fl = first_layers.find(a->id());
                if (fl == first_layers.end()) continue;
                if (fl->second <= layer && fl->second > best_layer) { best = a; best_layer = fl->second; }
            }
            if (!best) continue;
            resolved.push_back(cond);
            if (plan.insert(best).second) {
                auto pre = action_preconditions_.find(best);
                if (pre != action_preconditions_.end())
                    for (ExprID p : pre->second) if (!satisfied(p)) new_subgoals.push_back(p);
            }
        }
        for (ExprID c : resolved) unsatisfied.erase(c);
        for (ExprID p : new_subgoals) unsatisfied.insert(p);
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

// ===========================================================================
// Landmark guidance (M2) — adapted from els-wc2
// ===========================================================================

std::vector<ExprID> AnchorSearchWarmPlanner::satisfied_conditions(
    const z3::model& model, int t) const {
    std::vector<ExprID> sat;
    for (ExprID g : novelty_feature_ids_) {
        z3::expr cond = encoder_.convert_expr_id_to_z3(g, t);
        if (model.eval(cond, /*model_completion=*/true).is_true()) sat.push_back(g);
    }
    return sat;
}

std::shared_ptr<const LandmarkData> AnchorSearchWarmPlanner::recompute_landmarks(
    const StateValuation& sigma, const std::vector<ExprID>& sat_conditions) const {
    std::unordered_set<ExprID> sat(sat_conditions.begin(), sat_conditions.end());
    return std::make_shared<LandmarkData>(LandmarkAnalysisPass::compute(
        problem_, *achievers_,
        [&](ExprID cond) { return sat.count(cond) > 0; },
        [&](ExprID fluent) -> std::optional<double> {
            int fid = problem_.find_grounded_fluent_index(fluent);
            if (fid < 0 || fid >= static_cast<int>(sigma.values.size())) return std::nullopt;
            const auto& v = sigma.values[static_cast<size_t>(fid)];
            if (v.kind != StateValuation::Kind::Numeric) return std::nullopt;
            return v.numeric_value;
        }));
}

std::vector<int> AnchorSearchWarmPlanner::landmark_frontier(
    const std::vector<int>& open_idx, const LandmarkData& data) const {
    std::unordered_set<int> open(open_idx.begin(), open_idx.end());
    std::unordered_set<int> blocked;
    for (const auto& [earlier, later] : data.orderings)
        if (open.count(earlier) && open.count(later)) blocked.insert(later);
    std::vector<int> frontier;
    for (int idx : open_idx) if (!blocked.count(idx)) frontier.push_back(idx);
    return frontier;
}

z3::expr AnchorSearchWarmPlanner::steering_disjunction(
    const std::vector<int>& frontier_idx, int t_begin, int t_end,
    const LandmarkData& data) const {
    auto& factory = encoder_.get_variable_factory();
    z3::expr_vector fires(ctx_);
    for (int idx : frontier_idx) {
        const LandmarkInfo& lm = data.landmarks[static_cast<size_t>(idx)];
        for (int action_id : lm.achiever_action_ids) {
            auto it = action_by_id_.find(action_id);
            if (it == action_by_id_.end()) continue;
            for (int t = t_begin; t < t_end; ++t)
                fires.push_back(factory.get_action_variable(*it->second, t));
        }
    }
    if (fires.empty()) return ctx_.bool_val(true);
    return z3::mk_or(fires);
}

int AnchorSearchWarmPlanner::informed_window(
    const std::unordered_map<int, int>& first_layers, const LandmarkData& lm,
    int d_ff) const {
    int window = std::max(1, Config::instance().planner.wc_window);
    std::vector<int> open(lm.landmarks.size());
    std::iota(open.begin(), open.end(), 0);
    std::vector<int> front = landmark_frontier(open, lm);
    int informed = INT_MAX;
    for (int idx : front)
        for (int aid : lm.landmarks[static_cast<size_t>(idx)].achiever_action_ids) {
            auto it = first_layers.find(aid);
            if (it != first_layers.end()) informed = std::min(informed, it->second);
        }
    if (informed != INT_MAX) window = std::max(window, informed);
    (void)d_ff;
    return window;
}

void AnchorSearchWarmPlanner::append_masks(
    const std::unordered_map<int, int>& first_layers, int rpg_action_layers,
    bool saw_all_actions, int w, z3::expr_vector& out) const {
    if (!masks_on_) return;
    // Absent from first_layers = relaxed-unreachable, so mask-whole-window is
    // sound only when the RPG explored every action (or early termination is
    // off); otherwise fall back to the deepest layer as a conservative floor.
    bool absent_unreachable =
        saw_all_actions || !Config::instance().rpg.early_goal_termination;
    // masks_on_ == 2: unreachable-only (cheap, strong — few literals); the
    // default (1) also emits the numerous "before first-layer" partial masks,
    // which narrow more but add O(|actions|*w) assumptions per check.
    bool unreachable_only = masks_on_ >= 2;
    auto& factory = encoder_.get_variable_factory();
    for (const Action& action : problem_.actions()) {
        auto it = first_layers.find(action.id());
        int first_off;
        if (it != first_layers.end()) {
            if (unreachable_only) continue;                  // skip partial masks
            first_off = std::max(0, it->second - 1);
        } else if (absent_unreachable) {
            first_off = w;                                   // unreachable: mask all offsets
        } else {
            first_off = std::max(0, rpg_action_layers - 1);
        }
        int stop = std::min(first_off, w);
        for (int off = 0; off < stop; ++off)
            out.push_back(!factory.get_action_variable(action, off));
    }
}

// ===========================================================================
// The one warm solver: rolling unrolling, step 0 free
// ===========================================================================

void AnchorSearchWarmPlanner::ensure_started() {
    if (started_) return;
    z3::params p(ctx_);
    p.set("random_seed", (unsigned)42);
    solver_.set(p);
    propagator_strategy_->register_timestep_variables(0);
    // NO encode_initial_state(): step 0 stays free, pinned per query.
    filled_ = 0;
    started_ = true;
}

void AnchorSearchWarmPlanner::ensure_depth(int w) {
    for (int t = filled_; t < w; ++t)
        BasePlanner::add_timestep_constraints(solver_, encoder_, *propagator_strategy_,
                                              t, /*prefix_monotone=*/true);
    if (w > filled_) filled_ = w;
}

z3::expr AnchorSearchWarmPlanner::goal_lit(int h) {
    auto it = goal_lits_.find(h);
    if (it != goal_lits_.end()) return it->second;
    z3::expr lit = ctx_.bool_const(("wc3_goal_" + std::to_string(h)).c_str());
    solver_.add(z3::implies(lit, encoder_.encode_goal(h)));
    goal_lits_.emplace(h, lit);
    return lit;
}

bool AnchorSearchWarmPlanner::extract_root(Anchor& root) {
    if (!propagator_factory_) {
        Logger::instance().error("els-wc3 requires a propagator factory (extra solver).");
        return false;
    }
    z3::solver tmp(ctx_);
    auto prop = propagator_factory_(tmp);
    z3::params p(ctx_);
    p.set("random_seed", (unsigned)42);
    tmp.set(p);
    tmp.add(encoder_.encode_initial_state());
    prop->register_timestep_variables(0);
    bool ok = tmp.check() == z3::sat;
    if (!ok) { prop->cleanup(); return false; }
    z3::model m = tmp.get_model();
    StateValuation s0 = extract_state(m, 0, root.sigma_values);
    root.parent = -1; root.t = 0; root.D = 0;
    root.sigma_hash = s0.hash();
    if (!score_anchor(s0, root)) {
        prop->cleanup();
        Logger::instance().info("Goals relaxed-unreachable from the initial state.");
        return false;
    }
    root.h_ff = relaxed_plan_size(m, 0, root.first_layers, &root.d_ff);
    root.lm = recompute_landmarks(s0, satisfied_conditions(m, 0));
    root.u = static_cast<int>(root.lm->landmarks.size());
    root.d_lm = root.lm->depth_lower_bound;
    root.window = std::max(1, root.d_ff);
    prop->cleanup();
    return true;
}

// ===========================================================================
// Mode 1: DISCOVER — a bounded solve from an anchor
// ===========================================================================

AnchorSearchWarmPlanner::DiveResult
AnchorSearchWarmPlanner::discover(const Anchor& n, int w, const char* engine) {
    bool is_backbone = std::strcmp(engine, "backbone") == 0;
    ensure_depth(w);
    z3::expr glit = goal_lit(w);
    z3::expr_vector atoms(ctx_);
    state_equal_atoms(0, n.sigma_values, atoms);   // pin step 0 to the anchor
    z3::expr_vector a(ctx_);
    for (unsigned i = 0; i < atoms.size(); ++i) a.push_back(atoms[i]);
    a.push_back(glit);                              // + goal@w
    // Masks narrow the *shallow* greedy windows; on the deep backbone solve they
    // are pure overhead (thousands of assumptions), so gate them to non-backbone.
    if (!is_backbone)
        append_masks(n.first_layers, n.rpg_action_layers, n.rpg_saw_all_actions, w, a);

    if (!apply_solver_timeout(solver_)) { timed_out_ = true; return DiveResult::Timeout; }
    if (!is_backbone && dive_rlimit_ > 0) {
        z3::params pr(ctx_); pr.set("rlimit", dive_rlimit_); solver_.set(pr);
    }
    auto t0 = std::chrono::steady_clock::now();
    z3::check_result r = solver_.check(a);
    double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (!is_backbone && dive_rlimit_ > 0) {
        z3::params pr(ctx_); pr.set("rlimit", 0u); solver_.set(pr);
    }
    (is_backbone ? backbone_secs_ : dive_secs_) += secs;
    Stats::instance().add("planner.solve_time", secs);
    Stats::instance().add("planner.timesteps_explored");

    if (r == z3::sat) {
        z3::model model = solver_.get_model();
        Plan flat = n.prefix;
        Plan window = encoder_.extract_plan(model, w);
        for (const Action* act : window.actions()) flat.add_action(act);
        if (replay_validate(flat)) {
            if (!is_backbone) ++dive_sat_;
            found_plan_ = finish(flat, n.t + w, engine);
            return DiveResult::PlanFound;
        }
        Logger::instance().error(
            "wc3 plan failed replay validation — ignoring (concatenation bug?); continuing.");
        return DiveResult::Unsat;
    }
    if (r == z3::unknown) {
        if (remaining_ms() == 0) { timed_out_ = true; return DiveResult::Timeout; }
        return DiveResult::Unknown;   // bounded out by rlimit; caller keeps going
    }
    // UNSAT: no plan of length <= w from this anchor.  Channel A: lift the
    // core of the step-0 pin into a reusable nogood (dive engine only; the
    // backbone's UNSAT is captured by H_unsat_).
    if (!is_backbone) note_nogood(atoms, solver_.unsat_core(), w);
    return DiveResult::Unsat;
}

// ---- nogood channel (M4) --------------------------------------------------

void AnchorSearchWarmPlanner::note_nogood(const z3::expr_vector& atoms,
                                          const z3::expr_vector& core, int w) {
    if (nogoods_.size() >= 4000) return;             // bounded store
    Nogood ng;
    for (unsigned i = 0; i < atoms.size(); ++i) {
        bool in_core = false;
        for (unsigned j = 0; j < core.size(); ++j)
            if (z3::eq(atoms[i], core[j])) { in_core = true; break; }
        if (in_core) ng.core.emplace_back(static_cast<int>(i), atoms[i].arg(1));
    }
    if (trace_on_)
        Logger::instance().info("[wc3-ng] core_size=" + std::to_string(core.size()) +
                                " pin_atoms=" + std::to_string(atoms.size()) +
                                " matched=" + std::to_string(ng.core.size()) +
                                " w=" + std::to_string(w));
    // Keep only reasonably compact cores (the more general, the more they
    // prune); empty core = "no goal in w from any state" is the backbone's
    // business.  NOTE: dive cores here run large (13-37 fluents), so nogoods
    // are specific and prune little — see the final report.
    if (ng.core.empty() || ng.core.size() > 20) return;
    ng.w = w;
    nogoods_.push_back(std::move(ng));
    ++nogoods_learned_;
}

bool AnchorSearchWarmPlanner::nogood_blocks(const std::vector<z3::expr>& sigma_values,
                                            int w) const {
    for (const Nogood& ng : nogoods_) {
        if (ng.w < w) continue;                      // refutes only windows <= ng.w
        bool match = true;
        for (const auto& [idx, val] : ng.core)
            if (!z3::eq(sigma_values[static_cast<size_t>(idx)], val)) { match = false; break; }
        if (match) return true;
    }
    return false;
}

// ===========================================================================
// Mode 2: WIDEN — harvest forward children (DISCOVER at w=1, read the model)
// ===========================================================================

int AnchorSearchWarmPlanner::make_anchor(int parent, Plan prefix, int t,
                                         const StateValuation& sigma,
                                         std::vector<z3::expr> sigma_values,
                                         const z3::model& model, int model_t) {
    Anchor a;
    a.parent = parent;
    a.prefix = std::move(prefix);
    a.t = t;
    a.sigma_values = std::move(sigma_values);
    a.sigma_hash = sigma.hash();
    if (!score_anchor(sigma, a)) return -1;   // relaxed-dead: sound veto
    a.h_ff = relaxed_plan_size(model, model_t, a.first_layers, &a.d_ff);
    a.lm = recompute_landmarks(sigma, satisfied_conditions(model, model_t));
    a.u = static_cast<int>(a.lm->landmarks.size());
    a.d_lm = a.lm->depth_lower_bound;
    a.window = std::max(1, a.d_ff);
    a.id = static_cast<int>(anchors_.size());
    anchors_.push_back(std::move(a));
    return anchors_.back().id;
}

void AnchorSearchWarmPlanner::widen(int anchor_id, int w) {
    ++widens_;
    // Copy the fields we need BEFORE make_anchor() can reallocate anchors_.
    std::vector<z3::expr> sigma_values = anchors_[anchor_id].sigma_values;
    Plan base_prefix = anchors_[anchor_id].prefix;
    int base_t = anchors_[anchor_id].t;
    std::shared_ptr<const LandmarkData> lm = anchors_[anchor_id].lm;
    std::unordered_map<int, int> first_layers = anchors_[anchor_id].first_layers;
    int rpg_layers = anchors_[anchor_id].rpg_action_layers;
    bool saw_all = anchors_[anchor_id].rpg_saw_all_actions;

    ensure_depth(w);
    const int C = std::max(1, Config::instance().planner.wc_candidates);

    // Steering: some frontier-landmark achiever must fire within the window, so
    // children are w-step stepping stones that cross the next landmark frontier
    // (an assumption, not a permanent clause — the warm-solver anti-salt rule).
    z3::expr_vector assumptions(ctx_);
    state_equal_atoms(0, sigma_values, assumptions);
    if (lm && !lm->landmarks.empty()) {
        std::vector<int> open(lm->landmarks.size());
        std::iota(open.begin(), open.end(), 0);
        assumptions.push_back(steering_disjunction(landmark_frontier(open, *lm), 0, w, *lm));
    } else {
        assumptions.push_back(!state_equals(w, sigma_values));  // no landmarks: just move
    }
    append_masks(first_layers, rpg_layers, saw_all, w, assumptions);  // narrow the vocabulary

    auto t0 = std::chrono::steady_clock::now();
    for (int round = 0; round < C; ++round) {
        if (!apply_solver_timeout(solver_)) { timed_out_ = true; break; }
        z3::check_result r = solver_.check(assumptions);
        if (r == z3::unknown) { if (remaining_ms() == 0) timed_out_ = true; break; }
        if (r != z3::sat) break;

        z3::model model = solver_.get_model();
        std::vector<z3::expr> vals;
        StateValuation sp = extract_state(model, w, vals);
        assumptions.push_back(!state_equals(w, vals));   // block this child next round
        ++widen_children_;

        size_t h = sp.hash();
        auto cit = closed_.find(h);
        if (cit != closed_.end()) {
            if (cit->second.dead) continue;
            if (cit->second.min_depth <= base_t + w) { ++closed_hits_; continue; }
        }
        Plan child_prefix = base_prefix;
        Plan window = encoder_.extract_plan(model, w);
        for (const Action* act : window.actions()) child_prefix.add_action(act);
        int cid = make_anchor(anchor_id, std::move(child_prefix), base_t + w, sp,
                              std::move(vals), model, w);
        if (cid < 0) { closed_[h].dead = true; ++dead_children_; continue; }
        closed_[h] = {base_t + w, false};
        push_open(cid);
        max_depth_ = std::max(max_depth_, base_t + w);
    }
    widen_secs_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

// ===========================================================================
// Open-set plumbing and control law
// ===========================================================================

void AnchorSearchWarmPlanner::push_open(int id) {
    open_d_.insert(id);
    open_u_.insert(id);
    peak_open_ = std::max(peak_open_, open_d_.size());
}
void AnchorSearchWarmPlanner::erase_open(int id) { open_d_.erase(id); open_u_.erase(id); }
int AnchorSearchWarmPlanner::best_open() const {   // peek d_ff-best (trace only)
    return open_d_.empty() ? -1 : *open_d_.begin();
}
int AnchorSearchWarmPlanner::pop_open() {
    if (!uorder_) {                                // d_ff queue only
        if (open_d_.empty()) return -1;
        int id = *open_d_.begin(); open_d_.erase(id); open_u_.erase(id); return id;
    }
    std::set<int, Order>* q = (pop_turn_ == 0) ? &open_d_ : &open_u_;
    if (q->empty()) q = (pop_turn_ == 0) ? &open_u_ : &open_d_;
    if (q->empty()) return -1;
    int id = *q->begin();
    open_d_.erase(id);
    open_u_.erase(id);
    pop_turn_ ^= 1;                                 // alternate the two signals
    return id;
}

AnchorSearchWarmPlanner::Step AnchorSearchWarmPlanner::greedy_step() {
    int id = pop_open();                          // alternates d_ff / u queues
    if (id < 0) return Step::Retire;              // greedy frontier exhausted
    const int max_steps = Config::instance().planner.max_steps;
    const int t = anchors_[id].t;
    const int cap = std::min(window_cap_, max_steps - t);
    if (cap <= 0) return Step::Progress;          // no room; anchor dropped (already popped)

    // Widen stride: reach toward the next landmark frontier, but never stride
    // PAST the estimated distance to the goal (d_ff) — a large informed stride
    // overshoots, landing in dead/too-deep states.  So it is coarse far from
    // the goal and fine near it.
    const int iw = informed_window(anchors_[id].first_layers, *anchors_[id].lm,
                                   anchors_[id].d_ff);
    const int dff = std::max(1, anchors_[id].d_ff);
    const int gen_w = std::min(cap, std::max(1, std::min(iw, dff + 1)));
    // Probe window: sized to reach the GOAL (d_ff), past the backbone horizon.
    const int probe_w = std::min(cap, std::max({dff, H_unsat_ - t + 1, 1}));

    // (1) Goal probe from this committed prefix, gated so it is neither
    // redundant with the backbone (t+w > H_unsat), premature (w >= d_lm), nor
    // from the root (t>0 — the backbone owns from-s0).
    bool probe_gate = t > 0 && t + probe_w > H_unsat_ && probe_w >= anchors_[id].d_lm;
    bool blocked = probe_gate && nogood_blocks(anchors_[id].sigma_values, probe_w);
    if (blocked) ++nogood_prunes_;              // Channel B: refuted by a learned nogood
    if (probe_gate && !blocked) {
        ++dives_; dive_window_sum_ += probe_w;
        DiveResult r = discover(anchors_[id], probe_w, "dive");
        if (r == DiveResult::PlanFound) return Step::PlanFound;
        if (r == DiveResult::Timeout) return Step::Timeout;
        if (r == DiveResult::Unsat) ++dive_unsat_; else ++dive_unknown_;
    }

    // (2) Generate frontier-crossing children gen_w steps ahead (stepping
    // stones).  Full expansion — children carry the search; no reinsertion.
    widen(id, gen_w);
    if (timed_out_) return Step::Timeout;
    return Step::Progress;
}

AnchorSearchWarmPlanner::Step AnchorSearchWarmPlanner::backbone_step() {
    if (backbone_exhausted_) return Step::Retire;
    const int max_steps = Config::instance().planner.max_steps;
    if (backbone_h_ > max_steps) { backbone_exhausted_ = true; return Step::Retire; }
    ++backbone_checks_;
    DiveResult r = discover(anchors_[root_id_], backbone_h_, "backbone");
    if (r == DiveResult::PlanFound) return Step::PlanFound;
    if (r == DiveResult::Timeout) return Step::Timeout;
    if (r == DiveResult::Unknown) return Step::Progress;
    // UNSAT: no plan of length <= backbone_h_.
    ++backbone_unsat_;
    H_unsat_ = std::max(H_unsat_, backbone_h_);
    int next = schedule_.next(backbone_h_, max_steps);
    if (next <= backbone_h_) { backbone_exhausted_ = true; return Step::Retire; }
    backbone_h_ = next;
    return Step::Progress;
}

// ===========================================================================
// Finalisation and signals
// ===========================================================================

bool AnchorSearchWarmPlanner::replay_validate(const Plan& flat) const {
    int len = static_cast<int>(flat.length());
    z3::solver s(ctx_);
    auto prop = propagator_factory_(s);
    z3::params p(ctx_); p.set("random_seed", (unsigned)42); s.set(p);
    s.add(encoder_.encode_initial_state());
    prop->register_timestep_variables(0);
    for (int t = 0; t < len; ++t)
        BasePlanner::add_timestep_constraints(s, encoder_, *prop, t, /*prefix_monotone=*/false);
    auto& factory = encoder_.get_variable_factory();
    const auto& acts = flat.actions();
    for (int t = 0; t < len; ++t) {
        int want = acts[t]->id();
        for (const Action& a : problem_.actions()) {
            const z3::expr& av = factory.get_action_variable(a, t);
            s.add(a.id() == want ? av : !av);
        }
    }
    s.add(encoder_.encode_goal(len == 0 ? 0 : len));
    bool ok = s.check() == z3::sat;
    prop->cleanup();
    return ok;
}

Plan AnchorSearchWarmPlanner::finish(const Plan& flat, int horizon, const char* engine) {
    auto& config = Config::instance();
    auto& stats = Stats::instance();
    double total = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - search_start_).count();
    solution_found_ = true;
    Logger::instance().info(
        std::string("\n*** PLAN FOUND (wc3/") + engine + "): horizon=" +
        std::to_string(horizon) + ", actions=" + std::to_string(flat.length()) +
        " (total time: " + std::to_string(total) + "s) ***");
    flat.write_ipc(config.planner.output_plan, 1, -1.0, true,
                   config.planner.strategy, config.planner.mode, total);
    notify_intermediate_plan(flat);
    stats.set("planner.plan_length", static_cast<double>(flat.length()));
    stats.set("planner.solution_horizon", static_cast<double>(horizon));
    stats.set("planner.total_time", total);
    stats.set("planner.wc3_winning_engine",
              std::strcmp(engine, "backbone") == 0 ? 0.0 : 1.0);  // 0 backbone, 1 dive
    Logger::instance().info(summary_line());
    record_stats();
    collect_statistics();
    propagator_strategy_->cleanup();
    return flat;
}

void AnchorSearchWarmPlanner::record_stats() const {
    auto& s = Stats::instance();
    s.set("planner.wc3_widens", static_cast<double>(widens_));
    s.set("planner.wc3_widen_children", static_cast<double>(widen_children_));
    s.set("planner.wc3_dead_children", static_cast<double>(dead_children_));
    s.set("planner.wc3_closed_hits", static_cast<double>(closed_hits_));
    s.set("planner.wc3_dives", static_cast<double>(dives_));
    s.set("planner.wc3_dive_sat", static_cast<double>(dive_sat_));
    s.set("planner.wc3_dive_unsat", static_cast<double>(dive_unsat_));
    s.set("planner.wc3_dive_unknown", static_cast<double>(dive_unknown_));
    s.set("planner.wc3_backbone_checks", static_cast<double>(backbone_checks_));
    s.set("planner.wc3_backbone_unsat", static_cast<double>(backbone_unsat_));
    s.set("planner.wc3_nogoods_learned", static_cast<double>(nogoods_learned_));
    s.set("planner.wc3_nogood_prunes", static_cast<double>(nogood_prunes_));
    s.set("planner.wc3_throttled_iters", static_cast<double>(throttled_iters_));
    s.set("planner.wc3_H_unsat", static_cast<double>(H_unsat_));
    s.set("planner.wc3_backbone_H", static_cast<double>(backbone_h_));
    s.set("planner.wc3_demotions", static_cast<double>(demotions_));
    s.set("planner.wc3_anchors", static_cast<double>(anchors_.size()));
    s.set("planner.wc3_peak_open", static_cast<double>(peak_open_));
    s.set("planner.wc3_max_depth", static_cast<double>(max_depth_));
    s.set("planner.wc3_iters", static_cast<double>(iters_));
    s.set("planner.wc3_widen_secs", widen_secs_);
    s.set("planner.wc3_dive_secs", dive_secs_);
    s.set("planner.wc3_backbone_secs", backbone_secs_);
    s.set("planner.wc3_avg_dive_window",
          dives_ > 0 ? static_cast<double>(dive_window_sum_) / dives_ : 0.0);
    s.set("planner.wc3_final_solver_depth", static_cast<double>(filled_));
}

std::string AnchorSearchWarmPlanner::summary_line() const {
    auto n2 = [](double v) {
        char b[32]; std::snprintf(b, sizeof b, "%.2f", v); return std::string(b);
    };
    return "[wc3] iters=" + std::to_string(iters_) +
           " anchors=" + std::to_string(anchors_.size()) +
           " peak_open=" + std::to_string(peak_open_) +
           " max_depth=" + std::to_string(max_depth_) +
           " | widen=" + std::to_string(widens_) +
           " (children=" + std::to_string(widen_children_) +
           " dead=" + std::to_string(dead_children_) + ")" +
           " dive=" + std::to_string(dives_) +
           " (uns=" + std::to_string(dive_unsat_) +
           " unk=" + std::to_string(dive_unknown_) +
           " avgw=" + n2(dives_ ? double(dive_window_sum_) / dives_ : 0.0) + ")" +
           " backbone=" + std::to_string(backbone_checks_) +
           " (H=" + std::to_string(backbone_h_) + " Huns=" + std::to_string(H_unsat_) + ")" +
           " nogoods=" + std::to_string(nogoods_learned_) +
           " (prunes=" + std::to_string(nogood_prunes_) + ")" +
           " throttle=" + std::to_string(throttled_iters_) +
           " | secs widen=" + n2(widen_secs_) +
           " dive=" + n2(dive_secs_) +
           " backbone=" + n2(backbone_secs_) +
           " solver_depth=" + std::to_string(filled_);
}

void AnchorSearchWarmPlanner::trace(const char* event) {
    if (!trace_on_) return;
    int best = best_open();
    Logger::instance().info(
        "[wc3] " + std::string(event) +
        " it=" + std::to_string(iters_) +
        " open=" + std::to_string(open_d_.size()) +
        " bestdff=" + std::to_string(best >= 0 ? anchors_[best].d_ff : -1) +
        " depth=" + std::to_string(max_depth_) +
        " H=" + std::to_string(backbone_h_) + "/" + std::to_string(H_unsat_) +
        " widen=" + std::to_string(widens_) +
        " dive=" + std::to_string(dives_) +
        "(sat" + std::to_string(dive_sat_) + "/uns" + std::to_string(dive_unsat_) +
        "/unk" + std::to_string(dive_unknown_) + ")");
}

// ===========================================================================
// search()
// ===========================================================================

Plan AnchorSearchWarmPlanner::search() {
    auto& config = Config::instance();
    schedule_ = HorizonSchedule::from_string(config.planner.horizon_schedule);
    solution_found_ = false;
    timed_out_ = false;
    init_deadline();
    search_start_ = std::chrono::steady_clock::now();

    Logger::instance().info(
        "Starting anchor search (els-wc3, one warm solver): window_cap=" +
        std::to_string(window_cap_) + ", C=" + std::to_string(config.planner.wc_candidates) +
        ", propagator: " + propagator_strategy_->get_name() +
        ", timeout: " + format_timeout_string() +
        (dive_rlimit_ ? " [dive-rlimit=" + std::to_string(dive_rlimit_) + "]" : "") +
        (backbone_only_ ? " [backbone-only]" : ""));

    ensure_started();
    initialize_achievers();

    Anchor root;
    if (!extract_root(root)) {
        Logger::instance().info("els-wc3: no start anchor — reporting no plan.");
        record_stats();
        collect_statistics();
        propagator_strategy_->cleanup();
        return Plan();
    }
    root.id = 0;
    root_id_ = 0;
    anchors_.push_back(std::move(root));
    closed_[anchors_[0].sigma_hash] = {0, false};
    push_open(0);
    greedy_alive_ = !backbone_only_;
    backbone_h_ = std::max(config.planner.start_timestep,
                           static_cast<int>(std::ceil(
                               anchors_[0].lb < kInf ? anchors_[0].lb : 0.0)));
    if (backbone_h_ < 1) backbone_h_ = 1;

    while (greedy_alive_ || !backbone_exhausted_) {
        if (remaining_ms() == 0) { timed_out_ = true; break; }
        ++iters_;

        // k fast greedy steps per one (often expensive) backbone check, so the
        // greedy layer can find the plan before the backbone's deep solve blocks.
        // Tier 1: throttle k to 1 once the backbone horizon has caught up to the
        // root relaxed-plan depth estimate — the regime where greedy exploration
        // is unamortized (backbone in its closing zone, or a gross under-estimate
        // the greedy layer is overshooting on).  Below the estimate the greedy
        // layer is doing the reaching, so keep the full ratio.  Logical predicate,
        // no clocks.
        int eff_ratio = greedy_ratio_;
        if (dyn_ratio_ && root_id_ >= 0 && anchors_[root_id_].d_ff > 0 &&
            backbone_h_ >= anchors_[root_id_].d_ff) {
            eff_ratio = 1;
            ++throttled_iters_;
        }
        for (int k = 0; k < eff_ratio && greedy_alive_; ++k) {
            if (remaining_ms() == 0) { timed_out_ = true; break; }
            Step s = greedy_step();
            if (s == Step::PlanFound) return found_plan_;
            if (s == Step::Timeout) { timed_out_ = true; break; }
            if (s == Step::Retire) {
                greedy_alive_ = false;
                Logger::instance().info("els-wc3: greedy frontier exhausted.");
            }
        }
        if (timed_out_) break;
        if (!backbone_exhausted_) {
            Step s = backbone_step();
            if (s == Step::PlanFound) return found_plan_;
            if (s == Step::Timeout) { timed_out_ = true; break; }
            if (s == Step::Retire) {
                // No plan of length <= max_steps: a completeness proof.
                backbone_exhausted_ = true;
                greedy_alive_ = false;
            }
        }
        if (trace_on_ && (iters_ % 20 == 0)) trace("tick");
    }

    Logger::instance().info(timed_out_
        ? "els-wc3: no plan found (timeout)."
        : "\n*** NO PLAN FOUND within " + std::to_string(config.planner.max_steps) +
          " timesteps ***");
    Logger::instance().info(summary_line());
    record_stats();
    collect_statistics();
    propagator_strategy_->cleanup();
    return Plan();
}

} // namespace rantanplan
