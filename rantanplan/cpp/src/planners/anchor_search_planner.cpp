#include "anchor_search_planner.hpp"
#include "../analysis/numeric_relaxed_planning_graph.hpp"
#include "../passes/landmark_analysis_pass.hpp"
#include "../preprocessing/preprocessed_problem.hpp"
#include "../config/config.hpp"
#include "../util/logger.hpp"
#include "../util/stats.hpp"
#include "../util/memory_tracker.hpp"
#include <algorithm>
#include <cassert>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <iterator>
#include <numeric>
#include <string>

namespace rantanplan {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

/// Model numeral -> double (int fast path, then "num/den" rationals, then
/// decimal strings).  Mirrors the els-wc extractor.
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

AnchorSearchPlanner::AnchorSearchPlanner(const Problem& problem, BaseEncoder& encoder,
                                         z3::context& ctx)
    : BasePlanner(problem, encoder, ctx) {
    greedy_only_ = std::getenv("RANTANPLAN_WC2_GREEDY_ONLY") != nullptr;
    novelty_queue_ = std::getenv("RANTANPLAN_WC2_NOVELTY_QUEUE") != nullptr;
    dyn_window_ = std::getenv("RANTANPLAN_WC2_DYN_WINDOW") != nullptr;
    if (const char* rl = std::getenv("RANTANPLAN_WC2_PROBE_RLIMIT")) {
        probe_rlimit_ = static_cast<unsigned>(std::max(0L, std::atol(rl)));
    }
    if (const char* gr = std::getenv("RANTANPLAN_WC2_GREEDY_RATIO")) {
        greedy_ratio_ = std::max(1, static_cast<int>(std::atol(gr)));
    }

    // Dynamic ground fluents: those targeted by some action effect.  Only
    // these have Z3 state variables at every timestep (frame axioms create
    // them); statics are read from the initial state instead.
    std::unordered_set<int> dynamic_ids;
    for (const Action& action : problem_.actions()) {
        for (const Effect& effect : action.effects()) {
            int fid = problem_.find_grounded_fluent_index(
                effect.effect_expression().fluent_id());
            if (fid >= 0) dynamic_ids.insert(fid);
        }
    }
    dynamic_fluent_ids_.assign(dynamic_ids.begin(), dynamic_ids.end());
    std::sort(dynamic_fluent_ids_.begin(), dynamic_fluent_ids_.end());
    dynamic_fluent_eids_.reserve(dynamic_fluent_ids_.size());
    for (int fid : dynamic_fluent_ids_) {
        dynamic_fluent_eids_.push_back(problem_.grounded_fluent(fid));
    }

    // Full valuation of the (CWA-completed) initial state: the source for
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

void AnchorSearchPlanner::consume_artifacts(PreprocessedProblem& artifacts) {
    landmark_data_ = artifacts.landmark_data;
    if (artifacts.achievers) {
        achievers_ = std::move(artifacts.achievers);
    }
}

void AnchorSearchPlanner::initialize_achievers() {
    if (!achievers_) {
        achievers_ = std::make_unique<AchieversAnalysis>(problem_);
    }
    for (ExprID cond : achievers_->get_all_conditions()) {
        auto& ptrs = condition_achievers_[cond];
        for (const Action& a : achievers_->get_achievers(cond)) {
            auto it = action_by_id_.find(a.id());
            if (it != action_by_id_.end()) ptrs.push_back(it->second);
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
    // Panino novelty feature space: achiever conditions + goal conditions.
    std::unordered_set<ExprID> feature_set;
    for (const auto& [cond, ptrs] : condition_achievers_) feature_set.insert(cond);
    for (ExprID cond : goal_condition_ids_) feature_set.insert(cond);
    novelty_feature_ids_.assign(feature_set.begin(), feature_set.end());
}

// ---------------------------------------------------------------------------
// Queue order
// ---------------------------------------------------------------------------

bool AnchorSearchPlanner::QueueOrder::operator()(int a, int b) const {
    const Node& na = (*nodes)[a];
    const Node& nb = (*nodes)[b];
    if (na.D != nb.D) return na.D < nb.D;                   // demotions primary
    if (mode == 2) {                                        // novelty-primary queue
        if (na.novel != nb.novel) return na.novel > nb.novel;  // novel first
        if (na.h_ff != nb.h_ff) return na.h_ff < nb.h_ff;
        if (na.u != nb.u) return na.u < nb.u;
        if (na.t != nb.t) return na.t < nb.t;
        return a < b;
    }
    int a1 = (mode == 1) ? na.u : na.h_ff;
    int b1 = (mode == 1) ? nb.u : nb.h_ff;
    if (a1 != b1) return a1 < b1;
    int a2 = (mode == 1) ? na.h_ff : na.u;
    int b2 = (mode == 1) ? nb.h_ff : nb.u;
    if (a2 != b2) return a2 < b2;
    if (na.t != nb.t) return na.t < nb.t;
    if (na.novel != nb.novel) return na.novel > nb.novel;  // novel first
    return a < b;  // total-order tiebreak
}

// ---------------------------------------------------------------------------
// State extraction
// ---------------------------------------------------------------------------

StateValuation AnchorSearchPlanner::extract_state(
    const z3::model& model, int t, std::vector<z3::expr>& dynamic_values) const {
    StateValuation state = base_valuation_;  // statics from the initial state
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

z3::expr AnchorSearchPlanner::state_equals(
    int t, const std::vector<z3::expr>& values) const {
    z3::expr_vector equalities(ctx_);
    for (size_t i = 0; i < dynamic_fluent_eids_.size(); ++i) {
        z3::expr var = encoder_.convert_expr_id_to_z3(dynamic_fluent_eids_[i], t);
        equalities.push_back(var == values[i]);
    }
    if (equalities.empty()) return ctx_.bool_val(true);
    return z3::mk_and(equalities);
}

// ---------------------------------------------------------------------------
// Anchor-state scoring (RPG) and the FF relaxed-plan heuristic
// ---------------------------------------------------------------------------

bool AnchorSearchPlanner::score_anchor(const StateValuation& sigma, Node& out) const {
    NumericRelaxedPlanningGraph rpg(problem_, sigma);
    bool goals_reachable = rpg.build();
    out.first_layers = rpg.get_action_first_layers();
    out.rpg_action_layers = static_cast<int>(rpg.get_layer_count()) - 1;
    out.rpg_saw_all_actions = out.first_layers.size() == problem_.actions().size();
    if (!goals_reachable) {
        out.lb = kInf;  // sound dead-end veto: relaxed-unreachable => unreachable
        return false;
    }
    int lb = rpg.get_minimum_steps_lower_bound();
    out.lb = lb < 0 ? kInf : static_cast<double>(lb);
    return out.lb < kInf;
}

int AnchorSearchPlanner::relaxed_plan_size(
    const z3::model& model, int t,
    const std::unordered_map<int, int>& first_layers, int* depth_out) const {
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

std::vector<int> AnchorSearchPlanner::unachieved_landmark_indices(
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

std::shared_ptr<const LandmarkData> AnchorSearchPlanner::recompute_landmarks(
    const StateValuation& sigma, const std::vector<ExprID>& sat_conditions) const {
    std::unordered_set<ExprID> sat(sat_conditions.begin(), sat_conditions.end());
    auto data = std::make_shared<LandmarkData>(LandmarkAnalysisPass::compute(
        problem_, *achievers_,
        [&](ExprID cond) { return sat.count(cond) > 0; },
        [&](ExprID fluent) -> std::optional<double> {
            int fid = problem_.find_grounded_fluent_index(fluent);
            if (fid < 0 || fid >= static_cast<int>(sigma.values.size())) {
                return std::nullopt;
            }
            const auto& v = sigma.values[static_cast<size_t>(fid)];
            if (v.kind != StateValuation::Kind::Numeric) return std::nullopt;
            return v.numeric_value;
        }));
    return data;
}

// ---------------------------------------------------------------------------
// Landmark frontier + steering + masks + orderings (clean, solver-parametric)
// ---------------------------------------------------------------------------

std::vector<int> AnchorSearchPlanner::landmark_frontier(
    const std::vector<int>& open_idx, const LandmarkData& data) const {
    std::unordered_set<int> open(open_idx.begin(), open_idx.end());
    std::unordered_set<int> blocked;
    for (const auto& [earlier, later] : data.orderings) {
        if (open.count(earlier) && open.count(later)) blocked.insert(later);
    }
    std::vector<int> frontier;
    for (int idx : open_idx) {
        if (!blocked.count(idx)) frontier.push_back(idx);
    }
    return frontier;
}

z3::expr AnchorSearchPlanner::steering_disjunction(
    const std::vector<int>& frontier_idx, int t_begin, int t_end,
    const LandmarkData& data) const {
    auto& factory = encoder_.get_variable_factory();
    z3::expr_vector fires(ctx_);
    for (int idx : frontier_idx) {
        const LandmarkInfo& lm = data.landmarks[static_cast<size_t>(idx)];
        for (int action_id : lm.achiever_action_ids) {
            auto it = action_by_id_.find(action_id);
            if (it == action_by_id_.end()) continue;
            for (int t = t_begin; t < t_end; ++t) {
                fires.push_back(factory.get_action_variable(*it->second, t));
            }
        }
    }
    if (fires.empty()) return ctx_.bool_val(true);
    return z3::mk_or(fires);
}

void AnchorSearchPlanner::assert_masks(
    z3::solver& s, int t_anchor, int begin_off, int end_off,
    const std::unordered_map<int, int>& first_layers, int rpg_action_layers,
    bool saw_all_actions) const {
    bool absent_means_unreachable =
        saw_all_actions || !Config::instance().rpg.early_goal_termination;
    auto& factory = encoder_.get_variable_factory();
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
            s.add(!av);
        }
    }
}

void AnchorSearchPlanner::assert_ordering_clauses(
    z3::solver& s, int t_anchor, int begin_off, int end_off,
    const std::vector<int>& open_idx, const LandmarkData& data) const {
    if (data.orderings.empty() || open_idx.empty()) return;
    std::unordered_set<int> open(open_idx.begin(), open_idx.end());
    auto& factory = encoder_.get_variable_factory();
    for (const auto& [earlier, later] : data.orderings) {
        if (!open.count(earlier)) continue;  // achieved at the anchor: vacuous
        const LandmarkInfo& lm_a = data.landmarks[static_cast<size_t>(earlier)];
        const LandmarkInfo& lm_b = data.landmarks[static_cast<size_t>(later)];
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
                s.add(z3::implies(
                    factory.get_action_variable(*it->second, t), a_before));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Panino novelty (tie-break only; tables consulted at M6)
// ---------------------------------------------------------------------------

void AnchorSearchPlanner::extract_novelty_features(
    const z3::model& model, int t, std::vector<ExprID>& psi,
    std::vector<std::pair<ExprID, double>>& delta,
    std::vector<ExprID>& sat_conditions) const {
    psi.clear();
    delta.clear();
    sat_conditions.clear();
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
            double d = (kind == Z3_OP_GE || kind == Z3_OP_GT)
                           ? rhs - lhs
                           : (kind == Z3_OP_EQ ? std::abs(lhs - rhs) : lhs - rhs);
            delta.emplace_back(g, d);
            if (model.eval(cond, /*model_completion=*/true).is_true()) {
                sat_conditions.push_back(g);
            }
        } else if (model.eval(cond, /*model_completion=*/true).is_true()) {
            psi.push_back(g);
            sat_conditions.push_back(g);
        }
    }
}

bool AnchorSearchPlanner::is_novel(
    int h_ff, int u, const std::vector<ExprID>& psi,
    const std::vector<std::pair<ExprID, double>>& delta) const {
    NoveltyPartition key{h_ff, u};
    auto psi_it = psi_seen_.find(key);
    if (psi_it == psi_seen_.end()) {
        return !psi.empty() || !delta.empty();  // first sample of the partition
    }
    for (ExprID g : psi) {
        if (!psi_it->second.count(g)) return true;
    }
    auto delta_it = delta_min_.find(key);
    for (const auto& [g, d] : delta) {
        if (delta_it == delta_min_.end()) return true;
        auto min_it = delta_it->second.find(g);
        if (min_it == delta_it->second.end() || d < min_it->second) return true;
    }
    return false;
}

void AnchorSearchPlanner::update_novelty_tables(
    int h_ff, int u, const std::vector<ExprID>& psi,
    const std::vector<std::pair<ExprID, double>>& delta) {
    NoveltyPartition key{h_ff, u};
    auto& seen = psi_seen_[key];
    for (ExprID g : psi) seen.insert(g);
    auto& mins = delta_min_[key];
    for (const auto& [g, d] : delta) {
        auto [it, inserted] = mins.emplace(g, d);
        if (!inserted && d < it->second) it->second = d;
    }
}

// ---------------------------------------------------------------------------
// Oracle: a fresh bounded els encoding with the init override (§4, §9)
// ---------------------------------------------------------------------------

AnchorSearchPlanner::Oracle AnchorSearchPlanner::build_oracle(
    const Node& n, int w, bool with_guidance) const {
    Oracle o;
    o.solver = std::make_unique<z3::solver>(ctx_);
    o.prop = propagator_factory_(*o.solver);
    z3::params p(ctx_);
    p.set("random_seed", (unsigned)42);
    const char* logic = recommended_logic(arithmetic_profile_, problem_.all_integer());
    if (logic) p.set("logic", logic);
    o.solver->set(p);
    // INIT OVERRIDE: step-0 dynamic fluents pinned to the anchor state.
    o.solver->add(state_equals(0, n.sigma_values));
    o.prop->register_timestep_variables(0);
    o.filled = 0;
    for (int t = 0; t < w; ++t) {
        BasePlanner::add_timestep_constraints(*o.solver, encoder_, *o.prop, t,
                                              /*prefix_monotone=*/true);
    }
    o.filled = w;
    if (with_guidance && n.lm) {
        assert_masks(*o.solver, 0, 0, w, n.first_layers, n.rpg_action_layers,
                     n.rpg_saw_all_actions);
        std::vector<int> open(n.lm->landmarks.size());
        std::iota(open.begin(), open.end(), 0);
        assert_ordering_clauses(*o.solver, 0, 0, w, open, *n.lm);
    }
    return o;
}

void AnchorSearchPlanner::extend_oracle(Oracle& o, const Node& n, int from_w,
                                        int to_w, bool with_guidance) const {
    for (int t = from_w; t < to_w; ++t) {
        BasePlanner::add_timestep_constraints(*o.solver, encoder_, *o.prop, t,
                                              /*prefix_monotone=*/true);
    }
    o.filled = to_w;
    if (with_guidance && n.lm) {
        assert_masks(*o.solver, 0, from_w, to_w, n.first_layers,
                     n.rpg_action_layers, n.rpg_saw_all_actions);
        std::vector<int> open(n.lm->landmarks.size());
        std::iota(open.begin(), open.end(), 0);
        assert_ordering_clauses(*o.solver, 0, from_w, to_w, open, *n.lm);
    }
}

// ---------------------------------------------------------------------------
// Sound internal replay: force the serialized plan into a fresh native-init
// encoding and check the goal holds at its end.
// ---------------------------------------------------------------------------

bool AnchorSearchPlanner::replay_validate(const Plan& flat) const {
    int len = static_cast<int>(flat.length());
    z3::solver s(ctx_);
    auto prop = propagator_factory_(s);
    z3::params p(ctx_);
    p.set("random_seed", (unsigned)42);
    const char* logic = recommended_logic(arithmetic_profile_, problem_.all_integer());
    if (logic) p.set("logic", logic);
    s.set(p);
    s.add(encoder_.encode_initial_state());
    prop->register_timestep_variables(0);
    for (int t = 0; t < len; ++t) {
        BasePlanner::add_timestep_constraints(s, encoder_, *prop, t,
                                              /*prefix_monotone=*/false);
    }
    auto& factory = encoder_.get_variable_factory();
    const auto& acts = flat.actions();
    for (int t = 0; t < len; ++t) {
        int want = acts[t]->id();
        for (const Action& a : problem_.actions()) {
            const z3::expr& av = factory.get_action_variable(a, t);
            s.add(a.id() == want ? av : !av);
        }
    }
    if (len == 0) s.add(encoder_.encode_goal(0));
    else s.add(encoder_.encode_goal(len));
    bool ok = s.check() == z3::sat;
    prop->cleanup();
    return ok;
}

// ---------------------------------------------------------------------------
// M2 self-test: the init override reproduces native init, and a mid-plan
// state override plus concatenation replay-validates.
// ---------------------------------------------------------------------------

void AnchorSearchPlanner::selftest_init_override() {
    auto& log = Logger::instance();
    auto cr = [](z3::check_result r) -> std::string {
        return r == z3::sat ? "sat" : (r == z3::unsat ? "unsat" : "unknown");
    };
    log.info("[WC2-SELFTEST] init-override self-test starting");
    if (!propagator_factory_) {
        log.info("[WC2-SELFTEST] SKIP: no propagator factory");
        return;
    }
    // Native scratch solver: get the s0 model.
    z3::solver n(ctx_);
    auto nprop = propagator_factory_(n);
    z3::params p(ctx_);
    p.set("random_seed", (unsigned)42);
    const char* logic = recommended_logic(arithmetic_profile_, problem_.all_integer());
    if (logic) p.set("logic", logic);
    n.set(p);
    n.add(encoder_.encode_initial_state());
    nprop->register_timestep_variables(0);
    if (n.check() != z3::sat) {
        log.info("[WC2-SELFTEST] SKIP: initial state unsat");
        return;
    }
    Node root0;
    root0.t = 0;
    StateValuation s0 = extract_state(n.get_model(), 0, root0.sigma_values);

    // Property (a): override with s0 reproduces native init at every horizon.
    const int K = 6;
    bool pass_a = true;
    int first_sat = -1;
    int nfilled = 0;
    for (int H = 0; H <= K; ++H) {
        for (int t = nfilled; t < H; ++t) {
            BasePlanner::add_timestep_constraints(n, encoder_, *nprop, t, true);
        }
        nfilled = std::max(nfilled, H);
        z3::expr_vector ga(ctx_);
        ga.push_back(encoder_.encode_goal(H));
        z3::check_result native = n.check(ga);
        Oracle o = build_oracle(root0, H, /*with_guidance=*/false);
        z3::expr_vector go(ctx_);
        go.push_back(encoder_.encode_goal(H));
        z3::check_result over = o.solver->check(go);
        bool same = (native == over);
        if (!same) pass_a = false;
        log.info("[WC2-SELFTEST]  (a) H=" + std::to_string(H) +
                 " native=" + cr(native) + " override=" + cr(over) +
                 (same ? " OK" : " MISMATCH"));
        if (native == z3::sat && first_sat < 0) first_sat = H;
    }
    log.info(std::string("[WC2-SELFTEST] property (a) ") +
             (pass_a ? "PASS" : "FAIL"));

    // Property (b): split a native plan at a mid state, re-solve the suffix
    // from that state via the override, concatenate, and replay-validate.
    if (first_sat >= 2) {
        z3::expr_vector ga(ctx_);
        ga.push_back(encoder_.encode_goal(first_sat));
        if (n.check(ga) == z3::sat) {
            z3::model m1 = n.get_model();
            int k = std::max(1, first_sat / 2);
            Node midk;
            midk.t = k;
            StateValuation sk = extract_state(m1, k, midk.sigma_values);
            Plan prefix = encoder_.extract_plan(m1, k);
            int suffix_w = first_sat - k;
            Oracle o2 = build_oracle(midk, suffix_w, /*with_guidance=*/false);
            z3::expr_vector gs(ctx_);
            gs.push_back(encoder_.encode_goal(suffix_w));
            z3::check_result r2 = o2.solver->check(gs);
            if (r2 == z3::sat) {
                Plan suffix = encoder_.extract_plan(o2.solver->get_model(), suffix_w);
                Plan concat;
                for (const Action* a : prefix.actions()) concat.add_action(a);
                for (const Action* a : suffix.actions()) concat.add_action(a);
                bool valid = replay_validate(concat);
                log.info("[WC2-SELFTEST]  (b) split k=" + std::to_string(k) +
                         " prefix=" + std::to_string(prefix.length()) +
                         " suffix=" + std::to_string(suffix.length()) +
                         " concat=" + std::to_string(concat.length()) +
                         (valid ? "  replay OK" : "  replay INVALID"));
                log.info(std::string("[WC2-SELFTEST] property (b) ") +
                         (valid ? "PASS" : "FAIL"));
            } else {
                log.info("[WC2-SELFTEST]  (b) suffix solve not SAT — inconclusive");
            }
        }
    } else {
        log.info("[WC2-SELFTEST]  (b) skipped (first SAT horizon < 2)");
    }
    if (nprop) nprop->cleanup();
    log.info("[WC2-SELFTEST] done");
}

// ---------------------------------------------------------------------------
// Plan finalisation
// ---------------------------------------------------------------------------

void AnchorSearchPlanner::record_final_stats() {
    auto& stats = Stats::instance();
    stats.set("planner.wc2_peak_open", static_cast<double>(peak_open_));
    stats.set("planner.wc2_final_H_unsat", static_cast<double>(H_unsat_));
    stats.set("planner.wc2_expansion_build_secs", expansion_build_secs_);
    stats.set("planner.wc2_nodes", static_cast<double>(nodes_.size()));
}

Plan AnchorSearchPlanner::finish_with_plan_flat(const Plan& plan, int reported_horizon,
                                                const char* engine) {
    auto& config = Config::instance();
    auto& stats = Stats::instance();
    double total_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - search_start_).count();
    solution_found_ = true;
    Logger::instance().info(
        std::string("\n*** PLAN FOUND (") + engine + "): horizon=" +
        std::to_string(reported_horizon) + ", actions=" +
        std::to_string(plan.length()) +
        " (total time: " + std::to_string(total_time) + "s) ***");
    plan.write_ipc(config.planner.output_plan, 1, -1.0, true,
                   config.planner.strategy, config.planner.mode, total_time);
    notify_intermediate_plan(plan);
    stats.set("planner.plan_length", static_cast<double>(plan.length()));
    stats.set("planner.solution_horizon", static_cast<double>(reported_horizon));
    stats.set("planner.total_time", total_time);
    stats.set("planner.wc2_winning_engine",
              std::string(engine) == "root" ? 0.0
              : (std::string(engine) == "suffix" ? 1.0 : 2.0));
    record_final_stats();
    collect_statistics();
    propagator_strategy_->cleanup();
    if (suffix_prop_) suffix_prop_->cleanup();
    return plan;
}

// ---------------------------------------------------------------------------
// Root sweep lane (§7.1) — the completeness story, plain els from s0.
// ---------------------------------------------------------------------------

void AnchorSearchPlanner::ensure_root_started() {
    if (root_started_) return;
    auto& config = Config::instance();
    solver_.add(encoder_.encode_initial_state());
    propagator_strategy_->register_timestep_variables(0);
    int start = std::max(0, config.planner.start_timestep);
    for (int t = 0; t < start; ++t) {
        BasePlanner::add_timestep_constraints(solver_, encoder_,
                                              *propagator_strategy_, t,
                                              /*prefix_monotone=*/true);
    }
    root_filled_ = start;
    root_h_ = start;
    root_started_ = true;
}

AnchorSearchPlanner::TurnResult AnchorSearchPlanner::root_sweep_turn() {
    auto& config = Config::instance();
    auto& stats = Stats::instance();
    if (root_exhausted_) return TurnResult::Retire;
    if (remaining_ms() == 0) { timed_out_ = true; return TurnResult::Timeout; }
    ensure_root_started();

    // §7.1 horizon jump: prefix-monotonicity makes one check at H cover every
    // horizon <= H, so the completeness lane may skip ahead to the greedy
    // layer's own total-length estimate (max over open nodes of t + d_ff)
    // instead of grinding the UNSAT wall below it.  Forward only.
    int jump = std::min(best_horizon_target(), config.planner.max_steps);
    if (jump > root_h_) {
        root_h_ = jump;
        stats.add("planner.wc2_horizon_jumps");
    }
    if (root_h_ > config.planner.max_steps) {
        root_exhausted_ = true;
        return TurnResult::Retire;
    }

    // Fill skipped transitions [root_filled_, root_h_).
    for (int t = root_filled_; t < root_h_; ++t) {
        BasePlanner::add_timestep_constraints(solver_, encoder_,
                                              *propagator_strategy_, t,
                                              /*prefix_monotone=*/true);
    }
    if (root_h_ > root_filled_) root_filled_ = root_h_;

    // One goal literal at the scheduled horizon.
    auto it = root_goal_lits_.find(root_h_);
    z3::expr goal_lit = (it != root_goal_lits_.end())
        ? it->second
        : [&] {
            z3::expr lit = ctx_.bool_const(("goal_at_" + std::to_string(root_h_)).c_str());
            solver_.add(z3::implies(lit, encoder_.encode_goal(root_h_)));
            root_goal_lits_.emplace(root_h_, lit);
            return lit;
        }();

    if (!apply_solver_timeout(solver_)) { timed_out_ = true; return TurnResult::Timeout; }
    z3::expr_vector assumptions(ctx_);
    assumptions.push_back(goal_lit);
    auto solve_start = std::chrono::steady_clock::now();
    z3::check_result result = solver_.check(assumptions);
    double solve_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - solve_start).count();
    stats.add("planner.timesteps_explored");
    stats.add("planner.solve_time", solve_time);
    Logger::instance().timestep_solving(VerbosityLevel::INFO, root_h_, {
        {"solve", std::to_string(solve_time) + "s"},
        {"mem", std::to_string(static_cast<int>(
             MemoryTracker::instance().get_current_memory_mb())) + "MB"}
    }, "R");

    if (result == z3::sat) {
        Plan plan = encoder_.extract_plan(solver_.get_model(), root_h_);
        found_plan_ = finish_with_plan_flat(plan, root_h_, "root");
        return TurnResult::PlanFound;
    }
    if (result == z3::unknown) {
        if (remaining_ms() == 0) timed_out_ = true;
        return timed_out_ ? TurnResult::Timeout : TurnResult::Progress;
    }
    // UNSAT: no plan of length <= root_h_.  Global mute for goal queries.
    H_unsat_ = std::max(H_unsat_, root_h_);
    int next_h = schedule_.next(root_h_, config.planner.max_steps);
    if (next_h <= root_h_) {
        root_exhausted_ = true;
        Logger::instance().info("Root sweep: max_steps exhausted.");
        return TurnResult::Retire;
    }
    root_h_ = next_h;
    return TurnResult::Progress;
}

// ---------------------------------------------------------------------------
// Greedy layer: open queues, node construction, Expand (§3, §6)
// ---------------------------------------------------------------------------

int AnchorSearchPlanner::informed_window(
    const std::unordered_map<int, int>& first_layers, const LandmarkData& lm,
    int d_ff) const {
    int window = std::max(1, Config::instance().planner.wc_window);
    std::vector<int> open(lm.landmarks.size());
    std::iota(open.begin(), open.end(), 0);
    std::vector<int> front = landmark_frontier(open, lm);
    int informed = INT_MAX;
    for (int idx : front) {
        for (int aid : lm.landmarks[static_cast<size_t>(idx)].achiever_action_ids) {
            auto it = first_layers.find(aid);
            if (it != first_layers.end()) informed = std::min(informed, it->second);
        }
    }
    if (informed != INT_MAX) window = std::max(window, informed);
    // Dynamic window (RANTANPLAN_WC2_DYN_WINDOW): widen to the heuristic
    // estimate of remaining depth (relaxed-plan parallel depth d_ff), as the
    // late els-wc did — deeper informed strides that can reach a productive
    // frontier or the goal in one shot instead of iterating widens.  Bounded
    // by max_steps in the caller.
    if (dyn_window_ && d_ff > window) window = d_ff;
    return window;
}

int AnchorSearchPlanner::make_node(int parent, Plan prefix, int t,
                                   const StateValuation& sigma,
                                   std::vector<z3::expr> sigma_values,
                                   const z3::model& model, int model_t, int U0) {
    Node child;
    if (!score_anchor(sigma, child)) return -1;  // relaxed-dead veto
    child.id = static_cast<int>(nodes_.size());
    child.parent = parent;
    child.sigma_values = std::move(sigma_values);
    child.sigma_hash = sigma.hash();
    child.prefix = std::move(prefix);
    child.t = t;
    child.h_ff = relaxed_plan_size(model, model_t, child.first_layers, &child.d_ff);
    std::vector<ExprID> psi, sat;
    std::vector<std::pair<ExprID, double>> delta;
    extract_novelty_features(model, model_t, psi, delta, sat);
    child.lm = recompute_landmarks(sigma, sat);
    child.u = static_cast<int>(child.lm->landmarks.size());
    child.novel = is_novel(child.h_ff, child.u, psi, delta);
    update_novelty_tables(child.h_ff, child.u, psi, delta);
    int cap = std::max(1, Config::instance().planner.max_steps - t);
    child.window = std::min(cap,
        informed_window(child.first_layers, *child.lm, child.d_ff));
    child.U = U0;
    child.D = 0;
    int id = child.id;
    nodes_.push_back(std::move(child));
    return id;
}

void AnchorSearchPlanner::push_open(int id) {
    open_h_.insert(id);
    open_u_.insert(id);
    if (novelty_queue_) open_n_.insert(id);
    peak_open_ = std::max(peak_open_, open_h_.size());
}

void AnchorSearchPlanner::erase_open(int id) {
    open_h_.erase(id);
    open_u_.erase(id);
    if (novelty_queue_) open_n_.erase(id);
}

int AnchorSearchPlanner::pop_open() {
    if (open_h_.empty()) return -1;
    // Diagnostic: how often the h-queue top ties the runner-up on (D, h_ff, u)
    // — both heuristics uninformative at the frontier (motivates the novelty
    // queue).  Cheap: a single next-element peek.
    if (open_h_.size() >= 2) {
        auto it = open_h_.begin();
        const Node& top = nodes_[*it];
        const Node& next = nodes_[*std::next(it)];
        if (top.D == next.D && top.h_ff == next.h_ff && top.u == next.u) {
            Stats::instance().add("planner.wc2_flat_frontier_pops");
        }
    }
    int n_queues = novelty_queue_ ? 3 : 2;
    int turn = pop_turn_ % n_queues;
    pop_turn_ = (pop_turn_ + 1) % n_queues;
    int id;
    if (turn == 0) id = *open_h_.begin();
    else if (turn == 1) id = *open_u_.begin();
    else id = *open_n_.begin();
    erase_open(id);
    return id;
}

int AnchorSearchPlanner::best_estimate_node() const {
    int best = -1;
    long best_val = std::numeric_limits<long>::max();
    for (int id : open_h_) {
        long v = static_cast<long>(nodes_[id].t) + nodes_[id].d_ff;
        if (v < best_val) { best_val = v; best = id; }
    }
    return best;
}

int AnchorSearchPlanner::best_horizon_target() const {
    long best = 0;
    for (int id : open_h_) {
        best = std::max(best, static_cast<long>(nodes_[id].t) + nodes_[id].d_ff);
    }
    return static_cast<int>(best);
}

AnchorSearchPlanner::TurnResult AnchorSearchPlanner::greedy_turn() {
    int id = pop_open();
    if (id < 0) return TurnResult::Retire;
    return expand(id);
}

AnchorSearchPlanner::TurnResult AnchorSearchPlanner::expand(int node_id) {
    auto& config = Config::instance();
    auto& stats = Stats::instance();
    stats.add("planner.wc2_expansions");

    // Snapshot the fields that stay fixed during this expansion (nodes_ may
    // reallocate when make_node pushes children — never hold a Node& across it).
    int w = nodes_[node_id].window;
    int t = nodes_[node_id].t;
    int U = nodes_[node_id].U;
    std::shared_ptr<const LandmarkData> lm = nodes_[node_id].lm;
    Plan base_prefix = nodes_[node_id].prefix;

    std::vector<int> open(lm->landmarks.size());
    std::iota(open.begin(), open.end(), 0);
    std::vector<int> frontier = landmark_frontier(open, *lm);

    auto build_start = std::chrono::steady_clock::now();
    Oracle o = build_oracle(nodes_[node_id], w, /*with_guidance=*/true);
    for (const auto& sv : nodes_[node_id].served) {
        o.solver->add(!state_equals(w, sv));  // re-block prior children (cheap)
    }
    expansion_build_secs_ += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - build_start).count();

    // §6.1 Goal probe (gated): all three gates are sound mutes — a query that
    // must be UNSAT is skipped rather than run.  t+w<=H_unsat: the root sweep
    // already refuted every plan of length <= H_unsat.  w<=U: the suffix lane
    // refuted every completion within U steps.  w<d_LM: the count landmarks
    // refute any plan shorter than d_LM from this anchor.
    {
        int d_lm = lm->depth_lower_bound;
        if (t + w > H_unsat_ && w > nodes_[node_id].U && w >= d_lm) {
            if (!apply_solver_timeout(*o.solver)) return TurnResult::Timeout;
            z3::expr_vector ga(ctx_);
            ga.push_back(encoder_.encode_goal(w));
            stats.add("planner.wc2_goal_probes");
            // Deterministic resource bound (design §10): a bounded probe cannot
            // starve the stepping-stone dive.  An rlimit-exhausted probe falls
            // through to generation below (no plan proven, budget preserved).
            if (probe_rlimit_ > 0) {
                z3::params pr(ctx_);
                pr.set("rlimit", probe_rlimit_);
                o.solver->set(pr);
            }
            z3::check_result gr = o.solver->check(ga);
            if (probe_rlimit_ > 0) {
                z3::params pr(ctx_);
                pr.set("rlimit", 0u);  // unbounded again for generation checks
                o.solver->set(pr);
            }
            if (gr == z3::sat) {
                z3::model model = o.solver->get_model();
                Plan full = base_prefix;
                Plan window_plan = encoder_.extract_plan(model, w);
                for (const Action* a : window_plan.actions()) full.add_action(a);
                if (replay_validate(full)) {
                    found_plan_ = finish_with_plan_flat(full, t + w, "probe");
                    return TurnResult::PlanFound;
                }
                Logger::instance().error(
                    "WC2 goal-probe plan FAILED replay validation — ignoring "
                    "(this indicates a concatenation bug); continuing search.");
            } else if (gr == z3::unknown) {
                if (remaining_ms() == 0) return TurnResult::Timeout;
                stats.add("planner.wc2_probe_rlimit_hits");  // bounded out; dive continues
            }
        }
    }

    int produced_live = 0;
    const int C = std::max(1, config.planner.wc_candidates);
    int round = 0;
    while (round < C) {
        z3::expr steer = steering_disjunction(frontier, 0, w, *lm);
        if (!apply_solver_timeout(*o.solver)) return TurnResult::Timeout;
        z3::expr_vector assumptions(ctx_);
        assumptions.push_back(steer);
        z3::check_result r = o.solver->check(assumptions);

        if (r == z3::sat) {
            z3::model model = o.solver->get_model();
            std::vector<z3::expr> vals;
            StateValuation sp = extract_state(model, w, vals);
            o.solver->add(!state_equals(w, vals));       // block this state
            nodes_[node_id].served.push_back(vals);      // remember for re-expansion
            stats.add("planner.wc2_generated");
            ++round;

            size_t h = sp.hash();
            auto cit = closed_.find(h);
            bool seen = cit != closed_.end();
            int prior_depth = seen ? cit->second.min_depth : (t + w);
            if (seen && cit->second.dead) {
                stats.add("planner.wc2_vetoes");
                continue;
            }
            if (seen && cit->second.min_depth <= t + w) {
                stats.add("planner.wc2_duplicates");
                continue;
            }
            Plan child_prefix = base_prefix;
            Plan window_plan = encoder_.extract_plan(model, w);
            for (const Action* a : window_plan.actions()) child_prefix.add_action(a);
            int U0 = std::max(0, U - w);
            int cid = make_node(node_id, std::move(child_prefix), t + w, sp,
                                std::move(vals), model, w, U0);
            if (cid < 0) {                                // relaxed-dead
                Closed& ce = closed_[h];
                ce.dead = true;
                ce.min_depth = std::min(prior_depth, t + w);
                stats.add("planner.wc2_vetoes");
                continue;
            }
            // Record the (possibly shallower) depth for this state.
            Closed& ce = closed_[h];
            ce.min_depth = std::min(prior_depth, t + w);
            ce.dead = false;
            push_open(cid);
            ++produced_live;
        } else if (r == z3::unsat) {
            int new_w = std::min(2 * w, config.planner.max_steps - t);
            if (new_w > w) {
                extend_oracle(o, nodes_[node_id], w, new_w, /*with_guidance=*/true);
                w = new_w;
                nodes_[node_id].window = new_w;
                stats.add("planner.wc2_widenings");
                ++round;                                  // widening consumes a round
                continue;
            }
            // No widening room: the node is exhausted — drop it (do not reinsert).
            stats.add("planner.wc2_exhausted");
            return TurnResult::Progress;
        } else {
            if (remaining_ms() == 0) timed_out_ = true;
            return timed_out_ ? TurnResult::Timeout : TurnResult::Progress;
        }
    }

    // §6.4 reinsertion: live children -> D unchanged; otherwise demote (D+1).
    if (produced_live >= 1) {
        push_open(node_id);
    } else {
        nodes_[node_id].D += 1;
        stats.add("planner.wc2_demotions");
        push_open(node_id);
    }
    return TurnResult::Progress;
}

// ---------------------------------------------------------------------------
// Suffix sweep lane (§7.2) — goal direction from the best anchor as a hard
// initial state.  Replaces the deleted phase-hint machinery.
// ---------------------------------------------------------------------------

AnchorSearchPlanner::TurnResult AnchorSearchPlanner::suffix_sweep_turn() {
    auto& config = Config::instance();
    auto& stats = Stats::instance();
    if (!propagator_factory_) return TurnResult::Progress;

    // Target: the open node minimising t + d_ff that has not exhausted its
    // suffix search.
    int target = -1;
    long best = std::numeric_limits<long>::max();
    for (int id : open_h_) {
        if (suffix_done_.count(id)) continue;
        long v = static_cast<long>(nodes_[id].t) + nodes_[id].d_ff;
        if (v < best) { best = v; target = id; }
    }
    if (target < 0) return TurnResult::Progress;  // nothing to steer toward yet

    if (target != suffix_target_) {
        // Retarget: discard and rebuild the persistent cold solver from the
        // new incumbent's anchor state.
        suffix_target_ = target;
        stats.add("planner.wc2_suffix_retargets");
        const Node& N = nodes_[target];
        suffix_solver_ = std::make_unique<z3::solver>(ctx_);
        suffix_prop_ = propagator_factory_(*suffix_solver_);
        z3::params p(ctx_);
        p.set("random_seed", (unsigned)42);
        const char* logic =
            recommended_logic(arithmetic_profile_, problem_.all_integer());
        if (logic) p.set("logic", logic);
        suffix_solver_->set(p);
        suffix_solver_->add(state_equals(0, N.sigma_values));  // hard init override
        suffix_prop_->register_timestep_variables(0);
        suffix_filled_ = 0;
        suffix_goal_lits_.clear();
        suffix_h_ = std::max({N.d_ff, N.lm->depth_lower_bound, N.U + 1, 1});
    }

    int t_anchor = nodes_[suffix_target_].t;
    int cap = config.planner.max_steps - t_anchor;
    if (suffix_h_ > cap) {
        suffix_done_.insert(suffix_target_);
        suffix_target_ = -1;
        return TurnResult::Progress;
    }

    for (int t = suffix_filled_; t < suffix_h_; ++t) {
        BasePlanner::add_timestep_constraints(*suffix_solver_, encoder_,
                                              *suffix_prop_, t,
                                              /*prefix_monotone=*/true);
    }
    if (suffix_h_ > suffix_filled_) {
        const Node& N = nodes_[suffix_target_];
        assert_masks(*suffix_solver_, 0, suffix_filled_, suffix_h_, N.first_layers,
                     N.rpg_action_layers, N.rpg_saw_all_actions);
        std::vector<int> open(N.lm->landmarks.size());
        std::iota(open.begin(), open.end(), 0);
        assert_ordering_clauses(*suffix_solver_, 0, suffix_filled_, suffix_h_,
                                open, *N.lm);
        suffix_filled_ = suffix_h_;
    }

    auto git = suffix_goal_lits_.find(suffix_h_);
    z3::expr gl = (git != suffix_goal_lits_.end())
        ? git->second
        : [&] {
            z3::expr lit = ctx_.bool_const(
                ("wc2s_goal_at_" + std::to_string(suffix_h_)).c_str());
            suffix_solver_->add(z3::implies(lit, encoder_.encode_goal(suffix_h_)));
            suffix_goal_lits_.emplace(suffix_h_, lit);
            return lit;
        }();

    if (!apply_solver_timeout(*suffix_solver_)) { timed_out_ = true; return TurnResult::Timeout; }
    z3::expr_vector assumptions(ctx_);
    assumptions.push_back(gl);
    stats.add("planner.wc2_suffix_probes");
    z3::check_result r = suffix_solver_->check(assumptions);

    if (r == z3::sat) {
        z3::model model = suffix_solver_->get_model();
        Plan full = nodes_[suffix_target_].prefix;
        Plan suffix_plan = encoder_.extract_plan(model, suffix_h_);
        for (const Action* a : suffix_plan.actions()) full.add_action(a);
        if (replay_validate(full)) {
            found_plan_ = finish_with_plan_flat(full, t_anchor + suffix_h_, "suffix");
            return TurnResult::PlanFound;
        }
        Logger::instance().error(
            "WC2 suffix plan FAILED replay validation — ignoring; continuing.");
        return TurnResult::Progress;
    }
    if (r == z3::unknown) {
        if (remaining_ms() == 0) { timed_out_ = true; return TurnResult::Timeout; }
        return TurnResult::Progress;
    }
    // UNSAT: no completion of this prefix within suffix_h_ steps.  Node-local
    // mute (§7.3): raise U(target); children created after inherit U - w.
    nodes_[suffix_target_].U = std::max(nodes_[suffix_target_].U, suffix_h_);
    int next_h = schedule_.next(suffix_h_, cap);
    if (next_h <= suffix_h_) {
        suffix_done_.insert(suffix_target_);
        suffix_target_ = -1;
    } else {
        suffix_h_ = next_h;
    }
    return TurnResult::Progress;
}

// ---------------------------------------------------------------------------
// Sweep turn dispatcher (§5): alternate the root and suffix lanes by turns.
// ---------------------------------------------------------------------------

AnchorSearchPlanner::TurnResult AnchorSearchPlanner::sweep_turn() {
    bool pick_root = sweep_root_turn_;
    sweep_root_turn_ = !sweep_root_turn_;
    if (pick_root) {
        if (!root_exhausted_) return root_sweep_turn();
        return suffix_sweep_turn();
    }
    TurnResult r = suffix_sweep_turn();
    // If the suffix lane had nothing to do and the root lane is live, spend the
    // turn on the root lane instead of idling.
    if (r == TurnResult::Progress && suffix_target_ < 0 && !root_exhausted_) {
        return root_sweep_turn();
    }
    return r;
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------

Plan AnchorSearchPlanner::search() {
    auto& config = Config::instance();
    schedule_ = HorizonSchedule::from_string(config.planner.horizon_schedule);
    solution_found_ = false;
    timed_out_ = false;
    init_deadline();
    search_start_ = std::chrono::steady_clock::now();

    Logger::instance().info(
        "Starting anchor search (els-wc2): w0=" +
        std::to_string(std::max(1, config.planner.wc_window)) +
        ", C=" + std::to_string(config.planner.wc_candidates) +
        ", propagator: " + propagator_strategy_->get_name() +
        ", timeout: " + format_timeout_string() +
        std::string(novelty_queue_ ? " [+novelty-queue]" : "") +
        std::string(dyn_window_ ? " [+dyn-window]" : "") +
        (probe_rlimit_ ? " [probe-rlimit=" + std::to_string(probe_rlimit_) + "]" : "") +
        (greedy_ratio_ > 1 ? " [greedy-ratio=" + std::to_string(greedy_ratio_) + "]" : ""));

    if (std::getenv("RANTANPLAN_WC2_SELFTEST")) {
        selftest_init_override();
    }

    // Greedy layer needs extra solvers (the per-expansion oracle and the
    // suffix lane).  Without a propagator factory, only the root sweep runs.
    bool greedy_available = static_cast<bool>(propagator_factory_);
    if (greedy_available) {
        ensure_root_started();
        if (!apply_solver_timeout(solver_)) { timed_out_ = true; }
        else if (solver_.check() == z3::sat) {
            z3::model init_model = solver_.get_model();
            initialize_achievers();
            Node root;
            root.parent = -1;
            root.t = 0;
            root.D = 0;
            root.U = 0;
            StateValuation s0 = extract_state(init_model, 0, root.sigma_values);
            root.sigma_hash = s0.hash();
            if (!score_anchor(s0, root)) {
                Logger::instance().info(
                    "Goals relaxed-unreachable from the initial state — greedy "
                    "layer disabled.");
            } else {
                root.id = 0;
                root.h_ff = relaxed_plan_size(init_model, 0, root.first_layers,
                                              &root.d_ff);
                std::vector<ExprID> psi, sat;
                std::vector<std::pair<ExprID, double>> delta;
                extract_novelty_features(init_model, 0, psi, delta, sat);
                root.lm = recompute_landmarks(s0, sat);
                root.u = static_cast<int>(root.lm->landmarks.size());
                root.novel = true;              // root seeds the tables
                update_novelty_tables(root.h_ff, root.u, psi, delta);
                root.window = std::min(std::max(1, config.planner.max_steps),
                    informed_window(root.first_layers, *root.lm, root.d_ff));
                nodes_.push_back(std::move(root));
                closed_[nodes_[0].sigma_hash] = {0, false};
                push_open(0);
                greedy_alive_ = true;
            }
        }
    }
    if (greedy_only_) {
        sweeps_alive_ = false;
        Logger::instance().info("RANTANPLAN_WC2_GREEDY_ONLY: sweeps disabled.");
    }

    // Event-driven round-robin: one greedy turn, then one sweep turn.  No
    // clocks — a wall deadline only truncates.
    while (sweeps_alive_ || greedy_alive_) {
        if (remaining_ms() == 0) { timed_out_ = true; break; }

        // k greedy expansions per sweep check (count-based, no clock): the
        // bounded dive gets room to search between expensive root-sweep checks.
        for (int i = 0; i < greedy_ratio_ && greedy_alive_; ++i) {
            if (remaining_ms() == 0) { timed_out_ = true; break; }
            TurnResult r = greedy_turn();
            if (r == TurnResult::PlanFound) return found_plan_;
            if (r == TurnResult::Timeout) { timed_out_ = true; break; }
            if (r == TurnResult::Retire) {
                greedy_alive_ = false;
                Logger::instance().info("Greedy layer retired (queues exhausted).");
            }
        }
        if (timed_out_) break;

        if (sweeps_alive_) {
            TurnResult r = sweep_turn();
            if (r == TurnResult::PlanFound) return found_plan_;
            if (r == TurnResult::Timeout) { timed_out_ = true; break; }
            if (r == TurnResult::Retire) {
                // The root lane exhausted every horizon up to max_steps: a
                // completeness proof that no plan of length <= max_steps
                // exists.  Nothing the greedy or suffix lanes can do changes
                // that, so the whole search retires.
                sweeps_alive_ = false;
                greedy_alive_ = false;
            }
        }
    }

    if (timed_out_) {
        Logger::instance().info("No plan found (timeout).");
    } else {
        Logger::instance().info("\n*** NO PLAN FOUND within " +
            std::to_string(config.planner.max_steps) + " timesteps ***");
    }
    record_final_stats();
    collect_statistics();
    propagator_strategy_->cleanup();
    if (suffix_prop_) suffix_prop_->cleanup();
    return Plan();
}

} // namespace rantanplan
