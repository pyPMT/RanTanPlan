#include "cgb_planner.hpp"
#include "../preprocessing/preprocessed_problem.hpp"
#include "../encoders/grounded_encoder.hpp"
#include "../config/config.hpp"
#include "../util/logger.hpp"
#include "../util/stats.hpp"
#include "../util/memory_tracker.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace rantanplan {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

CGBPlanner::CGBPlanner(const Problem& problem, BaseEncoder& encoder, cvc5::TermManager& tm)
    : BasePlanner(problem, encoder, tm),
      goal_assumption_(tm.mkTrue()) {
    // The core loop reads the failed assumptions after every UNSAT, and cvc5
    // only tracks them when this option is set — which it only accepts before
    // the first assertion, so it has to happen here in the constructor.
    solver_.setOption("produce-unsat-assumptions", "true");
}

GroundedEncoder& CGBPlanner::grounded_encoder() {
    if (!grounded_encoder_) {
        grounded_encoder_ = &dynamic_cast<GroundedEncoder&>(encoder_);
    }
    return *grounded_encoder_;
}

void CGBPlanner::consume_artifacts(PreprocessedProblem& artifacts) {
    if (artifacts.achievers) {
        achievers_ = std::move(artifacts.achievers);
    }
    landmark_data_ = artifacts.landmark_data;
}

// ---------------------------------------------------------------------------
// Achiever setup (relaxed-plan seeding)
// ---------------------------------------------------------------------------

void CGBPlanner::initialize_achievers() {
    for (const Action& a : problem_.actions()) {
        action_id_to_ptr_[a.id()] = &a;
    }

    if (!achievers_) {
        achievers_ = std::make_unique<AchieversAnalysis>(problem_);
    }

    for (ExprID cond : achievers_->get_all_conditions()) {
        const auto& achiever_actions = achievers_->get_achievers(cond);
        auto& ptrs = condition_achievers_[cond];
        ptrs.reserve(achiever_actions.size());
        for (const Action& a : achiever_actions) {
            auto it = action_id_to_ptr_.find(a.id());
            if (it != action_id_to_ptr_.end()) {
                ptrs.push_back(it->second);
            }
        }
    }

    // Exact init-state evaluation, computed once inside the achievers
    // analysis and shared with the landmark pass.
    init_satisfied_conditions_ = achievers_->get_init_satisfied_conditions();

    for (ExprID cond : achievers_->get_goal_conditions()) {
        goal_condition_ids_.push_back(cond);
    }

    for (const Action& a : problem_.actions()) {
        const auto& precs = achievers_->get_preconditions(a);
        if (!precs.empty()) {
            auto* ptr = action_id_to_ptr_[a.id()];
            action_precondition_ids_[ptr].assign(precs.begin(), precs.end());
        }
    }
}

std::unordered_set<const Action*> CGBPlanner::extract_relaxed_plan() {
    std::unordered_set<const Action*> relaxed_plan;
    std::unordered_set<ExprID> unsatisfied;
    for (ExprID g : goal_condition_ids_) {
        if (!init_satisfied_conditions_.count(g)) {
            unsatisfied.insert(g);
        }
    }
    int max_layer = achievers_->get_arpg_num_layers();
    for (int layer = max_layer - 1; layer >= 0 && !unsatisfied.empty(); layer--) {
        std::vector<ExprID> newly_satisfied;
        for (ExprID cond : unsatisfied) {
            auto it = condition_achievers_.find(cond);
            if (it == condition_achievers_.end()) continue;
            const Action* best = nullptr;
            int best_layer = -1;
            for (const Action* a : it->second) {
                int a_layer = achievers_->get_action_first_layer(a->id());
                if (a_layer <= layer && a_layer > best_layer) {
                    best = a;
                    best_layer = a_layer;
                }
            }
            if (best && !relaxed_plan.count(best)) {
                relaxed_plan.insert(best);
                newly_satisfied.push_back(cond);
                auto prec_it = action_precondition_ids_.find(best);
                if (prec_it != action_precondition_ids_.end()) {
                    for (ExprID prec : prec_it->second) {
                        if (!init_satisfied_conditions_.count(prec)) {
                            unsatisfied.insert(prec);
                        }
                    }
                }
            }
        }
        for (ExprID cond : newly_satisfied) {
            unsatisfied.erase(cond);
        }
    }
    return relaxed_plan;
}

void CGBPlanner::seed_activations() {
    std::unordered_set<const Action*> relaxed_plan = extract_relaxed_plan();
    for (const Action* a : relaxed_plan) {
        activate_action(a);
    }
    Logger::instance().info("CGB seed: activated " +
        std::to_string(relaxed_plan.size()) + "/" +
        std::to_string(problem_.actions().size()) +
        " relaxed-plan actions");
}

// ---------------------------------------------------------------------------
// Landmarks
// ---------------------------------------------------------------------------

void CGBPlanner::adopt_landmarks() {
    // Landmarks come from the LandmarkAnalysisPass artifact — the closure
    // once lived here too (predating the pass) and the duplicate was
    // removed.  The planner only re-applies the clause width cap: a
    // landmark with hundreds of achievers propagates only once all but one
    // is excluded — not worth a clause.  The artifact keeps every landmark
    // regardless, so deficit budgets and horizon jumps still see the wide
    // ones.
    const size_t max_achievers = 32;
    int skipped_wide = 0;
    for (const LandmarkInfo& info : landmark_data_.landmarks) {
        if (info.achiever_action_ids.empty()) continue;
        if (info.achiever_action_ids.size() > max_achievers) {
            skipped_wide++;
            continue;
        }
        landmarks_.push_back(info.condition);
    }
    Logger::instance().info("CGB landmarks: adopted " +
        std::to_string(landmarks_.size()) + "/" +
        std::to_string(landmark_data_.landmarks.size()) +
        " from pass (" + std::to_string(skipped_wide) + " too wide for clauses)");
}

// ---------------------------------------------------------------------------
// Encoding
// ---------------------------------------------------------------------------

void CGBPlanner::encode_action_at(const Action* action, int timestep) {
    auto& encoded = action_encoded_at_[action];
    if (encoded.count(timestep)) return;
    encoded.insert(timestep);

    auto effects = grounded_encoder().encode_non_cumulative_effects(*action, timestep);
    if (effects) solver_.assertFormula(*effects);

    if (action->has_precondition()) {
        auto& vf = encoder_.get_variable_factory();
        const Term& action_var = vf.get_action_variable(*action, timestep);
        Term prec_z3 = encoder_.convert_expr_id_to_z3(action->precondition_id(), timestep);
        solver_.assertFormula(tb_.mk_implies(action_var, prec_z3));
    }
}

void CGBPlanner::add_timestep(int t) {
    grounded_encoder().ensure_action_variables(t);
    solver_.assertFormula(encoder_.encode_frames(t));
    solver_.assertFormula(encoder_.encode_symmetries(t));
    solver_.assertFormula(encoder_.encode_state_constraints(t));
    solver_.assertFormula(grounded_encoder().encode_cumulative_effects(t));

    auto& vf = encoder_.get_variable_factory();
    for (const Action* a : blocked_) {
        const Term& action_var = vf.get_action_variable(*a, t);
        solver_.assertFormula(tb_.mk_implies(block_lit_.at(a), tb_.mk_not(action_var)));
    }
    for (const Action* a : activated_) {
        encode_action_at(a, t);
    }

}

void CGBPlanner::refresh_goal(int t) {
    int version = next_goal_version_++;
    goal_assumption_ = tm_.mkConst(tm_.getBooleanSort(), "goal_v" + std::to_string(version));
    solver_.assertFormula(tb_.mk_implies(goal_assumption_, encoder_.encode_goal(t)));

    // Landmark clauses over presence literals (the paper's form): some
    // achiever of every landmark must be available, i.e. not blocked.
    // Tiny clauses that conflict with the blocking assumptions at the root
    // when all achievers are dormant, putting them straight into the core.
    // Clauses with an already-activated achiever are trivially satisfied
    // and skipped.  (Occurrence-level variants — wide a_t disjunctions or
    // latch chains — measurably disturbed search on satisfiable queries.)
    for (ExprID cond : landmarks_) {
        auto ach_it = condition_achievers_.find(cond);
        if (ach_it == condition_achievers_.end()) continue;
        const auto& achievers = ach_it->second;
        bool has_active = false;
        for (const Action* a : achievers) {
            if (!blocked_.count(a)) { has_active = true; break; }
        }
        if (has_active) continue;
        Term available = tb_.bool_val(false);
        for (const Action* a : achievers) {
            available = tb_.mk_or({available, tb_.mk_not(block_lit_.at(a))});
        }
        solver_.assertFormula(tb_.mk_implies(goal_assumption_, available));
    }

    assert_deficit_budgets(t);
}

namespace {
Term real_const(cvc5::TermManager& tm, double v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.12g", v);
    return tm.mkReal(std::string(buf));
}
} // namespace

/// Count-landmark deficit budgets over presence literals: within t steps an
/// unblocked contributor adds at most lambda*t toward the residual, so
/// goal -> sum of unblocked mass >= residual.  Blocked contributors enter
/// as ite(blk, 0, cap) — decided at the root by the assumptions, so a
/// deficit conflicts before search and its core names a SET of contributors
/// to activate; already-activated contributors fold into a constant (their
/// blk literal is free and counting it fully keeps the bound sound: caps
/// are maxima either way).  A deficit no activation can cover yields a
/// blk-free conflict, i.e. the horizon jump of count_bound_target.
void CGBPlanner::assert_deficit_budgets(int t) {
    int asserted = 0;
    for (const LandmarkInfo& info : landmark_data_.landmarks) {
        // Default landmarks (residual 1, unit contributions) carry no
        // quantitative information beyond their presence clause — skip.
        // Real count landmarks are emitted even when logically
        // presence-equivalent: the arithmetic form measurably drives
        // broader cores (settlers unseeded 26s vs 87s with these skipped).
        bool default_like = info.residual == 1.0 &&
            std::all_of(info.contributions.begin(), info.contributions.end(),
                        [](double l) { return l == 1.0; });
        if (default_like) continue;

        double active_mass = 0.0;
        std::vector<std::pair<const Action*, double>> dormant;
        for (size_t i = 0; i < info.achiever_action_ids.size(); i++) {
            auto it = action_id_to_ptr_.find(info.achiever_action_ids[i]);
            if (it == action_id_to_ptr_.end()) continue;
            double cap = info.contributions[i] * t;
            if (blocked_.count(it->second)) {
                dormant.push_back({it->second, cap});
            } else {
                active_mass += cap;
            }
        }
        if (active_mass >= info.residual) continue;  // already slack

        Term mass = real_const(tm_, active_mass);
        for (const auto& [action, cap] : dormant) {
            mass = tb_.add(mass, tb_.mk_ite(block_lit_.at(action),
                                            real_const(tm_, 0.0), real_const(tm_, cap)));
        }
        solver_.assertFormula(tb_.mk_implies(
            goal_assumption_, tb_.geq(mass, real_const(tm_, info.residual))));
        asserted++;
    }
    if (asserted > 0) {
        Stats::instance().add("planner.deficit_constraints",
                              static_cast<double>(asserted));
    }
}

/// Smallest horizon the count landmarks do not refute under the current
/// activations.  Landmarks with no active contributor give no bound (their
/// presence clauses and cores drive activation instead); default landmarks
/// (residual 1) bound at most 1 and never bind here.
int CGBPlanner::count_bound_target() const {
    int target = current_horizon_ + 1;
    for (const LandmarkInfo& info : landmark_data_.landmarks) {
        double active_step_contribution = 0.0;
        for (size_t i = 0; i < info.achiever_action_ids.size(); i++) {
            auto it = action_id_to_ptr_.find(info.achiever_action_ids[i]);
            if (it != action_id_to_ptr_.end() && activated_.count(it->second)) {
                active_step_contribution += info.contributions[i];
            }
        }
        if (active_step_contribution <= 0) continue;
        int bound = static_cast<int>(
            std::ceil(info.residual / active_step_contribution - 1e-9));
        if (bound > target) {
            target = bound;
        }
    }
    return target;
}

// ---------------------------------------------------------------------------
// Search helpers
// ---------------------------------------------------------------------------

void CGBPlanner::activate_action(const Action* action) {
    if (!blocked_.count(action)) return;
    blocked_.erase(action);
    activated_.insert(action);
    blk_id_to_action_.erase(block_lit_.at(action).getId());
    for (int t = 0; t <= current_horizon_; t++) {
        encode_action_at(action, t);
    }
}

TermVec CGBPlanner::build_assumptions() {
    TermVec assumptions;
    for (const Action* a : blocked_) {
        assumptions.push_back(block_lit_.at(a));
    }
    assumptions.push_back(goal_assumption_);
    return assumptions;
}

// PORT-QUESTION: cvc5 has no model object — values are read with
// solver.getValue(). This assumes BaseEncoder::extract_plan is re-typed to
// take the Solver in place of the z3::model (encoders layer, ported
// separately).
Plan CGBPlanner::extract_plan(const cvc5::Solver& solver) {
    return encoder_.extract_plan(solver, current_horizon_ + 1);
}

// ---------------------------------------------------------------------------
// Main search
// ---------------------------------------------------------------------------

Plan CGBPlanner::search() {
    auto& config = Config::instance();
    auto& stats = Stats::instance();

    solution_found_ = false;
    timed_out_ = false;
    init_deadline();

    Logger::instance().info("Starting CGB search, timeout: " + format_timeout_string());

    // 1. Existence guards: every ground action starts dormant
    for (const Action& a : problem_.actions()) {
        Term blk = tm_.mkConst(tm_.getBooleanSort(), "blk_a" + std::to_string(a.id()));
        const Action* ptr = &a;
        block_lit_.insert({ptr, blk});
        blocked_.insert(ptr);
        blk_id_to_action_[blk.getId()] = ptr;
    }

    // 2. Encode initial state
    solver_.assertFormula(encoder_.encode_initial_state());

    // 3. Relaxed-plan seeding, landmarks, pre-extension to the RPG lower bound
    initialize_achievers();
    seed_activations();
    adopt_landmarks();

    {
        int start_ts = config.planner.start_timestep;
        int num_timesteps = std::max(1, start_ts);
        for (int t = 0; t < num_timesteps; t++) {
            current_horizon_++;
            add_timestep(current_horizon_);
        }
        refresh_goal(current_horizon_ + 1);
        stats.set("planner.rpg_lower_bound", static_cast<double>(start_ts));
        Logger::instance().info("Pre-extended horizon to " +
            std::to_string(current_horizon_) +
            " (RPG lower bound: " + std::to_string(start_ts) + ")");
    }

    // 4. Main loop: solve under blocking assumptions, grow what the core names
    int round = 0;
    int activations = static_cast<int>(activated_.size());
    int horizon_extensions = 0;
    double total_time = 0.0;
    auto start_time = std::chrono::high_resolution_clock::now();

    while (true) {
        if (!apply_solver_timeout(solver_)) break;

        TermVec assumptions = build_assumptions();

        auto solve_start = std::chrono::high_resolution_clock::now();
        cvc5::Result result = solver_.checkSatAssuming(assumptions);
        auto solve_end = std::chrono::high_resolution_clock::now();
        double solve_time = std::chrono::duration<double>(solve_end - solve_start).count();
        total_time = std::chrono::duration<double>(solve_end - start_time).count();

        stats.add("planner.solve_time", solve_time);
        stats.add("planner.total_time", solve_time);

        if (config.global.verbosity >= VerbosityLevel::INFO) {
            double mem = (round % 50 == 0)
                ? MemoryTracker::instance().get_current_memory_mb() : -1.0;
            std::vector<std::pair<std::string, std::string>> fields = {
                {"solve", std::to_string(solve_time) + "s"},
                {"horizon", std::to_string(current_horizon_)},
                {"activated", std::to_string(activated_.size()) + "/" +
                              std::to_string(problem_.actions().size())},
            };
            if (mem >= 0) {
                fields.push_back({"mem", std::to_string(static_cast<int>(mem)) + "MB"});
            }
            Logger::instance().timestep_solving(VerbosityLevel::INFO, round, fields, "R");
        }

        if (result.isSat()) {
            solution_found_ = true;
            Plan plan = extract_plan(solver_);

            Logger::instance().info("\n*** PLAN FOUND: " +
                std::to_string(plan.length()) + " actions, " +
                std::to_string(round) + " rounds, " +
                "horizon=" + std::to_string(current_horizon_) +
                ", activated=" + std::to_string(activated_.size()) +
                "/" + std::to_string(problem_.actions().size()) +
                ", extensions=" + std::to_string(horizon_extensions) +
                " (total time: " + std::to_string(total_time) + "s) ***");

            stats.set("planner.plan_length", static_cast<double>(plan.length()));
            stats.set("planner.rounds", static_cast<double>(round));
            stats.set("planner.solution_horizon", static_cast<double>(current_horizon_));
            stats.set("planner.horizon_extensions", static_cast<double>(horizon_extensions));
            stats.set("planner.activated_actions", static_cast<double>(activated_.size()));
            stats.set("planner.total_actions", static_cast<double>(problem_.actions().size()));

            plan.write_ipc(config.planner.output_plan, 1,
                           -1.0, true,
                           config.planner.strategy, config.planner.mode, total_time);
            notify_intermediate_plan(plan);

            collect_statistics();
            return plan;

        } else if (result.isUnsat()) {
            // The failed assumptions are cvc5's counterpart of Z3's unsat core
            // over assumption literals (enabled by produce-unsat-assumptions in
            // the constructor).
            TermVec core = solver_.getUnsatAssumptions();
            stats.add("planner.total_core_size", static_cast<double>(core.size()));
            stats.add("planner.core_count");

            std::vector<const Action*> named;
            for (size_t i = 0; i < core.size(); ++i) {
                auto it = blk_id_to_action_.find(core[i].getId());
                if (it != blk_id_to_action_.end()) {
                    named.push_back(it->second);
                }
            }

            Logger::instance().info("  core: " + std::to_string(core.size()) +
                " assumptions (" + std::to_string(named.size()) + " blocking)");

            if (!named.empty()) {
                // The core names the missing actions: activate exactly those.
                for (const Action* a : named) {
                    activate_action(a);
                    if (config.is_verbose()) {
                        Logger::instance().info("  Core->activate: " + a->label());
                    }
                }
                activations += static_cast<int>(named.size());
            } else {
                // Core free of blocking literals: the horizon binds.  Jump
                // to the next horizon the count landmarks do not refute.
                if (current_horizon_ >= config.planner.max_steps) {
                    Logger::instance().info("Horizon limit reached (" +
                        std::to_string(current_horizon_) + ")");
                    break;
                }
                int target = std::min(count_bound_target(), config.planner.max_steps);
                horizon_extensions++;
                Logger::instance().info("Extending horizon (#" +
                    std::to_string(horizon_extensions) +
                    ", horizon=" + std::to_string(current_horizon_) +
                    (target > current_horizon_ + 1
                        ? " -> " + std::to_string(target) + " [count-landmark jump]"
                        : "") + ")");
                while (current_horizon_ < target) {
                    current_horizon_++;
                    add_timestep(current_horizon_);
                }
                refresh_goal(current_horizon_ + 1);
            }

        } else {
            if (handle_unknown_result(result, "round " + std::to_string(round))) break;
        }

        round++;
    }

    if (timed_out_) {
        Logger::instance().info("No plan found (timeout).");
    } else {
        Logger::instance().info("\n*** NO PLAN FOUND within horizon limit ***");
    }

    collect_statistics();
    return Plan();
}

} // namespace rantanplan
