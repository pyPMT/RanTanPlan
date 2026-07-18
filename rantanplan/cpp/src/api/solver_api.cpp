#include "solver_api.hpp"

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "config/config.hpp"
#include "config/strategy_registry.hpp"
#include "ir_io.hpp"
#include "strategy_factory.hpp"
#include "config/strategy_spec.hpp"
#include "planners/incremental_horizon_planner.hpp"
#include "planners/formula_exporter.hpp"
#include "util/memory_tracker.hpp"
#include "util/scoped_timer.hpp"
#include "util/stats.hpp"
#include "util/logger.hpp"
#include "planners/propagators/modules/logging_module.hpp"
#include "passes/pipeline.hpp"
#include "passes/action_pruning_pass.hpp"
#include "passes/arithmetic_profile_pass.hpp"
#include "passes/symmetry_analysis_pass.hpp"
#include "passes/int_parameter_instantiation_pass.hpp"
#include "passes/array_lowering_pass.hpp"
#include "passes/grounding_pass.hpp"
#include "passes/quantifier_expansion_pass.hpp"
#include "passes/cwa_initial_state_pass.hpp"
#include "passes/static_fluent_pass.hpp"
#include "passes/cnf_normalization_pass.hpp"
#include "passes/mutex_analysis_pass.hpp"
#include "passes/landmark_analysis_pass.hpp"
#include "analysis/interference_analysis.hpp"
#include "grounding/array_lowering.hpp"

#include "z3++.h"

namespace rantanplan::api {

void InterruptSlot::interrupt() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ctx_) {
        ctx_->interrupt();  // Z3_interrupt: safe from another thread
    }
}

ScopedContextPublisher::ScopedContextPublisher(InterruptSlot* slot, z3::context& ctx)
    : slot_(slot) {
    if (slot_) {
        std::lock_guard<std::mutex> lock(slot_->mutex_);
        slot_->ctx_ = &ctx;
    }
}

ScopedContextPublisher::~ScopedContextPublisher() {
    if (slot_) {
        std::lock_guard<std::mutex> lock(slot_->mutex_);
        slot_->ctx_ = nullptr;
    }
}

PreprocessedProblem preprocess(Problem problem, const PreprocessorOptions& options) {
    // A run starts here: drop statistics accumulated by any previous
    // in-process run so per-run stats don't leak across solves. (The CLI
    // driver runs once per process; in-process callers run many times.)
    rantanplan::Stats::instance().clear();

    // Array cells minted by ArrayLoweringPass have only been validated through
    // the native grounder's fixpoint; the UP-grounding path (--up-grounding)
    // has never been exercised with them, so reject the combination up front.
    if (!options.enable_grounding && rantanplan::has_array_fluents(problem)) {
        throw std::invalid_argument(
            "Array fluents require native grounding; --up-grounding is "
            "unsupported for array problems.");
    }

    // === PREPROCESSING PIPELINE ===
    //
    // Four phases, each with clear responsibilities and data dependencies:
    //
    //   CANONICALIZE → lifted problem → clean ground problem
    //   PRUNE       → remove unreachable/irrelevant actions via RPG + achievers
    //   DISCOVER    → find state invariants (mutex groups) and landmarks
    //   PROFILE     → symmetry data + numeric constraint classification
    //
    // Data flow between phases:
    //
    //   CANON    produces: grounded problem with complete initial state, simplified expressions
    //   PRUNE    produces: pruned problem, achievers, sdac_cost_lower_bounds, and
    //                      the bounds/conservations/domains part of state_constraints
    //                      (the fixpoint RPG stays local to the pass)
    //   DISCOVER appends mutex groups to state_constraints (MutexAnalysis) and
    //            extracts landmark_data from achievers (LandmarkAnalysis)
    //   PROFILE  produces: symmetry_data (using the pre-grounding snapshot for
    //                      detection), arithmetic_profile + all_integer flag
    //
    // Ordering constraints:
    //   - Grounding before CWA (CWA needs all grounded fluents visible)
    //   - CWA before StaticFluent (needs complete initial state to identify constants)
    //   - StaticFluent before CnfNormalization (folding first collapses disjunctions
    //     over static predicates; distributing before folding can blow up)
    //   - StaticFluent before ActionPruning (simpler expressions → tighter RPG bounds)
    //   - MutexAnalysis after ActionPruning (runs on the pruned action set)
    //   - LandmarkAnalysis after ActionPruning (consumes its achievers artifact;
    //     gated on goal_relevance because achievers only exist with it)
    //   - SymmetryAnalysis after all pruning (runs on smallest action set)
    //   - ArithmeticProfile last (reads final expressions, sets all_integer for
    //     every downstream Z3 translation)

    // Snapshot the original (pre-grounding) problem for symmetry detection.
    // Detection checks initial-state + goals equivalence under object swap;
    // this is cheaper on the lifted problem (smaller Z3 formula) and must
    // happen before StaticFluent removes distinguishing static assignments.
    std::optional<rantanplan::Problem> original_problem;
    if (options.enable_symmetries) {
        original_problem = problem;  // cheap copy (shared ExprPool)
    }

    rantanplan::IntParameterInstantiationPass int_parameter_instantiation_pass;
    rantanplan::QuantifierExpansionPass quantifier_expansion_pass;
    rantanplan::ArrayLoweringPass array_lowering_pass;
    rantanplan::GroundingPass grounding_pass;
    rantanplan::CWAInitialStatePass cwa_pass;
    rantanplan::StaticFluentPass static_fluent_pass;
    rantanplan::CnfNormalizationPass cnf_normalization_pass;
    rantanplan::ActionPruningPass action_pruning_pass(options);
    rantanplan::MutexAnalysisPass mutex_analysis_pass;
    rantanplan::LandmarkAnalysisPass landmark_analysis_pass;
    rantanplan::SymmetryAnalysisPass symmetry_analysis_pass(
        original_problem ? &*original_problem : nullptr);
    rantanplan::ArithmeticProfilePass arithmetic_profile_pass;

    std::vector<const rantanplan::Pass*> passes;

    // --- CANONICALIZE ---
    // Compile integer parameters away (syntactic), then remove all quantifiers
    // (conditions + forall-effects) — both before grounding. By now every range
    // bound is a constant and no parameter is numeric, so grounding is ordinary
    // object grounding over quantifier-free actions.
    passes.push_back(&int_parameter_instantiation_pass);
    passes.push_back(&quantifier_expansion_pass);
    // Array lowering must follow quantifier expansion (indices become ground
    // constant arithmetic) and precede grounding (cells must exist before the
    // reachability fixpoint enumerates facts). No-op without array fluents.
    passes.push_back(&array_lowering_pass);
    if (options.enable_grounding) {
        passes.push_back(&grounding_pass);
    }
    passes.push_back(&cwa_pass);
    passes.push_back(&static_fluent_pass);
    if (options.enable_cnf_normalization) {
        passes.push_back(&cnf_normalization_pass);
    }

    // --- PRUNE ---
    passes.push_back(&action_pruning_pass);

    // --- DISCOVER ---
    passes.push_back(&mutex_analysis_pass);
    // Landmark extraction runs over the achievers artifact, which
    // ActionPruningPass only builds when goal-relevance analysis is on.
    if (options.goal_relevance) {
        passes.push_back(&landmark_analysis_pass);
    }

    // --- PROFILE ---
    if (options.enable_symmetries) {
        passes.push_back(&symmetry_analysis_pass);
    }
    passes.push_back(&arithmetic_profile_pass);

    rantanplan::PreprocessedProblem initial_result;
    initial_result.problem = std::move(problem);

    auto pipeline_result = rantanplan::run_pipeline(std::move(initial_result), passes);

    // Record final action count after all pipeline passes (used for grounding analysis).
    rantanplan::Stats::instance().set("pipeline.final_actions",
                                      static_cast<double>(pipeline_result.problem.actions().size()));

    // Debug: dump problem info after pipeline
    if (rantanplan::Logger::instance().should_print(rantanplan::VerbosityLevel::DEBUG)) {
        const auto& prob = pipeline_result.problem;
        rantanplan::Logger::instance().info("After pipeline: " +
                    std::to_string(prob.actions().size()) + " actions, " +
                    std::to_string(prob.grounded_fluents().size()) + " grounded fluents, " +
                    std::to_string(prob.initial_state().size()) + " initial assignments, " +
                    std::to_string(prob.goals().size()) + " goals");
    }

    return pipeline_result;
}

ir::ResultIR solve(PreprocessedProblem& pipeline_result, const PlanCallback& on_plan,
                   InterruptSlot* interrupt_slot) {
    auto& config = rantanplan::Config::instance();
    const auto& problem = pipeline_result.problem;

    // Resolve the strategy spec: registry lookup, planner kind for the search
    // mode, problem-specific adjustments (e.g. SDAC + exists-step downgrade),
    // then validation of the adjusted spec.
    rantanplan::StrategySpec spec = rantanplan::StrategyRegistry::get(config.planner.strategy);
    rantanplan::SearchMode mode = rantanplan::parse_search_mode(config.planner.mode);
    spec.planner = rantanplan::resolve_planner_kind(mode, spec);
    rantanplan::StrategyFactory::adjust_spec(spec, problem);
    rantanplan::StrategyFactory::validate(spec, config.planner.strategy,
                                          config.planner.horizon_schedule);

    // Declared state invariants are trusted plan requirements asserted at
    // every state.  The R2E encoder deliberately skips state constraints
    // (chain-substituted intermediate states), so accepting the problem
    // would silently drop a requirement — reject instead.
    if (!problem.state_invariants().empty() &&
        spec.encoder == rantanplan::EncoderFamily::R2E) {
        throw std::invalid_argument(
            "Strategy '" + config.planner.strategy + "' (R2E encoder) does not "
            "support declared state invariants ((:constraints (always ...))). "
            "Use a grounded/chained strategy instead.");
    }

    ir::ResultIR result;
    auto add_info = [&result](const std::string& message) {
        result.log_messages.push_back(ir::LogMessageIR{1 /* INFO */, message});
    };

    z3::context ctx;
    // Declared after ctx: unpublishes (and waits for any in-flight
    // interrupt()) before ctx is destroyed.
    ScopedContextPublisher publish(interrupt_slot, ctx);

    rantanplan::Logger::instance().info("Using strategy: " + config.planner.strategy + " (mode: " + config.planner.mode + ")");

    // The factory returns fully wired components: interference is
    // constructor-injected into the parallelism strategy, which is
    // constructor-injected into the encoder; planners consume their
    // artifacts via BasePlanner::consume_artifacts.
    auto encoder = rantanplan::StrategyFactory::create_encoder(spec, pipeline_result, ctx);
    auto planner = rantanplan::StrategyFactory::create_planner(spec, pipeline_result, *encoder, ctx);

    // Get solver reference and create propagator using factory
    // (must come after the planner: the propagator attaches to its solver)
    auto propagator = rantanplan::StrategyFactory::create_propagator(spec, planner->get_solver(), problem, *encoder);

    // Add logging module if requested
    if (config.has_log_file()) {
        propagator->add_module(std::make_unique<rantanplan::LoggingModule>(
            config.logging.log_file, config.planner.strategy));
    }

    planner->set_propagator_strategy(std::move(propagator));

    if (on_plan) {
        planner->set_intermediate_plan_callback(on_plan);
    }

    // Call planner->search() and get the result
    rantanplan::Plan plan = planner->search();
    if (planner->solution_found()) {
        result.status = static_cast<int>(planner->optimality_proven()
                                             ? ir::SolveStatusIR::SOLVED_OPTIMALLY
                                             : ir::SolveStatusIR::SOLVED_SATISFICING);
        result.plan = rantanplan::plan_to_ir(plan);
        result.has_plan = true;

        if (config.is_info()) {
            std::string msg = "Plan found with " + std::to_string(plan.length()) + " actions.";
            if (rantanplan::uses_branch_and_bound(spec)) {
                msg += " Cost: " + std::to_string(planner->best_cost()) + ".";
                if (planner->optimality_proven()) {
                    msg += " (OPTIMAL)";
                }
            }
            add_info(msg);
        }
    } else if (planner->timed_out()) {
        result.status = static_cast<int>(ir::SolveStatusIR::TIMEOUT);
        if (config.is_info()) {
            add_info("Timeout reached.");
        }
    } else {
        // No plan found within search limits (max_steps exhausted)
        result.status = static_cast<int>(ir::SolveStatusIR::UNSOLVABLE_INCOMPLETELY);
        if (config.is_info()) {
            add_info("No plan found within the search limit.");
        }
    }

    if (config.is_verbose()) {
        add_info("Finished solving the problem.");
    }

    // Add memory tracking information to result
    double current_memory = rantanplan::MemoryTracker::instance().get_current_memory_mb();
    add_info("Memory usage - Current: " + std::to_string(current_memory) + " MB");

    return result;
}

std::string export_formula(const rantanplan::Problem& problem) {
    auto& config = rantanplan::Config::instance();

    // Validate formula export parameters
    if (config.formula_export.timestep < 0) {
        throw std::invalid_argument("--formula-timestep must be specified with --export-formula");
    }

    // Only allow formula export with strategies that support it
    const auto& spec = rantanplan::StrategyRegistry::get(config.planner.strategy);
    if (!rantanplan::supports_formula_export(spec)) {
        throw std::invalid_argument(
            "Formula export is not supported by strategy '" + config.planner.strategy + "'");
    }

    // Create Z3 context
    z3::context ctx;

    // Create the fully wired encoder (interference -> parallelism -> encoder)
    auto encoder = rantanplan::StrategyFactory::create_encoder(spec, problem, ctx);

    rantanplan::Logger::instance().info("Exporting formula for timestep " + std::to_string(config.formula_export.timestep)
                                   + " using strategy: " + encoder->get_parallelism_strategy_name());

    // Create formula exporter and generate formula
    rantanplan::FormulaExporter exporter(problem, *encoder, ctx);
    return exporter.export_formula(config.formula_export.timestep);
}

} // namespace rantanplan::api
