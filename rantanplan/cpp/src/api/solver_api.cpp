#include "solver_api.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "config/config.hpp"
#include "config/strategy_registry.hpp"
#include "problem/protobuf_io.hpp"
#include "config/strategy_factory.hpp"
#include "config/strategy_spec.hpp"
#include "planners/sequential.hpp"
#include "planners/pdla_planner.hpp"
#include "planners/formula_exporter.hpp"
#include "util/memory_tracker.hpp"
#include "util/stats.hpp"
#include "util/logger.hpp"
#include "planners/propagators/modules/logging_module.hpp"
#include "passes/pipeline.hpp"
#include "passes/action_pruning_pass.hpp"
#include "passes/symmetry_analysis_pass.hpp"
#include "passes/grounding_pass.hpp"
#include "passes/cwa_initial_state_pass.hpp"
#include "passes/strategy_resolution_pass.hpp"
#include "passes/interference_pass.hpp"
#include "passes/static_fluent_pass.hpp"
#include "passes/invariant_oracle_pass.hpp"
#include "analysis/numeric_relaxed_planning_graph.hpp"
#include "analysis/numeric_constraint_analyzer.hpp"

#include "z3++.h"

namespace rantanplan::api {

PipelineResult preprocess(Problem problem) {
    auto& config = rantanplan::Config::instance();

    // === PREPROCESSING PIPELINE ===
    //
    // Five phases, each with clear responsibilities and data dependencies:
    //
    //   SETUP       → resolve strategy config (needed by all downstream passes)
    //   CANONICALIZE → lifted problem → clean ground problem
    //   PRUNE       → remove unreachable/irrelevant actions via RPG + achievers
    //   DISCOVER    → find state invariants (mutex groups, bounds, conservation laws)
    //   PREPARE     → build analysis objects the solver consumes directly
    //
    // Data flow between phases:
    //
    //   SETUP    produces: resolved_spec
    //   CANON    produces: grounded problem with complete initial state, simplified expressions
    //   PRUNE    produces: pruned problem, fixpoint_rpg, tightened_bounds,
    //                      conservation_laws, achievers, sdac_cost_lower_bounds
    //   DISCOVER consumes tightened_bounds + conservation_laws from PRUNE (no re-derivation),
    //            reads object domains from fixpoint_rpg, produces state_constraints
    //   PREPARE  produces: symmetry_data (using original_problem for detection),
    //                      interference analyzer
    //
    // Ordering constraints:
    //   - Grounding before CWA (CWA needs all grounded fluents visible)
    //   - CWA before StaticFluent (needs complete initial state to identify constants)
    //   - StaticFluent before ActionPruning (simpler expressions → tighter RPG bounds)
    //   - ActionPruning before InvariantOracle (produces bounds + conservation laws it consumes)
    //   - SymmetryAnalysis after all pruning (runs on smallest action set)
    //   - Interference last (needs final problem + resolved spec)

    rantanplan::StrategyResolutionPass strategy_resolution_pass;
    rantanplan::GroundingPass grounding_pass;
    rantanplan::CWAInitialStatePass cwa_pass;
    rantanplan::StaticFluentPass static_fluent_pass;
    rantanplan::ActionPruningPass action_pruning_pass;
    rantanplan::InvariantOraclePass invariant_oracle_pass;
    rantanplan::SymmetryAnalysisPass symmetry_analysis_pass;
    rantanplan::InterferencePass interference_pass;

    std::vector<const rantanplan::Pass*> passes;

    // --- SETUP ---
    passes.push_back(&strategy_resolution_pass);

    // --- CANONICALIZE ---
    if (config.global.reachability_grounding) {
        passes.push_back(&grounding_pass);
    }
    passes.push_back(&cwa_pass);
    passes.push_back(&static_fluent_pass);

    // --- PRUNE ---
    passes.push_back(&action_pruning_pass);

    // --- DISCOVER ---
    passes.push_back(&invariant_oracle_pass);

    // --- PREPARE ---
    if (config.symmetry.enable_symmetries) {
        passes.push_back(&symmetry_analysis_pass);
    }
    passes.push_back(&interference_pass);

    // Snapshot the original (pre-grounding) problem for symmetry detection.
    // Detection checks initial-state + goals equivalence under object swap;
    // this is cheaper on the lifted problem (smaller Z3 formula) and must
    // happen before StaticFluent removes distinguishing static assignments.
    rantanplan::PipelineResult initial_result;
    if (config.symmetry.enable_symmetries) {
        initial_result.original_problem = problem;  // cheap copy (shared ExprPool)
    }
    initial_result.problem = std::move(problem);

    auto pipeline_result = rantanplan::run_pipeline(std::move(initial_result), passes);
    config.planner.start_timestep = pipeline_result.lower_bound;

    // Analyze numeric constraint structure for Z3 solver tuning
    auto constraint_analysis = rantanplan::NumericConstraintAnalyzer::analyze(pipeline_result.problem);
    pipeline_result.arithmetic_profile = constraint_analysis.profile;
    pipeline_result.problem.set_all_integer(constraint_analysis.all_integer);

    const char* logic_hint = rantanplan::recommended_logic(constraint_analysis.profile, constraint_analysis.all_integer);
    rantanplan::Logger::instance().component(rantanplan::VerbosityLevel::INFO, "ArithProfile", {
        {"class", rantanplan::arithmetic_profile_to_string(constraint_analysis.profile)},
        {"numeric fluents", std::to_string(constraint_analysis.num_numeric_fluents)},
        {"integer", constraint_analysis.num_numeric_fluents > 0
            ? (constraint_analysis.all_integer ? "yes" : "no") : "n/a"},
        {"logic", logic_hint ? logic_hint : "default"}
    });

    // Record final action count after all pipeline passes (used for grounding analysis).
    rantanplan::Stats::instance().set("pipeline.final_actions",
                                      static_cast<double>(pipeline_result.problem.actions().size()));

    // Debug: dump problem info after pipeline
    if (config.is_debug()) {
        const auto& prob = pipeline_result.problem;
        rantanplan::Logger::instance().info("After pipeline: " +
                    std::to_string(prob.actions().size()) + " actions, " +
                    std::to_string(prob.grounded_fluents().size()) + " grounded fluents, " +
                    std::to_string(prob.initial_state().size()) + " initial assignments, " +
                    std::to_string(prob.goals().size()) + " goals");
    }

    return pipeline_result;
}

PlanGenerationResult solve(PipelineResult& pipeline_result) {
    auto& config = rantanplan::Config::instance();
    const auto& problem = pipeline_result.problem;
    const auto& spec = pipeline_result.resolved_spec;

    PlanGenerationResult result;
    auto* log_message = result.add_log_messages();
    log_message->set_level(LogMessage_LogLevel_INFO);

    z3::context ctx;

    rantanplan::Logger::instance().info("Using strategy: " + config.planner.strategy + " (mode: " + config.planner.mode + ")");

    auto encoder = rantanplan::StrategyFactory::create_encoder(spec, problem, ctx);
    auto parallelism = rantanplan::StrategyFactory::create_parallelism(spec);

    // Take interference from pipeline result (pre-built by InterferencePass)
    const rantanplan::InterferenceAnalysis* interference_ptr = pipeline_result.interference.get();

    parallelism->set_interference_analyzer(std::move(pipeline_result.interference));
    encoder->set_parallelism_strategy(std::move(parallelism));
    encoder->set_symmetry_data(pipeline_result.symmetry_data);
    encoder->set_state_constraints(std::move(pipeline_result.state_constraints));

    auto planner = rantanplan::StrategyFactory::create_planner(
        spec, problem, *encoder, ctx, interference_ptr);
    planner->set_arithmetic_profile(pipeline_result.arithmetic_profile);
    rantanplan::StrategyFactory::configure_planner(*planner, spec, pipeline_result);

    // Pass pre-built achiever analysis to PDLA planners (from ActionPruningPass)
    if (pipeline_result.achievers && rantanplan::uses_pdla(spec)) {
        auto* pdla_planner = dynamic_cast<rantanplan::PDLAPlanner*>(planner.get());
        if (pdla_planner) {
            pdla_planner->set_achievers(std::move(pipeline_result.achievers));
        }
    }

    // Get solver reference and create propagator using factory
    auto propagator = rantanplan::StrategyFactory::create_propagator(spec, planner->get_solver(), problem, *encoder);

    // Add logging module if requested
    if (config.has_log_file()) {
        propagator->add_module(std::make_unique<rantanplan::LoggingModule>(
            config.logging.log_file, config.planner.strategy));
    }

    planner->set_propagator_strategy(std::move(propagator));

    // Call planner->search() and get the result
    rantanplan::Plan plan = planner->search();
    if (planner->solution_found()) {
        if (planner->optimality_proven()) {
            result.set_status(PlanGenerationResult_Status_SOLVED_OPTIMALLY);
        } else {
            result.set_status(PlanGenerationResult_Status_SOLVED_SATISFICING);
        }

        *result.mutable_plan() = rantanplan::plan_to_protobuf(plan);

        if (config.is_info()) {
            log_message = result.add_log_messages();
            log_message->set_level(LogMessage_LogLevel_INFO);
            std::string msg = "Plan found with " + std::to_string(plan.length()) + " actions.";
            if (rantanplan::uses_branch_and_bound(spec)) {
                msg += " Cost: " + std::to_string(planner->best_cost()) + ".";
                if (planner->optimality_proven()) {
                    msg += " (OPTIMAL)";
                }
            }
            log_message->set_message(msg);
        }
    } else if (planner->timed_out()) {
        result.set_status(PlanGenerationResult_Status_TIMEOUT);
        if (config.is_info()) {
            log_message = result.add_log_messages();
            log_message->set_level(LogMessage_LogLevel_INFO);
            log_message->set_message("Timeout reached.");
        }
    } else {
        // No plan found within search limits (max_steps exhausted)
        result.set_status(PlanGenerationResult_Status_UNSOLVABLE_INCOMPLETELY);
        if (config.is_info()) {
            log_message = result.add_log_messages();
            log_message->set_level(LogMessage_LogLevel_INFO);
            log_message->set_message("No plan found within the search limit.");
        }
    }

    if (config.is_verbose()) {
        log_message = result.add_log_messages();
        log_message->set_level(LogMessage_LogLevel_INFO);
        log_message->set_message("Finished solving the problem.");
    }

    // Add memory tracking information to result
    double current_memory = rantanplan::MemoryTracker::instance().get_current_memory_mb();

    log_message = result.add_log_messages();
    log_message->set_level(LogMessage_LogLevel_INFO);
    log_message->set_message("Memory usage - Current: " + std::to_string(current_memory) + " MB");

    return result;
}

int export_formula(const rantanplan::Problem& problem) {
    auto& config = rantanplan::Config::instance();

    // Validate formula export parameters
    if (config.formula_export.timestep < 0) {
        rantanplan::Logger::instance().error("--formula-timestep must be specified with --export-formula");
        return 1;
    }

    if (config.formula_export.output_file.empty()) {
        rantanplan::Logger::instance().error("--formula-output must be specified with --export-formula");
        return 1;
    }

    // Only allow formula export with strategies that support it
    const auto& spec = rantanplan::StrategyRegistry::get(config.planner.strategy);
    if (!rantanplan::supports_formula_export(spec)) {
        rantanplan::Logger::instance().error(
            "Formula export is not supported by strategy '" + config.planner.strategy + "'");
        return 1;
    }

    try {
        // Create Z3 context
        z3::context ctx;

        // Create encoder, parallelism, and interference analyzer using factory
        auto encoder = rantanplan::StrategyFactory::create_encoder(spec, problem, ctx);
        auto parallelism = rantanplan::StrategyFactory::create_parallelism(spec);
        auto interference = rantanplan::StrategyFactory::create_interference(spec, problem);

        // Inject interference analyzer into parallelism strategy
        parallelism->set_interference_analyzer(std::move(interference));

        // Set parallelism strategy on encoder
        encoder->set_parallelism_strategy(std::move(parallelism));

        rantanplan::Logger::instance().info("Exporting formula for timestep " + std::to_string(config.formula_export.timestep)
                                       + " using strategy: " + encoder->get_parallelism_strategy_name());

        // Create formula exporter and generate formula
        rantanplan::FormulaExporter exporter(problem, *encoder, ctx);
        std::string formula = exporter.export_formula(config.formula_export.timestep);

        // Output formula to file
        std::ofstream output_file(config.formula_export.output_file);
        if (!output_file) {
            rantanplan::Logger::instance().error("Could not open output file: " + config.formula_export.output_file);
            return 1;
        }
        output_file << formula;
        rantanplan::Logger::instance().info("Formula exported to: " + config.formula_export.output_file);

        google::protobuf::ShutdownProtobufLibrary();
        return 0;

    } catch (const std::exception& e) {
        rantanplan::Logger::instance().error("Error during formula export: " + std::string(e.what()));
        google::protobuf::ShutdownProtobufLibrary();
        return 1;
    }
}

} // namespace rantanplan::api
