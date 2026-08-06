#include "strategy_factory.hpp"

#include "../encoders/grounded_encoder.hpp"
#include "../encoders/chained_grounded_encoder.hpp"
#include "../encoders/r2e_grounded_encoder.hpp"

#include "../encoders/parallelism/sequential_semantics.hpp"
#include "../encoders/parallelism/forall_semantics.hpp"
#include "../encoders/parallelism/exists_semantics.hpp"

#include "../analysis/eager_interference_analysis.hpp"
#include "../analysis/eager_semantic_interference_analysis.hpp"
#include "../analysis/lazy_interference_analysis.hpp"
#include "../analysis/semantic_interference_analysis.hpp"

#include "../planners/incremental_horizon_planner.hpp"
#include "../planners/double_tail_planner.hpp"
#include "../planners/branch_and_bound_planner.hpp"
#include "../planners/cgb_planner.hpp"

#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"

#include <algorithm>
#include <stdexcept>

namespace rantanplan {

static bool is_lazy(InterferenceKind kind) {
    return kind == InterferenceKind::LazySyntactic ||
           kind == InterferenceKind::LazySemantic;
}

static bool is_none(InterferenceKind kind) {
    return kind == InterferenceKind::None;
}

void StrategyFactory::validate(const StrategySpec& spec,
                               const std::string& strategy_name,
                               const std::string& horizon_schedule) {
    // Lazy interference requires a propagator that uses has_interference()
    // rather than graph methods (encode_parallelism needs the graph).
    // Lazy + ForallEager needs no check here: ForallMutexModule takes an
    // InterferenceGraphAnalysis at construction, which lazy analyzers are not.
    if (is_lazy(spec.interference) && spec.propagator.empty()) {
        throw std::invalid_argument(
            "Lazy interference requires a propagator (empty propagator needs "
            "graph methods that lazy analysis does not provide)");
    }

    if (is_none(spec.interference) && spec.semantics != SemanticsKind::Sequential) {
        throw std::invalid_argument(
            "No interference analysis requires Sequential semantics "
            "(Forall/Exists semantics need interference for mutex constraints)");
    }

    if (spec.propagator.count(PropagatorModuleKind::FrameAxiom) &&
        (spec.planner == PlannerKind::CGB || uses_double_tail(spec))) {
        throw std::invalid_argument(
            "FrameAxiom propagator module is only validated with the "
            "Sequential planner (incompatible with CGB and DoubleTail)");
    }

    if (horizon_schedule != "linear" && uses_double_tail(spec)) {
        throw std::invalid_argument(
            "Non-linear horizon schedule '" + horizon_schedule +
            "' is not compatible with double-tail strategy '" + strategy_name + "'. "
            "Use a non-dt strategy or --horizon-schedule linear.");
    }
}

void StrategyFactory::adjust_spec(StrategySpec& spec, const Problem& problem) {
    // Declared numeric bounds (bounded int/real fluents) are only sound with the
    // Grounded encoder, which asserts them at every concrete timestep via
    // encode_state_constraints. The Chained and R2E encoders reason over
    // chain-substituted intermediate variables (ζ) that bypass those boundary
    // constraints, so a plan could transiently violate a declared bound. Bail
    // out rather than silently drop the bound (per-variable soundness analysis
    // and encoder demotion are left for a follow-up).
    if (problem.has_declared_bounds() && spec.encoder != EncoderFamily::Grounded) {
        throw std::invalid_argument(
            "Declared numeric bounds (bounded int/real fluent types) are only "
            "sound with the Grounded encoder; this strategy's chained/relaxed "
            "encoder uses intermediate variables that can violate the bounds. "
            "Use a Grounded-encoder strategy such as els, seq, or f.");
    }

    if (uses_branch_and_bound(spec) &&
        problem.has_metric() && problem.has_state_dependent_costs() &&
        sdac_unsafe(spec)) {
        Logger::instance().info(
            "SDAC detected with exists-step semantics — downgrading to "
            "forall semantics for sound cost evaluation.");
        spec.semantics = SemanticsKind::Forall;
        if (spec.propagator.count(PropagatorModuleKind::ExistsCycle)) {
            spec.propagator.erase(PropagatorModuleKind::ExistsCycle);
            spec.propagator.insert(PropagatorModuleKind::ForallLazy);
        }
    }

    if (uses_branch_and_bound(spec) &&
        problem.has_metric() && problem.has_state_dependent_costs()) {
        Logger::instance().info(
            "SDAC detected: abstract suffix uses RPG-derived cost lower bounds.");
    }
}

/// The interference analyzer plus, for eager kinds, its graph-bearing view
/// (same object; the typed pointer lets graph consumers avoid downcasts).
struct CreatedInterference {
    std::unique_ptr<InterferenceOracle> oracle;
    const InterferenceGraphAnalysis* graph = nullptr;
};

static CreatedInterference create_interference(
    const StrategySpec& spec, const Problem& problem) {
    CreatedInterference result;
    switch (spec.interference) {
        case InterferenceKind::None:
            return result;
        case InterferenceKind::EagerSyntactic: {
            auto eager = std::make_unique<EagerInterferenceAnalysis>(problem);
            result.graph = eager.get();
            result.oracle = std::move(eager);
            return result;
        }
        case InterferenceKind::EagerSemantic: {
            auto eager = std::make_unique<EagerSemanticInterferenceAnalysis>(problem);
            result.graph = eager.get();
            result.oracle = std::move(eager);
            return result;
        }
        case InterferenceKind::LazySyntactic:
            result.oracle = std::make_unique<LazyInterferenceAnalysis>(problem);
            return result;
        case InterferenceKind::LazySemantic:
            result.oracle = std::make_unique<SemanticInterferenceAnalysis>(problem);
            return result;
    }
    throw std::invalid_argument("Unknown interference kind");
}

/// Build the parallelism strategy with its interference analyzer injected.
/// The analyzer is null exactly for InterferenceKind::None, which validate()
/// restricts to Sequential semantics (the only strategy that ignores it).
static std::unique_ptr<ParallelismStrategy> create_parallelism(
    const StrategySpec& spec, CreatedInterference interference) {
    switch (spec.semantics) {
        case SemanticsKind::Sequential:
            return std::make_unique<SequentialSemantics>();
        case SemanticsKind::Forall:
            return std::make_unique<ForallSemantics>(std::move(interference.oracle),
                                                     interference.graph);
        case SemanticsKind::Exists:
            return std::make_unique<ExistsSemantics>(std::move(interference.oracle),
                                                     interference.graph);
    }
    throw std::invalid_argument("Unknown semantics kind");
}

std::unique_ptr<BaseEncoder> StrategyFactory::create_encoder(
    const StrategySpec& spec, const Problem& problem, cvc5::TermManager& ctx) {
    // Eager interference variants pay their analysis cost here (backend
    // setup), tracked separately from the pipeline as backend.interference_ms.
    CreatedInterference interference;
    {
        ScopedTimer timer("backend.interference_ms");
        interference = create_interference(spec, problem);
    }
    auto parallelism = create_parallelism(spec, std::move(interference));

    switch (spec.encoder) {
        case EncoderFamily::Grounded:
            return std::make_unique<GroundedEncoder>(problem, ctx, std::move(parallelism));
        case EncoderFamily::Chained:
            return std::make_unique<ChainedGroundedEncoder>(problem, ctx, std::move(parallelism));
        case EncoderFamily::R2E:
            return std::make_unique<R2EGroundedEncoder>(problem, ctx, std::move(parallelism));
    }
    throw std::invalid_argument("Unknown encoder family");
}

std::unique_ptr<BaseEncoder> StrategyFactory::create_encoder(
    const StrategySpec& spec, PreprocessedProblem& pre, cvc5::TermManager& ctx) {
    auto encoder = create_encoder(spec, pre.problem, ctx);
    encoder->set_symmetry_data(pre.symmetry_data);
    encoder->set_state_constraints(std::move(pre.state_constraints));
    return encoder;
}

// create_propagator is gone for M4: PropagatorStrategy and its four modules
// are out of the build while they are reimplemented as THEORY_PLAN reasoning
// (migration-plan.md §7.8). The M4 registry only lists strategies with an empty
// propagator list, so nothing calls it. It comes back in M5-M7 as a TheoryPlan
// module-list builder rather than a PropagatorStrategy builder.

std::unique_ptr<BasePlanner> StrategyFactory::create_planner(
    const StrategySpec& spec, PreprocessedProblem& pre,
    BaseEncoder& encoder, cvc5::TermManager& ctx) {
    const Problem& problem = pre.problem;
    const InterferenceOracle* interference =
        encoder.get_parallelism_strategy()->get_interference_analyzer();

    std::unique_ptr<BasePlanner> planner;
    switch (spec.planner) {
        case PlannerKind::Sequential:
            planner = std::make_unique<IncrementalHorizonPlanner>(problem, encoder, ctx);
            break;
        case PlannerKind::DoubleTail:
            planner = std::make_unique<DoubleTailPlanner>(problem, encoder, ctx);
            break;
        case PlannerKind::BranchAndBound:
            planner = std::make_unique<BranchAndBoundPlanner>(
                problem, encoder, ctx, interference, spec.semantics);
            break;
        case PlannerKind::CGB:
            planner = std::make_unique<CGBPlanner>(problem, encoder, ctx);
            break;
    }
    if (!planner) {
        throw std::invalid_argument("Unknown planner kind");
    }

    // Route preprocessing artifacts: profile for all planners, then
    // planner-declared consumption (achievers for CGB, SDAC bounds for B&B).
    planner->set_arithmetic_profile(pre.arithmetic_profile);
    // Options (seed, logic) must be set before the first assertFormula; the
    // planner asserts nothing until search(), so here is the last safe point.
    planner->configure_solver();
    planner->consume_artifacts(pre);
    return planner;
}

} // namespace rantanplan
