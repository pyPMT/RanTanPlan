#include "strategy_factory.hpp"

#include "../encoders/grounded_encoder.hpp"

#include "../encoders/parallelism/sequential_semantics.hpp"
#include "../encoders/parallelism/forall_semantics.hpp"
#include "../encoders/parallelism/exists_semantics.hpp"

#include "../analysis/eager_interference_analysis.hpp"
#include "../analysis/semantic_interference_analysis.hpp"

#include "../planners/incremental_horizon_planner.hpp"
#include "../planners/branch_and_bound_planner.hpp"

#include "../planners/propagators/modules/exists_cycle_module.hpp"
#include "../planners/propagators/modules/forall_mutex_module.hpp"
#include "../planners/propagators/modules/frame_axiom_module.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"

#include <algorithm>
#include <stdexcept>

namespace rantanplan {

static bool is_none(InterferenceKind kind) {
    return kind == InterferenceKind::None;
}

void StrategyFactory::validate(const StrategySpec& spec) {
    // Forall parallelism is enforced exclusively by propagator modules during
    // search (there is no encoder-side forall constraint).
    if (spec.semantics == SemanticsKind::Forall && spec.propagator.empty()) {
        throw std::invalid_argument(
            "Forall semantics requires a parallelism-managing propagator "
            "module (forall mutexes are enforced during search, not in the "
            "encoding)");
    }

    // Exists parallelism is either encoded a-priori (fixed-order/SCC mutexes,
    // needs the eager interference graph) or enforced by the ExistsCycle
    // propagator during search. No propagator + lazy interference has neither.
    if (spec.semantics == SemanticsKind::Exists && spec.propagator.empty() &&
        spec.interference != InterferenceKind::EagerSyntactic) {
        throw std::invalid_argument(
            "Propagator-free exists semantics requires eager syntactic "
            "interference (the encoded fixed-order mutexes need the whole "
            "interference graph)");
    }

    if (is_none(spec.interference) && spec.semantics != SemanticsKind::Sequential) {
        throw std::invalid_argument(
            "No interference analysis requires Sequential semantics "
            "(Forall/Exists semantics need interference for mutex constraints)");
    }
}

void StrategyFactory::adjust_spec(StrategySpec& spec, const Problem& problem) {
    if (uses_branch_and_bound(spec) &&
        problem.has_metric() && problem.has_state_dependent_costs() &&
        sdac_unsafe(spec)) {
        Logger::instance().info(
            "SDAC detected with exists-step semantics — downgrading to "
            "forall semantics for sound cost evaluation.");
        spec.semantics = SemanticsKind::Forall;
        // Forall has no encoded-parallelism path, so the downgraded spec
        // always needs the lazy forall module — including when the exists
        // strategy was propagator-free (`e`).
        spec.propagator.erase(PropagatorModuleKind::ExistsCycle);
        spec.propagator.insert(PropagatorModuleKind::ForallLazy);
    }

    if (uses_branch_and_bound(spec) &&
        problem.has_metric() && problem.has_state_dependent_costs()) {
        Logger::instance().info(
            "SDAC detected: abstract suffix uses RPG-derived cost lower bounds.");
    }
}

/// The interference oracle for the spec's InterferenceKind — the single
/// routing point deciding how interference queries are answered (precomputed
/// syntactic graph vs. on-demand semantic checks). Null for None.
static std::unique_ptr<InterferenceOracle> create_interference(
    const StrategySpec& spec, const Problem& problem) {
    switch (spec.interference) {
        case InterferenceKind::None:
            return nullptr;
        case InterferenceKind::EagerSyntactic:
            return std::make_unique<EagerInterferenceAnalysis>(problem);
        case InterferenceKind::LazySemantic:
            return std::make_unique<SemanticInterferenceAnalysis>(problem);
    }
    throw std::invalid_argument("Unknown interference kind");
}

/// Build the parallelism strategy with its interference analyzer injected.
/// The analyzer is null exactly for InterferenceKind::None, which validate()
/// restricts to Sequential semantics (the only strategy that ignores it).
/// For eager interference, ExistsSemantics additionally gets the concrete
/// graph-bearing analyzer — the encoded ∃-step mutexes need the whole graph.
static std::unique_ptr<ParallelismStrategy> create_parallelism(
    const StrategySpec& spec, std::unique_ptr<InterferenceOracle> interference) {
    const auto* eager =
        spec.interference == InterferenceKind::EagerSyntactic
            ? static_cast<const EagerInterferenceAnalysis*>(interference.get())
            : nullptr;
    switch (spec.semantics) {
        case SemanticsKind::Sequential:
            return std::make_unique<SequentialSemantics>();
        case SemanticsKind::Forall:
            return std::make_unique<ForallSemantics>(std::move(interference));
        case SemanticsKind::Exists:
            return std::make_unique<ExistsSemantics>(std::move(interference), eager);
    }
    throw std::invalid_argument("Unknown semantics kind");
}

std::unique_ptr<GroundedEncoder> StrategyFactory::create_encoder(
    const StrategySpec& spec, const Problem& problem, z3::context& ctx) {
    // Eager interference variants pay their analysis cost here (backend
    // setup), tracked separately from the pipeline as backend.interference_ms.
    std::unique_ptr<InterferenceOracle> interference;
    {
        ScopedTimer timer("backend.interference_ms");
        interference = create_interference(spec, problem);
    }
    auto parallelism = create_parallelism(spec, std::move(interference));

    return std::make_unique<GroundedEncoder>(problem, ctx, std::move(parallelism));
}

std::unique_ptr<GroundedEncoder> StrategyFactory::create_encoder(
    const StrategySpec& spec, PreprocessedProblem& pre, z3::context& ctx) {
    auto encoder = create_encoder(spec, pre.problem, ctx);
    encoder->set_symmetry_data(pre.symmetry_data);
    encoder->set_state_constraints(std::move(pre.state_constraints));
    return encoder;
}

std::unique_ptr<PropagatorStrategy> StrategyFactory::create_propagator(
    const StrategySpec& spec, z3::solver& solver,
    const Problem& problem, GroundedEncoder& encoder) {

    const auto& mods = spec.propagator;

    // Enable lazy frame mode in the encoder if frame axioms are active
    if (mods.count(PropagatorModuleKind::FrameAxiom)) {
        encoder.set_lazy_frames(true);
    }

    auto composite = std::make_unique<PropagatorStrategy>(solver, problem, encoder);

    // Parallelism modules (order: first, cheapest conflict source).
    if (mods.count(PropagatorModuleKind::ForallLazy))
        composite->add_module(std::make_unique<LazyForallMutexModule>());
    if (mods.count(PropagatorModuleKind::ExistsCycle))
        composite->add_module(std::make_unique<ExistsCycleModule>());

    // Frame axiom module (after parallelism — more expensive, skipped on conflict)
    if (mods.count(PropagatorModuleKind::FrameAxiom))
        composite->add_module(std::make_unique<FrameAxiomModule>());

    return composite;
}

std::unique_ptr<BasePlanner> StrategyFactory::create_planner(
    const StrategySpec& spec, PreprocessedProblem& pre,
    GroundedEncoder& encoder, z3::context& ctx) {
    const Problem& problem = pre.problem;

    std::unique_ptr<BasePlanner> planner;
    switch (spec.planner) {
        case PlannerKind::Sequential:
            planner = std::make_unique<IncrementalHorizonPlanner>(problem, encoder, ctx);
            break;
        case PlannerKind::BranchAndBound:
            planner = std::make_unique<BranchAndBoundPlanner>(
                problem, encoder, ctx, spec.semantics);
            break;
    }
    if (!planner) {
        throw std::invalid_argument("Unknown planner kind");
    }

    // Route preprocessing artifacts: profile for all planners, then
    // planner-declared consumption (SDAC cost bounds for B&B).
    planner->set_arithmetic_profile(pre.arithmetic_profile);
    planner->consume_artifacts(pre);
    return planner;
}

} // namespace rantanplan
