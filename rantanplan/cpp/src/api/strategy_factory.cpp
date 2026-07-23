#include "strategy_factory.hpp"

#include "../encoders/grounded_encoder.hpp"

#include "../encoders/parallelism/sequential_semantics.hpp"
#include "../encoders/parallelism/forall_semantics.hpp"
#include "../encoders/parallelism/exists_semantics.hpp"

#include "../analysis/eager_interference_analysis.hpp"
#include "../analysis/semantic_interference_analysis.hpp"

#include "../planners/incremental_horizon_planner.hpp"
#include "../planners/branch_and_bound_planner.hpp"

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
        // The SDAC exists->forall downgrade needs the lazy forall propagator
        // module, which is temporarily out of the build during the cvc5
        // migration (it returns with the in-solver propagator port). Failing
        // here is deliberate: the alternative -- downgrading to forall with no
        // module -- would either throw deep inside the encoder or, worse,
        // silently drop the parallelism constraints. RESTORE the downgrade
        // (semantics = Forall, propagator = {ForallLazy}) when the propagator
        // port lands; see migration-log.md.
        throw std::invalid_argument(
            "State-dependent action costs with exists-step semantics need the "
            "forall propagator, which is unavailable during the cvc5 "
            "migration. Use --strategy seq for SDAC optimization.");
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
    const StrategySpec& spec, const Problem& problem, cvc5::TermManager& ctx) {
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
    const StrategySpec& spec, PreprocessedProblem& pre, cvc5::TermManager& ctx) {
    auto encoder = create_encoder(spec, pre.problem, ctx);
    encoder->set_symmetry_data(pre.symmetry_data);
    encoder->set_state_constraints(std::move(pre.state_constraints));
    return encoder;
}

// create_propagator is gone for the cvc5 swap: PropagatorStrategy and its
// modules are out of the build while they are reimplemented as in-solver
// (THEORY_PLAN) reasoning. The reduced registry only lists strategies with an
// empty propagator list, so nothing calls it. It comes back with the
// propagator port (see migration-log.md).

std::unique_ptr<BasePlanner> StrategyFactory::create_planner(
    const StrategySpec& spec, PreprocessedProblem& pre,
    GroundedEncoder& encoder, cvc5::TermManager& ctx) {
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
    // Options (seed, logic) must be set before the first assertFormula; the
    // planner asserts nothing until search(), so here is the last safe point.
    planner->configure_solver();
    planner->consume_artifacts(pre);
    return planner;
}

} // namespace rantanplan
