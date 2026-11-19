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

#include "../planners/propagators/modules/exists_cycle_module.hpp"
#include "../planners/propagators/modules/forall_mutex_module.hpp"
#include "../planners/propagators/modules/frame_axiom_module.hpp"
#include "../config/config.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"
#include "../util/unsupported_problem_error.hpp"

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

static bool is_semantic(InterferenceKind kind) {
    return kind == InterferenceKind::EagerSemantic ||
           kind == InterferenceKind::LazySemantic;
}

/// The modules whose manages_parallelism_constraints() is true: with one of
/// them the planner never calls encode_parallelism.
static bool manages_parallelism(PropagatorModuleKind kind) {
    return kind == PropagatorModuleKind::ForallEager ||
           kind == PropagatorModuleKind::ForallLazy ||
           kind == PropagatorModuleKind::ExistsCycle;
}

void StrategyFactory::validate(const StrategySpec& spec,
                               const std::string& strategy_name,
                               const std::string& horizon_schedule) {
    // Semantic interference lets two actions share a step when their writes
    // to one fluent commute, and it takes the step to apply those writes one
    // after another: two increases of x add up, as PDDL 2.1 allows for
    // concurrent increases. The Grounded encoder does not add them up. It
    // states each write as its own equation on the next value,
    // x^{t+1} = x^t + d, so two increases by the same amount are encoded as
    // one, and the plan it returns is invalid. Semantic interference is
    // therefore only sound with an encoder that composes the writes of one
    // step, as the Chained encoder does. Syntactic interference keeps two
    // writers of one numeric fluent in different steps, so it is sound with
    // the Grounded encoder.
    if (is_semantic(spec.interference) && spec.encoder == EncoderFamily::Grounded) {
        throw std::invalid_argument(
            "Semantic interference is only sound with an encoder that composes "
            "the writes of one step to one fluent, which the Grounded encoder "
            "does not; use the chained encoder (strategies elsc and flsc) or "
            "syntactic interference (strategies e and f)");
    }

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

    // The exists-step chains forbid the pairs of the step order unit by unit
    // (InterferenceOracle::interference_units), which split the syntactic
    // interference relation, and the order is built from the eager graph.
    // encode_parallelism writes them, which the planner skips when a module
    // manages parallelism.
    if (spec.estep_mutexes == EstepMutexes::Chains) {
        if (spec.semantics != SemanticsKind::Exists) {
            throw std::invalid_argument(
                "Exists-step chains require Exists semantics");
        }
        if (spec.interference != InterferenceKind::EagerSyntactic) {
            throw std::invalid_argument(
                "Exists-step chains require eager syntactic interference: "
                "they split the syntactic interference relation by fluent, "
                "and the step order is built from the eager graph");
        }
        if (std::any_of(spec.propagator.begin(), spec.propagator.end(), manages_parallelism)) {
            throw std::invalid_argument(
                "Exists-step chains are encoded up front, and a propagator "
                "module that manages parallelism would replace them");
        }
    }

    if (spec.propagator.count(PropagatorModuleKind::FrameAxiom) &&
        uses_double_tail(spec)) {
        throw std::invalid_argument(
            "FrameAxiom propagator module is only validated with the "
            "Sequential planner (incompatible with DoubleTail). "
            "It registers transitions contiguously by stride arithmetic, "
            "which that planner violates.");
    }

    if (horizon_schedule != "linear" && uses_double_tail(spec)) {
        throw std::invalid_argument(
            "Non-linear horizon schedule '" + horizon_schedule +
            "' is not compatible with double-tail strategy '" + strategy_name + "'. "
            "Use a non-dt strategy or --horizon-schedule linear.");
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
        spec.estep_mutexes = EstepMutexes::Pairs;  // validate: chains are exists-step only
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

    check_supported(spec, problem, Config::instance().planner.strategy);
}

void StrategyFactory::check_supported(const StrategySpec& spec, const Problem& problem,
                                      const std::string& name) {
    // Declared numeric bounds (bounded int/real fluents) hold in every state a
    // plan passes through. The Grounded encoder asserts them at every concrete
    // timestep (encode_state_constraints), and the R2E encoder on every chain
    // variable (encode_declared_bounds). The Chained encoder reasons over
    // chain-substituted intermediate variables (ζ) that bypass those boundary
    // constraints, so a plan could transiently violate a declared bound. Bail
    // out rather than silently drop the bound.
    if (problem.has_declared_bounds() && spec.encoder == EncoderFamily::Chained) {
        throw UnsupportedProblemError(
            "Declared numeric bounds (bounded int/real fluent types) are not "
            "supported by the Chained encoder: its intermediate variables can "
            "violate the bounds. Use a strategy on another encoder, such as e, "
            "f, seq or r2e.");
    }

    // Declared state invariants must hold in every state of the plan. Only
    // sequential search, with the Grounded encoder, asserts them at each one:
    if (problem.state_invariants().empty()) return;
    // - The R2E encoder deliberately skips state constraints
    //   (chain-substituted intermediate states).
    if (spec.encoder == EncoderFamily::R2E) {
        throw UnsupportedProblemError(
            "Strategy '" + name + "' (R2E encoder) does not support declared "
            "state invariants ((:constraints (always ...))). Use seq instead.");
    }
    // - Parallel semantics assert them between steps, but the plan runs a
    //   step's actions one after another, through states in between that
    //   nothing checks: on counters with (always (<= x y)), e and f put
    //   inc_x and inc_y in one step and return x = 1 > y = 0 after inc_x.
    if (spec.semantics != SemanticsKind::Sequential) {
        const char* semantics =
            spec.semantics == SemanticsKind::Forall ? "forall-step" : "exists-step";
        throw UnsupportedProblemError(
            "Strategy '" + name + "' (" + semantics + " semantics) does not "
            "support declared state invariants ((:constraints (always ...))): "
            "they are checked between steps, not between the actions of one "
            "step. Use seq instead.");
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
                                                     interference.graph,
                                                     spec.estep_mutexes);
    }
    throw std::invalid_argument("Unknown semantics kind");
}

std::unique_ptr<BaseEncoder> StrategyFactory::create_encoder(
    const StrategySpec& spec, const Problem& problem, z3::context& ctx) {
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
    const StrategySpec& spec, PreprocessedProblem& pre, z3::context& ctx) {
    auto encoder = create_encoder(spec, pre.problem, ctx);
    encoder->set_symmetry_data(pre.symmetry_data);
    encoder->set_state_constraints(std::move(pre.state_constraints));
    return encoder;
}

std::unique_ptr<PropagatorStrategy> StrategyFactory::create_propagator(
    const StrategySpec& spec, z3::solver& solver,
    const Problem& problem, BaseEncoder& encoder) {

    const auto& mods = spec.propagator;

    // Without modules a propagator would only take a callback for every
    // variable Z3 assigns, with nothing to act on.
    if (mods.empty()) return nullptr;

    // Enable lazy frame mode in the encoder if frame axioms are active.
    //
    // >>> ANY NEW FRAME-PROPAGATOR MODULE MUST BE ADDED HERE, and to validate()
    // >>> above, and to the add_module block below.  Omitting this one leaves the
    // >>> encoder posting eager frame axioms *and* the module enforcing them
    // >>> lazily: sound, and the tests still pass, but doubly enforced -- and it
    // >>> silently invalidates every measurement, because the run then solves a
    // >>> different formula.  It cost a full invalid A/B once already.  The
    // >>> symptom is frame_prop.conflicts == 0; the check is that
    // >>> encoder.frame_axioms is ABSENT from the run's stats.
    if (mods.count(PropagatorModuleKind::FrameAxiom)) {
        encoder.set_lazy_frames(true);
    }

    auto composite = std::make_unique<PropagatorStrategy>(solver, problem, encoder);

    // Parallelism modules (order: first, cheapest conflict source).
    if (mods.count(PropagatorModuleKind::ForallEager)) {
        const InterferenceGraphAnalysis* graph =
            encoder.get_parallelism_strategy()->get_interference_graph_analysis();
        if (!graph) {
            throw std::invalid_argument(
                "ForallEager propagator module requires an eager "
                "(graph-bearing) interference analyzer");
        }
        composite->add_module(std::make_unique<ForallMutexModule>(*graph));
    }
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
    BaseEncoder& encoder, z3::context& ctx) {
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
    }
    if (!planner) {
        throw std::invalid_argument("Unknown planner kind");
    }

    // Route preprocessing artifacts: profile for all planners, then
    // planner-declared consumption (SDAC bounds for B&B).
    planner->set_arithmetic_profile(pre.arithmetic_profile);
    planner->consume_artifacts(pre);
    return planner;
}

} // namespace rantanplan
