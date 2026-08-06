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
            "Sequential planner (incompatible with CGB and DoubleTail). "
            "It registers transitions contiguously by stride arithmetic, "
            "which those planners violate.");
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

// create_propagator is gone for M4, and so is everything it built:
// src/planners/propagators/ was DELETED rather than kept as unbuilt z3 code,
// because THEORY_PLAN reasoning replaces it wholesale (migration-plan.md §7.8).
// The z3 implementation is recoverable from git history (origin/main @ c8518db)
// if the port wants it as reference. The M4 registry only lists strategies with
// an empty propagator list, so nothing calls this. It comes back in M5-M7 as a
// TheoryPlan module-list builder rather than a PropagatorStrategy builder.
//
// PropagatorModuleKind and PropagatorFlags survive in config/strategy_spec.hpp:
// they are the spec vocabulary validate() below still reasons about, not the
// implementation.
//
// CARRY THIS FORWARD WHEN IT COMES BACK (inherited from the z3 create_propagator
// this replaced; it cost a full invalid A/B once already). Whoever posts frame
// axioms must be exactly one party:
//   - Every frame-reasoning module has to flip the encoder into lazy-frame mode
//     (the old code's encoder.set_lazy_frames(true)) AND appear in validate()
//     AND in the module-list builder. Miss the first and the encoder posts eager
//     frame axioms *while* the module enforces them lazily: still sound, tests
//     still pass, but the run solves a different formula, so every measurement
//     taken against it is void. Symptom: frame_prop.conflicts == 0. Check:
//     encoder.frame_axioms is ABSENT from the run's stats.
//   - There is now a SECOND party skipping frame axioms, independent of any
//     propagator: GroundedEncoder::encode_frames consults
//     effects_determine_successor(), so the chained encoder already omits frames
//     for its chained fluents. A frame module must not assume it is the only
//     reason a fluent has no eager frame axiom.
//
// ─── WHAT THE FORK ALREADY GIVES US, AND WHAT IT STILL NEEDS ────────────────
// State of THEORY_PLAN in the cvc5 fork (branch `empty-theory`, src/theory/plan/,
// include/cvc5/cvc5_plan.h) as of this merge. Two of the three deleted modules
// are already covered there; the third is not, and one of them needs nothing
// from us at all:
//
//   ExistsCycleModule  -> DONE. TheoryPlan::findInterferenceCycle, selected by
//                         PlanSemantics::ExistsStep. Same directional relation,
//                         same "cycle is the conflict" argument.
//   ForallMutexModule  -> DONE. TheoryPlan::findInterferingPartner, selected by
//                         PlanSemantics::ForallStep. Only pairs involving the
//                         newly-asserted action are examined, which is complete
//                         because earlier pairs were checked when their later
//                         member arrived.
//   FrameAxiomModule   -> NOT STARTED. theory_plan.h names lazy frame axioms as
//                         a goal, but theory_plan.cpp has no frame or fluent
//                         handling whatsoever: preNotifyFact reacts only to
//                         POSITIVE PLAN_DOES, and d_activeActions tracks actions
//                         only. Nothing observes PLAN_FLUENT.
//
// So the wiring this file will need is narrow: build a PlanInterferenceOracle
// over our SemanticInterferencePredicate (one virtual, `interferes(a, b)`,
// directional -- our predicate is already exactly that shape, and its footprint
// prefilter belongs on our side of the boundary since the theory asks per
// ordered pair) and call Solver::configurePlanTheory(semantics, oracle). Atom
// identity already matches: PLAN_DOES(action.id(), t), PLAN_FLUENT(ground
// fluent id, t), both over the problem's own dense ids.
//
// TWO CONSTRAINTS THE ORACLE IMPOSES ON US, both from cvc5_plan.h:
//   - The verdict must be ASSIGNMENT INDEPENDENT. Ours is: interference is a
//     property of an action pair, not of a state. Do not "improve" it with
//     anything that reads the current assignment.
//   - The oracle MUST NOT call back into the asking solver. Our predicate runs
//     its own TermManager/Solver, so this holds today -- keep it that way.
//
// FOR FRAMES SPECIFICALLY, the open design question is what the theory can see.
// A lazy frame axiom has to notice f^t != f^{t+1} and name the actions that
// could explain it, so the theory must observe fluent VALUES:
//   - Bool-sorted PLAN_FLUENT is a literal owned by THEORY_PLAN, so it does
//     reach preNotifyFact. Boolean frames are therefore implementable with the
//     machinery already there, plus an EPC index handed over like the oracle.
//   - Non-Bool PLAN_FLUENT is a TERM, not a literal. Its value lives in
//     THEORY_ARITH, and TheoryPlan::needsEqualityEngine() returns false, so
//     nothing delivers it. Numeric frames need either an equality engine, arith
//     coupling, or a different formulation.
// Note how well that lines up with the chained encoder: is_chained() already
// removes the eager frame axiom for every modified NUMERIC fluent, so what a
// lazy frame module would be left to enforce is predominantly the BOOLEAN case
// -- exactly the case the theory can observe as-is. Worth confirming before
// investing in arith coupling.

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
