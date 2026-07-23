from typing import Callable, IO, Iterator, Optional

from unified_planning.shortcuts import Fraction, PlanValidator
from unified_planning.engines.results import PlanGenerationResult, LogMessage, LogLevel
from unified_planning.engines.mixins import OneshotPlannerMixin
from unified_planning.engines.mixins.oneshot_planner import OptimalityGuarantee
from unified_planning.engines.mixins.anytime_planner import AnytimePlannerMixin, AnytimeGuarantee
from unified_planning.engines import PlanGenerationResultStatus, Engine, CompilationKind, ValidationResultStatus
from unified_planning.engines.compilers import Grounder
from unified_planning.exceptions import UPException
from .converter import convert_problem
from unified_planning.model import ProblemKind, Problem
from unified_planning.plans import SequentialPlan, ActionInstance
from unified_planning.shortcuts import get_environment
from unified_planning.model.fluent import get_all_fluent_exp

import queue
import sys
import threading

try:
    from . import _rantanplan
except ImportError:  # extension module not built yet
    _rantanplan = None


# ═══════════════════════════════════════════════════════════════════════
# Shared base class
# ═══════════════════════════════════════════════════════════════════════

class _RantanPlanBase(Engine):
    """Shared logic for all RantanPlan UP engines."""

    def __init__(self, **options):
        Engine.__init__(self)

        self._verbosity = options.get('verbosity')
        # M4: 'els' (exists-step, LAZY semantic interference) needs a
        # propagator, and those are out of the build until M5. 'e' is the same
        # exists-step semantics with EAGER interference.
        # REVERT TO 'els' AT M5, together with config.hpp's Planner::strategy.
        # NOTE this default shadows the C++ one: the wrapper always passes
        # --strategy, so Config::Planner::strategy is never consulted.
        self._strategy = options.get('strategy', 'e')
        self._max_steps = options.get('max_steps')
        self._symmetries = options.get('symmetries', False)
        self._stats_file = options.get('stats_file')
        self._no_cnf_normalization = options.get('no_cnf_normalization', False)
        self._smt_rpg_checker = options.get('smt_rpg_checker', False)
        self._no_goal_relevance = options.get('no_goal_relevance', False)
        self._horizon_schedule = options.get('horizon_schedule')
        self._log_file = options.get('log_file')
        self._up_grounding = options.get('up_grounding', False)
        self._numeric_grounding = options.get('numeric_grounding', True)
        self._mode = options.get('mode')
        self._output_plan = options.get('output_plan')
        # Set per solve call when the caller passes output_stream (UP's
        # official log channel); None means stdout as usual.
        self._output_stream: Optional[IO[str]] = None

    # ── Verbosity helpers ────────────────────────────────────────────────

    @property
    def _is_silent(self) -> bool:
        return self._verbosity == "silent"

    @property
    def _is_verbose(self) -> bool:
        return self._verbosity in ("verbose", "debug")

    def _log(self, msg: str) -> None:
        if not self._is_silent:
            print(msg, file=self._output_stream or sys.stdout)

    def _log_verbose(self, msg: str) -> None:
        if self._is_verbose:
            print(msg, file=self._output_stream or sys.stdout)

    def _log_warning(self, msg: str) -> None:
        if not self._is_silent:
            print(msg, file=self._output_stream or sys.stdout)

    @staticmethod
    def _log_error(msg: str) -> None:
        print(msg, file=sys.stderr)

    # ── Engine metadata ──────────────────────────────────────────────────

    @property
    def name(self) -> str:
        return "rantanplan"

    @staticmethod
    def supported_kind():
        supported_kind = ProblemKind()
        supported_kind.set_problem_class("ACTION_BASED")
        supported_kind.set_problem_type("GENERAL_NUMERIC_PLANNING")
        supported_kind.set_problem_type("SIMPLE_NUMERIC_PLANNING")
        supported_kind.set_typing('FLAT_TYPING')
        supported_kind.set_typing('HIERARCHICAL_TYPING')
        supported_kind.set_numbers('CONTINUOUS_NUMBERS')
        supported_kind.set_numbers('DISCRETE_NUMBERS')
        supported_kind.set_numbers('BOUNDED_TYPES')
        supported_kind.set_parameters('BOUNDED_INT_ACTION_PARAMETERS')
        supported_kind.set_conditions_kind('INT_VARIABLES')
        supported_kind.set_fluents_type('NUMERIC_FLUENTS')
        supported_kind.set_fluents_type('REAL_FLUENTS')
        supported_kind.set_fluents_type('INT_FLUENTS')
        supported_kind.set_fluents_type('OBJECT_FLUENTS')
        supported_kind.set_fluents_type('ARRAY_FLUENTS')
        supported_kind.set_fluents_type('SET_FLUENTS')
        supported_kind.set_initial_state('UNDEFINED_INITIAL_NUMERIC')
        supported_kind.set_conditions_kind('NEGATIVE_CONDITIONS')
        supported_kind.set_conditions_kind('DISJUNCTIVE_CONDITIONS')
        supported_kind.set_conditions_kind('EQUALITIES')
        supported_kind.set_conditions_kind('EXISTENTIAL_CONDITIONS')
        supported_kind.set_conditions_kind('UNIVERSAL_CONDITIONS')
        supported_kind.set_conditions_kind('COUNTING')
        supported_kind.set_effects_kind('CONDITIONAL_EFFECTS')
        supported_kind.set_effects_kind('INCREASE_EFFECTS')
        supported_kind.set_effects_kind('DECREASE_EFFECTS')
        supported_kind.set_effects_kind('FLUENTS_IN_NUMERIC_ASSIGNMENTS')
        supported_kind.set_effects_kind('STATIC_FLUENTS_IN_NUMERIC_ASSIGNMENTS')
        supported_kind.set_effects_kind('STATIC_FLUENTS_IN_BOOLEAN_ASSIGNMENTS')
        supported_kind.set_effects_kind('FLUENTS_IN_BOOLEAN_ASSIGNMENTS')
        supported_kind.set_effects_kind('STATIC_FLUENTS_IN_OBJECT_ASSIGNMENTS')
        supported_kind.set_effects_kind('FLUENTS_IN_OBJECT_ASSIGNMENTS')
        supported_kind.set_effects_kind('FORALL_EFFECTS')
        supported_kind.set_quality_metrics('FINAL_VALUE')
        supported_kind.set_quality_metrics('ACTIONS_COST')
        supported_kind.set_quality_metrics('PLAN_LENGTH')
        supported_kind.set_actions_cost_kind('INT_NUMBERS_IN_ACTIONS_COST')
        supported_kind.set_actions_cost_kind('REAL_NUMBERS_IN_ACTIONS_COST')
        supported_kind.set_actions_cost_kind('FLUENTS_IN_ACTIONS_COST')
        supported_kind.set_actions_cost_kind('STATIC_FLUENTS_IN_ACTIONS_COST')
        # Declared state invariants: `(:constraints (always f))` only —
        # UP classifies always-only constraints as STATE_INVARIANTS, so
        # sometime/sometime-after/... problems are rejected upstream.
        supported_kind.set_constraints_kind('STATE_INVARIANTS')
        return supported_kind

    @staticmethod
    def supports(problem_kind):
        return problem_kind <= _RantanPlanBase.supported_kind()

    def destroy(self):
        pass

    # ── Solver arguments ─────────────────────────────────────────────────

    def _build_args(self, timeout: Optional[float] = None) -> list:
        """The planner's flags, passed to the in-process module."""
        command = ["--strategy", self._strategy]

        if self._verbosity is not None:
            command.extend(["--verbosity", self._verbosity])
        if self._max_steps is not None:
            command.extend(["--max-steps", str(self._max_steps)])
        if self._symmetries:
            command.append("--symmetries")
        if self._smt_rpg_checker:
            command.append("--smt-rpg-checker")
        if self._no_goal_relevance:
            command.append("--no-goal-relevance")
        if self._no_cnf_normalization:
            command.append("--no-cnf-normalization")
        if self._horizon_schedule is not None:
            command.extend(["--horizon-schedule", self._horizon_schedule])
        if self._log_file is not None:
            command.extend(["--log-file", self._log_file])
        if self._up_grounding:
            command.append("--up-grounding")
        if not self._numeric_grounding:
            command.append("--no-numeric-grounding")
        if self._mode is not None:
            command.extend(["--mode", self._mode])
        if self._output_plan is not None:
            command.extend(["--output-plan", self._output_plan])
        if timeout:
            command.extend(["--timeout", str(int(timeout))])

        return command

    # ── Problem preparation ──────────────────────────────────────────────

    def _validate_plan(self, problem: Problem, plan: SequentialPlan) -> bool:
        # UP ships no plan validator for array- or set-fluent problems, so
        # attempting validation here only raises. Skip gracefully — such plans
        # are checked externally in the test suite (arrays against the
        # ArrayFluentsRemover-compiled equivalent; sets against a set-semantics
        # simulator). Native array/set support is compiled to scalar cells /
        # boolean membership cells before solving, so the underlying search is
        # validated like any scalar problem.
        if problem.kind.has_array_fluents():
            self._log("Plan validation: SKIPPED (UP has no array-fluent validator)")
            return True
        if problem.kind.has_set_fluents():
            self._log("Plan validation: SKIPPED (UP has no set-fluent validator)")
            return True
        try:
            self._materialize_numeric_for_validator(problem)
            with PlanValidator(problem_kind=problem.kind, plan_kind=plan.kind) as validator:
                validation_result = validator.validate(problem, plan)  # type: ignore[attr-defined]

                if validation_result.status == ValidationResultStatus.VALID:
                    self._log("Plan validation: VALID")
                    self._log(f"  The plan with {len(plan.actions)} actions is correct and executable.")
                    return True
                else:
                    self._log_warning(f"Plan validation: {validation_result.status.name}")
                    if validation_result.log_messages:
                        for log_msg in validation_result.log_messages:
                            self._log_warning(f"  Validation {log_msg.level.name}: {log_msg.message}")
                    else:
                        self._log_warning("  No detailed validation messages available.")
                    return False

        except Exception as e:
            self._log_error(f"Plan validation failed with error: {e}")
            return False

    def _materialize_numeric_for_validator(self, problem: Problem):
        """Make every numeric fluent's initial value explicit for UP's validator.

        UP's PlanValidator leaves a numeric fluent without an explicit initial
        value *undefined* during simulation (UNDEFINED_INITIAL_NUMERIC) rather
        than applying initial_defaults, then reports "fluents with undefined
        values". The solver initialises them in the C++ CWA pass; this mirrors
        that same rule (a declared default_initial_value, else 0 clamped into the
        declared bounds) so the validator checks the plan against the state the
        solver actually used. It runs *after* IR conversion, only on the
        non-array validation path, so it never feeds the (sparse) IR.

        (This enumerates via ``get_all_fluent_exp``, not ``problem.initial_values``:
        the latter returns only explicitly-set values -- UP does not default
        numeric fluents at all -- so it cannot supply the un-init instances that
        are exactly the ones the validator leaves undefined.)"""
        _em = get_environment().expression_manager
        for fluent in problem.fluents:
            tpe = fluent.type
            if not (tpe.is_int_type() or tpe.is_real_type()):
                continue
            declared = problem.fluents_defaults.get(fluent)
            for fe in get_all_fluent_exp(problem, fluent):
                if fe in problem.explicit_initial_values:
                    continue
                if declared is not None:
                    problem.set_initial_value(fe, declared)
                    continue
                v = 0
                if tpe.lower_bound is not None and v < tpe.lower_bound:
                    v = tpe.lower_bound
                if tpe.upper_bound is not None and v > tpe.upper_bound:
                    v = tpe.upper_bound
                problem.set_initial_value(
                    fe, _em.Int(v) if tpe.is_int_type() else _em.Real(Fraction(v)))

    def _set_validator_defaults(self, task: Problem):
        """Declare the type-level defaults UP's *external* PlanValidator needs.

        Solver-side initialization now lives entirely in C++: the CWA pass
        materialises defaults over the reachable grounded fluents (closed-world
        false for booleans; a declared default_initial_value, else 0 clamped
        into the declared bounds, for numerics; a hard error for uninitialised
        object fluents). Array cells inherit those rules after lowering.

        This method does NOT feed the IR. It only lets UP's PlanValidator fill
        unspecified fluents when it checks a returned plan on the non-array path
        (array-fluent problems skip UP validation -- UP ships no validator for
        them). Without it, the validator raises on any closed-world boolean."""
        _env = get_environment()
        _tm = _env.type_manager
        _em = _env.expression_manager
        task.initial_defaults.update({_tm.RealType(): _em.Real(Fraction(0))})
        task.initial_defaults.update({_tm.IntType():  _em.Int(0)})
        task.initial_defaults.update({_tm.BoolType(): _em.Bool(False)})

    def _compile_problem(self, problem: Problem):
        current_problem = problem
        compilation_maps = []

        self._log_verbose("Starting problem compilation pipeline...")

        # Quantifiers (and integer parameters, and forall-effects) are compiled
        # away entirely in the C++ pipeline: IntParameterInstantiationPass then
        # QuantifierExpansionPass, both before grounding. The Python-side
        # QuantifiersRemover is intentionally not used.

        # Optionally ground on the Python side instead of in C++.
        # (CNF normalization happens in the C++ pipeline, after grounding and
        # static-fluent folding — see passes/cnf_normalization_pass.cpp.)
        if self._up_grounding:
            self._log_verbose("  Applying UP grounder ...")
            grounder = Grounder()
            grounding_result = grounder.compile(current_problem, CompilationKind.GROUNDING)
            current_problem = grounding_result.problem
            compilation_maps.append(grounding_result)
            self._log_verbose("  Grounding completed.")
        else:
            self._log_verbose("  Skipping Python-side grounding (the backend will ground).")

        self._log_verbose("Problem compilation pipeline completed.")

        class CombinedCompilationResult:
            def __init__(self, problem, compilation_maps):
                self.problem = problem
                self.compilation_maps = compilation_maps

            def map_back_action_instance(self, action_instance):
                current_ai = action_instance
                for comp_result in reversed(self.compilation_maps):
                    if hasattr(comp_result, 'map_back_action_instance'):
                        current_ai = comp_result.map_back_action_instance(current_ai)
                        if current_ai is None:
                            return None
                return current_ai

        combined_result = CombinedCompilationResult(current_problem, compilation_maps)
        return current_problem, combined_result

    @staticmethod
    def _check_no_nested_fluents(problem: Problem):
        """Reject the nested-fluent shapes term resolution does not support.

        Nested fluent terms — (city-of (vehicle-location ?t)) — are compiled
        away by the C++ TermResolutionPass: ghost parameters for
        object-valued inner terms in action expressions, per-position
        instance splitting for state-dependent array indices, and
        value-universe expansion for object-valued inner terms in goals
        (docs/specs/undefinedness-semantics.md). What remains rejected here
        is an inner term of int/real/bool type in a *fluent argument*
        position (as opposed to an array index slot); state-dependent
        indices in goals are rejected C++-side.
        """
        def contains_fluent(node):
            if node.is_fluent_exp():
                return True
            return any(contains_fluent(arg) for arg in node.args)

        def check(node, where):
            if node.is_fluent_exp():
                for arg in node.args:
                    if contains_fluent(arg) and not arg.type.is_user_type():
                        raise UPException(
                            f"Nested fluent term {arg} in {where} has "
                            f"non-object type '{arg.type}'; only "
                            f"object-valued nested fluent terms are "
                            f"supported.")
            for arg in node.args:
                check(arg, where)

        for action in problem.actions:
            for prec in action.preconditions:
                check(prec, f"action '{action.name}' precondition")
            for effect in action.effects:
                check(effect.fluent, f"action '{action.name}' effect target")
                check(effect.value, f"action '{action.name}' effect value")
                check(effect.condition,
                      f"action '{action.name}' effect condition")

        for goal in problem.goals:
            check(goal, "a goal")

    @staticmethod
    def _check_supported_action_parameters(problem: Problem):
        """Reject action-parameter types the grounder cannot enumerate.

        Only finite-domain (fully bounded) integer parameters are supported.
        A real parameter, or an integer parameter missing either bound, has no
        finite domain to ground over. Rejecting here (rather than letting the
        grounder drop the action) avoids a misleading UNSOLVABLE_PROVEN result,
        since name-based engine selection only warns about unsupported kinds
        instead of refusing the problem.
        """
        for action in problem.actions:
            for param in action.parameters:
                ptype = param.type
                if ptype.is_real_type():
                    raise UPException(
                        f"Real-valued action parameters are not supported. "
                        f"Action '{action.name}' parameter '{param.name}' has "
                        f"type {ptype}.")
                if ptype.is_int_type() and (
                        ptype.lower_bound is None or ptype.upper_bound is None):
                    raise UPException(
                        f"Only bounded integer action parameters are supported "
                        f"(the grounder enumerates their [lower, upper] "
                        f"interval). Action '{action.name}' parameter "
                        f"'{param.name}' has type {ptype}, which is missing a "
                        f"lower and/or upper bound.")

    def _prepare_problem(self, problem: Problem):
        """Initialize fluents, compile, convert to the IR.

        Returns (compiled_problem, compilation_result, problem_ir).
        """
        self._check_no_nested_fluents(problem)
        self._check_supported_action_parameters(problem)

        self._set_validator_defaults(problem)
        compiled_problem, compilation_result = self._compile_problem(problem)

        problem_ir = convert_problem(compiled_problem)

        return compiled_problem, compilation_result, problem_ir

    # ── Result conversion ────────────────────────────────────────────────

    @staticmethod
    def _up_status(status: int) -> PlanGenerationResultStatus:
        name = _rantanplan.SolveStatus(status).name
        return PlanGenerationResultStatus[name]

    @staticmethod
    def _up_log_level(level: int) -> LogLevel:
        return (LogLevel.DEBUG, LogLevel.INFO, LogLevel.WARNING, LogLevel.ERROR)[level]

    @staticmethod
    def _build_plan(instances, compiled_problem: Problem,
                    compilation_result) -> SequentialPlan:
        """Rebuild a SequentialPlan from IR action instances by name lookup,
        mapping each back through the compilation steps.

        CONTRACT (name-based plan mapping): the returned parameter strings
        are zipped against the *compiled* problem's lifted action signature,
        so plan mapping only works while lowerings preserve action names and
        signatures. A lowering that splits instances (e.g. ghost parameters)
        must bake the extra values into the instance name the way C++
        int-parameter instantiation already does; a feature whose per-step
        values only exist in the solver model (symbolic indices) must extend
        ResultIR to carry them — this zip cannot invent parameters.
        """
        action_map = {a.name: a for a in compiled_problem.actions}
        object_map = {o.name: o for o in compiled_problem.all_objects}
        em = compiled_problem.environment.expression_manager

        new_actions: list[ActionInstance] = []
        for instance in instances:
            action = action_map[instance.action_name]
            # A bounded-int parameter comes back as an integer literal string
            # (e.g. "2"); rebuild it as Int(2) against the lifted signature.
            # Object parameters resolve to their UP Object.
            params = tuple(
                em.Int(int(value)) if ap.type.is_int_type() else object_map[value]
                for value, ap in zip(instance.parameters, action.parameters))
            mapped_ai = compilation_result.map_back_action_instance(
                ActionInstance(action, params))
            assert mapped_ai is not None
            new_actions.append(mapped_ai)

        return SequentialPlan(new_actions)

    def _write_stats_file(self, stats) -> None:
        """Write the per-solve stats to self._stats_file (the --stats-file
        flag), in the same 'key: value' format the C++ side used to write."""
        try:
            with open(self._stats_file, "w") as f:
                for key, value in stats:
                    f.write(f"{key}: {value:.3f}\n")
            self._log(f"Statistics written to: {self._stats_file}")
        except OSError as e:
            self._log_error(f"Could not open stats file for writing: {e}")

    def _convert_result(self, result_ir,
                        compiled_problem: Problem,
                        compilation_result,
                        original_problem: Problem) -> PlanGenerationResult:
        """Map a ResultIR to a UP result: rebuild action instances by name,
        map them back through the compilation steps, validate. Per-solve
        stats surface as PlanGenerationResult.metrics (stringified)."""
        status = self._up_status(result_ir.status)
        log_messages = [LogMessage(level=self._up_log_level(msg.level), message=msg.message)
                        for msg in result_ir.log_messages]

        metrics = {key: str(value) for key, value in result_ir.stats}
        if self._stats_file is not None:
            self._write_stats_file(result_ir.stats)

        final_plan = None
        if result_ir.has_plan:
            try:
                final_plan = self._build_plan(result_ir.plan, compiled_problem,
                                              compilation_result)
            except Exception as e:
                self._log_error(f"Error mapping plan back: {e}")
                return PlanGenerationResult(
                    PlanGenerationResultStatus.INTERNAL_ERROR, None, self.name,
                    log_messages=[LogMessage(level=LogLevel.ERROR, message=f"Error mapping plan back: {e}")]
                )
        self._log(f"Status from planner: {status.name}")

        # Validate the plan if one was found
        if final_plan is not None and status in (
                PlanGenerationResultStatus.SOLVED_SATISFICING,
                PlanGenerationResultStatus.SOLVED_OPTIMALLY):
            plan_is_valid = self._validate_plan(original_problem, final_plan)

            if not plan_is_valid:
                validation_log = LogMessage(
                    level=LogLevel.WARNING,
                    message="Plan validation failed - the plan may not be correct"
                )
                log_messages = log_messages + [validation_log]

        return PlanGenerationResult(
            status,
            final_plan,
            self.name,
            metrics=metrics or None,
            log_messages=log_messages
        )

    # ── In-process solving ───────────────────────────────────────────────

    def _solve_in_process(self, problem: Problem,
                          timeout: Optional[float] = None,
                          output_stream: Optional[IO[str]] = None,
                          ) -> PlanGenerationResult:
        """Solve via the _rantanplan extension module. Timeouts are enforced
        by the C++ side (--timeout). When output_stream is given, the C++ log
        and the wrapper's own prints go there instead of stdout."""
        if _rantanplan is None:
            raise UPException(
                "The _rantanplan extension module is not built. Run "
                "'pip install -e . --no-build-isolation' first.")

        # Reject any problem using a feature not in supported_kind() up front,
        # rather than crashing deep in conversion. Calling an engine by name
        # bypasses UP's own supports() gate, so we enforce it here. This is an
        # allow-list: supported_kind() must list every feature RantanPlan can
        # handle, so an unlisted feature (e.g. DERIVED_FLUENTS / axioms) is
        # rejected cleanly.
        if not self.supports(problem.kind):
            unsupported = problem.kind.features - self.supported_kind().features
            return PlanGenerationResult(
                PlanGenerationResultStatus.UNSUPPORTED_PROBLEM, None, self.name,
                log_messages=[LogMessage(
                    LogLevel.ERROR,
                    "RantanPlan does not support these problem features: "
                    + ", ".join(sorted(unsupported)))])

        # The C++ log sink is installed for the duration of the call, on this
        # thread (the one that runs preprocess/solve -- the module requires
        # sink changes to happen on the solving thread).
        self._output_stream = output_stream
        if output_stream is not None:
            def _sink(level: int, text: str) -> None:
                output_stream.write(text + "\n")
            _rantanplan.set_log_sink(_sink)
        try:
            compiled_problem, compilation_result, problem_ir = \
                self._prepare_problem(problem)

            args = self._build_args(timeout)
            self._log_verbose(f"Solving in-process: {' '.join(args)}")

            try:
                preprocessed = _rantanplan.preprocess(problem_ir, args)
                result_ir = _rantanplan.solve(preprocessed)
                return self._convert_result(
                    result_ir, compiled_problem, compilation_result, problem)
            except Exception as e:
                self._log_error(f"An error occurred: {e}")
                return PlanGenerationResult(
                    PlanGenerationResultStatus.INTERNAL_ERROR, None, self.name,
                    log_messages=[LogMessage(level=LogLevel.ERROR, message=str(e))]
                )
        finally:
            if output_stream is not None:
                _rantanplan.set_log_sink(None)
            self._output_stream = None


# ═══════════════════════════════════════════════════════════════════════
# Satisficing planner (oneshot, default)
# ═══════════════════════════════════════════════════════════════════════

class RantanPlanPlanner(_RantanPlanBase, OneshotPlannerMixin):

    def __init__(self, **options):
        _RantanPlanBase.__init__(self, **options)
        OneshotPlannerMixin.__init__(self)

    def _solve(self, problem: Problem,
              heuristic: Optional[Callable] = None,
              timeout: Optional[float] = None,
              output_stream: Optional[IO[str]] = None) -> PlanGenerationResult:
        return self._solve_in_process(problem, timeout, output_stream)


# ═══════════════════════════════════════════════════════════════════════
# Optimal planner (oneshot, guarantees optimality)
# ═══════════════════════════════════════════════════════════════════════

class RantanPlanOptimalPlanner(_RantanPlanBase, OneshotPlannerMixin):

    def __init__(self, **options):
        _RantanPlanBase.__init__(self, **options)
        OneshotPlannerMixin.__init__(self)
        self._mode = "optimal"  # Force optimal mode

    @staticmethod
    def satisfies(optimality_guarantee: OptimalityGuarantee) -> bool:
        return True

    def _solve(self, problem: Problem,
              heuristic: Optional[Callable] = None,
              timeout: Optional[float] = None,
              output_stream: Optional[IO[str]] = None) -> PlanGenerationResult:
        return self._solve_in_process(problem, timeout, output_stream)


# ═══════════════════════════════════════════════════════════════════════
# Anytime planner (yields improving plans as the search finds them)
# ═══════════════════════════════════════════════════════════════════════

class RantanPlanAnytimePlanner(_RantanPlanBase, AnytimePlannerMixin):

    def __init__(self, **options):
        _RantanPlanBase.__init__(self, **options)
        AnytimePlannerMixin.__init__(self)
        self._mode = "anytime"  # Force anytime mode

    @staticmethod
    def ensures(anytime_guarantee: AnytimeGuarantee) -> bool:
        return anytime_guarantee == AnytimeGuarantee.INCREASING_QUALITY

    def _get_solutions(self, problem: Problem,
                       timeout: Optional[float] = None,
                       output_stream: Optional[IO[str]] = None,
                       ) -> Iterator[PlanGenerationResult]:
        """Run solve_anytime in a worker thread; the C++ plan callback feeds a
        queue that this generator drains. Each improving plan is yielded with
        INTERMEDIATE status; the terminal result carries the final status.

        Abandoning the generator early stops the solve: the cleanup interrupts
        the running cvc5 search (Preprocessed.interrupt()) and joins the worker.
        Only the search is interruptible -- if preprocessing or solver/encoder
        construction is still running, the cleanup waits for it to finish and
        interrupts the search as soon as it starts.

        When output_stream is given, the C++ log and the wrapper's own prints
        go there instead of stdout.
        """
        if _rantanplan is None:
            raise UPException(
                "The _rantanplan extension module is not built. Run "
                "'pip install -e . --no-build-isolation' first.")

        self._output_stream = output_stream
        try:
            compiled_problem, compilation_result, problem_ir = \
                self._prepare_problem(problem)

            args = self._build_args(timeout)
            self._log_verbose(f"Solving anytime in-process: {' '.join(args)}")

            results: queue.Queue = queue.Queue()
            # The worker publishes its Preprocessed handle here so the cleanup
            # below can interrupt() the running solve on abandonment.
            handle_box: dict = {}

            def on_plan(instances):
                results.put(("plan", instances))

            def run():
                # The C++ log sink must be set and cleared on the thread that
                # runs preprocess/solve_anytime (it is not synchronized against
                # a running solve), so it is managed here in the worker.
                if output_stream is not None:
                    def _sink(level: int, text: str) -> None:
                        output_stream.write(text + "\n")
                    _rantanplan.set_log_sink(_sink)
                try:
                    preprocessed = _rantanplan.preprocess(problem_ir, args)
                    handle_box["pre"] = preprocessed
                    result_ir = _rantanplan.solve_anytime(preprocessed, on_plan)
                    results.put(("final", result_ir))
                except Exception as e:
                    results.put(("error", e))
                finally:
                    if output_stream is not None:
                        _rantanplan.set_log_sink(None)

            worker = threading.Thread(target=run, daemon=True)
            worker.start()

            try:
                # The callback enqueues every plan before solve_anytime
                # returns, so all ("plan", ...) items precede the terminal
                # item in the queue.
                while True:
                    kind, payload = results.get()
                    if kind == "plan":
                        try:
                            plan = self._build_plan(payload, compiled_problem,
                                                    compilation_result)
                        except Exception as e:
                            self._log_error(f"Error mapping intermediate plan back: {e}")
                            continue
                        self._log(f"Improving plan found ({len(plan.actions)} actions)")
                        yield PlanGenerationResult(
                            PlanGenerationResultStatus.INTERMEDIATE, plan, self.name)
                    elif kind == "final":
                        worker.join()
                        yield self._convert_result(
                            payload, compiled_problem, compilation_result, problem)
                        return
                    else:  # error
                        worker.join()
                        self._log_error(f"An error occurred: {payload}")
                        yield PlanGenerationResult(
                            PlanGenerationResultStatus.INTERNAL_ERROR, None, self.name,
                            log_messages=[LogMessage(level=LogLevel.ERROR,
                                                     message=str(payload))])
                        return
            finally:
                # Runs on GeneratorExit (abandonment) and exceptions as well
                # as on normal return. Interrupt in a loop: the handle may not
                # exist yet (preprocessing) or the solve may not have
                # published its context yet -- keep poking until the worker
                # exits. On the normal path the worker is already joined.
                while worker.is_alive():
                    pre = handle_box.get("pre")
                    if pre is not None:
                        pre.interrupt()
                    worker.join(timeout=0.5)
        finally:
            self._output_stream = None
