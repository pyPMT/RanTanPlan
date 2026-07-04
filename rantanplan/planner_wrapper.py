from typing import Callable, IO, Iterator, Optional

import unified_planning as up
from unified_planning.shortcuts import Fraction, PlanValidator
from unified_planning.engines.results import PlanGenerationResult, LogMessage, LogLevel
from unified_planning.engines.mixins import OneshotPlannerMixin
from unified_planning.engines.mixins.oneshot_planner import OptimalityGuarantee
from unified_planning.engines.mixins.anytime_planner import AnytimePlannerMixin, AnytimeGuarantee
from unified_planning.engines import PlanGenerationResultStatus, Engine, CompilationKind, ValidationResultStatus
from unified_planning.engines.compilers import Grounder, QuantifiersRemover
from unified_planning.exceptions import UPException
from .cnf_condition_compiler import CNFConditionCompiler
from .bounded_types_simplifier import BoundedTypesSimplifier
from .converter import convert_problem
from unified_planning.model import ProblemKind, Problem
from unified_planning.plans import SequentialPlan, ActionInstance
from unified_planning.shortcuts import get_environment
from unified_planning.model.fluent import get_all_fluent_exp

import sys

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
        self._strategy = options.get('strategy', 'seq')
        self._max_steps = options.get('max_steps')
        self._persist_clauses = options.get('persist_clauses', False)
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
        self._no_local_reasoning = options.get('no_local_reasoning', False)

    # ── Verbosity helpers ────────────────────────────────────────────────

    @property
    def _is_silent(self) -> bool:
        return self._verbosity == "silent"

    @property
    def _is_verbose(self) -> bool:
        return self._verbosity in ("verbose", "debug")

    def _log(self, msg: str) -> None:
        if not self._is_silent:
            print(msg)

    def _log_verbose(self, msg: str) -> None:
        if self._is_verbose:
            print(msg)

    def _log_warning(self, msg: str) -> None:
        if not self._is_silent:
            print(msg)

    @staticmethod
    def _log_error(msg: str) -> None:
        print(msg, file=sys.stderr)

    # ── Engine metadata ──────────────────────────────────────────────────

    @property
    def name(self) -> str:
        return "RantanPlan"

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
        supported_kind.set_fluents_type('NUMERIC_FLUENTS')
        supported_kind.set_fluents_type('REAL_FLUENTS')
        supported_kind.set_fluents_type('INT_FLUENTS')
        supported_kind.set_fluents_type('OBJECT_FLUENTS')
        supported_kind.set_initial_state('UNDEFINED_INITIAL_NUMERIC')
        supported_kind.set_conditions_kind('NEGATIVE_CONDITIONS')
        supported_kind.set_conditions_kind('DISJUNCTIVE_CONDITIONS')
        supported_kind.set_conditions_kind('EQUALITIES')
        supported_kind.set_conditions_kind('EXISTENTIAL_CONDITIONS')
        supported_kind.set_conditions_kind('UNIVERSAL_CONDITIONS')
        supported_kind.set_effects_kind('CONDITIONAL_EFFECTS')
        supported_kind.set_effects_kind('INCREASE_EFFECTS')
        supported_kind.set_effects_kind('DECREASE_EFFECTS')
        supported_kind.set_effects_kind('FLUENTS_IN_NUMERIC_ASSIGNMENTS')
        supported_kind.set_effects_kind('STATIC_FLUENTS_IN_OBJECT_ASSIGNMENTS')
        supported_kind.set_effects_kind('FLUENTS_IN_OBJECT_ASSIGNMENTS')
        supported_kind.set_effects_kind('FORALL_EFFECTS')
        supported_kind.set_quality_metrics('FINAL_VALUE')
        supported_kind.set_quality_metrics('ACTIONS_COST')
        supported_kind.set_actions_cost_kind('INT_NUMBERS_IN_ACTIONS_COST')
        supported_kind.set_actions_cost_kind('REAL_NUMBERS_IN_ACTIONS_COST')
        supported_kind.set_actions_cost_kind('FLUENTS_IN_ACTIONS_COST')
        supported_kind.set_actions_cost_kind('STATIC_FLUENTS_IN_ACTIONS_COST')
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
        if self._persist_clauses:
            command.append("--persist-clauses")
        if self._symmetries:
            command.append("--symmetries")
        if self._stats_file is not None:
            command.extend(["--stats-file", self._stats_file])
        if self._smt_rpg_checker:
            command.append("--smt-rpg-checker")
        if self._no_goal_relevance:
            command.append("--no-goal-relevance")
        if self._horizon_schedule is not None:
            command.extend(["--horizon-schedule", self._horizon_schedule])
        if self._log_file is not None:
            command.extend(["--log-file", self._log_file])
        if self._up_grounding:
            command.append("--up-grounding")
        if not self._numeric_grounding:
            command.append("--no-numeric-grounding")
        if self._no_local_reasoning:
            command.append("--no-local-reasoning")
        if self._mode is not None:
            command.extend(["--mode", self._mode])
        if self._output_plan is not None:
            command.extend(["--output-plan", self._output_plan])
        if timeout:
            command.extend(["--timeout", str(int(timeout))])

        return command

    # ── Problem preparation ──────────────────────────────────────────────

    def _validate_plan(self, problem: Problem, plan: SequentialPlan) -> bool:
        try:
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

    def _initialize_fluents(self, task: Problem):
        _env = get_environment()
        _tm = _env.type_manager
        _em = _env.expression_manager
        task.initial_defaults.update({_tm.RealType():_em.Real(Fraction(0))})
        task.initial_defaults.update({_tm.IntType() :_em.Int(0)})
        task.initial_defaults.update({_tm.BoolType() :_em.Bool(False)})

        all_fluent_expressions = []
        for fluent in task.fluents:
            fluent_expressions = list(get_all_fluent_exp(task, fluent))
            all_fluent_expressions.extend(fluent_expressions)
        fluentslist = all_fluent_expressions

        initialized_fluents  = list(task.explicit_initial_values.keys())
        unintialized_fluents = list(filter(lambda x: not x in initialized_fluents, fluentslist))

        # Object fluents have no sensible default — they must be explicitly initialized.
        obj_fluents_missing_init = [fe for fe in unintialized_fluents
                                    if fe.type not in task.initial_defaults]
        if obj_fluents_missing_init:
            names = [str(f) for f in obj_fluents_missing_init]
            raise UPException(
                f"Object fluent(s) missing initial value (no default exists): {', '.join(names)}. "
                "All object fluents must be explicitly initialized in the problem's :init block.")

        real_int_fluents_to_init = []
        for fe in unintialized_fluents:
            if fe.type == _tm.RealType() or fe.type == _tm.IntType():
                real_int_fluents_to_init.append(fe)

        if real_int_fluents_to_init:
            fluent_names = [str(f) for f in real_int_fluents_to_init]
            self._log_warning(f"WARNING: Initializing {len(real_int_fluents_to_init)} real/integer fluents to 0")
            self._log_warning("         This is not semantically correct but is done in practice for planning purposes.")

        for fe in unintialized_fluents:
            task.set_initial_value(fe, task.initial_defaults[fe.type])

    def _compile_problem(self, problem: Problem):
        current_problem = problem
        compilation_maps = []

        self._log_verbose("Starting problem compilation pipeline...")

        # Step 1: Remove quantifiers if present
        quantifier_remover = QuantifiersRemover()

        if quantifier_remover.supports(current_problem.kind):
            self._log_verbose("  Applying quantifier removal...")
            quantifier_result = quantifier_remover.compile(current_problem, CompilationKind.QUANTIFIERS_REMOVING)
            current_problem = quantifier_result.problem
            compilation_maps.append(quantifier_result)
            self._log_verbose("  Quantifier removal completed.")
        else:
            self._log_verbose("  Quantifier removal not needed for this problem type.")

        # Step 2: CNF normalization of goals and preconditions
        try:
            if not self._no_cnf_normalization:
                cnf_compiler = CNFConditionCompiler()
                if cnf_compiler.supports(current_problem.kind):
                    self._log_verbose("  Applying CNF normalization (goals and preconditions)...")
                    cnf_result = cnf_compiler.compile(current_problem, CompilationKind.GROUNDING)
                    current_problem = cnf_result.problem
                    compilation_maps.append(cnf_result)
                    self._log_verbose("  CNF normalization completed.")
                else:
                    self._log_verbose("  CNF normalization not needed for this problem type.")
            else:
                self._log_verbose("  CNF normalization disabled by configuration.")
        except Exception as e:
            self._log_warning(f"  CNF normalization skipped due to error: {e}")

        # Step 3: Ground the problem (unless C++ backend will handle it)
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
        """Reject function composition (nested fluent terms) which is not supported.

        Detects patterns like (city-of (vehicle-location ?t)) where one fluent
        application appears as an argument to another fluent application.
        """
        def has_nested_fluent(node, inside_fluent=False):
            if node.is_fluent_exp():
                if inside_fluent:
                    return node
                for arg in node.args:
                    result = has_nested_fluent(arg, inside_fluent=True)
                    if result is not None:
                        return result
                return None
            for arg in node.args:
                result = has_nested_fluent(arg, inside_fluent)
                if result is not None:
                    return result
            return None

        for action in problem.actions:
            for prec in action.preconditions:
                nested = has_nested_fluent(prec)
                if nested is not None:
                    raise UPException(
                        f"Function composition (nested fluent terms) is not supported. "
                        f"Action '{action.name}' precondition contains nested fluent: {nested}. "
                        f"Rewrite using auxiliary parameters or boolean predicates.")
            for effect in action.effects:
                nested = has_nested_fluent(effect.value)
                if nested is not None:
                    raise UPException(
                        f"Function composition (nested fluent terms) is not supported. "
                        f"Action '{action.name}' effect value contains nested fluent: {nested}. "
                        f"Rewrite using auxiliary parameters or boolean predicates.")
                nested = has_nested_fluent(effect.fluent)
                if nested is not None:
                    raise UPException(
                        f"Function composition (nested fluent terms) is not supported. "
                        f"Action '{action.name}' effect fluent contains nested fluent: {nested}. "
                        f"Rewrite using auxiliary parameters or boolean predicates.")

        for goal in problem.goals:
            nested = has_nested_fluent(goal)
            if nested is not None:
                raise UPException(
                    f"Function composition (nested fluent terms) is not supported. "
                    f"Goal contains nested fluent: {nested}. "
                    f"Rewrite using auxiliary parameters or boolean predicates.")

    def _prepare_problem(self, problem: Problem):
        """Initialize fluents, compile, convert to the IR.

        Returns (compiled_problem, compilation_result, problem_ir).
        """
        self._check_no_nested_fluents(problem)

        # Strip bounded types before _initialize_fluents, which cannot handle
        # bounded int/real types in defaults or fluent parameter enumeration.
        pre_compilation_maps = []
        if BoundedTypesSimplifier._has_bounded_types(problem):
            self._log_verbose("  Stripping bounded integer/real types...")
            bounded_simplifier = BoundedTypesSimplifier()
            bounded_result = bounded_simplifier.compile(
                problem, CompilationKind.BOUNDED_TYPES_REMOVING
            )
            problem = bounded_result.problem
            pre_compilation_maps.append(bounded_result)
            self._log_verbose("  Bounded types simplified.")

        self._initialize_fluents(problem)
        compiled_problem, compilation_result = self._compile_problem(problem)

        # Prepend bounded-types maps so map_back_action_instance unwinds them last
        compilation_result.compilation_maps = pre_compilation_maps + compilation_result.compilation_maps

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

    def _convert_result(self, result_ir,
                        compiled_problem: Problem,
                        compilation_result,
                        original_problem: Problem) -> PlanGenerationResult:
        """Map a ResultIR to a UP result: rebuild action instances by name,
        map them back through the compilation steps, validate."""
        status = self._up_status(result_ir.status)
        log_messages = [LogMessage(level=self._up_log_level(msg.level), message=msg.message)
                        for msg in result_ir.log_messages]

        final_plan = None
        if result_ir.has_plan:
            try:
                action_map = {a.name: a for a in compiled_problem.actions}
                object_map = {o.name: o for o in compiled_problem.all_objects}

                new_actions: list[ActionInstance] = []
                for instance in result_ir.plan:
                    action = action_map[instance.action_name]
                    params = tuple(object_map[p] for p in instance.parameters)
                    mapped_ai = compilation_result.map_back_action_instance(
                        ActionInstance(action, params))
                    assert mapped_ai is not None
                    new_actions.append(mapped_ai)

                final_plan = SequentialPlan(new_actions)
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
            log_messages=log_messages
        )

    # ── In-process solving ───────────────────────────────────────────────

    def _solve_in_process(self, problem: Problem,
                          timeout: Optional[float] = None) -> PlanGenerationResult:
        """Solve via the _rantanplan extension module. Timeouts are enforced
        by the C++ side (--timeout)."""
        if _rantanplan is None:
            raise UPException(
                "The _rantanplan extension module is not built. Run "
                "'python build.py' first.")

        compiled_problem, compilation_result, problem_ir = \
            self._prepare_problem(problem)

        args = self._build_args(timeout)
        self._log_verbose(f"Solving in-process: {' '.join(args)}")

        try:
            preprocessed = _rantanplan.preprocess(problem_ir, args)
            result_ir = _rantanplan.solve(preprocessed, args)
            return self._convert_result(
                result_ir, compiled_problem, compilation_result, problem)
        except Exception as e:
            self._log_error(f"An error occurred: {e}")
            return PlanGenerationResult(
                PlanGenerationResultStatus.INTERNAL_ERROR, None, self.name,
                log_messages=[LogMessage(level=LogLevel.ERROR, message=str(e))]
            )


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
        return self._solve_in_process(problem, timeout)


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
        return self._solve_in_process(problem, timeout)


# ═══════════════════════════════════════════════════════════════════════
# Anytime planner (temporarily unavailable during the in-process migration)
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
        # The filesystem-polling anytime transport was deleted with the
        # subprocess path; the in-process callback transport is Phase 6 of
        # docs/specs/nanobind-in-process.md.
        raise UPException(
            "Anytime mode is temporarily unavailable during the in-process "
            "migration (restored by the solve_anytime callback, Phase 6).")
