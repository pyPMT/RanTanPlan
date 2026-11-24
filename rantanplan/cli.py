#!/usr/bin/env python3
"""
RantanPlan CLI - Command Line Interface for the RantanPlan planning system.

This CLI provides an interface to the RantanPlan planner, which uses a planning-as-SMT
approach with a C++ backend and Unified Planning frontend.
"""

import argparse
import os
import sys
from importlib import metadata
from typing import Any, Dict
from unified_planning.engines.results import LogLevel


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        prog="rantanplan",
        description="RantanPlan - A planning-as-SMT planner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s -d domain.pddl -p problem.pddl --strategy seq
  %(prog)s -d domain.pddl -p problem.pddl --strategy fl --timeout 60
  %(prog)s --list-strategies

Use --list-strategies to see available strategies and their descriptions.
        """
    )

    # Required arguments
    parser.add_argument("-d", "--domain", type=str,
                       help="Path to the PDDL domain file")
    parser.add_argument("-p", "--problem", type=str,
                       help="Path to the PDDL problem file")

    # List strategies
    parser.add_argument("--list-strategies", action="store_true",
                       help="List available strategies and exit")

    add_planner_arguments(parser)

    parser.add_argument(
        "--version",
        action="version",
        version=f"RantanPlan {metadata.version('rantanplan')}"
    )

    args = parser.parse_args()
    check_planner_arguments(parser, args)
    return args


def _non_negative_int(text):
    value = int(text)
    if value < 0:
        raise argparse.ArgumentTypeError(f"must be non-negative, not {value}")
    return value


def _positive_int(text):
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError(f"must be positive, not {value}")
    return value


def check_planner_arguments(parser, args):
    """Reject a strategy this build does not register, through parser.error
    like argparse's own checks ("rantanplan: error: ..." on stderr, exit
    status 2): a bad option is the user's mistake, not the problem's."""
    try:
        from rantanplan import _rantanplan
    except ImportError:
        return  # solve_problem reports the missing module
    strategies = _rantanplan.list_strategies()
    if args.strategy not in strategies:
        parser.error(f"argument --strategy: unknown strategy '{args.strategy}' "
                     f"(choose from {', '.join(strategies)})")


def add_planner_arguments(parser):
    """Add the options that configure the planner and its output.

    Shared with other commands that solve a problem but build it in Python
    instead of reading PDDL files.
    """
    # Strategy selection
    parser.add_argument("--strategy", type=str, default="e-lin",
                       help="Planning strategy to use (default: e-lin)")
    parser.add_argument("--mode",
                       choices=["satisficing", "optimal", "anytime"],
                       default=None,
                       help="Search mode: satisficing (first plan, default), "
                            "optimal (cost-optimal with proof), "
                            "anytime (keep improving, report intermediate plans)")

    # Optional global parameters
    parser.add_argument("--timeout", type=_non_negative_int, default=None,
                       help="Timeout in seconds (default: None)")
    parser.add_argument("--max-steps", type=_positive_int,
                       help="Maximum planning steps")
    parser.add_argument("--output-plan", type=str, default="plan.txt",
                       help="Base path for plan output in IPC format (files written as <path>.1, .2, etc.; "
                            "default: plan.txt). Used from Unified Planning, the planner writes no files "
                            "unless the output_plan parameter is set.")
    parser.add_argument("--stats-file", type=str,
                       help="Save statistics to file")
    
    # Verbosity
    parser.add_argument("-v", "--verbose", action="count", default=0,
                       help="Increase verbosity (-v: verbose, -vv: debug)")
    parser.add_argument("--silent", action="store_true",
                       help="Suppress all output")
    
    # Additional flags that work with all strategies
    parser.add_argument("--persist-clauses", action="store_true",
                       help="Enable Z3 clause persistence for user propagator lemmas (default)")
    parser.add_argument("--no-persist-clauses", action="store_true",
                       help="Discard user propagator lemmas on backtracking")
    parser.add_argument("--symmetries", action="store_true",
                       help="Enable symmetry detection and breaking")
    parser.add_argument("--no-cnf-normalization", action="store_true",
                       help="Disable CNF normalization of goals and preconditions")
    parser.add_argument("--smt-rpg-checker", action="store_true",
                       help="Use Z3 SMT solver for RPG applicability checks instead of interval arithmetic (default: interval)")
    parser.add_argument("--no-goal-relevance", action="store_true",
                       help="Disable semantic goal-relevance analysis (enabled by default)")
    parser.add_argument("--full-cycle-search", action="store_true",
                       help="Exists-step propagator: search for a cycle from every active action, "
                            "not only from the one just fixed")
    parser.add_argument("--plain-chains", action="store_true",
                       help="elsc: Rintanen's chains over the plain Boolean units, written up front; "
                            "the propagator checks the rest")

    # Logging
    parser.add_argument("--log-file", type=str,
                       help="Write solver decision log to file (for use with plot.py)")

    # Horizon schedule
    parser.add_argument("--horizon-schedule",
                       choices=["linear", "arithmetic", "geometric", "doubling"],
                       default=None,
                       help="Horizon growth schedule (default: linear / one-by-one). "
                            "'doubling' doubles each batch, 'geometric' multiplies by 1.5, "
                            "'arithmetic' adds 5 each batch.")

    parser.add_argument('--up-grounding', action='store_true',
                        help='Use Unified Planning grounding instead of '
                             'the default reachability-based grounding')
    parser.add_argument('--no-numeric-grounding', action='store_true',
                        help='Disable numeric interval bounds in reachability '
                             'grounder (enabled by default)')








def _build_planner_params(args) -> Dict[str, object]:
    """Build planner parameters dictionary from CLI arguments."""
    planner_params: Dict[str, object] = {
        'strategy': args.strategy,
    }

    if args.max_steps is not None:
        planner_params['max_steps'] = args.max_steps
    if args.stats_file:
        planner_params['stats_file'] = args.stats_file
    if args.no_persist_clauses:
        planner_params['persist_clauses'] = False
    elif args.persist_clauses:
        planner_params['persist_clauses'] = True
    if args.symmetries:
        planner_params['symmetries'] = True
    if args.no_cnf_normalization:
        planner_params['no_cnf_normalization'] = True
    if args.smt_rpg_checker:
        planner_params['smt_rpg_checker'] = True
    if args.no_goal_relevance:
        planner_params['no_goal_relevance'] = True
    if args.full_cycle_search:
        planner_params['full_cycle_search'] = True
    if args.plain_chains:
        planner_params['plain_chains'] = True
    if args.horizon_schedule is not None:
        planner_params['horizon_schedule'] = args.horizon_schedule
    if args.log_file:
        planner_params['log_file'] = args.log_file
    if args.up_grounding:
        planner_params['up_grounding'] = True
    if args.no_numeric_grounding:
        planner_params['numeric_grounding'] = False
    if args.mode is not None:
        planner_params['mode'] = args.mode
    if args.output_plan:
        planner_params['output_plan'] = args.output_plan
    # Handle verbosity
    if args.silent:
        planner_params['verbosity'] = "silent"
    elif args.verbose == 1:
        planner_params['verbosity'] = "verbose"
    elif args.verbose >= 2:
        planner_params['verbosity'] = "debug"

    return planner_params


def solve_problem(problem, args):
    """Solve problem using selected strategy and mode."""
    try:
        from unified_planning.shortcuts import OneshotPlanner, AnytimePlanner
        from unified_planning.engines import PlanGenerationResultStatus
        from unified_planning.exceptions import UPUsageError
        import rantanplan
    except ImportError as e:
        print(f"Error: Failed to import dependencies: {e}")
        return None

    planner_params = _build_planner_params(args)

    # Show configuration info
    if not args.silent and args.verbose >= 1:
        print(f"Strategy: {args.strategy}")

    try:
        PlannerClass: Any = OneshotPlanner

        # skip_checks: Unified Planning's own check of the problem kind, made
        # when an engine is picked by name, only warns ("We cannot establish
        # whether rantanplan can solve this problem!"). The engine checks the
        # kind itself, for its search mode, and refuses with
        # UNSUPPORTED_PROBLEM and the features it lacks, which
        # report_result prints.
        if args.mode == "anytime":
            with AnytimePlanner(name='rantanplan-anytime', params=planner_params) as planner:
                planner.skip_checks = True
                last_result = None
                for result in planner.get_solutions(problem, timeout=args.timeout):
                    if result.status == PlanGenerationResultStatus.INTERMEDIATE:
                        if not args.silent and result.plan:
                            print(f"Improving plan found ({len(result.plan.actions)} actions)")
                    last_result = result
                return last_result
        elif args.mode == "optimal":
            with PlannerClass(name='rantanplan-optimal', params=planner_params) as planner:
                planner.skip_checks = True
                result = planner.solve(problem, timeout=args.timeout)
                return result
        else:
            # satisficing (default)
            with PlannerClass(name='rantanplan', params=planner_params) as planner:
                planner.skip_checks = True
                result = planner.solve(problem, timeout=args.timeout)
                return result
    except UPUsageError as e:
        # An option the engine refuses (the parser checks most of them).
        print(f"rantanplan: error: {e}", file=sys.stderr)
        sys.exit(2)
    except Exception as e:
        print(f"Error during planning: {e}")
        return None



def report_result(result, args) -> int:
    """Print the status and the plan (unless --silent), and return the exit
    code: 0 if there is a plan, 1 otherwise.

    Without a plan, the result's ERROR log messages (why the problem was
    refused, or the internal error) go to stderr as "Error: ...", even with
    --silent, and its WARNING messages as "Warning: ...", unless --silent.
    The "Status:" and "No plan found" lines stay as they are: scripts read
    them."""
    if not result:
        return 1

    # Print result (unless --silent)
    if not args.silent:
        print(f"Status: {result.status}")
    if not result.plan:
        for message in result.log_messages or []:
            if message.level == LogLevel.ERROR:
                print(f"Error: {message.message}", file=sys.stderr)
            elif message.level == LogLevel.WARNING and not args.silent:
                print(f"Warning: {message.message}", file=sys.stderr)
    if result.plan and hasattr(result.plan, 'actions'):
        if not args.silent:
            print(f"Plan found with {len(result.plan.actions)} actions")
            for i, action in enumerate(result.plan.actions):
                params = ", ".join(map(str, action.actual_parameters))
                print(f"  {i+1}. {action.action.name}({params})")

        # Plan files are written by the C++ backend in IPC format
        return 0
    if not args.silent:
        print("No plan found")
    return 1




def main():
    """Main entry point."""
    args = parse_arguments()

    # Handle --list-strategies
    if args.list_strategies:
        try:
            from rantanplan import _rantanplan
            print("Available strategies:")
            for name in _rantanplan.list_strategies():
                print(f"  {name}")
            sys.exit(0)
        except Exception as e:
            print(f"Error listing strategies: {e}")
            sys.exit(1)

    # Validate required arguments for solving
    if not args.domain or not args.problem:
        print("Error: --domain and --problem are required (or use --list-strategies)")
        sys.exit(1)

    # Validate files
    for file_path in [args.domain, args.problem]:
        if not os.path.exists(file_path):
            print(f"Error: File not found: {file_path}")
            sys.exit(1)

    # Parse problem with Unified Planning's own reader. UP's default reader
    # parses with the `pddl` package when a domain allows it, and that package
    # keeps actions, objects and initial facts in sets whose order changes
    # from one run to the next. The planner numbers everything in the order
    # it receives and its encodings depend on that numbering (R2E's action
    # order, for one), so the same problem gave different horizons and plans
    # in different runs. UP's own reader gives the same order every time,
    # although it parses some domains more slowly.
    try:
        from unified_planning.io import PDDLReader
        reader = PDDLReader(force_up_pddl_reader=True)
        problem = reader.parse_problem(args.domain, args.problem)
    except Exception as e:
        print(f"Error parsing PDDL files: {e}")
        sys.exit(1)

    # Solve
    result = solve_problem(problem, args)
    sys.exit(report_result(result, args))


if __name__ == "__main__":
    main()