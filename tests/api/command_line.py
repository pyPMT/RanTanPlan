"""The command line's reporting: when there is no plan, the reason reaches
stderr, even with --silent; in anytime mode, each improving plan is announced
once.

The command runs as `python -m rantanplan.cli` with the package under test on
PYTHONPATH (the venv's `rantanplan` script may load another checkout).
"""
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import rantanplan

PACKAGE_ROOT = Path(rantanplan.__file__).resolve().parents[1]

# Seconds the command may take: these problems are refused before any search,
# or solved at once.
COMMAND_TIMEOUT = 120

TEMPORAL = ("""(define (domain tiny-temporal)
  (:requirements :durative-actions)
  (:predicates (done))
  (:durative-action finish
    :parameters ()
    :duration (= ?duration 1)
    :condition ()
    :effect (at end (done))))""",
            "(define (problem tiny) (:domain tiny-temporal) (:init) (:goal (done)))")

ORDERED_COUNTERS = ("""(define (domain ordered-counters)
 (:requirements :strips :numeric-fluents)
 (:functions (value_x) (value_y))
 (:action inc_x :parameters () :precondition (<= (value_x) 10)
  :effect (increase (value_x) 1))
 (:action inc_y :parameters () :precondition (<= (value_y) 10)
  :effect (increase (value_y) 1)))""",
                    """(define (problem counters-ordered) (:domain ordered-counters)
 (:init (= (value_x) 0) (= (value_y) 0))
 (:goal (and (= (value_x) 3) (= (value_y) 3)))
 (:constraints (always (<= (value_x) (value_y)))))""")

# Two plans: expensive (cost 10), then prepare; finish (cost 2).
TWO_PLANS = ("""(define (domain two-plans)
 (:requirements :strips :action-costs)
 (:predicates (prepared) (done))
 (:functions (total-cost) - number)
 (:action expensive :parameters ()
  :effect (and (done) (increase (total-cost) 10)))
 (:action prepare :parameters ()
  :effect (and (prepared) (increase (total-cost) 1)))
 (:action finish :parameters () :precondition (prepared)
  :effect (and (done) (increase (total-cost) 1))))""",
             """(define (problem two-plans) (:domain two-plans)
 (:init (= (total-cost) 0))
 (:goal (done))
 (:metric minimize (total-cost)))""")


def run_cli(directory, pddl, *options):
    """Write the (domain, problem) pair and run the command on it."""
    (directory / "domain.pddl").write_text(pddl[0])
    (directory / "problem.pddl").write_text(pddl[1])
    command = [sys.executable, "-m", "rantanplan.cli",
               "-d", "domain.pddl", "-p", "problem.pddl", *options]
    env = dict(os.environ, PYTHONPATH=str(PACKAGE_ROOT))
    return subprocess.run(command, cwd=directory, env=env, capture_output=True,
                          text=True, timeout=COMMAND_TIMEOUT)


def test_the_reason_for_no_plan_reaches_stderr():
    for case, pddl, options, reason in [
        # Outside the engine's supported kind: Unified Planning's own check only
        # warned ("We cannot establish whether rantanplan can solve this
        # problem!"), and the engine's refusal gave no reason on the command line.
        ("kind", TEMPORAL, [], "does not support these problem features: CONTINUOUS_TIME"),
        # Refused by the planner, for this strategy.
        ("strategy", ORDERED_COUNTERS, ["--strategy", "r2e"],
         "does not support declared state invariants"),
    ]:
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            out = run_cli(directory, pddl, *options, "--silent")
            assert out.returncode == 1, f"{case}: {out.stdout + out.stderr}"
            assert out.stdout == "", f"{case}: {out.stdout}"
            errors = [line for line in out.stderr.splitlines() if line.startswith("Error: ")]
            assert any(reason in line for line in errors), f"{case}: {out.stderr}"
            assert "cannot establish" not in out.stderr, f"{case}: {out.stderr}"

            out = run_cli(directory, pddl, *options)
            assert "Status: PlanGenerationResultStatus.UNSUPPORTED_PROBLEM" in out.stdout, \
                f"{case}: {out.stdout}"
            assert "No plan found" in out.stdout, f"{case}: {out.stdout}"


def test_bad_options_are_usage_errors():
    """A bad option is the user's mistake: "rantanplan: error: ..." and exit
    status 2, as for argparse's own checks (--max-steps 0 was ignored). The
    parser checks most; the engine, the combinations (a double-tail strategy
    searches by length, not by cost)."""
    for options, message in [
        (["--strategy", "bogus"], "unknown strategy 'bogus'"),
        (["--max-steps", "0"], "--max-steps: must be positive"),
        (["--timeout", "-1"], "--timeout: must be non-negative"),
        (["--strategy", "fl-dt", "--mode", "optimal"], "incompatible with the double-tail"),
    ]:
        with tempfile.TemporaryDirectory() as tmp:
            out = run_cli(Path(tmp), ORDERED_COUNTERS, *options)
            assert out.returncode == 2, f"{options}: {out.stdout + out.stderr}"
            assert "rantanplan: error:" in out.stderr and message in out.stderr, \
                f"{options}: {out.stderr}"


def test_anytime_prints_each_improving_plan_once():
    """The engine printed its own "Improving plan found" line beside the
    command line's, so each improving plan was announced twice. Two plans,
    two lines, and the final plan once; nothing with --silent."""
    for options, expected in (([], 2), (["-v"], 2), (["--silent"], 0)):
        with tempfile.TemporaryDirectory() as tmp:
            out = run_cli(Path(tmp), TWO_PLANS, "--mode", "anytime", "--strategy", "seq",
                          *options)
            assert out.returncode == 0, f"{options}: {out.stdout + out.stderr}"
            lines = out.stdout.splitlines()
            improving = [line for line in lines if line.startswith("Improving plan found")]
            assert len(improving) == expected, f"{options}: {out.stdout}"
            final = [line for line in lines if line.startswith("Plan found with")]
            assert len(final) == min(expected, 1), f"{options}: {out.stdout}"


CASES = [
    test_the_reason_for_no_plan_reaches_stderr,
    test_bad_options_are_usage_errors,
    test_anytime_prints_each_improving_plan_once,
]
