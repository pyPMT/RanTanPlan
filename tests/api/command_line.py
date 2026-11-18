"""The command line's reporting: when there is no plan, the reason reaches
stderr, even with --silent.

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

# Seconds the command may take: these problems are refused before any search.
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


CASES = [
    test_the_reason_for_no_plan_reaches_stderr,
]
