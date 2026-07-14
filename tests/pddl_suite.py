"""PDDL-file test harness.

Discovers domain/problem pairs under ``pddl/test/`` and runs the planner with
every available strategy, validating each resulting plan. Strategies are
auto-discovered from the extension module via ``_rantanplan.list_strategies()``.

Moved out of the old root ``test.py``; behaviour is unchanged. ``run()`` returns
``(passed, failed, failures)`` and prints one compact line per test — the final
summary is assembled by ``tests.cli`` so the PDDL and API suites share it.
"""
import os
from pathlib import Path

import rantanplan  # noqa: F401 — registers the UP engines (also in spawned workers)
from unified_planning.io import PDDLReader
from unified_planning.shortcuts import OneshotPlanner, PlanValidator
from unified_planning.engines.results import PlanGenerationResultStatus
from unified_planning.engines import ValidationResultStatus

from . import C

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

QUICK_TEST_DIRS = [
    "pddl/test/zenotravel",
    "pddl/test/rover",
    "pddl/test/gripper-round-1-adl",
    "pddl/test/hydropower",
    "pddl/test/sdac-simple",
    "pddl/test/sdac-zero-bound",
    "pddl/test/worksworld",
    "pddl/test/incremental-cascade",    # 2-step cascade (1 Z3 reverify in iter 2)
    "pddl/test/incremental-cascade-3",  # 3-step cascade (Z3 reverify in iter 2 AND iter 3)
]

# Strategies tested in --quick mode. Covers each major code path without
# redundant variants (eager-syntactic, eager-semantic, non-chained lazy, etc.).
QUICK_STRATEGIES = [
    "seq",        # No parallelism module (baseline)
    "r2e",        # R2E encoder, no parallelism module
    "f",          # ForallEager module (eager syntactic)
    "flsc",       # ForallLazy module (lazy semantic, chained)
    "flsc-dt",    # ForallLazy module + DoubleTail planner
    "elsc",       # ExistsCycle module (lazy semantic, chained)
    "elsc-dt",    # ExistsCycle module + DoubleTail planner
    "elsc-fp",    # ExistsCycle + FrameAxiom modules
    "cgb",        # ExistsCycle module + CGB planner (core-guided activation)
]

TIMEOUT = 60  # seconds per test

## Strategies to test with --mode optimal (replaces the old -opt registry entries)
OPTIMAL_MODE_STRATEGIES = [
    "seq", "r2e",
    "f", "f-p", "fl", "fls", "flsc",
    "e", "el", "els", "elsc",
]

QUICK_OPTIMAL_MODE_STRATEGIES = [
    "seq",
    "elsc",
]

# Soundness/completeness regression guards: specific (instance, strategies) pairs
# that exhibited a bug now fixed. They ALWAYS run (also in --quick, where the
# instance/strategy would otherwise be filtered out), so a reintroduction is
# caught in the fast dev loop. In full mode they are already part of the sweep
# and are de-duplicated. Each plan is validated like any other test.
REGRESSION_CONFIGS = [
    # Exists-step *semantic* interference must catch numeric read/write hazards;
    # fo-sailing previously produced a plan UP rejects on these encoders.
    # docs/specs/exists-step-semantic-interference-soundness.md
    ("fo-sailing", ["els", "elsc", "egs", "egsc"]),
    # Goal-relevance pruning must keep actions that establish a conditional
    # effect's guard; logistics-round-1-adl was UNSAT at every horizon before.
    # docs/specs/conditional-effect-condition-pruning.md
    ("logistics-round-1-adl", ["seq", "elsc"]),
]


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------

def get_strategies():
    """Query the extension module for available strategies."""
    from rantanplan import _rantanplan
    return list(_rantanplan.list_strategies())


def _domain_problem_in(dirpath, filenames):
    """Pick the domain/problem .pddl pair out of one directory's files."""
    domain = problem = None
    for f in filenames:
        if "domain" in f.lower() and f.endswith(".pddl"):
            domain = Path(dirpath) / f
        elif "problem" in f.lower() and f.endswith(".pddl"):
            problem = Path(dirpath) / f
    return domain, problem


def find_problems(quick=False):
    """Return sorted list of (name, domain_path, problem_path)."""
    problems = []
    search_dirs = QUICK_TEST_DIRS if quick else ["pddl/test"]
    for search_dir in search_dirs:
        for dirpath, _, filenames in os.walk(search_dir):
            domain, problem = _domain_problem_in(dirpath, filenames)
            if domain and problem:
                problems.append((Path(dirpath).name, domain, problem))
    return sorted(problems, key=lambda t: t[0])


def find_instance(name):
    """Locate a specific instance directory by name under pddl/test, regardless
    of quick-mode directory filtering. Returns (name, domain, problem) or None."""
    for dirpath, _, filenames in os.walk("pddl/test"):
        if Path(dirpath).name != name:
            continue
        domain, problem = _domain_problem_in(dirpath, filenames)
        if domain and problem:
            return (name, domain, problem)
    return None


def build_test_configs(strategies, quick=False):
    """Build list of (strategy, mode, label) tuples to test."""
    configs = []
    for s in strategies:
        configs.append((s, None, s))
    optimal_pool = QUICK_OPTIMAL_MODE_STRATEGIES if quick else OPTIMAL_MODE_STRATEGIES
    for s in optimal_pool:
        if s in strategies:
            configs.append((s, "optimal", f"{s} --mode optimal"))
    return configs


# ---------------------------------------------------------------------------
# Single test
# ---------------------------------------------------------------------------

def run_test(name, domain, problem, strategy, verbose=False, up_grounding=False, mode=None):
    """Run one test. Returns (passed: bool, detail: str)."""
    try:
        reader = PDDLReader()
        prob = reader.parse_problem(str(domain), str(problem))

        params = {
            'strategy': strategy,
            'verbosity': 'info' if verbose else 'silent',
        }
        if up_grounding:
            params['up_grounding'] = True
        if mode is not None:
            params['mode'] = mode

        engine_name = 'rantanplan-optimal' if mode == 'optimal' else 'rantanplan'
        with OneshotPlanner(name=engine_name, params=params) as planner:
            result = planner.solve(prob, timeout=TIMEOUT)

        if result.status not in (PlanGenerationResultStatus.SOLVED_SATISFICING,
                                  PlanGenerationResultStatus.SOLVED_OPTIMALLY):
            return False, result.status.name

        if result.plan is None:
            return False, "NO_PLAN"

        val = PlanValidator()
        vr = val.validate(prob, result.plan)
        if vr.status == ValidationResultStatus.VALID:
            return True, f"{len(result.plan.actions)} actions"
        else:
            return False, "INVALID_PLAN"

    except Exception as e:
        return False, str(e)[:80]


# ---------------------------------------------------------------------------
# Runners
# ---------------------------------------------------------------------------

def _run_sequential(work_items, total):
    """Run tests sequentially with live output."""
    results = {}
    for i, (test_label, name, domain, problem_file, strategy,
            verbose, up_grounding, mode) in enumerate(work_items, 1):
        print(f"  [{i}/{total}] {test_label}... ", end="", flush=True)
        ok, detail = run_test(name, domain, problem_file, strategy,
                              verbose=verbose, up_grounding=up_grounding, mode=mode)
        if ok:
            print(f"{C.GREEN}PASS{C.END} ({detail})")
        else:
            print(f"{C.RED}FAIL{C.END} ({detail})")
        results[test_label] = (ok, detail)
    return results


def _run_test_worker(args):
    """Worker for parallel execution. Unpacks args and calls run_test."""
    test_label, name, domain, problem_file, strategy, verbose, up_grounding, mode = args
    ok, detail = run_test(name, domain, problem_file, strategy,
                          verbose=verbose, up_grounding=up_grounding, mode=mode)
    return test_label, ok, detail


def _run_parallel(work_items, total, jobs):
    """Run tests in parallel using ProcessPoolExecutor."""
    from concurrent.futures import ProcessPoolExecutor, as_completed

    results = {}
    completed = 0

    with ProcessPoolExecutor(max_workers=jobs) as executor:
        futures = {executor.submit(_run_test_worker, item): item[0]
                   for item in work_items}

        for future in as_completed(futures):
            completed += 1
            test_label, ok, detail = future.result()
            results[test_label] = (ok, detail)
            status = f"{C.GREEN}PASS{C.END}" if ok else f"{C.RED}FAIL{C.END}"
            print(f"  [{completed}/{total}] {test_label}... {status} ({detail})")

    return results


# ---------------------------------------------------------------------------
# Entry point used by tests.cli
# ---------------------------------------------------------------------------

def run(quick=False, verbose=False, up_grounding=False, jobs=4):
    """Run the PDDL suite. Prints a section header and one line per test;
    returns (passed, failed, failures) where failures is [(label, detail)]."""
    all_strategies = get_strategies()
    if quick:
        strategies = [s for s in QUICK_STRATEGIES if s in all_strategies]
    else:
        strategies = all_strategies

    problems = find_problems(quick=quick)
    if not problems:
        print(f"{C.RED}No PDDL problems found.{C.END}")
        return 0, 1, [("pddl-discovery", "no problems found")]

    configs = build_test_configs(strategies, quick=quick)

    work_items = []
    seen_labels = set()
    for name, domain, problem_file in problems:
        for strategy, mode, label in configs:
            test_label = f"{name} / {label}"
            seen_labels.add(test_label)
            work_items.append((test_label, name, domain, problem_file, strategy,
                               verbose, up_grounding, mode))

    # Append the always-on regression guards, skipping any already in the sweep
    # (full mode) or whose strategy is not registered in this build.
    regression_count = 0
    for inst_name, strats in REGRESSION_CONFIGS:
        inst = find_instance(inst_name)
        if inst is None:
            continue
        name, domain, problem_file = inst
        for strategy in strats:
            if strategy not in all_strategies:
                continue
            test_label = f"{name} / {strategy}"
            if test_label in seen_labels:
                continue
            seen_labels.add(test_label)
            work_items.append((test_label, name, domain, problem_file, strategy,
                               verbose, up_grounding, None))
            regression_count += 1

    total = len(work_items)
    extra = f" (+{regression_count} regression)" if regression_count else ""
    print(f"{C.BOLD}PDDL suite{C.END}: {len(problems)} domains x {len(configs)} "
          f"configs = {total} tests{extra} (jobs: {jobs})\n")

    if jobs == 1 or verbose:
        # Sequential: verbose needs sequential for readable output.
        results = _run_sequential(work_items, total)
    else:
        results = _run_parallel(work_items, total, jobs)

    passed = sum(1 for ok, _ in results.values() if ok)
    failed = sum(1 for ok, _ in results.values() if not ok)
    failures = [(label, detail) for label, (ok, detail) in results.items() if not ok]
    return passed, failed, failures
