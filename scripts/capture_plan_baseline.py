#!/usr/bin/env python3
"""Capture plan *identity* (not just validity) for deterministic strategies.

Plan validity is already checked by ``test.py``; it catches soundness
regressions. Completeness regressions — a longer horizon, a missed plan — are
invisible to validation. This script writes a diffable transcript so a later
refactor that silently changes the search is caught by ``diff``.

Used by the Z3 -> cvc5 migration (``docs/cvc5/migration-plan.md`` §8, "The pass
bar"): capture at M0, re-capture after every pure-refactor milestone, diff.

**What is and is not stable.** RantanPlan is deterministic *within a process*
(the same problem solved four times in one process gives byte-identical plans),
but ``unified_planning``'s ``PDDLReader`` returns actions, fluents and objects
in an order that varies between processes — not a hash-seed effect, since
``PYTHONHASHSEED=0`` does not fix it. That order reaches the IR unchanged, so on
a *symmetric* domain the solver picks a different member of a symmetry class
each run. So:

- the **SIGNATURE** block (status, horizon, action count) is stable and is the
  hard diff target;
- the per-action listing below it is informational — diff it to understand a
  signature change, not as a pass/fail gate.

Usage:
    python scripts/capture_plan_baseline.py -o docs/specs/baselines/cvc5-m0-plans.txt
    python scripts/capture_plan_baseline.py -s seq            # one strategy

    # the actual regression gate:
    diff <(sed -n '/SIGNATURE/,/END SIGNATURE/p' old.txt) \\
         <(sed -n '/SIGNATURE/,/END SIGNATURE/p' new.txt)
"""
import argparse
import sys

from unified_planning.io import PDDLReader
from unified_planning.shortcuts import OneshotPlanner
from unified_planning.engines.results import PlanGenerationResultStatus

import rantanplan  # noqa: F401 — registers the UP engines
from tests.pddl_suite import find_problems

# Deterministic strategies only. `pdla`-flavoured search is documented as
# non-deterministic (docs/specs/packaging-and-library.md), so its plans are not
# a meaningful diff target.
DEFAULT_STRATEGIES = ["seq", "elsc"]

TIMEOUT = 60


def capture(name, domain, problem, strategy):
    """Run one (instance, strategy) pair.

    Returns ``(signature_line, detail_lines)``.
    """
    reader = PDDLReader()
    prob = reader.parse_problem(str(domain), str(problem))

    with OneshotPlanner(name="rantanplan",
                        params={"strategy": strategy, "verbosity": "silent"}) as planner:
        result = planner.solve(prob, timeout=TIMEOUT)

    header = f"## {name} / {strategy}"
    label = f"{name:28s} {strategy:10s}"

    if result.status not in (PlanGenerationResultStatus.SOLVED_SATISFICING,
                             PlanGenerationResultStatus.SOLVED_OPTIMALLY):
        return (f"{label} {result.status.name}",
                [header, f"   status: {result.status.name}", ""])
    if result.plan is None:
        return (f"{label} NO_PLAN", [header, "   status: NO_PLAN", ""])

    horizon = (result.metrics or {}).get("planner.solution_horizon", "?")
    n = len(result.plan.actions)
    signature = (f"{label} {result.status.name:20s} "
                 f"horizon={horizon:<6} actions={n}")
    detail = [header,
              f"   status: {result.status.name}",
              f"   horizon: {horizon}",
              f"   actions: {n}"]
    detail += [f"   {i:3d}: {a}" for i, a in enumerate(result.plan.actions)]
    detail.append("")
    return signature, detail


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-s", "--strategy", action="append",
                    help=f"Strategy (repeatable; default: {' '.join(DEFAULT_STRATEGIES)})")
    ap.add_argument("-o", "--output", default=None,
                    help="Write transcript here instead of stdout")
    ap.add_argument("--full", action="store_true",
                    help="All pddl/test domains instead of the --quick subset")
    args = ap.parse_args()

    strategies = args.strategy or DEFAULT_STRATEGIES
    problems = find_problems(quick=not args.full)

    signatures, details = [], []
    for name, domain, problem in problems:
        for strategy in strategies:
            print(f"  {name} / {strategy}...", file=sys.stderr, flush=True)
            try:
                sig, det = capture(name, domain, problem, strategy)
            except Exception as e:
                sig = f"{name:28s} {strategy:10s} ERROR"
                det = [f"## {name} / {strategy}", f"   error: {e}", ""]
            signatures.append(sig)
            details += det

    text = "\n".join(
        [f"# Plan identity transcript",
         f"# strategies: {', '.join(strategies)}",
         f"# instances:  {len(problems)} ({'full' if args.full else 'quick'} set)",
         "",
         "# ---- SIGNATURE (stable; this is the regression gate) ----"]
        + signatures
        + ["# ---- END SIGNATURE ----",
           "",
           "# Below: full plans. Informational only -- the action sequence is",
           "# NOT stable across processes on symmetric domains, because",
           "# unified_planning's PDDLReader orders actions/fluents/objects",
           "# differently in each process. See this file's module docstring.",
           ""]
        + details)
    if args.output:
        with open(args.output, "w") as f:
            f.write(text)
        print(f"\nTranscript written to {args.output}", file=sys.stderr)
    else:
        print(text)


if __name__ == "__main__":
    main()
