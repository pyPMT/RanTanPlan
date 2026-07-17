#!/usr/bin/env python3
"""Benchmark runner for the Python-generated ``benchmarks.domains`` suite.

These domains exercise RantanPlan's newer features (arrays, cardinality/``Count``,
integer-parameter/range variables) that have no PDDL syntax, so instances are
built through the UP model API rather than read from files. This is a
*benchmarking* runner: it solves and reports timings; it does not validate plans
(validation surrogates are a later step -- see docs/benchmarks-x-domains.md).

Usage:
    python scripts/benchmark_x_domains.py                      # all buildable domains, pfile1
    python scripts/benchmark_x_domains.py -d counters n_puzzle # named domains
    python scripts/benchmark_x_domains.py -d counters -n 1-10  # instance index range
    python scripts/benchmark_x_domains.py -s elsc --timeout 60
    python scripts/benchmark_x_domains.py --list              # list domains + instance counts
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import rantanplan  # noqa: F401 -- registers the "rantanplan" UP engine
from unified_planning.shortcuts import OneshotPlanner
from unified_planning.engines.results import PlanGenerationResultStatus

from benchmarks.domains import DOMAINS, require_fork

# Default to a Grounded-encoder strategy: these domains use bounded integer
# fluent types, which RantanPlan only encodes soundly with the Grounded encoder
# (chained/relaxed strategies like elsc raise on bounded numeric bounds).
STRATEGIES = ["els"]
DEFAULT_TIMEOUT = 120

G = "\033[92m"; R = "\033[91m"; Y = "\033[93m"; B = "\033[1m"; E = "\033[0m"


def parse_indices(spec):
    """Parse an instance spec like '1-10', '3', '1,4,7' into a list of ints."""
    out = []
    for part in spec.split(","):
        if "-" in part:
            lo, hi = part.split("-")
            out.extend(range(int(lo), int(hi) + 1))
        else:
            out.append(int(part))
    return out


def resolve_instances(domain, indices):
    """Map 1-based difficulty indices to instance names.

    Domains that expose a ``params(n)`` scaling function accept ``pfile<n>``
    for any ``n >= 1`` (generating on demand, including past the advertised
    range). Domains still on a fixed instance list are indexed positionally
    into ``list_instances()``. ``pfile<n>`` is tried first so the scaling
    convention wins wherever it exists.
    """
    listed = list(domain.list_instances())
    names = []
    for n in indices:
        cand = f"pfile{n}"
        if cand in listed or hasattr(domain, "params"):
            names.append(cand)
        elif 1 <= n <= len(listed):
            names.append(listed[n - 1])
    return names


def run_instance(problem, strategy, timeout):
    params = {"strategy": strategy, "verbosity": "silent"}
    try:
        with OneshotPlanner(name="rantanplan", params=params) as p:
            result = p.solve(problem, timeout=timeout)
        solved = result.status in (
            PlanGenerationResultStatus.SOLVED_SATISFICING,
            PlanGenerationResultStatus.SOLVED_OPTIMALLY,
        )
        plan_len = len(result.plan.actions) if solved and result.plan else 0
        stats = {}
        for key, val in (result.metrics or {}).items():
            try:
                stats[key] = float(val)
            except (ValueError, TypeError):
                pass
        return {"solved": solved, "plan_len": plan_len,
                "time": round(stats.get("planner.total_time", 0), 2)}
    except Exception as e:
        return {"solved": False, "plan_len": 0, "time": timeout, "error": str(e)}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-d", "--domains", nargs="+", help="domain names (default: all)")
    ap.add_argument("-n", "--instances", default="1",
                    help="instance index spec, e.g. '1', '1-10', '1,4,7' (default: 1)")
    ap.add_argument("-s", "--strategies", nargs="+", default=STRATEGIES)
    ap.add_argument("--timeout", type=int, default=DEFAULT_TIMEOUT)
    ap.add_argument("--list", action="store_true", help="list domains and exit")
    args = ap.parse_args()

    require_fork()

    if args.list:
        for name, d in sorted(DOMAINS.items()):
            try:
                n = len(list(d.list_instances()))
            except Exception as e:
                n = f"ERR: {type(e).__name__}"
            print(f"  {name:20s} {n}")
        return

    names = args.domains or sorted(DOMAINS)
    indices = parse_indices(args.instances)

    print(f"\n{B}{'Instance':32s} | {'strategy':10s} | {'time':>8s} | {'len':>5s}{E}")
    print("-" * 66)
    for dname in names:
        domain = DOMAINS[dname]
        for inst in resolve_instances(domain, indices):
            try:
                problem = domain.build_problem(inst)
            except Exception as e:
                print(f"{dname}/{inst:22s} | {R}build failed: {type(e).__name__}: "
                      f"{str(e)[:30]}{E}")
                continue
            for s in args.strategies:
                r = run_instance(problem, s, args.timeout)
                label = f"{dname}/{inst}"
                if r["solved"]:
                    t = r["time"]
                    color = G if t < 30 else (Y if t < 90 else R)
                    print(f"{label:32s} | {s:10s} | {color}{t:>7.1f}s{E} | {r['plan_len']:>5d}")
                else:
                    err = r.get("error", "TIMEOUT")
                    print(f"{label:32s} | {s:10s} | {R}{'FAIL':>8s}{E} | {str(err)[:20]}")


if __name__ == "__main__":
    main()
