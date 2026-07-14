"""Test-suite entry point: runs the PDDL suite and the API-only suite, then
prints one combined summary and exits non-zero if anything failed.

Invoked by the root ``test.py`` shim (or ``python -m tests``).
"""
import argparse
import sys

from . import C, pddl_suite, api


def main():
    parser = argparse.ArgumentParser(description="RantanPlan test suite")
    parser.add_argument("--quick", action="store_true",
                        help="Test only a small subset of PDDL domains")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Show planner output / failure tracebacks")
    parser.add_argument("--up-grounding", action="store_true",
                        help="Use UP cross-product grounding instead of C++ grounding")
    parser.add_argument("-j", "--jobs", type=int, default=4,
                        help="Number of parallel jobs for the PDDL suite (1 = sequential)")
    parser.add_argument("--pddl-only", action="store_true",
                        help="Run only the PDDL suite")
    parser.add_argument("--api-only", action="store_true",
                        help="Run only the API-only feature suite")
    args = parser.parse_args()

    if args.pddl_only and args.api_only:
        parser.error("--pddl-only and --api-only are mutually exclusive")

    print(f"\n{C.BOLD}RantanPlan Test Suite{C.END}\n")

    pddl = (0, 0, [])
    apis = (0, 0, [])

    if not args.api_only:
        pddl = pddl_suite.run(quick=args.quick, verbose=args.verbose,
                              up_grounding=args.up_grounding, jobs=args.jobs)
    if not args.pddl_only:
        if not args.api_only:
            print()
        apis = api.run_all(verbose=args.verbose)

    p_pass, p_fail, p_failures = pddl
    a_pass, a_fail, a_failures = apis
    passed = p_pass + a_pass
    failed = p_fail + a_fail
    failures = p_failures + a_failures

    print(f"\n{'=' * 60}")
    print(f"{C.BOLD}Test Summary{C.END}")
    print(f"{'=' * 60}")
    if not args.api_only:
        print(f"  PDDL:   {C.GREEN}{p_pass} passed{C.END}, {C.RED}{p_fail} failed{C.END}")
    if not args.pddl_only:
        print(f"  API:    {C.GREEN}{a_pass} passed{C.END}, {C.RED}{a_fail} failed{C.END}")
    print(f"  Total:  {passed + failed}  "
          f"({C.GREEN}{passed} passed{C.END}, {C.RED}{failed} failed{C.END})")

    if failures:
        print(f"\n{C.RED}Failed tests:{C.END}")
        for label, detail in failures:
            print(f"  {label}: {detail}")

    sys.exit(1 if failed else 0)
