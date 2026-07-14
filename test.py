#!/usr/bin/env python3
"""Entry point for the RantanPlan test suite.

The suite now lives in the ``tests/`` package (``tests.pddl_suite`` for the
PDDL-file harness and ``tests.api`` for API-only feature tests). This shim is
kept at the repo root so the documented command still works:

    python test.py            # all strategies, all PDDL domains, + API tests
    python test.py --quick    # fast subset of PDDL domains, + API tests
    python test.py --api-only # only the API-only feature tests

Equivalent to ``python -m tests``.
"""
import os
import sys

if __name__ == "__main__":
    sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
    from tests.cli import main
    main()
