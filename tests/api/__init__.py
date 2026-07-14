"""API-only feature tests — for features that have no PDDL syntax and so must be
exercised through the UP model API rather than the PDDL suite.

Expandable by convention: **one module per feature**. Each module defines

    CASES = [test_fn, test_fn, ...]

where every ``test_fn`` is a zero-argument callable that raises (e.g. via
``assert``) on failure and returns normally on success. Use the shared helpers
in ``tests.harness`` (``solve``, ``assert_valid``, ``int_args``). To add a
feature's tests, drop a new ``tests/api/<feature>.py`` with a ``CASES`` list —
``run_all()`` discovers it automatically; no wiring needed.
"""
import importlib
import pkgutil
import traceback

from .. import C


def _discover_modules():
    """Import every non-underscore module in this package, sorted by name."""
    modules = []
    for info in pkgutil.iter_modules(__path__):
        if info.name.startswith("_"):
            continue
        modules.append(importlib.import_module(f"{__name__}.{info.name}"))
    return sorted(modules, key=lambda m: m.__name__)


def run_all(verbose=False):
    """Run every CASES function across all feature modules. Prints a section
    header and one line per case; returns (passed, failed, failures) where
    failures is [(label, detail)]."""
    modules = _discover_modules()
    cases = [(m.__name__.rsplit(".", 1)[-1], fn)
             for m in modules for fn in getattr(m, "CASES", [])]
    total = len(cases)
    print(f"{C.BOLD}API-only suite{C.END}: {total} cases across "
          f"{len(modules)} feature module(s)\n")

    passed = 0
    failures = []
    for i, (feature, case) in enumerate(cases, 1):
        label = f"{feature} / {case.__name__}"
        try:
            case()
            print(f"  [{i}/{total}] {label}... {C.GREEN}PASS{C.END}")
            passed += 1
        except Exception as e:
            print(f"  [{i}/{total}] {label}... {C.RED}FAIL{C.END} ({e})")
            if verbose:
                traceback.print_exc()
            failures.append((label, str(e)[:200]))
    return passed, len(failures), failures
