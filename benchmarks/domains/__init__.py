"""Python-generated planning domains exercising RantanPlan's fork-only features.

Vendored from the ``domains/`` package of the x-unified-planning fork
(git@github.com:JoanEspasa/x-unified-planning.git). Only domains that use
features with **no standard-PDDL syntax** are kept here -- arrays, cardinality
(``Count``), integer-parameter/range variables, and set-typed fluents. Domains
expressible in plain numeric PDDL are deliberately excluded: they already exist
as PDDL and should be run through ``pddl/`` rather than rebuilt here.

The 8 kept domains and the fork feature each exercises:

  arrays              : n_puzzle, sokoban, rush_hour
  arrays+rangevar+int : labyrinth, pancake_sorting, puzznic
  arrays+Count+range  : plotting
  sets (SetType)      : dump_trucks

These build UP ``Problem`` objects through the model API rather than reading
PDDL. Domains that upstream parsed their instances out of PDDL files instead
define a ``params(n)`` scaling function mapping a difficulty index to instance
parameters. Instances ``pfile1``..``pfile20`` are advertised by convention, but
``params`` is defined for every ``n >= 1``, so larger indices scale the same way
and are generated on demand.

This package requires the fork -- stock unified-planning has no ``ArrayType``,
``Count`` or ``IntVariable``. ``require_fork()`` fails fast with an actionable
message rather than letting an ``ImportError`` surface from deep inside a
domain module.
"""
from .base import Domain


def require_fork():
    """Raise unless the UP fork providing arrays/Count/range variables is installed.

    Stock unified-planning from PyPI satisfies ``unified-planning>=1.2.0`` in
    pyproject.toml but lacks these features, which is a confusing failure mode
    to debug from the resulting ImportError alone.
    """
    missing = []
    try:
        from unified_planning.shortcuts import ArrayType  # noqa: F401
    except ImportError:
        missing.append("ArrayType (arrays)")
    try:
        from unified_planning.shortcuts import Count  # noqa: F401
    except ImportError:
        missing.append("Count (cardinality)")
    try:
        from unified_planning.model.int_variable import IntVariable  # noqa: F401
    except ImportError:
        missing.append("IntVariable (range variables)")

    if missing:
        import unified_planning
        raise RuntimeError(
            "benchmarks.domains requires the x-unified-planning fork; the "
            f"installed unified_planning {unified_planning.__version__} at "
            f"{unified_planning.__file__} is missing: {', '.join(missing)}.\n"
            "Install it with:\n"
            "  pip install -e /path/to/x-unified-planning\n"
            "(fork: git@github.com:JoanEspasa/x-unified-planning.git)"
        )


from .dump_trucks import DOMAIN as dump_trucks
from .labyrinth import DOMAIN as labyrinth
from .n_puzzle import DOMAIN as n_puzzle
from .pancake_sorting import DOMAIN as pancake_sorting
from .plotting import DOMAIN as plotting
from .puzznic import DOMAIN as puzznic
from .rush_hour import DOMAIN as rush_hour
from .sokoban import DOMAIN as sokoban

DOMAINS: dict[str, Domain] = {
    "dump_trucks": dump_trucks,
    "labyrinth": labyrinth,
    "n_puzzle": n_puzzle,
    "pancake_sorting": pancake_sorting,
    "plotting": plotting,
    "puzznic": puzznic,
    "rush_hour": rush_hour,
    "sokoban": sokoban,
}

__all__ = ["Domain", "DOMAINS", "require_fork"]
