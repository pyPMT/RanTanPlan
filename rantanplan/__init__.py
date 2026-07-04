import unified_planning as up
from .planner_wrapper import RantanPlanPlanner, RantanPlanOptimalPlanner, RantanPlanAnytimePlanner

# Register the planners to the UP framework
# This is done once the package is imported so its transparent to the user.
env = up.environment.get_environment()
env.factory.add_engine('rantanplan', 'rantanplan.planner_wrapper', 'RantanPlanPlanner')
env.factory.add_engine('rantanplan-optimal', 'rantanplan.planner_wrapper', 'RantanPlanOptimalPlanner')
env.factory.add_engine('rantanplan-anytime', 'rantanplan.planner_wrapper', 'RantanPlanAnytimePlanner')

# Fail fast on a libz3 mismatch: the extension module must link the exact libz3
# the z3 package loads (two copies in one process is undefined behavior).
# Compare major.minor.build only -- the module reports a 4th revision component.
# If the module is missing entirely, planner_wrapper's UPException covers it.
from .planner_wrapper import _rantanplan as _ext_module
if _ext_module is not None:
    import z3 as _z3
    _built = _ext_module.z3_version().split('.')[:3]
    _installed = _z3.get_version_string().split('.')[:3]
    if _built != _installed:
        raise ImportError(
            f"The _rantanplan extension module was built against Z3 "
            f"{'.'.join(_built)} but the installed z3-solver package is "
            f"{'.'.join(_installed)}. Rebuild it against the installed Z3: "
            f"pip install -e . --no-build-isolation")
