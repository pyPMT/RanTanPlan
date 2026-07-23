import unified_planning as up
from .planner_wrapper import RantanPlanPlanner, RantanPlanOptimalPlanner, RantanPlanAnytimePlanner

# Register the planners to the UP framework
# This is done once the package is imported so its transparent to the user.
env = up.environment.get_environment()
env.factory.add_engine('rantanplan', 'rantanplan.planner_wrapper', 'RantanPlanPlanner')
env.factory.add_engine('rantanplan-optimal', 'rantanplan.planner_wrapper', 'RantanPlanOptimalPlanner')
env.factory.add_engine('rantanplan-anytime', 'rantanplan.planner_wrapper', 'RantanPlanAnytimePlanner')

# No libz3 guard any more: the extension links the cvc5 fork's libcvc5 directly
# out of its build tree (CMakeLists CVC5_BUILD_DIR), not a pip package, so there
# is no second copy to disagree with. If the module is missing entirely,
# planner_wrapper's UPException covers it.
