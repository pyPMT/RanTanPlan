
#include <xplugin/xfactory.hpp>

#include "plugin_bases/planner_plugin_base.hpp"

class SinglePlanner : public PlannerPluginBase {
public:

    SinglePlanner(const std::string & args) {
        parse_args(args);
    }

    void parse_args(const std::string &args) {
        // No arguments to parse for this simple planner
    }
    
    std::vector<planmt::Plan> search(void) override {
        // Return a single empty plan as a placeholder
        return {planmt::Plan()};
    }
};

using factory_type = xp::xfactory<SinglePlanner, PlannerPluginBase, const std::string &>;
XPLUGIN_CREATE_XPLUGIN_FACTORY(factory_type);