
#include "planners/headers/single.h"


namespace planmt {

    SinglePlanner::SinglePlanner(const Problem& problem, BaseEncoder& encoder, DiversePlannerCfg &cfg):
    DiversePlanner(problem, encoder, cfg)
    { }

    Plan SinglePlanner::search(void) {
        std::vector<std::unique_ptr<DiversityDimension>> dummy;
        auto plans = DiversePlanner::search(1, 1.0, dummy);
        if (plans.size() == 0) { return Plan(); }
        return plans[0];
    }
}
