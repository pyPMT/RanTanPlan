
#include "planners/headers/single.h"


namespace planmt {

    SinglePlanner::SinglePlanner(const Problem& problem, BaseEncoder& encoder, DiversePlannerCfg &cfg):
    DiversePlanner(problem, encoder, cfg, std::vector<DiversityDimension>())
    { }

    Plan SinglePlanner::search(void) {
        auto plans = DiversePlanner::search(1, 1.0);
        if (plans.size() == 0) { return Plan(); }
        return plans[0];
    }
}
