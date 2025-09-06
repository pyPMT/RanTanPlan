
#include "planners/headers/single.h"


namespace planmt {

    SinglePlanner::SinglePlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx, DiversePlannerCfg &cfg):
    DiversePlanner(problem, encoder, ctx, cfg, std::vector<DiversityDimension>())
    { }

    Plan SinglePlanner::search(void) {
        auto plans = DiversePlanner::search(1, 1.0);
        if (plans.size() == 0) { return Plan(); }
        return plans[0];
    }
}
