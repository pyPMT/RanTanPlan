

#include "theories/headers/diversity/cost_bound_dim.h"
#include "planners/headers/topq.h"


namespace planmt {

TopQPlanner::TopQPlanner(const Problem& problem, BaseEncoder& encoder, DiversePlannerCfg &cfg):
DiversePlanner(problem, encoder, cfg)
{ }

std::vector<Plan> TopQPlanner::search(size_t k, float upperbound) {
    std::vector<std::unique_ptr<DiversityDimension>> diversity_features;
    diversity_features.push_back(std::make_unique<CostBoundDimension>(problem_, encoder_, upperbound));
    return DiversePlanner::search(k, upperbound, diversity_features);
}

}
