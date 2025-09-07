
#pragma once

#include "planners/headers/diverse.h"

namespace planmt {

class TopKPlanner : public DiversePlanner {
public:
    TopKPlanner(const Problem& problem, BaseEncoder& encoder, DiversePlannerCfg &cfg);
    std::vector<Plan> search(size_t k, float upperbound);
};
}
