
#pragma once

#include "planners/headers/diverse.h"

namespace planmt {

class TopKPlanner : public DiversePlanner {
public:
    TopKPlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx, DiversePlannerCfg &cfg);
    std::vector<Plan> search(size_t k, float upperbound);
};



}

