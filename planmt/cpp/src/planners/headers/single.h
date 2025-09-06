#pragma once

#include "planners/headers/diverse.h"

namespace planmt {

class SinglePlanner : public DiversePlanner {
public:
    SinglePlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx, DiversePlannerCfg &cfg);
    Plan search(void);
};



}
