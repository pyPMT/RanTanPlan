#pragma once

#include <string>
#include <vector>

#include "plan/plan.hpp"

class PlannerPluginBase {
public:
    virtual ~PlannerPluginBase() = default;
    virtual std::vector<planmt::Plan> search(void) = 0;
};