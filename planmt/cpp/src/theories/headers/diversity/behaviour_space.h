#pragma once

#include <vector>
#include "diversity_base_dimension.h"
#include "encoders/headers/base_encoder.h"

namespace planmt {

class BehaviourSpace {
public:
    BehaviourSpace(const Problem& problem, BaseEncoder& encoder, z3::context& ctx, std::vector<DiversityDimension> dimensions);
};

}
