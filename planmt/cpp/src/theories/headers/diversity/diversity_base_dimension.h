#pragma once

#include "problem/headers/problem.h"
#include "encoders/headers/base_encoder.h"

namespace planmt {

class DiversityDimension {
public:
    DiversityDimension(const Problem& problem, BaseEncoder& encoder, z3::context& ctx);
    DiversityDimension(); // A default constructor.

private:
//    const Problem& problem_;
//    BaseEncoder& encoder_;
//    z3::context& ctx_;
};



}
