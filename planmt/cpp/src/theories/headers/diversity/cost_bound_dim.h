
#pragma once

#include <string>
#include <vector>
#include <z3++.h>
#include "problem/headers/problem.h"
#include "encoders/headers/base_encoder.h"
#include "diversity_base_dimension.h"

namespace planmt {

class CostBoundDimension : public DiversityDimension {
public:
    CostBoundDimension(const Problem& problem, BaseEncoder& encoder, float cost_bound_factor);
    std::vector<z3::expr> encode(size_t timestep) override;
    std::unique_ptr<dimension_value_t> extract(const z3::model & model) override;

private:
    float cost_bound_factor_;
    z3::context &ctx_;
//    std::vector<z3::expr> cost_bound_vars_;
    z3::expr_vector cost_bound_vars_;
    z3::expr accumlated_cost_literal_;
    
    std::vector<z3::expr> sequnetial_step_encode(size_t timestep);
    std::vector<z3::expr> parallel_step_encode(size_t timestep);
    
};



}
