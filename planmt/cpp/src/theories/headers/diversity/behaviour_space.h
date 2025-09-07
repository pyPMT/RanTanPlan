#pragma once

#include <vector>

#include "diversity_base_dimension.h"
#include "encoders/headers/base_encoder.h"

namespace planmt {

using DiversityValue = std::variant<int, double, std::string, z3::expr>;

class Behaviour {
public:
    Behaviour(std::vector<dimension_value_t> values);
    Behaviour() = default;
    std::vector<dimension_value_t> get_values(void) { return dimensions_values_; }
private:
    std::vector<dimension_value_t> dimensions_values_;
};


class BehaviourSpace {
public:
    BehaviourSpace(const Problem& problem, BaseEncoder& encoder, std::vector<std::unique_ptr<DiversityDimension>> &dimensions);
    
    // Produce the diversity features encodings
    z3::expr encode(size_t timestep);
    
    // Extracts the plan's behaviour from the model.
    Behaviour extract_behaviour(z3::model &model);

private:
    const Problem& problem_;
    BaseEncoder& encoder_;
    z3::context& ctx_;
    std::vector<std::unique_ptr<DiversityDimension>> &dimensions_;
    
};

}
