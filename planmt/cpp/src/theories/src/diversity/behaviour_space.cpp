
#include <z3++.h>
#include "theories/headers/diversity/behaviour_space.h"

namespace planmt {


Behaviour::Behaviour(std::vector<dimension_value_t> values) {
    dimensions_values_ = values;
}





BehaviourSpace::BehaviourSpace(const Problem& problem, BaseEncoder& encoder, std::vector<std::unique_ptr<DiversityDimension>> &dimensions):
problem_(problem),
encoder_(encoder),
ctx_(encoder_.get_variable_factory().get_context()),
dimensions_(dimensions) {

}

z3::expr BehaviourSpace::encode(size_t timestep) {
    z3::expr_vector ret_encodings(ctx_);
    if (dimensions_.size() == 0) {
        ret_encodings.push_back(ctx_.bool_val(true));
    }
    for (auto &dim : dimensions_) {
        for (auto &expr: dim->encode(timestep)) {
            ret_encodings.push_back(expr);
        }
    }
    return z3::mk_and(ret_encodings);
}


Behaviour BehaviourSpace::extract_behaviour(z3::model &model) {
    std::vector<dimension_value_t> features_values;
    for (auto &dim : dimensions_) {
        features_values.push_back(dim->extract(model));
    }
    return Behaviour(features_values);
}

}
