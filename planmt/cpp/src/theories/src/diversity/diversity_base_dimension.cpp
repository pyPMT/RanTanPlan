

#include "theories/headers/diversity/diversity_base_dimension.h"

namespace planmt {


DiversityDimension::DiversityDimension(const Problem& problem, BaseEncoder& encoder, std::string name):
problem_(problem),
encoder_(encoder),
ctx_(encoder_.get_variable_factory().get_context()),
name_(name){ }

}
