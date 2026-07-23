#include "forall_semantics.hpp"
#include <stdexcept>

namespace rantanplan {

Term ForallSemantics::encode_parallelism(int /*timestep*/) {
    // Every forall strategy carries a parallelism-managing propagator module
    // (LazyForallMutexModule), so the planner never asserts encoder-side
    // mutex constraints for forall semantics.
    throw std::logic_error(
        "ForallSemantics::encode_parallelism is unreachable: forall "
        "parallelism is enforced by the propagator module during search");
}

const InterferenceOracle* ForallSemantics::get_interference_analyzer() const {
    return interference_analyzer_.get();
}

} // namespace rantanplan
