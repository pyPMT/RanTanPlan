#include "array_lowering_pass.hpp"
#include "../grounding/array_lowering.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"

namespace rantanplan {

void ArrayLoweringPass::apply(PreprocessedProblem& result) const {
    if (!has_array_fluents(result.problem)) {
        return;
    }

    ScopedTimer timer;  // measuring-only; the pipeline records the stats key

    result.problem = lower_arrays(result.problem);

    // Backstop: no ARRAY_INDEX node may survive the lowering (a regression
    // here would leak an array node into passes and encoders that never
    // audit for them).
    assert_no_array_index(result.problem);

    Logger::instance().component(VerbosityLevel::INFO, "ArrayLowering", {
        {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
        {"status", "arrays lowered to scalar cells"}
    });
}

} // namespace rantanplan
