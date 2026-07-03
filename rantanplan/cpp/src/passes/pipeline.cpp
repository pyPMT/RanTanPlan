#include "pipeline.hpp"
#include "../util/scoped_timer.hpp"

namespace rantanplan {

PreprocessedProblem run_pipeline(PreprocessedProblem initial_result, const std::vector<const Pass*>& passes) {
    PreprocessedProblem result = std::move(initial_result);

    for (const auto* pass : passes) {
        ScopedTimer timer("pass." + pass->name() + ".time_ms");

        pass->apply(result);

        if (result.proven_unsolvable) {
            break;
        }
    }

    return result;
}

} // namespace rantanplan
