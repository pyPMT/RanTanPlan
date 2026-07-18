#pragma once

#include "pass.hpp"

namespace rantanplan {

/**
 * @brief Lower array fluents to scalar cells (Phase 1, constant indices).
 *
 * Runs after `QuantifierExpansionPass` (by then every array index is ground
 * constant arithmetic) and before `GroundingPass` (cells must exist as
 * ordinary scalar fluents before the reachability fixpoint enumerates facts).
 * A no-op when the problem declares no array fluents, so the non-array path
 * is untouched.
 *
 * Thin pipeline wrapper over grounding/array_lowering.{hpp,cpp}; the
 * implementation stays co-located with the grounder because Phase 2
 * (symbolic indices) interleaves the lowering with the reachability fixpoint
 * itself (symbolic accesses must be treated conservatively *during*
 * grounding). Ends by asserting that no ARRAY_INDEX node survives, so later
 * work cannot silently regress the "arrays are fully lowered" invariant.
 *
 * Note: `api::preprocess` rejects array problems when native grounding is
 * disabled (--up-grounding) before any pass runs — cells minted here have
 * only been validated through the native grounder.
 */
class ArrayLoweringPass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "array-lowering"; }
};

} // namespace rantanplan
