#pragma once

#include "pass.hpp"

namespace rantanplan {

/**
 * @brief CNF normalization of ground conditions.
 *
 * Rewrites action preconditions, conditional-effect conditions, and goals
 * into conjunctions of clauses (AND of ORs of literals), eliminating
 * IMPLIES/IFF and pushing negations down to atoms. Runs after
 * StaticFluentPass: static-fluent folding first collapses disjunctions over
 * static predicates, so distribution operates on already-simplified
 * formulas (normalizing before folding can blow up exponentially on
 * quantifier-expanded conditions).
 *
 * Clause-level conditions give AchieversAnalysis — and the landmark/cgb
 * machinery built on it — finer condition granularity than opaque nested
 * formulas provide.
 *
 * Distribution is budget-guarded relative to the input size: a condition is
 * replaced only when its CNF stays within the clause budget; otherwise it is
 * left unchanged. Every downstream consumer handles non-CNF conditions
 * (the encoder translates arbitrary formulas, achievers treat residual ORs
 * as single semantic conditions), so bailing is always safe.
 */
class CnfNormalizationPass : public Pass {
public:
    void apply(PreprocessedProblem& result) const override;
    std::string name() const override { return "cnf-normalization"; }
};

} // namespace rantanplan
