#pragma once

#include "smt_types.hpp"

namespace rantanplan {

/**
 * @brief The one place cardinality constraints are built.
 *
 * cvc5 has no native at-most-k / at-least-k / pseudo-Boolean facility reachable
 * from its API (migration plan §6.3), so `z3::atmost`/`atleast`/`pble` have no
 * counterpart. These encode the constraints by hand instead.
 *
 * The encoding is the **binomial** one, chosen by measurement at M2 over an
 * Int-sum-of-ITEs alternative and over Z3's native pseudo-Boolean constraints
 * (see docs/cvc5/migration-log.md, "M2 — Results"). It beat the Int sum on every
 * instance measured -- by 11x on rlimit where the Int sum timed out -- and beat
 * *native* Z3 PB by up to 2.1x on mutex-sized groups, where pairwise clauses go
 * straight into the SAT core instead of through a PB propagator.
 *
 * Both forms are **logical equivalences with no auxiliary variables**, so the
 * result is safe to embed at any polarity. Keep it that way: an
 * equisatisfiable counter encoding (sequential counter, totalizer) would be
 * unsound under a negation, and `encode_atmost`'s result does reach negated
 * positions via `GroundedEncodingVisitor::convert_from_pool`.
 *
 * Known cost: at-most-one over a group of hundreds -- the `seq` strategy's
 * per-timestep action constraint -- is where native PB was genuinely better and
 * no clause encoding recovers it. See the M2 log; the answer is a THEORY_PLAN
 * propagator (M5–M7), not a cleverer encoding.
 */

/// At most @p k of @p lits are true. Handles k >= size (true) and k == 0.
Term encode_atmost(cvc5::TermManager& ctx, const TermVec& lits, unsigned k);

/// At least @p k of @p lits are true. Handles k == 0 (true) and k > size (false).
Term encode_atleast(cvc5::TermManager& ctx, const TermVec& lits, unsigned k);

}  // namespace rantanplan
