#include "cardinality_encoding.hpp"

#include "term_builder.hpp"
#include "../util/stats.hpp"

namespace rantanplan {

namespace {

// The TermBuilder is built from the TermManager on the spot: cvc5 has no
// operator overloading, so every connective and every arithmetic node below
// goes through it. It carries no problem pointer because these terms are a
// self-contained Int sum, never mixed with the problem's numeric sort.

Term int_sum(TermBuilder& tb, const TermVec& lits) {
    Term one = tb.int_val(1);
    Term zero = tb.int_val(0);
    Term sum = zero;
    for (size_t i = 0; i < lits.size(); ++i) {
        sum = tb.add(sum, tb.mk_ite(lits[i], one, zero));
    }
    return sum;
}

// At most one: the pairwise clauses (¬bᵢ ∨ ¬bⱼ). n(n-1)/2 of them.
Term pairwise_atmost_one(TermBuilder& tb, const TermVec& lits) {
    const size_t n = lits.size();
    TermVec clauses;
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            clauses.push_back(tb.mk_or({tb.mk_not(lits[i]), tb.mk_not(lits[j])}));
        }
    }
    return tb.mk_and(clauses);
}

// Why the encoding switches to an Int sum above k == 1.
//
// The binomial encoding of at-most-k is the AND over every (k+1)-subset of that
// subset's negations -- C(n, k+1) clauses. That is pairwise at k == 1 and
// combinatorially hopeless beyond it: `SetLoweringPass` compiles
// `cardinality(S)` into a COUNT over the whole element universe
// (passes/set_lowering_pass.cpp:207), so `(<= (cardinality s) 5)` over 30
// objects would be C(30,6) ≈ 594k clauses. k > 1 is also exactly where an Int
// sum is the natural reading of the constraint, and it arises *only* at the
// COUNT site -- mutex groups and the sequential at-most-one are k == 1 by
// construction. So the encoding splits there. This is a property of the two
// encodings, not a safety valve.
constexpr const char* ABOVE_ONE_STAT = "encoder.card_intsum_above_k1";

void record_group(Stats& stats, const char* calls_key, unsigned n) {
    stats.add(calls_key, 1);
    if (static_cast<double>(n) > stats.get("encoder.card_max_group_size")) {
        stats.set("encoder.card_max_group_size", static_cast<double>(n));
    }
}

}  // namespace

Term encode_atmost(cvc5::TermManager& ctx, const TermVec& lits, unsigned k) {
    TermBuilder tb(ctx);
    const unsigned n = static_cast<unsigned>(lits.size());
    if (n == 0 || k >= n) return tb.bool_val(true);

    auto& stats = Stats::instance();
    record_group(stats, "encoder.card_atmost_calls", n);

    if (k == 0) {
        TermVec negs;
        for (size_t i = 0; i < lits.size(); ++i) negs.push_back(tb.mk_not(lits[i]));
        return tb.mk_and(negs);
    }
    if (k == 1) return pairwise_atmost_one(tb, lits);

    stats.add(ABOVE_ONE_STAT, 1);
    return tb.leq(int_sum(tb, lits), tb.int_val(static_cast<int64_t>(k)));
}

Term encode_atleast(cvc5::TermManager& ctx, const TermVec& lits, unsigned k) {
    TermBuilder tb(ctx);
    const unsigned n = static_cast<unsigned>(lits.size());
    if (k == 0) return tb.bool_val(true);
    if (k > n) return tb.bool_val(false);

    auto& stats = Stats::instance();
    record_group(stats, "encoder.card_atleast_calls", n);

    if (k == 1) return tb.mk_or(lits);

    stats.add(ABOVE_ONE_STAT, 1);
    return tb.geq(int_sum(tb, lits), tb.int_val(static_cast<int64_t>(k)));
}

}  // namespace rantanplan
