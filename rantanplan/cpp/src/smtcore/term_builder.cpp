#include "term_builder.hpp"

#include "../problem/problem.hpp"
#include "../util/stats.hpp"

#include <stdexcept>
#include <string>

namespace rantanplan {

using cvc5::Kind;

namespace {

/// Promote an Integer-sorted term to Real.
///
/// cvc5 does not coerce Int to Real: `(= real_term 2)` is a type error
/// ("Subexpressions must have the same type"). Rather than chase every site
/// that can produce a mixed pair -- object indices, bounds, action costs,
/// effect values -- the coercion is applied once, here, in the builders that
/// combine two arithmetic terms.
Term to_real(cvc5::TermManager& tm, const Term& t) {
    return t.getSort().isInteger() ? tm.mkTerm(Kind::TO_REAL, {t}) : t;
}

}  // namespace

std::pair<Term, Term> TermBuilder::unify_arith(const Term& a, const Term& b) const {
    const bool ar = a.getSort().isReal(), br = b.getSort().isReal();
    if (ar == br) return {a, b};
    return {ar ? a : to_real(tm_, a), br ? b : to_real(tm_, b)};
}

Term TermBuilder::numeric_val(int64_t v) const {
    return problem_->all_integer() ? tm_.mkInteger(v) : tm_.mkReal(v, 1);
}

Term TermBuilder::numeric_val(double v) const {
    if (problem_->all_integer()) {
        // all_integer guarantees every constant in the problem is
        // integer-valued; a fractional value here means that invariant broke
        // somewhere upstream. Fail loudly instead of truncating.
        const auto as_int = static_cast<int64_t>(v);
        if (static_cast<double>(as_int) != v) {
            throw std::logic_error(
                "TermBuilder::numeric_val: non-integral constant "
                + std::to_string(v) + " in an all-integer problem");
        }
        return tm_.mkInteger(as_int);
    }
    return tm_.mkReal(std::to_string(v));
}

// ── boolean connectives ────────────────────────────────────────────────────
// The folding here mirrors ExprBuilder::make_and/make_or one layer up. It is
// load-bearing rather than decorative: encoders emit `true` for "no constraint
// at this timestep" all over the place and rely on it disappearing.

Term TermBuilder::mk_and(TermVec parts) const {
    TermVec kept;
    kept.reserve(parts.size());
    for (const Term& p : parts) {
        if (is_false(p)) return false_term();
        if (is_true(p)) continue;
        kept.push_back(p);
    }
    if (kept.empty()) return true_term();
    if (kept.size() == 1) return kept.front();
    return tm_.mkTerm(Kind::AND, kept);
}

Term TermBuilder::mk_or(TermVec parts) const {
    TermVec kept;
    kept.reserve(parts.size());
    for (const Term& p : parts) {
        if (is_true(p)) return true_term();
        if (is_false(p)) continue;
        kept.push_back(p);
    }
    if (kept.empty()) return false_term();
    if (kept.size() == 1) return kept.front();
    return tm_.mkTerm(Kind::OR, kept);
}

Term TermBuilder::mk_not(const Term& a) const {
    if (is_true(a)) return false_term();
    if (is_false(a)) return true_term();
    if (a.getKind() == Kind::NOT) return a[0];
    return tm_.mkTerm(Kind::NOT, {a});
}

Term TermBuilder::mk_implies(const Term& a, const Term& b) const {
    if (is_false(a) || is_true(b)) return true_term();
    if (is_true(a)) return b;
    if (is_false(b)) return mk_not(a);
    return tm_.mkTerm(Kind::IMPLIES, {a, b});
}

Term TermBuilder::mk_ite(const Term& c, const Term& t, const Term& e) const {
    if (is_true(c)) return t;
    if (is_false(c)) return e;
    if (t == e) return t;
    return tm_.mkTerm(Kind::ITE, {c, t, e});
}

Term TermBuilder::mk_eq(const Term& a, const Term& b) const {
    if (a == b) return true_term();
    // Kind::EQUAL is n-ary and chains into a conjunction; only ever binary here.
    auto [x, y] = unify_arith(a, b);
    return tm_.mkTerm(Kind::EQUAL, {x, y});
}

Term TermBuilder::mk_distinct(const Term& a, const Term& b) const {
    if (a == b) return false_term();
    auto [x, y] = unify_arith(a, b);
    return tm_.mkTerm(Kind::DISTINCT, {x, y});
}

// ── arithmetic and comparison ──────────────────────────────────────────────

Term TermBuilder::add(const Term& a, const Term& b) const { auto [x, y] = unify_arith(a, b); return tm_.mkTerm(Kind::ADD, {x, y}); }
Term TermBuilder::sub(const Term& a, const Term& b) const { auto [x, y] = unify_arith(a, b); return tm_.mkTerm(Kind::SUB, {x, y}); }
Term TermBuilder::mul(const Term& a, const Term& b) const { auto [x, y] = unify_arith(a, b); return tm_.mkTerm(Kind::MULT, {x, y}); }
Term TermBuilder::div(const Term& a, const Term& b) const { auto [x, y] = unify_arith(a, b); return tm_.mkTerm(Kind::DIVISION, {x, y}); }
Term TermBuilder::neg(const Term& a) const { return tm_.mkTerm(Kind::NEG, {a}); }
Term TermBuilder::lt(const Term& a, const Term& b) const { auto [x, y] = unify_arith(a, b); return tm_.mkTerm(Kind::LT, {x, y}); }
Term TermBuilder::leq(const Term& a, const Term& b) const { auto [x, y] = unify_arith(a, b); return tm_.mkTerm(Kind::LEQ, {x, y}); }
Term TermBuilder::gt(const Term& a, const Term& b) const { auto [x, y] = unify_arith(a, b); return tm_.mkTerm(Kind::GT, {x, y}); }
Term TermBuilder::geq(const Term& a, const Term& b) const { auto [x, y] = unify_arith(a, b); return tm_.mkTerm(Kind::GEQ, {x, y}); }

// ── cardinality ────────────────────────────────────────────────────────────
// See the section comment in the header for the encoding choice (binomial),
// the polarity-safety constraint (no auxiliary variables), and why the sums
// are int_val-only.

namespace {

Term int_sum(const TermBuilder& tb, const TermVec& lits) {
    Term one = tb.int_val(1);
    Term zero = tb.int_val(0);
    Term sum = zero;
    for (const Term& lit : lits) {
        sum = tb.add(sum, tb.mk_ite(lit, one, zero));
    }
    return sum;
}

// At most one: the pairwise clauses (¬bᵢ ∨ ¬bⱼ). n(n-1)/2 of them.
Term pairwise_atmost_one(const TermBuilder& tb, const TermVec& lits) {
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

Term TermBuilder::mk_atmost(const TermVec& lits, unsigned k) const {
    const unsigned n = static_cast<unsigned>(lits.size());
    if (n == 0 || k >= n) return true_term();

    auto& stats = Stats::instance();
    record_group(stats, "encoder.card_atmost_calls", n);

    if (k == 0) {
        TermVec negs;
        negs.reserve(lits.size());
        for (const Term& lit : lits) negs.push_back(mk_not(lit));
        return mk_and(negs);
    }
    if (k == 1) return pairwise_atmost_one(*this, lits);

    stats.add(ABOVE_ONE_STAT, 1);
    return leq(int_sum(*this, lits), int_val(static_cast<int64_t>(k)));
}

Term TermBuilder::mk_atleast(const TermVec& lits, unsigned k) const {
    const unsigned n = static_cast<unsigned>(lits.size());
    if (k == 0) return true_term();
    if (k > n) return false_term();

    auto& stats = Stats::instance();
    record_group(stats, "encoder.card_atleast_calls", n);

    if (k == 1) return mk_or(lits);

    stats.add(ABOVE_ONE_STAT, 1);
    return geq(int_sum(*this, lits), int_val(static_cast<int64_t>(k)));
}

}  // namespace rantanplan
