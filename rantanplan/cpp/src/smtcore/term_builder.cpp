#include "term_builder.hpp"

#include "../problem/problem.hpp"

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
    return (problem_ && problem_->all_integer())
        ? tm_.mkInteger(v)
        : tm_.mkReal(v, 1);
}

Term TermBuilder::numeric_val(double v) const {
    if (problem_ && problem_->all_integer()) {
        return tm_.mkInteger(static_cast<int64_t>(v));
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

}  // namespace rantanplan
