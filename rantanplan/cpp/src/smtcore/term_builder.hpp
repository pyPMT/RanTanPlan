#pragma once

#include "smt_types.hpp"

#include <cstdint>
#include <utility>
#include <string>

namespace rantanplan {

class Problem;

/**
 * @brief Builds cvc5 terms. The counterpart of ExprBuilder, one layer down.
 *
 * cvc5 has **no C++ operator overloading** for formula construction:
 * `Term::operator==` is syntactic equality returning `bool` (`cvc5.h:1216`), and
 * there is no `operator&&`, `||`, `!`, `+` or `<=`. Every construction therefore
 * goes through `TermManager::mkTerm(Kind, {...})`, which is unreadable at the
 * call site when nested. This class restores the readability, and folds
 * constants on the way through so encoders can freely emit `true`/`false`
 * placeholders.
 *
 * Folding matters for more than tidiness: the encoders lean on
 * `mk_and({}) == true` and on `true` conjuncts vanishing ("no constraint at
 * this timestep" is emitted as `true` all over the place).
 *
 * Numeric literals follow the problem's integer mode: mixing Int and Real in
 * one problem makes a single-sort logic (QF_LRA, QF_LIA) inconsistent with
 * the terms in it.
 */
class TermBuilder {
public:
    explicit TermBuilder(cvc5::TermManager& tm) : tm_(tm) {}

    cvc5::TermManager& tm() const { return tm_; }

    /// Set the problem, which decides Int-vs-Real for numeric literals.
    void set_problem(const Problem* p) { problem_ = p; }

    // ── constants ───────────────────────────────────────────────────────────
    Term bool_val(bool b) const { return b ? tm_.mkTrue() : tm_.mkFalse(); }
    Term true_term() const { return tm_.mkTrue(); }
    Term false_term() const { return tm_.mkFalse(); }
    bool is_true(const Term& t) const { return t == tm_.mkTrue(); }
    bool is_false(const Term& t) const { return t == tm_.mkFalse(); }

    /// Int or Real according to the problem's integer mode.
    Term numeric_val(int64_t v) const;
    Term numeric_val(double v) const;
    Term int_val(int64_t v) const { return tm_.mkInteger(v); }
    Term zero() const { return numeric_val(static_cast<int64_t>(0)); }

    // ── boolean connectives (constant-folding) ──────────────────────────────
    Term mk_and(TermVec parts) const;
    Term mk_or(TermVec parts) const;
    Term mk_not(const Term& a) const;
    Term mk_implies(const Term& a, const Term& b) const;
    Term mk_ite(const Term& c, const Term& t, const Term& e) const;

    /// Boolean bi-implication, or equality of two terms of any other sort.
    /// `Kind::EQUAL` is n-ary and *chains* into a conjunction, so it is only
    /// ever given exactly two children here.
    Term mk_eq(const Term& a, const Term& b) const;
    Term mk_distinct(const Term& a, const Term& b) const;

    // ── arithmetic and comparison ───────────────────────────────────────────
    Term add(const Term& a, const Term& b) const;
    Term sub(const Term& a, const Term& b) const;
    Term mul(const Term& a, const Term& b) const;
    Term div(const Term& a, const Term& b) const;
    Term neg(const Term& a) const;
    Term lt(const Term& a, const Term& b) const;
    Term leq(const Term& a, const Term& b) const;
    Term gt(const Term& a, const Term& b) const;
    Term geq(const Term& a, const Term& b) const;

private:
    /// Make two arithmetic terms share a sort, promoting Int to Real if they
    /// differ -- cvc5 rejects a mixed Int/Real pair as a type error.
    std::pair<Term, Term> unify_arith(const Term& a, const Term& b) const;

    cvc5::TermManager& tm_;
    const Problem* problem_ = nullptr;
};

}  // namespace rantanplan
