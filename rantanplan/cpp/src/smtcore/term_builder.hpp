#pragma once

#include "smt_types.hpp"

#include <cstdint>
#include <stdexcept>
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
    /// The problem decides Int-vs-Real for numeric literals; it is bound at
    /// construction and required, so a builder in a "sort undecided" state is
    /// unconstructible.
    TermBuilder(cvc5::TermManager& tm, const Problem* problem)
        : tm_(tm), problem_(problem) {
        if (!problem_) {
            throw std::logic_error(
                "TermBuilder: a problem is required -- Int-vs-Real for "
                "numeric terms is undecidable without one");
        }
    }

    cvc5::TermManager& tm() const { return tm_; }

    // ── deliberately-fresh literals ─────────────────────────────────────────
    /// A fresh Boolean constant: every call mints a NEW variable, even under
    /// the same name -- mkConst names are print labels, not identities. Use
    /// only for single-use literals (assumption literals, per-query selector
    /// variables) that are created once and passed around by value. A variable
    /// that must be re-derivable by name belongs in a SymbolTable or the
    /// SmtVariableFactory; this is the only sanctioned mkConst outside those.
    Term fresh_bool(const std::string& name) const {
        return tm_.mkConst(tm_.getBooleanSort(), name);
    }

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

    // ── cardinality (at-most-k / at-least-k) ────────────────────────────────
    // cvc5 has no native at-most-k / at-least-k / pseudo-Boolean facility
    // reachable from its API, so cardinality constraints are encoded by hand
    // here -- the one place they are built.
    //
    // The encoding is the **binomial** one (pairwise clauses at k == 1,
    // Int-sum of ITEs above), chosen by measurement over an Int-sum-only
    // alternative (see the migration record, "M2 -- Results" in the sister
    // repo's docs/cvc5/migration-log.md): pairwise clauses go straight into
    // the SAT core instead of through arithmetic, and won on every instance
    // measured.
    //
    // Both forms are **logical equivalences with no auxiliary variables**, so
    // the result is safe to embed at any polarity. Keep it that way: an
    // equisatisfiable counter encoding (sequential counter, totalizer) would
    // be unsound under a negation, and mk_atmost's result does reach negated
    // positions via GroundedEncodingVisitor::convert_from_pool.
    //
    // The internal sums use int_val exclusively: counting literals is
    // intrinsically Int arithmetic and self-contained -- never the problem's
    // numeric sort.
    //
    // Known cost: at-most-one over a group of hundreds -- the `seq`
    // strategy's per-timestep action constraint -- is where a propagator is
    // genuinely better and no clause encoding recovers it. The answer is a
    // THEORY_PLAN propagator, not a cleverer encoding.

    /// At most @p k of @p lits are true. Handles k >= size (true) and k == 0.
    Term mk_atmost(const TermVec& lits, unsigned k) const;
    /// At least @p k of @p lits are true. Handles k == 0 (true) and k > size (false).
    Term mk_atleast(const TermVec& lits, unsigned k) const;

private:
    /// Make two arithmetic terms share a sort, promoting Int to Real if they
    /// differ -- cvc5 rejects a mixed Int/Real pair as a type error.
    std::pair<Term, Term> unify_arith(const Term& a, const Term& b) const;

    cvc5::TermManager& tm_;
    const Problem* problem_;
};

}  // namespace rantanplan
