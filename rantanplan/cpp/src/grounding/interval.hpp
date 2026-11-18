#pragma once

#include "problem/rational.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace rantanplan {

/// Arithmetic rounded toward `toward` (-inf for a lower bound, +inf for an
/// upper one): the nearest-rounded result, stepped once outward when the exact
/// result lies beyond it. The error is known exactly (TwoSum for a sum, fma
/// for a product or a quotient), so an exact operation stays exact.
namespace directed {
/// Below this magnitude a product, or the dividend of a quotient, has an
/// error (a remainder) that may itself underflow, so the result is always
/// stepped: the least normal double, 2^-1022, times 2^53.
inline constexpr double tiny = 0x1p-969;

inline double step_if(bool beyond, double v, double toward) {
    return beyond ? std::nextafter(v, toward) : v;
}
/// A result that is not finite: from finite operands it overflowed, and the
/// largest finite double bounds it on the side away from the infinity.
inline double not_finite(double v, double a, double b, double toward) {
    return std::isfinite(a) && std::isfinite(b) ? std::nextafter(v, toward) : v;
}
inline double add(double a, double b, double toward) {
    const double s = a + b;
    if (!std::isfinite(s)) return not_finite(s, a, b, toward);
    const double bb = s - a;
    const double err = (a - (s - bb)) + (b - bb);  // a + b == s + err
    return step_if(err != 0 && (err < 0) == (toward < 0), s, toward);
}
inline double mul(double a, double b, double toward) {
    const double p = a * b;
    if (!std::isfinite(p)) return not_finite(p, a, b, toward);
    if (a == 0 || b == 0) return p;
    if (std::fabs(p) < tiny) return std::nextafter(p, toward);
    const double err = std::fma(a, b, -p);  // a * b == p + err
    return step_if(err != 0 && (err < 0) == (toward < 0), p, toward);
}
inline double div(double a, double b, double toward) {
    const double q = a / b;
    if (!std::isfinite(q)) return not_finite(q, a, b, toward);
    if (a == 0 || std::isinf(b)) return q;
    if (std::fabs(q) < tiny || std::fabs(a) < tiny) return std::nextafter(q, toward);
    const double rem = std::fma(-q, b, a);  // a / b == q + rem / b
    return step_if(rem != 0 && ((rem < 0) != (b < 0)) == (toward < 0), q, toward);
}
} // namespace directed

/// Lightweight closed interval [lower, upper] for numeric bounds in grounding.
/// Self-contained — no dependency on the ARPG Interval class. Its arithmetic
/// rounds outward, so an interval computed from others holds every exact value
/// it stands for (bounds derived with doubles never cut off a reachable value).
struct Interval {
    double lower;
    double upper;

    /// Point interval [val, val].
    explicit Interval(double val) : lower(val), upper(val) {}

    /// The tightest interval of doubles holding v: [v, v] up to 2^53.
    static Interval of(int64_t v) {
        constexpr double inf = std::numeric_limits<double>::infinity();
        const double d = static_cast<double>(v);  // an integer, the nearest to v
        // 2^63 exceeds every int64_t (and converting it back would overflow).
        const int above = d >= 0x1p63 ? 1 : (static_cast<int64_t>(d) > v) - (static_cast<int64_t>(d) < v);
        return {above > 0 ? std::nextafter(d, -inf) : d, above < 0 ? std::nextafter(d, inf) : d};
    }
    /// An interval of doubles holding r, the tightest when its numerator and
    /// denominator are at most 2^53 in magnitude (so doubles both), as those
    /// of real constants almost always are: [r, r] when r is a double.
    static Interval of(const Rational& r) { return of(r.num) / of(r.den); }

    /// Range interval [lo, hi], auto-swaps if reversed.
    Interval(double lo, double hi)
        : lower(std::min(lo, hi)), upper(std::max(lo, hi)) {}

    /// Full range (unknown / uninitialized).
    static Interval unbounded() {
        return {-std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity()};
    }

    // -- Interval arithmetic --------------------------------------------------

    Interval operator+(const Interval& o) const {
        constexpr double inf = std::numeric_limits<double>::infinity();
        return {directed::add(lower, o.lower, -inf), directed::add(upper, o.upper, inf)};
    }

    Interval operator-(const Interval& o) const {
        constexpr double inf = std::numeric_limits<double>::infinity();
        return {directed::add(lower, -o.upper, -inf), directed::add(upper, -o.lower, inf)};
    }

    Interval operator*(const Interval& o) const {
        constexpr double inf = std::numeric_limits<double>::infinity();
        using directed::mul;
        return {std::min({mul(lower, o.lower, -inf), mul(lower, o.upper, -inf),
                          mul(upper, o.lower, -inf), mul(upper, o.upper, -inf)}),
                std::max({mul(lower, o.lower, inf), mul(lower, o.upper, inf),
                          mul(upper, o.lower, inf), mul(upper, o.upper, inf)})};
    }

    Interval operator/(const Interval& o) const {
        // Division by interval crossing zero → unbounded (safe over-approx).
        if (o.lower <= 0.0 && o.upper >= 0.0) return unbounded();
        constexpr double inf = std::numeric_limits<double>::infinity();
        using directed::div;
        return {std::min({div(lower, o.lower, -inf), div(lower, o.upper, -inf),
                          div(upper, o.lower, -inf), div(upper, o.upper, -inf)}),
                std::max({div(lower, o.lower, inf), div(lower, o.upper, inf),
                          div(upper, o.lower, inf), div(upper, o.upper, inf)})};
    }

    // -- Set operations -------------------------------------------------------

    /// Convex union: [min(lowers), max(uppers)].
    Interval convex_union(const Interval& o) const {
        return {std::min(lower, o.lower), std::max(upper, o.upper)};
    }

    /// True if the intervals overlap (share at least one point).
    bool overlaps(const Interval& o) const {
        return lower <= o.upper && o.lower <= upper;
    }

    bool is_unbounded() const {
        return std::isinf(lower) || std::isinf(upper);
    }

    bool operator==(const Interval& o) const {
        return lower == o.lower && upper == o.upper;
    }

    bool operator!=(const Interval& o) const { return !(*this == o); }

    std::string to_string() const {
        return "[" + std::to_string(lower) + ", " + std::to_string(upper) + "]";
    }
};

} // namespace rantanplan
