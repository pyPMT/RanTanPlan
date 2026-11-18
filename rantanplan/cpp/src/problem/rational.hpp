#pragma once

#include <charconv>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

namespace rantanplan {

__extension__ using int128 = __int128;  // products of two int64 parts fit

/// An exact rational number, num/den in lowest terms with den > 0: the value
/// of a real constant. UP's reals are exact fractions and so is the SMT
/// encoding (mkReal(num, den)), so 1/3 + 1/3 + 1/3 is 1. Analyses that only
/// estimate a value read to_double(); those that bound one read the interval
/// of doubles holding it (Interval::of).
struct Rational {
    int64_t num = 0;
    int64_t den = 1;

    /// num/den in lowest terms; nothing if den is 0 or a part does not fit in
    /// 64 bits.
    static std::optional<Rational> make(int128 num, int128 den) {
        if (den == 0) return std::nullopt;
        if (den < 0) { num = -num; den = -den; }
        int128 a = num < 0 ? -num : num, b = den;
        while (b != 0) { int128 t = a % b; a = b; b = t; }
        if (a > 1) { num /= a; den /= a; }
        if (num < INT64_MIN || num > INT64_MAX || den > INT64_MAX) return std::nullopt;
        return Rational{static_cast<int64_t>(num), static_cast<int64_t>(den)};
    }
    static Rational of(int64_t v) { return Rational{v, 1}; }

    bool is_integer() const { return den == 1; }
    double to_double() const { return static_cast<double>(num) / static_cast<double>(den); }
    /// "5.0" for a whole number, so that it reads as a real, else "1/3".
    std::string to_string() const {
        return den == 1 ? std::to_string(num) + ".0"
                        : std::to_string(num) + "/" + std::to_string(den);
    }

    bool operator==(const Rational&) const = default;
};

// Exact arithmetic; nothing on overflow or division by zero.
inline std::optional<Rational> add(const Rational& a, const Rational& b) {
    return Rational::make(int128(a.num) * b.den + int128(b.num) * a.den, int128(a.den) * b.den);
}
inline std::optional<Rational> sub(const Rational& a, const Rational& b) {
    return Rational::make(int128(a.num) * b.den - int128(b.num) * a.den, int128(a.den) * b.den);
}
inline std::optional<Rational> mul(const Rational& a, const Rational& b) {
    return Rational::make(int128(a.num) * b.num, int128(a.den) * b.den);
}
inline std::optional<Rational> div(const Rational& a, const Rational& b) {
    return Rational::make(int128(a.num) * b.den, int128(a.den) * b.num);
}
inline std::optional<Rational> neg(const Rational& a) {
    return Rational::make(-int128(a.num), a.den);
}
/// Negative, zero or positive as a is below, equal to or above b.
inline int compare(const Rational& a, const Rational& b) {
    const int128 l = int128(a.num) * b.den, r = int128(b.num) * a.den;
    return l < r ? -1 : (l > r ? 1 : 0);
}

/// The exact value of a finite double as a decimal: "0.5", "3", or
/// "0.1000000000000000055511151231257827021181583404541015625" for the double
/// nearest 0.1. A double is a binary fraction, so 1074 fractional digits hold
/// it exactly; the trailing zeros are dropped.
inline std::string exact_decimal(double v) {
    char buf[1400];  // a sign, 309 integer digits, a point and 1074 fractional digits
    auto [end, ec] = std::to_chars(buf, buf + sizeof buf, v, std::chars_format::fixed, 1074);
    std::string s(buf, end);
    s.erase(s.find_last_not_of('0') + 1);
    if (s.back() == '.') s.pop_back();
    return s == "-0" ? "0" : s;
}

} // namespace rantanplan
