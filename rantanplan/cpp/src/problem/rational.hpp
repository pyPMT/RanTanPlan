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
/// estimate a value (intervals, bounds) read to_double().
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

/// The shortest decimal that reads back as `v` ("0.1", "0.0000001", "-2.5"):
/// the value written in the problem when `v` came from a decimal, or from a
/// fraction with one, as declared bounds do. `v` must be finite.
inline std::string shortest_decimal(double v) {
    char buf[400];  // the longest fixed form of a double has 309 integer digits
    auto [end, ec] = std::to_chars(buf, buf + sizeof buf, v, std::chars_format::fixed);
    return std::string(buf, end);
}

/// shortest_decimal(v) as a Rational; nothing if `v` is not finite or the
/// decimal does not fit in 64 bits.
inline std::optional<Rational> rational_from_double(double v) {
    if (!std::isfinite(v)) return std::nullopt;
    const std::string s = shortest_decimal(v);
    int128 num = 0, den = 1;
    bool fraction = false;
    for (char c : s) {
        if (c == '-') continue;
        if (c == '.') { fraction = true; continue; }
        num = num * 10 + (c - '0');
        if (fraction) den *= 10;
        if (num > INT64_MAX || den > INT64_MAX) return std::nullopt;
    }
    return Rational::make(s[0] == '-' ? -num : num, den);
}

} // namespace rantanplan
