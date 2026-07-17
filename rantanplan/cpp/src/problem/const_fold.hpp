#pragma once

#include "expr_enums.hpp"
#include "expr_pool.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>

namespace rantanplan {

/// Fold a constant-integer expression subtree to its value. Returns nullopt if
/// the subtree is not a compile-time integer constant (a symbolic value, e.g. an
/// un-substituted parameter). Handles the arithmetic operators a range bound or
/// array index may carry after parameter substitution (e.g. `c + 1`, `lc - 1`).
inline std::optional<int64_t> fold_const_int(ExprID eid, const ExprPool& pool) {
    if (!eid.valid()) return std::nullopt;

    if (pool.is_constant(eid)) {
        if (pool.payload_is_int(eid)) return pool.payload_int(eid);
        if (pool.payload_is_double(eid)) {
            double d = pool.payload_double(eid);
            int64_t i = static_cast<int64_t>(d);
            if (static_cast<double>(i) == d) return i;
        }
        return std::nullopt;
    }

    if (!pool.is_function_application(eid)) return std::nullopt;

    ExprOperator op = pool.op(eid);
    size_t nargs = pool.argument_count(eid);

    if (op == ExprOperator::ABSOLUTE && nargs == 1) {
        auto a = fold_const_int(pool.argument(eid, 0), pool);
        if (!a) return std::nullopt;
        return *a < 0 ? -*a : *a;
    }

    if (nargs != 2) return std::nullopt;
    auto a = fold_const_int(pool.argument(eid, 0), pool);
    auto b = fold_const_int(pool.argument(eid, 1), pool);
    if (!a || !b) return std::nullopt;

    switch (op) {
        case ExprOperator::PLUS:     return *a + *b;
        case ExprOperator::MINUS:    return *a - *b;
        case ExprOperator::MULTIPLY: return *a * *b;
        case ExprOperator::DIVIDE:   return *b != 0 ? std::optional<int64_t>(*a / *b) : std::nullopt;
        case ExprOperator::MODULO:   return *b != 0 ? std::optional<int64_t>(*a % *b) : std::nullopt;
        case ExprOperator::MAXIMUM:  return std::max(*a, *b);
        case ExprOperator::MINIMUM:  return std::min(*a, *b);
        default:                     return std::nullopt;
    }
}

} // namespace rantanplan
