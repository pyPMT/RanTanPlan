#pragma once

#include <string>

namespace rantanplan {

/**
 * @brief Expression structural kinds
 */
enum class ExprKind {
    UNKNOWN = 0,                // An unset node's kind; the loader rejects it
    CONSTANT = 1,               // Constant atom (e.g., `true`, `3`, `kitchen`)
    PARAMETER = 2,              // Parameter from outer scope (e.g., `from` in `(move from to)`)
    FLUENT_SYMBOL = 3,          // Fluent symbol (e.g., `at-robot`)
    FUNCTION_SYMBOL = 4,        // Function symbol (e.g., `+`, `=`, `and`)
    STATE_VARIABLE = 5,         // Fluent application (e.g., `(at-robot l1)`)
    FUNCTION_APPLICATION = 6,   // Function application (e.g., `(+ 1 3)`)
    VARIABLE = 7                // Variable from outer scope (existential/universal)
    // The values are UP's ExpressionKind numbers, which the IR carries as an
    // int. UP also defines CONTAINER_ID = 8, which is not supported; ir_io.cpp
    // rejects any number outside CONSTANT..VARIABLE.
};

/**
 * @brief Operator types for type safety and consistency
 */
enum class ExprOperator {
    // The operator of a node that has none (any kind but FUNCTION_SYMBOL and
    // FUNCTION_APPLICATION) and of one whose name is none of the below. The
    // kind tells the two apart, so one value does for both. First, so that a
    // value-initialised ExprOperator is NONE.
    NONE,

    // Arithmetic operators
    PLUS, MINUS, MULTIPLY, DIVIDE, MODULO, ABSOLUTE, MAXIMUM, MINIMUM,

    // Comparison operators
    EQUALS, LESS_EQUAL, LESS_THAN, GREATER_EQUAL, GREATER_THAN,

    // Logical operators
    AND, OR, NOT, IMPLIES, IFF,

    // Quantifiers. Only transiently present between the converter and the
    // QuantifierExpansionPass (which runs in the canonicalize phase, before
    // grounding, and rewrites them into AND/OR); no later pass or encoder
    // sees them.
    FORALL, EXISTS,

    // Pseudo-boolean: integer-valued count of n boolean arguments.
    // Deliberately NOT in the arithmetic/logical/comparison categories:
    // folding passes must treat it opaquely; the Z3 visitors encode it
    // natively (atmost/atleast for comparisons with constants).
    COUNT,

    // Array access `a[i]` (nested per dimension). Only transiently present
    // between the converter and the grounder, which lowers every array
    // access into ordinary scalar cells; no later pass or encoder sees it.
    ARRAY_INDEX,

    // Set operators (UP's SET_* family). Only transiently present between the
    // converter and the SetLoweringPass, which lowers every set fluent into
    // ordinary boolean membership cells and rewrites these operators into
    // logic/COUNT; no later pass or encoder sees a SET_* node. SET_CONSTANT is
    // a set literal `{c1..ck}` (zero children = the empty set).
    SET_MEMBER, SET_SUBSETEQ, SET_DISJOINT, SET_CARDINALITY,
    SET_ADD, SET_REMOVE, SET_UNION, SET_INTERSECT, SET_DIFFERENCE,
    SET_CONSTANT
};

inline std::string expr_operator_to_string(ExprOperator op) {
    switch (op) {
        case ExprOperator::PLUS: return "+";
        case ExprOperator::MINUS: return "-";
        case ExprOperator::MULTIPLY: return "*";
        case ExprOperator::DIVIDE: return "/";
        case ExprOperator::MODULO: return "mod";
        case ExprOperator::ABSOLUTE: return "abs";
        case ExprOperator::MAXIMUM: return "max";
        case ExprOperator::MINIMUM: return "min";
        case ExprOperator::EQUALS: return "=";
        case ExprOperator::LESS_EQUAL: return "<=";
        case ExprOperator::LESS_THAN: return "<";
        case ExprOperator::GREATER_EQUAL: return ">=";
        case ExprOperator::GREATER_THAN: return ">";
        case ExprOperator::AND: return "and";
        case ExprOperator::OR: return "or";
        case ExprOperator::NOT: return "not";
        case ExprOperator::IMPLIES: return "=>";
        case ExprOperator::IFF: return "=";
        case ExprOperator::FORALL: return "forall";
        case ExprOperator::EXISTS: return "exists";
        case ExprOperator::COUNT: return "count";
        case ExprOperator::ARRAY_INDEX: return "array_index";
        case ExprOperator::SET_MEMBER: return "set_member";
        case ExprOperator::SET_SUBSETEQ: return "set_subseteq";
        case ExprOperator::SET_DISJOINT: return "set_disjoint";
        case ExprOperator::SET_CARDINALITY: return "set_cardinality";
        case ExprOperator::SET_ADD: return "set_add";
        case ExprOperator::SET_REMOVE: return "set_remove";
        case ExprOperator::SET_UNION: return "set_union";
        case ExprOperator::SET_INTERSECT: return "set_intersect";
        case ExprOperator::SET_DIFFERENCE: return "set_difference";
        case ExprOperator::SET_CONSTANT: return "set_constant";
        default: return "";
    }
}

inline bool is_arithmetic_operator(ExprOperator op) {
    return op == ExprOperator::PLUS || op == ExprOperator::MINUS ||
           op == ExprOperator::MULTIPLY || op == ExprOperator::DIVIDE ||
           op == ExprOperator::MODULO || op == ExprOperator::ABSOLUTE ||
           op == ExprOperator::MAXIMUM || op == ExprOperator::MINIMUM;
}

inline bool is_comparison_operator(ExprOperator op) {
    return op == ExprOperator::EQUALS || op == ExprOperator::LESS_EQUAL ||
           op == ExprOperator::LESS_THAN || op == ExprOperator::GREATER_EQUAL ||
           op == ExprOperator::GREATER_THAN;
}

inline bool is_logical_operator(ExprOperator op) {
    return op == ExprOperator::AND || op == ExprOperator::OR ||
           op == ExprOperator::NOT || op == ExprOperator::IMPLIES ||
           op == ExprOperator::IFF;
}

/// A set operator (UP's SET_* family). All are lowered away by SetLoweringPass
/// before grounding; used by the pass and its no-survivor backstop.
inline bool is_set_operator(ExprOperator op) {
    return op == ExprOperator::SET_MEMBER || op == ExprOperator::SET_SUBSETEQ ||
           op == ExprOperator::SET_DISJOINT || op == ExprOperator::SET_CARDINALITY ||
           op == ExprOperator::SET_ADD || op == ExprOperator::SET_REMOVE ||
           op == ExprOperator::SET_UNION || op == ExprOperator::SET_INTERSECT ||
           op == ExprOperator::SET_DIFFERENCE || op == ExprOperator::SET_CONSTANT;
}

} // namespace rantanplan
