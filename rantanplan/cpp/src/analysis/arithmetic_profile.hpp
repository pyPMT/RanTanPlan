#pragma once

#include <string>

namespace rantanplan {

/**
 * @brief Classifies the numeric constraint structure of a planning problem.
 *
 * Used to select the SMT logic string passed to cvc5's setLogic(). The
 * logic is binding, not just a hint: it restricts which theories may be
 * asserted, and it also triggers logic-specific parameter tuning
 * (relevancy, phase selection, restarts, arithmetic solver backend, etc.).
 */
enum class ArithmeticProfile {
    NONE = 0,              // Purely propositional (no numeric fluents)
    DIFFERENCE_LOGIC = 1,  // x - y <= c, x <= c (at most 2 vars per comparison, unit coefficients)
    LINEAR = 2,            // General linear arithmetic (a1*x1 + a2*x2 + ... <= c)
    NONLINEAR = 3          // Variable*variable, variable/variable, abs/mod/min/max with variables
};

inline std::string arithmetic_profile_to_string(ArithmeticProfile p) {
    switch (p) {
        case ArithmeticProfile::NONE:             return "none";
        case ArithmeticProfile::DIFFERENCE_LOGIC: return "difference_logic";
        case ArithmeticProfile::LINEAR:           return "linear";
        case ArithmeticProfile::NONLINEAR:        return "nonlinear";
        default:                                  return "unknown";
    }
}

/**
 * @brief Returns the recommended SMT logic string for the given profile.
 *
 * The logic string is passed to cvc5's setLogic(), which restricts the
 * theories that may be asserted for the rest of the solve and triggers
 * logic-specific parameter tuning (relevancy, phase selection, restarts,
 * etc.). All logics route through the same underlying linear arithmetic
 * solver — the gains come from this parameter tuning, not from switching
 * arithmetic backends.
 *
 * When all_integer is true (all numeric fluents are int-typed, no division,
 * all constants integer-valued), integer logics (QF_IDL, QF_LIA) are used.
 * These enable tighter integer-specific reasoning in the solver.
 *
 * For nonlinear problems, we still use QF_LRA/QF_LIA — the solver's linear
 * arithmetic core handles the nonlinear constraints we produce without
 * needing a dedicated nonlinear logic, and picking one (QF_NRA) would
 * select a solver incompatible with incremental solving.
 *
 * @return Logic string, or nullptr if no logic hint is appropriate.
 */
inline const char* recommended_logic(ArithmeticProfile p, bool all_integer = false) {
    switch (p) {
        case ArithmeticProfile::NONE:             return nullptr;
        case ArithmeticProfile::DIFFERENCE_LOGIC: return all_integer ? "QF_IDL" : "QF_RDL";
        case ArithmeticProfile::LINEAR:           return all_integer ? "QF_LIA" : "QF_LRA";
        case ArithmeticProfile::NONLINEAR:        return all_integer ? "QF_LIA" : "QF_LRA";
        default:                                  return nullptr;
    }
}

} // namespace rantanplan
