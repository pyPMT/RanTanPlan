#pragma once

#include "../problem/problem.hpp"
#include <string>
#include <utility>
#include <vector>

namespace rantanplan {

/**
 * @brief Represents a detected object symmetry (two objects that can be swapped)
 */
struct ObjectSwap {
    std::string obj1_name;
    std::string obj2_name;
    std::string object_type;

    std::string to_string() const {
        return obj1_name + " ↔ " + obj2_name + " (type: " + object_type + ")";
    }
};

/**
 * @brief Represents a detected action symmetry (two actions that can be swapped)
 */
struct ActionSwap {
    const Action* action1;
    const Action* action2;

    std::string to_string() const;
};


/**
 * @brief Stores symmetry information for a pair of symmetric objects
 */
struct SymmetryInfo {
    ObjectSwap object_swap;
    std::vector<std::pair<ExprID, ExprID>> variable_pairs;
    std::vector<ActionSwap> action_pairs;

    SymmetryInfo(const ObjectSwap& swap,
                 const std::vector<std::pair<ExprID, ExprID>>& pairs,
                 const std::vector<ActionSwap>& actions = {})
        : object_swap(swap), variable_pairs(pairs), action_pairs(actions) {}
};

} // namespace rantanplan
