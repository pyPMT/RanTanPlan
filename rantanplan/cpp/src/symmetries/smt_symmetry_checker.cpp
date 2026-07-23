#include "smt_symmetry_checker.hpp"
#include "../util/memory_tracker.hpp"
#include "../util/scoped_timer.hpp"
#include "../util/logger.hpp"
#include "../util/stats.hpp"
#include "../config/config.hpp"
#include <iostream>
#include <algorithm>
#include <set>
#include <unordered_set>

namespace rantanplan {

SMTSymmetryChecker::SMTSymmetryChecker(const Problem* problem, cvc5::TermManager& ctx)
    : problem_(problem), context_(ctx), tb_(ctx), symbol_table_(),
      visitor_(ctx, symbol_table_, problem) {
    tb_.set_problem(problem);
}

std::string ActionSwap::to_string() const {
    return action1->name() + "(" + action1->to_string() + ") ↔ " + 
           action2->name() + "(" + action2->to_string() + ")";
}

std::vector<ObjectSwap> SMTSymmetryChecker::detect_all_object_swaps() {
    ScopedTimer timer("symmetry.detection_time_ms");
    double start_memory = MemoryTracker::instance().get_current_memory_mb();

    std::vector<ObjectSwap> detected_swaps;

    // Group objects by type - only objects of same type can be symmetric
    auto objects_by_type = get_objects_by_type();

    // For each type with multiple objects, check for symmetries using SMT
    for (const auto& [type_name, objects] : objects_by_type) {
        if (objects.size() < 2) continue;

        for (size_t i = 0; i < objects.size(); i++) {
            for (size_t j = i + 1; j < objects.size(); j++) {
                if (are_objects_symmetric(objects[i]->name(), objects[j]->name())) {
                    detected_swaps.push_back({objects[i]->name(), objects[j]->name(), type_name});
                }
            }
        }
    }

    // Record stats
    double memory_used = MemoryTracker::instance().get_current_memory_mb() - start_memory;
    Stats::instance().set("symmetry.count", detected_swaps.size());
    Stats::instance().set("symmetry.memory_mb", memory_used);

    Logger::instance().component(VerbosityLevel::INFO, "SymDetect", {
        {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
        {"detected", std::to_string(detected_swaps.size())},
        {"mem", std::to_string(static_cast<int>(memory_used)) + "MB"}
    });

    return detected_swaps;
}

void SMTSymmetryChecker::compute_symmetry_pairs(const std::vector<ObjectSwap>& swaps) {
    detected_symmetries_.clear();

    for (const auto& swap : swaps) {
        auto variable_pairs = get_symmetric_variable_pairs(swap.obj1_name, swap.obj2_name);
        auto action_pairs = get_symmetric_action_pairs(swap.obj1_name, swap.obj2_name);
        detected_symmetries_.emplace_back(swap, variable_pairs, action_pairs);
    }

    if (!detected_symmetries_.empty()) {
        std::string debug_msg = "Symmetry pairs for " + std::to_string(detected_symmetries_.size()) + " swaps:";
        for (const auto& symmetry : detected_symmetries_) {
            debug_msg += "\n  " + symmetry.object_swap.to_string() + " -> " +
                        std::to_string(symmetry.variable_pairs.size()) + " variable pairs, " +
                        std::to_string(symmetry.action_pairs.size()) + " action pairs";
        }
        Logger::instance().debug(debug_msg);
    }
}

bool SMTSymmetryChecker::are_objects_symmetric(const std::string& obj1, const std::string& obj2) {
    // Clear symbol table for clean start
    symbol_table_.clear();

    // Integer constants for the two objects being swapped.
    //
    // These MUST come from the visitor's cached symbol table — the same path
    // the visitor uses for object leaves — so they are the exact Term objects
    // that occur in the constraints built below. substitute() rewrites only
    // occurrences of the very terms passed to it, and `TermManager::mkConst`
    // mints a fresh variable on every call: raw mkConst constants here would
    // occur nowhere in the encoding, the swap would be a no-op, and every
    // object pair would come out "symmetric" (unsound symmetry breaking).
    Term obj1_const = visitor_.create_int_variable(obj1);
    Term obj2_const = visitor_.create_int_variable(obj2);

    // Build original problem formula using temporal encoding
    TermVec original_constraints;

    // Add initial state constraints at timestep 0
    for (const auto& assignment : problem_->initial_state()) {
        Term fluent_expr = visitor_.convert_from_pool(assignment.fluent_id(), 0);
        Term value_expr = visitor_.convert_from_pool(assignment.value_id(), 0);
        original_constraints.push_back(tb_.mk_eq(fluent_expr, value_expr));
    }

    // Add goal constraints at timestep 1 (different from initial state)
    for (const auto& goal : problem_->goals()) {
        Term goal_expr = visitor_.convert_from_pool(goal.goal_id(), 1);
        original_constraints.push_back(goal_expr);
    }

    // Create swapped problem by substituting obj1 ↔ obj2 in all constraints
    TermVec swapped_constraints;
    TermVec from_vec;
    TermVec to_vec;
    from_vec.push_back(obj1_const);
    from_vec.push_back(obj2_const);
    to_vec.push_back(obj2_const);
    to_vec.push_back(obj1_const);

    for (const auto& constraint : original_constraints) {
        Term constraint_copy = constraint;
        Term swapped = constraint_copy.substitute(from_vec, to_vec);
        swapped_constraints.push_back(swapped);
    }

    // Check semantic equivalence using logical biconditional
    Term original_formula = original_constraints.empty() ?
        tb_.bool_val(true) : tb_.mk_and(original_constraints);
    Term swapped_formula = swapped_constraints.empty() ?
        tb_.bool_val(true) : tb_.mk_and(swapped_constraints);

    // Objects are symmetric if: ¬(original ↔ swapped) is UNSAT
    cvc5::Solver solver(context_);
    Term equivalence = tb_.mk_eq(original_formula, swapped_formula);
    solver.assertFormula(tb_.mk_not(equivalence));
    cvc5::Result result = solver.checkSat();

    return result.isUnsat();
}

std::unordered_map<std::string, std::vector<const Object*>> 
SMTSymmetryChecker::get_objects_by_type() const {
    std::unordered_map<std::string, std::vector<const Object*>> result;
    
    for (size_t i = 0; i < problem_->object_count(); i++) {
        const Object& obj = problem_->object(i);
        const std::string type_name = obj.type() ? obj.type()->name() : "unknown";
        result[type_name].push_back(&obj);
    }
    return result;
}


std::vector<std::pair<ExprID, ExprID>> SMTSymmetryChecker::get_symmetric_variable_pairs(
    const std::string& obj1, const std::string& obj2) const {
    std::vector<std::pair<ExprID, ExprID>> variable_pairs;

    // Collect all fluent ExprIDs from initial state that involve either symmetric object
    std::vector<ExprID> relevant_fluents;
    for (const auto& assignment : problem_->initial_state()) {
        ExprID fluent_eid = assignment.fluent_id();
        if (expression_involves_object(fluent_eid, obj1) || expression_involves_object(fluent_eid, obj2)) {
            relevant_fluents.push_back(fluent_eid);
        }
    }

    // For each pair of relevant fluents, check if they are symmetric with respect to obj1 and obj2
    for (size_t i = 0; i < relevant_fluents.size(); i++) {
        for (size_t j = i + 1; j < relevant_fluents.size(); j++) {
            ExprID fluent1 = relevant_fluents[i];
            ExprID fluent2 = relevant_fluents[j];

            if (are_expressions_symmetric(fluent1, fluent2, obj1, obj2)) {
                variable_pairs.push_back({fluent1, fluent2});
            }
        }
    }

    return variable_pairs;
}

bool SMTSymmetryChecker::expression_involves_object(ExprID eid, const std::string& obj_name) const {
    const auto& pool = problem_->pool();

    // Check if this is a leaf node with a string payload matching the object name
    if (pool.is_constant(eid) && pool.payload_is_string(eid) && pool.payload_string(eid) == obj_name) {
        return true;
    }

    // Recursively check children
    for (ExprID child : pool.children(eid)) {
        if (expression_involves_object(child, obj_name)) {
            return true;
        }
    }
    return false;
}

bool SMTSymmetryChecker::are_expressions_symmetric(ExprID eid1, ExprID eid2,
                                                   const std::string& obj1, const std::string& obj2) const {
    const auto& pool = problem_->pool();
    const auto& node1 = pool.get(eid1);
    const auto& node2 = pool.get(eid2);

    if (node1.kind != node2.kind) return false;
    if (node1.op != node2.op) return false;
    if (node1.type_id != node2.type_id) return false;

    bool leaf1 = node1.children.empty();
    bool leaf2 = node2.children.empty();

    // Base case: both are leaf nodes
    if (leaf1 && leaf2) {
        if (pool.payload_is_string(eid1) && pool.payload_is_string(eid2)) {
            const std::string& name1 = pool.payload_string(eid1);
            const std::string& name2 = pool.payload_string(eid2);

            // Symmetric if one has obj1 where the other has obj2, or vice versa
            if (name1 == obj1 && name2 == obj2) return true;
            if (name1 == obj2 && name2 == obj1) return true;
            // Or if they're identical but don't involve the symmetric objects
            if (name1 == name2 && name1 != obj1 && name1 != obj2) return true;
            return false;
        }
        // For non-string leaves: must be identical
        return eid1 == eid2;
    }

    // Both must have children
    if (leaf1 || leaf2) return false;
    if (node1.children.size() != node2.children.size()) return false;

    // Recursively check all children
    for (size_t i = 0; i < node1.children.size(); ++i) {
        if (!are_expressions_symmetric(node1.children[i], node2.children[i], obj1, obj2)) {
            return false;
        }
    }

    return true;
}

std::vector<ActionSwap> SMTSymmetryChecker::get_symmetric_action_pairs(
    const std::string& obj1, const std::string& obj2) const {
    std::vector<ActionSwap> action_pairs;
    
    // Get all actions from the problem
    const auto& actions = problem_->actions();
    
    // Check all pairs of actions for symmetry
    for (size_t i = 0; i < actions.size(); i++) {
        for (size_t j = i + 1; j < actions.size(); j++) {
            const Action& action1 = actions[i];
            const Action& action2 = actions[j];
            
            // Check if these actions are symmetric with respect to obj1 and obj2
            if (are_actions_symmetric_by_signature(action1, action2, obj1, obj2)) {
                action_pairs.push_back(ActionSwap{&action1, &action2});
            }
        }
    }
    
    return action_pairs;
}

bool SMTSymmetryChecker::are_actions_symmetric_by_signature(const Action& action1, const Action& action2,
                                                           const std::string& obj1, const std::string& obj2) const {
    // Actions must have the same base action name
    if (action1.name() != action2.name()) return false;

    // Use the actual parameter objects (not name parsing)
    const auto& params1 = action1.parameters();
    const auto& params2 = action2.parameters();
    if (params1.size() != params2.size()) return false;

    // Check if parameters differ only by obj1<->obj2 swap
    bool has_symmetric_swap = false;
    for (size_t i = 0; i < params1.size(); i++) {
        const std::string& name1 = params1[i].name();
        const std::string& name2 = params2[i].name();

        if (name1 == name2) {
            continue;
        } else if ((name1 == obj1 && name2 == obj2) || (name1 == obj2 && name2 == obj1)) {
            has_symmetric_swap = true;
        } else {
            return false;
        }
    }

    return has_symmetric_swap;
}

// ========================================================================
// PUBLIC INTERFACE IMPLEMENTATIONS
// ========================================================================

std::vector<ObjectSwap> SMTSymmetryChecker::get_object_swaps() const {
    std::vector<ObjectSwap> swaps;
    for (const auto& symmetry : detected_symmetries_) {
        swaps.push_back(symmetry.object_swap);
    }
    return swaps;
}

std::vector<std::pair<ExprID, ExprID>>
SMTSymmetryChecker::get_variable_pairs_for_swap(const std::string& obj1, const std::string& obj2) const {
    for (const auto& symmetry : detected_symmetries_) {
        const ObjectSwap& swap = symmetry.object_swap;
        if ((swap.obj1_name == obj1 && swap.obj2_name == obj2) ||
            (swap.obj1_name == obj2 && swap.obj2_name == obj1)) {
            return symmetry.variable_pairs;
        }
    }
    return {}; // Empty vector if no symmetry found
}

bool SMTSymmetryChecker::are_objects_known_symmetric(const std::string& obj1, const std::string& obj2) const {
    for (const auto& symmetry : detected_symmetries_) {
        const ObjectSwap& swap = symmetry.object_swap;
        if ((swap.obj1_name == obj1 && swap.obj2_name == obj2) ||
            (swap.obj1_name == obj2 && swap.obj2_name == obj1)) {
            return true;
        }
    }
    return false;
}

std::vector<ActionSwap> SMTSymmetryChecker::get_action_pairs_for_swap(const std::string& obj1, const std::string& obj2) const {
    for (const auto& symmetry : detected_symmetries_) {
        const ObjectSwap& swap = symmetry.object_swap;
        if ((swap.obj1_name == obj1 && swap.obj2_name == obj2) ||
            (swap.obj1_name == obj2 && swap.obj2_name == obj1)) {
            return symmetry.action_pairs;
        }
    }
    return {}; // Empty vector if no symmetry found
}

} // namespace rantanplan