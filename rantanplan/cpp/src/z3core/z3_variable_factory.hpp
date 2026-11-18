#pragma once

#include "../problem/fluent.hpp"
#include "../problem/action.hpp"
#include "../problem/rational.hpp"
#include <z3++.h>
#include <string>
#include <unordered_map>
#include <memory>
#include <vector>
#include <optional>

namespace rantanplan {

class Problem;  // forward declaration

/**
 * @brief Factory class responsible for creating and managing Z3 variables
 * 
 * This factory encapsulates all Z3 variable creation logic and provides
 * type-safe methods for creating variables based on fluent types.
 * It maintains variable storage and caching to ensure consistency
 * across the encoding process.
 * 
 * Key responsibilities:
 * - Variable naming (fluents and actions)
 * - Variable creation with proper types
 * - Variable caching and lifecycle management
 * - Timestep-aware variable management
 *
 * CONTRACT (static atom identity): a ground fluent *is* its interned ExprID,
 * and its Z3 state variable is keyed by rendered name × timestep — so which
 * cell an access touches is decided at compile time, never by the model.
 * Everything structured (arrays, sets) is lowered to ground scalar cells
 * before grounding to preserve this; a feature that wants model-dependent
 * cell selection (symbolic indices) must introduce explicit read/write
 * variables and element constraints instead of breaking this keying.
 */
class Z3VariableFactory {
private:
    z3::context& ctx_;
    
    // Storage for SMT variables - outer vector is indexed by timestep
    // state_vars_[0]["at_airplane1_city1"] -> z3::expr(bool variable for fluent at timestep 0)
    std::vector<std::unordered_map<std::string, std::shared_ptr<z3::expr>>> state_vars_;
    
    // action_vars_[0]["move_airplane1_city1_city2"] -> z3::expr(bool variable for action at timestep 0)
    std::vector<std::unordered_map<std::string, std::shared_ptr<z3::expr>>> action_vars_;
    
    // Reverse mappings from variable names to fluents/actions and timesteps
    std::unordered_map<std::string, std::pair<std::shared_ptr<Fluent>, int>> state_var_name_to_fluent_;
    std::unordered_map<std::string, std::pair<std::shared_ptr<Action>, int>> action_var_name_to_action_;
    
public:
    // Constructor
    explicit Z3VariableFactory(z3::context& ctx);

    /// Access the underlying Z3 context.
    z3::context& ctx() const { return ctx_; }

    /// Set the problem pointer for integer mode detection.
    void set_problem(const Problem* p) { problem_ = p; }
    
    // Basic variable creation methods (create new variables without caching)
    z3::expr create_bool_variable(const std::string& name);
    z3::expr create_int_variable(const std::string& name);
    z3::expr create_real_variable(const std::string& name);
    z3::expr create_object_variable(const std::string& name);
    
    // Symbol-based variable creation (for visit_symbol)
    z3::expr create_symbol_variable(const std::string& name, const Type* type);
    
    // High-level variable management with caching
    const z3::expr& get_fluent_variable(const Fluent& fluent, int timestep);
    const z3::expr& get_action_variable(const Action& action, int timestep);

    // Const versions for read-only access (will lookup existing variables)
    const z3::expr& get_fluent_variable(const Fluent& fluent, int timestep) const;
    const z3::expr& get_action_variable(const Action& action, int timestep) const;
    
    // Variable naming methods
    std::string get_fluent_var_name(const Fluent& fluent) const;
    std::string get_fluent_var_name(const Fluent& fluent, int timestep) const;
    std::string get_action_var_name(const Action& action) const;
    std::string get_action_var_name(const Action& action, int timestep) const;
    
    // Reverse lookup methods
    std::optional<std::pair<const Fluent&, int>> get_fluent_from_variable(const z3::expr& var) const;
    std::optional<std::pair<const Action&, int>> get_action_from_variable(const z3::expr& var) const;
    bool is_fluent_variable(const z3::expr& var) const;
    bool is_action_variable(const z3::expr& var) const;
    
    // Variable enumeration methods for propagators
    std::vector<std::shared_ptr<z3::expr>> get_all_fluent_variables(int timestep) const;
    std::vector<std::shared_ptr<z3::expr>> get_all_action_variables(int timestep) const;
    std::vector<std::pair<std::shared_ptr<z3::expr>, int>> get_all_fluent_variables() const;
    std::vector<std::pair<std::shared_ptr<z3::expr>, int>> get_all_action_variables() const;
    
    // Numeric constant creation (Int or Real depending on all_integer mode)
    z3::expr make_numeric_val(int64_t v) const;
    z3::expr make_numeric_val(const Rational& v) const;
    /// A bound an analysis computed in floating point (rounded outward), as
    /// exactly that double: a nearby decimal could fall inside the bound and
    /// cut off the value it holds. Int in integer mode, truncated toward zero,
    /// which only weakens a bound on an Int.
    z3::expr make_numeric_val(double v) const;
    /// As make_numeric_val(double), but a Real whatever the integer mode.
    z3::expr real_val(double v) const;
    z3::expr make_zero() const;

    // Timestep management queries
    int get_max_timestep() const;
    bool has_variables_for_timestep(int timestep) const;

private:
    const Problem* problem_ = nullptr;

    // Helper methods for variable creation and management
    void ensure_timestep_capacity(int timestep);
    z3::expr create_new_fluent_variable(const Fluent& fluent, const std::string& var_name);
};

} // namespace rantanplan
