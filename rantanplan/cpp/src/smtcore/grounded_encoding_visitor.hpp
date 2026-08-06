#pragma once

#include "../problem/problem.hpp"
#include "../problem/object.hpp"
#include "../problem/expr_pool.hpp"
#include "smt_types.hpp"
#include "smt_variable_factory.hpp"
#include "term_builder.hpp"
#include <string>
#include <vector>
#include <functional>
#include <stdexcept>

namespace rantanplan {

// Forward declarations
class Fluent;

/**
 * @brief Converts ExprID expressions to cvc5 terms using grounded variables
 *
 * Creates individual SMT variables for each grounded fluent at specific timesteps.
 * Works in conjunction with GroundedEncoder to maintain consistency in variable
 * naming and creation.
 *
 * Key features:
 * - Fluent applications become individual variables (e.g., "located_plane1_city0_5")
 * - Proper type support for Boolean, Integer, Real, and Object fluents
 * - Uses SmtVariableFactory for consistent variable creation and naming
 * - Supports temporal encoding through timestep parameters
 *
 * All conversion methods throw std::runtime_error on malformed expressions.
 */
class GroundedEncodingVisitor {
private:
    cvc5::TermManager& ctx_;
    const Problem* problem_;
    int current_timestep_;
    SmtVariableFactory* variable_factory_;

    /// cvc5 has no operator overloading, so every connective/arithmetic node is
    /// built through the factory's TermBuilder. Read through the factory rather
    /// than held by value: the factory's problem pointer (Int-vs-Real policy) is
    /// often set after this visitor is constructed.
    const TermBuilder& tb() const { return variable_factory_->builder(); }

public:
    // Constructor
    GroundedEncodingVisitor(cvc5::TermManager& ctx, const Problem* problem,
                           SmtVariableFactory* factory);

    // Temporal encoding methods
    void set_timestep(int timestep) { current_timestep_ = timestep; }
    int get_timestep() const { return current_timestep_; }
    void clear_timestep() { current_timestep_ = -1; }

    // ExprID-based conversion: walks ExprNode directly via ExprPool, no Expression needed
    Term convert_from_pool(ExprID id, int timestep = -1);

private:
    // Recursive helper for ExprID-based conversion
    Term convert_node(ExprID id);
    // Helper methods for specific SMT operations
    Term handle_and(const std::vector<Term>& args);
    Term handle_or(const std::vector<Term>& args);
    Term handle_not(const std::vector<Term>& args);
    Term handle_equals(const std::vector<Term>& args);
    Term handle_less_than(const std::vector<Term>& args);
    Term handle_less_equal(const std::vector<Term>& args);
    Term handle_greater_than(const std::vector<Term>& args);
    Term handle_greater_equal(const std::vector<Term>& args);
    Term handle_plus(const std::vector<Term>& args);
    Term handle_minus(const std::vector<Term>& args);
    Term handle_multiply(const std::vector<Term>& args);
    Term handle_divide(const std::vector<Term>& args);
    Term handle_implies(const std::vector<Term>& args);
    // Count support: standalone counts become sums of ite(b,1,0);
    // comparisons against integer constants become clausal cardinality
    // constraints (atmost/atleast) built by cardinality_encoding.
    Term handle_count(const std::vector<Term>& args);
    Term encode_count_comparison(ExprID count_id, ExprOperator op,
                                     int64_t constant, bool count_on_left);
};

} // namespace rantanplan
