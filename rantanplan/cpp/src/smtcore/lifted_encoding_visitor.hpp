#pragma once

#include "../problem/problem.hpp"
#include "../problem/expr_pool.hpp"
#include "smt_types.hpp"
#include "term_builder.hpp"
#include <string>
#include <unordered_map>
#include <memory>
#include <stdexcept>

namespace rantanplan {

// Type alias for the unified symbol table.
//
// In cvc5 a function symbol is just a `Term` whose sort is a function sort, so
// one table holds variables and function symbols alike; the discriminator is
// `t.getSort().isFunction()`.
using SmtObject = Term;
using SymbolTable = std::unordered_map<std::string, SmtObject>;

/**
 * @brief Converts ExprID expressions to cvc5 terms using uninterpreted functions
 *
 * Creates uninterpreted functions for fluent applications (lifted encoding).
 * Used by SMTSymmetryChecker for symmetry detection via term substitution.
 * Supports temporal encoding through timestep parameters.
 *
 * CACHING IS LOAD-BEARING. `TermManager::mkConst` is documented as always
 * returning a **fresh** variable, "even if the same arguments were provided on a
 * previous call" (cvc5.h:4388) — the name is a print label, not an identity.
 * Every lookup in `symbol_table_` below is therefore what makes two occurrences
 * of one symbol be the same variable. Removing an apparently redundant cache
 * probe would make each occurrence a distinct variable and silently break the
 * symmetry checker's substitute()-based equivalence test.
 *
 * All conversion methods throw std::runtime_error on malformed expressions.
 */
class LiftedEncodingVisitor {
private:
    cvc5::TermManager& ctx_;
    TermBuilder tb_; // cvc5 has no operator overloading; all connectives go here
    SymbolTable& symbol_table_; // Reference to external symbol table
    const Problem* problem_; // Reference to problem instance for fluent type lookup
    int current_timestep_; // Current timestep for temporal encoding (-1 means no timestep)

public:
    // Constructor
    LiftedEncodingVisitor(cvc5::TermManager& ctx, SymbolTable& symbol_table, const Problem* problem = nullptr)
        : ctx_(ctx), tb_(ctx), symbol_table_(symbol_table), problem_(problem), current_timestep_(-1) {
        tb_.set_problem(problem);
    }

    void clear_symbol_table() {
        symbol_table_.clear();
    }

    // Temporal encoding methods
    void set_timestep(int timestep) { current_timestep_ = timestep; }
    int get_timestep() const { return current_timestep_; }
    void clear_timestep() { current_timestep_ = -1; }

    // Public methods for creating variables from names. These are cached by
    // name: callers (SMTSymmetryChecker) rely on getting back the *same* Term
    // the encoding already contains.
    Term create_bool_variable(const std::string& name);
    Term create_int_variable(const std::string& name);
    Term create_real_variable(const std::string& name);

    // ExprID-based conversion: walks ExprNode directly via ExprPool, no Expression needed
    Term convert_from_pool(ExprID id, int timestep = -1);

private:
    // Recursive helper for ExprID-based conversion
    Term convert_node_pool(ExprID id);
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
    Term handle_uninterpreted_function(const std::string& name, const std::vector<Term>& args, const cvc5::Sort& return_sort);
    Term handle_count(const std::vector<Term>& args);
};

} // namespace rantanplan
