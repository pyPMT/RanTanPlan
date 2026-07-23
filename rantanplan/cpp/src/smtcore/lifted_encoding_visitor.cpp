#include "lifted_encoding_visitor.hpp"
#include <stdexcept>

namespace rantanplan {

// Helper methods for handling specific operators
Term LiftedEncodingVisitor::handle_and(const std::vector<Term>& args) {
    if (args.empty()) {
        return tb_.bool_val(true);
    }

    TermVec smt_args;
    for (const auto& arg : args) {
        smt_args.push_back(arg);
    }
    return tb_.mk_and(smt_args);
}

Term LiftedEncodingVisitor::handle_or(const std::vector<Term>& args) {
    if (args.empty()) {
        return tb_.bool_val(false);
    }

    TermVec smt_args;
    for (const auto& arg : args) {
        smt_args.push_back(arg);
    }
    return tb_.mk_or(smt_args);
}

Term LiftedEncodingVisitor::handle_not(const std::vector<Term>& args) {
    if (args.size() != 1) {
        throw std::runtime_error("'not' operation expects exactly 1 argument, got " + std::to_string(args.size()));
    }
    return tb_.mk_not(args[0]);
}

Term LiftedEncodingVisitor::handle_equals(const std::vector<Term>& args) {
    if (args.size() != 2) {
        throw std::runtime_error("'=' operation expects exactly 2 arguments, got " + std::to_string(args.size()));
    }
    return tb_.mk_eq(args[0], args[1]);
}

Term LiftedEncodingVisitor::handle_less_than(const std::vector<Term>& args) {
    if (args.size() != 2) {
        throw std::runtime_error("'<' operation expects exactly 2 arguments, got " + std::to_string(args.size()));
    }
    return tb_.lt(args[0], args[1]);
}

Term LiftedEncodingVisitor::handle_less_equal(const std::vector<Term>& args) {
    if (args.size() != 2) {
        throw std::runtime_error("'<=' operation expects exactly 2 arguments, got " + std::to_string(args.size()));
    }
    return tb_.leq(args[0], args[1]);
}

Term LiftedEncodingVisitor::handle_greater_than(const std::vector<Term>& args) {
    if (args.size() != 2) {
        throw std::runtime_error("'>' operation expects exactly 2 arguments, got " + std::to_string(args.size()));
    }
    return tb_.gt(args[0], args[1]);
}

Term LiftedEncodingVisitor::handle_greater_equal(const std::vector<Term>& args) {
    if (args.size() != 2) {
        throw std::runtime_error("'>=' operation expects exactly 2 arguments, got " + std::to_string(args.size()));
    }
    return tb_.geq(args[0], args[1]);
}

Term LiftedEncodingVisitor::handle_plus(const std::vector<Term>& args) {
    if (args.empty()) {
        return tb_.int_val(0);
    }

    Term result = args[0];
    for (size_t i = 1; i < args.size(); ++i) {
        result = tb_.add(result, args[i]);
    }
    return result;
}

Term LiftedEncodingVisitor::handle_minus(const std::vector<Term>& args) {
    if (args.size() == 1) {
        return tb_.neg(args[0]);
    } else if (args.size() == 2) {
        return tb_.sub(args[0], args[1]);
    }
    throw std::runtime_error("'-' operation expects 1 or 2 arguments, got " + std::to_string(args.size()));
}

Term LiftedEncodingVisitor::handle_multiply(const std::vector<Term>& args) {
    if (args.empty()) {
        return tb_.int_val(1);
    }

    Term result = args[0];
    for (size_t i = 1; i < args.size(); ++i) {
        result = tb_.mul(result, args[i]);
    }
    return result;
}

Term LiftedEncodingVisitor::handle_divide(const std::vector<Term>& args) {
    if (args.size() != 2) {
        throw std::runtime_error("'/' operation expects exactly 2 arguments, got " + std::to_string(args.size()));
    }
    return tb_.div(args[0], args[1]);
}

Term LiftedEncodingVisitor::handle_count(const std::vector<Term>& args) {
    // Count of boolean conditions as a sum of ite(b, 1, 0). No simplification
    // pass runs here (in cvc5 that is a Solver method and no Solver is in
    // scope); TermBuilder's constant folding already collapses the trivial
    // cases.
    Term result = tb_.int_val(0);
    for (const auto& arg : args) {
        result = tb_.add(result, tb_.mk_ite(arg, tb_.int_val(1), tb_.int_val(0)));
    }
    return result;
}

Term LiftedEncodingVisitor::handle_uninterpreted_function(
    const std::string& function_name,
    const std::vector<Term>& args,
    const cvc5::Sort& return_sort) {

    // Get-or-create through the interned SymbolTable -- the identity
    // mechanism (see its class comment), not an optimization.
    std::vector<cvc5::Sort> domain;
    domain.reserve(args.size());
    for (const auto& arg : args) {
        domain.push_back(arg.getSort());
    }
    Term func_decl =
        symbol_table_.intern_function(ctx_, function_name, domain, return_sort);
    if (!func_decl.getSort().isFunction()) {
        // A 0-ary "function" is a plain constant -- or, indistinguishably, a
        // variable cached under the same name; see
        // SymbolTable::intern_function for why it is returned as-is.
        return func_decl;
    }

    // Apply function: in cvc5 the function symbol is the first child of an
    // APPLY_UF term rather than a separate func_decl object.
    TermVec smt_args;
    smt_args.push_back(func_decl);
    for (const auto& arg : args) {
        smt_args.push_back(arg);
    }

    return ctx_.mkTerm(cvc5::Kind::APPLY_UF, smt_args);
}

// Interning (in SymbolTable::intern_var) is what makes two occurrences of
// `name` the same variable: mkConst mints a fresh constant on every call.

Term LiftedEncodingVisitor::create_bool_variable(const std::string& name) {
    return symbol_table_.intern_var(ctx_, name, ctx_.getBooleanSort());
}

Term LiftedEncodingVisitor::create_int_variable(const std::string& name) {
    return symbol_table_.intern_var(ctx_, name, ctx_.getIntegerSort());
}

Term LiftedEncodingVisitor::create_real_variable(const std::string& name) {
    return symbol_table_.intern_var(ctx_, name, ctx_.getRealSort());
}

// ============================================================================
// ExprID-based conversion: walks ExprNode directly via ExprPool
// ============================================================================

Term LiftedEncodingVisitor::convert_from_pool(ExprID id, int timestep) {
    if (!id.valid()) {
        throw std::invalid_argument("convert_from_pool called with invalid ExprID");
    }

    int saved_timestep = current_timestep_;
    if (timestep >= 0) {
        current_timestep_ = timestep;
    }

    auto result = convert_node_pool(id);

    current_timestep_ = saved_timestep;
    return result;
}

Term LiftedEncodingVisitor::convert_node_pool(ExprID id) {
    const ExprPool& pool = problem_->pool();
    const ExprNode& node = pool.get(id);
    auto kind = static_cast<ExprKind>(node.kind);

    // ---- Leaf nodes (no children) ----
    if (node.children.empty()) {
        if (std::holds_alternative<std::string>(node.payload)) {
            const std::string& symbol = std::get<std::string>(node.payload);
            // Resolve type from type_id
            const Type* type = nullptr;
            if (node.type_id >= 0 && node.type_id < static_cast<int>(problem_->types().size())) {
                type = &problem_->types()[node.type_id];
            }
            // Use type-based variable creation (same as visit_symbol)
            if (!type) {
                return create_int_variable(symbol);
            }
            if (type->is_bool()) return create_bool_variable(symbol);
            if (type->is_int()) return create_int_variable(symbol);
            if (type->is_real()) return create_real_variable(symbol);
            // PDDL objects - use integer
            return create_int_variable(symbol);
        }
        if (std::holds_alternative<int64_t>(node.payload)) {
            return tb_.int_val(std::get<int64_t>(node.payload));
        }
        if (std::holds_alternative<double>(node.payload)) {
            return ctx_.mkReal(std::to_string(std::get<double>(node.payload)));
        }
        if (std::holds_alternative<bool>(node.payload)) {
            return tb_.bool_val(std::get<bool>(node.payload));
        }
        throw std::runtime_error("Unhandled payload type in leaf node (ExprID " + std::to_string(id.id) + ")");
    }

    // ---- State variable (fluent application) ----
    if (kind == ExprKind::STATE_VARIABLE) {
        const ExprNode& fluent_sym = pool.head_symbol_node(id);
        if (!std::holds_alternative<std::string>(fluent_sym.payload)) {
            throw std::runtime_error("STATE_VARIABLE first child is not a symbol");
        }
        const std::string& fluent_name = std::get<std::string>(fluent_sym.payload);

        // Convert argument children to terms
        std::vector<Term> smt_args;
        smt_args.reserve(pool.argument_count(id));
        for (ExprID arg : pool.arguments(id)) {
            smt_args.push_back(convert_node_pool(arg));
        }

        // Add timestep as final argument if temporal encoding is enabled
        if (current_timestep_ >= 0) {
            smt_args.push_back(tb_.int_val(current_timestep_));
        }

        // Determine return type based on fluent definition
        cvc5::Sort return_sort = ctx_.getIntegerSort();
        if (problem_) {
            const Fluent* fluent_def = problem_->find_fluent(fluent_name);
            if (fluent_def && fluent_def->is_predicate()) {
                return_sort = ctx_.getBooleanSort();
            }
        }

        return handle_uninterpreted_function(fluent_name, smt_args, return_sort);
    }

    // ---- Function application ----
    if (kind == ExprKind::FUNCTION_APPLICATION) {
        auto op = static_cast<ExprOperator>(node.op);

        // Recursively convert arguments
        std::vector<Term> smt_args;
        smt_args.reserve(pool.argument_count(id));
        for (ExprID arg : pool.arguments(id)) {
            smt_args.push_back(convert_node_pool(arg));
        }

        switch (op) {
            case ExprOperator::AND:           return handle_and(smt_args);
            case ExprOperator::OR:            return handle_or(smt_args);
            case ExprOperator::NOT:           return handle_not(smt_args);
            case ExprOperator::EQUALS:        return handle_equals(smt_args);
            case ExprOperator::LESS_THAN:     return handle_less_than(smt_args);
            case ExprOperator::LESS_EQUAL:    return handle_less_equal(smt_args);
            case ExprOperator::GREATER_THAN:  return handle_greater_than(smt_args);
            case ExprOperator::GREATER_EQUAL: return handle_greater_equal(smt_args);
            case ExprOperator::PLUS:          return handle_plus(smt_args);
            case ExprOperator::MINUS:         return handle_minus(smt_args);
            case ExprOperator::MULTIPLY:      return handle_multiply(smt_args);
            case ExprOperator::DIVIDE:        return handle_divide(smt_args);
            case ExprOperator::COUNT:         return handle_count(smt_args);
            default: {
                // Get function name from head symbol
                const ExprNode& func_sym = pool.head_symbol_node(id);
                std::string func_name = std::holds_alternative<std::string>(func_sym.payload)
                    ? std::get<std::string>(func_sym.payload)
                    : "unknown_func";
                return handle_uninterpreted_function(func_name, smt_args, ctx_.getIntegerSort());
            }
        }
    }

    throw std::runtime_error("Unhandled ExprNode kind in convert_node: " + std::to_string(node.kind));
}

} // namespace rantanplan
