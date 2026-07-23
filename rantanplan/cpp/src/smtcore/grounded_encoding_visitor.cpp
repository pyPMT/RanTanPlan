#include "grounded_encoding_visitor.hpp"
#include "smt_variable_factory.hpp"
#include "../problem/fluent.hpp"
#include "../problem/effect_expression.hpp"
#include <optional>
#include <stdexcept>

namespace rantanplan {

GroundedEncodingVisitor::GroundedEncodingVisitor(cvc5::TermManager& ctx,
                                                 const Problem* problem,
                                                 SmtVariableFactory* factory)
    : ctx_(ctx), problem_(problem), current_timestep_(-1),
      variable_factory_(factory) {}

// Arithmetic and logical operation handlers
Term GroundedEncodingVisitor::handle_and(const std::vector<Term>& args) {
    if (args.empty()) {
        return tb().bool_val(true);
    }

    TermVec smt_args;
    for (const auto& arg : args) {
        smt_args.push_back(arg);
    }
    return tb().mk_and(smt_args);
}

Term GroundedEncodingVisitor::handle_or(const std::vector<Term>& args) {
    if (args.empty()) {
        return tb().bool_val(false);
    }

    TermVec smt_args;
    for (const auto& arg : args) {
        smt_args.push_back(arg);
    }
    return tb().mk_or(smt_args);
}

Term GroundedEncodingVisitor::handle_not(const std::vector<Term>& args) {
    if (args.size() != 1) {
        throw std::runtime_error("'not' operation expects exactly 1 argument, got " + std::to_string(args.size()));
    }
    return tb().mk_not(args[0]);
}

Term GroundedEncodingVisitor::handle_equals(const std::vector<Term>& args) {
    if (args.size() != 2) {
        throw std::runtime_error("'=' operation expects exactly 2 arguments, got " + std::to_string(args.size()));
    }
    return tb().mk_eq(args[0], args[1]);
}

Term GroundedEncodingVisitor::handle_less_than(const std::vector<Term>& args) {
    if (args.size() != 2) {
        throw std::runtime_error("'<' operation expects exactly 2 arguments, got " + std::to_string(args.size()));
    }
    return tb().lt(args[0], args[1]);
}

Term GroundedEncodingVisitor::handle_less_equal(const std::vector<Term>& args) {
    if (args.size() != 2) {
        throw std::runtime_error("'<=' operation expects exactly 2 arguments, got " + std::to_string(args.size()));
    }
    return tb().leq(args[0], args[1]);
}

Term GroundedEncodingVisitor::handle_greater_than(const std::vector<Term>& args) {
    if (args.size() != 2) {
        throw std::runtime_error("'>' operation expects exactly 2 arguments, got " + std::to_string(args.size()));
    }
    return tb().gt(args[0], args[1]);
}

Term GroundedEncodingVisitor::handle_greater_equal(const std::vector<Term>& args) {
    if (args.size() != 2) {
        throw std::runtime_error("'>=' operation expects exactly 2 arguments, got " + std::to_string(args.size()));
    }
    return tb().geq(args[0], args[1]);
}

Term GroundedEncodingVisitor::handle_plus(const std::vector<Term>& args) {
    if (args.empty()) {
        return tb().int_val(0);
    }

    Term result = args[0];
    for (size_t i = 1; i < args.size(); ++i) {
        result = tb().add(result, args[i]);
    }
    return result;
}

Term GroundedEncodingVisitor::handle_minus(const std::vector<Term>& args) {
    if (args.empty()) {
        throw std::runtime_error("'-' operation expects at least 1 argument");
    }

    if (args.size() == 1) {
        return tb().neg(args[0]);
    } else if (args.size() == 2) {
        return tb().sub(args[0], args[1]);
    } else {
        throw std::runtime_error("'-' operation expects 1 or 2 arguments, got " + std::to_string(args.size()));
    }
}

Term GroundedEncodingVisitor::handle_multiply(const std::vector<Term>& args) {
    if (args.empty()) {
        return tb().int_val(1);
    }

    Term result = args[0];
    for (size_t i = 1; i < args.size(); ++i) {
        result = tb().mul(result, args[i]);
    }
    return result;
}

Term GroundedEncodingVisitor::handle_divide(const std::vector<Term>& args) {
    if (args.size() != 2) {
        throw std::runtime_error("'/' operation expects exactly 2 arguments, got " + std::to_string(args.size()));
    }
    return tb().div(args[0], args[1]);
}

Term GroundedEncodingVisitor::handle_implies(const std::vector<Term>& args) {
    if (args.size() != 2) {
        throw std::runtime_error("'implies' operation expects exactly 2 arguments, got " + std::to_string(args.size()));
    }
    return tb().mk_implies(args[0], args[1]);
}

Term GroundedEncodingVisitor::handle_count(const std::vector<Term>& args) {
    // Fallback for counts in arbitrary arithmetic contexts: sum of
    // ite(b, 1, 0). Comparisons against integer constants never reach
    // this — convert_node intercepts them into clausal cardinality
    // constraints.
    // No simplification pass runs here (in cvc5 that is a Solver method and
    // no Solver is in scope); TermBuilder's constant folding already collapses
    // the trivial cases.
    Term one = variable_factory_->make_numeric_val(static_cast<int64_t>(1));
    Term zero = variable_factory_->make_numeric_val(static_cast<int64_t>(0));
    Term result = zero;
    for (const auto& arg : args) {
        result = tb().add(result, tb().mk_ite(arg, one, zero));
    }
    return result;
}

Term GroundedEncodingVisitor::encode_count_comparison(ExprID count_id,
                                                          ExprOperator op,
                                                          int64_t constant,
                                                          bool count_on_left) {
    const ExprPool& pool = problem_->pool();

    // Convert the count's boolean arguments at the current timestep.
    TermVec bools;
    for (ExprID arg : pool.arguments(count_id)) {
        bools.push_back(convert_node(arg));
    }
    const int64_t n = static_cast<int64_t>(bools.size());

    // Normalize to bounds on the number of true arguments:
    // min_true <= |{true args}| <= max_true (either side optional).
    std::optional<int64_t> min_true, max_true;
    if (count_on_left) {
        switch (op) {
            case ExprOperator::LESS_EQUAL:    max_true = constant; break;
            case ExprOperator::LESS_THAN:     max_true = constant - 1; break;
            case ExprOperator::GREATER_EQUAL: min_true = constant; break;
            case ExprOperator::GREATER_THAN:  min_true = constant + 1; break;
            case ExprOperator::EQUALS:        min_true = max_true = constant; break;
            default:
                throw std::runtime_error("encode_count_comparison: unexpected operator");
        }
    } else {  // constant op Count(...)
        switch (op) {
            case ExprOperator::LESS_EQUAL:    min_true = constant; break;
            case ExprOperator::LESS_THAN:     min_true = constant + 1; break;
            case ExprOperator::GREATER_EQUAL: max_true = constant; break;
            case ExprOperator::GREATER_THAN:  max_true = constant - 1; break;
            case ExprOperator::EQUALS:        min_true = max_true = constant; break;
            default:
                throw std::runtime_error("encode_count_comparison: unexpected operator");
        }
    }

    // Note this result is a subterm of an arbitrary formula -- convert_from_pool
    // reaches it from preconditions, goals and conditional-effect conditions,
    // any of which may sit under a negation. Both cardinality arms are logical
    // equivalences with no auxiliary variables, so that is safe; an
    // equisatisfiable counter encoding would not be.
    TermVec parts;
    if (min_true) {
        if (*min_true > n) return tb().bool_val(false);
        if (*min_true > 0) {
            parts.push_back(tb().mk_atleast(bools, static_cast<unsigned>(*min_true)));
        }
    }
    if (max_true) {
        if (*max_true < 0) return tb().bool_val(false);
        if (*max_true < n) {
            parts.push_back(tb().mk_atmost(bools, static_cast<unsigned>(*max_true)));
        }
    }
    if (parts.empty()) return tb().bool_val(true);
    return parts.size() == 1 ? parts[0] : tb().mk_and(parts);
}

// ============================================================================
// ExprID-based conversion: walks ExprNode directly via ExprPool
// ============================================================================

Term GroundedEncodingVisitor::convert_from_pool(ExprID id, int timestep) {
    if (!id.valid()) {
        throw std::invalid_argument("convert_from_pool called with invalid ExprID");
    }

    int saved_timestep = current_timestep_;
    if (timestep >= 0) {
        current_timestep_ = timestep;
    }

    auto result = convert_node(id);

    current_timestep_ = saved_timestep;
    return result;
}

Term GroundedEncodingVisitor::convert_node(ExprID id) {
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
            // Object constants must be encoded as concrete numeric values
            // (not free SMT variables) to preserve distinct-object semantics
            // in equality comparisons like (= (location ?p) city0).
            if (problem_) {
                int idx = problem_->object_constant_index(id);
                if (idx >= 0) {
                    return variable_factory_->make_numeric_val(static_cast<int64_t>(idx));
                }
            }
            return variable_factory_->create_symbol_variable(symbol, type);
        }
        if (std::holds_alternative<int64_t>(node.payload)) {
            return variable_factory_->make_numeric_val(std::get<int64_t>(node.payload));
        }
        if (std::holds_alternative<double>(node.payload)) {
            return variable_factory_->make_numeric_val(std::get<double>(node.payload));
        }
        if (std::holds_alternative<bool>(node.payload)) {
            return tb().bool_val(std::get<bool>(node.payload));
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

        // Find fluent definition
        const Fluent* fluent_def = nullptr;
        for (const auto& fluent : problem_->fluents()) {
            if (fluent.name() == fluent_name) {
                fluent_def = &fluent;
                break;
            }
        }
        if (!fluent_def) {
            throw std::runtime_error("Fluent definition not found for: " + fluent_name);
        }

        // Build grounded parameters from argument children
        std::vector<Parameter> grounded_params;
        grounded_params.reserve(pool.argument_count(id));
        size_t arg_index = 0;
        for (ExprID arg_id : pool.arguments(id)) {
            const ExprNode& arg = pool.get(arg_id);
            if (!std::holds_alternative<std::string>(arg.payload)) {
                throw std::runtime_error(
                    "Function composition (nested fluent terms) is not supported. "
                    "Fluent '" + fluent_name + "' has a non-constant argument "
                    "(another fluent application). Rewrite the domain using "
                    "auxiliary action parameters or boolean predicates.");
            }
            const Type* param_type = nullptr;
            if (arg_index < fluent_def->parameters().size()) {
                param_type = fluent_def->parameters()[arg_index].type();
            }
            if (!param_type) {
                param_type = problem_->find_type("object");
            }
            grounded_params.emplace_back(std::get<std::string>(arg.payload), param_type);
            ++arg_index;
        }

        Fluent grounded_fluent(fluent_def->name(), fluent_def->value_type(), grounded_params);
        int timestep = (current_timestep_ >= 0) ? current_timestep_ : 0;
        // `id` IS the ground fluent's interned ExprID, which is what the
        // problem's ground-fluent registry is keyed by. The factory needs it to
        // index the PLAN_FLUENT atom by the problem's own dense ground-fluent
        // id instead of inventing a second numbering space.
        return variable_factory_->get_fluent_variable(grounded_fluent, timestep, id);
    }

    // ---- Function application ----
    if (kind == ExprKind::FUNCTION_APPLICATION) {
        auto op = static_cast<ExprOperator>(node.op);

        // Cardinality fast path: a comparison between a Count and an
        // integer constant becomes a clausal cardinality constraint
        // (atmost/atleast) instead of an ite-sum comparison.
        if (is_comparison_operator(op) && pool.argument_count(id) == 2) {
            ExprID lhs = pool.argument(id, 0);
            ExprID rhs = pool.argument(id, 1);
            auto is_count_app = [&](ExprID e) {
                return pool.is_function_application(e) && pool.is_count(e);
            };
            auto int_constant = [&](ExprID e) -> std::optional<int64_t> {
                const ExprNode& n = pool.get(e);
                if (!n.children.empty()) return std::nullopt;
                if (std::holds_alternative<int64_t>(n.payload)) {
                    return std::get<int64_t>(n.payload);
                }
                if (std::holds_alternative<double>(n.payload)) {
                    double d = std::get<double>(n.payload);
                    if (d == static_cast<int64_t>(d)) return static_cast<int64_t>(d);
                }
                return std::nullopt;
            };
            if (is_count_app(lhs)) {
                if (auto k = int_constant(rhs)) {
                    return encode_count_comparison(lhs, op, *k, true);
                }
            } else if (is_count_app(rhs)) {
                if (auto k = int_constant(lhs)) {
                    return encode_count_comparison(rhs, op, *k, false);
                }
            }
        }

        // Recursively convert arguments
        std::vector<Term> smt_args;
        smt_args.reserve(pool.argument_count(id));
        for (ExprID arg : pool.arguments(id)) {
            smt_args.push_back(convert_node(arg));
        }

        // Dispatch to existing handlers
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
            case ExprOperator::IMPLIES:       return handle_implies(smt_args);
            case ExprOperator::COUNT:         return handle_count(smt_args);
            default:
                throw std::runtime_error("Unsupported operator in ExprID conversion: " + std::to_string(static_cast<int>(op)));
        }
    }

    throw std::runtime_error("Unhandled ExprNode kind in convert_node: " + std::to_string(node.kind));
}

} // namespace rantanplan
