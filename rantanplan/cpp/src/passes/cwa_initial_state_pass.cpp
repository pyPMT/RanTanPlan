#include "cwa_initial_state_pass.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"
#include "../util/unsupported_problem_error.hpp"
#include <stdexcept>
#include <unordered_set>

namespace rantanplan {

void CWAInitialStatePass::apply(PreprocessedProblem& result) const {
    ScopedTimer timer;  // measuring-only; the pipeline records the stats key

    auto& problem = result.problem;
    auto pool = problem.pool_ptr();

    // Build set of fluent ExprIDs that already have an initial assignment.
    std::unordered_set<ExprID> assigned;
    for (const auto& assignment : problem.initial_state()) {
        assigned.insert(assignment.fluent_id());
    }

    // Fill every grounded fluent missing from the initial state. This is the
    // sole materialiser of default initial values: a declared
    // default_initial_value is authoritative (including an
    // object/UserType default, which the ObjectFluentIndex has already seeded
    // for the grounder); otherwise a boolean is closed-world false and a numeric
    // is 0 clamped into its declared bounds. An object fluent with neither an
    // explicit value nor a declared default is a hard error.
    std::vector<Assignment> defaults;
    for (ExprID eid : problem.grounded_fluents()) {
        if (assigned.count(eid)) continue;

        const Type* type = problem.type_for_id(eid);
        if (!type) continue;  // unknown type — skip

        // Resolve the fluent schema (via its head symbol) for the declared
        // default and bounds; array cells resolve to their cell fluent.
        const Fluent* fluent = nullptr;
        if (pool->is_state_variable(eid)) {
            ExprID head = pool->head_symbol_id(eid);
            if (pool->is_fluent_symbol(head)) {
                fluent = problem.find_fluent(pool->payload_string(head));
            }
        }

        // A declared default_initial_value is authoritative (a numeric value,
        // boolean false, or an object constant; boolean-true defaults are
        // rejected upstream and never reach here).
        if (fluent && fluent->declared_default()) {
            defaults.emplace_back(eid, *fluent->declared_default(), &problem.pool());
            continue;
        }

        // This hard error is what makes object fluents *total* functions over
        // a finite universe (equality-only semantics, encoded as ints in
        // z3_variable_factory.cpp): every reachable instance provably holds
        // some object at every step. Value-expansion features (ghost-parameter
        // enumeration of a fluent's possible values) rely on this totality —
        // don't soften it to a default without revisiting them.
        if (type->is_object()) {
            throw UnsupportedProblemError(
                "Object fluent '" + pool->to_string(eid) + "' has no initial "
                "value and object fluents have no default; it must be "
                "initialized explicitly in the problem's initial state.");
        }

        // No declared default_initial_value: fall back to the implicit rule
        // (the branch above already handled — and took precedence for — any
        // UP-declared default). Booleans are closed-world false; numerics are 0
        // clamped into the declared bounds. Unlike the declared-default branch,
        // the value comes from this rule rather than from a UP node.
        ExprID value_id;
        if (type->is_bool()) {
            value_id = pool->intern_bool_constant(false);
        } else {
            // int/real: 0 clamped into the declared bounds — UP rejects an
            // out-of-range initial value for a bounded type.
            Rational v = Rational::of(0);
            if (fluent) {
                if (auto lo = fluent->declared_lower_bound(); lo && compare(v, *lo) < 0) v = *lo;
                if (auto hi = fluent->declared_upper_bound(); hi && compare(v, *hi) > 0) v = *hi;
            }
            // An int type's bounds are integers.
            value_id = type->is_int() ? pool->intern_int_constant(v.num)
                                      : pool->intern_real_constant(v);
        }
        defaults.emplace_back(eid, value_id, &problem.pool());
    }

    if (defaults.empty()) {
        Logger::instance().component(VerbosityLevel::VERBOSE, "CWA", {
            {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
            {"status", "all grounded fluents already assigned"}
        });
        return;
    }

    Logger::instance().component(VerbosityLevel::INFO, "CWA", {
        {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
        {"added", std::to_string(defaults.size()) + " default assignments"}
    });

    result.problem = problem.with_additional_initial_state(defaults);
}

} // namespace rantanplan
