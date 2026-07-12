#include "cnf_normalization_pass.hpp"
#include "../util/logger.hpp"
#include "../util/scoped_timer.hpp"
#include "../util/stats.hpp"
#include <algorithm>
#include <string>
#include <vector>

namespace rantanplan {

namespace {

/// A clause: literals in first-occurrence order, deduplicated.
using Clause = std::vector<ExprID>;

constexpr size_t kMinClauseBudget = 64;    ///< floor so small formulas may still distribute
constexpr size_t kGrowthFactor = 4;        ///< clauses allowed per input formula node
constexpr size_t kMaxClauseBudget = 4096;  ///< absolute cap regardless of input size

/// Bottom-up NNF + distribution over the ExprPool. Clause sets are plain
/// vectors of ExprID tuples; pool nodes are only built once per accepted
/// result, so rejected (over-budget) conversions cost no interning.
class CnfConverter {
public:
    explicit CnfConverter(ExprPool& pool) : pool_(pool) {
        // Harvest operator head symbols so rebuilt applications intern
        // identically to loader-built ones (same head ⇒ same ExprID).
        for (int32_t i = 0; i < static_cast<int32_t>(pool.size()); ++i) {
            ExprID id{i};
            if (!pool.is_function_symbol(id)) continue;
            switch (pool.op(id)) {
                case ExprOperator::AND: if (!and_head_.valid()) and_head_ = id; break;
                case ExprOperator::OR:  if (!or_head_.valid())  or_head_ = id;  break;
                case ExprOperator::NOT: if (!not_head_.valid()) not_head_ = id; break;
                default: break;
            }
        }
    }

    /// CNF-normalize expr; returns expr unchanged when it is already in CNF
    /// or when distribution would exceed the clause budget.
    ExprID normalize(ExprID expr) {
        if (!expr.valid() || is_cnf(expr)) return expr;
        size_t budget = std::min(kMaxClauseBudget,
                                 std::max(kMinClauseBudget, kGrowthFactor * formula_size(expr)));
        std::vector<Clause> clauses;
        if (!to_clauses(expr, false, budget, clauses)) {
            ++bailed_;
            return expr;
        }
        ++rewritten_;
        return build(clauses, pool_.type_id(expr));
    }

    size_t rewritten() const { return rewritten_; }
    size_t bailed() const { return bailed_; }

private:
    // ── Structural predicates ────────────────────────────────────────────

    bool is_connective(ExprID e) const {
        if (!pool_.is_function_application(e)) return false;
        switch (pool_.op(e)) {
            case ExprOperator::AND:
            case ExprOperator::OR:
            case ExprOperator::NOT:
            case ExprOperator::IMPLIES:
            case ExprOperator::IFF:
                return true;
            default:
                return false;
        }
    }

    bool is_not_app(ExprID e) const {
        return pool_.is_function_application(e) && pool_.is_not(e) &&
               pool_.argument_count(e) == 1;
    }

    bool is_literal(ExprID e) const {
        if (is_not_app(e)) return !is_connective(pool_.argument(e, 0));
        return !is_connective(e);
    }

    bool is_clause_expr(ExprID e) const {
        if (is_literal(e)) return true;
        if (pool_.is_function_application(e) && pool_.is_or(e)) {
            for (ExprID a : pool_.arguments(e))
                if (!is_literal(a)) return false;
            return true;
        }
        return false;
    }

    bool is_cnf(ExprID e) const {
        if (is_clause_expr(e)) return true;
        if (pool_.is_function_application(e) && pool_.is_and(e)) {
            for (ExprID a : pool_.arguments(e))
                if (!is_clause_expr(a)) return false;
            return true;
        }
        return false;
    }

    size_t formula_size(ExprID e) const {
        if (!pool_.is_function_application(e)) return 1;
        size_t n = 1;
        for (ExprID a : pool_.arguments(e)) n += formula_size(a);
        return n;
    }

    // ── Clause-set conversion (NNF folded into the recursion) ────────────

    /// Append the CNF of e (negated if requested) to out. Returns false if
    /// the clause budget would be exceeded. Non-connective subformulas
    /// (atoms, comparisons) are treated as opaque literals.
    bool to_clauses(ExprID e, bool negated, size_t budget, std::vector<Clause>& out) {
        if (pool_.is_true_constant(e) || pool_.is_false_constant(e)) {
            bool value = pool_.is_true_constant(e) != negated;
            if (!value) out.push_back(Clause{});  // false → one empty clause
            return out.size() <= budget;
        }

        if (pool_.is_function_application(e)) {
            ExprOperator op = pool_.op(e);

            if (op == ExprOperator::NOT && pool_.argument_count(e) == 1) {
                return to_clauses(pool_.argument(e, 0), !negated, budget, out);
            }

            // Copy arguments before recursing: interning may reallocate nodes_.
            if (op == ExprOperator::AND || op == ExprOperator::OR) {
                auto span = pool_.arguments(e);
                std::vector<ExprID> args(span.begin(), span.end());
                bool and_like = (op == ExprOperator::AND) != negated;
                if (and_like) {
                    // Child polarity is unchanged: ¬(a ∨ b) = ¬a ∧ ¬b
                    for (ExprID a : args) {
                        if (!to_clauses(a, negated, budget, out)) return false;
                        if (out.size() > budget) return false;
                    }
                    return true;
                }
                std::vector<OrPart> parts;
                parts.reserve(args.size());
                for (ExprID a : args) parts.push_back({a, negated});
                return or_clauses(parts, budget, out);
            }

            if (op == ExprOperator::IMPLIES && pool_.argument_count(e) == 2) {
                ExprID a = pool_.argument(e, 0);
                ExprID b = pool_.argument(e, 1);
                if (!negated) {
                    // a → b  ≡  ¬a ∨ b
                    return or_clauses({{a, true}, {b, false}}, budget, out);
                }
                // ¬(a → b)  ≡  a ∧ ¬b
                return to_clauses(a, false, budget, out) &&
                       to_clauses(b, true, budget, out) && out.size() <= budget;
            }

            if (op == ExprOperator::IFF && pool_.argument_count(e) == 2) {
                ExprID a = pool_.argument(e, 0);
                ExprID b = pool_.argument(e, 1);
                // a ↔ b ≡ (¬a ∨ b) ∧ (a ∨ ¬b); negated: (a ∨ b) ∧ (¬a ∨ ¬b)
                return or_clauses({{a, !negated}, {b, false}}, budget, out) &&
                       or_clauses({{a, negated}, {b, true}}, budget, out) &&
                       out.size() <= budget;
            }
        }

        // Atom (state variable, comparison, unknown op, …) → unit clause
        out.push_back(Clause{negated ? make_not(e) : e});
        return out.size() <= budget;
    }

    struct OrPart {
        ExprID expr;
        bool negated;
    };

    /// Append the CNF of the disjunction of parts to out: the cartesian
    /// product of the parts' clause sets, with tautological clauses dropped
    /// and literals deduplicated per clause.
    bool or_clauses(const std::vector<OrPart>& parts, size_t budget, std::vector<Clause>& out) {
        std::vector<Clause> product{Clause{}};
        for (const auto& part : parts) {
            std::vector<Clause> sub;
            if (!to_clauses(part.expr, part.negated, budget, sub)) return false;
            if (sub.empty()) return true;  // true disjunct → whole OR is true
            if (product.size() * sub.size() > budget) return false;
            std::vector<Clause> next;
            next.reserve(product.size() * sub.size());
            for (const auto& base : product) {
                for (const auto& cl : sub) {
                    Clause merged = base;
                    bool tautology = false;
                    for (ExprID lit : cl) {
                        if (!add_literal(merged, lit)) { tautology = true; break; }
                    }
                    if (!tautology) next.push_back(std::move(merged));
                }
            }
            product = std::move(next);
            // All clauses tautological ⇒ the disjunction so far is true, and
            // OR-ing further parts keeps it true: the empty product persists.
        }
        if (out.size() + product.size() > budget) return false;
        for (auto& cl : product) out.push_back(std::move(cl));
        return true;
    }

    /// Add lit to clause unless already present; returns false when the
    /// clause becomes tautological (contains lit's complement).
    bool add_literal(Clause& clause, ExprID lit) {
        ExprID comp = is_not_app(lit) ? pool_.argument(lit, 0) : make_not(lit);
        for (ExprID existing : clause) {
            if (existing == comp) return false;
            if (existing == lit) return true;
        }
        clause.push_back(lit);
        return true;
    }

    // ── Rebuilding pool nodes ────────────────────────────────────────────

    ExprID build(const std::vector<Clause>& clauses, int root_type_id) {
        if (clauses.empty()) return pool_.intern_bool_constant(true);
        std::vector<ExprID> parts;
        parts.reserve(clauses.size());
        for (const auto& clause : clauses) {
            if (clause.empty()) return pool_.intern_bool_constant(false);
            parts.push_back(clause.size() == 1
                                ? clause[0]
                                : make_app(ExprOperator::OR, or_head(), clause, -1));
        }
        if (parts.size() == 1) return parts[0];
        return make_app(ExprOperator::AND, and_head(), parts, root_type_id);
    }

    ExprID make_not(ExprID e) {
        return make_app(ExprOperator::NOT, not_head(), {e}, -1);
    }

    ExprID make_app(ExprOperator op, ExprID head, const std::vector<ExprID>& args, int type_id) {
        ExprNode node;
        node.kind = static_cast<int>(ExprKind::FUNCTION_APPLICATION);
        node.op = static_cast<int>(op);
        node.type_id = type_id;
        node.children.reserve(args.size() + 1);
        node.children.push_back(head);
        for (ExprID a : args) node.children.push_back(a);
        return pool_.intern(std::move(node));
    }

    ExprID and_head() { return op_head(and_head_, ExprOperator::AND, "up:and"); }
    ExprID or_head() { return op_head(or_head_, ExprOperator::OR, "up:or"); }
    ExprID not_head() { return op_head(not_head_, ExprOperator::NOT, "up:not"); }

    ExprID op_head(ExprID& cache, ExprOperator op, const char* symbol) {
        if (!cache.valid()) {
            ExprNode node;
            node.kind = static_cast<int>(ExprKind::FUNCTION_SYMBOL);
            node.op = static_cast<int>(op);
            node.payload = std::string(symbol);
            cache = pool_.intern(std::move(node));
        }
        return cache;
    }

    ExprPool& pool_;
    ExprID and_head_ = EXPR_NULL;
    ExprID or_head_ = EXPR_NULL;
    ExprID not_head_ = EXPR_NULL;
    size_t rewritten_ = 0;
    size_t bailed_ = 0;
};

}  // namespace

void CnfNormalizationPass::apply(PreprocessedProblem& result) const {
    ScopedTimer timer("cnf_normalization.time_ms");
    const auto& problem = result.problem;
    ExprPool& pool = *problem.pool_ptr();
    CnfConverter converter(pool);

    bool any_changed = false;

    std::vector<Action> new_actions;
    new_actions.reserve(problem.actions().size());
    for (const auto& action : problem.actions()) {
        Action new_action = action;

        if (new_action.has_precondition()) {
            ExprID new_pre = converter.normalize(action.precondition_id());
            if (new_pre != action.precondition_id()) {
                new_action.set_precondition_id(new_pre);
                any_changed = true;
            }
        }

        for (auto& effect : new_action.mutable_effects()) {
            const auto& ee = effect.effect_expression();
            if (!ee.is_conditional()) continue;
            ExprID new_cond = converter.normalize(ee.condition_id());
            if (new_cond != ee.condition_id()) {
                EffectExpression new_ee(ee.kind(), ee.fluent_id(), ee.value_id(), new_cond,
                                        ee.forall_variable_ids(), &pool);
                effect.set_effect_expression(new_ee);
                any_changed = true;
            }
        }

        new_actions.push_back(std::move(new_action));
    }

    std::vector<Goal> new_goals;
    new_goals.reserve(problem.goals().size());
    for (const auto& goal : problem.goals()) {
        Goal new_goal = goal;
        ExprID new_gid = converter.normalize(goal.goal_id());
        if (new_gid != goal.goal_id()) {
            new_goal.set_goal_id(new_gid);
            any_changed = true;
        }
        new_goals.push_back(std::move(new_goal));
    }

    Stats::instance().set("cnf_normalization.rewritten",
                          static_cast<double>(converter.rewritten()));
    Stats::instance().set("cnf_normalization.bailed",
                          static_cast<double>(converter.bailed()));

    if (!any_changed) {
        Logger::instance().component(VerbosityLevel::INFO, name(), {
            {"status", "all conditions already in CNF"}
        });
        return;
    }

    result.problem = problem.with_simplified(
        std::move(new_actions), std::move(new_goals),
        std::vector<Assignment>(problem.initial_state()),
        std::vector<ExprID>(problem.state_invariants()));

    Logger::instance().component(VerbosityLevel::INFO, name(), {
        {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
        {"rewritten", std::to_string(converter.rewritten())},
        {"over budget", std::to_string(converter.bailed())}
    });
}

} // namespace rantanplan
