#include "gse_classification.hpp"

#include "../util/stats.hpp"

#include <algorithm>

namespace rantanplan {

// ---------------------------------------------------------------------------
// Expression helpers
// ---------------------------------------------------------------------------

void gse_collect_reads(ExprID e, const ExprPool& pool,
                       std::unordered_set<ExprID>& out) {
    if (!e.valid()) return;
    if (pool.is_state_variable(e)) {
        out.insert(e);
        return;  // ground fluent args are constants — nothing more to read
    }
    for (ExprID c : pool.children(e)) gse_collect_reads(c, pool, out);
}

namespace {

void scan_rec(ExprID e, const Problem& problem, GseExprScan& s) {
    const ExprPool& pool = problem.pool();
    if (!e.valid()) return;
    ExprKind kind = pool.kind(e);

    if (pool.is_leaf(e)) {
        switch (kind) {
            case ExprKind::CONSTANT:
            case ExprKind::FLUENT_SYMBOL:
                return;
            case ExprKind::STATE_VARIABLE:
                return;  // a fluent read — fine
            case ExprKind::PARAMETER:
            case ExprKind::VARIABLE:
                // Should be gone after grounding; if seen, refuse to guess.
                if (s.supported) { s.supported = false; s.first_unsupported = "ungrounded parameter/variable"; }
                return;
            default:
                if (s.supported) { s.supported = false; s.first_unsupported = "unknown-leaf"; }
                return;
        }
    }

    if (kind == ExprKind::STATE_VARIABLE) return;  // ground fluent application

    if (kind != ExprKind::FUNCTION_APPLICATION) {
        if (s.supported) { s.supported = false; s.first_unsupported = "non-application node"; }
        return;
    }

    ExprOperator op = pool.op(e);
    switch (op) {
        case ExprOperator::PLUS:
        case ExprOperator::MINUS:
            break;  // affine-preserving
        case ExprOperator::MULTIPLY:
            if (pool.argument_count(e) == 2) {
                ValueKind lk = classify_value_kind(pool.argument(e, 0), pool);
                ValueKind rk = classify_value_kind(pool.argument(e, 1), pool);
                if (lk != ValueKind::CONSTANT && rk != ValueKind::CONSTANT) s.nonlinear = true;
            } else {
                s.nonlinear = true;
            }
            break;
        case ExprOperator::DIVIDE:
            if (pool.argument_count(e) == 2) {
                if (classify_value_kind(pool.argument(e, 1), pool) != ValueKind::CONSTANT)
                    s.nonlinear = true;
            } else {
                s.nonlinear = true;
            }
            break;
        case ExprOperator::EQUALS:
        case ExprOperator::LESS_EQUAL:
        case ExprOperator::LESS_THAN:
        case ExprOperator::GREATER_EQUAL:
        case ExprOperator::GREATER_THAN:
        case ExprOperator::AND:
        case ExprOperator::OR:
        case ExprOperator::NOT:
        case ExprOperator::IMPLIES:
            break;
        default:
            if (s.supported) {
                s.supported = false;
                std::string n = expr_operator_to_string(op);
                s.first_unsupported = n.empty() ? "unsupported-operator" : n;
            }
            break;
    }
    // Recurse over semantic arguments (children[1..]); children[0] is the head
    // FUNCTION_SYMBOL, which is not a value to translate.
    for (ExprID c : pool.arguments(e)) scan_rec(c, problem, s);
}

}  // namespace

GseExprScan gse_scan_expr(ExprID e, const Problem& problem) {
    GseExprScan s;
    scan_rec(e, problem, s);
    return s;
}

// ---------------------------------------------------------------------------
// Classification
// ---------------------------------------------------------------------------

GseClassification gse_classify(const Problem& problem) {
    GseClassification c;
    const ExprPool& pool = problem.pool();

    auto fluent_record = [&](ExprID f) -> GseFluentClass& {
        auto it = c.fluents.find(f);
        if (it != c.fluents.end()) return it->second;
        GseFluentClass fc;
        fc.fluent = f;
        fc.is_bool = problem.is_bool_type(f);
        fc.is_numeric = problem.is_numeric_type(f);
        fc.is_object = problem.is_object_type(f);
        return c.fluents.emplace(f, std::move(fc)).first->second;
    };

    auto note_scan = [&](const GseExprScan& s) {
        if (!s.supported && !c.any_unsupported_operator) {
            c.any_unsupported_operator = true;
            c.first_unsupported_operator = s.first_unsupported;
        }
        if (s.nonlinear) c.any_nonlinear = true;
    };

    auto add_read = [&](int aid, ExprID sv) {
        c.touch[aid][sv].read = true;
        fluent_record(sv).reader_actions.insert(aid);
    };

    for (const Action& a : problem.actions()) {
        int aid = a.id();

        if (a.has_precondition()) {
            note_scan(gse_scan_expr(a.precondition_id(), problem));
            std::unordered_set<ExprID> reads;
            gse_collect_reads(a.precondition_id(), pool, reads);
            for (ExprID sv : reads) {
                fluent_record(sv);
                c.action_reads[aid].push_back(sv);
                add_read(aid, sv);
            }
        }

        for (size_t ei = 0; ei < a.effects().size(); ++ei) {
            const EffectExpression& ee = a.effect(ei).effect_expression();
            if (ee.is_conditional()) c.any_conditional_effect = true;
            if (ee.is_quantified()) c.any_quantified_effect = true;

            ExprID f = ee.fluent_id();
            GseFluentClass& fc = fluent_record(f);
            ValueKind vk = ee.value_kind();
            if (vk == ValueKind::NONLINEAR) c.any_nonlinear = true;

            note_scan(gse_scan_expr(ee.value_id(), problem));

            // RHS reads make this action a reader of those fluents.
            std::unordered_set<ExprID> rhs_reads;
            gse_collect_reads(ee.value_id(), pool, rhs_reads);
            for (ExprID sv : rhs_reads) {
                fluent_record(sv);
                c.action_reads[aid].push_back(sv);
                add_read(aid, sv);
            }

            GseEffectRef ref;
            ref.action_id = aid;
            ref.effect_index = static_cast<int>(ei);
            ref.value_kind = vk;

            switch (ee.kind()) {
                case EffectExpression::Kind::ASSIGN:
                    fc.writers.push_back(ref);
                    c.touch[aid][f].write = true;
                    c.action_writes[aid].push_back(f);
                    if (vk != ValueKind::CONSTANT) c.any_dynamic_writer = true;
                    break;
                case EffectExpression::Kind::INCREASE:
                case EffectExpression::Kind::DECREASE:
                    ref.negated = (ee.kind() == EffectExpression::Kind::DECREASE);
                    fc.adds.push_back(ref);
                    c.touch[aid][f].contrib = true;
                    c.action_contribs[aid].push_back(f);
                    if (vk != ValueKind::CONSTANT) c.any_dynamic_amount = true;
                    if (rhs_reads.count(f)) fc.self_referential = true;
                    break;
            }
        }
    }

    // Goal reads.
    {
        std::unordered_set<ExprID> greads;
        for (const Goal& g : problem.goals()) {
            note_scan(gse_scan_expr(g.goal_id(), problem));
            gse_collect_reads(g.goal_id(), pool, greads);
        }
        for (ExprID sv : greads) {
            fluent_record(sv).read_by_goal = true;
            c.goal_reads.push_back(sv);
        }
    }

    // Derive categories.
    for (auto& [f, fc] : c.fluents) {
        bool has_writers = !fc.writers.empty();
        bool has_adds = !fc.adds.empty();
        if (!has_adds) fc.category = GseFluentCategory::AssignOnly;
        else if (!has_writers) fc.category = GseFluentCategory::PureAdditive;
        else fc.category = GseFluentCategory::Mixed;
    }

    return c;
}

// ---------------------------------------------------------------------------
// Interference (C1)
// ---------------------------------------------------------------------------

bool gse_actions_interfere(const GseClassification& c, int action_a, int action_b) {
    auto ita = c.touch.find(action_a);
    auto itb = c.touch.find(action_b);
    if (ita == c.touch.end() || itb == c.touch.end()) return false;
    const auto& ta = ita->second;
    const auto& tb = itb->second;
    // Iterate the smaller map.
    const auto& small = (ta.size() <= tb.size()) ? ta : tb;
    const auto& big = (ta.size() <= tb.size()) ? tb : ta;
    for (const auto& [f, touch_small] : small) {
        auto it = big.find(f);
        if (it == big.end()) continue;
        if (gse_touches_interfere(touch_small, it->second)) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Applicability guard (D6)
// ---------------------------------------------------------------------------

std::optional<std::string> gse_applicability_reason(const Problem& problem,
                                                     const GseClassification& c,
                                                     GseScope scope) {
    if (!problem.state_invariants().empty())
        return std::string("declared state invariants ((:constraints (always ...)))");
    if (c.any_conditional_effect)
        return std::string("conditional effects");
    if (c.any_quantified_effect)
        return std::string("quantified (forall) effects");
    if (c.any_unsupported_operator)
        return std::string("unsupported operator '") + c.first_unsupported_operator + "'";
    if (c.any_nonlinear)
        return std::string("nonlinear arithmetic (state-variable products / abs / mod / min / max)");

    if (scope == GseScope::M2) {
        if (c.any_dynamic_writer)
            return std::string("non-constant ASSIGN right-hand side (needs M4)");
        if (c.any_dynamic_amount)
            return std::string("non-constant additive amount (needs M4)");
        for (const auto& [f, fc] : c.fluents) {
            if (fc.is_numeric && fc.category != GseFluentCategory::PureAdditive)
                return std::string("numeric fluent with assignments — not pure-additive (needs M4)");
        }
    }
    // M4: LINEAR writers/amounts and mixed fluents are in scope; NONLINEAR,
    // conditional, quantified and unsupported operators already rejected above.
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// E1 metrics
// ---------------------------------------------------------------------------

namespace {
double median_of(std::vector<int> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    size_t n = v.size();
    return (n % 2) ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2.0;
}
}  // namespace

void gse_record_e1_stats(const Problem& problem, const GseClassification& c) {
    auto& stats = Stats::instance();

    int assign_only = 0, pure_additive = 0, mixed = 0;
    int n_bool = 0, n_numeric = 0, n_object = 0;
    int self_ref = 0;
    std::vector<int> wf, df, rf_numeric, rf_all;

    for (const auto& [f, fc] : c.fluents) {
        switch (fc.category) {
            case GseFluentCategory::AssignOnly:   ++assign_only; break;
            case GseFluentCategory::PureAdditive: ++pure_additive; break;
            case GseFluentCategory::Mixed:        ++mixed; break;
        }
        if (fc.is_bool) ++n_bool;
        if (fc.is_numeric) ++n_numeric;
        if (fc.is_object) ++n_object;
        if (fc.self_referential) ++self_ref;
        rf_all.push_back(fc.reads());
        if (fc.is_numeric) {
            wf.push_back(static_cast<int>(fc.writers.size()));
            df.push_back(static_cast<int>(fc.adds.size()));
            rf_numeric.push_back(fc.reads());
        }
    }

    auto max_of = [](const std::vector<int>& v) {
        return v.empty() ? 0 : *std::max_element(v.begin(), v.end());
    };

    stats.set("gse.fluents.total", static_cast<double>(c.fluents.size()));
    stats.set("gse.fluents.assign_only", assign_only);
    stats.set("gse.fluents.pure_additive", pure_additive);
    stats.set("gse.fluents.mixed", mixed);
    stats.set("gse.fluents.bool", n_bool);
    stats.set("gse.fluents.numeric", n_numeric);
    stats.set("gse.fluents.object", n_object);
    stats.set("gse.numeric.wf_max", max_of(wf));
    stats.set("gse.numeric.wf_median", median_of(wf));
    stats.set("gse.numeric.df_max", max_of(df));
    stats.set("gse.numeric.df_median", median_of(df));
    stats.set("gse.numeric.rf_max", max_of(rf_numeric));
    stats.set("gse.numeric.rf_median", median_of(rf_numeric));
    stats.set("gse.reads.rf_max", max_of(rf_all));
    stats.set("gse.reads.rf_median", median_of(rf_all));
    stats.set("gse.self_referential", self_ref);
    stats.set("gse.conditional_effects", c.any_conditional_effect ? 1 : 0);
    stats.set("gse.dynamic_amount", c.any_dynamic_amount ? 1 : 0);
    stats.set("gse.dynamic_writer", c.any_dynamic_writer ? 1 : 0);
    stats.set("gse.nonlinear", c.any_nonlinear ? 1 : 0);
    stats.set("gse.instances_k1", static_cast<double>(problem.actions().size()));
}

} // namespace rantanplan
