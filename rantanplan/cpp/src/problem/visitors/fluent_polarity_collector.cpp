#include "fluent_polarity_collector.hpp"
#include "../problem.hpp"

namespace rantanplan {

static FluentPolarityCollector::Polarity flipped(FluentPolarityCollector::Polarity polarity) {
    using Polarity = FluentPolarityCollector::Polarity;
    switch (polarity) {
        case Polarity::POSITIVE: return Polarity::NEGATIVE;
        case Polarity::NEGATIVE: return Polarity::POSITIVE;
        case Polarity::BOTH: break;
    }
    return Polarity::BOTH;
}

void FluentPolarityCollector::collect_fluent_by_id(ExprID eid) {
    if (!eid.valid()) return;

    if (problem_->is_bool_type(eid)) {
        auto [it, inserted] = boolean_fluents_.emplace(eid, context_);
        if (!inserted && it->second != context_) it->second = Polarity::BOTH;
    } else {
        numeric_fluents_.insert(eid);
    }
}

void FluentPolarityCollector::collect_from_id(ExprID eid) {
    if (!eid.valid()) return;
    const ExprPool& pool = problem_->pool();

    ExprKind kind = pool.kind(eid);
    if (kind == ExprKind::FLUENT_SYMBOL || kind == ExprKind::STATE_VARIABLE) {
        collect_fluent_by_id(eid);
        return;
    }
    if (pool.child_count(eid) == 0) return;

    const Polarity outer = context_;
    if (pool.is_function_application(eid)) {
        auto args = pool.arguments(eid);
        if (pool.is_and(eid) || pool.is_or(eid)) {
            for (ExprID arg : args) collect_from_id(arg);
            return;
        }
        if (pool.is_not(eid)) {
            context_ = flipped(outer);
            for (ExprID arg : args) collect_from_id(arg);
            context_ = outer;
            return;
        }
        if (pool.is_implies(eid) && args.size() == 2) {
            context_ = flipped(outer);  // a -> b is not a or b
            collect_from_id(args[0]);
            context_ = outer;
            collect_from_id(args[1]);
            return;
        }
    }

    // Any other operator: either change of a boolean below it can matter.
    context_ = Polarity::BOTH;
    for (ExprID child : pool.children(eid)) collect_from_id(child);
    context_ = outer;
}

} // namespace rantanplan
