#include "landmark_analysis_pass.hpp"
#include "../analysis/achievers_analysis.hpp"
#include "../preprocessing/landmark_data.hpp"
#include "../util/logger.hpp"
#include "../util/stats.hpp"
#include <cmath>
#include <functional>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rantanplan {

namespace {

/// Evaluate a state-independent expression to a number.  Fails (nullopt) on
/// any state variable — thresholds must not depend on the state.
std::optional<double> eval_const_num(const ExprPool& pool, ExprID eid) {
    if (!eid.valid()) return std::nullopt;
    if (pool.is_constant(eid)) {
        if (pool.payload_is_double(eid)) return pool.payload_double(eid);
        if (pool.payload_is_int(eid))
            return static_cast<double>(pool.payload_int(eid));
        return std::nullopt;
    }
    if (pool.is_function_application(eid)) {
        ExprOperator op = pool.op(eid);
        size_t nargs = pool.argument_count(eid);
        if (nargs == 2) {
            auto lhs = eval_const_num(pool, pool.argument(eid, 0));
            auto rhs = eval_const_num(pool, pool.argument(eid, 1));
            if (!lhs || !rhs) return std::nullopt;
            switch (op) {
                case ExprOperator::PLUS:     return *lhs + *rhs;
                case ExprOperator::MINUS:    return *lhs - *rhs;
                case ExprOperator::MULTIPLY: return *lhs * *rhs;
                case ExprOperator::DIVIDE:
                    return *rhs != 0.0 ? std::optional<double>(*lhs / *rhs)
                                       : std::nullopt;
                default: return std::nullopt;
            }
        }
        if (nargs == 1 && op == ExprOperator::MINUS) {
            auto arg = eval_const_num(pool, pool.argument(eid, 0));
            if (!arg) return std::nullopt;
            return -*arg;
        }
    }
    return std::nullopt;
}

/// Linear normal form of a numeric threshold condition: the condition is
/// `constant + sum of coefficient*fluent >= 0` (strict > is treated as >=,
/// which only weakens the residual — sound).
struct LinearThreshold {
    std::unordered_map<int, std::pair<ExprID, double>> coeffs;  // fluent id -> (fluent, coefficient)
    double constant = 0.0;
};

/// Resolves a STATIC fluent (no action modifies it) to its constant initial
/// value; nullopt for dynamic fluents.
using StaticFold = std::function<std::optional<double>(ExprID)>;

/// eval_const_num extended with static-fluent folding: static fluents are
/// constants in disguise (their initial value holds forever).
std::optional<double> eval_static_num(const ExprPool& pool, ExprID eid,
                                      const StaticFold& fold) {
    if (!eid.valid()) return std::nullopt;
    if (pool.is_state_variable(eid)) return fold(eid);
    if (pool.is_constant(eid)) return eval_const_num(pool, eid);
    if (pool.is_function_application(eid)) {
        ExprOperator op = pool.op(eid);
        size_t nargs = pool.argument_count(eid);
        if (nargs == 2) {
            auto lhs = eval_static_num(pool, pool.argument(eid, 0), fold);
            auto rhs = eval_static_num(pool, pool.argument(eid, 1), fold);
            if (!lhs || !rhs) return std::nullopt;
            switch (op) {
                case ExprOperator::PLUS:     return *lhs + *rhs;
                case ExprOperator::MINUS:    return *lhs - *rhs;
                case ExprOperator::MULTIPLY: return *lhs * *rhs;
                case ExprOperator::DIVIDE:
                    return *rhs != 0.0 ? std::optional<double>(*lhs / *rhs)
                                       : std::nullopt;
                default: return std::nullopt;
            }
        }
        if (nargs == 1 && op == ExprOperator::MINUS) {
            auto arg = eval_static_num(pool, pool.argument(eid, 0), fold);
            if (!arg) return std::nullopt;
            return -*arg;
        }
    }
    return std::nullopt;
}

/// Coefficient collector: accumulate `sign * eid` into the normal form.
/// Handles +, binary/unary -, constant*expr, expr/constant; anything else
/// (fluent*fluent, unevaluable subterms) fails and the landmark keeps the
/// sound default.
bool linearize(const ExprPool& pool, ExprID eid, double sign,
               const StaticFold& fold, LinearThreshold& out) {
    if (!eid.valid()) return false;
    if (auto c = eval_static_num(pool, eid, fold)) {
        out.constant += sign * *c;
        return true;
    }
    if (pool.is_state_variable(eid)) {
        auto& slot = out.coeffs[eid.id];
        slot.first = eid;
        slot.second += sign;
        return true;
    }
    if (pool.is_function_application(eid)) {
        ExprOperator op = pool.op(eid);
        size_t nargs = pool.argument_count(eid);
        if (nargs == 2) {
            ExprID a = pool.argument(eid, 0);
            ExprID b = pool.argument(eid, 1);
            switch (op) {
                case ExprOperator::PLUS:
                    return linearize(pool, a, sign, fold, out) &&
                           linearize(pool, b, sign, fold, out);
                case ExprOperator::MINUS:
                    return linearize(pool, a, sign, fold, out) &&
                           linearize(pool, b, -sign, fold, out);
                case ExprOperator::MULTIPLY:
                    if (auto c = eval_static_num(pool, a, fold))
                        return linearize(pool, b, sign * *c, fold, out);
                    if (auto c = eval_static_num(pool, b, fold))
                        return linearize(pool, a, sign * *c, fold, out);
                    return false;
                case ExprOperator::DIVIDE:
                    if (auto c = eval_static_num(pool, b, fold); c && *c != 0.0)
                        return linearize(pool, a, sign / *c, fold, out);
                    return false;
                default:
                    return false;
            }
        }
        if (nargs == 1 && op == ExprOperator::MINUS) {
            return linearize(pool, pool.argument(eid, 0), -sign, fold, out);
        }
    }
    return false;
}

std::optional<LinearThreshold> decompose_linear(const ExprPool& pool, ExprID cond,
                                                const StaticFold& fold) {
    if (!pool.is_function_application(cond) || pool.argument_count(cond) != 2) {
        return std::nullopt;
    }
    ExprOperator op = pool.op(cond);
    bool ge = (op == ExprOperator::GREATER_EQUAL || op == ExprOperator::GREATER_THAN);
    bool le = (op == ExprOperator::LESS_EQUAL || op == ExprOperator::LESS_THAN);
    if (!ge && !le) return std::nullopt;

    // Normalize to E >= 0: E = lhs - rhs for >=, rhs - lhs for <=.
    LinearThreshold lt;
    double s = ge ? 1.0 : -1.0;
    if (!linearize(pool, pool.argument(cond, 0), s, fold, lt)) return std::nullopt;
    if (!linearize(pool, pool.argument(cond, 1), -s, fold, lt)) return std::nullopt;
    for (auto it = lt.coeffs.begin(); it != lt.coeffs.end();) {
        it = (it->second.second == 0.0) ? lt.coeffs.erase(it) : std::next(it);
    }
    if (lt.coeffs.empty()) return std::nullopt;  // constant condition
    return lt;
}

/// Net constant contribution of `action` toward E per firing (the weighted
/// sum of its constant additive increments on E's fluents), or nullopt when
/// any effect on those fluents is an assignment, state-dependent, nonlinear,
/// quantified, or unevaluable — the sound-and-complete default demanded by
/// the count semantics.
std::optional<double> linear_contribution(const ExprPool& pool,
                                          const Action& action,
                                          const LinearThreshold& lt,
                                          const StaticFold& fold) {
    double total = 0.0;
    for (const Effect& e : action.effects()) {
        const EffectExpression& ee = e.effect_expression();
        auto it = lt.coeffs.find(ee.fluent_id().id);
        if (it == lt.coeffs.end()) continue;
        if (ee.is_quantified() || ee.kind() == EffectExpression::Kind::ASSIGN) {
            return std::nullopt;
        }
        // eval_static_num decides constancy: it folds static fluents and
        // fails on anything referencing a dynamic one.
        auto value = eval_static_num(pool, ee.value_id(), fold);
        if (!value) return std::nullopt;
        double delta = ee.is_increase() ? *value : -*value;
        total += it->second.second * delta;
    }
    return total;
}

} // namespace

void LandmarkAnalysisPass::apply(PreprocessedProblem& result) const {
    if (!result.achievers) {
        Logger::instance().info("[Landmarks     ]  skipped (no achievers analysis)");
        return;
    }

    const Problem& problem = result.problem;
    const ExprPool& pool = problem.pool();
    const AchieversAnalysis& achievers = *result.achievers;
    std::unordered_map<int, ExprID> init_value;  // fluent ExprID.id -> value
    for (const Assignment& a : problem.initial_state()) {
        init_value[a.fluent_id().id] = a.value_id();
    }
    auto init_fluent_value = [&](ExprID fluent) -> std::optional<double> {
        auto it = init_value.find(fluent.id);
        if (it == init_value.end()) return std::nullopt;
        return eval_const_num(pool, it->second);
    };
    // Static fluents (modified by no action) are constants in disguise.
    std::unordered_set<int> modified_fluents;
    for (const Action& a : problem.actions()) {
        for (const Effect& e : a.effects()) {
            modified_fluents.insert(e.effect_expression().fluent_id().id);
        }
    }
    StaticFold fold_static = [&](ExprID fluent) -> std::optional<double> {
        if (modified_fluents.count(fluent.id)) return std::nullopt;
        return init_fluent_value(fluent);
    };

    LandmarkData& data = result.landmark_data;
    std::unordered_map<int, int> landmark_index;  // condition ExprID.id -> index
    std::unordered_set<ExprID> seen;

    // Exact init-state evaluation, computed once by the achievers analysis.
    const auto& init_sat = achievers.get_init_satisfied_conditions();
    auto init_satisfied = [&](ExprID cond) { return init_sat.count(cond) > 0; };

    // AND/OR closure from the goal conditions, recording natural orderings.
    std::vector<std::pair<ExprID, int>> queue;  // (condition, parent landmark index)
    for (ExprID g : achievers.get_goal_conditions()) {
        if (!init_satisfied(g) && seen.insert(g).second) {
            queue.push_back({g, -1});
        }
    }

    while (!queue.empty()) {
        auto [cond, parent] = queue.back();
        queue.pop_back();

        const auto& achiever_set = achievers.get_achievers(cond);
        if (achiever_set.empty()) continue;

        int index = static_cast<int>(data.landmarks.size());
        landmark_index[cond.id] = index;
        LandmarkInfo info;
        info.condition = cond;
        for (const Action& a : achiever_set) {
            info.achiever_action_ids.push_back(a.id());
        }
        if (parent >= 0) {
            data.orderings.push_back({index, parent});
        }

        // Uniform count semantics: default is residual 1 with the achievers
        // as unit contributors ("some achiever fires").  A linear numeric
        // threshold whose fluents every action changes by a constant
        // additive amount upgrades to the real deficit (each firing shifts
        // the linear functional by a state-independent constant).  The
        // deficit is filled by every action that moves the functional up,
        // not only by the achievers: an action that can never make the
        // condition true (factory-robot's precision-work, which needs low
        // workload) still adds to it.  Any assignment, state-dependent,
        // nonlinear, or quantified effect on those fluents, by any action,
        // keeps the sound default.
        info.contributor_action_ids = info.achiever_action_ids;
        info.contributions.assign(info.contributor_action_ids.size(), 1.0);
        if (auto lt = decompose_linear(pool, cond, fold_static)) {
            double e_init = lt->constant;
            bool init_known = true;
            for (const auto& [fid, cf] : lt->coeffs) {
                auto v = init_fluent_value(cf.first);
                if (!v) { init_known = false; break; }
                e_init += cf.second * *v;
            }
            double target = -e_init;  // mass needed to reach E >= 0
            if (init_known && target > 0) {
                std::vector<int> contributors;
                std::vector<double> lambdas;
                double total = 0.0;
                bool valid = true;
                for (const Action& a : problem.actions()) {
                    auto delta = linear_contribution(pool, a, *lt, fold_static);
                    if (!delta) {
                        valid = false;  // whole landmark keeps the default
                        break;
                    }
                    if (*delta > 0.0) {
                        contributors.push_back(a.id());
                        lambdas.push_back(*delta);
                        total += *delta;
                    }
                }
                if (valid && total > 0) {
                    info.contributor_action_ids = std::move(contributors);
                    info.contributions = std::move(lambdas);
                    info.residual = target;
                }
            }
        }

        // Recurse: conditions required by ALL achievers are landmarks too.
        std::unordered_map<int, int> prec_hits;
        std::unordered_map<int, ExprID> prec_ids;
        bool empty_intersection = false;
        for (const Action& a : achiever_set) {
            const auto& precs = achievers.get_preconditions(a);
            if (precs.empty()) {
                empty_intersection = true;
                break;
            }
            for (ExprID p : precs) {
                prec_hits[p.id]++;
                prec_ids.emplace(p.id, p);
            }
        }
        if (!empty_intersection) {
            int num_achievers = static_cast<int>(achiever_set.size());
            for (auto& [pid, hits] : prec_hits) {
                if (hits < num_achievers) continue;
                ExprID p = prec_ids.at(pid);
                if (init_satisfied(p)) continue;
                if (seen.insert(p).second) {
                    queue.push_back({p, index});
                } else {
                    // Rediscovered through another parent: still a valid
                    // ordering if it became a landmark.
                    auto it = landmark_index.find(p.id);
                    if (it != landmark_index.end()) {
                        data.orderings.push_back({it->second, index});
                    }
                }
            }
        }

        data.landmarks.push_back(std::move(info));
    }

    // Depth lower bound: even with every contributor firing cumulatively at
    // every step, covering the deficit needs this many steps.
    int informative = 0;  // landmarks with real (non-default) count structure
    for (const LandmarkInfo& info : data.landmarks) {
        double step_mass = 0.0;
        bool default_like = info.residual == 1.0;
        for (double lambda : info.contributions) {
            step_mass += lambda;
            default_like = default_like && lambda == 1.0;
        }
        if (step_mass <= 0) continue;
        if (!default_like) informative++;
        int bound = static_cast<int>(
            std::ceil(info.residual / step_mass - 1e-9));
        if (bound > data.depth_lower_bound) {
            data.depth_lower_bound = bound;
        }
    }
    if (data.depth_lower_bound > result.lower_bound) {
        result.lower_bound = data.depth_lower_bound;
    }

    Stats::instance().set("landmarks.count", static_cast<double>(data.landmarks.size()));
    Stats::instance().set("landmarks.count_landmarks", static_cast<double>(informative));
    Stats::instance().set("landmarks.depth_bound", static_cast<double>(data.depth_lower_bound));
    Logger::instance().info("[Landmarks     ]  landmarks: " +
        std::to_string(data.landmarks.size()) +
        "  |  informative counts: " + std::to_string(informative) +
        "  |  orderings: " + std::to_string(data.orderings.size()) +
        "  |  depth bound: " + std::to_string(data.depth_lower_bound));
}

} // namespace rantanplan
