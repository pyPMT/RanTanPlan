#include "bound_tightening.hpp"
#include <cmath>
#include <optional>
#include <unordered_set>

namespace rantanplan {

// ============================================================================
// Helpers
// ============================================================================

// The exact value of a numeric constant; nothing for anything else.
static std::optional<Rational> extract_constant(ExprID id, const ExprPool& pool) {
    if (!id.valid() || !pool.is_constant(id)) return std::nullopt;
    if (pool.payload_is_int(id)) return Rational::of(pool.payload_int(id));
    if (pool.payload_is_real(id)) return pool.payload_real(id);
    return std::nullopt;
}

static bool is_nonnegative(const std::optional<Rational>& c) { return c && c->num >= 0; }

// The smallest double at least the constant, or +inf when it is not one: an
// upper bound read from it then holds every value the constant allows.
static double upper_double(const std::optional<Rational>& c) {
    return c ? Interval::of(*c).upper : INFINITY;
}

// Check if expr is (- a b) with a == fluent_id and b == delta_id.
static bool is_minus_of(ExprID expr, ExprID fluent_id, ExprID delta_id,
                         const ExprPool& pool) {
    if (pool.kind(expr) != ExprKind::FUNCTION_APPLICATION) return false;
    if (!pool.is_minus(expr)) return false;
    if (pool.argument_count(expr) != 2) return false;
    return pool.argument(expr, 0) == fluent_id &&
           pool.argument(expr, 1) == delta_id;
}

// Search precondition AND-tree for guards equivalent to `fluent >= delta`:
//   - Direct:   (>= fluent delta) or (<= delta fluent)
//   - Subtract: (>= (- fluent delta) C) where C >= 0
static bool precondition_has_ge_guard(ExprID precond_id, ExprID fluent_id,
                                      ExprID delta_id, const ExprPool& pool) {
    if (!precond_id.valid()) return false;
    if (pool.kind(precond_id) != ExprKind::FUNCTION_APPLICATION) return false;

    if (pool.is_and(precond_id)) {
        for (ExprID child : pool.children(precond_id)) {
            if (precondition_has_ge_guard(child, fluent_id, delta_id, pool))
                return true;
        }
        return false;
    }

    if (pool.argument_count(precond_id) != 2) return false;
    ExprID lhs = pool.argument(precond_id, 0);
    ExprID rhs = pool.argument(precond_id, 1);

    if (pool.is_greater_equal(precond_id) || pool.is_greater_than(precond_id)) {
        // (>= fluent delta)
        if (lhs == fluent_id && rhs == delta_id) return true;
        // (>= (- fluent delta) C) where C >= 0
        if (is_minus_of(lhs, fluent_id, delta_id, pool)) {
            if (is_nonnegative(extract_constant(rhs, pool))) return true;
        }
    }
    if (pool.is_less_equal(precond_id) || pool.is_less_than(precond_id)) {
        // (<= delta fluent)
        if (lhs == delta_id && rhs == fluent_id) return true;
        // (<= C (- fluent delta)) where C >= 0
        if (is_minus_of(rhs, fluent_id, delta_id, pool)) {
            if (is_nonnegative(extract_constant(lhs, pool))) return true;
        }
    }

    return false;
}

static bool is_plus_of(ExprID expr, ExprID fluent_id, ExprID delta_id,
                        const ExprPool& pool) {
    if (pool.kind(expr) != ExprKind::FUNCTION_APPLICATION) return false;
    if (!pool.is_plus(expr)) return false;
    if (pool.argument_count(expr) != 2) return false;
    ExprID a0 = pool.argument(expr, 0);
    ExprID a1 = pool.argument(expr, 1);
    return (a0 == fluent_id && a1 == delta_id) ||
           (a0 == delta_id && a1 == fluent_id);
}

// C when expr is (- C fluent) for a constant C.
static std::optional<Rational> is_minus_capacity(ExprID expr, ExprID fluent_id,
                                                 const ExprPool& pool) {
    if (pool.kind(expr) != ExprKind::FUNCTION_APPLICATION) return std::nullopt;
    if (!pool.is_minus(expr)) return std::nullopt;
    if (pool.argument_count(expr) != 2) return std::nullopt;
    if (pool.argument(expr, 1) != fluent_id) return std::nullopt;
    return extract_constant(pool.argument(expr, 0), pool);
}

// The smallest double at least R + delta, for constants R and delta.
static double upper_sum(ExprID r, ExprID delta, const ExprPool& pool) {
    auto a = extract_constant(r, pool), b = extract_constant(delta, pool);
    if (!a || !b) return INFINITY;
    return (Interval::of(*a) + Interval::of(*b)).upper;
}

// Search precondition AND-tree for f + delta <= C (or equivalent forms).
// Returns C (rounded up to a double) if found, +inf otherwise.
static double find_ub_guard(ExprID precond_id, ExprID fluent_id,
                             ExprID delta_id, const ExprPool& pool) {
    if (!precond_id.valid()) return INFINITY;
    if (pool.kind(precond_id) != ExprKind::FUNCTION_APPLICATION) return INFINITY;

    if (pool.is_and(precond_id)) {
        double best = INFINITY;
        for (ExprID child : pool.children(precond_id)) {
            double c = find_ub_guard(child, fluent_id, delta_id, pool);
            if (c < best) best = c;
        }
        return best;
    }

    if (pool.argument_count(precond_id) != 2) return INFINITY;
    ExprID lhs = pool.argument(precond_id, 0);
    ExprID rhs = pool.argument(precond_id, 1);

    if (pool.is_less_equal(precond_id) || pool.is_less_than(precond_id)) {
        // f + delta <= C
        if (is_plus_of(lhs, fluent_id, delta_id, pool)) {
            if (auto c = extract_constant(rhs, pool)) return upper_double(c);
        }
        // delta <= C - f
        if (lhs == delta_id) {
            if (auto c = is_minus_capacity(rhs, fluent_id, pool)) return upper_double(c);
        }
        // f <= R (constant) → f + delta <= R + delta
        if (lhs == fluent_id) {
            double c = upper_sum(rhs, delta_id, pool);
            if (std::isfinite(c)) return c;
        }
    }

    if (pool.is_greater_equal(precond_id) || pool.is_greater_than(precond_id)) {
        // C >= f + delta
        if (is_plus_of(rhs, fluent_id, delta_id, pool)) {
            if (auto c = extract_constant(lhs, pool)) return upper_double(c);
        }
        // C - f >= delta
        if (rhs == delta_id) {
            if (auto c = is_minus_capacity(lhs, fluent_id, pool)) return upper_double(c);
        }
        // R (constant) >= f → f + delta <= R + delta
        if (rhs == fluent_id) {
            double c = upper_sum(lhs, delta_id, pool);
            if (std::isfinite(c)) return c;
        }
    }

    return INFINITY;
}

// ============================================================================
// Lower bound verification: f >= 0
// ============================================================================

static bool verify_lower_bound(ExprID fluent_id, const Problem& problem) {
    const auto& pool = problem.pool();

    for (const auto& action : problem.actions()) {
        for (const auto& effect : action.effects()) {
            const auto& ee = effect.effect_expression();
            if (ee.fluent_id() != fluent_id) continue;

            if (ee.is_conditional()) return false;

            if (ee.is_increase()) {
                // Only a constant amount >= 0 is known not to lower f; a negative
                // or non-constant one may (sailing-wind's moves increase y by
                // (* (v ?b) -0.71)).
                if (!is_nonnegative(extract_constant(ee.value_id(), pool))) return false;
                continue;
            }

            if (ee.is_decrease()) {
                if (!action.has_precondition()) return false;
                if (!precondition_has_ge_guard(action.precondition_id(),
                                               fluent_id, ee.value_id(), pool))
                    return false;
                continue;
            }

            if (ee.is_assign()) {
                ExprID val = ee.value_id();
                if (is_nonnegative(extract_constant(val, pool))) continue;
                return false;
            }

            return false;
        }
    }

    return true;
}

// ============================================================================
// Upper bound verification: f <= C
// ============================================================================

static double verify_upper_bound(ExprID fluent_id, const Problem& problem) {
    const auto& pool = problem.pool();
    double bound = -INFINITY;

    for (const auto& action : problem.actions()) {
        for (const auto& effect : action.effects()) {
            const auto& ee = effect.effect_expression();
            if (ee.fluent_id() != fluent_id) continue;

            if (ee.is_conditional()) return INFINITY;

            if (ee.is_decrease()) {
                // Only a constant amount >= 0 is known not to raise f; a negative
                // or non-constant one may (the mirror image of verify_lower_bound).
                if (!is_nonnegative(extract_constant(ee.value_id(), pool))) return INFINITY;
                continue;
            }

            if (ee.is_increase()) {
                if (!action.has_precondition()) return INFINITY;
                double c = find_ub_guard(action.precondition_id(),
                                          fluent_id, ee.value_id(), pool);
                if (!std::isfinite(c)) return INFINITY;
                if (c > bound) bound = c;
                continue;
            }

            if (ee.is_assign()) {
                double v = upper_double(extract_constant(ee.value_id(), pool));
                if (!std::isfinite(v)) return INFINITY;
                if (v > bound) bound = v;
                continue;
            }

            return INFINITY;
        }
    }

    // The upper bound is the max of the initial value and the highest value
    // reachable via effects. When bound == -INFINITY (no increase/assign effects,
    // only decreases), the fluent can never exceed its initial value.
    double init = 0.0;
    for (const auto& assignment : problem.initial_state()) {
        if (assignment.fluent_id() == fluent_id) {
            init = upper_double(extract_constant(assignment.value_id(), pool));
            if (!std::isfinite(init)) return INFINITY;
            break;
        }
    }

    return std::max(bound, init);
}

// ============================================================================
// Public API
// ============================================================================

int tighten_bounds_syntactically(
    std::unordered_map<ExprID, Interval>& bounds,
    const Problem& problem) {

    const auto& pool = problem.pool();

    // Fluents whose initial value is negative, for the lower-bound check.
    std::unordered_set<ExprID> negative_initially;
    for (const auto& assignment : problem.initial_state()) {
        ExprID fid = assignment.fluent_id();
        if (!problem.is_numeric_type(fid)) continue;
        auto v = extract_constant(assignment.value_id(), pool);
        if (v && v->num < 0) negative_initially.insert(fid);
    }

    int tightened = 0;

    for (auto& [eid, interval] : bounds) {
        if (!problem.is_numeric_type(eid)) continue;

        // Lower bound: tighten to 0 if syntactically verified and tighter than RPG.
        if (interval.lower < 0.0) {
            if (!negative_initially.count(eid) && verify_lower_bound(eid, problem)) {
                interval.lower = 0.0;
                tightened++;
            }
        }

        // Upper bound: tighten to C if syntactically verified and tighter than RPG.
        double c = verify_upper_bound(eid, problem);
        if (std::isfinite(c) && c < interval.upper) {
            interval.upper = c;
            tightened++;
        }
    }

    return tightened;
}

// ============================================================================
// Conservation Law Discovery
// ============================================================================

std::vector<ConservationConstraint> discover_conservation_laws(
    const Problem& problem) {

    const auto& pool = problem.pool();

    // Step 1: For each action, collect per-fluent effect information.
    //  - Constant deltas (for the existing constant-balance check)
    //  - All unconditional increase/decrease effects (for transfer detection)
    //  - Poisoned fluents (conditional / ASSIGN)
    struct EffectInfo {
        bool is_increase;
        ExprID value_id;
    };
    struct ActionDeltas {
        std::unordered_map<ExprID, Rational> deltas;         // net constant delta
        std::unordered_set<ExprID> poisoned;                  // conditional / ASSIGN
        std::unordered_map<ExprID, std::vector<EffectInfo>> effects;  // all unconditional inc/dec
    };

    std::vector<ActionDeltas> action_data;
    action_data.reserve(problem.actions().size());

    for (const auto& action : problem.actions()) {
        ActionDeltas ad;
        for (const auto& effect : action.effects()) {
            const auto& ee = effect.effect_expression();
            ExprID fid = ee.fluent_id();
            if (!problem.is_numeric_type(fid)) continue;

            if (ee.is_conditional() || ee.is_assign()) {
                ad.poisoned.insert(fid);
                continue;
            }

            if (ee.is_increase() || ee.is_decrease()) {
                // Track all unconditional inc/dec for transfer detection
                ad.effects[fid].push_back({ee.is_increase(), ee.value_id()});

                // Also compute constant delta where possible (exactly; a
                // fluent whose net delta overflows is left out of laws)
                if (ee.is_constant_value()) {
                    if (auto d = extract_constant(ee.value_id(), pool)) {
                        Rational& net = ad.deltas[fid];
                        auto next = ee.is_increase() ? add(net, *d) : sub(net, *d);
                        if (next) net = *next;
                        else ad.poisoned.insert(fid);
                    }
                }
            }
        }
        action_data.push_back(std::move(ad));
    }

    // Step 2: Collect candidate pairs.
    // Source A: constant opposite deltas (existing).
    // Source B: same-expression transfers (increase(f,E) + decrease(g,E)).
    struct PairHash {
        size_t operator()(const std::pair<ExprID, ExprID>& p) const {
            return std::hash<int32_t>()(p.first.id) ^
                   (std::hash<int32_t>()(p.second.id) << 16);
        }
    };
    std::unordered_set<std::pair<ExprID, ExprID>, PairHash> candidate_pairs;

    for (const auto& ad : action_data) {
        // Source A: constant opposite deltas
        for (auto it1 = ad.deltas.begin(); it1 != ad.deltas.end(); ++it1) {
            if (ad.poisoned.count(it1->first)) continue;
            if (it1->second.num == 0) continue;
            for (auto it2 = std::next(it1); it2 != ad.deltas.end(); ++it2) {
                if (ad.poisoned.count(it2->first)) continue;
                if (it2->second.num == 0) continue;
                if (add(it1->second, it2->second) == Rational::of(0)) {
                    ExprID a = it1->first, b = it2->first;
                    if (b < a) std::swap(a, b);
                    candidate_pairs.insert({a, b});
                }
            }
        }

        // Source B: same-expression transfers
        for (auto it1 = ad.effects.begin(); it1 != ad.effects.end(); ++it1) {
            if (ad.poisoned.count(it1->first)) continue;
            for (auto it2 = std::next(it1); it2 != ad.effects.end(); ++it2) {
                if (ad.poisoned.count(it2->first)) continue;
                // Check if any effect on f matches an effect on g
                for (const auto& e1 : it1->second) {
                    for (const auto& e2 : it2->second) {
                        if (e1.value_id == e2.value_id &&
                            e1.is_increase != e2.is_increase) {
                            ExprID a = it1->first, b = it2->first;
                            if (b < a) std::swap(a, b);
                            candidate_pairs.insert({a, b});
                        }
                    }
                }
            }
        }
    }

    // Helper: check if all effects on f are transfer-matched by effects on g.
    // Every increase(f,E) needs a decrease(g,E), and vice versa.
    auto transfer_match = [](const std::vector<EffectInfo>& f_eff,
                             const std::vector<EffectInfo>& g_eff) -> bool {
        if (f_eff.size() != g_eff.size()) return false;
        std::vector<bool> matched(g_eff.size(), false);
        for (const auto& fe : f_eff) {
            bool found = false;
            for (size_t i = 0; i < g_eff.size(); ++i) {
                if (!matched[i] &&
                    fe.value_id == g_eff[i].value_id &&
                    fe.is_increase != g_eff[i].is_increase) {
                    matched[i] = true;
                    found = true;
                    break;
                }
            }
            if (!found) return false;
        }
        return true;
    };

    // Step 3: Verify each candidate across ALL actions.
    // An action preserves f + g == C if:
    //  (a) neither f nor g is modified, OR
    //  (b) both have constant opposite deltas and neither is poisoned, OR
    //  (c) all effects on f transfer-match effects on g (same ExprID, opposite sign)
    std::vector<ConservationConstraint> result;

    for (const auto& [f, g] : candidate_pairs) {
        bool valid = true;
        for (const auto& ad : action_data) {
            if (ad.poisoned.count(f) || ad.poisoned.count(g)) {
                valid = false;
                break;
            }

            bool f_has_effects = ad.effects.count(f);
            bool g_has_effects = ad.effects.count(g);

            // (a) Neither modified
            if (!f_has_effects && !g_has_effects) continue;

            // (b) Constant opposite deltas
            if (!f_has_effects == !g_has_effects) {  // both have or both don't
                Rational df = Rational::of(0), dg = Rational::of(0);
                auto itf = ad.deltas.find(f);
                auto itg = ad.deltas.find(g);
                if (itf != ad.deltas.end()) df = itf->second;
                if (itg != ad.deltas.end()) dg = itg->second;
                if (add(df, dg) == Rational::of(0)) {
                    // (c) Also check transfer match for non-constant effects
                    auto fit = ad.effects.find(f);
                    auto git = ad.effects.find(g);
                    if (fit != ad.effects.end() && git != ad.effects.end() &&
                        transfer_match(fit->second, git->second))
                        continue;
                    // Constant-only case: check balance
                    if (fit != ad.effects.end() || git != ad.effects.end()) {
                        // Has effects but transfer didn't match
                        valid = false;
                        break;
                    }
                    continue;  // neither has effects, delta check already passed
                }
            }

            // (c) Transfer match (handles variable deltas)
            auto fit = ad.effects.find(f);
            auto git = ad.effects.find(g);
            if (fit != ad.effects.end() && git != ad.effects.end() &&
                transfer_match(fit->second, git->second))
                continue;

            valid = false;
            break;
        }
        if (!valid) continue;

        // Compute C = initial(f) + initial(g), exactly.
        std::optional<Rational> init_f = Rational::of(0), init_g = Rational::of(0);
        for (const auto& assignment : problem.initial_state()) {
            ExprID fid = assignment.fluent_id();
            if (fid == f) init_f = extract_constant(assignment.value_id(), pool);
            else if (fid == g) init_g = extract_constant(assignment.value_id(), pool);
        }
        auto constant = init_f && init_g ? add(*init_f, *init_g) : std::nullopt;
        if (!constant) continue;

        ConservationConstraint cc;
        cc.fluent1_id = f;
        cc.fluent2_id = g;
        cc.constant = *constant;
        cc.label = pool.to_string(f) + " + " + pool.to_string(g) +
                   " == " + cc.constant.to_string();
        result.push_back(std::move(cc));
    }

    return result;
}

// ============================================================================
// Bound Derivation from Conservation Laws
// ============================================================================

void derive_bounds_from_conservations(
    std::unordered_map<ExprID, Interval>& bounds,
    const std::vector<ConservationConstraint>& conservations) {

    for (const auto& cons : conservations) {
        auto it1 = bounds.find(cons.fluent1_id);
        auto it2 = bounds.find(cons.fluent2_id);
        if (it1 == bounds.end() || it2 == bounds.end()) continue;

        const Interval C = Interval::of(cons.constant);

        // f + g == C  =>  f in C - g (outward rounded), and symmetrically.
        auto narrow = [&](Interval& x, const Interval& y) {
            const Interval implied = C - y;
            x.lower = std::max(x.lower, implied.lower);
            x.upper = std::min(x.upper, implied.upper);
        };
        narrow(it1->second, it2->second);
        narrow(it2->second, it1->second);
    }
}

} // namespace rantanplan
