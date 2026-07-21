#include "gse_planner.hpp"

#include "../config/config.hpp"
#include "../preprocessing/preprocessed_problem.hpp"
#include "../problem/const_fold.hpp"
#include "../util/logger.hpp"
#include "../util/stats.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace rantanplan {

namespace {

// ===========================================================================
// Exact plan simulator (D7)
// ===========================================================================
//
// Walks the extracted action sequence over an exact state map, checking every
// precondition and the goal.  Booleans are tracked separately from numerics
// (object fluents are numeric object-indices).  A mismatch throws — it means
// the encoding and the intended semantics disagree, which must never ship.

struct SimValue {
    bool is_bool = false;
    bool b = false;
    double d = 0.0;
};

class Simulator {
public:
    Simulator(const Problem& problem, double eps) : problem_(problem), pool_(problem.pool()), eps_(eps) {
        for (const Assignment& asn : problem_.initial_state()) {
            SimValue v = fold_const(asn.value_id());
            if (v.is_bool) bool_state_[asn.fluent_id()] = v.b;
            else num_state_[asn.fluent_id()] = v.d;
        }
    }

    /// Returns "" on success, else a diagnostic describing the first failure.
    std::string run(const std::vector<const Action*>& actions) {
        for (const Action* a : actions) {
            if (a->has_precondition()) {
                SimValue pv = eval(a->precondition_id());
                if (!pv.is_bool || !pv.b) {
                    return "precondition of " + a->label() + " not satisfied in simulated state";
                }
            }
            // Snapshot effect targets against the pre-state, then apply.
            struct Pending { ExprID f; EffectExpression::Kind kind; SimValue v; };
            std::vector<Pending> pending;
            for (const Effect& eff : a->effects()) {
                const EffectExpression& ee = eff.effect_expression();
                pending.push_back({ee.fluent_id(), ee.kind(), eval(ee.value_id())});
            }
            for (const Pending& p : pending) {
                switch (p.kind) {
                    case EffectExpression::Kind::ASSIGN:
                        if (p.v.is_bool) bool_state_[p.f] = p.v.b;
                        else num_state_[p.f] = p.v.d;
                        break;
                    case EffectExpression::Kind::INCREASE:
                        num_state_[p.f] += p.v.d; break;
                    case EffectExpression::Kind::DECREASE:
                        num_state_[p.f] -= p.v.d; break;
                }
            }
        }
        for (const Goal& g : problem_.goals()) {
            SimValue gv = eval(g.goal_id());
            if (!gv.is_bool || !gv.b) return "goal not satisfied by simulated plan";
        }
        return "";
    }

private:
    const Problem& problem_;
    const ExprPool& pool_;
    double eps_;
    std::unordered_map<ExprID, double> num_state_;
    std::unordered_map<ExprID, bool> bool_state_;

    SimValue fold_const(ExprID e) {
        SimValue v;
        if (pool_.payload_is_bool(e)) { v.is_bool = true; v.b = pool_.payload_bool(e); return v; }
        if (pool_.payload_is_int(e)) { v.d = static_cast<double>(pool_.payload_int(e)); return v; }
        if (pool_.payload_is_double(e)) { v.d = pool_.payload_double(e); return v; }
        int oi = problem_.object_constant_index(e);
        if (oi >= 0) { v.d = static_cast<double>(oi); return v; }
        auto folded = fold_const_int(e, pool_);
        if (folded) { v.d = static_cast<double>(*folded); return v; }
        throw std::runtime_error("gse simulator: non-constant init/leaf value");
    }

    SimValue eval(ExprID e) {
        if (pool_.is_state_variable(e)) {
            SimValue v;
            if (problem_.is_bool_type(e)) { v.is_bool = true; v.b = lookup_bool(e); }
            else v.d = lookup_num(e);
            return v;
        }
        if (pool_.is_leaf(e)) return fold_const(e);
        if (!pool_.is_function_application(e))
            throw std::runtime_error("gse simulator: unexpected node kind");

        ExprOperator op = pool_.op(e);
        auto args = pool_.arguments(e);
        switch (op) {
            case ExprOperator::AND: {
                for (ExprID c : args) if (!eval(c).b) return bval(false);
                return bval(true);
            }
            case ExprOperator::OR: {
                for (ExprID c : args) if (eval(c).b) return bval(true);
                return bval(false);
            }
            case ExprOperator::NOT: return bval(!eval(args[0]).b);
            case ExprOperator::IMPLIES: return bval(!eval(args[0]).b || eval(args[1]).b);
            case ExprOperator::PLUS: {
                double s = 0; for (ExprID c : args) s += eval(c).d; return dval(s);
            }
            case ExprOperator::MINUS: {
                if (args.size() == 1) return dval(-eval(args[0]).d);
                return dval(eval(args[0]).d - eval(args[1]).d);
            }
            case ExprOperator::MULTIPLY: {
                double s = 1; for (ExprID c : args) s *= eval(c).d; return dval(s);
            }
            case ExprOperator::DIVIDE: return dval(eval(args[0]).d / eval(args[1]).d);
            case ExprOperator::EQUALS: {
                SimValue l = eval(args[0]), r = eval(args[1]);
                if (l.is_bool || r.is_bool) return bval(l.b == r.b);
                return bval(std::fabs(l.d - r.d) <= eps_);
            }
            case ExprOperator::LESS_EQUAL:    return bval(eval(args[0]).d <= eval(args[1]).d + eps_);
            case ExprOperator::LESS_THAN:     return bval(eval(args[1]).d - eval(args[0]).d > eps_);
            case ExprOperator::GREATER_EQUAL: return bval(eval(args[0]).d + eps_ >= eval(args[1]).d);
            case ExprOperator::GREATER_THAN:  return bval(eval(args[0]).d - eval(args[1]).d > eps_);
            default:
                throw std::runtime_error("gse simulator: unsupported operator");
        }
    }

    bool lookup_bool(ExprID f) { auto it = bool_state_.find(f); return it != bool_state_.end() && it->second; }
    double lookup_num(ExprID f) { auto it = num_state_.find(f); return it != num_state_.end() ? it->second : 0.0; }
    static SimValue bval(bool b) { SimValue v; v.is_bool = true; v.b = b; return v; }
    static SimValue dval(double d) { SimValue v; v.d = d; return v; }
};

// ===========================================================================
// One budget round: build the GSE formula on a fresh solver and solve it.
// ===========================================================================

struct Instance {
    const Action* action = nullptr;  // nullptr for bottom/top
    int copy = -1;
};

class Round {
public:
    Round(const Problem& problem, const GseClassification& cls,
          const std::unordered_map<int, int>& copies,
          const std::vector<LandmarkInfo>& landmarks, bool core_guided, z3::context& ctx)
        : problem_(problem), pool_(problem.pool()), cls_(cls), landmarks_(landmarks),
          core_guided_(core_guided), ctx_(ctx), all_integer_(problem.all_integer()) {
        build_instances(copies);
    }

    int num_ordinary() const { return N_; }
    /// M6i-b: C3 clauses elided as vacuous (same-constant writers, no contributors).
    long long protection_skipped() const { return protection_skipped_; }
    /// M6i-a: link candidates dropped because their constant output cannot
    /// satisfy the reader's guard.
    long long support_dropped() const { return support_dropped_; }
    /// M6b: fluents given the timeline realisation this round.
    long long timeline_fluents() const { return timeline_fluents_; }
    /// M6i-a: reads whose support set emptied — b is provably inapplicable.
    /// Short-circuits the search, so it is counted and logged, never silent.
    long long support_empty() const { return support_empty_; }

    /// Assert the whole encoding onto `solver`.
    void build(z3::solver& solver);

    /// Extract the plan from a model (occurring ordinary instances, rank order).
    Plan extract(z3::model& model);

    /// Core-guided: the per-action cap assumptions (each forces its sentinel
    /// copy dormant).  Empty unless core_guided.
    z3::expr_vector assumptions();
    /// Map a core literal's expr id back to the action id it caps, or -1.
    int cap_action(unsigned expr_id) const {
        auto it = cap_id_to_action_.find(expr_id);
        return it == cap_id_to_action_.end() ? -1 : it->second;
    }

private:
    const Problem& problem_;
    const ExprPool& pool_;
    const GseClassification& cls_;
    const std::vector<LandmarkInfo>& landmarks_;
    bool core_guided_;
    z3::context& ctx_;
    bool all_integer_;

    // Core-guided sentinels: one dormant probe copy per action id.
    std::unordered_map<int, int> sentinel_of_;      // action id -> instance idx
    std::unordered_map<int, z3::expr> cap_of_;      // action id -> cap literal
    std::unordered_map<unsigned, int> cap_id_to_action_;

    std::vector<Instance> inst_;              // 0..N-1 ordinary, N bottom, N+1 top
    std::vector<z3::expr> occ_;
    std::vector<z3::expr> tau_;
    int N_ = 0, B_ = 0, T_ = 0;
    long long protection_skipped_ = 0;
    long long support_dropped_ = 0;
    long long support_empty_ = 0;
    std::unordered_map<int, std::vector<int>> inst_of_;  // action id -> instance idxs

    // v(b,f): per instance, fluent -> seen-value var.
    std::vector<std::unordered_map<ExprID, z3::expr>> v_;
    // w(a,f): fluent -> writer instance -> output var/const.
    std::unordered_map<ExprID, std::unordered_map<int, z3::expr>> w_;
    // amt(c,f): fluent -> contributor instance -> amount var/const.
    std::unordered_map<ExprID, std::unordered_map<int, z3::expr>> amt_;

    /// M6b: one fluent's segment chain.  `V[k]` is the fluent's value after the
    /// k-th occurring writer; `seg_of(rank)` counts the writers before a rank,
    /// which is both a writer's slot index (minus one) and a reader's segment.
    struct Timeline {
        int K = 0;
        std::vector<z3::expr> V;                                ///< V[0..K]
        std::function<z3::expr(z3::expr)> seg_of;               ///< rank -> #writers before it
    };
    std::unordered_map<ExprID, Timeline> timelines_;
    long long timeline_fluents_ = 0;

    // Writer/contributor entries per fluent (ordinary instances + effect index).
    // `const_write` is the interned ExprID of the effect's RHS when that RHS is a
    // CONSTANT, else EXPR_NULL.  The ExprPool interns, so two writers assigning
    // the same constant share an ExprID — that identity is what P1(b) tests.
    struct Entry { int inst; int effect_index; bool negated; ExprID const_write = EXPR_NULL; };
    std::unordered_map<ExprID, std::vector<Entry>> writer_entries_;
    std::unordered_map<ExprID, std::vector<Entry>> add_entries_;

    // ---- helpers ----
    z3::expr make_num(int64_t v) { return all_integer_ ? ctx_.int_val(static_cast<int>(v)) : ctx_.real_val(static_cast<int>(v)); }
    z3::expr make_num_d(double v) {
        if (all_integer_) return ctx_.int_val(static_cast<int>(std::llround(v)));
        return ctx_.real_val(std::to_string(v).c_str());
    }
    z3::expr numeric_const(const std::string& n) { return all_integer_ ? ctx_.int_const(n.c_str()) : ctx_.real_const(n.c_str()); }
    z3::expr value_var(ExprID f, const std::string& name) {
        if (problem_.is_bool_type(f)) return ctx_.bool_const(name.c_str());
        return numeric_const(name);  // numeric or object (index)
    }

    void build_instances(const std::unordered_map<int, int>& copies);
    z3::expr translate(ExprID e, const std::unordered_map<ExprID, z3::expr>& subst);
    z3::expr init_value(ExprID f);
    /// The interned ExprID of f's initial value when it is a CONSTANT, else
    /// EXPR_NULL (M6i-b treats the virtual bottom writer like any other).
    ExprID init_const_id(ExprID f);
    /// M6i-a: does a *top-level conjunct* of `guard` pin Boolean fluent `f` to a
    /// constant?  Top-level conjunction only — a conjunct must hold whenever the
    /// guard does, which is exactly what licenses restricting the support set.
    /// `f` under a disjunction or a negated conjunction yields nullopt.
    std::optional<bool> required_bool(ExprID guard, ExprID f) const;
    /// The pinned value of `f` in instance `b`'s own guard (goal, for top).
    std::optional<bool> required_bool_for(int b, ExprID f) const;
    const std::unordered_map<ExprID, z3::expr>& subst_of(int inst) const { return v_[inst]; }
};

void Round::build_instances(const std::unordered_map<int, int>& copies) {
    for (const Action& a : problem_.actions()) {
        auto it = copies.find(a.id());
        int k = (it == copies.end()) ? 1 : it->second;
        for (int j = 0; j < k; ++j) {
            int idx = static_cast<int>(inst_.size());
            inst_.push_back({&a, j});
            inst_of_[a.id()].push_back(idx);
        }
        if (core_guided_) {
            // One dormant sentinel copy (the "+1 probe"), capped off by cap_a.
            int idx = static_cast<int>(inst_.size());
            inst_.push_back({&a, k});
            inst_of_[a.id()].push_back(idx);
            sentinel_of_[a.id()] = idx;
            z3::expr cap = ctx_.bool_const(("cap_" + std::to_string(a.id())).c_str());
            cap_of_.emplace(a.id(), cap);
            cap_id_to_action_[cap.id()] = a.id();
        }
    }
    N_ = static_cast<int>(inst_.size());
    B_ = N_;
    T_ = N_ + 1;
    inst_.push_back({nullptr, -1});  // bottom
    inst_.push_back({nullptr, -1});  // top

    occ_.reserve(N_ + 2);
    tau_.reserve(N_ + 2);
    for (int i = 0; i < N_; ++i) {
        occ_.push_back(ctx_.bool_const(("occ_" + std::to_string(i)).c_str()));
        tau_.push_back(ctx_.int_const(("tau_" + std::to_string(i)).c_str()));
    }
    occ_.push_back(ctx_.bool_val(true));       // bottom occ
    tau_.push_back(ctx_.int_val(0));           // bottom rank
    occ_.push_back(ctx_.bool_val(true));       // top occ
    tau_.push_back(ctx_.int_val(N_ + 1));      // top rank
}

z3::expr Round::init_value(ExprID f) {
    // I(f): the initial-state assignment for f, or CWA default.
    for (const Assignment& asn : problem_.initial_state()) {
        if (asn.fluent_id() == f) {
            std::unordered_map<ExprID, z3::expr> empty;
            return translate(asn.value_id(), empty);
        }
    }
    if (problem_.is_bool_type(f)) return ctx_.bool_val(false);  // CWA
    return make_num(0);
}

/// Estimated formula cost of the two realisations of one fluent.
///
/// Both the per-fluent realisation choice (M6b) and the formula-size guard must
/// use this: if they disagree, the guard rejects instances the encoder would
/// have built cheaply, or admits ones it cannot.
///
/// `link` models M6i-b exactly — protection between same-constant writers is
/// never emitted, so those ordered pairs are subtracted.  That is why the choice
/// cannot be a raw density threshold: on a *monotone* fluent (every writer
/// assigns the same value) M6i-b already drives `link` to ~0, and a timeline
/// would replace a free structure with `w²` channelling.
struct RealisationCost {
    long long link = 0;
    long long timeline = -1;  ///< -1 when inapplicable (fluent has contributors)
    /// A timeline is chosen only when links are *non-viable*, not merely larger.
    ///
    /// Measured: timelines are sound but propagate far worse than links — the
    /// link form gives Z3 clauses it can unit-propagate, the timeline gives it
    /// pseudo-Boolean arithmetic (`Σ ite(...) = k`) it must solve.  With a pure
    /// `timeline < link` rule, obj-blocks went 5.06 s → timeout and gripper
    /// stalled, while depots (link cost ~8.2 M for one fluent, refused outright)
    /// only *builds* with timelines.  So size alone is the wrong criterion; the
    /// floor keeps links wherever they are affordable.
    ///
    /// Provisional constant — a measurement subject, not a conclusion.
    static constexpr long long kLinkCostFloor = 1000000;
    bool prefer_timeline() const {
        if (getenv("GSE_NO_TIMELINE")) return false;
        return timeline >= 0 && timeline < link && link > kLinkCostFloor;
    }
    long long best() const { return prefer_timeline() ? timeline : link; }
};

/// `class_sizes` are the counts of writer instances sharing a constant output
/// (bottom included when its initial value is constant).
RealisationCost realisation_cost(long long w_inst, long long reads, long long d_inst,
                                 const std::vector<long long>& class_sizes) {
    RealisationCost c;
    const long long threats = w_inst + 1;  // + bottom
    long long prot = threats * (threats - 1);
    for (long long n : class_sizes) prot -= n * (n - 1);
    if (prot < 0) prot = 0;
    c.link = reads * (prot + threats * d_inst) + 4;
    // Timeline: O(w·K) channelling once per fluent, plus O(K) per read.
    // Only for contributor-free fluents — otherwise the value rule would
    // instantiate the interval sum per segment (r·K·d), worse than links' r·d.
    if (d_inst == 0 && w_inst > 0) c.timeline = w_inst * w_inst + reads * (w_inst + 1);
    return c;
}

/// The interned ExprID of f's initial value when it is a CONSTANT, else
/// EXPR_NULL.  Shared by the encoding (bottom's output) and the formula-size
/// estimator, which must agree on what counts as a same-constant writer.
ExprID init_const_id_of(const Problem& problem, ExprID f) {
    for (const Assignment& asn : problem.initial_state()) {
        if (asn.fluent_id() == f) {
            ExprID v = asn.value_id();
            return problem.pool().kind(v) == ExprKind::CONSTANT ? v : EXPR_NULL;
        }
    }
    // No explicit assignment (CWA default).  Conservative: no skip.  Harmless —
    // a CWA-default Boolean is false and its writers assign true, so the pair
    // would not have been skipped anyway.
    return EXPR_NULL;
}

ExprID Round::init_const_id(ExprID f) { return init_const_id_of(problem_, f); }

std::optional<bool> Round::required_bool(ExprID guard, ExprID f) const {
    if (guard == EXPR_NULL) return std::nullopt;
    if (guard == f) return true;
    if (!pool_.is_function_application(guard)) return std::nullopt;
    ExprOperator op = pool_.op(guard);
    if (op == ExprOperator::AND) {
        // Two conjuncts pinning f to different values make the guard
        // unsatisfiable; returning either is then harmless.
        for (ExprID a : pool_.arguments(guard))
            if (auto r = required_bool(a, f)) return r;
        return std::nullopt;
    }
    if (op == ExprOperator::NOT) {
        auto args = pool_.arguments(guard);
        if (!args.empty() && args[0] == f) return false;
    }
    return std::nullopt;
}

std::optional<bool> Round::required_bool_for(int b, ExprID f) const {
    if (!problem_.is_bool_type(f)) return std::nullopt;
    if (b == B_) return std::nullopt;
    if (b == T_) {
        for (const Goal& g : problem_.goals())
            if (auto r = required_bool(g.goal_id(), f)) return r;
        return std::nullopt;
    }
    const Action* a = inst_[b].action;
    if (!a || !a->has_precondition()) return std::nullopt;
    // Note: the pin applies to v(b,f) wherever else b reads f (e.g. an effect
    // RHS), because it is the same variable and the guard is asserted under
    // occ(b) alongside every other use.
    return required_bool(a->precondition_id(), f);
}

z3::expr Round::translate(ExprID e, const std::unordered_map<ExprID, z3::expr>& subst) {
    ExprKind kind = pool_.kind(e);

    if (pool_.is_state_variable(e)) {
        auto it = subst.find(e);
        if (it == subst.end())
            throw std::runtime_error("gse translate: unregistered fluent read " + pool_.to_string(e));
        return it->second;
    }

    if (pool_.is_leaf(e)) {
        if (kind == ExprKind::CONSTANT) {
            if (pool_.payload_is_bool(e)) return ctx_.bool_val(pool_.payload_bool(e));
            if (pool_.payload_is_int(e)) return make_num(pool_.payload_int(e));
            if (pool_.payload_is_double(e)) return make_num_d(pool_.payload_double(e));
            if (pool_.payload_is_string(e)) {
                int oi = problem_.object_constant_index(e);
                if (oi >= 0) return make_num(oi);
            }
        }
        throw std::runtime_error("gse translate: unsupported leaf " + pool_.to_string(e));
    }

    if (!pool_.is_function_application(e))
        throw std::runtime_error("gse translate: unsupported node " + pool_.to_string(e));

    ExprOperator op = pool_.op(e);
    auto args = pool_.arguments(e);
    auto tr = [&](ExprID a) { return translate(a, subst); };

    switch (op) {
        case ExprOperator::AND: {
            z3::expr_vector v(ctx_); for (ExprID a : args) v.push_back(tr(a));
            return v.empty() ? ctx_.bool_val(true) : z3::mk_and(v);
        }
        case ExprOperator::OR: {
            z3::expr_vector v(ctx_); for (ExprID a : args) v.push_back(tr(a));
            return v.empty() ? ctx_.bool_val(false) : z3::mk_or(v);
        }
        case ExprOperator::NOT:     return !tr(args[0]);
        case ExprOperator::IMPLIES: return z3::implies(tr(args[0]), tr(args[1]));
        case ExprOperator::PLUS: {
            z3::expr acc = make_num(0); for (ExprID a : args) acc = acc + tr(a); return acc;
        }
        case ExprOperator::MINUS:
            return args.size() == 1 ? -tr(args[0]) : tr(args[0]) - tr(args[1]);
        case ExprOperator::MULTIPLY: {
            z3::expr acc = make_num(1); for (ExprID a : args) acc = acc * tr(a); return acc;
        }
        case ExprOperator::DIVIDE:        return tr(args[0]) / tr(args[1]);
        case ExprOperator::EQUALS:        return tr(args[0]) == tr(args[1]);
        case ExprOperator::LESS_EQUAL:    return tr(args[0]) <= tr(args[1]);
        case ExprOperator::LESS_THAN:     return tr(args[0]) < tr(args[1]);
        case ExprOperator::GREATER_EQUAL: return tr(args[0]) >= tr(args[1]);
        case ExprOperator::GREATER_THAN:  return tr(args[0]) > tr(args[1]);
        default:
            throw std::runtime_error("gse translate: unsupported operator " + expr_operator_to_string(op));
    }
}

void Round::build(z3::solver& solver) {
    // --- Rank bounds + copy symmetry (C7) ---
    for (int i = 0; i < N_; ++i) {
        solver.add(z3::implies(occ_[i], tau_[i] >= 1 && tau_[i] <= N_));
    }
    for (const auto& [aid, insts] : inst_of_) {
        for (size_t j = 1; j < insts.size(); ++j) {
            int prev = insts[j - 1], cur = insts[j];
            solver.add(z3::implies(occ_[cur], occ_[prev]));                 // C7
            solver.add(z3::implies(occ_[cur], tau_[prev] < tau_[cur]));     // C7
        }
    }

    // --- Core-guided caps: cap_a forces the sentinel copy dormant ---
    for (const auto& [aid, cap] : cap_of_) {
        solver.add(z3::implies(cap, !occ_[sentinel_of_.at(aid)]));
    }

    // --- Collect the read fluents and mint v(b,f); build subst maps ---
    v_.assign(N_ + 2, {});
    std::unordered_set<ExprID> read_fluents;
    auto register_reads = [&](int b, const std::vector<ExprID>& reads) {
        std::unordered_set<ExprID> seen;
        for (ExprID f : reads) {
            if (!seen.insert(f).second) continue;
            std::string nm = "v_" + std::to_string(b) + "_" + std::to_string(f.id);
            v_[b].emplace(f, value_var(f, nm));
            read_fluents.insert(f);
        }
    };
    for (int i = 0; i < N_; ++i) {
        auto it = cls_.action_reads.find(inst_[i].action->id());
        if (it != cls_.action_reads.end()) register_reads(i, it->second);
    }
    register_reads(T_, cls_.goal_reads);

    // --- Writer / contributor entries per read fluent ---
    for (ExprID f : read_fluents) {
        auto fit = cls_.fluents.find(f);
        if (fit == cls_.fluents.end()) continue;
        for (const GseEffectRef& wr : fit->second.writers) {
            ExprID cw = EXPR_NULL;
            if (wr.value_kind == ValueKind::CONSTANT) {
                cw = problem_.action(wr.action_id).effect(wr.effect_index)
                         .effect_expression().value_id();
            }
            for (int inst : inst_of_[wr.action_id])
                writer_entries_[f].push_back({inst, wr.effect_index, false, cw});
        }
        for (const GseEffectRef& ad : fit->second.adds) {
            for (int inst : inst_of_[ad.action_id])
                add_entries_[f].push_back({inst, ad.effect_index, ad.negated});
        }
    }

    // --- w(a,f) (C5) and bottom outputs ---
    for (ExprID f : read_fluents) {
        w_[f].emplace(B_, init_value(f));
        for (const Entry& e : writer_entries_[f]) {
            std::string nm = "w_" + std::to_string(e.inst) + "_" + std::to_string(f.id);
            z3::expr wv = value_var(f, nm);
            ExprID rhs = problem_.action(inst_[e.inst].action->id()).effect(e.effect_index)
                             .effect_expression().value_id();
            solver.add(z3::implies(occ_[e.inst], wv == translate(rhs, subst_of(e.inst))));
            w_[f].emplace(e.inst, wv);
        }
    }

    // --- amt(c,f) (C5) ---
    for (ExprID f : read_fluents) {
        for (const Entry& e : add_entries_[f]) {
            std::string nm = "amt_" + std::to_string(e.inst) + "_" + std::to_string(f.id);
            z3::expr av = numeric_const(nm);
            ExprID rhs = problem_.action(inst_[e.inst].action->id()).effect(e.effect_index)
                             .effect_expression().value_id();
            z3::expr amount = translate(rhs, subst_of(e.inst));
            if (e.negated) amount = -amount;
            solver.add(z3::implies(occ_[e.inst], av == amount));
            amt_[f].emplace(e.inst, av);
        }
    }

    // --- M6b: per-fluent timeline chains (idea.md §5.1) ---
    //
    // The link realisation costs `r_f·w_f²` because every read re-derives "no
    // other writer in between" against every writer pair.  A timeline pays the
    // writer-ordering cost *once per fluent* and gives each read an O(K) segment
    // attachment instead:  writers occupy a packed, rank-ordered slot chain, and
    // a read attaches to the segment whose bracketing ranks straddle its own.
    // Cost becomes `w_f² + r_f·K_f`.
    //
    // Scope (deliberate):
    //  - Contributor-free fluents only.  With contributors the value rule would
    //    be `V(k) + intervalsum(rank_at(k), τ(b))`, instantiating the sum per
    //    segment (`r·K·d`) — worse than the link form's `r·d`.  With d = 0 the
    //    rule collapses to `v(b,f) = V(seg)`.  The measured offenders
    //    (`(available hoist1)`, `(available rover1)`) are all contributor-free.
    //  - Chosen per fluent by comparing estimated costs, NOT by a density
    //    threshold.  M6i-b already zeroes link protection on monotone fluents,
    //    so a naive "writer-dense ⇒ timeline" rule made rover/pfile14 strictly
    //    worse (back to the heap cap): it paid w² channelling where links were
    //    already free.
    auto read_count_of = [&](ExprID f) -> long long {
        long long n = 0;
        for (int b = 0; b < N_ + 2; ++b)
            if (b != B_ && v_[b].count(f)) ++n;
        return n;
    };
    for (ExprID f : read_fluents) {
        const std::vector<Entry>& wents = writer_entries_[f];
        if (!add_entries_[f].empty()) continue;  // needs d = 0
        if (wents.empty()) continue;

        std::unordered_map<ExprID, long long> cls_sizes;
        ExprID bi = init_const_id(f);
        if (bi != EXPR_NULL) cls_sizes[bi] += 1;
        for (const Entry& e : wents)
            if (e.const_write != EXPR_NULL) cls_sizes[e.const_write] += 1;
        std::vector<long long> sizes;
        sizes.reserve(cls_sizes.size());
        for (const auto& [id, n] : cls_sizes) sizes.push_back(n);

        RealisationCost rc = realisation_cost(static_cast<long long>(wents.size()),
                                              read_count_of(f), 0, sizes);
        if (!rc.prefer_timeline()) continue;

        Timeline tl;
        const int K = static_cast<int>(wents.size());
        tl.K = K;
        std::string fid = std::to_string(f.id);

        // Segment values: V(k) is f's value after the k-th occurring writer.
        tl.V.push_back(w_[f].at(B_));  // V(0) = I(f)
        for (int k = 1; k <= K; ++k)
            tl.V.push_back(value_var(f, "Vt_" + fid + "_" + std::to_string(k)));

        // A writer's slot and a reader's segment are *defined terms* over τ, not
        // free variables:
        //     slot(a) = 1 + |{a' : occ(a') ∧ τ(a') < τ(a)}|
        //     seg(b)  =     |{a  : occ(a)  ∧ τ(a)  < τ(b)}|
        //
        // The first cut gave each writer a free `slot` Int constrained to agree
        // with τ (plus packing/monotonicity/occupancy).  That is a *redundant
        // permutation* the solver has to search alongside τ, and it was
        // catastrophic: gripper (34 instances) went from instant to a >30 s
        // timeout even though the formula got smaller.  As counting sums both
        // are functionally determined once τ is fixed, so they propagate instead
        // of branching — and packing, occupancy, rank_at and the idle-slot rule
        // all disappear with them.
        auto writers_before = [&](z3::expr rank) {
            z3::expr acc = ctx_.int_val(0);
            for (const Entry& e : wents)
                acc = acc + z3::ite(occ_[e.inst] && (tau_[e.inst] < rank), ctx_.int_val(1), ctx_.int_val(0));
            return acc;
        };

        // A writer that occurs pins the value of its own segment.  `τ(a') < τ(a)`
        // is strict, so a writer never counts itself.
        for (const Entry& e : wents) {
            z3::expr slot = writers_before(tau_[e.inst]) + ctx_.int_val(1);
            for (int k = 1; k <= K; ++k)
                solver.add(z3::implies(occ_[e.inst] && (slot == ctx_.int_val(k)),
                                       tl.V[k] == w_[f].at(e.inst)));
        }
        tl.seg_of = writers_before;

        timelines_.emplace(f, std::move(tl));
        ++timeline_fluents_;
    }

    // --- Value reconstruction per read (C2/C3/C4) ---
    auto interval_sum = [&](ExprID f, int b, z3::expr lo_excl) -> z3::expr {
        // Sum of contributions c with lo_excl < tau(c) < tau(b), c != b.
        z3::expr acc = make_num(0);
        for (const Entry& e : add_entries_[f]) {
            if (e.inst == b) continue;
            z3::expr cond = occ_[e.inst] && (lo_excl < tau_[e.inst]) && (tau_[e.inst] < tau_[b]);
            acc = acc + z3::ite(cond, amt_[f].at(e.inst), make_num(0));
        }
        return acc;
    };

    for (int b = 0; b < N_ + 2; ++b) {
        if (b == B_) continue;  // bottom reads nothing
        for (const auto& [f, vbf] : v_[b]) {
            const std::vector<Entry>& wents = writer_entries_[f];
            bool numeric = problem_.is_numeric_type(f);

            if (wents.empty()) {
                // Only writer is bottom -> unconditional value (state equation /
                // constant).  Pure-additive numerics and never-assigned facts.
                z3::expr val = w_[f].at(B_);
                if (numeric) val = val + interval_sum(f, b, tau_[B_]);
                solver.add(z3::implies(occ_[b], vbf == val));
                continue;
            }

            // M6b: timeline realisation — attach the read to a segment instead of
            // linking it to a writer.  seg(b) = k iff the slot-k writer is the
            // last one before b, i.e. rank_at(k) < τ(b) < rank_at(k+1).  Those
            // brackets are mutually exclusive and exhaustive because rank_at is
            // strictly increasing over occupied slots and N+2 afterwards, so
            // exactly-one plus the forward implication pins the right segment.
            // No C3 protection at all: "no writer in between" is now a property
            // of the chain, not a quadratic over writer pairs.
            auto tlit = timelines_.find(f);
            if (tlit != timelines_.end()) {
                // seg(b) = number of occurring writers strictly before b, so
                // V(seg(b)) is the output of the last one.  Strictness excludes
                // b's own write when b both reads and writes f — the case that
                // made the first cut return UNSAT on refinery.  No C3 protection
                // at all: "no writer in between" is now arithmetic on the count,
                // not a quadratic over writer pairs.
                const Timeline& tl = tlit->second;
                z3::expr seg = tl.seg_of(tau_[b]);
                for (int k = 0; k <= tl.K; ++k)
                    solver.add(z3::implies(occ_[b] && (seg == ctx_.int_val(k)), vbf == tl.V[k]));
                continue;
            }

            // Threat set: bottom + every ordinary writer instance != b.  C3 must
            // range over ALL of these however the *support* set below is
            // restricted — a writer that cannot supply the value the guard wants
            // can still clobber it, so shrinking one set must not shrink the
            // other (M6i-a).
            std::vector<int> threats;
            threats.push_back(B_);
            for (const Entry& e : wents) if (e.inst != b) threats.push_back(e.inst);

            const bool no_contrib = add_entries_[f].empty();
            std::unordered_map<int, ExprID> const_write;
            const_write.emplace(B_, init_const_id(f));
            for (const Entry& e : wents) const_write.emplace(e.inst, e.const_write);
            auto const_write_of = [&](int x) {
                auto it = const_write.find(x);
                return it == const_write.end() ? EXPR_NULL : it->second;
            };

            // M6i-b — threat restriction.  Protection between two writers that
            // assign the *same constant* is vacuous when f has no contributors:
            // whichever of them is last, the value b sees is identical, so C4
            // stays correct.  Dropping the clause is a pure relaxation, and on a
            // monotone predicate (one only ever assigned true, e.g. rover's
            // `(available ?r)`) it removes the entire r_f·w_f² quadratic.
            auto same_constant = [&](int x, int y) {
                if (!no_contrib) return false;
                ExprID cx = const_write_of(x), cy = const_write_of(y);
                return cx != EXPR_NULL && cx == cy;
            };

            // M6i-a — support restriction.  If the guard pins f to a constant,
            // the last writer must have left that value there, so writers whose
            // constant output differs cannot be the support.  Unlike M6i-b this
            // *strengthens* the formula, so it is confined to what the argument
            // licenses: Boolean fluents (hence no contributors, so the value b
            // sees is exactly the last writer's output), and writers whose RHS is
            // a known constant — a dynamic-RHS writer could produce anything and
            // is always kept.
            std::vector<int> support;
            std::optional<bool> req = required_bool_for(b, f);
            if (req.has_value()) {
                for (int a : threats) {
                    ExprID cw = const_write_of(a);
                    if (cw != EXPR_NULL && pool_.payload_is_bool(cw) &&
                        pool_.payload_bool(cw) != *req) {
                        ++support_dropped_;
                        continue;
                    }
                    support.push_back(a);
                }
            } else {
                support = threats;
            }

            if (support.empty()) {
                // Nothing can supply the demanded value: b is provably
                // inapplicable.  (Sound — every dropped writer had a constant
                // output that differs, and bottom's initial value differs too.)
                ++support_empty_;
                solver.add(!occ_[b]);
                continue;
            }

            // Link vars.
            std::unordered_map<int, z3::expr> L;
            z3::expr_vector at_least(ctx_);
            for (int a : support) {
                std::string nm = "L_" + std::to_string(a) + "_" + std::to_string(b) + "_" + std::to_string(f.id);
                z3::expr la = ctx_.bool_const(nm.c_str());
                L.emplace(a, la);
                at_least.push_back(la);
                // L(a,b,f) -> occ(a) & tau(a) < tau(b)
                solver.add(z3::implies(la, occ_[a] && (tau_[a] < tau_[b])));
            }
            // C2 existence + exclusivity (guarded by occ(b)).
            solver.add(z3::implies(occ_[b], z3::mk_or(at_least)));
            for (size_t x = 0; x < support.size(); ++x)
                for (size_t y = x + 1; y < support.size(); ++y)
                    solver.add(!L.at(support[x]) || !L.at(support[y]));

            for (int a : support) {
                // C3 protection — over the full threat set.
                for (int c : threats) {
                    if (c == a) continue;
                    if (same_constant(a, c)) { ++protection_skipped_; continue; }
                    z3::expr promote = (tau_[b] <= tau_[c]);
                    z3::expr demote = (tau_[c] < tau_[a]);
                    solver.add(z3::implies(L.at(a) && occ_[c], demote || promote));
                }
                // C4 value.
                z3::expr val = w_[f].at(a);
                if (numeric) val = val + interval_sum(f, b, tau_[a]);
                solver.add(z3::implies(L.at(a), vbf == val));
            }
        }
    }

    // --- C1: distinct ranks for statically interfering occurring pairs ---
    {
        std::unordered_set<uint64_t> emitted;
        // Group instances by touched fluent (from the classification touch map).
        std::unordered_map<ExprID, std::vector<int>> touchers;
        for (int i = 0; i < N_; ++i) {
            auto it = cls_.touch.find(inst_[i].action->id());
            if (it == cls_.touch.end()) continue;
            for (const auto& [f, t] : it->second) touchers[f].push_back(i);
        }
        for (const auto& [f, insts] : touchers) {
            const auto& tmap = cls_.touch;
            for (size_t x = 0; x < insts.size(); ++x) {
                int i = insts[x];
                const GseTouch& ti = tmap.at(inst_[i].action->id()).at(f);
                for (size_t y = x + 1; y < insts.size(); ++y) {
                    int j = insts[y];
                    const GseTouch& tj = tmap.at(inst_[j].action->id()).at(f);
                    if (!gse_touches_interfere(ti, tj)) continue;
                    uint64_t key = (static_cast<uint64_t>(std::min(i, j)) << 32) | static_cast<uint32_t>(std::max(i, j));
                    if (!emitted.insert(key).second) continue;
                    solver.add(z3::implies(occ_[i] && occ_[j], tau_[i] != tau_[j]));
                }
            }
        }
    }

    // --- C6 guards + goal ---
    for (int i = 0; i < N_; ++i) {
        const Action* a = inst_[i].action;
        if (a->has_precondition())
            solver.add(z3::implies(occ_[i], translate(a->precondition_id(), subst_of(i))));
    }
    {
        z3::expr_vector goal_parts(ctx_);
        for (const Goal& g : problem_.goals())
            goal_parts.push_back(translate(g.goal_id(), subst_of(T_)));
        if (!goal_parts.empty()) solver.add(z3::mk_and(goal_parts));
    }

    // --- C8: redundant count-landmark cuts (order-free necessary conditions) ---
    // Sum of achiever firings weighted by per-firing mass must reach residual.
    for (const LandmarkInfo& lm : landmarks_) {
        z3::expr acc = make_num(0);
        bool any = false;
        for (size_t j = 0; j < lm.achiever_action_ids.size(); ++j) {
            int aid = lm.achiever_action_ids[j];
            auto it = inst_of_.find(aid);
            if (it == inst_of_.end()) continue;
            double contrib = (j < lm.contributions.size()) ? lm.contributions[j] : 1.0;
            for (int inst : it->second) {
                acc = acc + z3::ite(occ_[inst], make_num_d(contrib), make_num(0));
                any = true;
            }
        }
        if (any) solver.add(acc >= make_num_d(lm.residual));
    }

    // --- Domain cuts: declared numeric fluent bounds (soundness for bounded
    // int/real value types; pruning otherwise).  Assert on every materialised
    // value (v seen, w written). ---
    auto declared_bounds = [&](ExprID f) -> std::pair<std::optional<double>, std::optional<double>> {
        if (!problem_.is_numeric_type(f)) return {std::nullopt, std::nullopt};
        const ExprNode& head = pool_.head_symbol_node(f);
        if (!std::holds_alternative<std::string>(head.payload)) return {std::nullopt, std::nullopt};
        const Fluent* fl = problem_.find_fluent(std::get<std::string>(head.payload));
        if (!fl || !fl->has_declared_bounds()) return {std::nullopt, std::nullopt};
        return {fl->declared_lower_bound(), fl->declared_upper_bound()};
    };
    auto bound_var = [&](ExprID f, const z3::expr& var, const z3::expr& guard) {
        auto [lo, hi] = declared_bounds(f);
        if (lo) solver.add(z3::implies(guard, var >= make_num_d(*lo)));
        if (hi) solver.add(z3::implies(guard, var <= make_num_d(*hi)));
    };
    for (int b = 0; b < N_ + 2; ++b) {
        if (b == B_) continue;
        for (const auto& [f, vbf] : v_[b]) bound_var(f, vbf, occ_[b]);
    }
    for (const auto& [f, wmap] : w_) {
        for (const auto& [inst, wv] : wmap) {
            if (inst == B_) continue;  // bottom output is the (in-range) init value
            bound_var(f, wv, occ_[inst]);
        }
    }
}

z3::expr_vector Round::assumptions() {
    z3::expr_vector v(ctx_);
    for (const auto& [aid, cap] : cap_of_) v.push_back(cap);
    return v;
}

Plan Round::extract(z3::model& model) {
    std::vector<std::pair<long long, const Action*>> occurring;
    for (int i = 0; i < N_; ++i) {
        z3::expr ov = model.eval(occ_[i], true);
        if (!ov.is_true()) continue;
        long long rank = model.eval(tau_[i], true).get_numeral_int64();
        occurring.emplace_back(rank, inst_[i].action);
    }
    std::stable_sort(occurring.begin(), occurring.end(),
                     [](const auto& a, const auto& b) { return a.first < b.first; });
    Plan plan;
    for (const auto& [rank, action] : occurring) plan.add_action(action);
    return plan;
}

}  // namespace

// ===========================================================================
// GsePlanner
// ===========================================================================

GsePlanner::GsePlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx)
    : BasePlanner(problem, encoder, ctx) {
    // Inherited encoder_/solver_ accepted and ignored (F9).
}

Plan GsePlanner::search() {
    auto& config = Config::instance();
    auto& stats = Stats::instance();
    auto& log = Logger::instance();

    // ---- M1: static classification + E1 metrics + applicability guard ----
    cls_ = gse_classify(problem_);
    GseCostProfile cost = gse_record_e1_stats(problem_, cls_);

    auto reason = gse_applicability_reason(problem_, cls_, GseScope::M4);
    {
        std::ostringstream e1;
        e1 << "[GSE-E1] problem=" << problem_.problem_name()
           << " actions=" << problem_.actions().size()
           << " fluents=" << cls_.fluents.size()
           << " assign_only=" << stats.get("gse.fluents.assign_only")
           << " pure_additive=" << stats.get("gse.fluents.pure_additive")
           << " mixed=" << stats.get("gse.fluents.mixed")
           << " wf_max=" << stats.get("gse.numeric.wf_max")
           << " bool_wf_max=" << stats.get("gse.bool.wf_max")
           << " df_max=" << stats.get("gse.numeric.df_max")
           << " rf_max=" << stats.get("gse.reads.rf_max")
           << " est_terms=" << static_cast<long long>(cost.est_terms)
           << " top=" << (cost.top_fluent.empty() ? "-" : cost.top_fluent)
           << ":" << static_cast<long long>(cost.top_terms)
           << " self_ref=" << stats.get("gse.self_referential")
           << " dyn_amt=" << (cls_.any_dynamic_amount ? 1 : 0)
           << " dyn_wr=" << (cls_.any_dynamic_writer ? 1 : 0)
           << " nonlin=" << (cls_.any_nonlinear ? 1 : 0)
           << " cond=" << (cls_.any_conditional_effect ? 1 : 0)
           << " verdict=" << (reason ? ("REJECT(" + *reason + ")") : std::string("ACCEPT"));
        log.info(e1.str());
    }
    if (reason) {
        throw std::invalid_argument("gse: out of scope — " + *reason);
    }

    // ---- Budget loop (M3: core-guided growth, landmark-initialised) ----
    init_deadline();
    initialise_budget_from_landmarks();

    const bool core_guided = config.gse.core_guided;
    const int cap_total = config.gse.max_instances * static_cast<int>(problem_.actions().size());
    // Provisional formula-size ceiling (measurement subject, not a conclusion):
    // refuse to build a round whose estimated C2/C3/C4 term count exceeds this,
    // so a read/writer-dense instance is an honest logged E-finding rather than
    // an OOM.  --gse-max-terms.
    const long long kMaxTerms = config.gse.max_terms;

    // Heap ceiling (--gse-max-memory-mb).  The term estimate above only bounds
    // the *formula*; Z3's search can grow well past it on a dense instance, and
    // an unbounded round takes the whole machine down during a parallel sweep.
    //
    // Deliberately NOT Z3's own `memory_max_size`: that raises `out_of_memory_error`
    // from inside the allocator, which is not translated at every call site and
    // reaches std::terminate — killing the process (and, in a sweep, the whole
    // ProcessPoolExecutor with BrokenProcessPool).  Instead a watchdog thread polls
    // Z3's allocation counter and *interrupts* the round, which surfaces as a
    // benign `unknown` we can handle.
    const unsigned long long mem_cap_bytes =
        static_cast<unsigned long long>(std::max(0, config.gse.max_memory_mb)) * 1024ull * 1024ull;
    if (mem_cap_bytes > 0)
        log.info("GSE: heap watchdog at " + std::to_string(config.gse.max_memory_mb) + " MB");
    int round = 0;
    int rounds_since_uniform = 0;
    double eps = config.global.epsilon;
    log.info(std::string("GSE: budget loop starting (") +
             (core_guided ? "core-guided" : "uniform") + ", timeout " +
             format_timeout_string() + ")");

    auto copies_of = [&](int aid) -> long long {
        auto it = copies_.find(aid);
        return it == copies_.end() ? 1 : it->second;
    };
    // Per-read cost ~ protection (ordered writer pairs) + interval sums (w*d).
    //
    // This models M6i-b exactly: protection between two writers assigning the
    // *same constant* is never emitted, so those ordered pairs are subtracted.
    // On a monotone predicate (every writer assigns the same value) the whole
    // protection term therefore vanishes, which is the point — the previous
    // `w*w` estimate ignored both M6i analyses and would keep rejecting
    // instances whose formula they have already shrunk.
    //
    // It deliberately does NOT model M6i-a's further support restriction, so the
    // result stays an upper bound on what gets emitted.  The guard is a
    // memory-safety net; it must never be optimistic.
    auto fluent_total_cost = [&](ExprID f, long long reads) -> long long {
        auto it = cls_.fluents.find(f);
        if (it == cls_.fluents.end()) return reads;
        const GseFluentClass& fc = it->second;

        long long d = 0;
        for (const auto& ad : fc.adds) d += copies_of(ad.action_id);
        const bool no_contrib = fc.adds.empty();

        // Partition writer instances by the constant they assign; bottom joins
        // its own initial value's class.  Non-constant writers join no class.
        std::unordered_map<ExprID, long long> classes;
        if (no_contrib) {
            ExprID bi = init_const_id_of(problem_, f);
            if (bi != EXPR_NULL) classes[bi] += 1;
        }
        long long w_inst = 0;
        for (const auto& wr : fc.writers) {
            long long k = copies_of(wr.action_id);
            w_inst += k;
            if (no_contrib && wr.value_kind == ValueKind::CONSTANT) {
                ExprID cw = problem_.action(wr.action_id).effect(wr.effect_index)
                                .effect_expression().value_id();
                classes[cw] += k;
            }
        }
        std::vector<long long> sizes;
        sizes.reserve(classes.size());
        for (const auto& [id, n] : classes) sizes.push_back(n);
        return realisation_cost(w_inst, reads, d, sizes).best();
    };

    while (true) {
        ++round;
        int total_copies = 0;
        for (const auto& [aid, k] : copies_) total_copies += k;

        // Sentinels (and their formula overhead) only pay off once we need to
        // localise growth: skip them in round 1, where an instance that is SAT
        // at the initialised budget should solve without doubling the instances.
        bool cg_round = core_guided && round > 1;

        // Formula-size guard (memory protection).  Costed per *fluent*, since a
        // timeline's channelling is paid once per fluent while its attachment is
        // per read — a per-read cost cannot express that.
        std::unordered_map<ExprID, long long> reads_of;
        for (const Action& a : problem_.actions()) {
            long long k = copies_of(a.id()) + (cg_round ? 1 : 0);
            auto rit = cls_.action_reads.find(a.id());
            if (rit == cls_.action_reads.end()) continue;
            std::unordered_set<ExprID> seen;
            for (ExprID f : rit->second)
                if (seen.insert(f).second) reads_of[f] += k;
        }
        for (ExprID f : cls_.goal_reads) reads_of[f] += 1;
        long long est = 0;
        for (const auto& [f, r] : reads_of) est += fluent_total_cost(f, r);
        if (est > kMaxTerms) {
            log.info("GSE: estimated formula size " + std::to_string(est) +
                     " terms exceeds cap " + std::to_string(kMaxTerms) +
                     " (read/writer-dense); giving up.");
            stats.set("gse.gave_up_formula_size", 1);
            break;
        }

        // Fresh context per round (not the inherited ctx_): each round rebuilds
        // the whole formula, so reusing one context would accumulate every
        // round's terms and leak memory linearly in rounds.  A per-round context
        // frees all terms when the round ends.
        z3::context round_ctx;
        z3::solver solver(round_ctx);
        {
            z3::params p(round_ctx);
            p.set("random_seed", 42u);
            // Deliberately do NOT set a difference-logic hint: unlike the step
            // encoding, GSE's C4 value equations are interval *sums* (general
            // linear arithmetic, QF_LIA/QF_LRA), so a QF_IDL/QF_RDL logic makes
            // Z3 reject the benchmark as out-of-fragment.  Let Z3 auto-configure.
            solver.set(p);
        }
        if (!apply_solver_timeout(solver)) break;  // timed out

        Round r(problem_, cls_, copies_, landmark_data_.landmarks, cg_round, round_ctx);
        z3::check_result res;
        double solve_s = 0.0;
        auto build_start = std::chrono::high_resolution_clock::now();

        // Heap watchdog: poll Z3's allocation counter and interrupt this round if
        // it crosses the ceiling.  Covers both build and check.
        std::atomic<bool> round_done{false};
        std::atomic<bool> memout{false};
        std::thread watchdog;
        if (mem_cap_bytes > 0) {
            watchdog = std::thread([&]() {
                while (!round_done.load(std::memory_order_relaxed)) {
                    if (Z3_get_estimated_alloc_size() > mem_cap_bytes) {
                        memout.store(true, std::memory_order_relaxed);
                        round_ctx.interrupt();
                        return;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            });
        }
        struct WatchdogJoin {
            std::atomic<bool>& done; std::thread& t;
            ~WatchdogJoin() { done.store(true, std::memory_order_relaxed); if (t.joinable()) t.join(); }
        } watchdog_join{round_done, watchdog};

        try {
            r.build(solver);
            z3::expr_vector asmpt = r.assumptions();
            auto t0 = std::chrono::high_resolution_clock::now();
            res = asmpt.empty() ? solver.check() : solver.check(asmpt);
            auto t1 = std::chrono::high_resolution_clock::now();
            solve_s = std::chrono::duration<double>(t1 - t0).count();
            // Same accounting as the step planners (solve_time = check() only,
            // total_time = build + check), so sweep timings are comparable.
            stats.add("planner.solve_time", solve_s);
            stats.add("planner.total_time",
                      std::chrono::duration<double>(t1 - build_start).count());
        } catch (const z3::exception& e) {
            log.info(std::string("GSE: z3 exception during round ") +
                     std::to_string(round) + " (" + e.msg() + "); giving up.");
            stats.set("gse.gave_up_z3_exception", 1);
            break;
        } catch (const std::exception& e) {
            // Includes std::bad_alloc: degrade to give-up rather than crash the
            // process (protects the ProcessPoolExecutor worker during sweeps).
            log.info(std::string("GSE: exception during round ") +
                     std::to_string(round) + " (" + e.what() + "); giving up.");
            stats.set("gse.gave_up_exception", 1);
            break;
        }

        stats.set("gse.m6i_b.protection_skipped", static_cast<double>(r.protection_skipped()));
        stats.set("gse.m6i_a.support_dropped", static_cast<double>(r.support_dropped()));
        stats.set("gse.m6i_a.support_empty", static_cast<double>(r.support_empty()));
        stats.set("gse.m6b.timeline_fluents", static_cast<double>(r.timeline_fluents()));
        if (r.support_empty() > 0)
            log.info("GSE: " + std::to_string(r.support_empty()) +
                     " read(s) had no possible supporter — those instances forced dormant.");
        log.info("GSE round " + std::to_string(round) + ": copies=" +
                 std::to_string(total_copies) + " instances=" +
                 std::to_string(r.num_ordinary()) + " skipped_c3=" +
                 std::to_string(r.protection_skipped()) + " dropped_links=" +
                 std::to_string(r.support_dropped()) + " timelines=" +
                 std::to_string(r.timeline_fluents()) + " -> " +
                 (res == z3::sat ? "SAT" : res == z3::unsat ? "UNSAT" : "UNKNOWN") +
                 " (" + std::to_string(solve_s) + "s)");

        if (res == z3::sat) {
            z3::model model = solver.get_model();
            Plan plan = r.extract(model);

            // D7: exact simulation before returning.
            Simulator sim(problem_, eps);
            std::string err = sim.run(plan.actions());
            if (!err.empty()) {
                throw std::runtime_error("gse: extracted plan failed simulation — " + err +
                                         " (encoding/simulator mismatch)");
            }

            solution_found_ = true;
            stats.set("planner.rounds", round);
            stats.set("planner.solution_horizon", static_cast<double>(r.num_ordinary()));
            stats.set("gse.final_instances", static_cast<double>(r.num_ordinary()));
            stats.set("gse.final_copies", static_cast<double>(total_copies));
            stats.set("gse.plan_length", static_cast<double>(plan.length()));

            plan.write_ipc(config.planner.output_plan, 1, -1.0, true,
                           config.planner.strategy, config.planner.mode, 0.0);
            notify_intermediate_plan(plan);
            log.info("GSE: plan found with " + std::to_string(plan.length()) +
                     " actions at " + std::to_string(total_copies) + " copies.");
            return plan;
        }

        if (res == z3::unknown) {
            // A memout is terminal: the next round is strictly larger, so it
            // would hit the same ceiling immediately.  Stop instead of spinning.
            std::string why = solver.reason_unknown();
            if (memout.load(std::memory_order_relaxed) ||
                why.find("memory") != std::string::npos ||
                why.find("memout") != std::string::npos) {
                log.info("GSE: round " + std::to_string(round) +
                         " hit the heap cap (" + why + "); giving up.");
                stats.set("gse.gave_up_memory", 1);
                break;
            }
            if (handle_unknown_result(solver, "round " + std::to_string(round)))
                break;  // timeout
        }

        // UNSAT (or benign unknown): grow the budget.
        if (total_copies >= cap_total) {
            log.info("GSE: max instance budget reached; giving up.");
            break;
        }

        // Core-guided: grow exactly the actions whose sentinel appears in the
        // core; a cap-free core (or benign unknown) means one-more-of-everything
        // would not localise, so grow uniformly.  Force uniform every Nth round
        // as a completeness safeguard (cores are not minimal).
        bool forced_uniform = (config.gse.uniform_every > 0 &&
                               rounds_since_uniform + 1 >= config.gse.uniform_every);
        std::vector<int> grow;
        if (cg_round && res == z3::unsat && !forced_uniform) {
            z3::expr_vector core = solver.unsat_core();
            std::unordered_set<int> seen;
            for (unsigned i = 0; i < core.size(); ++i) {
                int aid = r.cap_action(core[i].id());
                if (aid >= 0 && seen.insert(aid).second) grow.push_back(aid);
            }
        }
        if (!grow.empty()) {
            for (int aid : grow) copies_[aid] += 1;
            ++rounds_since_uniform;
            stats.set("gse.last_core_grow", static_cast<double>(grow.size()));
        } else {
            for (auto& [aid, k] : copies_) ++k;
            rounds_since_uniform = 0;
        }
    }

    stats.set("planner.rounds", round);
    return Plan();
}

void GsePlanner::consume_artifacts(PreprocessedProblem& artifacts) {
    landmark_data_ = artifacts.landmark_data;
}

void GsePlanner::initialise_budget_from_landmarks() {
    auto& config = Config::instance();
    for (const Action& a : problem_.actions())
        copies_[a.id()] = std::max(1, config.gse.initial_copies);

    // A count landmark with a single achiever gives an exact per-action firing
    // lower bound: ceil(residual / per-firing contribution).  Multi-achiever
    // landmarks are left to core-guided growth (distribution is unknown here).
    for (const LandmarkInfo& lm : landmark_data_.landmarks) {
        if (lm.achiever_action_ids.size() != 1) continue;
        int aid = lm.achiever_action_ids[0];
        double contrib = lm.contributions.empty() ? 1.0 : lm.contributions[0];
        if (contrib <= 0.0) continue;
        int need = static_cast<int>(std::ceil(lm.residual / contrib - 1e-9));
        need = std::max(1, std::min(need, config.gse.max_instances));
        auto it = copies_.find(aid);
        if (it != copies_.end()) it->second = std::max(it->second, need);
    }
}

} // namespace rantanplan
