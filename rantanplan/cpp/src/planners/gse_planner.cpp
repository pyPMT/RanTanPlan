#include "gse_planner.hpp"

#include "../config/config.hpp"
#include "../preprocessing/preprocessed_problem.hpp"
#include "../problem/const_fold.hpp"
#include "../util/logger.hpp"
#include "../util/stats.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
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
    std::unordered_map<int, std::vector<int>> inst_of_;  // action id -> instance idxs

    // v(b,f): per instance, fluent -> seen-value var.
    std::vector<std::unordered_map<ExprID, z3::expr>> v_;
    // w(a,f): fluent -> writer instance -> output var/const.
    std::unordered_map<ExprID, std::unordered_map<int, z3::expr>> w_;
    // amt(c,f): fluent -> contributor instance -> amount var/const.
    std::unordered_map<ExprID, std::unordered_map<int, z3::expr>> amt_;

    // Writer/contributor entries per fluent (ordinary instances + effect index).
    struct Entry { int inst; int effect_index; bool negated; };
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
            for (int inst : inst_of_[wr.action_id])
                writer_entries_[f].push_back({inst, wr.effect_index, false});
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

            // Candidate last-writers: bottom + ordinary writer instances != b.
            std::vector<int> cand;
            cand.push_back(B_);
            for (const Entry& e : wents) if (e.inst != b) cand.push_back(e.inst);

            // Link vars.
            std::unordered_map<int, z3::expr> L;
            z3::expr_vector at_least(ctx_);
            for (int a : cand) {
                std::string nm = "L_" + std::to_string(a) + "_" + std::to_string(b) + "_" + std::to_string(f.id);
                z3::expr la = ctx_.bool_const(nm.c_str());
                L.emplace(a, la);
                at_least.push_back(la);
                // L(a,b,f) -> occ(a) & tau(a) < tau(b)
                solver.add(z3::implies(la, occ_[a] && (tau_[a] < tau_[b])));
            }
            // C2 existence + exclusivity (guarded by occ(b)).
            solver.add(z3::implies(occ_[b], z3::mk_or(at_least)));
            for (size_t x = 0; x < cand.size(); ++x)
                for (size_t y = x + 1; y < cand.size(); ++y)
                    solver.add(!L.at(cand[x]) || !L.at(cand[y]));

            for (int a : cand) {
                // C3 protection.
                for (int c : cand) {
                    if (c == a) continue;
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
    gse_record_e1_stats(problem_, cls_);

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
           << " df_max=" << stats.get("gse.numeric.df_max")
           << " rf_max=" << stats.get("gse.reads.rf_max")
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
    // Per-read cost ~ protection (w^2) + interval sums (w*d), w = writer copies
    // (+bottom), d = contributor copies.
    auto fluent_read_cost = [&](ExprID f) -> long long {
        auto it = cls_.fluents.find(f);
        if (it == cls_.fluents.end()) return 1;
        long long w = 1, d = 0;
        for (const auto& wr : it->second.writers) w += copies_of(wr.action_id);
        for (const auto& ad : it->second.adds) d += copies_of(ad.action_id);
        return w * w + w * d + 4;
    };

    while (true) {
        ++round;
        int total_copies = 0;
        for (const auto& [aid, k] : copies_) total_copies += k;

        // Sentinels (and their formula overhead) only pay off once we need to
        // localise growth: skip them in round 1, where an instance that is SAT
        // at the initialised budget should solve without doubling the instances.
        bool cg_round = core_guided && round > 1;

        // Formula-size guard (memory protection).
        long long est = 0;
        for (const Action& a : problem_.actions()) {
            long long k = copies_of(a.id()) + (cg_round ? 1 : 0);
            auto rit = cls_.action_reads.find(a.id());
            if (rit == cls_.action_reads.end()) continue;
            std::unordered_set<ExprID> seen;
            for (ExprID f : rit->second)
                if (seen.insert(f).second) est += k * fluent_read_cost(f);
        }
        for (ExprID f : cls_.goal_reads) est += fluent_read_cost(f);
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

        log.info("GSE round " + std::to_string(round) + ": copies=" +
                 std::to_string(total_copies) + " instances=" +
                 std::to_string(r.num_ordinary()) + " -> " +
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
