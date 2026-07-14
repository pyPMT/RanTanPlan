#include "axiom_dependency_analysis.hpp"
#include "../problem/expr_enums.hpp"

#include <algorithm>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace rantanplan {

int AxiomInfo::index_of(ExprID atom) const {
    for (size_t i = 0; i < atoms.size(); ++i) {
        if (atoms[i] == atom) return static_cast<int>(i);
    }
    return -1;
}

bool AxiomInfo::is_recursive(ExprID atom) const {
    int i = index_of(atom);
    return i >= 0 && recursive[i];
}

namespace {

bool is_derived_atom(const Problem& p, ExprID e) {
    const Type* t = p.type_for_id(e);
    return t && t->is_derived_bool();
}

/// Collect derived atoms in expression `e`, each tagged with whether it occurs
/// under an odd number of negations. For FUNCTION_APPLICATION nodes children[0]
/// is the head symbol and children[1..] are the arguments.
void collect_derived_polarity(const Problem& p, ExprID e, bool neg,
                              std::vector<std::pair<ExprID, bool>>& out) {
    const ExprNode& node = p.pool().get(e);
    auto kind = static_cast<ExprKind>(node.kind);
    if (kind == ExprKind::STATE_VARIABLE) {
        if (is_derived_atom(p, e)) out.emplace_back(e, neg);
        return;
    }
    if (kind != ExprKind::FUNCTION_APPLICATION) return;  // constants, params, ...

    const auto& c = node.children;
    auto op = static_cast<ExprOperator>(node.op);
    if (op == ExprOperator::NOT) {
        if (c.size() >= 2) collect_derived_polarity(p, c[1], !neg, out);
        return;
    }
    if (op == ExprOperator::IMPLIES) {
        if (c.size() >= 3) {
            collect_derived_polarity(p, c[1], !neg, out);  // antecedent flips
            collect_derived_polarity(p, c[2], neg, out);
        }
        return;
    }
    if (op == ExprOperator::IFF) {
        for (size_t i = 1; i < c.size(); ++i) {  // each side occurs both ways
            collect_derived_polarity(p, c[i], neg, out);
            collect_derived_polarity(p, c[i], !neg, out);
        }
        return;
    }
    // AND, OR, quantifiers, comparisons, arithmetic: recurse arguments with the
    // same polarity (comparison/arithmetic subtrees hold no derived atoms).
    for (size_t i = 1; i < c.size(); ++i) {
        collect_derived_polarity(p, c[i], neg, out);
    }
}

}  // namespace

AxiomInfo analyze_axioms(const Problem& problem) {
    AxiomInfo info;
    const auto& axioms = problem.axioms();
    if (axioms.empty()) return info;

    // 1. TDG nodes: every derived atom appearing as a head or in a body. Record
    //    per-axiom body derived atoms (with polarity) for edges + stratification.
    std::vector<std::vector<std::pair<ExprID, bool>>> body_derived(axioms.size());
    std::unordered_set<ExprID> node_set;
    for (size_t a = 0; a < axioms.size(); ++a) {
        node_set.insert(axioms[a].head);
        collect_derived_polarity(problem, axioms[a].body, false, body_derived[a]);
        for (const auto& [d, neg] : body_derived[a]) node_set.insert(d);
    }

    info.atoms.assign(node_set.begin(), node_set.end());
    std::sort(info.atoms.begin(), info.atoms.end());  // deterministic node order
    std::unordered_map<ExprID, int> idx;
    for (size_t i = 0; i < info.atoms.size(); ++i) idx[info.atoms[i]] = static_cast<int>(i);
    int n = static_cast<int>(info.atoms.size());

    // 2. Edges D -> H (D in the body of an axiom defining H); self-loops apart.
    std::vector<std::vector<int>> adj(n);
    std::vector<bool> self_loop(n, false);
    for (size_t a = 0; a < axioms.size(); ++a) {
        int h = idx[axioms[a].head];
        std::unordered_set<int> seen;
        for (const auto& [d, neg] : body_derived[a]) {
            int di = idx[d];
            if (di == h) { self_loop[h] = true; continue; }
            if (seen.insert(di).second) adj[di].push_back(h);
        }
    }

    // 3. Tarjan SCC.
    info.scc_id.assign(n, -1);
    std::vector<int> low(n, -1), num(n, -1);
    std::vector<bool> on_stack(n, false);
    std::vector<int> stk;
    int counter = 0, scc_count = 0;
    std::function<void(int)> dfs = [&](int u) {
        num[u] = low[u] = counter++;
        stk.push_back(u);
        on_stack[u] = true;
        for (int v : adj[u]) {
            if (num[v] == -1) {
                dfs(v);
                low[u] = std::min(low[u], low[v]);
            } else if (on_stack[v]) {
                low[u] = std::min(low[u], num[v]);
            }
        }
        if (low[u] == num[u]) {
            while (true) {
                int w = stk.back();
                stk.pop_back();
                on_stack[w] = false;
                info.scc_id[w] = scc_count;
                if (w == u) break;
            }
            ++scc_count;
        }
    };
    for (int i = 0; i < n; ++i) if (num[i] == -1) dfs(i);
    info.num_sccs = scc_count;

    // 4. Recursive iff the SCC has size > 1, or the atom has a self-loop.
    std::vector<int> scc_size(scc_count, 0);
    for (int i = 0; i < n; ++i) ++scc_size[info.scc_id[i]];
    info.recursive.assign(n, false);
    for (int i = 0; i < n; ++i) {
        info.recursive[i] = scc_size[info.scc_id[i]] > 1 || self_loop[i];
    }

    // 5. Stratification: a negated derived atom in the same SCC as its head makes
    //    the fixpoint non-unique (not a stratified program).
    for (size_t a = 0; a < axioms.size(); ++a) {
        int h = idx[axioms[a].head];
        for (const auto& [d, neg] : body_derived[a]) {
            if (neg && info.scc_id[idx[d]] == info.scc_id[h]) info.stratified = false;
        }
    }

    return info;
}

}  // namespace rantanplan
