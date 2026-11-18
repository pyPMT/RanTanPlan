#include "exists_semantics.hpp"
#include "../../problem/action.hpp"
#include "../../util/logger.hpp"
#include "../../util/scoped_timer.hpp"
#include "../../util/stats.hpp"
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace rantanplan {

/// The step order, as action ids. For each one-way pair u -> v (u affects v,
/// v does not affect u) v should run first, so that both can share a step:
///
///     use   : pre p    eff q := true
///     spoil :          eff p := false
///
///     spoil affects use (it makes p false), and use does not affect spoil.
///     In the order use, spoil both can share a step; in spoil, use not.
///
/// Finding the order that breaks fewest pairs is the minimum feedback arc set
/// problem, NP-hard in general; this is a greedy topological sort. It picks
/// the actions one at a time, from the first to run to the last. O(n^2 + pairs).
static std::vector<int> step_order(int n, const std::vector<std::pair<int, int>>& one_way) {
    // affected[x]: the actions x affects; affecting[x]: the actions that
    // affect x. affects_left[x] and affected_by_left[x] count those of them
    // not yet in the order: x's out- and in-degree among the actions left.
    std::vector<std::vector<int>> affected(n), affecting(n);
    std::vector<int> affects_left(n, 0), affected_by_left(n, 0);
    for (auto [u, v] : one_way) {
        affected[u].push_back(v);
        affecting[v].push_back(u);
        ++affects_left[u];
        ++affected_by_left[v];
    }

    // The best action to run next. One that affects no action left breaks no
    // pair by running now, so it comes first. If there is none, the actions
    // left affect each other in a cycle and some pair must break: take the
    // one with the highest in-degree minus out-degree, most affected and
    // least affecting, which keeps the most pairs. Ties go to the lower id,
    // so the order is the same on every run.
    //
    //     one-way pairs   a -> b,  b -> c,  c -> a      (a cycle: one breaks)
    //
    //     every action affects one action left: a tie at 1 - 1 = 0, so a
    //     runs first and breaks a -> b. Now c affects nothing left (a is
    //     placed), so c runs next, then b. Order a, c, b: only a -> b is
    //     forbidden.
    auto better = [&](int x, int y) {
        if ((affects_left[x] == 0) != (affects_left[y] == 0)) return affects_left[x] == 0;
        return affected_by_left[x] - affects_left[x] > affected_by_left[y] - affects_left[y];
    };

    std::vector<int> order;
    std::vector<char> placed(n, 0);
    while (static_cast<int>(order.size()) < n) {
        int next = -1;
        for (int x = 0; x < n; ++x) {
            if (!placed[x] && (next < 0 || better(x, next))) next = x;
        }
        order.push_back(next);
        placed[next] = 1;
        // `next` no longer counts as an action left.
        for (int v : affected[next]) --affected_by_left[v];
        for (int u : affecting[next]) --affects_left[u];
    }
    return order;
}

ExistsStepOrder::ExistsStepOrder(const Graph& graph) {
    ScopedTimer timer("estep.order_time_ms");
    const int n = static_cast<int>(graph.num_nodes());
    const std::vector<int> scc = graph.get_scc_mapping();

    // The one-way pairs inside a component, the only pairs the order decides.
    std::unordered_set<uint64_t> edges;
    auto key = [](int u, int v) {
        return (static_cast<uint64_t>(u) << 32) | static_cast<uint32_t>(v);
    };
    for (int u = 0; u < n; ++u) {
        for (int v : graph.get_neighbours(u)) edges.insert(key(u, v));
    }
    std::vector<std::pair<int, int>> one_way;
    for (int u = 0; u < n; ++u) {
        for (int v : graph.get_neighbours(u)) {
            if (u != v && scc[u] == scc[v] && !edges.count(key(v, u))) {
                one_way.emplace_back(u, v);
            }
        }
    }

    // The components one after another by id, each in the greedy order. An
    // edge u -> v across components has scc[v] < scc[u]
    // (Graph::get_scc_mapping), so v runs first: an edge u -> v is forbidden
    // exactly when u is placed before v.
    std::vector<int> order = step_order(n, one_way);
    std::stable_sort(order.begin(), order.end(), [&](int x, int y) { return scc[x] < scc[y]; });
    position_.assign(n, 0);
    for (int i = 0; i < n; ++i) position_[order[i]] = i;

    size_t forbidden = 0, broken = 0;
    for (int u = 0; u < n; ++u) {
        for (int v : graph.get_neighbours(u)) {
            if (forbids(u, v)) ++forbidden;
        }
    }
    for (auto [u, v] : one_way) {
        if (forbids(u, v)) ++broken;
    }

    auto& stats = Stats::instance();
    stats.set("estep.forbidden_pairs_per_step", static_cast<double>(forbidden));
    stats.set("estep.one_way_pairs", static_cast<double>(one_way.size()));
    stats.set("estep.one_way_pairs_broken", static_cast<double>(broken));
    Logger::instance().component(VerbosityLevel::INFO, "Exists", {
        {"forbidden pairs/step", std::to_string(forbidden)},
        {"one-way pairs", std::to_string(one_way.size())},
        {"broken", std::to_string(broken)},
        {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
    });
}

const ExistsStepOrder& ExistsSemantics::step_order() const {
    if (!graph_) {
        throw std::logic_error(
            "ExistsSemantics::step_order requires an eager "
            "(graph-bearing) interference analyzer");
    }
    if (!step_order_) {
        step_order_ = std::make_unique<ExistsStepOrder>(graph_->get_interference_graph());
    }
    return *step_order_;
}

void ExistsSemantics::prepare_pairs() {
    prepared_ = true;
    const ExistsStepOrder& order = step_order();
    const Graph& graph = graph_->get_interference_graph();
    for (int u = 0; u < static_cast<int>(graph.num_nodes()); ++u) {
        for (int v : graph.get_neighbours(u)) {
            if (order.forbids(u, v)) forbidden_.emplace_back(u, v);
        }
    }
}

z3::expr ExistsSemantics::encode_parallelism(int timestep) {
    // graph_ is non-null whenever encode_parallelism is reachable: lazy
    // analyzers require a propagator module that manages parallelism.
    if (!graph_) {
        throw std::logic_error(
            "ExistsSemantics::encode_parallelism requires an eager "
            "(graph-bearing) interference analyzer");
    }
    if (!prepared_) prepare_pairs();

    z3::expr_vector mutex_constraints(*ctx_);
    for (auto [u, v] : forbidden_) {
        z3::expr u_var = variable_factory_->get_action_variable(problem_->action(u), timestep);
        z3::expr v_var = variable_factory_->get_action_variable(problem_->action(v), timestep);
        mutex_constraints.push_back(!u_var || !v_var);
    }

    if (mutex_constraints.empty()) {
        return ctx_->bool_val(true);
    }

    auto& stats = Stats::instance();
    stats.add("encoder.mutex_constraints", mutex_constraints.size());

    return z3::mk_and(mutex_constraints);
}

const InterferenceOracle* ExistsSemantics::get_interference_analyzer() const {
    return interference_analyzer_.get();
}

} // namespace rantanplan
