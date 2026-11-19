#include "exists_semantics.hpp"
#include "../../problem/action.hpp"
#include "../../util/logger.hpp"
#include "../../util/scoped_timer.hpp"
#include "../../util/stats.hpp"
#include <algorithm>
#include <cstdint>
#include <optional>
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

void ExistsSemantics::prepare() {
    prepared_ = true;
    step_order();  // timed on its own, as estep.order_time_ms
    ScopedTimer timer("estep.prepare_time_ms");
    if (mutexes_ == EstepMutexes::Pairs) {
        prepare_pairs();
    } else {
        prepare_chains();
    }

    size_t clauses = forbidden_.size(), variables = 0;
    for (const Chain& chain : chains_) {
        clauses += chain.clauses.size();
        variables += chain.variables;
    }
    auto& stats = Stats::instance();
    stats.set("estep.clauses_per_step", static_cast<double>(clauses));
    stats.set("estep.chain_variables_per_step", static_cast<double>(variables));
    Logger::instance().component(VerbosityLevel::INFO, "Exists", {
        {"mutexes", mutexes_ == EstepMutexes::Pairs ? "pairs" : "chains"},
        {"clauses/step", std::to_string(clauses)},
        {"chain vars/step", std::to_string(variables)},
        {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
    });
}

void ExistsSemantics::prepare_pairs() {
    const ExistsStepOrder& order = step_order();
    const Graph& graph = graph_->get_interference_graph();
    for (int u = 0; u < static_cast<int>(graph.num_nodes()); ++u) {
        for (int v : graph.get_neighbours(u)) {
            if (order.forbids(u, v)) forbidden_.emplace_back(u, v);
        }
    }
}

void ExistsSemantics::prepare_chains() {
    // A unit's writer and a different affected action are an edge of the
    // graph, so the chain, which forbids each writer with every later
    // affected action, forbids exactly the unit's forbidden pairs. A pair
    // that two units forbid is forbidden by both chains.
    const ExistsStepOrder& order = step_order();
    auto by_position = [&](int x, int y) { return order.position(x) < order.position(y); };
    unit_roles_.assign(problem_->action_count(), {});
    for (auto& unit : interference_analyzer_->interference_units()) {
        std::sort(unit.writers.begin(), unit.writers.end(), by_position);
        std::sort(unit.affected.begin(), unit.affected.end(), by_position);
        const int index = static_cast<int>(chains_.size());
        for (int a : unit.writers) unit_roles_[a].emplace_back(index, Role::Writer);
        for (int a : unit.affected) unit_roles_[a].emplace_back(index, Role::Affected);
        add_chain(unit.writers, unit.affected);
    }
}

void ExistsSemantics::add_chain(const std::vector<int>& writers, const std::vector<int>& affected) {
    // Madagascar's ESTEPchain. Walking the step order, every writer seen so
    // far implies `head`, and the head implies the negation of each affected
    // action that comes next. The first writer is itself the head until a
    // second writer comes. A writer after an affected action must not reach
    // it, so it opens a new chain variable, which the old head implies; so
    // does the second writer, which must not imply the first. An action that
    // both writes and is affected is visited as affected first: the writers
    // before it forbid it, and as a writer it forbids only later actions.
    // The walk stops at the last affected action, as no writer after it has
    // anything to forbid.
    //
    //     step order   a    b    c    d    e    f    g
    //     writers      a         c    d    e         g
    //     affected          b              e    f
    //
    //     a   the first writer: the head
    //     b   a -> ~b
    //     c   a writer after b: new chain variable c0;   a -> c0,  c -> c0
    //     d   no affected action since c0 opened:        d -> c0
    //     e   affected: c0 -> ~e; then, as a writer after e: new c1;
    //                                                    c0 -> c1, e -> c1
    //     f   c1 -> ~f; the last affected action, so the walk stops
    //     g   not visited
    //
    //     forbidden: a-b, a-e, c-e, d-e, a-f, c-f, d-f, e-f, which are the
    //     pairs a writer -> later affected action. Here the chain takes 8
    //     clauses and 2 variables, as many clauses as the pairs; the saving
    //     grows with the numbers of writers and affected actions.
    const ExistsStepOrder& order = step_order();
    Chain& chain = chains_.emplace_back();
    auto action = [](int id) { return Literal{false, id, false}; };
    auto negation = [](Literal l) {
        l.negated = !l.negated;
        return l;
    };
    auto implies = [&](Literal from, Literal to) { chain.clauses.push_back({negation(from), to}); };

    // `closed`: a writer that comes now must not imply `head`, which is the
    // first writer or already implies the negation of an affected action.
    std::optional<Literal> head;
    bool closed = false;
    size_t w = 0, r = 0;
    while (r < affected.size()) {
        // The next action in step order. On a tie the two are one action,
        // which both writes and is affected: affected first.
        if (w < writers.size() && order.position(writers[w]) < order.position(affected[r])) {
            const Literal writer = action(writers[w++]);
            if (!head) {
                head = writer;
                closed = true;
            } else {
                if (closed) {
                    const Literal next{true, chain.variables++, false};
                    implies(*head, next);
                    head = next;
                    closed = false;
                }
                implies(writer, *head);
            }
        } else {
            if (head) {
                implies(*head, negation(action(affected[r])));
                closed = true;
            }
            ++r;
        }
    }
}

void ExistsSemantics::encode_chain(size_t unit, int timestep, z3::expr_vector& clauses) const {
    // The chain variables are new at every step, unlike the action variables,
    // which the factory keeps: each occurs only in its unit's clauses at its
    // step. State and action variables end in @<id>_<t> and !<id>_<t>, so
    // none has these names.
    const Chain& chain = chains_[unit];
    const std::string prefix = "estep.chain." + std::to_string(unit) + ".";
    const std::string suffix = "_" + std::to_string(timestep);
    std::vector<z3::expr> variables;
    variables.reserve(chain.variables);
    for (int k = 0; k < chain.variables; ++k) {
        variables.push_back(variable_factory_->create_bool_variable(prefix + std::to_string(k) + suffix));
    }

    auto term = [&](const Literal& l) {
        z3::expr t = l.chain
            ? variables[l.index]
            : variable_factory_->get_action_variable(problem_->action(l.index), timestep);
        return l.negated ? !t : t;
    };
    for (const auto& [a, b] : chain.clauses) {
        clauses.push_back(term(a) || term(b));
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
    if (!prepared_) prepare();

    z3::expr_vector mutex_constraints(*ctx_);
    for (auto [u, v] : forbidden_) {
        z3::expr u_var = variable_factory_->get_action_variable(problem_->action(u), timestep);
        z3::expr v_var = variable_factory_->get_action_variable(problem_->action(v), timestep);
        mutex_constraints.push_back(!u_var || !v_var);
    }
    for (size_t unit = 0; unit < chains_.size(); ++unit) {
        encode_chain(unit, timestep, mutex_constraints);
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
