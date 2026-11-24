#pragma once

#include "parallelism_strategy.hpp"
#include "../../analysis/interference_analysis.hpp"
#include "../../config/strategy_spec.hpp"
#include <memory>
#include <utility>
#include <vector>

namespace rantanplan {

/**
 * @brief The fixed step order of exists-step semantics
 *
 * The order is fixed in advance, the same at every step (Section 3.4.4 of the
 * paper cited on ExistsSemantics). An edge u -> v of the interference graph
 * (u affects v) is forbidden when u runs before v. The order runs the
 * strongly connected components one after another, the affected component
 * first for every edge across two of them, so such a pair is never
 * forbidden. A pair with edges both ways is forbidden whatever the order. So
 * the order matters only for the one-way pairs inside a component, and the
 * constructor picks one that runs the affected action first in as many of
 * them as it can.
 */
class ExistsStepOrder {
public:
    explicit ExistsStepOrder(const Graph& graph);

    /// For an edge u -> v (u affects v): whether u and v cannot share a
    /// step, as u runs before v.
    bool forbids(int u, int v) const { return position_[u] < position_[v]; }

    /// The action's place in the step order, 0 for the first to run.
    int position(int action) const { return position_[action]; }

private:
    std::vector<int> position_;  ///< action id -> place in the step order
};

/**
 * @brief Exists semantics
 *  Set of actions in a timestep can execute if there exists
 *  at least one order where actions do not interfere with actions further
 *  down the line.
 *
 * Jussi Rintanen, Keijo Heljanko, Ilkka Niemelä,
 * Planning as satisfiability: parallel plans and algorithms for plan search,
 * Artificial Intelligence, Volume 170, Issues 12–13, 2006,
 *
 * encode_parallelism writes the pairs the ExistsStepOrder forbids in one of
 * two ways (EstepMutexes):
 *
 * Pairs: one clause for each forbidden pair.
 *
 * Chains: for every unit (InterferenceOracle::interference_units), a chain
 * that forbids each of its writers with every affected action placed later,
 * through new chain variables (the paper's Section 3.2.2, one direction
 * only; add_chain has a worked example). With M writers and R affected
 * actions, a chain takes at most R + 2M binary clauses and M - 1 new
 * variables per step, where the pairs take up to M * R clauses. Both modes
 * forbid the same pairs, so they allow the same steps. Madagascar (Rintanen,
 * translate2sat.c, ESTEPchain) writes the same chain, with two differences:
 * it writes a unit as pairs when M * R <= 3 (M + R), where this encoding
 * always writes the chain, and it leaves out the pairs that cannot share a
 * step because their preconditions or effects contradict, which this
 * encoding keeps.
 */
class ExistsSemantics : public ParallelismStrategy {
public:
    /// The interference analyzer is mandatory for exists semantics —
    /// injected at construction so a half-wired strategy cannot exist.
    /// `graph` is the graph-bearing view of the same analyzer: non-null for
    /// eager analyzers, null for lazy ones (where encode_parallelism is never
    /// called because a propagator module manages parallelism instead).
    explicit ExistsSemantics(std::unique_ptr<InterferenceOracle> analyzer,
                             const InterferenceGraphAnalysis* graph = nullptr,
                             EstepMutexes mutexes = EstepMutexes::Pairs)
        : interference_analyzer_(std::move(analyzer)), graph_(graph), mutexes_(mutexes) {}

    z3::expr encode_parallelism(int timestep) override;

    std::string get_name() const override { return "ExistsSemantics"; }

    const InterferenceOracle* get_interference_analyzer() const override;
    const InterferenceGraphAnalysis* get_interference_graph_analysis() const override { return graph_; }

    bool allows_concurrent_actions() const override { return true; }

    /// The step order, built on first use from the interference graph, so
    /// it needs an eager (graph-bearing) analyzer.
    const ExistsStepOrder& step_order() const;

    /// PlainChains: chains over the plain Boolean units only; the ExistsCycle
    /// module checks every edge, so the planner still posts these chains.
    bool plain_chains() const { return mutexes_ == EstepMutexes::PlainChains; }

private:
    std::unique_ptr<InterferenceOracle> interference_analyzer_;
    const InterferenceGraphAnalysis* graph_ = nullptr;
    EstepMutexes mutexes_;

    mutable std::unique_ptr<ExistsStepOrder> step_order_;

    bool prepared_ = false;
    std::vector<std::pair<int, int>> forbidden_;  ///< Pairs: the pairs u -> v the order forbids

    /// One side of a clause of a chain: an action's variable or one of the
    /// chain's variables at the step, maybe negated. In unit 3 at step 2,
    /// {false, 4, true} is "not action 4" and {true, 1, false} is the chain
    /// variable estep.chain.3.1_2.
    struct Literal {
        bool chain;  ///< a chain variable, else an action
        int index;   ///< the action's id, or the chain variable's number in its unit
        bool negated;
    };

    /// One unit's chain, the same at every step.
    struct Chain {
        std::vector<std::pair<Literal, Literal>> clauses;  ///< each the disjunction of two literals
        int variables = 0;  ///< chain variables per step
    };

    /// Chains: one for each unit, in the order of interference_units(), so
    /// that a lazy encoding can add one unit's chain at one step.
    std::vector<Chain> chains_;

    /// What an action is in a unit; an action can be both.
    enum class Role { Writer, Affected };
    /// Chains: action id -> the units it is in, with its role in each, for a
    /// lazy encoding to find the chains between two actions.
    std::vector<std::vector<std::pair<int, Role>>> unit_roles_;

    /// Once, before the first step: the clauses of every step.
    void prepare();
    /// For Pairs: one clause for each forbidden pair, in the order of the
    /// graph.
    void prepare_pairs();
    /// For Chains: the chain of every unit.
    void prepare_chains();
    /// For PlainChains: the order of the graph of the plain units, and the
    /// chain of every plain unit none of whose pairs that order breaks.
    void prepare_plain_chains();
    /// The chain that forbids each of `writers` with every later action of
    /// `affected`, both in step order.
    void add_chain(const std::vector<int>& writers, const std::vector<int>& affected);
    /// The clauses of the chain of `unit` at `timestep`, onto `clauses`.
    void encode_chain(size_t unit, int timestep, z3::expr_vector& clauses) const;
};

} // namespace rantanplan
