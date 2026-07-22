#pragma once

#include "parallelism_strategy.hpp"
#include "../../analysis/interference_analysis.hpp"
#include <memory>

namespace rantanplan {

/**
 * @brief Forall semantics: actions can execute in parallel if they satisfy forall constraints
 *
 * This strategy allows multiple actions to execute simultaneously in the same timestep
 * as long as none of the actions in a step affect any other one
 *
 * The mutex constraints themselves are enforced during search by the
 * LazyForallMutexModule propagator (every forall strategy carries one), so
 * this class's role is to name the semantics and own the interference oracle
 * the propagator and plan extraction query.
 *
 * Jussi Rintanen, Keijo Heljanko, Ilkka Niemelä,
 * Planning as satisfiability: parallel plans and algorithms for plan search,
 * Artificial Intelligence, Volume 170, Issues 12–13, 2006,
 */
class ForallSemantics : public ParallelismStrategy {
public:
    /// The interference analyzer is mandatory for forall semantics —
    /// injected at construction so a half-wired strategy cannot exist.
    explicit ForallSemantics(std::unique_ptr<InterferenceOracle> analyzer)
        : interference_analyzer_(std::move(analyzer)) {}

    /// Never called: the propagator module manages parallelism (throws).
    z3::expr encode_parallelism(int timestep) override;

    std::string get_name() const override { return "ForallSemantics"; }

    const InterferenceOracle* get_interference_analyzer() const override;

    bool allows_concurrent_actions() const override { return true; }

private:
    std::unique_ptr<InterferenceOracle> interference_analyzer_;
};

} // namespace rantanplan
