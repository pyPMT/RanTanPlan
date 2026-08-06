#pragma once

#include "../../problem/problem.hpp"
#include "../../smtcore/smt_types.hpp"
#include "../../smtcore/smt_variable_factory.hpp"
#include "../../analysis/graph.hpp"
#include <memory>
#include <string>

namespace rantanplan {

// Forward declarations
class InterferenceOracle;
class InterferenceGraphAnalysis;

/**
 * @brief Abstract base class for different parallelism encoding strategies
 *
 * This class defines the interface for encoding parallelism constraints in planning problems.
 * Different concrete implementations provide various semantics for action execution.
 */
class ParallelismStrategy {
public:
    virtual ~ParallelismStrategy() = default;

    /**
     * @brief Encode parallelism constraints for a given timestep
     * @param timestep The timestep for which to encode parallelism
     * @return A term representing the parallelism constraints
     */
    virtual Term encode_parallelism(int timestep) = 0;

    /**
     * @brief Initialize the strategy with problem context
     * @param problem The planning problem instance
     * @param ctx cvc5 TermManager for creating terms
     * @param var_factory Factory for creating SMT variables
     */
    virtual void initialize(const Problem& problem, cvc5::TermManager& ctx, SmtVariableFactory& var_factory) {
        problem_ = &problem;
        ctx_ = &ctx;
        variable_factory_ = &var_factory;
    }

    /**
     * @brief Get the name of this parallelism strategy
     * @return String identifier for this strategy
     */
    virtual std::string get_name() const = 0;

    /**
     * @brief Get access to the interference analyzer (if available)
     * @return Pointer to the interference analyzer, or nullptr if not available
     */
    virtual const InterferenceOracle* get_interference_analyzer() const = 0;

    /**
     * @brief Get the graph-bearing view of the interference analyzer
     * @return Pointer to the graph analysis if the injected analyzer is eager
     *         (graph-bearing), nullptr for lazy analyzers and sequential semantics
     */
    virtual const InterferenceGraphAnalysis* get_interference_graph_analysis() const { return nullptr; }

    /**
     * @brief Check if this strategy allows concurrent action execution
     * @return true if multiple actions can execute at the same timestep (forall/exists semantics),
     *         false if at most one action per timestep (sequential semantics)
     */
    virtual bool allows_concurrent_actions() const = 0;

protected:
    const Problem* problem_ = nullptr;
    cvc5::TermManager* ctx_ = nullptr;
    SmtVariableFactory* variable_factory_ = nullptr;
};

} // namespace rantanplan
