#pragma once

#include "parallelism_strategy.hpp"
#include "../../analysis/interference_analysis.hpp"
#include <memory>

namespace rantanplan {

/**
 * @brief Exists semantics
 *  Set of actions in a timestep can execute if there exists
 *  at least one order where actions do not interfere with actions further
 *  down the line.
 * 
 * Jussi Rintanen, Keijo Heljanko, Ilkka Niemelä,
 * Planning as satisfiability: parallel plans and algorithms for plan search,
 * Artificial Intelligence, Volume 170, Issues 12–13, 2006,
 */
class ExistsSemantics : public ParallelismStrategy {
public:
    /// The interference analyzer is mandatory for exists semantics —
    /// injected at construction so a half-wired strategy cannot exist.
    explicit ExistsSemantics(std::unique_ptr<InterferenceAnalysis> analyzer)
        : interference_analyzer_(std::move(analyzer)) {}
    
    std::shared_ptr<z3::expr> encode_parallelism(int timestep) override;

    std::string get_name() const override { return "ExistsSemantics"; }

    const InterferenceAnalysis* get_interference_analyzer() const override;

    bool allows_concurrent_actions() const override { return true; }

private:
    std::unique_ptr<InterferenceAnalysis> interference_analyzer_;
};

} // namespace rantanplan
