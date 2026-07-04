#include "formula_exporter.hpp"
#include "../util/stats.hpp"
#include "../util/logger.hpp"
#include <iostream>

namespace rantanplan {

FormulaExporter::FormulaExporter(const Problem& problem, BaseEncoder& encoder, z3::context& ctx)
    : problem_(problem), encoder_(encoder), ctx_(ctx) {
}

std::string FormulaExporter::export_formula(int max_timestep) {
    z3::solver formula_solver(ctx_);
    auto& stats = Stats::instance();

    if (max_timestep < 0) {
        Logger::instance().error("max_timestep must be non-negative");
        return "";
    }

    // Add initial state constraints (timestep 0)
    formula_solver.add(encoder_.encode_initial_state());

    // Add transition constraints for each timestep
    for (int t = 0; t <= max_timestep; t++) {
        // Add action constraints
        formula_solver.add(encoder_.encode_actions(t));

        // Add frame axioms (for transitions t -> t+1, so skip the last timestep)
        if (t < max_timestep) {
            formula_solver.add(encoder_.encode_frames(t));
        }

        // Add parallelism constraints
        formula_solver.add(encoder_.encode_parallelism(t));

        // Add symmetry constraints
        formula_solver.add(encoder_.encode_symmetries(t));
    }

    // Add goal constraints at the final timestep
    formula_solver.add(encoder_.encode_goal(max_timestep));

    // Export the complete formula as SMT-LIB2
    std::string smt2_formula = formula_solver.to_smt2();

    // Collect some basic statistics
    stats.set("formula_exporter.max_timestep", max_timestep);
    stats.set("formula_exporter.num_assertions", formula_solver.assertions().size());

    return smt2_formula;
}

} // namespace rantanplan