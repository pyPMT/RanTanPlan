#include "formula_exporter.hpp"
#include "../util/stats.hpp"
#include "../util/logger.hpp"
#include <cstdint>
#include <iostream>
#include <sstream>
#include <unordered_set>

namespace rantanplan {

namespace {

/// Collect the free constants of a term, in first-seen order.
///
/// PORT-QUESTION: cvc5 has no `Solver::to_smt2()`, so the declarations that
/// make the exported formula self-contained (the documented contract of this
/// class) have to be recovered from the assertions themselves. This walks for
/// Kind::CONSTANT leaves. Two known gaps:
///   - symbols are printed raw; if a rendered fluent name is not a valid
///     SMT-LIB simple symbol, the declaration and the body may disagree on
///     quoting (Z3's to_smt2 handled the |...| quoting for us);
///   - the fork's PLAN_DOES / PLAN_HOLDS theory atoms are not constants and
///     have no standard SMT-LIB spelling at all, so a formula built with the
///     plan theory cannot be exported as portable SMT-LIB by any route.
void collect_constants(const Term& term,
                       std::unordered_set<uint64_t>& seen,
                       TermVec& out) {
    if (!seen.insert(term.getId()).second) return;
    if (term.getKind() == cvc5::Kind::CONSTANT) {
        out.push_back(term);
        return;
    }
    for (const Term& child : term) {
        collect_constants(child, seen, out);
    }
}

} // namespace

FormulaExporter::FormulaExporter(const Problem& problem, BaseEncoder& encoder, cvc5::TermManager& tm)
    : problem_(problem), encoder_(encoder), tm_(tm) {
}

std::string FormulaExporter::export_formula(int max_timestep) {
    cvc5::Solver formula_solver(tm_);
    auto& stats = Stats::instance();

    if (max_timestep < 0) {
        Logger::instance().error("max_timestep must be non-negative");
        return "";
    }

    // Add initial state constraints (timestep 0)
    formula_solver.assertFormula(encoder_.encode_initial_state());

    // Add transition constraints for each timestep
    for (int t = 0; t <= max_timestep; t++) {
        // Add action constraints
        formula_solver.assertFormula(encoder_.encode_actions(t));

        // Add frame axioms (for transitions t -> t+1, so skip the last timestep)
        if (t < max_timestep) {
            formula_solver.assertFormula(encoder_.encode_frames(t));
        }

        // Add parallelism constraints
        formula_solver.assertFormula(encoder_.encode_parallelism(t));

        // Add symmetry constraints
        formula_solver.assertFormula(encoder_.encode_symmetries(t));
    }

    // Add goal constraints at the final timestep
    formula_solver.assertFormula(encoder_.encode_goal(max_timestep));

    // Export the complete formula as SMT-LIB2. cvc5 has no to_smt2(), so the
    // script is assembled from the asserted formulas plus the declarations of
    // the constants they mention (see collect_constants above).
    std::vector<Term> assertions = formula_solver.getAssertions();

    std::unordered_set<uint64_t> seen;
    TermVec constants;
    for (const Term& assertion : assertions) {
        collect_constants(assertion, seen, constants);
    }

    std::ostringstream out;
    out << "(set-logic ALL)\n";
    for (const Term& c : constants) {
        out << "(declare-fun " << (c.hasSymbol() ? c.getSymbol() : c.toString())
            << " () " << c.getSort() << ")\n";
    }
    for (const Term& assertion : assertions) {
        out << "(assert " << assertion << ")\n";
    }
    out << "(check-sat)\n";
    std::string smt2_formula = out.str();

    // Collect some basic statistics
    stats.set("formula_exporter.max_timestep", max_timestep);
    stats.set("formula_exporter.num_assertions", assertions.size());

    return smt2_formula;
}

} // namespace rantanplan