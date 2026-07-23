#include "sequential_semantics.hpp"
#include "../../problem/action.hpp"
#include "../../util/stats.hpp"

namespace rantanplan {

Term SequentialSemantics::encode_parallelism(int timestep) {
    // Create a vector of all action variables at timestep
    TermVec action_vars;
    for (const Action& action : problem_->actions()) {
        Term action_var = variable_factory_->get_action_variable(action, timestep);
        action_vars.push_back(action_var);
    }

    if (action_vars.empty()) {
        // If no actions exist, return true (vacuously satisfied)
        return variable_factory_->builder().bool_val(true);
    }

    // AT MOST 1 action is true, via TermBuilder::mk_atmost (binomial/pairwise
    // construction; cvc5 has no pseudo-Boolean facility).
    // Changed from exactly-one to at-most-one so that empty steps are legal.
    // Empty steps are required for the prefix-monotone front-loading encoding: when a plan
    // of length k < h exists, the solver uses empty steps k..h-1 so frame axioms propagate
    // the goal state to horizon h, allowing a single goal literal per scheduled batch.
    // With a unit schedule this is behaviourally equivalent to the old exactly-one.
    //
    // This is the largest group in the encoding -- every action at the timestep.
    Term exactly_one = variable_factory_->builder().mk_atmost(action_vars, 1);

    auto& stats = Stats::instance();
    stats.add("encoder.parallelism_sequential_constraints", 1);

    return exactly_one;
}

} // namespace rantanplan
