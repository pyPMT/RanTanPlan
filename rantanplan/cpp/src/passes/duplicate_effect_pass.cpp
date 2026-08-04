#include "duplicate_effect_pass.hpp"
#include "../util/logger.hpp"
#include "../util/stats.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace rantanplan {

void DuplicateEffectPass::apply(PreprocessedProblem& result) const {
    const std::vector<Action>& actions = result.problem.actions();

    std::vector<size_t> offending;
    std::vector<std::string> report;

    for (size_t i = 0; i < actions.size(); ++i) {
        // Count writes per fluent, and how many of them are unconditional.
        std::unordered_map<ExprID, int> writes;
        std::unordered_map<ExprID, int> unconditional;
        for (const Effect& effect : actions[i].effects()) {
            ExprID fid = effect.effect_expression().fluent_id();
            writes[fid]++;
            if (!effect.is_conditional()) unconditional[fid]++;
        }

        // ONLY fluents written more than once with EVERY write unconditional.
        //
        // That distinction is empirical, and it is what separates the degenerate
        // shapes from the legitimate idioms.  Measured over pddl/test:
        //
        //   ALL-UNCONDITIONAL (degenerate — no sensible reading, and no encoder
        //   handles them):
        //     drink(x,x)      two additive updates that cancel  (reflexive
        //                     grounding of drink(?from,?to))
        //     feast(v,x,x)    (not (craves v x)) AND (craves v x) — a boolean
        //                     add and delete of one atom, undefined in PDDL
        //
        //   INVOLVING A CONDITIONAL (legitimate, and dropping them lost plans on
        //   cave-diving and petrobras — they must survive):
        //     hire-diver      an unconditional (not (available ?d1)) plus the
        //                     ?d2 = ?d1 case of a forall-when that also deletes
        //                     it.  Both writes AGREE; the action is well defined
        //                     and merely redundantly represented.
        //     refuel_at_port  two conditional effects on current_fuel with
        //                     MUTUALLY EXCLUSIVE guards (<= capacity -> increase,
        //                     > capacity -> clip).  The standard clip-to-capacity
        //                     idiom; exactly one arm ever applies.
        int duplicated = 0;
        for (const auto& [fid, n] : writes) {
            if (n > 1 && unconditional[fid] == n) duplicated++;
        }
        if (duplicated == 0) continue;

        offending.push_back(i);
        report.push_back(actions[i].name() + " (" + std::to_string(duplicated) +
                         " fluent(s) written more than once, unconditionally)");
    }

    Stats::instance().set("duplicate_effect.actions_dropped",
                          static_cast<double>(offending.size()));
    if (offending.empty()) return;

    // Loud on purpose: this is a lossy interim measure, not a normalisation.
    // See the header for what it costs and what it should become.
    // Logger has no warning level; info() is visible at the default verbosity and
    // silent under the test suite, which is the behaviour we want.
    Logger::instance().info(
        "WARNING: dropping " + std::to_string(offending.size()) + " of " +
        std::to_string(actions.size()) + " ground actions that write the same "
        "fluent more than once with every write unconditional. No encoder "
        "composes such effects correctly: the "
        "Grounded encoders make the action unsatisfiable, the Chained encoders "
        "apply only its first effect, and interference analysis reports it as "
        "conflicting with every action in the problem. Dropping is sound but may "
        "lose plans if the duplicate writes do not cancel.");
    for (const std::string& line : report)
        Logger::instance().info("  dropped: " + line);

    result.problem = result.problem.without_actions(offending);
}

} // namespace rantanplan
