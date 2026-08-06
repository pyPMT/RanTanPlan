#include "strategy_registry.hpp"
#include <stdexcept>
#include <sstream>

namespace rantanplan {

using E = EncoderFamily;
using S = SemanticsKind;
using I = InterferenceKind;
using M = PropagatorModuleKind;
using K = PlannerKind;

const std::map<std::string, StrategyEntry>& StrategyRegistry::get_registry() {
    static const std::map<std::string, StrategyEntry> registry = {
        // ── M4 (cvc5 swap): reduced registry ────────────────────────────────
        // PropagatorStrategy and its four modules are temporarily out of the
        // build while they are reimplemented as THEORY_PLAN reasoning, so only
        // strategies that need NO propagator are registered. test.py discovers
        // strategies from here, so the quick suite shrinks and stays green
        // rather than going red. Restoration schedule: migration-plan.md §7.8
        //   M5 (ExistsCycle): + e, el, els, elsc, egsc, and the -dt variants
        //   M6 (ForallMutex): + f-p, fl, fls, flsc, fgsc, and the -dt variants
        //   M7 (FrameAxiom):  + elsc-fp, cgb -- full registry restored
        //
        // Note f/e/fgs/egs below carry an EMPTY propagator list rather than
        // their usual module. That is the per-strategy form of the M4 bridge
        // experiment: with no module, manages_parallelism_constraints() is
        // false and BasePlanner emits the eager clause encoding instead
        // (base_planner.cpp:178). It is legal only because all four use EAGER
        // interference -- ForallSemantics/ExistsSemantics::encode_parallelism
        // need the interference graph, which lazy analyzers do not have, which
        // is exactly why els/elsc cannot be registered here.

        {"seq",       {{E::Grounded, S::Sequential, I::None,             {}, K::Sequential},
                       "Sequential encoding (baseline)"}},
        {"r2e",       {{E::R2E,      S::Sequential, I::None,             {}, K::Sequential},
                       "R2E encoding (relaxed-to-exact)"}},

        {"f",         {{E::Grounded, S::Forall,     I::EagerSyntactic,   {}, K::Sequential},
                       "Forall-step, eager syntactic interference"}},
        {"e",         {{E::Grounded, S::Exists,     I::EagerSyntactic,   {}, K::Sequential},
                       "Exists-step, eager syntactic interference"}},
        {"fgs",       {{E::Grounded, S::Forall,     I::EagerSemantic,    {}, K::Sequential},
                       "Forall-step, eager semantic interference"}},
        {"egs",       {{E::Grounded, S::Exists,     I::EagerSemantic,    {}, K::Sequential},
                       "Exists-step, eager semantic interference"}},
    };
    return registry;
}

const StrategySpec& StrategyRegistry::get(const std::string& name) {
    auto& registry = get_registry();
    auto it = registry.find(name);

    if (it == registry.end()) {
        std::ostringstream error;
        error << "Unknown strategy: '" << name << "'\n";
        error << "Available strategies: ";

        auto strategies = list_strategies();
        for (size_t i = 0; i < strategies.size(); ++i) {
            error << strategies[i];
            if (i < strategies.size() - 1) error << ", ";
        }
        error << "\n";

        throw std::invalid_argument(error.str());
    }

    return it->second.spec;
}

bool StrategyRegistry::exists(const std::string& name) {
    auto& registry = get_registry();
    return registry.find(name) != registry.end();
}

std::vector<std::string> StrategyRegistry::list_strategies() {
    auto& registry = get_registry();
    std::vector<std::string> names;
    names.reserve(registry.size());

    for (const auto& pair : registry) {
        names.push_back(pair.first);
    }

    return names;
}

} // namespace rantanplan
