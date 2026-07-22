#include "strategy_registry.hpp"
#include <stdexcept>
#include <sstream>

namespace rantanplan {

using S = SemanticsKind;
using I = InterferenceKind;
using M = PropagatorModuleKind;
using K = PlannerKind;

const std::map<std::string, StrategyEntry>& StrategyRegistry::get_registry() {
    static const std::map<std::string, StrategyEntry> registry = {
        // Sequential
        {"seq",       {{S::Sequential, I::None,            {},                              K::Sequential},
                       "Sequential encoding (baseline)"}},

        // Forall (also the spec the SDAC exists->forall downgrade produces)
        {"fls",       {{S::Forall,     I::LazySemantic,    {M::ForallLazy},                 K::Sequential},
                       "Forall-step, lazy semantic interference"}},

        // Exists variants
        {"e",         {{S::Exists,     I::EagerSyntactic,  {},                              K::Sequential},
                       "Exists-step, eager syntactic interference, encoded (a-priori) mutexes"}},
        {"els",       {{S::Exists,     I::LazySemantic,    {M::ExistsCycle},                K::Sequential},
                       "Exists-step, lazy semantic interference"}},
        {"els-fp",    {{S::Exists,     I::LazySemantic,    {M::ExistsCycle, M::FrameAxiom}, K::Sequential},
                       "Exists-step, lazy semantic interference, frame propagator"}},
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
