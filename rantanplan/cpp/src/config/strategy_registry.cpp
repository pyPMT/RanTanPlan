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
        // Semantic interference is registered only with the Chained encoder:
        // StrategyFactory::validate accepts it only with an encoder that
        // composes the writes of one step to one fluent, which the Grounded
        // encoder does not (the old els, fls, egs, fgs and their -dt/-fp
        // variants).

        // Sequential
        {"seq",       {{E::Grounded, S::Sequential, I::None,             {},                       K::Sequential},
                       "Sequential encoding (baseline)"}},

        // Forall variants
        {"f",         {{E::Grounded, S::Forall,     I::EagerSyntactic,  {M::ForallEager},         K::Sequential},
                       "Forall-step, eager syntactic interference"}},
        {"f-p",       {{E::Grounded, S::Forall,     I::EagerSyntactic,  {M::ForallEager},         K::Sequential},
                       "Forall-step with forall propagator"}},
        {"fl",        {{E::Grounded, S::Forall,     I::LazySyntactic,   {M::ForallLazy},          K::Sequential},
                       "Forall-step, lazy syntactic interference"}},
        {"flsc",      {{E::Chained,  S::Forall,     I::LazySemantic,    {M::ForallLazy},          K::Sequential},
                       "Forall-step, lazy semantic interference, chained encoder"}},

        // Exists variants
        {"e",         {{E::Grounded, S::Exists,     I::EagerSyntactic,  {M::ExistsCycle},         K::Sequential},
                       "Exists-step, eager syntactic interference"}},
        {"el",        {{E::Grounded, S::Exists,     I::LazySyntactic,   {M::ExistsCycle},         K::Sequential},
                       "Exists-step, lazy syntactic interference"}},
        {"elsc",      {{E::Chained,  S::Exists,     I::LazySemantic,    {M::ExistsCycle},         K::Sequential},
                       "Exists-step, lazy semantic interference, chained encoder"}},

        // Special strategies
        {"r2e",       {{E::R2E,      S::Sequential, I::None,             {},                       K::Sequential},
                       "R2E encoding (relaxed-to-exact)"}},

        // Experimental eager-semantic
        {"fgsc",      {{E::Chained,  S::Forall,     I::EagerSemantic,   {M::ForallEager},         K::Sequential},
                       "Forall-step, eager semantic interference, chained encoder"}},
        {"egsc",      {{E::Chained,  S::Exists,     I::EagerSemantic,   {M::ExistsCycle},         K::Sequential},
                       "Exists-step, eager semantic interference, chained encoder"}},

        // Double-tail sequential
        {"seq-dt",    {{E::Grounded, S::Sequential, I::None,             {},                       K::DoubleTail},
                       "Sequential encoding, double-tail planner"}},

        // Double-tail forall
        {"f-dt",      {{E::Grounded, S::Forall,     I::EagerSyntactic,  {M::ForallEager},         K::DoubleTail},
                       "Forall-step, eager syntactic interference, double-tail"}},
        {"fl-dt",     {{E::Grounded, S::Forall,     I::LazySyntactic,   {M::ForallLazy},          K::DoubleTail},
                       "Forall-step, lazy syntactic interference, double-tail"}},
        {"flsc-dt",   {{E::Chained,  S::Forall,     I::LazySemantic,    {M::ForallLazy},          K::DoubleTail},
                       "Forall-step, lazy semantic interference, chained encoder, double-tail"}},

        // Double-tail exists
        {"e-dt",      {{E::Grounded, S::Exists,     I::EagerSyntactic,  {M::ExistsCycle},         K::DoubleTail},
                       "Exists-step, eager syntactic interference, double-tail"}},
        {"el-dt",     {{E::Grounded, S::Exists,     I::LazySyntactic,   {M::ExistsCycle},         K::DoubleTail},
                       "Exists-step, lazy syntactic interference, double-tail"}},
        {"elsc-dt",   {{E::Chained,  S::Exists,     I::LazySemantic,    {M::ExistsCycle},         K::DoubleTail},
                       "Exists-step, lazy semantic interference, chained encoder, double-tail"}},

        // Frame propagator variants (each mirrors its non-fp counterpart, with
        // eager frame axioms replaced by the lazy frame propagator module).
        {"e-fp",      {{E::Grounded, S::Exists,     I::EagerSyntactic,  {M::ExistsCycle, M::FrameAxiom}, K::Sequential},
                       "Exists-step, eager syntactic interference, frame propagator"}},
        {"elsc-fp",   {{E::Chained,  S::Exists,     I::LazySemantic,    {M::ExistsCycle, M::FrameAxiom}, K::Sequential},
                       "Exists-step, lazy semantic, chained encoder, frame propagator"}},
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
