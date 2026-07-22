#include "eager_interference_analysis.hpp"
#include "../util/memory_tracker.hpp"
#include "../util/scoped_timer.hpp"
#include "../util/logger.hpp"
#include "../util/stats.hpp"

namespace rantanplan {

EagerInterferenceAnalysis::EagerInterferenceAnalysis(const Problem& problem)
    : problem_(problem) {
    // The syntactic analysis is construction-scoped: once the graph is
    // built, all queries are answered from the graph alone.
    SyntacticInterference syntactic(problem);

    for (size_t i = 0; i < problem.actions().size(); ++i) {
        interference_graph_.add_node();
    }

    double current_memory = MemoryTracker::instance().get_current_memory_mb();
    Logger::instance().verbose("EagerInterferenceAnalysis initialized with " + std::to_string(problem.actions().size()) +
                              " actions (memory: " + std::to_string(static_cast<int>(current_memory)) + "MB)");

    build_interference_graph(syntactic);
}

void EagerInterferenceAnalysis::build_interference_graph(const SyntacticInterference& syntactic) {
    ScopedTimer timer("interference.eager.build_time_ms");
    double start_memory = MemoryTracker::instance().get_current_memory_mb();

    // Expensive O(n²) preprocessing: analyze all pairs of actions for conflicts.
    // Results are cached in the interference graph for fast lookup during execution.
    const auto& actions = problem_.actions();
    for (size_t i = 0; i < actions.size(); ++i) {
        for (size_t j = 0; j < actions.size(); ++j) {
            if (i != j) {  // Don't check action against itself
                const Action& action1 = actions[i];
                const Action& action2 = actions[j];

                if (syntactic.interferes(action1, action2)) {
                    // Cache directional interference: action1 interferes with action2
                    interference_graph_.add_edge(action1.id(), action2.id());
                }
            }
        }
    }

    // Record stats
    double memory_used = MemoryTracker::instance().get_current_memory_mb() - start_memory;
    Stats::instance().set("interference.eager.nodes", interference_graph_.num_nodes());
    Stats::instance().set("interference.eager.edges", interference_graph_.num_edges());
    Stats::instance().set("interference.eager.memory_mb", memory_used);

    // Structured visual output
    Logger::instance().component(VerbosityLevel::INFO, "Interference.E", {
        {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
        {"nodes", std::to_string(interference_graph_.num_nodes())},
        {"edges", std::to_string(interference_graph_.num_edges())},
        {"mem", std::to_string(static_cast<int>(memory_used)) + "MB"}
    });
}

bool EagerInterferenceAnalysis::has_interference(const Action& a1, const Action& a2) const {
    // DIRECTIONAL CHECK: Does a1 interfere with a2? (a1 -> a2)
    // This is NOT symmetric: has_interference(A,B) != has_interference(B,A) in general
    return interference_graph_.has_edge(a1.id(), a2.id());
}

bool EagerInterferenceAnalysis::has_interference(int node_id1, int node_id2) const {
    // DIRECTIONAL CHECK: Does node_id1 interfere with node_id2? (node_id1 -> node_id2)
    // This is NOT symmetric: has_interference(A,B) != has_interference(B,A) in general
    return interference_graph_.has_edge(node_id1, node_id2);
}

std::vector<const Action*> EagerInterferenceAnalysis::topological_sort_actions(
    const std::vector<const Action*>& actions) const {
    if (actions.size() <= 1) {
        return actions;
    }

    // Convert actions to node IDs
    std::vector<int> node_ids;
    for (const Action* action : actions) {
        if (action->id() >= 0) {
            node_ids.push_back(action->id());
        }
    }
    if (node_ids.empty()) {
        return actions;
    }

    // Use the graph's topological sort and map node IDs back to actions
    std::vector<int> sorted_node_ids = interference_graph_.topological_sort(node_ids);
    std::vector<const Action*> result;
    result.reserve(sorted_node_ids.size());
    for (int node_id : sorted_node_ids) {
        if (node_id >= 0 && static_cast<size_t>(node_id) < problem_.action_count()) {
            result.push_back(&problem_.action(node_id));
        }
    }
    return result;
}

} // namespace rantanplan
