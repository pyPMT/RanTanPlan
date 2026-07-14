#include "eager_semantic_interference_analysis.hpp"
#include "../util/memory_tracker.hpp"
#include "../util/scoped_timer.hpp"
#include "../util/logger.hpp"
#include "../util/stats.hpp"
#include <fstream>

namespace rantanplan {

EagerSemanticInterferenceAnalysis::EagerSemanticInterferenceAnalysis(const Problem& problem) {
    initialize(problem);
}

void EagerSemanticInterferenceAnalysis::initialize(const Problem& problem) {
    problem_ = &problem;
    action_analysis_.clear();
    interference_graph_ = Graph();
    analyze_all_actions();  // base-class per-action metadata

    // One graph node per action (indexed by action id).
    for (size_t i = 0; i < problem.actions().size(); ++i) {
        interference_graph_.add_node();
    }

    // Build the graph via the shared predicate, then let it (and its Z3 state) go.
    ScopedTimer timer("interference.semantic.build_time_ms");
    double start_memory = MemoryTracker::instance().get_current_memory_mb();
    {
        SemanticInterferencePredicate predicate(problem);
        build_interference_graph(predicate);
    }
    double memory_used = MemoryTracker::instance().get_current_memory_mb() - start_memory;

    Stats::instance().set("interference.semantic.nodes", interference_graph_.num_nodes());
    Stats::instance().set("interference.semantic.edges", interference_graph_.num_edges());
    Stats::instance().set("interference.semantic.memory_mb", memory_used);
    Logger::instance().component(VerbosityLevel::INFO, "Interference.S", {
        {"time", std::to_string(static_cast<int>(timer.elapsed_ms())) + "ms"},
        {"nodes", std::to_string(interference_graph_.num_nodes())},
        {"edges", std::to_string(interference_graph_.num_edges())},
        {"mem", std::to_string(static_cast<int>(memory_used)) + "MB"}
    });
}

void EagerSemanticInterferenceAnalysis::build_interference_graph(
    const SemanticInterferencePredicate& predicate) {
    // O(n²): a directed edge i→j iff action i semantically affects action j.
    const auto& actions = problem_->actions();
    for (size_t i = 0; i < actions.size(); ++i) {
        for (size_t j = 0; j < actions.size(); ++j) {
            if (i == j) continue;
            if (predicate.affects(actions[i], actions[j])) {
                interference_graph_.add_edge(actions[i].id(), actions[j].id());
            }
        }
    }
}

bool EagerSemanticInterferenceAnalysis::has_interference(const Action& a1, const Action& a2) const {
    // Directional O(1) lookup in the pre-computed graph. NOT symmetric.
    return interference_graph_.has_edge(a1.id(), a2.id());
}

bool EagerSemanticInterferenceAnalysis::has_interference(int node_id1, int node_id2) const {
    return interference_graph_.has_edge(node_id1, node_id2);
}

std::vector<const Action*> EagerSemanticInterferenceAnalysis::topological_sort_actions(
    const std::vector<const Action*>& actions) const {
    if (actions.size() <= 1) {
        return actions;
    }

    std::vector<int> node_ids;
    for (const Action* action : actions) {
        if (action->id() >= 0) node_ids.push_back(action->id());
    }
    if (node_ids.empty()) {
        return actions;
    }

    std::vector<int> sorted_node_ids = interference_graph_.topological_sort(node_ids);
    std::vector<const Action*> result;
    result.reserve(sorted_node_ids.size());
    for (int node_id : sorted_node_ids) {
        if (node_id >= 0 && static_cast<size_t>(node_id) < problem_->action_count()) {
            result.push_back(&problem_->action(node_id));
        }
    }
    return result;
}

const Graph& EagerSemanticInterferenceAnalysis::get_interference_graph() const {
    return interference_graph_;
}

const std::vector<int>& EagerSemanticInterferenceAnalysis::get_neighbours(int node_id) const {
    return interference_graph_.get_neighbours(node_id);
}

void EagerSemanticInterferenceAnalysis::output_interference_graph_dot(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        Logger::instance().error("Could not open file " + filename + " for writing");
        return;
    }

    Logger::instance().info("Writing semantic interference graph to " + filename);

    file << "digraph SemanticInterferenceGraph {" << std::endl;
    file << "    rankdir=LR;" << std::endl;
    file << "    node [shape=box, style=rounded];" << std::endl;
    file << "    edge [color=blue, arrowhead=vee];" << std::endl;
    file << std::endl;

    for (size_t i = 0; i < problem_->action_count(); ++i) {
        file << "    " << i << " [label=\"" << problem_->action(i).name() << "\"];" << std::endl;
    }

    file << std::endl;

    for (int node_id = 0; node_id < static_cast<int>(problem_->action_count()); ++node_id) {
        for (int neighbor : interference_graph_.get_neighbours(node_id)) {
            file << "    " << node_id << " -> " << neighbor << ";" << std::endl;
        }
    }

    file << "}" << std::endl;
    file.close();

    Logger::instance().info("Semantic interference graph successfully written to " + filename);
}

} // namespace rantanplan
