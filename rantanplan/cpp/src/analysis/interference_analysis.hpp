#pragma once

#include "../problem/problem.hpp"
#include "../problem/action.hpp"
#include "../problem/visitors/fluent_polarity_collector.hpp"
#include "graph.hpp"
#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>

/**
 * @file interference_analysis.hpp
 * @brief The two interference interfaces: InterferenceOracle (universal) and
 *        InterferenceGraphAnalysis (adds whole-graph access; eager only).
 */
namespace rantanplan {

/**
 * @brief Abstract base class for action interference analysis
 *
 * Provides the universal query surface for analyzing action interferences in
 * planning problems: pairwise interference checks and execution-order sorting.
 * Supported by both eager (pre-computed) and lazy (on-demand) implementations.
 *
 * Consumers that need whole-graph access (neighbour lists, SCCs) take an
 * InterferenceGraphAnalysis instead — only eager implementations provide one.
 */
class InterferenceOracle {
public:
    virtual ~InterferenceOracle() = default;
    
    /**
     * @brief Initialize the analyzer with a planning problem
     * @param problem The planning problem to analyze
     */
    virtual void initialize(const Problem& problem) = 0;
    
    // Direct interference checking methods (supported by both eager and lazy)
    
    /**
     * @brief Check if first action interferes with second action (DIRECTIONAL)
     * 
     * IMPORTANT: This checks if a1 interferes with a2, NOT if they interfere with each other.
     * Interference is directional: has_interference(A, B) != has_interference(B, A) in general.
     * 
     * Returns true if executing a1 would prevent a2 from executing correctly, or if
     * a1's effects would interfere with a2's preconditions or effects.
     * 
     * @param a1 The potentially interfering action  
     * @param a2 The potentially interfered-with action
     * @return True if a1 interferes with a2, false otherwise
     */
    virtual bool has_interference(const Action& a1, const Action& a2) const = 0;
    
    /**
     * @brief Check if first action interferes with second action by node IDs (DIRECTIONAL)
     * 
     * IMPORTANT: This checks if node_id1 interferes with node_id2, NOT if they interfere with each other.
     * Interference is directional: has_interference(A, B) != has_interference(B, A) in general.
     * 
     * This is an optimized version that works directly with node IDs, avoiding Action object lookups.
     * 
     * @param node_id1 The node ID of the potentially interfering action  
     * @param node_id2 The node ID of the potentially interfered-with action
     * @return True if node_id1 interferes with node_id2, false otherwise
     */
    virtual bool has_interference(int node_id1, int node_id2) const = 0;
    
    
    /**
     * @brief Sort actions for safe execution order (reverse topological order)
     * 
     * IMPORTANT: This returns actions in EXECUTION ORDER, not standard topological order.
     * If action A interferes with action B (A -> B), then B will appear BEFORE A in the result.
     * This ensures that when actions are executed in the returned order, no interference occurs.
     * 
     * Example: If we have edges A->B and B->C, the result will be [C, B, A]
     * 
     * This method only requires the subgraph between the provided actions,
     * making it compatible with lazy analysis approaches.
     * 
     * @param actions The actions to sort
     * @return Actions sorted in safe execution order (reverse topological order)
     */
    virtual std::vector<const Action*> topological_sort_actions(const std::vector<const Action*>& actions) const = 0;

    /**
     * @brief Check if two actions interfere syntactically (sound overapproximation)
     *
     * Uses fast set-intersection checks on precondition/effect fluents.
     * This is an overapproximation: may return true for action pairs that
     * don't interfere semantically, but never returns false for pairs that do.
     */
    bool has_syntactic_interference(const Action& a1, const Action& a2) const {
        return actions_interfere(a1, a2);
    }

protected:
    // Common data structures for action analysis results
    struct ActionAnalysis {
        // Precondition analysis
        std::unordered_set<ExprID> positive_boolean_preconditions;  // Boolean fluents required to be true
        std::unordered_set<ExprID> negative_boolean_preconditions;  // Boolean fluents required to be false
        std::unordered_set<ExprID> numeric_preconditions;           // Numeric fluents in preconditions

        // Effect analysis (a computed boolean write, p := q, is in both)
        std::unordered_set<ExprID> positive_boolean_effects;  // Boolean fluents made true
        std::unordered_set<ExprID> negative_boolean_effects;  // Boolean fluents made false
        std::unordered_set<ExprID> numeric_effects;           // Numeric fluents modified

        // A set of all fluents modified by this action's effects (the union of all the previous 3 sets)
        std::unordered_set<ExprID> all_effects;

        // Fluents appearing in the conditions of conditional effects
        std::unordered_set<ExprID> conditional_effect_fluents;

        // Fluents read by an effect's value, numeric or computed boolean.
        // For example, x' = x + y + y + z^2 gives x, y and z, and p' = q gives q.
        std::unordered_set<ExprID> effect_value_dependencies;
    };

    // Common data that all implementations need
    const Problem* problem_ = nullptr;

    // Analysis results for each action (shared functionality)
    std::unordered_map<Action, ActionAnalysis> action_analysis_;
    
    /**
     * @brief Analyze all actions and populate the action_analysis_ map
     * 
     * This is common functionality that both implementations need.
     */
    void analyze_all_actions();
    
    /**
     * @brief Analyze a single action and populate its ActionAnalysis
     * @param action The action to analyze
     * @return ActionAnalysis structure with collected information
     */
    ActionAnalysis analyze_action(const Action& action) const;
    
    /**
     * @brief Analyze the preconditions of an action
     * @param action The action to analyze
     * @param analysis The analysis structure to populate
     */
    void analyze_preconditions(const Action& action, ActionAnalysis& analysis) const;
    
    /**
     * @brief Analyze the effects of an action
     * @param action The action to analyze
     * @param analysis The analysis structure to populate
     */
    void analyze_effects(const Action& action, ActionAnalysis& analysis) const;
    
    /**
     * @brief Check if two actions interfere with each other (common interference logic)
     * @param a1 First action
     * @param a2 Second action
     * @return True if actions interfere, false otherwise
     */
    bool actions_interfere(const Action& a1, const Action& a2) const;
};

/**
 * @brief Interference analysis with whole-graph access
 *
 * Extends the oracle with the pre-computed interference graph. Only eager
 * implementations derive from this; consumers that need neighbour lists or
 * SCC information (forall/exists parallelism encoding, the eager forall
 * mutex propagator module) take this type in their signatures, so passing
 * a lazy analyzer where a graph is required is a compile error.
 */
class InterferenceGraphAnalysis : public InterferenceOracle {
public:
    /**
     * @brief Get the complete interference graph
     * @return Reference to the interference graph
     */
    virtual const Graph& get_interference_graph() const = 0;

    /**
     * @brief Get all neighbors (interfering actions) for a given action node
     * @param node_id The node ID to get neighbors for
     * @return Vector of neighbor node IDs
     */
    virtual const std::vector<int>& get_neighbours(int node_id) const = 0;

    /**
     * @brief Output the interference graph to a DOT format file
     * @param filename The name of the output file (default: "interference.dot")
     */
    virtual void output_interference_graph_dot(const std::string& filename = "interference.dot") const = 0;
};

} // namespace rantanplan