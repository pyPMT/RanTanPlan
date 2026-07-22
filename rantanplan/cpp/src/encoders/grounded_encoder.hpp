#pragma once

#include "../problem/problem.hpp"
#include "../problem/plan.hpp"
#include "../problem/effect_expression.hpp"
#include "../preprocessing/local_invariants.hpp"
#include "../preprocessing/symmetry_info.hpp"
#include "../z3core/grounded_encoding_visitor.hpp"
#include "../z3core/z3_variable_factory.hpp"
#include "parallelism/parallelism_strategy.hpp"
#include <z3++.h>

#include <optional>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>

// This class handles the encoding of grounded fluents and actions. It is the
// single state-space encoder: planners, propagators, and the abstract suffix
// encoder all consume it directly.
namespace rantanplan {

class GroundedEncoder {
public:
    // Constructor — the parallelism strategy is mandatory and injected here.
    GroundedEncoder(const Problem& problem, z3::context& ctx,
                    std::unique_ptr<ParallelismStrategy> parallelism);

    // Pipeline data (set once before search, used by encode_symmetries)
    void set_symmetry_data(std::vector<SymmetryInfo> data) { symmetry_data_ = std::move(data); }

    /// Receive state constraints from the pipeline result (bounds/conservations/
    /// domains from ActionPruningPass, mutexes from MutexAnalysisPass).
    void set_state_constraints(StateConstraints sc) { state_constraints_ = std::move(sc); }

    // Encoding steps
    z3::expr encode_initial_state();
    z3::expr encode_actions(int t); // Encodes actions layer_idx
    z3::expr encode_frames(int t); // Encodes frame axioms for layer_idx to layer_idx+1
    z3::expr encode_goal(int t);    // Encodes goal conditions at layer_idx
    z3::expr encode_parallelism(int t); // Encodes parallelism semantics
    z3::expr encode_symmetries(int t);
    z3::expr encode_state_constraints(int t);

    /**
     * @brief Encode the prefix-monotone (front-loading) symmetry-breaking constraint.
     *
     * Adds:  (∀a. ¬a@t)  →  (∀a. ¬a@(t+1))
     *
     * This chains across all t during search, forcing all active action steps to
     * be contiguous at the front of the plan. Combined with at-most-one (pble)
     * parallelism semantics, this guarantees that if a valid plan of length k < h
     * exists, the solver can represent it with the actions at 0..k-1 and empty steps
     * at k..h-1. Frame axioms then propagate the goal state to timestep h, so a
     * single goal literal at h witnesses any plan length in the batch [prev_h+1..h].
     */
    z3::expr encode_prefix_monotone(int t); // Front-loading symmetry breaking

    // Strategy management
    std::string get_parallelism_strategy_name() const;

    // Access to variable factory for plan extraction
    Z3VariableFactory& get_variable_factory() { return variable_factory_; }
    const Z3VariableFactory& get_variable_factory() const { return variable_factory_; }

    // Get access to parallelism strategy for plan extraction
    const ParallelismStrategy* get_parallelism_strategy() const { return parallelism_strategy_.get(); }

    // Encode precondition + effect constraints for a single action at timestep t.
    // Returns nullopt if the action has no effects (nothing to encode).
    std::optional<z3::expr> encode_single_action(const Action& action, int t);

    // Per-action effect constraints (action → effects), without preconditions.
    // Together with encode_cumulative_effects, these compose encode_actions.
    std::optional<z3::expr> encode_non_cumulative_effects(const Action& action, int t);

    // Per-timestep joint effect constraints across multiple actions (no-op
    // for this encoder; kept as the seam where a chained encoder would emit
    // cumulative-effect chain variables).
    // Together with encode_non_cumulative_effects, these compose encode_actions.
    z3::expr encode_cumulative_effects(int t);

    Plan extract_plan(const z3::model& model, int max_timestep) const;

    // Helper functions to convert expressions/effects to Z3 using visitor
    // (used by the abstract suffix encoder and propagator modules)
    z3::expr convert_expr_id_to_z3(ExprID id, int timestep = -1);
    z3::expr convert_effect_to_z3(const EffectExpression& effect, int timestep);

    /// Convert an action's cost expression to Z3 at the given timestep,
    /// evaluated at the start-of-timestep state x_t.
    z3::expr convert_cost_to_z3(const Action& action, int timestep) {
        return convert_expr_id_to_z3(action.cost_id(), timestep);
    }

protected:

    // Helper function to print EPC index for debugging
    void print_epc_index(const std::string& context) const;

    // Member variables accessible to derived classes
    const Problem& problem_; // The planning problem instance
    z3::context& ctx_;       // Z3 context (shared)
    Z3VariableFactory variable_factory_; // Factory for creating and managing Z3 variables

    GroundedEncodingVisitor grounded_visitor_; // Grounded encoding visitor for individual variables

    // Parallelism strategy
    std::unique_ptr<ParallelismStrategy> parallelism_strategy_;

    // Pipeline artifacts
    std::vector<SymmetryInfo> symmetry_data_;
    StateConstraints state_constraints_;

    // Indices for the frame axioms

    // Map from grounded fluent (ExprID) to vector of (Action*, EffectExpression*). For example:
    // epc_index_[expr_id(at(airplane1, city1))] -> [(move_action*, effect_expr*), (fly_action*, effect_expr*)]
    // where each pair represents an action that can affect the fluent at(airplane1, city1)
    std::unordered_map<ExprID, std::vector<std::pair<const Action*, const EffectExpression*>>> epc_index_;

    void build_epc_index();

public:
    const auto& get_epc_index() const { return epc_index_; }

    /// Enable lazy frame mode (frame axioms supplied by a propagator module
    /// instead of the encoding).
    void set_lazy_frames(bool enabled) { lazy_frames_enabled_ = enabled; }

    /// Extract true action variables at a timestep from a Z3 model.
    std::vector<const Action*> extract_parallel_actions_at_timestep(const z3::model& model, int timestep) const;

protected:

    bool lazy_frames_enabled_ = false;

    int layers_encoded_ = -1; // Tracks the highest layer for which transitions are encoded

private:
    std::vector<const Action*> topologically_sort_actions(const std::vector<const Action*>& actions) const;
};

} // namespace rantanplan
