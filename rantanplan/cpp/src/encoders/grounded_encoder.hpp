#pragma once

#include "../problem/problem.hpp"
#include "base_encoder.hpp"
#include "../z3core/grounded_encoding_visitor.hpp"
#include "../z3core/z3_variable_factory.hpp"
#include "parallelism/parallelism_strategy.hpp"
#include <z3++.h>

#include <optional>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>

// This class is able to handle the encoding of grounded fluents and actions.
namespace rantanplan {

class GroundedEncoder : public BaseEncoder {
public:
    // Constructor — the parallelism strategy is mandatory and injected here.
    GroundedEncoder(const Problem& problem, z3::context& ctx,
                    std::unique_ptr<ParallelismStrategy> parallelism);

    // Encoding steps
    z3::expr encode_initial_state() override;
    z3::expr encode_actions(int t) override; // Encodes actions layer_idx 
    z3::expr encode_frames(int t) override; // Encodes frame axioms for layer_idx to layer_idx+1
    z3::expr encode_goal(int t) override;    // Encodes goal conditions at layer_idx
    z3::expr encode_parallelism(int t) override; // Encodes parallelism semantics
    z3::expr encode_symmetries(int t) override;
    z3::expr encode_state_constraints(int t) override;
    z3::expr encode_prefix_monotone(int t) override; // Front-loading symmetry breaking
    
    // Strategy management
    std::string get_parallelism_strategy_name() const override;
    
    // Access to variable factory for plan extraction
    Z3VariableFactory& get_variable_factory() override { return variable_factory_; }
    const Z3VariableFactory& get_variable_factory() const override { return variable_factory_; }
    
    // Get access to parallelism strategy for plan extraction
    const ParallelismStrategy* get_parallelism_strategy() const override { return parallelism_strategy_.get(); }
    
    // Encode precondition + effect constraints for a single action at timestep t.
    // Returns nullopt if the action has no effects (nothing to encode).
    std::optional<z3::expr> encode_single_action(const Action& action, int t);

    // Per-action effect constraints (action → effects), without preconditions.
    // ChainedGroundedEncoder overrides to exclude cumulative effects (those
    // require joint encoding across actions via encode_cumulative_effects).
    // Together with encode_cumulative_effects, these compose encode_actions.
    virtual std::optional<z3::expr> encode_non_cumulative_effects(const Action& action, int t);

    // Per-timestep joint effect constraints across multiple actions.
    // Base: no-op.  ChainedGroundedEncoder: chain variables (ζ) that link
    // actions modifying the same numeric variable for correct cumulative
    // semantics under exists-step.
    // Together with encode_non_cumulative_effects, these compose encode_actions.
    virtual z3::expr encode_cumulative_effects(int t);

    /// Whether the effect encoding already pins f^{t+1} as a TOTAL function of
    /// f^t and the action literals, which makes a frame axiom for f entailed.
    ///
    /// The frame axiom says (f^t != f^{t+1}) -> OR_i (a_i [AND c_i]).  When the
    /// successor is total, every a_i false forces f^{t+1} = f^t, so the
    /// antecedent is unsatisfiable exactly where the consequent is — the axiom
    /// adds nothing and encode_frames skips the fluent.
    ///
    /// False for every fluent here: the base encoding states effects as
    /// implications (a -> f^{t+1} = eff(f^t)) and says nothing about f^{t+1}
    /// when a is false, so the frame axiom is what supplies persistence.
    /// ChainedGroundedEncoder overrides it for the fluents its chain covers.
    ///
    /// This must stay in sync with which fluents encode_cumulative_effects
    /// actually chains — skipping the frame axiom for a fluent the chain does
    /// NOT define leaves f^{t+1} unconstrained whenever no action writes it.
    virtual bool effects_determine_successor(ExprID /*fluent*/) const { return false; }

    // Create action variables for all actions at timestep t without encoding constraints.
    void ensure_action_variables(int t);

    Plan extract_plan(const z3::model& model, int max_timestep) const override;
    
    // Helper functions to convert expressions/effects to Z3 using visitor (implementing base interface)
    z3::expr convert_expr_id_to_z3(ExprID id, int timestep = -1) override;
    z3::expr convert_effect_to_z3(const EffectExpression& effect, int timestep) override;
    
protected:
    
    // Helper function to print EPC index for debugging
    void print_epc_index(const std::string& context) const;
    
    /// Written state variables whose fluent declares bounds, with that fluent
    /// (filled with the EPC index). A variable no action writes keeps its
    /// initial value, which UP requires to lie within the bounds.
    std::vector<std::pair<ExprID, const Fluent*>> declared_bounded_;

    // Member variables accessible to derived classes
    const Problem& problem_; // The planning problem instance
    z3::context& ctx_;       // Z3 context (shared)
    Z3VariableFactory variable_factory_; // Factory for creating and managing Z3 variables

    GroundedEncodingVisitor grounded_visitor_; // Grounded encoding visitor for individual variables

    // Parallelism strategy
    std::unique_ptr<ParallelismStrategy> parallelism_strategy_;

    // Indices for the frame axioms

    // Map from grounded fluent (ExprID) to vector of (Action*, EffectExpression*). For example:
    // epc_index_[expr_id(at(airplane1, city1))] -> [(move_action*, effect_expr*), (fly_action*, effect_expr*)]
    // where each pair represents an action that can affect the fluent at(airplane1, city1)
    std::unordered_map<ExprID, std::vector<std::pair<const Action*, const EffectExpression*>>> epc_index_;

    void build_epc_index();

public:
    const auto& get_epc_index() const { return epc_index_; }
    void set_lazy_frames(bool enabled) override { lazy_frames_enabled_ = enabled; }

    /// Extract true action variables at a timestep from a Z3 model.
    std::vector<const Action*> extract_parallel_actions_at_timestep(const z3::model& model, int timestep) const;
    /// The execution order of the actions of step t in `model`.
    virtual std::vector<const Action*> order_step(const z3::model& /*model*/, int /*t*/,
                                                  const std::vector<const Action*>& actions) const {
        return topologically_sort_actions(actions);
    }

protected:

    bool lazy_frames_enabled_ = false;

    int layers_encoded_ = -1; // Tracks the highest layer for which transitions are encoded

private:
    std::vector<const Action*> topologically_sort_actions(const std::vector<const Action*>& actions) const;
};

} // namespace rantanplan
