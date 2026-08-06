#pragma once

#include "../problem/problem.hpp"
#include "base_encoder.hpp"
#include "../smtcore/grounded_encoding_visitor.hpp"
#include "../smtcore/smt_types.hpp"
#include "../smtcore/smt_variable_factory.hpp"
#include "../smtcore/term_builder.hpp"
#include "parallelism/parallelism_strategy.hpp"

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
    GroundedEncoder(const Problem& problem, cvc5::TermManager& ctx,
                    std::unique_ptr<ParallelismStrategy> parallelism);

    // Encoding steps
    Term encode_initial_state() override;
    Term encode_actions(int t) override; // Encodes actions layer_idx
    Term encode_frames(int t) override; // Encodes frame axioms for layer_idx to layer_idx+1
    Term encode_goal(int t) override;    // Encodes goal conditions at layer_idx
    Term encode_parallelism(int t) override; // Encodes parallelism semantics
    Term encode_symmetries(int t) override;
    Term encode_state_constraints(int t) override;
    Term encode_prefix_monotone(int t) override; // Front-loading symmetry breaking

    // Strategy management
    std::string get_parallelism_strategy_name() const override;

    // Access to variable factory for plan extraction
    SmtVariableFactory& get_variable_factory() override { return variable_factory_; }
    const SmtVariableFactory& get_variable_factory() const override { return variable_factory_; }

    // Get access to parallelism strategy for plan extraction
    const ParallelismStrategy* get_parallelism_strategy() const override { return parallelism_strategy_.get(); }

    // Encode precondition + effect constraints for a single action at timestep t.
    // Returns nullopt if the action has no effects (nothing to encode).
    std::optional<Term> encode_single_action(const Action& action, int t);

    // Per-action effect constraints (action → effects), without preconditions.
    // ChainedGroundedEncoder overrides to exclude cumulative effects (those
    // require joint encoding across actions via encode_cumulative_effects).
    // Together with encode_cumulative_effects, these compose encode_actions.
    virtual std::optional<Term> encode_non_cumulative_effects(const Action& action, int t);

    // Per-timestep joint effect constraints across multiple actions.
    // Base: no-op.  ChainedGroundedEncoder: chain variables (ζ) that link
    // actions modifying the same numeric variable for correct cumulative
    // semantics under exists-step.
    // Together with encode_non_cumulative_effects, these compose encode_actions.
    virtual Term encode_cumulative_effects(int t);

    // Create action variables for all actions at timestep t without encoding constraints.
    void ensure_action_variables(int t);

    Plan extract_plan(const cvc5::Solver& solver, int max_timestep) const override;

    // Helper functions to convert expressions/effects to terms using the visitor (implementing base interface)
    Term convert_expr_id_to_z3(ExprID id, int timestep = -1) override;
    Term convert_effect_to_z3(const EffectExpression& effect, int timestep) override;

protected:

    // Helper function to print EPC index for debugging
    void print_epc_index(const std::string& context) const;

    /// cvc5 has no operator overloading: every connective, comparison and
    /// arithmetic node is built through the factory's TermBuilder. Read through
    /// the factory so the Int-vs-Real policy (set with set_problem) is always
    /// the current one.
    const TermBuilder& tb() const { return variable_factory_.builder(); }

    // Member variables accessible to derived classes
    const Problem& problem_; // The planning problem instance
    cvc5::TermManager& ctx_; // cvc5 TermManager (shared)
    SmtVariableFactory variable_factory_; // Factory for creating and managing SMT variables

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

    /// Extract true action variables at a timestep from the solver's model.
    std::vector<const Action*> extract_parallel_actions_at_timestep(const cvc5::Solver& solver, int timestep) const;

protected:

    bool lazy_frames_enabled_ = false;

    int layers_encoded_ = -1; // Tracks the highest layer for which transitions are encoded

private:
    std::vector<const Action*> topologically_sort_actions(const std::vector<const Action*>& actions) const;
};

} // namespace rantanplan
