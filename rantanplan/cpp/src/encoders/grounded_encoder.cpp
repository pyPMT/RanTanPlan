#include "grounded_encoder.hpp"
#include "../analysis/interference_analysis.hpp"
#include "../smtcore/cardinality_encoding.hpp"
#include "../util/stats.hpp"
#include "../util/logger.hpp"
#include "../config/config.hpp"
#include <iostream>
#include <functional>

namespace rantanplan {

// Constructor
GroundedEncoder::GroundedEncoder(const Problem& problem, cvc5::TermManager& ctx,
                                 std::unique_ptr<ParallelismStrategy> parallelism)
    : problem_(problem), ctx_(ctx), variable_factory_(ctx), grounded_visitor_(ctx_, &problem_, &variable_factory_) {
    layers_encoded_ = -1;
    variable_factory_.set_problem(&problem_);
    build_epc_index();
    parallelism_strategy_ = std::move(parallelism);
    parallelism_strategy_->initialize(problem_, ctx_, variable_factory_);
}

Term GroundedEncoder::convert_expr_id_to_z3(ExprID id, int timestep) {
    return grounded_visitor_.convert_from_pool(id, timestep);
}

// Helper function to convert effect to an SMT constraint using the visitor
Term GroundedEncoder::convert_effect_to_z3(const EffectExpression& effect, int timestep) {
    Term fluent_curr_z3 = convert_expr_id_to_z3(effect.fluent_id(), timestep);
    Term fluent_next_z3 = convert_expr_id_to_z3(effect.fluent_id(), timestep + 1);
    Term value_z3 = convert_expr_id_to_z3(effect.value_id(), timestep);

    Term effect_constraint = tb().bool_val(true);
    switch (effect.kind()) {
        case EffectExpression::Kind::ASSIGN:
            effect_constraint = tb().mk_eq(fluent_next_z3, value_z3);
            break;
        case EffectExpression::Kind::INCREASE:
            effect_constraint = tb().mk_eq(fluent_next_z3, tb().add(fluent_curr_z3, value_z3));
            break;
        case EffectExpression::Kind::DECREASE:
            effect_constraint = tb().mk_eq(fluent_next_z3, tb().sub(fluent_curr_z3, value_z3));
            break;
    }

    if (effect.is_conditional()) {
        Term condition_z3 = convert_expr_id_to_z3(effect.condition_id(), timestep);
        effect_constraint = tb().mk_implies(condition_z3, effect_constraint);
    }

    return effect_constraint;
}

/**
 * @brief Encodes the initial state constraints at timestep 0
 *
 * Note: The Python side takes responsibility for initializing all fluents
 * with default values, so here we can just iterate over the initial state.
 */
Term GroundedEncoder::encode_initial_state() {
   TermVec initial_state;
    auto& stats = Stats::instance();

    // Process each assignment in the initial state at timestep 0
    for (const auto& assignment : problem_.initial_state()) {
        Term fluent_expr = convert_expr_id_to_z3(assignment.fluent_id(), 0);
        Term value_expr = convert_expr_id_to_z3(assignment.value_id(), 0);
        initial_state.push_back(tb().mk_eq(fluent_expr, value_expr));
    }

    // Declared state invariants hold at state 0 (states 1..h+1 are covered
    // by encode_state_constraints, which asserts them at t+1).  An initial
    // state violating a declaration makes the instance UNSAT — correct
    // under PDDL-requirement semantics.
    for (ExprID inv : problem_.state_invariants()) {
        initial_state.push_back(convert_expr_id_to_z3(inv, 0));
    }

    // Collect statistics
    stats.set("encoder.initial_constraints", initial_state.size());

    // Combine all initial state constraints with logical AND
    if (initial_state.empty()) {
        return tb().bool_val(true);
    }
    Term initial_state_formula = tb().mk_and(initial_state);
    return initial_state_formula;
}

Term GroundedEncoder::encode_actions(int t) {
    TermVec action_constraints;
    auto& stats = Stats::instance();

    for (const Action& action : problem_.actions()) {
        auto single = encode_single_action(action, t);
        if (single) {
            action_constraints.push_back(*single);
        }
    }

    stats.add("encoder.action_constraints", action_constraints.size());

    if (action_constraints.empty()) {
        return tb().bool_val(true);
    }
    return tb().mk_and(action_constraints);
}

std::optional<Term> GroundedEncoder::encode_single_action(const Action& action, int t) {
    if (action.effects().empty()) return std::nullopt;

    TermVec constraints;
    Term action_var = variable_factory_.get_action_variable(action, t);

    if (action.has_precondition()) {
        Term z3_precond = convert_expr_id_to_z3(action.precondition_id(), t);
        constraints.push_back(tb().mk_implies(action_var, z3_precond));
    }

    TermVec effect_exprs;
    for (const Effect& effect : action.effects()) {
        effect_exprs.push_back(convert_effect_to_z3(effect.effect_expression(), t));
    }
    constraints.push_back(tb().mk_implies(action_var, tb().mk_and(effect_exprs)));

    return tb().mk_and(constraints);
}

std::optional<Term> GroundedEncoder::encode_non_cumulative_effects(const Action& action, int t) {
    if (action.effects().empty()) return std::nullopt;

    Term action_var = variable_factory_.get_action_variable(action, t);

    TermVec effect_exprs;
    for (const Effect& effect : action.effects()) {
        effect_exprs.push_back(convert_effect_to_z3(effect.effect_expression(), t));
    }

    return tb().mk_implies(action_var, tb().mk_and(effect_exprs));
}

Term GroundedEncoder::encode_cumulative_effects(int t) {
    return tb().bool_val(true);
}

/**
 * @brief Encodes frame axioms for a specific time step
 *
 * Frame axioms ensure that fluents only change when explicitly caused by
 * actions.  For each fluent in the EPC index, adds a constraint that says: "if a
 * fluent changes between time t and t+1, then at least one action that can
 * cause this change must be executed at time t".
 *
 * (at_robot_A^t != at_robot_A^(t+1)) -> (move_A_to_B^t || (Effect_precondition^t && conditional_action^t))
 *
 * @param t The current time step for which to encode frame axioms
 * @return A term representing the conjunction of all frame
 * axioms for the transition from time t to t+1. Returns true if no fluents
 * exist in the EPC index.
 */
Term GroundedEncoder::encode_frames(int t) {
    // When the frame propagator is active, it handles ALL frame axioms lazily
    if (lazy_frames_enabled_) {
        return tb().bool_val(true);
    }

    std::vector<Term> frame_axioms;
    auto& stats = Stats::instance();

    // Iterate through all grounded fluents and look up in EPC index
    for (ExprID eid : problem_.grounded_fluents()) {
        auto epc_it = epc_index_.find(eid);
        if (epc_it == epc_index_.end()) continue;
        const auto& action_effects = epc_it->second;

        // Get fluent variables at timesteps t and t+1
        Term fluent_t = convert_expr_id_to_z3(eid, t);
        Term fluent_t_plus_1 = convert_expr_id_to_z3(eid, t + 1);

        // Frame axiom: (f^t != f^{t+1}) -> (a_1 v (a_2 ^ c_2) v ... v a_n)
        //
        // For boolean fluents, an alternative "direction-aware" encoding splits
        // this into two clauses by effect polarity — one with only the actions
        // that set f to true, one with only those that set f to false.  These
        // are derivable by resolving the combined frame clause with the effect
        // axioms and are strictly shorter.  However, a full experiment on 417
        // instances (branch frame-polarity, Apr 2026) showed this is 16% slower
        // on average (geo mean 1.16, 89 losses vs 45 wins).  The shorter
        // clauses disrupt the solver's VSIDS heuristics more than they help BCP,
        // especially on boolean-heavy domains (42% slower, 3:1 loss ratio).
        // Mixed boolean/numeric domains benefit slightly (geo 0.93) but not
        // enough to offset the overall loss.  Conclusion: keep the combined
        // encoding.
        Term fluent_changed = tb().mk_distinct(fluent_t, fluent_t_plus_1);

        // Build disjunction of all actions that can cause this change
        std::vector<Term> action_terms;

        for (const auto& [action, effect_expr] : action_effects) {
            // Get the action variable at timestep t
            Term action_var = variable_factory_.get_action_variable(*action, t);

            // For conditional effects: action_var && condition
            if (effect_expr->is_conditional()) {
                Term condition_z3 = convert_expr_id_to_z3(effect_expr->condition_id(), t);
                action_terms.push_back(tb().mk_and({action_var, condition_z3})); // condition /\ action_var
            } else {
                action_terms.push_back(action_var); // For unconditional effects: just the action variable
            }
        }

        // Create the action disjunction
        Term action_disjunction = tb().bool_val(false);
        if (!action_terms.empty()) {
            // Create a flat disjunction
            TermVec action_vector;
            for (const auto& term : action_terms) {
                action_vector.push_back(term);
            }
            action_disjunction = tb().mk_or(action_vector);
        }

        // Create the frame axiom: fluent_changed -> action_disjunction
        // This is equivalent to: (fluent^t != fluent^(t+1)) -> (a1 || (epc2 && a2) || a3 || ...)
        Term frame_axiom = tb().mk_implies(fluent_changed, action_disjunction);
        frame_axioms.push_back(frame_axiom);
    }

    // Collect statistics
    stats.add("encoder.frame_axioms", frame_axioms.size());

    // Combine all frame axioms with logical AND
    if (frame_axioms.empty()) {
        auto expr = tb().bool_val(true);
        return expr;
    }

    // Create a flat conjunction
    TermVec frame_vector;
    for (const auto& axiom : frame_axioms) {
        frame_vector.push_back(axiom);
    }
    Term big_and = tb().mk_and(frame_vector);
    return big_and;
}

Term GroundedEncoder::encode_goal(int t) {
    // Retrieve goals from the problem
    const auto& goals = problem_.goals();
    auto& stats = Stats::instance();

    if (goals.empty()) {
        return tb().bool_val(true); // If no goals, vacuously satisfied
    }

    // Convert each goal expression to a term and collect them
    std::vector<Term> goal_formulas;
    goal_formulas.reserve(goals.size());

    for (const auto& goal : goals) {
        goal_formulas.push_back(convert_expr_id_to_z3(goal.goal_id(), t));
    }

    // Collect statistics
    stats.set("encoder.goal_constraints", goal_formulas.size());

    // Combine all goal formulas with logical AND
    if (goal_formulas.empty()) {
        return tb().bool_val(true);
    }

    // Create a flat conjunction
    TermVec goal_vector;
    for (const auto& goal : goal_formulas) {
        goal_vector.push_back(goal);
    }
    Term goal_conjunction = tb().mk_and(goal_vector);
    return goal_conjunction;
}

Term GroundedEncoder::encode_parallelism(int t) {
    return parallelism_strategy_->encode_parallelism(t);
}
Term GroundedEncoder::encode_prefix_monotone(int t) {
    // Build: (∀a. ¬a@t)  →  (∀a. ¬a@(t+1))
    // i.e., if no action fires at t, then no action may fire at t+1.
    // Chained across all t during search, this front-loads all active steps
    // to the prefix of the plan (0..k-1), with empty steps (k..h-1) at the end.
    // Frame axioms then propagate the goal state through the empty suffix,
    // so a single goal literal placed at horizon h witnesses any plan of
    // length k <= h — enabling the horizon schedule's single-literal batching.
    TermVec no_action_t, no_action_t1;
    for (const Action& a : problem_.actions()) {
        no_action_t.push_back(tb().mk_not(variable_factory_.get_action_variable(a, t)));
        no_action_t1.push_back(tb().mk_not(variable_factory_.get_action_variable(a, t + 1)));
    }
    if (no_action_t.empty()) {
        return tb().bool_val(true);
    }
    return
        tb().mk_implies(tb().mk_and(no_action_t), tb().mk_and(no_action_t1))
    ;
}

Term GroundedEncoder::encode_symmetries(int t) {
    auto& stats = Stats::instance();

    if (symmetry_data_.empty()) {
        return tb().bool_val(true);
    }

    TermVec symmetry_constraints;
    int ordering_constraints_count = 0;

    for (const auto& sym : symmetry_data_) {
        // LHS: all variable pairs have the same value (symmetric state)
        TermVec variable_equality_constraints;
        for (const auto& [var1_eid, var2_eid] : sym.variable_pairs) {
            Term var1_z3 = convert_expr_id_to_z3(var1_eid, t);
            Term var2_z3 = convert_expr_id_to_z3(var2_eid, t);
            variable_equality_constraints.push_back(tb().mk_eq(var1_z3, var2_z3));
        }

        // RHS: lexicographic ordering on action pairs
        TermVec action_ordering_constraints;
        for (const auto& action_pair : sym.action_pairs) {
            Term action1_var = variable_factory_.get_action_variable(*action_pair.action1, t);
            Term action2_var = variable_factory_.get_action_variable(*action_pair.action2, t);

            std::string name1 = variable_factory_.get_action_var_name(*action_pair.action1);
            std::string name2 = variable_factory_.get_action_var_name(*action_pair.action2);
            Term ordering_constraint = (name1 < name2) ?
                tb().mk_implies(action1_var, action2_var) :
                tb().mk_implies(action2_var, action1_var);
            action_ordering_constraints.push_back(ordering_constraint);
            ordering_constraints_count++;
        }

        // (all variables are symmetric) => (lexicographic ordering on actions)
        if (!variable_equality_constraints.empty() && !action_ordering_constraints.empty()) {
            Term lhs = tb().mk_and(variable_equality_constraints);
            Term rhs = tb().mk_and(action_ordering_constraints);
            symmetry_constraints.push_back(tb().mk_implies(lhs, rhs));
        }
    }

    stats.add("encoder.symmetry_ordering_constraints", ordering_constraints_count);

    if (symmetry_constraints.empty()) {
        return tb().bool_val(true);
    }
    return tb().mk_and(symmetry_constraints);
}

// Two sources feed this method.  Mechanically they are the same thing —
// "assert formula f at every state" — and could share one convention;
// what actually distinguishes them is ONE bit: may the constraint be
// skipped?
//
//  - state_constraints_: DERIVED by the preprocessor (RPG bounds, domains,
//    and conservation laws from ActionPruningPass; mutex groups from
//    MutexAnalysisPass).  Proven redundant, so
//    skipping them anywhere is sound (options off, final state uncovered)
//    — they only ever cost speed.
//  - problem_.state_invariants(): DECLARED by the user via
//    `(:constraints (always f))`.  Trusted requirements that define plan
//    validity — never skippable, by anyone.  Hence: read from the problem
//    (immune to oracle options and its state_constraints_ overwrite),
//    asserted at t+1 here plus state 0 in encode_initial_state so every
//    state 0..h+1 including the GOAL state is covered exactly once.
//
// The differing timestep conventions (derived at t, declared at t+1) are
// NOT semantic: derived constraints could equally use the exactly-once
// convention.  They keep their historical at-t form so this feature does
// not perturb the measured behavior of every existing strategy; unifying
// the conventions is fine but needs a suite A/B.
Term GroundedEncoder::encode_state_constraints(int t) {
    if (state_constraints_.empty() && problem_.state_invariants().empty()) {
        return tb().bool_val(true);
    }

    TermVec conjuncts;

    // Declared state invariants: planners call this for every added
    // timestep t = 0..h, so asserting at t+1 covers states 1..h+1.
    for (ExprID inv : problem_.state_invariants()) {
        conjuncts.push_back(convert_expr_id_to_z3(inv, t + 1));
    }

    // Mutex constraints: atmost(members, 1) and optionally atleast(members, 1)
    auto& stats = Stats::instance();
    for (const auto& mutex : state_constraints_.mutexes) {
        TermVec group;
        for (ExprID member : mutex.members) {
            group.push_back(convert_expr_id_to_z3(member, t));
        }
        if (static_cast<double>(group.size()) > stats.get("encoder.max_mutex_group_size")) {
            stats.set("encoder.max_mutex_group_size", static_cast<double>(group.size()));
        }
        conjuncts.push_back(encode_atmost(ctx_, group, 1));
        if (mutex.exactly_one) {
            conjuncts.push_back(encode_atleast(ctx_, group, 1));
        }
    }

    // Numeric bound constraints (RPG-derived and declared bounded types).
    // Asserted at t+1: this is called per transition t = 0..h-1, so t+1 covers
    // states 1..h — including the goal state h, which declared bounds must cover
    // (state-invariant semantics). State 0 is pinned by encode_initial_state;
    // RPG bounds are over-approximations, so t+1 is equally sound for them.
    for (const auto& bound : state_constraints_.bounds) {
        Term fluent = convert_expr_id_to_z3(bound.fluent_id, t + 1);
        Term val = fluent.getSort().isInteger()
            ? tb().int_val(static_cast<int64_t>(bound.bound))
            : ctx_.mkReal(std::to_string(bound.bound));
        if (bound.is_lower) {
            conjuncts.push_back(tb().geq(fluent, val));
        } else {
            conjuncts.push_back(tb().leq(fluent, val));
        }
    }

    // Object fluent domain constraints: restrict to reachable values
    for (const auto& dc : state_constraints_.domains) {
        Term fluent = convert_expr_id_to_z3(dc.fluent_id, t);
        TermVec valid;
        for (int idx : dc.valid_object_indices) {
            Term val = fluent.getSort().isInteger()
                ? tb().int_val(static_cast<int64_t>(idx))
                : ctx_.mkReal(static_cast<int64_t>(idx));
            valid.push_back(tb().mk_eq(fluent, val));
        }
        if (!valid.empty()) {
            conjuncts.push_back(tb().mk_or(valid));
        }
    }

    // Conservation law constraints: f_t + g_t == C
    for (const auto& cons : state_constraints_.conservations) {
        Term f = convert_expr_id_to_z3(cons.fluent1_id, t);
        Term g = convert_expr_id_to_z3(cons.fluent2_id, t);
        Term c = f.getSort().isInteger()
            ? tb().int_val(static_cast<int64_t>(cons.constant))
            : ctx_.mkReal(std::to_string(cons.constant));
        conjuncts.push_back(tb().mk_eq(tb().add(f, g), c));
    }

    if (conjuncts.empty()) {
        return tb().bool_val(true);
    }
    return tb().mk_and(conjuncts);
}

std::string GroundedEncoder::get_parallelism_strategy_name() const {
    if (parallelism_strategy_) {
        return parallelism_strategy_->get_name();
    }
    return "Unknown";
}

void GroundedEncoder::print_epc_index(const std::string& context) const {
    std::cout << "\n===== EPC Index: " << context << " =====" << std::endl;

    if (epc_index_.empty()) {
        std::cout << "(empty)" << std::endl;
    } else {
        for (const auto& [eid, action_effects] : epc_index_) {
            std::cout << "Fluent: " << problem_.pool().to_string(eid) << std::endl;
            for (const auto& [action, eff_expr] : action_effects) {
                std::cout << "  Modified by Action: " << action->name() << " | Effect: " << eff_expr->to_string() << std::endl;
            }
        }
    }
    std::cout << "===== End EPC Index =====" << std::endl << std::endl;
}

void GroundedEncoder::build_epc_index() {

    // Local helper function to index effect fluents in case we need to handle complex effects
    // in the future like nested quantified/conditional effects. As of now, I think
    // the limitation is UP's though ...
    auto index_effect_fluents = [this](const Action* action, const EffectExpression* eff_expr) {
        // Index the direct effect
        epc_index_[eff_expr->fluent_id()].emplace_back(action, eff_expr);
    };

    epc_index_.clear();

    // Use the systematic grounded fluent collection from the problem
    // This ensures ALL grounded fluents get frame axioms, including those from
    // initial state, action effects, preconditions, and goals
    for (ExprID eid : problem_.grounded_fluents()) {
        epc_index_[eid] = std::vector<std::pair<const Action*, const EffectExpression*>>();
    }

    // Then, add action effects to the fluents that can be modified
    for (const auto& action : problem_.actions()) {
        for (const auto& effect : action.effects()) {
            const EffectExpression& eff_expr = effect.effect_expression();
            index_effect_fluents(&action, &eff_expr);
        }
    }

    // Print the index for debugging
    //print_epc_index("After building EPC index");
}

Plan GroundedEncoder::extract_plan(const cvc5::Solver& solver, int max_timestep) const {
    Plan plan;

    // cvc5 has no model object to size up: values are pulled one term at a time
    // with solver.getValue().
    Logger::instance().info("Extracting plan from the cvc5 model");

    // Use type-safe capability query instead of string comparison
    bool is_parallel = get_parallelism_strategy()->allows_concurrent_actions();

    // Iterate through each timestep
    for (int t = 0; t < max_timestep; ++t) {
        if (is_parallel) {
            // Extract and order parallel actions for this timestep
            std::vector<const Action*> parallel_actions = extract_parallel_actions_at_timestep(solver, t);

            if (!parallel_actions.empty()) {
                std::vector<const Action*> ordered_actions = topologically_sort_actions(parallel_actions);
                for (const Action* action : ordered_actions) {
                    plan.add_action(action);
                }
            }
        } else {
            // Original sequential extraction logic
            for (const Action& grounded_action : problem_.actions()) {
                Term action_var = variable_factory_.get_action_variable(grounded_action, t);
                Term action_value = solver.getValue(action_var);

                if (action_value.isBooleanValue() && action_value.getBooleanValue()) {
                    plan.add_action(&grounded_action);
                    break; // Only one action in sequential mode
                }
            }
        }
    }
    return plan;
}

std::vector<const Action*> GroundedEncoder::extract_parallel_actions_at_timestep(
    const cvc5::Solver& solver, int timestep) const {

    std::vector<const Action*> parallel_actions;

    for (const Action& grounded_action : problem_.actions()) {
        try {
            Term action_var = variable_factory_.get_action_variable(grounded_action, timestep);
            Term action_value = solver.getValue(action_var);

            if (action_value.isBooleanValue() && action_value.getBooleanValue()) {
                parallel_actions.push_back(&grounded_action);
            }
        } catch (const std::exception&) {
            // Skip actions whose variables don't exist
        }
    }

    return parallel_actions;
}

std::vector<const Action*> GroundedEncoder::topologically_sort_actions(
    const std::vector<const Action*>& actions) const {

    if (actions.size() <= 1) {
        return actions;
    }

    const ParallelismStrategy* strategy = get_parallelism_strategy();
    const InterferenceOracle* analyzer = strategy->get_interference_analyzer();

    return analyzer->topological_sort_actions(actions);
}

} // namespace rantanplan
