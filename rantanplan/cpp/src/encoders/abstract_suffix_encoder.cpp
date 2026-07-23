#include "abstract_suffix_encoder.hpp"
#include "grounded_encoder.hpp"
#include "../analysis/graph.hpp"
#include "../problem/visitors/fluent_collector.hpp"
#include "../problem/expr_enums.hpp"

#include <string>
#include <unordered_map>

namespace rantanplan {

// ============================================================================
// Construction
// ============================================================================

AbstractSuffixEncoder::AbstractSuffixEncoder(
    const Problem& problem, cvc5::TermManager& ctx,
    GroundedEncoder& encoder,
    SemanticsKind semantics)
    : problem_(problem), ctx_(ctx), tb_(ctx, &problem), encoder_(encoder),
      semantics_(semantics),
      syntactic_(semantics == SemanticsKind::Sequential
                     ? nullptr
                     : std::make_unique<SyntacticInterference>(problem)),
      epc_index_(&encoder.get_epc_index()),
      cached_loop_formulas_(tb_.bool_val(true)) {}

void AbstractSuffixEncoder::initialize() {
    const auto& actions = problem_.actions();
    const auto& fluents = problem_.grounded_fluents();

    // One abstract Bool per action.
    //
    // `action.name()` is the *schema* name (the grounded parameters live in
    // action.parameters()), so all groundings of one schema render the same
    // "abs_<name>" and share ONE abstract variable: the suffix abstracts over
    // schemas, not groundings. The name-keyed map is what enforces that
    // sharing — fresh_bool mints a fresh constant on every call, so each name
    // must be created once and reused. Switching to per-grounding abstraction
    // would be a semantic change to the abstract suffix, not a naming detail.
    std::unordered_map<std::string, Term> abs_by_name;
    action_abs_.reserve(actions.size());
    for (const auto& action : actions) {
        std::string name = "abs_" + action.name();
        auto it = abs_by_name.find(name);
        if (it == abs_by_name.end()) {
            it = abs_by_name.emplace(name, tb_.fresh_bool(name)).first;
        }
        action_abs_.push_back(it->second);
    }
    action_abs_unique_.reserve(abs_by_name.size());
    for (const auto& [name, var] : abs_by_name) action_abs_unique_.push_back(var);

    // One mod_v Bool per grounded fluent (each created exactly once and held
    // in mod_vars_, so freshness is fine here).
    mod_vars_.reserve(fluents.size());
    for (size_t i = 0; i < fluents.size(); ++i) {
        mod_vars_.push_back(tb_.fresh_bool("mod_" + std::to_string(i)));
        fluent_to_mod_index_[fluents[i]] = i;
    }
}

// ============================================================================
// Helpers
// ============================================================================

std::vector<ExprID> AbstractSuffixEncoder::get_precondition_conjuncts(ExprID precond_id) const {
    std::vector<ExprID> conjuncts;
    const auto& pool = problem_.pool();

    if (!precond_id.valid()) return conjuncts;

    if (pool.is_and(precond_id)) {
        // AND node — arguments (children[1..]) are the conjuncts.
        // children[0] is the AND function symbol.
        for (ExprID arg : pool.arguments(precond_id)) {
            conjuncts.push_back(arg);
        }
    } else {
        // Single conjunct
        conjuncts.push_back(precond_id);
    }
    return conjuncts;
}

TermVec AbstractSuffixEncoder::get_mod_vars_for_expr(ExprID expr_id) const {
    TermVec mods;
    FluentCollector collector(problem_);
    collector.collect_from_id(expr_id);
    for (ExprID fluent_id : collector.get_fluents()) {
        auto it = fluent_to_mod_index_.find(fluent_id);
        if (it != fluent_to_mod_index_.end()) {
            mods.push_back(mod_vars_[it->second]);
        }
    }
    return mods;
}

// ============================================================================
// Permanent: mod_v equivalences (Eq. 4B)
//
// For each grounded fluent v: mod_v <=> (OR of a^abs for actions affecting v).
// Uses the epc_index_ (effects-per-condition index) from GroundedEncoder.
// Horizon-independent — only references abstract vars and action structure.
// ============================================================================

Term AbstractSuffixEncoder::encode_mod_v_equivalences() {
    TermVec all_constraints;

    if (epc_index_) {
        for (size_t v_idx = 0; v_idx < mod_vars_.size(); ++v_idx) {
            ExprID fluent_id = problem_.grounded_fluents()[v_idx];
            auto it = epc_index_->find(fluent_id);

            TermVec affecting_abs;
            if (it != epc_index_->end()) {
                for (const auto& [action_ptr, _] : it->second) {
                    int aid = action_ptr->id();
                    if (aid >= 0 && static_cast<size_t>(aid) < action_abs_.size()) {
                        affecting_abs.push_back(action_abs_[aid]);
                    }
                }
            }

            if (affecting_abs.empty()) {
                // No action can affect this fluent — mod_v must be false
                all_constraints.push_back(tb_.mk_not(mod_vars_[v_idx]));
            } else {
                all_constraints.push_back(
                    tb_.mk_eq(mod_vars_[v_idx], tb_.mk_or(affecting_abs)));
            }
        }
    }

    return all_constraints.empty() ? tb_.bool_val(true) : tb_.mk_and(all_constraints);
}

// ============================================================================
// Permanent: Loop Formulas (Eq. 5)
//
// Prevent circular support among abstract actions. Without these, the abstract
// suffix could contain a cycle where action A's precondition is "justified" by
// mod_v from action B, and B's precondition is justified by mod_w from A —
// neither action has genuine external support, yet both appear feasible.
//
// Implementation:
//   1. Build a dependency graph with two kinds of nodes:
//      - mod_v nodes (one per grounded fluent, indices 0..num_fluents-1)
//      - precondition atom nodes (indices num_fluents..)
//      Edges capture "action a affects fluent v AND requires precondition phi"
//      and "precondition phi references fluent v".
//   2. Compute SCCs using Tarjan's algorithm (via existing Graph class).
//   3. For each non-trivial SCC: require that if any mod_v in the SCC is true,
//      there must be external support (a mod_v outside the SCC with an edge in).
//      If no external support exists, all mod_v nodes in the SCC must be false.
//
// The result is cached since it depends only on action structure, not horizon.
// ============================================================================

Term AbstractSuffixEncoder::encode_loop_formulas() {
    if (loop_formulas_built_) return cached_loop_formulas_;

    // Build dependency graph for loop formula detection.
    // Nodes: mod_v variables (indices 0..num_fluents-1)
    //        + precondition atoms (indices num_fluents..)
    // Edges: for each action a^abs, for each mod_v in effects of a,
    //        add edge from mod_v node to each precondition atom node of a.

    const auto& actions = problem_.actions();
    const auto& fluents = problem_.grounded_fluents();
    size_t num_fluents = fluents.size();

    // Map precondition atom ExprIDs to node indices
    std::unordered_map<ExprID, size_t> precond_to_node;
    std::vector<ExprID> node_to_precond;  // for reverse lookup

    // Assign node indices: [0..num_fluents-1] for mod_v, [num_fluents..] for precond atoms
    size_t next_node = num_fluents;

    for (const auto& action : actions) {
        if (!action.has_precondition()) continue;
        auto conjuncts = get_precondition_conjuncts(action.precondition_id());
        for (ExprID phi : conjuncts) {
            if (precond_to_node.find(phi) == precond_to_node.end()) {
                precond_to_node[phi] = next_node++;
                node_to_precond.push_back(phi);
            }
        }
    }

    size_t total_nodes = next_node;
    Graph graph;
    for (size_t i = 0; i < total_nodes; ++i) {
        graph.add_node();
    }

    // Build edges: for each action, for each fluent v it affects,
    // add edge from mod_v -> each precondition atom of the action
    for (size_t a_idx = 0; a_idx < actions.size(); ++a_idx) {
        const Action& action = actions[a_idx];
        if (!action.has_precondition()) continue;

        auto conjuncts = get_precondition_conjuncts(action.precondition_id());

        // Find fluents affected by this action (from epc_index, look for this action)
        for (const auto& effect : action.effects()) {
            ExprID fluent_id = effect.effect_expression().fluent_id();
            auto it = fluent_to_mod_index_.find(fluent_id);
            if (it == fluent_to_mod_index_.end()) continue;
            size_t mod_node = it->second;

            for (ExprID phi : conjuncts) {
                auto pit = precond_to_node.find(phi);
                if (pit != precond_to_node.end()) {
                    graph.add_edge(mod_node, pit->second);
                }
            }
        }

        // Also: precondition atoms depend on mod_v for fluents they reference
        for (ExprID phi : conjuncts) {
            auto pit = precond_to_node.find(phi);
            if (pit == precond_to_node.end()) continue;

            FluentCollector collector(problem_);
            collector.collect_from_id(phi);
            for (ExprID fid : collector.get_fluents()) {
                auto mit = fluent_to_mod_index_.find(fid);
                if (mit != fluent_to_mod_index_.end()) {
                    graph.add_edge(pit->second, mit->second);
                }
            }
        }
    }

    auto sccs = graph.compute_strongly_connected_components();

    TermVec loop_constraints;

    for (const auto& scc : sccs) {
        // Skip trivial SCCs (size 1, no self-loop)
        if (scc.size() <= 1) {
            if (scc.size() == 1 && !graph.has_edge(scc[0], scc[0])) {
                continue;
            }
        }

        std::unordered_set<int> scc_set(scc.begin(), scc.end());

        // Build OR of all nodes in SCC (as terms)
        TermVec scc_exprs;
        for (int node : scc) {
            if (static_cast<size_t>(node) < num_fluents) {
                scc_exprs.push_back(mod_vars_[node]);
            }
            // Precondition atoms are not SMT variables in the abstract suffix;
            // they participate in the dependency graph but the loop formula
            // constraint only applies to mod_v variables.
        }

        if (scc_exprs.empty()) continue;

        // Find R(L): nodes outside SCC with edges into SCC
        TermVec external_support;
        for (int node : scc) {
            // Check all potential predecessors
            for (size_t pred = 0; pred < total_nodes; ++pred) {
                if (scc_set.count(pred)) continue;  // skip SCC-internal
                if (graph.has_edge(pred, node)) {
                    if (pred < num_fluents) {
                        external_support.push_back(mod_vars_[pred]);
                    }
                    // External precondition atoms: they represent concrete
                    // state conditions that can provide external support.
                    // Treat them as always satisfiable (grounded in the
                    // concrete prefix) by adding unconditional support.
                    external_support.push_back(tb_.bool_val(true));
                }
            }
        }

        if (external_support.empty()) {
            // No external support — SCC nodes must all be false
            for (size_t i = 0; i < scc_exprs.size(); ++i) {
                loop_constraints.push_back(tb_.mk_not(scc_exprs[i]));
            }
        } else {
            // (OR scc_nodes) => (OR external_support)
            loop_constraints.push_back(
                tb_.mk_implies(tb_.mk_or(scc_exprs), tb_.mk_or(external_support)));
        }
    }

    cached_loop_formulas_ = loop_constraints.empty()
        ? tb_.bool_val(true)
        : tb_.mk_and(loop_constraints);
    loop_formulas_built_ = true;

    return cached_loop_formulas_;
}

// ============================================================================
// Per-step incremental: Axiom 6 — no arbitrary deferral (Eq. 6)
//
// Prevents the solver from trivially deferring actions to later timesteps
// without justification. For each concrete action a at step i > 0, a can only
// be true if at least one of:
//   - a was already executing at step i-1 (persistence)
//   - some precondition of a was unsatisfied at i-1 (couldn't execute earlier)
//   - a mutex action a' was executing at i-1 (blocked by interference)
//
// The mutex set M_a depends on the semantics:
//   - Sequential: M_a = all other actions (only one action per step)
//   - Forall/Exists: M_a = actions with bidirectional interference
//
// Only references concrete action variables at steps i and i-1 — once added,
// remains valid forever.
// ============================================================================

Term AbstractSuffixEncoder::encode_axiom_6(int i) {
    // For each action a, at step i > 0:
    // a_i => (a_{i-1} OR (OR_{phi in pre_a} NOT phi_{i-1}) OR (OR_{a' in M_a} a'_{i-1}))
    TermVec all_constraints;
    const auto& actions = problem_.actions();
    auto& vf = encoder_.get_variable_factory();

    for (size_t a_idx = 0; a_idx < actions.size(); ++a_idx) {
        const Action& action = actions[a_idx];
        const Term& a_i = vf.get_action_variable(action, i);
        const Term& a_prev = vf.get_action_variable(action, i - 1);

        TermVec reasons;
        reasons.push_back(a_prev);  // action was already executing

        // Precondition was not satisfied at i-1
        if (action.has_precondition()) {
            auto conjuncts = get_precondition_conjuncts(action.precondition_id());
            for (ExprID phi : conjuncts) {
                reasons.push_back(tb_.mk_not(encoder_.convert_expr_id_to_smt(phi, i - 1)));
            }
        }

        // Mutex action was executing at i-1
        if (semantics_ == SemanticsKind::Sequential) {
            // M_a = all other actions
            for (size_t b_idx = 0; b_idx < actions.size(); ++b_idx) {
                if (b_idx == a_idx) continue;
                reasons.push_back(vf.get_action_variable(actions[b_idx], i - 1));
            }
        } else if (syntactic_) {
            // M_a = actions with bidirectional syntactic interference.
            // Syntactic interference is a sound overapproximation: it may include
            // pairs that don't interfere semantically, making axiom 6 slightly
            // weaker (more deferral reasons) but still a valid lower bound.
            // The semantic precision is handled by the propagator during solving.
            // TODO: Consider handling axiom 6 via a propagator instead of
            // pre-materializing all M_a clauses. A propagator watching abstract
            // action variables could add the M_a disjunction on-demand only when
            // the solver sets a_i = true and no other justification (a_{i-1} or
            // unsatisfied preconditions) exists. This would avoid the O(h * |A|^2)
            // clause overhead entirely, matching the lazy philosophy of the exists
            // propagator for the concrete prefix.
            for (size_t b_idx = 0; b_idx < actions.size(); ++b_idx) {
                if (b_idx == a_idx) continue;
                if (syntactic_->interferes(action, actions[b_idx]) ||
                    syntactic_->interferes(actions[b_idx], action)) {
                    reasons.push_back(vf.get_action_variable(actions[b_idx], i - 1));
                }
            }
        }

        all_constraints.push_back(tb_.mk_implies(a_i, tb_.mk_or(reasons)));
    }

    return all_constraints.empty() ? tb_.bool_val(true) : tb_.mk_and(all_constraints);
}

// ============================================================================
// Per-step incremental: Axiom 7 step (Eq. 7, decomposed)
//
// Original Eq. 7: (OR a^abs) => AND_{i=0}^{n-1} (OR_a a_i)
// Decomposed into individual per-step clauses:
//   encode_axiom_7_step(i): (OR a^abs) => (OR_a a_i)
//
// The conjunction of all steps 0..n-1 is equivalent to the original.
// Each clause is independent and permanent once added.
// ============================================================================

Term AbstractSuffixEncoder::encode_axiom_7_step(int i) {
    const auto& actions = problem_.actions();
    auto& vf = encoder_.get_variable_factory();

    if (action_abs_.empty()) return tb_.bool_val(true);

    TermVec any_abstract;
    for (const auto& a_abs : action_abs_) {
        any_abstract.push_back(a_abs);
    }

    TermVec any_concrete;
    for (const auto& action : actions) {
        any_concrete.push_back(vf.get_action_variable(action, i));
    }

    if (any_concrete.empty()) return tb_.bool_val(true);

    return tb_.mk_implies(tb_.mk_or(any_abstract), tb_.mk_or(any_concrete));
}

// ============================================================================
// Horizon-dependent: Relaxed Preconditions (Eq. 4A)
//
// For each action a: a^abs => AND_{phi in pre_a} (phi_n OR OR_{v in vars(phi)} mod_v)
//
// The phi_n term evaluates preconditions at state n, making this
// horizon-dependent. The planner gates this with an assumption literal.
// ============================================================================

Term AbstractSuffixEncoder::encode_relaxed_preconditions(int n) {
    TermVec all_constraints;
    const auto& actions = problem_.actions();

    for (size_t a_idx = 0; a_idx < actions.size(); ++a_idx) {
        const Action& action = actions[a_idx];
        if (!action.has_precondition()) continue;

        auto conjuncts = get_precondition_conjuncts(action.precondition_id());
        TermVec relaxed_preconds;

        for (ExprID phi : conjuncts) {
            Term phi_at_n = encoder_.convert_expr_id_to_smt(phi, n);
            TermVec mods = get_mod_vars_for_expr(phi);

            if (mods.empty()) {
                relaxed_preconds.push_back(phi_at_n);
            } else {
                TermVec disj;
                disj.push_back(phi_at_n);
                for (size_t i = 0; i < mods.size(); ++i) {
                    disj.push_back(mods[i]);
                }
                relaxed_preconds.push_back(tb_.mk_or(disj));
            }
        }

        if (!relaxed_preconds.empty()) {
            all_constraints.push_back(
                tb_.mk_implies(action_abs_[a_idx], tb_.mk_and(relaxed_preconds)));
        }
    }

    return all_constraints.empty() ? tb_.bool_val(true) : tb_.mk_and(all_constraints);
}

// ============================================================================
// Horizon-dependent: Abstract Goal (Eq. 3)
//
// Each goal g is relaxed: (g holds at state n) OR (some fluent in g could be
// modified by the abstract suffix). This allows the solver to find plans where
// goals are not yet achieved but could plausibly be achieved in the suffix.
//
// The g_at_n term evaluates goals at state n, making this horizon-dependent.
// The planner gates this with an assumption literal.
// ============================================================================

Term AbstractSuffixEncoder::encode_abstract_goal(int n) {
    TermVec goal_constraints;

    for (const auto& goal : problem_.goals()) {
        Term g_at_n = encoder_.convert_expr_id_to_smt(goal.goal_id(), n);
        TermVec mods = get_mod_vars_for_expr(goal.goal_id());

        if (mods.empty()) {
            goal_constraints.push_back(g_at_n);
        } else {
            TermVec disj;
            disj.push_back(g_at_n);
            for (size_t i = 0; i < mods.size(); ++i) {
                disj.push_back(mods[i]);
            }
            goal_constraints.push_back(tb_.mk_or(disj));
        }
    }

    return goal_constraints.empty() ? tb_.bool_val(true) : tb_.mk_and(goal_constraints);
}

// ============================================================================
// Cost and solution queries
// ============================================================================

Term AbstractSuffixEncoder::build_total_cost(int n) {
    Term total = encoder_.get_variable_factory().make_zero();
    for (int i = 0; i < n; ++i) {
        total = tb_.add(total, build_concrete_cost_term(i));
    }
    total = tb_.add(total, build_abstract_cost_term());
    return total;
}

bool AbstractSuffixEncoder::is_concrete_solution(const cvc5::Solver& solver) const {
    for (const auto& a_abs : action_abs_) {
        Term val = solver.getValue(a_abs);
        if (val.isBooleanValue() && val.getBooleanValue()) return false;
    }
    return true;
}

// ============================================================================
// Cost terms
//
// Concrete cost: for each action at each step, ITE(action_active, cost, 0).
// Every action has a cost_id (defaulting to 1 if no explicit cost was defined
// in the problem). Cost evaluation goes through convert_cost_to_smt(), which
// evaluates at the start-of-timestep state x_t.
//
// Abstract cost: for each abstract action, uses the action's actual cost as
// the lower bound. For constant costs this is exact; for SDAC, externally
// provided cost_lower_bounds_ (computed from NumericRelaxedPlanningGraph
// interval analysis at fixpoint) are used. This is a sound lower bound: the
// RPG fixpoint bounds over-approximate all reachable fluent values, so
// min(cost_expr) over those bounds is ≤ the true minimum cost in any
// reachable state.
// ============================================================================

Term AbstractSuffixEncoder::build_concrete_cost_term(int i) {
    auto& vf = encoder_.get_variable_factory();
    Term zero = vf.make_zero();
    Term cost = zero;

    for (const auto& action : problem_.actions()) {
        const Term& a_var = vf.get_action_variable(action, i);
        Term c_a = encoder_.convert_cost_to_smt(action, i);
        cost = tb_.add(cost, tb_.mk_ite(a_var, c_a, zero));
    }
    return cost;
}

Term AbstractSuffixEncoder::build_abstract_cost_term() {
    auto& vf = encoder_.get_variable_factory();
    Term zero = vf.make_zero();
    Term cost = zero;
    const auto& actions = problem_.actions();
    for (size_t i = 0; i < action_abs_.size(); ++i) {
        // For constant costs: exact via the cost expression.
        // For SDAC: use externally provided lower bound from RPG interval analysis.
        Term c_a = (!cost_lower_bounds_.empty())
            ? vf.make_numeric_val(cost_lower_bounds_[i])
            : encoder_.convert_expr_id_to_smt(actions[i].cost_id(), 0);
        cost = tb_.add(cost, tb_.mk_ite(action_abs_[i], c_a, zero));
    }
    return cost;
}

} // namespace rantanplan
