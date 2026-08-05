#include "frame_axiom_module.hpp"
#include "../../../util/stats.hpp"
#include "../../../encoders/grounded_encoder.hpp"
#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace rantanplan {

// ============================================================================
// Initialization — build the timeless layer once
// ============================================================================

void FrameAxiomModule::initialize(PropagatorSharedState& shared,
                                    PropagatorStrategy& host) {
    PropagatorModule::initialize(shared, host);
    grounded_ = dynamic_cast<GroundedEncoder*>(shared.encoder);
    if (grounded_) build_clause_shapes();
}

void FrameAxiomModule::emit_clause_shape(
        ExprID fluent_id,
        const std::vector<std::pair<const Action*, const EffectExpression*>>& effects,
        std::unordered_map<int, uint32_t>& literal_of_action,
        std::unordered_map<ExprID, uint32_t>& position_of_condition) {

    ClauseShape shape;
    shape.fluent_id       = fluent_id;
    shape.first_explainer = static_cast<uint32_t>(explainers_.size());
    shape.num_explainers  = static_cast<uint32_t>(effects.size());

    for (const auto& [action, effect] : effects) {
        Explainer x;

        auto [a_it, a_new] = literal_of_action.emplace(
            action->id(), static_cast<uint32_t>(literal_actions_.size()));
        if (a_new) literal_actions_.push_back(action);
        x.action_literal = a_it->second;

        if (effect->is_conditional()) {
            auto [c_it, c_new] = position_of_condition.emplace(
                effect->condition_id(),
                static_cast<uint32_t>(literal_conditions_.size()));
            if (c_new) literal_conditions_.push_back(effect->condition_id());
            // Stored as a POSITION for now; shifted into the shared literal
            // namespace once the action count is final (see below).
            x.cond_literal = c_it->second;
        } else {
            x.cond_literal = kUnconditional;
        }

        explainers_.push_back(x);
    }

    clause_shapes_.push_back(shape);
}

void FrameAxiomModule::build_clause_shapes() {
    const auto& epc_index = grounded_->get_epc_index();
    const Problem& problem = *shared_->problem;

    std::unordered_map<int, uint32_t>    literal_of_action;
    std::unordered_map<ExprID, uint32_t> position_of_condition;

    auto modifiers_of = [&](ExprID eid)
        -> const std::vector<std::pair<const Action*, const EffectExpression*>>* {
        auto it = epc_index.find(eid);
        if (it == epc_index.end() || it->second.empty()) return nullptr;
        return &it->second;
    };

    // Two passes so that boolean shapes occupy [0, num_boolean_shapes_) and
    // numeric ones follow.  That ordering is what lets "shape < B" replace an
    // is_boolean flag and a pair of head-index lookup tables.
    for (ExprID eid : problem.grounded_fluents()) {
        const auto* effects = modifiers_of(eid);
        if (!effects) {
            // No action can ever touch this fluent: it is never a clause, it
            // gets an eagerly asserted persistence chain per transition.
            unmodifiable_fluents_.push_back(eid);
            continue;
        }
        if (problem.is_bool_type(eid))
            emit_clause_shape(eid, *effects, literal_of_action, position_of_condition);
    }
    num_boolean_shapes_ = static_cast<uint32_t>(clause_shapes_.size());

    for (ExprID eid : problem.grounded_fluents()) {
        const auto* effects = modifiers_of(eid);
        if (!effects || problem.is_bool_type(eid)) continue;
        emit_clause_shape(eid, *effects, literal_of_action, position_of_condition);
    }
    num_numeric_shapes_ =
        static_cast<uint32_t>(clause_shapes_.size()) - num_boolean_shapes_;

    // Conditions share one namespace with actions, sitting above them, so
    // that a single occurrence index and a single "which literal is this"
    // comparison cover both.
    num_action_literals_    = static_cast<uint32_t>(literal_actions_.size());
    num_condition_literals_ = static_cast<uint32_t>(literal_conditions_.size());
    num_literals_           = num_action_literals_ + num_condition_literals_;
    for (Explainer& x : explainers_)
        if (x.cond_literal != kUnconditional) x.cond_literal += num_action_literals_;

    slots_per_timestep_ = num_boolean_shapes_ + num_literals_ + num_numeric_shapes_;

    // Occurrence index: count, prefix-sum, fill.  Each explainer is listed
    // under its action literal and, if conditional, under its condition
    // literal as well.
    occurrences_.bucket_start.assign(num_literals_ + 1, 0);
    for (const Explainer& x : explainers_) {
        occurrences_.bucket_start[x.action_literal + 1]++;
        if (x.cond_literal != kUnconditional)
            occurrences_.bucket_start[x.cond_literal + 1]++;
    }
    for (uint32_t l = 1; l <= num_literals_; ++l)
        occurrences_.bucket_start[l] += occurrences_.bucket_start[l - 1];

    occurrences_.refs.resize(occurrences_.bucket_start[num_literals_]);
    std::vector<uint32_t> cursor(occurrences_.bucket_start.begin(),
                                 occurrences_.bucket_start.end() - 1);
    for (uint32_t k = 0; k < clause_shapes_.size(); ++k) {
        const ClauseShape& shape = clause_shapes_[k];
        for (uint32_t e = shape.first_explainer;
             e < shape.first_explainer + shape.num_explainers; ++e) {
            const Explainer& x = explainers_[e];
            occurrences_.refs[cursor[x.action_literal]++] = ExplainerRef{k, e};
            if (x.cond_literal != kUnconditional)
                occurrences_.refs[cursor[x.cond_literal]++] = ExplainerRef{k, e};
        }
    }

    // uint16 counters bound the width of a single clause.
    for (const ClauseShape& shape : clause_shapes_) {
        if (shape.num_explainers > 0xFFFFu)
            throw std::logic_error(
                "FrameAxiom: a fluent has more than 65535 modifiers; "
                "widen ClauseCounters");
    }
}

// ============================================================================
// Registration
// ============================================================================

void FrameAxiomModule::register_timestep_variables(int timestep) {
    if (timestep > 0) register_transition(timestep - 1);
}

void FrameAxiomModule::bind_slot(uint32_t slot, const z3::expr& e) {
    slot_expr_[slot] = e;
    const unsigned id = e.id();
    if (id >= slot_of_ast_.size())
        slot_of_ast_.resize(static_cast<size_t>(id) + 1, kNoSlot);
    slot_of_ast_[id] = slot;
    host_->module_add(e);
    frame_vars_registered_++;
}

void FrameAxiomModule::ensure_timestep_block(int t) {
    const size_t want = static_cast<size_t>(t + 1) * slots_per_timestep_;
    if (slot_value_.size() >= want) return;

    // Contiguity means blocks are only ever appended one at a time.
    assert(slot_value_.size() == static_cast<size_t>(t) * slots_per_timestep_);

    // Literal and equality-bit slots start as placeholders.  They become live
    // when their transition is registered; until then they are unreachable —
    // every reader touches either a boolean head or a registered transition's
    // block — and their values stay -1, which already reads as "unknown".
    const z3::expr placeholder = host_->z3_ctx().bool_val(true);
    slot_value_.resize(want, -1);
    slot_expr_.resize(want, placeholder);

    for (uint32_t k = 0; k < num_boolean_shapes_; ++k)
        bind_slot(boolean_head_slot(k, t),
                  grounded_->convert_expr_id_to_z3(clause_shapes_[k].fluent_id, t));
}

void FrameAxiomModule::activate_transition_slots(int t) {
    const Z3VariableFactory& vf = *shared_->variable_factory;
    z3::context& ctx = host_->z3_ctx();

    for (uint32_t a = 0; a < num_action_literals_; ++a)
        bind_slot(literal_slot(a, t),
                  vf.get_action_variable(*literal_actions_[a], t));

    // One reified bit per distinct condition per timestep, shared by every
    // explainer guarded by that condition.
    for (uint32_t c = 0; c < num_condition_literals_; ++c) {
        const uint32_t literal = num_action_literals_ + c;
        const std::string name =
            "fc_" + std::to_string(literal) + "_" + std::to_string(t);
        z3::expr reified = ctx.bool_const(name.c_str());
        shared_->solver->add(
            reified == grounded_->convert_expr_id_to_z3(literal_conditions_[c], t));
        bind_slot(literal_slot(literal, t), reified);
    }

    // Numeric fluents are watched through a reified equality bit: the
    // propagator can only observe booleans.
    for (uint32_t k = num_boolean_shapes_;
         k < clause_shapes_.size(); ++k) {
        const ExprID fluent = clause_shapes_[k].fluent_id;
        z3::expr f_t  = grounded_->convert_expr_id_to_z3(fluent, t);
        z3::expr f_t1 = grounded_->convert_expr_id_to_z3(fluent, t + 1);
        const std::string name =
            "feq_" + std::to_string(k) + "_" + std::to_string(t);
        z3::expr reified = ctx.bool_const(name.c_str());
        shared_->solver->add(reified == (f_t == f_t1));
        bind_slot(equality_bit_slot(k, t), reified);
    }
}

void FrameAxiomModule::register_transition(int t) {
    if (!grounded_) return;
    if (t < num_transitions_) return;   // idempotent re-registration
    if (t != num_transitions_)
        throw std::logic_error(
            "FrameAxiom requires contiguous transition registration from 0 "
            "(got t=" + std::to_string(t) + " with " +
            std::to_string(num_transitions_) + " already registered). "
            "The slot layout is stride arithmetic; a gap would fabricate "
            "phantom blocks. StrategyFactory::validate rejects this module "
            "with the CGB and DoubleTail planners for exactly this reason.");

    // f^t and f^{t+1} are both needed.  The latter is also f^t of transition
    // t+1 and shares its slot, so the tail block is created ahead of time.
    ensure_timestep_block(t);
    ensure_timestep_block(t + 1);
    activate_transition_slots(t);

    const size_t num_shapes = clause_shapes_.size();
    clause_counters_.resize(static_cast<size_t>(t + 1) * num_shapes);

    // F_eager accounting: encode_frames() short-circuits under lazy frames, so
    // the size of the encoding this module replaces is tallied here instead.
    // Every transition registers every shape, and explainers_ is the gapless
    // concatenation of all shapes' slices, so this is exact — no loop needed.
    frame_eager_axioms_   += static_cast<long long>(clause_shapes_.size());
    frame_eager_literals_ += static_cast<long long>(explainers_.size());

    for (ExprID eid : unmodifiable_fluents_) {
        z3::expr f_t  = grounded_->convert_expr_id_to_z3(eid, t);
        z3::expr f_t1 = grounded_->convert_expr_id_to_z3(eid, t + 1);
        shared_->solver->add(f_t1 == f_t);
        frame_static_persistence_++;
    }

    num_transitions_ = t + 1;
}

// ============================================================================
// Push / pop
// ============================================================================

void FrameAxiomModule::on_push() {
    level_marks_.push_back(static_cast<uint32_t>(fixed_slots_.size()));
}

void FrameAxiomModule::on_pop(unsigned num_scopes) {
    unsigned effective = std::min(num_scopes,
                                  static_cast<unsigned>(level_marks_.size()));
    assert(effective == num_scopes && "pop without matching push");
    if (effective == 0) return;

    const uint32_t mark = level_marks_[level_marks_.size() - effective];
    level_marks_.resize(level_marks_.size() - effective);

    // Reverse-delta replay.  Walking backwards means every slot fixed after
    // this one is already restored, so the "other" value each counter update
    // reads is exactly what it was when this slot was fixed — the inverse
    // cancels the forward step precisely.  Nothing derived needs repairing.
    while (fixed_slots_.size() > mark) {
        const uint32_t slot = fixed_slots_.back();
        fixed_slots_.pop_back();

        const int8_t old_value = slot_value_[slot];
        slot_value_[slot] = -1;

        const uint32_t r = slot % slots_per_timestep_;
        if (r >= num_boolean_shapes_ && r < num_boolean_shapes_ + num_literals_) {
            const int t = static_cast<int>(slot / slots_per_timestep_);
            visit_literal(r - num_boolean_shapes_, t, old_value, -1,
                          /*run_checks=*/false);
        }
        // Boolean heads and equality bits carry no counters: resetting the
        // value is the whole undo, because persistence is derived on read.
    }
}

// ============================================================================
// on_fixed
// ============================================================================

void FrameAxiomModule::update_counters(uint32_t instance, const Explainer& x,
                                         int8_t old_action, int8_t new_action,
                                         int8_t old_cond,   int8_t new_cond) {
    const bool conditional = (x.cond_literal != kUnconditional);

    const bool was_can  = can_explain(conditional, old_action, old_cond);
    const bool was_dead = ruled_out(conditional, old_action, old_cond);
    const bool now_can  = can_explain(conditional, new_action, new_cond);
    const bool now_dead = ruled_out(conditional, new_action, new_cond);

    ClauseCounters& counters = clause_counters_[instance];
    if (now_dead != was_dead) counters.num_ruled_out   += now_dead ? 1 : -1;
    if (now_can  != was_can)  counters.num_can_explain += now_can  ? 1 : -1;
}

void FrameAxiomModule::visit_literal(uint32_t literal, int t,
                                       int8_t old_value, int8_t new_value,
                                       bool run_checks) {
    const uint32_t lo = occurrences_.bucket_start[literal];
    const uint32_t hi = occurrences_.bucket_start[literal + 1];

    for (uint32_t i = lo; i < hi; ++i) {
        const ExplainerRef& ref = occurrences_.refs[i];
        const Explainer&    x   = explainers_[ref.explainer];
        const uint32_t      instance = clause_instance(ref.shape, t);

        if (literal == x.action_literal) {
            const int8_t cond = condition_value(x, t);
            update_counters(instance, x, old_value, new_value, cond, cond);
        } else {
            const int8_t action = action_value(x, t);
            update_counters(instance, x, action, action, old_value, new_value);
        }

        // ATOMICITY: the loop must finish updating counters for the whole
        // occurrence list even after a conflict, or the reverse replay in
        // on_pop would undo updates that were never applied.  Only the check
        // is suppressed.
        if (run_checks && !host_->conflict_raised())
            check_clause(ref.shape, t);
    }
}

void FrameAxiomModule::on_fixed(const z3::expr& ast, const z3::expr& value) {
    // Timed from the top: the slot lookup runs for every variable Z3 fixes,
    // watched or not, and is part of the module's overhead.
    const unsigned ast_id = ast.id();
    if (ast_id >= slot_of_ast_.size()) return;
    const uint32_t slot = slot_of_ast_[ast_id];
    if (slot == kNoSlot) return;

    frame_on_fixed_count_++;

    const int8_t   new_value = value.is_true() ? 1 : 0;
    const int8_t   old_value = slot_value_[slot];
    slot_value_[slot] = new_value;

    // Below the first push nothing can be undone, so base-level assignments
    // are permanent and deliberately not trailed.
    if (!level_marks_.empty()) {
        fixed_slots_.push_back(slot);
        fixed_slots_peak_ = std::max(fixed_slots_peak_, fixed_slots_.size());
    }

    const int      t = static_cast<int>(slot / slots_per_timestep_);
    const uint32_t r = slot % slots_per_timestep_;

    if (r < num_boolean_shapes_) {
        // A boolean head is the "after" side of transition t-1 and the
        // "before" side of transition t.  Contiguous registration means t-1
        // was registered first, so this order is registration order.
        const uint32_t shape = r;
        if (is_registered(t - 1)) {
            check_clause(shape, t - 1);
            if (host_->conflict_raised()) return;
        }
        if (is_registered(t)) check_clause(shape, t);

    } else if (r < num_boolean_shapes_ + num_literals_) {
        visit_literal(r - num_boolean_shapes_, t, old_value, new_value,
                      /*run_checks=*/true);

    } else {
        const uint32_t shape =
            num_boolean_shapes_ + (r - num_boolean_shapes_ - num_literals_);
        check_clause(shape, t);
    }
}

// ============================================================================
// Frame axiom core logic
// ============================================================================

int8_t FrameAxiomModule::persistence_state(uint32_t shape, int t) const {
    if (!is_boolean_shape(shape))
        return slot_value_[equality_bit_slot(shape, t)];

    const int8_t before = slot_value_[boolean_head_slot(shape, t)];
    const int8_t after  = slot_value_[boolean_head_slot(shape, t + 1)];
    if (before < 0 || after < 0) return -1;
    return before == after ? 1 : 0;
}

void FrameAxiomModule::check_clause(uint32_t shape, int t) {
    const ClauseShape&    s        = clause_shapes_[shape];
    const ClauseCounters& counters = clause_counters_[clause_instance(shape, t)];
    const int             n        = static_cast<int>(s.num_explainers);

    const int8_t persisted = persistence_state(shape, t);

    if (persisted == 1) return;                 // fluent unchanged: axiom vacuous
    if (counters.num_can_explain > 0) return;   // some explainer already fires
    // After this point: num_can_explain == 0.

    if (counters.num_ruled_out == n) {
        if (persisted == 0) report_conflict(shape, t);
        else                propagate_persistence(shape, t);
        return;
    }

    if (persisted == 0 && counters.num_ruled_out == n - 1)
        propagate_last_explainer(shape, t);
}

// ============================================================================
// Conflict & propagation
// ============================================================================

void FrameAxiomModule::build_reason(z3::expr_vector& reason,
                                      uint32_t shape, int t,
                                      bool include_change, int64_t skip_explainer) {
    const ClauseShape& s = clause_shapes_[shape];

    if (include_change) {
        if (is_boolean_shape(shape)) {
            reason.push_back(slot_expr_[boolean_head_slot(shape, t)]);
            reason.push_back(slot_expr_[boolean_head_slot(shape, t + 1)]);
        } else {
            reason.push_back(slot_expr_[equality_bit_slot(shape, t)]);
        }
    }

    for (uint32_t e = s.first_explainer;
         e < s.first_explainer + s.num_explainers; ++e) {
        if (static_cast<int64_t>(e) == skip_explainer) continue;
        const Explainer& x = explainers_[e];

        if (action_value(x, t) == 0) {
            reason.push_back(slot_expr_[literal_slot(x.action_literal, t)]);
        } else if (x.cond_literal != kUnconditional && condition_value(x, t) == 0) {
            // The condition alone rules out this explainer — the action's
            // value is irrelevant.  Omitting it gives a tighter reason that
            // matches one of the eager encoding's CNF clauses.
            reason.push_back(slot_expr_[literal_slot(x.cond_literal, t)]);
        }
    }
}

void FrameAxiomModule::report_conflict(uint32_t shape, int t) {
    z3::expr_vector reason(host_->z3_ctx());
    build_reason(reason, shape, t, /*include_change=*/true, /*skip=*/-1);
    frame_conflict_count_++;
    host_->module_conflict(reason);
}

bool FrameAxiomModule::propagate_persistence(uint32_t shape, int t) {
    if (!is_boolean_shape(shape)) {
        z3::expr_vector reason(host_->z3_ctx());
        build_reason(reason, shape, t, /*include_change=*/false, /*skip=*/-1);
        frame_persistence_prop_count_++;
        host_->module_propagate(reason, slot_expr_[equality_bit_slot(shape, t)]);
        return true;
    }

    // A boolean persistence consequence used to be posted as `after == before`.
    // That expression is BOOLEAN-SORTED, so Z3 internalizes it as an iff node:
    // one fresh boolean variable plus four gate clauses, manufactured in the
    // middle of the search at a non-base decision level, with the clause-reinit
    // churn that implies.  It was the module's only mid-search atom creation.
    //
    // Instead, resolve the equality here against whichever head is already
    // known and propagate the resulting DIRECTED LITERAL, which is an atom the
    // formula already contains:
    //
    //     killers ... , f^t      |-   f^{t+1}   (or its negation)
    //     killers ... , f^{t+1}  |-   f^t       (or its negation)
    //
    // When NEITHER head is assigned there is no literal to force and we emit
    // nothing.  No inference is lost: both heads are watched slots, and fixing
    // either one re-enters check_clause for this transition through the boolean
    // head branch of on_fixed, at which point this function runs again with one
    // head known.  on_final is the backstop, as before.
    const uint32_t before_slot = boolean_head_slot(shape, t);
    const uint32_t after_slot  = boolean_head_slot(shape, t + 1);
    const int8_t   before      = slot_value_[before_slot];
    const int8_t   after       = slot_value_[after_slot];

    uint32_t known_slot, target_slot;
    int8_t   known_value;
    if (before >= 0) {
        known_slot = before_slot; target_slot = after_slot;  known_value = before;
    } else if (after >= 0) {
        known_slot = after_slot;  target_slot = before_slot; known_value = after;
    } else {
        return false;   // both heads unknown: nothing to force yet
    }

    z3::expr_vector reason(host_->z3_ctx());
    build_reason(reason, shape, t, /*include_change=*/false, /*skip=*/-1);
    reason.push_back(slot_expr_[known_slot]);

    frame_persistence_prop_count_++;

    const z3::expr& target = slot_expr_[target_slot];
    host_->module_propagate(reason, known_value == 1 ? target : !target);
    return true;
}

void FrameAxiomModule::propagate_last_explainer(uint32_t shape, int t) {
    const ClauseShape& s = clause_shapes_[shape];

    // The call site guarantees num_can_explain == 0 and num_ruled_out == n-1,
    // so exactly one explainer is pending.  The not-found return is defensive:
    // without it, a caller that skipped the num_can_explain guard would emit a
    // lemma about an arbitrary explainer.
    bool     found = false;
    uint32_t index = 0;
    for (uint32_t e = s.first_explainer;
         e < s.first_explainer + s.num_explainers; ++e) {
        const Explainer& x = explainers_[e];
        const bool   conditional = (x.cond_literal != kUnconditional);
        const int8_t action      = action_value(x, t);
        const int8_t cond        = condition_value(x, t);
        if (!ruled_out(conditional, action, cond) &&
            !can_explain(conditional, action, cond)) {
            found = true;
            index = e;
            break;
        }
    }
    if (!found) return;

    const Explainer& x      = explainers_[index];
    const int8_t     action = action_value(x, t);
    const int8_t     cond   = condition_value(x, t);
    const uint32_t   instance = clause_instance(shape, t);

    z3::expr_vector reason(host_->z3_ctx());
    build_reason(reason, shape, t, /*include_change=*/true,
                 static_cast<int64_t>(index));

    if (action == -1) {
        frame_unit_prop_count_++;
        host_->module_propagate(reason,
                                slot_expr_[literal_slot(x.action_literal, t)]);
    } else if (x.cond_literal != kUnconditional && action == 1 && cond == -1) {
        reason.push_back(slot_expr_[literal_slot(x.action_literal, t)]);
        frame_unit_prop_count_++;
        host_->module_propagate(reason,
                                slot_expr_[literal_slot(x.cond_literal, t)]);
    }
}

// ============================================================================
// on_final — the completeness net
// ============================================================================

void FrameAxiomModule::on_final() {
    frame_on_final_count_++;

    // Note the missing `num_ruled_out == n` test, which check_clause() does
    // require.  It is redundant here, not forgotten: Z3 calls final() only
    // once every registered atom is fixed, so no explainer can still be
    // pending, and the invariant
    //     num_can_explain + num_ruled_out + #pending == n
    // collapses to num_ruled_out == n as soon as the num_can_explain guard
    // below has passed.  
    for (int t = 0; t < num_transitions_; ++t) {
        for (uint32_t shape = 0; shape < clause_shapes_.size(); ++shape) {
            const int8_t persisted = persistence_state(shape, t);
            if (persisted == 1) continue;
            if (clause_counters_[clause_instance(shape, t)].num_can_explain > 0)
                continue;

            if (persisted == 0) {
                frame_final_conflict_count_++;
                report_conflict(shape, t);
                return;
            }
            if (propagate_persistence(shape, t)) {
                frame_final_propagation_count_++;
                return;
            }
            // Both boolean heads unassigned even here: there is no literal to
            // force on this clause, so keep scanning instead of returning
            // empty-handed.  The axiom is not violated -- an unassigned head
            // means the solver has not committed to a change.
        }
    }
}

// ============================================================================
// Cleanup — the frame_prop.* statistics
// ============================================================================

void FrameAxiomModule::cleanup() {
    auto& stats = Stats::instance();

    const int propagations = frame_persistence_prop_count_ + frame_unit_prop_count_;
    const int final_violations =
        frame_final_conflict_count_ + frame_final_propagation_count_;

    stats.set("frame_prop.clauses_created", static_cast<double>(clause_counters_.size()));
    stats.set("frame_prop.conflicts", static_cast<double>(frame_conflict_count_));
    stats.set("frame_prop.propagations", static_cast<double>(propagations));
    stats.set("frame_prop.persistence_propagations",
              static_cast<double>(frame_persistence_prop_count_));
    stats.set("frame_prop.unit_propagations",
              static_cast<double>(frame_unit_prop_count_));
    stats.set("frame_prop.on_fixed_calls", static_cast<double>(frame_on_fixed_count_));
    stats.set("frame_prop.on_final_calls", static_cast<double>(frame_on_final_count_));
    stats.set("frame_prop.on_final_violations", static_cast<double>(final_violations));
    stats.set("frame_prop.final_conflicts",
              static_cast<double>(frame_final_conflict_count_));
    stats.set("frame_prop.final_propagations",
              static_cast<double>(frame_final_propagation_count_));
    stats.set("frame_prop.vars_registered", static_cast<double>(frame_vars_registered_));
    stats.set("frame_prop.n_boolean_shapes", static_cast<double>(num_boolean_shapes_));
    stats.set("frame_prop.n_literals", static_cast<double>(num_literals_));
    stats.set("frame_prop.n_numeric_shapes", static_cast<double>(num_numeric_shapes_));

    const double f_eager = static_cast<double>(frame_eager_axioms_);
    const double f_lazy_emissions =
        static_cast<double>(propagations + frame_conflict_count_);
    stats.set("frame_prop.f_eager_axioms", f_eager);
    stats.set("frame_prop.f_eager_literals", static_cast<double>(frame_eager_literals_));
    stats.set("frame_prop.static_persistence", static_cast<double>(frame_static_persistence_));
    stats.set("frame_prop.f_lazy_emissions", f_lazy_emissions);
    if (f_eager > 0.0) {
        stats.set("frame_prop.emissions_over_f_eager", f_lazy_emissions / f_eager);
    }

    // Layout counters, so a run can be attributed without a profiler.
    stats.set("frame_prop.clause_shapes", static_cast<double>(clause_shapes_.size()));
    stats.set("frame_prop.explainers", static_cast<double>(explainers_.size()));
    stats.set("frame_prop.occurrence_refs",
              static_cast<double>(occurrences_.refs.size()));
    stats.set("frame_prop.slots_per_timestep",
              static_cast<double>(slots_per_timestep_));
    stats.set("frame_prop.slots", static_cast<double>(slot_value_.size()));
    stats.set("frame_prop.trail_peak", static_cast<double>(fixed_slots_peak_));
}

} // namespace rantanplan
