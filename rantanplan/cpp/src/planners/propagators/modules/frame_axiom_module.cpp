#include "frame_axiom_module.hpp"
#include "../../../config/config.hpp"
#include "../../../util/scoped_timer.hpp"
#include "../../../util/stats.hpp"
#include "../../../encoders/grounded_encoder.hpp"
#include <algorithm>
#include <cassert>

namespace rantanplan {

// ============================================================================
// Push / Pop (private frame trail only)
// ============================================================================

void FrameAxiomModule::on_push() {
    frame_decision_levels_.push_back(frame_trail_.size());
}

void FrameAxiomModule::on_pop(unsigned num_scopes) {
    AccumulatingTimer timer(frame_callback_ms_);
    unsigned effective = std::min(num_scopes,
                                  static_cast<unsigned>(frame_decision_levels_.size()));
    assert(effective == num_scopes && "pop without matching push");
    if (effective == 0) return;

    size_t frame_level_start =
        frame_decision_levels_[frame_decision_levels_.size() - effective];
    frame_decision_levels_.resize(frame_decision_levels_.size() - effective);

    std::vector<uint32_t> dirty_clauses;

    while (frame_trail_.size() > frame_level_start) {
        const auto& te = frame_trail_.back();
        auto& clause = frame_clauses_[te.clause_idx];

        switch (te.kind) {
        case VarRole::FLUENT_T:
            clause.ft_val = te.prev_state; break;
        case VarRole::FLUENT_T1:
            clause.ft1_val = te.prev_state; break;
        case VarRole::EQ_BOOL:
            clause.eq_state = te.prev_state; break;
        case VarRole::ACTION:
            clause.entries[te.entry_idx].action_state = te.prev_state; break;
        case VarRole::CONDITION:
            clause.entries[te.entry_idx].cond_state = te.prev_state; break;
        }
        dirty_clauses.push_back(te.clause_idx);
        frame_trail_.pop_back();
    }

    std::sort(dirty_clauses.begin(), dirty_clauses.end());
    dirty_clauses.erase(
        std::unique(dirty_clauses.begin(), dirty_clauses.end()),
        dirty_clauses.end());
    for (uint32_t ci : dirty_clauses)
        recompute_derived(frame_clauses_[ci]);
}

// ============================================================================
// on_fixed
// ============================================================================

void FrameAxiomModule::on_fixed(const z3::expr& ast, const z3::expr& value) {
    // Timed from the top: the role lookup runs for every variable Z3 fixes,
    // watched or not, and is part of the module's overhead.
    AccumulatingTimer timer(frame_callback_ms_);

    auto it = frame_var_to_roles_.find(ast.id());
    if (it == frame_var_to_roles_.end()) return;

    frame_on_fixed_count_++;
    const int8_t val_int = value.is_true() ? 1 : 0;

    for (const VarRole& role : it->second) {
        auto& clause = frame_clauses_[role.clause_idx];

        FrameTrailEntry te;
        te.clause_idx = static_cast<uint32_t>(role.clause_idx);
        te.kind = role.kind;
        te.entry_idx = static_cast<uint32_t>(role.entry_idx);

        switch (role.kind) {
        case VarRole::FLUENT_T: {
            te.prev_state = clause.ft_val;
            clause.ft_val = val_int;
            if (clause.is_boolean && clause.ft1_val >= 0)
                clause.eq_state = (clause.ft_val == clause.ft1_val) ? 1 : 0;
            break;
        }
        case VarRole::FLUENT_T1: {
            te.prev_state = clause.ft1_val;
            clause.ft1_val = val_int;
            if (clause.is_boolean && clause.ft_val >= 0)
                clause.eq_state = (clause.ft_val == clause.ft1_val) ? 1 : 0;
            break;
        }
        case VarRole::EQ_BOOL: {
            te.prev_state = clause.eq_state;
            clause.eq_state = val_int;
            break;
        }
        case VarRole::ACTION: {
            auto& entry = clause.entries[role.entry_idx];
            te.prev_state = entry.action_state;
            update_epc_entry(clause, entry, entry.action_state, val_int);
            break;
        }
        case VarRole::CONDITION: {
            auto& entry = clause.entries[role.entry_idx];
            te.prev_state = entry.cond_state;
            update_epc_entry(clause, entry, entry.cond_state, val_int);
            break;
        }
        }

        frame_trail_.push_back(te);

        if (host_->conflict_raised()) return;
        check_frame_clause(clause, role.clause_idx);
        if (host_->conflict_raised()) return;
    }
}

// ============================================================================
// on_final
// ============================================================================

void FrameAxiomModule::on_final() {
    AccumulatingTimer timer(frame_callback_ms_);
    frame_on_final_count_++;

    for (size_t ci = 0; ci < frame_clauses_.size(); ++ci) {
        auto& clause = frame_clauses_[ci];

        if (clause.eq_state == 1) continue;
        if (clause.owned()) continue;
        // After owned() check: num_can_explain == 0

        if (clause.eq_state == 0) {
            frame_final_conflict_count_++;
            report_frame_conflict(clause, ci);
            return;
        }

        if (clause.eq_state == -1) {
            frame_final_propagation_count_++;
            propagate_fluent_preservation(clause, ci);
            return;
        }
    }
}

// ============================================================================
// Frame axiom core logic
// ============================================================================

void FrameAxiomModule::check_frame_clause(FrameClause& clause, size_t clause_idx) {
    int n = static_cast<int>(clause.entries.size());

    if (clause.eq_state == 1) return;
    if (clause.owned()) return;
    // After owned() check: num_can_explain == 0

    if (clause.num_cant_explain == n) {
        if (clause.eq_state == 0) {
            report_frame_conflict(clause, clause_idx);
        } else {
            propagate_fluent_preservation(clause, clause_idx);
        }
        return;
    }

    if (clause.eq_state == 0 && clause.num_cant_explain == n - 1) {
        propagate_last_entry(clause, clause_idx);
    }
}

// ============================================================================
// Conflict & propagation
// ============================================================================

void FrameAxiomModule::mark_materialized(size_t clause_idx) {
    auto& clause = frame_clauses_[clause_idx];
    if (clause.materialized) return;
    clause.materialized = true;
    frame_clauses_materialized_++;
}

void FrameAxiomModule::build_frame_fixed(
        z3::expr_vector& fixed,
        const FrameClause& clause, size_t clause_idx,
        bool include_change, int skip_idx) {
    if (include_change) {
        if (clause.is_boolean) {
            fixed.push_back(frame_fluent_ft_[clause_idx]);
            fixed.push_back(frame_fluent_ft1_[clause_idx]);
        } else {
            fixed.push_back(frame_eq_bool_[clause_idx]);
        }
    }

    const auto& actions = frame_action_expr_[clause_idx];
    const auto& conds = frame_cond_expr_[clause_idx];

    for (size_t i = 0; i < clause.entries.size(); ++i) {
        if (static_cast<int>(i) == skip_idx) continue;
        const auto& entry = clause.entries[i];
        if (entry.action_state == 0) {
            fixed.push_back(actions[i]);
        } else if (entry.is_conditional && entry.cond_state == 0) {
            // The condition alone rules out this entry — the action's value
            // is irrelevant (if action is true, the entry still can't explain
            // because the condition is false; if false, it can't explain
            // either).  Omitting the action produces a tighter reason that
            // matches one of the eager encoding's CNF clauses.
            fixed.push_back(conds[i]);
        }
    }
}

void FrameAxiomModule::report_frame_conflict(const FrameClause& clause, size_t clause_idx) {
    z3::expr_vector fixed(host_->z3_ctx());
    build_frame_fixed(fixed, clause, clause_idx, true);
    frame_conflict_count_++;
    mark_materialized(clause_idx);
    host_->module_conflict(fixed);
}

void FrameAxiomModule::propagate_fluent_preservation(const FrameClause& clause, size_t clause_idx) {
    z3::expr_vector fixed(host_->z3_ctx());
    build_frame_fixed(fixed, clause, clause_idx, false);

    frame_persistence_prop_count_++;
    mark_materialized(clause_idx);
    if (clause.is_boolean) {
        host_->module_propagate(fixed, frame_fluent_ft1_[clause_idx] == frame_fluent_ft_[clause_idx]);
    } else {
        host_->module_propagate(fixed, frame_eq_bool_[clause_idx]);
    }
}

void FrameAxiomModule::propagate_last_entry(const FrameClause& clause, size_t clause_idx) {
    size_t idx = 0;
    for (size_t i = 0; i < clause.entries.size(); ++i) {
        if (!clause.entries[i].cant_explain() && !clause.entries[i].can_explain()) {
            idx = i;
            break;
        }
    }
    const auto& entry = clause.entries[idx];

    z3::expr_vector fixed(host_->z3_ctx());
    build_frame_fixed(fixed, clause, clause_idx, true, static_cast<int>(idx));

    if (entry.action_state == -1) {
        frame_unit_prop_count_++;
        mark_materialized(clause_idx);
        host_->module_propagate(fixed, frame_action_expr_[clause_idx][idx]);
    } else if (entry.is_conditional && entry.action_state == 1 && entry.cond_state == -1) {
        fixed.push_back(frame_action_expr_[clause_idx][idx]);
        frame_unit_prop_count_++;
        mark_materialized(clause_idx);
        host_->module_propagate(fixed, frame_cond_expr_[clause_idx][idx]);
    }
}

// ============================================================================
// Variable registration
// ============================================================================

void FrameAxiomModule::register_timestep_variables(int timestep) {
    if (timestep > 0)
        register_frame_variables(timestep - 1);
}

void FrameAxiomModule::register_frame_variables(int t) {
    auto* grounded = dynamic_cast<GroundedEncoder*>(shared_->encoder);
    if (!grounded) return;

    const auto& epc_index = grounded->get_epc_index();
    const Z3VariableFactory& vf = *shared_->variable_factory;
    z3::context& ctx = host_->z3_ctx();

    for (ExprID eid : shared_->problem->grounded_fluents()) {
        auto epc_it = epc_index.find(eid);
        bool has_modifiers = (epc_it != epc_index.end() && !epc_it->second.empty());

        if (!has_modifiers) {
            z3::expr f_t = grounded->convert_expr_id_to_z3(eid, t);
            z3::expr f_t1 = grounded->convert_expr_id_to_z3(eid, t + 1);
            shared_->solver->add(f_t1 == f_t);
            frame_static_persistence_++;
            continue;
        }

        const auto& action_effects = epc_it->second;

        // F_eager accounting: encode_frames() would post exactly one axiom for
        // this (fluent, transition), a disjunction over |action_effects|
        // explainers.  It short-circuits under lazy frames, so count here.
        frame_eager_axioms_++;
        frame_eager_literals_ += static_cast<long long>(action_effects.size());

        bool is_bool = shared_->problem->is_bool_type(eid);
        size_t clause_idx = frame_clauses_.size();

        z3::expr f_t = grounded->convert_expr_id_to_z3(eid, t);
        z3::expr f_t1 = grounded->convert_expr_id_to_z3(eid, t + 1);

        frame_clauses_.emplace_back(t, eid, is_bool);
        auto& clause = frame_clauses_.back();

        frame_fluent_ft_.push_back(f_t);
        frame_fluent_ft1_.push_back(f_t1);

        if (is_bool) {
            frame_eq_bool_.push_back(ctx.bool_val(true));
            host_->module_add(f_t);
            frame_var_to_roles_[f_t.id()].push_back({VarRole::FLUENT_T, clause_idx, 0});
            host_->module_add(f_t1);
            frame_var_to_roles_[f_t1.id()].push_back({VarRole::FLUENT_T1, clause_idx, 0});
            frame_vars_registered_ += 2;
        } else {
            std::string eq_name = "feq_" + std::to_string(clause_idx);
            z3::expr eq_bool = ctx.bool_const(eq_name.c_str());
            shared_->solver->add(eq_bool == (f_t == f_t1));
            frame_eq_bool_.push_back(eq_bool);
            host_->module_add(eq_bool);
            frame_var_to_roles_[eq_bool.id()].push_back({VarRole::EQ_BOOL, clause_idx, 0});
            frame_vars_registered_++;
        }

        frame_action_expr_.emplace_back();
        frame_cond_expr_.emplace_back();
        auto& actions_vec = frame_action_expr_.back();
        auto& conds_vec = frame_cond_expr_.back();

        for (size_t i = 0; i < action_effects.size(); ++i) {
            const auto& [action, eff_expr] = action_effects[i];

            EPCEntry entry;
            entry.is_conditional = eff_expr->is_conditional();

            z3::expr a = vf.get_action_variable(*action, t);
            actions_vec.push_back(a);

            host_->module_add(a);
            frame_var_to_roles_[a.id()].push_back({VarRole::ACTION, clause_idx, i});
            frame_vars_registered_++;

            if (entry.is_conditional) {
                ExprID cond_id = eff_expr->condition_id();
                auto cache_key = std::make_pair(cond_id, t);
                auto cache_it = cond_reification_cache_.find(cache_key);
                z3::expr cond_reified = (cache_it != cond_reification_cache_.end())
                    ? cache_it->second
                    : [&]() {
                        std::string cname = "fc_" + std::to_string(clause_idx) + "_" + std::to_string(i);
                        z3::expr cr = ctx.bool_const(cname.c_str());
                        z3::expr cond_z3 = grounded->convert_expr_id_to_z3(cond_id, t);
                        shared_->solver->add(cr == cond_z3);
                        cond_reification_cache_.emplace(cache_key, cr);
                        return cr;
                    }();
                conds_vec.push_back(cond_reified);
                host_->module_add(cond_reified);
                frame_var_to_roles_[cond_reified.id()].push_back({VarRole::CONDITION, clause_idx, i});
                frame_vars_registered_++;
            } else {
                conds_vec.push_back(ctx.bool_val(true));
            }

            clause.entries.push_back(entry);
        }
    }
}

// ============================================================================
// Recompute derived counters
// ============================================================================

void FrameAxiomModule::update_epc_entry(FrameClause& clause, EPCEntry& entry,
                                        int8_t& field, int8_t new_val) {
    bool was_cant = entry.cant_explain();
    bool was_can = entry.can_explain();
    field = new_val;
    bool now_cant = entry.cant_explain();
    bool now_can = entry.can_explain();
    if (now_cant != was_cant) clause.num_cant_explain += now_cant ? 1 : -1;
    if (now_can != was_can) clause.num_can_explain += now_can ? 1 : -1;
}

void FrameAxiomModule::recompute_derived(FrameClause& clause) {
    if (clause.is_boolean) {
        if (clause.ft_val >= 0 && clause.ft1_val >= 0)
            clause.eq_state = (clause.ft_val == clause.ft1_val) ? 1 : 0;
        else
            clause.eq_state = -1;
    }
    clause.num_cant_explain = 0;
    clause.num_can_explain = 0;
    for (const auto& entry : clause.entries) {
        if (entry.cant_explain()) clause.num_cant_explain++;
        if (entry.can_explain()) clause.num_can_explain++;
    }
}

// ============================================================================
// Cleanup
// ============================================================================

void FrameAxiomModule::cleanup() {
    auto& stats = Stats::instance();

    const int propagations = frame_persistence_prop_count_ + frame_unit_prop_count_;
    const int final_violations =
        frame_final_conflict_count_ + frame_final_propagation_count_;

    stats.set("frame_prop.clauses_created", static_cast<double>(frame_clauses_.size()));
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
    stats.set("frame_prop.callback_ms", frame_callback_ms_);

    // Lazy-vs-eager frame constraint accounting.  F_eager is what the eager
    // encoding would have posted over all registered transitions.  Two
    // readings of F_lazy: distinct frame clauses ever materialized (the
    // measure comparable to the mutex generation ratio, always <= F_eager),
    // and total lemma emissions, which counts repeated visits to the same
    // clause across the search and can far exceed F_eager.
    const double f_eager = static_cast<double>(frame_eager_axioms_);
    const double f_lazy_distinct = static_cast<double>(frame_clauses_materialized_);
    const double f_lazy_emissions =
        static_cast<double>(propagations + frame_conflict_count_);
    stats.set("frame_prop.f_eager_axioms", f_eager);
    stats.set("frame_prop.f_eager_literals", static_cast<double>(frame_eager_literals_));
    stats.set("frame_prop.static_persistence", static_cast<double>(frame_static_persistence_));
    stats.set("frame_prop.clauses_materialized", f_lazy_distinct);
    stats.set("frame_prop.f_lazy", f_lazy_distinct);
    stats.set("frame_prop.f_lazy_emissions", f_lazy_emissions);
    if (f_eager > 0.0) {
        stats.set("frame_prop.f_lazy_over_f_eager", f_lazy_distinct / f_eager);
        stats.set("frame_prop.emissions_over_f_eager", f_lazy_emissions / f_eager);
    }
}

} // namespace rantanplan
