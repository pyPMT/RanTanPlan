#include "smt_variable_factory.hpp"

#include "../problem/problem.hpp"

#include <stdexcept>

namespace rantanplan {

using cvc5::Kind;

SmtVariableFactory::SmtVariableFactory(cvc5::TermManager& tm, const Problem* problem)
    : tm_(tm), builder_(tm, problem), problem_(problem) {}

// ── symbol variables ───────────────────────────────────────────────────────

Term SmtVariableFactory::create_symbol_variable(const std::string& name, const Type* type) {
    // The sort is the factory's decision; the identity is the SymbolTable's.
    // Int is the default (is_int() and unknown shapes); object symbols are
    // encoded as numeric values (each object maps to its global index) and
    // must share the numeric sort, or a single-sort logic (QF_LRA / QF_LIA)
    // becomes inconsistent with the terms in the problem.
    cvc5::Sort sort = tm_.getIntegerSort();
    if (type) {
        if (type->is_bool()) {
            sort = tm_.getBooleanSort();
        } else if (type->is_real() || type->is_object()) {
            sort = (problem_ && problem_->all_integer())
                ? tm_.getIntegerSort() : tm_.getRealSort();
        }
    }
    return symbols_.intern_var(tm_, name, sort);
}

// ── PLAN entity construction ───────────────────────────────────────────────

/// The dense ground-fluent id, from the problem's own registry. This is the
/// exact analogue of Action::id() for fluents -- see problem.hpp:96-100.
uint32_t SmtVariableFactory::ground_fluent_index(const Fluent& fluent,
                                                 ExprID fluent_eid) const {
    if (problem_ && fluent_eid.valid()) {
        const int idx = problem_->find_grounded_fluent_index(fluent_eid);
        if (idx >= 0) return static_cast<uint32_t>(idx);
    }
    // Deliberately fatal rather than falling back to a locally-invented index.
    // A second numbering space would silently collide with the registry's and
    // give two different fluents the same atom.
    throw std::runtime_error(
        "SmtVariableFactory: no ground-fluent id for " + fluent.name()
        + " [eid=" + std::to_string(fluent_eid.id)
        + " valid=" + std::to_string(fluent_eid.valid())
        + " registry_size=" + std::to_string(problem_ ? problem_->grounded_fluent_count() : 0)
        + "]"
        + "; the ExprID must be threaded through from the encoding visitor");
}

Term SmtVariableFactory::create_new_fluent_variable(const Fluent& fluent,
                                                    int timestep,
                                                    uint32_t idx) {
    const uint32_t t = static_cast<uint32_t>(timestep);

    // Role and sort are orthogonal: one kind, the sort carried as a real Sort
    // in the operator payload. That is what lets a fluent later range over a
    // datatype, a set or an array (migration-plan.md M4.5).
    cvc5::Sort sort;
    if (fluent.is_bool_fluent()) {
        sort = tm_.getBooleanSort();
    } else if (fluent.is_int_fluent() || fluent.is_real_fluent()
               || fluent.is_object_fluent()) {
        // Int when the whole problem is proven integer-safe, else Real, to
        // avoid mixed Int/Real terms. Object fluents follow the numeric sort --
        // each object encoded as its global index. M4.5 phase B replaces this
        // with a real finite sort; see the log for why it is not done yet.
        sort = (problem_ && problem_->all_integer()) ? tm_.getIntegerSort()
                                                     : tm_.getRealSort();
    } else {
        // No silent Real fallback: an unclassifiable fluent is a frontend
        // bug, and a wrong sort would violate a binding integer logic.
        throw std::runtime_error("SmtVariableFactory: unknown fluent type for "
                                 + fluent.name() + " -- cannot choose a sort");
    }

    return tm_.mkPlanFluent(idx, t, sort);
}

// ── cached accessors ───────────────────────────────────────────────────────
// Keyed by [timestep][dense ground id] -- the same indices that ride in the
// PLAN atom payloads. No string is rendered on this path.

const Term& SmtVariableFactory::get_fluent_variable(const Fluent& fluent, int timestep,
                                                    ExprID fluent_eid) {
    const uint32_t idx = ground_fluent_index(fluent, fluent_eid);
    ensure_timestep_capacity(timestep);

    auto& slots = state_vars_[timestep];
    if (idx >= slots.size()) {
        throw std::runtime_error(
            "SmtVariableFactory: ground-fluent id " + std::to_string(idx)
            + " out of range (registry size " + std::to_string(slots.size())
            + " at factory construction)");
    }
    Term& slot = slots[idx];
    if (slot.isNull()) {
        slot = create_new_fluent_variable(fluent, timestep, idx);
    }
    return slot;
}

const Term& SmtVariableFactory::get_action_variable(const Action& action, int timestep) {
    if (action.id() < 0) {
        throw std::runtime_error("SmtVariableFactory: action '" + action.name()
                                 + "' has no dense id");
    }
    const uint32_t id = static_cast<uint32_t>(action.id());
    ensure_timestep_capacity(timestep);

    auto& slots = action_vars_[timestep];
    if (id >= slots.size()) {
        throw std::runtime_error(
            "SmtVariableFactory: action id " + std::to_string(id)
            + " out of range (action count " + std::to_string(slots.size())
            + " at factory construction)");
    }
    Term& slot = slots[id];
    if (slot.isNull()) {
        // Actions are always Boolean -> always a THEORY_PLAN atom, indexed by
        // the action's own id so the reverse lookup needs no side table.
        slot = tm_.mkTerm(
            tm_.mkOp(Kind::PLAN_DOES, {id, static_cast<uint32_t>(timestep)}), {});
    }
    return slot;
}

const Term& SmtVariableFactory::get_action_variable(const Action& action, int timestep) const {
    const uint32_t id = static_cast<uint32_t>(action.id());
    if (action.id() >= 0 && timestep >= 0
        && timestep < static_cast<int>(action_vars_.size())
        && id < action_vars_[timestep].size()
        && !action_vars_[timestep][id].isNull()) {
        return action_vars_[timestep][id];
    }
    throw std::runtime_error("Action variable " + get_action_var_name(action, timestep)
                             + " has not been created yet");
}

// ── naming ─────────────────────────────────────────────────────────────────

std::string SmtVariableFactory::get_fluent_var_name(const Fluent& fluent) const {
    std::string name = fluent.name();
    for (const auto& param : fluent.parameters()) name += "_" + param.name();
    return name;
}

std::string SmtVariableFactory::get_fluent_var_name(const Fluent& fluent, int timestep) const {
    return get_fluent_var_name(fluent) + "_" + std::to_string(timestep);
}

std::string SmtVariableFactory::get_action_var_name(const Action& action) const {
    std::string name = action.name();
    for (const auto& param : action.parameters()) name += "_" + param.name();
    return name;
}

std::string SmtVariableFactory::get_action_var_name(const Action& action, int timestep) const {
    return get_action_var_name(action) + "_" + std::to_string(timestep);
}

// ── reverse lookup ─────────────────────────────────────────────────────────
// For PLAN atoms this is O(1) structural: read the operator payload, then
// index the problem's own registry -- the payload index IS the dense id the
// registry is positionally keyed by (actions()[i].id() == i;
// grounded_fluent_id == index in grounded_fluents(), problem.hpp). That is
// the point of giving the atoms theory kinds: identity lives in the term
// itself, so no name rendering, map probe or side table is needed.

std::optional<std::pair<const Action&, int>>
SmtVariableFactory::get_action_from_variable(const Term& var) const {
    if (var.getKind() != Kind::PLAN_DOES || !problem_) return std::nullopt;
    cvc5::Op op = var.getOp();
    const uint32_t id = op[0].getUInt32Value();
    const int t = static_cast<int>(op[1].getUInt32Value());
    if (id >= problem_->actions().size()) return std::nullopt;
    return std::make_pair(std::cref(problem_->actions()[id]), t);
}

std::optional<std::pair<ExprID, int>>
SmtVariableFactory::get_fluent_from_variable(const Term& var) const {
    if (var.getKind() != Kind::PLAN_FLUENT || !problem_) return std::nullopt;
    cvc5::Op op = var.getOp();
    const uint32_t idx = op[0].getUInt32Value();
    const int t = static_cast<int>(op[1].getUInt32Value());
    if (idx >= problem_->grounded_fluent_count()) return std::nullopt;
    return std::make_pair(problem_->grounded_fluent(static_cast<int>(idx)), t);
}

bool SmtVariableFactory::is_action_variable(const Term& var) const {
    return var.getKind() == Kind::PLAN_DOES;
}

bool SmtVariableFactory::is_fluent_variable(const Term& var) const {
    return var.getKind() == Kind::PLAN_FLUENT;
}

// ── timestep management ────────────────────────────────────────────────────

void SmtVariableFactory::ensure_timestep_capacity(int timestep) {
    // Preallocate the full dense range per timestep: both id spaces are fixed
    // once the problem is grounded, and a never-resized inner vector is what
    // keeps the references returned by the accessors stable.
    const size_t fluent_slots =
        problem_ ? static_cast<size_t>(problem_->grounded_fluent_count()) : 0;
    const size_t action_slots = problem_ ? problem_->actions().size() : 0;
    while (static_cast<int>(state_vars_.size()) <= timestep) {
        state_vars_.emplace_back(fluent_slots);
    }
    while (static_cast<int>(action_vars_.size()) <= timestep) {
        action_vars_.emplace_back(action_slots);
    }
}

}  // namespace rantanplan
