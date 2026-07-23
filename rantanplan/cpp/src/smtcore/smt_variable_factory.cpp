#include "smt_variable_factory.hpp"

#include "../problem/problem.hpp"
#include "../util/logger.hpp"

#include <stdexcept>

namespace rantanplan {

using cvc5::Kind;



SmtVariableFactory::SmtVariableFactory(cvc5::TermManager& tm)
    : tm_(tm), builder_(tm) {}

void SmtVariableFactory::set_problem(const Problem* p) {
    problem_ = p;
    builder_.set_problem(p);
}

// ── raw variable creation ──────────────────────────────────────────────────

Term SmtVariableFactory::create_bool_variable(const std::string& name) const {
    return tm_.mkConst(tm_.getBooleanSort(), name);
}

Term SmtVariableFactory::create_int_variable(const std::string& name) const {
    return tm_.mkConst(tm_.getIntegerSort(), name);
}

Term SmtVariableFactory::create_real_variable(const std::string& name) const {
    return tm_.mkConst(tm_.getRealSort(), name);
}

Term SmtVariableFactory::create_object_variable(const std::string& name) const {
    // Object fluents are encoded as numeric values (each object maps to its
    // global index) and must share the numeric sort, or a single-sort logic
    // (QF_LRA / QF_LIA) becomes inconsistent with the terms in the problem.
    return (problem_ && problem_->all_integer())
        ? create_int_variable(name)
        : create_real_variable(name);
}

Term SmtVariableFactory::create_symbol_variable(const std::string& name, const Type* type) const {
    if (!type) return create_int_variable(name);
    if (type->is_bool()) return create_bool_variable(name);
    if (type->is_int()) return create_int_variable(name);
    if (type->is_real()) {
        return (problem_ && problem_->all_integer())
            ? create_int_variable(name) : create_real_variable(name);
    }
    if (type->is_object()) return create_object_variable(name);
    return create_int_variable(name);
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
                                                    ExprID fluent_eid) {
    const uint32_t idx = ground_fluent_index(fluent, fluent_eid);
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
        Logger::instance().error("Unknown fluent type for " + fluent.name()
                                 + ", defaulting to real");
        sort = tm_.getRealSort();
    }

    id_to_fluent_[idx] = std::make_shared<Fluent>(fluent);
    return tm_.mkPlanFluent(idx, t, sort);
}

// ── cached accessors ───────────────────────────────────────────────────────

const Term& SmtVariableFactory::get_fluent_variable(const Fluent& fluent, int timestep,
                                                    ExprID fluent_eid) {
    const std::string var_name = get_fluent_var_name(fluent, timestep);
    ensure_timestep_capacity(timestep);

    auto& timestep_vars = state_vars_[timestep];
    auto it = timestep_vars.find(var_name);
    if (it == timestep_vars.end()) {
        Term new_var = create_new_fluent_variable(fluent, timestep, fluent_eid);
        timestep_vars[var_name] = std::make_shared<Term>(new_var);
        state_var_name_to_fluent_[var_name] = {std::make_shared<Fluent>(fluent), timestep};
        return *(timestep_vars[var_name]);
    }
    return *(it->second);
}

const Term& SmtVariableFactory::get_fluent_variable(const Fluent& fluent, int timestep) {
    // No ExprID: only legal for a fluent already created at this timestep.
    const std::string var_name = get_fluent_var_name(fluent, timestep);
    if (timestep < static_cast<int>(state_vars_.size())) {
        auto it = state_vars_[timestep].find(var_name);
        if (it != state_vars_[timestep].end()) return *(it->second);
    }
    throw std::runtime_error(
        "SmtVariableFactory: " + var_name + " has not been created yet; call the "
        "ExprID overload from the encoding visitor first");
}

const Term& SmtVariableFactory::get_action_variable(const Action& action, int timestep) {
    const std::string var_name = get_action_var_name(action, timestep);
    ensure_timestep_capacity(timestep);

    auto& timestep_vars = action_vars_[timestep];
    auto it = timestep_vars.find(var_name);
    if (it == timestep_vars.end()) {
        // Actions are always Boolean -> always a THEORY_PLAN atom, indexed by
        // the action's own id so the reverse lookup needs no side table.
        const uint32_t id = static_cast<uint32_t>(action.id());
        Term new_var = tm_.mkTerm(
            tm_.mkOp(Kind::PLAN_DOES, {id, static_cast<uint32_t>(timestep)}), {});
        timestep_vars[var_name] = std::make_shared<Term>(new_var);
        auto shared_action = std::make_shared<Action>(action);
        action_var_name_to_action_[var_name] = {shared_action, timestep};
        id_to_action_[id] = shared_action;
        return *(timestep_vars[var_name]);
    }
    return *(it->second);
}

const Term& SmtVariableFactory::get_fluent_variable(const Fluent& fluent, int timestep) const {
    const std::string var_name = get_fluent_var_name(fluent, timestep);
    if (timestep >= static_cast<int>(state_vars_.size())) {
        throw std::runtime_error("Timestep " + std::to_string(timestep) + " not allocated yet");
    }
    const auto& timestep_vars = state_vars_[timestep];
    auto it = timestep_vars.find(var_name);
    if (it == timestep_vars.end()) {
        throw std::runtime_error("Fluent variable " + var_name + " not found for timestep "
                                 + std::to_string(timestep));
    }
    return *(it->second);
}

const Term& SmtVariableFactory::get_action_variable(const Action& action, int timestep) const {
    const std::string var_name = get_action_var_name(action, timestep);
    if (timestep >= static_cast<int>(action_vars_.size())) {
        throw std::runtime_error("Timestep " + std::to_string(timestep) + " not allocated yet");
    }
    const auto& timestep_vars = action_vars_[timestep];
    auto it = timestep_vars.find(var_name);
    if (it == timestep_vars.end()) {
        throw std::runtime_error("Action variable " + var_name + " not found for timestep "
                                 + std::to_string(timestep));
    }
    return *(it->second);
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
// For PLAN atoms this is O(1) structural: read the operator payload. That is
// the point of giving the atoms theory kinds -- identity lives in the term
// itself, so no name rendering or map probe is needed on the hot path.

std::optional<std::pair<const Action&, int>>
SmtVariableFactory::get_action_from_variable(const Term& var) const {
    if (var.getKind() != Kind::PLAN_DOES) return std::nullopt;
    cvc5::Op op = var.getOp();
    const uint32_t id = op[0].getUInt32Value();
    const int t = static_cast<int>(op[1].getUInt32Value());
    auto it = id_to_action_.find(id);
    if (it == id_to_action_.end()) return std::nullopt;
    return std::make_pair(std::cref(*(it->second)), t);
}

std::optional<std::pair<const Fluent&, int>>
SmtVariableFactory::get_fluent_from_variable(const Term& var) const {
    if (var.getKind() != Kind::PLAN_FLUENT) return std::nullopt;
    cvc5::Op op = var.getOp();
    const uint32_t idx = op[0].getUInt32Value();
    const int t = static_cast<int>(op[1].getUInt32Value());
    auto it = id_to_fluent_.find(idx);
    if (it == id_to_fluent_.end()) return std::nullopt;
    return std::make_pair(std::cref(*(it->second)), t);
}

bool SmtVariableFactory::is_action_variable(const Term& var) const {
    return var.getKind() == Kind::PLAN_DOES;
}

bool SmtVariableFactory::is_fluent_variable(const Term& var) const {
    return var.getKind() == Kind::PLAN_FLUENT;
}

// ── enumeration ────────────────────────────────────────────────────────────

std::vector<std::shared_ptr<Term>>
SmtVariableFactory::get_all_fluent_variables(int timestep) const {
    std::vector<std::shared_ptr<Term>> variables;
    if (timestep >= static_cast<int>(state_vars_.size())) return variables;
    const auto& timestep_vars = state_vars_[timestep];
    variables.reserve(timestep_vars.size());
    for (const auto& pair : timestep_vars) variables.push_back(pair.second);
    return variables;
}

std::vector<std::shared_ptr<Term>>
SmtVariableFactory::get_all_action_variables(int timestep) const {
    std::vector<std::shared_ptr<Term>> variables;
    if (timestep >= static_cast<int>(action_vars_.size())) return variables;
    const auto& timestep_vars = action_vars_[timestep];
    variables.reserve(timestep_vars.size());
    for (const auto& pair : timestep_vars) variables.push_back(pair.second);
    return variables;
}

std::vector<std::pair<std::shared_ptr<Term>, int>>
SmtVariableFactory::get_all_fluent_variables() const {
    std::vector<std::pair<std::shared_ptr<Term>, int>> variables;
    for (int t = 0; t < static_cast<int>(state_vars_.size()); ++t) {
        for (const auto& pair : state_vars_[t]) variables.emplace_back(pair.second, t);
    }
    return variables;
}

std::vector<std::pair<std::shared_ptr<Term>, int>>
SmtVariableFactory::get_all_action_variables() const {
    std::vector<std::pair<std::shared_ptr<Term>, int>> variables;
    for (int t = 0; t < static_cast<int>(action_vars_.size()); ++t) {
        for (const auto& pair : action_vars_[t]) variables.emplace_back(pair.second, t);
    }
    return variables;
}

// ── timestep management ────────────────────────────────────────────────────

void SmtVariableFactory::ensure_timestep_capacity(int timestep) {
    while (static_cast<int>(state_vars_.size()) <= timestep) state_vars_.emplace_back();
    while (static_cast<int>(action_vars_.size()) <= timestep) action_vars_.emplace_back();
}

int SmtVariableFactory::get_max_timestep() const {
    return std::max(static_cast<int>(state_vars_.size()) - 1,
                    static_cast<int>(action_vars_.size()) - 1);
}

bool SmtVariableFactory::has_variables_for_timestep(int timestep) const {
    const bool has_state = timestep < static_cast<int>(state_vars_.size())
                           && !state_vars_[timestep].empty();
    const bool has_action = timestep < static_cast<int>(action_vars_.size())
                            && !action_vars_[timestep].empty();
    return has_state || has_action;
}

}  // namespace rantanplan
