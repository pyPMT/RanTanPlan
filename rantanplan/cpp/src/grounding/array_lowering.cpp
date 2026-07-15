#include "array_lowering.hpp"

#include "../problem/expr_enums.hpp"
#include "../util/logger.hpp"

#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rantanplan {

namespace {

// ---------------------------------------------------------------------------
// Per-array-fluent lowering metadata.
// ---------------------------------------------------------------------------

struct ArrayInfo {
    std::vector<int64_t> sizes;                       // per-dimension size
    std::set<std::vector<int64_t>> undefined;         // full coordinate tuples
    std::map<std::vector<int64_t>, ExprID> cells;     // defined coords -> cell state var
};

/// Fold a constant-integer index subtree to its value. Returns nullopt if the
/// subtree is not a compile-time integer constant (a symbolic index, which is
/// Phase 2 territory). Handles the arithmetic operators an index may carry
/// (e.g. `c + 1` after parameter substitution).
std::optional<int64_t> fold_const_int(ExprID eid, const ExprPool& pool) {
    if (!eid.valid()) return std::nullopt;

    if (pool.is_constant(eid)) {
        if (pool.payload_is_int(eid)) return pool.payload_int(eid);
        if (pool.payload_is_double(eid)) {
            double d = pool.payload_double(eid);
            int64_t i = static_cast<int64_t>(d);
            if (static_cast<double>(i) == d) return i;
        }
        return std::nullopt;
    }

    if (!pool.is_function_application(eid)) return std::nullopt;

    ExprOperator op = pool.op(eid);
    size_t nargs = pool.argument_count(eid);

    if (op == ExprOperator::ABSOLUTE && nargs == 1) {
        auto a = fold_const_int(pool.argument(eid, 0), pool);
        if (!a) return std::nullopt;
        return *a < 0 ? -*a : *a;
    }

    if (nargs != 2) return std::nullopt;
    auto a = fold_const_int(pool.argument(eid, 0), pool);
    auto b = fold_const_int(pool.argument(eid, 1), pool);
    if (!a || !b) return std::nullopt;

    switch (op) {
        case ExprOperator::PLUS:     return *a + *b;
        case ExprOperator::MINUS:    return *a - *b;
        case ExprOperator::MULTIPLY: return *a * *b;
        case ExprOperator::DIVIDE:   return *b != 0 ? std::optional<int64_t>(*a / *b) : std::nullopt;
        case ExprOperator::MODULO:   return *b != 0 ? std::optional<int64_t>(*a % *b) : std::nullopt;
        case ExprOperator::MAXIMUM:  return std::max(*a, *b);
        case ExprOperator::MINIMUM:  return std::min(*a, *b);
        default:                     return std::nullopt;
    }
}

/// The recursive worker: mints cells, resolves accesses, and rewrites
/// expression trees. Constructed once over the (const) source problem, whose
/// shared ExprPool it interns new nodes into.
class ArrayLowerer {
public:
    explicit ArrayLowerer(const Problem& problem)
        : problem_(problem), pool_(*problem.pool_ptr()) {
        mint_cells();
    }

    const std::vector<Fluent>& cell_fluents() const { return cell_fluents_; }

    /// Rewrite an expression, lowering every array access to its cell.
    /// `discard_on_oob`: on an out-of-bounds / undefined access, set
    /// `out_discard` and stop (the containing action instance is inapplicable).
    /// Otherwise (goals / initial state) an ill-formed access is a hard error.
    ExprID rewrite(ExprID eid, bool discard_on_oob, bool& out_discard,
                   const std::string& context) {
        if (!eid.valid() || out_discard) return eid;

        // An array access node — lower it to a cell (does not recurse into the
        // ARRAY_INDEX structure generically; the index subtrees hold no cells).
        if (pool_.is_function_application(eid) &&
            pool_.op(eid) == ExprOperator::ARRAY_INDEX) {
            ExprID cell;
            AccessResult r = resolve_access(eid, cell, context);
            if (r == AccessResult::OutOfBounds) {
                if (discard_on_oob) { out_discard = true; return eid; }
                throw std::runtime_error(
                    "Array access out of bounds or on an undefined position in " +
                    context + " (" + pool_.to_string(eid) + ")");
            }
            return cell;
        }

        // A bare (un-indexed) reference to an array fluent — a whole-array
        // operation (eq(a,b), whole-array assignment). Not supported in Phase 1.
        if (pool_.is_state_variable(eid)) {
            ExprID head = pool_.head_symbol_id(eid);
            if (pool_.is_fluent_symbol(head)) {
                const Fluent* f = problem_.find_fluent(pool_.payload_string(head));
                if (f && f->is_array()) {
                    throw std::runtime_error(
                        "Whole-array operation on array fluent '" + f->name() +
                        "' is not supported (only per-cell constant-index access "
                        "is lowered); appeared in " + context);
                }
            }
        }

        // Generic structural rewrite: recurse into children, re-intern if any
        // changed (mirrors substitute()).
        const auto& kids = pool_.children(eid);
        if (kids.empty()) return eid;

        std::vector<ExprID> new_kids;
        new_kids.reserve(kids.size());
        bool changed = false;
        for (ExprID c : kids) {
            ExprID nc = rewrite(c, discard_on_oob, out_discard, context);
            if (out_discard) return eid;
            new_kids.push_back(nc);
            changed |= (nc != c);
        }
        if (!changed) return eid;

        ExprNode node = pool_.get(eid);
        node.children = std::move(new_kids);
        return pool_.intern(std::move(node));
    }

    /// True if `eid` is one of the minted cell state variables.
    bool is_cell(ExprID eid) const { return cell_set_.count(eid) > 0; }

private:
    enum class AccessResult { Ok, OutOfBounds };

    const Problem& problem_;
    ExprPool& pool_;
    std::vector<Fluent> cell_fluents_;
    std::unordered_map<std::string, ArrayInfo> arrays_;   // array fluent name -> info
    std::unordered_set<ExprID> cell_set_;                 // all minted cell state vars

    /// Mint one scalar cell fluent + state variable per defined array position,
    /// in fluent-declaration order and row-major position order.
    void mint_cells() {
        const auto& types = problem_.types();
        for (const Fluent& f : problem_.fluents()) {
            if (!f.is_array()) continue;

            if (f.parameter_count() > 0) {
                throw std::runtime_error(
                    "Array fluent '" + f.name() + "' has signature parameters; "
                    "object-parameterised array fluents are not yet supported.");
            }

            const Type* elem_type = f.array_leaf_element_type();
            if (!elem_type) {
                throw std::runtime_error("Array fluent '" + f.name() +
                                         "' has no resolved element type.");
            }
            int elem_type_id = static_cast<int>(elem_type - &types[0]);

            ArrayInfo info;
            info.sizes = f.array_sizes();
            for (const auto& pos : f.array_undefined_positions()) {
                info.undefined.insert(pos);
            }

            const size_t ndim = info.sizes.size();
            std::vector<int64_t> coord(ndim, 0);
            bool done = ndim == 0;
            while (!done) {
                if (!info.undefined.count(coord)) {
                    ExprID cell = mint_cell(f, coord, elem_type, elem_type_id);
                    info.cells.emplace(coord, cell);
                }
                // Row-major odometer: last dimension varies fastest.
                size_t d = ndim;
                while (d > 0) {
                    --d;
                    if (++coord[d] < info.sizes[d]) break;
                    coord[d] = 0;
                    if (d == 0) { done = true; }
                }
            }
            arrays_.emplace(f.name(), std::move(info));
        }
    }

    ExprID mint_cell(const Fluent& array_fluent, const std::vector<int64_t>& coord,
                     const Type* elem_type, int elem_type_id) {
        std::string cell_name = array_fluent.name();
        for (int64_t c : coord) cell_name += "[" + std::to_string(c) + "]";

        // Head fluent symbol (matches converter/ir_io state-variable layout).
        ExprNode head;
        head.kind = static_cast<int>(ExprKind::FLUENT_SYMBOL);
        head.type_id = elem_type_id;
        head.payload = cell_name;
        ExprID head_id = pool_.intern(std::move(head));

        ExprNode sv;
        sv.kind = static_cast<int>(ExprKind::STATE_VARIABLE);
        sv.type_id = elem_type_id;
        sv.children = {head_id};
        ExprID sv_id = pool_.intern(std::move(sv));

        Fluent cell(cell_name, elem_type);
        if (array_fluent.has_declared_bounds()) {
            cell.set_declared_bounds(array_fluent.declared_lower_bound(),
                                     array_fluent.declared_upper_bound());
        }
        cell_fluents_.push_back(std::move(cell));
        cell_set_.insert(sv_id);
        return sv_id;
    }

    /// Resolve a (possibly nested) ARRAY_INDEX access to its cell.
    AccessResult resolve_access(ExprID eid, ExprID& out_cell,
                                const std::string& context) {
        std::string base_name;
        std::vector<int64_t> coords_rev;
        if (!collect_access(eid, base_name, coords_rev, context)) {
            throw std::runtime_error(
                "Malformed array access in " + context + " (" +
                pool_.to_string(eid) + ")");
        }
        std::vector<int64_t> coords(coords_rev.rbegin(), coords_rev.rend());

        auto it = arrays_.find(base_name);
        if (it == arrays_.end()) {
            throw std::runtime_error("Array access on unknown array fluent '" +
                                     base_name + "' in " + context);
        }
        const ArrayInfo& info = it->second;

        if (coords.size() < info.sizes.size()) {
            throw std::runtime_error(
                "Partial indexing of array fluent '" + base_name +
                "' (whole-row/slice access) is not supported; appeared in " +
                context);
        }
        if (coords.size() > info.sizes.size()) {
            throw std::runtime_error(
                "Array fluent '" + base_name + "' has " +
                std::to_string(info.sizes.size()) + " dimension(s) but was "
                "accessed with " + std::to_string(coords.size()) +
                " indices; appeared in " + context);
        }
        for (size_t d = 0; d < coords.size(); ++d) {
            if (coords[d] < 0 || coords[d] >= info.sizes[d]) {
                return AccessResult::OutOfBounds;
            }
        }
        if (info.undefined.count(coords)) return AccessResult::OutOfBounds;

        // A defined, in-bounds coordinate always has a minted cell (mint_cells
        // is exhaustive); guard against a future minting/validation mismatch.
        auto cit = info.cells.find(coords);
        if (cit == info.cells.end()) {
            throw std::runtime_error("Internal error: no cell minted for a "
                                     "defined position of array '" + base_name +
                                     "' in " + context);
        }
        out_cell = cit->second;
        return AccessResult::Ok;
    }

    /// Walk the ARRAY_INDEX chain, collecting the base array fluent name and
    /// the folded index per dimension (outermost first, i.e. reversed dim
    /// order). Returns false if `eid` is not an array access.
    bool collect_access(ExprID eid, std::string& base_name,
                        std::vector<int64_t>& coords_rev,
                        const std::string& context) {
        if (pool_.is_function_application(eid) &&
            pool_.op(eid) == ExprOperator::ARRAY_INDEX) {
            ExprID arr = pool_.argument(eid, 0);
            ExprID idx = pool_.argument(eid, 1);
            auto c = fold_const_int(idx, pool_);
            if (!c) {
                throw std::runtime_error(
                    "Array index did not fold to a constant in " + context +
                    " (" + pool_.to_string(idx) +
                    "); symbolic indices are not supported in Phase 1.");
            }
            coords_rev.push_back(*c);
            return collect_access(arr, base_name, coords_rev, context);
        }
        if (pool_.is_state_variable(eid)) {
            ExprID head = pool_.head_symbol_id(eid);
            if (!pool_.is_fluent_symbol(head)) return false;
            const std::string& name = pool_.payload_string(head);
            const Fluent* f = problem_.find_fluent(name);
            if (!f || !f->is_array()) return false;
            if (pool_.argument_count(eid) != 0) {
                throw std::runtime_error(
                    "Array fluent '" + name + "' has arguments; "
                    "object-parameterised array fluents are not yet supported.");
            }
            base_name = name;
            return true;
        }
        return false;
    }
};

// ---------------------------------------------------------------------------
// Action lowering (L1 reads + L3 writes + discard/collision handling).
// ---------------------------------------------------------------------------

/// Why an action instance was discarded during lowering.
enum class DiscardReason { OutOfBounds, Collision };

/// Lower one action; returns nullopt (with `out_reason` set) if the whole
/// instance is discarded: an out-of-bounds/undefined access (strict-⊥
/// inapplicability), or an intra-action same-cell write collision (e.g.
/// `a[i]:=x; a[j]:=y` instantiated at i=j). Whether a schema whose EVERY
/// instance collides is an unconditionally broken model is decided by the
/// caller, which sees all instances.
std::optional<Action> lower_action(const Action& action, ArrayLowerer& lowerer,
                                    ExprPool& pool, DiscardReason& out_reason) {
    const std::string ctx = "action '" + action.name() + "'";
    bool discard = false;
    out_reason = DiscardReason::OutOfBounds;

    Action out(action.name());
    out.set_pool(&pool);
    out.set_parameters(action.parameters());

    if (action.has_precondition()) {
        ExprID pre = lowerer.rewrite(action.precondition_id(), true, discard, ctx);
        if (discard) return std::nullopt;
        out.set_precondition_id(pre);
    }
    if (action.has_explicit_cost()) {
        ExprID cost = lowerer.rewrite(action.cost_id(), true, discard, ctx);
        if (discard) return std::nullopt;
        out.set_cost_id(cost);
    }

    std::vector<ExprID> cell_targets;
    for (const auto& eff : action.effects()) {
        const auto& ee = eff.effect_expression();
        // Pass-ordering invariant: forall-effects are expanded by the
        // QuantifierExpansionPass before grounding, so none may reach here.
        if (!ee.forall_variable_ids().empty()) {
            throw std::runtime_error(
                "Array lowering ran on an unexpanded forall-effect in " + ctx +
                " — QuantifierExpansionPass must run before grounding.");
        }
        ExprID fluent = lowerer.rewrite(ee.fluent_id(), true, discard, ctx);
        if (discard) return std::nullopt;
        ExprID value = lowerer.rewrite(ee.value_id(), true, discard, ctx);
        if (discard) return std::nullopt;
        ExprID cond = ee.is_conditional()
            ? lowerer.rewrite(ee.condition_id(), true, discard, ctx)
            : EXPR_NULL;
        if (discard) return std::nullopt;

        out.add_effect(Effect(EffectExpression(ee.kind(), fluent, value, cond, &pool)));
        if (lowerer.is_cell(fluent)) cell_targets.push_back(fluent);
    }

    // Intra-action same-cell write collision (the chained encoder keeps only
    // the first effect per (action, variable), so a second write to the same
    // cell would be silently dropped).
    std::unordered_set<ExprID> seen;
    for (ExprID t : cell_targets) {
        if (!seen.insert(t).second) {
            out_reason = DiscardReason::Collision;
            return std::nullopt;
        }
    }

    return out;
}

// ---------------------------------------------------------------------------
// ARRAY_INDEX survival check.
// ---------------------------------------------------------------------------

void walk_check(ExprID eid, const ExprPool& pool, std::unordered_set<ExprID>& seen) {
    if (!eid.valid() || !seen.insert(eid).second) return;
    if (pool.is_function_application(eid) && pool.op(eid) == ExprOperator::ARRAY_INDEX) {
        throw std::runtime_error(
            "Array lowering invariant violated: an ARRAY_INDEX node survived "
            "grounding (" + pool.to_string(eid) + ").");
    }
    for (ExprID c : pool.children(eid)) walk_check(c, pool, seen);
}

} // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool has_array_fluents(const Problem& problem) {
    for (const Fluent& f : problem.fluents()) {
        if (f.is_array()) return true;
    }
    return false;
}

Problem lower_arrays(const Problem& problem) {
    ExprPool& pool = *problem.pool_ptr();
    ArrayLowerer lowerer(problem);

    // Actions (L1/L3, discards, collisions). Per schema *name* we count total
    // instances vs. collision discards: if every instance of a schema collides
    // (two effects on one cell), the model is unconditionally broken and must
    // fail loudly instead of the schema silently vanishing.
    std::vector<Action> new_actions;
    new_actions.reserve(problem.actions().size());
    size_t discarded = 0;
    std::map<std::string, std::pair<size_t, size_t>> per_schema;  // name -> {total, collisions}
    for (const Action& a : problem.actions()) {
        auto& counts = per_schema[a.name()];
        ++counts.first;
        DiscardReason reason;
        auto lowered = lower_action(a, lowerer, pool, reason);
        if (lowered) {
            new_actions.push_back(std::move(*lowered));
            continue;
        }
        ++discarded;
        if (reason == DiscardReason::Collision) {
            ++counts.second;
            Logger::instance().verbose(
                "Array lowering: discarding instance " + a.label() +
                " — two effects write the same array cell");
        } else {
            Logger::instance().verbose(
                "Array lowering: discarding instance " + a.label() +
                " — out-of-bounds or undefined-position array access");
        }
    }
    for (const auto& [name, counts] : per_schema) {
        if (counts.first > 0 && counts.second == counts.first) {
            throw std::runtime_error(
                "Action '" + name + "' writes the same array cell with two "
                "effects in every instance (unconditionally broken model).");
        }
    }

    // Goals, invariants, initial state (ill-formed access = hard error).
    bool ignore = false;
    std::vector<Goal> new_goals;
    new_goals.reserve(problem.goals().size());
    for (const Goal& g : problem.goals()) {
        ExprID gid = lowerer.rewrite(g.goal_id(), false, ignore, "goal");
        new_goals.emplace_back(gid, &pool);
    }

    std::vector<ExprID> new_invariants;
    new_invariants.reserve(problem.state_invariants().size());
    for (ExprID inv : problem.state_invariants()) {
        new_invariants.push_back(lowerer.rewrite(inv, false, ignore, "state invariant"));
    }

    std::vector<Assignment> new_initial;
    new_initial.reserve(problem.initial_state().size());
    for (const Assignment& asn : problem.initial_state()) {
        ExprID fluent = lowerer.rewrite(asn.fluent_id(), false, ignore, "initial state");
        ExprID value = lowerer.rewrite(asn.value_id(), false, ignore, "initial state");
        new_initial.emplace_back(fluent, value, &pool);
    }

    // Assemble the lowered problem: drop array fluent schemas, append cells.
    Problem result = problem.clone_base();
    std::vector<Fluent> new_fluents;
    new_fluents.reserve(problem.fluent_count() + lowerer.cell_fluents().size());
    for (const Fluent& f : problem.fluents()) {
        if (!f.is_array()) new_fluents.push_back(f);
    }
    for (const Fluent& c : lowerer.cell_fluents()) new_fluents.push_back(c);
    result.fluents_ = std::move(new_fluents);
    result.build_fluent_mappings();

    result.actions_ = std::move(new_actions);
    for (size_t i = 0; i < result.actions_.size(); ++i)
        result.actions_[i].set_id(static_cast<int>(i));
    result.build_action_mappings();

    result.goals_ = std::move(new_goals);
    result.initial_state_ = std::move(new_initial);
    result.state_invariants_ = std::move(new_invariants);
    result.collect_grounded_fluents();

    Logger::instance().component(VerbosityLevel::INFO, "ArrayLowering", {
        {"cells", std::to_string(lowerer.cell_fluents().size())},
        {"actions", std::to_string(result.actions_.size())},
        {"discarded instances", std::to_string(discarded)}
    });

    return result;
}

void assert_no_array_index(const Problem& problem) {
    const ExprPool& pool = problem.pool();
    std::unordered_set<ExprID> seen;
    for (const Action& a : problem.actions()) {
        if (a.has_precondition()) walk_check(a.precondition_id(), pool, seen);
        if (a.has_explicit_cost()) walk_check(a.cost_id(), pool, seen);
        for (const auto& eff : a.effects()) {
            const auto& ee = eff.effect_expression();
            walk_check(ee.fluent_id(), pool, seen);
            walk_check(ee.value_id(), pool, seen);
            if (ee.is_conditional()) walk_check(ee.condition_id(), pool, seen);
        }
    }
    for (const Goal& g : problem.goals()) walk_check(g.goal_id(), pool, seen);
    for (ExprID inv : problem.state_invariants()) walk_check(inv, pool, seen);
    for (const Assignment& asn : problem.initial_state()) {
        walk_check(asn.fluent_id(), pool, seen);
        walk_check(asn.value_id(), pool, seen);
    }
}

} // namespace rantanplan
