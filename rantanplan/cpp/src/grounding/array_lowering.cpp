#include "array_lowering.hpp"

#include "../problem/const_fold.hpp"
#include "../problem/expr_builder.hpp"
#include "../problem/expr_enums.hpp"
#include "../problem/expr_walk.hpp"
#include "../util/logger.hpp"

#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace rantanplan {

namespace {

// ---------------------------------------------------------------------------
// Per-array-fluent lowering metadata.
// ---------------------------------------------------------------------------

struct ArrayInfo {
    std::vector<int64_t> sizes;                       // per-dimension size
    std::set<std::vector<int64_t>> undefined;         // full coordinate tuples
    std::map<std::vector<int64_t>, ExprID> cell_heads;  // defined coords -> cell FLUENT_SYMBOL id
    bool parameterized = false;                       // cells keep the array's object params
};

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

    /// Permissive lowering (for action preconditions/effects): an out-of-bounds
    /// or undefined-position array access becomes UNDEFINED (nullopt), which
    /// then propagates by the paper's Def. 9 (Davesa et al.) — strict for every
    /// operator (undefined if any operand is undefined) except OR and COUNT,
    /// which ignore undefined operands and are undefined only when *all* are.
    /// Callers decide what an undefined precondition / effect part means (a
    /// discarded action or a dropped conditional effect). Contrast rewrite(),
    /// which is strict-⊥ for goals/invariants/initial-state (a hard error).
    std::optional<ExprID> rewrite_permissive(ExprID eid, const std::string& context) {
        if (!eid.valid()) return eid;

        // An array access node — lower to its cell, or UNDEFINED if the (folded)
        // index is out of bounds / an undefined position.
        if (pool_.is_function_application(eid) &&
            pool_.op(eid) == ExprOperator::ARRAY_INDEX) {
            ExprID cell;
            if (resolve_access(eid, cell, context) == AccessResult::OutOfBounds)
                return std::nullopt;
            return cell;
        }

        // A bare (un-indexed) reference to an array fluent — a whole-array
        // operation, unsupported (same hard error as the strict rewrite).
        if (pool_.is_state_variable(eid)) {
            ExprID head = pool_.head_symbol_id(eid);
            if (pool_.is_fluent_symbol(head)) {
                const Fluent* f = problem_.find_fluent(pool_.payload_string(head));
                if (f && f->is_array())
                    throw std::runtime_error(
                        "Whole-array operation on array fluent '" + f->name() +
                        "' is not supported (only per-cell constant-index access "
                        "is lowered); appeared in " + context);
            }
        }

        // Leaf (constant, parameter, nullary state variable) rewrites to itself.
        if (!pool_.has_head_and_arguments(eid)) return eid;

        // Head+arguments node (a function application or an object state
        // variable): recurse the arguments and propagate undefinedness.
        const bool or_like = pool_.is_function_application(eid) &&
                             (pool_.is_or(eid) || pool_.is_count(eid));
        const ExprID head = pool_.head_symbol_id(eid);
        const size_t n = pool_.argument_count(eid);

        std::vector<ExprID> kept;
        kept.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            std::optional<ExprID> a = rewrite_permissive(pool_.argument(eid, i), context);
            if (!a) {
                if (or_like) continue;      // OR/COUNT ignore undefined operands
                return std::nullopt;        // strict: any undefined operand ⇒ undefined
            }
            kept.push_back(*a);
        }

        if (or_like && kept.empty()) return std::nullopt;   // all operands undefined
        if (pool_.is_or(eid) && kept.size() == 1) return kept.front();  // degenerate OR

        std::vector<ExprID> new_kids;
        new_kids.reserve(kept.size() + 1);
        new_kids.push_back(head);
        for (ExprID a : kept) new_kids.push_back(a);
        ExprNode node = pool_.get(eid);
        node.children = std::move(new_kids);
        return pool_.intern(std::move(node));
    }

private:
    enum class AccessResult { Ok, OutOfBounds };

    const Problem& problem_;
    ExprPool& pool_;
    std::vector<Fluent> cell_fluents_;
    std::unordered_map<int, ArrayInfo> arrays_;           // array fluent id -> info

    /// Mint one scalar cell fluent + state variable per defined array position,
    /// in fluent-declaration order and row-major position order.
    void mint_cells() {
        for (const Fluent& f : problem_.fluents()) {
            if (!f.is_array()) continue;

            const Type* elem_type = f.array_leaf_element_type();
            if (!elem_type) {
                throw std::runtime_error("Array fluent '" + f.name() +
                                         "' has no resolved element type.");
            }
            int elem_type_id = problem_.type_id_of(elem_type);

            ArrayInfo info;
            info.sizes = f.array_sizes();
            // Object-parameterised array fluents (e.g. grid(p)[r][c]) mint cell
            // fluents that keep the array's object parameters: grid[r][c](p),
            // resolved to a state variable with the access's object args and
            // grounded like any parameterised fluent.
            info.parameterized = f.parameter_count() > 0;
            for (const auto& pos : f.array_undefined_positions()) {
                info.undefined.insert(pos);
            }

            const size_t ndim = info.sizes.size();
            std::vector<int64_t> coord(ndim, 0);
            bool done = ndim == 0;
            while (!done) {
                if (!info.undefined.count(coord)) {
                    info.cell_heads.emplace(coord,
                                            mint_cell(f, coord, elem_type, elem_type_id));
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
            arrays_.emplace(f.id(), std::move(info));
        }
    }

    ExprID mint_cell(const Fluent& array_fluent, const std::vector<int64_t>& coord,
                     const Type* elem_type, int elem_type_id) {
        std::string cell_name = array_fluent.name();
        for (int64_t c : coord) cell_name += "[" + std::to_string(c) + "]";

        // A cell is a scalar fluent of the leaf element type, keeping the array
        // fluent's object parameters (empty for a nullary array).
        Fluent cell(cell_name, elem_type, array_fluent.parameters());
        if (array_fluent.has_declared_bounds()) {
            cell.set_declared_bounds(array_fluent.declared_lower_bound(),
                                     array_fluent.declared_upper_bound());
        }
        // The array's single scalar default (UP stores array defaults as
        // scalars) becomes every cell's default; CWA honours it per cell.
        if (array_fluent.declared_default()) {
            cell.set_declared_default(*array_fluent.declared_default());
        }
        cell_fluents_.push_back(std::move(cell));

        // The cell's FLUENT_SYMBOL head identifies it across accesses (resolve
        // builds a state variable from it plus the access's object args). For a
        // nullary array, also pre-intern the full state variable so its ExprID
        // is assigned in mint order; resolve_access rebuilds the identical
        // id via dedup.
        ExprID head_id = pool_.intern_fluent_symbol(cell_name, elem_type_id);
        if (array_fluent.parameter_count() == 0) pool_.intern_state_variable(head_id);
        return head_id;
    }

    /// Resolve a (possibly nested) ARRAY_INDEX access to its cell.
    AccessResult resolve_access(ExprID eid, ExprID& out_cell,
                                const std::string& context) {
        const Fluent* base = nullptr;
        std::vector<int64_t> coords_rev;
        std::vector<ExprID> base_args;  // object args on the base fluent (grid(p)[..])
        if (!collect_access(eid, base, coords_rev, base_args, context)) {
            throw std::runtime_error(
                "Malformed array access in " + context + " (" +
                pool_.to_string(eid) + ")");
        }
        std::vector<int64_t> coords(coords_rev.rbegin(), coords_rev.rend());

        auto it = arrays_.find(base->id());
        if (it == arrays_.end()) {
            throw std::runtime_error("Array access on unknown array fluent '" +
                                     base->name() + "' in " + context);
        }
        const ArrayInfo& info = it->second;

        if (coords.size() < info.sizes.size()) {
            throw std::runtime_error(
                "Partial indexing of array fluent '" + base->name() +
                "' (whole-row/slice access) is not supported; appeared in " +
                context);
        }
        if (coords.size() > info.sizes.size()) {
            throw std::runtime_error(
                "Array fluent '" + base->name() + "' has " +
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
        auto cit = info.cell_heads.find(coords);
        if (cit == info.cell_heads.end()) {
            throw std::runtime_error("Internal error: no cell minted for a "
                                     "defined position of array '" + base->name() +
                                     "' in " + context);
        }
        // The cell state variable: its FLUENT_SYMBOL head plus the base fluent's
        // object args (empty for a nullary array -> dedups to the pre-interned
        // cell).
        out_cell = pool_.intern_state_variable(cit->second, base_args);
        return AccessResult::Ok;
    }

    /// Walk the ARRAY_INDEX chain, collecting the base array fluent, the folded
    /// index per dimension (outermost first, i.e. reversed dim order), and the
    /// base fluent's object arguments. Returns false if `eid` is not an array
    /// access.
    bool collect_access(ExprID eid, const Fluent*& base_fluent,
                        std::vector<int64_t>& coords_rev,
                        std::vector<ExprID>& base_args,
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
            return collect_access(arr, base_fluent, coords_rev, base_args, context);
        }
        if (pool_.is_state_variable(eid)) {
            ExprID head = pool_.head_symbol_id(eid);
            if (!pool_.is_fluent_symbol(head)) return false;
            const Fluent* f = problem_.find_fluent(pool_.payload_string(head));
            if (!f || !f->is_array()) return false;
            // Keep the base fluent's object arguments (e.g. p in grid(p)[r][c]);
            // resolve_access re-applies them to the minted cell fluent.
            size_t nargs = pool_.argument_count(eid);
            base_args.reserve(nargs);
            for (size_t i = 0; i < nargs; ++i)
                base_args.push_back(pool_.argument(eid, i));
            base_fluent = f;
            return true;
        }
        return false;
    }
};

// ---------------------------------------------------------------------------
// Action lowering (L1 reads + L3 writes + discards).
// ---------------------------------------------------------------------------

/// Lower one action under permissive undefinedness. Returns nullopt if the
/// whole instance is discarded: the precondition or cost evaluates to ⊥
/// (undefined ⇒ never applicable), or an *unconditional* effect writes/reads an
/// out-of-bounds cell. A *conditional* effect ⟨C ▷ l := r⟩:
///   * whose condition C is ⊥ is dropped (it never fires);
///   * whose target l or value r is ⊥ is dropped and ¬C joins the
///     precondition: the effect would be invalid where C holds, so the action
///     must be inapplicable there.
/// A guarded boundary access (`a[i+1]:=v WHEN i<last` at i = last) keeps its
/// action: C is false there, so ¬C adds nothing.
///
/// Two effects may write one cell (`a[i] += 1; a[j] += 1` at i = j). The
/// instance is kept: a cell is an ordinary fluent, and EffectNormalizationPass
/// combines the two writes as it does for any fluent.
std::optional<Action> lower_action(const Action& action, ArrayLowerer& lowerer,
                                    ExprPool& pool) {
    const std::string ctx = "action '" + action.name() + "'";

    Action out(action.name());
    out.set_pool(&pool);
    out.set_parameters(action.parameters());

    if (action.has_precondition()) {
        std::optional<ExprID> pre = lowerer.rewrite_permissive(action.precondition_id(), ctx);
        if (!pre) return std::nullopt;  // undefined precondition ⇒ never applicable
        out.set_precondition_id(*pre);
    }
    if (action.has_explicit_cost()) {
        std::optional<ExprID> cost = lowerer.rewrite_permissive(action.cost_id(), ctx);
        if (!cost) return std::nullopt;  // undefined cost ⇒ ill-formed instance
        out.set_cost_id(*cost);
    }

    // Conditions of dropped effects whose target or value is undefined: the
    // action is inapplicable wherever one of them holds.
    std::vector<ExprID> blocking;

    for (const auto& eff : action.effects()) {
        const auto& ee = eff.effect_expression();
        // Pass-ordering invariant: forall-effects are expanded by the
        // QuantifierExpansionPass before grounding, so none may reach here.
        if (!ee.forall_variable_ids().empty()) {
            throw std::runtime_error(
                "Array lowering ran on an unexpanded forall-effect in " + ctx +
                " — QuantifierExpansionPass must run before grounding.");
        }
        const bool conditional = ee.is_conditional();

        // A conditional effect with an undefined condition can never fire — drop it.
        ExprID cond = EXPR_NULL;
        if (conditional) {
            std::optional<ExprID> c = lowerer.rewrite_permissive(ee.condition_id(), ctx);
            if (!c) continue;
            cond = *c;
        }

        std::optional<ExprID> fluent = lowerer.rewrite_permissive(ee.fluent_id(), ctx);
        std::optional<ExprID> value  = lowerer.rewrite_permissive(ee.value_id(), ctx);
        if (!fluent || !value) {
            // Out-of-bounds or undefined-position target or RHS: an
            // unconditional effect makes the whole action inapplicable; a
            // conditional one is dropped and blocks the action where C holds.
            if (!conditional) return std::nullopt;
            blocking.push_back(cond);
            continue;
        }

        out.add_effect(Effect(EffectExpression(ee.kind(), *fluent, *value, cond, &pool)));
    }

    if (!blocking.empty()) {
        ExprBuilder b(pool);
        std::vector<ExprID> parts;
        if (out.has_precondition()) parts.push_back(out.precondition_id());
        for (ExprID c : blocking) parts.push_back(b.make_not(c));
        ExprID pre = b.make_and(std::move(parts));
        if (pool.is_false_constant(pre)) return std::nullopt;  // some C is always true
        if (!pool.is_true_constant(pre)) out.set_precondition_id(pre);
    }
    return out;
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

    // Actions (L1/L3, discards).
    std::vector<Action> new_actions;
    new_actions.reserve(problem.actions().size());
    size_t discarded = 0;
    for (const Action& a : problem.actions()) {
        auto lowered = lower_action(a, lowerer, pool);
        if (lowered) {
            new_actions.push_back(std::move(*lowered));
            continue;
        }
        ++discarded;
        Logger::instance().verbose(
            "Array lowering: discarding instance " + a.label() +
            " — out-of-bounds or undefined-position array access");
    }

    // Goals and invariants: an out-of-bounds access is a hard error (an
    // ill-formed goal is a modelling mistake, not a runtime-guarded access).
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

    // Initial state: an out-of-bounds / undefined-position assignment is
    // permissively dropped (paper Def. 9 — undefinedness detected at compile
    // time is simply removed), with a warning, since a defaulted array cell that
    // does not exist cannot be assigned. Matches the fork's permissive AR.
    std::vector<Assignment> new_initial;
    new_initial.reserve(problem.initial_state().size());
    for (const Assignment& asn : problem.initial_state()) {
        std::optional<ExprID> fluent = lowerer.rewrite_permissive(asn.fluent_id(), "initial state");
        std::optional<ExprID> value = lowerer.rewrite_permissive(asn.value_id(), "initial state");
        if (!fluent || !value) {
            Logger::instance().info(
                "Warning: dropping out-of-bounds/undefined initial-state assignment "
                + pool.to_string(asn.fluent_id()) + " := " + pool.to_string(asn.value_id()));
            continue;
        }
        new_initial.emplace_back(*fluent, *value, &pool);
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

    result.actions_ = std::move(new_actions);
    for (size_t i = 0; i < result.actions_.size(); ++i)
        result.actions_[i].set_id(static_cast<int>(i));

    result.goals_ = std::move(new_goals);
    result.initial_state_ = std::move(new_initial);
    result.state_invariants_ = std::move(new_invariants);

    // One canonical finalisation over the replaced components (fluent id/name
    // maps, action name map, grounded-fluent collection).
    result.rebuild_derived_indices();

    Logger::instance().component(VerbosityLevel::INFO, "ArrayLowering", {
        {"cells", std::to_string(lowerer.cell_fluents().size())},
        {"actions", std::to_string(result.actions_.size())},
        {"discarded instances", std::to_string(discarded)}
    });

    return result;
}

void assert_no_array_index(const Problem& problem) {
    const ExprPool& pool = problem.pool();
    for_each_expression(problem, [&](ExprID eid) {
        if (pool.is_function_application(eid) &&
            pool.op(eid) == ExprOperator::ARRAY_INDEX)
            throw std::runtime_error(
                "Array lowering invariant violated: an ARRAY_INDEX node survived "
                "grounding (" + pool.to_string(eid) + ").");
    });
}

} // namespace rantanplan
