#pragma once

#include "../problem/problem.hpp"
#include "../problem/effect_expression.hpp"

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rantanplan {

/**
 * @file gse_classification.hpp
 * @brief Static classification for the GSE encoding (implementation-plan M1).
 *
 * One pass over the ground task builds, per grounded fluent, its writers
 * (ASSIGN effects), contributors (INCREASE/DECREASE effects) and readers, and
 * per action the fluents it reads / writes / contributes.  From this it derives
 * the fluent categories, the aggregate feature flags used by the applicability
 * guard (D6), and the static interference relation used by constraint family C1.
 *
 * The module is deliberately Z3-free: it is pure analysis over the ExprPool.
 */

enum class GseFluentCategory { AssignOnly, PureAdditive, Mixed };

/// How an action touches a fluent (bitset-ish; an action may read+write+contrib).
struct GseTouch {
    bool read = false;
    bool write = false;    ///< has an ASSIGN effect on the fluent
    bool contrib = false;  ///< has an INCREASE/DECREASE effect on the fluent
};

/// A single writer (ASSIGN) or contributor (INCREASE/DECREASE) effect reference.
struct GseEffectRef {
    int action_id = -1;
    int effect_index = -1;
    ValueKind value_kind = ValueKind::CONSTANT;
    bool negated = false;  ///< true for DECREASE (amount enters the sum negated)
};

/// Per-fluent classification record.
struct GseFluentClass {
    ExprID fluent = EXPR_NULL;
    GseFluentCategory category = GseFluentCategory::AssignOnly;
    std::vector<GseEffectRef> writers;  ///< ASSIGN effects (excludes virtual bottom)
    std::vector<GseEffectRef> adds;     ///< INCREASE/DECREASE effects
    std::unordered_set<int> reader_actions;
    bool read_by_goal = false;
    bool is_bool = false;
    bool is_numeric = false;
    bool is_object = false;
    bool self_referential = false;  ///< some contributor's amount reads this fluent
    int reads() const {
        return static_cast<int>(reader_actions.size()) + (read_by_goal ? 1 : 0);
    }
};

/// Result of walking one expression for translatability (supported ops, linearity).
struct GseExprScan {
    bool supported = true;       ///< only ops the translator can render
    bool nonlinear = false;      ///< a state-variable product / non-affine subterm
    std::string first_unsupported;  ///< human-readable name of the first bad op
};

/// The full static classification of a task for GSE.
struct GseClassification {
    std::unordered_map<ExprID, GseFluentClass> fluents;

    // Per-action touched-fluent views (fluent ExprIDs).
    std::unordered_map<int, std::vector<ExprID>> action_reads;
    std::unordered_map<int, std::vector<ExprID>> action_writes;
    std::unordered_map<int, std::vector<ExprID>> action_contribs;
    std::vector<ExprID> goal_reads;

    // Aggregate feature flags (drive the applicability guard).
    bool any_conditional_effect = false;
    bool any_quantified_effect = false;
    bool any_nonlinear = false;         ///< any nonlinear expression anywhere
    bool any_dynamic_amount = false;    ///< any contributor with non-CONSTANT amount
    bool any_dynamic_writer = false;    ///< any ASSIGN with non-CONSTANT RHS
    bool any_unsupported_operator = false;
    std::string first_unsupported_operator;

    /// touch[action_id][fluent] — how that action touches the fluent.
    std::unordered_map<int, std::unordered_map<ExprID, GseTouch>> touch;
};

/// Build the static classification (one pass over problem.actions() + goal).
GseClassification gse_classify(const Problem& problem);

/// Scope of the applicability guard — widens per milestone.
enum class GseScope {
    M2,  ///< assign-only + pure-additive with CONSTANT amounts; no dynamic writers
    M4   ///< + mixed fluents, LINEAR writers/amounts; still no NONLINEAR/conditional
};

/// Returns a rejection reason if the task is out of scope, else std::nullopt (D6).
std::optional<std::string> gse_applicability_reason(const Problem& problem,
                                                     const GseClassification& c,
                                                     GseScope scope);

/// Record the E1 metrics through the Stats singleton (surface via --stats-file).
void gse_record_e1_stats(const Problem& problem, const GseClassification& c);

/// Whether two touches of the same fluent statically interfere (C1): order can
/// matter unless both sides only contribute (additions commute) with neither
/// reading the shared fluent.
inline bool gse_touches_interfere(const GseTouch& a, const GseTouch& b) {
    bool a_wc = a.write || a.contrib;
    bool b_wc = b.write || b.contrib;
    bool a_rwc = a.read || a.write || a.contrib;
    bool b_rwc = b.read || b.write || b.contrib;
    bool base = (a_wc && b_rwc) || (b_wc && a_rwc);
    if (!base) return false;
    bool a_contrib_only = a.contrib && !a.write && !a.read;
    bool b_contrib_only = b.contrib && !b.write && !b.read;
    if (a_contrib_only && b_contrib_only) return false;
    return true;
}

/// Whether two actions statically interfere (share a fluent whose relative
/// order can matter), per C1.  Handles the same-action (self) case too.
bool gse_actions_interfere(const GseClassification& c, int action_a, int action_b);

// ---- Expression helpers (shared with the translator) ----

/// Collect the STATE_VARIABLE ExprIDs read in `e` (does not recurse into a
/// fluent's own constant arguments).
void gse_collect_reads(ExprID e, const ExprPool& pool,
                       std::unordered_set<ExprID>& out);

/// Walk `e` checking it uses only translator-supported operators and is affine
/// in state variables.
GseExprScan gse_scan_expr(ExprID e, const Problem& problem);

} // namespace rantanplan
