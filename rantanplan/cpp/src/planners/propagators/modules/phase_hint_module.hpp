#pragma once

#include "../propagator_module.hpp"
#include "../propagator_strategy.hpp"
#include <unordered_map>
#include <utility>

namespace rantanplan {

/**
 * @brief Phase hints: redirect Z3's case splits toward a supplied partial
 *        assignment (warm start).
 *
 * Arm A of the phase-hint ladder (design doc §5.2): the windowed-commitment
 * planner feeds the greedy dive's committed prefix — action variables of
 * every step below the current anchor — as (variable id -> phase) hints for
 * the cold complete solver.  When Z3 decides a hinted variable with the
 * other polarity, next_split redirects it.  Hints are soft: a wrong hint
 * costs conflicts, never correctness, and the solver's learning overrides
 * persistently-wrong hints on its own.
 */
class PhaseHintModule : public PropagatorModule {
public:
    /// Replace the hint map (Z3 ast id -> preferred phase).
    void set_hints(std::unordered_map<unsigned, bool> hints) {
        hints_ = std::move(hints);
    }
    size_t hint_count() const { return hints_.size(); }

    void on_decide(const z3::expr& val, unsigned bit, bool is_pos) override {
        auto it = hints_.find(val.id());
        if (it != hints_.end() && it->second != is_pos) {
            host_->module_next_split(val, bit,
                                     it->second ? Z3_L_TRUE : Z3_L_FALSE);
        }
    }

    bool wants_decide_callback() const override { return true; }
    std::string get_name() const override { return "PhaseHint"; }

private:
    std::unordered_map<unsigned, bool> hints_;
};

/**
 * @brief Order-based dynamic phase hints (arm C of the §5.2 ladder).
 *
 * What survives insertions is the greedy plan's action multiset and
 * RELATIVE order, not its absolute steps.  The trace is a sequence of
 * levels (one per nonempty committed step; actions within a level are
 * exists-step peers).  At decision time the module computes the frontier
 * level — the first whose actions are not all placed on the solver's
 * current trail — and positively hints any decided action variable whose
 * action belongs to it, at WHATEVER timestep the solver is considering:
 * the hints slide along with repair insertions instead of going stale.
 * Lax policy: positive hints only — repairs (non-trace actions) and
 * order-jumps are never blocked, so the module re-orders effort, never
 * search space.  Placement state is recomputed from the shared action
 * trail (already push/pop-synchronized by the host) whenever its length
 * changes — no private undo machinery.
 */
class OrderHintModule : public PropagatorModule {
public:
    /// Replace the trace: levels of action ids, dive order (root first).
    void set_trace(std::vector<std::vector<int>> levels) {
        levels_ = std::move(levels);
        cached_trail_size_ = static_cast<size_t>(-1);  // force recompute
    }

    void register_timestep_variables(int timestep) override {
        if (!shared_ || !shared_->variable_factory || !shared_->problem) return;
        for (const Action& action : shared_->problem->actions()) {
            const z3::expr& av =
                shared_->variable_factory->get_action_variable(action, timestep);
            var_to_action_[av.id()] = action.id();
        }
    }

    void on_decide(const z3::expr& val, unsigned bit, bool is_pos) override {
        if (levels_.empty()) return;
        auto va = var_to_action_.find(val.id());
        if (va == var_to_action_.end()) return;
        refresh_frontier();
        if (frontier_level_ >= levels_.size()) return;  // trace fully placed
        if (frontier_remaining_.count(va->second) && !is_pos) {
            host_->module_next_split(val, bit, Z3_L_TRUE);
        }
    }

    bool wants_decide_callback() const override { return true; }
    std::string get_name() const override { return "OrderHint"; }

private:
    /// Recompute the frontier level from the shared action trail (only when
    /// its length changed — pops and pushes both invalidate).  Each trail
    /// entry consumes its action's next unconsumed occurrence in level
    /// order; the frontier is the first level not fully consumed.
    void refresh_frontier() {
        size_t trail_size = shared_->action_trail.size();
        if (trail_size == cached_trail_size_) return;
        cached_trail_size_ = trail_size;

        scratch_consumed_.assign(levels_.size(), 0);
        scratch_level_taken_.clear();
        scratch_occurrence_.clear();
        // Occurrence lists per action id, in level order (rebuilt lazily
        // into scratch_occurrence_ cursors keyed by action id).
        for (const auto& [aid, t] : shared_->action_trail) {
            int& cursor = scratch_occurrence_[aid];
            // Find this action's next occurrence at or after `cursor`.
            for (size_t lvl = static_cast<size_t>(cursor); lvl < levels_.size();
                 ++lvl) {
                bool in_level = false;
                for (int laid : levels_[lvl]) {
                    if (laid == aid) { in_level = true; break; }
                }
                if (in_level &&
                    !scratch_level_taken_.count({static_cast<int>(lvl), aid})) {
                    scratch_level_taken_.insert({static_cast<int>(lvl), aid});
                    scratch_consumed_[lvl]++;
                    cursor = static_cast<int>(lvl);
                    break;
                }
            }
        }
        frontier_level_ = levels_.size();
        for (size_t lvl = 0; lvl < levels_.size(); ++lvl) {
            if (scratch_consumed_[lvl] <
                static_cast<int>(levels_[lvl].size())) {
                frontier_level_ = lvl;
                break;
            }
        }
        frontier_remaining_.clear();
        if (frontier_level_ < levels_.size()) {
            for (int aid : levels_[frontier_level_]) {
                if (!scratch_level_taken_.count(
                        {static_cast<int>(frontier_level_), aid})) {
                    frontier_remaining_.insert(aid);
                }
            }
        }
    }

    struct PairHash {
        size_t operator()(const std::pair<int, int>& p) const {
            return std::hash<long long>()(
                (static_cast<long long>(p.first) << 32) ^
                static_cast<unsigned>(p.second));
        }
    };

    std::vector<std::vector<int>> levels_;
    std::unordered_map<unsigned, int> var_to_action_;
    size_t cached_trail_size_ = static_cast<size_t>(-1);
    size_t frontier_level_ = 0;
    std::unordered_set<int> frontier_remaining_;
    std::vector<int> scratch_consumed_;
    std::unordered_set<std::pair<int, int>, PairHash> scratch_level_taken_;
    std::unordered_map<int, int> scratch_occurrence_;
};

} // namespace rantanplan
