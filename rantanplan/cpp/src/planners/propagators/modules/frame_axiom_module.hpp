#pragma once

#include "../propagator_module.hpp"
#include "../propagator_strategy.hpp"
#include <z3++.h>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace rantanplan {

/**
 * @brief Lazy frame-axiom enforcement (the "-fp" lazy-frames propagator).
 *
 * Enforces the frame axioms by WATCHING them rather than posting them, so a
 * clause becomes a real Z3 lemma only at the moment it conflicts or goes unit.
 * The structure of a clause is stated ONCE, timelessly; only its state is kept
 * per transition.
 *
 * ---------------------------------------------------------------------------
 * THE CONSTRAINT
 * ---------------------------------------------------------------------------
 * For every fluent f with at least one modifier and every registered
 * transition t -> t+1:
 *
 *     (f^t != f^{t+1})  ->  OR_i ( a_i^t [ AND c_i^t ] )
 *
 * over the actions (and conditional-effect conditions) that can modify f, per
 * the encoder's epc_index_.  Fluents with no modifiers instead get an eagerly
 * asserted f^{t+1} == f^t.  That lives here, not in the encoder, because
 * GroundedEncoder::encode_frames() short-circuits to `true` under lazy frames.
 *
 * ---------------------------------------------------------------------------
 * THE WHOLE DESIGN IN THREE SENTENCES
 * ---------------------------------------------------------------------------
 * 1.  The clauses are described ONCE, timelessly:
 *   * clause_shapes_ + explainers_: say which actions can change which fluent
 *   * occurrences_: says the same thing transposed (given a variable, which
 *     clauses care). None of it grows with the horizon.
 * 
 * 2.  Every watched Z3 variable gets one SLOT holding its tri-state value and
 *     its z3::expr.  Slots sit in fixed-size per-timestep blocks, so
 *     addressing is multiplication, not a lookup table.
 *
 * 3.  Every (clause, transition) gets TWO COUNTERS.  Everything else about a
 *     clause at a timestep — whether the fluent changed, what state each
 *     explainer is in — is derived from slots on read, so backtracking only
 *     has to undo slot values.
 *
 * ---------------------------------------------------------------------------
 * WHAT A SLOT IS
 * ---------------------------------------------------------------------------
 * A SLOT is not a Z3 object.  It is an index into two parallel arrays:
 * slot_value_ (one int8_t, tri-state -1 unknown / 0 false / 1 true) and
 * slot_expr_ (the z3::expr, needed only when building a lemma).  Think of the
 * slots as one big table with ONE ROW PER TIMESTEP and a fixed set of columns;
 * a slot id is a (row, column) pair flattened to an integer.
 *
 *              |<------- one row = one timestep block ------->|
 *              +--------------+-------------------+-----------+
 *   timestep 0 | boolean      | action literals   | equality  |
 *   timestep 1 | fluent heads | condition literals| bits      |
 *   timestep 2 |              |                   |           |
 *      ...     +--------------+-------------------+-----------+
 *
 * Every row has the identical shape, which is what makes addressing pure
 * arithmetic (next section).  The four things a row tracks:
 *
 *   1. BOOLEAN FLUENT HEADS   f^t, one per boolean fluent some action can
 *      modify.  Example: at(t1,depot), at(t1,market), on(c1,depot), in(c1,t1).
 *   2. ACTION LITERALS        a^t, one per distinct action appearing in any
 *      clause.  Example: drive_dm, drive_md, teleport, load_d, unload_m.
 *   3. CONDITION LITERALS     one per distinct conditional-effect condition.
 *      Example: has_charge(t1), the guard on teleport's effect.
 *   4. EQUALITY BITS          one per numeric fluent some action can modify,
 *      meaning "f KEPT its value across t -> t+1".  Example: fuel(t1).
 *      Note this column of row t is really about the TRANSITION t -> t+1,
 *      while the other three are about timestep t itself.
 *
 * THE REIFICATION RULE — the part that reliably confuses people.
 *
 * Two of the four are watched directly and two are reified, and the rule is
 * NOT "booleans direct, numerics reified".  It is:
 *
 *      REIFY IF AND ONLY IF THE THING IS NOT ALREADY A BOOLEAN CONSTANT.
 *
 *   f^t of a boolean fluent   IS already a boolean constant
 *                             -> watched directly, nothing posted
 *   a^t of an action          IS already a boolean constant
 *                             -> watched directly, nothing posted
 *   an effect condition       is a boolean-TYPED arbitrary FORMULA, e.g.
 *                             (and (>= (fuel t1) 3) (at t1 depot))
 *                             -> fresh bool fc_<literal>_<t>, and
 *                                fc == <the condition> posted to the solver
 *   "did the numeric fluent   is likewise a formula, over numeric terms
 *    keep its value"          -> fresh bool feq_<shape>_<t>, and
 *                                feq == (f^t == f^{t+1}) posted
 *
 * So conditions and numeric equalities are handled IDENTICALLY — both reified,
 * for the same reason.  Boolean fluent heads and action variables are the
 * exceptions, because they already ARE atoms.  There is no trick that lets
 * conditions be watched un-reified; if there were, it would apply to numerics
 * too.  Both are built the same way, in activate_transition_slots().
 *
 * The underlying constraint: a Z3 user propagator can only be told about, and
 * only receives fixed() callbacks for, boolean atoms it has registered.
 * Reification manufactures an atom where the formula did not already provide
 * one, at the cost of one small biconditional posted once per (object,
 * timestep).
 *
 * ---------------------------------------------------------------------------
 * SLOT ADDRESSING — the one formula
 * ---------------------------------------------------------------------------
 * Every timestep gets an identically-shaped block of slots_per_timestep_
 * entries, so:
 *
 *     block for timestep t  =  slots [ t*SPT , (t+1)*SPT )
 *
 *   +----------------------+---------------------------+-------------------+
 *   |  boolean heads       |  literals                 |  equality bits    |
 *   |  [0, B)              |  [B, B+L)                 |  [B+L, B+L+N)     |
 *   |                      |                           |                   |
 *   |  f^t of boolean      |  actions    a^t   [0, A)  |  reified          |
 *   |  clause shape k      |  conditions c^t   [A, L)  |  (f^t == f^t+1)   |
 *   +----------------------+---------------------------+-------------------+
 *      B = num_boolean_shapes_    L = num_literals_    N = num_numeric_shapes_
 *
 *     boolean_head_slot (shape k, timestep t)   = t*SPT + k
 *     literal_slot      (literal l, timestep t) = t*SPT + B + l
 *     equality_bit_slot (shape k, transition t) = t*SPT + B + L + (k - B)
 *     clause_instance   (shape k, transition t) = t*num_shapes + k
 *
 * And the decode, which is why no per-slot metadata is stored:
 *
 *     t = slot / SPT ;   r = slot % SPT
 *     r <  B      -> a boolean head, shape   k = r
 *     r <  B + L  -> a literal,      literal l = r - B
 *     else        -> an equality bit, shape  k = B + (r - B - L)
 *
 * Clause shapes are sorted BOOLEANS FIRST, so "k < num_boolean_shapes_" is
 * the is-boolean test, a boolean shape's head offset is just k, and a numeric
 * shape's equality-bit offset is k - num_boolean_shapes_.  No lookup table
 * from head index back to shape is needed.
 *
 * WORKED EXAMPLE (the tiny world used throughout the struct comments below
 * and in developer-docs/frame-propagator.md): B=4, A=5, C=1 so L=6, N=1,
 * 5 shapes, SPT=11.  Block 0 is slots 0..10, block 1 is 11..21:
 *
 *     slot  0..3   at(t1,depot)@0  at(t1,market)@0  on(c1,depot)@0  in(c1,t1)@0
 *     slot  4..8   drive_dm@0  drive_md@0  teleport@0  load_d@0  unload_m@0
 *     slot  9      fc_5_0        (reified has_charge(t1)@0)
 *     slot 10      feq_4_0       (reified fuel(t1)@0 == fuel(t1)@1)
 *     slot 11..    the same for timestep 1
 *
 *   so drive_md at timestep 0 is slot 0*11 + 4 + 1 = 5, and at timestep 1 it
 *   is 1*11 + 4 + 1 = 16.  That subtraction is the entire addressing layer.
 *
 * A boolean head slot is SHARED: f^{t+1} of transition t and f^t of
 * transition t+1 are the same Z3 AST and the same slot.  That is what keeps
 * slot_of_ast_ a 1:1 map to a bare uint32 rather than to a list of roles, and
 * it is why fixing a head has to check two transitions.
 *
 * ---------------------------------------------------------------------------
 * DETECTION
 * ---------------------------------------------------------------------------
 * Per explainer, from its two slot values (an unconditional explainer reads
 * its condition as 1):
 *     can_explain := action == 1 AND (unconditional OR condition == 1)
 *     ruled_out   := action == 0 OR  (conditional AND condition == 0)
 * These are mutually exclusive; the gap is "pending", and
 *     INVARIANT: num_can_explain + num_ruled_out + #pending == n
 * holds at all times.  check_clause() fires on exactly three situations:
 *     all n ruled out,  fluent changed            -> conflict
 *     all n ruled out,  change unknown            -> force persistence
 *     n-1 ruled out, none can, fluent changed     -> unit-propagate survivor
 *
 * "Force persistence" does NOT post f^{t+1} == f^t for a boolean shape.  That
 * expression is boolean-sorted, so Z3 would internalize it as an iff node --
 * a fresh boolean plus four gate clauses, manufactured mid-search at a
 * non-base level.  Instead the equality is resolved here against whichever
 * head is already assigned and the resulting DIRECTED LITERAL is propagated,
 * with that head added to the reason; when neither head is assigned nothing
 * is emitted, and fixing either one brings us straight back here.  Numeric
 * shapes still propagate their reified equality bit, which is already an atom.
 * with early exits when the fluent provably persisted (clause vacuous) and
 * when num_can_explain > 0 (clause discharged).  AFTER that second guard,
 * num_can_explain == 0 holds — which is what licenses
 * propagate_last_explainer() to take the first pending explainer it finds.
 *
 * Reasons match the eager encoding's CNF: when a condition alone rules out an
 * explainer, the action literal is deliberately omitted (see build_reason).
 *
 * ---------------------------------------------------------------------------
 * BACKTRACKING — undo by replay, not by snapshot
 * ---------------------------------------------------------------------------
 * The only trailed state is the list of slot ids fixed since the mark.  To
 * undo one, re-run the counter update with old and new SWAPPED, then set the
 * slot back to -1.  Because the trail is walked in reverse, every slot fixed
 * after this one has already been restored, so the "other" value the update
 * reads is exactly what it was when this slot was fixed, and the inverse
 * cancels the forward step precisely.
 *
 * There is nothing else to repair: whether the fluent changed is derived on
 * read, so there is no cached value that could go stale and need rebuilding.
 *
 * >>> ATOMICITY REQUIREMENT.  A trailed event must apply its counter updates
 * >>> to its ENTIRE occurrence list, or replay will undo updates that were
 * >>> never applied.  So on_fixed must NOT return early from the literal loop
 * >>> when a conflict is raised — it suppresses the CHECKS and keeps applying
 * >>> the UPDATES.  (Note also that one action can own two explainers of the
 * >>> same shape, since epc_index_ appends per effect expression, so batching
 * >>> all updates before all checks would change the lemma stream.  Update
 * >>> and check per explainer; gate only the check.)
 *
 * ---------------------------------------------------------------------------
 * ASSUMPTIONS
 * ---------------------------------------------------------------------------
 *  (1) A fluent's explainer set is TIMESTEP-INVARIANT.  epc_index_ is built
 *      once from grounded actions with no timestep in its construction, and
 *      registration walks grounded_fluents() in identical order every call.
 *      A future encoder with timestep-dependent grounding breaks this and
 *      must not enable the module.
 *  (2) GroundedEncoder only — the module no-ops for any other encoder.
 *  (3) TRANSITIONS REGISTER CONTIGUOUSLY FROM 0.  The layout is stride
 *      arithmetic, so a first call with t=40 would fabricate 39 phantom
 *      blocks.  StrategyFactory::validate rejects this module with the CGB
 *      and DoubleTail planners (the only non-contiguous registrars), and
 *      register_transition() THROWS if the contract is violated anyway — a
 *      throw and not an assert, because a release build must fail loudly too.
 *  (4) Registration happens at base level, between check() calls.
 *
 * ---------------------------------------------------------------------------
 * MEASUREMENT
 * ---------------------------------------------------------------------------
 * cleanup() reports frame_prop.*, including the lazy-versus-eager accounting
 * this module exists to support: F_eager (what encode_frames() would have
 * posted, tallied at registration because it short-circuits) against F_lazy
 * (distinct clauses that ever produced a lemma, via ever_fired_).  Preserve
 * those keys — an active study consumes them.
 *
 * Compare against the eager encoding, not just against another lazy run: the
 * `-fp` strategies each mirror a non-`fp` counterpart (els-fp <-> els, and so
 * on) that differs only in that encode_frames() actually emits its axioms.
 *
 * Beware wall-clock: this module's reified bits change Z3's term ids and so
 * its search path, and solve time tracks the number of propagator callbacks
 * far more closely than it tracks this module's own cost.  The attributable
 * number is cost per callback,
 *     frame_prop.callback_ms * 1e6 / propagator.fixed_calls   (nanoseconds)
 * which is stable where wall-clock is not.  See developer-docs/
 * frame-propagator.md §8.
 */
class FrameAxiomModule : public PropagatorModule {
public:
    std::string get_name() const override { return "FrameAxiom"; }

    void initialize(PropagatorSharedState& shared, PropagatorStrategy& host) override;
    void on_push() override;
    void on_pop(unsigned num_scopes) override;
    void on_fixed(const z3::expr& ast, const z3::expr& value) override;
    void on_final() override;
    void register_timestep_variables(int timestep) override;
    void cleanup() override;

private:
    /// Sentinel in Explainer::cond_literal for an unconditional effect.
    static constexpr uint32_t kUnconditional = 0xFFFFFFFFu;

    // ######################################################################
    //
    //  RUNNING EXAMPLE for every struct comment below — the same world as
    //  developer-docs/frame-propagator.md.  One truck, two locations, one
    //  crate:
    //
    //    fluents   at(t1,depot)  at(t1,market)  on(c1,depot)  in(c1,t1)  bool
    //              fuel(t1)                                              numeric
    //              has_charge(t1)                      bool, NEVER in any effect
    //
    //    actions   drive_dm   not at(t1,depot), at(t1,market), fuel -= 1
    //              drive_md   not at(t1,market), at(t1,depot), fuel -= 1
    //              load_d     not on(c1,depot), in(c1,t1)
    //              unload_m   not in(c1,t1), on(c1,market)
    //              teleport   WHEN has_charge(t1): at(t1,depot), not at(t1,market)
    //
    //  Five fluents have modifiers, so there are 5 clause shapes (four
    //  boolean, then one numeric).  has_charge(t1) has none, so it is an
    //  unmodifiable fluent.  A = 5 action literals, C = 1 condition literal,
    //  so L = 6; slots_per_timestep_ = 4 + 6 + 1 = 11.
    //
    // ######################################################################

    // ======================================================================
    // TIMELESS — built once in initialize(), never mutated, and its size does
    // NOT depend on the horizon.  An eager encoding, or a per-transition clause
    // rebuilds all of it inside every per-transition clause object.
    // ======================================================================

    /// One disjunct of a frame clause: "this action, optionally guarded by
    /// this condition, could explain a change to the fluent".
    ///
    /// Both fields index the single LITERAL namespace, whose two halves are
    /// disjoint — [0, A) are actions and [A, A+C) are conditions — so when a
    /// literal is fixed, comparing it against action_literal is enough to tell
    /// which of the two roles it is playing.
    ///
    ///   explainers_[2] = the teleport disjunct of at(t1,depot):
    ///                    { action_literal = 2, cond_literal = 5 }
    ///   explainers_[0] = the drive_dm disjunct of at(t1,depot):
    ///                    { action_literal = 0, cond_literal = kUnconditional }
    struct Explainer {
        uint32_t action_literal;
        uint32_t cond_literal;    ///< kUnconditional iff the effect is unconditional
    };

    /// One frame clause SHAPE — a fluent, and the contiguous slice of
    /// explainers_ that can account for a change to it.
    ///
    ///   clause_shapes_[0] = { fluent_id = at(t1,depot),
    ///                         first_explainer = 0, num_explainers = 3 }
    ///                       i.e. explainers_[0..2]
    ///   clause_shapes_[4] = { fluent_id = fuel(t1),
    ///                         first_explainer = 9, num_explainers = 2 }
    ///
    /// There is no is_boolean field and no head index: shapes are sorted
    /// booleans-first, so "k < num_boolean_shapes_" IS the boolean test.
    struct ClauseShape {
        ExprID   fluent_id;
        uint32_t first_explainer;
        uint32_t num_explainers;
    };

    /// A (clause shape, explainer) coordinate — the payload of occurrences_.
    ///
    ///   ExplainerRef{ shape = 4, explainer = 9 }
    ///     = "the drive_dm disjunct of the fuel(t1) clause"
    ///   explainer is an ABSOLUTE index into explainers_, so reading it needs
    ///   no further arithmetic.
    struct ExplainerRef {
        uint32_t shape;
        uint32_t explainer;
    };

    /// Compressed adjacency: literal -> the explainers that use it.  This is
    /// the timeless clause description read backwards, and it answers "a
    /// variable was just fixed; which clauses care?".  ONE copy serves every
    /// timestep.
    ///
    ///   occurrences_ for the running example (6 literals):
    ///
    ///     bucket_start = [ 0,    3,    6,   8,   10,  11,  13 ]
    ///                      |     |     |    |    |    |    |
    ///                      lit0  lit1  lit2 lit3 lit4 lit5 end
    ///                      drv_dm      tele load unld has_charge
    ///                            drv_md
    ///
    ///     refs = [ (s0,x0) (s1,x3) (s4,x9)     <- drive_dm: three clauses
    ///              (s0,x1) (s1,x4) (s4,x10)    <- drive_md
    ///              (s0,x2) (s1,x5)             <- teleport
    ///              (s2,x6) (s3,x7)             <- load_d
    ///              (s3,x8)                     <- unload_m
    ///              (s0,x2) (s1,x5) ]           <- has_charge: the SAME two
    ///                                             explainers as teleport,
    ///                                             since each explainer is
    ///                                             listed under BOTH of its
    ///                                             literals
    ///
    ///   Walking literal l is
    ///       for (i = bucket_start[l]; i < bucket_start[l+1]; ++i) refs[i]
    ///   and that is the entire on_fixed dispatch for a literal.
    struct OccurrenceIndex {
        std::vector<uint32_t>     bucket_start;   ///< size = num_literals_ + 1
        std::vector<ExplainerRef> refs;
    };

    std::vector<ClauseShape> clause_shapes_;  ///< booleans first, then numerics
    std::vector<Explainer>   explainers_;     ///< flat; all shapes' disjuncts
    OccurrenceIndex          occurrences_;    ///< literal -> explainers using it

    /// What each literal index denotes, so registration can mint the Z3
    /// variable for it at a given timestep.
    ///
    ///   literal_actions_    = [ drive_dm, drive_md, teleport, load_d, unload_m ]
    ///                           ^literal 0                            ^literal 4
    ///   literal_conditions_ = [ has_charge(t1) ]
    ///                           ^literal 5  (index = num_action_literals_ + pos)
    ///   unmodifiable_fluents_ = [ has_charge(t1) ]
    ///                           ^ nothing can change it, so it is never a
    ///                             clause; it gets an eager f^{t+1} == f^t
    ///                             asserted once per transition
    std::vector<const Action*> literal_actions_;
    std::vector<ExprID>        literal_conditions_;
    std::vector<ExprID>        unmodifiable_fluents_;

    /// Layout constants, all fixed once clause_shapes_ is built.
    /// Running example: 4 boolean shapes, 5 action + 1 condition literals,
    /// 1 numeric shape, so 11 slots per timestep.
    uint32_t num_boolean_shapes_   = 0;   ///< B; also the "is this shape boolean" test
    uint32_t num_action_literals_  = 0;   ///< A
    uint32_t num_condition_literals_ = 0; ///< C
    uint32_t num_literals_         = 0;   ///< L = A + C
    uint32_t num_numeric_shapes_   = 0;   ///< N
    uint32_t slots_per_timestep_   = 0;   ///< B + L + N — one block's size

    // ======================================================================
    // SLOTS — one per watched Z3 variable, addressed by the stride formula.
    // These vectors are parallel; index both with the same slot id.
    // ======================================================================

    /// slot_value_ is the ONLY trailed state in the module.
    /// slot_expr_ is read only when a lemma is actually being built.
    ///
    ///   block 0 of the running example, freshly registered:
    ///
    ///     slot  slot_expr_                     slot_value_
    ///     ----  ---------------------------    -----------
    ///        0  at(t1,depot)@0                     -1
    ///        1  at(t1,market)@0                    -1
    ///        2  on(c1,depot)@0                     -1
    ///        3  in(c1,t1)@0                        -1
    ///        4  drive_dm@0                         -1
    ///        5  drive_md@0                         -1
    ///        6  teleport@0                         -1
    ///        7  load_d@0                           -1
    ///        8  unload_m@0                         -1
    ///        9  fc_5_0   (has_charge(t1)@0)       -1
    ///       10  feq_4_0  (fuel@0 == fuel@1)       -1
    ///
    /// The tail block (timestep num_transitions_) exists with live BOOLEAN
    /// HEAD slots only — it is the "after" side of the last transition.  Its
    /// literal and equality-bit slots hold a placeholder expression and are
    /// provably never read: every reader either touches a boolean head, or
    /// touches a block belonging to a registered transition.
    std::vector<int8_t>   slot_value_;   ///< -1 unknown / 0 false / 1 true
    std::vector<z3::expr> slot_expr_;

    /// AST id -> slot, as a FLAT ARRAY rather than a hash map.
    ///
    /// Measured: every variable Z3 fixes in an -fp run is one of ours
    /// (frame_prop.on_fixed_calls / propagator.fixed_calls = 1.000, p10 0.996),
    /// so this lookup runs on 100% of callbacks and never misses.  A hash of the
    /// id plus a modulo plus a bucket chase is pure loss against one indexed
    /// load, so the map is gone.
    ///
    /// Z3 hands out ast ids from a dense counter, so the array stays small
    /// relative to the formula; ids of REGISTERED expressions cannot be recycled
    /// underneath us because slot_expr_ holds a live reference to each one.
    /// Unregistered ids either fall past the end or read kNoSlot.
    static constexpr uint32_t kNoSlot = 0xFFFFFFFFu;
    std::vector<uint32_t> slot_of_ast_;

    // ======================================================================
    // CLAUSE STATE — two counters per (clause shape, transition), at index
    // t*num_shapes + k.  4 bytes.  Nothing else is stored: whether the fluent
    // changed, and each explainer's state, are derived from slots.
    // ======================================================================

    /// clause_counters_[0] is clause_shapes_[0] (at(t1,depot)) at transition 0.
    /// Mid-trace, after drive_md@0 and the teleport condition are falsified:
    ///
    ///   { num_ruled_out = 2, num_can_explain = 0 }
    ///     ^ two of three explainers dead; with n = 3 that is n-1, so if the
    ///       fluent is known to have changed, this clause unit-propagates.
    struct ClauseCounters {
        uint16_t num_ruled_out   = 0;
        uint16_t num_can_explain = 0;
    };

    std::vector<ClauseCounters> clause_counters_;
    std::vector<uint8_t>        ever_fired_;   ///< backs frame_prop.f_lazy; NOT trailed
    /// One bit per clause instance: was this fluent EVER observed to change
    /// across its transition?  Backs frame_prop.f_changed.  Not trailed --
    /// the question is "ever, anywhere in the search", so it is monotone.
    /// This is the set a change-triggered propagator would have to
    /// materialise; F_lazy counts persistence firings too, so it is NOT the
    /// same set and is an overestimate for that purpose.
    std::vector<uint8_t>        ever_changed_;
    int                         num_transitions_ = 0;  ///< 0..n-1 are registered

    // ======================================================================
    // TRAIL — slot ids, undone by reverse-delta replay.
    // ======================================================================

    /// fixed_slots_ holds the slots fixed since the enclosing push;
    /// level_marks_ holds one fixed_slots_ length per push.
    ///
    ///   level_marks_ = [ 2 ]              // this level opened at length 2
    ///   fixed_slots_ = [ ..., 5, 9 ]      // then slots 5 and 9 were fixed
    ///
    ///   on_pop(1): undo slot 9 (replay its counter updates backwards, set the
    ///              value to -1), then slot 5, then truncate.  No sort, no
    ///              rescan, and nothing derived to repair.
    ///
    /// Below the first push nothing can ever be undone, so base-level
    /// assignments are deliberately not trailed at all.
    std::vector<uint32_t> fixed_slots_;
    std::vector<uint32_t> level_marks_;

    // ======================================================================
    // Cached externals
    // ======================================================================

    GroundedEncoder* grounded_ = nullptr;

    // ======================================================================
    // Statistics (frame_prop.*)
    // ======================================================================

    int frame_conflict_count_ = 0;
    int frame_persistence_prop_count_ = 0;
    int frame_unit_prop_count_ = 0;
    int frame_on_fixed_count_ = 0;
    int frame_on_final_count_ = 0;
    int frame_final_conflict_count_ = 0;
    int frame_final_propagation_count_ = 0;
    int frame_vars_registered_ = 0;

    long long frame_clauses_materialized_ = 0;
    long long frame_eager_axioms_ = 0;
    long long frame_eager_literals_ = 0;
    long long frame_static_persistence_ = 0;
    size_t    fixed_slots_peak_ = 0;

    // DIAGNOSTIC: which firing branch check_clause reaches.  Kept because
    // C12 (see developer-docs/c12-change-triggered-propagator.md) needs the
    // split to predict what a change-triggered scheme would emit.
    long long frame_clause_visits_ = 0;         ///< forward clause examinations
    long long frame_changed_instances_ = 0;     ///< |{(shape,t) ever seen changed}|
    // DIAGNOSTIC: callbacks split by slot class, to price what watching the
    // STATE side (heads + equality bits) actually costs against watching only
    // the ACTION side (action + condition literals).
    long long frame_fixed_head_ = 0;
    long long frame_fixed_literal_ = 0;
    long long frame_fixed_eqbit_ = 0;
    long long frame_reach_conflict_ = 0;        ///< all n ruled out AND fluent changed
    long long frame_reach_unit_persist_ = 0;    ///< all n ruled out, change unknown
    long long frame_reach_unit_explainer_ = 0;  ///< n-1 ruled out AND fluent changed

    double frame_callback_ms_ = 0.0;
    double frame_pop_ms_ = 0.0;   ///< split out of callback_ms: replay's cost

    // ======================================================================
    // Helpers
    // ======================================================================

    void build_clause_shapes();
    void emit_clause_shape(
        ExprID fluent_id,
        const std::vector<std::pair<const Action*, const EffectExpression*>>& effects,
        std::unordered_map<int, uint32_t>& literal_of_action,
        std::unordered_map<ExprID, uint32_t>& position_of_condition);
    void register_transition(int t);
    void ensure_timestep_block(int t);      ///< live heads, placeholder remainder
    void activate_transition_slots(int t);  ///< make block t's literals + equality bit live
    void bind_slot(uint32_t slot, const z3::expr& e);

    // --- addressing: the stride formula, in one place ---
    uint32_t timestep_base(int t) const {
        return static_cast<uint32_t>(t) * slots_per_timestep_;
    }
    uint32_t boolean_head_slot(uint32_t shape, int t) const {
        return timestep_base(t) + shape;
    }
    uint32_t literal_slot(uint32_t literal, int t) const {
        return timestep_base(t) + num_boolean_shapes_ + literal;
    }
    uint32_t equality_bit_slot(uint32_t shape, int t) const {
        return timestep_base(t) + num_boolean_shapes_ + num_literals_
             + (shape - num_boolean_shapes_);
    }
    uint32_t clause_instance(uint32_t shape, int t) const {
        return static_cast<uint32_t>(t)
             * static_cast<uint32_t>(clause_shapes_.size()) + shape;
    }
    bool is_boolean_shape(uint32_t shape) const { return shape < num_boolean_shapes_; }
    bool is_registered(int t) const { return t >= 0 && t < num_transitions_; }

    // --- derived state: all pure reads of slot_value_ ---
    int8_t action_value(const Explainer& x, int t) const {
        return slot_value_[literal_slot(x.action_literal, t)];
    }
    int8_t condition_value(const Explainer& x, int t) const {
        return x.cond_literal == kUnconditional
             ? static_cast<int8_t>(1)
             : slot_value_[literal_slot(x.cond_literal, t)];
    }
    /// Did the fluent keep its value across the transition?
    ///   1 persisted · 0 changed · -1 not yet known.   Derived, never cached.
    int8_t persistence_state(uint32_t shape, int t) const;

    static bool can_explain(bool conditional, int8_t action, int8_t cond) {
        return action == 1 && (!conditional || cond == 1);
    }
    static bool ruled_out(bool conditional, int8_t action, int8_t cond) {
        return action == 0 || (conditional && cond == 0);
    }

    void update_counters(uint32_t instance, const Explainer& x,
                         int8_t old_action, int8_t new_action,
                         int8_t old_cond,   int8_t new_cond);
    void visit_literal(uint32_t literal, int t, int8_t old_value, int8_t new_value,
                       bool run_checks);
    void check_clause(uint32_t shape, int t);

    void build_reason(z3::expr_vector& reason, uint32_t shape, int t,
                      bool include_change, int64_t skip_explainer);
    void report_conflict(uint32_t shape, int t);
    /// Force the fluent to keep its value across t -> t+1.  For a boolean shape
    /// this resolves the equality against whichever head is already assigned and
    /// propagates the resulting directed literal; it returns false, having
    /// emitted nothing, when neither head is assigned yet.  Numeric shapes always
    /// propagate their equality bit and always return true.
    bool propagate_persistence(uint32_t shape, int t);
    void propagate_last_explainer(uint32_t shape, int t);
    void mark_fired(uint32_t instance);
};

} // namespace rantanplan
