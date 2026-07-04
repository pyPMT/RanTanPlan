# Encoders and planners

This doc covers the two axes of a `StrategySpec` not already covered
elsewhere: which SAT encoder builds the formula, and which planner backend
drives the search over it. See
[`preprocessing-and-strategy.md`](preprocessing-and-strategy.md) for how
these fit into `StrategySpec`/`StrategyFactory`, and
[`interference-and-propagators.md`](interference-and-propagators.md) for the
interference/propagator axes.

## Encoder families

`BaseEncoder` (`encoders/base_encoder.hpp`) is the interface every encoding
strategy implements: `encode_initial_state()`, `encode_actions(t)`,
`encode_frames(t)`, `encode_goal(t)`, `encode_parallelism(t)`,
`encode_symmetries(t)`, `encode_state_constraints(t)`, plus
`extract_plan()` to turn a Z3 model back into a `Plan`. Three families exist
(`EncoderFamily` in `config/strategy_spec.hpp`):

- **Grounded** (`grounded_encoder.{hpp,cpp}`) — the baseline: one set of
  clauses per ground action per timestep, generated directly.
- **Chained** (`chained_grounded_encoder.{hpp,cpp}`) — frame-axiom chaining
  that trades some encoding structure for reuse. This is the encoder behind
  the `elsc`/`elsc-dt`/`pdla`/`pdla-sel`/`pdla-fg`/`elsc-fp` family and the
  `flsc`/`flsc-dt`/`fgsc`/`egsc` family — a single encoder implementation
  serves both sequential-planner and PDLA consumers, which is the strategy
  pattern's component-reuse payoff described in
  [`architecture-overview.md`](architecture-overview.md): the contributor
  unit is "one encoder," not "one encoder-planner pair."
- **R2E** (`r2e_grounded_encoder.{hpp,cpp}`) — relaxed-to-exact encoding
  (`r2e` strategy).

One encoder-level detail worth knowing if you touch cost handling:
`convert_cost_to_z3()` has a default (evaluate the action's cost expression
at the start-of-timestep state) that `R2EGroundedEncoder` overrides, because
under R2E's fixed action ordering each action actually observes a
chain-substituted intermediate state, not the raw start-of-timestep state —
using the default there would silently miscompute SDAC (state-dependent
action cost) costs.

`encode_prefix_monotone(t)` is specific to `IncrementalHorizonPlanner`: it
encodes `(∀a. ¬a@t) → (∀a. ¬a@(t+1))`, forcing active steps to the front of
the plan. Combined with the horizon-schedule batching described below, this
is what lets a single goal literal at a batched horizon `h` witness any
shorter plan length in the batch — frame axioms propagate a shorter plan's
final state forward to `h` unchanged. `DoubleTailPlanner` doesn't use it: its
forward and backward stacks are structurally disjoint, so there's no single
uniform "front" to chain.

## Planner backends

`BasePlanner` (`planners/base_planner.hpp`) owns what every planner shares:
the Z3 solver, deadline/timeout handling (`init_deadline`,
`apply_solver_timeout`, `handle_unknown_result` — a `z3::unknown` result is
disambiguated into "genuinely timed out" vs. other causes), the horizon
schedule, and the intermediate-plan callback used for anytime results (see
[`python-cpp-boundary.md`](python-cpp-boundary.md)). Concrete planners:

- **`IncrementalHorizonPlanner`** (named `Sequential` in `PlannerKind` —
  the name predates this class and now refers to a strategy role, not a
  class) — satisficing search with a growing horizon, one SAT call per
  scheduled horizon.
- **`DoubleTailPlanner`** — the double-tail incremental encoding from
  Gocht & Balyo, *"Accelerating SAT Based Planning with Incremental SAT
  Solving"* (ICAPS 2017): builds the formula from **both** ends at once
  (forward from the initial state, backward from the goal), alternating
  which end extends each iteration and connecting the two with link
  constraints; iteration `i` tests for plans of exactly length `i`. Because
  it has no single linear horizon to schedule non-uniformly, it's
  incompatible with non-linear horizon schedules (`Config::validate()`
  enforces this) and cannot run under optimal/anytime search modes
  (`resolve_planner_kind` raises if you try).
- **`BranchAndBoundPlanner`** — the cost-optimal (and anytime) planner.
  Notably, it does **not** use `z3::optimize` — that API is incompatible
  with Z3 user propagators, which every forall/exists/lazy strategy
  depends on. Instead it reformulates cost minimization as a sequence of
  plain `z3::solver` feasibility checks with progressively tighter cost
  bounds (an abstract suffix lower bound plus an inner branch-and-bound
  loop per horizon), preserving full propagator compatibility.
- **PDLA family** (`PDLAPlanner`, selective and fine-grained variants) —
  *Property-Directed lazy Activation*, a core-guided lazy-activation loop
  under ∃-step semantics inspired by IC3/PDR and CEGAR: actions are
  activated (given a blocking literal usable at every timestep) only when
  an unsatisfied-precondition "obligation," chained backward from the goal
  through the achiever graph, demands one. This is why PDLA is the planner
  that consumes the achiever analysis artifact from `PreprocessedProblem`
  (via `consume_artifacts`) — it needs the achiever graph to drive
  obligation chaining, not just for a one-off heuristic. Like double-tail,
  it cannot run under optimal/anytime modes.

## Search modes and how a planner gets picked

`SearchMode` (satisficing / optimal / anytime) is a CLI/`Config`-level
choice, orthogonal to the strategy's own declared `PlannerKind`.
`resolve_planner_kind(mode, spec)` (`config/strategy_spec.hpp`) reconciles
the two: satisficing mode uses whatever planner the strategy names;
optimal and anytime modes always force `BranchAndBoundPlanner`, and raise
`std::invalid_argument` up front if the strategy is double-tail or PDLA —
both of those are structurally satisficing-only, not something the search
loop could accommodate.

**Example.** Four `(mode, strategy)` combinations and what
`resolve_planner_kind` does with each:

| `SearchMode` | strategy's `PlannerKind` | Result |
|---|---|---|
| Satisficing | `Sequential` (e.g. `"elsc"`) | `Sequential` — unchanged, runs as `IncrementalHorizonPlanner` |
| Satisficing | `DoubleTail` (e.g. `"elsc-dt"`) | `DoubleTail` — unchanged, runs as `DoubleTailPlanner` |
| Optimal | `Sequential` (e.g. `"seq --mode optimal"`) | `BranchAndBound` — forced, regardless of the strategy's own planner |
| Optimal or Anytime | `DoubleTail` or any `PDLA*` | throws `std::invalid_argument` at resolution time, before any solving starts |

The forcing in row 3 is why `"elsc"` under `--mode optimal` and `"elsc"`
under `--mode satisficing` run entirely different planner classes despite
being the same `StrategySpec` — cost-optimality is a search-algorithm
property, not an encoding one, so it overrides the strategy's default
rather than combining with it.

## The horizon schedule

`HorizonSchedule` (`planners/base_planner.hpp`) controls which horizons
actually get a SAT call versus which just get their transitions filled in
without a goal check: `linear` (every horizon, +1), `arithmetic` (+5),
`geometric` (×1.5), `doubling` (×2). Skipping goal checks at intermediate
horizons is safe *only* because the prefix-monotone encoding guarantees any
shorter valid plan is representable within a later batch's horizon — the
schedule changes how often you ask "is there a plan yet," not whether a
shorter plan would be missed. This is also the concrete reason non-linear
schedules and `DoubleTailPlanner` don't mix: double-tail's iteration `i`
*is* "test for plans of exactly length `i`" — there's no batching concept
to schedule non-linearly.

**Example.** `HorizonSchedule::next(current, max)` always advances by at
least 1 and never exceeds `max`. For `doubling` (`n = max(current+1,
current*2)`) starting from horizon 0 with `max = 20`, the sequence of
horizons that actually get a goal check is:

```
0 → 1 → 2 → 4 → 8 → 16 → 20
```

(`next(16, 20)` computes `max(17, 32) = 32`, then clamps to `min(32, 20) =
20`.) Transitions for timesteps 0 through 19 are still all added to the
solver as the schedule advances — only the *goal literal* is skipped at 3,
5, 6, 7, 9–15, 17–19. If a plan of length 6 exists, the solver won't be
asked "is there a plan at horizon 6?" directly; it gets asked at horizon 8
instead, and finds it there because the prefix-monotone encoding lets a
length-6 plan sit at the front of an 8-slot formula with 2 empty trailing
steps.
