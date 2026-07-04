# Interference analysis and propagators

## What interference means here

Two actions **interfere** if executing them "together" (in the same
parallel step) would be unsound — one's effects would clobber the other's
precondition or effect, or otherwise change what the other observes.
Sequential semantics doesn't need this at all (one action per timestep, by
definition nothing runs alongside anything else); ∀-step and ∃-step both
need to know, for pairs or groups of actions, whether they can safely share
a timestep — see
[`architecture-overview.md`](architecture-overview.md) (see "Why three
parallelism semantics") for why those semantics exist at all.

**Interference is directional.** `has_interference(A, B)` asks whether
executing `A` would prevent `B` from executing correctly, or corrupt what
`B` reads or writes — not whether the pair interferes symmetrically.
`has_interference(A, B) != has_interference(B, A)` in general. This shows
up again in `topological_sort_actions()`, which returns actions in
*execution order*, not standard topological order: if `A` interferes with
`B`, `B` must appear before `A` in the result, because executing in that
order means `A`'s effects land after `B` has already read/acted on the
pre-`A` state.

## Two interfaces, not one, and it's a compile-time distinction

`analysis/interference_analysis.hpp` defines two interfaces:

- **`InterferenceOracle`** — the universal surface every implementation
  provides: `has_interference()` (by `Action` or by node ID) and
  `topological_sort_actions()`. Both eager and lazy analyzers satisfy this.
- **`InterferenceGraphAnalysis : InterferenceOracle`** — adds whole-graph
  access: `get_interference_graph()`, `get_neighbours()`,
  `output_interference_graph_dot()`. **Only eager implementations derive
  from this.**

```mermaid
classDiagram
    class InterferenceOracle {
        <<interface>>
        +has_interference(a1, a2) bool
        +topological_sort_actions(actions) vector
    }
    class InterferenceGraphAnalysis {
        <<interface>>
        +get_interference_graph() Graph
        +get_neighbours(node_id) vector
    }
    InterferenceOracle <|-- InterferenceGraphAnalysis
    InterferenceGraphAnalysis <|-- EagerInterferenceAnalysis
    InterferenceGraphAnalysis <|-- EagerSemanticInterferenceAnalysis
    InterferenceOracle <|-- LazyInterferenceAnalysis
    InterferenceOracle <|-- SemanticInterferenceAnalysis
```

Why split it at all: a consumer that needs neighbour lists or the whole
graph (the ∀-step encoding, the eager forall-mutex propagator module)
*cannot function* against a lazy analyzer — there's no graph to hand back.
An earlier version of this code had one interface with graph methods that
*threw at runtime* on lazy implementations, guarded by a runtime `is_lazy()`
flag consumers were supposed to check first. The split turns that into a
compile error: `ForallMutexModule` takes a `const InterferenceGraphAnalysis&`
at construction, so passing a lazy analyzer there simply doesn't compile.
This is the same "make illegal states unrepresentable" instinct mentioned in
[`architecture-overview.md`](architecture-overview.md) (see "Design philosophy").

## The eager/lazy × syntactic/semantic matrix

`InterferenceKind` has five values — `None` (sequential only) and a 2×2
matrix of eager/lazy crossed with syntactic/semantic:

| | Syntactic | Semantic |
|---|---|---|
| **Eager** | `EagerInterferenceAnalysis` — builds the full graph once, using fast fluent-set-intersection checks | `EagerSemanticInterferenceAnalysis` — builds the full graph once, using Z3-checked precision |
| **Lazy** | `LazyInterferenceAnalysis` — queries pairs on demand, syntactic checks | `SemanticInterferenceAnalysis` — queries pairs on demand, Z3-checked |

**Syntactic** interference is a sound *overapproximation*: it intersects
precondition/effect fluent sets (positive/negative boolean, numeric,
conditional-effect fluents, numeric-effect RHS dependencies — see
`InterferenceOracle::ActionAnalysis` for the exact sets collected per
action) and may report interference for pairs that don't actually conflict
semantically, but never misses a pair that does. It's cheap: no Z3 calls.

**Semantic** interference asks Z3 directly whether the pair can genuinely
conflict, which is precise but costlier per query. Lazy analyzers trade
build-once cost for per-query cost — worthwhile when the encoding only ever
asks about a subset of all pairs (e.g. `elsc`/`elsc-dt`, the chained-encoder
strategies, use lazy semantic interference).

One documented soundness-relevant scheduling detail: eager-semantic
interference is built *after* `Config::set_all_integer` runs on a problem
where static-fluent folding made costs constant, so its Z3 queries use `Int`
sorts rather than mixed sorts — this yields fewer spurious edges (still
sound, just more precise) than building it earlier in the pipeline would.

**Example: where syntactic and semantic disagree.** Take two actions that
both increase the same numeric fluent: `refuel_a` does
`fuel(truck1) := fuel(truck1) + 5` and `refuel_b` does
`fuel(truck1) := fuel(truck1) + 3`. Syntactic analysis sees `fuel(truck1)`
in both actions' `numeric_effects` sets and — conservatively, without
reasoning about *what* the effects do — flags the pair as interfering: two
writers to the same fluent look unsafe to run in the same step. Semantic
analysis instead asks Z3 whether the two orderings actually differ:
`(fuel + 5) + 3 == (fuel + 3) + 5` is a tautology, so Z3 proves they commute
and reports no real interference. The difference has the most bite under ∀-step, where pairwise interference
blocks co-scheduling outright with no escape valve: syntactic interference
would permanently forbid `refuel_a` and `refuel_b` from ever sharing a
timestep, while semantic interference lets them run together freely. (Under
∃-step the gap is narrower — a merely-commuting pair like this one already
admits *some* execution order, so the cycle-detection propagator wouldn't
block it either way; semantic precision matters most there for pairs where
no order is obviously safe on paper but Z3 can still prove one is.) This is
the concrete payoff of paying Z3's per-query cost — more parallelism, hence
shorter plans, on exactly the pairs where the cheap check is too coarse.

## Propagators: one Z3 slot, many composable modules

Z3 allows exactly **one** `user_propagator_base` per solver. Rather than
writing one monolithic propagator per strategy, `PropagatorStrategy`
(`planners/propagators/propagator_strategy.hpp`) owns that one slot and
dispatches each Z3 callback (`push`, `pop`, `fixed`, `decide`, `final`) to
an ordered list of `PropagatorModule`s:

```mermaid
classDiagram
    class PropagatorStrategy {
        <<z3::user_propagator_base>>
        +add_module(module)
        +push() / pop() / fixed() / decide() / final()
        -modules_ : vector~PropagatorModule~
        -shared_ : PropagatorSharedState
    }
    class PropagatorModule {
        <<interface>>
        +on_push() / on_pop() / on_fixed() / on_decide() / on_final()
        +wants_decide_callback() bool
        +manages_parallelism_constraints() bool
    }
    class PropagatorSharedState {
        +action_trail
        +potential_interferers
        +build_footprint_index()
    }
    PropagatorStrategy o-- "many" PropagatorModule
    PropagatorStrategy o-- PropagatorSharedState
    PropagatorModule <|-- ForallMutexModule
    PropagatorModule <|-- ExistsCycleModule
    PropagatorModule <|-- FrameAxiomModule
    PropagatorModule <|-- LoggingModule
```

`PropagatorSharedState` holds what every module might need to read: the
action trail (which actions are fixed true, and at which decision level, for
`push`/`pop` bookkeeping), and a **footprint index** —
`potential_interferers`, a precomputed map from each action to the other
actions that share a fluent with it — built lazily once and shared
read-only, so modules don't each redo the same fluent-overlap scan. Each
module additionally keeps its own private trail for its own concern (e.g.
which edge literals it introduced), rather than polluting the shared state
with per-module bookkeeping.

**Conflict short-circuit**: when any module calls `module_conflict()`
(routed through `PropagatorStrategy` rather than Z3's API directly, so the
host can track it), the remaining modules are skipped for that callback
invocation. Propagations (unit implications via `module_propagate()`) do
*not* short-circuit — multiple modules can each propagate independently
within one callback.

**Example.** Say a strategy runs `{ExistsCycle, FrameAxiom}` together (like
`elsc-fp`). The solver fixes three action variables true at the same
timestep — `A`, `B`, `C` — where `A` interferes with `B`, `B` interferes
with `C`, and `C` interferes with `A`: a genuine 3-cycle, meaning no
execution order of the three is safe (each would need to come both before
and after another). `ExistsCycleModule::on_fixed()` runs its DFS over the
interference subgraph of currently-active actions, finds the cycle, and
calls `module_conflict({A, B, C})`. `PropagatorStrategy` sets
`conflict_raised_ = true` and returns without dispatching `on_fixed()` to
`FrameAxiomModule` for this callback — there is no point computing frame
axioms for a partial assignment Z3 is about to backtrack out of anyway.
Had `ExistsCycleModule` instead found only a safe partial order (no cycle),
it would call nothing, and `FrameAxiomModule::on_fixed()` would run
normally afterward.

`wants_decide_callback()` gates whether the (expensive) Z3 decide callback
gets registered at all — only modules that actually need `on_decide`
notifications should return `true`, since every module pays the cost of the
callback being registered if any one of them needs it.

### The module kinds

`PropagatorModuleKind` (`config/strategy_spec.hpp`) lists the composable
units: `ForallEager` and `ForallLazy` (mutex-based blocking for ∀-step,
differing in whether they consume an eager or lazy interference graph),
`ExistsCycle` (cycle detection for ∃-step — a candidate parallel step is
valid iff its interference subgraph is acyclic), and `FrameAxiom` (supplies
frame axioms lazily from the propagator instead of the encoding, for the
`elsc-fp` strategy). A `StrategySpec`'s `propagator` field is a
`std::set<PropagatorModuleKind>` — strategies can combine modules (e.g.
`elsc-fp` uses `{ExistsCycle, FrameAxiom}` together).
