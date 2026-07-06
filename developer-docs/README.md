# RantanPlan developer documentation

This is the onboarding path for contributors: the design philosophy and the
reasoning behind it, not just the API surface. For day-to-day commands
(build, test, benchmark) and the file-by-file architecture reference, see the
project root [`CLAUDE.md`](../CLAUDE.md) — these docs complement it rather
than duplicate it.

## Reading order

Concepts first, plumbing last — each doc leans only on the ones before it.

1. [`architecture-overview.md`](architecture-overview.md) — what RantanPlan
   is, why it's built as two layers (Python frontend / C++ core, and within
   the core, preprocessor / backend), and the philosophy that shapes
   everything else here.
2. [`the-encoding.md`](the-encoding.md) — the formula itself: which Z3
   variables exist, the clause schemas every encoder emits (actions, frame
   axioms, parallelism, goals), and the complete formula for a two-room
   robot problem. Everything after this doc exists to build or exploit
   that formula.
3. [`preprocessing-and-strategy.md`](preprocessing-and-strategy.md) — the
   pass pipeline, the `PreprocessedProblem` contract, invariant and
   symmetry discovery, and the strategy registry/factory that assembles a
   backend.
4. [`grounding.md`](grounding.md) — how lifted actions become ground ones,
   restricted to what's reachable, via join-based matching instead of a
   naive cross product.
5. [`relaxed-planning-graph.md`](relaxed-planning-graph.md) — the
   layer-by-layer reachability graph `ActionPruningPass` builds for
   tightened bounds, goal-relevance pruning, and cost lower bounds.
6. [`interference-and-propagators.md`](interference-and-propagators.md) —
   why interference analysis has two interfaces, and how the Z3 user
   propagator is split into composable modules.
7. [`encoders-and-planners.md`](encoders-and-planners.md) — the encoder
   families, the planner backends, optimal/anytime branch-and-bound, and
   how a search mode resolves to a planner.
8. [`z3-control-and-interruption.md`](z3-control-and-interruption.md) — how
   the Z3 solver is driven incrementally, timeout budgeting, and cross-thread
   cancellation.
9. [`python-cpp-boundary.md`](python-cpp-boundary.md) — the in-process
   nanobind contract: `ProblemIR`, the GIL, the anytime callback,
   cancellation from Python. Plumbing — nothing conceptual depends on it,
   but touch `converter.py` or `bindings.cpp` and this is the doc.

Each doc includes a Mermaid diagram of its central flow — GitHub and most
Markdown viewers render these inline — and ends with a **Try it** box: a
small runnable command or snippet that makes the doc's subject observable
on a real instance.

## Glossary

The docs define each term where it first matters; this is the lookup table
for when you meet one out of order.

- **Achiever** — a ground action that can make a given fact true.
  `AchieversAnalysis` precomputes, for every fact, its possible achievers;
  the PDLA planners chain backward through this graph to decide which
  actions to activate ([`encoders-and-planners.md`](encoders-and-planners.md)).
- **Anytime** — search mode that reports every improving plan as it's
  found, converging toward optimal. Runs on the branch-and-bound planner
  ([`encoders-and-planners.md`](encoders-and-planners.md)).
- **CWA (closed-world assumption)** — anything not stated true in the
  initial state is false. A preprocessing pass makes this explicit so the
  encoder can assert a value for *every* fluent at timestep 0
  ([`the-encoding.md`](the-encoding.md)).
- **Delete relaxation** — pretend actions only produce and never destroy;
  facts then accumulate monotonically, turning reachability into a cheap
  fixpoint instead of a search. Underlies both the grounder and the RPG
  ([`grounding.md`](grounding.md), [`relaxed-planning-graph.md`](relaxed-planning-graph.md)).
- **∀-step / ∃-step** — the two parallel-plan semantics: any pairwise
  non-interfering subset may share a timestep (∀), or any subset with *some*
  valid execution order may (∃)
  ([`architecture-overview.md`](architecture-overview.md)).
- **Frame axiom** — the clause asserting a fluent keeps its value across a
  timestep unless an action that can change it actually fired; without
  them the solver could "wish" the goal true ([`the-encoding.md`](the-encoding.md)).
- **Ground / lifted** — a lifted action has typed parameters
  (`move ?r ?from ?to`); a ground action is a full instantiation
  (`move(rob1, room-a, room-b)`). Grounding is the step from one to the
  other ([`grounding.md`](grounding.md)).
- **Horizon** — the plan length the current formula can express: "is there
  a plan of at most *h* steps?" Search grows the horizon until Z3 says sat.
- **Interference** — directional relation between actions: A interferes
  with B if executing A alongside B could corrupt what B reads or writes
  ([`interference-and-propagators.md`](interference-and-propagators.md)).
- **Mutex / invariant / conservation law** — facts provably true in every
  reachable state: at most one of a fluent group holds (mutex), a numeric
  bound always holds, a sum of fluents stays constant (conservation law).
  Discovered by the invariant oracle, asserted at every timestep
  ([`preprocessing-and-strategy.md`](preprocessing-and-strategy.md)).
- **Propagator (user propagator)** — Z3's extension point for injecting
  custom reasoning *during* search via callbacks; RantanPlan hosts one and
  dispatches to composable modules
  ([`interference-and-propagators.md`](interference-and-propagators.md)).
- **RPG (relaxed planning graph)** — layer-by-layer reachability under
  delete relaxation; source of horizon lower bounds, pruning, and bounds
  ([`relaxed-planning-graph.md`](relaxed-planning-graph.md)).
- **Satisficing** — search mode where any valid plan is acceptable (the
  default), as opposed to **optimal**, which must prove no cheaper plan
  exists ([`encoders-and-planners.md`](encoders-and-planners.md)).
- **SDAC (state-dependent action cost)** — an action cost given as an
  expression over the state rather than a constant, so the same action can
  cost different amounts at different points in the plan.
- **Timestep** — one slot of the unrolled trajectory; each has its own copy
  of every state and action variable ([`the-encoding.md`](the-encoding.md)).

## Generated API reference

`developer-docs/doxygen/` holds the Doxygen config for the full class/file
reference (browsable HTML with class and collaboration graphs, generated
from the actual C++ source — including the doc comments now on the
architecturally central headers this documentation set points to). Build it
with:

```bash
make docs
```

This runs Doxygen into `developer-docs/doxygen/html/` (gitignored — it's a
build artifact, regenerate it after pulling). Open
`developer-docs/doxygen/html/index.html` in a browser. Requires `doxygen`
and `graphviz` (`brew install doxygen graphviz`); without Graphviz the class
diagrams are silently skipped.

The Doxygen reference and the prose docs here are complementary, not
duplicates: Doxygen gives you the exhaustive, always-current class hierarchy;
the prose gives you the *why* behind the shape of that hierarchy.
