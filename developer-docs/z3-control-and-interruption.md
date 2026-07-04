# Controlling and interrupting the Z3 process

This doc is about *process control*: how RantanPlan drives Z3's solver
incrementally, how it budgets and enforces timeouts, and how a solve gets
cancelled mid-search. It complements
[`interference-and-propagators.md`](interference-and-propagators.md) (the
propagator *callback* mechanics — push/pop/fixed/decide) and
[`python-cpp-boundary.md`](python-cpp-boundary.md) (the Python-facing
`interrupt()` API) rather than repeating them.

## One solver per planner

Every `BasePlanner` owns exactly one `z3::solver`, constructed once in the
`BasePlanner` constructor (`solver_(ctx)` — `planners/base_planner.cpp`) and
used for the entire search. There is no per-timestep or per-check solver
reset: constraints accumulate in `solver_` as the horizon grows (or, for
`BranchAndBoundPlanner`, within a `push()`/`pop()` scope — see below), which
is what lets Z3 reuse its learned clauses across an incrementally-growing
formula instead of re-deriving everything from scratch at every horizon.

Solver tuning parameters (`random_seed`, an SMT `logic` hint derived from
the problem's `ArithmeticProfile`) are set once, in
`BasePlanner::set_propagator_strategy()`, and — this is a real, previously
debugged gotcha worth knowing — **must be set after the propagator is
registered**. Z3 has two solver backends with different parameter
namespaces: a standalone SAT backend (`sat.*` parameters) and an SMT
backend (`phase_selection`, `restart_strategy`, ...) that Z3 switches to
the moment a `z3::user_propagator_base` is registered. Setting `sat.*`
parameters *before* that switch silently no-ops — the propagator object
exists and is attached, but Z3 never invokes any of its callbacks, and
every propagator-dependent strategy (forall/exists/lazy) would produce
invalid plans with interfering actions in the same timestep. The fix, and
the reason for the ordering, is recorded directly in the source comment at
`set_propagator_strategy()`.

## The timeout: a shrinking budget, not one deadline

`BasePlanner` doesn't set a Z3 timeout once and forget it. Instead:

1. `init_deadline()` computes one absolute wall-clock deadline from
   `Config::global.timeout` (seconds from now), or `time_point::max()` if
   no timeout was requested.
2. Before **every** `solver.check()` call, `apply_solver_timeout(solver_)`
   calls `remaining_ms()` — milliseconds left until the deadline — and sets
   Z3's own `timeout` parameter to that value:

```cpp
bool BasePlanner::apply_solver_timeout(z3::solver& solver) {
    unsigned ms = remaining_ms();
    if (ms == 0) { timed_out_ = true; return false; }   // budget exhausted
    if (ms != UINT_MAX) {
        z3::params p(solver.ctx());
        p.set("timeout", ms);
        solver.set(p);
    }
    return true;
}
```

So a run with a 60-second budget doesn't give Z3 a flat 60-second timeout
for every horizon's `check()` — the *n*-th check only gets whatever time is
left after the previous *n − 1* checks. If `apply_solver_timeout` returns
`false` (budget already gone), the planner breaks out of its search loop
before even building the next horizon's formula.

When `solver.check()` returns `z3::unknown`, that's ambiguous — it could
mean the timeout fired, an external interrupt happened, or Z3 genuinely
gave up for some other reason. `handle_unknown_result()` disambiguates by
string-matching `solver.reason_unknown()`:

```cpp
bool BasePlanner::handle_unknown_result(const z3::solver& solver, const std::string& context) {
    std::string reason = solver.reason_unknown();
    if (reason.find("timeout") != std::string::npos ||
        reason.find("canceled") != std::string::npos) {
        timed_out_ = true;
        return true;
    }
    return false;  // some other cause; logged, not treated as a timeout
}
```

Both a real wall-clock timeout *and* a cross-thread `interrupt()` call (next
section) end up producing a `reason_unknown()` that matches this check —
from the planner's point of view they're the same outcome, surfaced to
Python as `SolveStatusIR::TIMEOUT` either way.

## Cross-thread cancellation

Python's `Preprocessed.interrupt()` (see
[`python-cpp-boundary.md`](python-cpp-boundary.md)) needs to cancel a solve
running on a *different* thread, without touching the GIL and without
racing the C++ side tearing down the `z3::context` it's cancelling. The
mechanism, entirely in `api/solver_api.cpp`:

```cpp
void InterruptSlot::interrupt() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ctx_) ctx_->interrupt();   // Z3_interrupt — documented safe cross-thread
}

ScopedContextPublisher::ScopedContextPublisher(InterruptSlot* slot, z3::context& ctx)
    : slot_(slot) {
    if (slot_) { std::lock_guard<std::mutex> lock(slot_->mutex_); slot_->ctx_ = &ctx; }
}
ScopedContextPublisher::~ScopedContextPublisher() {
    if (slot_) { std::lock_guard<std::mutex> lock(slot_->mutex_); slot_->ctx_ = nullptr; }
}
```

`api::solve()` constructs its `z3::context ctx` and immediately publishes it
via a `ScopedContextPublisher`, declared right after `ctx` so C++'s
destruction order unpublishes it *before* `ctx` itself is destroyed:

```cpp
z3::context ctx;
ScopedContextPublisher publish(interrupt_slot, ctx);   // published for encoder
                                                        // + planner + propagator
                                                        // setup AND the search
```

The publish window covers strategy assembly (encoder/planner/propagator
construction) as well as the search itself, not just the `check()` calls —
so `interrupt()` can technically fire before search even starts, though in
practice that window is short. Both sides take the same mutex, so there's
no way to `interrupt()` a context that's mid-destruction, or destroy a
context an in-flight `interrupt()` is about to dereference — one path always
waits for the other.

`z3::context::interrupt()` itself just flips an internal Z3 flag; it
doesn't force an immediate stop. The *next* place Z3 checks that flag (Z3
checks it periodically during search) is where an in-flight `check()`
actually returns — with result `unknown` and a `reason_unknown()`
containing "canceled", which is exactly what
`handle_unknown_result()` above is watching for. Outside an active
`solver.check()` — during preprocessing, or between horizons while the
planner is building the next formula — `interrupt()` has nothing to
interrupt and is a no-op until the next `check()` call starts.

## Incremental search: assumptions, not push/pop, for the common case

Growing the horizon doesn't mean re-solving from scratch or wrapping each
horizon in a `push()`/`pop()` scope. `IncrementalHorizonPlanner` and
`DoubleTailPlanner` both use **assumption literals**: a fresh Boolean
constant is created per horizon, permanently asserted behind an implication
(never asserted true), and passed as the sole assumption to `check()`:

```cpp
// planners/incremental_horizon_planner.cpp
std::string goal_var_name = "goal_at_" + std::to_string(h);
z3::expr goal_literal = ctx_.bool_const(goal_var_name.c_str());
solver_.add(z3::implies(goal_literal, encoder_.encode_goal(h)));

z3::expr_vector assumptions(ctx_);
assumptions.push_back(goal_literal);
z3::check_result result = solver_.check(assumptions);
```

Because `goal_literal` is never permanently asserted, `solver_` never
actually contains "the goal holds at horizon `h`" as a hard fact — it's
active only for the duration of that one `check(assumptions)` call. This
means every clause and lemma Z3 has already learned while proving earlier
horizons unsatisfiable stays valid and reusable at the next horizon; nothing
needs to be popped or re-derived. It's a much cheaper style of incrementality
than the classic `push()`-solve-`pop()`-solve-again loop.

`BranchAndBoundPlanner` is the one place that *does* use explicit
`push()`/`pop()`: it opens a scope per horizon for the horizon-dependent
abstract-suffix constraints (relaxed preconditions, abstract goal, the
running cost bound), asserts permanent concrete transitions *outside* that
scope, and pops the scope before moving to the next horizon. It needs
`push`/`pop` rather than an assumption literal because it's asserting a
whole family of horizon-specific constraints, not gating a single fact.

## `persist_clauses`

One more Z3-behavior knob worth knowing about: `Config::global.persist_clauses`
(CLI: `--persist-clauses`) maps directly to Z3's `smt.up.persist_clauses`
parameter, set once in `PropagatorStrategy`'s construction:

```cpp
solver.set("smt.up.persist_clauses", Config::instance().global.persist_clauses);
```

Z3's unit-propagation engine can discard theory lemmas it derived once a
`pop()` backtracks past the point they were learned (the default,
`persist_clauses = false`), forcing propagators to re-derive them if the
same situation recurs — or keep them around across backtracking
(`persist_clauses = true`) at the cost of extra memory. It's a
throughput/memory tradeoff exposed as a flag rather than a fixed choice
because which side wins depends on how expensive a given propagator
module's re-derivation is versus how much backtracking a given problem
triggers; there's no universally-better setting.
