# Axiom / derived-predicate debug benchmarks

Tiny hand-built instances that exercise every edge case of the axiom design
([docs/specs/axioms-derived-predicates.md](../../docs/specs/axioms-derived-predicates.md)).
They are deliberately minimal — each isolates one property so a failure points at
one mechanism.

**These cannot be run through the pipeline yet:** the UP PDDL reader does not
parse `:derived` (see the doc's "Frontend / ingestion gap"). Until ingestion or a
hand-translation exists, treat each `.pddl` pair as the *specification* for a
hand-built IR fixture in a unit test. Once ingestion lands, they become directly
runnable regression instances.

Every case is solvable by the short plan noted below **iff** the axiom semantics
are implemented correctly. Several are designed so that a specific bug flips the
instance to *unsolvable* (or, for the unfounded-cycle case, spuriously solvable) —
that is the point.

| Case | Exercises | Correct plan | Bug it catches |
|------|-----------|--------------|----------------|
| `01-acyclic` | Non-recursive axiom (eager completion); derived precondition; default-false init | `prepare, finish` | Completion of an acyclic stratum missing/wrong |
| `02-recursive-reach` | Recursive axiom (transitive closure) via lazy propagator; quantified body; disjunctive head (two axioms); derived goal; layer-0 derivation | `connect(b,c)` | Recursion not evaluated → `reach(a,c)` never derived |
| `03-unfounded-cycle` | **Least-fixpoint soundness.** Mutually-supporting `p↔q` with no base | `set-base, reach-goal` | Naive `head↔body` completion accepts unfounded `p=q=true` → goal spuriously reachable at horizon 0 |
| `04-noninertial` | **Frame suppression.** Derived `hot` must flip false when basic `fire` removed | `extinguish, declare-safe` | A frame axiom pins `hot=true` → goal unreachable |
| `05-stratified-negation` | Stratified negation across two strata; non-inertia of two derived preds | `remove-obstacle, move` | Strata evaluated out of order, or `clear`/`blocked` pinned |
| `06-numeric-guard` | Numeric comparison in body (guard materialization); Z3 owns arithmetic | `refuel, fly` | Guard atom not materialized/watched → `flyable` never derives |

## Coverage notes

- **Default-false** derived predicates are exercised throughout (fluents start
  false / low, so derived heads start false until founded).
- **Negative derived preconditions** appear in `04` (`declare-safe` needs
  `¬hot`) and `05` (`move` needs `clear = ¬blocked`).
- **Deliberately deferred** (needs parallelism, out of the sequential first cut):
  interference through a shared derived precondition — two actions that each
  change a basic fluent underlying one derived precondition. Add when ∀/∃-step
  support lands.
- Larger, realistic recursion lives in
  [`pddl/KEPS2023/pddl/reachability/{alice,andy}`](../KEPS2023/pddl/reachability)
  (`snowman` `reachable`).
