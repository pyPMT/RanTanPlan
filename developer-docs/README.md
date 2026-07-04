# RantanPlan developer documentation

This is the onboarding path for contributors: the design philosophy and the
reasoning behind it, not just the API surface. For day-to-day commands
(build, test, benchmark) and the file-by-file architecture reference, see the
project root [`CLAUDE.md`](../CLAUDE.md) — these docs complement it rather
than duplicate it.

## Reading order

1. [`architecture-overview.md`](architecture-overview.md) — what RantanPlan
   is, why it's built as two layers (Python frontend / C++ core, and within
   the core, preprocessor / backend), and the philosophy that shapes
   everything else here.
2. [`python-cpp-boundary.md`](python-cpp-boundary.md) — the in-process
   nanobind contract: `ProblemIR`, the GIL, the anytime callback, cancellation.
3. [`preprocessing-and-strategy.md`](preprocessing-and-strategy.md) — the
   pass pipeline, the `PreprocessedProblem` contract, and the strategy
   registry/factory that assembles a backend.
4. [`interference-and-propagators.md`](interference-and-propagators.md) —
   why interference analysis has two interfaces, and how the Z3 user
   propagator is split into composable modules.
5. [`encoders-and-planners.md`](encoders-and-planners.md) — the encoder
   families, the planner backends, and how a search mode resolves to one.
6. [`z3-control-and-interruption.md`](z3-control-and-interruption.md) — how
   the Z3 solver is driven incrementally, timeout budgeting, and cross-thread
   cancellation.

Each doc includes a Mermaid diagram of its central flow — GitHub and most
Markdown viewers render these inline.

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
