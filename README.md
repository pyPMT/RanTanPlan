# RantanPlan

A SAT/SMT-based automated planner with parallel action semantics. Combines a Python frontend with a high-performance C++ backend using a forked cvc5, in which planning-specific reasoning lives as a first-class SMT theory (`THEORY_PLAN`).

## Key Features

- **Four parallelism semantics**: Sequential, ∀-step (forall), ∃-step (exists), R2∃-step
- **Multiple encoding strategies**: Grounded, chained, R2∃-step
- **Search modes**: Satisficing, cost-optimal (Branch & Bound), anytime
- **Core-guided lazy activation** (`cgb`): actions start dormant behind blocking assumptions; UNSAT cores drive activation and horizon growth
- **Interference analysis**: Syntactic (eager/lazy) and semantic (eager/lazy)
- **Planning as an SMT theory**: actions, fluents and auxiliaries are `THEORY_PLAN` atoms in the fork, so planning-specific inference (interference cycle detection, forall mutexes, lazy frame axioms) runs inside the solver rather than around it
- **Numeric planning**: Full support for numeric fluents with SMT-based relaxed planning graph
- **Bounded-integer action parameters & range variables**: finite-domain integer parameters (`Int[lb,ub]`) and integer-range quantifiers, compiled away before grounding (C++-native, no Python compiler)
- **Native array fluents**: fixed-size array-valued fluents (Unified Planning's `ARRAY_FLUENTS` kind) compiled to ordinary scalar cells before solving — constant, action-parameter, and quantified indices are supported; state-dependent indices and object-parameterised arrays are not yet
- **Native set fluents**: set-valued fluents (Unified Planning's `SET_FLUENTS` kind — `member`/`subseteq`/`disjoint`/`cardinality`/`add`/`remove`/`union`/`intersect`/`difference`, with full expression nesting) compiled to boolean membership cells before solving; cardinality lowers to a pseudo-boolean `COUNT`. User-type elements only (int and nested-set elements not yet)
- **Declared state invariants**: PDDL `(:constraints (always φ))` asserted at every state
- **Symmetry breaking**: Object symmetry detection and breaking
- **Preprocessing pipeline**: Integer-parameter instantiation, quantifier expansion, array/set lowering, reachability grounding, RPG-based action removal, CWA completion, invariant discovery, landmark analysis
- **Unified Planning integration**: Plug-and-play with UP framework

**Limitation**: Does not support PDDL actions that both delete and add the same fluent.

## Installation

**Prerequisites**: Python 3.8+, CMake ≥ 3.15, a C++20 compiler, GNU Make, and
(on macOS) `brew install gmp`.

The build is a **three-stage chain** — CaDiCaL → cvc5 → RantanPlan — and CMake
cannot drive it end to end: cvc5 has its own `configure.sh` and its own
dependency-fetching machinery, and CaDiCaL is built by its own makefile.
RantanPlan's CMake consumes the *result*. Stages 1 and 2 are one-time; only
stage 3 is repeated during normal development.

> **Migration in progress.** Z3 has been removed; the backend is the cvc5 fork
> (see `docs/cvc5/migration-plan.md`). The propagator modules are being
> reimplemented as `THEORY_PLAN` reasoning over milestones M5–M7, so the
> strategy registry is temporarily reduced to those that need no propagator:
> `seq`, `r2e`, `f`, `e`, `fgs`, `egs`. Run `rantanplan --list-strategies` for
> the current set.

### 1. Check out CaDiCaL

CaDiCaL is the CDCL(T) SAT backend cvc5 runs on, and planning-specific reasoning
is expected to live inside it eventually — so it is built from a checkout rather
than downloaded as a tarball.

```bash
git clone https://github.com/arminbiere/cadical.git ~/git/cadical
cd ~/git/cadical && git checkout rel-2.1.3-elevate   # a branch, not a tag
```

The sources are currently **unmodified** upstream. Nothing else is needed here:
cvc5 builds this checkout itself in stage 2 (its `ExternalProject` writes only
into `<checkout>/build/`, which CaDiCaL's own `.gitignore` covers). If you do
fork it, keep `VERSION` within `[2.1.0, 2.1.3]` — cvc5 compiles *and runs* a
probe calling `CaDiCaL::Solver::version()`.

### 2. Build the cvc5 fork

```bash
git clone https://github.com/JoanEspasa/cvc5.git ~/git/cvc5
cd ~/git/cvc5
./configure.sh --auto-download --cadical-path=$HOME/git/cadical
make -C build -j
```

This produces a shared `Production` build at `build/src/libcvc5.dylib` (or
`.so`), which is what RantanPlan links. `--auto-download` fetches GMP, SymFPU
and LibPoly; `--cadical-path` is a fork addition that replaces the pinned
upstream tarball with your checkout.

### 3. Build RantanPlan

```bash
git clone https://github.com/pyPMT/RantanPlan.git
cd RantanPlan
python -m venv .venv
source .venv/bin/activate  # Always activate before build/run

# Build dependencies go in the venv because the install runs without build
# isolation (see note below)
pip install scikit-build-core nanobind

pip install -e . --no-build-isolation
```

The install itself builds the C++ extension module — there is no separate build
step. `--no-build-isolation` is required so the build sees the venv's own
`scikit-build-core` and `nanobind` rather than a temporary isolated copy.

cvc5 is located at `$HOME/git/cvc5` by default. Elsewhere:

```bash
pip install -e . --no-build-isolation \
    --config-settings=cmake.define.CVC5_SOURCE_DIR=/path/to/cvc5
```

### The iteration loop

`libcvc5` is *shared* and linked by full path, so:

| You edited | You must run | Cost |
|---|---|---|
| RantanPlan C++ | `pip install -e . --no-build-isolation` | seconds |
| cvc5 | `make -C ~/git/cvc5/build -j` — **no RantanPlan rebuild** | ~20–40 s |
| a cvc5 `*.toml` (options, kinds) | same, but code generation cascades | minutes |
| CaDiCaL | `make -C ~/git/cvc5/build -j` — CaDiCaL is linked *statically* into `libcvc5`, so cvc5 relinks | ~1 min |

## Usage

### Command Line

```bash
# Sequential planning (satisficing by default)
rantanplan -d domain.pddl -p problem.pddl --strategy seq

# Forall-step with lazy semantic interference and chaining
rantanplan -d domain.pddl -p problem.pddl --strategy flsc --timeout 120

# Cost-optimal planning with Branch & Bound
rantanplan -d domain.pddl -p problem.pddl --strategy elsc --mode optimal

# Anytime planning (reports improving plans as they are found)
rantanplan -d domain.pddl -p problem.pddl --strategy fl --mode anytime

# R2E semantics (declaration-order parallelism)
rantanplan -d domain.pddl -p problem.pddl --strategy r2e --output-plan solution.txt

# Double-tail bidirectional search
rantanplan -d domain.pddl -p problem.pddl --strategy flsc-dt

# Core-guided lazy activation over exists-step encoding
rantanplan -d domain.pddl -p problem.pddl --strategy cgb
```

### Declared state invariants

Problems may declare state invariants with PDDL 3 syntax, in the **instance
file**, after `:goal` and before `:metric`:

```lisp
(:constraints (always (<= (value_x) (value_y))))
```

Declarations are trusted as given (PDDL-requirement semantics, no
verification) and asserted at every state, initial and goal states
included — a declaration inconsistent with the dynamics correctly makes
the instance unsolvable. Only `always` constraints are supported
(`sometime`-style constraints are rejected), and the `r2e` strategy
rejects problems with invariants. Under parallel-step strategies the
invariant is checked at step boundaries; `seq` gives exact PDDL 3
semantics. Declaring known structure (e.g. an exactly-one invariant over
numeric fluents) can speed up solving substantially.

### Search Modes

| Mode | Description |
|------|-------------|
| `satisficing` | (default) Find first valid plan, return immediately |
| `optimal` | Cost-optimal planning using Branch & Bound with abstract suffix. Returns `SOLVED_OPTIMALLY` when optimality is proven |
| `anytime` | Continuously find improving plans, reporting each as it is found (each is also written to disk as `plan.txt.1`, `.2`, etc.) |

### Available Strategies

| Strategy | Parallelism | Interference | Encoding | Notes |
|----------|-------------|--------------|----------|-------|
| `seq` | Sequential | N/A | Grounded | Classic planning |
| `f` | ∀-step | Eager syntactic | Grounded | Basic parallel |
| `f-p` | ∀-step | Eager + propagator | Grounded | Optimized |
| `fl` | ∀-step | Lazy syntactic | Grounded | On-demand |
| `fls` | ∀-step | Lazy semantic | Grounded | Advanced |
| `flsc` | ∀-step | Lazy semantic | Chained | Best for ∀-step |
| `fgs` | ∀-step | Eager semantic | Grounded | Experimental |
| `fgsc` | ∀-step | Eager semantic | Chained | Experimental |
| `e` | ∃-step | Eager syntactic | Grounded | Basic parallel |
| `el` | ∃-step | Lazy syntactic | Grounded | With cycle detection |
| `els` | ∃-step | Lazy semantic | Grounded | Advanced |
| `elsc` | ∃-step | Lazy semantic | Chained | Best for ∃-step |
| `egs` | ∃-step | Eager semantic | Grounded | Experimental |
| `egsc` | ∃-step | Eager semantic | Chained | Experimental |
| `elsc-fp` | ∃-step | Lazy semantic | Chained | elsc + lazy frame-axiom propagator |
| `r2e` | R2∃-step | ARPG-based order | R2E | Novel encoding |
| `cgb` | ∃-step | Lazy semantic | Chained | Core-guided lazy activation |

Strategy names encode components: **e**xists/**f**orall, **l**azy/ea**g**er, **s**emantic, **c**hain. Use `--list-strategies` for a full list with descriptions.

All strategies except the `-dt` variants and `cgb` support `--mode optimal` and `--mode anytime` for cost-optimal and anytime planning respectively.

**Double-tail variants** (`seq-dt`, `f-dt`, `fl-dt`, `e-dt`, `el-dt`, etc.) use bidirectional (forward + backward) search for faster satisficing planning. Only compatible with `--mode satisficing`.

### Python API

```python
from unified_planning.shortcuts import *

problem = Problem()
# ... define problem ...

# Unified Planning integration (auto-discovered)
with OneshotPlanner(name='rantanplan') as planner:
    result = planner.solve(problem)
    print(result.plan)

# Or use directly
from rantanplan.planner_wrapper import RantanPlanPlanner
planner = RantanPlanPlanner(strategy='flsc')
result = planner.solve(problem)
```

## Architecture

```
Python Frontend              C++ Backend (_rantanplan module)
┌─────────────────┐         ┌──────────────────────┐
│ CLI / UP Engine │ ───┐    │  Preprocessing       │
│                 │    │    │  ┌──────────────────┐ │
│ • PDDL parsing  │    └───▶│  │ Canonicalize     │ │
│ • opt. UP ground│ in-     │  │ Grounding        │ │
│ • UP→IR convert │ process │  │ RPG analysis     │ │
│ • Plan validate │◀────┘   │  │ Symmetry/Interf. │ │
└─────────────────┘ nanobind│  └──────────────────┘ │
                            │  Solving               │
                            │  ┌──────────────────┐ │
                            │  │ Encoder          │ │
                            │  │ Parallelism      │ │
                            │  │ THEORY_PLAN      │ │
                            │  │ B&B / Sequential │ │
                            │  └──────────────────┘ │
                            │                        │
                            │  cvc5 (fork)           │
                            └──────────────────────┘
```

The Python frontend uses Unified Planning to parse PDDL, converts the problem to a flat plain-data IR, and calls the C++ backend in-process through a nanobind extension module (no subprocess, no serialization). Canonicalization that used to run in Python — quantifier removal, integer-parameter instantiation, CNF normalization — now happens in C++; the only optional Python-side compilation step is UP cross-product grounding (`--up-grounding`). The backend runs a preprocessing pipeline (integer-parameter instantiation, quantifier expansion, array and set lowering, reachability grounding, RPG-based action removal, symmetry detection), then encodes the problem as SMT and solves with the cvc5 fork, whose `THEORY_PLAN` owns the planning atoms.

## Development

```bash
source .venv/bin/activate  # Always required

# Building — after any C++ change, re-run the install (incremental, a few
# seconds; the CMake build tree persists in build/). Python-only changes
# need nothing.
pip install -e . --no-build-isolation

# Clean rebuild: remove the build tree first
rm -rf build && pip install -e . --no-build-isolation

# Debug build
pip install -e . --no-build-isolation --config-settings=cmake.build-type=Debug

# Testing (suites live in tests/; test.py is a thin entry point)
python test.py               # PDDL suite (all strategies/domains) + API-only suite
python test.py --quick       # Fast PDDL subset + API-only suite
python test.py --api-only    # Only the API-only feature tests (e.g. int parameters)
python test.py -v            # Verbose output
```

The `tests/` package holds two suites: `tests.pddl_suite` (PDDL domain/problem
files under `pddl/test/`) and `tests.api` (API-only feature tests for things with
no PDDL syntax). To add tests for a new API-only feature, drop a
`tests/api/<feature>.py` exposing a `CASES` list — it is discovered automatically
(see `tests/README.md`).

Experimental: scikit-build-core can rebuild the extension automatically on
`import rantanplan` if installed with
`--config-settings=editable.rebuild=true` (off by default; see the
scikit-build-core docs).

## Citation

If you use RantanPlan in your research, please cite:

```bibtex
@article{Bofill_Espasa_Villaret_2016,
  title={The RANTANPLAN planner: system description},
  volume={31},
  DOI={10.1017/S0269888916000229},
  number={5},
  journal={The Knowledge Engineering Review},
  author={Bofill, Miquel and Espasa, Joan and Villaret, Mateu},
  year={2016},
  pages={452–464}
}
```
