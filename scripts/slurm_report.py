#!/usr/bin/env python3
"""
Report generator for RantanPlan SLURM benchmark results.

Consumes the ``.out``/``.err`` pairs written by a SLURM array run of the
commands produced by ``prepare_batch_run.py`` and, optionally, the ``.stat``
files those runs wrote via ``--stats-file``.

Generates:
1. Survival (cactus) plots comparing systems by solving time
2. Pairwise scatter plots between systems (linear + log scale)
3. CSV summary tables (per-instance, per-domain, overall)
4. Solver/metric analysis: pairwise metric scatters, variant impact,
   time breakdowns, encoding size, per-domain summaries

Usage:
    python3 scripts/slurm_report.py logs/ --stats-dir stats_and_plans/ -o report/

Log format contract
-------------------
Each ``.out`` file is expected to look like::

    [Sun  2 Aug 11:42:25 CEST 2026] Running task 1 from batches/batch_1.txt: <cmd>
    domain=<name>
    instance=<name>
    strategy=<name>
    mode=<name>
    ... planner output ...
    [Sun  2 Aug 11:44:30 CEST 2026] Task 1 finished with exit code 0

Both the ``key=value`` echoes and the wrapper command line are used; the
command line additionally yields the horizon schedule and the exact stats-file
name, neither of which is echoed.  Everything is optional: a job is kept as
long as its domain, instance and strategy can be recovered from either source.
"""
import argparse
import csv
import multiprocessing
import os
import re
import sys
from dataclasses import dataclass, field
from enum import Enum
from itertools import combinations
from math import ceil, log2
from pathlib import Path
from statistics import median
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


# ===========================================================================
# Data model
# ===========================================================================

class Status(Enum):
    SOLVED = "solved"
    UNSOLVABLE = "unsolvable"
    TIMEOUT = "timeout"
    MEMOUT = "memout"
    ERROR = "error"
    #: The wrapper never wrote a verdict: the job is still in flight, or the
    #: whole array was torn down before it could report.  Kept apart from the
    #: failure statuses so an ongoing run is not scored as a wall of timeouts.
    RUNNING = "running"
    UNKNOWN = "unknown"


#: Statuses that count as "the planner answered the question".
CONCLUSIVE = (Status.SOLVED, Status.UNSOLVABLE)


@dataclass
class Job:
    """One (domain, instance, strategy, mode, schedule) run."""
    domain: str
    instance: str
    strategy: str
    mode: str
    schedule: str
    log_file: Path
    status: Status
    total_time: Optional[float] = None
    wall_time: Optional[float] = None
    horizon: Optional[int] = None
    num_actions: Optional[int] = None
    last_memory_mb: Optional[int] = None
    exit_code: Optional[int] = None
    stats_file: Optional[str] = None
    stats: Dict[str, float] = field(default_factory=dict)

    #: Label used everywhere the run is compared against others.  Set by
    #: :func:`assign_system_labels` once the whole experiment is known, so that
    #: mode/schedule only clutter the label when the run actually varies them.
    system: str = ""

    @property
    def instance_key(self) -> Tuple[str, str]:
        return (self.domain, self.instance)


@dataclass
class Experiment:
    jobs: List[Job]
    timeout: float
    systems: List[str]
    domains: List[str]
    skipped_empty: int = 0
    skipped_unparsed: int = 0
    duplicates: int = 0

    _by_key: Dict[Tuple[str, str, str], Job] = field(default_factory=dict, repr=False)
    _instance_keys: List[Tuple[str, str]] = field(default_factory=list, repr=False)
    _by_system: Dict[str, List[Job]] = field(default_factory=dict, repr=False)
    _by_domain_system: Dict[Tuple[str, str], List[Job]] = field(
        default_factory=dict, repr=False
    )

    def build_indexes(self) -> None:
        self._by_key = {}
        self._by_system = {}
        self._by_domain_system = {}
        keys: Set[Tuple[str, str]] = set()
        for j in self.jobs:
            self._by_key[(j.domain, j.instance, j.system)] = j
            keys.add(j.instance_key)
            self._by_system.setdefault(j.system, []).append(j)
            self._by_domain_system.setdefault((j.domain, j.system), []).append(j)
        self._instance_keys = sorted(keys)

    def job_for(self, domain: str, instance: str, system: str) -> Optional[Job]:
        return self._by_key.get((domain, instance, system))

    def instance_keys(self) -> List[Tuple[str, str]]:
        return self._instance_keys

    def by_system(self, system: str) -> List[Job]:
        return self._by_system.get(system, [])

    def by_domain_system(self, domain: str, system: str) -> List[Job]:
        return self._by_domain_system.get((domain, system), [])

    def solved_with_stats(self, system: str) -> List[Job]:
        return [j for j in self.by_system(system) if j.status == Status.SOLVED and j.stats]

    def stat_keys(self) -> Set[str]:
        return {k for j in self.jobs for k in j.stats}


# ===========================================================================
# Log parsing
# ===========================================================================

# Wrapper lines emitted by the SLURM array driver around each task.
RE_RUNNING = re.compile(r"^\[[^\]]*\]\s+Running task\s+\d+\b[^:]*:\s*(?P<cmd>.+)$")
RE_FINISHED = re.compile(r"^\[[^\]]*\]\s+Task\s+\d+\s+finished with exit code\s+(-?\d+)")
RE_CLOCK = re.compile(r"^\[[^\]]*?(\d{2}):(\d{2}):(\d{2})[^\]]*\]")

RE_META = re.compile(r"^([A-Za-z_]\w*)=(.+)$")
RE_CMD_OPT = re.compile(
    r"--(strategy|mode|horizon-schedule|stats-file|output-plan)[= ]\s*(\S+)"
)

# One tolerant matcher for every planner's success line.  Covers:
#   *** PLAN FOUND: horizon=H, actions=A (total time: Ts) ***
#   *** PLAN FOUND: A actions, R rounds, horizon=H, activated=x/y, extensions=E (total time: Ts) ***
#   *** PLAN FOUND at iteration I (plan length L, total time: Ts) ***
#   *** OPTIMAL|BEST PLAN FOUND: horizon=H, actions=A, cost=C (... total time: Ts) ***
RE_PLAN_FOUND = re.compile(
    r"\*\*\*\s+(?:OPTIMAL\s+|BEST\s+)?PLAN FOUND\b(.*?)total time:\s*([\d.]+)\s*s"
)
RE_NO_PLAN = re.compile(r"\*\*\*\s+NO PLAN FOUND\b(.*)")
RE_BODY_HORIZON = re.compile(r"horizon=(\d+)")
RE_BODY_ACTIONS = re.compile(r"actions=(\d+)|\b(\d+)\s+actions\b|plan length\s+(\d+)")

RE_STATUS = re.compile(r"Status:\s*PlanGenerationResultStatus\.(\w+)")
RE_MEM = re.compile(r"\bmem:\s*(\d+)\s*MB")

RE_OOM = re.compile(
    r"std::bad_alloc|[Oo]ut of memory|oom[-_]kill|Killed process|\bKilled\b", re.IGNORECASE
)
RE_FATAL = re.compile(
    r"Error during planning:|Error parsing PDDL|Aborted \(core dumped\)"
    r"|Traceback \(most recent call last\)|terminate called|Segmentation fault"
)
RE_CANCELLED = re.compile(r"CANCELLED.*DUE TO TIME LIMIT", re.IGNORECASE)

#: Exit codes with a known meaning.  124 is GNU ``timeout``; 137/139 are
#: 128+SIGKILL / 128+SIGSEGV, the former almost always an OOM kill.
EXIT_STATUS = {0: None, 124: Status.TIMEOUT, 137: Status.MEMOUT, 139: Status.ERROR}

#: Statuses the planner itself reports through the Python CLI.
UP_STATUS = {
    "SOLVED_SATISFICING": Status.SOLVED,
    "SOLVED_OPTIMALLY": Status.SOLVED,
    "UNSOLVABLE_PROVEN": Status.UNSOLVABLE,
    "UNSOLVABLE_INCOMPLETELY": Status.UNSOLVABLE,
    "TIMEOUT": Status.TIMEOUT,
    "INTERNAL_ERROR": Status.ERROR,
    "UNSUPPORTED_PROBLEM": Status.ERROR,
    "MEMOUT": Status.MEMOUT,
}


def _clock_seconds(line: str) -> Optional[int]:
    m = RE_CLOCK.match(line)
    if not m:
        return None
    h, mi, s = (int(g) for g in m.groups())
    return h * 3600 + mi * 60 + s


def _parse_command(cmd: str) -> Dict[str, str]:
    """Recover run parameters from the wrapper's echoed shell command."""
    opts: Dict[str, str] = {}
    for key, value in RE_CMD_OPT.findall(cmd):
        opts.setdefault(key, value.strip("'\""))
    # The `echo 'domain=...'` prefix carries domain/instance verbatim.
    for token in re.findall(r"echo\s+'([^']*)'", cmd):
        m = RE_META.match(token)
        if m:
            opts.setdefault(m.group(1), m.group(2))
    return opts


def _plan_body_fields(body: str) -> Tuple[Optional[int], Optional[int]]:
    horizon = None
    actions = None
    m = RE_BODY_HORIZON.search(body)
    if m:
        horizon = int(m.group(1))
    m = RE_BODY_ACTIONS.search(body)
    if m:
        actions = int(next(g for g in m.groups() if g is not None))
    return horizon, actions


def _scan_err_file(log_path: Path) -> Optional[Status]:
    """SLURM writes cancellation and OOM notices to the sibling .err file."""
    err_path = log_path.with_suffix(".err")
    try:
        if not err_path.is_file() or err_path.stat().st_size == 0:
            return None
        text = err_path.read_text(errors="replace")
    except OSError:
        return None
    if RE_OOM.search(text):
        return Status.MEMOUT
    if RE_CANCELLED.search(text):
        return Status.TIMEOUT
    return None


def parse_log(log_path: Path, exclude: Optional[Sequence[str]] = None) -> Optional[Job]:
    """Parse one SLURM ``.out`` file.  Streams line by line; logs run to
    hundreds of thousands of lines when a plan is printed."""
    meta: Dict[str, str] = {}
    start_clock: Optional[int] = None
    end_clock: Optional[int] = None
    exit_code: Optional[int] = None
    total_time: Optional[float] = None
    horizon: Optional[int] = None
    actions: Optional[int] = None
    last_mem: Optional[int] = None
    reported: Optional[Status] = None       # from the planner's own Status: line
    no_plan: Optional[Status] = None        # from a *** NO PLAN FOUND *** line
    solved = False
    saw_oom = False
    saw_fatal = False
    have_cmd = False

    try:
        with open(log_path, "r", errors="replace") as f:
            for lineno, raw in enumerate(f):
                # Every line we care about starts at column 0.  Indented lines
                # are printed plan steps ("  12. move(a, b)") and secondary
                # detail lines, and they dominate the file — a solved log runs
                # to hundreds of thousands of lines — so reject them before
                # paying for a strip.
                if not raw or raw[0].isspace():
                    continue
                line = raw.rstrip()
                head = line[0]

                if head == "[":
                    if line.startswith("[Solving"):
                        if "mem:" in line:
                            m = RE_MEM.search(line)
                            if m:
                                last_mem = int(m.group(1))
                        continue
                    if not have_cmd and " Running task " in line:
                        m = RE_RUNNING.match(line)
                        if m:
                            have_cmd = True
                            start_clock = _clock_seconds(line)
                            for k, v in _parse_command(m.group("cmd")).items():
                                meta.setdefault(k, v)
                            # Bail out early on excluded strategies: avoids
                            # reading the (often enormous) rest of the file.
                            strategy = meta.get("strategy", "")
                            if exclude and any(pat in strategy for pat in exclude):
                                return None
                        continue
                    if "finished with exit code" in line:
                        m = RE_FINISHED.match(line)
                        if m:
                            exit_code = int(m.group(1))
                            end_clock = _clock_seconds(line)
                    continue

                if head == "*":
                    m = RE_PLAN_FOUND.search(line)
                    if m:
                        solved = True
                        total_time = float(m.group(2))
                        h, a = _plan_body_fields(m.group(1))
                        horizon = h if h is not None else horizon
                        actions = a if a is not None else actions
                        continue
                    m = RE_NO_PLAN.search(line)
                    if m:
                        # "(timeout after Ns)" vs "within N iterations" — the
                        # latter means the search space was exhausted.
                        no_plan = (
                            Status.TIMEOUT if "timeout" in m.group(1).lower()
                            else Status.UNSOLVABLE
                        )
                    continue

                if head.isdigit():
                    continue  # printed plan step, "  12. move(a, b)"

                if head == "S" and line.startswith("Status:"):
                    m = RE_STATUS.search(line)
                    if m:
                        reported = UP_STATUS.get(m.group(1), Status.UNKNOWN)
                    continue

                if lineno < 16 and "=" in line:
                    m = RE_META.match(line)
                    if m:
                        meta.setdefault(m.group(1), m.group(2))
                        continue

                if RE_OOM.search(line):
                    saw_oom = True
                elif RE_FATAL.search(line):
                    saw_fatal = True
    except OSError:
        return None

    if "domain" not in meta or "instance" not in meta or "strategy" not in meta:
        return None
    if exclude and any(pat in meta["strategy"] for pat in exclude):
        return None

    status = _resolve_status(
        solved=solved, reported=reported, no_plan=no_plan, exit_code=exit_code,
        saw_oom=saw_oom, saw_fatal=saw_fatal, log_path=log_path,
    )

    wall_time = None
    if start_clock is not None and end_clock is not None:
        wall_time = float((end_clock - start_clock) % 86400)

    return Job(
        domain=meta["domain"],
        instance=meta["instance"],
        strategy=meta["strategy"],
        mode=meta.get("mode", "unknown"),
        schedule=meta.get("horizon-schedule", "default"),
        log_file=log_path,
        status=status,
        total_time=total_time,
        wall_time=wall_time,
        horizon=horizon,
        num_actions=actions,
        last_memory_mb=last_mem,
        exit_code=exit_code,
        stats_file=meta.get("stats-file"),
    )


def _resolve_status(
    *, solved: bool, reported: Optional[Status], no_plan: Optional[Status],
    exit_code: Optional[int], saw_oom: bool, saw_fatal: bool, log_path: Path,
) -> Status:
    """Decide a job's outcome from the strongest evidence available.

    Precedence: a plan was actually printed > the planner's own Status: line >
    an explicit "no plan" verdict > the wrapper's exit code > SLURM's .err
    notice > in-log crash markers.  Nothing is silently folded into TIMEOUT: a
    job with no evidence at all is RUNNING (still in flight), and a job that
    exited cleanly yet reported nothing is UNKNOWN.
    """
    if solved or reported == Status.SOLVED:
        return Status.SOLVED
    if reported is not None and reported != Status.UNKNOWN:
        return reported
    if no_plan is not None:
        return no_plan
    if exit_code is not None:
        mapped = EXIT_STATUS.get(exit_code, Status.ERROR)
        if mapped is not None:
            return mapped
        # Exit code 0 without any verdict: treat crash markers as decisive.
        return Status.MEMOUT if saw_oom else (Status.ERROR if saw_fatal else Status.UNKNOWN)
    from_err = _scan_err_file(log_path)
    if from_err is not None:
        return from_err
    if saw_oom:
        return Status.MEMOUT
    if saw_fatal:
        return Status.ERROR
    return Status.RUNNING


_worker_exclude: Optional[Sequence[str]] = None


def _init_worker(exclude: Optional[Sequence[str]]) -> None:
    global _worker_exclude
    _worker_exclude = exclude


def _parse_worker(path_str: str) -> Optional[Job]:
    return parse_log(Path(path_str), _worker_exclude)


def parse_stats_file(path: Path) -> Dict[str, float]:
    """Parse a ``key: value`` stats file, keeping only numeric values."""
    out: Dict[str, float] = {}
    try:
        with open(path, "r", errors="replace") as f:
            for line in f:
                key, sep, val = line.partition(": ")
                if not sep:
                    continue
                try:
                    out[key.strip()] = float(val.strip())
                except ValueError:
                    pass
    except OSError:
        pass
    return out


# ===========================================================================
# Experiment assembly
# ===========================================================================

def assign_system_labels(jobs: List[Job]) -> None:
    """Label each job with the identity it is compared under.

    ``prepare_batch_run.py`` sweeps modes and horizon schedules by default, but
    neither is echoed into the log metadata.  Folding them into the label keeps
    two schedules of the same strategy from silently overwriting each other,
    while leaving the label as a bare strategy name for the common single-mode,
    single-schedule run.
    """
    vary_mode = len({j.mode for j in jobs}) > 1
    vary_schedule = len({j.schedule for j in jobs}) > 1
    for j in jobs:
        parts = [j.strategy]
        if vary_mode:
            parts.append(j.mode)
        if vary_schedule:
            parts.append(j.schedule)
        j.system = "@".join(parts)


def attach_stats(jobs: List[Job], stats_dir: Path) -> int:
    """Load each job's ``.stat`` file and let it refine the log-derived numbers."""
    attached = 0
    for job in jobs:
        name = Path(job.stats_file).name if job.stats_file else (
            f"{job.domain}_{job.instance}_{job.strategy}_{job.mode}_{job.schedule}.stat"
        )
        path = stats_dir / name
        if not path.is_file():
            continue
        stats = parse_stats_file(path)
        if not stats:
            continue
        job.stats = stats
        attached += 1
        if "planner.total_time" in stats:
            job.total_time = stats["planner.total_time"]
        if job.horizon is None and "planner.solution_horizon" in stats:
            job.horizon = int(stats["planner.solution_horizon"])
        if job.num_actions is None and "planner.plan_length" in stats:
            job.num_actions = int(stats["planner.plan_length"])
    return attached


def infer_timeout(jobs: List[Job], override: Optional[float]) -> Tuple[float, str]:
    """Determine the per-job wall-clock limit.

    The wrapper stamps a start and a finish time around every task, so jobs
    that hit the limit measure it directly — far more reliable than guessing
    from the slowest successful run.
    """
    if override is not None:
        return override, "user override"
    killed = [j.wall_time for j in jobs
              if j.status in (Status.TIMEOUT, Status.MEMOUT) and j.wall_time]
    if killed:
        return float(round(median(killed))), f"median wall time of {len(killed)} killed jobs"
    solved = [j.total_time for j in jobs if j.status == Status.SOLVED and j.total_time]
    if solved:
        return max(max(solved) * 1.1, 60.0), "1.1x slowest solve (no killed jobs to measure)"
    return 1800.0, "default (no timing evidence)"


def load_experiment(
    logs_dir: Path,
    stats_dir: Optional[Path],
    timeout_override: Optional[float],
    exclude: Optional[Sequence[str]] = None,
) -> Experiment:
    log_files = sorted(logs_dir.glob("*.out"))
    if not log_files:
        sys.exit(f"Error: no .out files found in {logs_dir}")

    non_empty: List[Path] = []
    skipped_empty = 0
    for lf in log_files:
        if lf.stat().st_size == 0:
            skipped_empty += 1
        else:
            non_empty.append(lf)

    workers = max(1, min(os.cpu_count() or 1, len(non_empty)))
    print(f"Parsing {len(non_empty)} log files with {workers} workers...", file=sys.stderr)

    paths = [str(p) for p in non_empty]
    if workers == 1:
        _init_worker(exclude)
        results = [_parse_worker(p) for p in paths]
    else:
        with multiprocessing.Pool(workers, initializer=_init_worker, initargs=(exclude,)) as pool:
            results = pool.map(_parse_worker, paths, chunksize=32)

    jobs = [r for r in results if r is not None]
    skipped_unparsed = len(results) - len(jobs)
    print(f"Parsed {len(jobs)} jobs.", file=sys.stderr)
    if skipped_empty:
        print(f"  {skipped_empty} empty log files (job died before any output)", file=sys.stderr)
    if skipped_unparsed:
        print(f"  {skipped_unparsed} files skipped (excluded or no recoverable metadata)",
              file=sys.stderr)
    if not jobs:
        sys.exit("Error: no usable jobs — check that the logs carry the "
                 "'domain='/'instance='/'strategy=' header or the wrapper command line.")

    assign_system_labels(jobs)

    # Detect genuine collisions rather than letting the index swallow them.
    seen: Set[Tuple[str, str, str]] = set()
    duplicates = 0
    for j in jobs:
        key = (j.domain, j.instance, j.system)
        if key in seen:
            duplicates += 1
        seen.add(key)
    if duplicates:
        print(f"  Warning: {duplicates} duplicate (domain, instance, system) runs; "
              f"the last one wins", file=sys.stderr)

    if stats_dir and stats_dir.is_dir():
        attached = attach_stats(jobs, stats_dir)
        print(f"Attached stats to {attached}/{len(jobs)} jobs.", file=sys.stderr)

    timeout, how = infer_timeout(jobs, timeout_override)
    print(f"Timeout: {timeout:.0f}s ({how})", file=sys.stderr)

    exp = Experiment(
        jobs=jobs,
        timeout=timeout,
        systems=sorted({j.system for j in jobs}),
        domains=sorted({j.domain for j in jobs}),
        skipped_empty=skipped_empty,
        skipped_unparsed=skipped_unparsed,
        duplicates=duplicates,
    )
    exp.build_indexes()
    return exp


# ===========================================================================
# Metric resolution (solver-agnostic)
# ===========================================================================

#: Prefixes under which an SMT solver publishes its own counters.  RantanPlan
#: emitted ``z3.*`` before the cvc5 port and ``cvc5.*`` after it
#: (``planners/base_planner.cpp``), with entirely different counter names, so
#: nothing below may hardcode either spelling.
SOLVER_PREFIXES = ("z3", "cvc5")

#: (concept, label, ordered suffix patterns).  The first pattern that matches
#: any key wins; among equally-ranked matches the shortest key is taken, which
#: prefers the aggregate counter over its per-theory refinements.
CONCEPTS: List[Tuple[str, str, List[str]]] = [
    ("decisions",    "Decisions",           [r"^decisions$", r"decisions$"]),
    ("conflicts",    "Conflicts",           [r"^conflicts$", r"conflicts$"]),
    ("propagations", "Propagations",        [r"^propagations$", r"propagations$"]),
    ("restarts",     "Restarts",            [r"^restarts$", r"restarts$"]),
    ("solver_time",  "Solver time (s)",     [r"^time$", r"totalTime$", r"[-_:]time$"]),
    ("solver_mem",   "Solver memory (MB)",  [r"^max memory$", r"^memory$", r"memory$"]),
    ("rlimit",       "Resource count",      [r"^rlimit count$", r"rlimit", r"resource"]),
    ("bool_vars",    "Boolean variables",   [r"^mk bool var$", r"[Vv]ariables$"]),
    ("clauses",      "Clauses",             [r"^mk clause$", r"[Cc]lauses$"]),
    ("lp_columns",   "LP columns",          [r"max-columns$", r"columns$"]),
    ("lp_rows",      "LP rows",             [r"max-rows$", r"rows$"]),
]

#: Concepts that come from RantanPlan itself and are stable across solvers.
FIXED_METRICS: Dict[str, Tuple[str, str]] = {
    "total_time":   ("planner.total_time", "Total time (s)"),
    "solve_time":   ("planner.solve_time", "Solve time (s)"),
    "formula_time": ("planner.formula_time", "Formula time (s)"),
    "peak_mem":     ("memory.current_mb", "Process memory (MB)"),
    "plan_length":  ("planner.plan_length", "Plan length"),
}


class Metrics:
    """Maps stable concept names onto whatever keys the data actually holds."""

    def __init__(self, keys: Set[str]) -> None:
        self.keys = keys
        self.solver_prefixes = sorted(
            p for p in SOLVER_PREFIXES if any(k.startswith(p + ".") for k in keys)
        )
        self._resolved: Dict[str, str] = {}
        self._labels: Dict[str, str] = {}
        for concept, label, patterns in CONCEPTS:
            key = self._match(patterns)
            if key:
                self._resolved[concept] = key
                self._labels[concept] = label
        for concept, (key, label) in FIXED_METRICS.items():
            if key in keys:
                self._resolved[concept] = key
                self._labels[concept] = label

    def _match(self, patterns: Iterable[str]) -> Optional[str]:
        for pattern in patterns:
            rx = re.compile(pattern)
            hits = [
                k for k in self.keys
                for prefix in self.solver_prefixes
                if k.startswith(prefix + ".") and rx.search(k[len(prefix) + 1:])
            ]
            if hits:
                return min(hits, key=lambda k: (len(k), k))
        return None

    def key(self, concept: str) -> Optional[str]:
        return self._resolved.get(concept)

    def label(self, concept: str) -> str:
        return self._labels.get(concept, concept)

    def select(self, concepts: Sequence[str]) -> List[Tuple[str, str]]:
        """Return [(key, label)] for the concepts that resolved."""
        return [(self._resolved[c], self._labels[c]) for c in concepts if c in self._resolved]

    def describe(self) -> str:
        if not self._resolved:
            return "no recognisable metrics in the stats files"
        prefixes = ", ".join(self.solver_prefixes) or "none"
        return (f"solver prefix: {prefixes}; resolved "
                f"{len(self._resolved)}/{len(CONCEPTS) + len(FIXED_METRICS)} concepts")


#: Concepts used by each plotting section, in display order.
SEARCH_CONCEPTS = ["decisions", "conflicts", "propagations", "rlimit", "solver_time"]
ENCODING_CONCEPTS = ["bool_vars", "clauses", "lp_columns", "lp_rows"]
SUMMARY_CONCEPTS = ["total_time", "decisions", "conflicts", "solver_mem", "bool_vars"]
RATIO_CONCEPTS = ["total_time", "solve_time", "solver_time", "decisions",
                  "conflicts", "propagations", "restarts", "rlimit",
                  "bool_vars", "clauses"]
AGGREGATE_CONCEPTS = ["total_time", "solve_time", "solver_time", "decisions",
                      "conflicts", "propagations", "restarts", "rlimit", "solver_mem"]


def discover_time_components(exp: Experiment, metrics: Metrics) -> List[Tuple[str, str, float]]:
    """Auto-discover (key, label, scale) time components from the stats keys.

    Pass timings are named ``pass.<name>.time_ms`` and appear and disappear as
    the pipeline changes, so they are discovered rather than listed.
    """
    components: List[Tuple[str, str, float]] = []
    solver_time = metrics.key("solver_time")
    if solver_time:
        components.append((solver_time, "Solver", 1.0))
    for key in sorted(exp.stat_keys()):
        if key.startswith("pass.") and key.endswith(".time_ms"):
            components.append((key, key[len("pass."):-len(".time_ms")], 0.001))
        elif key == "achievers_analysis.total_time_seconds":
            components.append((key, "achievers", 1.0))
    return components


# ===========================================================================
# CSV output
# ===========================================================================

def _fmt(value: Optional[float], spec: str = ".6g") -> str:
    return format(value, spec) if value is not None else ""


def write_instances_csv(exp: Experiment, path: Path) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["domain", "instance", "system", "strategy", "mode", "schedule",
                    "status", "total_time", "wall_time", "horizon", "num_actions",
                    "last_memory_mb", "exit_code", "log_file"])
        for j in sorted(exp.jobs, key=lambda j: (j.domain, j.instance, j.system)):
            w.writerow([
                j.domain, j.instance, j.system, j.strategy, j.mode, j.schedule,
                j.status.value, _fmt(j.total_time, ".3f"), _fmt(j.wall_time, ".0f"),
                j.horizon if j.horizon is not None else "",
                j.num_actions if j.num_actions is not None else "",
                j.last_memory_mb if j.last_memory_mb is not None else "",
                j.exit_code if j.exit_code is not None else "",
                j.log_file.name,
            ])
    print(f"Wrote {path}", file=sys.stderr)


def _tally(jobs: Sequence[Job]) -> List[int]:
    counts = {s: 0 for s in Status}
    for j in jobs:
        counts[j.status] += 1
    return [counts[s] for s in Status] + [len(jobs)]


_TALLY_HEADERS = [s.value for s in Status] + ["total"]


def write_summary_by_domain_csv(exp: Experiment, path: Path) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        header = ["domain"]
        for s in exp.systems:
            header.extend(f"{s}_{h}" for h in _TALLY_HEADERS)
        w.writerow(header)
        for domain in exp.domains:
            row: List[object] = [domain]
            for system in exp.systems:
                row.extend(_tally(exp.by_domain_system(domain, system)))
            w.writerow(row)
    print(f"Wrote {path}", file=sys.stderr)


def write_summary_overall_csv(exp: Experiment, path: Path) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["system"] + _TALLY_HEADERS + ["mean_solve_time", "median_solve_time"])
        for system in exp.systems:
            jobs = exp.by_system(system)
            times = sorted(j.total_time for j in jobs
                           if j.status == Status.SOLVED and j.total_time is not None)
            mean = sum(times) / len(times) if times else None
            med = median(times) if times else None
            w.writerow([system] + _tally(jobs) + [_fmt(mean, ".3f"), _fmt(med, ".3f")])
        skipped = exp.skipped_empty + exp.skipped_unparsed
        if skipped:
            w.writerow([f"(unparseable: {skipped} log files)"] +
                       [""] * len(_TALLY_HEADERS) + ["", ""])
    print(f"Wrote {path}", file=sys.stderr)


def write_stats_full_csv(exp: Experiment, path: Path) -> None:
    keys = sorted(exp.stat_keys())
    if not keys:
        return
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["domain", "instance", "system", "status", "total_time"] + keys)
        for j in sorted(exp.jobs, key=lambda j: (j.domain, j.instance, j.system)):
            if not j.stats:
                continue
            w.writerow([j.domain, j.instance, j.system, j.status.value,
                        _fmt(j.total_time, ".3f")] +
                       [_fmt(j.stats.get(k)) for k in keys])
    print(f"Wrote {path}", file=sys.stderr)


def write_solver_comparison_csv(exp: Experiment, metrics: Metrics, path: Path) -> None:
    """Resolved metrics, one row per instance, one column block per system."""
    selected = metrics.select(AGGREGATE_CONCEPTS)
    if not selected:
        return
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        header = ["domain", "instance"]
        for system in exp.systems:
            header.extend(f"{system}|{label}" for _, label in selected)
        w.writerow(header)
        for domain, instance in exp.instance_keys():
            present = {s: exp.job_for(domain, instance, s) for s in exp.systems}
            present = {s: j for s, j in present.items()
                       if j and j.status == Status.SOLVED and j.stats}
            if len(present) < 2:
                continue
            row: List[str] = [domain, instance]
            for system in exp.systems:
                job = present.get(system)
                row.extend(_fmt(job.stats.get(k)) if job else "" for k, _ in selected)
            w.writerow(row)
    print(f"Wrote {path}", file=sys.stderr)


def write_variant_impact_csv(exp: Experiment, metrics: Metrics, path: Path) -> None:
    pairs = discover_variant_pairs(exp.systems)
    if not pairs:
        return
    selected = metrics.select(RATIO_CONCEPTS)
    extra_keys = sorted({k for base, ext, _ in pairs
                         for k in variant_only_keys(exp, base, ext)})

    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        header = ["domain", "instance", "base", "variant", "suffix"]
        for _, label in selected:
            header.extend([f"base|{label}", f"variant|{label}", f"ratio|{label}"])
        header.extend(f"variant|{k}" for k in extra_keys)
        w.writerow(header)
        for base, ext, suffix in pairs:
            for domain, instance, jb, je in common_solved(exp, base, ext):
                row: List[str] = [domain, instance, base, ext, suffix]
                for key, _ in selected:
                    vb, ve = jb.stats.get(key), je.stats.get(key)
                    row.extend([_fmt(vb), _fmt(ve),
                                _fmt(ve / vb, ".4f") if vb and ve is not None else ""])
                row.extend(_fmt(je.stats.get(k)) for k in extra_keys)
                w.writerow(row)
    print(f"Wrote {path}", file=sys.stderr)


def write_domain_aggregate_csv(exp: Experiment, metrics: Metrics, path: Path) -> None:
    selected = metrics.select(AGGREGATE_CONCEPTS)
    if not selected:
        return
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        header = ["domain", "system", "n_solved"]
        for _, label in selected:
            header.extend([f"median|{label}", f"mean|{label}"])
        w.writerow(header)
        for domain in exp.domains:
            for system in exp.systems:
                jobs = [j for j in exp.by_domain_system(domain, system)
                        if j.status == Status.SOLVED and j.stats]
                if not jobs:
                    continue
                row: List[str] = [domain, system, str(len(jobs))]
                for key, _ in selected:
                    vals = [j.stats[key] for j in jobs if key in j.stats]
                    if vals:
                        row.extend([_fmt(median(vals)), _fmt(sum(vals) / len(vals))])
                    else:
                        row.extend(["", ""])
                w.writerow(row)
    print(f"Wrote {path}", file=sys.stderr)


# ===========================================================================
# Analysis helpers
# ===========================================================================

def discover_variant_pairs(systems: Sequence[str]) -> List[Tuple[str, str, str]]:
    """Find (base, variant, suffix) triples where one name extends another with
    a dash suffix, e.g. ``els`` / ``els-fp``."""
    pairs: List[Tuple[str, str, str]] = []
    ordered = sorted(systems, key=len)
    for i, base in enumerate(ordered):
        for ext in ordered[i + 1:]:
            if ext.startswith(base + "-"):
                pairs.append((base, ext, ext[len(base) + 1:]))
    return pairs


def common_solved(exp: Experiment, a: str, b: str) -> List[Tuple[str, str, Job, Job]]:
    """Instances solved by both systems, both carrying stats."""
    out: List[Tuple[str, str, Job, Job]] = []
    for domain, instance in exp.instance_keys():
        ja = exp.job_for(domain, instance, a)
        jb = exp.job_for(domain, instance, b)
        if (ja and jb and ja.status == Status.SOLVED and jb.status == Status.SOLVED
                and ja.stats and jb.stats):
            out.append((domain, instance, ja, jb))
    return out


def variant_only_keys(exp: Experiment, base: str, variant: str) -> Set[str]:
    """Stat keys whose prefix exists for the variant but not for the base —
    i.e. counters the extra component contributes."""
    def prefixes(system: str) -> Set[str]:
        return {k.split(".", 1)[0] for j in exp.by_system(system) for k in j.stats}
    only = prefixes(variant) - prefixes(base)
    return {k for j in exp.by_system(variant) for k in j.stats
            if k.split(".", 1)[0] in only}


# ===========================================================================
# Plotting
# ===========================================================================

MARKERS = ["o", "s", "^", "D", "v", "P", "X", "*", "p", "h", "<", ">", "d", "H", "8"]


def system_colors(systems: Sequence[str]) -> Dict[str, object]:
    cmap = matplotlib.colormaps["tab10"]
    return {s: cmap(i % 10) for i, s in enumerate(systems)}


def domain_styles(domains: Sequence[str]) -> Dict[str, Tuple[str, object]]:
    cmap = matplotlib.colormaps["tab20"]
    return {d: (MARKERS[i % len(MARKERS)], cmap(i % 20)) for i, d in enumerate(domains)}


def _solved_times(exp: Experiment, system: str) -> List[float]:
    return sorted(j.total_time for j in exp.by_system(system)
                  if j.status == Status.SOLVED and j.total_time is not None)


def plot_cactus(exp: Experiment, path: Path, log_scale: bool) -> None:
    """Classic survival plot: cumulative instances solved against time."""
    colors = system_colors(exp.systems)
    fig, ax = plt.subplots(figsize=(12, 7))
    total = len(exp.instance_keys())

    for system in exp.systems:
        times = _solved_times(exp, system)
        if not times:
            continue
        ax.plot(range(1, len(times) + 1), times, marker="o", markersize=3,
                label=f"{system} ({len(times)})", color=colors[system])

    ax.axhline(y=exp.timeout, color="gray", linestyle="--", linewidth=0.8,
               label=f"timeout ({exp.timeout:.0f}s)")
    ax.set_xlabel(f"Instances solved (of {total})")
    ax.set_ylabel("Time (s)")
    if log_scale:
        ax.set_yscale("log")
    ax.set_title(f"Survival plot ({'log' if log_scale else 'linear'} scale)")
    ax.legend(fontsize=8, loc="best")
    ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout()
    fig.savefig(str(path))
    plt.close(fig)
    print(f"Wrote {path}", file=sys.stderr)


def _draw_by_domain(ax: plt.Axes, points: Dict[str, Tuple[List[float], List[float]]],
                    styles: Dict[str, Tuple[str, object]], size: int) -> None:
    """One scatter call per domain rather than per point — with a few hundred
    instances per pair and dozens of pairs, per-point calls dominate runtime."""
    for domain in sorted(points):
        xs, ys = points[domain]
        marker, color = styles.get(domain, ("o", "gray"))
        ax.scatter(xs, ys, marker=marker, color=color, s=size, alpha=0.7,
                   label=domain, edgecolors="none")


def _runtime_points(exp: Experiment, a: str, b: str
                    ) -> Dict[str, Tuple[List[float], List[float]]]:
    """Runtime of both systems per instance; unsolved runs sit at the timeout.

    Runs with no verdict yet (still in flight) are left out entirely — plotting
    them at the timeout would read as a failure they have not earned.
    """
    timeout = exp.timeout
    points: Dict[str, Tuple[List[float], List[float]]] = {}
    for domain, instance in exp.instance_keys():
        ja = exp.job_for(domain, instance, a)
        jb = exp.job_for(domain, instance, b)
        if ja is None or jb is None:
            continue
        if ja.status == Status.RUNNING or jb.status == Status.RUNNING:
            continue
        ta = ja.total_time if ja.status == Status.SOLVED and ja.total_time else timeout
        tb = jb.total_time if jb.status == Status.SOLVED and jb.total_time else timeout
        xs, ys = points.setdefault(domain, ([], []))
        xs.append(ta)
        ys.append(tb)
    return points


def _scatter_pair(ax: plt.Axes, exp: Experiment, a: str, b: str, log_scale: bool,
                  styles: Dict[str, Tuple[str, object]],
                  points: Dict[str, Tuple[List[float], List[float]]]) -> None:
    timeout = exp.timeout
    _draw_by_domain(ax, points, styles, size=30)

    lo = 0.01 if log_scale else 0.0
    hi = timeout * 1.1
    ax.plot([lo, hi], [lo, hi], "k--", linewidth=0.5, alpha=0.5)
    ax.axhline(y=timeout, color="gray", linestyle=":", linewidth=0.5, alpha=0.5)
    ax.axvline(x=timeout, color="gray", linestyle=":", linewidth=0.5, alpha=0.5)
    ax.set_xlabel(f"{a} time (s)")
    ax.set_ylabel(f"{b} time (s)")
    ax.set_title(f"{a} vs {b} ({'log' if log_scale else 'linear'})")
    if log_scale:
        ax.set_xscale("log")
        ax.set_yscale("log")
    ax.set_xlim(lo if log_scale else -timeout * 0.02, hi)
    ax.set_ylim(lo if log_scale else -timeout * 0.02, hi)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, which="both", alpha=0.3)


def plot_runtime_scatters(exp: Experiment, path: Path) -> None:
    """All pairwise runtime scatters, one page per pair."""
    styles = domain_styles(exp.domains)
    pairs = list(combinations(exp.systems, 2))
    if not pairs:
        return
    with PdfPages(str(path)) as pdf:
        for a, b in pairs:
            points = _runtime_points(exp, a, b)
            fig, (ax_lin, ax_log) = plt.subplots(1, 2, figsize=(16, 7))
            _scatter_pair(ax_lin, exp, a, b, False, styles, points)
            _scatter_pair(ax_log, exp, a, b, True, styles, points)
            _shared_legend(fig, ax_log)
            fig.suptitle(f"Runtime: {a} vs {b}", fontsize=13)
            fig.tight_layout(rect=(0, 0, 0.85, 0.95))
            pdf.savefig(fig)
            plt.close(fig)
    print(f"Wrote {path} ({len(pairs)} pages)", file=sys.stderr)


def _shared_legend(fig, ax, loc: str = "center right") -> None:
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc=loc, fontsize=7,
                   bbox_to_anchor=(1.0, 0.5), ncol=max(1, len(labels) // 18 + 1))


def _metric_scatter(ax: plt.Axes, common: Sequence[Tuple[str, str, Job, Job]],
                    key: str, title: str, xlabel: str, ylabel: str,
                    styles: Dict[str, Tuple[str, object]]) -> bool:
    """Log-log scatter of one metric between two systems.  False if empty."""
    points: Dict[str, Tuple[List[float], List[float]]] = {}
    values: List[float] = []
    for domain, _instance, ja, jb in common:
        va, vb = ja.stats.get(key, 0.0), jb.stats.get(key, 0.0)
        if va <= 0 or vb <= 0:
            continue
        values.extend((va, vb))
        xs, ys = points.setdefault(domain, ([], []))
        xs.append(va)
        ys.append(vb)
    if not values:
        ax.set_visible(False)
        return False
    _draw_by_domain(ax, points, styles, size=25)
    lo = max(min(values) * 0.5, 1e-4)
    hi = max(values) * 2
    ax.plot([lo, hi], [lo, hi], "k--", linewidth=0.5, alpha=0.5)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.set_xlim(lo, hi)
    ax.set_ylim(lo, hi)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, which="both", alpha=0.3)
    return True


def _metric_grid(pdf: PdfPages, common: Sequence[Tuple[str, str, Job, Job]],
                 selected: Sequence[Tuple[str, str]], a: str, b: str,
                 styles: Dict[str, Tuple[str, object]], title: str,
                 ncols: int = 3) -> None:
    ncols = min(ncols, len(selected))
    nrows = (len(selected) + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(6 * ncols, 5.5 * nrows), squeeze=False)
    flat = axes.flatten()
    plotted = False
    for idx, (key, label) in enumerate(selected):
        if _metric_scatter(flat[idx], common, key, label, a, b, styles):
            plotted = True
    for idx in range(len(selected), len(flat)):
        flat[idx].set_visible(False)
    if not plotted:
        plt.close(fig)
        return
    _shared_legend(fig, flat[0])
    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=(0, 0, 0.88, 0.95))
    pdf.savefig(fig)
    plt.close(fig)


def plot_search_metrics(exp: Experiment, metrics: Metrics, pdf: PdfPages) -> None:
    selected = metrics.select(SEARCH_CONCEPTS)
    if not selected:
        print("    (no search metrics resolved — skipped)", file=sys.stderr)
        return
    styles = domain_styles(exp.domains)
    for a, b in combinations(exp.systems, 2):
        common = common_solved(exp, a, b)
        if len(common) < 2:
            continue
        _metric_grid(pdf, common, selected, a, b, styles, f"Search effort: {a} vs {b}")


def plot_encoding(exp: Experiment, metrics: Metrics, pdf: PdfPages) -> None:
    selected = metrics.select(ENCODING_CONCEPTS)
    if not selected:
        print("    (no encoding-size metrics resolved — skipped)", file=sys.stderr)
        return
    styles = domain_styles(exp.domains)
    for a, b in combinations(exp.systems, 2):
        common = common_solved(exp, a, b)
        if len(common) < 2:
            continue
        _metric_grid(pdf, common, selected, a, b, styles,
                     f"Encoding size: {a} vs {b}", ncols=2)


def plot_variant_impact(exp: Experiment, metrics: Metrics, pdf: PdfPages) -> None:
    """For each auto-discovered ``base`` / ``base-suffix`` pair: a speedup bar
    chart, metric deltas, and box plots of the counters only the variant has."""
    pairs = discover_variant_pairs(exp.systems)
    if not pairs:
        print("    (no base/variant system pairs found — skipped)", file=sys.stderr)
        return
    styles = domain_styles(exp.domains)
    time_key = metrics.key("total_time")
    delta = metrics.select(["decisions", "conflicts", "propagations"])

    for base, variant, suffix in pairs:
        common = common_solved(exp, base, variant)
        if len(common) < 2:
            continue

        if time_key:
            ratios = [(je.stats[time_key] / jb.stats[time_key], f"{d}/{i}")
                      for d, i, jb, je in common
                      if jb.stats.get(time_key, 0.0) > 0 and je.stats.get(time_key, 0.0) > 0]
            if ratios:
                ratios.sort()
                values = [r for r, _ in ratios]
                labels = [l for _, l in ratios]
                # Bars carry log2 of the ratio so they grow from parity in both
                # directions and a 2x speed-up reads as the mirror image of a
                # 2x slowdown; the axis is relabelled back into plain ratios.
                heights = [log2(r) for r in values]
                colors = ["tab:green" if r < 1 else "tab:red" for r in values]
                faster = sum(1 for r in values if r < 1)
                fig, ax = plt.subplots(figsize=(max(10, len(values) * 0.25), 6))
                ax.bar(range(len(values)), heights, color=colors, alpha=0.75, width=0.8)
                ax.axhline(y=0.0, color="black", linewidth=0.8)
                span = ceil(max(abs(h) for h in heights))
                ticks = list(range(-span, span + 1))
                ax.set_yticks(ticks)
                ax.set_yticklabels([f"{2.0 ** t:g}x" for t in ticks])
                ax.set_ylabel(f"time ratio ({variant} / {base})")
                ax.set_title(
                    f"Impact of -{suffix}: green = faster, red = slower\n"
                    f"{faster}/{len(values)} instances faster "
                    f"(median ratio {median(values):.3f})"
                )
                ax.set_xticks(range(len(values)))
                ax.set_xticklabels(labels, rotation=90, fontsize=5)
                ax.grid(True, axis="y", alpha=0.3)
                fig.tight_layout()
                pdf.savefig(fig)
                plt.close(fig)

        if delta:
            _metric_grid(pdf, common, delta, base, variant, styles,
                         f"Search effort: {base} vs {variant}")

        keys = sorted(variant_only_keys(exp, base, variant))
        if not keys:
            continue
        groups: Dict[str, List[Job]] = {}
        for domain, _instance, _jb, je in common:
            groups.setdefault(domain, []).append(je)
        names = sorted(groups)

        ncols = min(3, len(keys))
        nrows = (len(keys) + ncols - 1) // ncols
        fig, axes = plt.subplots(nrows, ncols, figsize=(6 * ncols, 5 * nrows), squeeze=False)
        flat = axes.flatten()
        for idx, key in enumerate(keys):
            ax = flat[idx]
            data, labels = [], []
            for d in names:
                vals = [j.stats.get(key, 0.0) for j in groups[d]]
                if any(v > 0 for v in vals):
                    data.append(vals)
                    labels.append(d)
            if data:
                ax.boxplot(data)
                ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=7)
            ax.set_title(key.split(".", 1)[-1], fontsize=9)
            ax.grid(True, axis="y", alpha=0.3)
        for idx in range(len(keys), len(flat)):
            flat[idx].set_visible(False)
        fig.suptitle(f"Counters unique to {variant}", fontsize=13)
        fig.tight_layout(rect=(0, 0, 1, 0.95))
        pdf.savefig(fig)
        plt.close(fig)


def plot_time_breakdown(exp: Experiment, metrics: Metrics, pdf: PdfPages) -> None:
    """Per-system stacked bars of where the mean solve time goes, by domain."""
    components = discover_time_components(exp, metrics)
    total_key = metrics.key("total_time")
    if not components or not total_key:
        print("    (no time components resolved — skipped)", file=sys.stderr)
        return
    cmap = matplotlib.colormaps["tab20"]

    for system in exp.systems:
        jobs = exp.solved_with_stats(system)
        if not jobs:
            continue
        groups: Dict[str, List[Job]] = {}
        for j in jobs:
            groups.setdefault(j.domain, []).append(j)
        domains = sorted(groups)

        def mean_of(key: str, scale: float, domain: str) -> float:
            vals = [j.stats.get(key, 0.0) * scale for j in groups[domain]]
            return sum(vals) / len(vals)

        totals = {d: mean_of(total_key, 1.0, d) for d in domains}
        grand_total = sum(totals.values()) or 1.0

        # Keep components worth at least 1% of the total; the rest go to "Other"
        # together with whatever the accounted components fail to explain.
        active = [(k, lbl, sc) for k, lbl, sc in components
                  if sum(mean_of(k, sc, d) for d in domains) / grand_total >= 0.01]
        if not active:
            continue

        names = [lbl for _, lbl, _ in active] + ["unaccounted"]
        values: Dict[str, List[float]] = {}
        for d in domains:
            row = [mean_of(k, sc, d) for k, _, sc in active]
            row.append(max(0.0, totals[d] - sum(row)))
            values[d] = row

        fig, ax = plt.subplots(figsize=(max(10, len(domains) * 0.8), 7))
        x = range(len(domains))
        bottom = [0.0] * len(domains)
        for i, name in enumerate(names):
            heights = [values[d][i] for d in domains]
            ax.bar(x, heights, bottom=bottom, label=name, color=cmap(i % 20), alpha=0.85)
            bottom = [b + h for b, h in zip(bottom, heights)]
        ax.set_xticks(list(x))
        ax.set_xticklabels(domains, rotation=45, ha="right", fontsize=8)
        ax.set_ylabel("Mean time (s)")
        ax.set_title(f"Time breakdown: {system}")
        ax.legend(fontsize=7, loc="best", ncol=2)
        ax.grid(True, axis="y", alpha=0.3)
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)


def plot_domain_summaries(exp: Experiment, metrics: Metrics, pdf: PdfPages) -> None:
    selected = metrics.select(SUMMARY_CONCEPTS)
    if not selected:
        print("    (no summary metrics resolved — skipped)", file=sys.stderr)
        return
    colors = system_colors(exp.systems)

    for domain in exp.domains:
        per_system = {s: [j for j in exp.by_domain_system(domain, s)
                          if j.status == Status.SOLVED and j.stats]
                      for s in exp.systems}
        per_system = {s: jobs for s, jobs in per_system.items() if len(jobs) >= 3}
        if len(per_system) < 2:
            continue
        systems = sorted(per_system)

        fig, axes = plt.subplots(1, len(selected), figsize=(3.5 * len(selected), 5),
                                 squeeze=False)
        flat = axes.flatten()
        width = 0.8 / len(systems)
        for m_idx, (key, label) in enumerate(selected):
            ax = flat[m_idx]
            for s_idx, system in enumerate(systems):
                vals = [j.stats.get(key, 0.0) for j in per_system[system]]
                offset = (s_idx - len(systems) / 2 + 0.5) * width
                ax.bar(offset, median(vals), width=width, color=colors[system],
                       alpha=0.85, label=system if m_idx == 0 else None)
            ax.set_title(label, fontsize=9)
            ax.set_xticks([])
            ax.grid(True, axis="y", alpha=0.3)

        handles, labels = flat[0].get_legend_handles_labels()
        if handles:
            fig.legend(handles, labels, loc="lower center", fontsize=7,
                       ncol=min(4, len(systems)), bbox_to_anchor=(0.5, 0.01))
        n = min(len(v) for v in per_system.values())
        fig.suptitle(f"Domain: {domain} (median over commonly solved, n>={n})", fontsize=13)
        fig.tight_layout(rect=(0, 0.08, 1, 0.93))
        pdf.savefig(fig)
        plt.close(fig)


# ===========================================================================
# Report orchestration
# ===========================================================================

SECTIONS: Dict[str, str] = {
    "search":         "Search-effort metric scatters (pairwise)",
    "variant-impact": "Base vs variant impact (auto-discovered name pairs)",
    "time-breakdown": "Time breakdown stacked bars (per system)",
    "encoding":       "Encoding size comparison (pairwise)",
    "domain-summary": "Per-domain summary (grouped medians)",
}

_CSV_DISPATCH = {
    "search":         ("stats_solver_comparison.csv", write_solver_comparison_csv),
    "variant-impact": ("stats_variant_impact.csv", write_variant_impact_csv),
    "domain-summary": ("stats_domain_aggregate.csv", write_domain_aggregate_csv),
}

_PLOT_DISPATCH = {
    "search":         plot_search_metrics,
    "variant-impact": plot_variant_impact,
    "time-breakdown": plot_time_breakdown,
    "encoding":       plot_encoding,
    "domain-summary": plot_domain_summaries,
}


def generate_stats_report(exp: Experiment, output_dir: Path,
                          sections: Optional[Sequence[str]]) -> None:
    if not any(j.stats for j in exp.jobs):
        print("No stats files loaded — skipping metric analysis "
              "(pass --stats-dir to enable it).", file=sys.stderr)
        return
    active = set(SECTIONS) if sections is None else (set(sections) & set(SECTIONS))
    if not active:
        return

    metrics = Metrics(exp.stat_keys())
    plural = "section" if len(active) == 1 else "sections"
    print(f"\nMetric analysis ({len(active)} {plural}) — {metrics.describe()}", file=sys.stderr)

    write_stats_full_csv(exp, output_dir / "stats_full.csv")
    for name in SECTIONS:
        if name in active and name in _CSV_DISPATCH:
            filename, writer = _CSV_DISPATCH[name]
            writer(exp, metrics, output_dir / filename)

    pdf_path = output_dir / "stats_analysis.pdf"
    with PdfPages(str(pdf_path)) as pdf:
        for name in SECTIONS:
            if name in active:
                print(f"  Plotting: {SECTIONS[name]}", file=sys.stderr)
                _PLOT_DISPATCH[name](exp, metrics, pdf)
    print(f"Wrote {pdf_path}", file=sys.stderr)


def print_summary(exp: Experiment) -> None:
    print("\n=== Summary ===", file=sys.stderr)
    width = max(len(s) for s in exp.systems)
    columns = [s for s in Status]
    header = " " * (width + 2) + "  ".join(f"{s.value:>10}" for s in columns) + f"{'total':>10}"
    print(header, file=sys.stderr)
    for system in exp.systems:
        counts = _tally(exp.by_system(system))
        cells = "  ".join(f"{c:>10}" for c in counts[:-1])
        print(f"  {system:<{width}}{cells}{counts[-1]:>10}", file=sys.stderr)

    skipped = exp.skipped_empty + exp.skipped_unparsed
    if skipped:
        print(f"\n  {skipped} log files not counted "
              f"({exp.skipped_empty} empty, {exp.skipped_unparsed} unparsed).", file=sys.stderr)
    running = sum(1 for j in exp.jobs if j.status == Status.RUNNING)
    if running:
        print(f"\n  {running} jobs have no verdict yet (no plan, no status line, no exit "
              f"code) — still in flight, or the array was torn down. They are excluded "
              f"from every comparison rather than counted as failures.", file=sys.stderr)
    unknown = sum(1 for j in exp.jobs if j.status == Status.UNKNOWN)
    if unknown:
        print(f"  {unknown} jobs exited cleanly but reported no result — worth a look.",
              file=sys.stderr)
    missing = len(exp.systems) * len(exp.instance_keys()) - len(exp.jobs)
    if missing > 0:
        print(f"  {missing} (instance, system) cells have no log at all — expected while "
              f"the run is still being submitted.", file=sys.stderr)


def generate_all_outputs(exp: Experiment, output_dir: Path,
                         sections: Optional[Sequence[str]]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    write_instances_csv(exp, output_dir / "instances.csv")
    write_summary_by_domain_csv(exp, output_dir / "summary_by_domain.csv")
    write_summary_overall_csv(exp, output_dir / "summary_overall.csv")

    plot_cactus(exp, output_dir / "cactus.pdf", log_scale=True)
    plot_cactus(exp, output_dir / "cactus_linear.pdf", log_scale=False)
    plot_runtime_scatters(exp, output_dir / "scatters.pdf")

    if sections is None or sections:
        generate_stats_report(exp, output_dir, sections)

    print_summary(exp)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyse SLURM benchmark results from RantanPlan experiments.",
    )
    parser.add_argument("logs_dir", help="Directory with SLURM .out/.err log files")
    parser.add_argument("--stats-dir", default=None,
                        help="Directory with the .stat files the runs wrote "
                             "(optional; enables the metric analysis)")
    parser.add_argument("-o", "--output-dir", default="slurm_report",
                        help="Output directory (default: slurm_report)")
    parser.add_argument("--timeout", type=float, default=None,
                        help="Override the per-job wall-clock limit in seconds "
                             "(measured from the logs if not set)")
    parser.add_argument("--stats-sections", default=None,
                        help="Comma-separated sections to generate. Available: "
                             + ", ".join(SECTIONS) + ". Default: all")
    parser.add_argument("--skip-stats-sections", default=None,
                        help="Comma-separated sections to skip")
    parser.add_argument("--no-stats", action="store_true",
                        help="Skip all metric analysis")
    parser.add_argument("--exclude-strategy", action="append", default=[],
                        metavar="SUBSTR",
                        help="Exclude strategies whose name contains SUBSTR (repeatable)")
    return parser.parse_args()


def resolve_sections(args: argparse.Namespace) -> Optional[List[str]]:
    if args.no_stats:
        return []
    if args.stats_sections:
        requested = [s.strip() for s in args.stats_sections.split(",") if s.strip()]
        unknown = [s for s in requested if s not in SECTIONS]
        if unknown:
            sys.exit(f"Error: unknown section(s) {', '.join(unknown)}. "
                     f"Available: {', '.join(SECTIONS)}")
        return requested
    if args.skip_stats_sections:
        skip = {s.strip() for s in args.skip_stats_sections.split(",")}
        return [s for s in SECTIONS if s not in skip]
    return None


def main() -> None:
    args = parse_args()

    logs_dir = Path(args.logs_dir)
    if not logs_dir.is_dir():
        sys.exit(f"Error: {logs_dir} is not a directory")
    stats_dir = Path(args.stats_dir) if args.stats_dir else None
    if stats_dir and not stats_dir.is_dir():
        sys.exit(f"Error: {stats_dir} is not a directory")
    # Validate before the (slow) load so a typo fails in milliseconds.
    sections = resolve_sections(args)

    exp = load_experiment(logs_dir, stats_dir, args.timeout, args.exclude_strategy or None)
    generate_all_outputs(exp, Path(args.output_dir), sections)


if __name__ == "__main__":
    main()
