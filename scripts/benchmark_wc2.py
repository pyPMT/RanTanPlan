#!/usr/bin/env python3
"""
Unified head-to-head benchmark for the anchor-search (els-wc2) experiment.

One script, -j parallel, N repetitions with per-(instance, strategy) MEDIAN
reporting (design ideas/new_planner/anchor-search-design.md §12).  Runs three
instance pools:

  small : pddl/small-test/      (15 curated diverse instances, timeout 120 s)
  hard  : pddl/hard-test/       (10 harder instances,          timeout 300 s)
  extra : a curated set of UNSEEN numeric-domains instances (variety, §12)

The `extra` pool pulls domains NOT present in small/hard-test from
/home/foobar/git/numeric-domains, to guard against overfitting to the tuned
suites.  benchmark_small.py does NOT validate plans; correctness is gated
separately by `python test.py --quick` (UP PlanValidator).

Usage:
    python scripts/benchmark_wc2.py --suite small -s els -s els-wc2 \
        --timeout 120 -j 4 --reps 3 --csv small.csv
"""
import argparse
import csv
import statistics
import sys
from pathlib import Path

# Reuse the proven runner + discovery.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from benchmark_small import run_instance, discover_instances  # noqa: E402

ND = Path("/home/foobar/git/numeric-domains")

# Curated UNSEEN numeric-domains instances (domains not in small/hard-test).
# Kept modest; each is verified to at least parse+run with the base encoding.
EXTRA_SPECS = [
    # Verified to parse+run with the base encoding (driverlog dropped: PDDL
    # parse error on 'driven').  elevators/planes solve fast (unseen-domain
    # no-regression check); gear-car is numeric-hard (unseen-domain stress).
    ("elevators", ["p02", "p04", "p06"]),
    ("planes", ["planes_1", "planes_2", "planes_3"]),
    ("gear-car", ["p01", "p02", "p03"]),
]


# Large scaling instances in the long-horizon-friendly domains (from
# numeric-domains) — the regime where the monolithic els encoding blows up but
# a stepping-stone dive stays cheap.  The point is to find LONG PLANS that were
# impossible before (els TO, els-wc2 solves).
LONG_SPECS = [
    ("zenotravel", ["pfile13", "pfile15", "pfile17", "pfile19", "pfile21", "pfile23"]),
    ("rover", ["pfile15", "pfile17", "pfile19", "pfile20"]),
    ("tpp", ["p15", "p20", "p25", "p30", "p35", "p40"]),
    ("depots", ["pfile12", "pfile14", "pfile16", "pfile18", "pfile20"]),
    ("delivery", ["pfile10", "pfile13", "pfile16", "pfile20"]),
    ("elevators", ["p16", "p20", "p24", "p28", "p30"]),
    ("markettrader", ["pfile10", "pfile13", "pfile16", "pfile20"]),
]


def discover_specs(specs):
    out = []
    for domain, stems in specs:
        ddir = ND / domain
        dom = ddir / "domain.pddl"
        if not dom.exists():
            continue
        inst_dir = ddir / "instances"
        for stem in stems:
            prob = inst_dir / f"{stem}.pddl"
            if prob.exists():
                out.append((f"{domain}/{stem}", str(dom), str(prob)))
    return out


def discover_extra():
    """Resolve the curated numeric-domains instances that exist on disk."""
    out = []
    for domain, stems in EXTRA_SPECS:
        ddir = ND / domain
        dom = ddir / "domain.pddl"
        if not dom.exists():
            continue
        inst_dir = ddir / "instances"
        for stem in stems:
            prob = inst_dir / f"{stem}.pddl"
            if prob.exists():
                out.append((f"{domain}/{stem}", str(dom), str(prob)))
    return out


def pool_instances(suite):
    if suite == "small":
        return discover_instances(Path("pddl/small-test"))
    if suite == "hard":
        return discover_instances(Path("pddl/hard-test"))
    if suite == "extra":
        return discover_extra()
    if suite == "long":
        return discover_specs(LONG_SPECS)
    raise ValueError(suite)


def worker(item):
    name, dom, prob, strat, timeout, rep = item
    r = run_instance(dom, prob, strat, timeout)
    return name, strat, rep, r


def main():
    ap = argparse.ArgumentParser(description="els-wc2 unified head-to-head")
    ap.add_argument("--suite", choices=["small", "hard", "extra", "long"], required=True)
    ap.add_argument("-s", "--strategy", action="append", required=True,
                    help="Strategy (repeatable), e.g. -s els -s els-wc2")
    ap.add_argument("--timeout", type=int, default=120)
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("-j", "--jobs", type=int, default=4)
    ap.add_argument("--csv", type=str, default=None)
    args = ap.parse_args()

    instances = pool_instances(args.suite)
    if not instances:
        print(f"No instances found for suite '{args.suite}'")
        sys.exit(1)

    strategies = args.strategy
    work = []
    for name, dom, prob in instances:
        for s in strategies:
            for rep in range(args.reps):
                work.append((name, dom, prob, s, args.timeout, rep))

    total = len(work)
    print(f"Suite '{args.suite}': {len(instances)} instances x "
          f"{len(strategies)} strategies x {args.reps} reps = {total} runs, "
          f"timeout {args.timeout}s, jobs {args.jobs}")

    # (name, strat) -> list of run dicts across reps
    runs = {}
    from concurrent.futures import ProcessPoolExecutor, as_completed
    done = 0
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        futs = {ex.submit(worker, it): it for it in work}
        for fut in as_completed(futs):
            done += 1
            name, strat, rep, r = fut.result()
            runs.setdefault((name, strat), []).append(r)
            st = f"{r['time']:.1f}s" if r["solved"] else "TO"
            print(f"  [{done}/{total}] {name} / {strat} r{rep}... {st}", flush=True)

    # Per-(instance, strategy) medians.
    rows = []
    for name, dom, prob in instances:
        for s in strategies:
            rs = runs.get((name, s), [])
            solved = [x for x in rs if x["solved"]]
            n_solved = len(solved)
            # Median over reps that solved; if none solved, time = timeout.
            if solved:
                med_t = statistics.median(x["time"] for x in solved)
                med_len = statistics.median(x["plan_len"] for x in solved)
                med_h = statistics.median(x["horizon"] for x in solved)
            else:
                med_t, med_len, med_h = args.timeout, 0, 0
            rows.append({
                "instance": name, "strategy": s,
                "solved_reps": n_solved, "reps": len(rs),
                "median_time": round(med_t, 2),
                "median_len": med_len, "median_horizon": med_h,
            })

    # Summary table.
    print()
    hdr = f"{'Instance':32s}"
    for s in strategies:
        hdr += f" | {s:>16s}"
    print(hdr)
    print("-" * len(hdr))
    cov = {s: 0 for s in strategies}
    by_inst = {}
    for r in rows:
        by_inst.setdefault(r["instance"], {})[r["strategy"]] = r
    for name, dom, prob in instances:
        line = f"{name:32s}"
        for s in strategies:
            r = by_inst.get(name, {}).get(s)
            if r and r["solved_reps"] > 0:
                line += f" | {r['median_time']:>8.1f}s {r['solved_reps']}/{r['reps']}    "
                cov[s] += 1
            else:
                line += f" | {'TO':>8s} 0/{args.reps}    "
        print(line)
    print()
    for s in strategies:
        print(f"  {s:>16s}: {cov[s]}/{len(instances)} solved")

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=[
                "instance", "strategy", "solved_reps", "reps",
                "median_time", "median_len", "median_horizon"])
            w.writeheader()
            for r in rows:
                w.writerow(r)
        print(f"\nSaved {args.csv}")


if __name__ == "__main__":
    main()
