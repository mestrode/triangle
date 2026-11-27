#!/usr/bin/env python3
"""Benchmark runner for the performance gate (wayfinder ticket #11).

Times whole-process wall time over the harness corpus (tools/oracle/
generate_corpus.py): the pinned C oracle (tools/oracle/build-oracle.sh)
against a candidate binary, under the same process-level invocation contract
as the differential harness - './candidate <options> <input>' inside a
prepared run directory, always through a symlink named 'candidate', stdout
redirected to a file. The timed quantity is whole-process wall time (exec to
exit): input reading and output writing are included, identically on both
sides. The single-threaded benchmark corpus IS the harness corpus; this
runner adds no inputs of its own.

Timed set: by default sizes m and l of every case class x option family
(80 runs). Size s stays corpus-only: its runs are startup-dominated and
measure process overhead, not the meshing kernels. --sizes and the
--cases/--families globs narrow the set exactly like run-harness.sh.

Interleaving: for every timed entry, oracle and candidate runs alternate -
--warmup untimed warmup runs per side first, then --repeats timed runs per
side. Shared-vCPU neighbors and wall-clock drift therefore hit both sides
equally; the candidate is always measured in the same loop as the oracle,
never against stale numbers.

Statistic: per entry the MEDIAN wall time per side; ratio =
median(candidate) / median(oracle). The gate (--gate TOL) passes iff neither
any per-run ratio nor the geometric mean over all timed entries exceeds TOL.
TOL is deliberately not hardcoded: it is the binding tolerance set in ticket
#11 from the measured build-server noise floor, and it should be re-derived
(run the noise probe) whenever the gate machine changes.

Report totals (issue #64): besides the overall fields, the report's totals
carry a per-size-class split ("by_size") - for every size class present in
the timed rows, the sum of medians per side, the total ratio, the geomean of
the class's per-run ratios, and the class's worst ratio with its case.
Report-only addition: measurement behavior, run counts, interleaving and
all existing fields are unchanged.

Noise-floor probe: point --candidate at the pinned oracle itself. The ratio
distribution around 1.0 is the machine's measurement noise; ticket #11
derived the tolerance band from exactly this.

MT axis: --threads N passes TRIANGLE_THREADS=N to the CANDIDATE only (the
pinned C oracle is single-threaded and ignores the variable). An MT variant
of a run is the same corpus entry at a different N; the scaling baseline is
the same candidate at N=1 / without the variable, not the C oracle. The
single-threaded tolerance gate never sets the variable. Pinning rule: --pin
CPULIST prefixes every invocation with 'taskset -c CPULIST' on both sides,
so each thread-count variant can be pinned to exactly N vCPUs.

Usage:
  python3 tools/oracle/bench.py --candidate BIN [--baseline BIN]
        [--sizes m,l] [--cases GLOB] [--families GLOB]
        [--repeats N] [--warmup N] [--threads N]
        [--gate TOL] [--report PATH] [--quiet]

Exit codes: 0 = ran (with --gate also: within tolerance); 1 = gate exceeded;
2 = usage/config error.
"""

import argparse
import fnmatch
import json
import math
import os
import platform
import shutil
import statistics
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORK = os.path.join(SCRIPT_DIR, "work")
DEFAULT_BASELINE = os.path.join(WORK, "oracle", "triangle-oracle")
DEFAULT_CORPUS = os.path.join(WORK, "corpus")
DEFAULT_BENCH_DIR = os.path.join(WORK, "bench")
DEFAULT_REPORT = os.path.join(WORK, "reports", "last-bench.json")
DEFAULT_SIZES = "m,l"


def log(msg):
    sys.stdout.write(msg + "\n")
    sys.stdout.flush()


def die(msg, code=2):
    sys.stdout.write("[bench] FAIL: %s\n" % msg)
    sys.stdout.flush()
    sys.exit(code)


def load_manifest(corpus_dir):
    with open(os.path.join(corpus_dir, "manifest.json")) as f:
        manifest = json.load(f)
    if manifest.get("manifest_version") != 1:
        die("unsupported manifest version in %s" % corpus_dir)
    return manifest


def timed_entries(manifest, sizes, case_filter, family_filter):
    entries = []
    for case in manifest["cases"]:
        if case["size"] not in sizes:
            continue
        if case_filter and not fnmatch.fnmatchcase(case["id"], case_filter):
            continue
        for family in case["families"]:
            if family_filter and not fnmatch.fnmatchcase(family["tag"], family_filter):
                continue
            entries.append((case, family))
    return entries


def prepare_run_dir(bench_root, side, case, family, corpus_dir):
    path = os.path.join(bench_root, case["id"], family["tag"], side)
    if os.path.exists(path):
        shutil.rmtree(path)
    os.makedirs(path)
    shutil.copy(os.path.join(corpus_dir, case["id"], family["input"]),
                os.path.join(path, family["input"]))
    return path


def run_once(run_dir, tool, argv, env_extra, label, pin=None):
    """One process-level run, timed; mirrors the harness invocation contract.

    With pin set, every invocation runs under taskset -c PIN (MT pinning)."""
    if not os.path.lexists(os.path.join(run_dir, "candidate")):
        os.symlink(os.path.abspath(tool), os.path.join(run_dir, "candidate"))
    env = dict(os.environ)
    env.update(env_extra)
    with open(os.path.join(run_dir, "candidate-stdout.txt"), "wb") as out:
        cmd = ["./candidate"] + argv
        if pin:
            cmd = ["taskset", "-c", pin] + cmd
        t0 = time.perf_counter()
        proc = subprocess.run(cmd, cwd=run_dir,
                              stdout=out, stderr=subprocess.STDOUT, env=env)
        dt = time.perf_counter() - t0
    if proc.returncode != 0:
        die("exited %d during %s" % (proc.returncode, label))
    return dt


def spread(xs):
    med = statistics.median(xs)
    return (max(xs) - min(xs)) / med if med > 0 else float("nan")


def geomean(xs):
    return math.exp(sum(math.log(x) for x in xs) / len(xs))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--candidate", required=True,
                    help="candidate binary (triangle-CLI interface)")
    ap.add_argument("--baseline", default=DEFAULT_BASELINE,
                    help="baseline binary (default: the pinned C oracle)")
    ap.add_argument("--corpus", default=DEFAULT_CORPUS,
                    help="corpus directory with manifest.json")
    ap.add_argument("--bench-dir", default=DEFAULT_BENCH_DIR,
                    help="scratch dir for run directories")
    ap.add_argument("--sizes", default=DEFAULT_SIZES,
                    help="comma-separated size classes to time (default m,l; "
                         "s is corpus-only: startup-dominated)")
    ap.add_argument("--cases", default=None,
                    help="fnmatch glob over case ids (like run-harness.sh)")
    ap.add_argument("--families", default=None,
                    help="fnmatch glob over family tags (like run-harness.sh)")
    ap.add_argument("--repeats", type=int, default=25,
                    help="timed runs per side, interleaved (default 25, the "
                         "gate configuration from ticket #11)")
    ap.add_argument("--warmup", type=int, default=3,
                    help="untimed warmup runs per side (default 3)")
    ap.add_argument("--threads", type=int, default=None,
                    help="pass TRIANGLE_THREADS=N to the candidate (MT variants)")
    ap.add_argument("--pin", default=None,
                    help="taskset CPU list applied to every invocation on both "
                         "sides, e.g. 0-3 (MT pinning: pin each variant to N "
                         "vCPUs via 'taskset -c')")
    ap.add_argument("--gate", type=float, default=None,
                    help="binding tolerance: fails (exit 1) if any per-run "
                         "ratio or the geometric mean exceeds it")
    ap.add_argument("--report", default=DEFAULT_REPORT,
                    help="JSON report path")
    ap.add_argument("--quiet", action="store_true", help="suppress per-run rows")
    args = ap.parse_args()

    for binary, name in ((args.baseline, "baseline"), (args.candidate, "candidate")):
        if not (os.path.isfile(binary) and os.access(binary, os.X_OK)):
            die("%s binary is not executable: %s" % (name, binary))
    if args.repeats < 1:
        die("--repeats must be >= 1")
    if args.warmup < 0:
        die("--warmup must be >= 0")
    if args.threads is not None and args.threads < 1:
        die("--threads must be >= 1")

    sizes = [s.strip() for s in args.sizes.split(",") if s.strip()]
    manifest = load_manifest(args.corpus)
    entries = timed_entries(manifest, set(sizes), args.cases, args.families)
    if not entries:
        die("no corpus runs selected (sizes=%s, cases=%s, families=%s)"
            % (args.sizes, args.cases, args.families))

    log("[bench] candidate:      %s" % os.path.abspath(args.candidate))
    log("[bench] baseline:       %s" % os.path.abspath(args.baseline))
    log("[bench] timed entries: %d (sizes %s, %d repeats + %d warmup per side, interleaved)"
        % (len(entries), ",".join(sizes), args.repeats, args.warmup))
    if args.threads is not None:
        log("[bench] MT variant: TRIANGLE_THREADS=%d (candidate only)" % args.threads)

    rows = []
    for idx, (case, family) in enumerate(entries, 1):
        label = "%s/%s" % (case["id"], family["tag"])
        base_dir = prepare_run_dir(args.bench_dir, "oracle", case, family, args.corpus)
        cand_dir = prepare_run_dir(args.bench_dir, "candidate", case, family, args.corpus)
        env_base = {"ORACLE_HARNESS_CASE": case["id"], "ORACLE_HARNESS_FAMILY": family["tag"]}
        env_cand = dict(env_base)
        if args.threads is not None:
            env_cand["TRIANGLE_THREADS"] = str(args.threads)
        for _ in range(args.warmup):
            run_once(base_dir, args.baseline, family["argv"], env_base,
                     "%s (baseline warmup)" % label, args.pin)
            run_once(cand_dir, args.candidate, family["argv"], env_cand,
                     "%s (candidate warmup)" % label, args.pin)
        t_base = []
        t_cand = []
        for _ in range(args.repeats):
            t_base.append(run_once(base_dir, args.baseline, family["argv"],
                                   env_base, "%s (baseline)" % label, args.pin))
            t_cand.append(run_once(cand_dir, args.candidate, family["argv"],
                                   env_cand, "%s (candidate)" % label, args.pin))
        med_base = statistics.median(t_base)
        med_cand = statistics.median(t_cand)
        rows.append({
            "case": case["id"], "family": family["tag"], "size": case["size"],
            "options": family["options"], "argv": family["argv"],
            "oracle_median_s": med_base, "candidate_median_s": med_cand,
            "ratio": med_cand / med_base,
            "oracle_min_s": min(t_base), "oracle_max_s": max(t_base),
            "candidate_min_s": min(t_cand), "candidate_max_s": max(t_cand),
            "oracle_spread": spread(t_base), "candidate_spread": spread(t_cand),
            "oracle_runs_s": t_base, "candidate_runs_s": t_cand,
        })
        if not args.quiet:
            log("[%3d/%d] %-20s %-8s options=%-12s oracle=%.4fs cand=%.4fs "
                "ratio=%.3f (spread %.1f%%/%.1f%%)"
                % (idx, len(entries), case["id"], family["tag"],
                   family["options"] or "(none)", med_base, med_cand,
                   rows[-1]["ratio"], 100.0 * rows[-1]["oracle_spread"],
                   100.0 * rows[-1]["candidate_spread"]))

    total_base = sum(r["oracle_median_s"] for r in rows)
    total_cand = sum(r["candidate_median_s"] for r in rows)
    ratios = [r["ratio"] for r in rows]
    gm = geomean(ratios)
    worst = max(rows, key=lambda r: r["ratio"])
    log("")
    log("[bench] oracle total (sum of per-run medians):    %.4f s" % total_base)
    log("[bench] candidate total (sum of per-run medians): %.4f s" % total_cand)
    log("[bench] total ratio (candidate/oracle):          %.3f" % (total_cand / total_base))
    log("[bench] geometric mean of per-run ratios:        %.3f" % gm)
    log("[bench] worst per-run ratio: %.3f (%s/%s)"
        % (worst["ratio"], worst["case"], worst["family"]))

    # Per-size-class totals (issue #64, report-only): for every size class
    # present in the timed rows, the same fields as the overall totals,
    # computed over that class's rows. The overall fields, the measurement
    # itself, and all existing report fields are unchanged; existing reports
    # are not rewritten.
    by_size = {}
    for size in sorted(set(r["size"] for r in rows)):
        sub = [r for r in rows if r["size"] == size]
        sub_base = sum(r["oracle_median_s"] for r in sub)
        sub_cand = sum(r["candidate_median_s"] for r in sub)
        sub_worst = max(sub, key=lambda r: r["ratio"])
        by_size[size] = {
            "oracle_sum_medians_s": sub_base,
            "candidate_sum_medians_s": sub_cand,
            "total_ratio": sub_cand / sub_base,
            "geomean_ratio": geomean([r["ratio"] for r in sub]),
            "worst_ratio": sub_worst["ratio"],
            "worst_case": "%s/%s" % (sub_worst["case"], sub_worst["family"]),
        }

    report = {
        "bench_version": 1,
        "ticket": "mestrode/triangle#11",
        "machine": {"platform": platform.platform(), "cpus": os.cpu_count()},
        "config": {
            "baseline": os.path.abspath(args.baseline),
            "candidate": os.path.abspath(args.candidate),
            "sizes": sizes, "repeats": args.repeats, "warmup": args.warmup,
            "threads": args.threads,
            "cases": args.cases, "families": args.families,
        },
        "rows": rows,
        "totals": {
            "oracle_sum_medians_s": total_base,
            "candidate_sum_medians_s": total_cand,
            "total_ratio": total_cand / total_base,
            "geomean_ratio": gm,
            "worst_ratio": worst["ratio"],
            "worst_case": "%s/%s" % (worst["case"], worst["family"]),
            "by_size": by_size,
        },
    }
    os.makedirs(os.path.dirname(args.report), exist_ok=True)
    with open(args.report, "w") as f:
        json.dump(report, f, indent=2)
        f.write("\n")
    log("[bench] report: %s" % args.report)

    if args.gate is not None:
        if worst["ratio"] > args.gate or gm > args.gate:
            die("gate FAILED: worst per-run ratio %.3f, geomean %.3f, tolerance %.2f"
                % (worst["ratio"], gm, args.gate), 1)
        log("[bench] gate PASSED: worst per-run ratio %.3f, geomean %.3f (tolerance %.2f)"
            % (worst["ratio"], gm, args.gate))
    sys.exit(0)


if __name__ == "__main__":
    main()
