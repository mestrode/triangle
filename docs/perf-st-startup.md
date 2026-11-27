# Performance ST startup floor: the per-exec fixed premium, quantified and levered

Diagnosis deliverable for the ST end-gate lever 1 (issue #62). Branch
`vibe/st-startup-floor`, branched from master `52d8160` (R4 head, PR #61).
The repository change on this branch is one CLI main-path amortization plus this
document; the build recipe is untouched.

Evidence rules per #50: every number below was measured on the build server for
this run, in the sessions named. Absolute times are NOT comparable across boxes
or across sessions; only within-session comparisons are claims. Expectations
about effects are explicitly marked as estimates. Raw evidence is kept on the
box under `tools/oracle/work/st-diag/` (probe JSONs, strace transcripts, perf
data, loader statistics, binary comparisons).

## 1. Environment and provenance

- Machine: managed Hetzner vServer, server_type cx23 (2 vCPU, 4 GB RAM, shared
  vCPU, x86), the same box class as the standing end-gate numbers. OS: Debian
  12, kernel 6.1.0-56-amd64, glibc 2.36, gcc 12.2.0.
- Toolchain: rustc 1.100.0, pinned by the committed `rust-toolchain.toml`.
- Candidate at master: fresh `cargo build --release --locked` at `52d8160`,
  sha256 `fd532a996cd063ce972b9a3dab01c9b25d33e90555b3d7fc1d523adff9092172`,
  which reproduces the standing R4 head binary byte-for-byte. Recipe
  determinism is therefore demonstrated for the committed recipe this run.
- Oracle: `tools/oracle/build-oracle.sh`, verified against the pin before any
  measurement: sha256
  `9af462136c77f812f44304ed2befe3bdf958f0d95eea879793db5e2678bf34e4`.
- Controls: `/bin/true` (fork/exec floor) and `rustnull`, a one-line Rust
  `fn main() {}` binary built with the same release profile (runtime-init
  floor), both built/used this run.
- Instrumentation: `strace` 6.1, `perf` 6.1.192 (perf_event_paranoid lowered
  from 3 to 0 for the profiles this run), `LD_SHOW_AUXV`-style loader
  statistics via `LD_DEBUG=statistics`, `size`, `readelf`, `ldd`.
- Session naming: probe 1 (`startup-probe-master.json`), probe 2
  (`startup-probe2-master.json`), variant probe (`variant-probe.json`), A/B
  probe (`st-diag-ab-patched.json`), strace runs (`st-diag-strace-gridm-*`).
  Absolute times differ between sessions; deltas are quoted within a session
  only.

## 2. Standing end-gate context (orientation, not this run's measurement)

The standing binding bench (R4 session, this box class, report
`r4-ab-new.json` on the R4 workspace) is: aggregate 0.9709 (inside 1.05),
geomean 1.0671 (outside 1.05), worst per-run 1.3413 at grid_l/p (inside
1.35), oracle sum of medians 0.7512 s vs candidate 0.7293 s. Those numbers are
context for why the geomean is the binding axis; they are not carried into any
effect claim below. The end gate is absolute against the pinned oracle; the
1.05/1.35 bounds are not a band to chase.

## 3. The startup premium, quantified (section-5 method, this run)

No-args usage runs, 500 execs per round through an identical fork loop,
3 rounds, medians (probe 1 session):

- candidate 1.6021 ms/exec, oracle 0.9809 ms/exec, `/bin/true` 0.8904
  ms/exec. Candidate-oracle premium in that session: ~+0.62 ms/exec.

Probe 2 session (same method, argv variants plus the rustnull control,
medians):

| target | wall ms/exec | user ms | sys ms | minflt/exec |
|---|---|---|---|---|
| candidate, no args | 1.5221 | 0.9944 | 0.3204 | 120.3 |
| candidate, `-h` | 1.5153 | 0.9838 | 0.3232 | 121.4 |
| candidate, `-Q` missing file | 1.4340 | 0.9628 | 0.2748 | 122.3 |
| oracle, no args | 1.1307 | 0.7564 | 0.1546 | 70.1 |
| oracle, `-h` | 1.1530 | 0.7865 | 0.1685 | 70.1 |
| oracle, `-Q` missing file | 1.2352 | 0.8262 | 0.1676 | 79.4 |
| rustnull, no args | 1.3054 | 0.8879 | 0.2051 | 68.6 |
| `/bin/true` | 0.9646 | 0.6665 | 0.1008 | 50.8 |

Within-session decomposition (probe 2, deltas of the medians above):

- Rust runtime + std init over the fork/exec floor: rustnull minus
  `/bin/true` = +0.34 ms/exec.
- Candidate CLI main path over the runtime floor: candidate minus rustnull =
  +0.22 ms/exec.
- Oracle over the floor: +0.17 ms/exec.
- Candidate-oracle premium in this session: +0.39 ms/exec.

The premium's session-to-session spread (0.39 to 0.62 ms/exec across probes 1
and 2) is the instrument's within-box drift floor for this kind of
comparison: no effect smaller than roughly 0.1 ms/exec is attributable in a
single session, and nothing below ~0.03 ms/exec is visible at all.

The rusage split shows where the wall premium lives: candidate user 0.99 /
sys 0.32 ms vs oracle user 0.76 / sys 0.15 ms. The candidate touches ~120
minor faults per exec vs the oracle's ~70; rustnull sits at ~69, so ~50 of the
candidate's extra faults come from the CLI main path and heap, not from Rust
runtime init. `perf stat` over 200 no-args execs agrees: 24,974 page faults
for the candidate vs 14,940 for the oracle (~125 vs ~75 per exec), and the
no-args flat profile is dominated by fault machinery (`do_user_addr_fault`
5.17%, `filemap_map_pages` 1.37%, `clear_page_erms` 1.46%) with
`triangle_cli::main` at 1.74% and, per the event-attribution pass, ~88% of
page-fault events initiated beneath `triangle_cli::main`.

## 4. Attribution: where the premium lives

All this run, same session per comparison.

Dynamic loader (`LD_DEBUG=statistics`, one exec each): candidate total loader
startup 139,069 cycles (relocation 30,649 cycles over 164 + 790-relative
relocations) vs oracle 92,403 cycles (relocation 19,708 cycles over 93 +
3-relative). At a rough 2.5 GHz this is ~56 vs ~37 us, a ~+18 us gap
(cycle-to-time conversion is an estimate; the cycle counts are measured).
The loader is a real but small component.

Runtime init: the rustnull control shows ~+0.34 ms over `/bin/true` for Rust
runtime + std init alone, and the candidate pulls in an extra DSO the oracle
does not link (`libgcc_s.so.1`, visible in both `ldd` and the profile, where
`__cpu_indicator_init` in libgcc_s costs 1.23% of no-args samples).

Binary shape (`size`, `readelf`): candidate 826,888 B (text 706,955) vs
oracle 210,872 B (text 189,445); relocations 856 vs 36 (787 vs 3 relative,
0 vs 0 JUMP_SLOT); both PIE, both not stripped, 4 PT_LOAD each. The candidate
pays for a 3.9x larger text with ~4x the page faults of the oracle.

Syscall floor on a timed-path run (strace, grid_m/plain, this run, one exec
each): candidate at master 97 syscalls with 8 `write(1, ...)`; oracle 66 with
1. The candidate's fixed extras are Rust runtime setup (`rt_sigaction` 5,
`sigaltstack` 2, `gettid`/`poll`/`sched_getaffinity` 1 each, `mmap` +7,
`munmap` +2, `mprotect` +2) plus the port's line-buffered statistics block:
the oracle emits its whole statistics block in one `write(2)` (C stdio fully
buffers a non-tty stdout) while the port flushed one `write(2)` per
`println!`.

Summary: the premium decomposes into (i) Rust runtime/std init (~0.34 ms over
the floor, control-measured), (ii) candidate CLI main path beyond the runtime
floor (~0.22 ms, including ~50 extra minor faults and the fmt/allocator work
on the main path), and (iii) loader/relocation work (~+18 us, measured in
loader cycles). Only (ii) is addressable by the authorized CLI lever, and
only its constant fraction.

## 5. Authorized levers, exhaustively

Lever (a), the committed build recipe. The R4 session already measured
codegen-units, strip, and opt-level variants as null (R4 evidence,
orientation here, not re-measured this run). This run probed the remainder:

- `-static-libgcc` (RUSTFLAGS probe): binary sha256 identical to
  `fd532a99...`, libgcc_s still linked. Null.
- Non-PIE (`-C relocation-model=static`): relocations 856 to 65, size
  826,888 to 809,880 B. Exec cost in the variant probe session (medians, 500
  execs/round): v1 1.9049, v2 1.9382, v3 1.9766, v4 1.8299 ms/exec vs oracle
  1.3528 and `/bin/true` 1.1641 in the same session. The variants' spread
  (1.83 to 1.98) is the same order as the session's round-to-round spread;
  no variant is attributable. Null.

The recipe space is exhausted: every committed-recipe lever available without
changing target triple, toolchain, or panic semantics is measured null on
this box. The recipe stays untouched.

Lever (b), CLI fixed costs. The only removable per-exec constant on the main
path is the statistics block's write granularity: 8 `write(2)` per
non-verbose run vs the oracle's 1. The committed change formats the
pre-verbose statistics block into one `String` and prints it with a single
`print!` before the verbose gate. Measured effect (strace, grid_m/plain, this
run): stdout writes 8 to 1 (one 135-byte `write(2)`), total syscalls 97 to 90;
the oracle sits at 66. The verbose quality block keeps its per-line
`println!` verbatim so that the pre-existing `-V` panic inside it (the aspect
histogram reads index 14 of a 14-wide table, the registered D33 area) leaves
byte-identical partial stdout (827 bytes, exit 101, verified against the
master binary this run).

Exec-cost attribution of the lever (A/B probe session, master vs patched, no
args, 500 execs x 3 rounds interleaved): master median 1.9159 vs patched
2.0512 ms/exec with patched rounds spanning 1.7626 to 2.0566 in the same
session, i.e. the spread swamps the comparison, as expected: the no-args
usage path never prints statistics, so the lever cannot act there. On timed
corpus runs the lever removes 7 `write(2)` per exec; at the ~1-2 us/kernel
syscall order observed on this box that is ~10-20 us per run (estimate). That
is below the single-session attribution floor (~0.03 ms) and far below the
premium it targets.

## 6. Needed cut vs delivered cut

Against the standing per-run medians (orientation, section 2), a uniform
candidate-side cut of X ms per run moves the geomean as follows
(arithmetic on the standing 80 rows, not a measurement):

| uniform cut | geomean | worst per-run |
|---|---|---|
| 0.00 ms | 1.0671 | 1.3413 |
| 0.05 ms | 1.0583 | 1.3329 |
| 0.10 ms | 1.0495 | 1.3245 |
| 0.15 ms | 1.0406 | 1.3162 |
| 0.20 ms | 1.0318 | 1.3078 |

A uniform ~0.10 ms cut just reaches the 1.05 geomean bound; the measured
premium is ~0.4-0.6 ms/exec, most of it Rust runtime/std init (~0.34 ms over
floor) and main-path work, neither of which the authorized levers can touch:
the recipe levers are all null (section 5), and the one real CLI lever
delivers ~0.01-0.02 ms (estimate, section 5), roughly a tenth of the needed
cut at best.

Finding: the authorized lever space cannot move the startup floor far enough.
The per-exec premium is dominated by Rust process initialization, which is
inherent to the runtime and not addressable by the committed build recipe
or by constant-work removal on the CLI main path within the zero-behavior,
zero-unsafe, deterministic-recipe constraints. Reaching the geomean bound
would need a different lever class (for example the registered
bounds-check-relaxation ticket, or an instrument/protocol decision), which
is out of scope here and a maintainer decision, per the ticket.

## 7. What was changed

- `rust/crates/triangle-cli/src/main.rs`: `print_statistics` formats its
  pre-verbose block into one `String` (fmt::Write) and prints it with a
  single `print!` before the verbose gate; the verbose quality block is
  untouched. Output bytes are identical on every path (harness-gated, plus
  this run's direct checks: plain stdout identical to the oracle, `-V` panic
  path stdout and exit code identical to the master binary).
- This document.
- No build-recipe change, no Cargo.toml change, no new deviation: the change
  is port-level I/O granularity, not a behavioral departure from the C
  sources (C's stdio emits the same bytes in one buffered write; the port now
  does the same).
