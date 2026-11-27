# Performance R1 findings: where the R0 baseline's outliers spend their time

Diagnosis-only follow-up to `docs/perf-baseline-r0.md`. Branch `perf-r1-findings`,
branched from master `180392dd`.

Scope: this investigation changed no production code. The only repository change
is this document. Every number below was measured on the build server for this
run; no figure is carried over from the R0 baseline doc or from the tasking
brief. Expectations about future fixes are explicitly marked as estimates.

## 1. Environment and provenance

- Machine: fresh Hetzner vServer, 8 vCPU AMD EPYC-Genoa, 16 GB RAM, Debian 12
  (kernel 6.1.0-56-amd64, glibc 2.36), provisioned for this run. All timed and
  profiled runs pinned to one core (`taskset -c 2`).
- Toolchain: rustc 1.100.0 (0ef1bc386 2026-11-09).
- Candidate build: `cargo build --release --locked -p triangle-cli`, default
  features, exactly as R0 built it.
- Candidate binary sha256:
  `5b755ada46f68295760b2f3b1326fe4b1ff5cd7aa41e339db011a82f4072c8e0`.
  A copy was saved as `tools/oracle/work/profile-r1/triangle-cli.r0-exact`; all
  profiles and strace runs below used that exact binary.
- Oracle: `tools/oracle/build-oracle.sh`, sha256
  `9af462136c77f812f44304ed2befe3bdf958f0d95eea879793db5e2678bf34e4`
  (same pin as R0).
- Corpus: regenerated with the committed `tools/oracle/generate_corpus.py`
  (seeded; inputs byte-identical to the baseline corpus by construction).
- Profilers: `perf` 6.1.192 (installed via apt on the server;
  `perf_event_paranoid=0`) and `strace` 6.1. Single runs are milliseconds, so
  each profiled case ran in a repetition loop (100-500 iterations):
  flat sampling at `-F 2499`, caller-chain sampling at `-F 499 --call-graph
  dwarf`. Flat profiles were taken on the exact release binary (kernel symbols
  plus `.eh_frame` unwinding); inline-aware caller chains additionally on a
  `-g` debug-info rebuild of the same commit, used for attribution only (its
  zone ranking matches the exact binary's). `valgrind --tool=callgrind` was not
  needed: perf was available and preserves kernel time, which turned out to be
  where the cost lives.
- Attribution caveat: `addr2line`-style inline expansion proved unreliable for
  this binary (it misattributed instruction addresses to the wrong compilation
  units in spot checks) and was discarded as evidence. Every file:line below
  comes from reading the source at the functions that perf's DWARF caller
  chains name; every percentage comes from `perf report`/`perf script` output
  or strace counts.

## 2. R0 re-measured (this machine, this run)

`bench.py`, 25 repeats, sizes m+l, interleaved, medians:

- oracle sum-of-medians 0.3789 s, candidate 1.7238 s, total ratio 4.550,
  geomean 3.851, worst 16.51 (grid_l/pen).
- Per-run medians, oracle / candidate (ms), with ratios:
  grid_l/pen 4.044 / 66.756 (16.51x); random_l/pen 4.873 / 74.381 (15.27x);
  pslg_hole_l/pen 3.023 / 37.523 (12.41x); grid_m/pen 2.368 / 21.228 (8.96x);
  grid_l/plain 2.943 / 14.437 (4.91x); random_l/pq20U120 13.824 / 29.928
  (2.16x); grid_m/plain 1.914 / 5.680 (2.97x); grid_s/plain 1.104 / 2.570
  (2.33x; separate `--sizes s --cases grid_s --families plain` run).

This reproduces the R0 symptom split: the pen family at 8.96-16.51x, every
non-pen family in a broad 1.95x (pslg_wall_m/pq20U120) to 6.70x
(random_l/pq20a) band.

## 3. Q1, pen family: the time is write(2) syscalls, not neighbor computation

### 3.1 Syscall counts (strace, one grid_l/pen run, exact binary)

| output file | candidate writes | candidate bytes | oracle writes | oracle bytes |
|---|---:|---:|---:|---:|
| input.1.node | 1,212 | 23,332 | 6 | 23,322 |
| input.1.ele | 2,222 | 55,329 | 14 | 55,319 |
| input.1.poly | 212 | 5,152 | 2 | 5,142 |
| input.1.edge | 3,416 | 65,559 | 17 | 65,549 |
| input.1.neigh | 17,688 | 51,997 | 14 | 51,987 |
| total | 24,750 | 201,369 | 53 | ~201,319 |

Byte columns are strace's per-call return values summed; the output files
themselves are byte-identical (the harness gate re-verifies 120/120). The
oracle writes 4096-byte stdio buffers. The candidate's writes are per
fragment: 7,951 four-byte, 4,870 two-byte, 2,637 three-byte, 2,273 one-byte
calls plus line-sized ones. 24,750 vs 53 write(2) calls for the same five
files is a 467x syscall-count difference, and the .neigh writer alone carries
17,688 of them (71.5%).

### 3.2 Why: per-fragment `write!` onto an unbuffered File

- Every output file is a raw `std::fs::File`; there is no `BufWriter` anywhere
  on the output path (`triangle-cli/src/main.rs:561-613` creates the files,
  `rust/crates/triangle/src/api.rs:461-496` passes them to the io writers).
- `file_writeneighbors` (`rust/crates/triangle/src/io.rs:363-422`) writes each
  line with one `write!(neighborfile, "{}    {}  {}  {}\n", ...)` at
  `io.rs:407-413`. A `write!` on a File goes through `core::fmt`, which calls
  the adapter's `write_str` once per format piece; each piece becomes one
  write(2). One .neigh line has 8 pieces (triangle number, 3 separators, 3
  neighbor numbers, newline); the trace shows exactly 8 syscalls per line, and
  2,208 element lines plus header pieces give the measured 17,688.
- `file_writeedges` (`io.rs:279-361`) and `file_writeelements`
  (`io.rs:119-185`) build each line with `format!` into a String and then
  `write_all` it once (io.rs:348; io.rs:176): one syscall per line (3,416 and
  2,222 measured) at the cost of one String per line and per number.
- `file_writenodes` does the same (`format!` at io.rs:85, `write_all` at
  io.rs:106): 1,212 syscalls.

### 3.3 Where the samples go (perf, exact binary)

Caller chains (DWARF/`.eh_frame`), grid_l/pen, 6,842 samples, 83.4% of them
kernel-leaf (inside write(2)):

| writer subtree | share of ALL samples | samples |
|---|---:|---:|
| file_writeneighbors | 44.3% | 3,029 |
| file_writeedges | 10.5% | 717 |
| file_writeelements | 7.3% | 499 |
| file_writenodes | 4.9% | 337 |
| file_writepoly | 0.4% | 25 |

Inside the `file_writeneighbors` subtree (3,029 samples):

| zone | share | samples |
|---|---:|---:|
| kernel write path | 82.9% | 2,511 |
| libc internals under the write path (memcpy/malloc) | 13.8% | 418 |
| fmt machinery | 1.3% | 40 |
| std File write path | 1.3% | 38 |
| String/alloc | 0.4% | 13 |
| writer's own code | 0.2% | 7 |
| mesh accessors (sym / tri_int) | 0.1% | 2 |

Flat profile, grid_l/pen, 52,378 samples: kernel 80.42% (42,124), std File
write path 10.38% (5,436), per-number fmt allocations 2.67% (1,400),
unresolved-hex 1.77% (925), core::fmt 1.25% (654), mesh cores 1.14% (598),
predicates 0.64% (336). The other pen cases show the same shape:
random_l/pen kernel 80.34%, File write path 10.85%, mesh cores 1.12%;
pslg_hole_l/pen kernel 81.09%, File write path 9.63%, mesh cores 1.13%.

For contrast, the oracle's grid_l/pen profile spends its samples in meshing
(vertexsort 6.79%) with the writers as noise (file_writeedges 0.38% = 75
samples, file_writeneighbors 0.07% = 13) and stdio (_IO_file_xsputn) at 1.99%.

### 3.4 Per-number formatter allocations

Every integer the writers print goes through the formatter helpers `d()` /
`ld()` / `ld4()` (`rust/crates/triangle/src/formatter.rs:148-163`), each
returning a fresh `String` via `to_string()` (SpecToString), and `ld4`/`d4`
additionally copy through `pad_left` (formatter.rs:172-183). In grid_l/pen the
"per-number fmt allocations" zone carries 2.67% of all samples (1,400) plus
core::fmt machinery 1.25% (654). In the inline-aware user-sample set (1,220
user samples, grid_l/pen) the alloc/String zone is 15.8% and fmt machinery
14.5% of user time. So formatting is the second cost of the pen path, an
order of magnitude behind the syscalls.

### 3.5 Neighbor computation outside writing

Essentially zero. `file_writeneighbors`'s two mesh walks (the numbering
overlay stores at io.rs:389-396, the three `tri_int` reads per line at
io.rs:400-403) plus every mesh accessor inside the subtree carry 9 of 3,029
subtree samples (0.3%); the flat-profile mesh-cores zone for grid_l/pen is
1.14% of all samples. The pen family's +50-62 ms over its plain twin is not
neighbor computation. It is the ~23,700 extra write(2) calls and their
kernel time.

## 4. Q2, non-pen families: I/O first, formatting second, accessors nowhere

### 4.1 Flat zone split (exact binary, sample counts)

| zone | grid_l/plain (22,365) | random_l/pq20U120 (31,735) | grid_m/plain (22,129) |
|---|---:|---:|---:|
| kernel (write path + per-exec) | 67.35% (15,063) | 48.51% (15,394) | 72.10% (15,954) |
| mesh cores (construct/delaunay/quality/refine/acute/locate) | 4.60% (1,029) | 27.02% (8,576) | 3.96% (877) |
| per-number fmt allocations | 6.74% (1,507) | 5.19% (1,646) | 5.36% (1,187) |
| unresolved-hex (libc internals) | 5.94% (1,328) | 7.76% (2,463) | 5.66% (1,253) |
| std File write path | 5.89% (1,317) | 5.17% (1,640) | 4.43% (981) |
| predicates | 2.99% (668) | 1.33% (421) | 2.62% (579) |
| flt2dec (float formatting) | 1.57% (352) | 0.68% (216) | 1.35% (298) |
| core::fmt machinery | 1.47% (329) | 1.11% (352) | 1.24% (275) |
| slots/pool/handles/mesh primitives | 0.19% (43) | 0.32% (103) | 0.25% (55) |

Even without .neigh/.edge output, a plain grid_m run still issues 1,048
write(2) calls (378 .node + 670 .ele, one per line, measured) where the oracle
issues 7.

### 4.2 Where the user time goes (inline-aware, -g build)

random_l/pq20U120, 1,657 user samples: acute (smoothing) 43.6% (722), with the
leaf chain `findNewSPLocation` -> `splittriangle` (refine) ->
`enforcequality` -> `Mesh::create`, and leaves lineLineSegmentIntersection
212, splitConvexPolygon 168, halfPlaneIntersection 113, testTriangleAngle 101,
getWedgeIntersectionWithMaxAngle 95; file-write 11.6% (192); alloc 10.4%
(173); fmt 3.3% (55); predicates 1.8% (29); delaunay 1.1% (19); pool row_index
1.0% (16); quality 0.8% (13); slots 0.8% (14); slice::index 0.36% (6).

grid_l/plain, 797 user samples: predicates 21.3% (170), alloc 15.4% (123),
file-write 11.4% (91), fmt 4.1% (33), delaunay 3.3% (26), formatter 2.6%
(21), pool row_index 1.6% (13), slice::index 1.3% (10), slots 0.4% (3).

### 4.3 Bounds checks and handle encode/decode

- `core::slice::index` (the panic-checked indexing path, e.g. the
  `self.words[self.row_index(id, slot)]` at `rust/crates/triangle/src/pool.rs:221-231`)
  carries 0.33% of user samples in grid_l/pen (4 of 1,220), 0.36% in
  random_l/pq20U120 (6 of 1,657), 1.3% in grid_l/plain (10 of 797), 1.4% in
  grid_m/plain (4 of 281).
- `WordPool::row_index` (pool.rs:233-236) carries 1.0-1.6% of user samples;
  the slots.rs accessors (`tri_int` slots.rs:203, `set_tri_int` slots.rs:208,
  `vx_real` slots.rs:259, `vx_mark` slots.rs:294, `sym` slots.rs:350, `org`
  slots.rs:439, `dest` slots.rs:444) are mostly inlined into their callers and
  appear as the 0.19-0.32% slots/pool/handles zone plus small shares inside
  mesh cores.
- Even attributing the whole slots+pool+handles zone to guard overhead, the
  decode/bounds cost stays under ~2% of user time. Honest caveat: cmp/ja
guard instructions inlined into hot functions cannot be separated from the
surrounding code by sampling, but that ambiguity cannot hide a 2-6x gap: the
accessor zone is two orders of magnitude too small.

Q2 conclusion: the non-pen band is (a) per-line write syscalls (kernel zone
48-72% of samples across the profiled controls), (b) per-number String
formatting plus flt2dec float formatting (roughly 10% of user samples), and
(c) genuine meshing work, which dominates the quality cases (acute smoothing
43.6% of user samples in random_l/pq20U120; predicates 21.3% in grid_l/plain).
Slots/pool/handles accessors and bounds checks are not measurable
contributors.

## 5. Q3, fixed overhead vs meshing at m size

Candidate grid_m/plain runs 5.680 ms vs oracle 1.914 ms (medians above), a
3.77 ms gap. Measured pieces:

- Process startup (no-args usage run, 500 execs, identical fork loop):
  candidate 2.117 ms/exec, oracle 1.520 ms/exec, `/bin/true` control 1.293
  ms/exec. Above the fork/exec floor: candidate ~0.82 ms, oracle ~0.23 ms, a
  candidate startup premium of ~+0.6 ms per run.
- Output writing: grid_m/plain issues 1,048 write(2) per run vs the oracle's 7.
  strace -c puts the candidate's write(2) time at 0.1829 s over 50 runs =
  3.66 ms/run (an upper bound; ptrace inflates syscall time) and the oracle's
  at 0.001127 s / 50 = 0.023 ms/run. Sample attribution agrees: 72.10% of
  grid_m/plain samples are kernel, the `file_writeelements` subtree is 92.7%
  of all samples, and 74.3% of that subtree is the kernel write path.
- Meshing: candidate mesh cores 3.96% + predicates 2.62% = ~6.6% of samples
  (~0.4 ms at the 5.68 ms median). The oracle's grid_m/plain profile is
  dominated by exactly these kernels (tri_scale_expansion_zeroelim 5.61%,
  vertexmedian 4.36%, tri_incircle 2.55%, mergehulls 2.33%) with stdio
  (_IO_file_xsputn) at 1.37%: its ~1.9 ms is startup plus meshing, and its
  I/O is nearly free.

Bound: startup premium (~0.6 ms) plus the write path (3.0-3.7 ms across the
strace and sample-share estimates, minus the oracle's ~0.02 ms) accounts for
~3.5-4.3 ms of the 3.77 ms gap. Within measurement error, the entire m-size
gap is fixed per-run overhead (startup plus unbuffered output); the meshing
delta is at most a few tenths of a millisecond. The grid_s/plain control
corroborates: candidate 2.570 ms vs oracle 1.104 ms with 83.33% of the
candidate's samples in the kernel; at s size the run is almost nothing but
fixed overhead.

## 6. Ranked causes, with measured contributions

1. Unbuffered output onto raw `std::fs::File`: 24,750 write(2) per pen run vs
   the oracle's 53 (467x); 1,048 vs 7 in grid_m/plain. Kernel time is 80.42% of
   grid_l/pen samples, 67-72% in the non-pen controls. Dominant cause of both
   symptom families.
2. The .neigh writer's per-fragment `write!` (io.rs:407-413): 8 syscalls per
   line, 17,688 syscalls = 71.5% of a pen run's writes; its subtree is 44.3%
   of all grid_l/pen samples. Dominant cause of the pen family specifically.
3. Per-line String building plus per-number `to_string` (io.rs:85/106,
   io.rs:144/159/176, io.rs:318/340/348; formatter.rs:148-183): fmt-allocation
   zone 2.5-6.7% of all samples; alloc plus fmt machinery 16-30% of user
   samples. Second-order cost, an order of magnitude behind the syscalls.
4. Startup premium ~+0.6 ms per run (2.117 vs 1.520 ms/exec over a 1.293 ms
   fork floor). Matters only at s/m sizes.
5. Bounds checks and handle encode/decode: <=1.4% of user samples
   (slice::index plus the whole slots/pool/handles zone). Not a cause of the
   2-6x band.
6. Meshing itself is not a differentiator: the oracle spends its time in the
   same kernels (vertexsort 6.79% of its grid_l/pen profile; the predicates
   family ~10% of its grid_m/plain profile). This is why the quality-heavy
   families sit lower (random_l/pq20U120 at 2.16x) than the output-heavy ones.

## 7. Recommended fix order

1. Wrap every output File in `BufWriter` (triangle-cli/src/main.rs:561-613).
   Safe, local, no model change. Expected effect (estimate): the pen family
   loses most of its 80% kernel share, grid_l/pen from ~67 ms toward the
   oracle's per-line cost profile (order 10-15 ms); grid_m/plain from 5.68 ms
   toward ~2.0-2.3 ms; grid_s/plain toward ~1.5 ms. Risk to byte parity: none
   by construction - identical bytes in identical order, flush before drop;
   the 120-run harness gate verifies it.
2. Make `file_writeneighbors` build one line per write like the .edge writer
   (io.rs:407-413). Expected effect: large for .neigh-heavy cases on its own;
   minor once fix 1 is in (the buffer absorbs the fragments), but it removes
   the per-fragment fmt dispatch. Risk to byte parity: none - same bytes.
3. Reuse a line buffer and format numbers into it, dropping the per-number
   String allocations (formatter.rs d()/ld()/ld4() call sites in the writers).
   Expected effect (estimate): single-digit percent of wall time; alloc plus
   fmt is 16-30% of user samples, but user time is small once I/O is fixed.
   Risk to byte parity: none - same digits, same field widths.
4. Startup: leave alone for now; the ~0.6 ms premium is Rust runtime and std
   init, a model change for at most ~0.3 ms of gap at m size.
5. The registered unsafe relaxation (bounds-check removal): last, and on
   this evidence not worth its registration cost: the measured ceiling is
   ~1-2% of user time, and user time itself shrinks once I/O is fixed. Risk to
   byte parity: outputs do not depend on the checks (they panic or pass), so
   parity risk is low; the memory-safety risk is exactly why it is registered
   and stays last.

Steps 1-3 change no model and touch no registered deviation; all preserve
byte parity by construction.

## 8. What was changed

No production code was changed: `rust/crates/**`, `src/**` and
`tools/oracle/**` are untouched at the branch head, and the release binary
built from the branch is byte-identical (sha256 above) to the R0 baseline
build. No deviation-register rows were added. The only repository change is
this document, `docs/perf-r1-findings.md`.

All raw evidence was kept on the build server (git-ignored) under
`tools/oracle/work/profile-r1/` (perf.data files for seven candidate cases
and two oracle cases, perf report and perf script caller-chain outputs,
strace raw traces and summaries, the saved exact binary, and the analysis
scripts) and `tools/oracle/work/reports/` (bench re-measurement reports
r1-remeasure-full.json, r1-grid_s.json).
