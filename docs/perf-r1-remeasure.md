# R1 performance remeasure (post-campaign reference)

Per ADR 0001 (docs/adr/0001-port-strategy.md) the performance gate
(wayfinder #11) runs at stage-1 completion (R0) and at every
idiomatization rung boundary before a rung lands. This document
records the R1 remeasure of the post-campaign candidate: the
release build of master 104172dc, which contains all four merged
campaign fixes (PR #50 buffers the CLI output files, PR #51 moves
the file writers to reusable line buffers, PR #52 sets fat LTO for
the release profile, PR #53 reworks the mesh-load orchestration),
measured on a fresh instance of the machine class of record
(Hetzner CPX42 class, the class the R0 baseline was recorded on).
The PR that carries this document is docs-only: the candidate
measured is the release build of master 104172dc exactly, no code
changes. From merge on, the table below is the no-regression
reference for every idiomatization rung boundary; it supersedes
the R0 numbers in docs/perf-baseline-r0.md as the reference (that
document stays unchanged as the R0 record of its own measurement).

## Measurement

- Date: 2026-11-27
- Machine: managed build-server vServer, freshly provisioned for
  this measurement: 8 vCPUs, AMD EPYC-Genoa (Hetzner CPX42 class;
  1 socket, 8 cores, 1 thread per core, SMT off, one NUMA node;
  MemTotal 15982068 kB, about 15.2 GiB), Debian 12.15,
  kernel 6.1.0-56-amd64, gcc (Debian 12.2.0-14+deb12u1) 12.2.0 (bench.py machine string:
  Linux-6.1.0-56-amd64-x86_64-with-glibc2.36, 8 CPUs). Same machine class as the R0 instance of the
  baseline (same kernel, same gcc, same bench.py machine string),
  but a different fresh instance, and the instance difference is
  recorded, not assumed away: the identical pinned oracle over the
  identical m+l corpus sums to 0.3241 s here versus 0.3784 s in R0
  (this instance runs the identical binary and corpus about
  14% faster), and the noise floor re-derived below
  is tighter than on the R0 instance (+/-0.1% versus +/-0.5%
  aggregate wander). The machine instance changed, so the noise
  floor was re-derived FIRST, before any candidate timing.
- Toolchain: rustc 1.100.0 (0ef1bc386 2026-11-09), cargo 1.100.0 (3acc79c0c 2026-11-05); C compiler
  gcc (Debian 12.2.0-14+deb12u1) 12.2.0. Same stable pin as the R0 measurement.
- Pinned oracle: tools/oracle/build-oracle.sh, binary sha256
  9af462136c77f812f44304ed2befe3bdf958f0d95eea879793db5e2678bf34e4 (recorded in work/oracle/build-info.txt).
  Cross-instance reproduction measured, not assumed: the fresh
  build on this box reproduces the oracle sha256 recorded in R0
  exactly.
- Candidate provenance: the release build of master 104172dc (the
  merge of PR #53) exactly, no code changes: cd rust && cargo
  build --release --locked -p triangle-cli (release profile,
  default features, against rust/Cargo.lock; the release profile
  uses lto = "fat" per PR #52, so the build links slower than the
  pre-campaign profile, expected). Candidate binary sha256
  580686a2dec20de77604577f2291c160b7c9a95e1684c8799bf3e47f1e53311c.
- Harness first: the differential harness passed 120/120
  byte-identical before any timing was recorded (see Corpus
  status below).
- Command: tools/oracle/run-bench.sh --candidate
  rust/target/release/triangle-cli --report
  tools/oracle/work/reports/r1-remeasure.json (timed set sizes
  m+l of the harness corpus = 80 runs; size s stays corpus-only,
  startup-dominated; 3 warmup + 25 timed per side, strictly
  interleaved, median per side; every number below comes from
  the bench.py report JSON).

## Noise floor (re-derived first: the machine instance changed)

Per the rule of the baseline document the noise floor was
re-derived before any candidate timing, because the machine
instance changed. Candidate = the pinned oracle binary itself,
same command, three probes (report JSONs noise-probe-1/2/3.json):

| probe | total ratio | geomean | worst per-run ratio |
| --- | --- | --- | --- |
| 1 | 1.001 | 1.000 | 1.053 (grid_m/plain) |
| 2 | 1.000 | 0.999 | 1.025 (pslg_hole_m/pen) |
| 3 | 0.999 | 0.999 | 1.032 (grid_m/p) |

- Aggregate wander within about +/-0.1% of 1.000 across
  probes (the R0 instance measured +/-0.5%; the #11 cx23 floor
  was +/-1.6%; the #11 CPX42 re-probe about +/-2.5%).
- Worst identical-binary per-run ratio across the three probes
  (240 per-run ratios): 1.053 (grid_m/plain), below the 1.35 per-run
  tripwire.
- Per-exec bimodality, the behavior documented for this machine
  class in ticket #11 (two discrete per-exec wall-time modes
  about 2.2x apart, per-run CV about 28-30%, aggregate wander
  about +/-2.5%, per-run tripwire overruns on identical
  binaries): NOT present on this box. Per-run CV averages
  5.6% across the probes (worst single-run CV 16.6%),
  with no two-mode pattern. This instance behaves like the
  R0-closeout instance (+/-0.5% aggregate wander, worst
  identical-binary ratio 1.062), only tighter. The
  disagreement with the #11 CPX42 re-probe is recorded here;
  the measurement on this box wins for this box.
- The band stays 1.05 / 1.05 / 1.35 either way: it clears this
  floor with margin, and it also cleared the aggregate floor of
  the bimodal #11 box. Unchanged and binding.

## Aggregate (side by side with the R0 baseline of record)

| statistic | R0 baseline of record | R1 remeasure |
| --- | --- | --- |
| oracle total (sum of per-run medians) | 0.3784 s | 0.3241 s |
| candidate total (sum of per-run medians) | 1.7322 s | 0.3309 s |
| sum-of-medians total ratio (candidate/oracle) | 4.578 | 1.021 |
| geometric mean of per-run ratios | 3.857 | 1.119 |
| worst per-run ratio | 16.098 (grid_l/pen) | 1.409 (grid_m/plain) |

The campaign moved the m+l aggregate total ratio from 4.578
to 1.021 (candidate and oracle are at aggregate parity now;
the size-l subset measures 0.985, the size-m subset 1.095) and the
worst per-run ratio from 16.098 to 1.409. Absolute times are
instance-dependent: two fresh instances of the same machine class
provisioned hours apart differ by about 14% on the identical
pinned oracle and corpus (0.3784 s versus 0.3241 s sum of medians),
so the side-by-side comparison reads the ratios; the R0 absolute
times are what the R0 instance measured.

## Gate numbers (1.05 / 1.05 / 1.35, ticket #11): FAIL, stated plainly

- sum-of-medians total ratio 1.021 <= 1.05: PASS
- geometric mean 1.119 > 1.05: FAIL
- worst per-run ratio 1.409 (grid_m/plain) > 1.35: FAIL

Two of the three gate numbers are exceeded, so the oracle-parity
verdict is FAIL, recorded as measured from this single run of the
identical command (the reference record, run once like R0; no
rerun was taken). The excess is concentrated in the
startup-dominated smallest runs: the size-m geomean is 1.159
versus 1.081 at size l, 50 of the 80 per-run ratios sit above 1.05
and 5 above 1.35, all 5 of them sub-2.5 ms plain/p runs; the
candidate is faster than the oracle on 18 runs, including the
heaviest quality runs (grid_l/pa 0.841, grid_l/pq20a 0.829). This PR
records the measurement and blocks nothing by itself; the binding
comparison for future rung boundaries runs against the table below,
not against oracle parity.

## Per-run ratio table (80 timed runs, median per side)

| case | family | size | oracle median (s) | candidate median (s) | ratio |
| --- | --- | --- | --- | --- | --- |
| grid_m | plain | m | 0.0013 | 0.0019 | 1.409 |
| grid_m | q20 | m | 0.0016 | 0.0021 | 1.303 |
| grid_m | p | m | 0.0014 | 0.0019 | 1.354 |
| grid_m | pq20 | m | 0.0015 | 0.0020 | 1.330 |
| grid_m | pq20U120 | m | 0.0014 | 0.0019 | 1.339 |
| grid_m | pa | m | 0.0038 | 0.0040 | 1.048 |
| grid_m | pq20a | m | 0.0040 | 0.0041 | 1.041 |
| grid_m | pen | m | 0.0017 | 0.0022 | 1.295 |
| grid_l | plain | l | 0.0023 | 0.0031 | 1.362 |
| grid_l | q20 | l | 0.0027 | 0.0034 | 1.271 |
| grid_l | p | l | 0.0024 | 0.0033 | 1.342 |
| grid_l | pq20 | l | 0.0025 | 0.0033 | 1.324 |
| grid_l | pq20U120 | l | 0.0025 | 0.0033 | 1.292 |
| grid_l | pa | l | 0.0339 | 0.0286 | 0.841 |
| grid_l | pq20a | l | 0.0340 | 0.0282 | 0.829 |
| grid_l | pen | l | 0.0033 | 0.0039 | 1.195 |
| random_m | plain | m | 0.0013 | 0.0017 | 1.282 |
| random_m | q20 | m | 0.0030 | 0.0032 | 1.055 |
| random_m | p | m | 0.0014 | 0.0018 | 1.260 |
| random_m | pq20 | m | 0.0030 | 0.0032 | 1.045 |
| random_m | pq20U120 | m | 0.0042 | 0.0044 | 1.043 |
| random_m | pa | m | 0.0047 | 0.0047 | 1.000 |
| random_m | pq20a | m | 0.0047 | 0.0047 | 0.991 |
| random_m | pen | m | 0.0017 | 0.0020 | 1.186 |
| random_l | plain | l | 0.0028 | 0.0031 | 1.093 |
| random_l | q20 | l | 0.0086 | 0.0080 | 0.937 |
| random_l | p | l | 0.0029 | 0.0032 | 1.109 |
| random_l | pq20 | l | 0.0087 | 0.0081 | 0.929 |
| random_l | pq20U120 | l | 0.0124 | 0.0120 | 0.965 |
| random_l | pa | l | 0.0035 | 0.0036 | 1.045 |
| random_l | pq20a | l | 0.0050 | 0.0049 | 0.985 |
| random_l | pen | l | 0.0038 | 0.0038 | 1.017 |
| pslg_hole_m | plain | m | 0.0011 | 0.0015 | 1.377 |
| pslg_hole_m | q20 | m | 0.0023 | 0.0025 | 1.085 |
| pslg_hole_m | p | m | 0.0012 | 0.0016 | 1.336 |
| pslg_hole_m | pq20 | m | 0.0024 | 0.0027 | 1.104 |
| pslg_hole_m | pq20U120 | m | 0.0031 | 0.0033 | 1.080 |
| pslg_hole_m | pa | m | 0.0048 | 0.0048 | 0.985 |
| pslg_hole_m | pq20a | m | 0.0048 | 0.0048 | 1.011 |
| pslg_hole_m | pen | m | 0.0014 | 0.0018 | 1.260 |
| pslg_hole_l | plain | l | 0.0019 | 0.0023 | 1.205 |
| pslg_hole_l | q20 | l | 0.0053 | 0.0053 | 0.989 |
| pslg_hole_l | p | l | 0.0020 | 0.0024 | 1.203 |
| pslg_hole_l | pq20 | l | 0.0056 | 0.0055 | 0.987 |
| pslg_hole_l | pq20U120 | l | 0.0082 | 0.0082 | 0.997 |
| pslg_hole_l | pa | l | 0.0024 | 0.0027 | 1.140 |
| pslg_hole_l | pq20a | l | 0.0032 | 0.0034 | 1.066 |
| pslg_hole_l | pen | l | 0.0025 | 0.0027 | 1.109 |
| pslg_concave_m | plain | m | 0.0012 | 0.0016 | 1.340 |
| pslg_concave_m | q20 | m | 0.0022 | 0.0024 | 1.114 |
| pslg_concave_m | p | m | 0.0013 | 0.0017 | 1.313 |
| pslg_concave_m | pq20 | m | 0.0023 | 0.0025 | 1.120 |
| pslg_concave_m | pq20U120 | m | 0.0029 | 0.0032 | 1.106 |
| pslg_concave_m | pa | m | 0.0065 | 0.0062 | 0.959 |
| pslg_concave_m | pq20a | m | 0.0062 | 0.0060 | 0.965 |
| pslg_concave_m | pen | m | 0.0014 | 0.0017 | 1.279 |
| pslg_concave_l | plain | l | 0.0019 | 0.0022 | 1.167 |
| pslg_concave_l | q20 | l | 0.0044 | 0.0044 | 0.999 |
| pslg_concave_l | p | l | 0.0019 | 0.0023 | 1.192 |
| pslg_concave_l | pq20 | l | 0.0045 | 0.0046 | 1.007 |
| pslg_concave_l | pq20U120 | l | 0.0065 | 0.0065 | 1.007 |
| pslg_concave_l | pa | l | 0.0023 | 0.0026 | 1.128 |
| pslg_concave_l | pq20a | l | 0.0030 | 0.0031 | 1.040 |
| pslg_concave_l | pen | l | 0.0024 | 0.0027 | 1.113 |
| pslg_wall_m | plain | m | 0.0011 | 0.0015 | 1.358 |
| pslg_wall_m | q20 | m | 0.0022 | 0.0024 | 1.096 |
| pslg_wall_m | p | m | 0.0012 | 0.0016 | 1.350 |
| pslg_wall_m | pq20 | m | 0.0023 | 0.0026 | 1.108 |
| pslg_wall_m | pq20U120 | m | 0.0030 | 0.0033 | 1.092 |
| pslg_wall_m | pa | m | 0.0048 | 0.0047 | 0.989 |
| pslg_wall_m | pq20a | m | 0.0046 | 0.0046 | 0.995 |
| pslg_wall_m | pen | m | 0.0014 | 0.0018 | 1.282 |
| pslg_wall_l | plain | l | 0.0018 | 0.0022 | 1.190 |
| pslg_wall_l | q20 | l | 0.0048 | 0.0048 | 0.989 |
| pslg_wall_l | p | l | 0.0020 | 0.0024 | 1.205 |
| pslg_wall_l | pq20 | l | 0.0049 | 0.0048 | 0.988 |
| pslg_wall_l | pq20U120 | l | 0.0071 | 0.0071 | 1.006 |
| pslg_wall_l | pa | l | 0.0023 | 0.0026 | 1.111 |
| pslg_wall_l | pq20a | l | 0.0030 | 0.0032 | 1.059 |
| pslg_wall_l | pen | l | 0.0024 | 0.0027 | 1.107 |

## Pen family before/after, and where the new worst per-run sits

The pen family was the R0 outlier family (the per-fragment .neigh
writer dominated its band; see docs/perf-r1-findings.md).
Before/after, per-run median ratios:

| case | R0 ratio | R1 ratio |
| --- | --- | --- |
| grid_m/pen | 10.183 | 1.295 |
| grid_l/pen | 16.098 | 1.195 |
| random_m/pen | 8.795 | 1.186 |
| random_l/pen | 15.393 | 1.017 |
| pslg_hole_m/pen | 5.723 | 1.260 |
| pslg_hole_l/pen | 12.476 | 1.109 |
| pslg_concave_m/pen | 6.452 | 1.279 |
| pslg_concave_l/pen | 11.857 | 1.113 |
| pslg_wall_m/pen | 6.686 | 1.282 |
| pslg_wall_l/pen | 12.419 | 1.107 |

The pen family collapsed from 5.723-16.098 (R0) to 1.017-1.295 (R1) and
is no longer the outlier family. The new worst per-run sits in the
plain family, on the smallest startup-dominated runs: grid_m/plain at 1.409
(oracle median 0.0013 s, candidate 0.0019 s). The 5 runs above the
1.35 tripwire are all tiny plain/p runs (grid_m/plain 1.409, pslg_hole_m/plain 1.377, grid_l/plain 1.362, pslg_wall_m/plain 1.358, grid_m/p 1.354); every
pen run now measures 1.017-1.295.

## Ruling

From the merge of this PR on, the R1 table above is the
no-regression reference for every idiomatization rung boundary.
It supersedes the R0 numbers in docs/perf-baseline-r0.md as the
reference; that document stays unchanged as the R0 record. Each
rung is measured with this same command on the same class of
machine, and a rung may not regress this table beyond the
comparison tolerance: 2x the aggregate wander measured by the
noise-floor probe on the then-current box, re-derived on that
box before the comparison per the rules of the baseline document
(this box: 2 x about +/-0.1%, about +/-0.2% around the
aggregate total ratio). The 1.05 / 1.05 / 1.35 oracle-parity band
of ticket #11 stays binding as the port-level target.

## Corpus status

Differential harness, candidate = the same built release CLI, run
at master 104172dc before any timing (tools/oracle/run-harness.sh
--candidate rust/target/release/triangle-cli): all 120 runs
byte-identical (harness report work/reports/last-run.json: 120
entries, zero mismatches), against the pinned oracle binary
sha256 9af462136c77f812f44304ed2befe3bdf958f0d95eea879793db5e2678bf34e4. The serial byte-identity gate is green at
the head of this PR and stays binding (also the permanent CI
gate, .github/workflows/oracle-harness.yml).

## Raw artifacts

The raw measurements stay on the build server (workspace
triangle-cpx42, git-ignored paths in the repo worktree):
tools/oracle/work/reports/noise-probe-1.json, noise-probe-2.json,
noise-probe-3.json, r1-remeasure.json, last-run.json, plus
tools/oracle/work/oracle/build-info.txt (oracle build record and
binary sha256). Every number in this document comes from those
report JSONs or from a command run on this box; the R0-side
comparison numbers are parsed from docs/perf-baseline-r0.md, the
record of its own measurement.
