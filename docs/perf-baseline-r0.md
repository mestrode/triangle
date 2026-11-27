# R0 performance baseline

Per ADR 0001 (docs/adr/0001-port-strategy.md) the performance gate
(wayfinder #11) runs at stage-1 completion (R0) and at every
idiomatization rung boundary before a rung lands. This document
records the R0 baseline and fixes it as the no-regression
reference for every idiomatization rung boundary from now on:
a rung may not regress these numbers.

## Measurement

- Date: 2026-11-27
- Machine: managed build-server vServer, 2 vCPUs, Intel Xeon
  (Skylake, IBRS, no TSX), Debian 12.15, kernel 6.1.0-56-amd64,
  gcc 12.2.0 (bench.py machine string:
  Linux-6.1.0-56-amd64-x86_64-with-glibc2.36, 2 CPUs). This is
  NOT the machine the +/-1.6% noise floor of #11 was probed on
  (that probe ran on a Hetzner cx23, 2 vCPUs, AMD EPYC-Rome):
  same vCPU count, different CPU line. The noise floor was
  therefore re-derived on this machine before recording (see
  "Noise floor" below).
- Toolchain: rustc 1.100.0 (0ef1bc386 2026-11-09)
- Pinned oracle: tools/oracle/build-oracle.sh, binary sha256
  9af462136c77f812f44304ed2befe3bdf958f0d95eea879793db5e2678bf34e4
  (recorded in work/oracle/build-info.txt; identical to the oracle
  sha256 documented for the corpus, so cross-machine build
  determinism holds on this box too).
- Candidate build line: cd rust && cargo build --release --locked
  -p triangle-cli (release profile, default features, against
  rust/Cargo.lock), from the tree at master aededb55 plus this
  PR's CI/doc changes.
- Command: tools/oracle/run-bench.sh --candidate
  rust/target/release/triangle-cli --report
  tools/oracle/work/reports/r0-baseline.json
  (timed set sizes m+l of the harness corpus = 80 runs,
  3 warmup + 25 timed per side, strictly interleaved, median per
  side; all numbers below come from bench.py's own report JSON).

## Aggregate

| statistic | value |
| --- | --- |
| oracle total (sum of per-run medians) | 0.9540 s |
| candidate total (sum of per-run medians) | 2.6158 s |
| sum-of-medians total ratio (candidate/oracle) | 2.742 |
| geometric mean of per-run ratios | 2.614 |
| worst per-run ratio | 8.184 (grid_l/pen) |

## Gate numbers (1.05 / 1.05 / 1.35, ticket #11): FAIL, stated plainly

- sum-of-medians total ratio 2.742 > 1.05: FAIL
- geometric mean 2.614 > 1.05: FAIL
- worst per-run ratio 8.184 (grid_l/pen) > 1.35: FAIL

The R0 baseline fails all three gate numbers. At R0 this is
advisory, exactly as ADR 0001 provides for a stage-1
transliteration: the transliterated code may legitimately sit
above the band, and this fail blocks nothing here. No
optimization, tuning, or number improvement was attempted in the
PR that records this baseline; the bad ratio is recorded as
measured. From now on this table is the no-regression reference
for every idiomatization rung boundary: each rung is measured
against it with the same command on the same class of machine,
and a rung may not regress it.

## Per-run ratio table (80 timed runs, median per side)

| case | family | size | oracle median (s) | candidate median (s) | ratio |
| --- | --- | --- | --- | --- | --- |
| grid_m | plain | m | 0.0036 | 0.0089 | 2.435 |
| grid_m | q20 | m | 0.0045 | 0.0099 | 2.220 |
| grid_m | p | m | 0.0037 | 0.0095 | 2.583 |
| grid_m | pq20 | m | 0.0038 | 0.0091 | 2.383 |
| grid_m | pq20U120 | m | 0.0039 | 0.0098 | 2.489 |
| grid_m | pa | m | 0.0126 | 0.0370 | 2.930 |
| grid_m | pq20a | m | 0.0127 | 0.0370 | 2.924 |
| grid_m | pen | m | 0.0052 | 0.0291 | 5.640 |
| grid_l | plain | l | 0.0077 | 0.0230 | 2.981 |
| grid_l | q20 | l | 0.0083 | 0.0230 | 2.759 |
| grid_l | p | l | 0.0078 | 0.0223 | 2.848 |
| grid_l | pq20 | l | 0.0084 | 0.0248 | 2.962 |
| grid_l | pq20U120 | l | 0.0083 | 0.0237 | 2.845 |
| grid_l | pa | l | 0.1117 | 0.3284 | 2.939 |
| grid_l | pq20a | l | 0.1064 | 0.3041 | 2.859 |
| grid_l | pen | l | 0.0106 | 0.0868 | 8.184 |
| random_m | plain | m | 0.0036 | 0.0087 | 2.375 |
| random_m | q20 | m | 0.0086 | 0.0148 | 1.709 |
| random_m | p | m | 0.0037 | 0.0085 | 2.335 |
| random_m | pq20 | m | 0.0075 | 0.0135 | 1.811 |
| random_m | pq20U120 | m | 0.0106 | 0.0190 | 1.797 |
| random_m | pa | m | 0.0136 | 0.0375 | 2.765 |
| random_m | pq20a | m | 0.0135 | 0.0414 | 3.060 |
| random_m | pen | m | 0.0046 | 0.0259 | 5.573 |
| random_l | plain | l | 0.0076 | 0.0211 | 2.781 |
| random_l | q20 | l | 0.0258 | 0.0458 | 1.777 |
| random_l | p | l | 0.0084 | 0.0242 | 2.882 |
| random_l | pq20 | l | 0.0288 | 0.0522 | 1.811 |
| random_l | pq20U120 | l | 0.0407 | 0.0624 | 1.534 |
| random_l | pa | l | 0.0108 | 0.0378 | 3.499 |
| random_l | pq20a | l | 0.0153 | 0.0581 | 3.790 |
| random_l | pen | l | 0.0106 | 0.0832 | 7.855 |
| pslg_hole_m | plain | m | 0.0028 | 0.0059 | 2.127 |
| pslg_hole_m | q20 | m | 0.0061 | 0.0100 | 1.627 |
| pslg_hole_m | p | m | 0.0028 | 0.0056 | 2.027 |
| pslg_hole_m | pq20 | m | 0.0057 | 0.0105 | 1.827 |
| pslg_hole_m | pq20U120 | m | 0.0083 | 0.0129 | 1.559 |
| pslg_hole_m | pa | m | 0.0146 | 0.0427 | 2.926 |
| pslg_hole_m | pq20a | m | 0.0145 | 0.0429 | 2.955 |
| pslg_hole_m | pen | m | 0.0037 | 0.0167 | 4.516 |
| pslg_hole_l | plain | l | 0.0056 | 0.0140 | 2.481 |
| pslg_hole_l | q20 | l | 0.0162 | 0.0274 | 1.688 |
| pslg_hole_l | p | l | 0.0052 | 0.0139 | 2.668 |
| pslg_hole_l | pq20 | l | 0.0148 | 0.0271 | 1.829 |
| pslg_hole_l | pq20U120 | l | 0.0229 | 0.0351 | 1.532 |
| pslg_hole_l | pa | l | 0.0063 | 0.0207 | 3.292 |
| pslg_hole_l | pq20a | l | 0.0098 | 0.0340 | 3.471 |
| pslg_hole_l | pen | l | 0.0073 | 0.0496 | 6.829 |
| pslg_concave_m | plain | m | 0.0028 | 0.0061 | 2.145 |
| pslg_concave_m | q20 | m | 0.0057 | 0.0099 | 1.718 |
| pslg_concave_m | p | m | 0.0033 | 0.0066 | 2.014 |
| pslg_concave_m | pq20 | m | 0.0066 | 0.0115 | 1.755 |
| pslg_concave_m | pq20U120 | m | 0.0073 | 0.0121 | 1.670 |
| pslg_concave_m | pa | m | 0.0185 | 0.0539 | 2.909 |
| pslg_concave_m | pq20a | m | 0.0160 | 0.0518 | 3.236 |
| pslg_concave_m | pen | m | 0.0034 | 0.0143 | 4.164 |
| pslg_concave_l | plain | l | 0.0053 | 0.0128 | 2.433 |
| pslg_concave_l | q20 | l | 0.0123 | 0.0216 | 1.758 |
| pslg_concave_l | p | l | 0.0052 | 0.0138 | 2.634 |
| pslg_concave_l | pq20 | l | 0.0125 | 0.0239 | 1.913 |
| pslg_concave_l | pq20U120 | l | 0.0185 | 0.0294 | 1.583 |
| pslg_concave_l | pa | l | 0.0061 | 0.0185 | 3.029 |
| pslg_concave_l | pq20a | l | 0.0073 | 0.0268 | 3.677 |
| pslg_concave_l | pen | l | 0.0069 | 0.0463 | 6.740 |
| pslg_wall_m | plain | m | 0.0030 | 0.0061 | 2.046 |
| pslg_wall_m | q20 | m | 0.0056 | 0.0095 | 1.692 |
| pslg_wall_m | p | m | 0.0028 | 0.0060 | 2.114 |
| pslg_wall_m | pq20 | m | 0.0055 | 0.0096 | 1.737 |
| pslg_wall_m | pq20U120 | m | 0.0075 | 0.0120 | 1.614 |
| pslg_wall_m | pa | m | 0.0126 | 0.0377 | 2.978 |
| pslg_wall_m | pq20a | m | 0.0147 | 0.0446 | 3.038 |
| pslg_wall_m | pen | m | 0.0036 | 0.0159 | 4.449 |
| pslg_wall_l | plain | l | 0.0055 | 0.0136 | 2.467 |
| pslg_wall_l | q20 | l | 0.0140 | 0.0245 | 1.751 |
| pslg_wall_l | p | l | 0.0061 | 0.0148 | 2.432 |
| pslg_wall_l | pq20 | l | 0.0137 | 0.0245 | 1.784 |
| pslg_wall_l | pq20U120 | l | 0.0197 | 0.0315 | 1.602 |
| pslg_wall_l | pa | l | 0.0063 | 0.0201 | 3.192 |
| pslg_wall_l | pq20a | l | 0.0089 | 0.0307 | 3.442 |
| pslg_wall_l | pen | l | 0.0069 | 0.0457 | 6.637 |

## Noise floor (re-derived on this machine)

The +/-1.6% aggregate wander documented in #11 was probed on a
Hetzner cx23 (2 vCPUs, AMD EPYC-Rome). This baseline was measured
on the Intel Skylake build server above, so the noise probe was
re-derived first (candidate = pinned oracle, same command, three
probes, report JSONs noise-probe-1/2/3.json):

| probe | total ratio | geomean | worst per-run ratio |
| --- | --- | --- | --- |
| 1 | 0.997 | 0.992 | 1.068 (pslg_wall_m/pen) |
| 2 | 1.007 | 1.000 | 1.127 (pslg_wall_m/pq20) |
| 3 | 1.004 | 1.008 | 1.120 (random_l/pq20a) |

- Aggregate wander within about +/-0.7% of 1.000 across probes
  (tighter than the documented +/-1.6%).
- Worst identical-binary per-run ratio across the three probes
  (240 per-run ratios): 1.127, below the 1.35 per-run tripwire.
- The 1.05 / 1.05 / 1.35 band from #11 therefore still clears
  this machine's noise floor; the band is unchanged and stays
  binding.

## Corpus status at R0

Differential harness, candidate = the same built release CLI:
120/120 runs byte-identical at the head of this PR (RESULT: all
120 runs byte-identical; report work/reports/last-run.json),
branched from master aededb55, against the pinned oracle binary
sha256
9af462136c77f812f44304ed2befe3bdf958f0d95eea879793db5e2678bf34e4.
The serial byte-identity gate is green at R0 and stays binding
(now also wired as the permanent CI gate,
.github/workflows/oracle-harness.yml).
