# R0 performance baseline

Per ADR 0001 (docs/adr/0001-port-strategy.md) the performance gate
(wayfinder #11) runs at stage-1 completion (R0) and at every
idiomatization rung boundary before a rung lands. This document
records the R0 baseline and fixes it as the no-regression
reference for every idiomatization rung boundary from now on:
a rung may not regress these numbers.

## Measurement

- Date: 2026-11-27
- Machine: managed build-server vServer, freshly provisioned for
  this measurement: 8 vCPUs, AMD EPYC-Genoa (Hetzner CPX42
  class, the class of the #11 CPX42 re-probe), Debian 12.15,
  kernel 6.1.0-56-amd64, gcc 12.2.0 (bench.py machine string:
  Linux-6.1.0-56-amd64-x86_64-with-glibc2.36, 8 CPUs). This is
  NOT the machine the +/-1.6% noise floor of #11 was originally
  probed on (that was a Hetzner cx23, 2 vCPUs, AMD EPYC-Rome):
  different machine class. The noise floor was therefore
  re-derived on this machine before recording (see "Noise
  floor" below). An earlier same-day measurement of this same
  command on a 2-vCPU Intel Xeon build server was discarded
  and is superseded by this run, repeated at the user's
  direction on a freshly provisioned server.
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
| oracle total (sum of per-run medians) | 0.3784 s |
| candidate total (sum of per-run medians) | 1.7322 s |
| sum-of-medians total ratio (candidate/oracle) | 4.578 |
| geometric mean of per-run ratios | 3.857 |
| worst per-run ratio | 16.098 (grid_l/pen) |

## Gate numbers (1.05 / 1.05 / 1.35, ticket #11): FAIL, stated plainly

- sum-of-medians total ratio 4.578 > 1.05: FAIL
- geometric mean 3.857 > 1.05: FAIL
- worst per-run ratio 16.098 (grid_l/pen) > 1.35: FAIL

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
| grid_m | plain | m | 0.0016 | 0.0054 | 3.342 |
| grid_m | q20 | m | 0.0020 | 0.0058 | 2.903 |
| grid_m | p | m | 0.0017 | 0.0057 | 3.449 |
| grid_m | pq20 | m | 0.0017 | 0.0057 | 3.386 |
| grid_m | pq20U120 | m | 0.0018 | 0.0058 | 3.201 |
| grid_m | pa | m | 0.0046 | 0.0242 | 5.231 |
| grid_m | pq20a | m | 0.0046 | 0.0244 | 5.256 |
| grid_m | pen | m | 0.0021 | 0.0211 | 10.183 |
| grid_l | plain | l | 0.0027 | 0.0141 | 5.204 |
| grid_l | q20 | l | 0.0031 | 0.0144 | 4.661 |
| grid_l | p | l | 0.0028 | 0.0144 | 5.192 |
| grid_l | pq20 | l | 0.0029 | 0.0144 | 5.022 |
| grid_l | pq20U120 | l | 0.0029 | 0.0146 | 4.963 |
| grid_l | pa | l | 0.0380 | 0.2164 | 5.692 |
| grid_l | pq20a | l | 0.0392 | 0.2199 | 5.608 |
| grid_l | pen | l | 0.0041 | 0.0667 | 16.098 |
| random_m | plain | m | 0.0018 | 0.0052 | 2.978 |
| random_m | q20 | m | 0.0035 | 0.0078 | 2.245 |
| random_m | p | m | 0.0016 | 0.0051 | 3.097 |
| random_m | pq20 | m | 0.0035 | 0.0082 | 2.322 |
| random_m | pq20U120 | m | 0.0048 | 0.0097 | 2.006 |
| random_m | pa | m | 0.0056 | 0.0271 | 4.864 |
| random_m | pq20a | m | 0.0056 | 0.0278 | 4.929 |
| random_m | pen | m | 0.0023 | 0.0207 | 8.795 |
| random_l | plain | l | 0.0035 | 0.0152 | 4.325 |
| random_l | q20 | l | 0.0101 | 0.0259 | 2.563 |
| random_l | p | l | 0.0035 | 0.0149 | 4.293 |
| random_l | pq20 | l | 0.0099 | 0.0258 | 2.614 |
| random_l | pq20U120 | l | 0.0142 | 0.0306 | 2.155 |
| random_l | pa | l | 0.0044 | 0.0258 | 5.867 |
| random_l | pq20a | l | 0.0062 | 0.0402 | 6.449 |
| random_l | pen | l | 0.0047 | 0.0731 | 15.393 |
| pslg_hole_m | plain | m | 0.0014 | 0.0036 | 2.483 |
| pslg_hole_m | q20 | m | 0.0028 | 0.0056 | 2.013 |
| pslg_hole_m | p | m | 0.0014 | 0.0037 | 2.593 |
| pslg_hole_m | pq20 | m | 0.0030 | 0.0064 | 2.113 |
| pslg_hole_m | pq20U120 | m | 0.0035 | 0.0068 | 1.939 |
| pslg_hole_m | pa | m | 0.0056 | 0.0263 | 4.729 |
| pslg_hole_m | pq20a | m | 0.0062 | 0.0273 | 4.424 |
| pslg_hole_m | pen | m | 0.0022 | 0.0126 | 5.723 |
| pslg_hole_l | plain | l | 0.0022 | 0.0084 | 3.836 |
| pslg_hole_l | q20 | l | 0.0058 | 0.0140 | 2.404 |
| pslg_hole_l | p | l | 0.0023 | 0.0084 | 3.715 |
| pslg_hole_l | pq20 | l | 0.0061 | 0.0149 | 2.445 |
| pslg_hole_l | pq20U120 | l | 0.0089 | 0.0179 | 2.014 |
| pslg_hole_l | pa | l | 0.0027 | 0.0140 | 5.153 |
| pslg_hole_l | pq20a | l | 0.0038 | 0.0223 | 5.833 |
| pslg_hole_l | pen | l | 0.0030 | 0.0379 | 12.476 |
| pslg_concave_m | plain | m | 0.0013 | 0.0034 | 2.610 |
| pslg_concave_m | q20 | m | 0.0024 | 0.0052 | 2.144 |
| pslg_concave_m | p | m | 0.0014 | 0.0036 | 2.525 |
| pslg_concave_m | pq20 | m | 0.0025 | 0.0055 | 2.237 |
| pslg_concave_m | pq20U120 | m | 0.0032 | 0.0063 | 1.975 |
| pslg_concave_m | pa | m | 0.0073 | 0.0367 | 5.016 |
| pslg_concave_m | pq20a | m | 0.0073 | 0.0381 | 5.216 |
| pslg_concave_m | pen | m | 0.0018 | 0.0113 | 6.452 |
| pslg_concave_l | plain | l | 0.0026 | 0.0092 | 3.483 |
| pslg_concave_l | q20 | l | 0.0051 | 0.0131 | 2.561 |
| pslg_concave_l | p | l | 0.0022 | 0.0084 | 3.762 |
| pslg_concave_l | pq20 | l | 0.0051 | 0.0134 | 2.647 |
| pslg_concave_l | pq20U120 | l | 0.0072 | 0.0158 | 2.202 |
| pslg_concave_l | pa | l | 0.0028 | 0.0143 | 5.147 |
| pslg_concave_l | pq20a | l | 0.0037 | 0.0210 | 5.750 |
| pslg_concave_l | pen | l | 0.0032 | 0.0381 | 11.857 |
| pslg_wall_m | plain | m | 0.0015 | 0.0037 | 2.510 |
| pslg_wall_m | q20 | m | 0.0027 | 0.0056 | 2.083 |
| pslg_wall_m | p | m | 0.0016 | 0.0038 | 2.375 |
| pslg_wall_m | pq20 | m | 0.0029 | 0.0061 | 2.104 |
| pslg_wall_m | pq20U120 | m | 0.0036 | 0.0067 | 1.899 |
| pslg_wall_m | pa | m | 0.0057 | 0.0270 | 4.749 |
| pslg_wall_m | pq20a | m | 0.0056 | 0.0274 | 4.944 |
| pslg_wall_m | pen | m | 0.0016 | 0.0108 | 6.686 |
| pslg_wall_l | plain | l | 0.0022 | 0.0081 | 3.712 |
| pslg_wall_l | q20 | l | 0.0053 | 0.0131 | 2.482 |
| pslg_wall_l | p | l | 0.0022 | 0.0083 | 3.750 |
| pslg_wall_l | pq20 | l | 0.0053 | 0.0136 | 2.546 |
| pslg_wall_l | pq20U120 | l | 0.0077 | 0.0162 | 2.122 |
| pslg_wall_l | pa | l | 0.0027 | 0.0137 | 5.106 |
| pslg_wall_l | pq20a | l | 0.0037 | 0.0207 | 5.663 |
| pslg_wall_l | pen | l | 0.0030 | 0.0373 | 12.419 |

## Noise floor (re-derived on this machine)

The +/-1.6% aggregate wander documented in #11 was probed on a
Hetzner cx23 (2 vCPUs, AMD EPYC-Rome). This baseline was measured
on the 8-vCPU EPYC-Genoa build server above, so the noise probe
was re-derived first (candidate = pinned oracle, same command,
three probes, report JSONs noise-probe-1/2/3.json):

| probe | total ratio | geomean | worst per-run ratio |
| --- | --- | --- | --- |
| 1 | 0.998 | 0.998 | 1.032 (grid_l/plain) |
| 2 | 0.995 | 0.999 | 1.062 (pslg_concave_l/plain) |
| 3 | 1.002 | 1.001 | 1.046 (grid_m/pq20) |

- Aggregate wander within about +/-0.5% of 1.000 across probes.
- Worst identical-binary per-run ratio across the three probes
  (240 per-run ratios): 1.062, below the 1.35 per-run tripwire.
- Note: the earlier CPX42-class re-probe documented in #11
  reported aggregate wander of about +/-2.5% and per-run
  tripwire overruns from host-side per-exec bimodality. That
  behavior is not reproduced on this box in these probes; these
  measurements are the binding floor for this baseline (the
  disagreement with the documented re-probe is recorded here,
  today's measurement wins). If the bimodality returns, the
  m+l aggregate stays the binding statistic (interleave +
  median + 80-run sum averages it out).
- The 1.05 / 1.05 / 1.35 band from #11 therefore clears this
  machine's noise floor; the band is unchanged and stays
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

Superseded as the no-regression reference by the R1 post-campaign
(docs/perf-r1-remeasure.md, measured on a fresh CPX42-class instance
at master 104172dc): from the merge of that PR on, that table is
the no-regression reference for every idiomatization rung
boundary; this document stays unchanged as the R0 record.
