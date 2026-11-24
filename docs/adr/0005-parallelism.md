# Parallelism: phase-scoped std::thread multithreading, claim-based refinement, structural validation

Status: accepted 2026-11-24; decided in the wayfinder ticket Parallelism design: phases, determinism, and primitives for internal multithreading (#12), on the port strategy (ADR 0001, ticket #6), the core data model (ADR 0002, ticket #8), the memory pools (ADR 0003, ticket #9), the crate and API architecture (ADR 0004, ticket #10, whose Unsuitable/Send-Sync, Mesh-reentrancy, and TRIANGLE_THREADS rulings this ADR owns), the performance gate (ticket #11 including its CPX42 noise re-derivation), and the C UB inventory (ticket #4, docs/research/ub-inventory.md on branch research/ub-inventory). Glossary terms added: serial path, parallel path, structural validation, worker, pool shard, claim sidecar, star claim, worker stream, MT axis.

## Decision inputs (settled, not re-derived here)

- Entry condition: a parity-green core. Internal multithreading (MT) lands as its own milestone after the idiomatization rungs and the ST end gate (ADR 0001; the #11 schedule table), not inside the transliteration rungs.
- The serial path keeps producing byte-identical outputs permanently; the CI corpus gate holds during MT integration (ADR 0001, #6, #11). The standing queues (badtriangles, badsubsegs, flipstackers, viri) are the expected swap-in points for parallel structures (#6 constraint on this ticket).
- The pool contract of ADR 0003 is binding and was deliberately kept MT-neutral: Copy u32 row ids, single-word packed handles, nothing thread-affine; per-worker pool shards plus a merge ordering were explicitly left decidable here without API change.
- ADR 0002's chokepoint is the only code that touches pool storage; unsafe is workspace-forbidden mechanically, with ADR 0004's rule that a later change relaxes it "consciously, module-locally, with a deviation-register entry" - a rule this ADR exercises exactly once (section 4).
- ADR 0004 shipped Unsuitable without Send/Sync supertraits and the Mesh handle without pinned bounds, both explicitly awaiting this ticket; TRIANGLE_THREADS is the env spelling the CLI parses; the trait, the funnel, and the candidate contract survive every ruling made here.
- MT hardware, settled by the user (2026-11-24, superseding the #11 Option A recommendation of a 16-vCPU CCX33-class box): the MT gate runs on the Hetzner CPX42 (8 AMD EPYC vCPUs, 16 GB), the machine the #11 noise re-derivation already measured. Gated thread counts T in {1, 2, 4, 8}; there is no 16-vCPU tier.
- New measurement, this session, on that CPX42 (single-threaded pinned C oracle, 7-rep medians, sizes from the harness corpus plus the opt-in xl class; method in the ticket comment): the phase-cost profile in section 2, the basis for every Amdahl number below.

## 1. The measured C workload profile (CPX42, single-threaded oracle)

Medians in seconds; oracle runs, process-level wall time, taskset 0-7, 7 reps (xl) / 15 reps (l):

| case (size) | plain | q20 | p | pq20 | pq20U120 | pa | pq20a | pen | notes |
|---|---|---|---|---|---|---|---|---|---|
| grid_xl | 0.033 | 0.037 | 0.028 | 0.032 | 0.030 | 3.113 | 3.083 | 0.041 | pq20a with -N -E (no node/ele output): 1.567 |
| random_xl | 0.359 | 1.444 | 0.370 | 1.450 | 2.086 | - | 0.836 | - | -NE variants: plain 0.183, pq20a 0.702, pq20U120 1.96; -i 1.013, -F 0.474 |
| pslg_hole_xl | - | - | 0.185 | 0.727 | 1.047 | - | 0.408 | - | pq20a -N -E: 0.260 |
| grid_l / random_l (medians) | 0.007 / 0.006 | 0.007 / 0.020 | - | 0.006 / 0.019 | 0.006 / 0.029 | - | 0.084 / 0.012 | - | l-class runs are startup-dominated on this machine |

Readings:

- Refinement dominates the timed aggregate. The quality/aCute/area families spend 75-97% of their own wall time inside quality enforcement (random_xl: q20 minus plain = 1.085 s of 1.444 s; the aCute machinery adds ~0.63 s on pq20U120; grid pq20a is ~97% refinement). Across the xl timed set the refinement share of the aggregate is ~69%.
- Output serialization is the second phase, not a rounding error. Suppressing the .node/.ele outputs (-N -E) cuts the output-heavy family grid_xl/pq20a from 3.083 s to 1.567 s (49% of the run) and random_xl/plain from 0.359 s to 0.183 s. Across the xl timed set, numbering+marshalling+formatting+writing is ~25% of the aggregate. Any MT milestone that leaves output serial caps the T=8 aggregate at ~3x no matter what refinement does.
- Construction is small but real. plainNE 0.183 s (random_xl) covers parse + divide-and-conquer construction + numbering; construction alone is ~2-3% of the xl aggregate (incremental -i is ~3x slower than divconq even serially: 1.013 vs 0.359 s; sweepline -F: 0.474 s).
- Parse and the trivial passes are noise. Text input for 100k points is ~1-3% of the aggregate; highorder runs only under -o2, which the corpus does not exercise.
- The l class is startup-dominated on the CPX42 (grid_l medians 6-8 ms; the machine's documented per-exec bimodality, two wall-time modes ~2.2x apart on short runs, lands exactly there), while xl runs are clean (#11 follow-up). This drives the binding/advisory split of section 9.

Amdahl decomposition used below (xl timed set, oracle-side): refinement ~0.69 (parallelizable, with conflict losses), output serialization ~0.25 (parallelizable, conflict-free), construction ~0.02 (parallelizable), parse + numbering + fixed serial rest ~0.04-0.05 (serial).

## 2. Phase inventory: what the MT milestone parallelizes

triangulate()'s phases (triangle_api.c tri_mesh_create + tri_mesh_copy), ruled IN or OUT for the first MT milestone, with the structural reason:

| Phase (C anchor) | Ruling | Structural reason |
|---|---|---|
| Input parse + transfernodes (triangle.c:7697) | OUT | O(N) sequential copy into the vertex pool in input order (vertex ids are output-observable numbering); ~1-3% of the aggregate; parallelizing buys nothing measurable. |
| Bulk Delaunay construction, divide-and-conquer default (divconqrecurse triangle.c:4366, mergehulls 4062) | IN | Recursive spatial partition = natural fork-join: disjoint vertex subsets build private triangles; only the pairwise hull merges touch both sides; the recursion tree is deterministic given the input order, so joins merge in recursion order. Idempotent output: the Delaunay triangulation of a point set is unique, so scheduling cannot corrupt it (strong validation). |
| Bulk construction, incremental (-i) and sweepline (-F) | OUT | The incremental path is built around the splay tree (splayinsert 5134), a single mutate-while-locate structure; sweepline is a sweep with global event state. Both are also ~3x/1.3x slower than divconq serially (measured), so the corpus families never exercise them. They keep running serial inside a parallel run (phase-serial sandwich, section 8); no error is raised. |
| Segment insertion (formskeleton, scoutsegment 6003, insertsegment 6420, triangulatepolygon 3559; flipstackers) | OUT for milestone 1 | Per-segment channel walks flip a shared channel (flipstackers is a shared LIFO scratch mutated across the walk, poolrestart'ed per insertion at 3130/3239); conflicts are dense along segment paths and the C algorithm's flip decisions are order-sensitive. Measured share is small (~2-4%: random_xl p vs plain = 11 ms; pslg_hole p = 0.185 s of ~2.4 s class aggregate). Refinement already runs after segments, so serial segments do not block the scaling phases. |
| Hole carving, region attributes (carveholes 6964, plague; viri) | OUT for milestone 1 | Infection propagation is global and order-shaped; viri is a shared traversal pool (poolrestart 6868/6949); share ~1-2%. |
| Quality refinement (enforcequality 7527, testtriangle 2001, splittriangle 7409; badsubsegs + badtriangles) | IN, the core target | Work-item loop over standing queues: each item is local (star-sized) compute + one point insertion with a local location hint (the work item itself; no global point-location structure on this path). Items conflict only when their stars overlap; the mesh is large (xl: 1-2M triangles) so stars are small relative to the mesh. ~69% of the aggregate; the T=8 target lives or dies here. |
| aCute refinement (findNewSPLocation via splittriangle 7437, acute scratch buffers) | IN, as part of quality refinement | Per-item compute over the bad triangle's star via the eight f64 scratch buffers; parallelizes with the claiming worker, no new queue. Requires per-worker scratch (section 6). ~30% of pq20U120 runtime rides here. |
| Output serialization (numbernodes 7840, writenodes 7768, writeelements 7863, writepoly 7933, writeedges 7983; CLI file writers) | IN | ~25% of the aggregate (measured) - the cheapest scaling win. Numbering stays serial (vertex numbering derives from pool traversal and is an O(N) in-order scan), then the per-element bodies format/write flat id ranges: conflict-free map. The writer body (the I/O-phase code that formats and writes files) must exist and be parallel-safe by the MT milestone; per-worker byte buffers concatenate in id order, which keeps output byte layout fixed on the parallel path (this is partitioned I/O, not mesh determinism). |
| Statistics/quality surface (quality_statistics 8143, checkmesh/checkdelaunay) | OUT | Post-run &Self scans; cheap; also the parallel path's validation instrument (section 3). |

Which phases the T=8 target actually depends on: refinement (~69%) and output serialization (~25%); construction (~2%) is IN because it is nearly free to include and keeps the plain/q20-lite families moving, but it cannot move the aggregate. The serial tail (parse, numbering, joins, fixed overhead) is ~4-5% and hard-caps the T=8 aggregate at ~20x even with perfect phases; at realistic phase efficiencies the modelled aggregate lands at 4.6-4.9x (section 9).

## 3. Determinism ruling (decided first; it conditions everything below)

Ruling: option (b) of the ticket - freer scheduling on the parallel path, validated structurally per the #11 method; parallel outputs are never byte-compared; the serial path remains the port's only byte-contract and its anchor for reproducibility, debugging, and the corpus gate forever. A deterministic parallel scheduler is rejected for this milestone and recorded as a revisitable alternative, not as a deferred feature.

Justification against the corpus-gate reality:

- The reproducibility need is already served. The binding gate (run-harness.sh, corpus, byte-for-byte) runs the serial path, at every gate invocation, permanently, including during and after MT integration. Gate runs are reproducible exactly because they are serial; parallel runs are benchmark subjects, not parity subjects.
- Deterministic scheduling forfeits what MT is buying. The bad-triangle discipline is priority-ordered (dequeuebadtriang 1835 pops the globally largest-worst bucket, 4096 buckets, firstnonemptyq). Bit-stable parallel refinement means the global pop order is preserved, i.e. a lookahead scheduler that runs only provably-independent items concurrently and sequence-aligns everything else; the measured conflict structure of refinement (star overlaps at the pop frontier) makes that effectively the serial order on one worker with idle helpers, collapsing the 69% phase. Determinism is not a tax on top of scaling here; it is the negation of it.
- The debugging cost of freer scheduling is bounded and instrumented. The serial ground truth (same options, T=1) is one env var away for any mesh; structural validation (section 3) plus checkmesh/checkdelaunay catch corruption independent of scheduling; the construction phase is provably scheduling-independent (Delaunay uniqueness), and the rng derivation (section 7) is per-subtree so construction's randomized pivots do not even depend on worker assignment. What genuinely varies across T is refinement outcome, and it varies within a structural band, not chaotically.
- Pool and queue constraints. Pool LIFO dead stacks and allocation-order traversal are per-pool invariants ADR 0003 pins for byte-identity, which only the serial path promises. A deterministic-parallel pool would need globally sequenced id assignment and merge protocols on every alloc path - all of that machinery exists in the design below anyway (shards merge deterministically by owner order where merges occur), but requiring bit-order across T adds a global sequence discipline to the hot path with no external consumer of the result.

Advisory note recorded for later tickets: if a strong need for reproducible parallel outputs ever materializes (e.g. regression bisection across thread counts), the revisit is a new ticket against the map; the primitives chosen below do not structurally forbid adding a sequenced pop discipline later, only its performance.

Structural validation definition (this sharpens the #11 wording, which assumed count equality everywhere; the honest split was unavoidable - see below): parallel runs are validated against the serial Rust run on the same input by

1. parallel-invariant families (plain, p, anything without quality enforcement): identical vertex/segment/triangle counts and identical attribute sets - strong equality, because the underlying Delaunay/CDT is unique regardless of schedule;
2. refinement families (q20, pq20, pq20U120, pa, pq20a, pen with -q): identical input-derived counts (segments, markers), triangle count and vertex count within a recorded band (provisional: within +/-5% of the serial run's counts; calibrated at the first MT milestone run), because Ruppert-style refinement outcome depends on processing order - the parallel mesh is valid, not identical;
3. zero failures of the ported invariant checkers (check_mesh()/check_delaunay() on the produced Mesh) on every variant.

Point 2 corrects the #11 comment line ("identical vertex/segment/triangle counts and attributes" for every variant): that expectation holds for the parallel-invariant families and cannot hold for order-sensitive refinement; the validation method carries the split, and the provisional band replaces the impossible equality there.

## 4. Primitives: std::thread with explicit scoped phases

Decision: std::thread::scope with explicitly coded per-phase coordination - static id-range partitioning for construction and output, and a claim-based work queue for refinement. No rayon, no custom work-stealing layer.

Against the criteria:

- Dependency footprint: ADR 0004 keeps the lib crate at zero dependencies day one; rayon pulls rayon-core plus crossbeam-utils/epoch-style transitive deps into the workspace. Whatever MT needs beyond std (scoped spawns, atomics, one mutex per bucket, channels) is std already.
- Determinism compatibility: irrelevant by the ruling in section 3 - freer scheduling is chosen, so rayon's work-stealing would be permissible; but nothing in the three work shapes rewards it: construction is a balanced fork-join tree over a deterministic partition (no steal needed), output serialization is a flat range map (no steal needed), and refinement gains work-sharing from a claim queue, where stealing's benefit (load balancing) is provided by the queue itself and locality comes from the claim discipline.
- Work shapes: point-location walks stay inside the claiming worker's star (the work item carries the hint); refinement queue draining is irregular, contention-managed work - a queue with ordered claims, not a data-parallel map; output serialization is the one shape rayon fits natively, and it is a 5-line scoped map over id ranges in std.
- Runtime shape: threads are spawned per run inside the call scope, exactly T of them, Tor-size-independent, so taskset pinning (#11's --pin) and the benchmark's process-level wall-time metric stay honest; a global rayon pool sized independently of TRIANGLE_THREADS would decouple "threads used" from "threads requested" on the very axis the gate measures.
-unsafe-free preference: std::thread::scope, atomics and Mutex are safe; a custom work-stealing deque done right is the kind of code that drags unsafe into the workspace against the ADR 0004 rule; if load imbalance ever shows up in milestone telemetry, the queue grows a steal primitive then, behind measurements.

The first consciously gated unsafe pocket of the port (the one place this ADR exercises ADR 0004's relaxation rule): parallel-phase row access through the pools' flat store by several workers. Mutating refinement work (cavity rebuild) touches rows from multiple threads in place; the pools are Vec<u64> behind &mut. Ruling: the parallel path wraps each pool's word store in an UnsafeCell-backed store inside the existing slots/pool chokepoint, with the claim discipline of section 6 guaranteeing exclusive write access per row (and read access from anywhere on live, unclaimed rows is schedule-tolerant because the algorithm only reads rows under the same claims it holds). Pool growth is synchronized separately (a grow mutex and an epoch flag; amortized, off the hot path). This is a deviation-register entry (assigned at the MT implementation), module-local to the chokepoint, debug-asserted (claim sidecar assertions), and the MT corpus subset later stays Miri-runnable: Miri's thread support detects discipline violations at the exact level that matters here (Miri does not care about the C-serial path, which keeps the existing full-corpus Miri acceptance unchanged). The serial path bypasses the wrapper entirely and keeps today's safe Vec access.

The parallel-path code lives in its own module (parallel.rs sketch per the ADR 0004 module map); T=1 routes to the untouched transliterated path at runtime - same binary, no feature gate, no compile-time fork of the core.

## 5. Pools and state strategy (MT mapped onto ADR 0003's WordPool)

Option chosen: a per-worker shard overlay on top of unchanged WordPools, plus the UnsafeCell-backed shared store of section 4 - not shared-pool-behind-fine-grained-locks, not phase-split snapshotting.

- The WordPool type and its contract (flat Vec<u64>, stride-fixed rows, LIFO dead stack through slot 0, allocation-order traversal, counts) are unchanged; every ADR 0003 guarantee a serial caller observes is untouched. Parallel machinery sits above the pool, inside the chokepoint.
- Serial path (T=1): the plain pools, byte-identical behavior, zero unsafe. This is what the corpus gate runs.
- Parallel construction (divconq): workers build subtrees on private pool shards (private WordPool instances), mergehulls-style joins run per recursion-pair at the scoped joins, and the join merges shard rows into the main pool with a parallel id-rebase pass (bases deterministic: the recursion tree fixes all merge orders independent of timing; the rebase is one flat map adding the shard base to the high 32 bits of every packed word - the ADR 0002 encoding makes rebase purely arithmetic). Outcome: allocation-order traversal remains meaningful (rows ordered by recursion order), and construction output is provably schedule-invariant (unique Delaunay).
- Parallel refinement: the mesh pools become the shared UnsafeCell-backed store; fresh ids are handed from per-shard reserved id blocks (contiguity per shard; blocks pre-reserved from the pool's growth epoch); dead rows return to the killing worker's own dead-stack chain (LIFO per shard). Exact global LIFO is a byte-identity property and is deliberately relaxed on the parallel path (per-pool LIFO per shard instead); the deviation is recorded because reuse order is output-observable - on the serial path only, which keeps it exact.
- Refinement point location needs no shared locational structure: work items carry their hints (the bad triangle, the encroached subseg), the walks stay inside claimed stars. The splay tree (splayinsert 5134) exists only on the incremental path, which stays serial; no concurrent splay access ever occurs. sampleraise-style sampled starts (the randomnation 2503 site) may read arbitrary rows: sampled reads are claims-free reads on a schedule-tolerant store (they only pick a walk start; a stale start costs walk length, never correctness).

## 6. Queue strategy: the standing queues under MT

- badtriangles -> concurrent claim queue (the core swap-in). C's shape is kept: badtriag rows in the pool (payload opaque words per ADR 0003 section 4), intrusive nexttriang links, the 4096 size buckets, worst-bucket-first pops (dequeuebadtriang 1835). Parallel shape: each bucket is a mutex-guarded intrusive list; bucket occupancy bookkeeping (firstnonemptyq/nextnonemptyq) becomes an atomic lattice exactly mirroring the C linkage pattern; a worker pops from the worst non-empty bucket under the bucket mutex and immediately attempts the star claim (below). Deliberately not built: per-worker replica queues with global ordering - they complicate the bucket priority and the shared-bucket pop is cheap relative to a split (pops are one per mesh surgery). The pool rows holding queue payloads are ordinary rows; their nexttriang links are touched only under the bucket mutex.
- badsubsegs -> the same discipline on one mutex-guarded FIFO list (volume is small; subseg splits enqueue bad triangles into the claim queue and adjacent star work rides that path).
- flipstackers -> serial, unchanged (segment insertion runs inside the phase-serial sandwich for milestone 1).
- viri -> serial, unchanged (carving, plague, region infection run in serial phases).
- The aCute scratch buffers become per-worker instances of the same eight-buffer shape (ADR 0003 section 4, only-grow, zero-filled resize), one set per worker; no ids, no sharing.

Conflict resolution, ruled option by option (ticket criterion: work on triangle stars, edge locking, plane sweep partitioning, conservative two-phase):

- Chosen: star claims on a claim sidecar - a per-row claim flag array (atomic), one flag word per triangle row, outside the row words so row layouts stay C-identical. A worker processing a refinement item claims, all-or-nothing, the set of triangle rows its operation will touch (the star of the bad triangle; growing to the cavity as it is discovered during insertion), acquiring rows in row-id order (deadlock-free by total order). A row already claimed, dead, or stale fails the claim: the worker releases everything, re-enqueues the item, and takes the next one (optimistic concurrency; conflicts on ~1M-triangle meshes are rare because stars are small). Staleness revalidation happens on pop (the dequeued entry's triangle must still be live and unprocessed, the same "is it still the same triangle" discipline C performs serially via its marks).
- Edge locking (per-mesh-edge locks): rejected - strictly finer-grained than the algorithm's touch set (a cavity edit rewrites several rows), so it composes into the same exclusivity with more lock traffic; row-order claims give a trivial deadlock discipline edge locks lack.
- Plane sweep / static mesh partitioning (workers own slabs, boundary work exchanged via halo queues): rejected for milestone 1 - refinement insertions are data-driven and rebalance the load geometrically; slabs would need live repartitioning machinery to keep cores busy, which is a second scheduler to build and validate. Recorded as the fallback if milestone telemetry shows claim-conflict rates high enough to miss the scaling targets (the telemetry - conflicts, aborts, retries per phase - is part of the milestone's debug build).
- Conservative two-phase (workers propose cavity edits, a serial applicator applies): rejected - the applicator is the serial phase measured into the 69% share; Amdahl kills it.

## 7. RNG ruling

Serial path: the thread-local LCG (randomseed = 1 per context, randomnation triangle.c:1521-1524, constants 1366/150889/714025) is already owned by the run state at the API shell (ADR 0004 D10); its call sequence stays exactly C's.

Parallel path: the RNG does not influence any output that structural validation observes. Measured consumers: divconq sort pivots (randomnation sites in the vertex-median recursion), the sampled starting triangles of jump-and-march location (sampleraise/getvertex), and the SAMPLERATE jumps of the incremental splay path (5305-5365, serial anyway). Pivots affect sort cost, not the (unique) triangulation; sampled starts affect walk length, not the located triangle. Exact arithmetic decides the geometry, never the LCG. Ruling: one derived LCG worker stream per spawn, same constants, seeds derived from a fixed hash of the context seed with a structure key: construction streams key on the subtree identity (level, index in recursion tree), making construction's random choices invariant to worker assignment and thread count; location-sample streams key on the worker. No insertion-order dependency survives: refinement itself draws no randomness, and its remaining order dependence is exactly the structural band of section 3, point 2. Deviation-register entry at the implementation (derived streams vs the single context sequence).

## 8. API rulings deferred to this ticket (ADR 0004 hand-offs)

- Unsuitable: the trait itself stays object-safe, one method, no supertraits; the storage site bound becomes Send + Sync: Options carries Box<dyn Unsuitable + Send + Sync>. Reason: testtriangle (the seam, triangle.c:2089 in C) runs inside the claiming worker on the parallel path; funneling callbacks to a single coordinating thread would serialize the hottest loop of the scaling-critical phase; -u with T=1 stays available and unaffected. The ruling is the deviation-register entry D10's sibling (assigned at the MT implementation); the CLI-facing surface does not change shape.
- Callback invocation under MT: per worker, concurrently, in arbitrary order; a stateful implementation must be Sync (documented in the trait docs); return values may feed refinement that differs within the structural band - same validation as any refinement nondeterminism.
- TRIANGLE_THREADS: the env knob of the CLI (ADR 0004 spelling, exact) maps to a lib-side Options builder setting, Options::threads(usize), default 1; the CLI copies the parsed env into it (exit with Error::Options on unparsable, zero, or > 1024 values). T=1 (default or explicit) executes the serial path verbatim - the corpus-gated byte-identical route. T>1 selects the parallel path for the supported phases; unsupported combinations (currently -i construction, -F construction; segment insertion and carving within any run) execute their serial counterparts inside the phase-serial sandwich without error - the knob requests parallelism, it does not promise it per-option; the docs state which phases scale under which options.
- Mesh handle: auto Send + Sync (owned data, no interior sharing after the run; ADR 0002's chokepoint keeps no references alive). Reentrancy guarantees: independent triangulate() calls from different OS threads are fully reentrant (each context owns its run state incl. the RNG field, D10); moving a finished Mesh across threads and calling its &self surface from several threads is supported; concurrent refine()/mutating calls on one Mesh are not (no interior locking; document, do not enforce).

## 9. Gate hand-off: targets, schedule, and noise character (CPX42)

Final scaling targets (supersede the #11 provisional ladder, per this ticket's phase design and the measured profile):

| T | Aggregate target | Efficiency vs T=1 | Implied refinement-phase efficiency (with output ~0.9@T8, tail 4-5%) |
|---|---|---|---|
| 2 | >= 1.8x | 90% | ~95% |
| 4 | >= 3.0x | 75% | ~80% |
| 8 | >= 4.5x | 56.25% | ~63% |

Derivation: with the measured phase shares (~0.69 refinement, ~0.25 output, ~0.02 construction, ~0.04-0.05 serial) and the phase efficiencies the chosen primitives justify on million-triangle meshes (conflict losses at T=8 dominate the refinement-entropy; output formatting is memory-bandwidth-near-linear), the modelled aggregate lands at ~4.6-4.9x at T=8; 4.5x binds the milestone without demanding miracles. The #11 provisional 5.0x at T=8 (62.5%) presumed a lighter serial tail than the measured ~25% output share - unless output scales, that target is unmeetable, and even with output in, 62.5% aggregate efficiency demanded > 75% refinement efficiency at 8 threads, beyond the claimed-cavity ceiling. T=2/T=4 keep #11's 90/75% rungs: the derivation puts them at ~1.8x/3.1x.

Binding rules of the MT gate:

1. Method: unchanged #11 - TRIANGLE_THREADS variants of the MT timed set (sizes l and xl), candidate vs the same candidate at T=1 (never the C oracle), --pin taskset masks 0-1 / 0-3 / 0-7 for T=2/4/8, xl: 1 warmup + 5 timed, median per side; l-class runs add deeper medians on bimodal machine classes (below).
2. Binding statistic on the CPX42: the xl medians - the aggregate is the sum-of-medians over the xl entries, and the geomean of per-family xl efficiency; l-class entries run and are reported but are advisory on this machine (measured per-exec bimodality, two wall-time modes ~2.2x apart, lands exactly on l-sized runs: grid_l T=1 medians 6-8 ms). The aggregate+geomean gate then binds per the table above; one rerun allowed, binding on the rerun (unchanged).
3. Per-family binding floor (the anti-hiding guard #11 deferred to here): every timed family with a T=1 median >= 100 ms must reach >= 60% of the aggregate target at the same T (e.g. >= 2.7x at T=8) on its xl entries; sub-100ms families (the grid-lite set in this profile) are report-only on the CPX42 - startup noise, not scaling.
4. Structural validation of every variant per section 3 (parallel-invariant families exact; refinement families within the band, provisional +/-5%, calibrated at the milestone run; check_mesh/check_delaunay zero failures).
5. Schedule (per #11's table, MT milestone row): at the MT milestone, on the CPX42, binding: run-harness.sh serial corpus green; the ST end gate rerun on this machine (m+l aggregate 1.05 and geomean 1.05 binding; per-run 1.35 advisory on this machine class per the #11 noise re-derivation); the MT targets above. The stage-1 baseline and rung regressions are unaffected.
6. Noise character of the CPX42 vs the shared 2-vCPU box: aggregate wander ~+/-2.5% (cx23-class: ~+/-1.6%) - both comfortably inside the 1.05 band; per-exec bimodality (~2.2x modes) makes single short runs untrustworthy (per-run CV 28-30% vs 15-18%), so l-class MT medians get deeper repeats on this box: spec change, --repeats 9 --warmup 2 for the l tier (below). The plus sides: 8 threads of headroom, zero steal, SMT off, single NUMA node, xl runs clean (spot probe total ratio 1.006, worst per-run 1.18), sustained runs 27-40% faster than the cx23 - the MT axis measures scaling, and its baseline (T=1 on this box) is measured under the same conditions.

run-bench.sh / bench.py spec changes for the T axis (spec only here; no implementation in this PR):

- Canonical MT gate command documented: tools/oracle/run-bench.sh --candidate <rust-cli> --baseline <rust-cli> --threads T --pin <mask> --sizes l,xl - the baseline side runs the same binary at default T=1 (TRIANGLE_THREADS is set on the candidate side only), which is exactly the candidate-vs-candidate comparison #11 defines; no bench.py change required for the pairing itself.
- run-bench.sh: when --sizes includes xl and work/corpus-xl is missing, generate it (generate_corpus.py --sizes xl --out .../corpus-xl) exactly as it builds the default corpus today; the sealed s,m,l corpus stays untouched.
- bench.py: MT repeat depth per size tier on bimodal machine classes - l-class entries timed with --repeats 9 --warmup 2, xl unchanged at 5+1 (documented; the two tiers can be two invocations until the flag exists).
- bench.py report: MT mode adds per-family efficiency columns (median(T)/median(T=1), efficiency = speedup/T), the per-family binding floor of rule 3, and the xl-binding/l-advisory split propagated to the printed aggregate lines.
- bench.py (or the harness validator): a structural compare step for the acceptance of each variant - parse the canonical outputs (.node/.ele/.poly) of the serial run and the MT run; assert exact equality for parallel-invariant families, the band rule for refinement families, and record the deltas in the report. It compares Rust-vs-Rust; the pinned C oracle is not party to it.
- The documented one-line switch to a CLI thread flag when the I/O phase lands (unchanged from #11).

## Consequences

- ADR 0002 and ADR 0003 survive untouched: the pool contract holds; handles stay Copy values; the packed-word id rebase of construction shards uses the ADR 0002 encoding arithmetically. The pools gain a parallel overlay, never a lock inside themselves.
- The port's first consciously gated unsafe pocket is specified, bounded (one store wrapper inside the chokepoint), deviation-registered, claim-asserted in debug builds, and Miri-covered on both paths (serial full corpus today; MT corpus subset at the milestone - Miri's thread support validates the claim discipline).
- ADR 0004's open hand-offs are resolved as recorded in section 8; the Options surface gains exactly one builder setting (threads); the CLI env spelling and the candidate contract are unchanged; the differential harness and the CI corpus gate run the serial path exactly as before - nothing in tools/oracle or the C sources changes in this PR.
- Glossary terms added: serial path, parallel path, structural validation, worker, pool shard, claim sidecar, star claim, worker stream, MT axis.
- The MT implementation tickets cut along: (a) the scoped-phase scheduler + pool shard overlay + store wrapper (chokepoint), (b) construction fork-join with deterministic rebasing, (c) claim queues + sidecar + refinement claim integration, (d) per-worker aCute scratch + worker-stream RNG, (e) output marshalling/writer parallel body, (f) the MT validation/telemetry tooling and the gate run.

## Considered alternatives

- Deterministic parallel scheduling: rejected - see section 3; recorded as revisit-able only through a new ticket if reproducible parallel outputs ever become a requirement worth the scaling.
- rayon: rejected - dependency footprint against ADR 0004's zero-dep day one, a global pool decoupled from TRIANGLE_THREADS on the axis the gate pins, and no work shape in the inventory that stealing serves better than a claim queue and a range map.
- Minimal custom work-stealing layer: rejected - second scheduler, unsafe pressure, no measured imbalance to solve; revisit behind milestone telemetry.
- Shared pools behind fine-grained locks (per-word or per-row-block mutexes): rejected - locks in the hottest kernels of the port (slot access is nanoseconds; the C-hot-path ADR 0003 measurement program rules this out), and it duplicates the exclusivity the claim sidecar provides at transaction granularity.
- Phase-split snapshotting for refinement (parallel read-only planning, serial applicator): rejected - the applicator re-serializes the 69% phase; Amdahl kills it. Its output-serialization cousin (number serially, write concurrently) is the kept kernel of the idea.
- Plane sweep / static slab partitioning of refinement: rejected for milestone 1 (dynamic insertion imbalance vs static slabs; repartitioning machinery), recorded as the fallback discipline if claim-conflict telemetry misses the targets.
- Edge locking: rejected - strictly weaker granularity economics than row claims; total row-id ordering is the deadlock story edge locks don't have.
- Parallel incremental (-i) or sweepline construction: rejected - splay-tree and sweep state are serial locational structures, and the measured serial costs (3x/1.3x slower than divconq) argue for never investing; divconq is the default and the only IN.
- Per-worker replica bad queues with a global sequencer, or a fully lock-free bucket lattice for milestone 1: rejected as premature - mutex-per-bucket over C's existing linkage pattern is the honest transliteration-shaped first rung; the queue seam is isolated so a lock-free interior is a contained later optimization.

## Explicitly not decided here

- The numeric refinement-family band beyond the provisional +/-5% (calibrated at the first MT milestone run, with its derivation recorded there).
- MT for segment insertion, carving, the incremental and sweepline constructors, and any phase beyond the milestone-1 inventory: no commitment made or foreclosed.
- Work-stealing or lock-free queue interiors (telemetry-gated, contained seams).
- Unsafe admission details inside the chokepoint wrapper (implementation ticket, bound by this ADR's rules).
- Oversubscription behavior (T > 8) beyond "allowed, unmeasured, unpromised".
- NUMA affinitization beyond taskset pinning.
- whether the CLI grows a --threads flag in addition to the env (I/O-phase ticket, per ADR 0004).
