# Architecture spec: the Triangle C-to-Rust port (library plus binary)

Status: accepted: merged via PR #20 on 2026-11-24. This document is the destination artifact of the wayfinder map ([issue #1](https://github.com/mestrode/triangle/issues/1)): it synthesizes the twelve resolved tickets (#2 through #13) and the five accepted ADRs into one implementable architecture. It adds no new decisions; every ruling below cites its owning document. Sign-off is the user's merge of the PR carrying this file, after which the map closes as completed.

## 1. Scope and goals

Port the C core of this repository (Triangle 1.6 + aCute, about 19k lines of C) to Rust as a library plus a binary, at performance parity with the C original.

Binding user constraints (map #1):

- Idiomatic Rust API: Result-based errors, no libc types at the boundary, no C ABI drop-in, no bindings.
- Core first; I/O and the CLI are later phases, sketched in section 8.
- 64-bit targets only (x86-64, aarch64), enforced by compile error, not implied.
- Performance: single-threaded parity with the pinned C oracle is the base criterion; internal multithreading (one mesh, several threads) is in scope and MT scaling is part of the binding gate.
- Tagged raw-pointer handles (the C trick) are permanently out; representation is index-based packed handles ([ADR 0002](adr/0002-core-data-model.md)).

Out of scope: 32-bit targets, C ABI compatibility (extern "C", context/triangleio structs), bindings to the C library instead of a port, tagged raw-pointer handles.

## 2. Normative sources

This spec is binding as a whole-of-architecture view; the per-decision documents are normative for their content. On any conflict, the ADR wins and this spec gets corrected.

| Source | Owning ticket | Content |
| --- | --- | --- |
| [ADR 0001](adr/0001-port-strategy.md) | #6 | Port strategy: verified transliteration, staged idiomatization |
| [ADR 0002](adr/0002-core-data-model.md) | #8 | Data model: packed handles, rows, chokepoint, zero stage-1 unsafe |
| [ADR 0003](adr/0003-memory-pools.md) | #9 | Memory management: the uniform WordPool |
| [ADR 0004](adr/0004-crate-api-architecture.md) | #10 | Crate layout, module map, public API, error funnel, deviation register |
| [ADR 0005](adr/0005-parallelism.md) | #12 | Parallelism: phases, primitives, structural validation, MT gate |
| [docs/function-mapping.md](function-mapping.md) | #10 | 195 seeded rows: every C function to its Rust module, stage and naming |
| [docs/deviation-register.md](deviation-register.md) | #10, #9, #12 | D1 through D11: every deliberate departure from 1:1 transliteration |
| [GLOSSARY.md](../GLOSSARY.md) | all | The shared vocabulary (this spec uses its terms) |
| [tools/oracle/README.md](../tools/oracle/README.md) | #13, #11 | The harness, the corpus, the performance gate commands |

Research backgrounds live on the read-only branches research/rust-landscape, research/robust-predicates, research/ub-inventory, research/oracle-determinism (docs/research/); they are inputs, never merged.

## 3. Verification architecture (built first, gates everything)

The differential harness exists since ticket #13 and precedes all core porting; every implementation ticket carries it in its Blocked by line.

- Pinned C oracle: built from this repo's C sources with -O2 (or -O3), -ffp-contract=off, no -march overrides, 64-bit, aCute compiled in. tools/oracle/build-oracle.sh records compiler, flags, source hashes and binary sha256. The C core is deterministic under this pinning (per-run LCG seeded 1 per context, no time or environment dependence).
- Corpus: deterministic and seeded (tools/oracle/generate_corpus.py), 15 cases (grid with degeneracy stress, random, PSLG with hole, concave L-shape, interior wall; sizes s/m/l) x 12 option families (plain, p, q20, pq20, pq20U120, pa, pq20a, pen, plus the -r refine families r, rq20, ra, rpq20U120 reading oracle-sealed base meshes; issue #68) = 180 runs; regenerated, manifest sha256-sealed. The opt-in xl class extends sizes for the MT axis. The standing bench timed set is m+l of all 12 families (120 timed runs, issue #68).
- Byte-parity gate: tools/oracle/run-harness.sh runs oracle and candidate over the corpus and compares outputs byte-for-byte; on divergence it reports case, option string, output file, first differing byte/line/column with context, precise enough to bisect against the strict function mapping. This gate is binding at every stage-1 PR, every idiomatization rung, and permanently, including after MT lands. The serial path is the port's only byte-contract forever.
- Performance gate: tools/oracle/run-bench.sh times the candidate against the pinned oracle, process-level wall time, interleaved oracle/candidate runs, 3 warmup + 25 timed per run, median per side. Timed set: sizes m and l of every case x family (80 runs). Binding on the build server: m+l total ratio 1.05, geometric mean 1.05, per-run tripwire 1.35 (derived from the measured +/-1.6% aggregate noise floor; re-derivable in one command against any new gate machine; the end-gate geomean axis is amended to the l-class - see the gate schedule below, issue #64).
- Miri and debug_asserts: stage 1 runs the corpus (subset) under Miri with debug_assert invariants on; this is rung-1 acceptance, not optional hygiene.

Gate invocation schedule (ticket #11, mapped to ADR 0001's sequence):

| Port phase | Byte-parity (run-harness.sh) | Performance gate (run-bench.sh) |
| --- | --- | --- |
| Stage 1, per PR | binding | advisory only |
| Stage 1 completion (R0: parity green) | binding, full corpus | record the baseline number (transliteration may legitimately sit above the band; R0 becomes the no-regression reference) |
| Idiomatization rungs 1 through 4, each boundary | binding | binding: the rung-boundary instrument is the back-to-back binary A/B vs the pinned oracle (old master binary vs rung-head binary, same box, same session, bench order old-then-new; gate = aggregate and geomean ratio deltas within +/-0.75pp, demonstrated resolution 0.012pp from the R3 verification pair). The oracle-settle protocol is retired (cx23 class evidence; the failed settle record lives in the r3-settle-* reports); the criterion corpus suite stays a diagnostic only, never a gate (11.34% per-bench same-binary wander on cx23); bit-identity is excluded as a gate form (panic locations embed file:line, so any line or path shift changes the hash). A justified slowdown still needs an explicit exception in the rung's ticket |
| End gate (after rung 4) | binding | binding absolute, amended (issue #64, maintainer decision, user sign-off): the verdict is PASS iff the aggregate total ratio <= 1.05 (over m+l), the geomean of per-run ratios <= 1.05 computed over the l-class rows only, and every per-run ratio <= 1.35 (all runs), vs the pinned oracle. Amendment rationale: the m-class per-run ratios measure a per-exec process constant (Rust runtime/std init; control-measured premium 0.39-0.62 ms/exec, of which ~0.34 ms is runtime/std init over the fork floor, per docs/perf-st-startup.md), which multiplies into short-run ratios without reflecting meshing performance; the m-class stays bounded additively by the aggregate and singly by the tripwire. Evidence, both binding benches, same box, own runs, #50 evidence rules; the R4 session report (r4-ab-new.json) is the bench of record, its candidate binary fd532a99... byte-identical to the master build: R4 session - aggregate 0.9709 (oracle sum of medians 0.7512 s vs candidate 0.7293 s), m-class geomean 1.0998 (0.2337 s vs 0.2440 s), l-class geomean 1.0353 (0.5174 s vs 0.4852 s), worst per-run 1.3413 at grid_l/p; ST session (st-endgate-lever1.json, cross-session replication; that bench ran on the patched branch head c898e67, whose effect was below the attribution floor) - aggregate 0.9680 (0.7370 s vs 0.7134 s), m-class geomean 1.1012 (0.2256 s vs 0.2370 s), l-class geomean 1.0324 (0.5115 s vs 0.4765 s), worst per-run 1.3116 at grid_m/p; cross-session stability: l-geomean spread 0.3pp, m-geomean spread 0.15pp. The rung-boundary no-regression instrument is unchanged (one-sided, candidate regression only) |
| MT milestone (on the CPX42) | binding, serial path unchanged | MT targets of section 5.5 plus the ST end gate re-run on that machine |

## 4. Port strategy (ADR 0001)

Hybrid, decided and user-confirmed:

- The core algorithms are transliterated function-for-function: C names kept, call order and control flow identical, the strict mapping recorded per function in docs/function-mapping.md, every deliberate departure in the deviation register (D1-D11 plus the entries #9 and #12 assign at their implementations). Side-by-side debugging against C stays possible throughout stage 1.
- The API shell (options, Result errors, entry points) is idiomatic Rust from day one (ADR 0004 section 5).
- After parity, the core is idiomatized in four corpus-gated rungs: R1 accessor invariants, R2 error flow, R3 storage typing (typed rows through the chokepoint seam; includes the SoA rematch trigger from ADR 0002 if #11 profiling demands it), R4 module reorganization and idiomatically renames internals.
- Serial byte-identity stays a permanent regression gate under all of it, including multithreading.

## 5. Core architecture

### 5.1 Robust predicates - predicates.rs

1:1 pure Rust port of predicates.c. The inert x87 control-word code is dropped on 64-bit targets (deviation D5); the SSE2 double math actually used is unaffected. Exact-arithmetic behavior is identical; the byte-parity corpus over all families (including grid degeneracies) is the gate. The georust robust crate served as the cross-check oracle in research (#3); it is a research artifact, not a dependency.

### 5.2 Data model - ADR 0002

- Every stored handle is a packed u64: 32-bit pool row index in the high bits, orientation in the low 2 bits (triangles) or 1 bit (subsegments); the all-zero word keeps the C NULL meaning. Exact C slot density: 8 bytes per neighbor slot, 48-byte triangle rows.
- Register handles Otri { tri: u32, orient: u8 } and Osub { ss: u32, orient: u8 } are the unpacked C mirrors (struct otri/osub); ring rotations rotate only the orientation byte, encode/decode run only at slot stores and loads with identical semantics to the C macros (deviation D6: macros become methods).
- Triangle, subsegment and vertex rows are untyped u64 word arrays with runtime stride and slot roles fixed at init by the transliterated C formulas; vertex int fields (mark, type) are exact 4-byte half-word overlays through the accessors (deviation D2, no aliasing UB); with -p the vertex2tri word holds an encoded Otri exactly as C stores it.
- One private accessor chokepoint (mesh/slots.rs) is the only code touching pool storage: values in and out, no storage references escape, bounds-checked indexing, bit-exact conversions. Everything outside it is safe by construction.
- Sentinels (dummytri/dummysub) become reserved fixed row ids, excluded from allocation, traversal and the dead stack (deviation D3). The dead stack threads u32 row ids through slot 0 (deviation D4); dead-detection keeps the C killtri NULL pattern.
- Stage 1 unsafe: zero, enforced mechanically (unsafe_code = "forbid" workspace-wide). Invariants held by debug_asserts; the corpus runs under Miri. Any later unsafe is chokepoint-local and rung-gated (deviation D7; the single currently foreseen pocket is section 5.5).

### 5.3 Memory management - ADR 0003

- One uniform WordPool type serves all eight mesh pools and the five small pools (viri, badsubsegs, badtriangles, flipstackers, splaynodes): a flat Vec<u64> per pool, stride-fixed rows, Copy u32 row ids (0 null, 1 sentinel row for tri/subseg pools), exact-LIFO dead stack threaded through slot 0, allocation-order traversal covering 0..high-water including dead rows, poolrestart with retained capacity. Byte identity depends on this order contract; it is non-negotiable.
- Growth: amortized Vec doubling (a deliberate deviation from C's fixed-block steps; on a flat store C-step growth aggregates O(N^1.5) copying). Initial capacities mirror C's first-block hints, memory parity only. OOM routes through try_reserve to the error funnel's Error::OutOfMemory.
- The aCute pool stays eight typed Vec<f64> scratch buffers, only-grow, zero-filled resize (deviation-register entry).
- The C raw-pointer ORDER comparisons (edge/neighbor/eps output paths) are mirrored arithmetically as oracle_address_order (blocks descending, rows ascending within a block; glibc-verified; the pen corpus family is the canary).

### 5.4 Crate, API and error architecture - ADR 0004

- Workspace: rust/ with crates/triangle (lib), crates/triangle-io (the interchange crate: the six Triangle file formats .poly/.node/.ele/.edge/.neigh/.area over neutral, self-contained types - parsing and serialization, pure std, zero dependencies, zero unsafe, never a dependency of the meshing core; extracted by issue #70, ADR 0006, version 0.1.0 with publish = false pending the maintainer's publication/naming decision, and the triangle crate's only dependency) and crates/triangle-cli (bin, executable triangle-cli, the #13 candidate contract). The C sources and tools/oracle are untouched. 64-bit enforcement in lib.rs; rust/Cargo.lock committed; rust/target ignored.
- Feature gates: exactly one, acute (default on), the inverted NO_ACUTE switch; --no-default-features reproduces a C NO_ACUTE build and rejects -U exactly as C does.
- Module map (the full 195-row assignment is docs/function-mapping.md): triangle.h to mesh/handles.rs + mesh/slots.rs; triangle.c splits along its section banners into mesh/util.rs, options.rs, mesh/debug.rs, mesh/pool.rs, mesh/mod.rs, algorithm/quality.rs, algorithm/locate.rs, algorithm/transform.rs, algorithm/delaunay.rs, algorithm/segments.rs, algorithm/refine.rs, io/arrays.rs; acute.c to acute.rs (feature-gated); predicates.c to predicates.rs; eps_writer.c to io/eps.rs; triangle_io.c's writers to io/mod.rs and its readers plus the format emission to the triangle-io crate (I/O phase; the issue #70 extraction, ADR 0006 - format ownership: the six interchange formats live in triangle-io, the Mesh-bound writer orchestration and api.rs seam conversion stay in the triangle crate); triangle_helper.c and triangle_api.c dissolve into options.rs, input.rs, api.rs, error.rs and lib.rs. After the R4 reorganization (issue #60) the private core is grouped along its seams - mesh/ (the storage model: mod.rs, handles.rs, slots.rs, pool.rs, debug.rs, util.rs), algorithm/ (the triangle.c algorithm sections: locate.rs, transform.rs, delaunay.rs, segments.rs, quality.rs, refine.rs) and io/ (mod.rs, arrays.rs, eps.rs) - while the API shell and predicates.rs stay top-level and formatter.rs keeps its public path at the src root (ADR 0004).
- Public surface day one: Options (the struct behavior mirror; Options::new defaults, Options::parse of the C options string 1:1, validate computing the derived-field tail; no options invented, none dropped), triangulate(), the owned Mesh handle (load_mesh, refine, output, statistics, quality, check_mesh, check_delaunay, memory_bytes), MeshInput/MeshOutput (owned Vecs, no libc types), Statistics/Quality/BoundingBox (exact C arithmetic), the Unsuitable trait (object-safe, one method, receives copied VertexSample values, deviation D9), VERSION (1, 7, 0) and HAS_ACUTE. The crate never exposes otri/osub, row ids, pools or chokepoint types.
- Error funnel: internal status codes mirrored 1:1 as InternalStatus with the exact C discriminants until rung 2; the exit sites become returned statuses (internalerror, OOM; deviation D8); TRI_NULL dissolves into ownership. One translation point From<InternalStatus> for Error at the api.rs boundary. Public taxonomy: Error { Options, Input, SegmentRecovery(kind: FindDirection/Split/Intersect/Insert/Scout/Cycle), OutOfMemory, Internal, Io }. The lib never prints and never exits; C's messages survive verbatim as error payloads.
- Lifecycle: RAII, Drop, no triangle_free (deviation D11); the LCG seed is an owned per-run field, not thread-local (deviation D10).
- Options::threads(usize) (default 1; validated 1..=1024) is the lib-side MT knob; the CLI reads TRIANGLE_THREADS (exact spelling) today.

### 5.5 Parallelism - ADR 0005

Lands as its own milestone after the ST end gate; entry condition is a parity-green core.

- Determinism ruling: freer scheduling on the parallel path, validated structurally; the serial path remains the only byte-contract. Parallel-invariant families (plain, p) must match the serial run exactly; refinement families (q20, pq20, pq20U120, pa, pq20a, pen with -q) are valid-within-band (provisional +/-5% on counts, calibrated at the first MT run); every variant must pass check_mesh/check_delaunay with zero failures. Bit-stable parallel refinement was rejected: it negates the scaling in the 69% phase.
- Phase inventory (first MT milestone): IN: divide-and-conquer construction (fork-join over the deterministic recursion tree, shard merge with parallel id rebase), quality and aCute refinement into claim queues (the core target, ~69% of the measured aggregate), output serialization (~25%; serial numbering pass, then per-range marshalling with per-worker byte buffers concatenated in id order). OUT (phase-serial sandwich, no error): parse, incremental and sweepline construction, segment insertion, hole carving, statistics.
- Primitives: std::thread::scope with explicit per-phase coordination; no rayon, no custom work-stealing. Threads spawn per run, exactly T, so pinning and wall-time measurement stay honest.
- Pools: per-worker shard overlay above unchanged WordPools; the join merges shard rows with an arithmetic id rebase using the packed encoding. The one consciously gated unsafe pocket of the port: an UnsafeCell-backed store wrapper inside the slots/pool chokepoint for parallel-phase row mutation under the claim discipline (module-local, deviation-registered at implementation, claim-asserted in debug, Miri-covered on the MT corpus subset). The serial path bypasses the wrapper entirely.
- Queues: badtriangles becomes a concurrent claim queue (mutex per size bucket over C's intrusive linkage, worst-bucket-first pops); badsubsegs one mutex-guarded FIFO; flipstackers and viri stay serial. Conflict resolution by star claims on a claim sidecar (per-row atomic claim flags, all-or-nothing, row-id order, stale items released and re-enqueued). Edge locks, slab partitioning and serial-apply two-phase were rejected; slab partitioning is the recorded fallback on conflict-telemetry.
- RNG: derived worker streams (same LCG constants), keyed on the subtree identity for construction pivots and on the worker for location samples; the serial path keeps the single context-seeded sequence untouched.
- ADR 0004 hand-offs resolved: Unsuitable storage becomes Box<dyn Unsuitable + Send + Sync>, invoked per worker concurrently; Mesh is Send + Sync after a run, reentrant across independent contexts, concurrent mutating calls on one Mesh unsupported; TRIANGLE_THREADS maps to Options::threads.
- MT gate (on the user-provided Hetzner CPX42, 8 dedicated AMD EPYC vCPUs, T in {1, 2, 4, 8}): binding statistic is the xl medians (sum-of-medians aggregate plus geomean of per-family efficiency), l-class advisory on that machine's measured bimodality; aggregate targets >= 1.8x at T=2, >= 3.0x at T=4, >= 4.5x at T=8 (efficiency vs the T=1 Rust baseline, never the C oracle); per-family floor: every timed family with a T=1 median >= 100 ms must reach >= 60% of the aggregate target at the same T; structural validation of every variant per the split above; one rerun allowed, binding on the rerun.

## 6. Implementation plan

Stage 1 to R0 (transliteration to full byte parity), landing per module with docs/function-mapping.md as the shared assignment and the harness binding per PR:

1. Foundation: mesh/handles.rs, mesh/slots.rs (the chokepoint), mesh/pool.rs (ADR 0003's WordPool), mesh/mod.rs. Independently landable base for everything else.
2. Shell, early for parity visibility: options.rs + error.rs + input.rs + lib.rs (parsecommandline and validate are the cheapest full-family behavioral parity tests), api.rs as the surface lands.
3. predicates.rs (independent; gates the geometry).
4. Algorithm core in C call order: mesh/util.rs, algorithm/delaunay.rs, algorithm/locate.rs, algorithm/transform.rs, algorithm/segments.rs, algorithm/quality.rs, algorithm/refine.rs, acute.rs.
5. io/arrays.rs (transfernodes early with the input path; array writers late, they serve Mesh::output).
6. Hermetic per-PR acceptance: run-harness.sh --candidate <rust-cli> on the committed subset green; a thin binary entry arrives with the first ioarrays capabilities (the candidate contract activates fully in the I/O phase).

R0 completion: full 120-run corpus byte-identical, Miri subset green with debug asserts on, R0 performance baseline recorded.

Rungs R1-R4 per ADR 0001 (each boundary: corpus green, no-regression vs the best recorded milestone, exceptions recorded in the rung's ticket). R3 owns the typed-rows swap and the SoA rematch decision. End gate: absolute ST parity binding.

MT milestone per ADR 0005's cut: (a) scoped-phase scheduler + shard overlay + store wrapper, (b) construction fork-join with deterministic rebase, (c) claim queues + sidecar + refinement integration, (d) per-worker aCute scratch + worker-stream RNG, (e) output marshalling parallel body, (f) MT validation/telemetry and the CPX42 gate run.

I/O phase (ticketed when the core API shape stands): .poly/node/ele readers, writers matching the oracle's byte formats exactly (%.17g), io/eps.rs, io/mod.rs; the CLI argument compatibility surface and file-name conventions get pinned there, including whether a --threads flag joins TRIANGLE_THREADS.

CLI phase: triangle-cli graduates from stub to the full C-compatible surface; exit codes 0/1; stdout chatter parity is pinned there.

## 7. Risks and their standing counters

- Idiomatization stall (temporary code lives longest): explicit rung tickets, the permanent oracle gate (ADR 0001).
- API freeze violations after stage 1: the section 5.4 surface is the v0 line; changes require their own ticket against the map.
- CPX42 per-exec bimodality (~2.2x modes on short runs): xl binding, deeper l-tier repeats (ADR 0005).
- The unsafe store wrapper: bounded to one chokepoint pocket, claim-asserted, Miri-covered, deviation-registered; any growth re-runs its own ticket.
- Refinement-family structural band (provisional +/-5%): calibrated and recorded at the first MT milestone run; the calibration is a gate artifact, not an opinion.

## 8. Open items (deliberately unpinned)

- Numeric refinement-family band beyond the provisional +/-5% (first MT milestone).
- MT beyond the milestone-1 inventory (segment insertion, carving, -i/-F): no commitment made or foreclosed.
- File I/O crate shape: decided for the interchange formats (the triangle-io crate, issue #70, ADR 0006). CLI argument surface, file-name conventions and the --threads flag remain open (CLI phase tickets).
- crates.io publication and crate naming (both crates; triangle-io is 0.1.0 and publish = false pending this decision).
- Work-stealing or lock-free queue interiors (telemetry-gated, contained seams).

## 9. Sign-off

User decisions already confirmed and binding: the hybrid port strategy (#6), the representation ruling (no tagged raw pointers, 2026-11-24), internal MT in scope with MT scaling binding (2026-11-24), the differential-harness-first strategy, and the MT hardware (Hetzner CPX42).

Sign-off recorded: PR #20 merged 2026-11-24; wayfinder map issue #1 is closed as completed. The stage-1 implementation tickets are cut next against docs/function-mapping.md with tools/oracle/run-harness.sh binding from their first PR.
