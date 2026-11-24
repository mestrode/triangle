# Differential C oracle: determinism and comparability of mesh outputs

Findings for wayfinder ticket #5. Date: 2026-11-23. Base: master @ 451c932.

## 1. Is the C version deterministic? Yes, at source level

- No time(), clock(), getpid(), or getenv() anywhere in the core (grep over triangle.c, acute.c, triangle_io.c, triangle_api.c: zero hits).
- RNG: thread-local LCG seeded to 1 per context (triangle.c:1502, 1521-1525). Largest intermediate 714025 * 1366 + 150889 is under 2^30, so the sequence is identical across platforms, word widths, and runs. Contexts are independent of each other.
- Practical caveat: run-to-run identical, yes. C-build vs Rust-build identical only under pinned FP flags, below.

## 2. Floating-point: where C and Rust can diverge (the real risk)

- GCC/Clang default to -ffp-contract=fast (FMA contraction of a*b+c). Rust/LLVM does not contract by default. If the C oracle build enables FMA (e.g., -march=native on FMA-capable hardware, or -mfma), rounding of predicates/circumcenters can differ from Rust, and mesh paths diverge.
- Default x86-64 codegen uses SSE2 with no FMA, so plain gcc -O2/-O3 without -march=native emits no fused ops; Rust default x86-64 codegen likewise SSE2. With -ffp-contract=off pinned on the C side as an extra guarantee, C and Rust f64 arithmetic is bit-identical (IEEE 754, no reassociation, no fast-math anywhere by default).
- Recommendation for the oracle and the perf gate: build C with -O2 or -O3, -ffp-contract=off, no -march overrides; build Rust --release default flags. Then byte-identical outputs are a realistic target for a faithful port.
- aarch64: same pinning logic (GCC -ffp-contract default is fast there too).

## 3. Output ordering: allocation order, fully reproducible

- triangle_io.c enumerates outputs via traversalinit/traverse over each pool (triangle_io.c:45, 94, 153, 219, 278, 289; pool traversal in triangle.c:940-990): items appear in allocation order, freed items skipped. Numbering runs from b->firstnumber upward in traversal order (node-writer shown in triangle_io.c writing loop; mesh-copy paths follow the same traversal machinery).
- The freelist (deaditemstack, first-slot threading) has deterministic reuse order too, so allocation order is a pure function of the algorithm's operation sequence.
- Byte-identical outputs therefore require: same insertion/deallocation sequence (a transliteration property), bit-identical FP (2 above), identical RNG stream (integer-only, trivial), and identical sort/comparison behavior including equal-key handling (a faithful port inherits identical comparators).

## 4. What triangle-test covers today

tests.c: 10 smoke tests (version, context create/destroy, behavior parse, mesh create with count invariants, mesh copy with counts, statistics, quality, refine, load+refine). Counts and flags only: no geometry verification, no differential comparison, no large inputs, no determinism assertions. The oracle harness is greenfield work.

## 5. File-based oracle is also safe

The file writers use %.17g (triangle_io.c), which round-trips binary64 exactly and yields deterministic digits for a given value across libcs, so CLI-level corpus comparison (outputs as .node/.ele/.poly files) is a valid oracle route alongside in-memory triangleio comparison.

## 6. Recommendation (input for the spec's test strategy)

- Oracle route: corpus of generated polys (regular grids, random point sets, PSLGs with holes/segments, several size classes) x option families (plain Delaunay, -p, quality -q with/without aCute, area constraints). C CLI vs Rust CLI, compare output files byte-for-byte; any mismatch is a port bug.
- The corpus doubles as the benchmark corpus for the performance gate (ticket #11).
- Structural invariants (Delaunay property, topology consistency, angle bounds) are added for the later idiomatic phases where byte-identity may be dropped intentionally.