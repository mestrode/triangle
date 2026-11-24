# Rust landscape: building blocks and references for the Triangle port

Findings for wayfinder ticket #2. Date: 2026-11-23. Sources opened and checked during research.

## 1. Delaunay crates: what exists, what transfers

- spade (github.com/Stoeoef/spade), v2, actively maintained: incremental and bulk Delaunay/CDT, walk-based point location with hierarchy hints, a refinement config (Ruppert-style quality), interpolation, nearest-neighbor. The closest Rust neighbor feature-wise. It does NOT cover Triangle's scope: no conforming Delaunay from PSLGs with holes/regions/area constraints, no aCute, no Voronoi output, different data structures and different quality logic. Transfers: point-location walk machinery ideas, robustness patterns, and its delaunay_compare benchmark methodology.
- delaunator-rs (port of Mapbox Delaunator, crates.io/delaunator): static bulk triangulation only, no constraints, no refinement; built for raw speed (at launch it measured roughly an order of magnitude faster than old spade bulk on 1M points, per its author on r/rust). Transfers nothing structural for this port; it is the strongest public evidence that Rust geometric hot loops reach and keep C-level speeds.
- cdt crate: constrained Delaunay bulk loading. spade's delaunay_compare on a real 2.9M-vertex / 2.8M-edge Europe coastline CDT: spade 3668 ms, cdt 4032 ms with constraints. Shows criterion-based corpus benchmarking is the established methodology; transfers to the perf gate (#11).
- Conclusion: no existing crate covers Triangle's feature scope (constrained + conforming Delaunay + aCute quality refinement + Voronoi/neighbor outputs). The port is justified; these crates are references, not replacements.

## 2. Predicate crates (details in ticket #3)

- georust/robust (v1.2.0, May 2025, maintained): pure Rust port of Shewchuk's predicates.c.
- robust-predicates crate (2022): links the ORIGINAL predicates.c compiled via cc; a foreign-c-code bridge, not a Rust port.
- mourner/robust-predicates (JS, referenced design): port that dropped FP-environment setup entirely and is still used in production (Delaunator), evidence the predicates do not need x87 control-word tricks on strict IEEE-double platforms.

## 3. Allocation and arena landscape, fit against Triangle's pools

- bumpalo: bump arena, ~11 fast-path instructions per alloc, chunked; NO per-item free (deallocation is a no-op; Drop not run). collections feature provides arena-backed Vec/String.
- typed-arena: typed bump, same no-reclaim model.
- bump-scope: newer arena claiming roughly 2x bumpalo in wasm benchmarks.
- slotmap / slab / thunderdome: Vec + (versioned) indices with slot reuse, safe; slotmap author describes it as essentially the Vec-plus-indices solution with memory reuse and version checks.
- Fit analysis (this is the important part for #9): Triangle's core DELETES items constantly (triangle deletion during Delaunay/refinement flips, vertex jettisoning in quality mode) and its output ORDER is the pools' allocation order (freelist reuse included). Pure bump arenas never reclaim a slot: they cannot reproduce the C pool's memory behavior and would change or even break output ordering. Ready-made slotmap adds generation bits beyond what the algorithm needs. The two viable routes: (a) a literal custom pool: block list + freelist + first-slot threading, transliteration-exact; (b) Vec-with-free-index-list: same reuse semantics, indices instead of addresses, orthogonal to the tagged-pointer question (#7/#8).
- Supporting references: r/rust arena comparison thread (2024), vorner's blog on bumpalo fast-path cost, submillisecond arena cookbook, "Object Soup is Made of Indexes" (Lobsters) and "Indices, not Pointers" (HN) on index-vs-pointer engineering tradeoffs.

## 4. Tagged-handle patterns in the wild

- rust-hosted-langs book, tagged-pointers chapter: the canonical pattern of packing tags into alignment-guaranteed low pointer bits.
- Niche optimization background: Option<NonNull>/NonZero give 8-byte optional pointers; a (ptr, orient) struct without packing costs 16 bytes and doubles handle bandwidth.
- Slot/handle designs pack (32-bit index, extra tag bits) into a single u64: functionally the same packing as the tagged pointer, applied to indices.

## 5. Inputs derived for the follow-up tickets

- #6 (strategy): strong precedent that faithful Rust ports sustain C-level geometric performance; no public evidence that raw tagged pointers are REQUIRED for parity; equally none that they are not. The microbench (#7) must decide on data.
- #7/#8 (representation): the alignment-based packing pattern is idiomatic-enough in Rust (unsafe encapsulated behind a handle type); the index-packing alternative exists with the same bandwidth characteristics.
- #9 (memory): bump arenas are ruled out for the transliteration path (no reclamation); custom pool-with-freelist or Vec-plus-free-list are the two candidates; block-linear traversal must be preserved for output ordering.
- #11 (perf gate): spade's delaunay_compare (criterion, distributions: uniform + local-walk insertion) is the model for the benchmark corpus design.