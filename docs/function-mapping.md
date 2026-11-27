# Function mapping: C core to Rust modules

The module map of the Rust port, seeded by the crate and API architecture
decision ([ADR 0004](adr/0004-crate-api-architecture.md), wayfinder ticket
#10). Module assignment is decided there; the stage-1 implementation tickets
fill the table completely and keep it current as functions land (a landed row
notes its PR). No published Rust implementation exists yet (empty skeleton
under `rust/`).

Layout: C compilation units mirror to Rust modules in the lib crate
`rust/crates/triangle`; `triangle.c` splits along its own section banners.
The transliterated core is private; the public surface is the API shell
(`api.rs`, `options.rs`, `input.rs`, `error.rs`, `lib.rs` re-exports).

Stage column: **S1** = stage-1 core transliteration; **S1 shell** = day-one
idiomatic API shell; **IO** = the later I/O phase; **-** = no Rust
counterpart (the C construct dissolves; see the deviation register).
Rung notes follow [ADR 0001](adr/0001-port-strategy.md): C function names
survived transliteration verbatim through stage 1 and rungs 1-3; rung 4
(issue #60) idiomatized the crate-internal names. The symbol column
carries the R4 names, and the R4 rename record below keeps every
stage-1 name greppable from its C anchor. Deviations are cross-referenced as D-numbers in
[deviation-register.md](deviation-register.md).

Status of this seed: every C function of every compilation unit of the port's
core (`src/Triangle/*.c`) has a row (195 functions) plus the macro/type
pseudo-rows below. Nothing is unassigned.

## Per-compilation-unit overview

| C unit (`src/Triangle/`) | Rust module(s) in `rust/crates/triangle/src/` | Stage |
| --- | --- | --- |
| `triangle.h` (types, macros, encode/decode) | `mesh/handles.rs`, `mesh/slots.rs`, `error.rs`, `options.rs` | S1 |
| `triangle.c` | `mesh/util.rs`, `options.rs`, `error.rs`, `mesh/debug.rs`, `mesh/pool.rs`, `mesh/mod.rs`, `algorithm/quality.rs`, `algorithm/locate.rs`, `algorithm/transform.rs`, `algorithm/delaunay.rs`, `algorithm/segments.rs`, `algorithm/refine.rs`, `io/arrays.rs` | S1 |
| `acute.c` | `acute.rs` (behind feature `acute`, default on) | S1 |
| `predicates.c` | `predicates.rs` | S1 |
| `eps_writer.c` | `io/eps.rs` | IO |
| `triangle_helper.c` | `options.rs`, `input.rs` (unit dissolves) | S1 shell |
| `triangle_io.c` | `io/mod.rs` | IO |
| `triangle_api.c` | `api.rs`, `options.rs`, `error.rs`, `lib.rs`, `io/mod.rs` (context lifecycle dissolves) | S1 shell / IO |
| `triangle_config.h` (TRI_* codes) | `error.rs` (public Error taxonomy; the raising sites construct the variants in place since rung 2, D8) | S1 shell |
| `triangle_version.h.in` | `lib.rs` (VERSION, HAS_ACUTE) | S1 shell |
| `src/examples/triangle-cli` | `rust/crates/triangle-cli` (the real main, landed in the I/O-phase PR (#46)) | IO |


R4 (issue #60): the module column carries the post-R4 locations (module
reorganization) and the symbol column the post-R4 names; the stage-1
flat layout and names live in the git history and in the R4 rename
record below.

## Function table

Function names and C sites are extracted from the C sources; `file:line`
refers to the source as of this seed commit (line numbers drift as the C
side changes; the function names are the stable key).


### `triangle.c` (src/Triangle/triangle.c)

| C function | C site | Rust module | Rust symbol (R4) | Stage | Notes |
| --- | --- | --- | --- | --- | --- |
| `interpolate` | triangle.c:238 | `mesh/util.rs` | same name (snake_case) | S1 | landed in PR #30 |
| `behavior_update` | triangle.c:275 | `options.rs` | `update_derived_fields` | S1 shell | Options::defaults and Options::parse; derived-field tail transliterated bit-exact |
| `triexit` | triangle.c:313 | `error.rs` | same name (snake_case) | S1 shell | no process exit, no manual alloc/free (D8, D11); the pool's Oom converts through From<Oom> for Error to Error::OutOfMemory at the call sites (rung 2) |
| `trimalloc` | triangle.c:318 | `error.rs` | same name (snake_case) | S1 shell | no process exit, no manual alloc/free (D8, D11); the pool's Oom converts through From<Oom> for Error to Error::OutOfMemory at the call sites (rung 2) |
| `trifree` | triangle.c:330 | `error.rs` | same name (snake_case) | S1 shell | no process exit, no manual alloc/free (D8, D11); the pool's Oom converts through From<Oom> for Error to Error::OutOfMemory at the call sites (rung 2) |
| `internalerror` | triangle.c:349 | `error.rs` | same name (snake_case) | S1 shell | no process exit, no manual alloc/free (D8, D11); the payload rides Error::Internal verbatim, raised in place at the conformingedge site (rung 2) |
| `parsecommandline` | triangle.c:364 | `options.rs` | same name (snake_case) | S1 shell | Options::defaults and Options::parse; derived-field tail transliterated bit-exact |
| `printtriangle` | triangle.c:574 | `mesh/debug.rs` | `print_triangle` | S1 | |
| `printsubseg` | triangle.c:660 | `mesh/debug.rs` | `print_subsegment` | S1 | |
| `poolzero` | triangle.c:746 | `mesh/pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): dissolves into WordPool construction/Drop (D11, D4); landed in PR #23 |
| `poolrestart` | triangle.c:774 | `mesh/pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): restart; landed in PR #23 |
| `poolinit` | triangle.c:814 | `mesh/pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): WordPool::new; landed in PR #23 |
| `pooldeinit` | triangle.c:852 | `mesh/pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): dissolves into WordPool construction/Drop (D11, D4); landed in PR #23 |
| `poolalloc` | triangle.c:867 | `mesh/pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): alloc; landed in PR #23 |
| `pooldealloc` | triangle.c:924 | `mesh/pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): dealloc; landed in PR #23 |
| `traversalinit` | triangle.c:940 | `mesh/pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): traversal; landed in PR #23 |
| `traverse` | triangle.c:970 | `mesh/pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): traverse_next; landed in PR #23 |
| `dummyinit` | triangle.c:1029 | `mesh/mod.rs` | `init_sentinel_rows` | S1 | D3 sentinel rows here (dummyinit) |
| `initializevertexpool` | triangle.c:1100 | `mesh/mod.rs` | `init_vertex_pool` | S1 | D3 sentinel rows here (dummyinit) |
| `initializetrisubpools` | triangle.c:1136 | `mesh/mod.rs` | `init_tri_and_subseg_pools` | S1 | D3 sentinel rows here (dummyinit) |
| `triangledealloc` | triangle.c:1196 | `mesh/mod.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `triangletraverse` | triangle.c:1210 | `mesh/mod.rs` | `traverse_triangle` | S1 | D3 sentinel rows here (dummyinit); landed in PR #35 (the reconstruct consumer) |
| `subsegdealloc` | triangle.c:1229 | `mesh/mod.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `subsegtraverse` | triangle.c:1243 | `mesh/mod.rs` | `traverse_subseg` | S1 | D3 sentinel rows here (dummyinit); landed in PR #35 (the reconstruct consumer) |
| `vertexdealloc` | triangle.c:1262 | `mesh/mod.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `vertextraverse` | triangle.c:1276 | `mesh/mod.rs` | `traverse_vertex` | S1 | D3 sentinel rows here (dummyinit); vertextraverse landed in PR #33 (the divconqdelaunay consumer) |
| `badsubsegdealloc` | triangle.c:1298 | `mesh/mod.rs` | `dealloc_bad_subseg` | S1 | D3 sentinel rows here (dummyinit); landed in the refine PR (#39), the splitencsegs consumer |
| `badsubsegtraverse` | triangle.c:1316 | `mesh/mod.rs` | `traverse_bad_subseg` | S1 | D3 sentinel rows here (dummyinit); landed in the refine PR (#39), the splitencsegs consumer |
| `getvertex` | triangle.c:1343 | `mesh/mod.rs` | `get_vertex` | S1 | D3 sentinel rows here (dummyinit); landed in PR #35 (the reconstruct consumer) |
| `triangledeinit` | triangle.c:1376 | `mesh/mod.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit); the acute free (trifree(m->acute_mem) and the NULL store, triangle.c:1395-1396) dissolves into the Drop of the mesh-owned acute_mem field (D23, the stage-1 acute B PR #41) |
| `maketriangle` | triangle.c:1414 | `mesh/mod.rs` | `make_triangle` | S1 | D3 sentinel rows here (dummyinit) |
| `makesubseg` | triangle.c:1450 | `mesh/mod.rs` | `make_subseg` | S1 | D3 sentinel rows here (dummyinit) |
| `triangleinit` | triangle.c:1482 | `mesh/mod.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `randomnation` | triangle.c:1521 | `mesh/util.rs` | `random_below` | S1 | seed moves from _Thread_local to a per-run owned field (D10); landed in PR #30 |
| `checkmesh` | triangle.c:1537 | `algorithm/quality.rs` | `check_mesh` | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion; landed in PR #36 (with the #28 restoration of the algorithm/transform.rs call sites, the bad-queue fields on mesh/mod.rs, the badtriang accessors in mesh/slots.rs, and the api.rs statistics/quality/check wiring) |
| `checkdelaunay` | triangle.c:1622 | `algorithm/quality.rs` | `check_delaunay` | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion; landed in PR #36 (with the #28 restoration of the algorithm/transform.rs call sites, the bad-queue fields on mesh/mod.rs, the badtriang accessors in mesh/slots.rs, and the api.rs statistics/quality/check wiring) |
| `enqueuebadtriang` | triangle.c:1714 | `algorithm/quality.rs` | `enqueue_bad_triangle_row` | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion; landed in PR #36 (with the #28 restoration of the algorithm/transform.rs call sites, the bad-queue fields on mesh/mod.rs, the badtriang accessors in mesh/slots.rs, and the api.rs statistics/quality/check wiring) |
| `enqueuebadtri` | triangle.c:1810 | `algorithm/quality.rs` | `enqueue_bad_triangle` | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion; landed in PR #36 (with the #28 restoration of the algorithm/transform.rs call sites, the bad-queue fields on mesh/mod.rs, the badtriang accessors in mesh/slots.rs, and the api.rs statistics/quality/check wiring) |
| `dequeuebadtriang` | triangle.c:1835 | `algorithm/quality.rs` | `dequeue_bad_triangle` | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion; landed in PR #36 (with the #28 restoration of the algorithm/transform.rs call sites, the bad-queue fields on mesh/mod.rs, the badtriang accessors in mesh/slots.rs, and the api.rs statistics/quality/check wiring) |
| `inlens` | triangle.c:1878 | `algorithm/quality.rs` | `in_lens` | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion; landed in PR #36 (with the #28 restoration of the algorithm/transform.rs call sites, the bad-queue fields on mesh/mod.rs, the badtriang accessors in mesh/slots.rs, and the api.rs statistics/quality/check wiring) |
| `checkseg4encroach` | triangle.c:1925 | `algorithm/quality.rs` | `check_seg_for_encroachment` | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion; landed in PR #36 (with the #28 restoration of the algorithm/transform.rs call sites, the bad-queue fields on mesh/mod.rs, the badtriang accessors in mesh/slots.rs, and the api.rs statistics/quality/check wiring) |
| `testtriangle` | triangle.c:2001 | `algorithm/quality.rs` | `test_triangle` | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion; landed in PR #36 (with the #28 restoration of the algorithm/transform.rs call sites, the bad-queue fields on mesh/mod.rs, the badtriang accessors in mesh/slots.rs, and the api.rs statistics/quality/check wiring) |
| `makevertexmap` | triangle.c:2206 | `algorithm/quality.rs` | `make_vertex_map` | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion; landed in PR #35 (with the formskeleton consumer); the rest of the module landed in PR #36 |
| `preciselocate` | triangle.c:2291 | `algorithm/locate.rs` | `precise_locate` | S1 | landed in PR #30 |
| `locate` | triangle.c:2427 | `algorithm/locate.rs` | same name (snake_case) | S1 | landed in PR #30 |
| `insertsubseg` | triangle.c:2573 | `algorithm/transform.rs` | `insert_subseg` | S1 | landed in PR #30 |
| `flip` | triangle.c:2664 | `algorithm/transform.rs` | same name (snake_case) | S1 | landed in PR #30 |
| `unflip` | triangle.c:2785 | `algorithm/transform.rs` | same name (snake_case) | S1 | landed in PR #30 |
| `insertvertex` | triangle.c:2920 | `algorithm/transform.rs` | `insert_vertex` | S1 | landed in PR #30 |
| `triangulatepolygon` | triangle.c:3559 | `algorithm/transform.rs` | `triangulate_polygon` | S1 | landed in PR #30 |
| `deletevertex` | triangle.c:3638 | `algorithm/transform.rs` | `delete_vertex` | S1 | landed in PR #30 |
| `undovertex` | triangle.c:3730 | `algorithm/transform.rs` | `undo_vertex` | S1 | landed in PR #30 |
| `vertexsort` | triangle.c:3870 | `algorithm/delaunay.rs` | `vertex_sort` | S1 | landed in PR #33 |
| `vertexmedian` | triangle.c:3937 | `algorithm/delaunay.rs` | `vertex_median` | S1 | landed in PR #33 |
| `alternateaxes` | triangle.c:4006 | `algorithm/delaunay.rs` | `alternate_axes` | S1 | landed in PR #33 |
| `mergehulls` | triangle.c:4062 | `algorithm/delaunay.rs` | `merge_hulls` | S1 | landed in PR #33 |
| `divconqrecurse` | triangle.c:4366 | `algorithm/delaunay.rs` | `divconq_recurse` | S1 | landed in PR #33 |
| `removeghosts` | triangle.c:4495 | `algorithm/delaunay.rs` | `remove_ghosts` | S1 | landed in PR #33 |
| `divconqdelaunay` | triangle.c:4547 | `algorithm/delaunay.rs` | `divconq_delaunay` | S1 | landed in PR #33 |
| `boundingbox` | triangle.c:4620 | `algorithm/delaunay.rs` | `bounding_box` | S1 | landed in PR #33 (out-of-pool infvertices per D12) |
| `removebox` | triangle.c:4672 | `algorithm/delaunay.rs` | `remove_box` | S1 | landed in PR #33 (out-of-pool infvertices per D12) |
| `incrementaldelaunay` | triangle.c:4761 | `algorithm/delaunay.rs` | `incremental_delaunay` | S1 | landed in PR #34 |
| `eventheapinsert` | triangle.c:4800 | `algorithm/delaunay.rs` | `event_heap_insert` | S1 | landed in PR #34 |
| `eventheapify` | triangle.c:4833 | `algorithm/delaunay.rs` | `event_heapify` | S1 | landed in PR #34 |
| `eventheapdelete` | triangle.c:4881 | `algorithm/delaunay.rs` | `event_heap_delete` | S1 | landed in PR #34 |
| `createeventheap` | triangle.c:4916 | `algorithm/delaunay.rs` | `create_event_heap` | S1 | landed in PR #34 (event arena transliterates as an owned Vec + `EventPtr` free list) |
| `rightofhyperbola` | triangle.c:4946 | `algorithm/delaunay.rs` | `right_of_hyperbola` | S1 | landed in PR #34 (carries `hyperbolacount`) |
| `circletop` | triangle.c:4977 | `algorithm/delaunay.rs` | `circle_top` | S1 | landed in PR #34 (carries `circletopcount`) |
| `check4deadevent` | triangle.c:5001 | `algorithm/delaunay.rs` | `check_for_dead_event` | S1 | landed in PR #34 |
| `splay` | triangle.c:5024 | `algorithm/delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 |
| `splayinsert` | triangle.c:5134 | `algorithm/delaunay.rs` | `splay_insert` | S1 | landed in PR #34 (allocates from the `splaynodes` WordPool) |
| `circletopinsert` | triangle.c:5161 | `algorithm/delaunay.rs` | `circle_top_insert` | S1 | landed in PR #34 |
| `frontlocate` | triangle.c:5189 | `algorithm/delaunay.rs` | `front_locate` | S1 | landed in PR #34 |
| `sweeplinedelaunay` | triangle.c:5212 | `algorithm/delaunay.rs` | `sweepline_delaunay` | S1 | landed in PR #34 |
| `delaunay` | triangle.c:5429 | `algorithm/delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 (dispatcher: incremental/sweepline/divconq per the landed `Behavior` flags) |
| `reconstruct` | triangle.c:5483 | `algorithm/segments.rs` | same name (snake_case) | S1 | rung 2 (D8): every C status/negative-value flow raises its public Error variant at the raising site; landed in PR #35 |
| `finddirection` | triangle.c:5776 | `algorithm/segments.rs` | `find_direction` | S1 | rung 2 (D8): every C status/negative-value flow raises its error at the raising site; fix pass (perf): the hot chain's signatures carry the narrow internal Copy SegmentChainError, converted to the public variant at the formskeleton boundary; landed in PR #35 |
| `segmentintersection` | triangle.c:5873 | `algorithm/segments.rs` | `segment_intersection` | S1 | rung 2 (D8): every C status/negative-value flow raises its error at the raising site; fix pass (perf): the hot chain's signatures carry the narrow internal Copy SegmentChainError, converted to the public variant at the formskeleton boundary; landed in PR #35 |
| `scoutsegment` | triangle.c:6003 | `algorithm/segments.rs` | `scout_segment` | S1 | rung 2 (D8): every C status/negative-value flow raises its error at the raising site; fix pass (perf): the hot chain's signatures carry the narrow internal Copy SegmentChainError, converted to the public variant at the formskeleton boundary; landed in PR #35 |
| `conformingedge` | triangle.c:6089 | `algorithm/segments.rs` | `conforming_edge` | S1 | rung 2 (D8): every C status/negative-value flow raises its error at the raising site; fix pass (perf): the hot chain's signatures carry the narrow internal Copy SegmentChainError, converted to the public variant at the formskeleton boundary; landed in PR #35 |
| `delaunayfixup` | triangle.c:6201 | `algorithm/segments.rs` | `delaunay_fixup` | S1 | rung 2 (D8): every C status/negative-value flow raises its public Error variant at the raising site; landed in PR #35 |
| `constrainededge` | triangle.c:6312 | `algorithm/segments.rs` | `constrained_edge` | S1 | rung 2 (D8): every C status/negative-value flow raises its error at the raising site; fix pass (perf): the hot chain's signatures carry the narrow internal Copy SegmentChainError, converted to the public variant at the formskeleton boundary; landed in PR #35 |
| `insertsegment` | triangle.c:6420 | `algorithm/segments.rs` | `insert_segment` | S1 | rung 2 (D8): every C status/negative-value flow raises its error at the raising site; fix pass (perf): the hot chain's signatures carry the narrow internal Copy SegmentChainError, converted to the public variant at the formskeleton boundary; landed in PR #35 |
| `markhull` | triangle.c:6517 | `algorithm/segments.rs` | `mark_hull` | S1 | rung 2 (D8): every C status/negative-value flow raises its public Error variant at the raising site; landed in PR #35 |
| `formskeleton` | triangle.c:6554 | `algorithm/segments.rs` | `form_skeleton` | S1 | rung 2 (D8): every C status/negative-value flow raises its public Error variant at the raising site; landed in PR #35 |
| `infecthull` | triangle.c:6642 | `algorithm/segments.rs` | `infect_hull` | S1 | rung 2 (D8): every C status/negative-value flow raises its public Error variant at the raising site; landed in PR #35 |
| `plague` | triangle.c:6714 | `algorithm/segments.rs` | same name (snake_case) | S1 | rung 2 (D8): every C status/negative-value flow raises its public Error variant at the raising site; landed in PR #35 |
| `regionplague` | triangle.c:6886 | `algorithm/segments.rs` | `region_plague` | S1 | rung 2 (D8): every C status/negative-value flow raises its public Error variant at the raising site; landed in PR #35 |
| `carveholes` | triangle.c:6964 | `algorithm/segments.rs` | `carve_holes` | S1 | rung 2 (D8): every C status/negative-value flow raises its public Error variant at the raising site; landed in PR #35 |
| `tallyencs` | triangle.c:7129 | `algorithm/refine.rs` | `tally_encroached_subsegs` | S1 | landed in the refine PR (#39); consumer-carry: the badsubseg queue rows come from checkseg4encroach (algorithm/quality.rs) and drain through the mesh/slots.rs accessors |
| `precisionerror` | triangle.c:7154 | `algorithm/refine.rs` | `precision_error` | S1 | landed in the refine PR (#39); the only C call site (triangle.c:7510) is _DEBUG-gated, so the verbatim body is ahead-of-use until the CLI wiring |
| `splitencsegs` | triangle.c:7183 | `algorithm/refine.rs` | `split_encroached_subsegs` | S1 | landed in the refine PR (#39); consumer-carry: badsubsegdealloc/badsubsegtraverse (mesh/mod.rs rows below) and the badsubseg row readers (mesh/slots.rs, promoted from test-only); rung 2 (D8): the TRI_SEG_SPLIT/TRI_CYCLE writes raise Error::SegmentRecovery in place |
| `tallyfaces` | triangle.c:7383 | `algorithm/refine.rs` | `tally_faces` | S1 | landed in the refine PR (#39); the queue it fills is the one enforcequality re-inits verbatim (triangle.c:7559-7561) |
| `splittriangle` | triangle.c:7409 | `algorithm/refine.rs` | `split_triangle` | S1 | landed in the refine PR (#39); the findNewSPLocation site (triangle.c:7437) mirrors the acute compile switch through the crate's acute feature and is live since the stage-1 acute C PR (#43) through the family's D26 seam (tuple interpolants, the Oom funnel); D16 superseded |
| `enforcequality` | triangle.c:7527 | `algorithm/refine.rs` | `enforce_quality` | S1 | landed in the refine PR (#39); re-runs C's bad-triangle queue init verbatim (the mesh/mod.rs state) and carries the badsubseg/badtriang pool restarts (mesh/slots.rs); rung 2 (D8): the TRI_CYCLE write raises Error::SegmentRecovery in place; wired by api.rs Mesh::refine (triangle_mesh_refine row); the aCute scratch-pool init site (triangle.c:7536-7540) is a documented no-op per D23 (the pool is eager at Mesh::new) since stage-1 acute C (#43); D16 superseded |
| `highorder` | triangle.c:7624 | `algorithm/refine.rs` | `high_order` | S1 | landed in the refine PR (#39); deliberately moved from just before C's Array I/O banner (triangle.c:7687) for cohesion with its only caller, triangle_mesh_create (triangle_api.c:256), which calls it through api.rs Mesh::create since the ioarrays part-1 PR (#44) |
| `transfernodes` | triangle.c:7697 | `io/arrays.rs` | `transfer_nodes` | S1 | landed in the ioarrays part-1 PR (#44); the TRI_INPUT return funnels through the Result channel exactly where triangle_api.c:204-206 funnels the status (D8); wired by api.rs Mesh::create, which now runs the full triangle_mesh_create sequence (triangle_api.c:200-256); the writers below landed in the ioarrays part-2 PR (#45) |
| `writenodes` | triangle.c:7768 | `io/arrays.rs` | `write_nodes` | S1 | landed in the ioarrays part-2 PR (#45); the numbering-over-marks scratch reuse (triangle.c:7820-7823) mirrors through the vx chokepoint, writepoly/writeedges/writeneighbors read the numbers back exactly where C does, and Mesh::output restores the original markers afterwards (triangle_api.c:423-425, the input.rs restore_pointmarkers row) |
| `numbernodes` | triangle.c:7840 | `io/arrays.rs` | `number_nodes` | S1 | landed in the ioarrays part-2 PR (#45); no live C caller anywhere - both real numbering sites inline the loop (writenodes triangle.c:7823; file_writenodes' setvertexmark inside its own traversal, triangle_io.c:64-65) and the only other trace is the commented-out call at src/examples/triangle-cli/main.c:65 - so it is dead_code-allowed as a parity landing of C's own uncalled function; the undead row under jettison receives the current number without advancing it, C's own collision (triangle.c:7849-7852, unobservable because writenodes never writes an undead row) |
| `writeelements` | triangle.c:7863 | `io/arrays.rs` | `write_elements` | S1 | landed in the ioarrays part-2 PR (#45); the order-2 mid-edge reads mirror C's mid1/mid2/mid3 indexing over the highorder slots (triangle.c:7906-7915) |
| `writepoly` | triangle.c:7933 | `io/arrays.rs` | `write_poly` | S1 | landed in the ioarrays part-2 PR (#45); the endpoint numbers read back through vertexmark (triangle.c:7965-7966) and Mesh::output surfaces the hole/region counters around the call (triangle_api.c:404-405) |
| `writeedges` | triangle.c:7983 | `io/arrays.rs` | `write_edges` | S1 | landed in the ioarrays part-2 PR (#45); the emit gate's conjunct swap is D28 (triangle.c:8021, the D18 precedent at 7653); the hull marker is the flag itself without segments, the bonded subsegment's boundary marker under -p (triangle.c:8026-8039) |
| `writeneighbors` | triangle.c:8054 | `io/arrays.rs` | `write_neighbors` | S1 | landed in the ioarrays part-2 PR (#45); the numbering pun over word 6 rides the D2 int overlay with the scratch save/restore (triangle.c:8088-8089, 8119-8129) and the dummy's -1 (8094) left exactly as C leaves it; C's out-of-row TODO without -n (triangle.c:8073-8076) is refused loudly at the chokepoint instead |
| `quality_statistics` | triangle.c:8143 | `algorithm/quality.rs` | same name (snake_case) | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion; landed in PR #36 (with the #28 restoration of the algorithm/transform.rs call sites, the bad-queue fields on mesh/mod.rs, the badtriang accessors in mesh/slots.rs, and the api.rs statistics/quality/check wiring) |
### `acute.c` (src/Triangle/acute.c)

| C function | C site | Rust module | Rust symbol (R4) | Stage | Notes |
| --- | --- | --- | --- | --- | --- |
| `acutepool_init` | acute.c:22 | `acute.rs` | dissolves: eager AcutePool construction (D23) | S1 | consumes the #9 pool interface (ADR 0002 §4 item 8); landed in the stage-1 acute B PR (#41) as the D23 dissolution: the trimalloc'd struct hanging off m->acute_mem (triangle.h:496) becomes the eight typed Vec<f64> buffers of the AcutePool owned by Mesh.acute_mem behind the feature, eagerly constructed at the Mesh::new site mapping triangleinit's m->acute_mem = NULL (triangle.c:1507) with exactly the acutepool_init(20) capacities of the enforcequality call (triangle.c:7538; acute.c:26-40) |
| `acutepool_resize` | acute.c:45 | `acute.rs` | `acute_pool_resize` | S1 | consumes the #9 pool interface (ADR 0002 §4 item 8); landed in the stage-1 acute B PR (#41): acutepool_resize survives verbatim with C's only-if-smaller guard (acute.c:46-47) and the m-for-pool parameter split (D23), called where C calls it (acute.c:2276, 2522) |
| `acutepool_deinit` | acute.c:64 | `acute.rs` | dissolves: Drop of Mesh.acute_mem (D23) | S1 | consumes the #9 pool interface (ADR 0002 §4 item 8); landed in the stage-1 acute B PR (#41) as the D23 dissolution: deinit and the trifree that follows it in triangledeinit (triangle.c:1395-1396) dissolve into Drop of the owned field; the port's triangledeinit mirror has not landed yet (the mesh/mod.rs module doc documents the site) |
| `findNewSPLocation` | acute.c:95 | `acute.rs` | `find_new_sp_location` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the stage-1 acute C PR (#43): the dispatcher and its two variants, the D26 seam (tuple interpolants, unread offcenter, Result funnel) |
| `findNewSPLocationWithoutMaxAngle` | acute.c:113 | `acute.rs` | `find_new_sp_location_without_max_angle` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the stage-1 acute C PR (#43): the dispatcher and its two variants, the D26 seam (tuple interpolants, unread offcenter, Result funnel) |
| `findNewSPLocationWithMaxAngle` | acute.c:722 | `acute.rs` | `find_new_sp_location_with_max_angle` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the stage-1 acute C PR (#43): the dispatcher and its two variants, the D26 seam (tuple interpolants, unread offcenter, Result funnel) |
| `longestShortestEdge` | acute.c:1816 | `acute.rs` | `longest_shortest_edge` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the acute substrate PR (#40) (PR A of the three-PR acute.c unit; pure geometry only, the findNewSPLocation callers at acute.c:221 and 843 landed with stage-1 acute C (#43)) |
| `doSmoothing` | acute.c:1858 | `acute.rs` | `do_smoothing` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the stage-1 acute B PR (#41) (PR B of three; the star-walk and wedge consumers landed with it; the scoped pool move of the buffer pointers at acute.c:1877-1879 per D23); the findNewSPLocation callers (acute.c:397, 1019) landed with stage-1 acute C (#43) |
| `getStarPoints` | acute.c:2040 | `acute.rs` | `get_star_points` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the stage-1 acute B PR (#41) (PR B of three; the doSmoothing consumer landed with it; the getNeighborsVertex walk at acute.c:2093 landed with it) |
| `getNeighborsVertex` | acute.c:2121 | `acute.rs` | `get_neighbors_vertex` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the stage-1 acute B PR (#41) (PR B of three; the getStarPoints consumer landed with it; the not-found arm keeps C's last-sym'd neighbor write of acute.c:2238) |
| `getWedgeIntersectionWithoutMaxAngle` | acute.c:2250 | `acute.rs` | `get_wedge_intersection_without_max_angle` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the stage-1 acute B PR (#41) (PR B of three; the resize at acute.c:2276 routes to the mesh-owned pool through the D23 parameter split; the buffer pulls at acute.c:2278-2282 are the wedge_scratch borrow tuple; the doSmoothing callers at acute.c:1908-1910 landed with it) |
| `getWedgeIntersectionWithMaxAngle` | acute.c:2490 | `acute.rs` | `get_wedge_intersection_with_max_angle` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the stage-1 acute B PR (#41) (PR B of three; the resize at acute.c:2522 and the buffer pulls at acute.c:2524-2528 per D23; the testTriangleAngle scans at acute.c:2849, 2853, 2881 and 2885 landed with it; the doSmoothing callers at acute.c:1948-1950, 1989-1991 landed with it) |
| `polygonAngles` | acute.c:2918 | `acute.rs` | `polygon_angles` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the stage-1 acute B PR (#41) (PR B of three; the doSmoothing scans at acute.c:1904, 1944 and 1985 landed with it) |
| `testPolygonAngle` | acute.c:2944 | `acute.rs` | `test_polygon_angle` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the stage-1 acute B PR (#41) (PR B of three; the polygonAngles calls at acute.c:2922-2930 landed with it) |
| `lineLineIntersection` | acute.c:2983 | `acute.rs` | `line_line_intersection` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the acute substrate PR (#40) (PR A of three; the getWedgeIntersection callers, first at acute.c:2393, land with the later acute PRs) |
| `halfPlaneIntersection` | acute.c:3032 | `acute.rs` | `half_plane_intersection` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the acute substrate PR (#40) (PR A of three; the getWedgeIntersection callers at acute.c:2428, 2437, 2811 and 2832 land with the later acute PRs; the malloc'd polys scratch mirrors as the D21 stack substrate) |
| `splitConvexPolygon` | acute.c:3096 | `acute.rs` | `split_convex_polygon` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the acute substrate PR (#40) (PR A of three; the only C caller is halfPlaneIntersection, acute.c:3048); structural note: C's REAL *polys[] out-parameter (the two caller-malloc'd 100-REAL buffers of acute.c:3037, 3043-3044) mirrors as one &mut [[f64; 100]; 2], the poly1/poly2 aliasing locals (acute.c:3105, 3108) write through polys[0]/polys[1] directly, and the pointer-identity stores (acute.c:3239, 3242) drop (D21) |
| `linePointLocation` | acute.c:3255 | `acute.rs` | `line_point_location` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the acute substrate PR (#40) (PR A of three; the only C callers are splitConvexPolygon's split test, acute.c:3171-3172) |
| `lineLineSegmentIntersection` | acute.c:3279 | `acute.rs` | `line_line_segment_intersection` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the acute substrate PR (#40) (PR A of three; the only C caller is splitConvexPolygon's edge walk, acute.c:3118) |
| `findPolyCentroid` | acute.c:3330 | `acute.rs` | `find_poly_centroid` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the acute substrate PR (#40) (PR A of three; the getWedgeIntersection callers at acute.c:2448 and 2844 land with the later acute PRs) |
| `circleLineIntersection` | acute.c:3354 | `acute.rs` | `circle_line_intersection` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the acute substrate PR (#40) (PR A of three; the findNewSPLocation callers at acute.c:478, 590, 1141 and 1408 landed with stage-1 acute C (#43)) |
| `chooseCorrectPoint` | acute.c:3411 | `acute.rs` | `choose_correct_point` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the acute substrate PR (#40) (PR A of three; the findNewSPLocation callers at acute.c:485, 599, 1109, 1144 and 1413 landed with stage-1 acute C (#43)) |
| `pointBetweenPoints` | acute.c:3458 | `acute.rs` | `point_between_points` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the acute substrate PR (#40) (PR A of three; the findNewSPLocation callers at acute.c:497, 612, 1181, 1191, 1269, 1449, 1458 and 1531 landed with stage-1 acute C (#43)) |
| `testTriangleAngle` | acute.c:3493 | `acute.rs` | `test_triangle_angle` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the stage-1 acute B PR (#41) (PR B of three; the in-PR callers are the With scans at acute.c:2849, 2853, 2881 and 2885; the findNewSPLocation callers, first at acute.c:505, landed with stage-1 acute C (#43)) |
| `minDistanceToNeigbor` | acute.c:3584 | `acute.rs` | `min_distance_to_neighbor` | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase idiomatized at rung 4 (issue #60); landed in the stage-1 acute B PR (#41) (PR B of three; the collinear-not-between pin of D25; the findNewSPLocation callers at acute.c:1203, 1280, 1470 and 1542 landed with stage-1 acute C (#43)) |
### `predicates.c` (src/Triangle/predicates.c)

| C function | C site | Rust module | Rust symbol (R4) | Stage | Notes |
| --- | --- | --- | --- | --- | --- |
| `tri_exactinit` | predicates.c:173 | `predicates.rs` | `exact_init` | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_fast_expansion_sum_zeroelim` | predicates.c:246 | `predicates.rs` | `fast_expansion_sum_zeroelim` | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_scale_expansion_zeroelim` | predicates.c:330 | `predicates.rs` | `scale_expansion_zeroelim` | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_estimate` | predicates.c:377 | `predicates.rs` | `estimate` | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_counterclockwiseadapt` | predicates.c:409 | `predicates.rs` | `counterclockwise_adapt` | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_counterclockwise` | predicates.c:489 | `predicates.rs` | `counterclockwise` | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_incircleadapt` | predicates.c:548 | `predicates.rs` | `incircle_adapt` | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_incircle` | predicates.c:1117 | `predicates.rs` | `incircle` | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_orient3dadapt` | predicates.c:1188 | `predicates.rs` | `orient3d_adapt` | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_orient3d` | predicates.c:1598 | `predicates.rs` | `orient3d` | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_nonregular` | predicates.c:1666 | `predicates.rs` | `nonregular` | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_findcircumcenter` | predicates.c:1696 | `predicates.rs` | `find_circumcenter` | S1 | pure Rust port; x87 control-word code dropped (D5) |
### `eps_writer.c` (src/Triangle/eps_writer.c)

| C function | C site | Rust module | Rust symbol (R4) | Stage | Notes |
| --- | --- | --- | --- | --- | --- |
| `eps_write_header` | eps_writer.c:14 | `io/eps.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46); byte-for-byte through the formatter seam |
| `eps_write_trailer` | eps_writer.c:32 | `io/eps.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46) |
| `eps_draw_clip` | eps_writer.c:43 | `io/eps.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46) |
| `eps_draw_edges` | eps_writer.c:87 | `io/eps.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46); the visit gate's conjunct swap is D30 |
| `eps_draw_segments` | eps_writer.c:132 | `io/eps.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46) |
| `eps_draw_points` | eps_writer.c:161 | `io/eps.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46) |
| `eps_set_color` | eps_writer.c:192 | `io/eps.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46) |
| `eps_set_stroke` | eps_writer.c:198 | `io/eps.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46) |
| `eps_update_metrics` | eps_writer.c:205 | `io/eps.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46); the 5% margins and the aspect-fit delta |
### `triangle_helper.c` (src/Triangle/triangle_helper.c)

| C function | C site | Rust module | Rust symbol (R4) | Stage | Notes |
| --- | --- | --- | --- | --- | --- |
| `triangle_check_context` | triangle_helper.c:5 | `-` | same name (snake_case) | - | dissolves: owned values cannot be null (D8) |
| `triangle_check_behavior` | triangle_helper.c:17 | `options.rs` | same name (snake_case) | S1 shell | Options validation |
| `triangle_check_triangleio` | triangle_helper.c:25 | `input.rs` | same name (snake_case) | S1 shell | MeshInput validation; restore markers is an internal step of the output path |
| `triangle_restore_pointmarkers` | triangle_helper.c:77 | `input.rs` | same name (snake_case) | S1 | landed in the ioarrays part-2 PR (#45); the output path's internal step (restore_pointmarkers), wired by api.rs Mesh::output behind the marker-list gate (triangle_api.c:423-425); the list index starts at 0 and skips the undead under jettison exactly like the writer that produced the list (triangle_helper.c:88-96) |
### `triangle_io.c` (src/Triangle/triangle_io.c)

| C function | C site | Rust module | Rust symbol (R4) | Stage | Notes |
| --- | --- | --- | --- | --- | --- |
| `file_writenodes` | triangle_io.c:23 | `io/mod.rs` | `file_write_nodes` | IO | landed in the I/O-phase PR (#46); inline numbering (setvertexmark at triangle_io.c:64-65), the writer seam is D31, the open funnels to Error::Io (D8) |
| `file_writeelements` | triangle_io.c:78 | `io/mod.rs` | `file_write_elements` | IO | landed in the I/O-phase PR (#46); the order-2 mid slots ho+1, ho+2, ho (triangle_io.c:107-118) |
| `file_writepoly` | triangle_io.c:133 | `io/mod.rs` | `file_write_poly` | IO | landed in the I/O-phase PR (#46); the hole/region tails (triangle_io.c:173-193) sit behind #ifndef CDT_ONLY, which this fork's build leaves undefined, so they always render in the measured binary; the port builds the full shape |
| `file_writeedges` | triangle_io.c:203 | `io/mod.rs` | `file_write_edges` | IO | landed in the I/O-phase PR (#46); the emit gate's conjunct swap is D29 |
| `file_writeneighbors` | triangle_io.c:264 | `io/mod.rs` | `file_write_neighbors` | IO | landed in the I/O-phase PR (#46); the word-6 overlay without the restore, the dummy's -1 stays (the overlay numbering at triangle_io.c:283-287 with the dummy's -1 at 287, the neighbor reads at 295-301) |
| `file_write_eps` | triangle_io.c:312 | `io/mod.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46); composes the io/eps.rs unit (triangle_io.c:331-347) |
| `readline` | triangle_io.c:370 | `io/mod.rs` | `read_line` | IO | landed in the I/O-phase PR (#46); INPUTLINESIZE, '#' comments, the number-lookahead charset, the rescan loop |
| `findfield` | triangle_io.c:401 | `io/mod.rs` | `find_field` | IO | landed in the I/O-phase PR (#46) |
| `file_readnodes_internal` | triangle_io.c:432 | `io/mod.rs` | `file_read_nodes_internal` | IO | landed in the I/O-phase PR (#46); firstnode captured at i == 0, the attribute/marker defaults fill (triangle_io.c:432-536) |
| `file_readsegments_internal` | triangle_io.c:546 | `io/mod.rs` | `file_read_segments_internal` | IO | landed in the I/O-phase PR (#46) |
| `file_readholes_internal` | triangle_io.c:616 | `io/mod.rs` | `file_read_holes_internal` | IO | landed in the I/O-phase PR (#46); the missing-area branch copies the attribute (triangle_io.c:694) |
| `file_readnodes` | triangle_io.c:712 | `io/mod.rs` | `file_read_nodes` | IO | landed in the I/O-phase PR (#46) |
| `file_readpoly` | triangle_io.c:723 | `io/mod.rs` | `file_read_poly` | IO | landed in the I/O-phase PR (#46) |
| `file_readelements` | triangle_io.c:744 | `io/mod.rs` | `file_read_elements` | IO | landed in the I/O-phase PR (#46); numberofcorners pinned to 3 (triangle_io.c:783) |
| `file_readelementsarea` | triangle_io.c:828 | `io/mod.rs` | `file_read_elements_area` | IO | landed in the I/O-phase PR (#46); a missing field stores -1.0 (triangle_io.c:864) |
### `triangle_api.c` (src/Triangle/triangle_api.c)

| C function | C site | Rust module | Rust symbol (R4) | Stage | Notes |
| --- | --- | --- | --- | --- | --- |
| `triangle_version` | triangle_api.c:7 | `lib.rs` | VERSION, HAS_ACUTE | S1 shell | VERSION / HAS_ACUTE constants |
| `triangle_context_create` | triangle_api.c:20 | `api.rs` | triangulate/Mesh constructors | S1 shell | dissolves into owned Options + Mesh state (D11); defaults via Options::new |
| `triangle_context_destroy` | triangle_api.c:55 | `-` | same name (snake_case) | - | dissolves: Drop/RAII (D11) |
| `triangle_context_options` | triangle_api.c:69 | `options.rs` | Options::parse | S1 shell | Options::parse (options string) |
| `triangle_context_get_behavior` | triangle_api.c:80 | `api.rs` | Options / Behavior | S1 shell | Options is consumed by value at the entry points; the normalized Behavior is read-only state of Mesh |
| `triangle_context_set_behavior` | triangle_api.c:89 | `api.rs` | Options / Behavior | S1 shell | Options is consumed by value at the entry points; the normalized Behavior is read-only state of Mesh |
| `triangle_mesh_quality` | triangle_api.c:102 | `api.rs` | Mesh::quality | S1 shell | |
| `triangle_mesh_statistics` | triangle_api.c:111 | `api.rs` | Mesh::statistics | S1 shell | |
| `triangle_memory` | triangle_api.c:143 | `api.rs` | Mesh::memory_bytes | S1 shell | |
| `triangle_check_mesh` | triangle_api.c:163 | `api.rs` | Mesh::check_mesh, Mesh::check_delaunay | S1 shell | failure counts (usize), non-error semantics |
| `triangle_check_delaunay` | triangle_api.c:172 | `api.rs` | Mesh::check_mesh, Mesh::check_delaunay | S1 shell | failure counts (usize), non-error semantics |
| `triangle_mesh_create` | triangle_api.c:181 | `api.rs` | triangulate | S1 shell | entry point |
| `triangle_mesh_load` | triangle_api.c:262 | `api.rs` | Mesh::load_mesh | S1 shell | landed in the load-orchestration PR (#53): the api-level sequence around the landed reconstruct (transfernodes and its status funnel, triangle_api.c:285-291; reconstruct with the negative-value TRI_INPUT funnel, 293-303; the infvertex NULLs, 305-309; the usesegments/checksegments store with no formskeleton call, 311-313; the poly holes/regions gate, 315-324; the edges writer, 326-327); the context NULL check (269-271) and the mesh-already-created TRI_FAILURE (276-279) dissolve over the owned fresh-constructed mesh (D8, Mesh::create's twin-check precedent at 195-197); the -r CLI flow runs end to end (load then refine, main.c:44-52) |
| `triangle_mesh_refine` | triangle_api.c:332 | `api.rs` | Mesh::refine | S1 shell | wired in the refine PR (#39): the quality gate (triangle_api.c:347), the status funnel, and the m->edges writer inside the gate (triangle_api.c:354-355) |
| `triangle_mesh_copy` | triangle_api.c:362 | `api.rs` | Mesh::output | S1 shell | |
| `triangle_free` | triangle_api.c:430 | `-` | same name (snake_case) | - | dissolves: Drop/RAII (D11) |
| `triangle_write_nodes` | triangle_api.c:437 | `io/mod.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46): the api.rs wrapper owns the open (fopen NULL -> TRI_FILE_OPEN dissolves into File::open's Error::Io, D8) and calls the io/mod.rs writer through the D15 RefCell bridge |
| `triangle_write_elements` | triangle_api.c:446 | `io/mod.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46): the api.rs wrapper owns the open (fopen NULL -> TRI_FILE_OPEN dissolves into File::open's Error::Io, D8) and calls the io/mod.rs writer through the D15 RefCell bridge |
| `triangle_write_poly` | triangle_api.c:455 | `io/mod.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46): the api.rs wrapper owns the open (fopen NULL -> TRI_FILE_OPEN dissolves into File::open's Error::Io, D8) and calls the io/mod.rs writer through the D15 RefCell bridge |
| `triangle_write_edges` | triangle_api.c:466 | `io/mod.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46): the api.rs wrapper owns the open (fopen NULL -> TRI_FILE_OPEN dissolves into File::open's Error::Io, D8) and calls the io/mod.rs writer through the D15 RefCell bridge |
| `triangle_write_neighbors` | triangle_api.c:475 | `io/mod.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46): the api.rs wrapper owns the open (fopen NULL -> TRI_FILE_OPEN dissolves into File::open's Error::Io, D8) and calls the io/mod.rs writer through the D15 RefCell bridge |
| `triangle_write_eps` | triangle_api.c:484 | `io/mod.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46): the api.rs wrapper owns the open (fopen NULL -> TRI_FILE_OPEN dissolves into File::open's Error::Io, D8) and calls the io/mod.rs writer through the D15 RefCell bridge |
| `triangle_read_nodes` | triangle_api.c:493 | `io/mod.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46): the api.rs wrapper owns the open and fills MeshInput through the io/mod.rs readers (fopen NULL -> Error::Io, D8; the reader status funnels per D8) |
| `triangle_read_poly` | triangle_api.c:509 | `io/mod.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46): the api.rs wrapper owns the open and fills MeshInput through the io/mod.rs readers (fopen NULL -> Error::Io, D8; the reader status funnels per D8) |
| `triangle_read_elements` | triangle_api.c:525 | `io/mod.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46): the api.rs wrapper owns the open and fills MeshInput through the io/mod.rs readers (fopen NULL -> Error::Io, D8; the reader status funnels per D8) |
| `triangle_read_area` | triangle_api.c:541 | `io/mod.rs` | same name (snake_case) | IO | landed in the I/O-phase PR (#46): the api.rs wrapper owns the open and fills MeshInput through the io/mod.rs readers (fopen NULL -> Error::Io, D8; the reader status funnels per D8) |

## Types and macros (triangle.h and friends)

| C construct | C site | Rust location | Notes |
| --- | --- | --- | --- |
| `encode()` / `decode()` | triangle.h:703-708 | `mesh/handles.rs` | tagged pointers become packed u64 handles (D1) |
| `sencode()` / `sdecode()` | triangle.h:703-708 | `mesh/handles.rs` | subsegments pack 1 bit (D1) |
| handle macro family (`sym`, `lnext`, `lprev`, `onext`, `oprev`, `dnext`, `dprev`, `rnext`, `rprev`, `bond`, `dissolve`, `tspivot`, `stpivot`, `tsbond`, `tsdissolve`, `stdissolve`, ...) | triangle.h:703-1090 | `mesh/handles.rs` (pure rotations), `mesh/slots.rs` (storage-touching) | macro-method mirror; hidden ptr/sptr locals disappear (D6) |
| `org()` / `dest()` / `apex()` | triangle.h:703-1090 | `mesh/slots.rs` | vertex-id reads, no references escape |
| `vertexmark` / `vertextype` / `vertex2tri` accessors | triangle.h:1077-1088 | `mesh/slots.rs` | int overlays / encoded word behind the chokepoint (D2) |
| `struct behavior` | triangle.h | `options.rs` | public `Options` mirror per switch family |
| `struct mesh` | triangle.h | `mesh/mod.rs` (struct + lifecycle) + `mesh/slots.rs` (accessor chokepoint) | pool ownership per ADR 0002; storage share corrected per the ADR 0002 amendment (2026-11-24) |
| `struct memorypool` | triangle.c | `mesh/pool.rs` | ticket #9 per the ADR 0002 §4 contract (D4) |
| `struct quality` / `struct statistics` / `struct rect` | triangle_api.h / triangle.h | `api.rs` | public `Quality`, `Statistics`, `BoundingBox` |
| `TRI_*` status codes | triangle_config.h:28-40 | `error.rs` | the public `Error` taxonomy; the raising sites construct the variants in place (rung 2, D8) |
| `enumerations` (`locateresult`, `insertvertexresult`, `finddirectionresult`, ...) | triangle.h | the owning module of the function that returns them | transliterated verbatim |
| `triunsuitable_user_func` (typedef) | triangle.h | `options.rs` | `Unsuitable` trait + `VertexSample` values (D9) |
| `FILENAMESIZE`, `TRINAME`-style string helpers | triangle_io.c / util.c of the example CLI | `io/mod.rs` / `triangle-cli` | landed in the I/O-phase PR (#46): owned Strings, no FILENAMESIZE truncation (D32) |


## R4 idiomatization rename record

Rung 4 (issue #60) idiomatized the crate-internal names kept
through stage 1 and rungs 1-3: verb-first, snake_case multi-word,
no C-style identifier concatenation. This is the complete
old-internal-name -> R4-name mapping, so every transliterated
function stays greppable from its C anchor (the C anchors in the
code comments, e.g. 'C: insertsegment (triangle.c:6420)', are
untouched, and so are all string payloads).

| Stage-1 Rust name | R4 Rust name |
| --- | --- |
| `acutepool_resize` | `acute_pool_resize` |
| `alternateaxes` | `alternate_axes` |
| `bad_sub_seg_alloc` | `bad_subseg_alloc` |
| `bad_sub_seg_dest` | `bad_subseg_dest` |
| `bad_sub_seg_enc` | `bad_subseg_enc` |
| `bad_sub_seg_org` | `bad_subseg_org` |
| `bad_sub_seg_restart` | `bad_subseg_restart` |
| `bad_tri_poortri` | `bad_tri_poor_tri` |
| `badsubsegdealloc` | `dealloc_bad_subseg` |
| `BADSUBSEGPERBLOCK` | `BAD_SUBSEG_PER_BLOCK` |
| `badsubsegtraverse` | `traverse_bad_subseg` |
| `BADTRIPERBLOCK` | `BAD_TRI_PER_BLOCK` |
| `behavior_update` | `update_derived_fields` |
| `boundingbox` | `bounding_box` |
| `carveholes` | `carve_holes` |
| `check4deadevent` | `check_for_dead_event` |
| `checkdelaunay` | `check_delaunay` |
| `checkmesh` | `check_mesh` |
| `checkseg4encroach` | `check_seg_for_encroachment` |
| `chooseCorrectPoint` | `choose_correct_point` |
| `circleLineIntersection` | `circle_line_intersection` |
| `circletop` | `circle_top` |
| `circletopinsert` | `circle_top_insert` |
| `compConst` | `COMP_CONST` |
| `conformingedge` | `conforming_edge` |
| `constrainededge` | `constrained_edge` |
| `createeventheap` | `create_event_heap` |
| `deadsubseg` | `is_dead_subseg` |
| `deadtri` | `is_dead_tri` |
| `DEADVERTEX` | `DEAD_VERTEX` |
| `delaunayfixup` | `delaunay_fixup` |
| `deletevertex` | `delete_vertex` |
| `dequeuebadtriang` | `dequeue_bad_triangle` |
| `divconqdelaunay` | `divconq_delaunay` |
| `divconqrecurse` | `divconq_recurse` |
| `dnextself` | `dnext_self` |
| `doSmoothing` | `do_smoothing` |
| `dprevself` | `dprev_self` |
| `dummyinit` | `init_sentinel_rows` |
| `enforcequality` | `enforce_quality` |
| `enqueuebadtri` | `enqueue_bad_triangle` |
| `enqueuebadtriang` | `enqueue_bad_triangle_row` |
| `eventheapdelete` | `event_heap_delete` |
| `eventheapify` | `event_heapify` |
| `eventheapinsert` | `event_heap_insert` |
| `file_readelements` | `file_read_elements` |
| `file_readelementsarea` | `file_read_elements_area` |
| `file_readholes_internal` | `file_read_holes_internal` |
| `file_readnodes` | `file_read_nodes` |
| `file_readnodes_internal` | `file_read_nodes_internal` |
| `file_readpoly` | `file_read_poly` |
| `file_readsegments_internal` | `file_read_segments_internal` |
| `file_writeedges` | `file_write_edges` |
| `file_writeelements` | `file_write_elements` |
| `file_writeneighbors` | `file_write_neighbors` |
| `file_writenodes` | `file_write_nodes` |
| `file_writepoly` | `file_write_poly` |
| `finddirection` | `find_direction` |
| `findfield` | `find_field` |
| `findNewSPLocation` | `find_new_sp_location` |
| `findNewSPLocationWithMaxAngle` | `find_new_sp_location_with_max_angle` |
| `findNewSPLocationWithoutMaxAngle` | `find_new_sp_location_without_max_angle` |
| `findPolyCentroid` | `find_poly_centroid` |
| `finishfile` | `finish_file` |
| `flip_flippedtri` | `flip_flipped_tri` |
| `flip_prevflip` | `flip_prev_flip` |
| `FLIPSTACKERPERBLOCK` | `FLIP_STACKER_PER_BLOCK` |
| `formskeleton` | `form_skeleton` |
| `FREEVERTEX` | `FREE_VERTEX` |
| `frontlocate` | `front_locate` |
| `getNeighborsVertex` | `get_neighbors_vertex` |
| `getStarPoints` | `get_star_points` |
| `getvertex` | `get_vertex` |
| `getWedgeIntersectionWithMaxAngle` | `get_wedge_intersection_with_max_angle` |
| `getWedgeIntersectionWithoutMaxAngle` | `get_wedge_intersection_without_max_angle` |
| `halfPlaneIntersection` | `half_plane_intersection` |
| `highorder` | `high_order` |
| `incrementaldelaunay` | `incremental_delaunay` |
| `infecthull` | `infect_hull` |
| `INFVERTEX1` | `INF_VERTEX_1` |
| `INFVERTEX2` | `INF_VERTEX_2` |
| `INFVERTEX3` | `INF_VERTEX_3` |
| `initializetrisubpools` | `init_tri_and_subseg_pools` |
| `initializevertexpool` | `init_vertex_pool` |
| `inlens` | `in_lens` |
| `INPUTLINESIZE` | `INPUT_LINE_SIZE` |
| `INPUTVERTEX` | `INPUT_VERTEX` |
| `insertsegment` | `insert_segment` |
| `insertsubseg` | `insert_subseg` |
| `insertvertex` | `insert_vertex` |
| `killsubseg` | `kill_subseg` |
| `killtri` | `kill_tri` |
| `lineLineIntersection` | `line_line_intersection` |
| `lineLineSegmentIntersection` | `line_line_segment_intersection` |
| `linePointLocation` | `line_point_location` |
| `lnextself` | `lnext_self` |
| `longestShortestEdge` | `longest_shortest_edge` |
| `lprevself` | `lprev_self` |
| `makesubseg` | `make_subseg` |
| `maketriangle` | `make_triangle` |
| `makevertexmap` | `make_vertex_map` |
| `markhull` | `mark_hull` |
| `mergehulls` | `merge_hulls` |
| `minDistanceToNeigbor` | `min_distance_to_neighbor` |
| `MINUS1MOD3` | `MINUS_ONE_MOD_3` |
| `NULL` | `NULL_ROW` |
| `numbernodes` | `number_nodes` |
| `onextself` | `onext_self` |
| `oprevself` | `oprev_self` |
| `orgword` | `org_word` |
| `parsecommandline_legacy` | `parse_command_line_legacy` |
| `PLUS1MOD3` | `PLUS_ONE_MOD_3` |
| `pointBetweenPoints` | `point_between_points` |
| `polygonAngles` | `polygon_angles` |
| `preciselocate` | `precise_locate` |
| `precisionerror` | `precision_error` |
| `printsubseg` | `print_subsegment` |
| `printtriangle` | `print_triangle` |
| `randomnation` | `random_below` |
| `readline` | `read_line` |
| `regionplague` | `region_plague` |
| `removebox` | `remove_box` |
| `removeghosts` | `remove_ghosts` |
| `restore_pointmarkers` | `restore_point_markers` |
| `rightofhyperbola` | `right_of_hyperbola` |
| `rnextself` | `rnext_self` |
| `rprevself` | `rprev_self` |
| `SAMPLEFACTOR` | `SAMPLE_FACTOR` |
| `SAMPLERATE` | `SAMPLE_RATE` |
| `sbond` | `bond_subsegs` |
| `scoutsegment` | `scout_segment` |
| `sdecode` | `decode_subseg` |
| `sdest` | `subseg_dest` |
| `sdissolve` | `dissolve_subseg` |
| `segdest` | `segment_dest` |
| `segmentintersection` | `segment_intersection` |
| `SEGMENTVERTEX` | `SEGMENT_VERTEX` |
| `segorg` | `segment_org` |
| `sencode` | `encode_subseg` |
| `set_bad_tri_poortri` | `set_bad_tri_poor_tri` |
| `set_flip_flippedtri` | `set_flip_flipped_tri` |
| `set_flip_prevflip` | `set_flip_prev_flip` |
| `set_splay_keydest` | `set_splay_key_dest` |
| `set_splay_keyedge` | `set_splay_key_edge` |
| `set_splay_lchild` | `set_splay_left_child` |
| `set_splay_rchild` | `set_splay_right_child` |
| `set_tri_areabound` | `set_tri_area_bound` |
| `setapex` | `set_apex` |
| `setdest` | `set_dest` |
| `setorg` | `set_org` |
| `setorgword` | `set_org_word` |
| `setsdest` | `set_subseg_dest` |
| `setsegdest` | `set_segment_dest` |
| `setsegorg` | `set_segment_org` |
| `setsorg` | `set_subseg_org` |
| `snext` | `next_subseg` |
| `snextself` | `next_subseg_self` |
| `sorg` | `subseg_org` |
| `spivot` | `pivot_subseg` |
| `spivotself` | `pivot_subseg_self` |
| `splay_keydest` | `splay_key_dest` |
| `splay_keyedge` | `splay_key_edge` |
| `splay_lchild` | `splay_left_child` |
| `splay_rchild` | `splay_right_child` |
| `splayinsert` | `splay_insert` |
| `SPLAYNODEPERBLOCK` | `SPLAY_NODE_PER_BLOCK` |
| `splitConvexPolygon` | `split_convex_polygon` |
| `splitencsegs` | `split_encroached_subsegs` |
| `splittriangle` | `split_triangle` |
| `SQUAREROOTTWO` | `SQUARE_ROOT_TWO` |
| `ssorient` | `orient` |
| `ssym` | `sym` |
| `ssymself` | `sym_self` |
| `stdissolve` | `dissolve_subseg_tri` |
| `stpivot` | `subseg_tri` |
| `SUBSEGPERBLOCK` | `SUBSEG_PER_BLOCK` |
| `subsegtraverse` | `traverse_subseg` |
| `sweeplinedelaunay` | `sweepline_delaunay` |
| `symself` | `sym_self` |
| `tallyencs` | `tally_encroached_subsegs` |
| `tallyfaces` | `tally_faces` |
| `testPolygonAngle` | `test_polygon_angle` |
| `testtriangle` | `test_triangle` |
| `testTriangleAngle` | `test_triangle_angle` |
| `transfernodes` | `transfer_nodes` |
| `tri_areabound` | `tri_area_bound` |
| `tri_counterclockwise` | `counterclockwise` |
| `tri_counterclockwiseadapt` | `counterclockwise_adapt` |
| `tri_estimate` | `estimate` |
| `tri_exactinit` | `exact_init` |
| `tri_fast_expansion_sum_zeroelim` | `fast_expansion_sum_zeroelim` |
| `tri_findcircumcenter` | `find_circumcenter` |
| `tri_incircle` | `incircle` |
| `tri_incircleadapt` | `incircle_adapt` |
| `tri_nonregular` | `nonregular` |
| `tri_orient3d` | `orient3d` |
| `tri_orient3dadapt` | `orient3d_adapt` |
| `tri_scale_expansion_zeroelim` | `scale_expansion_zeroelim` |
| `triangletraverse` | `traverse_triangle` |
| `triangulatepolygon` | `triangulate_polygon` |
| `TRIPERBLOCK` | `TRI_PER_BLOCK` |
| `tsbond` | `bond_tri_subseg` |
| `tsdissolve` | `dissolve_tri_subseg` |
| `tspivot` | `tri_subseg` |
| `UNDEADVERTEX` | `UNDEAD_VERTEX` |
| `undovertex` | `undo_vertex` |
| `vertexmedian` | `vertex_median` |
| `VERTEXPERBLOCK` | `VERTEX_PER_BLOCK` |
| `vertexsort` | `vertex_sort` |
| `vertextraverse` | `traverse_vertex` |
| `VIRUSPERBLOCK` | `VIRUS_PER_BLOCK` |
| `writeedges` | `write_edges` |
| `writeelement` | `write_element` |
| `writeelements` | `write_elements` |
| `writeneighbors` | `write_neighbors` |
| `writenodes` | `write_nodes` |
| `writepoly` | `write_poly` |

Kept vocabulary (recorded so the absence of a rename row is not an
oversight): Otri/Osub (glossary-pinned register-handle types); the
atomic Shewchuk ring vocabulary (lnext, lprev, onext, oprev, dnext,
dprev, rnext, rprev, sym, org, dest, apex, bond, dissolve, infect,
uninfect, infected, encode, decode, splay) - single C words, not
concatenations; the libc mirrors in io/ (strtod, strtol, fgets,
c_rint); the Mesh/Behavior field mirrors of ADR 0004's
field-mirror contract (poly, usesegments, recenttri, infvertex1,
steinerleft, ...). Test names and mesh-local identifiers that
contained an old token as an underscore-separated segment followed
the same renames (e.g. finddirectionresult_discriminants_are_the_c_values
-> find_direction_result_discriminants_are_the_c_values); every
changed test is a pure rename. minDistanceToNeigbor ->
min_distance_to_neighbor additionally fixes the C function's typo.
These planned idiomatization renames and module moves are not
deviation-register entries; rung 4 used no new deviation number
(D35 stays free).


## Maintenance rules

1. Every new C function that a stage-1 ticket transliterates gets its row
   updated (stage stays, the row gains the landing PR reference). The C core
   is frozen for this port, so the table is append-only in practice.
2. A function that moves during rungs 3-4 gets both rows: the original
   assignment and the rung note with the new location.
3. The extractor for this table lives in the ticket #10 session transcript;
   regenerating it is a grep, not a hand-edit: names and sites come from the
   sources.


