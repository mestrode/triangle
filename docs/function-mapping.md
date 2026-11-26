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
survive transliteration verbatim; idiomatically renamed internals land per
rung 4, not before. Deviations are cross-referenced as D-numbers in
[deviation-register.md](deviation-register.md).

Status of this seed: every C function of every compilation unit of the port's
core (`src/Triangle/*.c`) has a row (195 functions) plus the macro/type
pseudo-rows below. Nothing is unassigned.

## Per-compilation-unit overview

| C unit (`src/Triangle/`) | Rust module(s) in `rust/crates/triangle/src/` | Stage |
| --- | --- | --- |
| `triangle.h` (types, macros, encode/decode) | `handles.rs`, `slots.rs`, `error.rs`, `options.rs` | S1 |
| `triangle.c` | `util.rs`, `options.rs`, `error.rs`, `debug.rs`, `pool.rs`, `mesh.rs`, `quality.rs`, `locate.rs`, `transform.rs`, `delaunay.rs`, `construct.rs`, `refine.rs`, `ioarrays.rs` | S1 |
| `acute.c` | `acute.rs` (behind feature `acute`, default on) | S1 |
| `predicates.c` | `predicates.rs` | S1 |
| `eps_writer.c` | `eps.rs` | IO |
| `triangle_helper.c` | `options.rs`, `input.rs` (unit dissolves) | S1 shell |
| `triangle_io.c` | `io.rs` | IO |
| `triangle_api.c` | `api.rs`, `options.rs`, `error.rs`, `lib.rs`, `io.rs` (context lifecycle dissolves) | S1 shell / IO |
| `triangle_config.h` (TRI_* codes) | `error.rs` (InternalStatus mirror + funnel) | S1 shell |
| `triangle_version.h.in` | `lib.rs` (VERSION, HAS_ACUTE) | S1 shell |
| `src/examples/triangle-cli` | `rust/crates/triangle-cli` (sketch only; argument compatibility pinned in the I/O phase) | IO |

## Function table

Function names and C sites are extracted from the C sources; `file:line`
refers to the source as of this seed commit (line numbers drift as the C
side changes; the function names are the stable key).


### `triangle.c` (src/Triangle/triangle.c)

| C function | C site | Rust module | Rust symbol (stage 1) | Stage | Notes |
| --- | --- | --- | --- | --- | --- |
| `interpolate` | triangle.c:238 | `util.rs` | same name (snake_case) | S1 | landed in PR #30 |
| `behavior_update` | triangle.c:275 | `options.rs` | same name (snake_case) | S1 shell | Options::defaults and Options::parse; derived-field tail transliterated bit-exact |
| `triexit` | triangle.c:313 | `error.rs` | same name (snake_case) | S1 shell | funnel: no process exit, no manual alloc/free (D8, D11); trimalloc OOM becomes InternalStatus::Oom |
| `trimalloc` | triangle.c:318 | `error.rs` | same name (snake_case) | S1 shell | funnel: no process exit, no manual alloc/free (D8, D11); trimalloc OOM becomes InternalStatus::Oom |
| `trifree` | triangle.c:330 | `error.rs` | same name (snake_case) | S1 shell | funnel: no process exit, no manual alloc/free (D8, D11); trimalloc OOM becomes InternalStatus::Oom |
| `internalerror` | triangle.c:349 | `error.rs` | same name (snake_case) | S1 shell | funnel: no process exit, no manual alloc/free (D8, D11); trimalloc OOM becomes InternalStatus::Oom |
| `parsecommandline` | triangle.c:364 | `options.rs` | same name (snake_case) | S1 shell | Options::defaults and Options::parse; derived-field tail transliterated bit-exact |
| `printtriangle` | triangle.c:574 | `debug.rs` | same name (snake_case) | S1 | |
| `printsubseg` | triangle.c:660 | `debug.rs` | same name (snake_case) | S1 | |
| `poolzero` | triangle.c:746 | `pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): dissolves into WordPool construction/Drop (D11, D4); landed in PR #23 |
| `poolrestart` | triangle.c:774 | `pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): restart; landed in PR #23 |
| `poolinit` | triangle.c:814 | `pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): WordPool::new; landed in PR #23 |
| `pooldeinit` | triangle.c:852 | `pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): dissolves into WordPool construction/Drop (D11, D4); landed in PR #23 |
| `poolalloc` | triangle.c:867 | `pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): alloc; landed in PR #23 |
| `pooldealloc` | triangle.c:924 | `pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): dealloc; landed in PR #23 |
| `traversalinit` | triangle.c:940 | `pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): traversal; landed in PR #23 |
| `traverse` | triangle.c:970 | `pool.rs` | per ADR 0003 §3 | S1 | naming authority is ADR 0003 §3 (`signatures are the contract`): traverse_next; landed in PR #23 |
| `dummyinit` | triangle.c:1029 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `initializevertexpool` | triangle.c:1100 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `initializetrisubpools` | triangle.c:1136 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `triangledealloc` | triangle.c:1196 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `triangletraverse` | triangle.c:1210 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit); landed in PR #35 (the reconstruct consumer) |
| `subsegdealloc` | triangle.c:1229 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `subsegtraverse` | triangle.c:1243 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit); landed in PR #35 (the reconstruct consumer) |
| `vertexdealloc` | triangle.c:1262 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `vertextraverse` | triangle.c:1276 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit); vertextraverse landed in PR #33 (the divconqdelaunay consumer) |
| `badsubsegdealloc` | triangle.c:1298 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `badsubsegtraverse` | triangle.c:1316 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `getvertex` | triangle.c:1343 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit); landed in PR #35 (the reconstruct consumer) |
| `triangledeinit` | triangle.c:1376 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `maketriangle` | triangle.c:1414 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `makesubseg` | triangle.c:1450 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `triangleinit` | triangle.c:1482 | `mesh.rs` | same name (snake_case) | S1 | D3 sentinel rows here (dummyinit) |
| `randomnation` | triangle.c:1521 | `util.rs` | same name (snake_case) | S1 | seed moves from _Thread_local to a per-run owned field (D10); landed in PR #30 |
| `checkmesh` | triangle.c:1537 | `quality.rs` | same name (snake_case) | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion |
| `checkdelaunay` | triangle.c:1622 | `quality.rs` | same name (snake_case) | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion |
| `enqueuebadtriang` | triangle.c:1714 | `quality.rs` | same name (snake_case) | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion |
| `enqueuebadtri` | triangle.c:1810 | `quality.rs` | same name (snake_case) | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion |
| `dequeuebadtriang` | triangle.c:1835 | `quality.rs` | same name (snake_case) | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion |
| `inlens` | triangle.c:1878 | `quality.rs` | same name (snake_case) | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion |
| `checkseg4encroach` | triangle.c:1925 | `quality.rs` | same name (snake_case) | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion |
| `testtriangle` | triangle.c:2001 | `quality.rs` | same name (snake_case) | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion |
| `makevertexmap` | triangle.c:2206 | `quality.rs` | same name (snake_case) | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion; landed in PR #35 (with the formskeleton consumer; the rest of quality.rs lands with refine) |
| `preciselocate` | triangle.c:2291 | `locate.rs` | same name (snake_case) | S1 | landed in PR #30 |
| `locate` | triangle.c:2427 | `locate.rs` | same name (snake_case) | S1 | landed in PR #30 |
| `insertsubseg` | triangle.c:2573 | `transform.rs` | same name (snake_case) | S1 | landed in PR #30 |
| `flip` | triangle.c:2664 | `transform.rs` | same name (snake_case) | S1 | landed in PR #30 |
| `unflip` | triangle.c:2785 | `transform.rs` | same name (snake_case) | S1 | landed in PR #30 |
| `insertvertex` | triangle.c:2920 | `transform.rs` | same name (snake_case) | S1 | landed in PR #30 |
| `triangulatepolygon` | triangle.c:3559 | `transform.rs` | same name (snake_case) | S1 | landed in PR #30 |
| `deletevertex` | triangle.c:3638 | `transform.rs` | same name (snake_case) | S1 | landed in PR #30 |
| `undovertex` | triangle.c:3730 | `transform.rs` | same name (snake_case) | S1 | landed in PR #30 |
| `vertexsort` | triangle.c:3870 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #33 |
| `vertexmedian` | triangle.c:3937 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #33 |
| `alternateaxes` | triangle.c:4006 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #33 |
| `mergehulls` | triangle.c:4062 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #33 |
| `divconqrecurse` | triangle.c:4366 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #33 |
| `removeghosts` | triangle.c:4495 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #33 |
| `divconqdelaunay` | triangle.c:4547 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #33 |
| `boundingbox` | triangle.c:4620 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #33 (out-of-pool infvertices per D12) |
| `removebox` | triangle.c:4672 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #33 (out-of-pool infvertices per D12) |
| `incrementaldelaunay` | triangle.c:4761 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 |
| `eventheapinsert` | triangle.c:4800 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 |
| `eventheapify` | triangle.c:4833 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 |
| `eventheapdelete` | triangle.c:4881 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 |
| `createeventheap` | triangle.c:4916 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 (event arena transliterates as an owned Vec + `EventPtr` free list) |
| `rightofhyperbola` | triangle.c:4946 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 (carries `hyperbolacount`) |
| `circletop` | triangle.c:4977 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 (carries `circletopcount`) |
| `check4deadevent` | triangle.c:5001 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 |
| `splay` | triangle.c:5024 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 |
| `splayinsert` | triangle.c:5134 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 (allocates from the `splaynodes` WordPool) |
| `circletopinsert` | triangle.c:5161 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 |
| `frontlocate` | triangle.c:5189 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 |
| `sweeplinedelaunay` | triangle.c:5212 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 |
| `delaunay` | triangle.c:5429 | `delaunay.rs` | same name (snake_case) | S1 | landed in PR #34 (dispatcher: incremental/sweepline/divconq per the landed `Behavior` flags) |
| `reconstruct` | triangle.c:5483 | `construct.rs` | same name (snake_case) | S1 | status out-params mirror InternalStatus until rung 2 (D8); landed in PR #35 |
| `finddirection` | triangle.c:5776 | `construct.rs` | same name (snake_case) | S1 | status out-params mirror InternalStatus until rung 2 (D8); landed in PR #35 |
| `segmentintersection` | triangle.c:5873 | `construct.rs` | same name (snake_case) | S1 | status out-params mirror InternalStatus until rung 2 (D8); landed in PR #35 |
| `scoutsegment` | triangle.c:6003 | `construct.rs` | same name (snake_case) | S1 | status out-params mirror InternalStatus until rung 2 (D8); landed in PR #35 |
| `conformingedge` | triangle.c:6089 | `construct.rs` | same name (snake_case) | S1 | status out-params mirror InternalStatus until rung 2 (D8); landed in PR #35 |
| `delaunayfixup` | triangle.c:6201 | `construct.rs` | same name (snake_case) | S1 | status out-params mirror InternalStatus until rung 2 (D8); landed in PR #35 |
| `constrainededge` | triangle.c:6312 | `construct.rs` | same name (snake_case) | S1 | status out-params mirror InternalStatus until rung 2 (D8); landed in PR #35 |
| `insertsegment` | triangle.c:6420 | `construct.rs` | same name (snake_case) | S1 | status out-params mirror InternalStatus until rung 2 (D8); landed in PR #35 |
| `markhull` | triangle.c:6517 | `construct.rs` | same name (snake_case) | S1 | status out-params mirror InternalStatus until rung 2 (D8); landed in PR #35 |
| `formskeleton` | triangle.c:6554 | `construct.rs` | same name (snake_case) | S1 | status out-params mirror InternalStatus until rung 2 (D8); landed in PR #35 |
| `infecthull` | triangle.c:6642 | `construct.rs` | same name (snake_case) | S1 | status out-params mirror InternalStatus until rung 2 (D8); landed in PR #35 |
| `plague` | triangle.c:6714 | `construct.rs` | same name (snake_case) | S1 | status out-params mirror InternalStatus until rung 2 (D8); landed in PR #35 |
| `regionplague` | triangle.c:6886 | `construct.rs` | same name (snake_case) | S1 | status out-params mirror InternalStatus until rung 2 (D8); landed in PR #35 |
| `carveholes` | triangle.c:6964 | `construct.rs` | same name (snake_case) | S1 | status out-params mirror InternalStatus until rung 2 (D8); landed in PR #35 |
| `tallyencs` | triangle.c:7129 | `refine.rs` | same name (snake_case) | S1 | highorder moves from the C Array I/O tail for cohesion |
| `precisionerror` | triangle.c:7154 | `refine.rs` | same name (snake_case) | S1 | highorder moves from the C Array I/O tail for cohesion |
| `splitencsegs` | triangle.c:7183 | `refine.rs` | same name (snake_case) | S1 | highorder moves from the C Array I/O tail for cohesion |
| `tallyfaces` | triangle.c:7383 | `refine.rs` | same name (snake_case) | S1 | highorder moves from the C Array I/O tail for cohesion |
| `splittriangle` | triangle.c:7409 | `refine.rs` | same name (snake_case) | S1 | highorder moves from the C Array I/O tail for cohesion |
| `enforcequality` | triangle.c:7527 | `refine.rs` | same name (snake_case) | S1 | highorder moves from the C Array I/O tail for cohesion |
| `highorder` | triangle.c:7624 | `refine.rs` | same name (snake_case) | S1 | highorder moves from the C Array I/O tail for cohesion |
| `transfernodes` | triangle.c:7697 | `ioarrays.rs` | same name (snake_case) | S1 | transfernodes is the stage-1 input path; the writers feed Mesh::output (writers land late stage 1) |
| `writenodes` | triangle.c:7768 | `ioarrays.rs` | same name (snake_case) | S1 | transfernodes is the stage-1 input path; the writers feed Mesh::output (writers land late stage 1) |
| `numbernodes` | triangle.c:7840 | `ioarrays.rs` | same name (snake_case) | S1 | transfernodes is the stage-1 input path; the writers feed Mesh::output (writers land late stage 1) |
| `writeelements` | triangle.c:7863 | `ioarrays.rs` | same name (snake_case) | S1 | transfernodes is the stage-1 input path; the writers feed Mesh::output (writers land late stage 1) |
| `writepoly` | triangle.c:7933 | `ioarrays.rs` | same name (snake_case) | S1 | transfernodes is the stage-1 input path; the writers feed Mesh::output (writers land late stage 1) |
| `writeedges` | triangle.c:7983 | `ioarrays.rs` | same name (snake_case) | S1 | transfernodes is the stage-1 input path; the writers feed Mesh::output (writers land late stage 1) |
| `writeneighbors` | triangle.c:8054 | `ioarrays.rs` | same name (snake_case) | S1 | transfernodes is the stage-1 input path; the writers feed Mesh::output (writers land late stage 1) |
| `quality_statistics` | triangle.c:8143 | `quality.rs` | same name (snake_case) | S1 | testtriangle carries the Unsuitable callback site (D9); quality_statistics moves from the C Array I/O tail for cohesion |
### `acute.c` (src/Triangle/acute.c)

| C function | C site | Rust module | Rust symbol (stage 1) | Stage | Notes |
| --- | --- | --- | --- | --- | --- |
| `acutepool_init` | acute.c:22 | `acute.rs` | same name | S1 | consumes the #9 pool interface (ADR 0002 §4 item 8) |
| `acutepool_resize` | acute.c:45 | `acute.rs` | same name | S1 | consumes the #9 pool interface (ADR 0002 §4 item 8) |
| `acutepool_deinit` | acute.c:64 | `acute.rs` | same name | S1 | consumes the #9 pool interface (ADR 0002 §4 item 8) |
| `findNewSPLocation` | acute.c:95 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `findNewSPLocationWithoutMaxAngle` | acute.c:113 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `findNewSPLocationWithMaxAngle` | acute.c:722 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `longestShortestEdge` | acute.c:1816 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `doSmoothing` | acute.c:1858 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `getStarPoints` | acute.c:2040 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `getNeighborsVertex` | acute.c:2121 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `getWedgeIntersectionWithoutMaxAngle` | acute.c:2250 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `getWedgeIntersectionWithMaxAngle` | acute.c:2490 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `polygonAngles` | acute.c:2918 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `testPolygonAngle` | acute.c:2944 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `lineLineIntersection` | acute.c:2983 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `halfPlaneIntersection` | acute.c:3032 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `splitConvexPolygon` | acute.c:3096 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `linePointLocation` | acute.c:3255 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `lineLineSegmentIntersection` | acute.c:3279 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `findPolyCentroid` | acute.c:3330 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `circleLineIntersection` | acute.c:3354 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `chooseCorrectPoint` | acute.c:3411 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `pointBetweenPoints` | acute.c:3458 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `testTriangleAngle` | acute.c:3493 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
| `minDistanceToNeigbor` | acute.c:3584 | `acute.rs` | same name | S1 | whole module behind cfg(feature = "acute") (default on = NO_ACUTE inverse); C camelCase names stay until rung 4 |
### `predicates.c` (src/Triangle/predicates.c)

| C function | C site | Rust module | Rust symbol (stage 1) | Stage | Notes |
| --- | --- | --- | --- | --- | --- |
| `tri_exactinit` | predicates.c:173 | `predicates.rs` | same name (snake_case) | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_fast_expansion_sum_zeroelim` | predicates.c:246 | `predicates.rs` | same name (snake_case) | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_scale_expansion_zeroelim` | predicates.c:330 | `predicates.rs` | same name (snake_case) | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_estimate` | predicates.c:377 | `predicates.rs` | same name (snake_case) | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_counterclockwiseadapt` | predicates.c:409 | `predicates.rs` | same name (snake_case) | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_counterclockwise` | predicates.c:489 | `predicates.rs` | same name (snake_case) | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_incircleadapt` | predicates.c:548 | `predicates.rs` | same name (snake_case) | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_incircle` | predicates.c:1117 | `predicates.rs` | same name (snake_case) | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_orient3dadapt` | predicates.c:1188 | `predicates.rs` | same name (snake_case) | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_orient3d` | predicates.c:1598 | `predicates.rs` | same name (snake_case) | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_nonregular` | predicates.c:1666 | `predicates.rs` | same name (snake_case) | S1 | pure Rust port; x87 control-word code dropped (D5) |
| `tri_findcircumcenter` | predicates.c:1696 | `predicates.rs` | same name (snake_case) | S1 | pure Rust port; x87 control-word code dropped (D5) |
### `eps_writer.c` (src/Triangle/eps_writer.c)

| C function | C site | Rust module | Rust symbol (stage 1) | Stage | Notes |
| --- | --- | --- | --- | --- | --- |
| `eps_write_header` | eps_writer.c:14 | `eps.rs` | same name (snake_case) | IO | |
| `eps_write_trailer` | eps_writer.c:32 | `eps.rs` | same name (snake_case) | IO | |
| `eps_draw_clip` | eps_writer.c:43 | `eps.rs` | same name (snake_case) | IO | |
| `eps_draw_edges` | eps_writer.c:87 | `eps.rs` | same name (snake_case) | IO | |
| `eps_draw_segments` | eps_writer.c:132 | `eps.rs` | same name (snake_case) | IO | |
| `eps_draw_points` | eps_writer.c:161 | `eps.rs` | same name (snake_case) | IO | |
| `eps_set_color` | eps_writer.c:192 | `eps.rs` | same name (snake_case) | IO | |
| `eps_set_stroke` | eps_writer.c:198 | `eps.rs` | same name (snake_case) | IO | |
| `eps_update_metrics` | eps_writer.c:205 | `eps.rs` | same name (snake_case) | IO | |
### `triangle_helper.c` (src/Triangle/triangle_helper.c)

| C function | C site | Rust module | Rust symbol (stage 1) | Stage | Notes |
| --- | --- | --- | --- | --- | --- |
| `triangle_check_context` | triangle_helper.c:5 | `-` | same name (snake_case) | - | dissolves: owned values cannot be null (D8) |
| `triangle_check_behavior` | triangle_helper.c:17 | `options.rs` | same name (snake_case) | S1 shell | Options validation |
| `triangle_check_triangleio` | triangle_helper.c:25 | `input.rs` | same name (snake_case) | S1 shell | MeshInput validation; restore markers is an internal step of the output path |
| `triangle_restore_pointmarkers` | triangle_helper.c:77 | `input.rs` | same name (snake_case) | S1 shell | MeshInput validation; restore markers is an internal step of the output path |
### `triangle_io.c` (src/Triangle/triangle_io.c)

| C function | C site | Rust module | Rust symbol (stage 1) | Stage | Notes |
| --- | --- | --- | --- | --- | --- |
| `file_writenodes` | triangle_io.c:23 | `io.rs` | same name (snake_case) | IO | |
| `file_writeelements` | triangle_io.c:78 | `io.rs` | same name (snake_case) | IO | |
| `file_writepoly` | triangle_io.c:133 | `io.rs` | same name (snake_case) | IO | |
| `file_writeedges` | triangle_io.c:203 | `io.rs` | same name (snake_case) | IO | |
| `file_writeneighbors` | triangle_io.c:264 | `io.rs` | same name (snake_case) | IO | |
| `file_write_eps` | triangle_io.c:312 | `io.rs` | same name (snake_case) | IO | |
| `readline` | triangle_io.c:370 | `io.rs` | same name (snake_case) | IO | |
| `findfield` | triangle_io.c:401 | `io.rs` | same name (snake_case) | IO | |
| `file_readnodes_internal` | triangle_io.c:432 | `io.rs` | same name (snake_case) | IO | |
| `file_readsegments_internal` | triangle_io.c:546 | `io.rs` | same name (snake_case) | IO | |
| `file_readholes_internal` | triangle_io.c:616 | `io.rs` | same name (snake_case) | IO | |
| `file_readnodes` | triangle_io.c:712 | `io.rs` | same name (snake_case) | IO | |
| `file_readpoly` | triangle_io.c:723 | `io.rs` | same name (snake_case) | IO | |
| `file_readelements` | triangle_io.c:744 | `io.rs` | same name (snake_case) | IO | |
| `file_readelementsarea` | triangle_io.c:828 | `io.rs` | same name (snake_case) | IO | |
### `triangle_api.c` (src/Triangle/triangle_api.c)

| C function | C site | Rust module | Rust symbol (stage 1) | Stage | Notes |
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
| `triangle_mesh_load` | triangle_api.c:262 | `api.rs` | Mesh::load_mesh | S1 shell | |
| `triangle_mesh_refine` | triangle_api.c:332 | `api.rs` | Mesh::refine | S1 shell | |
| `triangle_mesh_copy` | triangle_api.c:362 | `api.rs` | Mesh::output | S1 shell | |
| `triangle_free` | triangle_api.c:430 | `-` | same name (snake_case) | - | dissolves: Drop/RAII (D11) |
| `triangle_write_nodes` | triangle_api.c:437 | `io.rs` | same name (snake_case) | IO | serializes MeshOutput (formats pinned by the I/O-phase ticket) |
| `triangle_write_elements` | triangle_api.c:446 | `io.rs` | same name (snake_case) | IO | serializes MeshOutput (formats pinned by the I/O-phase ticket) |
| `triangle_write_poly` | triangle_api.c:455 | `io.rs` | same name (snake_case) | IO | serializes MeshOutput (formats pinned by the I/O-phase ticket) |
| `triangle_write_edges` | triangle_api.c:466 | `io.rs` | same name (snake_case) | IO | serializes MeshOutput (formats pinned by the I/O-phase ticket) |
| `triangle_write_neighbors` | triangle_api.c:475 | `io.rs` | same name (snake_case) | IO | serializes MeshOutput (formats pinned by the I/O-phase ticket) |
| `triangle_write_eps` | triangle_api.c:484 | `io.rs` | same name (snake_case) | IO | serializes MeshOutput (formats pinned by the I/O-phase ticket) |
| `triangle_read_nodes` | triangle_api.c:493 | `io.rs` | same name (snake_case) | IO | fills MeshInput (formats pinned by the I/O-phase ticket) |
| `triangle_read_poly` | triangle_api.c:509 | `io.rs` | same name (snake_case) | IO | fills MeshInput (formats pinned by the I/O-phase ticket) |
| `triangle_read_elements` | triangle_api.c:525 | `io.rs` | same name (snake_case) | IO | fills MeshInput (formats pinned by the I/O-phase ticket) |
| `triangle_read_area` | triangle_api.c:541 | `io.rs` | same name (snake_case) | IO | fills MeshInput (formats pinned by the I/O-phase ticket) |

## Types and macros (triangle.h and friends)

| C construct | C site | Rust location | Notes |
| --- | --- | --- | --- |
| `encode()` / `decode()` | triangle.h:703-708 | `handles.rs` | tagged pointers become packed u64 handles (D1) |
| `sencode()` / `sdecode()` | triangle.h:703-708 | `handles.rs` | subsegments pack 1 bit (D1) |
| handle macro family (`sym`, `lnext`, `lprev`, `onext`, `oprev`, `dnext`, `dprev`, `rnext`, `rprev`, `bond`, `dissolve`, `tspivot`, `stpivot`, `tsbond`, `tsdissolve`, `stdissolve`, ...) | triangle.h:703-1090 | `handles.rs` (pure rotations), `slots.rs` (storage-touching) | macro-method mirror; hidden ptr/sptr locals disappear (D6) |
| `org()` / `dest()` / `apex()` | triangle.h:703-1090 | `slots.rs` | vertex-id reads, no references escape |
| `vertexmark` / `vertextype` / `vertex2tri` accessors | triangle.h:1077-1088 | `slots.rs` | int overlays / encoded word behind the chokepoint (D2) |
| `struct behavior` | triangle.h | `options.rs` | public `Options` mirror per switch family |
| `struct mesh` | triangle.h | `mesh.rs` (struct + lifecycle) + `slots.rs` (accessor chokepoint) | pool ownership per ADR 0002; storage share corrected per the ADR 0002 amendment (2026-11-24) |
| `struct memorypool` | triangle.c | `pool.rs` | ticket #9 per the ADR 0002 §4 contract (D4) |
| `struct quality` / `struct statistics` / `struct rect` | triangle_api.h / triangle.h | `api.rs` | public `Quality`, `Statistics`, `BoundingBox` |
| `TRI_*` status codes | triangle_config.h:28-40 | `error.rs` | `InternalStatus` mirror + `Error` funnel (D8) |
| `enumerations` (`locateresult`, `insertvertexresult`, `finddirectionresult`, ...) | triangle.h | the owning module of the function that returns them | transliterated verbatim |
| `triunsuitable_user_func` (typedef) | triangle.h | `options.rs` | `Unsuitable` trait + `VertexSample` values (D9) |
| `FILENAMESIZE`, `TRINAME`-style string helpers | triangle_io.c / util.c of the example CLI | `io.rs` / `triangle-cli` | I/O phase |


## Maintenance rules

1. Every new C function that a stage-1 ticket transliterates gets its row
   updated (stage stays, the row gains the landing PR reference). The C core
   is frozen for this port, so the table is append-only in practice.
2. A function that moves during rungs 3-4 gets both rows: the original
   assignment and the rung note with the new location.
3. The extractor for this table lives in the ticket #10 session transcript;
   regenerating it is a grep, not a hand-edit: names and sites come from the
   sources.


