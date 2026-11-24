# UB and implementation-detail inventory of the C core

Findings for wayfinder ticket #4. Date: 2026-11-23. Base: master @ 451c932. Line refs relative to src/Triangle.

## 1. Tagged pointers: the actual mechanism

- decode()/encode(): triangle.h:703-708. Orientation in the 2 low bits (mask 3l). Subsegments: sencode()/sdecode() with 1 bit. This is the trick the user described: pool addresses end in b00, and those bits are repurposed.
- Invariant: items come from struct memorypool. poolinit(..., alignment=4) for triangles and subsegs (triangle.c:1172, 1179), but poolinit raises alignbytes to max(alignment, sizeof(VOID*)) (triangle.c:817-822), which is 8 on x64. Every item address ends in b000 (three zero bits); two bits are repurposed for triangle orientation, one for subsegment orientation. One spare bit remains on x64.
- Pointer/integer casts go through ULONG_PTR (unsigned long long on 64-bit, including Win64 LLP64; triangle_config.h:42-47). Shewchuk calls the alignment and word-size assumptions "two of the few kludges in this program" (triangle.h:597-602).
- Consumers (all in triangle.h:703-1090): sym/symself (neighbor slot tri[orient]), lnext/lprev and self variants (pure orient arithmetic), onext/oprev/dnext/dprev/rnext/rprev, bond/dissolve, tspivot/stpivot/tsbond/tsdissolve/stdissolve (subsegment slots tri[6+orient]). org/dest/apex read the untagged vertex slots tri[3..5].
- Macro hygiene hazard: sym(), decode() and friends require caller-declared locals named ptr (triangle) and sptr (subseg) in every using function; macros evaluate arguments multiple times. In Rust these become methods on named handle types and the hidden local disappears.
- Sentinels: dummytri ("outer space") and dummysub (omnipresent subsegment) are allocated manually via trimalloc plus explicit alignbytes padding inside dummyinit (triangle.c, search dummyinit): their tag-ability is guaranteed by hand, not by a pool. dummytri[0..2] point at dummytri itself (tag 0), dummytri[3..5] are NULL vertices; with segments, dummytri[6..8] = dummysub, dummysub[0..1] self, dummysub[6..7] = dummytri, and the subseg boundary marker is an int cast at slot 8.
- acute.c adds no tagged-pointer logic of its own; it consumes the handle macros. Its "orientation" variables (acute.c:134-363 and parallels) are edge-ranking labels, unrelated to the pointer tags.

## 2. Vertex layout: mixed-type array behind REAL*

vertex == REAL* (triangle.h:266). Storage: (mesh_dim + nextras) REALs of coordinates/attributes, then an int marker, an int type, and (with -p) a triangle pointer for vertex2tri. Offsets computed at runtime in triangle.c:1105-1118; vertex2triindex must land pointer-aligned. Access is via (int*) and (triangle*) casts on the REAL* base (triangle.h:1077-1088): strict-aliasing UB that works because pool memory has no declared type.

## 3. Triangle/subsegment layout: REAL** arrays with runtime indices

triangle == REAL** (triangle.h:231): t[0..2] neighbor triangles (tagged), t[3..5] vertices, t[6..8] subsegments (tagged, only with -p), then optional payload: high-order midpoints, element attributes, area bound (indices highorderindex/elemattribindex/areaboundindex, triangle.c:1140-1170), an int numbering slot for the -n neighbors output, and subseg boundary markers. Implementation-defined reliance: pointers stored in REAL-sized slots, exact on x64 (8 = 8); row size only known at runtime.

## 4. Memory pools and the freelist

struct memorypool hands out items contiguously; pooldealloc (triangle.c:924-928) pushes freed items onto deaditemstack, threading the list through the item's FIRST slot. Freed items therefore have slot 0 overwritten; traversals skip dead items (triangletraverse/vertextraverse/subsegtraverse, triangle.c:1206-1290). traversalinit/traverse (triangle.c:940-990) scan blocks linearly in allocation order; this is what determines output ordering.

## 5. Random number generator

randomnation() (triangle.c:1521-1525): LCG randomseed = (randomseed * 1366 + 150889) % 714025, returns seed / (714025 / choices + 1). Seeded to 1 per context (triangle.c:1502). Thread-local (reentrancy-safe, commit "Give each thread its own random seed"). All intermediates stay below 2^30, so the integer sequence is identical on any word width. Call sites: triangle.c:2503 (encroachment choice), 3889 and 3956 (quicksort pivot sampling during vertex sorting), 5305/5363/5365 (point-location random walk sampling, SAMPLERATE).

## 6. Control flow and error paths

triexit() still calls exit(1) (triangle.c:313-315), reached from allocation failure ("Out of memory", ~324) and internalerror (~354). Everything else at the API surface returns error codes. Map to Result errors at both sites.

## 7. Predicates (deep dive in ticket #3)

predicates.c:157-210: on LINUX, sets the x87 FPU control word via fpu_control.h _FPU_SETCW(4722) (double precision); on CPU86 via _control87. This is precision control for x87, not a rounding-mode switch; SSE2 double math on x86-64 is unaffected. epsilon/splitter are derived by runtime bisection, which self-validates FP behavior.

## 8. Checklist for the design tickets

1. Tagged pointer encode/decode: usize with addr()/from_exposed_addr equivalents; alignment invariant (>= 4-byte, effectively 8 from pools) as debug_assert at pool allocation.
2. otri/osub become explicit handle types; ptr/sptr hidden macro locals disappear.
3. Vertex/triangle/subsegment flat arrays with runtime index roles: keep transliterated, accessor functions centralize the raw casts.
4. Freelist-over-first-slot: reproduce in a transliteration; traversals must skip dead items exactly like the C code.
5. dummytri/dummysub: manually aligned sentinel allocation needs a special case in the Rust pool.
6. RNG: trivial u32-config LCG, identical sequence by construction.
7. exit(1) x2 becomes Result errors.
8. Bit-identity hazards with the C build (FMA contraction, -march) are ticket #5's findings.