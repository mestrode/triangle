# Memory pools: flat Vec word pools, C-logical block geometry, amortized growth

Status: accepted 2026-11-24; decided in the wayfinder ticket Memory management: replacing the memory pools (#9), on the #8 interface contract (ADR 0002, section 4), the #4 UB inventory (pool mechanics, docs/research/ub-inventory.md on branch research/ub-inventory), the #2 landscape findings (bump arenas ruled out; docs/research/rust-landscape.md on branch research/rust-landscape), and the #7 microbench data (flat Vec slot access already measured at C kernel speeds). Glossary terms added: row id, word pool, logical block geometry, oracle address order, acute scratch buffers.

## Decision inputs (settled, not re-derived here)

- The #8 contract (ADR 0002, section 4) is binding: Copy u32 row ids per pool; flat u64 word stores with a per-pool runtime stride fixed at init; O(1) word access by (row, slot); fresh ids monotonically increasing; traversal covering 0..high water, dead rows included, never rearranged (traversal order equals the oracle's output ordering); an exact LIFO dead stack per pool with slot 0 pool-private between dealloc and realloc; live count and high water (the `items`/`maxitems` shapes); one reserved sentinel id per triangle and subseg pool excluded from allocation, traversal, and the dead stack; row byte sizes multiples of 8; the same alloc/dealloc and LIFO semantics for the typed small pools and the aCute pool; nothing thread-affine (MT is #12; nothing may preclude it).
- The C reference (struct memorypool, triangle.h:352-366): poolinit rounds itembytes up to a multiple of alignbytes with alignbytes at least sizeof(VOID*) (triangle.c:812-826, so every row is whole 8-byte words); poolalloc pops the dead stack first, else hands the next fresh item from the current block, allocating fixed-size blocks linked in a chain when one is exhausted (triangle.c:867-915); maxitems counts fresh allocations only, never dead reuse and never drops on dealloc (triangle.c:914); pooldealloc pushes onto deaditemstack through the dying item's first slot (triangle.c:924-928); traversalinit/traverse walk items in allocation order, dead included, ending at the allocation frontier and extending if the pool grows mid-traversal (triangle.c:940-990); poolrestart logically resets a pool and retains all blocks (triangle.c:774-793); pooldeinit frees the block chain (triangle.c:852-860). Allocation failure prints "Error: Out of memory." and exits 1 via trimalloc/triexit (triangle.c:313-324), which becomes a Result error per #4. Block constants: TRIPERBLOCK 4092, SUBSEGPERBLOCK 508, VERTEXPERBLOCK 4092, VIRUSPERBLOCK 1020, BADSUBSEGPERBLOCK 252, BADTRIPERBLOCK 4092, FLIPSTACKERPERBLOCK 252, SPLAYNODEPERBLOCK 508 (triangle_config.h:120-131). Initial hints: vertices first block max(invertices, 4092) rows, alignment sizeof(REAL); triangles first block max(2*invertices-2, 4092) rows, alignment 4 (raised to 8 by poolinit); subsegs 508 rows, alignment 4 (triangle.c:1120, 1172-1180). The small pools init with first block equal to every block (triangle.c:5234, 6988, 7533-7566); flipstackers and viri are repeatedly poolrestart'ed during quality enforcement (triangle.c:3130, 3239, 6868); highorder deliberately clears the vertex dead stack (triangle.c:7639).
- The aCute pool is not an item pool at all: acutepool_init (acute.c:22-42) allocates eight REAL scratch buffers sized from an n=20 hint (petalx/y/r at 2n, wedges at 2n*20+40, initialpoly and points_p/q/r at 500); acutepool_resize (acute.c:45-62) frees and re-allocates the four n-dependent buffers only when n grows; contents are not preserved and every consumer refills before reading.
- The #7 microbench already measured Vec-backed word access and traversal for the packed-u64 variant (decode 1.84 / encode 2.49 ns/op, both best of bench): slot access cost is covered; this ticket is a storage and growth decision, not an access-speed one. No new bench is added.
- New finding from this ticket (not in #8's list): raw-pointer ORDER comparisons in the C core are output-observable, not just an internal detail. `triangleloop.tri < oppotri.tri` decides which triangle of an interior pair reports an edge in the edge-list output (triangle.c:8021, triangle_io.c:232, eps_writer.c:117), deduplicates the Delaunay self-check (triangle.c:1655), and orders the high-order edge walk (triangle.c:7653). The corpus family "pen" (-pen, .edge and .neigh outputs) covers the output-visible path. Row ids mirror C trigger order (same block, ascending row) but C trigger addresses also depend on the C allocator: with glibc, blocks larger than the 128 KiB mmap threshold (every triangle-pool block: 4092 x 48-72 bytes plus header) are placed by the kernel top-down, so consecutive blocks get DESCENDING addresses. Verified empirically this session on the pinned build server (Debian 12, glibc 2.36, x86-64): twelve consecutive 196,432-byte mallocs interleaved with in-arena-sized requests produced strictly descending block addresses (11/11 DESC; sub-threshold requests stay on the brk heap and do not interfere). Within a block, addresses ascend with allocation order and blocks are page-separated (196,608 >= 196,432 + 16 bytes; likewise 294,912 >= 294,640 + 16 with -p), so the oracle's total trigger order is: block index descending, then row index ascending. Id comparison alone would NOT reproduce it.

## Decision

### 1. Pool shape: one WordPool type, a flat Vec<u64> per pool with a fixed row stride

All eight mesh pools (triangles, subsegs, vertices, viri, badsubsegs, badtriangles, flipstackers, splaynodes) are instances of one type:

    pub(crate) struct WordPool {
        words:   Vec<u64>,     // flat store, row-major: word i of row r at words[r*stride + i]
        stride:  u32,          // words per row, fixed at init (row bytes = stride*8)
        first_block_items: u32, // C itemsfirstblock, logical (see section 2)
        items_per_block:   u32, // C itemsperblock, logical (see section 2)
        dead_head: u32,        // id of most recently deallocated row; 0 terminates (0 = null id)
        items:    u64,          // C items: live count
        maxitems: u64,          // C maxitems: fresh-allocation high water since init/restart
        has_sentinel: bool,     // true only for the triangle and subseg pools (sentinel = id 1)
    }

Ids are indices into this store, so growth never invalidates them (ADR 0002 section 4.2). Row addressing is base + id*stride + slot: one multiply, one add, one bounds-checked index - the access shape #7 measured. Zero unsafe anywhere: bounds-checked indexing, Copy ids, no interior references escaping (the #8 chokepoint guarantees).

Why not the byte-close custom arena (C's block list, alignment padding, freelist): a block list turns every (row, slot) access into two-level indexing (block pointer load plus divmod), and the accessor chokepoint is the entire port's hot path - the #7 numbers were measured on flat Vecs, and paying an extra dependent load plus block arithmetic per slot access in sym/decode/encode streams risks the 1.05 parity band for no behavioral gain, since behavior (order, reuse, traversal) is already fully reproduced by ids. It also roughly doubles the implementation surface that stage-1 transliteration must keep honest (block headers, alignment slack, block-chain bugs), for a model whose only observable advantages are memory-transient freedom and zero word copying at growth. Rejected for stage 1; recorded as the fallback shape (see Considered alternatives) if the perf gate ever implicates realloc spikes.

Why not bumpalo/typed-arena (#2): no reclamation, dead reuse impossible; ruled out already. Standard per-object allocation (Box): no index dense enough for the 8-byte handles, no controlled reuse order, no allocation-order traversal; ruled out. Slab/slotmap with generation bits: adds generations the algorithm does not need and breaks the 8-byte slot density; ruled out with ADR 0002.

### 2. Growth policy and the memory model

Capacity schedule:

- Initial capacity mirrors C's first block (the #8 memory-parity hint, passed through init): vertices reserve max(invertices, VERTEXPERBLOCK) rows, triangles max(2*invertices-2, TRIPERBLOCK) rows, subsegs SUBSEGPERBLOCK, each small pool its constant. This is capacity only; no output-visible or traversal-visible state depends on it.
- Growth is Vec's amortized reallocation (capacity roughly doubling), explicitly NOT C's fixed-size block steps. Mirroring C's step size on a flat store means the store is reallocated once per block, and each reallocation moves the whole store: aggregate copying is the arithmetic series N^2 / (2*itemsperblock) words - for a 10M-row triangle store on the order of hundreds of GB of copying, tens of seconds at memcpy speed, against a workload budget the 1.05 gate cannot absorb. C pays nothing of this because its blocks are separate allocations. Amortized doubling keeps the aggregate at O(N) and, on glibc, reallocations at or above the 128 KiB mmap threshold are mremap page-table moves (no content copy, RSS carried by the same physical pages).
- poolrestart() is mirrored exactly: logical reset (items = maxitems = 0, dead stack empty) with capacity retained; fresh ids are handed from the bottom again, like C handing the same block addresses again. The dead-stack-first allocation order means the reuse sequence stays byte-identical.

Memory-retention model per option (worst case allocated vs high-water live), stated per the ticket:

| Option | Steady slack over high water | Growth transient | Through poolrestart | Released |
|---|---|---|---|---|
| C block chain (reference) | < 1 block (4092 rows; for the triangle pool ~192/288 KiB, ~2% of a large mesh) | none | retained, reused | at pooldeinit only |
| Flat Vec, C-step growth (rejected) | < 1 block | store move per block; aggregate O(N^1.5) copying | retained | at deinit |
| Flat Vec, amortized doubling (chosen) | up to 2x high water (typical 1.3-1.5x) | old + new mappings during one move per doubling (virtual 3x at the worst moment; RSS unchanged, no copy for pool-sized stores on glibc) | retained | at deinit |

The chosen model pays worst-case 2x of the word store in allocator-visible capacity where C pays under one block. The trade is bought with the flat store's hot-path zero-indirection (measured) and a total growth cost of O(N), and it tilts exactly where the ticket criteria order them: real footprint loss is bounded (the 2x applies to the word store only, and only at settled mesh sizes just above a doubling boundary), fragmentation is identical to C's (all retention is at the pool level; neither design returns memory below the pool's persistence), dead-reuse byte identity is untouched, and the stride-fixed addressing from #8 wants one flat allocation. Deviation-register entry: the physical growth schedule diverges from C (not output-visible; not memory-parity-critical beyond the initial hint). Revisit rights recorded at the end.

OOM is mirrored as an error, not an abort: fresh-capacity growth goes through Vec::try_reserve; failure returns the error the trimalloc/triexit site (triangle.c:318-324) maps to at the API shell (ADR 0002, #4). Dead-stack pops and in-capacity fresh allocations cannot fail, matching C.

### 3. Pool API surface (the #8 chokepoint implements against this; signatures are the contract)

Ids and sentinels. 0 is the null id (C's NULL / the all-zero slot word, ADR 0002 section 1). The triangle and subseg pools reserve id 1 for the sentinel row (dummytri/dummysub; excluded from allocation, traversal, and the dead stack; initialized and bonded by the dummyinit mirror). Fresh ids in those pools run from 2; in all other pools from 1. High water counts fresh rows exactly like C's maxitems (which never counted the sentinels, being outside the pool).

    impl WordPool {
        // init: byte_count rounded up to whole 8-byte words is the stride (C's alignbytes rounding,
        // triangle.c:812-826; strides: triangle 6..9+ words by options/order, subseg 9, vertex
        // (mesh_dim + nextras) + 1 word (+1 with -p), viri 1, flipstacker 2, badsubseg 3,
        // badtriang 6, splaynode 5); first_block_items/items_per_block carry the C
        // logical geometry; the initial Vec reservation equals first_block_items rows.
        fn new(byte_count: u32, items_per_block: u32, first_block_items: u32,
               sentinel: bool) -> Result<WordPool, TriangleError>;
        fn sentinel_id(&self) -> Option<u32>;                    // Some(1) for tri/sub pools
        // allocation and deallocation (C poolalloc/pooldealloc semantics, triangle.c:867-928)
        fn alloc(&mut self) -> Result<u32, TriangleError>;       // pops dead (LIFO), else next fresh id;
                                                                // Err = Out of memory (growth via try_reserve)
        fn dealloc(&mut self, id: u32);                          // pushes onto the dead stack, threading the
                                                                // previous dead_head id through slot 0 of id
        // O(1) word access (the chokepoint's innermost accessors)
        #[inline] fn word(&self, id: u32, slot: usize) -> u64;
        #[inline] fn set_word(&mut self, id: u32, slot: usize, w: u64);
        // logical reset, capacity retained (C poolrestart, triangle.c:774-793)
        fn restart(&mut self);
        // counts (C items/maxitems shapes; sampleraise and the output list sizing read these)
        fn live_count(&self) -> u64;
        fn high_water(&self) -> u64;
        // logical block geometry (C itemsfirstblock/itemsperblock): sampleraise
        // (triangle.c:2474-2500) and getvertex (triangle.c:1348-1369) formulas need them;
        // independent of physical capacity
        fn first_block_items(&self) -> u32;
        fn block_items(&self) -> u32;
        fn block_base_id(&self, block: u64) -> u32;              // id of index 0 of block `block`,
                                                                 // sentinel shift included
        // traversal (C traversalinit/traverse, triangle.c:940-990): ascending fresh ids,
        // from each pool's first fresh id through the current high water, dead rows
        // included, sentinel skipped; the frontier is read live at each step so a pool
        // growing mid-traversal extends the walk exactly like C's pathitem/nextitem
        // comparison (a traversal started before growth ends at the new frontier)
        fn traversal(&self) -> Traversal { .. }                   // cursor: next id
        fn traverse_next(&self, t: &mut Traversal) -> Option<u32>;
        // C pointer-order comparison mirror (see section 5)
        fn oracle_address_order(&self, a: u32, b: u32) -> Ordering;
    }

Dead-stack state is `dead_head` in the pool plus the next-id link in slot 0 of dead rows (ids where plain C kept pointers); slot 0 stays pool-private between dealloc and realloc. The pool stays opaque to everything else: payload meanings, the killtri dead-detection pattern in slots 1 and 3, handle encode/decode, send encode vs trigger, all data-model matters live at the chokepoint (#8, ADR 0002). Debug-assert inventory pool-side (rung 1): slot < stride; id within the pool's handed range; alloc density (a fresh id is exactly one above the previous frontier); double-walk the dead chain to catch duplicates and loops; sentinel never handed to alloc, traversal, or the dead stack.

### 4. The small pools and the aCute pool

The five small pools and their word strides mirror C itembyte roundings exactly (struct shapes: triangle.h:271-274, 280-285, 291-294, 323-327):

| Pool | C item | Bytes | Stride (u64) | Blocks | Notes |
|---|---|---|---|---|---|
| viri | triangle* | 8 | 1 | 1020 | poolrestart repeatedly (plague/region carving) |
| flipstackers | struct flipstacker | 16 | 2 | 252 | poolrestart per vertex insertion (triangle.c:3130, 3239) |
| badsubsegs | struct badsubseg | 24 | 3 | 252 | |
| badtriangles | struct badtriang | 48 | 6 | 4092 | |
| splaynodes | struct splaynode | 40 | 5 | 508 | divcon sweep only |

All five are uniform WordPool instances; payloads are opaque words through the pool and typed at the consumer side (the queue "next-poortri" and prevflip links become packed handle words, exactly as the C structs store pointers). No divergence to plain typed Vec<T> pools is taken: uniformity keeps every applicable C line (poolinit/poolalloc/pooldealloc/poolrestart, traversal of badtriangles in tallyfaces, viri traversal in carveholes) a 1:1 transliteration against one shape, and rung 3 may re-type the payloads behind the opaque-word API without touching pools or ids. Boundary note: the bad-triangle queues' firstnonemptyq bookkeeping is mirror state in the data model, not the pool.

The aCute pool diverges from everything above because C's does: it is a set of eight REAL scratch buffers, not an item pool (acute.c:22-70). Mirror: eight Vec<f64> members (initialpoly: 500, petalx/petaly/petalr: 2n, wedges: 2n*20 + 40, points_p/points_q/points_r: 500, with the initial n = 20 hint from triangle.c:7537-7539), plus the same only-grow resize rule as acutepool_resize (acute.c:45-62): a resize drops and re-allocates the four n-dependent buffers when numpoints exceeds the current size. C's free+malloc leaves contents undefined; the Rust mirror re-allocates zeroed - benign-divergence note: every consumer refills before reading (verified on the pq20U120 corpus family; the design relies on the corpus, and the deviation register records it). No ids, no dead stack, no traversal there; the buffers are plain typed f64 storage since nothing packs them.

Nothing thread-affine: ids and single-word handles are values; a worker-thread pool shard in #12 keeps this whole contract unchanged (each pool may be replicated per worker; the address-order mirror of section 5 is only consumed on the writer paths and would apply per-shard plus a merge step, decided in #12 - the pool API carries no obstacle: no thread-local state, no embedding of addresses).

### 5. Oracle address order (the pointer-comparison mirror)

Five sites compare triangle pointers for ORDER, not equality (triangle.c:1655, 7653, 8021; triangle_io.c:232; eps_writer.c:117). Under the pinned oracle platform (glibc 2.36, Linux x86-64, verified empirically this session), fresh triangle blocks are mmap'd top-down and land at descending addresses while all pool-row sizes below the threshold (vertices, subsegs, and the small pools) stay on the brk heap; within a block, addresses ascending with allocation order; blocks page-separated so no two blocks interleave address ranges. Therefore the oracle's raw-address order over two triangle rows is: higher row in an EARLIER-allocated block = larger address; same block = larger row index wins. The C comparison `x.tri < y.tri` at those sites mirrors as `oracle_address_order(x_id, y_id) == Less`:

    block(id) + index_in_block(id) via first_block_items/items_perblock (sentinel shift applied);
    oracle_address_order(a, b) == Less iff block(a) > block(b) ||
        (block(a) == block(b) && index_in_block(a) < index_in_block(b))

Caveats: this is a platform-model assumption inside the C mirror (the allocator's placement, not the C code itself), and therefore an explicit deviation-register entry with a fenced canary: the corpus family "pen" writes -e edge lists whose ordering and numbering are decided by these comparisons, so the differential harness's first-divergence localization fires on any wrong ordering immediately. Vertex/subseg pointer comparisons in C are equalities only (infvertex checks, dummy checks), covered by id equality. If the corpus ever shows an allocator-driven reorder (a freed large chunk reopening a high gap and changing a later block's placement), the implementing ticket handles it at that site; this decision records the empirically verified model, not a new lookup into the C heap.

### 6. Miri and CI

- Zero unsafe, bounds-checked indexing, Copy ids: the pools keep the corpus runnable under Miri in debug builds, per ADR 0002's rung-1 acceptance. The pool adds no I/O, no threads, no time, no randomness. Vec under Miri checks bounds and tracks provenance; the int overlays and slot-0 threading are plain u64 reads/writes inside the chokepoint.
- The stage-1 guarantee of this ticket: the pool layer and the chokepoint accessors are the only code that can touch storage, both land entirely in safe code, and both debug_assert their invariants (strides, id ranges, LIFO chains, sentinel exclusivity), so a green corpus under Miri covers the pool mechanics specifically (freelist threading, traversal frontier extension, restart reset).
- CI: no new gate and no new bench (settled: #7 data covers slot access; growth cost is amortized and rides the #11 perf gate at stage-1 completion with the rest of the core).

### 7. What later idiomatization rungs may change without touching this contract

- The growth schedule (doubling -> 1.5x, shrink at API boundaries, capacity hints from the public API): memory-only, corpus-invisible, gated by #11's runs.
- The allocator backing (global allocator, arenas, per-worker pools for #12): the API holds only values and plain Vec storage.
- Growth mechanics (exact-capacity growth, chunked dual buffers) if the perf gate ever implicates realloc spikes; the custom block arena stays the recorded fallback shape.
- Typed row payloads (rung 3 re-typing, ADR 0002 section 2) behind opaque words: pools and ids unchanged.
- get_unchecked-style elisions inside the chokepoint (deviation-register entry + invariant comment, corpus and Miri still green), per ADR 0002 section 3.
- The SoA rematch (rung 3 / #11 trigger) - orthogonal: it changes the chokepoint's internals, and the pools would then be split-word stores through the same (pool, row, slot) addressing.

## Considered alternatives

- Byte-close custom arena (C's block list, alignment, freelist): the exact-C memory model (block-step growth, no realloc transient) and pointer-true growth, at the price of two-level access in the port's hottest path, the largest implementation surface among the candidates, and no behavioral gain over ids. Rejected for stage 1 on parity-risk and #7-covered-access grounds; recorded as fallback if #11 ever shows realloc spikes that flat growth cannot tune away.
- Bump arenas (bumpalo / typed-arena / bump-scope): no reclamation, dead reuse impossible; ruled out by #2's fit analysis (output ordering depends on the reuse of freed items).
- Vec with exact C-step capacity growth: memory parity with C, but arithmetic-series reallocation of the whole store, O(N^1.5) aggregate copying; ruled out on the perf-gate risk (section 2).
- Plain typed Vec<T> per small pool: diverges from the single mirrored shape C actually provides (one alloc/dealloc/path machinery for small and big pools alike) for zero performance value; rung 3 gets typed payloads behind the same opaque-word pool anyway.
- slab / slotmap (generational indices): generations bloat or shrink the 8-byte slot space and solve a stale-handle problem the algorithm does not have (dead detection is structural); ruled out with ADR 0002.
- Standard per-object allocation: kills dense indexing, allocation-order traversal, and cheap reuse order; never viable.
- C-heap placement mirroring (tracking actual glibc addresses to reproduce tri order "exactly"): rejected; the observable content at those five sites is a total order, and section 5 reproduces that order arithmetically from the block geometry, with the corpus as canary.

## Consequences

- Every mesh pool is one uniform WordPool (section 1); the transliteration of every poolinit/poolalloc/pooldealloc/poolrestart/traversal call site maps 1:1. Fresh-id assignment stays monotonic; dead-reuse stays exact LIFO through slot 0; traversal stays allocation order across the live frontier; restarts retain capacity exactly like C; output ordering (allocation order plus reuse order) is preserved.
- Deviation-register entries (for #10's spec): (1) physical storage is one flat Vec per pool, not a block chain (growth schedule amortized-doubling, initial C first-block hint; transient regrow mapping vs none in C); (2) allocation failure becomes a Result error at the API shell instead of printf+exit (already mandated by #4); (3) the oracle address order of section 5 replaces raw pointer comparisons at the five sites (platform-model assumption, corpus-canaried); (4) sentinel rows take ids 0 (null) and 1 (dummytri/dummysub) instead of hand-aligned allocations (from ADR 0002); (5) the aCute pool re-allocates zeroed buffers on resize where C's contents are undefined (benign, consumer-refilled).
- For #10 (module map): one pools module behind the chokepoint; the handles and macros stay as pinned in ADR 0002. For #12 (MT): pools are plain-owned per mesh; per-worker shard pools plus a merge ordering are decidable later without API change.
- Memory expectations for the perf gate (#11) and later runs: worst-case 2x of the word store in allocated capacity (typical 1.3-1.5x) vs C's sub-one-block slack; RSS growth is carried by the data itself; the transient at each doubling is a mapping move, not a copy, for realistic pool sizes on glibc. The stage-1 parity gate (#11) now also covers growth behavior implicitly through runtimes.
- Glossary terms added: row id, word pool, logical block geometry, oracle address order, acute scratch buffers.
