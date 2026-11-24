# Core data model: packed u64 slot handles, u64-word rows, one safe accessor chokepoint

Status: accepted 2026-11-24; decided in the wayfinder ticket Core data model: handle representation for otri/osub and vertex layout (#8), on the #7 microbench data, the #4 UB inventory, and the stage-1 constraints of the port strategy (ADR 0001, #6). Glossary terms added: packed handle, register handle, row, slot, accessor chokepoint, int overlay, dead stack, sentinel rows.

## Decision inputs (settled, not re-derived here)

- Tagged raw-pointer handles are permanently out (user ruling 2026-11-24, map #1). The C encode/decode trick keeps only its slot-density effect.
- Stage-1 constraints (ADR 0001 / #6): the handle variant must support mirror transliteration of the C handle macros (sym, lnext, lprev, and the rest) as methods with identical encode/decode semantics; triangle and vertex rows keep the C slot machinery, runtime-indexed slots behind one private accessor chokepoint; the C strict-aliasing UB is not reproduced, its layout and access semantics are; typed vertex/triangle struct layouts are deferred to idiomatization rung 3, must not be precluded, and must keep the byte-identity corpus green when they land.
- The C mechanism (UB inventory, #4; docs/research/ub-inventory.md on branch research/ub-inventory):
  - encode()/decode() (triangle.h:703-708) pack the orientation into the 2 low bits of one 8-byte neighbor slot; subsegments pack 1 bit (sencode/sdecode). The register handles are unpacked structs: otri with fields triangle *tri and int orient, osub with subseg *ss and int ssorient (triangle.h:239-260); every handle macro (triangle.h:703-1090) operates on these; pure ring rotations (plus1mod3/minus1mod3) never touch the packed form. Encoding happens only when a handle is stored into a row slot.
  - Triangle rows (REAL**, triangle.h:231): slots 0-2 tagged neighbor triangles, 3-5 vertices, 6-8 tagged subsegments (-p only), then runtime-indexed payload (high-order midpoints, element attributes, area bound, -n numbering int); row size computed at runtime (triangle.c:1138-1172).
  - Subsegment rows: slots 0-1 tagged subsegs, 2-5 vertices, 6-7 tagged triangles, 8 = boundary marker int overlay (bnd_mark, triangle.h:995-1004).
  - Vertex rows (REAL*, triangle.h:266): (mesh_dim + nextras) REALs of coordinates/attributes, then an int boundary marker, an int vertex type, and (with -p) an encode()d triangle word (vertex2tri); offsets computed at runtime (triangle.c:1098-1119); sizes come out as multiples of 8 bytes.
  - The pools round itembytes up to multiples of alignbytes, with alignbytes at least sizeof(void*) (poolinit, triangle.c:812-826), so every row is whole 8-byte words; the dead-item stack is threaded through slot 0 of freed rows (pooldealloc, triangle.c:924-928); traversals walk items in allocation order and include dead rows (traverse, triangle.c:937-990) - and allocation order is the oracle's output ordering (#4, #5).
  - dummytri ("outer space") and dummysub ("omnipresent subsegment") are hand-aligned sentinel allocations outside the pools (dummyinit, triangle.c:1029-1078).
- The #7 microbench (build server, Xeon Skylake shared vCPU, single-threaded; periodic 724x724 grid, 1,048,352 triangles; footprints 72/48/39 MiB, all above the 16 MiB shared L3; medians of 20 reps, ns/op; full analysis in ticket #7):

      kernel (C analog)               AoS 16B   packed 8B   SoA 8B-equiv
      lnext ring (lnextself)           1.48       2.54        1.68
      lprev ring (lprevself)           1.49       1.90        1.57
      sym sweep, grid order            2.68       1.84        1.82
      sym sweep, shuffled              2.52       1.87        1.75
      neighbor-deref chase, grid       22.54      19.93       17.78
      neighbor-deref chase, shuffled   36.70      32.74       34.49
      decode stream, all slots         2.48       1.84        1.62
      encode stream, all slots         3.29       2.49        3.47

  Headline (#7): packed wins the store side and the uniform-scatter chase at exact C slot density; SoA wins most read kernels while the working set fits cache; the 16-byte AoS struct handle did not win a single hot kernel; no candidate dominates.

## Decision

### 1. Handle representation: packed u64 slot encoding, C-mirror unpacked register form

Stored handles. Every stored handle (neighbor slots tri[0..2], subsegment slots tri[6..8] and ss[0..1], triangle words ss[6..7], vertex2tri) is one packed u64: a 32-bit row index in the high bits, the orientation in the low 2 bits (triangles and subsegments alike; a subsegment's bit 1 stays permanently spare - the mirror of C's pointer-alignment bit that the viral-infection flag occupies on triangle word 6, triangle.h:875 - see the amendment at the end). The all-zero word keeps the exact meaning of the C NULL slot and is never decoded, exactly as C never decodes NULL.

    encode(o: Otri)  = (o.tri as u64) << 2 | o.orient     C analog: pointer | orient
    decode(w: u64)   = tri = w >> 2, orient = w & 3       C analog: mask 3l, xor
    sencode(s: Osub) = (s.ss as u64) << 2 | s.ssorient    subsegments mask the low 2 bits, like C; bit 1 = spare virus bit
    sdecode(w: u64)  = ss = w >> 2, ssorient = w & 1

Semantics are identical to triangle.h:703-708: a one-to-one mapping between (row, orientation) and one machine word, orientation recovered from the low bits. Density is exactly C's: 8 bytes per slot, 48-byte triangle rows without -p. 32-bit row ids cap a pool at 4.29e9 items (settled on the map as far above realistic mesh sizes); the unused high bits of the u64 are spare, the mirror of C's unused alignment bits.

Register handles. Otri/Osub stay unpacked on the C model, and the register form is what the handle-macro methods operate on: pure ring rotations act on the orientation field alone and never re-encode. encode/decode run only at slot stores and loads, exactly where the C macros run them. The hidden macro locals (ptr/sptr) disappear; every macro becomes a method on the handle type.

Negatives weighed explicitly:

- Ring kernels: the #7 packed variant paid a real tax (+72% lnext, +28% lprev vs AoS) because it kept packed u64s in registers across rotations, decoding and re-encoding at every step. The mirror register form never rotates packed words, so this cost reduces to C's own orientation arithmetic (the AoS/SoA ring numbers, 1.48-1.68 ns/op, bracket it); packing pays only where C pays it: 1.84 ns/op decode and 2.49 ns/op encode, both the best measured of all variants.
- Read side: SoA leads the decode stream by 13% (1.62 vs 1.84) and the grid-locality chase by 12% (17.78 vs 19.93). Those wins are real but LLC-dependent: the SoA column pair (15.7 MiB) approximately fills this machine's 16 MiB L3; beyond LLC the advantage washes out and the two dependent misses per hop from disjoint regions start to bite. The bench could not cover that regime (capped by runtime, not RAM); #7 records the risk growth with mesh size.
- Store side: SoA loses 39% to packed (2.49 vs 3.47). Triangle mutates heavily and destructively (flips rewrite about six slots, freed rows churn continuously), so the store side is structurally load-bearing, not a corner case.
- Uniform scatter: packed leads (32.74 vs 34.49), and 1M triangles is already SoA's bracket-best regime there (its column pair nearly fills L3).
- Constraint fit: a single-word slot with the orientation folded in is not one option among several for stage 1; it is the C slot machinery that ADR 0001 pins (identical encode/decode semantics, rows keep the slot machinery). SoA turns one neighbor slot into two words in two arrays: a layout rewrite, which belongs to an idiomatization rung (rung 3), not to transliteration. SoA also makes each neighbor update two non-atomic words in disjoint arrays: the torn-update shape internal multithreading (map destination) would then have to lock or secure differently.
- AoS is eliminated by data alone: no kernel wins anywhere (sweeps +35-45%, chases +12/+27%, decode +53%, encode +32%), and it doubles the connectivity working set, which hurts most exactly in the beyond-LLC regime the bench could not yet test.

Verdict: packed u64 slot encoding with unpacked register handles. Its remaining regressions (decode stream +13% vs SoA; sub-ns ring deltas against the fold-chain floor) are bounded register/ALU-level costs that do not grow with mesh size; its wins (store, scatter, density) and its constraint fit are structural. SoA is recorded as the alternative to revisit at rung 3 / #11 profiling if decode-side reads or locality-bound chases dominate real workloads at mesh sizes where SoA columns stay cache-resident; such a revisit stays corpus-gated.

### 2. Rows: u64 slot words, runtime slot roles, int overlays (vertex storage included)

All three row kinds are arrays of untyped u64 slot words in flat pool stores; stride and slot roles are fixed at init by the transliterated C formulas - the same numbers C computes - so layout and access semantics are mirrored without the aliasing UB (rows have no type beyond u64 word; nothing is read as a type it was not, except where C does exactly that via the int casts, see below).

- Triangle row: words 0-2 packed neighbor handles, 3-5 vertex row ids, 6-8 packed subseg handles (-p only), then runtime-indexed payload: high-order midpoint ids, element attributes (f64 words), area bound (f64 word), -n numbering (int overlay). highorderindex = 6 + usesegments*3; elemattribindex and areaboundindex keep the C byte-offset formulas, expressed in 8-byte words (identical to REAL units on 64-bit).
- Subsegment row: words 0-1 packed neighbor subsegs, 2-5 vertex ids, 6-7 packed triangle handles, 8 = boundary marker int overlay (low half; the upper half stays spare, as in C).
- Vertex row: words 0..(mesh_dim+nextras-1) hold coordinates/attributes (f64-valued words), then the boundary marker and vertex type as 32-bit int overlays in the two halves of one word, then (with -p) the vertex2tri word holding an encode()d Otri. vertexmarkindex (int units) and vertex2triindex (word units) come from the transliterated initializevertexpool. Worked example (dim 2, no extras): x and y in words 0-1, mark = low half of word 2, type = high half of word 2, stride 3 words = 24 bytes, or 4 words = 32 bytes with -p (vertex2tri in word 3) - exactly C's vertexsize in both cases.

Int overlays are touched with C's byte semantics: only the addressed 4 bytes are written (read-modify-write on the containing word), preserving whatever the other half holds, exactly like the C int cast on pool memory. f64 slots are whole-word accesses through bit-exact views (from_bits/to_bits); no arithmetic ever evaluates a word written through an overlay, and no overlay is ever read as a coordinate.

Rung 3 is not precluded: all access goes through the chokepoint API, so a typed row implementation can provide the same accessors over real struct fields with ids and call sites unchanged; the byte-identity corpus is the gate that keeps such a change honest.

### 3. Unsafe strategy: zero unsafe in the stage-1 core

Every mechanism that is UB or a raw-pointer trick in C lands as safe Rust without losing bit-level exactness. Where C is safe, Rust is safe:

1. Tagged encode/decode becomes integer shifts and masks on u64. No pointer is ever aliased into a handle word; handles are plain Copy values.
2. Pointers stored in REAL-sized slots become untyped u64 words in untyped rows.
3. Vertex and payload int casts become half-word reads/writes on whole words (section 2), mirroring the C byte behavior without aliasing.
4. The pool freelist threading through the first slot of freed rows (pooldealloc) is reproduced: a dead row carries the next-dead row id (u64) in its slot 0, end of stack = 0 (the null row id). Same slot as C, the pointer replaced by an id, zero unsafe; killtri-adjacent dead-marking (slots 1 and 3 nulled - killtri sets tria[1] and tria[3] to NULL, never slot 0) keeps the C dead-detection working unchanged.
5. Sentinel rows: dummytri/dummysub become reserved rows at fixed ids, excluded from traversal and the dead stack, initialized and bonded exactly like dummyinit does. Deviation register: sentinels stop being hand-aligned allocations; manual alignment becomes irrelevant because no tag bits need to be found in addresses.

Invariants and how they are secured:

- Range invariant: a decoded handle's row is live-or-sentinel and its orientation is in range (orient < 3 triangles, < 2 subsegments); debug_asserted on every decode (one mask compare, debug builds only).
- Null discipline: the all-zero word is never decoded (C never decodes a NULL slot); a debug_assert in decode catches violations.
- Slot-role invariant: slot index below stride; role table built only by the init formulas; chokepoint accessors debug_assert bounds.
- Dead-stack discipline: slot 0 of a dead row is pool-private between kill and realloc (exactly as in C); live code never reads a dead row's slots as data (the trans-literated control flow never does); debug builds can double-walk the chain to detect duplicates and loops.
- Reuse order: exact LIFO, mirror of deaditemstack - reuse order is output-observable, so the pools must keep it exact (interface section 4.4).

The accessor chokepoint guarantees (so that everything outside it is safe):

- The chokepoint module is the only code that touches pool storage. Its API takes and returns values only (u64 words, u32 row ids, f64, i32) or Copy handle structs; no reference into pool storage survives a call.
- All conversions are bit-exact (from_bits/to_bits, half-word masks). Nothing is read as a type it was not written as, except int overlays, which is exactly the C behavior being mirrored.
- Indexing is bounds-checked. get_unchecked stays out of stage 1; if profiling later demands one, it lands only inside the chokepoint, with a deviation-register entry, an invariant comment, and the corpus plus Miri still green.
- With zero unsafe, the rung-1 acceptance state is simple: debug_asserts on, the corpus (sub)set under Miri. That is what this decision hands to the implementation tickets.

### 4. Pool interface contract for the memory pools (#9; interface only, implementation stays there)

1. Item identity: Copy u32 row ids per pool. An allocated id stays valid until dealloc; after dead-reuse the id is re-handed for the new item. An id means "the row at this position in the pool's index space", matching what the item address means in C.
2. Slot addressing: the chokepoint needs O(1) word access by (row, slot). The pools expose flat u64 word stores with a per-pool runtime stride (recommended shape: one Vec u64 per pool, offset = id * stride + slot); block list vs single Vec is #9's choice - ids are indices, not addresses, so growth or moving storage never invalidates them.
3. Allocation order: fresh ids are handed out in monotonically increasing order; traversal covers the id range up to the high water mark, in order, dead rows included (dead-detection stays a data-model concern: the killtri NULL pattern). Traversal order equals the oracle's output ordering (#4, #5); no pool may rearrange it. Initial-capacity hints are passed through for memory parity only.
4. Dead stack: dealloc pushes and alloc pops one stack per pool, exact LIFO (the C deaditemstack; reuse order is output-observable). Slot 0 of a dead row is pool-private between dealloc and realloc; stage 1 threads the linkage there, mirroring C.
5. Counts: live count and high water (the m->items and m->maxitems shapes) exposed for the API surface and traversal setup.
6. Sentinels: the triangle and subseg pools each reserve one fixed id for the sentinel row (recommended: id 1, with 0 = the null encoding), excluded from allocation, traversal, and the dead stack.
7. Word granularity: row byte sizes are multiples of 8 (poolinit's alignbytes rounding is mirrored); stride is a whole word count fixed at init.
8. The typed small pools (viri, badsubsegs, badtriangles, flipstackers, splaynodes) and the aCute pool follow the same alloc/dealloc and LIFO dead-reuse semantics; their payload words are opaque to the pool.
9. MT note (non-binding now): plain-value ids and single-word handles keep storage shardable per worker without handle form changes; nothing in the interface may assume thread-affine storage.

## Considered alternatives

- AoS struct handle (16 B slots): rejected on data (not a single kernel win; sweeps +35-45%, chases +12/+27%, decode +53%, encode +32%; doubled footprint penalizes every beyond-LLC regime) and on constraints (a 16-byte neighbor slot is already an encode/decode departure from C's word slot).
- SoA split arrays: rejected for stage 1 by constraints (the single-word slot with folded orientation is the pinned C slot machinery; a sym/decode mirror over two split words is not the C encoding) and by data (store side +39% on a destructively mutating workload; uniform-scatter loss; LLC-bounded read wins; MT torn-update shape). Recorded as the rung 3 / #11 revisit candidate under the trigger recorded in section 1.
- Tagged raw-pointer handles: permanently out by user ruling; only the slot-density effect survives, via packing.
- Generational slotmap-style keys: fail the 8-byte slot density requirement (generation bits either bloat the stored word past C density or shrink the index space); the freelist discipline, debug_asserts, and Miri carry stage-1 safety instead. Revisitable with concrete stale-handle failure evidence.

## Consequences

- Every C handle macro transliterates to a method on Otri/Osub with identical encode/decode semantics; the hidden macro locals (ptr/sptr) disappear from every using function.
- Deviation-register entries (for the spec, #10): tagged pointers become packed ids; vertex int casts become int overlays behind the chokepoint; first-slot threading carries row ids instead of pointers; sentinels become reserved rows. All layout and access semantics are preserved.
- The performance gate (#11) profiles the real workload mix; representation changes beyond this design are rung-gated (SoA is the recorded alternative).
- The pool design (#9) is bound only by the interface contract in section 4; nothing else in this ticket constrains its implementation.
- Rung 3 keeps its mandate (typed rows) with the corpus as gate; nothing in the stage-1 layout blocks it: the chokepoint API is the seam.

## Appendix: type sketches

Implementable shapes (not a full transliteration; module placement lands with #10's module map):

    // handles.rs - register forms and slot packing (all safe code)

    pub(crate) const NULL: u32 = 0;             // the all-zero word, as C's NULL slot value
    // triangle pool: id 1 = DUMMY_TRI  ("outer space")
    // subseg pool:    id 1 = DUMMY_SUB  ("omnipresent subsegment")

    #[derive(Clone, Copy, PartialEq, Eq)]
    pub(crate) struct Otri { pub tri: u32, pub orient: u8 }      // 8 B register handle (C: {triangle *, int})
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub(crate) struct Osub { pub ss: u32, pub ssorient: u8 }     // 8 B register handle (C: {subseg *, int})

    #[inline]
    pub(crate) fn encode(t: Otri) -> u64 {
        debug_assert!(t.orient < 3);
        ((t.tri as u64) << 2) | t.orient as u64
    }
    #[inline]
    pub(crate) fn decode(w: u64) -> Otri {
        debug_assert!(w != 0, "null slot must never be decoded (C never does)");
        Otri { tri: (w >> 2) as u32, orient: (w & 0b11) as u8 }
    }
    #[inline]
    pub(crate) fn sencode(s: Osub) -> u64 {
        debug_assert!(s.ssorient < 2);
        ((s.ss as u64) << 2) | s.ssorient as u64
    }
    #[inline]
    pub(crate) fn sdecode(w: u64) -> Osub {
        debug_assert!(w != 0);
        Osub { ss: (w >> 2) as u32, ssorient: (w & 0b1) as u8 }
    }

    // slots.rs - the one private accessor chokepoint (all pool storage access lives here)

    pub(crate) struct Mesh {
        tri_pool: WordPool, sub_pool: WordPool, vertex_pool: WordPool,
        tri_shape: TriShape, vertex_shape: VertexShape,
        /* dead-stack heads per pool, counters, runtime options */
    }

    impl Mesh {
        // triangle rows
        #[inline] pub(crate) fn tri_slot(&self, t: u32, i: usize) -> u64;
        #[inline] pub(crate) fn set_tri_slot(&mut self, t: u32, i: usize, w: u64);
        #[inline] pub(crate) fn tri_real(&self, t: u32, i: usize) -> f64;             // attributes / area bound
        #[inline] pub(crate) fn tri_int(&self, t: u32, half: HalfWord) -> i32;        // -n numbering overlay
        #[inline] pub(crate) fn set_tri_int(&mut self, t: u32, half: HalfWord, val: i32);
        // subsegment rows (word 8 marker half included)
        #[inline] pub(crate) fn sub_slot(&self, s: u32, i: usize) -> u64;
        #[inline] pub(crate) fn set_sub_slot(&mut self, s: u32, i: usize, w: u64);
        #[inline] pub(crate) fn sub_mark(&self, s: u32) -> i32;
        #[inline] pub(crate) fn set_sub_mark(&mut self, s: u32, m: i32);              // bnd_mark / setmark
        // vertex rows
        #[inline] pub(crate) fn vx_real(&self, v: u32, comp: usize) -> f64;
        #[inline] pub(crate) fn vx_set_real(&mut self, v: u32, comp: usize, x: f64);
        #[inline] pub(crate) fn vx_mark(&self, v: u32) -> i32;
        #[inline] pub(crate) fn vx_set_mark(&mut self, v: u32, m: i32);
        #[inline] pub(crate) fn vx_type(&self, v: u32) -> i32;
        #[inline] pub(crate) fn vx_set_type(&mut self, v: u32, ty: i32);
        #[inline] pub(crate) fn vx_2tri(&self, v: u32) -> u64;                        // C: encode()d otri word
        #[inline] pub(crate) fn vx_set_2tri(&mut self, v: u32, w: u64);
        // dead stack mirrors (pool-owned #9 interface; stage-1 threads through slot 0)
        pub(crate) fn tri_alloc(&mut self) -> u32;       // LIFO dead pop, else next fresh id
        pub(crate) fn tri_dealloc(&mut self, t: u32);   // mirror killtri + push
    }

    // runtime row shapes (transliterated formulas, triangle.c:1098-1172)

    pub(crate) struct TriShape {
        pub stride: usize,             // words per row: C trisize / 8
        pub usesegments: bool,         // words 6..8 active (-p)
        pub highorderindex: usize,     // 6 + usesegments * 3
        pub elemattribindex: usize,    // C byte formula / 8
        pub areaboundindex: usize,
        pub numberindex: Option<HalfWord>,   // -n numbering int overlay
    }
    pub(crate) struct VertexShape {
        pub stride: usize,             // C vertexsize / 8
        pub mark: HalfWord,           // vertexmarkindex (int units), always word-aligned half
        pub vtype: HalfWord,           // vertexmarkindex + 1, the other half of the same word
        pub v2tri: Option<usize>,      // vertex2triindex (word units, -p only)
    }
    pub(crate) struct HalfWord { pub word: usize, pub high: bool }

    // handle-macro mirrors (transliteration modules; no storage reference escapes)

    impl Otri {
        // C lnext: pure orientation arithmetic, no memory, no re-encode
        pub(crate) fn lnext(mut self) -> Otri { self.orient = plus1mod3[self.orient as usize]; self }
        pub(crate) fn lprev(mut self) -> Otri { self.orient = minus1mod3[self.orient as usize]; self }
    }
    impl Mesh {
        // C sym: decode(tri[orient])
        pub(crate) fn sym(&self, o: &mut Otri) {
            *o = decode(self.tri_slot(o.tri, o.orient as usize));
        }
        // C bond: encode both ways
        pub(crate) fn bond(&mut self, a: &mut Otri, b: &mut Otri) { /* two set_tri_slot calls */ }
        // C tspivot: sdecode(tri[6 + orient]); C tsbond: both slots
        pub(crate) fn tspivot(&self, o: Otri, out: &mut Osub) { /* tri_slot 6+orient */ }
        // C org/dest/apex: vertex word slots 3..5 with triangle.h orientation arithmetic
        pub(crate) fn org(&self, o: Otri) -> u32;   // tri_slot(o.tri, 3 + o.orient) as u32
    }

Notes:
- plus1mod3/minus1mod3 exist in the C core already; org/dest/apex, onext, and friends keep the exact slot arithmetic of triangle.h:703-1090; the sketch shows the shapes, not every offset table.
- WordPool is the #9 pool behind the section 4 interface; the chokepoint is its only client.
- All functions above are safe; the whole core targets zero unsafe, with debug_asserts carrying the invariants and Miri runs of the corpus as the rung-1 acceptance test.

## Amendment (2026-11-24): subsegment packing reserves both low bits (issue #22)

Section 1 pinned subsegment handles at ss<<1 - one orientation bit, the row
id from bit 1 upward. That collided with C's viral-infection flag: C's
infect() tags bit 1 of triangle word 6 (triangle.h:875-887), whose masked
-out low bits come from sdecode's ~3 (triangle.h:925-933) - C keeps bit 1
free by 4-byte pointer alignment regardless of what the slot holds. Under
ss<<1 the infection tag destroyed the packed row id instead. Found while
landing the stage-1 foundation (PR #23, issue #22); resolved by moving the
whole subsegment packing up one bit:

    sencode(s: Osub) = (s.ss as u64) << 2 | s.ssorient    bit 0 orient, bit 1 spare (the virus)
    sdecode(w: u64)  = ss = w >> 2, ssorient = w & 1      the literal mirror of C's mask ~3

Section 1 and the appendix sketch above are updated in place to match. Consequences:
- Triangle encode/decode are untouched.
- Subsegment row ids cap at 2^30 (still far above realistic subseg counts);
  triangle ids still cap at 2^32.
- The DUMMY_SUB stored word moves from 2 to 4.
- infect/uninfect/infected (slots.rs) become literal C mirrors at bit 1 of
  word 6, on -p rows and plain rows alike; the KNOWN GAP comments are gone.
- Deviation D1's wording is updated ("low 2 bits" for both handle kinds).
