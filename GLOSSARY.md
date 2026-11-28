# Triangle Rust Port

The C-to-Rust port of Triangle (Triangle 1.6 + aCute) as charted by the wayfinder map: a Rust library plus binary at performance parity with the C original.

## Language

**Transliteration**:
Porting the C core function-for-function, keeping function boundaries, names, call order and control flow identical in stage 1.
_Avoid_: rewrite, translation

**Transliterated core**:
The ported algorithms below the API shell: triangulation, quality refinement, predicates, mesh internals, memory pools.
_Avoid_: legacy core, C part

**Idiomatic shell**:
The public API layer (options, errors, entry points) written in idiomatic Rust from day one.
_Avoid_: wrapper, bindings

**Idiomatization**:
The staged, post-parity conversion of the transliterated core toward idiomatic Rust, each stage holding byte-identical outputs.
_Avoid_: cleanup, refactoring

**Idiomatization rung**:
One corpus-gated stage of the idiomatization ladder: invariants, error flow, storage typing, reorganization.
_Avoid_: phase

**C oracle**:
The pinned C build (-O2/-O3, -ffp-contract=off, no -march overrides) whose outputs are the port's ground truth.
_Avoid_: reference build, upstream

**Parity**:
Byte-identical equality of the Rust port's outputs with the oracle's over the corpus, including element ordering.
_Avoid_: equivalence, compatibility

**Differential harness**: 
The build-first tooling that runs the corpus against oracle and Rust port and compares byte-for-byte, reporting the first divergence.
_Avoid_: diff tool, test suite

**Corpus**:
The generated workloads (poly classes times option families) compared by the harness and reused for benchmarks.
_Avoid_: test set, fixtures

**Deviation register**:
The list recording every deliberate departure from 1:1 during transliteration.
_Avoid_: exception list

## Architecture

**Module map**: The standing assignment of every C compilation unit, and of each named split of triangle.c, to exactly one Rust module, recorded in docs/function-mapping.md; the stage-1 tickets fill the per-function rows, the module assignment itself is decided by ADR 0004.
_Avoid_: mirror directory, file-by-file port

**Error funnel**: The single seam at the api module boundary where the transliterated core's mirrored internal status codes are translated into the public Error taxonomy; no other code maps internal codes to public errors.
_Avoid_: error codes, exceptions, panics

**Candidate contract**: The differential harness's invocation rule (tools/oracle): the Rust CLI runs as ./candidate <options> <input> inside a prepared run directory, writing the standard input.1.* outputs with exit 0 on success.
_Avoid_: wrapper script, argument compatibility set

**Output phase**:
The numbering + marshalling + serialization of the six mesh-bound artifacts as one phase (ADR 0005's phase (e)), owned by io/marshal.rs behind one record seam per artifact.
_Avoid_: writer bundle, I/O layer

## Data model

**Packed handle**:
The one-word storage form of an oriented triangle or subsegment: a u64 carrying a 32-bit pool row index and the orientation in the low 2 bits (triangles) or 1 bit (subsegments), keeping the C slot density and encode/decode semantics without raw pointers.
_Avoid_: tagged pointer, slot key, fat handle

**Register handle**:
The unpacked form the handle-macro methods operate on: Otri (row index, orientation), Osub (row index, side); a handle is packed only when stored into a row slot, exactly where the C macros encode and decode.
_Avoid_: hot handle, wrapper handle

**Row**:
One pool item's storage: a runtime-sized array of u64 slot words (triangle, subsegment, or vertex row), addressed by a copy row id and laid out by the mirrored C slot formulas.
_Avoid_: record, element, object

**Slot**:
One u64 word of a row; its role (neighbor, subsegment, vertex id, coordinate, attribute, area bound, int overlay, payload) is fixed at runtime by the pool-init formulas.
_Avoid_: field, cell

**Accessor chokepoint**:
The single private module through which all row storage is read and written; outside it only handles and values exist, so the transliterated core is safe and Miri-testable.
_Avoid_: accessor layer, gateway

**Int overlay**:
A 32-bit integer kept in one half of a slot word (vertex boundary marker, vertex type, subsegment boundary marker, element numbering), written through the chokepoint touching exactly those 4 bytes, mirroring the C int casts without aliasing.
_Avoid_: int cast, punned field

**Dead stack**:
The LIFO pool freelist threaded through slot 0 of dead rows, as in C pooldealloc; carrying row ids until reuse. Deadness detection (killtri's null slots) is separate and stays a data-model concern.
_Avoid_: garbage list, open freelist

**Sentinel rows**:
The reserved out-of-band rows for outer space (dummytri) and the omnipresent subsegment (dummysub): fixed row ids, excluded from traversal and the dead stack, bonded exactly as in C.
_Avoid_: guard objects, dummy pointers

## Memory management

**Row id**:
The Copy u32 index naming one pool row: valid until dealloc, re-handed on dead-reuse, meaning the row position in the pool's index space, exactly what the item address means in C.
_Avoid_: handle id, slot index, item number

**Word pool**:
The single pool type replacing C's struct memorypool: one flat Vec of untyped u64 words per mesh pool, stride-fixed rows, LIFO dead stack threaded through slot 0, monotonic fresh ids, allocation-order traversal; every mesh pool is an instance.
_Avoid_: arena, bump allocator, slot container

**Logical block geometry**:
The first-block and per-block item counts each pool carries as plain numbers from C: they drive the sampleraise and getvertex formulas (output-observable) and the memory-parity initial reservation, independent of the physical Vec capacity and its growth.
_Avoid_: block size, capacity plan, growth factor

**Oracle address order**:
The total order over triangle rows that replaces the C raw-pointer comparisons in the edge-list outputs: blocks in reverse allocation order (glibc top-down mmap placement), rows ascending within a block; provided by the pool, canaried by the pen corpus family.
_Avoid_: pointer order, id order, row order

**Acute scratch buffers**:
The aCute pool in Rust: eight typed f64 buffers replacing acutepool, sized from the vertex-count hint and grown only, never walked, recycled, or id-addressed like the item pools.
_Avoid_: acute allocator, petal pool, acute arena

## Parallelism

**Serial path**: The execution route with thread count 1: the transliterated core exactly as ported, whose byte-identity against the C oracle over the corpus is the permanently binding gate; every other guarantee stands on it as the reproducibility anchor.
_Avoid_: fallback path, single-threaded mode

**Parallel path**: The execution route with thread count above 1: freer-scheduled workers over the same pools via a shard overlay and claim discipline, structurally validated, never byte-compared.
_Avoid_: MT mode, threaded mode

**Structural validation**: The acceptance method for parallel outputs against the serial Rust run: exact count and attribute equality for parallel-invariant families; counts within a recorded band plus zero invariant-checker failures for refinement families.
_Avoid_: fuzzy compare, relaxed diff

**Worker**: One scoped OS thread executing a phase shard or claiming queue work during a parallel run; never owns the mesh pools exclusively.
_Avoid_: lane, executor

**Pool shard**: The parallel path's per-worker overlay on a WordPool: reserved fresh-id blocks and per-worker dead-stack chains above the unchanged pool; merged at scoped joins in deterministic owner order where merges occur.
_Avoid_: thread pool, arena

**Claim sidecar**: The per-row claim-flag array outside the pool words of the parallel path, so row layouts stay C-identical and claims are cheap atomic words.
_Avoid_: lock word, flag slot

**Star claim**: The refinement conflict primitive: all-or-nothing acquisition of the triangle rows an operation will touch, taken in row-id order, with stale items released and re-enqueued on conflict.
_Avoid_: edge lock, optimistic retry

**Worker stream**: A derived LCG stream on the parallel path (same constants, seed hashed from the context seed and a structure key), keyed per subtree in construction and per worker in sampling; the serial path keeps the single context-seeded sequence.
_Avoid_: thread RNG, seeded lane

**MT axis**: The scaling measurement of the performance gate: the same candidate timed at T=1 versus T pinned to N vCPUs over the l/xl timed set, efficiency against the T=1 baseline, never against the C oracle.
_Avoid_: MT benchmark, threaded sweep
