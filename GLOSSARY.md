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
