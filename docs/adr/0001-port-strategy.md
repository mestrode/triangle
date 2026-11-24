# Port strategy: verified transliteration with staged idiomatization

Status: accepted 2026-11-24; decided in the wayfinder ticket Port strategy: staged transliteration vs idiomatic rewrite (#6).

The port of the roughly 19k-line C core (Triangle 1.6 + aCute) proceeds as a hybrid: the core algorithms are transliterated function-for-function into the mirrored slot machinery (index-based packed handles replace the C tagged raw pointers, which are ruled out), verified byte-identical against the pinned C oracle over the full-option corpus, while the API shell (options struct, Result errors, no libc at the boundary) is idiomatic from day one. After parity, the core is idiomatized in four corpus-gated rungs (accessor invariants, error flow, storage typing, module reorganization); the differential harness is built before any porting, and serial byte-identity stays a permanent regression gate, including under later internal multithreading.

## Considered options

Pure staged transliteration (1:1 also for the shell): rejected, the shell would ship non-idiomatic error and options surfaces first and the idiomatization stall risk grows with an unrusted API on top. Pure idiomatic rewrite: rejected for the highest correctness risk: redesigned queue ownership, iteration order and call order threaten the output ordering that the byte-identity oracle forbids, and a divergence has no structural correspondence to bisect along; it also front-loads design work, pushing first verified parity furthest out.

## Consequences

Strict mapping with a deviation register keeps side-by-side debugging against C possible throughout stage 1. The known risk, idiomatization stalling because temporary code lives longest, is countered by explicit rung tickets and the permanent oracle gate.
