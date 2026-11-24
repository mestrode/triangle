# Robust predicates: portability of Shewchuk's exact predicates to Rust

Findings for wayfinder ticket #3. Date: 2026-11-23. Base: master @ 451c932. Line refs relative to src/Triangle.

## 1. What our predicates.c actually depends on (repo inspection)

- Adaptive orient2d/incircle (plus 3D variants the core does not call) with error-bound fallbacks: ccwerrboundA/B/C, iccerrboundA/B/C etc., all derived inside exactinit from epsilon and splitter, which in turn are computed by runtime bisection (predicates.c:157-210).
- On LINUX, exactinit sets the x87 FPU control word via fpu_control.h _FPU_SETCW(4722) (double precision); on CPU86 via _control87. This is an x87 precision-control switch, NOT a rounding-mode change. On our targets (x86-64, aarch64) double arithmetic compiles to SSE2/NEON IEEE-754 ops that the x87 control word does not influence. Conclusion: the FP-env manipulation is inert on 64-bit targets; the runtime epsilon/splitter bisection is the part that actually matters and is pure arithmetic.
- No other environment dependency: constants are f64 arithmetic only; the expansion machinery (Two_Sum, Two_Product, splitter) is value-pure once initialized.

## 2. Options assessed

1. 1:1 pure Rust port (recommended baseline): translate predicates.c module-for-module, drop the x87 control-word blocks for 64-bit targets (with a doc comment explaining why that is safe), keep exactinit's runtime bisection. Zero external dependencies; bit-identical sign decisions to the pinned C build (ticket #5 flag pinning) are expected because operations map one-to-one (SSE2 double, no contraction).
2. georust robust crate (v1.2.0, May 2025, actively maintained): pure Rust port of the same predicates.c, used in production via the geo ecosystem. Excellent as a cross-check in unit tests (same sign decisions), not adopted as the core dependency: it exposes a narrower API surface with its own conventions, and keeping the machines' hottest leaf functions in-tree preserves control over inlining and later specialization.
3. robust-predicates crate (2022): compiles the ORIGINAL predicates.c with the cc crate and wraps it. Not a Rust port; reintroduces C compilation (against the port goals). Useful only as a bit-exact oracle in the diff test harness if desired.
4. mourner/robust-predicates (JS): the public proof that these predicates work unchanged without FP-env control on strict IEEE-754 double platforms (SSE2 era); used by Delaunator in production. This is the external justification that option 1 is correct, not just convenient.
5. Exact-arithmetic replacement (i128 rationals, EXACTFLOAT-style, or crates like exact/decimal): rejected: slower on the filter path and, more importantly, changes tie-case sign decisions relative to the C build, breaking the byte-identity oracle (#5) for zero practical benefit.

## 3. Performance expectations

- The adaptive machinery almost never falls out of the filter level for real inputs: the hot path is one vector determinant (orient2d: ~10 flops) and its cached constant bounds. A straight port has no structural disadvantage: no allocation, no branches beyond the error-bound checks. rustc optimizes these functions like gcc does at -O2.
- Precedent: georust robust powers production geometry in the geo stack; delaunator-rs uses its JS sibling port at C-competitive speeds.
- The predicates module is the hottest leaf code in Triangle: keeping it in-tree also lets the port inline orient2d/incircle into the mesh hot loops without cross-crate LTO concerns.

## 4. Recommendation for the spec

- Port predicates.c 1:1 into the Rust core as its own module (package-parallel to the C file), keep exactinit's runtime-derived constants, drop the x87 control-word code on 64-bit targets with documentation, and add a dual validation: unit tests cross-checking against georust robust, optional CI oracle against the cc-compiled original in the diff harness (ticket #5).
- This preserves identical sign decisions (mesh quality depends on them, the oracle compares on them), keeps zero foreign dependencies, and leaves performance on par by construction.