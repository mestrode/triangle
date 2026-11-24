//! Shewchuk's exact predicates, ported 1:1 as pure Rust (tri_exactinit,
//! the expansion arithmetic, tri_counterclockwise/incircle/nonregular,
//! tri_orient3d, tri_findcircumcenter). The inert x87 control-word code
//! is dropped (deviation D5, ticket #3 findings). Assignment:
//! docs/function-mapping.md.
