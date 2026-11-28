//! The formatter seam, re-exported from the triangle-io crate since
//! the interchange extraction (ADR 0006, issue #70): the module
//! itself moved with the format surface - its C printf forms, tests
//! and anchors are unchanged in triangle_io::formatter, and the
//! migration table in docs/function-mapping.md carries the move.
//! The re-export keeps this crate's public surface item-for-item
//! identical (the v0 API): the example-CLI renderers
//! (print_statistics) keep formatting through
//! [`g_padded`]/[`g_padded_left`] and the EPS renderer through [`g`].

pub use triangle_io::formatter::{g, g_padded, g_padded_left};
