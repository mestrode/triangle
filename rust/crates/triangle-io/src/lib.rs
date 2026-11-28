//! The Triangle interchange file formats over neutral, self-contained
//! types (ADR 0006, issue #70): parsing and serialization for the six
//! file formats Triangle programs exchange - .node, .poly, .ele,
//! .edge, .neigh, .area - with no dependency on (and no reference to)
//! the meshing core. Third-party crates read and write these formats
//! here without linking the ported mesher; the `triangle` crate
//! consumes this crate for its own file surface, so the dependency
//! direction is triangle -> triangle-io only.
//!
//! C anchors: the read/write functions of src/Triangle/triangle_io.c
//! (this repo's C core), transliterated 1:1; the module map and the
//! extraction migration table live in docs/function-mapping.md.
//! Deliberate departures: docs/deviation-register.md (this crate adds
//! none).
//!
//! - Pure std: io::Read/io::Write based, zero dependencies, zero
//!   unsafe (the workspace forbid), no libc.
//! - Neutral types (module `types`): owned structs describing file
//!   contents only - points with coordinates/markers/attributes,
//!   segments, holes, regions, elements, edges, neighbors, areas, and
//!   the header fields (counts, dimensions, nattributes, nmarkers,
//!   firstnumber). Nothing references the meshing core.
//! - Parsing semantics mirror the C readers exactly (the
//!   readline/findfield/strtol/strtod seam, firstnumber 0/1 handling,
//!   marker and attribute presence rules, blank-line and comment
//!   tolerance); the writers reproduce the C printf formats
//!   byte-for-byte through the formatter seam. The .edge and .neigh
//!   readers have no C counterpart (C only writes those formats);
//!   they parse the writer grammar under the readers' own
//!   conventions, and the .area writer composes the reader's grammar
//!   the same way - all pinned by round-trip construction tests,
//!   never via the C oracle binary.
//! - Errors: this crate's own Error (module `error`); consumers
//!   convert at their seam (the triangle crate maps it onto its own
//!   unchanged public taxonomy).

pub mod error;
pub mod formatter;
pub mod parse;
pub mod types;
pub mod write;

pub use error::{Error, Result};
pub use parse::{
    read_area_file, read_edge_file, read_ele_file, read_neigh_file, read_node_file, read_poly_file,
};
pub use types::{AreaFile, EdgeFile, EleFile, Hole, NeighFile, NodeFile, Point2, PolyFile, Region};
pub use write::{
    write_area_file, write_edge_file, write_ele_file, write_neigh_file, write_node_file,
    write_poly_file,
};
