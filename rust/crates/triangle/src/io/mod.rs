//! The mesh-bound I/O phase (the R4 io/ module map,
//! docs/architecture-spec.md section 5): the input transfer
//! (arrays.rs), the output phase (marshal.rs - numbering +
//! marshalling + serialization of the six artifacts behind one
//! record seam, both output forms, the issue #72 consolidation) and
//! the EPS renderer (eps.rs). The interchange formats themselves -
//! the readline/findfield/strtol/strtod parse seam, the readers, and
//! the header/line/trailer emission machinery - live in the
//! triangle-io crate since the interchange extraction (ADR 0006,
//! issue #70); the code here keeps the Mesh traversal orchestration
//! (numbering, jettison gates, the D29 edge-walk order, the -n
//! overlay) and calls into triangle-io's emitters for the bytes.
//! Assignment: the triangle_io.c and triangle-api file rows of
//! docs/function-mapping.md, plus the extraction's migration table
//! and the consolidation's migration table (issue #72).
//!
//! Boundary decisions (the deviation register and the PR bodies
//! carry the details; the writers themselves live in marshal.rs):
//! - The C FILE* NULL checks (file == NULL -> TRI_FILE_OPEN,
//!   triangle_io.c:30, 86, 141, 212, 271, 318; triangle_api.c fopen
//!   NULL -> TRI_FILE_OPEN at 498, 514, 530, 546) dissolve: the
//!   port's api.rs wrappers own the open, and File::open's error
//!   carries straight into Error::Io (the TRI_FILE_OPEN equivalence
//!   class; D31).
//! - C's unchecked fprintf return values in the writers surface as
//!   io::Result in the port (register row: the writers' seam, the
//!   file adapter of io/marshal.rs since issue #72). Every reader
//!   failure raises in triangle-io since rung 2: TRI_FILE_READ (a
//!   fgets NULL) is Error::Read with the same io::Error payload the
//!   stage-1 funnel arm carried (D31's equivalence class), and the
//!   raw -1 TODO returns are Error::Input with the funnel arm's
//!   exact TRI_FAILURE payload text; api.rs maps both back onto this
//!   crate's unchanged public Error at the seam.
//!
//! Test fixtures shared across the io tests live in the cfg(test)
//! fixtures module (io/fixtures.rs, the issue #72 consolidation).

pub(crate) mod arrays;
pub(crate) mod eps;
pub(crate) mod marshal;

#[cfg(test)]
pub(crate) mod fixtures;
