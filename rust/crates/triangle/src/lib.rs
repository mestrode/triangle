//! The Rust port of Triangle (Shewchuk's Triangle 1.6 + the aCute refinement).
//!
//! Port strategy: the core algorithms are transliterated function-for-function
//! (the module map and C correspondence live in docs/function-mapping.md,
//! the deliberate departures in docs/deviation-register.md) and verified
//! byte-identically against the pinned C oracle over the harness corpus
//! (tools/oracle/, see docs/adr/0001-port-strategy.md). On the serial path
//! this parity guarantee is permanent.
//!
//! On day one only the API shell described in
//! docs/adr/0004-crate-api-architecture.md exists; the core modules below
//! are transliteration zones filled by the stage-1 tickets.
//!
//! - 64-bit targets only (x86-64, aarch64): the packed-handle data model and
//!   the byte-parity gate are pinned to 64-bit (docs/adr/0002).
//! - aCute is compiled in by default via the acute feature; build with
//!   --no-default-features for the C NO_ACUTE equivalent.
//! - The thread-count knob for the (future) internal multithreading is the
//!   TRIANGLE_THREADS environment variable, read by the triangle-cli
//!   binary; the library API does not expose a thread switch yet.

#![cfg_attr(not(target_pointer_width = "64"), compile_error(
    "the Rust port targets 64-bit platforms only (map #1 scope ruling, docs/adr/0002)"
))]

// ---------------------------------------------------------------------------
// Public API shell modules (idiomatic from day one; ADR 0004, section 5).
// They re-export through the crate root once their types exist:
//     pub use api::{triangulate, Mesh, MeshOutput, Statistics, Quality, BoundingBox};
//     pub use error::{Error, Result};
//     pub use input::{MeshInput, Point2};
//     pub use options::{Options, VertexSample};
// ---------------------------------------------------------------------------

pub mod api;
pub mod error;
pub mod input;
pub mod options;

// ---------------------------------------------------------------------------
// Transliterated core (private; fills with the stage-1 implementation
// tickets, assignment per the module map: docs/function-mapping.md).
// ---------------------------------------------------------------------------

#[cfg(feature = "acute")]
mod acute;
mod construct;
mod debug;
mod delaunay;
mod handles;
mod ioarrays;
mod locate;
mod mesh;
mod pool;
mod predicates;
mod quality;
mod refine;
mod slots;
mod transform;
mod util;

// I/O phase (module assignment decided in ADR 0004; content lands with the
// I/O-phase tickets).
mod eps;
mod io;

/// Crate/lib version, mirroring triangle_version()'s numeric form and the
/// C Triangle_VERSION 1.7.0 in src/CMakeLists.txt (the fork's numbering).
pub const VERSION: (u32, u32, u32) = (1, 7, 0);

/// Whether the aCute refinement (-U) is compiled in, mirroring
/// triangle_version()'s has_acute output.
pub const HAS_ACUTE: bool = cfg!(feature = "acute");
