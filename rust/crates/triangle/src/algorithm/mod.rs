//! The algorithm sections of triangle.c, regrouped at the R4 seams
//! (docs/architecture-spec.md section 5): the point-location primitives
//! (locate), the insertion/deletion transforms (transform), the Delaunay
//! constructions (delaunay), the PSLG segment machinery (segments), and
//! the quality refinement engine (quality, refine). Assignment of every
//! C function to its module: docs/function-mapping.md.

pub(crate) mod delaunay;
pub(crate) mod locate;
pub(crate) mod quality;
pub(crate) mod refine;
pub(crate) mod segments;
pub(crate) mod transform;
