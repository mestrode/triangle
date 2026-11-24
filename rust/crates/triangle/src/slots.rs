//! The single private accessor chokepoint: the Mesh storage struct, the
//! runtime row shapes (TriShape/VertexShape), all storage accessors, and
//! the handle-macro methods that touch storage (sym, bond, org/dest/apex,
//! tspivot/tsbond, ...). No reference into pool storage escapes a call.
//! C: triangle.h triangle/vertex/subseg slot formulas; ADR 0002.
