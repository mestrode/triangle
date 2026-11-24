//! The public entry points and the owned Mesh handle: the idiomatic mirror
//! of the surviving surface of triangle_api.c (triangulate, Mesh::load_mesh
//! / refine / output / statistics / quality / check_mesh / check_delaunay
//! / memory_bytes) plus the public MeshOutput, Statistics, Quality,
//! BoundingBox types.
//! 
//! ADR 0004 section 5.3; C correspondence in docs/function-mapping.md.
