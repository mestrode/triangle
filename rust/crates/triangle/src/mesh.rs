//! Mesh-level lifecycle: mesh/pool initialization and teardown (dummyinit,
//! initializevertexpool, initializetrisubpools), the alloc/dealloc/traverse
//! wrappers (triangle/subseg/vertex/badsubseg), getvertex, triangleinit/
//! triangledeinit, and the constructors maketriangle/makesubseg.
//! C: triangle.c sections Memory management routines (tail) and
//! Constructors. Assignment: docs/function-mapping.md.
