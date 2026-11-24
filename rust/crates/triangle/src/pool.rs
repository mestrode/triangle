//! The memory-pool machinery: the landing zone of ticket #9 (the binding
//! interface contract of ADR 0002 section 4). C: the pool functions of
//! triangle.c's Memory management routines section (poolzero, poolrestart,
//! poolinit, pooldeinit, poolalloc, pooldealloc, traversalinit, traverse).
//! The chokepoint (slots.rs) is its only client. Assignment:
//! docs/function-mapping.md.
