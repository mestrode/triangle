//! The pool machinery of the port: one uniform word pool type (ADR 0003).
//!
//! C: the "Memory management routines" section of triangle.c (poolzero,
//! poolrestart, poolinit, pooldeinit, poolalloc, pooldealloc,
//! traversalinit, traverse) and `struct memorypool` (triangle.h:352-366).
//! Assignment: the pool rows of docs/function-mapping.md; the chokepoint
//! (mesh/mesh/slots.rs) is its only client. Method names are the resolved pool
//! contract of ADR 0003 section 3 ("signatures are the contract"); every
//! method documents its C analog so the mapping stays greppable:
//! poolzero/pooldeinit dissolve into construction/Drop (D11), poolinit is
//! `WordPool::new`, poolalloc is `alloc`, pooldealloc is `dealloc`,
//! poolrestart is `restart`, traversalinit is `traversal`, traverse is
//! `traverse_next`.
//!
//! The mechanical mirror this module pins (ADR 0003, binding): Copy `u32`
//! row ids replace item pointers; deallocated rows are pushed on an
//! exact-LIFO dead stack threaded through slot 0 of the dying row (deviation
//! D4; reuse order is output-observable, hence exact); traversals walk
//! allocation order through the live high-water frontier and never
//! rearrange rows (allocation order is the oracle's output ordering); the
//! sentinels are reserved fixed rows excluded from allocation, traversal
//! and the dead stack (D3). The physical store deliberately diverges (ADR
//! 0003 section 2, deviation register): one flat `Vec<u64>` per pool with
//! stride-fixed rows and amortized-doubling growth through `try_reserve`
//! (the initial reservation mirrors C's first-block hints; capacity only,
//! rows materialize as ids are handed out). Payload meaning is the
//! data-model's business, all of it behind mesh/mesh/slots.rs.
//!
//! R1 accessor-invariant rung (ADR 0002 section 3): the five core-data
//! invariants are enforced at this seam in debug builds only - range (a)
//! on every word access (the materialized-row frontier) and on every
//! decoded handle, null discipline (b) on every decode and on every
//! alloc and traversal yield (never the null id; the reserved null row
//! itself stays readable as never-written zeros - the port's substitute
//! for the NULL-vertex deref the legal C flow never performs), slot-role
//! (c) on every access, dead-stack discipline (d) by the cfg-gated
//! integrity walk at every push/pop, exact-LIFO reuse (e) by
//! construction and the pool tests. Release builds compile no check at
//! all; debug_assert and cfg(debug_assertions) leave no trace in the
//! release binary.
//!
//! Stage-1 note: items here are consumed by the not-yet-transliterated
//! traversing modules per docs/function-mapping.md; the blanket below
//! silences the ahead-of-use dead_code and is removed when they land.
#![allow(dead_code)]

use std::cmp::Ordering;

use crate::mesh::handles::NULL_ROW;

/// The pool layer's only failure: allocation failure (C prints
/// "Error:  Out of memory." and exits 1 at the trimalloc site,
/// triangle.c:318-324).
///
/// Current flow (the stage-1 `InternalStatus` bridge of ADR 0004 section 6
/// is dissolved per D8's rung-2 resolution): the traversing core propagates
/// `Oom` directly, the hot segment chain carries it as
/// `SegmentChainError::Oom` (algorithm/segments.rs), and the `From<Oom> for Error`
/// bridge (error.rs, D26) maps it to the public `Error::OutOfMemory` at
/// the api boundary - exactly the process end C's fatal site performed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) struct Oom;

/// One uniform word pool (ADR 0003 section 1). All eight mesh pools
/// (triangles, subsegs, vertices, viri, badsubsegs, badtriangles,
/// flipstackers, splaynodes) are instances of this type; payload words are
/// opaque here and typed at the chokepoint.
pub(crate) struct WordPool {
    /// Flat word store: row-major, word `slot` of row `id` at
    /// `words[id * stride + slot]` (ADR 0003 section 1).
    words: Vec<u64>,
    /// Rows with reserved capacity (the null row and the sentinel row
    /// included; C's block chain collapsed into one reservation).
    row_capacity: usize,
    /// Rows materialized in `words` (`== words.len() / stride`).
    handed_rows: usize,
    /// Words per row (C itembytes / 8; fixed at init, never 0 for a live
    /// pool - a stride-0 pool mimics C's post-poolzero never-inited state).
    stride: u32,
    /// C itemsfirstblock (logical block geometry).
    first_block_items: u32,
    /// C itemsperblock (logical block geometry).
    items_per_block: u32,
    /// C deaditemstack: the most recently deallocated row id; the null id
    /// terminates the chain (D4; slot 0 of a dead row holds the next id).
    dead_head: u32,
    /// Rows orphaned by clear_dead_stack (C highorder's
    /// m->vertices.deaditemstack = NULL, triangle.c:7639): dead rows
    /// whose chain was dropped and that are never handed out again;
    /// C's pool keeps no invariant over them, so the debug invariant
    /// discounts them through this counter.
    cleared_dead: u64,
    /// C items: live rows.
    items: u64,
    /// C maxitems: fresh-allocation high water since init/restart; counts
    /// fresh ids only, never dead reuse, never drops on dealloc.
    maxitems: u64,
    /// C nextitem, as an id: the next row to hand out fresh (monotonic).
    next_fresh: u32,
    /// True only for the triangle and subseg pools (sentinel = id 1, D3).
    has_sentinel: bool,
}

impl WordPool {
    /// C: poolinit (triangle.c:814).
    ///
    /// `byte_count` rounds up to whole 8-byte rows exactly like C's
    /// alignbytes rounding: C takes `alignbytes = max(alignment,
    /// sizeof(VOID*))`, and both C alignment arguments (4 and 8) collapse
    /// to 8 on the port's 64-bit targets, so every row is whole words. A
    /// `first_block_items` of 0 means the first block equals every block
    /// (C's firstitemcount == 0 case). `sentinel` reserves row id 1 for
    /// the D3 sentinel row; row id 0 is the null id everywhere.
    pub(crate) fn new(
        byte_count: u32,
        items_per_block: u32,
        first_block_items: u32,
        sentinel: bool,
    ) -> Result<WordPool, Oom> {
        let stride = if byte_count == 0 {
            // dormant: no poolinit performed (C's poolzero state)
            0
        } else {
            (byte_count as usize).div_ceil(8)
        };
        let first_block = if first_block_items == 0 {
            items_per_block
        } else {
            first_block_items
        };
        let base_rows: usize = if sentinel { 2 } else { 1 };
        // ADR 0003 section 2: the initial Vec reservation equals C's first
        // block (the memory-parity hint); checked arithmetic makes any
        // insane request an Oom instead of an overflow.
        let target_rows = base_rows.checked_add(first_block as usize).ok_or(Oom)?;
        let total_words = target_rows.checked_mul(stride).ok_or(Oom)?;
        let mut words: Vec<u64> = Vec::new();
        words.try_reserve(total_words).map_err(|_| Oom)?;
        // the null row and the sentinel row exist, zeroed, from the start
        words.resize(base_rows * stride, 0);
        Ok(WordPool {
            words,
            row_capacity: target_rows,
            handed_rows: base_rows,
            stride: stride as u32,
            first_block_items: first_block,
            items_per_block,
            dead_head: NULL_ROW,
            cleared_dead: 0,
            items: 0,
            maxitems: 0,
            next_fresh: base_rows as u32,
            has_sentinel: sentinel,
        })
    }

    /// The id space: fresh ids run from 2 when the pool has a sentinel row
    /// (D3), else from 1 (row 0 is the null id everywhere).
    #[inline]
    pub(crate) fn first_fresh_id(&self) -> u32 {
        if self.has_sentinel {
            2
        } else {
            1
        }
    }

    /// The reserved sentinel row id (D3), if this pool has one.
    #[inline]
    pub(crate) fn sentinel_id(&self) -> Option<u32> {
        if self.has_sentinel {
            Some(1)
        } else {
            None
        }
    }

    /// Debug-only decoded-handle range check (ADR 0002 section 3,
    /// invariants a and b): a handle fresh out of decode names a
    /// materialized row of this pool - never the null id (no legal
    /// decode produces it: the all-zero word is never decoded, exactly
    /// as C never dereferences a NULL slot), and strictly below the
    /// materialized row frontier (the pool counters are the whole state
    /// this needs; the reserved sentinel row, D3, is inside the space).
    /// The chokepoint decode sites (mesh/mesh/slots.rs) and getvertex (mesh/mod.rs)
    /// run this on every decoded or computed row id; release builds
    /// compile it away entirely.
    #[inline]
    pub(crate) fn debug_assert_row_in_space(&self, id: u32) {
        debug_assert!(
            id != NULL_ROW,
            "the null id is never a live-or-sentinel row (C never dereferences NULL)"
        );
        debug_assert!(
            (id as usize) < self.handed_rows,
            "row id {} outside the pool id space (corrupt stored handle?)",
            id
        );
    }

    /// Words per row (C itembytes / 8).
    #[inline]
    pub(crate) fn stride(&self) -> u32 {
        self.stride
    }

    /// C: poolalloc (triangle.c:867): pop the exact-LIFO dead stack first;
    /// else hand out the next fresh id (monotonically increasing).
    ///
    /// Dead-stack pops and in-capacity fresh allocations cannot fail,
    /// matching C; only fresh growth can, through `try_reserve` (below).
    ///
    /// Invariants (ADR 0002 section 3): exact LIFO (e) - the dead pop is
    /// the chain head; dead-stack discipline (d) - the pop reads slot 0
    /// of the dead head, pool-private between dealloc and realloc; the
    /// null id (b) and the reserved sentinel row (D3) are never handed
    /// out (debug-pinned below).
    pub(crate) fn alloc(&mut self) -> Result<u32, Oom> {
        debug_assert!(self.stride > 0, "alloc from a dormant (never-inited) pool");
        let id = if self.dead_head != NULL_ROW {
            // Take the first item in the dead list (C poolalloc's first branch);
            // slot 0 of the dead row holds the next dead id (D4).
            let id = self.dead_head;
            self.dead_head = self.word(id, 0) as u32;
            id
        } else {
            // fresh frontier contiguity: maxitems is exactly the number of
            // fresh ids ever handed (ADR 0003 section 3 debug inventory)
            debug_assert!(
                self.maxitems == self.next_fresh as u64 - self.first_fresh_id() as u64,
                "fresh frontier must be contiguous"
            );
            let id = self.next_fresh;
            self.ensure_row(id)?;
            self.next_fresh = id.checked_add(1).ok_or(Oom)?;
            self.maxitems += 1;
            id
        };
        self.items += 1;
        // Sentinel and null guards (ADR 0002 section 3, invariants a/b and
        // the D3 discipline): neither id is ever a legal allocation; a
        // dead-chain pop is the only path that could surface one, and the
        // debug_assert fires before the id escapes.
        debug_assert!(id != NULL_ROW, "alloc must never hand out the null id");
        debug_assert!(
            Some(id) != self.sentinel_id(),
            "alloc must never hand out the reserved sentinel row (D3)"
        );
        self.debug_assert_dead_stack();
        Ok(id)
    }

    /// C: pooldealloc (triangle.c:924): push onto the dead stack, threading
    /// the previous dead-head id through slot 0 of the dying row (D4).
    ///
    /// Invariants (ADR 0002 section 3): the null id (b) and the reserved
    /// sentinel row (D3) are rejected; the push makes slot 0 of the dying
    /// row pool-private until its realloc (d); the push order keeps the
    /// reuse exact-LIFO (e).
    pub(crate) fn dealloc(&mut self, id: u32) {
        debug_assert!(id != NULL_ROW, "the null id is never allocated");
        debug_assert!(
            Some(id) != self.sentinel_id(),
            "the sentinel row is never deallocated"
        );
        debug_assert!(self.items > 0, "dealloc without a live item");
        self.set_word(id, 0, self.dead_head as u64);
        self.dead_head = id;
        self.items -= 1;
        self.debug_assert_dead_stack();
    }

    /// O(1) word access (ADR 0002 section 4.2): the chokepoint's innermost
    /// accessors. Bounds-checked indexing. Invariants (ADR 0002 section
    /// 3, debug builds only): slot-role (c) `slot < stride` here; range
    /// (a) - the row id names a materialized row - below. The null
    /// discipline (b) is decode-side per ADR 0002 section 3 (mesh/mesh/handles.rs
    /// plus the mesh/mesh/slots.rs decode sites): the reserved null row is inside
    /// the materialized frontier and reads as its never-written zeros,
    /// the port's defined substitute for the NULL-vertex deref the
    /// legal C flow never performs (rightofhyperbola on an outer-space
    /// handle; C's dummytri carries NULL vertices, triangle.c:1049).
    #[inline]
    pub(crate) fn word(&self, id: u32, slot: usize) -> u64 {
        debug_assert!(slot < self.stride as usize, "slot out of row");
        debug_assert!(
            (id as usize) < self.handed_rows,
            "row id {} outside the pool id space (corrupt stored handle?)",
            id
        );
        self.words[self.row_index(id, slot)]
    }

    /// The write mirror of [`WordPool::word`]: the same invariants (ADR
    /// 0002 section 3) are debug_asserted - slot-role (c) and the range
    /// (a) frontier check on the row id (the null discipline (b) is
    /// decode-side; see `word`).
    #[inline]
    pub(crate) fn set_word(&mut self, id: u32, slot: usize, w: u64) {
        debug_assert!(slot < self.stride as usize, "slot out of row");
        debug_assert!(
            (id as usize) < self.handed_rows,
            "row id {} outside the pool id space (corrupt stored handle?)",
            id
        );
        let index = self.row_index(id, slot);
        self.words[index] = w;
    }

    #[inline]
    fn row_index(&self, id: u32, slot: usize) -> usize {
        id as usize * self.stride as usize + slot
    }

    /// C: poolrestart (triangle.c:774): deallocate all items logically; the
    /// store and its capacity are retained and the same rows are handed
    /// again from the bottom ("like C handing the same block addresses
    /// again"), so the fresh/reuse sequence stays byte-identical.
    ///
    /// Invariants (ADR 0002 section 3): the dead chain is dropped (every
    /// row becomes fresh-handed again) and the debug integrity walk runs
    /// on the emptied chain.
    pub(crate) fn restart(&mut self) {
        self.items = 0;
        self.maxitems = 0;
        self.dead_head = NULL_ROW;
        self.cleared_dead = 0;
        self.next_fresh = self.first_fresh_id();
        self.debug_assert_dead_stack();
    }

    /// C: m->vertices.deaditemstack = (VOID *) NULL in highorder (the
    /// "dead items in the pool of nodes cannot be allocated for the
    /// extra nodes" reset, triangle.c:7634-7639): drop the dead-item
    /// chain without touching items or the allocation frontier, so
    /// highorder's midpoint allocations take fresh rows instead of dead
    /// ones, and the primary nodes keep the lower indices (the comment
    /// 7634-7638 rationale). The dropped rows are dead forever; C's
    /// chain is simply unreachable, and the counter mirrors that for
    /// the debug invariant. highorder (algorithm/algorithm/refine.rs) is the only caller.
    ///
    /// Invariant (ADR 0002 section 3 d): the dropped rows stay dead
    /// forever (never re-handed); `cleared_dead` keeps the recorded dead
    /// count exact so the integrity walk stays checkable.
    pub(crate) fn clear_dead_stack(&mut self) {
        let mut current = self.dead_head;
        let mut steps = 0u64;
        while current != NULL_ROW {
            steps += 1;
            current = self.word(current, 0) as u32;
        }
        self.cleared_dead += steps;
        self.dead_head = NULL_ROW;
        self.debug_assert_dead_stack();
    }

    /// C: `items` - the live count (the m->items shape).
    pub(crate) fn live_count(&self) -> u64 {
        self.items
    }

    /// C: `maxitems` - the fresh high water (the m->maxitems shape; the
    /// allocation frontier traversals read live).
    pub(crate) fn high_water(&self) -> u64 {
        self.maxitems
    }

    /// C itemsfirstblock (logical geometry; sampleraise and getvertex read
    /// it, triangle.c:2474-2500 / 1343-1369).
    pub(crate) fn first_block_items(&self) -> u32 {
        self.first_block_items
    }

    /// C itemsperblock (logical geometry).
    pub(crate) fn block_items(&self) -> u32 {
        self.items_per_block
    }

    /// The id of index 0 of block `block`, sentinel shift included: block 0
    /// holds the first-block items, every later block `items_per_block`
    /// items (C getvertex's block walk).
    pub(crate) fn block_base_id(&self, block: u64) -> u32 {
        let first = self.first_fresh_id() as u64;
        if block == 0 {
            first as u32
        } else {
            let base =
                first + self.first_block_items as u64 + (block - 1) * self.items_per_block as u64;
            debug_assert!(base <= u32::MAX as u64, "block base beyond the id space");
            base as u32
        }
    }

    #[inline]
    fn block_and_index(&self, id: u32) -> (u64, u64) {
        debug_assert!(
            self.items_per_block > 0,
            "block geometry of a dormant pool is undefined"
        );
        let first = self.first_fresh_id() as u64;
        debug_assert!(
            id as u64 >= first,
            "id below the first fresh one has no block"
        );
        let rel = id as u64 - first;
        if rel < self.first_block_items as u64 {
            (0, rel)
        } else {
            let past = rel - self.first_block_items as u64;
            let per = self.items_per_block as u64;
            (1 + past / per, past % per)
        }
    }

    /// ADR 0003 section 5: the C raw-pointer ORDER comparisons mirrored
    /// arithmetically (`oracle_address_order(a, b) == Less` mirrors the C
    /// test `a_ptr < b_ptr` at the five output-visible sites:
    /// triangle.c:1655, 7653, 8021; triangle_io.c:232; eps_writer.c:117).
    ///
    /// Under the pinned oracle platform (glibc 2.36, Linux x86-64, verified
    /// empirically, ADR 0003), fresh large blocks land top-down at
    /// descending addresses while rows ascend within a block, so "earlier
    /// in pointer order" means: an earlier-allocated block, or a lower
    /// index in the same block. This is a platform-model assumption inside
    /// the C mirror (deviation register); the "pen" corpus family is the
    /// canary.
    pub(crate) fn oracle_address_order(&self, a: u32, b: u32) -> Ordering {
        if a == b {
            return Ordering::Equal;
        }
        let (block_a, index_a) = self.block_and_index(a);
        let (block_b, index_b) = self.block_and_index(b);
        if block_a != block_b {
            // a higher row in an EARLIER-allocated block = larger address
            if block_a > block_b {
                Ordering::Less
            } else {
                Ordering::Greater
            }
        } else {
            index_a.cmp(&index_b)
        }
    }

    /// C: traversalinit (triangle.c:940): prepare to traverse the entire
    /// list of items. The cursor is plain data (no mutable pool state).
    pub(crate) fn traversal(&self) -> Traversal {
        Traversal {
            next: self.first_fresh_id(),
        }
    }

    /// C: traverse (triangle.c:970): the next id in allocation order.
    ///
    /// Dead rows are INCLUDED (dead detection stays a data-model concern,
    /// the killtri NULL pattern through the chokepoint); the sentinel row
    /// is skipped by construction (the walk starts above it); the frontier
    /// is read live at each step, so a pool growing mid-traversal extends
    /// the walk exactly like C's pathitem/nextitem comparison.
    /// The D3 sentinel exclusion and the null row are debug-pinned on
    /// every yielded id (ADR 0002 section 3).
    pub(crate) fn traverse_next(&self, t: &mut Traversal) -> Option<u32> {
        let frontier = self.first_fresh_id() as u64 + self.maxitems;
        if t.next as u64 >= frontier {
            None
        } else {
            let id = t.next;
            t.next += 1;
            // D3 discipline (ADR 0002 section 3): the walk starts above the
            // reserved rows by construction; the debug pins catch any
            // cursor arithmetic that could ever yield one.
            debug_assert!(id != NULL_ROW, "traversal must never yield the null row");
            debug_assert!(
                Some(id) != self.sentinel_id(),
                "traversal must never yield the sentinel row (D3)"
            );
            Some(id)
        }
    }

    /// Materialize row `id` (the C poolalloc fresh branch's growth).
    ///
    /// Growth is Vec's amortized doubling (ADR 0003 section 2: the
    /// deliberate departure from C's fixed block steps - fixed-step
    /// regrowth of a flat store aggregates quadratic word copying) through
    /// `try_reserve`, so OOM becomes `Err(Oom)` instead of C's
    /// "Error: Out of memory." + exit(1). Rows are defined zeroed when
    /// materialized; C rows come from malloc'uninit memory, and the
    /// transliterated control flow never reads a slot it did not write
    /// first (the corpus keeps that honest), so zero-fill is the benign
    /// deterministic superset. After `restart` the re-handed rows are NOT
    /// re-cleared, matching C handing the same block addresses again.
    fn ensure_row(&mut self, id: u32) -> Result<(), Oom> {
        debug_assert!(self.stride > 0);
        let need = id as usize + 1;
        if need <= self.handed_rows {
            return Ok(());
        }
        debug_assert!(
            need == self.handed_rows + 1,
            "fresh ids are materialized one row at a time, contiguously"
        );
        if need > self.row_capacity {
            let target = (self.row_capacity.saturating_mul(2)).max(need);
            let additional = target
                .checked_sub(self.row_capacity)
                .and_then(|rows| rows.checked_mul(self.stride as usize))
                .ok_or(Oom)?;
            self.words.try_reserve(additional).map_err(|_| Oom)?;
            self.row_capacity = target;
        }
        let new_len = need * self.stride as usize;
        self.words.resize(new_len, 0);
        self.handed_rows = need;
        Ok(())
    }

    /// Debug-only dead-chain integrity walk (ADR 0002 section 3,
    /// invariant d; ADR 0003 section 3), the mechanism ADR 0002 names:
    /// walk the dead chain at the push/pop seam - every `alloc`,
    /// `dealloc`, `restart` and `clear_dead_stack` - and require it to
    /// reach the null id in exactly `maxitems - items - cleared_dead`
    /// steps, so a duplicate or a loop fires the bound ("longer than the
    /// recorded dead count") and a truncated chain fires the length
    /// equality. The walk is bounded by the recorded dead count, so a
    /// corrupt chain terminates at the bound instead of hanging; its
    /// cost is O(dead chain length) per operation, debug builds only.
    /// Because every pool of the mesh is one WordPool (the triangle,
    /// subsegment and vertex pools plus every typed small pool of ADR
    /// 0002 section 4 item 8), this one seam covers them all; the aCute
    /// scratch (D23) is resize-only typed Vec state with no dead stack,
    /// hence no walk. cfg(debug_assertions)-gated, not just
    /// branch-guarded: the release binary carries no trace of the
    /// walker.
    #[cfg(debug_assertions)]
    #[inline]
    fn debug_assert_dead_stack(&self) {
        let expected = self.maxitems - self.items - self.cleared_dead;
        let mut current = self.dead_head;
        let mut steps = 0u64;
        while current != NULL_ROW {
            steps += 1;
            assert!(
                steps <= expected,
                "dead chain corrupt: longer than the recorded dead count (duplicate/loop)"
            );
            current = self.word(current, 0) as u32;
        }
        assert_eq!(
            steps, expected,
            "dead chain corrupt: length != maxitems - items"
        );
    }

    /// The release mirror of the debug dead-chain walk: nothing at all
    /// (cfg-gated out; no trace in the release binary).
    #[cfg(not(debug_assertions))]
    #[inline]
    fn debug_assert_dead_stack(&self) {}

    /// Reserved row capacity (test observation of the restart-keeps-capacity
    /// and amortized-doubling contracts).
    #[cfg(test)]
    pub(crate) fn test_row_capacity(&self) -> usize {
        self.row_capacity
    }
}

/// The traversal cursor: the next id to hand out (C pathitem; plain data,
/// ADR 0002 section 4 - no storage reference escapes).
#[derive(Clone, Copy, Debug)]
pub(crate) struct Traversal {
    next: u32,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pool(bytes: u32, per_block: u32, first: u32, sentinel: bool) -> WordPool {
        WordPool::new(bytes, per_block, first, sentinel).expect("pool init")
    }

    #[test]
    fn dead_stack_reuse_is_exact_lifo() {
        // D4: the C deaditemstack; reuse order is output-observable.
        let mut p = pool(48, 16, 0, false);
        let a = p.alloc().unwrap();
        let b = p.alloc().unwrap();
        let c = p.alloc().unwrap();
        assert_eq!((a, b, c), (1, 2, 3));
        // push order a, then b, then c: pops come back reversed, then fresh
        p.dealloc(a);
        p.dealloc(b);
        p.dealloc(c);
        assert_eq!(p.live_count(), 0);
        assert_eq!(p.alloc().unwrap(), c);
        assert_eq!(p.alloc().unwrap(), b);
        assert_eq!(p.alloc().unwrap(), a);
        assert_eq!(p.alloc().unwrap(), 4);
        assert_eq!(p.live_count(), 4);
    }

    #[test]
    fn interleaved_alloc_dealloc_keeps_lifo_and_counts() {
        let mut p = pool(48, 8, 0, false);
        let a = p.alloc().unwrap();
        p.dealloc(a);
        let b = p.alloc().unwrap();
        assert_eq!(b, a, "immediate re-hand after push");
        let c = p.alloc().unwrap();
        let d = p.alloc().unwrap();
        p.dealloc(c);
        p.dealloc(d);
        assert_eq!(p.live_count(), 1);
        assert_eq!(p.high_water(), 3);
        assert_eq!(p.alloc().unwrap(), d);
        assert_eq!(p.alloc().unwrap(), c);
        assert_eq!(p.alloc().unwrap(), 4);
        assert_eq!(p.high_water(), 4);
    }

    #[test]
    fn restart_retains_capacity_and_restarts_the_frontier() {
        let mut p = pool(48, 4, 16, false);
        for _ in 0..20 {
            p.alloc().unwrap();
        }
        let capacity = p.test_row_capacity();
        assert_eq!(p.high_water(), 20);
        p.restart();
        assert_eq!(p.live_count(), 0);
        assert_eq!(p.high_water(), 0);
        assert_eq!(
            p.test_row_capacity(),
            capacity,
            "restart keeps the store (C poolrestart retains all blocks)"
        );
        // the same rows are handed from the bottom again, and no fresh
        // reservation happens below the retained capacity
        assert_eq!(p.alloc().unwrap(), p.first_fresh_id());
        for _ in 1..20 {
            p.alloc().unwrap();
        }
        assert_eq!(p.test_row_capacity(), capacity);
        assert_eq!(p.high_water(), 20);
        assert_eq!(p.live_count(), 20);
    }

    #[test]
    fn traversal_walks_allocation_order_dead_rows_included() {
        let mut p = pool(8, 32, 0, false);
        for _ in 0..5 {
            p.alloc().unwrap();
        }
        p.dealloc(2);
        p.dealloc(4);
        // 3 live, frontier unchanged: the walk yields all five ids in order,
        // dead ones included (dead detection is a data-model concern).
        let mut t = p.traversal();
        let mut walked = vec![];
        while let Some(id) = p.traverse_next(&mut t) {
            walked.push(id);
        }
        assert_eq!(walked, vec![1, 2, 3, 4, 5]);
        assert_eq!(p.live_count(), 3);
        assert_eq!(p.high_water(), 5);
    }

    #[test]
    fn traversal_frontier_is_read_live() {
        let mut p = pool(8, 8, 0, false);
        p.alloc().unwrap();
        p.alloc().unwrap();
        let mut t = p.traversal();
        assert_eq!(p.traverse_next(&mut t), Some(1));
        // growth mid-traversal extends the walk exactly like C
        let grown = p.alloc().unwrap();
        assert_eq!(grown, 3);
        assert_eq!(p.traverse_next(&mut t), Some(2));
        assert_eq!(p.traverse_next(&mut t), Some(3));
        assert_eq!(p.traverse_next(&mut t), None);
        // an empty pool's traversal stops immediately
        p.restart();
        let mut t2 = p.traversal();
        assert_eq!(p.traverse_next(&mut t2), None);
    }

    #[test]
    fn sentinel_row_is_reserved_never_allocated_or_traversed() {
        let mut p = pool(48, 4092, 0, true);
        assert_eq!(p.sentinel_id(), Some(1));
        let mut ids = vec![];
        for _ in 0..5 {
            ids.push(p.alloc().unwrap());
        }
        assert_eq!(ids, vec![2, 3, 4, 5, 6]);
        let mut t = p.traversal();
        let mut walked = vec![];
        while let Some(id) = p.traverse_next(&mut t) {
            walked.push(id);
        }
        assert_eq!(walked, vec![2, 3, 4, 5, 6]);
        assert!(!walked.contains(&1));
        // high water counts fresh rows exactly like C (sentinels never counted)
        assert_eq!(p.high_water(), 5);
    }

    #[test]
    #[should_panic(expected = "the sentinel row is never deallocated")]
    fn sentinel_cannot_be_deallocated() {
        let mut p = pool(48, 8, 0, true);
        p.dealloc(1);
    }

    #[test]
    fn oom_routes_through_try_reserve_on_capacity_overflow() {
        // stride = ceil(u32::MAX / 8) words and first block u32::MAX rows
        // overflow the Vec element-capacity limit: try_reserve answers Err
        // deterministically without any allocation attempt (capacity
        // overflow path, no dependence on machine memory).
        let res = WordPool::new(u32::MAX, u32::MAX, u32::MAX, false);
        assert!(res.is_err());
        match res {
            Err(Oom) => {}
            Ok(_) => panic!("capacity overflow must be Err(Oom)"),
        }
    }

    #[test]
    fn growth_is_amortized_doubling_of_row_capacity() {
        // capacity counts the null row too: base 1 + first block 1 row
        // covers exactly id 1 (the C first block covers first-block items);
        // every row past that is amortized doubling of the whole store
        let mut p = pool(8, 4, 1, false);
        assert_eq!(p.test_row_capacity(), 2);
        p.alloc().unwrap(); // id 1, in capacity
        assert_eq!(p.test_row_capacity(), 2);
        p.alloc().unwrap(); // id 2: beyond the first block -> doubling
        assert_eq!(p.test_row_capacity(), 4);
        p.alloc().unwrap(); // id 3, in capacity
        assert_eq!(p.test_row_capacity(), 4);
        p.alloc().unwrap(); // id 4: doubling again
        assert_eq!(p.test_row_capacity(), 8);
        p.alloc().unwrap(); // id 5, in capacity
        assert_eq!(p.test_row_capacity(), 8);
        assert_eq!(p.high_water(), 5);
    }

    #[test]
    fn block_geometry_and_oracle_address_order() {
        // ADR 0003 section 5: block index descending, row ascending within a block
        let p = pool(8, 5, 10, true);
        assert_eq!(p.first_block_items(), 10);
        assert_eq!(p.block_items(), 5);
        assert_eq!(p.block_base_id(0), 2);
        assert_eq!(p.block_base_id(1), 12);
        assert_eq!(p.block_base_id(2), 17);
        // same block: ascending row wins "less"
        assert_eq!(p.oracle_address_order(3, 7), Ordering::Less);
        assert_eq!(p.oracle_address_order(7, 3), Ordering::Greater);
        assert_eq!(p.oracle_address_order(4, 4), Ordering::Equal);
        // blocks land top-down under the pinned oracle: an EARLIER block is
        // at a HIGHER address, so it is never "less" than a later block
        assert_eq!(p.oracle_address_order(2, 12), Ordering::Greater);
        assert_eq!(p.oracle_address_order(12, 2), Ordering::Less);
        assert_eq!(p.oracle_address_order(13, 19), Ordering::Greater);
    }

    #[test]
    fn alloc_dealloc_stress_keeps_the_chain_invariants() {
        // deterministic stress; the debug dead-chain walker double-walks
        // on every operation (duplicates and loops would fire it)
        let mut p = pool(24, 64, 0, true);
        let mut live: Vec<u32> = vec![];
        for i in 0..500u32 {
            if i % 3 == 0 && !live.is_empty() {
                let victim = ((i as usize) * 7) % live.len();
                let id = live.swap_remove(victim);
                p.dealloc(id);
            } else {
                live.push(p.alloc().unwrap());
            }
            assert_eq!(p.live_count(), live.len() as u64);
        }
        assert!(p.high_water() > 0);
    }

    #[test]
    fn alloc_never_hands_out_the_null_or_sentinel_id() {
        // Invariants (ADR 0002 section 3 b, D3) pinned by construction
        // over fresh allocations and dead-reuse cycles alike.
        let mut p = pool(24, 8, 0, true);
        let mut live = vec![];
        for k in 0..20u32 {
            let id = p.alloc().unwrap();
            assert_ne!(id, NULL_ROW);
            assert_ne!(Some(id), p.sentinel_id());
            live.push(id);
            if k % 3 == 2 {
                let victim = live.swap_remove(0);
                p.dealloc(victim);
            }
        }
        // and the traversal never yields either reserved row
        let mut t = p.traversal();
        while let Some(id) = p.traverse_next(&mut t) {
            assert_ne!(id, NULL_ROW);
            assert_ne!(Some(id), p.sentinel_id());
        }
    }

    #[test]
    fn dead_chain_integrity_walk_over_push_pop_and_reuse() {
        // R1: the double-walk runs at every push/pop in debug builds; this
        // sequence interleaves pushes, pops and dead reuse so every chain
        // shape is walked: pushes build it, pops shorten it from the head,
        // reuse re-pushes from the middle.
        let mut p = pool(24, 16, 0, false);
        let a = p.alloc().unwrap();
        let b = p.alloc().unwrap();
        let c = p.alloc().unwrap();
        assert_eq!((a, b, c), (1, 2, 3));
        p.dealloc(a); // chain: a
        p.dealloc(b); // chain: b -> a
        assert_eq!(p.alloc().unwrap(), b, "LIFO pops the head"); // chain: a
        p.dealloc(c); // chain: c -> a
        assert_eq!(p.alloc().unwrap(), c); // chain: a
        assert_eq!(p.alloc().unwrap(), a); // chain empty
        assert_eq!(p.alloc().unwrap(), 4, "fresh again past the reuse");
        assert_eq!(p.live_count(), 4);
        assert_eq!(p.high_water(), 4);
    }

    #[test]
    #[should_panic(expected = "dead chain corrupt")]
    fn a_dead_chain_loop_fires_the_integrity_walk() {
        // Invariant (d): slot 0 of a dead row is pool-private; corrupting
        // it into a loop must fire the bounded walk at the next pop.
        let mut p = pool(24, 64, 0, false);
        let a = p.alloc().unwrap();
        let b = p.alloc().unwrap();
        p.dealloc(a);
        p.dealloc(b);
        p.set_word(b, 0, b as u64); // loop: b -> b
        let _ = p.alloc();
    }

    #[test]
    #[should_panic(expected = "dead chain corrupt")]
    fn a_truncated_dead_chain_fires_the_length_check() {
        // Invariant (d): a chain shorter than the recorded dead count is
        // corruption, not a legal state.
        let mut p = pool(24, 64, 0, false);
        let a = p.alloc().unwrap();
        let b = p.alloc().unwrap();
        p.dealloc(a);
        p.dealloc(b);
        p.set_word(b, 0, NULL_ROW as u64); // truncate: b -> NULL, recorded 2
        let _ = p.alloc();
    }

    #[test]
    #[should_panic(expected = "outside the pool id space")]
    fn a_word_access_beyond_the_frontier_is_a_range_violation() {
        // Invariant (a): ids the pool never materialized are outside the
        // id space.
        let mut p = pool(8, 8, 0, false);
        p.alloc().unwrap(); // rows 0 (null) and 1 materialized
        let _ = p.word(9, 0);
    }

    #[test]
    fn the_null_row_reads_as_the_never_written_zeros() {
        // Null discipline (ADR 0002 section 3 b) is decode-side: the
        // all-zero word is never decoded, and alloc and traversal never
        // surface the null id. The reserved null row itself stays inside
        // the materialized frontier and reads as its never-written zero
        // words - the port's defined substitute for the NULL-vertex
        // deref (algorithm/algorithm/delaunay.rs rightofhyperbola on an outer-space handle;
        // C's dummytri carries NULL vertices there, triangle.c:1049,
        // and the legal C flow never performs that read).
        let p = pool(8, 8, 0, false);
        assert!(p.handed_rows >= 1);
        for slot in 0..p.stride() {
            assert_eq!(p.word(NULL_ROW, slot as usize), 0);
        }
    }
}
