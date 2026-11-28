//! The standing queues of refinement behind one seam: the bad-triangle
//! lattice (BadTriangles) and the bad-subseg work queue (BadSubsegs).
//! The queue discipline that algorithm/quality.rs and algorithm/refine.rs
//! drove inline over the mesh fields is owned here, one Rust item per C
//! anchor: enqueuebadtriang / enqueuebadtri / dequeuebadtriang
//! (triangle.c:1714-1795, 1810-1823, 1835-1859), the queuefront sweep
//! and firstnonemptyq reset of enforcequality (triangle.c:7559-7561,
//! BadTriangles::reset), badsubsegdealloc / badsubsegtraverse
//! (triangle.c:1298-1304, 1316-1327) and the poolrestart bridges
//! (triangle.c:7533-7534, 7555-7556); the drain loops of
//! algorithm/refine.rs consume the seam's predicates. ADR 0005 section 6
//! swaps the concurrent claim queue in behind this seam (the MT swap-in
//! point of the standing queues); no MT scaffolding lives here.
//! Assignment: docs/function-mapping.md.
//!
//! Row storage stays behind the accessor chokepoint (GLOSSARY.md, ADR
//! 0002/0003): this module never reads or writes pool words directly;
//! every row value goes through the bad_tri_* / bad_subseg_* accessors
//! of mesh/slots.rs, which route through the pools inside the two
//! structs below.

use crate::mesh::handles::{encode, Otri, NULL_ROW};
use crate::mesh::pool::{Oom, Traversal, WordPool};
use crate::mesh::Mesh;
use crate::options::Behavior;

// The C per-block constants of the two queue pools (triangle_config.h:
// 120-131). Their values are part of the contract: block geometry feeds
// oracle_address_order (ADR 0003 section 5) and the initial capacity
// hints.
const BAD_SUBSEG_PER_BLOCK: i32 = 252;
const BAD_TRI_PER_BLOCK: i32 = 4092;

/// C SQUAREROOTTWO (triangle_config.h:163), the full-precision literal.
#[allow(clippy::approx_constant)] // C-parity: the C macro carries this
                                  // literal, verbatim, not std's SQRT_2 constant.
const SQUARE_ROOT_TWO: f64 = 1.4142135623730950488016887242096980785696718753769480732;

/// The standing bad-subseg work queue (the pool of C struct badsubseg
/// rows, triangle.h:271-274, behind its Mesh field `badsubsegs`): the
/// WordPool plus the queue-discipline surface. The bad_sub_seg_* row
/// accessors (mesh/slots.rs, the chokepoint) route through the pool-seam
/// delegates below; the drain loops of algorithm/refine.rs read the
/// predicates.
pub(crate) struct BadSubsegs {
    /// The badsubseg pool (C m->badsubsegs; the ADR 0003 section 4
    /// geometry: 24 itembytes, 252 first-block items, pool-declared
    /// first blocks, eager since the Mesh::new site).
    pool: WordPool,
}

impl BadSubsegs {
    /// C: the poolinit geometry of m->badsubsegs (triangle_config.h
    /// itembytes/blocks pair; eager at the Mesh::new site per the
    /// lazy-pool-init convention, ADR 0003 section 4).
    pub(crate) fn new() -> Result<Self, Oom> {
        Ok(BadSubsegs {
            pool: WordPool::new(
                24,
                BAD_SUBSEG_PER_BLOCK as u32,
                BAD_SUBSEG_PER_BLOCK as u32,
                false,
            )?,
        })
    }

    // The pool-seam delegates: the only route to the row storage, used
    // by the bad_sub_seg_* accessors of mesh/slots.rs (the chokepoint;
    // pub(super), so no code outside mesh/ can touch a word).

    /// C: poolalloc(&m->badsubsegs) at insertvertex's encroachment
    /// enqueue (triangle.c:2975), through the seam
    /// (Mesh::bad_sub_seg_alloc).
    pub(super) fn alloc(&mut self) -> Result<u32, Oom> {
        self.pool.alloc()
    }

    /// C: the badsubseg row word writes (triangle.c:2976-2978), through
    /// the seam (Mesh::set_bad_sub_seg_*).
    pub(super) fn set_word(&mut self, id: u32, slot: usize, w: u64) {
        self.pool.set_word(id, slot, w);
    }

    /// C: the badsubseg row word reads (triangle.c:7208, 7216), through
    /// the seam (Mesh::bad_sub_seg_enc / _org / _dest).
    pub(super) fn word(&self, id: u32, slot: usize) -> u64 {
        self.pool.word(id, slot)
    }

    // The seam interface.

    /// C: pooldealloc(&m->badsubsegs) inside badsubsegdealloc
    /// (triangle.c:1303).
    fn dealloc(&mut self, id: u32) {
        self.pool.dealloc(id);
    }

    /// C: the traversalinit start of badsubsegtraverse (triangle.c:1316;
    /// splitencsegs' traversalinit, triangle.c:7367).
    pub(crate) fn traversal(&self) -> Traversal {
        self.pool.traversal()
    }

    /// C: the pool walk of badsubsegtraverse (triangle.c:1321); the
    /// dead-row skip lives in the Mesh method below.
    fn traverse_next(&self, t: &mut Traversal) -> Option<u32> {
        self.pool.traverse_next(t)
    }

    /// The drain-loop predicate: the count of live badsubseg rows
    /// (WordPool::live_count through the seam).
    pub(crate) fn live_count(&self) -> u64 {
        self.pool.live_count()
    }

    /// C: poolrestart(&m->badsubsegs) (triangle.c:7533-7534,
    /// enforcequality's badsubseg pool init).
    /// Invariants (ADR 0002 section 3 d): the dead chain is dropped; the
    /// debug integrity walk runs on the emptied chain.
    pub(crate) fn restart(&mut self) {
        self.pool.restart();
    }

    /// The pool geometry reads of the mesh tests (WordPool::stride /
    /// WordPool::block_items through the seam).
    pub(super) fn stride(&self) -> u32 {
        self.pool.stride()
    }

    /// As `stride` (WordPool::block_items through the seam).
    pub(super) fn block_items(&self) -> u32 {
        self.pool.block_items()
    }

    /// triangle_memory's per-pool sum (triangle_api.c:143): C's
    /// maxitems * itembytes (WordPool high_water is the C maxitems
    /// mirror and stride * 8 is itembytes, ADR 0003).
    pub(crate) fn pool_bytes(&self) -> usize {
        self.pool.high_water() as usize * self.pool.stride() as usize * 8
    }
}

/// The standing bad-triangle work queue (the pool of C struct badtriang
/// rows, triangle.h:277-286, behind its Mesh field `badtriangles`) plus
/// the bucket lattice of C triangle.h:502-505: 4096 queues keyed by the
/// square of the shortest edge, front/tail links into the pool, the
/// nextnonemptyq chain and its head. D4: the C badtriang pointers mirror
/// as u32 pool row ids, the null row id 0 marking queue ends;
/// firstnonemptyq -1 marks "all empty", 4096 the sentinel value dequeue
/// treats as empty. The bad_tri_* row accessors (mesh/slots.rs, the
/// chokepoint) route through the pool-seam delegates; the enqueue /
/// dequeue discipline runs in the impl Mesh block below over the
/// private lattice state.
pub(crate) struct BadTriangles {
    /// C m->queuefront[4096]: the first badtriang row of each queue (the
    /// null row id 0 for an empty queue).
    queuefront: [u32; 4096],
    /// C m->queuetail[4096]: the last badtriang row of each queue.
    /// C leaves this uninitialized until an enqueue writes it, and the
    /// queue discipline never reads it before that first write; the port
    /// pins the null row id 0 as the deterministic superset.
    queuetail: [u32; 4096],
    /// C m->nextnonemptyq[4096]: the next lower-priority nonempty queue
    /// after each nonempty queue. Same unread-before-write discipline as
    /// `queuetail`; the port pins -1 as the deterministic superset.
    nextnonemptyq: [i32; 4096],
    /// C m->firstnonemptyq: the highest-priority nonempty queue, -1 when
    /// all queues are empty.
    firstnonemptyq: i32,
    /// The badtriangles pool (C m->badtriangles; the ADR 0003 section 4
    /// geometry: 48 itembytes, 4092 first-block items, pool-declared
    /// first blocks, eager since the Mesh::new site).
    pool: WordPool,
}

impl BadTriangles {
    /// C: the construction of the badtriang pool plus the queue state
    /// (C triangle.h:502-505). C initializes queuefront[] to NULL and
    /// firstnonemptyq to -1 inside enforcequality (triangle.c:7559-7561)
    /// right before tallyfaces fills the queues; queuetail[] and
    /// nextnonemptyq[] are uninitialized struct memory there, never read
    /// before their first enqueue writes them (the queue discipline of
    /// the impl Mesh block below). The port pins the whole state at
    /// construction: the enforcequality-init values for the two
    /// C-initialized members and the null superset for the other two,
    /// following the established lazy-pool-init convention (ADR 0003
    /// section 4). algorithm/refine.rs's enforcequality re-runs the C
    /// init loop through [`BadTriangles::reset`] at exactly C's point.
    pub(crate) fn new() -> Result<Self, Oom> {
        Ok(BadTriangles {
            queuefront: [NULL_ROW; 4096],
            queuetail: [NULL_ROW; 4096],
            nextnonemptyq: [-1; 4096],
            firstnonemptyq: -1,
            pool: WordPool::new(
                48,
                BAD_TRI_PER_BLOCK as u32,
                BAD_TRI_PER_BLOCK as u32,
                false,
            )?,
        })
    }

    /// C: the queuefront sweep and firstnonemptyq reset of enforcequality
    /// (triangle.c:7559-7561): every queue empty, no highest-priority
    /// queue. Called from algorithm/refine.rs at exactly C's point
    /// (right before tallyfaces fills the queues); queuetail[] and
    /// nextnonemptyq[] stay untouched exactly like C (never read before
    /// their next enqueue writes them).
    pub(crate) fn reset(&mut self) {
        for slot in self.queuefront.iter_mut() {
            *slot = NULL_ROW;
        }
        self.firstnonemptyq = -1;
    }

    // The pool-seam delegates: the only route to the row storage, used
    // by the bad_tri_* accessors of mesh/slots.rs (the chokepoint;
    // pub(super), so no code outside mesh/ can touch a word).

    /// C: poolalloc(&m->badtriangles) at enqueuebadtri (triangle.c:1820),
    /// through the seam (Mesh::bad_tri_alloc).
    pub(super) fn alloc(&mut self) -> Result<u32, Oom> {
        self.pool.alloc()
    }

    /// C: the badtriang row word writes (triangle.c:1791, 1814,
    /// 1821-1825), through the seam (Mesh::set_bad_tri_*).
    pub(super) fn set_word(&mut self, id: u32, slot: usize, w: u64) {
        self.pool.set_word(id, slot, w);
    }

    /// C: the badtriang row word reads (triangle.c:1721-1751, 1846ff),
    /// through the seam (Mesh::bad_tri_key and the other readers).
    pub(super) fn word(&self, id: u32, slot: usize) -> u64 {
        self.pool.word(id, slot)
    }

    /// C: pooldealloc(&m->badtriangles) in enforcequality's else arm
    /// (triangle.c:7586; C defines no badtridealloc wrapper for this
    /// site - the queue empties by dequeue, so no dead-row mark),
    /// through the seam (Mesh::bad_tri_dealloc).
    ///
    /// Invariants (ADR 0002 section 3): the push makes slot 0
    /// pool-private (d); reuse stays exact-LIFO (e); the integrity walk
    /// runs at the push (debug only).
    pub(super) fn dealloc(&mut self, id: u32) {
        self.pool.dealloc(id);
    }

    // The seam interface.

    /// The drain-loop predicate: the count of live badtriang rows
    /// (WordPool::live_count through the seam).
    pub(crate) fn live_count(&self) -> u64 {
        self.pool.live_count()
    }

    /// C: poolrestart(&m->badtriangles) (triangle.c:7555-7556,
    /// enforcequality's badtriang pool init).
    /// Invariants (ADR 0002 section 3 d): as BadSubsegs::restart.
    pub(crate) fn restart(&mut self) {
        self.pool.restart();
    }

    /// The pool geometry reads of the mesh tests (WordPool::stride /
    /// WordPool::block_items through the seam).
    pub(super) fn stride(&self) -> u32 {
        self.pool.stride()
    }

    /// As `stride` (WordPool::block_items through the seam).
    pub(super) fn block_items(&self) -> u32 {
        self.pool.block_items()
    }

    /// triangle_memory's per-pool sum (triangle_api.c:143): C's
    /// maxitems * itembytes (WordPool high_water is the C maxitems
    /// mirror and stride * 8 is itembytes, ADR 0003).
    pub(crate) fn pool_bytes(&self) -> usize {
        self.pool.high_water() as usize * self.pool.stride() as usize * 8
    }
}

impl Mesh {
    /// C: badsubsegdealloc (triangle.c:1298-1304). The dead-row idiom:
    /// subsegorg = NULL first (triangle.c:1302) so badsubsegtraverse
    /// skips the row, then pooldealloc (triangle.c:1303); splitencsegs
    /// (algorithm/algorithm/refine.rs) is the consumer.
    ///
    /// Invariants (ADR 0002 section 3): the NULL word is the dead-row
    /// mark, never decoded (b); the push makes slot 0 pool-private (d);
    /// reuse stays exact-LIFO (e).
    pub(crate) fn dealloc_bad_subseg(&mut self, dyingsubseg: u32) {
        self.set_bad_sub_seg_org(dyingsubseg, NULL_ROW);
        self.badsubsegs.dealloc(dyingsubseg);
    }

    /// C: badsubsegtraverse (triangle.c:1316-1327). Walk the pool and
    /// skip the rows badsubsegdealloc marked (the subsegorg == NULL
    /// test of the do-while, triangle.c:1321-1326); splitencsegs'
    /// traversalinit (triangle.c:7367) starts it.
    ///
    /// Invariants (ADR 0002 section 3): the NULL mark is the skip test
    /// (b); the null row itself is never yielded (b, debug-pinned at
    /// the pool seam).
    pub(crate) fn traverse_bad_subseg(&self, t: &mut Traversal) -> Option<u32> {
        loop {
            let newsub = self.badsubsegs.traverse_next(t)?;
            if self.bad_subseg_org(newsub) != NULL_ROW {
                return Some(newsub);
            }
        }
    }

    /// C: enqueuebadtriang (triangle.c:1714-1795). Add a bad triangle data
    /// structure to the end of a queue. The queue is actually a set of 4096
    /// queues, keyed by the square of the shortest edge length, giving
    /// priority to smaller edges (queue 4095 is the highest priority). The
    /// key-to-queue arithmetic (length, posexponent, the repeated-squaring
    /// exponent loop and the SQUAREROOTTWO ladder) is byte-parity-relevant
    /// for the refinement drivers: identical arithmetic, identical queue
    /// number, identical linkage order.
    ///
    /// `b` is unused in the C body too (kept for the exact C shape).
    pub(crate) fn enqueue_bad_triangle_row(&mut self, _b: &Behavior, badtri: u32) {
        // Determine the appropriate queue to put the bad triangle into.
        //   Recall that the key is the square of its shortest edge length.
        let key = self.bad_tri_key(badtri);
        let (mut length, posexponent) = if key == 0.0 {
            (0.0, 0)
        } else if key >= 1.0 {
            (key, 1)
        } else {
            // `badtri->key' is 2.0 to a negative exponent, so we'll record
            //   that fact and use the reciprocal of `badtri->key',
            //   which is > 1.0.
            (1.0 / key, 0)
        };
        // `length' is approximately 2.0 to what exponent?  The following code
        //   determines the answer in time logarithmic in the exponent.
        let mut exponent: i32 = 0;
        while length > 2.0 {
            // Find an approximation by repeated squaring of two.
            let mut expincrement: i32 = 1;
            let mut multiplier: f64 = 0.5;
            while length * multiplier * multiplier > 1.0 {
                expincrement *= 2;
                multiplier *= multiplier;
            }
            // Reduce the value of `length', then iterate if necessary.
            exponent += expincrement;
            length *= multiplier;
        }
        if length == 0.0 {
            exponent = 1023;
        }
        // `length' is approximately squareroot(2.0) to what exponent?
        exponent = 2 * exponent + (length > SQUARE_ROOT_TWO) as i32;
        // `exponent' is now in the range 0...2047 for IEEE double precision.
        //   Choose a queue in the range 0...4095.  The shortest edges have the
        //   highest priority (queue 4095).
        let queuenumber: i32 = if posexponent != 0 {
            2047 - exponent
        } else {
            2048 + exponent
        };

        // Are we inserting into an empty queue?
        if self.badtriangles.queuefront[queuenumber as usize] == NULL_ROW {
            // Yes, we are inserting into an empty queue.
            //   Will this become the highest-priority queue?
            if queuenumber > self.badtriangles.firstnonemptyq {
                // Yes, this is the highest-priority queue.
                self.badtriangles.nextnonemptyq[queuenumber as usize] =
                    self.badtriangles.firstnonemptyq;
                self.badtriangles.firstnonemptyq = queuenumber;
            } else {
                // No, this is not the highest-priority queue.
                //   Find the queue with next higher priority.
                let mut i: i32 = queuenumber + 1;
                while self.badtriangles.queuefront[i as usize] == NULL_ROW {
                    i += 1;
                }
                // Mark the newly nonempty queue as following a
                //   higher-priority queue.
                self.badtriangles.nextnonemptyq[queuenumber as usize] =
                    self.badtriangles.nextnonemptyq[i as usize];
                self.badtriangles.nextnonemptyq[i as usize] = queuenumber;
            }
            // Put the bad triangle at the beginning of the (empty) queue.
            self.badtriangles.queuefront[queuenumber as usize] = badtri;
        } else {
            // Add the bad triangle to the end of an already nonempty queue.
            let tail = self.badtriangles.queuetail[queuenumber as usize];
            self.set_bad_tri_next(tail, badtri);
        }
        // Maintain a pointer to the last triangle of the queue.
        self.badtriangles.queuetail[queuenumber as usize] = badtri;
        // Newly enqueued bad triangle has no successor in the queue.
        self.set_bad_tri_next(badtri, NULL_ROW);
    }

    /// C: enqueuebadtri (triangle.c:1810-1823). Add a bad triangle to the end
    /// of a queue: allocate a badtriang row for the triangle, fill it
    /// (poortri, key, apex, org, dest, exactly C's write order), then pass it
    /// to enqueuebadtriang. OOM routes through the pool bridge.
    pub(crate) fn enqueue_bad_triangle(
        &mut self,
        b: &Behavior,
        enqtri: Otri,
        minedge: f64,
        enqapex: u32,
        enqorg: u32,
        enqdest: u32,
    ) -> Result<(), Oom> {
        // Allocate space for the bad triangle.
        let newbad = self.bad_tri_alloc()?;
        self.set_bad_tri_poor_tri(newbad, encode(enqtri));
        self.set_bad_tri_key(newbad, minedge);
        self.set_bad_tri_apex(newbad, enqapex);
        self.set_bad_tri_org(newbad, enqorg);
        self.set_bad_tri_dest(newbad, enqdest);
        self.enqueue_bad_triangle_row(b, newbad);
        Ok(())
    }

    /// C: dequeuebadtriang (triangle.c:1835-1859). Remove a triangle from the
    /// front of the highest-priority queue. The C NULL returns (no queues
    /// nonempty, the out-of-range guards, an empty front) mirror as None;
    /// the row id of the dequeued badtriang row otherwise.
    pub(crate) fn dequeue_bad_triangle(&mut self) -> Option<u32> {
        // If no queues are nonempty, return None.
        if self.badtriangles.firstnonemptyq < 0 {
            return None;
        }
        if self.badtriangles.firstnonemptyq >= 4096 {
            return None;
        }
        // Find the first triangle of the highest-priority queue.
        let result = self.badtriangles.queuefront[self.badtriangles.firstnonemptyq as usize];
        if result == NULL_ROW {
            return None;
        }
        // Remove the triangle from the queue.
        self.badtriangles.queuefront[self.badtriangles.firstnonemptyq as usize] =
            self.bad_tri_next(result);
        // If this queue is now empty, note the new highest-priority
        //   nonempty queue.
        if result == self.badtriangles.queuetail[self.badtriangles.firstnonemptyq as usize] {
            self.badtriangles.firstnonemptyq =
                self.badtriangles.nextnonemptyq[self.badtriangles.firstnonemptyq as usize];
        }
        Some(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// mesh_dim 2 like every stage-1 test; no extras.
    fn mesh(b: &Behavior) -> Mesh {
        Mesh::new(b, 16, 2, 0, 0).expect("mesh init")
    }

    fn vxy(m: &mut Mesh, x: f64, y: f64) -> u32 {
        let v = m.vx_alloc().expect("vertex alloc");
        m.set_vx_x(v, x);
        m.set_vx_y(v, y);
        v
    }

    /// A triangle (a, b, c) CCW at orient 0, unbonded (hull edges).
    fn lone_tri(
        m: &mut Mesh,
        usesegments: bool,
        a: (f64, f64),
        b: (f64, f64),
        c: (f64, f64),
    ) -> Otri {
        let va = vxy(m, a.0, a.1);
        let vb = vxy(m, b.0, b.1);
        let vc = vxy(m, c.0, c.1);
        let t = m.make_triangle(usesegments, false).expect("tri");
        m.set_org(t, va);
        m.set_dest(t, vb);
        m.set_apex(t, vc);
        t
    }

    // ------------------------------------------------------------------
    // enqueuebadtriang / enqueuebadtri / dequeuebadtriang
    // ------------------------------------------------------------------

    /// The C key-to-queue arithmetic, hand-derived per C's ladder:
    /// key 0.0 -> exponent 1023 -> queue 2048 + 2046 = 4094; key 1.0 ->
    /// queue 2047; key 4.0 -> exponent 2*1+1 (length 4 -> 2 via one
    /// halving, 2.0 > SQUAREROOTTWO) -> queue 2047 - 3 = 2044; key 0.25
    /// -> length 4.0 with posexponent 0 -> queue 2048 + 3 = 2051;
    /// key 2.0 -> exponent 2*0+1 = 1 -> queue 2047 - 1 = 2046.
    #[test]
    fn queue_number_arithmetic_matches_the_c_ladder() {
        let b = Behavior::default_form();
        let mut m = mesh(&b);
        let t = lone_tri(&mut m, false, (0.0, 0.0), (1.0, 0.0), (0.0, 1.0));
        for (i, (key, expected_qn)) in [
            (0.0, 4094usize),
            (1.0, 2047),
            (4.0, 2044),
            (0.25, 2051),
            (2.0, 2046),
        ]
        .iter()
        .enumerate()
        {
            m.enqueue_bad_triangle(&b, t, *key, 0, 0, 0)
                .expect("enqueue");
            // the freshly allocated row (ids 1..=5; the pool has no
            // sentinel) is the new front of exactly the expected queue.
            assert_eq!(m.badtriangles.queuefront[*expected_qn], i as u32 + 1);
        }
        // firstnonemptyq is the highest-priority (largest) queue number
        // used: the first row (key 0.0) sits at 4094.
        assert_eq!(m.badtriangles.firstnonemptyq, 4094);
        // The 4094 queue holds exactly the key-0.0 row; the 2047 queue
        // holds the key-1.0 row; ... the linkage order (front == then
        // tail ==) pins which row is where.
        let row0 = m.badtriangles.queuefront[4094];
        assert_eq!(m.bad_tri_key(row0), 0.0);
        assert_eq!(
            m.badtriangles.queuefront[2047],
            m.badtriangles.queuefront[4094] + 1
        );
        assert_eq!(m.bad_tri_key(m.badtriangles.queuefront[2047]), 1.0);
        assert_eq!(m.bad_tri_key(m.badtriangles.queuefront[2044]), 4.0);
        assert_eq!(m.bad_tri_key(m.badtriangles.queuefront[2051]), 0.25);
        assert_eq!(m.bad_tri_key(m.badtriangles.queuefront[2046]), 2.0);
        // every queue's tail points at itself (single-element queues)
        for qn in [4094usize, 2047, 2044, 2051, 2046] {
            assert_eq!(m.badtriangles.queuetail[qn], m.badtriangles.queuefront[qn]);
        }
        // pool row accounting: five rows live, ids 1..=5 (no sentinel)
        assert_eq!(m.badtriangles.live_count(), 5);
    }

    /// FIFO within one queue: two rows with the same key dequeue in
    /// insertion order; priority across queues: smaller keys first.
    #[test]
    fn dequeue_is_fifo_within_a_queue_and_priority_across_queues() {
        let b = Behavior::default_form();
        let mut m = mesh(&b);
        let t = lone_tri(&mut m, false, (0.0, 0.0), (1.0, 0.0), (0.0, 1.0));
        m.enqueue_bad_triangle(&b, t, 4.0, 3, 1, 2)
            .expect("enqueue"); // r1
        m.enqueue_bad_triangle(&b, t, 4.0, 6, 4, 5)
            .expect("enqueue"); // r2, same queue
        m.enqueue_bad_triangle(&b, t, 1.0, 9, 7, 8)
            .expect("enqueue"); // r3, higher priority
        assert_eq!(m.badtriangles.live_count(), 3);
        // dequeue: r3 first (queue 2047 before 2044), then r2 was
        // appended after r1 in queue 2044: r1, then r2.
        let d1 = m.dequeue_bad_triangle().expect("row");
        assert_eq!(m.bad_tri_key(d1), 1.0);
        assert_eq!(m.bad_tri_apex(d1), 9);
        assert_eq!(m.bad_tri_org(d1), 7);
        assert_eq!(m.bad_tri_dest(d1), 8);
        assert_eq!(m.bad_tri_poor_tri(d1), encode(t));
        let d2 = m.dequeue_bad_triangle().expect("row");
        assert_eq!(m.bad_tri_key(d2), 4.0);
        assert_eq!(m.bad_tri_org(d2), 1);
        let d3 = m.dequeue_bad_triangle().expect("row");
        assert_eq!(m.bad_tri_key(d3), 4.0);
        assert_eq!(m.bad_tri_org(d3), 4);
        // The queue drained: firstnonemptyq falls back through the
        // nextnonemptyq chain to -1.
        assert_eq!(m.badtriangles.firstnonemptyq, -1);
        assert!(m.dequeue_bad_triangle().is_none());
        // dequeued rows stay allocated (deallocation belongs to the
        // algorithm/algorithm/refine.rs drainers).
        assert_eq!(m.badtriangles.live_count(), 3);
        assert_eq!(m.bad_tri_next(d2), d3, "the FIFO link survives the pop");
    }

    /// An empty-queue insert below the current head chains nextnonemptyq
    /// exactly like C: (first -> ...) -> new -> ... .
    #[test]
    fn empty_queue_insert_links_the_nextnonemptyq_chain() {
        let b = Behavior::default_form();
        let mut m = mesh(&b);
        let t = lone_tri(&mut m, false, (0.0, 0.0), (1.0, 0.0), (0.0, 1.0));
        // queue 2047 (key 1.0), then the lower-priority queue 2044
        // (key 4.0): the second insert walks i upward to 2047.
        m.enqueue_bad_triangle(&b, t, 1.0, 0, 0, 0)
            .expect("enqueue");
        let r_high = m.badtriangles.queuefront[2047];
        m.enqueue_bad_triangle(&b, t, 4.0, 0, 0, 0)
            .expect("enqueue");
        let r_low = m.badtriangles.queuefront[2044];
        assert_eq!(m.badtriangles.firstnonemptyq, 2047);
        assert_eq!(
            m.badtriangles.nextnonemptyq[2047], 2044,
            "2047 points at 2044"
        );
        assert_eq!(
            m.badtriangles.nextnonemptyq[2044], -1,
            "2044 ends the chain"
        );
        // and one more between them: queue 2046 (key 2.0).
        m.enqueue_bad_triangle(&b, t, 2.0, 0, 0, 0)
            .expect("enqueue");
        assert_eq!(m.badtriangles.nextnonemptyq[2047], 2046);
        assert_eq!(m.badtriangles.nextnonemptyq[2046], 2044);
        // the drain order follows the chain: 2047, 2046, 2044.
        assert_eq!(m.dequeue_bad_triangle(), Some(r_high));
        assert_eq!(m.badtriangles.firstnonemptyq, 2046);
        let r_mid = m.badtriangles.queuefront[2046];
        assert_eq!(m.dequeue_bad_triangle(), Some(r_mid));
        assert_eq!(m.badtriangles.firstnonemptyq, 2044);
        assert_eq!(m.dequeue_bad_triangle(), Some(r_low));
        assert_eq!(m.badtriangles.firstnonemptyq, -1);
    }

    /// The reset method's end state (triangle.c:7559-7561): every
    /// queuefront slot is the null row id and firstnonemptyq is -1, so
    /// dequeue reports the empty lattice; the pool's live rows are
    /// untouched (the sweep is the lattice init, not a pool restart).
    #[test]
    fn reset_empties_the_lattice_and_leaves_the_pool_alone() {
        let b = Behavior::default_form();
        let mut m = mesh(&b);
        let t = lone_tri(&mut m, false, (0.0, 0.0), (1.0, 0.0), (0.0, 1.0));
        m.enqueue_bad_triangle(&b, t, 1.0, 0, 0, 0)
            .expect("enqueue");
        m.enqueue_bad_triangle(&b, t, 4.0, 0, 0, 0)
            .expect("enqueue");
        assert_eq!(m.badtriangles.live_count(), 2);
        assert_eq!(m.badtriangles.firstnonemptyq, 2047);
        m.badtriangles.reset();
        for front in &m.badtriangles.queuefront {
            assert_eq!(*front, NULL_ROW);
        }
        assert_eq!(m.badtriangles.firstnonemptyq, -1);
        assert!(m.dequeue_bad_triangle().is_none());
        assert_eq!(m.badtriangles.live_count(), 2, "the pool rows stay live");
    }
}
