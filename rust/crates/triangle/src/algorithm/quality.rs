//! Mesh quality testing: checkmesh, checkdelaunay, the bad-triangle queue
//! (enqueuebadtriang, enqueuebadtri, dequeuebadtriang), inlens,
//! checkseg4encroach, testtriangle (with the Unsuitable callback call site),
//! quality_statistics.
//! C: triangle.c section Mesh quality testing routines (+ quality_statistics
//! from the Array I/O section tail, moved for cohesion). Assignment:
//! docs/function-mapping.md.
//!
//! Stage-1 landing state: complete. The rows landed in two steps:
//! makevertexmap with its algorithm/segments.rs consumer (PR #35), the rest,
//! together with the #28 restoration of testtriangle/checkseg4encroach/
//! inlens at the algorithm/transform.rs call sites, in the quality PR. The queue
//! state (queuefront/queuetail/nextnonemptyq/firstnonemptyq) lives on
//! mesh/mod.rs since this PR; the badtriang row accessors live in mesh/slots.rs.
//! dequeuebadtriang's consumer is enforcequality, which landed with
//! algorithm/refine.rs; the ahead-of-use dead_code blanket is removed with that
//! PR (every item of this module now has its production consumer).

use std::f64::consts::PI;

use crate::api::Quality;
use crate::mesh::handles::{
    encode, sencode, Osub, Otri, DUMMY_SUB, DUMMY_TRI, MINUS1MOD3, NULL, PLUS1MOD3,
};

use crate::mesh::pool::Oom;
use crate::mesh::slots::SEGMENTVERTEX;
use crate::mesh::util::vertex_coords;
use crate::mesh::Mesh;
use crate::options::{Behavior, VertexSample};
use crate::predicates::{tri_counterclockwise, tri_nonregular, Vertex};

/// C SQUAREROOTTWO (triangle_config.h:163), the full-precision literal.
#[allow(clippy::approx_constant)] // C-parity: the C macro carries this
                                  // literal, verbatim, not std's SQRT_2 constant.
const SQUAREROOTTWO: f64 = 1.4142135623730950488016887242096980785696718753769480732;

/// C: makevertexmap (triangle.c:2206-2222). Compute a mapping from each
/// mesh vertex to one adjacent triangle (vertex2tri): walk the triangles
/// in allocation order and, for each of the three orientations in turn,
/// store encode(triangleloop) on the origin vertex's vertex2tri slot.
/// Later vertex2tri writes overwrite earlier ones, but in the end every
/// vertex points to some triangle that contains it.
pub(crate) fn makevertexmap(m: &mut Mesh, _b: &Behavior) {
    let mut t = m.triangles.traversal();
    // C: triangleloop.tri = triangletraverse(m) ... while != NULL.
    while let Some(tri) = m.triangletraverse(&mut t) {
        // Check all three vertices of the triangle.
        for orient in 0..3u8 {
            let triangleloop = Otri { tri, orient };
            // C: org(triangleloop, triorg); setvertex2tri(triorg,
            // encode(triangleloop)).
            let triorg = m.org(triangleloop);
            m.vx_set_2tri(triorg, encode(triangleloop));
        }
    }
}

/// C: checkmesh (triangle.c:1537-1614). Run through the list of
/// triangles, checking each one. Each triangle's three edges are checked;
/// the triangle is tested for inversion once (at orient 0, through
/// counterclockwise), the neighbor across each edge is checked for
/// knowing the triangle as its neighbor, and both triangles are checked
/// to agree on the identities of their shared vertices. Returns the
/// number of horrors; 0 if the mesh is consistent.
///
/// The C body temporarily turns on exact arithmetic (save/restore
/// b->noexact), mirrored 1:1 through `b: &mut Behavior`.
pub(crate) fn checkmesh(m: &mut Mesh, b: &mut Behavior) -> i32 {
    // Temporarily turn on exact arithmetic if it's off.
    let saveexact = b.noexact;
    b.noexact = false;
    let mut horrors = 0;
    // Run through the list of triangles, checking each one.
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.triangletraverse(&mut t) {
        // Check all three edges of the triangle.
        for orient in 0..3u8 {
            let triangleloop = Otri { tri, orient };
            let triorg = m.org(triangleloop);
            let tridest = m.dest(triangleloop);
            if orient == 0 {
                // Only test for inversion once.
                // Test if the triangle is flat or inverted.
                let triapex = m.apex(triangleloop);
                if tri_counterclockwise(
                    m,
                    b,
                    vertex_coords(m, triorg),
                    vertex_coords(m, tridest),
                    vertex_coords(m, triapex),
                ) <= 0.0
                {
                    horrors += 1;
                }
            }
            // Find the neighboring triangle on this edge.
            let oppotri = m.sym(triangleloop);
            if oppotri.tri != DUMMY_TRI {
                // Check that the triangle's neighbor knows it's a neighbor.
                let oppooppotri = m.sym(oppotri);
                if triangleloop.tri != oppooppotri.tri || triangleloop.orient != oppooppotri.orient
                {
                    horrors += 1;
                }
                // Check that both triangles agree on the identities
                //   of their shared vertices.
                let oppoorg = m.org(oppotri);
                let oppodest = m.dest(oppotri);
                if triorg != oppodest || tridest != oppoorg {
                    horrors += 1;
                }
            }
        }
    }

    // Restore the status of exact arithmetic.
    b.noexact = saveexact;

    horrors
}

/// C: checkdelaunay (triangle.c:1622-1699). Ensure that the mesh is
/// (constrained) Delaunay: run through the triangles, and where an
/// adjoining triangle exists, test that the shared edge is locally
/// Delaunay through nonregular (skipping subsegment-separated pairs
/// when checksegments is set, and any pair touching one of the three
/// infinite vertices). Returns the number of horrors.
///
/// The C pair-once guard is a raw pointer comparison
/// (`triangleloop.tri < oppotri.tri`, triangle.c:1655). Under D1 the
/// mirror compares triangle pool row ids: ids are handed out
/// monotonically, so each unordered pair satisfies the comparison in
/// exactly one direction, preserving the guard's only purpose (testing
/// each pair once) with a deterministic order; the horror count is
/// unaffected either way, because the nonregular sign is the same from
/// both sides of a shared edge.
pub(crate) fn checkdelaunay(m: &mut Mesh, b: &mut Behavior) -> i32 {
    // Temporarily turn on exact arithmetic if it's off.
    let saveexact = b.noexact;
    b.noexact = false;
    let mut horrors = 0;
    // Run through the list of triangles, checking each one.
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.triangletraverse(&mut t) {
        // Check all three edges of the triangle.
        for orient in 0..3u8 {
            let triangleloop = Otri { tri, orient };
            let triorg = m.org(triangleloop);
            let tridest = m.dest(triangleloop);
            let triapex = m.apex(triangleloop);
            let oppotri = m.sym(triangleloop);
            let oppoapex = m.apex(oppotri);
            // Only test that the edge is locally Delaunay if there is an
            //   adjoining triangle whose pointer is larger (to ensure that
            //   each pair isn't tested twice).
            let infv = [m.infvertex1, m.infvertex2, m.infvertex3];
            let mut shouldbedelaunay = oppotri.tri != DUMMY_TRI
                && !m.deadtri(oppotri.tri)
                && triangleloop.tri < oppotri.tri
                && !infv.contains(&triorg)
                && !infv.contains(&tridest)
                && !infv.contains(&triapex)
                && !infv.contains(&oppoapex);
            if m.checksegments && shouldbedelaunay {
                // If a subsegment separates the triangles, then the edge is
                //   constrained, so no local Delaunay test should be done.
                let opposubseg = m.tspivot(triangleloop);
                if opposubseg.ss != DUMMY_SUB {
                    shouldbedelaunay = false;
                }
            }
            // C-parity: triangle.c:1672-1673 nests the two guards exactly
            // like this.
            #[allow(clippy::collapsible_if)]
            if shouldbedelaunay {
                if tri_nonregular(
                    m,
                    b,
                    vertex_coords(m, triorg),
                    vertex_coords(m, tridest),
                    vertex_coords(m, triapex),
                    vertex_coords(m, oppoapex),
                ) > 0.0
                {
                    horrors += 1;
                }
            }
        }
    }

    // Restore the status of exact arithmetic.
    b.noexact = saveexact;

    horrors
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
pub(crate) fn enqueuebadtriang(m: &mut Mesh, _b: &Behavior, badtri: u32) {
    // Determine the appropriate queue to put the bad triangle into.
    //   Recall that the key is the square of its shortest edge length.
    let key = m.bad_tri_key(badtri);
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
    exponent = 2 * exponent + (length > SQUAREROOTTWO) as i32;
    // `exponent' is now in the range 0...2047 for IEEE double precision.
    //   Choose a queue in the range 0...4095.  The shortest edges have the
    //   highest priority (queue 4095).
    let queuenumber: i32 = if posexponent != 0 {
        2047 - exponent
    } else {
        2048 + exponent
    };

    // Are we inserting into an empty queue?
    if m.queuefront[queuenumber as usize] == NULL {
        // Yes, we are inserting into an empty queue.
        //   Will this become the highest-priority queue?
        if queuenumber > m.firstnonemptyq {
            // Yes, this is the highest-priority queue.
            m.nextnonemptyq[queuenumber as usize] = m.firstnonemptyq;
            m.firstnonemptyq = queuenumber;
        } else {
            // No, this is not the highest-priority queue.
            //   Find the queue with next higher priority.
            let mut i: i32 = queuenumber + 1;
            while m.queuefront[i as usize] == NULL {
                i += 1;
            }
            // Mark the newly nonempty queue as following a
            //   higher-priority queue.
            m.nextnonemptyq[queuenumber as usize] = m.nextnonemptyq[i as usize];
            m.nextnonemptyq[i as usize] = queuenumber;
        }
        // Put the bad triangle at the beginning of the (empty) queue.
        m.queuefront[queuenumber as usize] = badtri;
    } else {
        // Add the bad triangle to the end of an already nonempty queue.
        let tail = m.queuetail[queuenumber as usize];
        m.set_bad_tri_next(tail, badtri);
    }
    // Maintain a pointer to the last triangle of the queue.
    m.queuetail[queuenumber as usize] = badtri;
    // Newly enqueued bad triangle has no successor in the queue.
    m.set_bad_tri_next(badtri, NULL);
}

/// C: enqueuebadtri (triangle.c:1810-1823). Add a bad triangle to the end
/// of a queue: allocate a badtriang row for the triangle, fill it
/// (poortri, key, apex, org, dest, exactly C's write order), then pass it
/// to enqueuebadtriang. OOM routes through the pool bridge.
pub(crate) fn enqueuebadtri(
    m: &mut Mesh,
    b: &Behavior,
    enqtri: Otri,
    minedge: f64,
    enqapex: u32,
    enqorg: u32,
    enqdest: u32,
) -> Result<(), Oom> {
    // Allocate space for the bad triangle.
    let newbad = m.bad_tri_alloc()?;
    m.set_bad_tri_poortri(newbad, encode(enqtri));
    m.set_bad_tri_key(newbad, minedge);
    m.set_bad_tri_apex(newbad, enqapex);
    m.set_bad_tri_org(newbad, enqorg);
    m.set_bad_tri_dest(newbad, enqdest);
    enqueuebadtriang(m, b, newbad);
    Ok(())
}

/// C: dequeuebadtriang (triangle.c:1835-1859). Remove a triangle from the
/// front of the highest-priority queue. The C NULL returns (no queues
/// nonempty, the out-of-range guards, an empty front) mirror as None;
/// the row id of the dequeued badtriang row otherwise.
pub(crate) fn dequeuebadtriang(m: &mut Mesh) -> Option<u32> {
    // If no queues are nonempty, return None.
    if m.firstnonemptyq < 0 {
        return None;
    }
    if m.firstnonemptyq >= 4096 {
        return None;
    }
    // Find the first triangle of the highest-priority queue.
    let result = m.queuefront[m.firstnonemptyq as usize];
    if result == NULL {
        return None;
    }
    // Remove the triangle from the queue.
    m.queuefront[m.firstnonemptyq as usize] = m.bad_tri_next(result);
    // If this queue is now empty, note the new highest-priority
    //   nonempty queue.
    if result == m.queuetail[m.firstnonemptyq as usize] {
        m.firstnonemptyq = m.nextnonemptyq[m.firstnonemptyq as usize];
    }
    Some(result)
}

/// C: inlens (triangle.c:1878-1896). Check whether a vertex lies in the
/// diametral lens of the subsegment from `eorg' to `edest' (the diametral
/// circle if `conformdel' is set). A dot product of two sides of the
/// triangle the vertex makes with the subsegment is used to check
/// whether the angle at the vertex is greater than (180 - 2 `minangle')
/// degrees (for lenses; 90 degrees for diametral circles).
///
/// The vertex arguments are the coordinate arrays the predicates use
/// (C REAL* vertex values copied through the chokepoint, D1/D5).
pub(crate) fn inlens(b: &Behavior, eorg: Vertex, edest: Vertex, eapex: Vertex) -> bool {
    let dotproduct =
        (eorg[0] - eapex[0]) * (edest[0] - eapex[0]) + (eorg[1] - eapex[1]) * (edest[1] - eapex[1]);
    if dotproduct < 0.0 {
        // b.goodangle is cos^2(minangle) (behavior_update squares it), so
        //   (2.0 * b->goodangle - 1.0) is cos(2 * minangle) exactly as the
        //   C arithmetic shapes it (D5: unchanged arithmetic).
        if b.conformdel
            || (dotproduct * dotproduct
                >= (2.0 * b.goodangle - 1.0)
                    * (2.0 * b.goodangle - 1.0)
                    * ((eorg[0] - eapex[0]) * (eorg[0] - eapex[0])
                        + (eorg[1] - eapex[1]) * (eorg[1] - eapex[1]))
                    * ((edest[0] - eapex[0]) * (edest[0] - eapex[0])
                        + (edest[1] - eapex[1]) * (edest[1] - eapex[1])))
        {
            return true;
        }
    }
    false
}

/// C: checkseg4encroach (triangle.c:1925-1985). Check a subsegment to see
/// if it is encroached; add it to the list of encroached subsegments if
/// it is. A subsegment is encroached if there is a vertex in its
/// diametral lens (the diametral circle with -D); each of the (at most
/// two) adjoining triangles' apices is tested. Returns the encroached
/// side mask (1 for the subsegment's own side, +2 for the other side),
/// allocating a badsubseg row (via the pool bridge) when the mask and
/// the nobisect gate admit it.
pub(crate) fn checkseg4encroach(m: &mut Mesh, b: &Behavior, testsubseg: Osub) -> Result<i32, Oom> {
    let mut encroached = 0;
    let mut sides = 0;

    let eorg = m.sorg(testsubseg);
    let edest = m.sdest(testsubseg);
    // Check one neighbor of the subsegment.
    let neighbortri = m.stpivot(testsubseg);
    // Does the neighbor exist, or is this a boundary edge?
    if neighbortri.tri != DUMMY_TRI {
        sides += 1;
        // Find a vertex opposite this subsegment.
        let eapex = m.apex(neighbortri);
        // Check whether the apex is in the diametral lens of the subsegment
        //   (the diametral circle if `conformdel' is set).
        if inlens(
            b,
            vertex_coords(m, eorg),
            vertex_coords(m, edest),
            vertex_coords(m, eapex),
        ) {
            encroached = 1;
        }
    }
    // Check the other neighbor of the subsegment.
    let testsym = testsubseg.ssym();
    let neighbortri = m.stpivot(testsym);
    // Does the neighbor exist, or is this a boundary edge?
    if neighbortri.tri != DUMMY_TRI {
        sides += 1;
        // Find the other vertex opposite this subsegment.
        let eapex = m.apex(neighbortri);
        // Check whether the apex is in the diametral lens of the subsegment
        //   (or the diametral circle, if `conformdel' is set).
        if inlens(
            b,
            vertex_coords(m, eorg),
            vertex_coords(m, edest),
            vertex_coords(m, eapex),
        ) {
            encroached += 2;
        }
    }

    if encroached != 0 && (b.nobisect == 0 || (b.nobisect == 1 && sides == 2)) {
        // Add the subsegment to the list of encroached subsegments.
        //   Be sure to get the orientation right.
        let encroachedseg = m.bad_subseg_alloc()?;
        if encroached == 1 {
            m.set_bad_sub_seg_enc(encroachedseg, sencode(testsubseg));
            m.set_bad_sub_seg_org(encroachedseg, eorg);
            m.set_bad_sub_seg_dest(encroachedseg, edest);
        } else {
            m.set_bad_sub_seg_enc(encroachedseg, sencode(testsym));
            m.set_bad_sub_seg_org(encroachedseg, edest);
            m.set_bad_sub_seg_dest(encroachedseg, eorg);
        }
    }

    Ok(encroached)
}

/// C: testtriangle (triangle.c:2001-2180). Test a triangle for quality
/// and size: the minimum-angle condition (the angle test against
/// b.goodangle, plus the aCute max-angle test under the acute feature,
/// exactly the fork's NO_ACUTE-inverted compile shape), the fixed/area
/// constraints, and the -u user test. Triangles that aren't up to spec
/// are added to the bad triangle queue.
///
/// This is the D9 callback site: C's
/// `b->triunsuitable_user_func(torg, tdest, tapex, area)' is invoked
/// exactly here, with copied VertexSample values (x, y, marker) and the
/// computed area; a true (= nonzero) return enqueues the triangle and
/// returns, exactly like C.
pub(crate) fn testtriangle(m: &mut Mesh, b: &Behavior, testtri: Otri) -> Result<(), Oom> {
    let torg = m.org(testtri);
    let tdest = m.dest(testtri);
    let tapex = m.apex(testtri);
    let torg_v = vertex_coords(m, torg);
    let tdest_v = vertex_coords(m, tdest);
    let tapex_v = vertex_coords(m, tapex);
    let dxod = torg_v[0] - tdest_v[0];
    let dyod = torg_v[1] - tdest_v[1];
    let dxda = tdest_v[0] - tapex_v[0];
    let dyda = tdest_v[1] - tapex_v[1];
    let dxao = tapex_v[0] - torg_v[0];
    let dyao = tapex_v[1] - torg_v[1];
    let dxod2 = dxod * dxod;
    let dyod2 = dyod * dyod;
    let dxda2 = dxda * dxda;
    let dyda2 = dyda * dyda;
    let dxao2 = dxao * dxao;
    let dyao2 = dyao * dyao;
    // Find the lengths of the triangle's three edges.
    let apexlen = dxod2 + dyod2;
    let orglen = dxda2 + dyda2;
    let destlen = dxao2 + dyao2;

    let minedge: f64;
    let mut angle: f64;
    let base1: u32;
    let base2: u32;
    let mut tri1: Otri;
    if (apexlen < orglen) && (apexlen < destlen) {
        // The edge opposite the apex is shortest.
        minedge = apexlen;
        // Find the square of the cosine of the angle at the apex.
        angle = dxda * dxao + dyda * dyao;
        angle = angle * angle / (orglen * destlen);
        base1 = torg;
        base2 = tdest;
        tri1 = testtri;
    } else if orglen < destlen {
        // The edge opposite the origin is shortest.
        minedge = orglen;
        // Find the square of the cosine of the angle at the origin.
        angle = dxod * dxao + dyod * dyao;
        angle = angle * angle / (apexlen * destlen);
        base1 = tdest;
        base2 = tapex;
        tri1 = testtri.lnext();
    } else {
        // The edge opposite the destination is shortest.
        minedge = destlen;
        // Find the square of the cosine of the angle at the destination.
        angle = dxod * dxda + dyod * dyda;
        angle = angle * angle / (apexlen * orglen);
        base1 = tapex;
        base2 = torg;
        tri1 = testtri.lprev();
    }

    if b.vararea || b.fixedarea || b.usertest {
        // Check whether the area is larger than permitted.
        let area = 0.5 * (dxod * dyda - dyod * dxda);
        if b.fixedarea && (area > b.maxarea) {
            // Add this triangle to the list of bad triangles.
            enqueuebadtri(m, b, testtri, minedge, tapex, torg, tdest)?;
            return Ok(());
        }

        // Nonpositive area constraints are treated as unconstrained.
        // (C reads areabound(*testtri) twice inside a single chained &&
        // that is already gated on b->vararea; the slot read is pure,
        // so the port reads it once inside the same gate.)
        if b.vararea {
            let areabound = m.tri_areabound(testtri.tri);
            if (area > areabound) && (areabound > 0.0) {
                // Add this triangle to the list of bad triangles.
                enqueuebadtri(m, b, testtri, minedge, tapex, torg, tdest)?;
                return Ok(());
            }
        }

        if b.usertest {
            // Check whether the user thinks this triangle is too large.
            // D9: the callback receives copied vertex data; D13 lets the
            //   &Behavior plumbing invoke the &mut self method here.
            if let Some(cb) = &b.user_func {
                let org_sample = VertexSample {
                    x: torg_v[0],
                    y: torg_v[1],
                    marker: m.vx_mark(torg),
                };
                let dest_sample = VertexSample {
                    x: tdest_v[0],
                    y: tdest_v[1],
                    marker: m.vx_mark(tdest),
                };
                let apex_sample = VertexSample {
                    x: tapex_v[0],
                    y: tapex_v[1],
                    marker: m.vx_mark(tapex),
                };
                if cb
                    .borrow_mut()
                    .is_unsuitable(org_sample, dest_sample, apex_sample, area)
                {
                    // Add this triangle to the list of bad triangles.
                    enqueuebadtri(m, b, testtri, minedge, tapex, torg, tdest)?;
                    return Ok(());
                }
            }
        }
    }

    #[cfg(feature = "acute")]
    // find the maximum edge and accordingly the pqr orientation
    // (C aCute fork, triangle.c:2096-2114; C computes `maxedge' but
    // never reads it - a dead store, not mirrored - and reads
    // `maxangle' below for the -U test.)
    let maxangle: f64 = {
        if (apexlen > orglen) && (apexlen > destlen) {
            // The edge opposite the apex is longest.
            // Find the cosine of the angle at the apex.
            (orglen + destlen - apexlen) / (2.0 * (orglen * destlen).sqrt())
        } else if orglen > destlen {
            // The edge opposite the origin is longest.
            // Find the cosine of the angle at the origin.
            (apexlen + destlen - orglen) / (2.0 * (apexlen * destlen).sqrt())
        } else {
            // The edge opposite the destination is longest.
            // Find the cosine of the angle at the destination.
            (apexlen + orglen - destlen) / (2.0 * (apexlen * orglen).sqrt())
        }
    };

    // Check whether the angle is smaller than permitted.
    // The aCute build (C triangle.c:2117-2121) adds the max-angle test;
    // C builds without it when NO_ACUTE is defined, the exact mirror of
    // the crate's `acute` feature (inverted compile switch).
    #[cfg(feature = "acute")]
    let bad = angle > b.goodangle || (maxangle < b.maxgoodangle && b.maxangle != 0.0);
    #[cfg(not(feature = "acute"))]
    let bad = angle > b.goodangle;
    if bad {
        // Use the rules of Miller, Pav, and Walkington to decide that
        //   certain triangles should not be split, even if they have bad
        //   angles.  A skinny triangle is not split if its shortest edge
        //   subtends a small input angle, and both endpoints of the edge
        //   lie on a concentric circular shell.  For convenience, I make
        //   a small adjustment to that rule:  I check if the endpoints
        //   of the edge both lie in segment interiors, equidistant from
        //   the apex where the two segments meet.
        // First, check if both points lie in segment interiors.
        if m.vx_type(base1) == SEGMENTVERTEX && m.vx_type(base2) == SEGMENTVERTEX {
            // Check if both points lie in a common segment.  If they do,
            //   the skinny triangle is enqueued to be split as usual.
            let mut testsub = m.tspivot(tri1);
            if testsub.ss == DUMMY_SUB {
                let mut tri2 = tri1;
                // No common segment.  Find a subsegment that contains
                //   `torg'.
                loop {
                    m.oprevself(&mut tri1);
                    testsub = m.tspivot(tri1);
                    if testsub.ss != DUMMY_SUB {
                        break;
                    }
                }
                // Find the endpoints of the containing segment.
                let org1 = m.segorg(testsub);
                let dest1 = m.segdest(testsub);
                // Find a subsegment that contains `tdest'.
                loop {
                    m.dnextself(&mut tri2);
                    testsub = m.tspivot(tri2);
                    if testsub.ss != DUMMY_SUB {
                        break;
                    }
                }
                // Find the endpoints of the containing segment.
                let org2 = m.segorg(testsub);
                let dest2 = m.segdest(testsub);
                // Check if the two containing segments have an endpoint in common.
                //   C compares the shared endpoint by coordinate equality
                //   (REAL loads), mirrored exactly.
                let mut joinvertex = NULL;
                if m.vx_x(dest1) == m.vx_x(org2) && m.vx_y(dest1) == m.vx_y(org2) {
                    joinvertex = dest1;
                } else if m.vx_x(org1) == m.vx_x(dest2) && m.vx_y(org1) == m.vx_y(dest2) {
                    joinvertex = org1;
                }
                if joinvertex != NULL {
                    // Compute the distance from the common endpoint (of the
                    //   two segments) to each of the endpoints of the shortest
                    //   edge.
                    let x1 = m.vx_x(base1) - m.vx_x(joinvertex);
                    let y1 = m.vx_y(base1) - m.vx_y(joinvertex);
                    let x2 = m.vx_x(base2) - m.vx_x(joinvertex);
                    let y2 = m.vx_y(base2) - m.vx_y(joinvertex);
                    let dist1 = x1 * x1 + y1 * y1;
                    let dist2 = x2 * x2 + y2 * y2;
                    // If the two distances are equal, don't split the triangle.
                    if (dist1 < 1.001 * dist2) && (dist1 > 0.999 * dist2) {
                        // Return now to avoid enqueueing the bad triangle.
                        return Ok(());
                    }
                }
            }
        }

        // Add this triangle to the list of bad triangles.
        enqueuebadtri(m, b, testtri, minedge, tapex, torg, tdest)?;
    }
    Ok(())
}

/// C: quality_statistics (triangle.c:8143-8312). Measure the mesh: the
/// shortest/longest squared-edge
/// extrema, the smallest/biggest doubled areas, the squared-cosine angle
/// extrema (converted to degrees at the end), the minimum altitude per
/// longest-edge and the worst aspect ratio, plus the angle histogram
/// (18 ten-degree buckets) and the aspect-ratio histogram (16 buckets).
/// The exact arithmetic through tri_counterclockwise and the table
/// constants (cossquaretable computed from cos, ratiotable literals)
/// are byte-parity-relevant for the CLI's statistics output.
///
/// C returns 0; triangle_mesh_quality discards it, so the port returns
/// nothing. The C stores `q->worstaspect' twice (idempotent); the port
/// stores it once.
pub(crate) fn quality_statistics(m: &mut Mesh, b: &Behavior, q: &mut Quality) {
    let radconst = PI / 18.0;
    let degconst = 180.0 / PI;
    let mut cossquaretable = [0.0f64; 8];
    for (i, slot) in cossquaretable.iter_mut().enumerate() {
        let angle = radconst * (i + 1) as f64;
        *slot = angle.cos() * angle.cos();
    }
    for slot in q.angle_table.iter_mut() {
        *slot = 0;
    }

    let ratiotable: [f64; 16] = [
        1.5, 2.0, 2.5, 3.0, 4.0, 6.0, 10.0, 15.0, 25.0, 50.0, 100.0, 300.0, 1000.0, 10000.0,
        100000.0, 0.0,
    ];
    for slot in q.aspect_table.iter_mut() {
        *slot = 0;
    }

    let mut minaltitude = m.xmax - m.xmin + m.ymax - m.ymin;
    minaltitude = minaltitude * minaltitude;
    let mut shortest = minaltitude;
    let mut longest = 0.0;
    let mut smallestarea = minaltitude;
    let mut biggestarea = 0.0;
    let mut worstaspect = 0.0;
    let mut smallestangle = 0.0;
    let mut biggestangle = 2.0;
    let mut acutebiggest = true;

    let mut t = m.triangles.traversal();
    while let Some(tri) = m.triangletraverse(&mut t) {
        let triangleloop = Otri { tri, orient: 0 };
        let p = [
            vertex_coords(m, m.org(triangleloop)),
            vertex_coords(m, m.dest(triangleloop)),
            vertex_coords(m, m.apex(triangleloop)),
        ];
        let mut dx = [0.0f64; 3];
        let mut dy = [0.0f64; 3];
        let mut edgelength = [0.0f64; 3];
        let mut trilongest2 = 0.0f64;

        for i in 0..3 {
            let j = PLUS1MOD3[i] as usize;
            let k = MINUS1MOD3[i] as usize;
            dx[i] = p[j][0] - p[k][0];
            dy[i] = p[j][1] - p[k][1];
            edgelength[i] = dx[i] * dx[i] + dy[i] * dy[i];
            if edgelength[i] > trilongest2 {
                trilongest2 = edgelength[i];
            }
            if edgelength[i] > longest {
                longest = edgelength[i];
            }
            if edgelength[i] < shortest {
                shortest = edgelength[i];
            }
        }

        // C: triarea = tri_counterclockwise(m, b, p[0], p[1], p[2]) (the
        //   doubled area; the C histogram arithmetic works on that value).
        let triarea = tri_counterclockwise(m, b, p[0], p[1], p[2]);
        if triarea < smallestarea {
            smallestarea = triarea;
        }
        if triarea > biggestarea {
            biggestarea = triarea;
        }
        let triminaltitude2 = triarea * triarea / trilongest2;
        if triminaltitude2 < minaltitude {
            minaltitude = triminaltitude2;
        }
        let triaspect2 = trilongest2 / triminaltitude2;
        if triaspect2 > worstaspect {
            worstaspect = triaspect2;
        }
        // C reads ratiotable[aspectindex] before the bound guard; the
        //   value never crosses the 15 boundary there, so the conjunct
        //   order swap below is semantics-preserving.
        let mut aspectindex: usize = 0;
        while aspectindex < 15 && triaspect2 > ratiotable[aspectindex] * ratiotable[aspectindex] {
            aspectindex += 1;
        }
        q.aspect_table[aspectindex] += 1;

        for i in 0..3 {
            let j = PLUS1MOD3[i] as usize;
            let k = MINUS1MOD3[i] as usize;
            let dotproduct = dx[j] * dx[k] + dy[j] * dy[k];
            let cossquare = dotproduct * dotproduct / (edgelength[j] * edgelength[k]);
            // C: tendegree = 8; for (ii = 7; ii >= 0; ii--) { if
            //   (cossquare > cossquaretable[ii]) tendegree = ii; }  The
            //   loop has no break; the last (smallest satisfying) ii wins.
            let mut tendegree: usize = 8;
            for ii in (0..8).rev() {
                if cossquare > cossquaretable[ii] {
                    tendegree = ii;
                }
            }
            if dotproduct <= 0.0 {
                q.angle_table[tendegree] += 1;
                if cossquare > smallestangle {
                    smallestangle = cossquare;
                }
                if acutebiggest && (cossquare < biggestangle) {
                    biggestangle = cossquare;
                }
            } else {
                q.angle_table[17 - tendegree] += 1;
                if acutebiggest || (cossquare > biggestangle) {
                    biggestangle = cossquare;
                    acutebiggest = false;
                }
            }
        }
    }

    let shortest = shortest.sqrt();
    let longest = longest.sqrt();
    let minaltitude = minaltitude.sqrt();
    let worstaspect = worstaspect.sqrt();
    let smallestarea = smallestarea * 0.5;
    let biggestarea = biggestarea * 0.5;
    let smallestangle = if smallestangle >= 1.0 {
        0.0
    } else {
        degconst * smallestangle.sqrt().acos()
    };
    let biggestangle = if biggestangle >= 1.0 {
        180.0
    } else if acutebiggest {
        degconst * biggestangle.sqrt().acos()
    } else {
        180.0 - degconst * biggestangle.sqrt().acos()
    };

    q.worst_aspect = worstaspect;
    q.min_altitude = minaltitude;
    q.shortest = shortest;
    q.longest = longest;
    q.smallest_area = smallestarea;
    q.biggest_area = biggestarea;
    q.smallest_angle = smallestangle;
    q.biggest_angle = biggestangle;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algorithm::delaunay::delaunay;
    use crate::mesh::handles::decode;

    /// A hand-usable Behavior: C's parsecommandline defaults plus the
    /// given minangle (which behavior_update turns into goodangle).
    fn behavior_q(minangle: f64) -> Behavior {
        let mut b = Behavior {
            quality: true,
            ..Behavior::default_form()
        };
        b.minangle = minangle;
        b.behavior_update();
        b
    }

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
        let t = m.maketriangle(usesegments, false).expect("tri");
        m.setorg(t, va);
        m.setdest(t, vb);
        m.setapex(t, vc);
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
            enqueuebadtri(&mut m, &b, t, *key, 0, 0, 0).expect("enqueue");
            // the freshly allocated row (ids 1..=5; the pool has no
            // sentinel) is the new front of exactly the expected queue.
            assert_eq!(m.queuefront[*expected_qn], i as u32 + 1);
        }
        // firstnonemptyq is the highest-priority (largest) queue number
        // used: the first row (key 0.0) sits at 4094.
        assert_eq!(m.firstnonemptyq, 4094);
        // The 4094 queue holds exactly the key-0.0 row; the 2047 queue
        // holds the key-1.0 row; ... the linkage order (front == then
        // tail ==) pins which row is where.
        let row0 = m.queuefront[4094];
        assert_eq!(m.bad_tri_key(row0), 0.0);
        assert_eq!(m.queuefront[2047], m.queuefront[4094] + 1);
        assert_eq!(m.bad_tri_key(m.queuefront[2047]), 1.0);
        assert_eq!(m.bad_tri_key(m.queuefront[2044]), 4.0);
        assert_eq!(m.bad_tri_key(m.queuefront[2051]), 0.25);
        assert_eq!(m.bad_tri_key(m.queuefront[2046]), 2.0);
        // every queue's tail points at itself (single-element queues)
        for qn in [4094usize, 2047, 2044, 2051, 2046] {
            assert_eq!(m.queuetail[qn], m.queuefront[qn]);
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
        enqueuebadtri(&mut m, &b, t, 4.0, 3, 1, 2).expect("enqueue"); // r1
        enqueuebadtri(&mut m, &b, t, 4.0, 6, 4, 5).expect("enqueue"); // r2, same queue
        enqueuebadtri(&mut m, &b, t, 1.0, 9, 7, 8).expect("enqueue"); // r3, higher priority
        assert_eq!(m.badtriangles.live_count(), 3);
        // dequeue: r3 first (queue 2047 before 2044), then r2 was
        // appended after r1 in queue 2044: r1, then r2.
        let d1 = dequeuebadtriang(&mut m).expect("row");
        assert_eq!(m.bad_tri_key(d1), 1.0);
        assert_eq!(m.bad_tri_apex(d1), 9);
        assert_eq!(m.bad_tri_org(d1), 7);
        assert_eq!(m.bad_tri_dest(d1), 8);
        assert_eq!(m.bad_tri_poortri(d1), encode(t));
        let d2 = dequeuebadtriang(&mut m).expect("row");
        assert_eq!(m.bad_tri_key(d2), 4.0);
        assert_eq!(m.bad_tri_org(d2), 1);
        let d3 = dequeuebadtriang(&mut m).expect("row");
        assert_eq!(m.bad_tri_key(d3), 4.0);
        assert_eq!(m.bad_tri_org(d3), 4);
        // The queue drained: firstnonemptyq falls back through the
        // nextnonemptyq chain to -1.
        assert_eq!(m.firstnonemptyq, -1);
        assert!(dequeuebadtriang(&mut m).is_none());
        // dequeued rows stay allocated (deallocation belongs to the
        // algorithm/refine.rs drainers).
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
        enqueuebadtri(&mut m, &b, t, 1.0, 0, 0, 0).expect("enqueue");
        let r_high = m.queuefront[2047];
        enqueuebadtri(&mut m, &b, t, 4.0, 0, 0, 0).expect("enqueue");
        let r_low = m.queuefront[2044];
        assert_eq!(m.firstnonemptyq, 2047);
        assert_eq!(m.nextnonemptyq[2047], 2044, "2047 points at 2044");
        assert_eq!(m.nextnonemptyq[2044], -1, "2044 ends the chain");
        // and one more between them: queue 2046 (key 2.0).
        enqueuebadtri(&mut m, &b, t, 2.0, 0, 0, 0).expect("enqueue");
        assert_eq!(m.nextnonemptyq[2047], 2046);
        assert_eq!(m.nextnonemptyq[2046], 2044);
        // the drain order follows the chain: 2047, 2046, 2044.
        assert_eq!(dequeuebadtriang(&mut m), Some(r_high));
        assert_eq!(m.firstnonemptyq, 2046);
        let r_mid = m.queuefront[2046];
        assert_eq!(dequeuebadtriang(&mut m), Some(r_mid));
        assert_eq!(m.firstnonemptyq, 2044);
        assert_eq!(dequeuebadtriang(&mut m), Some(r_low));
        assert_eq!(m.firstnonemptyq, -1);
    }

    // ------------------------------------------------------------------
    // inlens
    // ------------------------------------------------------------------

    /// Hand-checked lens cases on the subsegment (0,0)-(2,0): the angle
    /// threshold derives from cos(2*minangle), all margins >= 2%.
    #[test]
    fn inlens_decides_by_dot_and_the_minangle_threshold() {
        let mut b = behavior_q(20.0);
        let eorg = [0.0, 0.0, 0.0];
        let edest = [2.0, 0.0, 0.0];
        // apex (1, 0.1): dot = -0.99 < 0 and dot^2 = 0.9801 beats the
        // cos^2(40)*1.0201 = 0.5987 threshold => in the lens at 20 deg.
        let near = [1.0, 0.1, 0.0];
        assert!(inlens(&b, eorg, edest, near));
        // apex (1, 10): dot = -1 + 100 = 99 > 0 => far outside, no lens.
        let far = [1.0, 10.0, 0.0];
        assert!(!inlens(&b, eorg, edest, far));
        // apex (1, 1.5): dot = -1 + 2.25 = 1.25 > 0 => outside.
        let high = [1.0, 1.5, 0.0];
        assert!(!inlens(&b, eorg, edest, high));
        // same near apex but a 1-degree lens: threshold
        // cos^2(2) * 1.0201 = 1.0189 > dot^2 = 0.9801 => outside.
        b.minangle = 1.0;
        b.behavior_update();
        assert!(!inlens(&b, eorg, edest, near));
        // a 45-degree lens: threshold (2*0.5-1)^2 = 0 => inside.
        b.minangle = 45.0;
        b.behavior_update();
        assert!(inlens(&b, eorg, edest, near));
        // conformdel turns the test into the diametral circle: any
        // dot < 0 decides, so even the 1-degree behavior passes.
        let mut b1 = behavior_q(1.0);
        b1.conformdel = true;
        b1.behavior_update();
        assert!(inlens(&b1, eorg, edest, near));
    }

    // ------------------------------------------------------------------
    // checkseg4encroach
    // ------------------------------------------------------------------

    /// The pair-with-subseg fixture: two triangles sharing the edge
    /// (0,0)-(2,0) with a subsegment on it, their apexes at the given
    /// points. Returns the subsegment handle as `tspivot` hands it over
    /// on the first triangle's orient-0 edge.
    fn seg_fixture(m: &mut Mesh, above: (f64, f64), below: (f64, f64)) -> Osub {
        let b = Behavior::default_form();
        let a = vxy(m, 0.0, 0.0);
        let e = vxy(m, 2.0, 0.0);
        let c = vxy(m, above.0, above.1);
        let d = vxy(m, below.0, below.1);
        let t1 = m.maketriangle(true, false).expect("tri");
        m.setorg(t1, a);
        m.setdest(t1, e);
        m.setapex(t1, c);
        let t2 = m.maketriangle(true, false).expect("tri");
        m.setorg(t2, e);
        m.setdest(t2, a);
        m.setapex(t2, d);
        m.bond(
            Otri {
                tri: t1.tri,
                orient: 0,
            },
            Otri {
                tri: t2.tri,
                orient: 0,
            },
        );
        let shared = Otri {
            tri: t1.tri,
            orient: 0,
        };
        crate::algorithm::transform::insertsubseg(m, &b, shared, 0).expect("insertsubseg");
        m.tspivot(shared)
    }

    /// A behavior with minangle (through behavior_update's derived
    /// goodangle) and the given nobisect, the -Y semantics gate.
    fn behavior_y(minangle: f64, nobisect: i32) -> Behavior {
        let mut b = behavior_q(minangle);
        b.nobisect = nobisect;
        b
    }

    #[test]
    fn checkseg4encroach_reports_all_side_and_gate_outcomes() {
        // Above-inside (1, 0.1) and below-clear (1, -10): encroached 1,
        // a badsubseg row lands with the subsegment's own orientation
        // (sencode(sub), sorg, sdest) exactly as C writes it.
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        let sub = seg_fixture(&mut m, (1.0, 0.1), (1.0, -10.0));
        assert_eq!(m.badsubsegs.live_count(), 0);
        assert_eq!(checkseg4encroach(&mut m, &b, sub).expect("check"), 1);
        assert_eq!(m.badsubsegs.live_count(), 1);
        assert_eq!(m.bad_sub_seg_enc(1), sencode(sub));
        assert_eq!(m.bad_sub_seg_org(1), m.sorg(sub));
        assert_eq!(m.bad_sub_seg_dest(1), m.sdest(sub));

        // Both sides inside (1, 0.1) and (1, -0.1): encroached 3, the
        // row carries the flipped (ssym) orientation.
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        let sub = seg_fixture(&mut m, (1.0, 0.1), (1.0, -0.1));
        assert_eq!(checkseg4encroach(&mut m, &b, sub).expect("check"), 3);
        let testsym = sub.ssym();
        assert_eq!(m.bad_sub_seg_enc(1), sencode(testsym));
        assert_eq!(m.bad_sub_seg_org(1), m.sdest(sub), "org = edest");
        assert_eq!(m.bad_sub_seg_dest(1), m.sorg(sub), "dest = eorg");

        // Neither side inside: no encroachment, no row.
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        let sub = seg_fixture(&mut m, (10.0, 1.0), (1.0, -10.0));
        assert_eq!(checkseg4encroach(&mut m, &b, sub).expect("check"), 0);
        assert_eq!(m.badsubsegs.live_count(), 0);

        // nobisect == 2: encroached is still reported, but the row is
        // gated off (the -Y2 semantics).
        let b = behavior_y(20.0, 2);
        let mut m = mesh(&b);
        let sub = seg_fixture(&mut m, (1.0, 0.1), (1.0, -10.0));
        assert_eq!(checkseg4encroach(&mut m, &b, sub).expect("check"), 1);
        assert_eq!(m.badsubsegs.live_count(), 0);

        // nobisect == 1 with both sides (sides == 2): the row lands.
        let b = behavior_y(20.0, 1);
        let mut m = mesh(&b);
        let sub = seg_fixture(&mut m, (1.0, 0.1), (1.0, -0.1));
        assert_eq!(checkseg4encroach(&mut m, &b, sub).expect("check"), 3);
        assert_eq!(m.badsubsegs.live_count(), 1);

        // nobisect == 1 with a single side (sides == 1): gated off.
        // (One-sided: a lone triangle with the subsegment on the
        // (0,0)-(2,0) edge, the other side is space; the two-sided
        // seg_fixture above would count sides == 2 and let the row
        // through, the boundary test below owns the geometry itself.)
        let b = behavior_y(20.0, 1);
        let mut m = mesh(&b);
        let a = vxy(&mut m, 0.0, 0.0);
        let e = vxy(&mut m, 2.0, 0.0);
        let apex = vxy(&mut m, 1.0, 0.1);
        let t = m.maketriangle(true, false).expect("tri");
        m.setorg(t, a);
        m.setdest(t, e);
        m.setapex(t, apex);
        crate::algorithm::transform::insertsubseg(
            &mut m,
            &b,
            Otri {
                tri: t.tri,
                orient: 0,
            },
            0,
        )
        .expect("insertsubseg");
        let sub = m.tspivot(Otri {
            tri: t.tri,
            orient: 0,
        });
        assert_eq!(checkseg4encroach(&mut m, &b, sub).expect("check"), 1);
        assert_eq!(m.badsubsegs.live_count(), 0);
    }

    /// A one-sided subsegment (boundary): stpivot on the empty side
    /// returns the dummy triangle, sides stays 1, and the encroached
    /// mask is 1 with the subsegment's own orientation in the row.
    #[test]
    fn checkseg4encroach_on_a_boundary_subsegment_counts_one_side() {
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        // A lone triangle (a, e, apex-inside) with a subsegment on the
        // (a, e) edge; the other side of the subsegment is space.
        let a = vxy(&mut m, 0.0, 0.0);
        let e = vxy(&mut m, 2.0, 0.0);
        let apex = vxy(&mut m, 1.0, 0.1);
        let t = m.maketriangle(true, false).expect("tri");
        m.setorg(t, a);
        m.setdest(t, e);
        m.setapex(t, apex);
        crate::algorithm::transform::insertsubseg(
            &mut m,
            &b,
            Otri {
                tri: t.tri,
                orient: 0,
            },
            0,
        )
        .expect("insertsubseg");
        let sub = m.tspivot(Otri {
            tri: t.tri,
            orient: 0,
        });
        assert_eq!(checkseg4encroach(&mut m, &b, sub).expect("check"), 1);
        assert_eq!(m.badsubsegs.live_count(), 1);
        assert_eq!(m.bad_sub_seg_enc(1), sencode(sub));
        assert_eq!(m.bad_sub_seg_org(1), m.sorg(sub));
        assert_eq!(m.bad_sub_seg_dest(1), m.sdest(sub));
    }

    // ------------------------------------------------------------------
    // testtriangle (the Unsuitable callback site, D9/D13)
    // ------------------------------------------------------------------

    #[test]
    fn testtriangle_invokes_the_unsuitable_callback_at_the_c_site() {
        // A recording Unsuitable stub: pins the D9 passing semantics.
        // The Box<dyn Unsuitable> is opaque (D9: no downcast surface), so
        // the sink is an Rc the test holds next to the box.
        type Call = (VertexSample, VertexSample, VertexSample, f64);
        struct Recorder {
            sink: std::rc::Rc<std::cell::RefCell<Vec<Call>>>,
            verdict: bool,
        }
        impl crate::options::Unsuitable for Recorder {
            fn is_unsuitable(
                &mut self,
                org: VertexSample,
                dest: VertexSample,
                apex: VertexSample,
                area: f64,
            ) -> bool {
                self.sink.borrow_mut().push((org, dest, apex, area));
                self.verdict
            }
        }
        let recorded = std::rc::Rc::new(std::cell::RefCell::new(Vec::<Call>::new()));
        let mut b = behavior_q(20.0);
        b.usertest = true;
        b.user_func = Some(std::cell::RefCell::new(Box::new(Recorder {
            sink: recorded.clone(),
            verdict: true,
        })));
        let mut m = mesh(&b);
        let t = lone_tri(&mut m, false, (0.0, 0.0), (1.0, 0.0), (0.0, 1.0));
        m.vx_set_mark(m.org(t), 11);
        m.vx_set_mark(m.dest(t), 22);
        m.vx_set_mark(m.apex(t), 33);
        assert_eq!(m.badtriangles.live_count(), 0);
        testtriangle(&mut m, &b, t).expect("test");
        // exactly one call, with the copied samples and the hand-derived
        // area: the triangle (0,0),(1,0),(0,1) has doubled area 1.0, so
        // the C `area' (half the doubled area) is 0.5.
        let calls = recorded.borrow();
        assert_eq!(calls.len(), 1);
        let (org, dest, apex, area) = calls[0];
        assert_eq!((org.x, org.y, org.marker), (0.0, 0.0, 11));
        assert_eq!((dest.x, dest.y, dest.marker), (1.0, 0.0, 22));
        assert_eq!((apex.x, apex.y, apex.marker), (0.0, 1.0, 33));
        assert_eq!(area, 0.5);
        drop(calls);
        // true => enqueue exactly the C row: poortri, minedge (squared
        // shortest edge = 1.0), apex/org/dest ids.
        assert_eq!(m.badtriangles.live_count(), 1);
        assert_eq!(m.bad_tri_poortri(1), encode(t));
        assert_eq!(m.bad_tri_key(1), 1.0);
        assert_eq!(m.bad_tri_apex(1), m.apex(t));
        assert_eq!(m.bad_tri_org(1), m.org(t));
        assert_eq!(m.bad_tri_dest(1), m.dest(t));
    }

    #[test]
    fn testtriangle_skips_the_callback_when_usertest_is_off() {
        struct Recorder {
            calls: std::rc::Rc<std::cell::Cell<usize>>,
        }
        impl crate::options::Unsuitable for Recorder {
            fn is_unsuitable(
                &mut self,
                _: VertexSample,
                _: VertexSample,
                _: VertexSample,
                _: f64,
            ) -> bool {
                self.calls.set(self.calls.get() + 1);
                true
            }
        }
        let calls = std::rc::Rc::new(std::cell::Cell::new(0usize));
        let mut b = behavior_q(20.0);
        // usertest off: the callback rides the behavior but is never
        // invoked; the triangle is equilateral-ish enough to pass the
        // 20-degree angle test, so nothing is enqueued.
        b.user_func = Some(std::cell::RefCell::new(Box::new(Recorder {
            calls: calls.clone(),
        })));
        let mut m = mesh(&b);
        let t = lone_tri(&mut m, false, (0.0, 0.0), (1.0, 0.0), (0.0, 1.0));
        testtriangle(&mut m, &b, t).expect("test");
        assert_eq!(m.badtriangles.live_count(), 0);
        assert_eq!(calls.get(), 0);
    }

    #[test]
    fn testtriangle_enqueues_by_fixed_area_and_relaxes_below_it() {
        // fixedarea: above maxarea -> enqueue with the min-edge key;
        // below -> no enqueue (and the angle test passes on this shape).
        let mut b = behavior_q(20.0);
        b.fixedarea = true;
        b.maxarea = 0.1;
        let mut m = mesh(&b);
        let t = lone_tri(&mut m, false, (0.0, 0.0), (1.0, 0.0), (0.0, 1.0));
        // the halved area 0.5 > maxarea 0.1: enqueue, key = the squared
        // shortest edge 1.0.
        testtriangle(&mut m, &b, t).expect("test");
        assert_eq!(m.badtriangles.live_count(), 1);
        assert_eq!(m.bad_tri_key(1), 1.0);

        // maxarea 1.0 above the area: nothing enqueued (the right
        // triangle's 45-degree minimum passes the q20 angle test).
        let mut b2 = behavior_q(20.0);
        b2.fixedarea = true;
        b2.maxarea = 1.0;
        let mut m = mesh(&b2);
        let t = lone_tri(&mut m, false, (0.0, 0.0), (1.0, 0.0), (0.0, 1.0));
        testtriangle(&mut m, &b2, t).expect("test");
        assert_eq!(m.badtriangles.live_count(), 0);
    }

    #[test]
    fn testtriangle_enqueues_skinny_triangles_by_min_angle() {
        // q20: the nearly-degenerate (0,0),(2,0),(1,0.1) has its
        // smallest-edge angle far below 20 degrees (hand-derived
        // cossquare 0.9901 > cos^2(20) = 0.883), so it enqueues;
        // the right triangle (0,0),(1,0),(0,1) has 45-degree minimum,
        // cossquare 0.5 < 0.883, so it passes.
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        let skinny = lone_tri(&mut m, false, (0.0, 0.0), (2.0, 0.0), (1.0, 0.1));
        testtriangle(&mut m, &b, skinny).expect("test");
        assert_eq!(m.badtriangles.live_count(), 1);
        // minedge: the squared shortest edge: (1,0.1)-(0,0) length
        // squared = 1.01 (decisively below the other two).
        assert_eq!(m.bad_tri_key(1), 1.01);
        let good = lone_tri(&mut m, false, (0.0, 0.0), (1.0, 0.0), (0.0, 1.0));
        testtriangle(&mut m, &b, good).expect("test");
        assert_eq!(m.badtriangles.live_count(), 1, "no second enqueue");
    }

    // ------------------------------------------------------------------
    // checkmesh / checkdelaunay
    // ------------------------------------------------------------------

    /// A delaunay() square triangulation is a real mesh: checkmesh and
    /// checkdelaunay report zero horrors, and checkmesh's inversion
    /// probes advanced the counterclockwise counter (the mirror of the
    /// C counter side effects inside the check loop).
    #[test]
    fn checkmesh_and_checkdelaunay_pass_a_delaunay_mesh() {
        let mut b = Behavior {
            poly: true,
            usesegments: true,
            ..Behavior::default_form()
        };
        let mut m = Mesh::new(&b, 4, 2, 0, 0).expect("mesh init");
        for (x, y) in [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)] {
            let v = m.vx_alloc().expect("vertex alloc");
            m.set_vx_x(v, x);
            m.set_vx_y(v, y);
        }
        assert_eq!(delaunay(&mut m, &b).expect("delaunay"), 4);

        let counterclock_before = m.counterclockcount;
        assert_eq!(checkmesh(&mut m, &mut b), 0);
        // the inversion test ran once per triangle through the exact
        // predicates
        assert!(m.counterclockcount > counterclock_before);
        let nonregular_before = m.incirclecount + m.hyperbolacount + m.orient3dcount;
        assert_eq!(checkdelaunay(&mut m, &mut b), 0);
        assert!(m.incirclecount + m.hyperbolacount + m.orient3dcount > nonregular_before);
    }

    #[test]
    fn checkmesh_counts_a_flat_inverted_triangle() {
        // The CW triangle (0,0),(1,0),(0,-1): inversion counted once at
        // orient 0; the unbonded edges are hull edges (sym = dummy), so
        // no other horrors.
        let mut b = Behavior::default_form();
        let mut m = mesh(&b);
        lone_tri(&mut m, false, (0.0, 0.0), (1.0, 0.0), (0.0, -1.0));
        assert_eq!(checkmesh(&mut m, &mut b), 1);
    }

    #[test]
    fn checkmesh_counts_two_bonded_triangles_disagreeing_on_the_shared_edge() {
        // t1 = (a, b, c); t3 = (d, a, e), both CCW, bonded at orient 0
        // (a hybrid edge: the bond is reciprocal, but the two sides
        // disagree on the shared vertices - t1's orient-0 edge is
        // a->b, t3's is d->a). The identity check fires from both
        // sides of the shared edge (checkmesh has no pair-once guard):
        // t1's visit counts (b != d), t3's visit counts (d != b) - two
        // horrors; no inversion (both CCW), the other four edges are
        // unbonded hull edges (skipped like C's dummytri branch).
        let mut b = Behavior::default_form();
        let mut m = mesh(&b);
        let a = (0.0, 0.0);
        let bb = (2.0, 0.0);
        let c = (1.0, 1.0);
        let d = (1.0, -1.0);
        let e = (0.5, -2.0);
        let t1 = lone_tri(&mut m, false, a, bb, c);
        let t3 = lone_tri(&mut m, false, d, a, e);
        m.bond(
            Otri {
                tri: t1.tri,
                orient: 0,
            },
            Otri {
                tri: t3.tri,
                orient: 0,
            },
        );
        assert_eq!(checkmesh(&mut m, &mut b), 2);
    }

    #[test]
    fn checkdelaunay_counts_a_non_locally_delaunay_edge() {
        // (a, b) shared by t1 = (a, b, c) and t2 = (b, a, d) with
        // d = (0.4, 0.4) strictly inside the circumcircle of abc
        // (center (0.5, 0.5), squared radius 0.5; |d - center|^2 =
        // 0.02): the pair is tested once (the row-id pair-once guard),
        // one horror. The far variant d = (3, 3) is outside: zero.
        let mut b = Behavior::default_form();
        let mut m = mesh(&b);
        let t1 = lone_tri(&mut m, false, (0.0, 0.0), (1.0, 0.0), (0.0, 1.0));
        let x = vxy(&mut m, 0.4, 0.4);
        let t2 = m.maketriangle(false, false).expect("tri");
        m.setorg(t2, m.dest(t1));
        m.setdest(t2, m.org(t1));
        m.setapex(t2, x);
        m.bond(
            Otri {
                tri: t1.tri,
                orient: 0,
            },
            Otri {
                tri: t2.tri,
                orient: 0,
            },
        );
        assert_eq!(checkdelaunay(&mut m, &mut b), 1);

        let mut b = Behavior::default_form();
        let mut m = mesh(&b);
        let t1 = lone_tri(&mut m, false, (0.0, 0.0), (1.0, 0.0), (0.0, 1.0));
        let x = vxy(&mut m, 3.0, 3.0);
        let t2 = m.maketriangle(false, false).expect("tri");
        m.setorg(t2, m.dest(t1));
        m.setdest(t2, m.org(t1));
        m.setapex(t2, x);
        m.bond(
            Otri {
                tri: t1.tri,
                orient: 0,
            },
            Otri {
                tri: t2.tri,
                orient: 0,
            },
        );
        assert_eq!(checkdelaunay(&mut m, &mut b), 0);
    }

    // ------------------------------------------------------------------
    // quality_statistics
    // ------------------------------------------------------------------

    /// Hand-derived expected values for the right triangle
    /// (0,0),(1,0),(0,1) (all values exact in binary except the two
    /// acos/degconst conversions, asserted as 45 degrees within the
    /// last-ulp rounding):
    /// doubled area 1.0 -> areas 0.5; squared edges 1, 2, 1 -> shortest
    /// 1, longest sqrt(2); min altitude^2 = 1/2 -> sqrt(1/2); aspect =
    /// 2/0.5 = 4 -> sqrt(4) = 2 (aspect bucket: 4 > 2.25 but not > 4
    /// -> index 1); angle tables: the 90-degree corner lands at bucket
    /// 8 with cossquare 0, each 45-degree corner at bucket 4 with
    /// cossquare 0.5 (dot < 0 columns); smallest/biggest angle = 45
    /// degrees each.
    #[test]
    fn quality_statistics_measures_the_right_triangle() {
        let b = Behavior::default_form();
        let mut m = mesh(&b);
        // The run's bounding box, as boundingbox would leave it (a real
        // run always has the span set before quality statistics runs).
        m.xmin = -10.0;
        m.ymin = -10.0;
        m.xmax = 10.0;
        m.ymax = 10.0;
        lone_tri(&mut m, false, (0.0, 0.0), (1.0, 0.0), (0.0, 1.0));
        let mut q = Quality::default();
        quality_statistics(&mut m, &b, &mut q);
        assert_eq!(q.smallest_area, 0.5);
        assert_eq!(q.biggest_area, 0.5);
        assert_eq!(q.shortest, 1.0);
        assert_eq!(q.longest, 2.0f64.sqrt());
        assert_eq!(q.min_altitude, 0.5f64.sqrt());
        assert_eq!(q.worst_aspect, 2.0);
        let mut expected_aspect = [0usize; 16];
        expected_aspect[1] = 1;
        assert_eq!(q.aspect_table, expected_aspect);
        let mut expected_angle = [0usize; 18];
        expected_angle[8] = 1;
        expected_angle[4] = 2;
        assert_eq!(q.angle_table, expected_angle);
        // smallest = degconst * acos(sqrt(0.5)) = 45 degrees; the
        // right triangle's corner angles are 45, 45, 90, so the
        // biggest is 90. The exact f64 values depend on the libm acos,
        // so assert within one rounding step, not bit-exact.
        assert!(
            (q.smallest_angle - 45.0).abs() < 1e-13,
            "{}",
            q.smallest_angle
        );
        assert!(
            (q.biggest_angle - 90.0).abs() < 1e-13,
            "{}",
            q.biggest_angle
        );
        // A quality probe mutated the counter through tri_counterclockwise.
        assert!(m.counterclockcount >= 1);
    }

    /// An empty (triangle-free) mesh reports the degenerate-but-init
    /// shape: angle and aspect tables zero, extrema at their C init
    /// values (bounds-driven minaltitude; shortest collapses to it).
    #[test]
    fn quality_statistics_on_an_empty_mesh_keeps_the_c_init_values() {
        let b = Behavior::default_form();
        let mut m = mesh(&b);
        // bounds-driven init: minaltitude = (span)^2 = (20+20)^2
        m.xmin = -10.0;
        m.ymin = -10.0;
        m.xmax = 10.0;
        m.ymax = 10.0;
        let mut q = Quality::default();
        quality_statistics(&mut m, &b, &mut q);
        assert_eq!(q.angle_table, [0usize; 18]);
        assert_eq!(q.aspect_table, [0usize; 16]);
        assert_eq!(q.min_altitude, 1600.0f64.sqrt());
        assert_eq!(q.shortest, 1600.0f64.sqrt());
        assert_eq!(q.longest, 0.0);
        // smallestarea starts at the squared span = 1600 -> *0.5
        assert_eq!(q.smallest_area, 800.0);
        assert_eq!(q.biggest_area, 0.0);
        // no triangle touched the angle extrema: the C tail converts
        // smallestangle 0.0 -> 90 degrees (acos(0) = pi/2), biggestangle
        // 2.0 -> 180.
        assert!((q.smallest_angle - 90.0).abs() < 1e-13);
        assert_eq!(q.biggest_angle, 180.0);
    }

    /// The vertex2tri contract still pins makevertexmap (PR #35): the
    /// landed row re-tested against the growing module.
    #[test]
    fn makevertexmap_points_every_vertex_at_an_org_match_after_the_landing() {
        let b = Behavior {
            poly: true,
            usesegments: true,
            ..Behavior::default_form()
        };
        let mut m = Mesh::new(&b, 4, 2, 0, 0).expect("mesh init");
        for (x, y) in [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)] {
            let v = m.vx_alloc().expect("vertex alloc");
            m.set_vx_x(v, x);
            m.set_vx_y(v, y);
        }
        assert_eq!(delaunay(&mut m, &b).expect("delaunay"), 4);
        makevertexmap(&mut m, &b);
        for v in 1..=4u32 {
            let word = m.vx_2tri(v);
            let tri = decode(word);
            assert_eq!(m.org(tri), v);
        }
    }
}
