//! Mesh transformation: insertsubseg, flip, unflip, insertvertex,
//! triangulatepolygon, deletevertex, undovertex (triangle.c:2558-3810, the
//! "Mesh transformation routines" section). Assignment: the seven transform
//! rows of docs/function-mapping.md. The C SELF_CHECK diagnostic blocks of
//! flip/unflip/insertvertex/deletevertex are compiled out of the pinned
//! oracle build (triangle_config.h:62 leaves SELF_CHECK undefined, see
//! tools/oracle/build-oracle.sh) and are not part of the transliterated
//! control flow; deletevertex's two validity guards (boundary walk /
//! degree < 3) are mirrored as debug-only asserts because C's compiled-out
//! variants exit via internalerror() on the same impossible states.
//!
//! Restored call sites (the #28 restoration, landed with quality.rs): the
//! encroachment and quality checks the oracle build compiles are
//!   - insertvertex's checkseg4encroach + inlens (triangle.c:3302-3307)
//!     and its triflaws testtriangle (triangle.c:3468-3473),
//!   - triangulatepolygon's testtriangle of the committed triangle
//!     (triangle.c:3608-3617),
//!   - deletevertex's testtriangle of the relocated origin
//!     (triangle.c:3705-3707).
//!
//! The C runtime behavior of these paths equals the CDT_ONLY build with
//! segmentflaws/triflaws off; with the flags on it is the full C quality
//! flow, bad-triangle queues included.
//!
//! C vertex pointers mirror to vertex row ids (D1); pool storage is touched
//! only through slots.rs; OOM routes through the landed pool bridge.
//!
//! Stage-1 note: consumers are delaunay.rs/construct.rs/refine.rs/quality.rs
//! and the api.rs entry points; api.rs wiring and quality.rs have landed,
//! the remaining ioarrays/refine consumers carry their own notes; the
//! blanket below silences ahead-of-use dead_code until they land.
#![allow(dead_code)]

use crate::handles::{decode, encode, sencode, Osub, Otri, DUMMY_SUB, DUMMY_TRI, NULL};
use crate::locate::{locate, preciselocate, LocateResult};
use crate::mesh::Mesh;
use crate::options::Behavior;
use crate::pool::Oom;
use crate::predicates::{tri_counterclockwise, tri_incircle};
use crate::quality::{checkseg4encroach, inlens, testtriangle};
use crate::util::{interpolate, vertex_coords};

/// C: enum insertvertexresult (triangle.h:105-106): SUCCESSFULVERTEX,
/// ENCROACHINGVERTEX, VIOLATINGVERTEX, DUPLICATEVERTEX - the exact C
/// discriminants (pinned by test below). Owned by this module per the
/// mapping's enumeration rule: it is the return type of insertvertex. All
/// four C names end in VERTEX; the common-postfix lint is C fidelity here.
#[allow(clippy::enum_variant_names)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(i32)]
pub(crate) enum InsertVertexResult {
    /// C SUCCESSFULVERTEX.
    SuccessfulVertex = 0,
    /// C ENCROACHINGVERTEX (unreachable until the quality.rs rows land;
    /// see the module doc).
    EncroachingVertex = 1,
    /// C VIOLATINGVERTEX.
    ViolatingVertex = 2,
    /// C DUPLICATEVERTEX.
    DuplicateVertex = 3,
}

/// The port mirror of C's entry marker in the flipstacker chain: insertvertex
/// stores "(struct flipstacker *) &insertvertex" (the function's own code
/// address, triangle.c:3163) as prevflip to mark the end-of-list of an
/// edge-split insertion; undovertex compares against it (triangle.c:3770).
/// C uses an address no row can alias; under D4 prevflip carries row ids,
/// and the 32-bit id space's top value can never be handed out by the
/// flipstackers pool.
pub(crate) const INSERTVERTEX_MARKER: u32 = u32::MAX;

/// C: insertsubseg (triangle.c:2573-2601). Create a new subsegment and
/// insert it between two triangles. The new subsegment is inserted at the
/// edge described by the handle 'tri'. Its vertices are properly
/// initialized. The marker 'subsegmark' is applied to the subsegment and, if
/// appropriate, its vertices. (C passes behavior* b; the body reads mesh
/// storage only.)
pub(crate) fn insertsubseg(
    m: &mut Mesh,
    _b: &Behavior,
    tri: Otri,
    subsegmark: i32,
) -> Result<(), Oom> {
    let triorg = m.org(tri);
    let tridest = m.dest(tri);
    // Mark vertices if possible.
    if m.vx_mark(triorg) == 0 {
        m.vx_set_mark(triorg, subsegmark);
    }
    if m.vx_mark(tridest) == 0 {
        m.vx_set_mark(tridest, subsegmark);
    }
    // Check if there's already a subsegment here.
    let mut newsubseg = m.tspivot(tri);
    if newsubseg.ss == DUMMY_SUB {
        // Make new subsegment and initialize its vertices.
        newsubseg = m.makesubseg()?;
        m.setsorg(newsubseg, tridest);
        m.setsdest(newsubseg, triorg);
        m.setsegorg(newsubseg, tridest);
        m.setsegdest(newsubseg, triorg);
        // Bond new subsegment to the two triangles it is sandwiched between.
        //   Note that the facing triangle 'oppotri' might be equal to
        //   'dummytri' (outer space), but the new subsegment is bonded to it
        //   all the same.
        m.tsbond(tri, newsubseg);
        let oppotri = m.sym(tri);
        newsubseg.ssymself();
        m.tsbond(oppotri, newsubseg);
        m.set_sub_mark(newsubseg.ss, subsegmark);
    } else {
        if m.sub_mark(newsubseg.ss) == 0 {
            m.set_sub_mark(newsubseg.ss, subsegmark);
        }
    }
    Ok(())
}

/// C: flip (triangle.c:2664-2746). Transform two triangles to two different
/// triangles by flipping an edge counterclockwise within a quadrilateral.
/// The triangles abc and bad are deleted and replaced by the triangles cdb
/// and dca. The triangles that represent abc and bad are NOT deallocated;
/// they are reused for dca and cdb, respectively. Hence, any handles that
/// may have held the original triangles are still valid, although not
/// directed as they were before. WARNING (C comment): this transformation
/// is geometrically valid only if the quadrilateral adbc is convex, and
/// only if there is no subsegment between the triangles abc and bad; the
/// routine does not check either precondition. (C passes behavior* b; the
/// body reads mesh storage only.)
pub(crate) fn flip(m: &mut Mesh, _b: &Behavior, flipedge: Otri) {
    // Identify the vertices of the quadrilateral.
    let rightvertex = m.org(flipedge);
    let leftvertex = m.dest(flipedge);
    let botvertex = m.apex(flipedge);
    let top = m.sym(flipedge);
    let farvertex = m.apex(top);

    // Identify the casing of the quadrilateral.
    let topleft = top.lprev();
    let toplcasing = m.sym(topleft);
    let topright = top.lnext();
    let toprcasing = m.sym(topright);
    let botleft = flipedge.lnext();
    let botlcasing = m.sym(botleft);
    let botright = flipedge.lprev();
    let botrcasing = m.sym(botright);
    // Rotate the quadrilateral one-quarter turn counterclockwise.
    m.bond(topleft, botlcasing);
    m.bond(botleft, botrcasing);
    m.bond(botright, toprcasing);
    m.bond(topright, toplcasing);

    if m.checksegments {
        // Check for subsegments and rebond them to the quadrilateral.
        let toplsubseg = m.tspivot(topleft);
        let botlsubseg = m.tspivot(botleft);
        let botrsubseg = m.tspivot(botright);
        let toprsubseg = m.tspivot(topright);
        if toplsubseg.ss == DUMMY_SUB {
            m.tsdissolve(topright);
        } else {
            m.tsbond(topright, toplsubseg);
        }
        if botlsubseg.ss == DUMMY_SUB {
            m.tsdissolve(topleft);
        } else {
            m.tsbond(topleft, botlsubseg);
        }
        if botrsubseg.ss == DUMMY_SUB {
            m.tsdissolve(botleft);
        } else {
            m.tsbond(botleft, botrsubseg);
        }
        if toprsubseg.ss == DUMMY_SUB {
            m.tsdissolve(botright);
        } else {
            m.tsbond(botright, toprsubseg);
        }
    }

    // New vertex assignments for the rotated quadrilateral.
    m.setorg(flipedge, farvertex);
    m.setdest(flipedge, botvertex);
    m.setapex(flipedge, rightvertex);
    m.setorg(top, botvertex);
    m.setdest(top, farvertex);
    m.setapex(top, leftvertex);
}

/// C: unflip (triangle.c:2785-2862). Transform two triangles to two
/// different triangles by flipping an edge clockwise within a quadrilateral.
/// Reverses the flip() operation so that the data structures representing
/// the triangles are back where they were before the flip(). WARNING (C
/// comment): valid only for a convex quadrilateral with no subsegment on
/// the shared edge; the routine does not check. (C passes behavior* b; the
/// body reads mesh storage only.)
pub(crate) fn unflip(m: &mut Mesh, _b: &Behavior, flipedge: Otri) {
    // Identify the vertices of the quadrilateral.
    let rightvertex = m.org(flipedge);
    let leftvertex = m.dest(flipedge);
    let botvertex = m.apex(flipedge);
    let top = m.sym(flipedge);
    let farvertex = m.apex(top);

    // Identify the casing of the quadrilateral.
    let topleft = top.lprev();
    let toplcasing = m.sym(topleft);
    let topright = top.lnext();
    let toprcasing = m.sym(topright);
    let botleft = flipedge.lnext();
    let botlcasing = m.sym(botleft);
    let botright = flipedge.lprev();
    let botrcasing = m.sym(botright);
    // Rotate the quadrilateral one-quarter turn clockwise.
    m.bond(topleft, toprcasing);
    m.bond(botleft, toplcasing);
    m.bond(botright, botlcasing);
    m.bond(topright, botrcasing);

    if m.checksegments {
        // Check for subsegments and rebond them to the quadrilateral.
        let toplsubseg = m.tspivot(topleft);
        let botlsubseg = m.tspivot(botleft);
        let botrsubseg = m.tspivot(botright);
        let toprsubseg = m.tspivot(topright);
        if toplsubseg.ss == DUMMY_SUB {
            m.tsdissolve(botleft);
        } else {
            m.tsbond(botleft, toplsubseg);
        }
        if botlsubseg.ss == DUMMY_SUB {
            m.tsdissolve(botright);
        } else {
            m.tsbond(botright, botlsubseg);
        }
        if botrsubseg.ss == DUMMY_SUB {
            m.tsdissolve(topright);
        } else {
            m.tsbond(topright, botrsubseg);
        }
        if toprsubseg.ss == DUMMY_SUB {
            m.tsdissolve(topleft);
        } else {
            m.tsbond(topleft, toprsubseg);
        }
    }

    // New vertex assignments for the rotated quadrilateral.
    m.setorg(flipedge, botvertex);
    m.setdest(flipedge, farvertex);
    m.setapex(flipedge, leftvertex);
    m.setorg(top, farvertex);
    m.setdest(top, botvertex);
    m.setapex(top, rightvertex);
}

/// C: insertvertex (triangle.c:2920-3465). Insert a vertex into a Delaunay
/// triangulation, performing flips as necessary to maintain the Delaunay
/// property. The point 'newvertex' is located (through 'searchtri' when the
/// caller provides a live starting handle, through locate() otherwise; a
/// 'searchtri' on the sentinel row is the C searchtri.tri == dummytri, "no
/// starting triangle"). The vertex is found on an existing vertex
/// (DUPLICATEVERTEX, 'searchtri' then has it as origin), on a subsegment
/// (VIOLATINGVERTEX, 'searchtri' then has it as primary edge; possibly
/// enqueued for encroachment when 'segmentflaws' is set), or inserted:
/// inside a triangle (split into three) or on an edge (the two adjacent
/// triangles split into four, the boundary case splits one into two and
/// grows the hull), then edge flips restore the Delaunay property.
/// 'splitseg' (Some) is the subsegment the caller wants split; C requires
/// 'searchtri' to be the triangle reached by pivoting from it, and no point
/// location is done. The return value is SUCCESSFULVERTEX, or
/// ENCROACHINGVERTEX when a checked subsegment is encroached (unreachable
/// until quality.rs lands; see the module doc). On return 'searchtri' has
/// the new (or existing) vertex as origin. insertvertex() does not use
/// flip() for reasons of speed; some information can be reused from edge
/// flip to edge flip (C comments preserved in the body). The C five-value
/// 'attribs > 0' collapses to the bool 'attribs' for the nonzero value the
/// callers pass today.
#[allow(clippy::too_many_arguments)] // the C signature carries eight parameters
pub(crate) fn insertvertex(
    m: &mut Mesh,
    b: &Behavior,
    newvertex: u32,
    searchtri: &mut Otri,
    splitseg: Option<&mut Osub>,
    segmentflaws: bool,
    triflaws: bool,
    attribs: bool,
) -> Result<InsertVertexResult, Oom> {
    // The search point as locate and the predicates see it (C: the REAL*
    // newvertex pointer; D1 copies the coordinates through the chokepoint).
    let newvertex_pt = (m.vx_real(newvertex, 0), m.vx_real(newvertex, 1));
    let newvertex_v = [newvertex_pt.0, newvertex_pt.1, 0.0];

    let mut horiz: Otri;
    let intersect: LocateResult;
    if splitseg.is_none() {
        // Find the location of the vertex to be inserted.  Check if a good
        //   starting triangle has already been provided by the caller.
        if searchtri.tri == DUMMY_TRI {
            // Find a boundary triangle.
            let mut boundary = Otri {
                tri: DUMMY_TRI,
                orient: 0,
            };
            m.symself(&mut boundary);
            // Search for a triangle containing 'newvertex'.
            horiz = boundary;
            intersect = locate(m, b, newvertex_pt, &mut horiz);
        } else {
            // Start searching from the triangle provided by the caller.
            horiz = *searchtri;
            intersect = preciselocate(m, b, newvertex_pt, &mut horiz, true);
        }
    } else {
        // The calling routine provides the subsegment in which
        //   the vertex is inserted.
        horiz = *searchtri;
        intersect = LocateResult::OnEdge;
    }

    if intersect == LocateResult::OnVertex {
        // There's already a vertex there.  Return in 'searchtri' a triangle
        //   whose origin is the existing vertex.
        *searchtri = horiz;
        m.recenttri = Some(horiz);
        return Ok(InsertVertexResult::DuplicateVertex);
    }
    if intersect == LocateResult::OnEdge || intersect == LocateResult::Outside {
        // The vertex falls on an edge or boundary.
        if m.checksegments && splitseg.is_none() {
            // Check whether the vertex falls on a subsegment.
            let brokensubseg = m.tspivot(horiz);
            if brokensubseg.ss != DUMMY_SUB {
                // The vertex falls on a subsegment, and hence will not be inserted.
                if segmentflaws {
                    let mut enq = b.nobisect != 2;
                    if enq && b.nobisect == 1 {
                        // This subsegment may be split only if it is an
                        //   internal boundary.
                        let testtri = m.sym(horiz);
                        enq = testtri.tri != DUMMY_TRI;
                    }
                    if enq {
                        // Add the subsegment to the list of encroached subsegments.
                        let encroached = m.bad_subseg_alloc()?;
                        m.set_bad_sub_seg_enc(encroached, sencode(brokensubseg));
                        m.set_bad_sub_seg_org(encroached, m.sorg(brokensubseg));
                        m.set_bad_sub_seg_dest(encroached, m.sdest(brokensubseg));
                    }
                }
                // Return a handle whose primary edge contains the vertex,
                //   which has not been inserted.
                *searchtri = horiz;
                m.recenttri = Some(horiz);
                return Ok(InsertVertexResult::ViolatingVertex);
            }
        }

        // Insert the vertex on an edge, dividing one triangle into two (if
        //   the edge lies on a boundary) or two triangles into four.
        let botright = horiz.lprev();
        let botrcasing = m.sym(botright);
        let mut topright = m.sym(horiz);
        // Is there a second triangle?  (Or does this edge lie on a boundary?)
        let mirrorflag = topright.tri != DUMMY_TRI;
        let mut newtopright: Option<Otri> = None;
        let toprcasing;
        if mirrorflag {
            topright.lnextself();
            toprcasing = m.sym(topright);
            newtopright = Some(m.maketriangle(b.usesegments, b.vararea)?);
        } else {
            // Splitting a boundary edge increases the number of boundary edges.
            m.hullsize += 1;
            // (C leaves toprcasing uninitialized here; the value is never read
            //   on the boundary path. The sentinel row is the deterministic
            //   placeholder.)
            toprcasing = Otri {
                tri: DUMMY_TRI,
                orient: 0,
            };
        }
        let mut newbotright = m.maketriangle(b.usesegments, b.vararea)?;

        // Set the vertices of changed and new triangles.
        let rightvertex = m.org(horiz);
        let leftvertex = m.dest(horiz);
        let botvertex = m.apex(horiz);
        m.setorg(newbotright, botvertex);
        m.setdest(newbotright, rightvertex);
        m.setapex(newbotright, newvertex);
        m.setorg(horiz, newvertex);

        // Interpolate attributes of new vertex.
        if attribs && m.nextras > 0 {
            // TODO: vertex attribute interpolation
            //   The new vertex lies on the segment (rightvertex, leftvertex),
            //   so we could use line interpolation.
            interpolate(m, newvertex, rightvertex, leftvertex, botvertex, m.nextras);
        }

        for i in 0..m.eextras {
            // Set the element attributes of a new triangle.
            m.set_tri_attribute(
                newbotright.tri,
                i as usize,
                m.tri_attribute(botright.tri, i as usize),
            );
        }
        if b.vararea {
            // Set the area constraint of a new triangle.
            m.set_tri_areabound(newbotright.tri, m.tri_areabound(botright.tri));
        }
        if mirrorflag {
            let newtopright = newtopright.expect("mirror set newtopright");
            let topvertex = m.dest(topright);
            m.setorg(newtopright, rightvertex);
            m.setdest(newtopright, topvertex);
            m.setapex(newtopright, newvertex);
            m.setorg(topright, newvertex);
            for i in 0..m.eextras {
                // Set the element attributes of another new triangle.
                m.set_tri_attribute(
                    newtopright.tri,
                    i as usize,
                    m.tri_attribute(topright.tri, i as usize),
                );
            }
            if b.vararea {
                // Set the area constraint of another new triangle.
                m.set_tri_areabound(newtopright.tri, m.tri_areabound(topright.tri));
            }
        }

        // There may be subsegments that need to be bonded
        //   to the new triangle(s).
        if m.checksegments {
            let botrsubseg = m.tspivot(botright);
            if botrsubseg.ss != DUMMY_SUB {
                m.tsdissolve(botright);
                m.tsbond(newbotright, botrsubseg);
            }
            if mirrorflag {
                let toprsubseg = m.tspivot(topright);
                if toprsubseg.ss != DUMMY_SUB {
                    m.tsdissolve(topright);
                    m.tsbond(newtopright.expect("mirror set newtopright"), toprsubseg);
                }
            }
        }

        // Bond the new triangle(s) to the surrounding triangles.
        m.bond(newbotright, botrcasing);
        newbotright.lprevself();
        m.bond(newbotright, botright);
        newbotright.lprevself();
        if mirrorflag {
            let mut newtopright = newtopright.expect("mirror set newtopright");
            m.bond(newtopright, toprcasing);
            newtopright.lnextself();
            m.bond(newtopright, topright);
            newtopright.lnextself();
            m.bond(newtopright, newbotright);
        }

        if let Some(splitseg) = splitseg {
            // Split the subsegment into two.
            m.setsdest(*splitseg, newvertex);
            let segmentorg = m.segorg(*splitseg);
            let segmentdest = m.segdest(*splitseg);
            splitseg.ssymself();
            let rightsubseg = m.spivot(*splitseg);
            let bndmark = m.sub_mark(splitseg.ss);
            insertsubseg(m, b, newbotright, bndmark)?;
            let mut newsubseg = m.tspivot(newbotright);
            m.setsegorg(newsubseg, segmentorg);
            m.setsegdest(newsubseg, segmentdest);
            m.sbond(*splitseg, newsubseg);
            newsubseg.ssymself();
            m.sbond(newsubseg, rightsubseg);
            splitseg.ssymself();
            // Transfer the subsegment's boundary marker to the vertex
            //   if required.
            if m.vx_mark(newvertex) == 0 {
                m.vx_set_mark(newvertex, m.sub_mark(splitseg.ss));
            }
        }

        if m.checkquality {
            m.flip_restart();
            let row = m.flip_alloc()?;
            m.set_flip_flippedtri(row, encode(horiz));
            m.set_flip_prevflip(row, INSERTVERTEX_MARKER);
            m.lastflip = row;
        }

        // Position 'horiz' on the first edge to check for
        //   the Delaunay property.
        horiz.lnextself();
    } else {
        // Insert the vertex in a triangle, splitting it into three.
        let botleft = horiz.lnext();
        let botright = horiz.lprev();
        let botlcasing = m.sym(botleft);
        let botrcasing = m.sym(botright);
        let mut newbotleft = m.maketriangle(b.usesegments, b.vararea)?;
        let mut newbotright = m.maketriangle(b.usesegments, b.vararea)?;

        // Set the vertices of changed and new triangles.
        let rightvertex = m.org(horiz);
        let leftvertex = m.dest(horiz);
        let botvertex = m.apex(horiz);

        // Interpolate attributes of new vertex.
        if attribs && m.nextras > 0 {
            interpolate(m, newvertex, rightvertex, leftvertex, botvertex, m.nextras);
        }

        m.setorg(newbotleft, leftvertex);
        m.setdest(newbotleft, botvertex);
        m.setapex(newbotleft, newvertex);
        m.setorg(newbotright, botvertex);
        m.setdest(newbotright, rightvertex);
        m.setapex(newbotright, newvertex);
        m.setapex(horiz, newvertex);
        for i in 0..m.eextras {
            // Set the element attributes of the new triangles.
            let attrib = m.tri_attribute(horiz.tri, i as usize);
            m.set_tri_attribute(newbotleft.tri, i as usize, attrib);
            m.set_tri_attribute(newbotright.tri, i as usize, attrib);
        }
        if b.vararea {
            // Set the area constraint of the new triangles.
            let area = m.tri_areabound(horiz.tri);
            m.set_tri_areabound(newbotleft.tri, area);
            m.set_tri_areabound(newbotright.tri, area);
        }

        // There may be subsegments that need to be bonded
        //   to the new triangles.
        if m.checksegments {
            let botlsubseg = m.tspivot(botleft);
            if botlsubseg.ss != DUMMY_SUB {
                m.tsdissolve(botleft);
                m.tsbond(newbotleft, botlsubseg);
            }
            let botrsubseg = m.tspivot(botright);
            if botrsubseg.ss != DUMMY_SUB {
                m.tsdissolve(botright);
                m.tsbond(newbotright, botrsubseg);
            }
        }

        // Bond the new triangles to the surrounding triangles.
        m.bond(newbotleft, botlcasing);
        m.bond(newbotright, botrcasing);
        newbotleft.lnextself();
        newbotright.lprevself();
        m.bond(newbotleft, newbotright);
        newbotleft.lnextself();
        m.bond(botleft, newbotleft);
        newbotright.lprevself();
        m.bond(botright, newbotright);

        if m.checkquality {
            m.flip_restart();
            let row = m.flip_alloc()?;
            m.set_flip_flippedtri(row, encode(horiz));
            m.set_flip_prevflip(row, NULL);
            m.lastflip = row;
        }
    }

    // The insertion is successful by default, unless an encroached
    //   subsegment is found (C's 'success' can only become
    //   ENCROACHINGVERTEX at the subsegment check of the flip loop).
    let mut success = InsertVertexResult::SuccessfulVertex;
    // Circle around the newly inserted vertex, checking each edge opposite
    //   it for the Delaunay property.  Non-Delaunay edges are flipped.
    //   'horiz' is always the edge being checked.  'first' marks where to
    //   stop circling.
    let first = m.org(horiz);
    let mut rightvertex = first;
    let mut leftvertex = m.dest(horiz);
    // Circle until finished.  Each pass flips an edge, which adds a triangle
    //   around the new vertex, or moves on to the next edge, so more passes
    //   than twice the triangles in the mesh means the flips are going in
    //   circles.  Input whose segments cross can leave the mesh that way.
    //   The vertex is in the mesh either way; the caller is told to stop.
    let mut circles: i64 = 0;
    loop {
        circles += 1;
        if circles > 2 * m.triangles.live_count() as i64 {
            m.cyclefail = true;
            return Ok(InsertVertexResult::SuccessfulVertex);
        }
        // By default, the edge will be flipped.
        let mut doflip = true;

        if m.checksegments {
            // Check for a subsegment, which cannot be flipped.
            let checksubseg = m.tspivot(horiz);
            if checksubseg.ss != DUMMY_SUB {
                // The edge is a subsegment and cannot be flipped.
                doflip = false;
                if segmentflaws {
                    // Does the new vertex encroach upon this subsegment?
                    //   A vertex on the far side may encroach upon it
                    //   too; that one was there before, and is left for
                    //   the subsegment queue.
                    let subsegorg = m.sorg(checksubseg);
                    let subsegdest = m.sdest(checksubseg);
                    if checkseg4encroach(m, b, checksubseg)? != 0
                        && inlens(
                            b,
                            vertex_coords(m, subsegorg),
                            vertex_coords(m, subsegdest),
                            newvertex_v,
                        )
                    {
                        success = InsertVertexResult::EncroachingVertex;
                    }
                }
            }
        }

        if doflip {
            // Check if the edge is a boundary edge.
            let top = m.sym(horiz);
            if top.tri == DUMMY_TRI {
                // The edge is a boundary edge and cannot be flipped.
                doflip = false;
            } else {
                // Find the vertex on the other side of the edge.
                let farvertex = m.apex(top);
                // In the incremental Delaunay triangulation algorithm, any of
                //   'leftvertex', 'rightvertex', and 'farvertex' could be vertices
                //   of the triangular bounding box.  These vertices must be
                //   treated as if they are infinitely distant, even though their
                //   "coordinates" are not.
                if leftvertex == m.infvertex1
                    || leftvertex == m.infvertex2
                    || leftvertex == m.infvertex3
                {
                    // 'leftvertex' is infinitely distant.  Check the convexity of
                    //   the boundary of the triangulation.  'farvertex' might be
                    //   infinite as well, but trust me, this same condition should
                    //   be applied.
                    doflip = tri_counterclockwise(
                        m,
                        b,
                        newvertex_v,
                        vertex_coords(m, rightvertex),
                        vertex_coords(m, farvertex),
                    ) > 0.0;
                } else if rightvertex == m.infvertex1
                    || rightvertex == m.infvertex2
                    || rightvertex == m.infvertex3
                {
                    // 'rightvertex' is infinitely distant.  Check the convexity of
                    //   the boundary of the triangulation.  'farvertex' might be
                    //   infinite as well, but trust me, this same condition should
                    //   be applied.
                    doflip = tri_counterclockwise(
                        m,
                        b,
                        vertex_coords(m, farvertex),
                        vertex_coords(m, leftvertex),
                        newvertex_v,
                    ) > 0.0;
                } else if farvertex == m.infvertex1
                    || farvertex == m.infvertex2
                    || farvertex == m.infvertex3
                {
                    // 'farvertex' is infinitely distant and cannot be inside
                    //   the circumcircle of the triangle 'horiz'.
                    doflip = false;
                } else {
                    // Test whether the edge is locally Delaunay.
                    doflip = tri_incircle(
                        m,
                        b,
                        vertex_coords(m, leftvertex),
                        newvertex_v,
                        vertex_coords(m, rightvertex),
                        vertex_coords(m, farvertex),
                    ) > 0.0;
                }
                if doflip {
                    // We made it!  Flip the edge 'horiz' by rotating its containing
                    //   quadrilateral (the two triangles adjacent to 'horiz').
                    // Identify the casing of the quadrilateral.
                    let topleft = top.lprev();
                    let toplcasing = m.sym(topleft);
                    let topright = top.lnext();
                    let toprcasing = m.sym(topright);
                    let botleft = horiz.lnext();
                    let botlcasing = m.sym(botleft);
                    let botright = horiz.lprev();
                    let botrcasing = m.sym(botright);
                    // Rotate the quadrilateral one-quarter turn counterclockwise.
                    m.bond(topleft, botlcasing);
                    m.bond(botleft, botrcasing);
                    m.bond(botright, toprcasing);
                    m.bond(topright, toplcasing);
                    if m.checksegments {
                        // Check for subsegments and rebond them to the quadrilateral.
                        let toplsubseg = m.tspivot(topleft);
                        let botlsubseg = m.tspivot(botleft);
                        let botrsubseg = m.tspivot(botright);
                        let toprsubseg = m.tspivot(topright);
                        if toplsubseg.ss == DUMMY_SUB {
                            m.tsdissolve(topright);
                        } else {
                            m.tsbond(topright, toplsubseg);
                        }
                        if botlsubseg.ss == DUMMY_SUB {
                            m.tsdissolve(topleft);
                        } else {
                            m.tsbond(topleft, botlsubseg);
                        }
                        if botrsubseg.ss == DUMMY_SUB {
                            m.tsdissolve(botleft);
                        } else {
                            m.tsbond(botleft, botrsubseg);
                        }
                        if toprsubseg.ss == DUMMY_SUB {
                            m.tsdissolve(botright);
                        } else {
                            m.tsbond(botright, toprsubseg);
                        }
                    }
                    // New vertex assignments for the rotated quadrilateral.
                    m.setorg(horiz, farvertex);
                    m.setdest(horiz, newvertex);
                    m.setapex(horiz, rightvertex);
                    m.setorg(top, newvertex);
                    m.setdest(top, farvertex);
                    m.setapex(top, leftvertex);
                    for i in 0..m.eextras {
                        // Take the average of the two triangles' attributes.
                        let attrib = 0.5
                            * (m.tri_attribute(top.tri, i as usize)
                                + m.tri_attribute(horiz.tri, i as usize));
                        m.set_tri_attribute(top.tri, i as usize, attrib);
                        m.set_tri_attribute(horiz.tri, i as usize, attrib);
                    }
                    if b.vararea {
                        // Take the average of the two triangles' area constraints.
                        //   This prevents small area constraints from migrating a
                        //   long, long way from their original location due to flips.
                        let area = if m.tri_areabound(top.tri) <= 0.0
                            || m.tri_areabound(horiz.tri) <= 0.0
                        {
                            -1.0
                        } else {
                            0.5 * (m.tri_areabound(top.tri) + m.tri_areabound(horiz.tri))
                        };
                        m.set_tri_areabound(top.tri, area);
                        m.set_tri_areabound(horiz.tri, area);
                    }

                    if m.checkquality {
                        let newflip = m.flip_alloc()?;
                        m.set_flip_flippedtri(newflip, encode(horiz));
                        m.set_flip_prevflip(newflip, m.lastflip);
                        m.lastflip = newflip;
                    }

                    // On the next iterations, consider the two edges that were
                    //   exposed (this is, are now visible to the newly inserted
                    //   vertex) by the edge flip.
                    horiz.lprevself();
                    leftvertex = farvertex;
                }
            }
        }
        if !doflip {
            // The handle 'horiz' is accepted as locally Delaunay.
            if triflaws {
                // Check the triangle 'horiz' for quality.
                testtriangle(m, b, horiz)?;
            }
            // Look for the next edge around the newly inserted vertex.
            horiz.lnextself();
            let testtri = m.sym(horiz);
            // Check for finishing a complete revolution about the new vertex, or
            //   falling outside  of the triangulation.  The latter will happen
            //   when a vertex is inserted at a boundary.
            if leftvertex == first || testtri.tri == DUMMY_TRI {
                // We're done.  Return a triangle whose origin is the new vertex.
                let l = horiz.lnext();
                *searchtri = l;
                m.recenttri = Some(l);
                return Ok(success);
            }
            // Finish finding the next edge around the newly inserted vertex.
            horiz = testtri.lnext();
            rightvertex = leftvertex;
            leftvertex = m.dest(horiz);
        }
    }
}

/// C: triangulatepolygon (triangle.c:3559-3612). Find the Delaunay
/// triangulation of a polygon that has a certain "nice" shape. This
/// includes the polygons that result from deletion of a vertex or insertion
/// of a segment. The 'edgecount' parameter is the number of sides of the
/// polygon, including its base. 'doflip' is set if the final flip will be
/// performed, resulting in a fan of one triangle; if not, the final flip is
/// not performed, resulting in a fan of two triangles. 'triflaws' is a
/// flag that determines whether the new triangles should be tested for
/// quality. On completion 'lastedge' is the last remaining triangle, or
/// the leftmost of the last two. (C comments preserved in the body.)
///
/// Returns Oom exactly when a triflaws quality test reaches the
/// bad-triangle pool bridge (C cannot fail there).
pub(crate) fn triangulatepolygon(
    m: &mut Mesh,
    b: &Behavior,
    firstedge: &Otri,
    lastedge: &mut Otri,
    edgecount: i32,
    doflip: bool,
    triflaws: bool,
) -> Result<(), Oom> {
    // Identify the base vertices.
    let leftbasevertex = m.apex(*lastedge);
    let rightbasevertex = m.dest(*firstedge);

    // Find the best vertex to connect the base to.
    let mut besttri = m.onext(*firstedge);
    let mut bestvertex = m.dest(besttri);
    let mut testtri = besttri;
    let mut bestnumber: i32 = 1;
    for i in 2..=edgecount - 2 {
        m.onextself(&mut testtri);
        let testvertex = m.dest(testtri);
        // Is this a better vertex?
        if tri_incircle(
            m,
            b,
            vertex_coords(m, leftbasevertex),
            vertex_coords(m, rightbasevertex),
            vertex_coords(m, bestvertex),
            vertex_coords(m, testvertex),
        ) > 0.0
        {
            besttri = testtri;
            bestvertex = testvertex;
            bestnumber = i;
        }
    }
    if bestnumber > 1 {
        // Recursively triangulate the smaller polygon on the right.
        let mut tempedge = m.oprev(besttri);
        triangulatepolygon(
            m,
            b,
            firstedge,
            &mut tempedge,
            bestnumber + 1,
            true,
            triflaws,
        )?;
    }
    if bestnumber < edgecount - 2 {
        // Recursively triangulate the smaller polygon on the left.
        let tempedge = m.sym(besttri);
        triangulatepolygon(
            m,
            b,
            &besttri,
            lastedge,
            edgecount - bestnumber,
            true,
            triflaws,
        )?;
        // Find 'besttri' again; it may have been lost to edge flips.
        besttri = m.sym(tempedge);
    }
    if doflip {
        // Do one final edge flip.
        flip(m, b, besttri);
        if triflaws {
            // Check the quality of the newly committed triangle.
            let testtri = m.sym(besttri);
            testtriangle(m, b, testtri)?;
        }
    }
    // Return the base triangle.
    *lastedge = besttri;
    Ok(())
}

/// C: deletevertex (triangle.c:3638-3702). Delete a vertex from a Delaunay
/// triangulation, ensuring that the triangulation remains Delaunay. The
/// origin of 'deltri' is deleted. The union of the triangles adjacent to
/// this vertex is a polygon, for which the Delaunay triangulation is found.
/// Two triangles are removed from the mesh. Only interior vertices that do
/// not lie on a subsegment are candidates for deletion.
///
/// Returns Oom exactly when a quality test reaches the bad-triangle pool
/// bridge (C cannot fail there).
///
/// The quality flank (the triflaws triangulatepolygon argument, the
/// post-relocation testtriangle) is the #28 restoration.
pub(crate) fn deletevertex(m: &mut Mesh, b: &Behavior, deltri: &mut Otri) -> Result<(), Oom> {
    let delvertex = m.org(*deltri);
    // C: vertexdealloc (triangle.c:1262-1271).
    m.vx_dealloc(delvertex);

    // Count the degree of the vertex being deleted.
    let mut countingtri = m.onext(*deltri);
    let mut edgecount: i32 = 1;
    while *deltri != countingtri {
        // C SELF_CHECK (compiled out of the oracle): the boundary-walk
        //   guard mirrors internalerror()'s impossible state only.
        debug_assert!(
            countingtri.tri != DUMMY_TRI,
            "attempt to delete a boundary vertex"
        );
        edgecount += 1;
        m.onextself(&mut countingtri);
    }
    // C SELF_CHECK (compiled out of the oracle): the degree guard mirrors
    //   internalerror()'s impossible state only.
    debug_assert!(edgecount >= 3, "vertex of degree < 3");

    if edgecount > 3 {
        // Triangulate the polygon defined by the union of all triangles
        //   adjacent to the vertex being deleted.  Check the quality of
        //   the resulting triangles.
        let firstedge = m.onext(*deltri);
        let mut lastedge = m.oprev(*deltri);
        triangulatepolygon(
            m,
            b,
            &firstedge,
            &mut lastedge,
            edgecount,
            false,
            b.nobisect == 0,
        )?;
    }
    // Splice out two triangles.
    let deltriright = (*deltri).lprev();
    let lefttri = m.dnext(*deltri);
    let leftcasing = m.sym(lefttri);
    let righttri = m.oprev(deltriright);
    let rightcasing = m.sym(righttri);
    m.bond(*deltri, leftcasing);
    m.bond(deltriright, rightcasing);
    let leftsubseg = m.tspivot(lefttri);
    if leftsubseg.ss != DUMMY_SUB {
        m.tsbond(*deltri, leftsubseg);
    }
    let rightsubseg = m.tspivot(righttri);
    if rightsubseg.ss != DUMMY_SUB {
        m.tsbond(deltriright, rightsubseg);
    }

    // Set the new origin of 'deltri' and check its quality.
    let neworg = m.org(lefttri);
    m.setorg(*deltri, neworg);
    if b.nobisect == 0 {
        testtriangle(m, b, *deltri)?;
    }

    // Delete the two spliced-out triangles.
    // C: triangledealloc (triangle.c:1196-1202).
    m.tri_dealloc(lefttri.tri);
    m.tri_dealloc(righttri.tri);
    Ok(())
}

/// C: undovertex (triangle.c:3730-3818). Undo the most recent vertex
/// insertion. Walks through the list of transformations (flips and a
/// vertex insertion) in the reverse of the order in which they were done,
/// and undoes them. The inserted vertex is removed from the triangulation;
/// two triangles (possibly just one) are also deallocated. (The inserted
/// vertex's row itself stays allocated, exactly like the C body, which
/// leaves the vertex dealloc to its callers; C comments preserved.)
pub(crate) fn undovertex(m: &mut Mesh, b: &Behavior) {
    // Walk through the list of transformations (flips and a vertex insertion)
    //   in the reverse of the order in which they were done, and undo them.
    while m.lastflip != NULL {
        // Find a triangle involved in the last unreversed transformation.
        let cur = m.lastflip;
        let mut fliptri = decode(m.flip_flippedtri(cur));

        // We are reversing one of three transformations:  a trisection of one
        //   triangle into three (by inserting a vertex in the triangle), a
        //   bisection of two triangles into four (by inserting a vertex in an
        //   edge), or an edge flip.
        if m.flip_prevflip(cur) == NULL {
            // Restore a triangle that was split into three triangles,
            //   so it is again one triangle.
            let mut botleft = m.dprev(fliptri);
            botleft.lnextself();
            let mut botright = m.onext(fliptri);
            botright.lprevself();
            let botlcasing = m.sym(botleft);
            let botrcasing = m.sym(botright);
            let botvertex = m.dest(botleft);

            m.setapex(fliptri, botvertex);
            fliptri.lnextself();
            m.bond(fliptri, botlcasing);
            let botlsubseg = m.tspivot(botleft);
            m.tsbond(fliptri, botlsubseg);
            fliptri.lnextself();
            m.bond(fliptri, botrcasing);
            let botrsubseg = m.tspivot(botright);
            m.tsbond(fliptri, botrsubseg);

            // Delete the two spliced-out triangles.
            // C: triangledealloc (triangle.c:1196-1202).
            m.tri_dealloc(botleft.tri);
            m.tri_dealloc(botright.tri);
        } else if m.flip_prevflip(cur) == INSERTVERTEX_MARKER {
            // Restore two triangles that were split into four triangles,
            //   so they are again two triangles.
            let gluetri = fliptri.lprev();
            let mut botright = m.sym(gluetri);
            botright.lnextself();
            let botrcasing = m.sym(botright);
            let rightvertex = m.dest(botright);

            m.setorg(fliptri, rightvertex);
            m.bond(gluetri, botrcasing);
            let botrsubseg = m.tspivot(botright);
            m.tsbond(gluetri, botrsubseg);

            // Delete the spliced-out triangle.
            // C: triangledealloc (triangle.c:1196-1202).
            m.tri_dealloc(botright.tri);

            let mut gluetri = m.sym(fliptri);
            if gluetri.tri != DUMMY_TRI {
                gluetri.lnextself();
                let topright = m.dnext(gluetri);
                let toprcasing = m.sym(topright);

                m.setorg(gluetri, rightvertex);
                m.bond(gluetri, toprcasing);
                let toprsubseg = m.tspivot(topright);
                m.tsbond(gluetri, toprsubseg);

                // Delete the spliced-out triangle.
                // C: triangledealloc (triangle.c:1196-1202).
                m.tri_dealloc(topright.tri);
            }

            // This is the end of the list, sneakily encoded.
            m.set_flip_prevflip(cur, NULL);
        } else {
            // Undo an edge flip.
            unflip(m, b, fliptri);
        }

        // Go on and process the next transformation.
        m.lastflip = m.flip_prevflip(cur);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::slots::DEADVERTEX;

    /// The Behavior for the test at hand, composed exactly like the
    /// locate.rs/util.rs test helpers (Behavior::default_form() plus the
    /// fields the case needs, mirroring the C tests' hand-filled structs).
    fn behavior(poly: bool) -> Behavior {
        Behavior {
            poly,
            usesegments: poly,
            ..Behavior::default_form()
        }
    }

    /// mesh_dim 2 like every stage-1 test; nextras/eextras per case.
    fn mesh(poly: bool, nextras: i32, eextras: i32) -> Mesh {
        Mesh::new(&behavior(poly), 16, 2, nextras, eextras).expect("mesh init")
    }

    /// A vertex row with real coordinates (the fan-builder pattern of
    /// locate.rs; vx ids start at 1, D1).
    fn vxy(m: &mut Mesh, x: f64, y: f64) -> u32 {
        let v = m.vx_alloc().expect("vertex alloc");
        m.vx_set_real(v, 0, x);
        m.vx_set_real(v, 1, y);
        v
    }

    /// Bit-level snapshot of the structural pools (triangles, subsegments,
    /// vertices): every mesh row's full word content up to the
    /// fresh-allocation frontier (the row-0/1 null and dummy rows are
    /// excluded, see below). The flipstackers/badsubsegs pools are
    /// deliberately excluded: undovertex's C contract restores the mesh
    /// structure and leaves the bookkeeping rows written until their pools
    /// restart (m->flipstackers.items stays at the post-insert count in C
    /// too).
    fn snap(m: &Mesh) -> Vec<Vec<Vec<u64>>> {
        // Mesh rows only: tri/subseg row 1 is the dummy row, which C's
        // bond() also writes through (the dummytribase mirror stays dirtied
        // after flip+unflip in C exactly like here), and vx row 0 is the
        // null row; neither is part of the tri-state claim.
        let starts = [2u32, 2, 1];
        let mut pools = vec![];
        for (pool, start) in [&m.triangles, &m.subsegs, &m.vertices]
            .into_iter()
            .zip(starts)
        {
            let frontier = pool.first_fresh_id() as u64 + pool.high_water();
            let mut rows = vec![];
            for id in start..frontier as u32 {
                let mut words = vec![];
                for slot in 0..pool.stride() as usize {
                    words.push(pool.word(id, slot));
                }
                rows.push(words);
            }
            pools.push(rows);
        }
        pools
    }

    /// The rows that existed at snapshot time are restored bit-for-bit (a
    /// prefix compare: rows deallocated by the undo's callers never existed
    /// at snapshot time; their dead-marker words lie beyond the prefix).
    /// The count claims are per-test, because undovertex leaves the inserted
    /// vertex row allocated exactly like the C body does.
    fn assert_rows_reverted(before: &[Vec<Vec<u64>>], m: &Mesh) {
        let after = snap(m);
        for (pool, bf) in after.iter().zip(before.iter()) {
            assert_eq!(
                &pool[..bf.len()],
                bf,
                "pool rows must be restored bit-for-bit"
            );
        }
    }

    /// A pair of triangles sharing the edge (a, b): t1 = (a, b, c) and
    /// t2 = (b, a, d), bonded at orient 0 like the fan builder of locate.rs.
    fn pair(m: &mut Mesh, usesegments: bool, a: u32, b: u32, c: u32, d: u32) -> (Otri, Otri) {
        let t1 = m.maketriangle(usesegments, false).expect("tri");
        m.setorg(t1, a);
        m.setdest(t1, b);
        m.setapex(t1, c);
        let t2 = m.maketriangle(usesegments, false).expect("tri");
        m.setorg(t2, b);
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
        (t1, t2)
    }

    #[test]
    fn insertvertexresult_discriminants_are_the_c_values() {
        assert_eq!(InsertVertexResult::SuccessfulVertex as i32, 0);
        assert_eq!(InsertVertexResult::EncroachingVertex as i32, 1);
        assert_eq!(InsertVertexResult::ViolatingVertex as i32, 2);
        assert_eq!(InsertVertexResult::DuplicateVertex as i32, 3);
    }

    #[test]
    fn flip_unflip_restore_the_mesh_bit_for_bit() {
        // A poly quad with a subsegment on a casing edge, so the
        // checksegments rebond block runs with a live subsegment.
        let mut m = mesh(true, 0, 0);
        m.checksegments = true;
        let b = behavior(true);
        let a = vxy(&mut m, 0.0, 0.0);
        let e = vxy(&mut m, 1.0, 0.0);
        let c = vxy(&mut m, 0.0, 1.0);
        let d = vxy(&mut m, 0.5, -1.0);
        let (t1, _t2) = pair(&mut m, true, a, e, c, d);
        // subsegment on the rim edge (e -> c) of t1: orient 1.
        let casing = Otri {
            tri: t1.tri,
            orient: 1,
        };
        insertsubseg(&mut m, &b, casing, 7).expect("insertsubseg");
        assert_eq!(m.subsegs.live_count(), 1);

        let before = snap(&m);
        let flipedge = Otri {
            tri: t1.tri,
            orient: 0,
        };
        flip(&mut m, &b, flipedge);
        // the flip really changed the pool contents
        assert_ne!(snap(&m), before);
        unflip(&mut m, &b, flipedge);
        assert_rows_reverted(&before, &m);
        assert_eq!(m.triangles.live_count(), 2);
        assert_eq!(m.subsegs.live_count(), 1);
    }

    #[test]
    fn insertvertex_in_a_triangle_splits_it_into_three() {
        // nextras 1 / eextras 1 / vararea: the attribute interpolation and
        // the element-attribute and areabound copies all run.
        let mut m = Mesh::new(
            &Behavior {
                vararea: true,
                regionattrib: true,
                ..behavior(false)
            },
            16,
            2,
            1,
            1,
        )
        .expect("mesh init");
        let a = vxy(&mut m, 0.0, 0.0);
        let b = vxy(&mut m, 1.0, 0.0);
        let c = vxy(&mut m, 0.0, 1.0);
        m.vx_set_real(a, 2, 1.0);
        m.vx_set_real(b, 2, 3.0);
        m.vx_set_real(c, 2, 2.0);
        let t = m.maketriangle(false, true).expect("tri");
        m.setorg(t, a);
        m.setdest(t, b);
        m.setapex(t, c);
        let tri_live = m.triangles.live_count();

        let x = vxy(&mut m, 0.2, 0.2);
        let mut searchtri = Otri {
            tri: t.tri,
            orient: 0,
        };
        let res = insertvertex(
            &mut m,
            &behavior(false),
            x,
            &mut searchtri,
            None,
            false,
            false,
            true,
        )
        .expect("insertvertex");
        assert_eq!(res, InsertVertexResult::SuccessfulVertex);
        assert_eq!(m.org(searchtri), x);
        assert_eq!(m.recenttri, Some(searchtri));
        assert_eq!(m.triangles.live_count(), tri_live + 2);
        // the attribute row of the new vertex was filled by interpolate
        assert!(m.vx_real(x, 2).is_finite());
        // a full circle about the new vertex visits exactly three triangles
        let mut cur = searchtri;
        let mut star = 0;
        loop {
            star += 1;
            cur = m.onext(cur);
            if cur == searchtri {
                break;
            }
        }
        assert_eq!(star, 3);
    }

    #[test]
    fn insertvertex_on_an_existing_vertex_reports_duplicate() {
        let mut m = mesh(false, 0, 0);
        let a = vxy(&mut m, 0.0, 0.0);
        let b = vxy(&mut m, 1.0, 0.0);
        let c = vxy(&mut m, 0.0, 1.0);
        let t = m.maketriangle(false, false).expect("tri");
        m.setorg(t, a);
        m.setdest(t, b);
        m.setapex(t, c);
        let tri_live = m.triangles.live_count();

        // a fresh vertex row carrying the coordinates of the existing a;
        // the search starts at orient 1 so that a is the triangle's apex
        // (the vertex check preciselocate performs first)
        let x = vxy(&mut m, 0.0, 0.0);
        let mut searchtri = Otri {
            tri: t.tri,
            orient: 1,
        };
        let res = insertvertex(
            &mut m,
            &behavior(false),
            x,
            &mut searchtri,
            None,
            false,
            false,
            false,
        )
        .expect("insertvertex");
        assert_eq!(res, InsertVertexResult::DuplicateVertex);
        assert_eq!(m.org(searchtri), a);
        assert!(m.recenttri.is_some());
        assert_eq!(m.triangles.live_count(), tri_live);
    }

    #[test]
    fn insertvertex_on_a_subsegment_reports_violating_and_enqueues() {
        let mut m = mesh(true, 0, 0);
        m.checksegments = true;
        let b = behavior(true);
        let a = vxy(&mut m, 0.0, 0.0);
        let e = vxy(&mut m, 2.0, 0.0);
        let c = vxy(&mut m, 1.0, 1.0);
        let d = vxy(&mut m, 1.0, -1.0);
        let (t1, _t2) = pair(&mut m, true, a, e, c, d);
        // subsegment on the shared edge (a -> e) of t1: orient 0.
        let shared = Otri {
            tri: t1.tri,
            orient: 0,
        };
        insertsubseg(&mut m, &b, shared, 0).expect("insertsubseg");
        let tri_live = m.triangles.live_count();

        // a vertex at the midpoint of the subsegmented shared edge. The
        // search starts at orient 1 so preciselocate finds the point ON the
        // (a, e) edge (stopatsubsegment then returns OUTSIDE before the
        // subsegment walk-through, with the edge returned as primary).
        let x = vxy(&mut m, 1.0, 0.0);
        let mut searchtri = Otri {
            tri: t1.tri,
            orient: 1,
        };
        let res = insertvertex(&mut m, &b, x, &mut searchtri, None, true, false, false)
            .expect("insertvertex");
        assert_eq!(res, InsertVertexResult::ViolatingVertex);
        assert_eq!(m.triangles.live_count(), tri_live);
        assert_eq!(m.badsubsegs.live_count(), 1);
        // the badsubseg row pins the broken subsegment exactly as C writes
        // it (sencode + sorg + sdest of the very handle 'searchtri' holds);
        // the badsubsegs pool takes its first row at id 1 (no sentinel)
        let row = 1;
        let broken = m.tspivot(searchtri);
        assert_eq!(m.bad_sub_seg_enc(row), sencode(broken));
        assert_eq!(m.bad_sub_seg_org(row), m.sorg(broken));
        assert_eq!(m.bad_sub_seg_dest(row), m.sdest(broken));
    }

    #[test]
    fn insertvertex_with_splitseg_splits_the_subsegment() {
        let mut m = mesh(true, 0, 0);
        m.checksegments = true;
        let b = behavior(true);
        let a = vxy(&mut m, 0.0, 0.0);
        let e = vxy(&mut m, 2.0, 0.0);
        let c = vxy(&mut m, 1.0, 1.0);
        let d = vxy(&mut m, 1.0, -1.0);
        let (t1, _t2) = pair(&mut m, true, a, e, c, d);
        let shared = Otri {
            tri: t1.tri,
            orient: 0,
        };
        insertsubseg(&mut m, &b, shared, 7).expect("insertsubseg");
        assert_eq!(m.subsegs.live_count(), 1);

        // the caller's searchtri must be the triangle flipped from the
        // subsegment (C inserts no point location when splitseg is set)
        let x = vxy(&mut m, 1.0, 0.0);
        let mut splitseg = m.tspivot(shared);
        let mut searchtri = m.stpivot(splitseg);
        let res = insertvertex(
            &mut m,
            &b,
            x,
            &mut searchtri,
            Some(&mut splitseg),
            false,
            false,
            false,
        )
        .expect("insertvertex");
        assert_eq!(res, InsertVertexResult::SuccessfulVertex);
        assert_eq!(m.org(searchtri), x);
        // the subsegment became two subsegments
        assert_eq!(m.subsegs.live_count(), 2);
        // the split half ends at the inserted vertex
        assert_eq!(m.sdest(splitseg), x);
        // and the boundary marker transferred to the inserted vertex
        assert_eq!(m.vx_mark(x), 7);
    }

    #[test]
    fn insertvertex_on_a_boundary_edge_grows_the_hull_and_undo_reverses() {
        let mut m = mesh(true, 0, 0);
        // poly rows carry the subseg overlay that undovertex touches
        // unconditionally (tspivot/tsbond in the unflip chain walk)
        m.checkquality = true;
        let a = vxy(&mut m, 0.0, 0.0);
        let b = vxy(&mut m, 1.0, 0.0);
        let c = vxy(&mut m, 0.0, 1.0);
        let t = m.maketriangle(true, false).expect("tri");
        m.setorg(t, a);
        m.setdest(t, b);
        m.setapex(t, c);
        let tri_live = m.triangles.live_count();
        assert_eq!(m.hullsize, 0);

        // the search starts at orient 1 so preciselocate finds the point ON
        // the (a, b) hull edge; the split runs over the boundary path
        let before = snap(&m);
        let x = vxy(&mut m, 0.5, 0.0);
        let mut searchtri = Otri {
            tri: t.tri,
            orient: 1,
        };
        let res = insertvertex(
            &mut m,
            &behavior(true),
            x,
            &mut searchtri,
            None,
            false,
            false,
            false,
        )
        .expect("insertvertex");
        assert_eq!(res, InsertVertexResult::SuccessfulVertex);
        assert_eq!(m.org(searchtri), x);
        // a boundary edge was split: one new triangle, one more hull edge
        assert_eq!(m.triangles.live_count(), tri_live + 1);
        assert_eq!(m.hullsize, 1);
        // the bisection record is on the flip stack (the marker chain end)
        assert_eq!(m.flip_prevflip(m.lastflip), INSERTVERTEX_MARKER);

        // and the boundary bisection undoes bit-for-bit (the gluetri branch
        // skips its second half on the boundary side)
        undovertex(&mut m, &behavior(false));
        assert_eq!(m.lastflip, 0);
        assert_rows_reverted(&before, &m);
        assert_eq!(m.triangles.live_count(), tri_live);
    }

    #[test]
    fn deletevertex_removes_an_interior_vertex_and_retriangulates() {
        // A square fan around the center, four triangles: deleting the
        // center leaves two triangles, exactly like C's degree-4 path
        // (triangulatepolygon into a fan of two, then splice).
        let mut m = mesh(true, 0, 0);
        let mut b = behavior(true);
        b.nobisect = 1; // -Y: the quality flank of deletevertex is off
        let center = vxy(&mut m, 0.0, 0.0);
        let outer = [
            vxy(&mut m, 1.0, 0.0),
            vxy(&mut m, 0.0, 1.0),
            vxy(&mut m, -1.0, 0.0),
            vxy(&mut m, 0.0, -1.0),
        ];
        let n = 4;
        let mut tris = vec![];
        for k in 0..n {
            let t = m.maketriangle(true, false).expect("tri");
            m.setorg(t, center);
            m.setdest(t, outer[k]);
            m.setapex(t, outer[(k + 1) % n]);
            tris.push(t);
        }
        // bond the spokes exactly like the fan builder of locate.rs
        for k in 0..n {
            let a = tris[k].lprev();
            let hb = Otri {
                tri: tris[(k + 1) % n].tri,
                orient: 0,
            };
            m.bond(a, hb);
        }
        assert_eq!(m.triangles.live_count(), 4);

        let mut deltri = Otri {
            tri: tris[0].tri,
            orient: 0,
        };
        deletevertex(&mut m, &b, &mut deltri).expect("deletevertex");
        // two triangles were spliced out
        assert_eq!(m.triangles.live_count(), 2);
        // the deleted vertex is dead
        assert_eq!(m.vx_type(center), DEADVERTEX);
        // the deleted vertex is gone from every surviving triangle
        let mut tr = m.triangles.traversal();
        while let Some(id) = m.triangles.traverse_next(&mut tr) {
            if m.deadtri(id) {
                continue;
            }
            for orient in 0..3u8 {
                let h = Otri { tri: id, orient };
                assert_ne!(m.org(h), center, "center must be gone from org");
                assert_ne!(m.dest(h), center, "center must be gone from dest");
                assert_ne!(m.apex(h), center, "center must be gone from apex");
            }
        }
    }

    /// A behavior mirroring C's '-pq20' quality flows (the lens and
    /// angle thresholds need the derived goodangle), computed through
    /// the landed behavior_update exactly like a parsed command line.
    fn behavior_q(poly: bool, minangle: f64) -> Behavior {
        let mut b = behavior(poly);
        b.quality = true;
        b.minangle = minangle;
        b.behavior_update();
        b
    }

    /// The #28 restoration, triangle.c:3302-3307: inserting a vertex
    /// inside a triangle whose opposite edge carries a subsegment runs
    /// checkseg4encroach on that subsegment and inlens on the new
    /// vertex; with both inside the lens the insertion reports
    /// ENCROACHINGVERTEX and the encroached-subsegment row is enqueued
    /// (hand-derived: at q20 the lens threshold is
    /// cos^2(40)*1.0201 = 0.5987 and the vertex (1, 0.1) scores
    /// dot^2 = 0.9801 above it; the far-side apex (1, -1) sits on the
    /// lens boundary circle - dot = 0.0 - and does not encroach).
    #[test]
    fn insertvertex_encroaching_vertex_reports_and_enqueues() {
        let mut m = mesh(true, 0, 0);
        m.checksegments = true;
        let b = behavior_q(true, 20.0);
        let a = vxy(&mut m, 0.0, 0.0);
        let e = vxy(&mut m, 2.0, 0.0);
        let c = vxy(&mut m, 1.0, 3.0);
        let d = vxy(&mut m, 1.0, -1.0);
        let (t1, _t2) = pair(&mut m, true, a, e, c, d);
        let shared = Otri {
            tri: t1.tri,
            orient: 0,
        };
        insertsubseg(&mut m, &b, shared, 0).expect("insertsubseg");
        assert_eq!(m.badsubsegs.live_count(), 0);

        let x = vxy(&mut m, 1.0, 0.1);
        let mut searchtri = Otri {
            tri: t1.tri,
            orient: 0,
        };
        let res = insertvertex(&mut m, &b, x, &mut searchtri, None, true, false, false)
            .expect("insertvertex");
        assert_eq!(res, InsertVertexResult::EncroachingVertex);
        // the encroached subsegment row (encroached == 1: the near side
        // only, so the row carries the subsegment's own orientation)
        assert_eq!(m.badsubsegs.live_count(), 1);
        // the (a, e) edge's triangle kept its subsegment bond through
        // the insertion; the enqueued row is that subsegment.
        let sub = m.tspivot(Otri {
            tri: t1.tri,
            orient: 0,
        });
        assert_ne!(sub.ss, DUMMY_SUB);
        assert_eq!(m.bad_sub_seg_enc(1), sencode(sub));
        assert_eq!(m.bad_sub_seg_org(1), m.sorg(sub));
        assert_eq!(m.bad_sub_seg_dest(1), m.sdest(sub));
    }

    /// The #28 restoration, triangle.c:3468-3473 (and the encroachment
    /// site staying composed): with triflaws set, every edge accepted
    /// without a flip (the subsegmented (a, e), the two hull edges)
    /// runs testtriangle on its triangle; all three new triangles are
    /// skinny at q20 (hand-derived cossquares: (a, e, x) scores
    /// 4/(4*1.01) = 0.9901, (e, c, x) and (c, a, x) score
    /// 75.69/84.1 = 0.8999, all above cos^2(20) = 0.883), so three
    /// bad-triangle rows land, each keyed by the squared shortest edge
    /// 1.01.
    #[test]
    fn insertvertex_with_triflaws_tests_the_committed_triangles() {
        let mut m = mesh(true, 0, 0);
        m.checksegments = true;
        let b = behavior_q(true, 20.0);
        let a = vxy(&mut m, 0.0, 0.0);
        let e = vxy(&mut m, 2.0, 0.0);
        let c = vxy(&mut m, 1.0, 3.0);
        let d = vxy(&mut m, 1.0, -1.0);
        let (t1, _t2) = pair(&mut m, true, a, e, c, d);
        let shared = Otri {
            tri: t1.tri,
            orient: 0,
        };
        insertsubseg(&mut m, &b, shared, 0).expect("insertsubseg");

        let x = vxy(&mut m, 1.0, 0.1);
        let mut searchtri = Otri {
            tri: t1.tri,
            orient: 0,
        };
        let res = insertvertex(&mut m, &b, x, &mut searchtri, None, false, true, false)
            .expect("insertvertex");
        // no segmentflaws: the insertion reports success; only the
        // quality tests ran.
        assert_eq!(res, InsertVertexResult::SuccessfulVertex);
        assert_eq!(m.badsubsegs.live_count(), 0);
        assert_eq!(m.badtriangles.live_count(), 3);
        for row in 1..=3u32 {
            assert_eq!(m.bad_tri_key(row), 1.01, "row {}", row);
            let poor = decode(m.bad_tri_poortri(row));
            assert!(
                !m.deadtri(poor.tri),
                "row {} decodes to a live triangle",
                row
            );
            // each row's triangle has the new vertex as a corner
            let (o, ds, ap) = (m.org(poor), m.dest(poor), m.apex(poor));
            assert!(
                o == x || ds == x || ap == x,
                "row {} must reference the inserted vertex",
                row
            );
        }
    }

    /// The #28 restoration, triangle.c:3705-3707 (plus the
    /// triangulatepolygon flip site at 3608-3617, reached because the
    /// recursion hard-codes doflip = 1 at C 3597/3601 - only
    /// deletevertex's top call passes doflip = 0): deleting the center
    /// of a unit-distance square fan with nobisect == 0 tests both the
    /// triangle across the recursive flip and the relocated deltri.
    /// All fan triangles and both diamond halves are 45-45-90; at q50
    /// (goodangle cos^2(50) = 0.4132) the 45-degree minimum scores
    /// cossquare 0.5 above it, so exactly two rows land: the
    /// flip-site triangle first, then the relocated deltri itself.
    #[test]
    fn deletevertex_with_nobisect_zero_tests_the_relocated_triangle() {
        // The same square fan the retriangulation test uses.
        let mut m = mesh(true, 0, 0);
        let b = behavior_q(true, 50.0);
        assert_eq!(b.nobisect, 0, "the default behavior runs the flank");
        let center = vxy(&mut m, 0.0, 0.0);
        let outer = [
            vxy(&mut m, 1.0, 0.0),
            vxy(&mut m, 0.0, 1.0),
            vxy(&mut m, -1.0, 0.0),
            vxy(&mut m, 0.0, -1.0),
        ];
        let n = 4;
        let mut tris = vec![];
        for k in 0..n {
            let t = m.maketriangle(true, false).expect("tri");
            m.setorg(t, center);
            m.setdest(t, outer[k]);
            m.setapex(t, outer[(k + 1) % n]);
            tris.push(t);
        }
        for k in 0..n {
            let a = tris[k].lprev();
            let hb = Otri {
                tri: tris[(k + 1) % n].tri,
                orient: 0,
            };
            m.bond(a, hb);
        }

        let mut deltri = Otri {
            tri: tris[0].tri,
            orient: 0,
        };
        assert_eq!(m.badtriangles.live_count(), 0);
        deletevertex(&mut m, &b, &mut deltri).expect("deletevertex");
        assert_eq!(m.badtriangles.live_count(), 2);
        // both tested triangles are diamond halves, keyed by the
        // squared shortest edge (edges sqrt(2), sqrt(2), 2 squared
        // give 2.0 as the minimum).
        assert_eq!(m.bad_tri_key(1), 2.0);
        assert_eq!(m.bad_tri_key(2), 2.0);
        // the second row is the relocated triangle itself (the
        // 3705-3707 site; the first is the recursive flip site).
        assert_eq!(m.bad_tri_poortri(2), encode(deltri));
    }

    #[test]
    fn undovertex_restores_a_trisection_bit_for_bit() {
        let mut m = mesh(true, 0, 0);
        // poly rows carry the subseg overlay that undovertex touches
        // unconditionally (tspivot/tsbond in the unflip chain walk)
        m.checkquality = true;
        let a = vxy(&mut m, 0.0, 0.0);
        let b = vxy(&mut m, 1.0, 0.0);
        let c = vxy(&mut m, 0.0, 1.0);
        let t = m.maketriangle(true, false).expect("tri");
        m.setorg(t, a);
        m.setdest(t, b);
        m.setapex(t, c);
        let tri_live = m.triangles.live_count();

        let before = snap(&m);
        let x = vxy(&mut m, 0.2, 0.2);
        let mut searchtri = Otri {
            tri: t.tri,
            orient: 0,
        };
        let res = insertvertex(
            &mut m,
            &behavior(true),
            x,
            &mut searchtri,
            None,
            false,
            false,
            false,
        )
        .expect("insertvertex");
        assert_eq!(res, InsertVertexResult::SuccessfulVertex);
        assert_eq!(m.triangles.live_count(), tri_live + 2);
        // the trisection record is on the flip stack
        assert_eq!(m.flipstackers.live_count(), 1);
        assert_ne!(m.lastflip, 0);

        undovertex(&mut m, &behavior(false));
        assert_eq!(m.lastflip, 0);
        assert_rows_reverted(&before, &m);
        assert_eq!(m.triangles.live_count(), tri_live);
        // the inserted vertex row stays allocated, exactly like C's body
        // (undovertex leaves the vertex dealloc to its callers)
    }

    #[test]
    fn undovertex_restores_a_bisection_bit_for_bit() {
        let mut m = mesh(true, 0, 0);
        // poly rows carry the subseg overlay that undovertex touches
        // unconditionally (tspivot/tsbond in the unflip chain walk)
        m.checkquality = true;
        // two triangles sharing the interior edge (a, e)
        let a = vxy(&mut m, 0.0, 0.0);
        let e = vxy(&mut m, 2.0, 0.0);
        let c = vxy(&mut m, 1.0, 1.0);
        let d = vxy(&mut m, 1.0, -1.0);
        let (t1, _t2) = pair(&mut m, true, a, e, c, d);
        let tri_live = m.triangles.live_count();

        // the search starts at orient 1 so preciselocate finds the point ON
        // the interior shared edge (a, e): both sides split into four
        let before = snap(&m);
        let x = vxy(&mut m, 1.0, 0.0);
        let mut searchtri = Otri {
            tri: t1.tri,
            orient: 1,
        };
        let res = insertvertex(
            &mut m,
            &behavior(true),
            x,
            &mut searchtri,
            None,
            false,
            false,
            false,
        )
        .expect("insertvertex");
        assert_eq!(res, InsertVertexResult::SuccessfulVertex);
        // an interior edge split: two triangles became four
        assert_eq!(m.triangles.live_count(), tri_live + 2);
        assert_eq!(m.hullsize, 0); // no boundary edge was split
                                   // the bisection record (the &insertvertex marker) is on the stack
        assert_eq!(m.flipstackers.live_count(), 1);
        assert_eq!(m.flip_prevflip(m.lastflip), INSERTVERTEX_MARKER);

        undovertex(&mut m, &behavior(false));
        assert_eq!(m.lastflip, 0);
        assert_rows_reverted(&before, &m);
        assert_eq!(m.triangles.live_count(), tri_live);
    }

    #[test]
    fn undovertex_restores_a_flip_chain_bit_for_bit() {
        // Two triangles whose shared edge stops being Delaunay once the
        // new vertex lands inside the left one: the insertion records a
        // trisection plus at least one flip, so undovertex walks the unflip
        // branch too.
        let mut m = mesh(true, 0, 0);
        // poly rows carry the subseg overlay that undovertex touches
        // unconditionally (tspivot/tsbond in the unflip chain walk)
        m.checkquality = true;
        // t1 = (a, b, c); t2 = (a, c, d) shares the edge (c -> a); the edge
        // (c -> a) of t1 is orient 2, its mirror in t2 is orient 0.
        let a = vxy(&mut m, 0.0, 0.0);
        let b = vxy(&mut m, 2.0, 0.0);
        let c = vxy(&mut m, 1.0, 1.0);
        let d = vxy(&mut m, 0.2, 0.5);
        let t1 = m.maketriangle(true, false).expect("tri");
        m.setorg(t1, a);
        m.setdest(t1, b);
        m.setapex(t1, c);
        let t2 = m.maketriangle(true, false).expect("tri");
        m.setorg(t2, a);
        m.setdest(t2, c);
        m.setapex(t2, d);
        m.bond(
            Otri {
                tri: t1.tri,
                orient: 2,
            },
            Otri {
                tri: t2.tri,
                orient: 0,
            },
        );
        let tri_live = m.triangles.live_count();

        // x lands strictly inside t1, close enough to the (c, a) edge that
        // the far vertex d falls inside the circumcircle of (a, x, c): the
        // recorded flips include the shared edge's flip
        let before = snap(&m);
        let x = vxy(&mut m, 0.9, 0.15);
        let mut searchtri = Otri {
            tri: t1.tri,
            orient: 0,
        };
        let res = insertvertex(
            &mut m,
            &behavior(true),
            x,
            &mut searchtri,
            None,
            false,
            false,
            false,
        )
        .expect("insertvertex");
        assert_eq!(res, InsertVertexResult::SuccessfulVertex);
        // trisection record plus flip records on the flip stack
        assert!(
            m.flipstackers.live_count() >= 2,
            "the insertion must have recorded at least one flip"
        );

        undovertex(&mut m, &behavior(false));
        assert_eq!(m.lastflip, 0);
        assert_rows_reverted(&before, &m);
        assert_eq!(m.triangles.live_count(), tri_live);
    }
}
