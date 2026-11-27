//! General mesh construction: reconstruct, finddirection,
//! segmentintersection, scoutsegment, conformingedge, delaunayfixup,
//! constrainededge, insertsegment, markhull, formskeleton, infecthull,
//! plague, regionplague, carveholes (triangle.c section General mesh
//! construction routines, lines 5419-7687). Assignment:
//! docs/function-mapping.md.
//!
//! Transliteration notes (beyond the module defaults):
//! - The C 'int *status' out-params are dissolved (rung 2, D8): every
//!   raising site raises its error in place. The hot
//!   segment-insertion chain (finddirection through insertsegment)
//!   carries the narrow internal Copy SegmentChainError of the fix
//!   pass - Recovery(SegmentRecoveryKind) at the TRI_* status writes,
//!   Internal at conformingedge's internalerror site (the constant
//!   payload is constructed only at the boundary), propagated
//!   pool::Oom as Oom - converted losslessly to the public Error at
//!   the formskeleton boundary, where the error leaves the hot
//!   region. reconstruct's TRI_INPUT returns raise Error::Input in
//!   place, and the C int channels that carry non-error values stay
//!   as Ok payloads (scoutsegment's 1/0 result, reconstruct's hull
//!   size).
//! - C's #ifdef _DEBUG printf sites of formskeleton/reconstruct and
//!   the internal-error printf texts of conformingedge are compiled
//!   out of the pinned oracle build and are not transliterated; the
//!   conformingedge site raises SegmentChainError::Internal, whose
//!   boundary conversion constructs internalerror()'s verbatim
//!   payload instead (the printf text rides as the comment).
//! - The C -D guards (REDUCED/CDT_ONLY) around conformingedge are open
//!   in the pinned oracle; the port has a single build.
//! - reconstruct's vertexarray is C trimalloc'd scratch (not mesh
//!   state), mirrored to an owned local Vec<u64> arena with C's
//!   allocation order sequence; the prevlink pointer-into-slot pun
//!   (a pointer to vertexarray[i] or to tri[6 + orient]) becomes the
//!   PrevLink carrier below, stored and loaded through the same raw
//!   slot accesses the C macro expansion performs. The tri[3] = tri /
//!   ss[2] = ss "mark as living" scribbles are raw id words on the
//!   org slots, exactly C's stores, overwritten by the setorg pass.
//! - carveholes consumes the typed Hole/Region records of the frozen
//!   input.rs surface (the flat C holelist/regionlist strides mirror
//!   to the record fields); the trimalloc'd regiontris array is an
//!   owned local Vec<Otri> arena, D11 zero/dummy superset of C's
//!   uninitialized entries, which C only reads after the .tri != NULL
//!   filter or as full otricopy results.
//! - The region attribute column is written PAST the count eextras
//!   (regionattrib's extra column, allocated by initializetrisubpools
//!   exactly like C's setelemattribute(testtri, m->eextras, x) macro,
//!   which does not range-check); the chokepoint's set_tri_real at
//!   elemattribindex + eextras mirrors that macro exactly. eextras
//!   itself is only incremented after the region loop (triangle.c:7097).
//! - No direct randomnation draws live in this module; the
//!   determinism-critical locate()/insertvertex() draws underneath
//!   are the landed ones under the D10 per-run seed.
//!
//! Stage-1 note: the consumers of this module are reachable only
//! through the api.rs entry points, which stay scaffolded until the
//! refine slice lands; the blanket below silences that ahead-of-use
//! dead_code and is removed when the scaffolding is consumed.
#![allow(dead_code)]

use crate::error::{Error, SegmentRecoveryKind};
use crate::handles::{decode, encode, Osub, Otri, DUMMY_SUB, DUMMY_TRI};
use crate::locate::{locate, LocateResult};
use crate::mesh::Mesh;
use crate::options::Behavior;
use crate::pool::Oom;
use crate::predicates::{tri_counterclockwise, tri_incircle, Vertex};
use crate::slots::{DEADVERTEX, INPUTVERTEX, SEGMENTVERTEX, UNDEADVERTEX};
use crate::transform::{insertsubseg, insertvertex, InsertVertexResult};
use crate::util::vertex_coords;

// ---------------------------------------------------------------------------
// SegmentChainError (the fix pass's narrow hot-chain error)
// ---------------------------------------------------------------------------

/// The narrow internal error of the hot segment-insertion chain
/// (finddirection, segmentintersection, scoutsegment, conformingedge,
/// constrainededge, insertsegment; the rung-2 fix pass). The chain's
/// error union is payload-free: the TRI_* *status writes raise
/// Recovery(SegmentRecoveryKind) (the kinds carry no payload, so this
/// is lossless), conformingedge's internalerror site raises Internal
/// (the payload is internalerror()'s verbatim constant text,
/// constructed only at the boundary conversion), and pool allocation
/// failure propagates as Oom (pool::Oom converts here exactly as
/// From<pool::Oom> for Error does at the public seam). The enum is
/// Copy and one byte wide, so every Result in the chain is
/// niche-narrowed to its Ok payload's width - the i32-shaped status
/// channel width C had, which rung 2's wide Result<T, Error> inflated
/// on these per-edge calls. The public taxonomy is unchanged: the
/// error converts to crate::error::Error at the boundary where it
/// leaves the hot region (formskeleton's insertsegment call).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SegmentChainError {
    /// A TRI_* status write raised at its site; the
    /// SegmentRecoveryKind is the exact C code.
    Recovery(SegmentRecoveryKind),
    /// conformingedge's internalerror site (triangle.c:6089-6175; the
    /// printf compiles out, the payload is the verbatim
    /// triangle.c:349-354 text, constructed at the boundary).
    Internal,
    /// Pool allocation failure propagated through the chain (D26's
    /// Oom bridge, at chain width).
    Oom,
}

impl From<Oom> for SegmentChainError {
    fn from(_: Oom) -> SegmentChainError {
        SegmentChainError::Oom
    }
}

/// The boundary conversion: lossless; the public taxonomy and its
/// Display strings are byte-identical to rung 2's direct raises.
impl From<SegmentChainError> for Error {
    fn from(e: SegmentChainError) -> Error {
        match e {
            SegmentChainError::Recovery(kind) => Error::SegmentRecovery(kind),
            SegmentChainError::Internal => Error::Internal(
                "  Please report this bug to jrs@cs.berkeley.edu
  Include the message above, your input data set, and the exact
    command line you used to run Triangle."
                    .to_string(),
            ),
            SegmentChainError::Oom => Error::OutOfMemory,
        }
    }
}

/// C: enum finddirectionresult (triangle.h:113): WITHIN, LEFTCOLLINEAR,
/// RIGHTCOLLINEAR - the exact C discriminants (pinned by test below).
/// Owned by this module per the mapping's enumeration rule: it is the
/// return type of the functions that live here.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(i32)]
pub(crate) enum FindDirectionResult {
    /// C WITHIN.
    Within = 0,
    /// C LEFTCOLLINEAR.
    LeftCollinear = 1,
    /// C RIGHTCOLLINEAR.
    RightCollinear = 2,
}

/// C reconstruct's prevlink: a pointer either into the vertexarray
/// element for a vertex's stack head, or into a triangle row's slot
/// 6 + orient (the subseg slot borrowed as the vertex-stack link).
/// The pun stores and loads through the same raw slot accesses the C
/// macro expansion performs (D1: ids replace the pointer halves).
#[derive(Clone, Copy)]
enum PrevLink {
    /// &vertexarray[i].
    Array(usize),
    /// &tri[6 + orient] (row id, orient).
    Tri(u32, u8),
}

// ---------------------------------------------------------------------------
// reconstruct
// ---------------------------------------------------------------------------

/// C: reconstruct (triangle.c:5483-5722). Reconstruct a triangulation
/// from its .ele list (and, under -p, its .poly segments): create rows
/// for every element, read and link every triangle, read and link every
/// subsegment, then mark the remaining edges and count the boundary.
/// Returns the hull size as the Ok payload; the three TRI_INPUT
/// returns of C (triangle.c:5522, 5573, 5664) raise Error::Input at
/// exactly those return points (the caller stores the Ok result in
/// m->hullsize exactly like the api layer does).
#[allow(unused_assignments)] // C declares boundmarker/hullsize upfront and re-arms them at the section boundaries
#[allow(clippy::too_many_arguments)]
// C-parity: the indecremented index walks over corner/end/vertexarray
// mirror C's `for (j = 0; ...)` shapes verbatim.
#[allow(clippy::needless_range_loop)]
pub(crate) fn reconstruct(
    m: &mut Mesh,
    b: &Behavior,
    trianglelist: &[i32],
    triangleattriblist: &[f64],
    trianglearealist: &[f64],
    elements: i32,
    corners: i32,
    attribs: i32,
    segmentlist: &[i32],
    segmentmarkerlist: Option<&[i32]>,
    numberofsegments: i32,
) -> Result<i32, Error> {
    let mut vertexindex = 0usize;
    let mut attribindex = 0usize;
    let mut corner = [0i32; 3];
    let mut end = [0i32; 2];
    let mut segmentmarkers = false;
    let mut boundmarker = 0i32;
    let mut hullsize: i64 = 0;
    let mut elementnumber: i64;

    m.inelements = elements;
    let incorners = corners;
    if incorners < 3 {
        // C: Triangles must have at least 3 vertices.
        return Err(Error::Input("invalid input (TRI_INPUT)".to_string()));
    }
    m.eextras = attribs;

    m.initializetrisubpools(
        b.order,
        b.usesegments,
        b.vararea,
        b.regionattrib,
        b.neighbors,
    )?;

    // Create the triangles.
    for _ in 1..=m.inelements {
        let triangleloop = m.maketriangle(b.usesegments, b.vararea)?;
        // Mark the triangle as living: C's raw tri[3] = (triangle) tri
        // self-scribble on the org slot, later overwritten by setorg.
        m.set_tri_vertex_word(triangleloop.tri, 0, triangleloop.tri as u64);
    }

    if b.poly {
        m.insegments = numberofsegments;
        segmentmarkers = segmentmarkerlist.is_some();

        // Create the subsegments.
        for _ in 1..=m.insegments {
            let subsegloop = m.makesubseg()?;
            // Mark the subsegment as living: C's raw ss[2] = (subseg) ss.
            m.set_sub_vertex_word(subsegloop.ss, 0, subsegloop.ss as u64);
        }
    }

    // Allocate a temporary array that maps each vertex to some adjacent
    //   triangle.  I took care to allocate all the permanent memory for
    //   triangles and subsegments first.  (The trimalloc rule: owned
    //   local arena, reservation failure routed to the pool OOM bridge.)
    let array_items = m.vertices.live_count() as usize;
    let mut vertexarray: Vec<u64> = Vec::new();
    vertexarray
        .try_reserve_exact(array_items)
        .map_err(|_| Oom)?;
    // Each vertex is initially unrepresented: C vertexarray[i] =
    // (triangle) m->dummytri is the unencoded dummy pointer, the word
    // decode() maps onto the {DUMMY_TRI, 0} handle.
    let dummy_word = encode(Otri {
        tri: DUMMY_TRI,
        orient: 0,
    });
    for _ in 0..array_items {
        vertexarray.push(dummy_word);
    }

    // Read the triangles from the .ele file, and link
    // together those that share an edge.
    let mut t = m.triangles.traversal();
    let mut triangleloop_id = m.triangletraverse(&mut t);
    elementnumber = b.firstnumber as i64;
    while let Some(tri_id) = triangleloop_id {
        let mut triangleloop = Otri {
            tri: tri_id,
            orient: 0,
        };
        // Copy the triangle's three corners.
        for j in 0..3usize {
            corner[j] = trianglelist[vertexindex];
            vertexindex += 1;
            if (corner[j] < b.firstnumber) || (corner[j] >= b.firstnumber + m.invertices) {
                // C: Triangle (elementnumber) has an invalid vertex index.
                return Err(Error::Input("invalid input (TRI_INPUT)".to_string()));
            }
        }

        // Find out about (and throw away) extra nodes.
        for _j in 3..incorners {
            let killvertexindex = trianglelist[vertexindex];
            vertexindex += 1;
            if (killvertexindex >= b.firstnumber)
                && (killvertexindex < b.firstnumber + m.invertices)
            {
                // Delete the non-corner vertex if it's not already deleted.
                let killvertex = m.getvertex(b.firstnumber, killvertexindex);
                if m.vx_type(killvertex) != DEADVERTEX {
                    m.vx_dealloc(killvertex);
                }
            }
        }

        // Read the triangle's attributes.
        for j in 0..m.eextras {
            m.set_tri_attribute(
                triangleloop.tri,
                j as usize,
                triangleattriblist[attribindex],
            );
            attribindex += 1;
        }

        if b.vararea {
            let area = trianglearealist[(elementnumber - b.firstnumber as i64) as usize];
            m.set_tri_areabound(triangleloop.tri, area);
        }
        // Set the triangle's vertices.
        triangleloop.orient = 0;
        m.setorg(triangleloop, m.getvertex(b.firstnumber, corner[0]));
        m.setdest(triangleloop, m.getvertex(b.firstnumber, corner[1]));
        m.setapex(triangleloop, m.getvertex(b.firstnumber, corner[2]));
        // Try linking the triangle to others that share these vertices.
        for orient in 0..3u8 {
            triangleloop.orient = orient;
            let aroundvertex = corner[triangleloop.orient as usize];
            let ai = (aroundvertex - b.firstnumber) as usize;
            // Look for other triangles having this vertex.
            let mut nexttri = vertexarray[ai];
            // Link the current triangle to the next one in the stack.
            m.set_tri_subseg_word(triangleloop.tri, triangleloop.orient as usize, nexttri);
            // Push the current triangle onto the stack.
            vertexarray[ai] = encode(triangleloop);
            let mut checktri = decode(nexttri);
            if checktri.tri != DUMMY_TRI {
                let tdest = m.dest(triangleloop);
                let tapex = m.apex(triangleloop);
                // Look for other triangles that share an edge.
                loop {
                    let checkdest = m.dest(checktri);
                    let checkapex = m.apex(checktri);
                    if tapex == checkdest {
                        // The two triangles share an edge; bond them together.
                        let triangleleft = triangleloop.lprev();
                        m.bond(triangleleft, checktri);
                    }
                    if tdest == checkapex {
                        // The two triangles share an edge; bond them together.
                        let checkleft = checktri.lprev();
                        m.bond(triangleloop, checkleft);
                    }
                    // Find the next triangle in the stack.
                    nexttri = m.tri_subseg_word(checktri.tri, checktri.orient as usize);
                    checktri = decode(nexttri);
                    if checktri.tri == DUMMY_TRI {
                        break;
                    }
                }
            }
        }
        triangleloop_id = m.triangletraverse(&mut t);
        elementnumber += 1;
    }

    vertexindex = 0;
    hullsize = 0; // Prepare to count the boundary edges.
    if b.poly {
        let markerlist = segmentmarkerlist.unwrap_or(&[]);
        // Read the segments from the .poly file, and link them
        // to their neighboring triangles.
        boundmarker = 0;
        let mut st = m.subsegs.traversal();
        let mut subsegloop_id = m.subsegtraverse(&mut st);
        let mut segmentnumber = b.firstnumber;
        while let Some(ss_id) = subsegloop_id {
            let mut subsegloop = Osub {
                ss: ss_id,
                ssorient: 0,
            };
            end[0] = segmentlist[vertexindex];
            vertexindex += 1;
            end[1] = segmentlist[vertexindex];
            vertexindex += 1;
            if segmentmarkers {
                boundmarker = markerlist[(segmentnumber - b.firstnumber) as usize];
            }

            for j in 0..2usize {
                if (end[j] < b.firstnumber) || (end[j] >= b.firstnumber + m.invertices) {
                    // C: Segment (segmentnumber) has an invalid vertex index.
                    return Err(Error::Input("invalid input (TRI_INPUT)".to_string()));
                }
            }

            // Set the subsegment's vertices.
            subsegloop.ssorient = 0;
            let segmentorg = m.getvertex(b.firstnumber, end[0]);
            let segmentdest = m.getvertex(b.firstnumber, end[1]);
            m.setsorg(subsegloop, segmentorg);
            m.setsdest(subsegloop, segmentdest);
            m.setsegorg(subsegloop, segmentorg);
            m.setsegdest(subsegloop, segmentdest);
            m.set_sub_mark(subsegloop.ss, boundmarker);
            // Try linking the subsegment to triangles that share these
            // vertices.
            for ssorient in 0..2u8 {
                subsegloop.ssorient = ssorient;
                // Take the number for the destination of subsegloop.
                let aroundvertex = end[(1 - subsegloop.ssorient) as usize];
                let ai = (aroundvertex - b.firstnumber) as usize;
                // Look for triangles having this vertex.
                let mut prevlink = PrevLink::Array(ai);
                let mut nexttri = vertexarray[ai];
                let mut checktri = decode(nexttri);
                let shorg = m.sorg(subsegloop);
                let mut notfound = true;
                // Look for triangles having this edge.  (Same traversal
                // shape as C, including the trailing prevlink advance on
                // the found iteration.)
                while notfound && checktri.tri != DUMMY_TRI {
                    let checkdest = m.dest(checktri);
                    if shorg == checkdest {
                        // We have a match.  Remove this triangle from
                        //   the list: *prevlink = tri[6 + orient].
                        let unlink = m.tri_subseg_word(checktri.tri, checktri.orient as usize);
                        match prevlink {
                            PrevLink::Array(i) => vertexarray[i] = unlink,
                            PrevLink::Tri(t, o) => m.set_tri_subseg_word(t, o as usize, unlink),
                        }
                        // Bond the subsegment to the triangle.
                        m.tsbond(checktri, subsegloop);
                        // Check if this is a boundary edge.
                        let checkneighbor = m.sym(checktri);
                        if checkneighbor.tri == DUMMY_TRI {
                            // The next line doesn't insert a subsegment
                            //   (because there's already one there), but it
                            //   sets the boundary markers of the existing
                            //   subsegment and its vertices.
                            insertsubseg(m, b, checktri, 1)?;
                            hullsize += 1;
                        }
                        notfound = false;
                    }
                    // Find the next triangle in the stack.
                    prevlink = PrevLink::Tri(checktri.tri, checktri.orient);
                    nexttri = m.tri_subseg_word(checktri.tri, checktri.orient as usize);
                    checktri = decode(nexttri);
                }
            }
            subsegloop_id = m.subsegtraverse(&mut st);
            segmentnumber += 1;
        }
    }

    // Mark the remaining edges as not being attached to any subsegment.
    //   Also, count the (yet uncounted) boundary edges.  (The bound is
    //   C's live items read at this point, like the alloc site above.)
    for i in 0..m.vertices.live_count() as usize {
        // Search the stack of triangles adjacent to a vertex.
        let mut nexttri = vertexarray[i];
        let mut checktri = decode(nexttri);
        while checktri.tri != DUMMY_TRI {
            // Find the next triangle in the stack before this
            // information gets overwritten.
            nexttri = m.tri_subseg_word(checktri.tri, checktri.orient as usize);
            // No adjacent subsegment.  (This overwrites the stack info.)
            m.tsdissolve(checktri);
            let checkneighbor = m.sym(checktri);
            if checkneighbor.tri == DUMMY_TRI {
                insertsubseg(m, b, checktri, 1)?;
                hullsize += 1;
            }
            checktri = decode(nexttri);
        }
    }

    // C: trifree(vertexarray) - the arena drops here.
    Ok(hullsize as i32)
}

// ---------------------------------------------------------------------------
// finddirection
// ---------------------------------------------------------------------------

/// C: finddirection (triangle.c:5776-5867). Find the first triangle on
/// the path from one point to another: the triangle that intersects a
/// line segment drawn from the origin of 'searchtri' to 'searchpoint'.
/// The origin of the returned 'searchtri' does not change. Returns
/// whether the destination or apex of the found triangle is collinear
/// with the two points in question.
pub(crate) fn finddirection(
    m: &mut Mesh,
    b: &Behavior,
    searchtri: &mut Otri,
    searchpoint: (f64, f64),
) -> Result<FindDirectionResult, SegmentChainError> {
    let searchpoint_v: Vertex = [searchpoint.0, searchpoint.1, 0.0];

    let startvertex = m.org(*searchtri);
    let mut rightvertex = m.dest(*searchtri);
    let mut leftvertex = m.apex(*searchtri);
    // Is 'searchpoint' to the left?
    let mut leftccw = tri_counterclockwise(
        m,
        b,
        searchpoint_v,
        vertex_coords(m, startvertex),
        vertex_coords(m, leftvertex),
    );
    let mut leftflag = leftccw > 0.0;
    // Is 'searchpoint' to the right?
    let mut rightccw = tri_counterclockwise(
        m,
        b,
        vertex_coords(m, startvertex),
        searchpoint_v,
        vertex_coords(m, rightvertex),
    );
    let mut rightflag = rightccw > 0.0;
    if leftflag && rightflag {
        // 'searchtri' faces directly away from 'searchpoint'.  We could
        //   go left or right.  Ask whether it's a triangle or a boundary
        //   on the left.
        let checktri = m.onext(*searchtri);
        if checktri.tri == DUMMY_TRI {
            leftflag = false;
        } else {
            rightflag = false;
        }
    }
    // Each turn moves to a different triangle around 'startvertex', so
    //   more turns than there are triangles is going in circles.  A
    //   vertex at the end of a long row of collinear vertices can
    //   legitimately have a fan of thousands of triangles around it.
    let maxtries = m.triangles.live_count() as i64;
    let mut tries: i64 = 0;
    while leftflag && tries < maxtries {
        // Turn left until satisfied.
        m.onextself(searchtri);
        if searchtri.tri == DUMMY_TRI {
            // C: Unable to find a triangle leading from the origin to
            // the searchpoint.
            return Err(SegmentChainError::Recovery(
                SegmentRecoveryKind::FindDirection,
            ));
        }
        leftvertex = m.apex(*searchtri);
        rightccw = leftccw;
        leftccw = tri_counterclockwise(
            m,
            b,
            searchpoint_v,
            vertex_coords(m, startvertex),
            vertex_coords(m, leftvertex),
        );
        leftflag = leftccw > 0.0;
        tries += 1;
    }
    while rightflag && tries < maxtries {
        // Turn right until satisfied.
        m.oprevself(searchtri);
        if searchtri.tri == DUMMY_TRI {
            return Err(SegmentChainError::Recovery(
                SegmentRecoveryKind::FindDirection,
            ));
        }
        rightvertex = m.dest(*searchtri);
        leftccw = rightccw;
        rightccw = tri_counterclockwise(
            m,
            b,
            vertex_coords(m, startvertex),
            searchpoint_v,
            vertex_coords(m, rightvertex),
        );
        rightflag = rightccw > 0.0;
        tries += 1;
    }
    if tries >= maxtries {
        return Err(SegmentChainError::Recovery(
            SegmentRecoveryKind::FindDirection,
        ));
    }
    if leftccw == 0.0 {
        Ok(FindDirectionResult::LeftCollinear)
    } else if rightccw == 0.0 {
        Ok(FindDirectionResult::RightCollinear)
    } else {
        Ok(FindDirectionResult::Within)
    }
}

// ---------------------------------------------------------------------------
// segmentintersection
// ---------------------------------------------------------------------------

/// C: segmentintersection (triangle.c:5873-6001). Find the intersection
/// of an existing subsegment and the segment being inserted (from the
/// apex of splittri to endpoint2), insert a vertex there, and split both
/// subsegment copies. On completion splittri has the new intersection
/// point as origin and endpoint1 (the old apex) as destination.
// C-parity: the split roundoff window is C's hand-written conjunction
// (triangle.c:5927), kept verbatim.
#[allow(clippy::manual_range_contains)]
pub(crate) fn segmentintersection(
    m: &mut Mesh,
    b: &Behavior,
    splittri: &mut Otri,
    splitsubseg: &mut Osub,
    endpoint2: u32,
) -> Result<(), SegmentChainError> {
    // Find the other three segment endpoints.
    let endpoint1 = m.apex(*splittri);
    let torg = m.org(*splittri);
    let tdest = m.dest(*splittri);
    let endpoint1_xy = (m.vx_x(endpoint1), m.vx_y(endpoint1));

    // Segment intersection formulae; see the Antonio reference.
    let tx = m.vx_x(tdest) - m.vx_x(torg);
    let ty = m.vx_y(tdest) - m.vx_y(torg);
    let ex = m.vx_x(endpoint2) - endpoint1_xy.0;
    let ey = m.vx_y(endpoint2) - endpoint1_xy.1;
    let etx = m.vx_x(torg) - m.vx_x(endpoint2);
    let ety = m.vx_y(torg) - m.vx_y(endpoint2);
    let denom = ty * ex - tx * ey;
    if denom == 0.0 {
        // C: Attempt to find intersection of parallel segments.
        return Err(SegmentChainError::Recovery(SegmentRecoveryKind::Intersect));
    }
    let split = (ey * etx - ex * ety) / denom;
    // The intersection lies on the subsegment being split, give or take
    //   roundoff.  Segments that are nearly parallel can give a
    //   denominator so small that 'split' is not finite, or lies well
    //   off the subsegment; inserting that vertex corrupts the mesh, or
    //   puts a point with NaN coordinates into it.
    if !(split >= -1.0e-9 && split <= 1.0 + 1.0e-9) {
        return Err(SegmentChainError::Recovery(SegmentRecoveryKind::Intersect));
    }
    // Create the new vertex.
    let newvertex = m.vx_alloc()?;
    // Interpolate its coordinate and attributes (C reads the vertex
    // rows component by component: newvertex[i] = torg[i] + split *
    // (tdest[i] - torg[i]), i < 2 + nextras).
    for i in 0..(2 + m.nextras) as usize {
        m.vx_set_real(
            newvertex,
            i,
            m.vx_real(torg, i) + split * (m.vx_real(tdest, i) - m.vx_real(torg, i)),
        );
    }
    m.vx_set_mark(newvertex, m.sub_mark(splitsubseg.ss));
    m.vx_set_type(newvertex, INPUTVERTEX);
    // Insert the intersection vertex.  This should always succeed.
    let success = insertvertex(
        m,
        b,
        newvertex,
        splittri,
        Some(&mut *splitsubseg),
        false,
        false,
        false,
    )?;
    if m.cyclefail {
        return Err(SegmentChainError::Recovery(SegmentRecoveryKind::Cycle));
    }
    if success != InsertVertexResult::SuccessfulVertex {
        // C: Failure to split a segment.
        return Err(SegmentChainError::Recovery(SegmentRecoveryKind::Intersect));
    }
    // Record a triangle whose origin is the new vertex.
    m.vx_set_2tri(newvertex, encode(*splittri));
    if m.steinerleft > 0 {
        m.steinerleft -= 1;
    }

    // Divide the segment into two, and correct the segment endpoints.
    splitsubseg.ssymself();
    let mut opposubseg = m.spivot(*splitsubseg);
    m.sdissolve(*splitsubseg);
    m.sdissolve(opposubseg);
    loop {
        m.setsegorg(*splitsubseg, newvertex);
        m.snextself(splitsubseg);
        if splitsubseg.ss == DUMMY_SUB {
            break;
        }
    }
    loop {
        m.setsegorg(opposubseg, newvertex);
        m.snextself(&mut opposubseg);
        if opposubseg.ss == DUMMY_SUB {
            break;
        }
    }

    // Inserting the vertex may have caused edge flips.  We wish to
    //   rediscover the edge connecting endpoint1 to the new
    //   intersection vertex.
    let _collinear = finddirection(m, b, splittri, endpoint1_xy)?;

    let rightvertex = m.dest(*splittri);
    let leftvertex = m.apex(*splittri);
    if (m.vx_x(leftvertex) == endpoint1_xy.0) && (m.vx_y(leftvertex) == endpoint1_xy.1) {
        m.onextself(splittri);
    } else if (m.vx_x(rightvertex) != endpoint1_xy.0) || (m.vx_y(rightvertex) != endpoint1_xy.1) {
        // C: Topological inconsistency after splitting a segment.
        return Err(SegmentChainError::Recovery(SegmentRecoveryKind::Intersect));
    }
    // 'splittri' should have destination endpoint1.
    Ok(())
}

// ---------------------------------------------------------------------------
// scoutsegment
// ---------------------------------------------------------------------------

/// C: scoutsegment (triangle.c:6003-6076). Scout the first triangle on
/// the path from one endpoint to another, and check for completion
/// (reaching the second endpoint), a collinear vertex, or the
/// intersection of two segments. Returns C's 1 if the entire segment is
/// successfully inserted, 0 if the job must be finished by
/// conformingedge() or constrainededge(); the scoutsteps budget site
/// (triangle.c:6016-6017) raises SegmentChainError::Recovery(Scout)
/// exactly
/// where C sets *status and returns TRI_SEG_SCOUT.
// C's explicit `return 1;` / `return 0;` / tail `return scoutsegment(...)`
// statements are kept verbatim; eliding them is C-invisible control flow.
#[allow(clippy::needless_return)]
pub(crate) fn scoutsegment(
    m: &mut Mesh,
    b: &Behavior,
    searchtri: &mut Otri,
    endpoint2: u32,
    newmark: i32,
) -> Result<i32, SegmentChainError> {
    // Each call reaches a vertex further along the segment, or is one
    //   of the two that follow a midpoint conformingedge() adds, so
    //   more calls than twice the vertices means the insertion is
    //   going in circles.
    m.scoutsteps += 1;
    if m.scoutsteps > 2 * m.vertices.live_count() as i64 {
        return Err(SegmentChainError::Recovery(SegmentRecoveryKind::Scout));
    }

    let collinear = finddirection(m, b, searchtri, (m.vx_x(endpoint2), m.vx_y(endpoint2)))?;

    let rightvertex = m.dest(*searchtri);
    let leftvertex = m.apex(*searchtri);
    let e2x = m.vx_x(endpoint2);
    let e2y = m.vx_y(endpoint2);
    if ((m.vx_x(leftvertex) == e2x) && (m.vx_y(leftvertex) == e2y))
        || ((m.vx_x(rightvertex) == e2x) && (m.vx_y(rightvertex) == e2y))
    {
        // The segment is already an edge in the mesh.
        if (m.vx_x(leftvertex) == e2x) && (m.vx_y(leftvertex) == e2y) {
            searchtri.lprevself();
        }
        // Insert a subsegment, if there isn't already one there.
        insertsubseg(m, b, *searchtri, newmark)?;
        return Ok(1);
    } else if collinear == FindDirectionResult::LeftCollinear {
        // We've collided with a vertex between the segment's endpoints.
        // Make the collinear vertex be the triangle's origin.
        searchtri.lprevself();
        insertsubseg(m, b, *searchtri, newmark)?;
        // Insert the remainder of the segment.
        return scoutsegment(m, b, searchtri, endpoint2, newmark);
    } else if collinear == FindDirectionResult::RightCollinear {
        // We've collided with a vertex between the segment's endpoints.
        insertsubseg(m, b, *searchtri, newmark)?;
        // Make the collinear vertex be the triangle's origin.
        searchtri.lnextself();
        // Insert the remainder of the segment.
        return scoutsegment(m, b, searchtri, endpoint2, newmark);
    } else {
        let mut crosstri = searchtri.lnext();
        // C: tspivot(crosstri, crosssubseg).
        let mut crosssubseg = m.tspivot(crosstri);
        // Check for a crossing segment.
        if crosssubseg.ss == DUMMY_SUB {
            return Ok(0);
        } else {
            // Insert a vertex at the intersection.
            segmentintersection(m, b, &mut crosstri, &mut crosssubseg, endpoint2)?;

            *searchtri = crosstri;
            insertsubseg(m, b, *searchtri, newmark)?;
            // Insert the remainder of the segment.
            return scoutsegment(m, b, searchtri, endpoint2, newmark);
        }
    }
}

// ---------------------------------------------------------------------------
// conformingedge
// ---------------------------------------------------------------------------

/// C: conformingedge (triangle.c:6089-6175). Force a segment into a
/// conforming Delaunay triangulation by inserting a vertex at its
/// midpoint, and recursively forcing in the two half-segments if
/// necessary. Generates a sequence of subsegments connecting endpoint1
/// to endpoint2; newmark is the boundary marker of the segment.
pub(crate) fn conformingedge(
    m: &mut Mesh,
    b: &Behavior,
    endpoint1: u32,
    endpoint2: u32,
    newmark: i32,
) -> Result<(), SegmentChainError> {
    // Create a new vertex to insert in the middle of the segment.
    let newvertex = m.vx_alloc()?;
    // Interpolate coordinates and attributes.
    for i in 0..(2 + m.nextras) as usize {
        m.vx_set_real(
            newvertex,
            i,
            0.5 * (m.vx_real(endpoint1, i) + m.vx_real(endpoint2, i)),
        );
    }
    m.vx_set_mark(newvertex, newmark);
    m.vx_set_type(newvertex, SEGMENTVERTEX);
    // No known triangle to search from.
    let mut searchtri1 = Otri {
        tri: DUMMY_TRI,
        orient: 0,
    };
    // Attempt to insert the new vertex.
    let mut success = insertvertex(m, b, newvertex, &mut searchtri1, None, false, false, false)?;
    if success == InsertVertexResult::DuplicateVertex {
        // Use the vertex that's already there.  (C writes it back into
        //   newvertex: org(searchtri1, newvertex); the value is
        //   consumed through searchtri1's origin below, so the port
        //   reads it there instead of keeping the dead local.)
        m.vx_dealloc(newvertex);
    } else {
        if success == InsertVertexResult::ViolatingVertex {
            // By fluke, we've landed right on another segment.  Split it.
            let mut brokensubseg = m.tspivot(searchtri1);
            success = insertvertex(
                m,
                b,
                newvertex,
                &mut searchtri1,
                Some(&mut brokensubseg),
                false,
                false,
                false,
            )?;
            if success != InsertVertexResult::SuccessfulVertex {
                // C: Internal error in conformingedge(): Failure to
                //   split a segment. (printf + internalerror(); the
                //   payload is internalerror()'s verbatim stdout text,
                //   constructed at the boundary conversion;
                //   triangle.c:349-354.)
                return Err(SegmentChainError::Internal);
            }
        }
        // The vertex has been inserted successfully.
        if m.steinerleft > 0 {
            m.steinerleft -= 1;
        }
    }
    let mut searchtri2 = searchtri1;
    // 'searchtri1' and 'searchtri2' are fastened at their origins to
    //   'newvertex', and will be directed toward 'endpoint1' and
    //   'endpoint2' respectively.  First, we must get 'searchtri2' out
    //   of the way so it won't be invalidated during the insertion of
    //   the first half of the segment.
    finddirection(
        m,
        b,
        &mut searchtri2,
        (m.vx_x(endpoint2), m.vx_y(endpoint2)),
    )?;
    if scoutsegment(m, b, &mut searchtri1, endpoint1, newmark)? == 0 {
        // The origin of searchtri1 may have changed if a collision with
        //   an intervening vertex on the segment occurred.
        let midvertex1 = m.org(searchtri1);
        conformingedge(m, b, midvertex1, endpoint1, newmark)?;
    }
    if scoutsegment(m, b, &mut searchtri2, endpoint2, newmark)? == 0 {
        // The origin of searchtri2 may have changed if a collision
        //   with an intervening vertex on the segment occurred.
        let midvertex2 = m.org(searchtri2);
        conformingedge(m, b, midvertex2, endpoint2, newmark)?;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// delaunayfixup
// ---------------------------------------------------------------------------

/// C: delaunayfixup (triangle.c:6201-6299). Enforce the Delaunay
/// condition at an edge, fanning out recursively from an existing
/// vertex. The origin of fixuptri is treated as if it has just been
/// inserted; the local Delaunay condition is enforced only in the
/// sector defined by fixuptri. 'leftside' indicates whether fixuptri is
/// to the left of the segment being inserted.
pub(crate) fn delaunayfixup(m: &mut Mesh, b: &Behavior, fixuptri: &mut Otri, leftside: bool) {
    let neartri = fixuptri.lnext();
    let mut fartri = m.sym(neartri);
    // Check if the edge opposite the origin of fixuptri can be flipped.
    if fartri.tri == DUMMY_TRI {
        return;
    }
    let faredge = m.tspivot(neartri);
    if faredge.ss != DUMMY_SUB {
        return;
    }
    // Find all the relevant vertices.
    let nearvertex = m.apex(neartri);
    let leftvertex = m.org(neartri);
    let rightvertex = m.dest(neartri);
    let farvertex = m.apex(fartri);
    // Check whether the previous polygon vertex is a reflex vertex.
    if leftside {
        if tri_counterclockwise(
            m,
            b,
            vertex_coords(m, nearvertex),
            vertex_coords(m, leftvertex),
            vertex_coords(m, farvertex),
        ) <= 0.0
        {
            // leftvertex is a reflex vertex too.  Nothing can be done
            //   until a convex section is found.
            return;
        }
    } else if tri_counterclockwise(
        m,
        b,
        vertex_coords(m, farvertex),
        vertex_coords(m, rightvertex),
        vertex_coords(m, nearvertex),
    ) <= 0.0
    {
        // rightvertex is a reflex vertex too.  Nothing can be done
        //   until a convex section is found.
        return;
    }
    if tri_counterclockwise(
        m,
        b,
        vertex_coords(m, rightvertex),
        vertex_coords(m, leftvertex),
        vertex_coords(m, farvertex),
    ) > 0.0
    {
        // fartri is not an inverted triangle, and farvertex is not a
        //   reflex vertex.  As there are no reflex vertices, fixuptri
        //   isn't an inverted triangle either.  Hence, test the edge
        //   between the triangles to ensure it is locally Delaunay.
        if tri_incircle(
            m,
            b,
            vertex_coords(m, leftvertex),
            vertex_coords(m, farvertex),
            vertex_coords(m, rightvertex),
            vertex_coords(m, nearvertex),
        ) <= 0.0
        {
            return;
        }
        // Not locally Delaunay; go on to an edge flip.
    } // else fartri is inverted; remove it from the stack by flipping.
    crate::transform::flip(m, b, neartri);
    fixuptri.lprevself(); // Restore the origin of fixuptri after the flip.
                          // Recursively process the two triangles that result from the flip.
    delaunayfixup(m, b, fixuptri, leftside);
    delaunayfixup(m, b, &mut fartri, leftside);
}

// ---------------------------------------------------------------------------
// constrainededge
// ---------------------------------------------------------------------------

/// C: constrainededge (triangle.c:6312-6413). Force a segment into a
/// constrained Delaunay triangulation by deleting the triangles it
/// intersects, and triangulating the polygons that form on each side of
/// it. 'starttri' has endpoint1 as origin; the segment connects endpoint1
/// to endpoint2; newmark is the boundary marker.
// C-parity: the block-head declarations are C's initializer list kept
// verbatim; each is assigned before use along C's control flow.
#[allow(clippy::needless_late_init)]
pub(crate) fn constrainededge(
    m: &mut Mesh,
    b: &Behavior,
    starttri: &mut Otri,
    endpoint2: u32,
    newmark: i32,
) -> Result<(), SegmentChainError> {
    let mut crosssubseg;
    let endpoint1;
    let mut farvertex;
    let mut collision;
    let mut done;
    let mut digs: i64;

    endpoint1 = m.org(*starttri);
    let mut fixuptri = starttri.lnext();
    crate::transform::flip(m, b, fixuptri);
    // 'collision' indicates whether we have found a vertex directly
    //   between endpoint1 and endpoint2.
    collision = false;
    done = false;
    digs = 0;
    loop {
        // Each pass removes an edge crossing the segment, so more
        //   passes than there are triangles means the dig is going in
        //   circles.
        digs += 1;
        if digs > m.triangles.live_count() as i64 {
            return Err(SegmentChainError::Recovery(SegmentRecoveryKind::Insert));
        }
        farvertex = m.org(fixuptri);
        if farvertex == 0 {
            // The dig has left the triangulation; the segment cannot be
            //   inserted. (C farvertex == NULL check; the zero id is
            //   C's NULL under D1.)
            return Err(SegmentChainError::Recovery(SegmentRecoveryKind::Insert));
        }
        // 'farvertex' is the extreme point of the polygon we are
        //   "digging" to get from endpoint1 to endpoint2.
        if (m.vx_x(farvertex) == m.vx_x(endpoint2)) && (m.vx_y(farvertex) == m.vx_y(endpoint2)) {
            let mut fixuptri2 = m.oprev(fixuptri);
            // Enforce the Delaunay condition around endpoint2.
            delaunayfixup(m, b, &mut fixuptri, false);
            delaunayfixup(m, b, &mut fixuptri2, true);
            done = true;
        } else {
            // Check whether farvertex is to the left or right of the
            //   segment being inserted, to decide which edge of fixuptri
            //   to dig through next.
            let area = tri_counterclockwise(
                m,
                b,
                vertex_coords(m, endpoint1),
                vertex_coords(m, endpoint2),
                vertex_coords(m, farvertex),
            );
            if area == 0.0 {
                // We've collided with a vertex between endpoint1 and
                //   endpoint2.
                collision = true;
                let mut fixuptri2 = m.oprev(fixuptri);
                // Enforce the Delaunay condition around farvertex.
                delaunayfixup(m, b, &mut fixuptri, false);
                delaunayfixup(m, b, &mut fixuptri2, true);
                done = true;
            } else {
                if area > 0.0 {
                    // farvertex is to the left of the segment.
                    let mut fixuptri2 = m.oprev(fixuptri);
                    // Enforce the Delaunay condition around farvertex,
                    //   on the left side of the segment only.
                    delaunayfixup(m, b, &mut fixuptri2, true);
                    // Flip the edge that crosses the segment.  After
                    //   the edge is flipped, one of its endpoints is
                    //   the fan vertex, and the destination of fixuptri
                    //   is the fan vertex.
                    fixuptri.lprevself();
                } else {
                    // farvertex is to the right of the segment.
                    // Enforce the Delaunay condition around farvertex,
                    //   on the right side of the segment only.
                    delaunayfixup(m, b, &mut fixuptri, false);
                    // Flip the edge that crosses the segment.  After
                    //   the edge is flipped, one of its endpoints is
                    //   the fan vertex, and the destination of fixuptri
                    //   is the fan vertex.
                    m.oprevself(&mut fixuptri);
                }
                // Check for two intersecting segments.
                crosssubseg = m.tspivot(fixuptri);
                if crosssubseg.ss == DUMMY_SUB {
                    crate::transform::flip(m, b, fixuptri); // May create inverted triangle at left.
                } else {
                    // We've collided with a segment between endpoint1
                    //   and endpoint2.
                    collision = true;
                    // Insert a vertex at the intersection.
                    segmentintersection(m, b, &mut fixuptri, &mut crosssubseg, endpoint2)?;
                    done = true;
                }
            }
        }
        if done {
            break;
        }
    }
    // Insert a subsegment to make the segment permanent.
    insertsubseg(m, b, fixuptri, newmark)?;
    // If there was a collision with an interceding vertex, install
    //   another segment connecting that vertex with endpoint2.
    if collision {
        // Insert the remainder of the segment.
        if scoutsegment(m, b, &mut fixuptri, endpoint2, newmark)? == 0 {
            constrainededge(m, b, &mut fixuptri, endpoint2, newmark)?;
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// insertsegment
// ---------------------------------------------------------------------------

/// C: insertsegment (triangle.c:6420-6514). Insert a PSLG segment into
/// a triangulation: scout from each endpoint, then force the segment in
/// either by recursive midpoint splitting (-D) or by the dig
/// (-p, constrainededge).
pub(crate) fn insertsegment(
    m: &mut Mesh,
    b: &Behavior,
    mut endpoint1: u32,
    endpoint2: u32,
    newmark: i32,
) -> Result<(), SegmentChainError> {
    let mut searchtri1 = Otri {
        tri: DUMMY_TRI,
        orient: 0,
    };

    m.scoutsteps = 0;

    // Find a triangle whose origin is the segment's first endpoint.
    let mut checkvertex = 0; // C (vertex) NULL.
    let encodedtri = m.vx_2tri(endpoint1);
    if encodedtri != 0 {
        searchtri1 = decode(encodedtri);
        checkvertex = m.org(searchtri1);
    }
    if checkvertex != endpoint1 {
        // Find a boundary triangle to search from.
        searchtri1 = Otri {
            tri: DUMMY_TRI,
            orient: 0,
        };
        m.symself(&mut searchtri1);
        // Search for the segment's first endpoint by point location.
        if locate(
            m,
            b,
            (m.vx_x(endpoint1), m.vx_y(endpoint1)),
            &mut searchtri1,
        ) != LocateResult::OnVertex
        {
            // C: Unable to locate PSLG vertex (endpoint1) in the
            //   triangulation.
            return Err(SegmentChainError::Recovery(SegmentRecoveryKind::Insert));
        }
    }
    // Remember this triangle to improve subsequent point location.
    m.recenttri = Some(searchtri1);
    // Scout the beginnings of a path from the first endpoint
    //   toward the second.
    if scoutsegment(m, b, &mut searchtri1, endpoint2, newmark)? != 0 {
        // The segment was easily inserted.
        return Ok(());
    }
    // The first endpoint may have changed if a collision with an
    //   intervening vertex on the segment occurred.
    endpoint1 = m.org(searchtri1);

    // Find a triangle whose origin is the segment's second endpoint.
    let mut searchtri2 = Otri {
        tri: DUMMY_TRI,
        orient: 0,
    };
    let mut checkvertex = 0; // C (vertex) NULL.
    let encodedtri = m.vx_2tri(endpoint2);
    if encodedtri != 0 {
        searchtri2 = decode(encodedtri);
        checkvertex = m.org(searchtri2);
    }
    if checkvertex != endpoint2 {
        // Find a boundary triangle to search from.
        searchtri2 = Otri {
            tri: DUMMY_TRI,
            orient: 0,
        };
        m.symself(&mut searchtri2);
        // Search for the segment's second endpoint by point location.
        if locate(
            m,
            b,
            (m.vx_x(endpoint2), m.vx_y(endpoint2)),
            &mut searchtri2,
        ) != LocateResult::OnVertex
        {
            // C: Unable to locate PSLG vertex (endpoint2) in the
            //   triangulation.
            return Err(SegmentChainError::Recovery(SegmentRecoveryKind::Insert));
        }
    }
    // Remember this triangle to improve subsequent point location.
    m.recenttri = Some(searchtri2);
    // Scout the beginnings of a path from the second endpoint
    //   toward the first.
    if scoutsegment(m, b, &mut searchtri2, endpoint1, newmark)? != 0 {
        // The segment was easily inserted.
        return Ok(());
    }
    // The second endpoint may have changed if a collision with an
    //   intervening vertex on the segment occurred.
    let endpoint2_moved = m.org(searchtri2);

    if b.splitseg {
        // Insert vertices to force the segment into the triangulation.
        conformingedge(m, b, endpoint1, endpoint2_moved, newmark)?;
    } else {
        // Insert the segment directly into the triangulation.
        constrainededge(m, b, &mut searchtri1, endpoint2_moved, newmark)?;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// markhull
// ---------------------------------------------------------------------------

/// C: markhull (triangle.c:6517-6552). Cover the convex hull of a
/// triangulation with subsegments.
pub(crate) fn markhull(m: &mut Mesh, b: &Behavior) -> Result<(), Oom> {
    // Find a triangle handle on the hull.
    let mut hulltri = Otri {
        tri: DUMMY_TRI,
        orient: 0,
    };
    m.symself(&mut hulltri);
    // Remember where we started so we know when to stop.
    let starttri = hulltri;
    // Go once counterclockwise around the convex hull.
    loop {
        // Create a subsegment if there isn't already one here.
        insertsubseg(m, b, hulltri, 1)?;
        // To find the next hull edge, go clockwise around the next
        //   vertex.
        hulltri.lnextself();
        let mut nexttri = m.oprev(hulltri);
        while nexttri.tri != DUMMY_TRI {
            hulltri = nexttri;
            nexttri = m.oprev(hulltri);
        }
        if hulltri == starttri {
            break;
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// formskeleton
// ---------------------------------------------------------------------------

/// C: formskeleton (triangle.c:6554-6633). Create the segments of a
/// triangulation, including PSLG segments and edges on the convex hull.
// C-parity: the markerlist index walk mirrors C's `for (i = 0; ...)`
// segment loop verbatim.
#[allow(clippy::needless_range_loop)]
pub(crate) fn formskeleton(
    m: &mut Mesh,
    b: &Behavior,
    segmentlist: &[i32],
    segmentmarkerlist: Option<&[i32]>,
    numberofsegments: i32,
) -> Result<(), Error> {
    if b.poly {
        m.insegments = numberofsegments;
        let segmentmarkers = segmentmarkerlist.is_some();
        let markerlist = segmentmarkerlist.unwrap_or(&[]);
        let mut index = 0usize;

        // If the input vertices are collinear, there is no triangulation,
        //   so don't try to insert segments.
        if m.triangles.live_count() == 0 {
            return Ok(());
        }

        // If segments are to be inserted, compute a mapping
        //   from vertices to triangles.
        if m.insegments > 0 {
            crate::quality::makevertexmap(m, b);
        }

        let mut boundmarker = 0i32;
        // Read and insert the segments.
        for i in 0..m.insegments as usize {
            let end1 = segmentlist[index];
            index += 1;
            let end2 = segmentlist[index];
            index += 1;
            if segmentmarkers {
                boundmarker = markerlist[i];
            }

            if (end1 < b.firstnumber) || (end1 >= b.firstnumber + m.invertices) {
                // C: Invalid first endpoint of segment (the _DEBUG
                //   printf compiles out of the oracle build; the silent
                //   skip is C's compiled behavior).
            } else if (end2 < b.firstnumber) || (end2 >= b.firstnumber + m.invertices) {
                // C: Invalid second endpoint of segment.
            } else {
                // Find the vertices numbered 'end1' and 'end2'.
                let endpoint1 = m.getvertex(b.firstnumber, end1);
                let endpoint2 = m.getvertex(b.firstnumber, end2);
                if (m.vx_x(endpoint1) == m.vx_x(endpoint2))
                    && (m.vx_y(endpoint1) == m.vx_y(endpoint2))
                {
                    // C: Endpoints of segment are coincident (the
                    //   _DEBUG printf compiles out; C skips silently).
                } else {
                    insertsegment(m, b, endpoint1, endpoint2, boundmarker)?;
                }
            }
        }
    } else {
        m.insegments = 0;
    }
    if b.convex || !b.poly {
        // Enclose the convex hull with subsegments.
        markhull(m, b)?;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// infecthull
// ---------------------------------------------------------------------------

/// C: infecthull (triangle.c:6642-6712). Virally infect all of the
/// triangles of the convex hull that are not protected by subsegments.
/// Where there are subsegments, set boundary markers as appropriate.
pub(crate) fn infecthull(m: &mut Mesh, _b: &Behavior) -> Result<(), Oom> {
    // Find a triangle handle on the hull.
    let mut hulltri = Otri {
        tri: DUMMY_TRI,
        orient: 0,
    };
    m.symself(&mut hulltri);
    // Remember where we started so we know when to stop.
    let starttri = hulltri;
    // Go once counterclockwise around the convex hull.
    loop {
        // Ignore triangles that are already infected.
        if !m.infected(hulltri) {
            // Is the triangle protected by a subsegment?
            let hullsubseg = m.tspivot(hulltri);
            if hullsubseg.ss == DUMMY_SUB {
                // The triangle is not protected; infect it.
                if !m.infected(hulltri) {
                    m.infect(hulltri);
                    m.viri_push_tri(hulltri.tri)?;
                }
            } else {
                // The triangle is protected; set boundary markers if
                //   appropriate.
                if m.sub_mark(hullsubseg.ss) == 0 {
                    m.set_sub_mark(hullsubseg.ss, 1);
                    let horg = m.org(hulltri);
                    let hdest = m.dest(hulltri);
                    if m.vx_mark(horg) == 0 {
                        m.vx_set_mark(horg, 1);
                    }
                    if m.vx_mark(hdest) == 0 {
                        m.vx_set_mark(hdest, 1);
                    }
                }
            }
        }
        // To find the next hull edge, go clockwise around the next
        //   vertex.
        hulltri.lnextself();
        let mut nexttri = m.oprev(hulltri);
        while nexttri.tri != DUMMY_TRI {
            hulltri = nexttri;
            nexttri = m.oprev(hulltri);
        }
        if hulltri == starttri {
            break;
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// plague
// ---------------------------------------------------------------------------

/// C: plague (triangle.c:6714-6865). Spread the virus from all infected
/// triangles to any neighbors not protected by subsegments, and delete
/// all infected triangles (the procedure that actually creates holes and
/// concavities). Operates in two phases: identify the dying triangles,
/// then eliminate them (including orphaned vertices).
pub(crate) fn plague(m: &mut Mesh, _b: &Behavior) -> Result<(), Oom> {
    // Phase 1: loop through all the infected triangles, spreading the
    //   virus to their neighbors, then to their neighbors' neighbors.
    let mut vt = m.viri_traversal();
    while let Some(virusrow) = m.viri_traverse(&mut vt) {
        let mut testtri = Otri {
            tri: m.viri_tri(virusrow),
            orient: 0,
        };
        // A triangle is marked as infected by messing with one of its
        //   pointers to subsegments, setting it to an illegal value.
        //   Hence, we have to temporarily uninfect this triangle so
        //   that we can examine its adjacent subsegments.
        m.uninfect(testtri);

        // Check each of the triangle's three neighbors.
        for orient in 0..3u8 {
            testtri.orient = orient;
            // Find the neighbor.
            let neighbor = m.sym(testtri);
            // Check for a subsegment between the triangle and its
            //   neighbor.
            let neighborsubseg = m.tspivot(testtri);
            // Check if the neighbor is nonexistent or already infected.
            if neighbor.tri == DUMMY_TRI || m.infected(neighbor) {
                if neighborsubseg.ss != DUMMY_SUB {
                    // There is a subsegment separating the triangle
                    //   from its neighbor, but both triangles are
                    //   dying, so the subsegment dies too.
                    m.sub_dealloc(neighborsubseg.ss);
                    if neighbor.tri != DUMMY_TRI {
                        // Make sure the subsegment doesn't get
                        //   deallocated again later when the infected
                        //   neighbor is visited.
                        m.uninfect(neighbor);
                        m.tsdissolve(neighbor);
                        m.infect(neighbor);
                    }
                }
            } else {
                // The neighbor exists and is not infected.
                if neighborsubseg.ss == DUMMY_SUB {
                    // There is no subsegment protecting the neighbor,
                    //   so the neighbor becomes infected.
                    m.infect(neighbor);
                    // Ensure that the neighbor's neighbors will be
                    //   infected. (The live frontier read extends the
                    //   traversal exactly like C's traverse.)
                    m.viri_push_tri(neighbor.tri)?;
                } else {
                    // The neighbor is protected by a subsegment.
                    // Remove this triangle from the subsegment.
                    m.stdissolve(neighborsubseg);
                    // The subsegment becomes a boundary.  Set markers
                    //   accordingly.
                    if m.sub_mark(neighborsubseg.ss) == 0 {
                        m.set_sub_mark(neighborsubseg.ss, 1);
                    }
                    let norg = m.org(neighbor);
                    let ndest = m.dest(neighbor);
                    if m.vx_mark(norg) == 0 {
                        m.vx_set_mark(norg, 1);
                    }
                    if m.vx_mark(ndest) == 0 {
                        m.vx_set_mark(ndest, 1);
                    }
                }
            }
        }
        // Remark the triangle as infected, so it doesn't get added to
        //   the virus pool again.
        m.infect(testtri);
    }

    // Phase 2: eliminate the infected triangles and orphaned vertices.
    let mut vt = m.viri_traversal();
    while let Some(virusrow) = m.viri_traverse(&mut vt) {
        let mut testtri = Otri {
            tri: m.viri_tri(virusrow),
            orient: 0,
        };

        // Check each of the three corners of the triangle for
        //   elimination.  This is done by walking around each vertex,
        //   checking if it is still connected to at least one live
        //   triangle.
        for orient in 0..3u8 {
            testtri.orient = orient;
            let testvertex = m.org(testtri);
            // Check if the vertex has already been tested. (C NULL
            //   vertex = the zero id under D1.)
            if testvertex != 0 {
                let mut killorg = true;
                // Mark the corner of the triangle as having been tested.
                m.setorg(testtri, 0);
                // Walk counterclockwise about the vertex.
                let mut neighbor = m.onext(testtri);
                // Stop upon reaching a boundary or the starting triangle.
                while neighbor.tri != DUMMY_TRI && neighbor != testtri {
                    if m.infected(neighbor) {
                        // Mark the corner of this triangle as having
                        //   been tested.
                        m.setorg(neighbor, 0);
                    } else {
                        // A live triangle.  The vertex survives.
                        killorg = false;
                    }
                    // Walk counterclockwise about the vertex.
                    m.onextself(&mut neighbor);
                }
                // If we reached a boundary, we must walk clockwise as
                //   well.
                if neighbor.tri == DUMMY_TRI {
                    // Walk clockwise about the vertex.
                    let mut neighbor = m.oprev(testtri);
                    // Stop upon reaching a boundary.
                    while neighbor.tri != DUMMY_TRI {
                        if m.infected(neighbor) {
                            // Mark the corner of this triangle as having
                            //   been tested.
                            m.setorg(neighbor, 0);
                        } else {
                            // A live triangle.  The vertex survives.
                            killorg = false;
                        }
                        // Walk clockwise about the vertex.
                        m.oprevself(&mut neighbor);
                    }
                }
                if killorg {
                    m.vx_set_type(testvertex, UNDEADVERTEX);
                    m.undeads += 1;
                }
            }
        }

        // Record changes in the number of boundary edges, and
        //   disconnect dead triangles from their neighbors.
        for orient in 0..3u8 {
            testtri.orient = orient;
            let neighbor = m.sym(testtri);
            if neighbor.tri == DUMMY_TRI {
                // There is no neighboring triangle on this edge, so
                //   this edge is a boundary edge.  This triangle is
                //   being deleted, so this boundary edge is deleted.
                m.hullsize -= 1;
            } else {
                // Disconnect the triangle from its neighbor.
                m.dissolve(neighbor);
                // There is a neighboring triangle on this edge, so
                //   this edge becomes a boundary edge when this
                //   triangle is deleted.
                m.hullsize += 1;
            }
        }
        // Return the dead triangle to the pool of triangles.
        m.tri_dealloc(testtri.tri);
    }
    // Empty the virus pool.
    m.viri_restart();
    Ok(())
}

// ---------------------------------------------------------------------------
// regionplague
// ---------------------------------------------------------------------------

/// C: regionplague (triangle.c:6886-6950). Spread regional attributes
/// and/or area constraints (from a .poly file) throughout a
/// segment-bounded region: infect, spread, uninfect.
pub(crate) fn regionplague(
    m: &mut Mesh,
    b: &Behavior,
    attribute: f64,
    area: f64,
) -> Result<(), Oom> {
    // Phase 1: loop through all the infected triangles, spreading the
    //   attribute and/or area constraint to their neighbors, then to
    //   their neighbors' neighbors.
    let mut vt = m.viri_traversal();
    while let Some(virusrow) = m.viri_traverse(&mut vt) {
        let mut testtri = Otri {
            tri: m.viri_tri(virusrow),
            orient: 0,
        };
        // Temporarily uninfect this triangle so that we can examine its
        //   adjacent subsegments (C's infected-bit mess).
        m.uninfect(testtri);
        if b.regionattrib {
            // Set an attribute: C's setelemattribute(testtri, m->eextras,
            //   attribute) macro stores past the eextras count (the
            //   regionattrib column allocated by initializetrisubpools);
            //   mirrored by the same direct slot store.
            m.set_tri_real(
                testtri.tri,
                m.tri_shape.elemattribindex + m.eextras as usize,
                attribute,
            );
        }
        if b.vararea {
            // Set an area constraint.
            m.set_tri_areabound(testtri.tri, area);
        }
        // Check each of the triangle's three neighbors.
        for orient in 0..3u8 {
            testtri.orient = orient;
            // Find the neighbor.
            let neighbor = m.sym(testtri);
            // Check for a subsegment between the triangle and its
            //   neighbor.
            let neighborsubseg = m.tspivot(testtri);
            // Make sure the neighbor exists, is not already infected,
            //   and isn't protected by a subsegment.
            if neighbor.tri != DUMMY_TRI && !m.infected(neighbor) && neighborsubseg.ss == DUMMY_SUB
            {
                // Infect the neighbor.
                m.infect(neighbor);
                // Ensure that the neighbor's neighbors will be infected.
                m.viri_push_tri(neighbor.tri)?;
            }
        }
        // Remark the triangle as infected, so it doesn't get added to
        //   the virus pool again.
        m.infect(testtri);
    }

    // Phase 2: uninfect all triangles.
    let mut vt = m.viri_traversal();
    while let Some(virusrow) = m.viri_traverse(&mut vt) {
        // C: testtri.tri = *virusloop; orient never read.
        let testtri = Otri {
            tri: m.viri_tri(virusrow),
            orient: 0,
        };
        m.uninfect(testtri);
    }
    // Empty the virus pool.
    m.viri_restart();
    Ok(())
}

// ---------------------------------------------------------------------------
// carveholes
// ---------------------------------------------------------------------------

/// C: carveholes (triangle.c:6964-7105). Find the holes and infect
/// them.  Find the area constraints and infect them.  Infect the convex
/// hull.  Spread the infection and kill triangles.  Spread the area
/// constraints. (The C flat holelist/regionlist strides mirror to the
/// typed Hole/Region records of the frozen input surface; the counts
/// derive from the record lengths.)
pub(crate) fn carveholes(
    m: &mut Mesh,
    b: &Behavior,
    holelist: &[crate::input::Hole],
    regionlist: &[crate::input::Region],
) -> Result<(), Oom> {
    let holes = holelist.len() as i32;
    let regions = regionlist.len() as i32;

    // Allocate storage for the triangles in which region points fall
    //   (C trimalloc'd scratch: the owned local arena rule; the entries
    //   are the dummy handle, the deterministic superset of C's
    //   uninitialized rows that are only read after the .tri != NULL
    //   filter or as full otricopy results).
    let mut regiontris: Vec<Otri> = Vec::new();
    if regions > 0 {
        regiontris
            .try_reserve_exact(regions as usize)
            .map_err(|_| Oom)?;
        for _ in 0..regions {
            regiontris.push(Otri {
                tri: DUMMY_TRI,
                orient: 0,
            });
        }
    }
    // C: poolinit(&m->viri ... 6988) - the pool is eagerly initialized
    //   in Mesh::new; its items are zero here and poolrestart keeps it
    //   empty between runs, matching C's fresh pool.

    if !b.convex {
        // Mark as infected any unprotected triangles on the boundary.
        //   This is one way by which concavities are created.
        infecthull(m, b)?;
    }

    if holes > 0 && !b.noholes {
        // Infect each triangle in which a hole lies.
        for hole in holelist.iter().take(holes as usize) {
            let hole_v: Vertex = [hole.x, hole.y, 0.0];
            // Ignore holes that aren't within the bounds of the mesh.
            if (hole.x >= m.xmin) && (hole.x <= m.xmax) && (hole.y >= m.ymin) && (hole.y <= m.ymax)
            {
                // Start searching from some triangle on the outer boundary.
                let mut searchtri = Otri {
                    tri: DUMMY_TRI,
                    orient: 0,
                };
                m.symself(&mut searchtri);
                // Ensure that the hole is to the left of this boundary
                //   edge; otherwise, locate() will falsely report that
                //   the hole falls within the starting triangle.
                let searchorg = m.org(searchtri);
                let searchdest = m.dest(searchtri);
                if tri_counterclockwise(
                    m,
                    b,
                    vertex_coords(m, searchorg),
                    vertex_coords(m, searchdest),
                    hole_v,
                ) > 0.0
                {
                    // Find a triangle that contains the hole.
                    let intersect = locate(m, b, (hole.x, hole.y), &mut searchtri);
                    if intersect != LocateResult::Outside && !m.infected(searchtri) {
                        // Infect the triangle.  This is done by marking
                        //   the triangle as infected and including the
                        //   triangle in the virus pool.
                        m.infect(searchtri);
                        m.viri_push_tri(searchtri.tri)?;
                    }
                }
            }
        }
    }

    // Now, we have to find all the regions BEFORE we carve the holes,
    //   because locate() won't work when the triangulation is no longer
    //   convex.  (Incidentally, this is the reason why regional
    //   attributes and area constraints can't be used when refining a
    //   preexisting mesh, which might not be convex; they can only be
    //   used with a freshly triangulated PSLG.)
    if regions > 0 {
        // Find the starting triangle for each region.
        for i in 0..regions as usize {
            regiontris[i] = Otri {
                tri: DUMMY_TRI,
                orient: 0,
            };
            let region = &regionlist[i];
            let region_v: Vertex = [region.x, region.y, 0.0];
            // Ignore region points that aren't within the bounds of the
            //   mesh.
            if (region.x >= m.xmin)
                && (region.x <= m.xmax)
                && (region.y >= m.ymin)
                && (region.y <= m.ymax)
            {
                // Start searching from some triangle on the outer boundary.
                let mut searchtri = Otri {
                    tri: DUMMY_TRI,
                    orient: 0,
                };
                m.symself(&mut searchtri);
                // Ensure that the region point is to the left of this
                //   boundary edge; otherwise, locate() will falsely
                //   report that the region point falls within the
                //   starting triangle.
                let searchorg = m.org(searchtri);
                let searchdest = m.dest(searchtri);
                if tri_counterclockwise(
                    m,
                    b,
                    vertex_coords(m, searchorg),
                    vertex_coords(m, searchdest),
                    region_v,
                ) > 0.0
                {
                    // Find a triangle that contains the region point.
                    let intersect = locate(m, b, (region.x, region.y), &mut searchtri);
                    if intersect != LocateResult::Outside && !m.infected(searchtri) {
                        // Record the triangle for processing after the
                        //   holes have been carved.
                        regiontris[i] = searchtri;
                    }
                }
            }
        }
    }

    if m.viri.live_count() > 0 {
        // Carve the holes and concavities.
        plague(m, b)?;
    }
    // The virus pool should be empty now.

    if regions > 0 {
        if b.regionattrib && !b.refine {
            // Assign every triangle a regional attribute of zero.
            //   (Same direct slot store as regionplague.)
            let mut t = m.triangles.traversal();
            while let Some(tri) = m.triangletraverse(&mut t) {
                m.set_tri_real(tri, m.tri_shape.elemattribindex + m.eextras as usize, 0.0);
            }
        }
        for i in 0..regions as usize {
            if regiontris[i].tri != DUMMY_TRI {
                // Make sure the triangle under consideration still
                //   exists.  It may have been eaten by the virus.
                if !m.deadtri(regiontris[i].tri) {
                    // Put one triangle in the virus pool.
                    m.infect(regiontris[i]);
                    m.viri_push_tri(regiontris[i].tri)?;
                    // Apply one region's attribute and/or area
                    //   constraint.
                    regionplague(m, b, regionlist[i].attribute, regionlist[i].max_area)?;
                    // The virus pool should be empty now.
                }
            }
        }
        if b.regionattrib && !b.refine {
            // Note the fact that each triangle has an additional
            //   attribute.
            m.eextras += 1;
        }
    }

    // C: pooldeinit(&m->viri) and trifree(regiontris) - the D11
    //   truncation (the run ends with the Mesh drop; the splay-pool
    //   precedent).
    Ok(())
}

// ---------------------------------------------------------------------------
// Tests (hand-built configurations, D10-deterministic; every expectation is
// derived by hand from the C semantics at the exact call sites, never by
// probing the C oracle).
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::delaunay::delaunay;
    use crate::handles::NULL;
    use crate::input::Hole;

    /// The Behavior for the test at hand, composed exactly like the
    /// landed modules' test helpers.
    fn behavior(poly: bool) -> Behavior {
        Behavior {
            poly,
            usesegments: poly,
            ..Behavior::default_form()
        }
    }

    /// A Behavior the -r reconstruct path runs under: refine implies
    /// usesegments even without -p (the raw delaunay rows must carry
    /// the subseg slots reconstruct's vertex-stack pun borrows).
    fn behavior_refine() -> Behavior {
        Behavior {
            refine: true,
            usesegments: true,
            ..Behavior::default_form()
        }
    }

    /// A vertex row with real coordinates (the fan-builder pattern of
    /// the landed tests; vx ids start at 1, D1).
    fn vxy(m: &mut Mesh, x: f64, y: f64) -> u32 {
        let v = m.vx_alloc().expect("vertex alloc");
        m.set_vx_x(v, x);
        m.set_vx_y(v, y);
        v
    }

    /// The transform.rs pair: t1 = {org a, dest b, apex c} and
    /// t2 = {org b, dest a, apex d} bonded across the shared edge a-b
    /// (c and d on opposite sides), the smallest well-bonded poly mesh.
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

    /// A flat square of 4 input vertices, triangulated by the landed
    /// delaunay() dispatcher (divconq; deterministic under D10), with
    /// the api's post-delaunay state mirrored by hand (hullsize, the
    /// infvertex nulls, checksegments under usesegments).
    fn delaunay_square(poly: bool) -> (Mesh, Behavior) {
        let b = behavior(poly);
        let mut m = Mesh::new(&b, 4, 2, 0, 0).expect("mesh init");
        vxy(&mut m, 0.0, 0.0); // vertex 1 (A)
        vxy(&mut m, 1.0, 0.0); // vertex 2 (B)
        vxy(&mut m, 1.0, 1.0); // vertex 3 (C)
        vxy(&mut m, 0.0, 1.0); // vertex 4 (D)
        let hulledges = delaunay(&mut m, &b).expect("delaunay");
        assert_eq!(hulledges, 4); // the square's boundary
                                  // triangle_api.c:208-217: hullsize from the delaunay result, the
                                  // three infvertex nulls, checksegments under usesegments.
        m.hullsize = hulledges;
        m.infvertex1 = NULL;
        m.infvertex2 = NULL;
        m.infvertex3 = NULL;
        if b.usesegments {
            m.checksegments = true;
        }
        (m, b)
    }

    /// The live vertex row with the exact given coordinates, if any.
    fn vertex_at(m: &Mesh, x: f64, y: f64) -> Option<u32> {
        let mut vt = m.vertices.traversal();
        while let Some(v) = m.vertices.traverse_next(&mut vt) {
            if m.vx_type(v) != DEADVERTEX && m.vx_x(v) == x && m.vx_y(v) == y {
                return Some(v);
            }
        }
        None
    }

    /// Number of mesh edges facing outer space (sym across them is the
    /// dummy triangle), the loop invariant behind hullsize.
    fn boundary_edges(m: &Mesh) -> u64 {
        let mut count = 0u64;
        let mut t = m.triangles.traversal();
        while let Some(tri) = m.triangles.traverse_next(&mut t) {
            if m.deadtri(tri) {
                continue;
            }
            for orient in 0..3u8 {
                if m.sym(Otri { tri, orient }).tri == DUMMY_TRI {
                    count += 1;
                }
            }
        }
        count
    }

    /// The live subsegment rows (id list) for pool accounting asserts.
    fn live_subsegs(m: &Mesh) -> Vec<u32> {
        let mut subs = vec![];
        let mut st = m.subsegs.traversal();
        while let Some(ss) = m.subsegs.traverse_next(&mut st) {
            if !m.deadsubseg(ss) {
                subs.push(ss);
            }
        }
        subs
    }

    /// The live triangle rows.
    fn live_tris(m: &Mesh) -> Vec<u32> {
        let mut tris = vec![];
        let mut t = m.triangles.traversal();
        while let Some(tri) = m.triangles.traverse_next(&mut t) {
            if !m.deadtri(tri) {
                tris.push(tri);
            }
        }
        tris
    }

    #[test]
    fn finddirectionresult_discriminants_are_the_c_values() {
        assert_eq!(FindDirectionResult::Within as i32, 0); // C WITHIN
        assert_eq!(FindDirectionResult::LeftCollinear as i32, 1); // C LEFTCOLLINEAR
        assert_eq!(FindDirectionResult::RightCollinear as i32, 2); // C RIGHTCOLLINEAR
    }

    /// The diamond pair a(0,0), b(1,0), c(0,1), d(0.5,-1): t1@0 has org
    /// a, dest b, apex c. All predicate signs below are hand-derived
    /// from tri_counterclockwise's formula at the exact call sites.
    #[test]
    fn finddirection_covers_all_result_cases() {
        let mut m = Mesh::new(&behavior(true), 8, 2, 0, 0).expect("mesh init");
        let a = vxy(&mut m, 0.0, 0.0);
        let b = vxy(&mut m, 1.0, 0.0);
        let c = vxy(&mut m, 0.0, 1.0);
        let d = vxy(&mut m, 0.5, -1.0);
        let (t1, t2) = pair(&mut m, true, a, b, c, d);
        let t1_0 = Otri {
            tri: t1.tri,
            orient: 0,
        };
        let t2_1 = Otri {
            tri: t2.tri,
            orient: 1,
        };
        let bb = behavior(true);

        // P(0.2, 0.2) strictly inside t1's cone at a:
        // ccw(P,a,c) = -0.2 < 0 (leftflag false),
        // ccw(a,P,b) = -0.2 < 0 (rightflag false) -> no walk, WITHIN.
        let mut st = t1_0;
        assert_eq!(
            finddirection(&mut m, &bb, &mut st, (0.2, 0.2)).expect("finddirection"),
            FindDirectionResult::Within
        );

        // P(2, 0) collinear through dest b: ccw(a,P,b) = 0 -> RIGHTCOLLINEAR.
        let mut st = t1_0;
        assert_eq!(
            finddirection(&mut m, &bb, &mut st, (2.0, 0.0)).expect("finddirection"),
            FindDirectionResult::RightCollinear
        );

        // P(0, 2) collinear through apex c: ccw(P,a,c) = 0 -> LEFTCOLLINEAR.
        let mut st = t1_0;
        assert_eq!(
            finddirection(&mut m, &bb, &mut st, (0.0, 2.0)).expect("finddirection"),
            FindDirectionResult::LeftCollinear
        );

        // P(0.5, -0.1) below the shared edge; rightflag true once
        // (ccw(a,P,b) = 0.1 > 0), one oprevself turn lands on t2@1
        // (org a, dest d: ccw(a,P,d) = -0.45 < 0) -> WITHIN after the turn.
        let mut st = t1_0;
        assert_eq!(
            finddirection(&mut m, &bb, &mut st, (0.5, -0.1)).expect("finddirection"),
            FindDirectionResult::Within
        );
        assert_eq!(st, t2_1);
        assert_eq!(m.org(st), a); // the origin never changes

        // P(-0.5, 0.1) outside the fan beyond the hull edge a-c of t1:
        // both flags on, the onext tie-break keeps the left walk, which
        // walks past t1 into outer space: TRI_FIND_DIRECTION.
        // The walk raises SegmentChainError::Recovery(FindDirection)
        // at the site (the boundary maps it to the public
        // Error::SegmentRecovery) - the outcome pin replaces the
        // *status pin.
        let mut st = t2_1;
        assert!(matches!(
            finddirection(&mut m, &bb, &mut st, (-0.5, 0.1)),
            Err(SegmentChainError::Recovery(
                SegmentRecoveryKind::FindDirection
            ))
        ));
    }

    #[test]
    fn finddirection_finds_then_evaluates_on_a_larger_fan() {
        // Three triangles around the origin: a full-circle fan (no hull
        // edges), the walk from any of them finds any searchpoint by
        // turning. Fan: center(0,0), rim at 1/n-scaled angles, hand-bonded
        // ring (the locate.rs fan pattern, with -p rows).
        let b = behavior(true);
        let mut m = Mesh::new(&b, 9, 2, 0, 0).expect("mesh init");
        let center = vxy(&mut m, 0.0, 0.0);
        let n = 3;
        let mut rim = vec![];
        for k in 0..n {
            let ang = 2.0 * std::f64::consts::PI * (k as f64) / (n as f64);
            rim.push(vxy(&mut m, ang.cos(), ang.sin()));
        }
        let mut tris = vec![];
        for k in 0..n as usize {
            let t = m.maketriangle(true, false).expect("tri");
            m.setorg(t, center);
            m.setdest(t, rim[k]);
            m.setapex(t, rim[(k + 1) % n as usize]);
            tris.push(t);
        }
        for k in 0..n as usize {
            // fan[k] = {center, rim[k], rim[k+1]}: its rim[k]->rim[k+1]
            // edge at orient 1 pairs with fan[k+1]'s orient-2 edge
            // (rim[k+1] -> rim[k+2]?? no: the shared edge is
            // center->rim[k+1]: fan[k]'s orient-2 (rim[k+1]->center)... bond
            // the center edges: fan[k]@0 (center->rim[k]) with
            // fan[(k+2)%n]@2 (rim[k]->center), exactly the two halves of
            // edge center-rim[k].
            let a = Otri {
                tri: tris[k].tri,
                orient: 0,
            };
            let half = (k + n as usize - 1) % n as usize;
            let bb2 = Otri {
                tri: tris[half].tri,
                orient: 2,
            };
            m.bond(a, bb2);
        }
        let tris0 = (0..n as usize).map(|k| Otri {
            tri: tris[k].tri,
            orient: 0,
        });
        for start in tris0 {
            // any searchpoint strictly inside the fan polygon turns and
            // settles WITHIN, origin = center preserved.
            let mut st = start;
            let res = finddirection(&mut m, &b, &mut st, (0.0, 0.7)).expect("finddirection");
            // the walk rotates around center; (0,0.7) lies between the
            // 90-degree rim spokes... rim[1] is at 120 degrees,
            // rim[2] at 240; the segment from each org heads there or
            // terminates; in all cases no error and origin kept.
            assert_eq!(m.org(st), center);
            assert!(matches!(
                res,
                FindDirectionResult::Within
                    | FindDirectionResult::LeftCollinear
                    | FindDirectionResult::RightCollinear
            ));
        }
    }

    /// segmentintersection around the subsegmented shared edge a-b of
    /// the diamond pair (t1 = {org a, dest b, apex c(1,1)}); the headed
    /// segment runs apex c -> d(0.5,-1) and must cross a-b at the
    /// binary-exact point (0.75, 0):
    /// tx=1, ty=0, ex=-0.5, ey=-2 -> denom = ty*ex - tx*ey = 2;
    /// etx = a - d = (-0.5, 1), ety = 1;
    /// split = (ey*etx - ex*ety)/denom = ((-2)(-0.5) - (-0.5)(1))/2 = 0.75.
    #[test]
    fn segmentintersection_splits_at_hand_derived_point() {
        let b = Behavior {
            steiner: 5,
            ..behavior(true)
        };
        let mut m = Mesh::new(&b, 8, 2, 0, 0).expect("mesh init");
        let a = vxy(&mut m, 0.0, 0.0);
        let bpt = vxy(&mut m, 1.0, 0.0);
        let c = vxy(&mut m, 1.0, 1.0);
        let d = vxy(&mut m, 0.5, -1.0);
        let (t1, _t2) = pair(&mut m, true, a, bpt, c, d);
        m.checksegments = true;
        let shared = Otri {
            tri: t1.tri,
            orient: 0,
        };
        insertsubseg(&mut m, &b, shared, 7).expect("insertsubseg");
        assert_eq!(m.subsegs.live_count(), 1);

        let mut splittri = shared;
        let mut splitsubseg = m.tspivot(splittri);
        assert_ne!(splitsubseg.ss, DUMMY_SUB);
        segmentintersection(&mut m, &b, &mut splittri, &mut splitsubseg, d)
            .expect("segmentintersection");

        // the new vertex: torg + split * (tdest - torg) = (0.75, 0), exact
        // in binary; its vertex2tri was recorded and steinerleft charged.
        // C copies the split subsegment's mark onto it (mark 7).
        let nx = vertex_at(&m, 0.75, 0.0).expect("split vertex");
        assert_ne!(m.vx_2tri(nx), 0);
        assert_eq!(m.steinerleft, 4); // 5 -> 4
        assert_eq!(m.vx_type(nx), INPUTVERTEX);
        assert_eq!(m.vx_mark(nx), 7);
        // the subsegment splits into the two halves {a, n} and {n, b},
        // both newvertex-org'ed by the endpoint-correction loops (C:
        // setsegorg(..., newvertex) around both pivot chains).
        assert_eq!(m.subsegs.live_count(), 2);
        let mut halves = vec![];
        for ss in live_subsegs(&m) {
            let sub = Osub { ss, ssorient: 0 };
            let (o, dd) = (m.segorg(sub), m.segdest(sub));
            assert!(o == nx || dd == nx);
            halves.push((o.min(dd), o.max(dd)));
        }
        halves.sort();
        assert_eq!(
            halves,
            vec![(a.min(nx), a.max(nx)), (bpt.min(nx), bpt.max(nx))]
        );
        // both triangles of the pair split along the shared edge: 2 -> 4.
        assert_eq!(m.triangles.live_count(), 4);
        // contract: splittri's origin is the new vertex, its destination
        // endpoint1 (the old apex c).
        assert_eq!(m.org(splittri), nx);
        assert_eq!(m.dest(splittri), c);
    }

    #[test]
    fn segmentintersection_parallel_and_offsegment_short_paths() {
        // parallel: the headed segment apex c -> (2,1) runs parallel to
        // the split edge a-b: denom = ty*ex - tx*ey = 0*2 - 1*0 = 0 ->
        // TRI_SEG_INTERSECT, no alloc.
        {
            let mut m = Mesh::new(&behavior(true), 8, 2, 0, 0).expect("mesh init");
            let b = behavior(true);
            let a = vxy(&mut m, 0.0, 0.0);
            let bb = vxy(&mut m, 1.0, 0.0);
            let c = vxy(&mut m, 0.0, 1.0);
            let d = vxy(&mut m, 0.5, -1.0);
            let far = vxy(&mut m, 2.0, 1.0);
            let (t1, _t2) = pair(&mut m, true, a, bb, c, d);
            m.checksegments = true;
            let shared = Otri {
                tri: t1.tri,
                orient: 0,
            };
            insertsubseg(&mut m, &b, shared, 7).expect("insertsubseg");
            let mut splittri = shared;
            let mut splitsubseg = m.tspivot(splittri);
            let before = m.vertices.live_count();
            // The parallel/off-segment failures raise
            // SegmentChainError::Recovery(Intersect) at the site - the
            // outcome pin replaces the *status pin.
            assert!(matches!(
                segmentintersection(&mut m, &b, &mut splittri, &mut splitsubseg, far),
                Err(SegmentChainError::Recovery(SegmentRecoveryKind::Intersect))
            ));
            assert_eq!(m.vertices.live_count(), before);
        }
        // off-segment crossing: apex c(0,1) -> (-1,-1) crosses the
        // carrier line of the split edge a-b left of org a:
        // ex = -1, ey = -2, denom = 0 - 1*(-2) = 2;
        // etx = a - far = (1, 1), ety = 1;
        // split = ((-2)(1) - (-1)(1))/2 = -0.5, outside
        // [-1e-9, 1 + 1e-9] -> TRI_SEG_INTERSECT before any allocation.
        {
            let mut m = Mesh::new(&behavior(true), 9, 2, 0, 0).expect("mesh init");
            let b = behavior(true);
            let a = vxy(&mut m, 0.0, 0.0);
            let bb = vxy(&mut m, 1.0, 0.0);
            let c = vxy(&mut m, 0.0, 1.0);
            let d = vxy(&mut m, 0.5, -1.0);
            let far = vxy(&mut m, -1.0, -1.0);
            let (t1, _t2) = pair(&mut m, true, a, bb, c, d);
            m.checksegments = true;
            let shared = Otri {
                tri: t1.tri,
                orient: 0,
            };
            insertsubseg(&mut m, &b, shared, 7).expect("insertsubseg");
            let mut splittri = shared;
            let mut splitsubseg = m.tspivot(splittri);
            let before = m.vertices.live_count();
            let before_subs = m.subsegs.live_count();
            // The parallel/off-segment failures raise
            // SegmentChainError::Recovery(Intersect) at the site - the
            // outcome pin replaces the *status pin.
            assert!(matches!(
                segmentintersection(&mut m, &b, &mut splittri, &mut splitsubseg, far),
                Err(SegmentChainError::Recovery(SegmentRecoveryKind::Intersect))
            ));
            assert_eq!(m.vertices.live_count(), before);
            assert_eq!(m.subsegs.live_count(), before_subs);
        }
    }

    /// insertsegment against a hull edge and the interior diagonal of
    /// the diamond pair: both are mesh edges already, so scoutsegment
    /// installs the subsegment directly (C return 1 branch).
    #[test]
    fn insertsegment_hull_edge_and_interior_diagonal() {
        let mut m = Mesh::new(&behavior(true), 8, 2, 0, 0).expect("mesh init");
        let b = behavior(true);
        let a = vxy(&mut m, 0.0, 0.0);
        let bb = vxy(&mut m, 1.0, 0.0);
        let c = vxy(&mut m, 0.0, 1.0);
        let d = vxy(&mut m, 0.5, -1.0);
        pair(&mut m, true, a, bb, c, d);
        // formskeleton's C flow: makevertexmap supplies insertsegment's
        // vertex2tri start triangles on every PSLG mesh.
        crate::quality::makevertexmap(&mut m, &b);
        m.checksegments = true;

        // hull edge a-c (t1's orient-2 rim edge): subsegment installed by
        // the first scoutsegment, marks propagated to the endpoints.
        insertsegment(&mut m, &b, a, c, 6).expect("insertsegment");
        assert_eq!(m.subsegs.live_count(), 1);
        assert_eq!(m.vx_mark(a), 6);
        assert_eq!(m.vx_mark(c), 6);
        let seg1 = live_subsegs(&m)[0];
        assert_eq!(m.sub_mark(seg1), 6);
        // the subsegment bonds the hull edge from its one mesh side.
        let s = Osub {
            ss: seg1,
            ssorient: 0,
        };
        let sites: Vec<(u32, u32)> = vec![(m.segorg(s), m.segdest(s))];
        assert!(sites == vec![(c, a)] || sites == vec![(a, c)]);

        // interior diagonal a-b: the shared edge of the pair; a second
        // direct install from BOTH endpoints' scouts is prevented by the
        // existing-subsegment check in insertsubseg, and scoutsegment's
        // second endpoint also just installs/keeps one subsegment.
        insertsegment(&mut m, &b, a, bb, 5).expect("insertsegment");
        assert_eq!(m.subsegs.live_count(), 2);
        assert_eq!(m.triangles.live_count(), 2); // no dig, no split
                                                 // recenttri carries the endpoint2 locate result (the api's
                                                 // subsequent point-location mirror).
        assert!(m.recenttri.is_some());

        // both segments' vertex marks: a was marked 6 already (kept),
        // b gets 5 (mark 0 -> 5 by insertsubseg).
        assert_eq!(m.vx_mark(bb), 5);
    }

    /// The crossing case: segment c -> d must cross the shared edge a-b;
    /// with -D (splitseg) conformingedge inserts the binary-exact
    /// midpoint (0.25, 0) ON the shared edge, then scouts both halves.
    #[test]
    fn conformingedge_splitseg_walk() {
        let b = Behavior {
            splitseg: true,
            steiner: 5,
            ..behavior(true)
        };
        let mut m = Mesh::new(&b, 8, 2, 0, 0).expect("mesh init");
        let a = vxy(&mut m, 0.0, 0.0);
        let bb = vxy(&mut m, 1.0, 0.0);
        let c = vxy(&mut m, 0.0, 1.0);
        let d = vxy(&mut m, 0.5, -1.0);
        let (t1, _t2) = pair(&mut m, true, a, bb, c, d);
        crate::quality::makevertexmap(&mut m, &b);
        // Mirror the -p driver's dummytri[0] linkage (written by
        // removeghosts, triangle.c:4507; reconstruct never writes it):
        // conformingedge's from-scratch insertvertex locates through the
        // dummy row, so point it at the hull edge t1@2 (edge c -> a).
        m.set_tri_neighbor_word(
            DUMMY_TRI,
            0,
            encode(Otri {
                tri: t1.tri,
                orient: 2,
            }),
        );
        m.checksegments = true;

        insertsegment(&mut m, &b, c, d, 9).expect("insertsegment");

        // midpoint of c(0,1)-d(0.5,-1): (0.25, 0) exact; one Steiner used.
        let mid = vertex_at(&m, 0.25, 0.0).expect("midpoint vertex");
        assert_eq!(m.vx_type(mid), SEGMENTVERTEX);
        assert_eq!(m.vx_mark(mid), 9);
        assert_eq!(m.steinerleft, 4);
        // the edge a-b split both triangles: 2 -> 4.
        assert_eq!(m.triangles.live_count(), 4);
        // two half-segments: {mid, c} and {mid, d}, both newmark'd;
        // each half's orient-0 segorg runs mid -> far end (insertsubseg
        // sets sorg/sdest from the scouted edge's org/dest).
        let subs = live_subsegs(&m);
        assert_eq!(subs.len(), 2);
        let mut halves = vec![];
        for ss in subs {
            let s = Osub { ss, ssorient: 0 };
            let (org, dst) = (m.segorg(s), m.segdest(s));
            assert!(org == mid || dst == mid);
            assert!(org == c || org == d || dst == c || dst == d);
            assert_eq!(m.sub_mark(ss), 9);
            halves.push((org.min(dst), org.max(dst)));
        }
        halves.sort();
        assert_eq!(
            halves,
            vec![(c.min(mid), c.max(mid)), (d.min(mid), d.max(mid))]
        );
    }

    /// The same crossing without -D: constrainededge digs from c, flips
    /// the shared edge once (the dig's single edge removal), installs
    /// the c-d subsegment. Triangle count, hull size and vertex count
    /// are all invariant under the dig.
    #[test]
    fn constrainededge_flip_dig_walk() {
        let mut m = Mesh::new(&behavior(true), 8, 2, 0, 0).expect("mesh init");
        let b = behavior(true);
        let a = vxy(&mut m, 0.0, 0.0);
        let bb = vxy(&mut m, 1.0, 0.0);
        let c = vxy(&mut m, 0.0, 1.0);
        let d = vxy(&mut m, 0.5, -1.0);
        pair(&mut m, true, a, bb, c, d);
        crate::quality::makevertexmap(&mut m, &b);
        m.checksegments = true;

        insertsegment(&mut m, &b, c, d, 8).expect("insertsegment");

        // the dig preserves the pools: 2 triangles, 4 hull edges, 4
        // vertices; the a-b edge gave way to the c-d segment edge.
        assert_eq!(m.triangles.live_count(), 2);
        assert_eq!(m.vertices.live_count(), 4);
        assert_eq!(boundary_edges(&m), 4);
        let subs = live_subsegs(&m);
        assert_eq!(subs.len(), 1);
        let s = Osub {
            ss: subs[0],
            ssorient: 0,
        };
        let org = m.segorg(s);
        let dst = m.segdest(s);
        assert!(org == c && dst == d || org == d && dst == c);
        assert_eq!(m.sub_mark(subs[0]), 8);
        assert_eq!(m.vx_mark(c), 8);
        assert_eq!(m.vx_mark(d), 8);
        // the c-d edge is truly a bonded mesh edge now: the two live
        // triangles each carry it once, in each direction.
        let mut found = 0;
        for tri in live_tris(&m) {
            for orient in 0..3u8 {
                let e = Otri { tri, orient };
                let (o, dd) = (m.org(e), m.dest(e));
                if (o == c && dd == d) || (o == d && dd == c) {
                    found += 1;
                }
            }
        }
        assert_eq!(found, 2);
        // and the old diagonal is gone.
        for tri in live_tris(&m) {
            for orient in 0..3u8 {
                let e = Otri { tri, orient };
                let (o, dd) = (m.org(e), m.dest(e));
                assert!(!(o == a && dd == bb || o == bb && dd == a));
            }
        }
    }

    /// formskeleton over the delaunay square's four PSLG hull segments:
    /// makevertexmap runs first (all four vertex2tri words set), each
    /// segment installs directly, markers propagate first-touch.
    #[test]
    fn formskeleton_square_pslg() {
        let (mut m, b) = delaunay_square(true);
        formskeleton(
            &mut m,
            &b,
            &[1, 2, 2, 3, 3, 4, 4, 1],
            Some(&[11, 12, 13, 14]),
            4,
        )
        .expect("formskeleton");
        assert_eq!(m.insegments, 4);
        // the vertex2tri map: every vertex's word decodes to a triangle
        // whose org is that vertex (the makevertexmap contract).
        for v in 1..=4u32 {
            let w = m.vx_2tri(v);
            assert_ne!(w, 0);
            let tri = decode(w);
            assert_eq!(m.org(tri), v);
        }
        assert_eq!(m.subsegs.live_count(), 4);
        // marker propagation, insertion order 0..3, first nonzero touch
        // wins (insertsubseg's mark-0 checks): A=11 B=11 C=12 D=13.
        assert_eq!(m.vx_mark(1), 11);
        assert_eq!(m.vx_mark(2), 11);
        assert_eq!(m.vx_mark(3), 12);
        assert_eq!(m.vx_mark(4), 13);
        // b.convex is false and poly true: no markhull (still 4 hull
        // edges, all four of them segment-bounded now).
        assert_eq!(boundary_edges(&m), 4);
    }

    /// infecthull + plague on the bare delaunay square (no segments):
    /// every hull triangle is unprotected, so both get infected, and
    /// plague eats the whole mesh. Accounting derived per C line:
    /// the shared diagonal nets zero (the second dying triangle sees
    /// it flipped to dummy: +1 then -1), the four hull edges cost four:
    /// hullsize 4 -> 0; all four vertices orphan (UNDEADVERTEX,
    /// undeads 4); pools: tris 2 -> 0.
    #[test]
    fn infecthull_and_plague_eat_the_bare_square() {
        let (mut m, b) = delaunay_square(true);
        let triangles_before = m.triangles.live_count();
        assert_eq!(triangles_before, 2);

        infecthull(&mut m, &b).expect("infecthull");
        // both triangles sit on the hull: viri rows == infected count 2.
        assert_eq!(m.viri.live_count(), 2);
        let mut infected = 0;
        for tri in live_tris(&m) {
            if m.infected(Otri { tri, orient: 0 }) {
                infected += 1;
            }
        }
        assert_eq!(infected, 2);

        plague(&mut m, &b).expect("plague");
        assert_eq!(m.triangles.live_count(), 0);
        assert_eq!(m.subsegs.live_count(), 0);
        assert_eq!(m.viri.live_count(), 0); // poolrestart
                                            // hull accounting per C line by line: the first dying triangle's
                                            // diagonal edge flips the neighbor to dummy (hullsize++), so the
                                            // second dying triangle's diagonal edge already syms to dummy
                                            // (hullsize--); the diagonal nets zero, the four hull edges cost
                                            // four: 4 - 4 = 0.
        assert_eq!(m.hullsize, 0);
        assert_eq!(m.undeads, 4);
        for v in 1..=4u32 {
            assert_eq!(m.vx_type(v), UNDEADVERTEX);
        }
    }

    /// The hand-written 10-triangle ring mesh: outer square
    /// A(0,0) B(4,0) C(4,4) D(0,4), inner square E(1,1) F(3,1)
    /// G(3,3) H(1,3), an 8-triangle band plus the two inner square
    /// triangles on the E-G diagonal. reconstruct links it from the
    /// .ele/.poly lists (numbered from 1), returns the outer hull
    /// size 4, bonds all shared edges, installs the 8 PSLG segments.
    /// vertex rows are typed INPUTVERTEX (0) by poolalloc; row ids of
    /// elements 1..10 are 2..11 (the triangle pool's first fresh id is
    /// the sentinel shift of D3).
    fn ring_mesh(vararea: bool) -> (Mesh, Behavior) {
        let b = Behavior {
            poly: true,
            usesegments: true,
            regionattrib: true,
            vararea,
            convex: false,
            ..Behavior::default_form()
        };
        let mut m = Mesh::new(&b, 8, 2, 0, 0).expect("mesh init");
        vxy(&mut m, 0.0, 0.0); // 1 A
        vxy(&mut m, 4.0, 0.0); // 2 B
        vxy(&mut m, 4.0, 4.0); // 3 C
        vxy(&mut m, 0.0, 4.0); // 4 D
        vxy(&mut m, 1.0, 1.0); // 5 E
        vxy(&mut m, 3.0, 1.0); // 6 F
        vxy(&mut m, 3.0, 3.0); // 7 G
        vxy(&mut m, 1.0, 3.0); // 8 H
        let trianglelist = [
            1, 2, 6, // t1  A B F
            1, 6, 5, // t2  A F E
            2, 3, 7, // t3  B C G
            2, 7, 6, // t4  B G F
            3, 4, 8, // t5  C D H
            3, 8, 7, // t6  C H G
            4, 1, 5, // t7  D A E
            4, 5, 8, // t8  D E H
            5, 6, 7, // t9  E F G
            5, 7, 8, // t10 E G H
        ];
        let segmentlist = [
            1, 2, 2, 3, 3, 4, 4, 1, // outer ring
            5, 6, 6, 7, 7, 8, 8, 5, // inner ring
        ];
        let arealist = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        let arealist_ref: Vec<f64> = arealist.to_vec();
        let attribute_list: Vec<f64> = vec![];
        let hullsize = reconstruct(
            &mut m,
            &b,
            &trianglelist,
            &attribute_list,
            &arealist_ref, // read only when vararea
            10,
            3,
            0,
            &segmentlist,
            None, // no .poly boundary markers: every mark stays 0
            8,
        )
        .expect("reconstruct");
        assert_eq!(hullsize, 4);
        assert_eq!(m.triangles.live_count(), 10);
        assert_eq!(m.subsegs.live_count(), 8);
        m.checksegments = true;
        // triangle_api.c:297: the api stores reconstruct's result.
        m.hullsize = hullsize as i64;
        // Mirror the -p driver's dummytri[0] linkage (removeghosts,
        // triangle.c:4507): on reconstructed meshes C never writes the
        // dummy row (reconstruct bonds only interior edges), while
        // carveholes locates holes and regions from the dummy row.
        // Point it at the hull edge t1@0 (org A, dest B).
        m.set_tri_neighbor_word(DUMMY_TRI, 0, encode(Otri { tri: 2, orient: 0 }));
        // the transfernodes bounds write (ioarrays.rs not landed): the
        // input coordinates' bounding box, mirrored by hand.
        m.xmin = 0.0;
        m.xmax = 4.0;
        m.ymin = 0.0;
        m.ymax = 4.0;
        (m, b)
    }

    /// reconstruct's end state on the -r shape (usesegments rows, no
    /// poly): 10 linked triangles, every hull edge counted once, the
    /// interior band bonds in place (every non-hull edge shared by two
    /// triangles), each element's vertices on the right row, counts
    /// per the input lists.
    #[test]
    fn reconstruct_end_state() {
        let (m, _b) = ring_mesh(false);
        // 10 elements: rows 2..=11, all alive, in input order with the
        // input corners (t1 = A B F = vertices 1 2 6).
        assert!(!m.deadtri(2) && !m.deadtri(11));
        let t1 = Otri { tri: 2, orient: 0 };
        assert_eq!(m.org(t1), 1);
        assert_eq!(m.dest(t1), 2);
        assert_eq!(m.apex(t1), 6);
        let t10 = Otri { tri: 11, orient: 0 };
        assert_eq!(m.org(t10), 5);
        assert_eq!(m.dest(t10), 7);
        assert_eq!(m.apex(t10), 8);
        // topology: exactly 4 edges face outer space (the outer ring);
        // every other edge has a living neighbor.
        assert_eq!(boundary_edges(&m), 4);
        // all 8 input vertices live, typed INPUTVERTEX by the pool.
        for v in 1..=8u32 {
            assert_eq!(m.vx_type(v), INPUTVERTEX);
        }
    }

    /// carveholes with a hole at the ring center: the infection spreads
    /// to both inner triangles (the E-G diagonal is unprotected) and
    /// stops at the four inner-ring subsegments. Pools row for row:
    /// triangles 10 -> 8, subsegments 8 (none dies: each inner segment
    /// separates a dying triangle from a live one), the four inner
    /// segments dissolve from their band side and become boundaries
    /// (mark 1, endpoints marked). Hull accounting per C lines, for
    /// either dying order: each of the six dying-triangle edges syms to
    /// a live band neighbor (hullsize++) except the shared E-G diagonal,
    /// which counts +1 from the first dyer (it dissolves the second
    /// dyer's side to dummy) and -1 from the second (the edge is a
    /// boundary by then): net +4, hullsize 4 -> 8. No vertex orphans.
    #[test]
    fn carveholes_ring_hole() {
        let (mut m, b) = ring_mesh(false);
        carveholes(&mut m, &b, &[Hole { x: 2.0, y: 2.0 }], &[]).expect("carveholes");
        assert_eq!(m.triangles.live_count(), 8);
        assert_eq!(m.subsegs.live_count(), 8);
        assert_eq!(m.viri.live_count(), 0);
        assert_eq!(m.undeads, 0);
        assert_eq!(m.hullsize, 8); // 4 + 4, the C accounting above
                                   // the two inner triangles (rows 10 and 11: elements 9, 10) died.
        assert!(m.deadtri(10));
        assert!(m.deadtri(11));
        for tri in 2..=9u32 {
            assert!(!m.deadtri(tri));
        }
        // infecthull ran first (convex false): the outer ring segments
        // carry mark 1 and marked their endpoints; plague's protected
        // branches marked the inner ring and its vertices the same way.
        for ss in live_subsegs(&m) {
            assert_eq!(m.sub_mark(ss), 1);
        }
        for v in 1..=8u32 {
            assert_eq!(m.vx_mark(v), 1);
            assert_eq!(m.vx_type(v), INPUTVERTEX);
        }
        // all four inner-ring vertices kept a live triangle: no orphans.
        assert_eq!(m.undeads, 0);
    }

    /// carveholes with a region point inside the band triangle t2:
    /// regionplague spreads through the whole unprotected band (it
    /// crosses A-F, A-E, E-D, D-H, H-C, C-G, G-B and B-F), stopping at
    /// the outer-ring and inner-ring subsegments, and stamps the
    /// regional attribute 7 on exactly the 8 band triangles. The
    /// pre-spread zeroing left 0.0 on the two inner triangles;
    /// eextras increments by 1 afterwards exactly at triangle.c:7097.
    #[test]
    fn carveholes_region_attribute_spreads_to_the_band() {
        let (mut m, b) = ring_mesh(false);
        let attrib_word = m.tri_shape.elemattribindex + m.eextras as usize;
        carveholes(
            &mut m,
            &b,
            &[],
            &[crate::input::Region {
                x: 1.2,
                y: 0.9, // inside t2 (A F E): y between x/3 and 1, y < x
                attribute: 7.0,
                max_area: 0.5,
            }],
        )
        .expect("carveholes");
        // no holes, no concavity, the region triangle lives: nothing
        // was deleted or accounted, eextras took its +1.
        assert_eq!(m.triangles.live_count(), 10);
        assert_eq!(m.hullsize, 4);
        assert_eq!(m.eextras, 1);
        assert_eq!(m.viri.live_count(), 0);
        // band triangles: rows 2..=9 carry 7.0; the inner rows 10, 11
        // keep the zeroing value.
        for tri in 2..=9u32 {
            assert_eq!(m.tri_real(tri, attrib_word), 7.0);
        }
        for tri in 10..=11u32 {
            assert_eq!(m.tri_real(tri, attrib_word), 0.0);
        }
    }

    /// The same region with vararea: regionplague's areabound write
    /// stamps 0.5 on the band only; the inner triangles keep the
    /// arealist values reconstruct wrote per element (9.0 and 10.0).
    #[test]
    fn carveholes_region_area_constraint() {
        let (mut m, b) = ring_mesh(true);
        carveholes(
            &mut m,
            &b,
            &[],
            &[crate::input::Region {
                x: 1.2,
                y: 0.9,
                attribute: 3.0,
                max_area: 0.5,
            }],
        )
        .expect("carveholes");
        assert_eq!(m.triangles.live_count(), 10);
        for tri in 2..=9u32 {
            assert_eq!(m.tri_areabound(tri), 0.5);
        }
        // reconstruct wrote arealist[elementnumber - 1]: elements 9, 10
        // (rows 10, 11) keep 9.0 and 10.0.
        assert_eq!(m.tri_areabound(10), 9.0);
        assert_eq!(m.tri_areabound(11), 10.0);
        assert_eq!(m.eextras, 1);
    }

    /// scoutsegment's collinear mid-vertex chain: the square's bottom
    /// edge extended with M(2,0): the segment A -> M collides with the
    /// vertex B between its endpoints; each half installs a subsegment
    /// (the LEFTCOLLINEAR/RIGHTCOLLINEAR recursion), the boundary mark
    /// propagates, and scoutsteps accounts two scouts.
    #[test]
    fn insertsegment_collinear_midvertex_chain() {
        let mut m = Mesh::new(&behavior(true), 9, 2, 0, 0).expect("mesh init");
        let b = behavior(true);
        let a = vxy(&mut m, 0.0, 0.0);
        let v1 = vxy(&mut m, 1.0, 0.0);
        let mpt = vxy(&mut m, 2.0, 0.0);
        let top = vxy(&mut m, 0.5, 1.0);
        // t1 = {A, B, top}, t2 = {B, M, top} sharing edge B-top.
        let t1 = m.maketriangle(true, false).expect("tri");
        m.setorg(t1, a);
        m.setdest(t1, v1);
        m.setapex(t1, top);
        let t2 = m.maketriangle(true, false).expect("tri");
        m.setorg(t2, v1);
        m.setdest(t2, mpt);
        m.setapex(t2, top);
        // bond the shared edge: t1's orient-1 (B->top) with t2's orient-2
        // (top->B).
        m.bond(
            Otri {
                tri: t1.tri,
                orient: 1,
            },
            Otri {
                tri: t2.tri,
                orient: 2,
            },
        );
        crate::quality::makevertexmap(&mut m, &b);
        m.checksegments = true;

        insertsegment(&mut m, &b, a, mpt, 3).expect("insertsegment");
        // two collinear half-segments: A-B and B-M.
        assert_eq!(m.subsegs.live_count(), 2);
        let subs = live_subsegs(&m);
        let mut endsets = vec![];
        for ss in subs {
            let s = Osub { ss, ssorient: 0 };
            let (o, dd) = (m.segorg(s), m.segdest(s));
            endsets.push((o.min(dd), o.max(dd)));
        }
        endsets.sort();
        assert_eq!(
            endsets,
            vec![(a.min(v1), a.max(v1)), (mpt.min(v1), mpt.max(v1))]
        );
        // the scoutsteps mirror: exactly two scoutsegment invocations
        // (the collinear half A-B, then B-M), each ++ing once
        // (triangle.c:6011).
        assert_eq!(m.scoutsteps, 2);
    }

    #[test]
    fn scoutsegment_outcome_scout_pins_the_raising_site() {
        // Replaces the funnel test's SegScout arm pin: the scoutsteps
        // budget site (triangle.c:6016-6017) raises
        // SegmentChainError::Recovery(Scout) exactly where C sets *status
        // and returns TRI_SEG_SCOUT. A budget already past twice the
        // vertex count fires the check on the first call.
        let b = behavior(true);
        let mut m = Mesh::new(&b, 8, 2, 0, 0).expect("mesh init");
        let a = vxy(&mut m, 0.0, 0.0);
        let bb = vxy(&mut m, 1.0, 0.0);
        let c = vxy(&mut m, 0.0, 1.0);
        let d = vxy(&mut m, 0.5, -1.0);
        let (t1, _t2) = pair(&mut m, true, a, bb, c, d);
        m.scoutsteps = 2 * m.vertices.live_count() as i64 + 1;
        let mut searchtri = Otri {
            tri: t1.tri,
            orient: 0,
        };
        let out = scoutsegment(&mut m, &b, &mut searchtri, c, 7);
        assert!(matches!(
            out,
            Err(SegmentChainError::Recovery(SegmentRecoveryKind::Scout))
        ));
    }

    #[test]
    fn insertsegment_outcome_insert_pins_the_raising_site() {
        // Replaces the funnel test's SegInsert arm pin: the endpoint
        // locate site (triangle.c:6446; the port's first-endpoint
        // branch) raises SegmentChainError::Recovery(Insert) where C sets
        // *status and returns. endpoint1 is a freshly allocated vertex
        // with no vertex2tri word and coordinates outside the mesh,
        // so the locate start walks off the hull without finding it.
        let mut m = Mesh::new(&behavior(true), 8, 2, 0, 0).expect("mesh init");
        let b = behavior(true);
        let a = vxy(&mut m, 0.0, 0.0);
        let bb = vxy(&mut m, 1.0, 0.0);
        let c = vxy(&mut m, 0.0, 1.0);
        let d = vxy(&mut m, 0.5, -1.0);
        let (t1, _t2) = pair(&mut m, true, a, bb, c, d);
        // Mirror the -p driver's dummytri[0] linkage (the
        // conformingedge test's pattern): the locate start decodes it.
        m.set_tri_neighbor_word(
            DUMMY_TRI,
            0,
            encode(Otri {
                tri: t1.tri,
                orient: 2,
            }),
        );
        let outside = vxy(&mut m, 100.0, 100.0);
        let out = insertsegment(&mut m, &b, outside, a, 5);
        assert!(matches!(
            out,
            Err(SegmentChainError::Recovery(SegmentRecoveryKind::Insert))
        ));
    }

    #[test]
    fn segmentintersection_outcome_cycle_pins_the_raising_site() {
        // Replaces the funnel test's Cycle arm pin: the cyclefail
        // check after insertvertex (triangle.c:5931) raises
        // SegmentChainError::Recovery(Cycle). Nothing on this path resets
        // the flag (only triangleinit does), so the preset state is
        // exactly the state C's insertvertex leaves after a circling
        // insertion (triangle.c:3282-3284) - the pin constructs the
        // raising site's input state directly.
        let b = Behavior {
            steiner: 5,
            ..behavior(true)
        };
        let mut m = Mesh::new(&b, 8, 2, 0, 0).expect("mesh init");
        let a = vxy(&mut m, 0.0, 0.0);
        let bpt = vxy(&mut m, 1.0, 0.0);
        let c = vxy(&mut m, 1.0, 1.0);
        let d = vxy(&mut m, 0.5, -1.0);
        let (t1, _t2) = pair(&mut m, true, a, bpt, c, d);
        m.checksegments = true;
        let shared = Otri {
            tri: t1.tri,
            orient: 0,
        };
        insertsubseg(&mut m, &b, shared, 7).expect("insertsubseg");
        let mut splittri = shared;
        let mut splitsubseg = m.tspivot(splittri);
        m.cyclefail = true;
        let out = segmentintersection(&mut m, &b, &mut splittri, &mut splitsubseg, d);
        assert!(matches!(
            out,
            Err(SegmentChainError::Recovery(SegmentRecoveryKind::Cycle))
        ));
    }

    /// The boundary conversion of the fix pass's narrow chain error
    /// is lossless: the Recovery kinds map to their public variants,
    /// Internal constructs internalerror()'s verbatim payload
    /// (triangle.c:349-354, pinned byte-identical to the pre-fix
    /// raise), and the chain's Oom bridge maps to the fatal
    /// OutOfMemory (D26). Pins by construction; no oracle involved.
    #[test]
    fn segment_chain_error_boundary_conversion_is_lossless() {
        assert!(matches!(
            Error::from(SegmentChainError::Recovery(SegmentRecoveryKind::Scout)),
            Error::SegmentRecovery(SegmentRecoveryKind::Scout)
        ));
        assert!(matches!(
            Error::from(SegmentChainError::Oom),
            Error::OutOfMemory
        ));
        assert_eq!(
            Error::from(SegmentChainError::Internal).to_string(),
            "Triangle internal error:   Please report this bug to jrs@cs.berkeley.edu\n  Include the message above, your input data set, and the exact\n    command line you used to run Triangle."
        );
        // pool::Oom enters the chain as Oom (the From impl the ?
        // operator uses at the chain's allocation sites).
        assert!(matches!(
            SegmentChainError::from(Oom),
            SegmentChainError::Oom
        ));
    }
}
