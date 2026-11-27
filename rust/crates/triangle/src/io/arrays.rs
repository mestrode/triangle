//! The C Array I/O routines at the mesh level (triangle.c:7687-8134,
//! the section's opening banner at 7687 and its closing banner at
//! 8134): transfernodes reads the vertices from memory (landed,
//! wired into triangle_mesh_create's orchestration at the api.rs
//! boundary), and the six writers - writenodes (triangle.c:7768),
//! numbernodes (7840), writeelements (7863), writepoly (7933),
//! writeedges (7983), writeneighbors (8054) - marshal the mesh into
//! the public MeshOutput types and feed Mesh::output
//! (triangle_mesh_copy, landed in api.rs).
//! quality_statistics once closed the C section (triangle.c:8143); it
//! moved to algorithm/quality.rs for cohesion with its only caller
//! (docs/function-mapping.md).
//!
//! Input seam (ADR 0004 section 5.2): C's flat `REAL *pointlist`,
//! `REAL *pointattriblist` and `int *pointmarkerlist` arrive as the
//! owned MeshInput fields, passed by reference at the api.rs call
//! site exactly where triangle_api.c hands its triangleio pointers to
//! transfernodes (triangle_api.c:200-202). The Point2 records carry
//! C's two-REAL stride per point (the carveholes precedent for the
//! typed records of the frozen input surface, algorithm/segments.rs), the
//! attribute list stays flat as in C, and the marker list mirrors as
//! Option, preserving C's NULL-vs-present branch (triangle.c:7732): an
//! empty MeshInput marker list is C's NULL list.
//!
//! Output seam (the input seam's mirror): C's `REAL **pointlist`
//! out-parameters become the owned MeshOutput fields handed in as
//! &mut Vecs at the api.rs call site exactly where triangle_mesh_copy
//! hands its triangleio members to the writers (triangle_api.c:
//! 393-421). The flat lists stay flat where the frozen surface froze
//! flat Vecs (coordinates, element corners, attributes, neighbors)
//! and arrive as endpoint pairs where it froze pairs (segments,
//! edges); C's trimalloc sizing becomes the exact Vec reserves, a
//! failed allocation funneling to Oom where C's trimalloc exits the
//! process (D8).
//!
//! The numbering trick (the writenodes banner, triangle.c:7763-7764):
//! to save memory, C writes the vertex numbers over the boundary
//! markers after copying each marker into the output list. The port
//! mirrors the scratch reuse exactly through the vx chokepoint
//! (triangle.c:7820-7823); writeelements, writepoly and writeedges
//! read the numbers back through vertexmark exactly where C does
//! (triangle.c:7902-7915, 7965-7966, 8024-8025); and
//! triangle_mesh_copy restores the original markers from the output
//! list afterwards (triangle_api.c:423-425;
//! triangle_restore_pointmarkers, its mapping row in input.rs).
//! writeneighbors works the same trick on the triangle side: C
//! overwrites the first subseg pointer of each row with an int id
//! (triangle.c:8088-8094), which the port mirrors as the D2 int
//! overlay the row shape carries (the numberindex of
//! initializetrisubpools), saving and restoring exactly like C's
//! scratch array.
//! numbernodes (triangle.c:7840) has no live C caller anywhere - both
//! real numbering sites inline the loop (writenodes, triangle.c:7823;
//! file_writenodes' setvertexmark inside its own traversal,
//! triangle_io.c:64-65) - the only other trace is the commented-out
//! call at src/examples/triangle-cli/main.c:65 - and C's own banner
//! (triangle.c:7830-7838) describes a use no live call site realizes,
//! so it lands dead_code-allowed with that noted at the site.

use crate::error::Error;
use crate::input::Point2;
use crate::mesh::handles::{Osub, Otri, DUMMY_SUB, DUMMY_TRI};
use crate::mesh::pool::Oom;
use crate::mesh::slots::{INPUTVERTEX, UNDEADVERTEX};
use crate::mesh::Mesh;
use crate::options::Behavior;
use std::cmp::Ordering;

/// C: transfernodes (triangle.c:7697-7757, banner 7691-7696). Read the
/// vertices from memory: store the input counters, size the vertex
/// pool, then walk the input once - one poolalloc'd vertex row per
/// point, written with its two coordinates, its attributes, its
/// boundary marker (or the default 0) and its INPUTVERTEX type - while
/// updating the bounds, and finish with the sweepline circle-event
/// flag.
///
/// C returns TRI_INPUT (triangle.c:7712) where the port raises
/// Error::Input with the funnel arm's exact payload (rung 2, D8),
/// and TRI_OK (triangle.c:7756) as Ok. The `b->weighted`
/// reset (triangle.c:7715) mutates the core's Behavior exactly where
/// C mutates b.
// C-parity: the single indexed `for (i = 0; i < m->invertices; i++)`
// loop (triangle.c:7723) walks the point records and the marker list
// by index verbatim; C's coordindex++ pair is absorbed by the Point2
// records (two REALs per point, the module's input seam).
#[allow(clippy::needless_range_loop)]
pub(crate) fn transfernodes(
    m: &mut Mesh,
    b: &mut Behavior,
    pointlist: &[Point2],
    pointattriblist: &[f64],
    pointmarkerlist: Option<&[i32]>,
    numberofpoints: i32,
    numberofpointattribs: i32,
) -> Result<(), Error> {
    m.invertices = numberofpoints;
    m.mesh_dim = 2;
    m.nextras = numberofpointattribs;
    if m.invertices < 3 {
        // Error:  Input must have at least three input vertices.
        return Err(Error::Input("invalid input (TRI_INPUT)".to_string()));
    }
    if m.nextras == 0 {
        b.weighted = 0;
    }

    m.initializevertexpool(b.poly).map_err(Error::from)?;
    // The pool geometry already ran eagerly in Mesh::new over the same
    // input counters (the ADR 0003 section 4 convention, the same one
    // algorithm/delaunay.rs cites at its initializetrisubpools site), and no
    // vertex row exists on any transfernodes call path yet, so this
    // C-site call re-creates an identical empty pool: no allocation,
    // id, reuse, or traversal state exists that could observe it.

    // Read the vertices.
    let mut attribindex = 0usize;
    for i in 0..m.invertices as usize {
        let vertexloop = m.vx_alloc().map_err(Error::from)?;
        // Read the vertex coordinates.
        let x = pointlist[i].x;
        let y = pointlist[i].y;
        m.set_vx_x(vertexloop, x);
        m.set_vx_y(vertexloop, y);
        // Read the vertex attributes.
        for j in 0..numberofpointattribs as usize {
            let attribute = pointattriblist[attribindex];
            attribindex += 1;
            m.vx_set_real(vertexloop, 2 + j, attribute);
        }
        if let Some(markerlist) = pointmarkerlist {
            // Read a vertex marker.
            m.vx_set_mark(vertexloop, markerlist[i]);
        } else {
            // If no markers are specified, they default to zero.
            m.vx_set_mark(vertexloop, 0);
        }
        m.vx_set_type(vertexloop, INPUTVERTEX);
        // Determine the smallest and largest x and y coordinates.
        if i == 0 {
            m.xmin = x;
            m.xmax = x;
            m.ymin = y;
            m.ymax = y;
        } else {
            m.xmin = if x < m.xmin { x } else { m.xmin };
            m.xmax = if x > m.xmax { x } else { m.xmax };
            m.ymin = if y < m.ymin { y } else { m.ymin };
            m.ymax = if y > m.ymax { y } else { m.ymax };
        }
    }

    // Nonexistent x value used as a flag to mark circle events in sweepline
    //   Delaunay algorithm.
    m.xminextreme = 10.0 * m.xmin - 9.0 * m.xmax;

    Ok(())
}

/// C: writenodes (triangle.c:7768-7828, banner 7759-7766). Number the
/// vertices and write them to arrays: walk the vertex pool in
/// allocation order, copy each non-jettisoned vertex's coordinates,
/// attributes and boundary marker into the output lists, then write
/// the vertex number over the boundary marker through the vx
/// chokepoint - the banner's memory trick (triangle.c:7763-7764: "To
/// save memory, the vertex numbers are written over the boundary
/// markers") - so the later writers read the numbers back through
/// vertexmark, and triangle_mesh_copy restores the originals from the
/// marker list at the end (triangle_api.c:423-425).
///
/// The out-parameters are the owned MeshOutput Vecs handed in by
/// api.rs exactly where C passes its triangleio members
/// (triangle_api.c:393-395); C's "allocate if NULL" branches
/// (triangle.c:7787-7799) are the exact Vec reserves on the fresh
/// lists, a failed allocation funneling to Oom where C's trimalloc
/// exits the process (D8). C's coordindex/attribindex cursor pair
/// (triangle.c:7803-7804) is absorbed by the push order, and the
/// indexed marker store pmlist[vertexnumber - firstnumber]
/// (triangle.c:7820) is the push in the same traversal order (the
/// marker list counts only the output vertices).
pub(crate) fn writenodes(
    m: &mut Mesh,
    b: &Behavior,
    pointlist: &mut Vec<Point2>,
    pointattriblist: &mut Vec<f64>,
    pointmarkerlist: &mut Vec<i32>,
) -> Result<(), Oom> {
    // C: outvertices (triangle.c:7781-7785): jettison shrinks the
    // output by the undead count.
    let outvertices: usize = if b.jettison {
        (m.vertices.live_count() - m.undeads as u64) as usize
    } else {
        m.vertices.live_count() as usize
    };

    // Allocate memory for output vertices if necessary
    // (triangle.c:7787-7789).
    pointlist.try_reserve_exact(outvertices).map_err(|_| Oom)?;
    // Allocate memory for output vertex attributes if necessary
    // (triangle.c:7791-7795).
    if m.nextras > 0 {
        pointattriblist
            .try_reserve_exact(outvertices * m.nextras as usize)
            .map_err(|_| Oom)?;
    }
    // Allocate memory for output vertex markers if necessary
    // (triangle.c:7796-7799).
    if !b.nobound {
        pointmarkerlist
            .try_reserve_exact(outvertices)
            .map_err(|_| Oom)?;
    }

    // C: traversalinit + the vertexnumber walk (triangle.c:7806-7808).
    let mut vertexnumber = b.firstnumber;
    let mut t = m.vertices.traversal();
    while let Some(vertexloop) = m.vertextraverse(&mut t) {
        // C: the jettison gate (triangle.c:7810).
        if !b.jettison || m.vx_type(vertexloop) != UNDEADVERTEX {
            // X and y coordinates (triangle.c:7811-7813).
            pointlist.push(Point2 {
                x: m.vx_x(vertexloop),
                y: m.vx_y(vertexloop),
            });
            // Vertex attributes (triangle.c:7814-7817).
            for i in 0..m.nextras as usize {
                pointattriblist.push(m.vx_real(vertexloop, 2 + i));
            }
            if !b.nobound {
                // Copy the boundary marker (triangle.c:7818-7821).
                pointmarkerlist.push(m.vx_mark(vertexloop));
            }

            // To save memory, the vertex numbers are written over the
            // boundary markers (triangle.c:7763-7764, the store at
            // 7823) - the numbering the later writers read back.
            m.vx_set_mark(vertexloop, vertexnumber);
            vertexnumber += 1;
        }
    }
    Ok(())
}

/// C: numbernodes (triangle.c:7840-7855, banner 7830-7838). Number the
/// vertices: each vertex is assigned a marker equal to its number
/// (the banner, triangle.c:7834). Used when writenodes() is not called
/// because no .node file is written (triangle.c:7836) - but no live
/// call site in this fork realizes that use: numbernodes has no live
/// C caller anywhere, both real numbering sites inline the loop
/// (writenodes, triangle.c:7823; file_writenodes' setvertexmark inside
/// its own traversal, triangle_io.c:64-65), and the only other trace
/// is the commented-out call at src/examples/triangle-cli/main.c:65.
// C-parity: the marker store (triangle.c:7849) runs for every vertex,
// the undead included, while the number advances only past
// non-undead vertices under jettison (triangle.c:7850-7852).
#[allow(dead_code)] // no live C caller exists; parity landing
                    // of C's own uncalled function (C's banner,
                    // triangle.c:7836: used when writenodes is not
                    // called; the only trace is main.c:65, commented
                    // out); the io/mod.rs file writer will inline the
                    // numbering exactly like C's file_writenodes
                    // (triangle_io.c:64-65).
pub(crate) fn numbernodes(m: &mut Mesh, b: &Behavior) {
    let mut vertexnumber = b.firstnumber;
    let mut t = m.vertices.traversal();
    while let Some(vertexloop) = m.vertextraverse(&mut t) {
        m.vx_set_mark(vertexloop, vertexnumber);
        if !b.jettison || m.vx_type(vertexloop) != UNDEADVERTEX {
            vertexnumber += 1;
        }
    }
}

/// C: writeelements (triangle.c:7863-7925). Write the triangles to
/// arrays: walk the triangle pool in allocation order and record each
/// element's corner numbers - the vertexmark reads return the
/// writenodes numbering (triangle.c:7902-7915) - plus, above order 1,
/// the three mid-edge nodes highorder recorded at highorderindex,
/// highorderindex + 1 and highorderindex + 2 (triangle.c:7906-7915),
/// and the element attributes (triangle.c:7918-7920).
///
/// The out-parameters are the owned MeshOutput Vecs (triangle_api.c:
/// 397-398); C's allocation branches (triangle.c:7876-7887) are the
/// exact reserves over the (order + 1) * (order + 2) / 2 corners, and
/// C's elementnumber cursor (triangle.c:7896, 7923) is absorbed by
/// the push order.
pub(crate) fn writeelements(
    m: &mut Mesh,
    b: &Behavior,
    trianglelist: &mut Vec<i32>,
    triangleattriblist: &mut Vec<f64>,
) -> Result<(), Oom> {
    // Allocate memory for output triangles if necessary
    // (triangle.c:7876-7881).
    let corners = ((b.order + 1) * (b.order + 2) / 2) as usize;
    trianglelist
        .try_reserve_exact(m.triangles.live_count() as usize * corners)
        .map_err(|_| Oom)?;
    // Allocate memory for output triangle attributes if necessary
    // (triangle.c:7882-7887).
    if m.eextras > 0 {
        triangleattriblist
            .try_reserve_exact(m.triangles.live_count() as usize * m.eextras as usize)
            .map_err(|_| Oom)?;
    }

    // C: traversalinit + triangleloop.orient = 0 (triangle.c:7893-7895).
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.triangletraverse(&mut t) {
        let triangleloop = Otri { tri, orient: 0 };
        let p1 = m.org(triangleloop);
        let p2 = m.dest(triangleloop);
        let p3 = m.apex(triangleloop);
        if b.order == 1 {
            // C: the three corner stores (triangle.c:7901-7904).
            trianglelist.push(m.vx_mark(p1));
            trianglelist.push(m.vx_mark(p2));
            trianglelist.push(m.vx_mark(p3));
        } else {
            // C: the mid-edge node reads (triangle.c:7906-7915) - mid1
            // at highorderindex + 1, mid2 at highorderindex + 2, mid3
            // at highorderindex, exactly where highorder stored them.
            let mid1 = m.tri_midnode(tri, 1) as u32;
            let mid2 = m.tri_midnode(tri, 2) as u32;
            let mid3 = m.tri_midnode(tri, 0) as u32;

            trianglelist.push(m.vx_mark(p1));
            trianglelist.push(m.vx_mark(p2));
            trianglelist.push(m.vx_mark(p3));
            trianglelist.push(m.vx_mark(mid1));
            trianglelist.push(m.vx_mark(mid2));
            trianglelist.push(m.vx_mark(mid3));
        }

        // C: the element attributes (triangle.c:7918-7920).
        for i in 0..m.eextras as usize {
            triangleattriblist.push(m.tri_attribute(tri, i));
        }
    }
    Ok(())
}

/// C: writepoly (triangle.c:7933-7975). Write the segments to arrays:
/// walk the subsegment pool in allocation order and record each
/// segment's endpoint numbers - the vertexmark reads return the
/// writenodes numbering (triangle.c:7965-7966) - plus its boundary
/// marker (triangle.c:7967-7970). (C's banner mentions holes, but the
/// array writer emits no hole list: triangle_mesh_copy stores only
/// the hole/region counters, triangle_api.c:404-405.)
///
/// The out-parameters are the owned MeshOutput Vecs (triangle_api.c:
/// 402-403); the endpoint pairs are the frozen surface's segment pairs
/// over C's flat two-int stride, C's allocation branches
/// (triangle.c:7943-7952) are the exact reserves, and the indexed
/// marker store smlist[subsegnumber - firstnumber] (triangle.c:7969)
/// is the push in the same traversal order.
pub(crate) fn writepoly(
    m: &mut Mesh,
    b: &Behavior,
    segmentlist: &mut Vec<[i32; 2]>,
    segmentmarkerlist: &mut Vec<i32>,
) -> Result<(), Oom> {
    // Allocate memory for output segments if necessary
    // (triangle.c:7943-7947): one endpoint pair per subsegment.
    segmentlist
        .try_reserve_exact(m.subsegs.live_count() as usize)
        .map_err(|_| Oom)?;
    // Allocate memory for output segment markers if necessary
    // (triangle.c:7948-7952).
    if !b.nobound {
        segmentmarkerlist
            .try_reserve_exact(m.subsegs.live_count() as usize)
            .map_err(|_| Oom)?;
    }

    // C: traversalinit + subsegloop.ssorient = 0 (triangle.c:7957-7959).
    let mut t = m.subsegs.traversal();
    while let Some(ss) = m.subsegtraverse(&mut t) {
        let subsegloop = Osub { ss, ssorient: 0 };
        let endpoint1 = m.sorg(subsegloop);
        let endpoint2 = m.sdest(subsegloop);
        // Copy indices of the segment's two endpoints
        // (triangle.c:7964-7966).
        segmentlist.push([m.vx_mark(endpoint1), m.vx_mark(endpoint2)]);
        if !b.nobound {
            // Copy the boundary marker (triangle.c:7967-7970).
            segmentmarkerlist.push(m.sub_mark(subsegloop.ss));
        }
    }
    Ok(())
}

/// C: writeedges (triangle.c:7983-8046). Write the edges to arrays:
/// loop over all triangles and look at the three edges of each; emit
/// an edge when there is no adjacent triangle, or when the current
/// triangle has a smaller pointer than its neighbor, so each edge is
/// considered only once (the C comment, triangle.c:8011-8016). The
/// endpoint numbers are the writenodes numbering read back through
/// vertexmark (triangle.c:8024-8025); the marker is the abutting
/// subsegment's boundary marker (zero without one) under
/// usesegments, and the hull flag itself otherwise (triangle.c:
/// 8026-8039).
pub(crate) fn writeedges(
    m: &mut Mesh,
    b: &Behavior,
    edgelist: &mut Vec<[i32; 2]>,
    edgemarkerlist: &mut Vec<i32>,
) -> Result<(), Oom> {
    // Allocate memory for edges if necessary (triangle.c:7996-7999):
    // one endpoint pair per edge.
    edgelist
        .try_reserve_exact(m.edges as usize)
        .map_err(|_| Oom)?;
    // Allocate memory for edge markers if necessary
    // (triangle.c:8000-8003).
    if !b.nobound {
        edgemarkerlist
            .try_reserve_exact(m.edges as usize)
            .map_err(|_| Oom)?;
    }

    // C: traversalinit + the edgenumber walk (triangle.c:8008-8010);
    // the indexed marker store emlist[edgenumber - firstnumber]
    // (triangle.c:8033-8038) is the push in emission order.
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.triangletraverse(&mut t) {
        for orient in 0..3u8 {
            let triangleloop = Otri { tri, orient };
            let trisym = m.sym(triangleloop);
            // C: (triangleloop.tri < trisym.tri) ||
            // (trisym.tri == m->dummytri) (triangle.c:8021), the
            // address compare first. The port evaluates the dummy arm
            // first as a deliberate, registered deviation (D28, the
            // D18 precedent at triangle.c:7653): the packed row-id
            // mirror of the address order cannot evaluate the reserved
            // DUMMY_TRI sentinel row, so the dummy arm must
            // short-circuit before it. The boolean is equivalent on
            // every input (a hull edge passes through the dummy arm
            // either way; an interior edge is emitted from the same
            // side through the oracle address order, ADR 0003 section
            // 5).
            if trisym.tri == DUMMY_TRI
                || m.triangles.oracle_address_order(tri, trisym.tri) == Ordering::Less
            {
                let p1 = m.org(triangleloop);
                let p2 = m.dest(triangleloop);
                edgelist.push([m.vx_mark(p1), m.vx_mark(p2)]);
                if !b.nobound {
                    // C's empty then-branch (triangle.c:8026-8027)
                    // stays an if-not-nobound. Edge number, indices
                    // of two endpoints, and a boundary marker: if
                    // there's no subsegment, the boundary marker is
                    // zero (the C comment, triangle.c:8028-8029).
                    if b.usesegments {
                        let checkmark = m.tspivot(triangleloop);
                        if checkmark.ss == DUMMY_SUB {
                            edgemarkerlist.push(0);
                        } else {
                            edgemarkerlist.push(m.sub_mark(checkmark.ss));
                        }
                    } else {
                        // C: emlist[...] = trisym.tri == m->dummytri
                        // (triangle.c:8038) - the hull flag itself.
                        edgemarkerlist.push((trisym.tri == DUMMY_TRI) as i32);
                    }
                }
            }
        }
    }
    Ok(())
}

/// C: writeneighbors (triangle.c:8054-8130). Write the list of
/// neighbours to an array: number the triangles by overwriting the
/// first subseg pointer with an integer id (triangle.c:8080-8093),
/// read each element's three neighbours back through the same slot
/// (triangle.c:8100-8113), then restore the original subseg values
/// (triangle.c:8119-8129).
///
/// C puns the row's word 6 as an int (its own TODO at triangle.c:
/// 8073-8076: "(triangleloop.tri + 6) actually is a pointer to a
/// subseg and not an int"); the port mirrors the pun as the D2 int
/// overlay the row shape carries - the numberindex overlay of
/// initializetrisubpools, the same 4 bytes at the same slot - saving
/// and restoring through the chokepoint exactly like C's scratch
/// array. Outer space's neighbour reads -1 (triangle.c:8094), and C
/// never restores the dummy's slot (the restore loop walks only the
/// traversed triangles), so the port leaves the sentinel's overlay
/// at -1 exactly like C.
pub(crate) fn writeneighbors(
    m: &mut Mesh,
    b: &Behavior,
    neighborlist: &mut Vec<i32>,
) -> Result<(), Oom> {
    // Allocate memory for neighbors if necessary
    // (triangle.c:8065-8069).
    neighborlist
        .try_reserve_exact(m.triangles.live_count() as usize * 3)
        .map_err(|_| Oom)?;

    // The -n numbering overlay: the widened row exists only under -n
    // (C extends trisize at triangle.c:491-495); a neighbors request
    // without the option puns past C's row and the port refuses it
    // loudly at the chokepoint (D2's stance on the C UB).
    let numberindex = m
        .tri_shape
        .numberindex
        .expect("the writeneighbors numbering overlay exists only with -n (C: triangle.c:491-495)");

    // C: the trimalloc'd scratch (triangle.c:8078), one int per
    // triangle; the pushes are C's subsegs[index] stores in the same
    // allocation order.
    let mut subsegs: Vec<i32> = Vec::new();
    subsegs
        .try_reserve_exact(m.triangles.live_count() as usize)
        .map_err(|_| Oom)?;

    // Number the triangles by overwriting the first subseg pointer
    // with an integer id (triangle.c:8080-8093; the save at 8088, the
    // store at 8089).
    let mut elementnumber = b.firstnumber;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.triangletraverse(&mut t) {
        subsegs.push(m.tri_int(tri, numberindex));
        m.set_tri_int(tri, numberindex, elementnumber);
        elementnumber += 1;
    }

    // C: *(int *)(m->dummytri + 6) = -1 (triangle.c:8094).
    m.set_tri_int(DUMMY_TRI, numberindex, -1);

    // C: the second walk (triangle.c:8096-8117): neighbor1 from
    // orient 1, neighbor2 from orient 2, neighbor3 from orient 0; C's
    // elementnumber cursor (8099, 8116) is absorbed by the push order.
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.triangletraverse(&mut t) {
        let trisym = m.sym(Otri { tri, orient: 1 });
        let neighbor1 = m.tri_int(trisym.tri, numberindex);
        let trisym = m.sym(Otri { tri, orient: 2 });
        let neighbor2 = m.tri_int(trisym.tri, numberindex);
        let trisym = m.sym(Otri { tri, orient: 0 });
        let neighbor3 = m.tri_int(trisym.tri, numberindex);

        neighborlist.push(neighbor1);
        neighborlist.push(neighbor2);
        neighborlist.push(neighbor3);
    }

    // Restore original subseg values (triangle.c:8119-8129; the
    // store at 8126).
    let mut index = 0usize;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.triangletraverse(&mut t) {
        m.set_tri_int(tri, numberindex, subsegs[index]);
        index += 1;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::Error;
    use crate::options::Options;

    /// A fresh core mesh for a load of `numberofpoints` points with
    /// `numberofpointattribs` attributes: the pool geometry Mesh::new
    /// pins, pre-fed the same input counters the api.rs construct path
    /// feeds it.
    fn core_mesh(b: &Behavior, numberofpoints: i32, numberofpointattribs: i32) -> Mesh {
        Mesh::new(b, numberofpoints, 2, numberofpointattribs, 0).expect("mesh init")
    }

    fn behavior() -> Behavior {
        Options::new().validate().expect("validate")
    }

    fn weighted_behavior() -> Behavior {
        let mut opts = Options::new();
        opts.weighted();
        opts.validate().expect("validate")
    }

    /// A three-point load off the axes (every bound extreme is a
    /// distinct fixture point), one attribute per point, explicit
    /// markers.
    fn three_points() -> ([Point2; 3], [f64; 3], [i32; 3]) {
        (
            [
                Point2 { x: 2.0, y: 3.0 },
                Point2 { x: -1.0, y: 0.5 },
                Point2 { x: 4.0, y: -2.0 },
            ],
            [7.0, 8.0, 9.0],
            [5, 0, -3],
        )
    }

    #[test]
    fn three_point_load_pins_the_c_shape() {
        // transfernodes loads the three points into three INPUTVERTEX
        // rows in input order - coordinates, attribute, marker each -
        // after the counter stores (triangle.c:7707-7709), with the
        // bounds (triangle.c:7741-7749) and the sweepline flag
        // (triangle.c:7754).
        let mut b = behavior();
        let mut m = core_mesh(&b, 3, 1);
        let (points, attrs, markers) = three_points();
        transfernodes(&mut m, &mut b, &points, &attrs, Some(&markers), 3, 1).expect("load");
        assert_eq!(m.invertices, 3);
        assert_eq!(m.mesh_dim, 2);
        assert_eq!(m.nextras, 1);
        let mut t = m.vertices.traversal();
        for (k, point) in points.iter().enumerate() {
            let v = m.vertextraverse(&mut t).expect("three rows");
            assert_eq!(m.vx_x(v), point.x);
            assert_eq!(m.vx_y(v), point.y);
            assert_eq!(m.vx_real(v, 2), attrs[k]);
            assert_eq!(m.vx_mark(v), markers[k]);
            assert_eq!(m.vx_type(v), INPUTVERTEX);
        }
        assert!(m.vertextraverse(&mut t).is_none());
        assert_eq!(m.xmin, -1.0);
        assert_eq!(m.xmax, 4.0);
        assert_eq!(m.ymin, -2.0);
        assert_eq!(m.ymax, 3.0);
        assert_eq!(m.xminextreme, -46.0);
    }

    #[test]
    fn markers_default_to_zero_without_a_list() {
        // C's NULL-vs-present branch (triangle.c:7732-7738): without a
        // marker list every row keeps the default 0; without
        // attributes no attribute word is written.
        let mut b = behavior();
        let mut m = core_mesh(&b, 3, 0);
        let (points, _attrs, _markers) = three_points();
        transfernodes(&mut m, &mut b, &points, &[], None, 3, 0).expect("load");
        assert_eq!(m.nextras, 0);
        let mut t = m.vertices.traversal();
        while let Some(v) = m.vertextraverse(&mut t) {
            assert_eq!(m.vx_mark(v), 0);
            assert_eq!(m.vx_type(v), INPUTVERTEX);
        }
    }

    #[test]
    fn fewer_than_three_points_raises_the_input_error() {
        // C returns TRI_INPUT (triangle.c:7710-7712) after the counter
        // stores and before the weighted reset; the port raises
        // Error::Input at exactly that return point (rung 2, D8), the
        // same place triangle_api.c returns the status
        // (triangle_api.c:204-206).
        let mut b = weighted_behavior();
        let mut m = core_mesh(&b, 2, 0);
        let points = [Point2 { x: 0.0, y: 0.0 }, Point2 { x: 1.0, y: 0.0 }];
        let err = transfernodes(&mut m, &mut b, &points, &[], Some(&[1, 2]), 2, 0)
            .expect_err("fewer than three points must fail");
        // The raising site keeps the funnel arm's exact payload.
        assert!(matches!(err, Error::Input(ref msg) if msg == "invalid input (TRI_INPUT)"));
        // The stores at triangle.c:7707-7709 ran before the check...
        assert_eq!(m.invertices, 2);
        assert_eq!(m.mesh_dim, 2);
        assert_eq!(m.nextras, 0);
        // ...and the weighted reset at triangle.c:7713-7715 did not.
        assert_eq!(b.weighted, 1);
    }

    #[test]
    fn weighted_resets_without_attributes() {
        // triangle.c:7713-7715: -w with no point attributes collapses
        // to the unweighted triangulation; with attributes the weight
        // survives the load.
        let mut b = weighted_behavior();
        let mut m = core_mesh(&b, 3, 0);
        let (points, _attrs, _markers) = three_points();
        transfernodes(&mut m, &mut b, &points, &[], None, 3, 0).expect("load");
        assert_eq!(b.weighted, 0);

        let mut b = weighted_behavior();
        let mut m = core_mesh(&b, 3, 1);
        let (points, attrs, _markers) = three_points();
        transfernodes(&mut m, &mut b, &points, &attrs, None, 3, 1).expect("load");
        assert_eq!(b.weighted, 1);
    }

    /// An assembled Behavior from a closure over Options (the
    /// option-gated writer shapes; Options::validate runs C's
    /// behavior_update tail exactly like the entry points).
    fn behavior_with(setup: impl FnOnce(&mut Options)) -> Behavior {
        let mut opts = Options::new();
        setup(&mut opts);
        opts.validate().expect("validate")
    }

    /// The two-triangle unit square of the algorithm/refine.rs highorder
    /// fixture: t1 (a, e, c) bonded to t2 (a, c, d) across t1's
    /// orient-2 edge (the diagonal a-c); the vertices are the pool
    /// rows 1..=4 in traversal order, the triangles the rows 2 and 3
    /// (both returned by row id).
    fn square_two_triangles(b: &Behavior, eextras: i32) -> (Mesh, u32, u32, [u32; 4]) {
        let mut m = Mesh::new(b, 8, 2, 0, eextras).expect("mesh init");
        let a = m.vx_alloc().expect("vx");
        m.set_vx_x(a, 0.0);
        m.set_vx_y(a, 0.0);
        let e = m.vx_alloc().expect("vx");
        m.set_vx_x(e, 1.0);
        m.set_vx_y(e, 0.0);
        let c = m.vx_alloc().expect("vx");
        m.set_vx_x(c, 1.0);
        m.set_vx_y(c, 1.0);
        let d = m.vx_alloc().expect("vx");
        m.set_vx_x(d, 0.0);
        m.set_vx_y(d, 1.0);
        let t1 = m.maketriangle(b.usesegments, b.vararea).expect("tri");
        m.setorg(t1, a);
        m.setdest(t1, e);
        m.setapex(t1, c);
        let t2 = m.maketriangle(b.usesegments, b.vararea).expect("tri");
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
        (m, t1.tri, t2.tri, [a, e, c, d])
    }

    #[test]
    fn writenodes_marshals_and_numbers_over_the_marks() {
        // The writenodes walk (triangle.c:7806-7827) over a
        // transfernodes load: the points come back in traversal
        // order with their attributes and markers, and the numbering
        // trick writes the numbers over the marks exactly where C
        // does (triangle.c:7823).
        let mut b = behavior();
        let mut m = core_mesh(&b, 3, 1);
        let (points, attrs, markers) = three_points();
        transfernodes(&mut m, &mut b, &points, &attrs, Some(&markers), 3, 1).expect("load");
        let mut plist: Vec<Point2> = Vec::new();
        let mut palist: Vec<f64> = Vec::new();
        let mut pmlist: Vec<i32> = Vec::new();
        writenodes(&mut m, &b, &mut plist, &mut palist, &mut pmlist).expect("writenodes");
        assert_eq!(plist, points.to_vec());
        assert_eq!(palist, attrs.to_vec());
        // the ORIGINAL markers leave in the list...
        assert_eq!(pmlist, markers.to_vec());
        // ...and the numbers sit over the marks in the rows (the
        // banner's memory trick, triangle.c:7763-7764).
        let mut number = 1;
        let mut t = m.vertices.traversal();
        while let Some(v) = m.vertextraverse(&mut t) {
            assert_eq!(m.vx_mark(v), number);
            number += 1;
        }
        // -z shifts the walk's start exactly like C's vertexnumber
        // init (triangle.c:7807).
        let mut b = behavior_with(|o| {
            o.zero_based();
        });
        let mut m = core_mesh(&b, 3, 0);
        let (points, _attrs, _markers) = three_points();
        transfernodes(&mut m, &mut b, &points, &[], None, 3, 0).expect("load");
        writenodes(
            &mut m,
            &b,
            &mut Vec::new(),
            &mut Vec::new(),
            &mut Vec::new(),
        )
        .expect("writenodes");
        let mut t = m.vertices.traversal();
        assert_eq!(m.vx_mark(m.vertextraverse(&mut t).expect("vx")), 0);
    }

    #[test]
    fn writenodes_without_markers_still_numbers() {
        // The !nobound gate (triangle.c:7818-7821): no marker list is
        // written, and the numbering store (7823) stays outside the
        // gate.
        let mut b = behavior_with(|o| {
            o.no_bounds();
        });
        let mut m = core_mesh(&b, 3, 1);
        let (points, attrs, _markers) = three_points();
        transfernodes(&mut m, &mut b, &points, &attrs, None, 3, 1).expect("load");
        let mut plist: Vec<Point2> = Vec::new();
        let mut palist: Vec<f64> = Vec::new();
        let mut pmlist: Vec<i32> = Vec::new();
        writenodes(&mut m, &b, &mut plist, &mut palist, &mut pmlist).expect("writenodes");
        assert_eq!(plist, points.to_vec());
        assert_eq!(palist, attrs.to_vec());
        assert!(pmlist.is_empty());
        let mut t = m.vertices.traversal();
        assert_eq!(m.vx_mark(m.vertextraverse(&mut t).expect("vx")), 1);
    }

    #[test]
    fn writenodes_jettison_skips_the_undead() {
        // The jettison gate (triangle.c:7810): an UNDEADVERTEX row is
        // skipped in the output and keeps its old marker (no number
        // lands on it), and the numbering of the later vertices runs
        // on without it (7823-7824 inside the gate).
        let mut b = behavior_with(|o| {
            o.jettison();
        });
        let mut m = core_mesh(&b, 3, 0);
        let (points, _attrs, _markers) = three_points();
        transfernodes(&mut m, &mut b, &points, &[], None, 3, 0).expect("load");
        let mut t = m.vertices.traversal();
        let v1 = m.vertextraverse(&mut t).expect("vx");
        let v2 = m.vertextraverse(&mut t).expect("vx");
        let v3 = m.vertextraverse(&mut t).expect("vx");
        // the graveyard row the way divconqdelaunay leaves it (type
        // UNDEADVERTEX; m->undeads is the C counter shape)
        m.vx_set_type(v2, UNDEADVERTEX);
        m.undeads = 1;
        let mut plist: Vec<Point2> = Vec::new();
        let mut palist: Vec<f64> = Vec::new();
        let mut pmlist: Vec<i32> = Vec::new();
        writenodes(&mut m, &b, &mut plist, &mut palist, &mut pmlist).expect("writenodes");
        assert_eq!(plist, vec![points[0], points[2]]);
        assert_eq!(pmlist, vec![0, 0]);
        // v1 is numbered 1, the undead keeps the default 0 mark, v3 is
        // numbered 2 (the numbers advance past the living only)
        assert_eq!(m.vx_mark(v1), 1);
        assert_eq!(m.vx_mark(v2), 0);
        assert_eq!(m.vx_mark(v3), 2);
    }

    #[test]
    fn numbernodes_numbers_all_and_advances_past_the_living() {
        // C numbernodes (triangle.c:7845-7854): every vertex gets a
        // marker equal to its number, the undead included, while the
        // number advances only past non-undead rows under jettison
        // (7850-7852).
        let mut b = behavior_with(|o| {
            o.jettison();
        });
        let mut m = core_mesh(&b, 3, 0);
        let (points, _attrs, _markers) = three_points();
        transfernodes(&mut m, &mut b, &points, &[], None, 3, 0).expect("load");
        let mut t = m.vertices.traversal();
        let v1 = m.vertextraverse(&mut t).expect("vx");
        let v2 = m.vertextraverse(&mut t).expect("vx");
        let v3 = m.vertextraverse(&mut t).expect("vx");
        m.vx_set_type(v2, UNDEADVERTEX);
        m.undeads = 1;
        numbernodes(&mut m, &b);
        assert_eq!(m.vx_mark(v1), 1);
        // the undead row is numbered with the current number (the
        // number v1's advance left) and does not advance it, so v3
        // re-receives the same number (the C collision is unobservable
        // in output: writenodes never writes an undead row)
        assert_eq!(m.vx_mark(v2), 2);
        assert_eq!(m.vx_mark(v3), 2);
    }

    #[test]
    fn writeelements_records_the_corner_numbers() {
        // C writeelements (triangle.c:7893-7924) over the
        // two-triangle square, numbered by numbernodes: the corners
        // are the vertex numbers in org/dest/apex order, and the
        // element attributes follow in their flat list (7918-7920).
        let b = behavior();
        let (mut m, t1, t2, _v) = square_two_triangles(&b, 2);
        // the attribute payloads of the two rows
        m.set_tri_attribute(t1, 0, 7.0);
        m.set_tri_attribute(t1, 1, 8.0);
        m.set_tri_attribute(t2, 0, 9.0);
        m.set_tri_attribute(t2, 1, 10.0);
        numbernodes(&mut m, &b);
        let mut tlist: Vec<i32> = Vec::new();
        let mut talist: Vec<f64> = Vec::new();
        writeelements(&mut m, &b, &mut tlist, &mut talist).expect("writeelements");
        assert_eq!(tlist, vec![1, 2, 3, 1, 3, 4]);
        assert_eq!(talist, vec![7.0, 8.0, 9.0, 10.0]);
    }

    #[test]
    fn writeelements_above_order_one_adds_the_mid_edge_nodes() {
        // C writeelements' order-2 branch (triangle.c:7906-7915):
        // after highorder records the mid-edge nodes, the six corners
        // are p1, p2, p3, mid1 (highorderindex + 1), mid2
        // (highorderindex + 2), mid3 (highorderindex) - the node ids
        // highorder stored per edge orientation.
        let b = behavior_with(|o| {
            o.element_order(2);
        });
        let (mut m, _t1, _t2, _v) = square_two_triangles(&b, 0);
        crate::algorithm::refine::highorder(&mut m, &b).expect("highorder");
        numbernodes(&mut m, &b);
        let mut tlist: Vec<i32> = Vec::new();
        let mut talist: Vec<f64> = Vec::new();
        writeelements(&mut m, &b, &mut tlist, &mut talist).expect("writeelements");
        // the five mid-edge nodes are pool rows 5..=9 in creation
        // order (t1's edges 0, 1, 2 then t2's edges 1, 2; the
        // interior edge's node is created once, from t1), so the
        // numbers are 5..=9 and each element carries the mid nodes
        // of its edges 1, 2, 0 in C's mid1/mid2/mid3 order
        assert_eq!(tlist, vec![1, 2, 3, 6, 7, 5, 1, 3, 4, 8, 9, 7]);
    }

    #[test]
    fn writepoly_records_endpoint_numbers_and_markers() {
        // C writepoly (triangle.c:7957-7974) over two hand-made
        // subsegments on the square's vertices (numbered by
        // numbernodes): the endpoint pairs are the vertexmark numbers
        // of sorg/sdest at ssorient 0 (triangle.c:7965-7966), the
        // markers the subseg boundary markers in allocation order
        // (7967-7970).
        let b = behavior_with(|o| {
            o.pslg();
        });
        let (mut m, _t1, _t2, v) = square_two_triangles(&b, 0);
        let [a, e, c, _d] = v;
        let s1 = m.makesubseg().expect("subseg");
        m.setsorg(s1, a);
        m.setsdest(s1, e);
        m.set_sub_mark(s1.ss, 7);
        let s2 = m.makesubseg().expect("subseg");
        m.setsorg(s2, e);
        m.setsdest(s2, c);
        m.set_sub_mark(s2.ss, 8);
        numbernodes(&mut m, &b);
        let mut slist: Vec<[i32; 2]> = Vec::new();
        let mut smlist: Vec<i32> = Vec::new();
        writepoly(&mut m, &b, &mut slist, &mut smlist).expect("writepoly");
        assert_eq!(slist, vec![[1, 2], [2, 3]]);
        assert_eq!(smlist, vec![7, 8]);
        // -B: the segment list is written, the markers are not
        // (triangle.c:7949, 7967).
        let b = behavior_with(|o| {
            o.pslg();
            o.no_bounds();
        });
        let (mut m, _t1, _t2, v) = square_two_triangles(&b, 0);
        let [a, e, _c, _d] = v;
        let s1 = m.makesubseg().expect("subseg");
        m.setsorg(s1, a);
        m.setsdest(s1, e);
        numbernodes(&mut m, &b);
        let mut slist: Vec<[i32; 2]> = Vec::new();
        let mut smlist: Vec<i32> = Vec::new();
        writepoly(&mut m, &b, &mut slist, &mut smlist).expect("writepoly");
        assert_eq!(slist, vec![[1, 2]]);
        assert!(smlist.is_empty());
    }

    #[test]
    fn writeedges_visits_every_edge_once() {
        // C writeedges (triangle.c:8017-8045) over the two-triangle
        // square without segments: the five edges are emitted from
        // t1's three orients then t2's hull orients (the interior
        // edge from the lower-id side, the address order through the
        // D28 dummy-first conjunct), and the markers are the hull
        // flag (triangle.c:8038).
        let b = behavior();
        let (mut m, _t1, _t2, _v) = square_two_triangles(&b, 0);
        m.edges = 5; // the m->edges writer of the C entry points
        numbernodes(&mut m, &b);
        let mut elist: Vec<[i32; 2]> = Vec::new();
        let mut emlist: Vec<i32> = Vec::new();
        writeedges(&mut m, &b, &mut elist, &mut emlist).expect("writeedges");
        assert_eq!(elist, vec![[1, 2], [2, 3], [3, 1], [3, 4], [4, 1]]);
        assert_eq!(emlist, vec![1, 1, 0, 1, 1]);
    }

    #[test]
    fn writeedges_marks_the_segment_edges_under_pslg() {
        // The usesegments branch (triangle.c:8030-8036): a bonded
        // subsegment lends its boundary marker to the edge, an
        // unbonded edge writes 0.
        let b = behavior_with(|o| {
            o.pslg();
        });
        let (mut m, t1, _t2, _v) = square_two_triangles(&b, 0);
        let s = m.makesubseg().expect("subseg");
        m.set_sub_mark(s.ss, 7);
        m.tsbond(Otri { tri: t1, orient: 0 }, s);
        m.edges = 5;
        numbernodes(&mut m, &b);
        let mut elist: Vec<[i32; 2]> = Vec::new();
        let mut emlist: Vec<i32> = Vec::new();
        writeedges(&mut m, &b, &mut elist, &mut emlist).expect("writeedges");
        assert_eq!(elist, vec![[1, 2], [2, 3], [3, 1], [3, 4], [4, 1]]);
        assert_eq!(emlist, vec![7, 0, 0, 0, 0]);
    }

    #[test]
    fn writeneighbors_numbers_reads_and_restores() {
        // C writeneighbors (triangle.c:8083-8129) over the
        // two-triangle square under -n: the numbering writes the
        // element numbers over word 6's low half, the neighbor reads
        // see them (outer space -1, triangle.c:8094) and the restore
        // puts the original overlay values back.
        let b = behavior_with(|o| {
            o.neighbors();
        });
        let (mut m, t1, t2, _v) = square_two_triangles(&b, 0);
        let mut nlist: Vec<i32> = Vec::new();
        writeneighbors(&mut m, &b, &mut nlist).expect("writeneighbors");
        // t1 is element 1, t2 element 2 (allocation order); t1's
        // neighbors from orients 1, 2, 0: hull, t2, hull; t2's: hull,
        // hull, t1 (triangle.c:8101-8113).
        assert_eq!(nlist, vec![-1, 2, -1, -1, -1, 1]);
        // the overlay is restored (the original zero of the unused
        // payload word)...
        let numberindex = m.tri_shape.numberindex.expect("overlay");
        assert_eq!(m.tri_int(t1, numberindex), 0);
        assert_eq!(m.tri_int(t2, numberindex), 0);
        // ...and the dummy's slot keeps C's -1 (triangle.c:8094,
        // never restored: the restore loop walks only the traversed
        // rows)
        assert_eq!(m.tri_int(DUMMY_TRI, numberindex), -1);
    }

    #[test]
    fn writeneighbors_restores_the_subseg_slots_under_pslg() {
        // The -p twin: word 6 is a live subseg slot, and the C pun
        // saves and restores its low half around the numbering
        // (triangle.c:8078-8093, 8119-8129) - the bonded edge pivots
        // to its subsegment again after the writer ran.
        let b = behavior_with(|o| {
            o.pslg();
            o.neighbors();
        });
        let (mut m, t1, _t2, _v) = square_two_triangles(&b, 0);
        let s = m.makesubseg().expect("subseg");
        m.tsbond(Otri { tri: t1, orient: 0 }, s);
        let mut nlist: Vec<i32> = Vec::new();
        writeneighbors(&mut m, &b, &mut nlist).expect("writeneighbors");
        assert_eq!(nlist, vec![-1, 2, -1, -1, -1, 1]);
        assert_eq!(m.tspivot(Otri { tri: t1, orient: 0 }), s);
    }

    #[test]
    #[should_panic(expected = "the writeneighbors numbering overlay exists only with -n")]
    fn writeneighbors_without_the_option_refuses_the_out_of_row_pun() {
        // The widened row exists only under -n (C: triangle.c:
        // 491-495); C puns past the row here (its own TODO,
        // triangle.c:8073-8076), the port refuses loudly at the
        // chokepoint (D2's stance on the C UB).
        let b = behavior_with(|o| {
            o.pslg();
        });
        let (mut m, _t1, _t2, _v) = square_two_triangles(&b, 0);
        let mut nlist: Vec<i32> = Vec::new();
        let _ = writeneighbors(&mut m, &b, &mut nlist);
    }

    #[test]
    fn writeneighbors_on_an_empty_mesh_still_numbers_outer_space() {
        // With no triangles the loops do not run, but C's dummy
        // store (triangle.c:8094) is unconditional.
        let b = behavior_with(|o| {
            o.neighbors();
        });
        let mut m = core_mesh(&b, 3, 0);
        let mut nlist: Vec<i32> = Vec::new();
        writeneighbors(&mut m, &b, &mut nlist).expect("writeneighbors");
        assert!(nlist.is_empty());
        let numberindex = m.tri_shape.numberindex.expect("overlay");
        assert_eq!(m.tri_int(DUMMY_TRI, numberindex), -1);
    }
}
