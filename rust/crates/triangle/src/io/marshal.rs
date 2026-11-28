//! The output phase, one deep module: the numbering, marshalling and
//! serialization of the six mesh-bound artifacts (the "Output phase"
//! term of GLOSSARY.md, ADR 0005's phase (e)). Both C output forms
//! live here behind one owner - the array writers of triangle.c
//! (writenodes 7768, numbernodes 7840, writeelements 7863, writepoly
//! 7933, writeedges 7983, writeneighbors 8054-8130) marshalling the
//! mesh into the owned MeshOutput Vecs for Mesh::output (api.rs, the
//! triangle_api.c:377-425 gates), and the file writers of
//! triangle_io.c (file_writenodes through file_write_eps, 23-351)
//! writing io::Write sinks for the six Mesh::write_* wrappers
//! (triangle_api.c:437-492). The interchange formats themselves -
//! the parse seam, the readers, and the header/line/trailer emission
//! machinery - live in the triangle-io crate (ADR 0006, issue #70);
//! this module keeps the Mesh traversal orchestration and calls into
//! triangle-io's emitters for the bytes.
//!
//! One record seam per artifact: the per-item record (number,
//! values, marker) plus the header facts, built once per artifact and
//! consumed by the two adapters - the array collector (pushes into
//! the owned Vecs) and the file formatter (writes lines through
//! triangle-io's emitters). The seam exists so ADR 0005 item (e) can
//! later build the parallel-safe writer body on it; this module adds
//! no MT scaffolding of its own.
//!
//! The numbering discipline stays exactly at its C anchor sites, in
//! each form's own traversal - no renumbering pre-pass (a pre-pass
//! would change observable bytes; the harness would catch it, the
//! code cites the anchors instead):
//! - file_writenodes numbers in its own walk, the setvertexmark
//!   store after the line write (triangle_io.c:64-65), the line
//!   carrying the ORIGINAL marker read before the store;
//! - file_writeneighbors rides the two-walk word-6 overlay with no
//!   save and no restore, the dummy's slot left at -1
//!   (triangle_io.c:282-287);
//! - the array form numbers the way writenodes does, the store after
//!   the record is marshalled (triangle.c:7823), and writeneighbors
//!   saves and restores through the D2 overlay exactly like C's
//!   scratch array (triangle.c:8088-8089, 8119-8129);
//! - number_nodes stays the numbernumbers mirror (triangle.c:
//!   7840-7855).
//!
//! Boundary decisions (unchanged by this consolidation; the
//! deviation register and the PR bodies carry the details): C's
//! unchecked fprintf return values surface as io::Result at this
//! module's file adapter seam (D31's writer row), the FILE* NULL
//! checks dissolve over the api.rs-owned opens (D31), and the
//! trimalloc sizing of the array form is the exact Vec reserves, a
//! failed allocation funneling to Oom where C exits the process
//! (D8).

use std::cmp::Ordering;
use std::io::Write;

use triangle_io::types::{Hole as IoHole, Region as IoRegion};
use triangle_io::write::{
    write_edge_header, write_edge_line, write_ele_header, write_ele_line, write_neigh_header,
    write_neigh_line, write_node_header, write_node_line, write_poly_header, write_poly_holes,
    write_poly_regions, write_segment_line,
};

use crate::input::{Hole, Point2, Region};
use crate::mesh::handles::{Osub, Otri, DUMMY_SUB, DUMMY_TRI};
use crate::mesh::pool::Oom;
use crate::mesh::slots::{HalfWord, UNDEAD_VERTEX};
use crate::mesh::Mesh;
use crate::options::Behavior;

// ---------------------------------------------------------------------------
// The record seams: one per artifact, consumed by the two adapters.
// ---------------------------------------------------------------------------

/// The .node header facts, shared by both adapters: the output vertex
/// count (C's outvertices, triangle.c:7781-7785 / triangle_io.c:34-38,
/// the jettison discount by the undead), the mesh dimension, the
/// attribute count and the boundary flag (triangle_io.c:42-43). The
/// array adapter sizes its exact reserves off the same facts
/// (triangle.c:7787-7799), the file adapter writes the header line
/// from them.
struct NodeFacts {
    outvertices: i64,
    mesh_dim: i32,
    nextras: i32,
    bound: i32,
}

fn node_facts(m: &Mesh, b: &Behavior) -> NodeFacts {
    NodeFacts {
        outvertices: if b.jettison {
            m.vertices.live_count() as i64 - m.undeads as i64
        } else {
            m.vertices.live_count() as i64
        },
        mesh_dim: m.mesh_dim,
        nextras: m.nextras,
        bound: 1 - i32::from(b.nobound),
    }
}

/// The attribute view of a node record: the vertex's rows
/// 2..2+nextras in order, exactly the reads of both C forms
/// (triangle.c:7814-7817 / triangle_io.c:53-60). A lazy view over the
/// mesh, no per-item allocation: both adapters consume the same
/// reads.
struct NodeAttrs<'a> {
    m: &'a Mesh,
    vertex: u32,
    index: usize,
    count: usize,
}

impl<'a> Iterator for NodeAttrs<'a> {
    type Item = f64;

    fn next(&mut self) -> Option<f64> {
        if self.index < self.count {
            let value = self.m.vx_real(self.vertex, 2 + self.index);
            self.index += 1;
            Some(value)
        } else {
            None
        }
    }
}

/// The per-item node record: the vertex number (the caller's cursor,
/// from b->firstnumber), the two coordinates, the attribute values
/// and the ORIGINAL boundary marker - read before the numbering
/// store, so the emitted line/list carries the original marker
/// exactly where both C forms read it (triangle.c:7818-7821 /
/// triangle_io.c:61).
struct NodeRecord<'a> {
    number: i32,
    x: f64,
    y: f64,
    marker: Option<i32>,
    attributes: NodeAttrs<'a>,
}

fn node_record<'a>(m: &'a Mesh, b: &Behavior, vertex: u32, number: i32) -> NodeRecord<'a> {
    NodeRecord {
        number,
        x: m.vx_x(vertex),
        y: m.vx_y(vertex),
        marker: if b.nobound {
            None
        } else {
            Some(m.vx_mark(vertex))
        },
        attributes: NodeAttrs {
            m,
            vertex,
            index: 0,
            count: m.nextras as usize,
        },
    }
}

/// The .ele header facts: the output element count, the corners per
/// element ((order + 1) * (order + 2) / 2) and the attribute count
/// (triangle_io.c:91-92; the array form's reserves, triangle.c:
/// 7876-7887).
struct ElementFacts {
    triangles: i64,
    corners: i32,
    eextras: i32,
}

fn element_facts(m: &Mesh, b: &Behavior) -> ElementFacts {
    ElementFacts {
        triangles: m.triangles.live_count() as i64,
        corners: (b.order + 1) * (b.order + 2) / 2,
        eextras: m.eextras,
    }
}

/// The attribute view of an element record: the triangle's attribute
/// rows in order (triangle.c:7918-7920 / triangle_io.c:116-117).
struct ElementAttrs<'a> {
    m: &'a Mesh,
    tri: u32,
    index: usize,
    count: usize,
}

impl<'a> Iterator for ElementAttrs<'a> {
    type Item = f64;

    fn next(&mut self) -> Option<f64> {
        if self.index < self.count {
            let value = self.m.tri_attribute(self.tri, self.index);
            self.index += 1;
            Some(value)
        } else {
            None
        }
    }
}

/// The per-item element record: the element number, the corner
/// numbers - the writenodes numbering read back through vertexmark
/// (triangle.c:7902-7915 / triangle_io.c:104-113), above order 1 the
/// three mid-edge nodes in C's mid1/mid2/mid3 order - and the
/// attribute values.
struct ElementRecord<'a> {
    number: i64,
    corners: [i32; 6],
    ncorners: usize,
    attributes: ElementAttrs<'a>,
}

fn element_record<'a>(m: &'a Mesh, b: &Behavior, tri: u32, number: i64) -> ElementRecord<'a> {
    let triangleloop = Otri { tri, orient: 0 };
    let p1 = m.org(triangleloop);
    let p2 = m.dest(triangleloop);
    let p3 = m.apex(triangleloop);
    let mut corners = [0i32; 6];
    corners[0] = m.vx_mark(p1);
    corners[1] = m.vx_mark(p2);
    corners[2] = m.vx_mark(p3);
    let ncorners = if b.order != 1 {
        // C: the mid-edge node reads (triangle.c:7906-7915 /
        // triangle_io.c:107-113) - mid1 at highorderindex + 1, mid2
        // at highorderindex + 2, mid3 at highorderindex, exactly
        // where highorder stored them.
        let mid1 = m.tri_midnode(tri, 1) as u32;
        let mid2 = m.tri_midnode(tri, 2) as u32;
        let mid3 = m.tri_midnode(tri, 0) as u32;
        corners[3] = m.vx_mark(mid1);
        corners[4] = m.vx_mark(mid2);
        corners[5] = m.vx_mark(mid3);
        6
    } else {
        3
    };
    ElementRecord {
        number,
        corners,
        ncorners,
        attributes: ElementAttrs {
            m,
            tri,
            index: 0,
            count: m.eextras as usize,
        },
    }
}

/// The .poly header facts: the zero node count (the vertices are in
/// a separate .node file), the dimension and attribute counts, the
/// boundary flags and the segment count (triangle_io.c:148-151; the
/// array form's reserves, triangle.c:7943-7952).
struct SegmentFacts {
    subsegs: i64,
    mesh_dim: i32,
    nextras: i32,
    bound: i32,
}

fn segment_facts(m: &Mesh, b: &Behavior) -> SegmentFacts {
    SegmentFacts {
        subsegs: m.subsegs.live_count() as i64,
        mesh_dim: m.mesh_dim,
        nextras: m.nextras,
        bound: 1 - i32::from(b.nobound),
    }
}

/// The per-item segment record: the segment number, the endpoint
/// numbers - the writenodes numbering read back through vertexmark at
/// ssorient 0 (triangle.c:7965-7966 / triangle_io.c:161-163) - and the
/// subsegment's boundary marker.
struct SegmentRecord {
    number: i64,
    endpoints: [i32; 2],
    marker: Option<i32>,
}

fn segment_record(m: &Mesh, b: &Behavior, ss: u32, number: i64) -> SegmentRecord {
    let subsegloop = Osub { ss, orient: 0 };
    let endpoint1 = m.subseg_org(subsegloop);
    let endpoint2 = m.subseg_dest(subsegloop);
    SegmentRecord {
        number,
        endpoints: [m.vx_mark(endpoint1), m.vx_mark(endpoint2)],
        marker: if b.nobound {
            None
        } else {
            Some(m.sub_mark(subsegloop.ss))
        },
    }
}

/// The .edge header facts: the edge count and the boundary flag
/// (triangle_io.c:217; the array form's reserves, triangle.c:
/// 7996-8003).
struct EdgeFacts {
    edges: i64,
    bound: i32,
}

fn edge_facts(m: &Mesh, b: &Behavior) -> EdgeFacts {
    EdgeFacts {
        edges: m.edges,
        bound: 1 - i32::from(b.nobound),
    }
}

/// The per-item edge record: the edge number, the endpoint numbers -
/// the writenodes numbering read back through vertexmark
/// (triangle.c:8024-8025 / triangle_io.c:237) - and the boundary
/// marker: under usesegments the abutting subsegment's boundary
/// marker, zero without one (triangle.c:8026-8036 / triangle_io.c:
/// 240-254), otherwise the hull flag itself (triangle.c:8038 /
/// triangle_io.c:253).
struct EdgeRecord {
    number: i64,
    endpoints: [i32; 2],
    marker: Option<i32>,
}

fn edge_record(
    m: &Mesh,
    b: &Behavior,
    triangleloop: Otri,
    trisym: Otri,
    number: i64,
) -> EdgeRecord {
    let p1 = m.org(triangleloop);
    let p2 = m.dest(triangleloop);
    EdgeRecord {
        number,
        endpoints: [m.vx_mark(p1), m.vx_mark(p2)],
        marker: if b.nobound {
            None
        } else {
            Some(if b.usesegments {
                let checkmark = m.tri_subseg(triangleloop);
                if checkmark.ss == DUMMY_SUB {
                    0
                } else {
                    m.sub_mark(checkmark.ss)
                }
            } else {
                i32::from(trisym.tri == DUMMY_TRI)
            })
        },
    }
}

/// The .neigh header facts: the element count (triangle_io.c:276).
struct NeighborFacts {
    triangles: i64,
}

fn neighbor_facts(m: &Mesh) -> NeighborFacts {
    NeighborFacts {
        triangles: m.triangles.live_count() as i64,
    }
}

/// The per-item neighbor record: the element number and the three
/// neighbor numbers read off the numbering overlay - neighbor1 from
/// orient 1, neighbor2 from orient 2, neighbor3 from orient 0
/// (triangle.c:8101-8113 / triangle_io.c:295-301), outer space -1.
/// Both adapters run their own numbering walk first (the anchors
/// cited in the module doc); this record is what the walks' second
/// pass consumes.
struct NeighborRecord {
    number: i64,
    neighbors: [i32; 3],
}

fn neighbor_record(m: &Mesh, tri: u32, numberindex: HalfWord, number: i64) -> NeighborRecord {
    NeighborRecord {
        number,
        neighbors: [
            m.tri_int(m.sym(Otri { tri, orient: 1 }).tri, numberindex),
            m.tri_int(m.sym(Otri { tri, orient: 2 }).tri, numberindex),
            m.tri_int(m.sym(Otri { tri, orient: 0 }).tri, numberindex),
        ],
    }
}

// ---------------------------------------------------------------------------
// The array adapter: the triangle.c writers marshalling into the
// owned MeshOutput Vecs (api.rs Mesh::output, triangle_api.c:377-425).
// ---------------------------------------------------------------------------

/// C: writenodes (triangle.c:7768-7828, banner 7759-7766). Number the
/// vertices and write them to arrays: walk the vertex pool in
/// allocation order, marshal each non-jettisoned vertex's record into
/// the output lists, then write the vertex number over the boundary
/// marker through the vx chokepoint - the banner's memory trick
/// (triangle.c:7763-7764: "To save memory, the vertex numbers are
/// written over the boundary markers") - so the later writers read
/// the numbers back through vertexmark, and triangle_mesh_copy
/// restores the originals from the marker list at the end
/// (triangle_api.c:423-425).
///
/// The out-parameters are the owned MeshOutput Vecs handed in by
/// api.rs exactly where C passes its triangleio members
/// (triangle_api.c:393-395); C's "allocate if NULL" branches
/// (triangle.c:7787-7799) are the exact Vec reserves over the shared
/// header facts, a failed allocation funneling to Oom where C's
/// trimalloc exits the process (D8). C's coordindex/attribindex
/// cursor pair (triangle.c:7803-7804) is absorbed by the push order,
/// and the indexed marker store pmlist[vertexnumber - firstnumber]
/// (triangle.c:7820) is the push in the same traversal order (the
/// marker list counts only the output vertices).
pub(crate) fn write_nodes(
    m: &mut Mesh,
    b: &Behavior,
    pointlist: &mut Vec<Point2>,
    pointattriblist: &mut Vec<f64>,
    pointmarkerlist: &mut Vec<i32>,
) -> Result<(), Oom> {
    let facts = node_facts(m, b);

    // Allocate memory for output vertices if necessary
    // (triangle.c:7787-7789).
    pointlist
        .try_reserve_exact(facts.outvertices as usize)
        .map_err(|_| Oom)?;
    // Allocate memory for output vertex attributes if necessary
    // (triangle.c:7791-7795).
    if facts.nextras > 0 {
        pointattriblist
            .try_reserve_exact(facts.outvertices as usize * facts.nextras as usize)
            .map_err(|_| Oom)?;
    }
    // Allocate memory for output vertex markers if necessary
    // (triangle.c:7796-7799).
    if !b.nobound {
        pointmarkerlist
            .try_reserve_exact(facts.outvertices as usize)
            .map_err(|_| Oom)?;
    }

    // C: traversalinit + the vertexnumber walk (triangle.c:7806-7808).
    let mut vertexnumber = b.firstnumber;
    let mut t = m.vertices.traversal();
    while let Some(vertexloop) = m.traverse_vertex(&mut t) {
        // C: the jettison gate (triangle.c:7810).
        if !b.jettison || m.vx_type(vertexloop) != UNDEAD_VERTEX {
            let record = node_record(m, b, vertexloop, vertexnumber);
            // X and y coordinates (triangle.c:7811-7813), the
            // attributes (7814-7817) and the boundary marker
            // (7818-7821) - the record's original marker.
            pointlist.push(Point2 {
                x: record.x,
                y: record.y,
            });
            for attribute in record.attributes {
                pointattriblist.push(attribute);
            }
            if let Some(marker) = record.marker {
                pointmarkerlist.push(marker);
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
                    // out); the file writer (file_write_nodes, this
                    // module) inlines the numbering exactly like C's
                    // file_writenodes (triangle_io.c:64-65).
pub(crate) fn number_nodes(m: &mut Mesh, b: &Behavior) {
    let mut vertexnumber = b.firstnumber;
    let mut t = m.vertices.traversal();
    while let Some(vertexloop) = m.traverse_vertex(&mut t) {
        m.vx_set_mark(vertexloop, vertexnumber);
        if !b.jettison || m.vx_type(vertexloop) != UNDEAD_VERTEX {
            vertexnumber += 1;
        }
    }
}

/// C: writeelements (triangle.c:7863-7925). Write the triangles to
/// arrays: walk the triangle pool in allocation order and marshal
/// each element's record - the corner numbers are the writenodes
/// numbering read back through vertexmark (triangle.c:7902-7915).
///
/// The out-parameters are the owned MeshOutput Vecs (triangle_api.c:
/// 397-398); C's allocation branches (triangle.c:7876-7887) are the
/// exact reserves over the shared header facts.
pub(crate) fn write_elements(
    m: &mut Mesh,
    b: &Behavior,
    trianglelist: &mut Vec<i32>,
    triangleattriblist: &mut Vec<f64>,
) -> Result<(), Oom> {
    let facts = element_facts(m, b);

    // Allocate memory for output triangles if necessary
    // (triangle.c:7876-7881).
    trianglelist
        .try_reserve_exact(facts.triangles as usize * facts.corners as usize)
        .map_err(|_| Oom)?;
    // Allocate memory for output triangle attributes if necessary
    // (triangle.c:7882-7887).
    if facts.eextras > 0 {
        triangleattriblist
            .try_reserve_exact(facts.triangles as usize * facts.eextras as usize)
            .map_err(|_| Oom)?;
    }

    // C: traversalinit + triangleloop.orient = 0 (triangle.c:7893-7895);
    // C's elementnumber cursor (7896, 7923) rides the record's number
    // field and advances with the push order.
    let mut elementnumber: i64 = b.firstnumber as i64;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.traverse_triangle(&mut t) {
        let record = element_record(m, b, tri, elementnumber);
        trianglelist.extend_from_slice(&record.corners[..record.ncorners]);
        // C: the element attributes (triangle.c:7918-7920).
        for attribute in record.attributes {
            triangleattriblist.push(attribute);
        }
        elementnumber += 1;
    }
    Ok(())
}

/// C: writepoly (triangle.c:7933-7975). Write the segments to arrays:
/// walk the subsegment pool in allocation order and marshal each
/// segment's record - the endpoint numbers are the writenodes
/// numbering read back through vertexmark (triangle.c:7965-7966),
/// the marker the subsegment's boundary marker (7967-7970). (C's
/// banner mentions holes, but the array writer emits no hole list:
/// triangle_mesh_copy stores only the hole/region counters,
/// triangle_api.c:404-405.)
///
/// The out-parameters are the owned MeshOutput Vecs (triangle_api.c:
/// 402-403); the endpoint pairs are the frozen surface's segment pairs
/// over C's flat two-int stride, C's allocation branches
/// (triangle.c:7943-7952) are the exact reserves over the shared
/// header facts, and the indexed marker store
/// smlist[subsegnumber - firstnumber] (triangle.c:7969) is the push
/// in the same traversal order.
pub(crate) fn write_poly(
    m: &mut Mesh,
    b: &Behavior,
    segmentlist: &mut Vec<[i32; 2]>,
    segmentmarkerlist: &mut Vec<i32>,
) -> Result<(), Oom> {
    let facts = segment_facts(m, b);

    // Allocate memory for output segments if necessary
    // (triangle.c:7943-7947): one endpoint pair per subsegment.
    segmentlist
        .try_reserve_exact(facts.subsegs as usize)
        .map_err(|_| Oom)?;
    // Allocate memory for output segment markers if necessary
    // (triangle.c:7948-7952).
    if !b.nobound {
        segmentmarkerlist
            .try_reserve_exact(facts.subsegs as usize)
            .map_err(|_| Oom)?;
    }

    // C: traversalinit + subsegloop.ssorient = 0 (triangle.c:7957-7959);
    // C's subsegnumber cursor (7960, 7974) rides the record's number
    // field and advances with the push order.
    let mut subsegnumber: i64 = b.firstnumber as i64;
    let mut t = m.subsegs.traversal();
    while let Some(ss) = m.traverse_subseg(&mut t) {
        let record = segment_record(m, b, ss, subsegnumber);
        // Copy indices of the segment's two endpoints
        // (triangle.c:7964-7966) and the boundary marker
        // (7967-7970).
        segmentlist.push(record.endpoints);
        if let Some(marker) = record.marker {
            segmentmarkerlist.push(marker);
        }
        subsegnumber += 1;
    }
    Ok(())
}

/// C: writeedges (triangle.c:7983-8046). Write the edges to arrays:
/// loop over all triangles and look at the three edges of each; emit
/// an edge when there is no adjacent triangle, or when the current
/// triangle has a smaller pointer than its neighbor, so each edge is
/// considered only once (the C comment, triangle.c:8011-8016).
pub(crate) fn write_edges(
    m: &mut Mesh,
    b: &Behavior,
    edgelist: &mut Vec<[i32; 2]>,
    edgemarkerlist: &mut Vec<i32>,
) -> Result<(), Oom> {
    let facts = edge_facts(m, b);

    // Allocate memory for edges if necessary (triangle.c:7996-7999):
    // one endpoint pair per edge.
    edgelist
        .try_reserve_exact(facts.edges as usize)
        .map_err(|_| Oom)?;
    // Allocate memory for edge markers if necessary
    // (triangle.c:8000-8003).
    if !b.nobound {
        edgemarkerlist
            .try_reserve_exact(facts.edges as usize)
            .map_err(|_| Oom)?;
    }

    // C: traversalinit + the edgenumber walk (triangle.c:8008-8010);
    // the indexed marker store emlist[edgenumber - firstnumber]
    // (triangle.c:8033-8038) is the push in emission order, and C's
    // edgenumber cursor (8010, 8045) rides the record's number field.
    let mut edgenumber: i64 = b.firstnumber as i64;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.traverse_triangle(&mut t) {
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
                let record = edge_record(m, b, triangleloop, trisym, edgenumber);
                edgelist.push(record.endpoints);
                if let Some(marker) = record.marker {
                    edgemarkerlist.push(marker);
                }
                edgenumber += 1;
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
pub(crate) fn write_neighbors(
    m: &mut Mesh,
    b: &Behavior,
    neighborlist: &mut Vec<i32>,
) -> Result<(), Oom> {
    let facts = neighbor_facts(m);

    // Allocate memory for neighbors if necessary
    // (triangle.c:8065-8069).
    neighborlist
        .try_reserve_exact(facts.triangles as usize * 3)
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
    while let Some(tri) = m.traverse_triangle(&mut t) {
        subsegs.push(m.tri_int(tri, numberindex));
        m.set_tri_int(tri, numberindex, elementnumber);
        elementnumber += 1;
    }

    // C: *(int *)(m->dummytri + 6) = -1 (triangle.c:8094).
    m.set_tri_int(DUMMY_TRI, numberindex, -1);

    // C: the second walk (triangle.c:8096-8117): the records carry
    // the neighbours off the overlay; C's elementnumber cursor
    // (8099, 8116) rides the record's number field and advances with
    // the push order.
    let mut elementnumber: i64 = b.firstnumber as i64;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.traverse_triangle(&mut t) {
        let record = neighbor_record(m, tri, numberindex, elementnumber);
        neighborlist.push(record.neighbors[0]);
        neighborlist.push(record.neighbors[1]);
        neighborlist.push(record.neighbors[2]);
        elementnumber += 1;
    }

    // Restore original subseg values (triangle.c:8119-8129; the
    // store at 8126).
    let mut index = 0usize;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.traverse_triangle(&mut t) {
        m.set_tri_int(tri, numberindex, subsegs[index]);
        index += 1;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// The file adapter: the triangle_io.c writers writing io::Write sinks
// (the api.rs Mesh::write_* wrappers, triangle_api.c:437-492).
// ---------------------------------------------------------------------------

/// file_writenodes (triangle_io.c:23-70): number the vertices over
/// their boundary markers (the setvertexmark store inside the walk,
/// triangle_io.c:64-65) and write the .node file. The numbering stays
/// in this form's own traversal exactly like C; numbernumbers gains
/// no consumer.
pub(crate) fn file_write_nodes(
    m: &mut Mesh,
    b: &Behavior,
    nodefile: &mut dyn Write,
) -> std::io::Result<()> {
    let facts = node_facts(m, b);

    // Number of vertices, number of dimensions, number of vertex
    // attributes, and number of boundary markers (zero or one)
    // (triangle_io.c:42-43: "%ld  %d  %d  %d\n").
    write_node_header(
        nodefile,
        facts.outvertices,
        facts.mesh_dim,
        facts.nextras,
        facts.bound,
    )?;

    // C: traversalinit + vertexnumber = b->firstnumber (45-46).
    // The reusable line buffer (cleared per line, capacity reused):
    // every data line is formatted into it whole, then written with
    // a single write_all.
    let mut line = String::new();
    let mut vertexnumber = b.firstnumber;
    let mut t = m.vertices.traversal();
    while let Some(vertexloop) = m.traverse_vertex(&mut t) {
        // C: the jettison gate (triangle_io.c:49).
        if !b.jettison || m.vx_type(vertexloop) != UNDEAD_VERTEX {
            // The record carries the ORIGINAL boundary marker
            // (triangle_io.c:61) - the number store below happens
            // after the write, exactly like C.
            let record = node_record(m, b, vertexloop, vertexnumber);
            line.clear();
            write_node_line(
                &mut line,
                record.number,
                record.x,
                record.y,
                record.attributes,
                record.marker,
            );
            nodefile.write_all(line.as_bytes())?;

            // To save memory, the vertex numbers are written over the
            // boundary markers (triangle_io.c:64-65).
            m.vx_set_mark(vertexloop, vertexnumber);
            vertexnumber += 1;
        }
    }
    Ok(())
}

/// file_writeelements (triangle_io.c:78-125): the .ele writer. Reads
/// the vertex numbers writenodes left on the markers.
pub(crate) fn file_write_elements(
    m: &mut Mesh,
    b: &Behavior,
    elefile: &mut dyn Write,
) -> std::io::Result<()> {
    let facts = element_facts(m, b);

    // Number of triangles, vertices per triangle, attributes per
    // triangle (91-92: "%ld  %d  %d\n").
    write_ele_header(elefile, facts.triangles, facts.corners, facts.eextras)?;

    // The reusable line buffer (cleared per line, capacity reused):
    // every data line is formatted into it whole, then written with
    // a single write_all.
    let mut line = String::new();
    let mut elementnumber: i64 = b.firstnumber as i64;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.traverse_triangle(&mut t) {
        let record = element_record(m, b, tri, elementnumber);
        line.clear();
        write_ele_line(
            &mut line,
            record.number,
            &record.corners[..record.ncorners],
            record.attributes,
        );
        elefile.write_all(line.as_bytes())?;

        elementnumber += 1;
    }
    Ok(())
}

/// file_writepoly (triangle_io.c:133-195): the .poly writer. The hole
/// and region lists come from the caller exactly like C's holelist /
/// regionlist parameters (triangle_api.c:447-449).
pub(crate) fn file_write_poly(
    m: &mut Mesh,
    b: &Behavior,
    polyfile: &mut dyn Write,
    holelist: &[Hole],
    regionlist: &[Region],
) -> std::io::Result<()> {
    let facts = segment_facts(m, b);

    // The zero indicates that the vertices are in a separate .node
    // file, followed by number of dimensions, number of vertex
    // attributes, and number of boundary markers (148-149:
    // "%d  %d  %d  %d\n"), then number of segments and number of
    // boundary markers (151: "%ld  %d\n").
    write_poly_header(
        polyfile,
        0,
        facts.mesh_dim,
        facts.nextras,
        facts.bound,
        facts.subsegs,
        facts.bound,
    )?;

    // The reusable line buffer (cleared per line, capacity reused):
    // every data line is formatted into it whole, then written with
    // a single write_all.
    let mut line = String::new();
    let mut subsegnumber: i64 = b.firstnumber as i64;
    let mut t = m.subsegs.traversal();
    while let Some(ss) = m.traverse_subseg(&mut t) {
        let record = segment_record(m, b, ss, subsegnumber);
        // Segment number, indices of its two endpoints, and possibly
        // a marker (161-167).
        line.clear();
        write_segment_line(&mut line, record.number, record.endpoints, record.marker);
        polyfile.write_all(line.as_bytes())?;

        subsegnumber += 1;
    }

    // C: the hole tail (173-181; behind #ifndef CDT_ONLY, which this
    // fork's build leaves undefined, so the tails always render in
    // the measured binary - unconditional here, the port builds the
    // full shape) and the region tail (182-193, gated on the
    // non-empty list). The interchange crate owns both; the lists
    // convert at the seam (the v0 API's Hole/Region are this crate's
    // own input types).
    let holes: Vec<IoHole> = holelist.iter().map(|h| IoHole { x: h.x, y: h.y }).collect();
    write_poly_holes(polyfile, &holes, b.firstnumber)?;
    let regions: Vec<IoRegion> = regionlist
        .iter()
        .map(|r| IoRegion {
            x: r.x,
            y: r.y,
            attribute: r.attribute,
            max_area: r.max_area,
        })
        .collect();
    write_poly_regions(polyfile, &regions, b.firstnumber)?;
    Ok(())
}

/// file_writeedges (triangle_io.c:203-262): the .edge writer.
pub(crate) fn file_write_edges(
    m: &mut Mesh,
    b: &Behavior,
    edgefile: &mut dyn Write,
) -> std::io::Result<()> {
    let facts = edge_facts(m, b);

    // Number of edges, number of boundary markers (217:
    // "%ld  %d\n").
    write_edge_header(edgefile, facts.edges, facts.bound)?;

    // The reusable line buffer (cleared per line, capacity reused):
    // every data line is formatted into it whole, then written with
    // a single write_all.
    let mut line = String::new();
    let mut edgenumber: i64 = b.firstnumber as i64;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.traverse_triangle(&mut t) {
        for orient in 0..3u8 {
            let triangleloop = Otri { tri, orient };
            let trisym = m.sym(triangleloop);
            // C: (triangleloop.tri < trisym.tri) ||
            // (trisym.tri == m->dummytri) (triangle_io.c:232), the
            // address compare first. The port evaluates the dummy arm
            // first as a deliberate, registered deviation (D29, the
            // D18/D28 precedent): the packed row-id mirror of the
            // address order cannot evaluate the reserved DUMMY_TRI
            // sentinel row, so the dummy arm must short-circuit
            // before it. The boolean is equivalent on every input (a
            // hull edge passes through the dummy arm either way; an
            // interior edge is emitted from the same side through the
            // oracle address order, ADR 0003 section 5).
            if trisym.tri == DUMMY_TRI
                || m.triangles.oracle_address_order(tri, trisym.tri) == Ordering::Less
            {
                let record = edge_record(m, b, triangleloop, trisym, edgenumber);
                line.clear();
                write_edge_line(&mut line, record.number, record.endpoints, record.marker);
                edgefile.write_all(line.as_bytes())?;
                edgenumber += 1;
            }
        }
    }
    Ok(())
}

/// file_writeneighbors (triangle_io.c:264-310): the .neigh writer.
/// The word-6 overlay rides the D2 numbering int exactly like the
/// array writer (write_neighbors, this module); unlike the array
/// writer, C's file writer saves nothing and restores nothing
/// (triangle_io.c has no subsegs scratch - only the array writer at
/// triangle.c:8078, 8119-8129 restores), so the port does not
/// either: the element numbers stay over the overlay, and the
/// dummy's -1 stays.
pub(crate) fn file_write_neighbors(
    m: &mut Mesh,
    b: &Behavior,
    neighborfile: &mut dyn Write,
) -> std::io::Result<()> {
    let facts = neighbor_facts(m);

    // Number of triangles, three neighbors per triangle (276:
    // "%ld  %d\n").
    write_neigh_header(neighborfile, facts.triangles)?;

    // C puns *(int *)(triangleloop.tri + 6) unconditionally
    // (triangle_io.c:283): word 6 is the numbering word of the
    // widened -n row (the ioarrays precedent; the port refuses the
    // past-row pun of the bare row, D2's stance on C's own TODO).
    let numberindex = m.tri_shape.numberindex.expect(
        "the file_writeneighbors numbering overlay exists only with -n \
         (C puns word 6 at triangle_io.c:283, its own TODO)",
    );

    // Number the triangles (282-286): the overlay store inside the
    // first walk, elementnumber from b->firstnumber.
    let mut elementnumber: i64 = b.firstnumber as i64;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.traverse_triangle(&mut t) {
        m.set_tri_int(tri, numberindex, elementnumber as i32);
        elementnumber += 1;
    }
    // C: *(int *)(m->dummytri + 6) = -1 (287), never restored.
    m.set_tri_int(DUMMY_TRI, numberindex, -1);

    // The second walk (289-308): the records carry the neighbours
    // off the overlay. The reusable line buffer (cleared per line,
    // capacity reused): one write_all per data line instead of one
    // write! per line - the R1 cause-2 site.
    let mut line = String::new();
    let mut elementnumber: i64 = b.firstnumber as i64;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.traverse_triangle(&mut t) {
        let record = neighbor_record(m, tri, numberindex, elementnumber);
        line.clear();
        write_neigh_line(&mut line, record.number, record.neighbors);
        neighborfile.write_all(line.as_bytes())?;
        elementnumber += 1;
    }
    Ok(())
}

/// file_write_eps (triangle_io.c:312-351): the .eps writer. Composes
/// the eps_writer.c unit exactly where C calls it (331-347), with the
/// fixed page and clip rectangles (323-330) and the mesh bounds
/// (332-336).
pub(crate) fn file_write_eps(
    m: &mut Mesh,
    _b: &Behavior,
    file: &mut dyn Write,
) -> std::io::Result<()> {
    use crate::io::eps::Rect;
    // EPS page metrics (323-326).
    let mut ps = Rect {
        xmin: 36.0,
        ymin: 126.0,
        xmax: 576.0,
        ymax: 666.0,
    };
    // EPS page clip metrics (328-331).
    let mut clip = Rect {
        xmin: 18.0,
        ymin: 108.0,
        xmax: 594.0,
        ymax: 684.0,
    };
    // Mesh metrics (333-337).
    let mut ms = Rect {
        xmin: m.xmin,
        ymin: m.ymin,
        xmax: m.xmax,
        ymax: m.ymax,
    };

    crate::io::eps::eps_update_metrics(&mut ps, &mut clip, &mut ms);

    crate::io::eps::eps_write_header(file, "mesh.eps", &ps)?;
    crate::io::eps::eps_draw_clip(file, &ps, &clip)?;
    crate::io::eps::eps_draw_edges(file, m, &ps, &ms)?;
    crate::io::eps::eps_draw_segments(file, m, &ps, &ms)?;
    crate::io::eps::eps_draw_points(file, m, &ps, &ms)?;
    crate::io::eps::eps_write_trailer(file)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::fixtures::{behavior_with, core_mesh, square_two_triangles, three_points};
    use crate::options::Options;

    /// The plain assembled Behavior (the io/arrays.rs test precedent).
    fn behavior() -> Behavior {
        Options::new().validate().expect("validate")
    }

    #[test]
    fn write_nodes_marshals_and_numbers_over_the_marks() {
        // The writenodes walk (triangle.c:7806-7827) over a
        // transfernodes load: the points come back in traversal
        // order with their attributes and markers, and the numbering
        // trick writes the numbers over the marks exactly where C
        // does (triangle.c:7823).
        let mut b = behavior();
        let mut m = core_mesh(&b, 3, 1);
        let (points, attrs, markers) = three_points();
        crate::io::arrays::transfer_nodes(&mut m, &mut b, &points, &attrs, Some(&markers), 3, 1)
            .expect("load");
        let mut plist: Vec<Point2> = Vec::new();
        let mut palist: Vec<f64> = Vec::new();
        let mut pmlist: Vec<i32> = Vec::new();
        write_nodes(&mut m, &b, &mut plist, &mut palist, &mut pmlist).expect("writenodes");
        assert_eq!(plist, points.to_vec());
        assert_eq!(palist, attrs.to_vec());
        // the ORIGINAL markers leave in the list...
        assert_eq!(pmlist, markers.to_vec());
        // ...and the numbers sit over the marks in the rows (the
        // banner's memory trick, triangle.c:7763-7764).
        let mut number = 1;
        let mut t = m.vertices.traversal();
        while let Some(v) = m.traverse_vertex(&mut t) {
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
        crate::io::arrays::transfer_nodes(&mut m, &mut b, &points, &[], None, 3, 0).expect("load");
        write_nodes(
            &mut m,
            &b,
            &mut Vec::new(),
            &mut Vec::new(),
            &mut Vec::new(),
        )
        .expect("writenodes");
        let mut t = m.vertices.traversal();
        assert_eq!(m.vx_mark(m.traverse_vertex(&mut t).expect("vx")), 0);
    }

    #[test]
    fn write_nodes_without_markers_still_numbers() {
        // The !nobound gate (triangle.c:7818-7821): no marker list is
        // written, and the numbering store (7823) stays outside the
        // gate.
        let mut b = behavior_with(|o| {
            o.no_bounds();
        });
        let mut m = core_mesh(&b, 3, 1);
        let (points, attrs, _markers) = three_points();
        crate::io::arrays::transfer_nodes(&mut m, &mut b, &points, &attrs, None, 3, 1)
            .expect("load");
        let mut plist: Vec<Point2> = Vec::new();
        let mut palist: Vec<f64> = Vec::new();
        let mut pmlist: Vec<i32> = Vec::new();
        write_nodes(&mut m, &b, &mut plist, &mut palist, &mut pmlist).expect("writenodes");
        assert_eq!(plist, points.to_vec());
        assert_eq!(palist, attrs.to_vec());
        assert!(pmlist.is_empty());
        let mut t = m.vertices.traversal();
        assert_eq!(m.vx_mark(m.traverse_vertex(&mut t).expect("vx")), 1);
    }

    #[test]
    fn write_nodes_jettison_skips_the_undead() {
        // The jettison gate (triangle.c:7810): an UNDEADVERTEX row is
        // skipped in the output and keeps its old marker (no number
        // lands on it), and the numbering of the later vertices runs
        // on without it (7823-7824 inside the gate).
        let mut b = behavior_with(|o| {
            o.jettison();
        });
        let mut m = core_mesh(&b, 3, 0);
        let (points, _attrs, _markers) = three_points();
        crate::io::arrays::transfer_nodes(&mut m, &mut b, &points, &[], None, 3, 0).expect("load");
        let mut t = m.vertices.traversal();
        let v1 = m.traverse_vertex(&mut t).expect("vx");
        let v2 = m.traverse_vertex(&mut t).expect("vx");
        let v3 = m.traverse_vertex(&mut t).expect("vx");
        // the graveyard row the way divconqdelaunay leaves it (type
        // UNDEADVERTEX; m->undeads is the C counter shape)
        m.vx_set_type(v2, UNDEAD_VERTEX);
        m.undeads = 1;
        let mut plist: Vec<Point2> = Vec::new();
        let mut palist: Vec<f64> = Vec::new();
        let mut pmlist: Vec<i32> = Vec::new();
        write_nodes(&mut m, &b, &mut plist, &mut palist, &mut pmlist).expect("writenodes");
        assert_eq!(plist, vec![points[0], points[2]]);
        assert_eq!(pmlist, vec![0, 0]);
        // v1 is numbered 1, the undead keeps the default 0 mark, v3 is
        // numbered 2 (the numbers advance past the living only)
        assert_eq!(m.vx_mark(v1), 1);
        assert_eq!(m.vx_mark(v2), 0);
        assert_eq!(m.vx_mark(v3), 2);
    }

    #[test]
    fn number_nodes_numbers_all_and_advances_past_the_living() {
        // C numbernodes (triangle.c:7845-7854): every vertex gets a
        // marker equal to its number, the undead included, while the
        // number advances only past non-undead rows under jettison
        // (7850-7852).
        let mut b = behavior_with(|o| {
            o.jettison();
        });
        let mut m = core_mesh(&b, 3, 0);
        let (points, _attrs, _markers) = three_points();
        crate::io::arrays::transfer_nodes(&mut m, &mut b, &points, &[], None, 3, 0).expect("load");
        let mut t = m.vertices.traversal();
        let v1 = m.traverse_vertex(&mut t).expect("vx");
        let v2 = m.traverse_vertex(&mut t).expect("vx");
        let v3 = m.traverse_vertex(&mut t).expect("vx");
        m.vx_set_type(v2, UNDEAD_VERTEX);
        m.undeads = 1;
        number_nodes(&mut m, &b);
        assert_eq!(m.vx_mark(v1), 1);
        // the undead row is numbered with the current number (the
        // number v1's advance left) and does not advance it, so v3
        // re-receives the same number (the C collision is unobservable
        // in output: writenodes never writes an undead row)
        assert_eq!(m.vx_mark(v2), 2);
        assert_eq!(m.vx_mark(v3), 2);
    }

    #[test]
    fn write_elements_records_the_corner_numbers() {
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
        number_nodes(&mut m, &b);
        let mut tlist: Vec<i32> = Vec::new();
        let mut talist: Vec<f64> = Vec::new();
        write_elements(&mut m, &b, &mut tlist, &mut talist).expect("writeelements");
        assert_eq!(tlist, vec![1, 2, 3, 1, 3, 4]);
        assert_eq!(talist, vec![7.0, 8.0, 9.0, 10.0]);
    }

    #[test]
    fn write_elements_above_order_one_adds_the_mid_edge_nodes() {
        // C writeelements' order-2 branch (triangle.c:7906-7915):
        // after highorder records the mid-edge nodes, the six corners
        // are p1, p2, p3, mid1 (highorderindex + 1), mid2
        // (highorderindex + 2), mid3 (highorderindex) - the node ids
        // highorder stored per edge orientation.
        let b = behavior_with(|o| {
            o.element_order(2);
        });
        let (mut m, _t1, _t2, _v) = square_two_triangles(&b, 0);
        crate::algorithm::refine::high_order(&mut m, &b).expect("highorder");
        number_nodes(&mut m, &b);
        let mut tlist: Vec<i32> = Vec::new();
        let mut talist: Vec<f64> = Vec::new();
        write_elements(&mut m, &b, &mut tlist, &mut talist).expect("writeelements");
        // the five mid-edge nodes are pool rows 5..=9 in creation
        // order (t1's edges 0, 1, 2 then t2's edges 1, 2; the
        // interior edge's node is created once, from t1), so the
        // numbers are 5..=9 and each element carries the mid nodes
        // of its edges 1, 2, 0 in C's mid1/mid2/mid3 order
        assert_eq!(tlist, vec![1, 2, 3, 6, 7, 5, 1, 3, 4, 8, 9, 7]);
    }

    #[test]
    fn write_poly_records_endpoint_numbers_and_markers() {
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
        let s1 = m.make_subseg().expect("subseg");
        m.set_subseg_org(s1, a);
        m.set_subseg_dest(s1, e);
        m.set_sub_mark(s1.ss, 7);
        let s2 = m.make_subseg().expect("subseg");
        m.set_subseg_org(s2, e);
        m.set_subseg_dest(s2, c);
        m.set_sub_mark(s2.ss, 8);
        number_nodes(&mut m, &b);
        let mut slist: Vec<[i32; 2]> = Vec::new();
        let mut smlist: Vec<i32> = Vec::new();
        write_poly(&mut m, &b, &mut slist, &mut smlist).expect("writepoly");
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
        let s1 = m.make_subseg().expect("subseg");
        m.set_subseg_org(s1, a);
        m.set_subseg_dest(s1, e);
        number_nodes(&mut m, &b);
        let mut slist: Vec<[i32; 2]> = Vec::new();
        let mut smlist: Vec<i32> = Vec::new();
        write_poly(&mut m, &b, &mut slist, &mut smlist).expect("writepoly");
        assert_eq!(slist, vec![[1, 2]]);
        assert!(smlist.is_empty());
    }

    #[test]
    fn write_edges_visits_every_edge_once() {
        // C writeedges (triangle.c:8017-8045) over the two-triangle
        // square without segments: the five edges are emitted from
        // t1's three orients then t2's hull orients (the interior
        // edge from the lower-id side, the address order through the
        // D28 dummy-first conjunct), and the markers are the hull
        // flag (triangle.c:8038).
        let b = behavior();
        let (mut m, _t1, _t2, _v) = square_two_triangles(&b, 0);
        m.edges = 5; // the m->edges writer of the C entry points
        number_nodes(&mut m, &b);
        let mut elist: Vec<[i32; 2]> = Vec::new();
        let mut emlist: Vec<i32> = Vec::new();
        write_edges(&mut m, &b, &mut elist, &mut emlist).expect("writeedges");
        assert_eq!(elist, vec![[1, 2], [2, 3], [3, 1], [3, 4], [4, 1]]);
        assert_eq!(emlist, vec![1, 1, 0, 1, 1]);
    }

    #[test]
    fn write_edges_marks_the_segment_edges_under_pslg() {
        // The usesegments branch (triangle.c:8030-8036): a bonded
        // subsegment lends its boundary marker to the edge, an
        // unbonded edge writes 0.
        let b = behavior_with(|o| {
            o.pslg();
        });
        let (mut m, t1, _t2, _v) = square_two_triangles(&b, 0);
        let s = m.make_subseg().expect("subseg");
        m.set_sub_mark(s.ss, 7);
        m.bond_tri_subseg(Otri { tri: t1, orient: 0 }, s);
        m.edges = 5;
        number_nodes(&mut m, &b);
        let mut elist: Vec<[i32; 2]> = Vec::new();
        let mut emlist: Vec<i32> = Vec::new();
        write_edges(&mut m, &b, &mut elist, &mut emlist).expect("writeedges");
        assert_eq!(elist, vec![[1, 2], [2, 3], [3, 1], [3, 4], [4, 1]]);
        assert_eq!(emlist, vec![7, 0, 0, 0, 0]);
    }

    #[test]
    fn write_neighbors_numbers_reads_and_restores() {
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
        write_neighbors(&mut m, &b, &mut nlist).expect("writeneighbors");
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
    fn write_neighbors_restores_the_subseg_slots_under_pslg() {
        // The -p twin: word 6 is a live subseg slot, and the C pun
        // saves and restores its low half around the numbering
        // (triangle.c:8078-8093, 8119-8129) - the bonded edge pivots
        // to its subsegment again after the writer ran.
        let b = behavior_with(|o| {
            o.pslg();
            o.neighbors();
        });
        let (mut m, t1, _t2, _v) = square_two_triangles(&b, 0);
        let s = m.make_subseg().expect("subseg");
        m.bond_tri_subseg(Otri { tri: t1, orient: 0 }, s);
        let mut nlist: Vec<i32> = Vec::new();
        write_neighbors(&mut m, &b, &mut nlist).expect("writeneighbors");
        assert_eq!(nlist, vec![-1, 2, -1, -1, -1, 1]);
        assert_eq!(m.tri_subseg(Otri { tri: t1, orient: 0 }), s);
    }

    #[test]
    #[should_panic(expected = "the writeneighbors numbering overlay exists only with -n")]
    fn write_neighbors_without_the_option_refuses_the_out_of_row_pun() {
        // The widened row exists only under -n (C: triangle.c:
        // 491-495); C puns past the row here (its own TODO,
        // triangle.c:8073-8076), the port refuses loudly at the
        // chokepoint (D2's stance on the C UB).
        let b = behavior_with(|o| {
            o.pslg();
        });
        let (mut m, _t1, _t2, _v) = square_two_triangles(&b, 0);
        let mut nlist: Vec<i32> = Vec::new();
        let _ = write_neighbors(&mut m, &b, &mut nlist);
    }

    #[test]
    fn write_neighbors_on_an_empty_mesh_still_numbers_outer_space() {
        // With no triangles the loops do not run, but C's dummy
        // store (triangle.c:8094) is unconditional.
        let b = behavior_with(|o| {
            o.neighbors();
        });
        let mut m = core_mesh(&b, 3, 0);
        let mut nlist: Vec<i32> = Vec::new();
        write_neighbors(&mut m, &b, &mut nlist).expect("writeneighbors");
        assert!(nlist.is_empty());
        let numberindex = m.tri_shape.numberindex.expect("overlay");
        assert_eq!(m.tri_int(DUMMY_TRI, numberindex), -1);
    }

    #[test]
    fn file_write_nodes_numbers_and_writes_bytes() {
        let b = behavior_with(|_| {});
        let (mut m, _t1, _t2, _v) = square_two_triangles(&b, 0);
        let mut out: Vec<u8> = Vec::new();
        file_write_nodes(&mut m, &b, &mut out).expect("writenodes");
        assert_eq!(
            String::from_utf8(out).expect("ascii"),
            "4  2  0  1\n\
             \x20\x20\x201    0  0    0\n\
             \x20\x20\x202    1  0    0\n\
             \x20\x20\x203    1  1    0\n\
             \x20\x20\x204    0  1    0\n"
        );
        // The numbering sits over the marks (triangle_io.c:64-65).
        let mut t = m.vertices.traversal();
        let mut number = 1;
        while let Some(v) = m.traverse_vertex(&mut t) {
            assert_eq!(m.vx_mark(v), number);
            number += 1;
        }
    }

    #[test]
    fn file_write_nodes_zero_based_and_no_bounds() {
        let b = behavior_with(|o| {
            o.zero_based();
            o.no_bounds();
        });
        let (mut m, _t1, _t2, _v) = square_two_triangles(&b, 0);
        let mut out: Vec<u8> = Vec::new();
        file_write_nodes(&mut m, &b, &mut out).expect("writenodes");
        assert_eq!(
            String::from_utf8(out).expect("ascii"),
            "4  2  0  0\n\
             \x20\x20\x200    0  0\n\
             \x20\x20\x201    1  0\n\
             \x20\x20\x202    1  1\n\
             \x20\x20\x203    0  1\n"
        );
    }

    #[test]
    fn file_write_elements_reads_the_write_nodes_numbers() {
        let b = behavior_with(|_| {});
        let (mut m, _t1, _t2, _v) = square_two_triangles(&b, 0);
        let mut sink: Vec<u8> = Vec::new();
        file_write_nodes(&mut m, &b, &mut sink).expect("writenodes");
        let mut out: Vec<u8> = Vec::new();
        file_write_elements(&mut m, &b, &mut out).expect("writeelements");
        assert_eq!(
            String::from_utf8(out).expect("ascii"),
            "2  3  0\n\
             \x20\x20\x201    \x20\x20 1  \x20\x20 2  \x20\x20 3\n\
             \x20\x20\x202    \x20\x20 1  \x20\x20 3  \x20\x20 4\n"
        );
    }

    #[test]
    fn file_write_poly_writes_segments_holes_and_regions() {
        let b = behavior_with(|o| {
            o.pslg();
        });
        let (mut m, t1, _t2, v) = square_two_triangles(&b, 0);
        // One subseg bonded to t1's orient-0 edge (a - e). The
        // subseg carries its own endpoint slots (sorg/sdest,
        // the ioarrays writer's fixture precedent); writenodes
        // has numbered the vertices 1-4 before the poly writer
        // reads their marks.
        let s = m.make_subseg().expect("subseg");
        m.set_subseg_org(s, v[0]);
        m.set_subseg_dest(s, v[1]);
        m.bond_tri_subseg(Otri { tri: t1, orient: 0 }, s);
        let mut sink: Vec<u8> = Vec::new();
        file_write_nodes(&mut m, &b, &mut sink).expect("writenodes");
        let mut out: Vec<u8> = Vec::new();
        file_write_poly(
            &mut m,
            &b,
            &mut out,
            &[Hole { x: 0.5, y: 0.5 }],
            &[Region {
                x: 0.25,
                y: 0.75,
                attribute: 0.5,
                max_area: 0.001,
            }],
        )
        .expect("writepoly");
        assert_eq!(
            String::from_utf8(out).expect("ascii"),
            "0  2  0  1\n\
             1  1\n\
             \x20\x20\x201    \x20\x20 1  \x20\x20 2    \x20\x20 0\n\
             1\n\
             \x20\x20\x201   0.5  0.5\n\
             1\n\
             \x20\x20\x201   0.25  0.75  0.5  0.001\n"
        );
    }

    #[test]
    fn file_write_edges_walks_the_hull_and_interior_once() {
        let b = behavior_with(|_| {});
        let (mut m, _t1, _t2, _v) = square_two_triangles(&b, 0);
        m.edges = 5; // (3 * items + hullsize) / 2 for the square.
        let mut sink: Vec<u8> = Vec::new();
        file_write_nodes(&mut m, &b, &mut sink).expect("writenodes");
        let mut out: Vec<u8> = Vec::new();
        file_write_edges(&mut m, &b, &mut out).expect("writeedges");
        assert_eq!(
            String::from_utf8(out).expect("ascii"),
            "5  1\n\
             \x20\x20\x201   1  2  1\n\
             \x20\x20\x202   2  3  1\n\
             \x20\x20\x203   3  1  0\n\
             \x20\x20\x204   3  4  1\n\
             \x20\x20\x205   4  1  1\n"
        );
    }

    #[test]
    fn file_write_neighbors_overlays_without_restore() {
        let b = behavior_with(|o| {
            o.neighbors();
        });
        let (mut m, t1, t2, _v) = square_two_triangles(&b, 0);
        let mut sink: Vec<u8> = Vec::new();
        file_write_nodes(&mut m, &b, &mut sink).expect("writenodes");
        let mut out: Vec<u8> = Vec::new();
        file_write_neighbors(&mut m, &b, &mut out).expect("writeneighbors");
        assert_eq!(
            String::from_utf8(out).expect("ascii"),
            "2  3\n\
             \x20\x20\x201    -1  2  -1\n\
             \x20\x20\x202    -1  -1  1\n"
        );
        // No restore, unlike the array writer (triangle_io.c has no
        // subsegs scratch): the element numbers stay over the
        // overlay, and the dummy's -1 stays.
        let numberindex = m.tri_shape.numberindex.expect("overlay");
        assert_eq!(m.tri_int(t1, numberindex), 1);
        assert_eq!(m.tri_int(t2, numberindex), 2);
        assert_eq!(m.tri_int(DUMMY_TRI, numberindex), -1);
    }

    // -- the record seams, pinned by construction (never via the
    // oracle binary) --

    #[test]
    fn node_record_carries_number_values_and_the_original_marker() {
        // The seam both adapters consume: for a fixed transfernodes
        // load the record echoes the caller's number cursor (the -z
        // arm feeds it 0 exactly like the adapters do), carries the
        // coordinates and the attribute values in row order, and the
        // ORIGINAL boundary marker - read before the numbering store
        // both forms make after consuming the record (triangle.c:
        // 7818-7823 / triangle_io.c:61-65).
        let mut b = behavior();
        let mut m = core_mesh(&b, 3, 1);
        let (points, attrs, markers) = three_points();
        crate::io::arrays::transfer_nodes(&mut m, &mut b, &points, &attrs, Some(&markers), 3, 1)
            .expect("load");
        let mut t = m.vertices.traversal();
        let v1 = m.traverse_vertex(&mut t).expect("vx");
        let record = node_record(&m, &b, v1, 1);
        assert_eq!(record.number, 1);
        assert_eq!(record.x, points[0].x);
        assert_eq!(record.y, points[0].y);
        assert_eq!(record.attributes.collect::<Vec<f64>>(), vec![attrs[0]]);
        assert_eq!(record.marker, Some(markers[0]));
        // -z: the caller's cursor starts at 0 and the record echoes it.
        let record = node_record(&m, &b, v1, 0);
        assert_eq!(record.number, 0);
        // -B: the marker arm closes, the values stay.
        let b = behavior_with(|o| {
            o.no_bounds();
        });
        let record = node_record(&m, &b, v1, 1);
        assert_eq!(record.marker, None);
        assert_eq!(record.attributes.collect::<Vec<f64>>(), vec![attrs[0]]);
    }

    #[test]
    fn element_record_carries_the_corner_numbers_and_attributes() {
        // Over the two-triangle square, numbered by numbernodes: the
        // record carries the corners in org/dest/apex order (plus the
        // mid-edge tail above order 1, triangle.c:7906-7915) and the
        // element attributes in row order (7918-7920).
        let b = behavior();
        let (mut m, t1, t2, _v) = square_two_triangles(&b, 2);
        m.set_tri_attribute(t1, 0, 7.0);
        m.set_tri_attribute(t1, 1, 8.0);
        m.set_tri_attribute(t2, 0, 9.0);
        m.set_tri_attribute(t2, 1, 10.0);
        number_nodes(&mut m, &b);
        let r1 = element_record(&m, &b, t1, 1);
        assert_eq!(r1.number, 1);
        assert_eq!(r1.ncorners, 3);
        assert_eq!(&r1.corners[..r1.ncorners], &[1, 2, 3]);
        assert_eq!(r1.attributes.collect::<Vec<f64>>(), vec![7.0, 8.0]);
        let r2 = element_record(&m, &b, t2, 2);
        assert_eq!(&r2.corners[..r2.ncorners], &[1, 3, 4]);
        assert_eq!(r2.attributes.collect::<Vec<f64>>(), vec![9.0, 10.0]);
    }

    #[test]
    fn segment_record_carries_endpoints_and_the_marker() {
        // C writepoly's reads (triangle.c:7965-7970): the endpoint
        // numbers at ssorient 0 and the subsegment's boundary marker;
        // -B closes the marker arm and the -z cursor echoes.
        let b = behavior_with(|o| {
            o.pslg();
        });
        let (mut m, _t1, _t2, v) = square_two_triangles(&b, 0);
        let [a, e, _c, _d] = v;
        let s1 = m.make_subseg().expect("subseg");
        m.set_subseg_org(s1, a);
        m.set_subseg_dest(s1, e);
        m.set_sub_mark(s1.ss, 7);
        number_nodes(&mut m, &b);
        let record = segment_record(&m, &b, s1.ss, 1);
        assert_eq!(record.number, 1);
        assert_eq!(record.endpoints, [1, 2]);
        assert_eq!(record.marker, Some(7));
        let b = behavior_with(|o| {
            o.pslg();
            o.no_bounds();
            o.zero_based();
        });
        let (mut m, _t1, _t2, v) = square_two_triangles(&b, 0);
        let [a, e, _c, _d] = v;
        let s1 = m.make_subseg().expect("subseg");
        m.set_subseg_org(s1, a);
        m.set_subseg_dest(s1, e);
        number_nodes(&mut m, &b);
        let record = segment_record(&m, &b, s1.ss, 0);
        assert_eq!(record.number, 0);
        assert_eq!(record.endpoints, [0, 1]);
        assert_eq!(record.marker, None);
    }

    #[test]
    fn edge_record_carries_endpoints_and_the_marker() {
        // The two marker arms of the seam (triangle.c:8026-8038): the
        // hull flag without segments, the bonded subsegment's boundary
        // marker under -p, and 0 on an interior edge.
        let b = behavior();
        let (mut m, t1, _t2, _v) = square_two_triangles(&b, 0);
        number_nodes(&mut m, &b);
        let hull = Otri { tri: t1, orient: 0 };
        let record = edge_record(&m, &b, hull, m.sym(hull), 1);
        assert_eq!(record.endpoints, [1, 2]);
        assert_eq!(record.marker, Some(1));
        let interior = Otri { tri: t1, orient: 2 };
        let record = edge_record(&m, &b, interior, m.sym(interior), 2);
        assert_eq!(record.endpoints, [3, 1]);
        assert_eq!(record.marker, Some(0));
        let b = behavior_with(|o| {
            o.pslg();
        });
        let (mut m, t1, _t2, _v) = square_two_triangles(&b, 0);
        let s = m.make_subseg().expect("subseg");
        m.set_sub_mark(s.ss, 7);
        m.bond_tri_subseg(Otri { tri: t1, orient: 0 }, s);
        number_nodes(&mut m, &b);
        let hull = Otri { tri: t1, orient: 0 };
        let record = edge_record(&m, &b, hull, m.sym(hull), 1);
        assert_eq!(record.endpoints, [1, 2]);
        assert_eq!(record.marker, Some(7));
    }

    #[test]
    fn neighbor_record_reads_the_overlay_in_orients_1_2_0() {
        // After the numbering walk both forms run (the anchors cited
        // in the module doc), the record reads neighbor1 from orient
        // 1, neighbor2 from orient 2, neighbor3 from orient 0
        // (triangle.c:8101-8113 / triangle_io.c:295-301), outer space
        // -1.
        let b = behavior_with(|o| {
            o.neighbors();
        });
        let (mut m, t1, t2, _v) = square_two_triangles(&b, 0);
        let numberindex = m.tri_shape.numberindex.expect("overlay");
        m.set_tri_int(t1, numberindex, 1);
        m.set_tri_int(t2, numberindex, 2);
        m.set_tri_int(DUMMY_TRI, numberindex, -1);
        let r1 = neighbor_record(&m, t1, numberindex, 1);
        assert_eq!(r1.number, 1);
        assert_eq!(r1.neighbors, [-1, 2, -1]);
        let r2 = neighbor_record(&m, t2, numberindex, 2);
        assert_eq!(r2.neighbors, [-1, -1, 1]);
    }
}
