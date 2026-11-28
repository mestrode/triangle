//! File I/O, the mesh-bound half (triangle_io.c's writers): the six
//! file writers and the file entry points of triangle_api.c (the
//! api.rs wrappers). The interchange formats themselves - the
//! readline/findfield/strtol/strtod parse seam, the readers, and the
//! header/line/trailer emission machinery - live in the triangle-io
//! crate since the interchange extraction (ADR 0006, issue #70); the
//! writers here keep the Mesh traversal orchestration (numbering,
//! jettison gates, the D29 edge-walk order, the -n overlay) and call
//! into triangle-io's emitters for the bytes. Assignment: the
//! triangle_io.c and triangle-api file rows of
//! docs/function-mapping.md, plus the extraction's migration table.
//!
//! Boundary decisions (the deviation register and the PR bodies
//! carry the details):
//! - The C FILE* NULL checks (file == NULL -> TRI_FILE_OPEN,
//!   triangle_io.c:30, 86, 141, 212, 271, 318; triangle_api.c fopen
//!   NULL -> TRI_FILE_OPEN at 498, 514, 530, 546) dissolve: the
//!   port's api.rs wrappers own the open, and File::open's error
//!   carries straight into Error::Io (the TRI_FILE_OPEN equivalence
//!   class; D31).
//! - C's unchecked fprintf return values in the writers surface as
//!   io::Result in the port (register row: the writers' seam). Every
//!   reader failure raises in triangle-io since rung 2: TRI_FILE_READ
//!   (a fgets NULL) is Error::Read with the same io::Error payload
//!   the stage-1 funnel arm carried (D31's equivalence class), and
//!   the raw -1 TODO returns are Error::Input with the funnel arm's
//!   exact TRI_FAILURE payload text; api.rs maps both back onto this
//!   crate's unchanged public Error at the seam.

use std::io::Write;

use triangle_io::types::{Hole as IoHole, Region as IoRegion};
use triangle_io::write::{
    write_edge_header, write_edge_line, write_ele_header, write_ele_line, write_neigh_header,
    write_neigh_line, write_node_header, write_node_line, write_poly_header, write_poly_holes,
    write_poly_regions, write_segment_line,
};

use crate::input::{Hole, Region};
use crate::mesh::handles::{Osub, Otri, DUMMY_SUB, DUMMY_TRI};
use crate::mesh::slots::UNDEAD_VERTEX;
use crate::mesh::Mesh;
use crate::options::Behavior;

// ---------------------------------------------------------------------------
// File writing routines (triangle_io.c:10-352).
// ---------------------------------------------------------------------------

/// file_writenodes (triangle_io.c:23-70): number the vertices over
/// their boundary markers (the setvertexmark store inside the walk,
/// triangle_io.c:64-65) and write the .node file. The writers inline
/// their own numbering exactly like C; numbernumbers gains no
/// consumer.
pub(crate) fn file_write_nodes(
    m: &mut Mesh,
    b: &Behavior,
    nodefile: &mut dyn Write,
) -> std::io::Result<()> {
    // C: outvertices (triangle_io.c:34-38): jettison shrinks the
    // output count by the undead.
    let outvertices: i64 = if b.jettison {
        m.vertices.live_count() as i64 - m.undeads as i64
    } else {
        m.vertices.live_count() as i64
    };

    // Number of vertices, number of dimensions, number of vertex
    // attributes, and number of boundary markers (zero or one)
    // (triangle_io.c:42-43: "%ld  %d  %d  %d\n").
    write_node_header(
        nodefile,
        outvertices,
        m.mesh_dim,
        m.nextras,
        1 - i32::from(b.nobound),
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
            // Vertex number, x and y coordinates, the attributes
            // (51-61), and the ORIGINAL boundary marker (61) - the
            // number store below happens after the write, exactly
            // like C.
            line.clear();
            write_node_line(
                &mut line,
                vertexnumber,
                m.vx_x(vertexloop),
                m.vx_y(vertexloop),
                (0..m.nextras as usize).map(|i| m.vx_real(vertexloop, 2 + i)),
                if b.nobound {
                    None
                } else {
                    Some(m.vx_mark(vertexloop))
                },
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
    // Number of triangles, vertices per triangle, attributes per
    // triangle (91-92: "%ld  %d  %d\n").
    write_ele_header(
        elefile,
        m.triangles.live_count() as i64,
        (b.order + 1) * (b.order + 2) / 2,
        m.eextras,
    )?;

    // The reusable line buffer (cleared per line, capacity reused):
    // every data line is formatted into it whole, then written with
    // a single write_all.
    let mut line = String::new();
    let mut elementnumber: i64 = b.firstnumber as i64;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.traverse_triangle(&mut t) {
        let triangleloop = Otri { tri, orient: 0 };
        let p1 = m.org(triangleloop);
        let p2 = m.dest(triangleloop);
        let p3 = m.apex(triangleloop);
        // Triangle number, indices for three vertices (104-105:
        // "%4ld    %4d  %4d  %4d"); if order 2, the mid-edge nodes
        // follow (107-113: highorderindex + 1, + 2 and
        // highorderindex, exactly where highorder stored them).
        let mut corners = [0i32; 6];
        corners[0] = m.vx_mark(p1);
        corners[1] = m.vx_mark(p2);
        corners[2] = m.vx_mark(p3);
        let ncorners = if b.order != 1 {
            let mid1 = m.tri_midnode(tri, 1) as u32;
            let mid2 = m.tri_midnode(tri, 2) as u32;
            let mid3 = m.tri_midnode(tri, 0) as u32;
            // Triangle number, indices for six vertices (111-113).
            corners[3] = m.vx_mark(mid1);
            corners[4] = m.vx_mark(mid2);
            corners[5] = m.vx_mark(mid3);
            6
        } else {
            3
        };
        // Then the attributes (116-117: "  %.17g").
        line.clear();
        write_ele_line(
            &mut line,
            elementnumber,
            &corners[..ncorners],
            (0..m.eextras as usize).map(|i| m.tri_attribute(tri, i)),
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
    // The zero indicates that the vertices are in a separate .node
    // file, followed by number of dimensions, number of vertex
    // attributes, and number of boundary markers (148-149:
    // "%d  %d  %d  %d\n"), then number of segments and number of
    // boundary markers (151: "%ld  %d\n").
    write_poly_header(
        polyfile,
        0,
        m.mesh_dim,
        m.nextras,
        1 - i32::from(b.nobound),
        m.subsegs.live_count() as i64,
        1 - i32::from(b.nobound),
    )?;

    // The reusable line buffer (cleared per line, capacity reused):
    // every data line is formatted into it whole, then written with
    // a single write_all.
    let mut line = String::new();
    let mut subsegnumber: i64 = b.firstnumber as i64;
    let mut t = m.subsegs.traversal();
    while let Some(ss) = m.traverse_subseg(&mut t) {
        let subsegloop = Osub { ss, orient: 0 };
        let endpoint1 = m.subseg_org(subsegloop);
        let endpoint2 = m.subseg_dest(subsegloop);
        // Segment number, indices of its two endpoints, and possibly
        // a marker (161-167).
        line.clear();
        write_segment_line(
            &mut line,
            subsegnumber,
            [m.vx_mark(endpoint1), m.vx_mark(endpoint2)],
            if b.nobound {
                None
            } else {
                Some(m.sub_mark(subsegloop.ss))
            },
        );
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
    // Number of edges, number of boundary markers (217:
    // "%ld  %d\n").
    write_edge_header(edgefile, m.edges, 1 - i32::from(b.nobound))?;

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
            // interior edge is emitted from the same side through
            // the oracle address order, ADR 0003 section 5).
            if trisym.tri == DUMMY_TRI
                || m.triangles.oracle_address_order(tri, trisym.tri) == std::cmp::Ordering::Less
            {
                let p1 = m.org(triangleloop);
                let p2 = m.dest(triangleloop);
                // Edge number, indices of two endpoints (237:
                // "%4ld   %d  %d\n").
                line.clear();
                write_edge_line(
                    &mut line,
                    edgenumber,
                    [m.vx_mark(p1), m.vx_mark(p2)],
                    if b.nobound {
                        None
                    } else {
                        // Edge number, indices of two endpoints, and a
                        // boundary marker. If there's no subsegment, the
                        // boundary marker is zero (240-254).
                        Some(if b.usesegments {
                            let checkmark = m.tri_subseg(triangleloop);
                            if checkmark.ss == DUMMY_SUB {
                                0
                            } else {
                                m.sub_mark(checkmark.ss)
                            }
                        } else {
                            // C: trisym.tri == m->dummytri (253) - the
                            // hull flag itself.
                            i32::from(trisym.tri == DUMMY_TRI)
                        })
                    },
                );
                edgefile.write_all(line.as_bytes())?;
                edgenumber += 1;
            }
        }
    }
    Ok(())
}

/// file_writeneighbors (triangle_io.c:264-310): the .neigh writer.
/// The word-6 overlay rides the D2 numbering int exactly like the
/// array writer (io/arrays.rs); unlike the array writer, C's file
/// writer saves nothing and restores nothing (triangle_io.c has no
/// subsegs scratch - only the array writer at triangle.c:8078,
/// 8119-8129 restores), so the port does not either: the element
/// numbers stay over the overlay, and the dummy's -1 stays.
pub(crate) fn file_write_neighbors(
    m: &mut Mesh,
    b: &Behavior,
    neighborfile: &mut dyn Write,
) -> std::io::Result<()> {
    // Number of triangles, three neighbors per triangle (276:
    // "%ld  %d\n").
    write_neigh_header(neighborfile, m.triangles.live_count() as i64)?;

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

    // The second walk (289-308): neighbor1 from orient 1, neighbor2
    // from orient 2, neighbor3 from orient 0. The reusable line
    // buffer (cleared per line, capacity reused): one write_all per
    // data line instead of one write! per line - the R1 cause-2
    // site.
    let mut line = String::new();
    let mut elementnumber: i64 = b.firstnumber as i64;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.traverse_triangle(&mut t) {
        let neighbor1 = m.tri_int(m.sym(Otri { tri, orient: 1 }).tri, numberindex);
        let neighbor2 = m.tri_int(m.sym(Otri { tri, orient: 2 }).tri, numberindex);
        let neighbor3 = m.tri_int(m.sym(Otri { tri, orient: 0 }).tri, numberindex);
        // Triangle number, neighboring triangle numbers (303-304:
        // "%4ld    %d  %d  %d\n").
        line.clear();
        write_neigh_line(&mut line, elementnumber, [neighbor1, neighbor2, neighbor3]);
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
    use crate::options::Options;

    /// An assembled Behavior from a closure over Options (the
    /// io/arrays.rs test precedent; Options::validate runs C's
    /// behavior_update tail exactly like the entry points).
    fn behavior_with(setup: impl FnOnce(&mut Options)) -> Behavior {
        let mut opts = Options::new();
        setup(&mut opts);
        opts.validate().expect("validate")
    }

    /// The two-triangle unit square of the io/arrays.rs fixture: t1
    /// (a, e, c) bonded to t2 (a, c, d) across t1's orient-2 edge;
    /// the vertices are the pool rows 1..=4, the triangles rows 2
    /// and 3.
    #[allow(clippy::type_complexity)]
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
        let t1 = m.make_triangle(b.usesegments, b.vararea).expect("tri");
        m.set_org(t1, a);
        m.set_dest(t1, e);
        m.set_apex(t1, c);
        let t2 = m.make_triangle(b.usesegments, b.vararea).expect("tri");
        m.set_org(t2, a);
        m.set_dest(t2, c);
        m.set_apex(t2, d);
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

    // -- the writers, pinned by construction from the printf spec --

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
}

// R4 module reorganization (docs/architecture-spec.md section 5): the
// I/O phase lives under io/ - the in-memory array transfer and the
// writers (arrays.rs), and the EPS renderer (io/eps.rs). The
// interchange formats (parse seam, readers, field/header/trailer
// emitters) moved to the triangle-io crate (ADR 0006, issue #70).
pub(crate) mod arrays;
pub(crate) mod eps;
