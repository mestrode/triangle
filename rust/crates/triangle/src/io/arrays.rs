//! The C Array I/O routines at the mesh level (triangle.c:7687-7757):
//! transfernodes reads the vertices from memory (landed, wired into
//! triangle_mesh_create's orchestration at the api.rs boundary). The
//! section's six writers - writenodes, numbernodes, writeelements,
//! writepoly, writeedges, writeneighbors - moved to io/marshal.rs
//! with the output-phase consolidation (issue #72): the mesh-bound
//! output code has one owner and one record seam per artifact there.
//! The C section's banner pair is triangle.c:7687 and 8134.
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
//! typed records of the frozen input surface, algorithm/segments.rs),
//! the attribute list stays flat as in C, and the marker list mirrors
//! as Option, preserving C's NULL-vs-present branch (triangle.c:7732):
//! an empty MeshInput marker list is C's NULL list.

use crate::error::Error;
use crate::input::Point2;
use crate::mesh::slots::INPUT_VERTEX;
use crate::mesh::Mesh;
use crate::options::Behavior;

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
pub(crate) fn transfer_nodes(
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

    m.init_vertex_pool(b.poly).map_err(Error::from)?;
    // The pool geometry already ran eagerly in Mesh::new over the same
    // input counters (the ADR 0003 section 4 convention, the same one
    // algorithm/algorithm/delaunay.rs cites at its initializetrisubpools site), and no
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
        m.vx_set_type(vertexloop, INPUT_VERTEX);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::Error;
    use crate::io::fixtures::{core_mesh, three_points};
    use crate::options::Options;

    fn behavior() -> Behavior {
        Options::new().validate().expect("validate")
    }

    fn weighted_behavior() -> Behavior {
        let mut opts = Options::new();
        opts.weighted();
        opts.validate().expect("validate")
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
        transfer_nodes(&mut m, &mut b, &points, &attrs, Some(&markers), 3, 1).expect("load");
        assert_eq!(m.invertices, 3);
        assert_eq!(m.mesh_dim, 2);
        assert_eq!(m.nextras, 1);
        let mut t = m.vertices.traversal();
        for (k, point) in points.iter().enumerate() {
            let v = m.traverse_vertex(&mut t).expect("three rows");
            assert_eq!(m.vx_x(v), point.x);
            assert_eq!(m.vx_y(v), point.y);
            assert_eq!(m.vx_real(v, 2), attrs[k]);
            assert_eq!(m.vx_mark(v), markers[k]);
            assert_eq!(m.vx_type(v), INPUT_VERTEX);
        }
        assert!(m.traverse_vertex(&mut t).is_none());
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
        transfer_nodes(&mut m, &mut b, &points, &[], None, 3, 0).expect("load");
        assert_eq!(m.nextras, 0);
        let mut t = m.vertices.traversal();
        while let Some(v) = m.traverse_vertex(&mut t) {
            assert_eq!(m.vx_mark(v), 0);
            assert_eq!(m.vx_type(v), INPUT_VERTEX);
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
        let err = transfer_nodes(&mut m, &mut b, &points, &[], Some(&[1, 2]), 2, 0)
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
        transfer_nodes(&mut m, &mut b, &points, &[], None, 3, 0).expect("load");
        assert_eq!(b.weighted, 0);

        let mut b = weighted_behavior();
        let mut m = core_mesh(&b, 3, 1);
        let (points, attrs, _markers) = three_points();
        transfer_nodes(&mut m, &mut b, &points, &attrs, None, 3, 1).expect("load");
        assert_eq!(b.weighted, 1);
    }
}
