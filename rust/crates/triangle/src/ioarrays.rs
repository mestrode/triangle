//! The C Array I/O routines at the mesh level (triangle.c:7687-8134,
//! the section's opening banner at 7687 and its closing banner at
//! 8134). The section's input half is transfernodes, the path that
//! reads the vertices from memory (landed, wired into
//! triangle_mesh_create's orchestration at the api.rs boundary); the
//! output half - writenodes (triangle.c:7768), numbernodes (7840),
//! writeelements (7863), writepoly (7933), writeedges (7983),
//! writeneighbors (8054) - is NOT landed: the writers marshal the mesh
//! into the public MeshOutput types and feed Mesh::output
//! (triangle_mesh_copy), and they are the next ioarrays PR, so that
//! entry point keeps its funnel-marked scaffolding.
//! quality_statistics once closed the C section (triangle.c:8143); it
//! moved to quality.rs for cohesion with its only caller
//! (docs/function-mapping.md).
//!
//! Input seam (ADR 0004 section 5.2): C's flat `REAL *pointlist`,
//! `REAL *pointattriblist` and `int *pointmarkerlist` arrive as the
//! owned MeshInput fields, passed by reference at the api.rs call
//! site exactly where triangle_api.c hands its triangleio pointers to
//! transfernodes (triangle_api.c:200-202). The Point2 records carry
//! C's two-REAL stride per point (the carveholes precedent for the
//! typed records of the frozen input surface, construct.rs), the
//! attribute list stays flat as in C, and the marker list mirrors as
//! Option, preserving C's NULL-vs-present branch (triangle.c:7732): an
//! empty MeshInput marker list is C's NULL list.

use crate::error::InternalStatus;
use crate::input::Point2;
use crate::mesh::Mesh;
use crate::options::Behavior;
use crate::slots::INPUTVERTEX;

/// C: transfernodes (triangle.c:7697-7757, banner 7691-7696). Read the
/// vertices from memory: store the input counters, size the vertex
/// pool, then walk the input once - one poolalloc'd vertex row per
/// point, written with its two coordinates, its attributes, its
/// boundary marker (or the default 0) and its INPUTVERTEX type - while
/// updating the bounds, and finish with the sweepline circle-event
/// flag.
///
/// C returns TRI_INPUT (triangle.c:7712) where the port carries the
/// mirrored code on the Result's error channel (the splittriangle
/// precedent, refine.rs; the api.rs boundary maps it through the
/// funnel, D8), and TRI_OK (triangle.c:7756) as Ok. The `b->weighted`
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
) -> Result<(), InternalStatus> {
    m.invertices = numberofpoints;
    m.mesh_dim = 2;
    m.nextras = numberofpointattribs;
    if m.invertices < 3 {
        // Error:  Input must have at least three input vertices.
        return Err(InternalStatus::Input);
    }
    if m.nextras == 0 {
        b.weighted = 0;
    }

    m.initializevertexpool(b.poly)
        .map_err(InternalStatus::from)?;
    // The pool geometry already ran eagerly in Mesh::new over the same
    // input counters (the ADR 0003 section 4 convention, the same one
    // delaunay.rs cites at its initializetrisubpools site), and no
    // vertex row exists on any transfernodes call path yet, so this
    // C-site call re-creates an identical empty pool: no allocation,
    // id, reuse, or traversal state exists that could observe it.

    // Read the vertices.
    let mut attribindex = 0usize;
    for i in 0..m.invertices as usize {
        let vertexloop = m.vx_alloc().map_err(InternalStatus::from)?;
        // Read the vertex coordinates.
        let x = pointlist[i].x;
        let y = pointlist[i].y;
        m.vx_set_real(vertexloop, 0, x);
        m.vx_set_real(vertexloop, 1, y);
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
            assert_eq!(m.vx_real(v, 0), point.x);
            assert_eq!(m.vx_real(v, 1), point.y);
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
    fn fewer_than_three_points_funnels_the_input_error() {
        // C returns TRI_INPUT (triangle.c:7710-7712) after the counter
        // stores and before the weighted reset; the port carries the
        // same code on the error channel and the funnel maps it to
        // Error::Input exactly where triangle_api.c returns the status
        // (triangle_api.c:204-206).
        let mut b = weighted_behavior();
        let mut m = core_mesh(&b, 2, 0);
        let points = [Point2 { x: 0.0, y: 0.0 }, Point2 { x: 1.0, y: 0.0 }];
        let err = transfernodes(&mut m, &mut b, &points, &[], Some(&[1, 2]), 2, 0)
            .expect_err("fewer than three points must fail");
        assert_eq!(err, InternalStatus::Input);
        assert!(matches!(Error::from(err), Error::Input(_)));
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
}
