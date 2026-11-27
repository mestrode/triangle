//! The public input types (ADR 0004, sections 5.1/5.2): MeshInput and
//! Point2, mirroring the input half of C's 'triangleio' struct
//! (triangle.h:83-90) as owned Vecs with no libc types, validated by the
//! transliterated semantics of 'triangle_check_triangleio'
//! (triangle_helper.c:25-72), plus the output path's internal
//! 'triangle_restore_pointmarkers' (triangle_helper.c:77-97): the
//! vertex traversal that puts the original point markers back over
//! the vertex numbers writenodes left on the marks (triangle_api.c:
//! 423-425). It reduces to vertextraverse/setvertexmark and rides the
//! mesh/mod.rs vertex traversal row.

use crate::error::Error;
use crate::mesh::slots::UNDEAD_VERTEX;
use crate::mesh::Mesh;
use crate::options::Behavior;

/// A point in the plane: the two REALs of C's pointlist entry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}

/// A hole seed: the two REALs of C's holelist entry (input only).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hole {
    pub x: f64,
    pub y: f64,
}

/// A regional attribute / area constraint entry: the four REALs of C's
/// regionlist entry (input only).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Region {
    pub x: f64,
    pub y: f64,
    pub attribute: f64,
    pub max_area: f64,
}

/// The input half of C's 'triangleio', owned and typed. The flat-list
/// shapes stay flat where the transliterated consumers
/// (transfernodes, formskeleton, reconstruct) read flat lists in C;
/// counts are derived from the Vec lengths instead of being carried as
/// separate numbers of points/segments/holes/regions.
///
/// The refine path ('-r') reads 'elements', 'corners_per_element',
/// 'element_attributes', 'attributes_per_element' and 'triangle_areas';
/// for a pure PSLG triangulation they stay empty.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MeshInput {
    /// Points to triangulate; C pointlist.
    pub points: Vec<Point2>,
    /// C pointmarkerlist, one per point.
    pub point_markers: Vec<i32>,
    /// C pointattributelist, flat: 'attributes_per_point' REALs per
    /// point. nextras in C.
    pub point_attributes: Vec<f64>,
    pub attributes_per_point: usize,
    /// C segmentlist, endpoint pairs per segment ('-p').
    pub segments: Vec<[i32; 2]>,
    /// C segmentmarkerlist, one per segment.
    pub segment_markers: Vec<i32>,
    /// C holelist ('-p' holes).
    pub holes: Vec<Hole>,
    /// C regionlist ('-p' regions).
    pub regions: Vec<Region>,
    /// C trianglelist, flat corners of the elements to reconstruct
    /// ('-r' refine path).
    pub elements: Vec<i32>,
    /// C numberofcorners (3, or 6 with elements of order 2). Zero means
    /// "no elements".
    pub corners_per_element: usize,
    /// C triangleattributelist, flat: 'attributes_per_element' REALs per
    /// element. eextras in C.
    pub element_attributes: Vec<f64>,
    pub attributes_per_element: usize,
    /// C trianglearealist, one REAL per element ('-r').
    pub triangle_areas: Vec<f64>,
}

impl MeshInput {
    /// The transliterated validation semantics of
    /// 'triangle_check_triangleio' (triangle_helper.c:25-72), returning
    /// Error::Input where C returns TRI_FAILURE. C validates segment
    /// endpoints and element corners against [firstnumber,
    /// firstnumber + numberofpoints) and rejects degenerate segments.
    ///
    /// Two port notes (flagged in the PR body):
    /// - C reads the coordinate of endpoint 'a' at pointlist[2*a], using
    ///   the raw (firstnumber-offset) endpoint number as a zero-based
    ///   index; with firstnumber = 1 that skips the first point and
    ///   reads one entry past the last. This port indexes correctly
    ///   (a - firstnumber); the C site is never called anywhere in C
    ///   (fork-added helper) so no parity path observes the difference.
    /// - The C coincident-endpoint test computes
    ///   (x_a == y_a) && (x_b == y_b) instead of comparing the two
    ///   endpoints with each other (triangle_helper.c:50-54, its TODO
    ///   text disagrees). Transliterated literally: a segment whose
    ///   endpoints both have x == y is rejected.
    pub fn check(&self, firstnumber: i32) -> Result<(), Error> {
        let invertices = self.points.len() as i32;
        let corners = self.corners_per_element;
        // C: numberoftriangles, derived from the flat list length.
        let inelements = self.elements.len().checked_div(corners).unwrap_or(0);

        for (i, seg) in self.segments.iter().enumerate() {
            let (a, b) = (seg[0], seg[1]);
            if (a < firstnumber) || (a >= firstnumber + invertices) {
                // C: TODO: Warning: Invalid first endpoint of segment.
                return Err(Error::Input(format!(
                    "invalid first endpoint {} of segment {} (TRI_FAILURE)",
                    a, i
                )));
            } else if (b < firstnumber) || (b >= firstnumber + invertices) {
                // C: TODO: Warning: Invalid second endpoint of segment.
                return Err(Error::Input(format!(
                    "invalid second endpoint {} of segment {} (TRI_FAILURE)",
                    b, i
                )));
            } else if a == b {
                // C: TODO: Warning: Endpoints of segment are coincident.
                return Err(Error::Input(format!(
                    "endpoints of segment {} are the same vertex (TRI_FAILURE)",
                    i
                )));
            } else {
                let (ia, ib) = ((a - firstnumber) as usize, (b - firstnumber) as usize);
                // The literal C comparison (triangle_helper.c:50-56):
                // x1 = x_a, x2 = y_a, y1 = x_b, y2 = y_b, then
                // (x1 == x2) && (y1 == y2). See the doc note above.
                let pa = &self.points[ia];
                let pb = &self.points[ib];
                let (x1, x2) = (pa.x, pa.y);
                let (y1, y2) = (pb.x, pb.y);
                if (x1 == x2) && (y1 == y2) {
                    // C: TODO: Warning: Endpoints of segment are coincident.
                    return Err(Error::Input(format!(
                        "endpoints of segment {} are coincident (TRI_FAILURE)",
                        i
                    )));
                }
            }
        }

        // C reads corners 0, 1 and 2 of every element unconditionally
        // (triangle_helper.c:58-68); with a malformed corners count
        // below 3 there is nothing well-defined to validate, so the
        // loop mirrors C only for the shapes C accepts (3 or 6
        // corners). chunks(0) is a panic, hence the guard.
        if corners >= 3 {
            for (i, elem) in self.elements.chunks(corners).take(inelements).enumerate() {
                let (a, b, c) = (elem[0], elem[1], elem[2]);
                let in_range = |v: i32| !(v < firstnumber || v >= firstnumber + invertices);
                if !in_range(a) || !in_range(b) || !in_range(c) {
                    // C: TODO: Error: Triangle (i) has an invalid vertex index.
                    return Err(Error::Input(format!(
                        "triangle {} has an invalid vertex index (TRI_FAILURE)",
                        i
                    )));
                }
            }
        }

        // C returns TRI_OK.
        Ok(())
    }
}

/// triangle_restore_pointmarkers (triangle_helper.c:77-97): the
/// output path's internal step - after writenodes numbered the
/// vertices (the numbers sat over the marks, triangle.c:7823), the
/// original markers left in the pointmarkerlist and this walk puts
/// them back. The vertexnumber here is a list index (it starts at 0,
/// not at b->firstnumber) and the jettison gate skips the undead
/// exactly like the writer that produced the list (triangle.c:
/// 7810-7814); C's ctx plumbing dissolves (D8).
pub(crate) fn restore_point_markers(m: &mut Mesh, b: &Behavior, pointmarkers: &[i32]) {
    let mut vertexnumber: usize = 0;
    let mut t = m.vertices.traversal();
    while let Some(vertexloop) = m.traverse_vertex(&mut t) {
        if !b.jettison || m.vx_type(vertexloop) != UNDEAD_VERTEX {
            m.vx_set_mark(vertexloop, pointmarkers[vertexnumber]);
            vertexnumber += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::options::Options;

    fn simple_input() -> MeshInput {
        MeshInput {
            points: vec![
                Point2 { x: 0.0, y: 0.0 },
                Point2 { x: 1.0, y: 0.0 },
                Point2 { x: 0.0, y: 1.0 },
            ],
            segments: vec![[0, 1], [1, 2], [2, 0]],
            ..MeshInput::default()
        }
    }

    #[test]
    fn valid_input_passes() {
        // Both numberings work when endpoints are in range.
        simple_input().check(0).expect("0-based ok");
        let mut inp = simple_input();
        inp.segments = vec![[1, 2], [2, 3], [3, 1]];
        inp.check(1).expect("1-based ok");
    }

    #[test]
    fn empty_input_passes() {
        MeshInput::default().check(0).expect("empty ok");
        MeshInput::default().check(1).expect("empty ok");
    }

    #[test]
    fn segment_endpoint_out_of_range() {
        let mut inp = simple_input();
        inp.segments[0] = [0, 3];
        assert!(matches!(inp.check(0), Err(Error::Input(_))));
        let mut inp = simple_input();
        inp.segments[0] = [-1, 1];
        assert!(matches!(inp.check(0), Err(Error::Input(_))));
    }

    #[test]
    fn segment_same_vertex_rejected() {
        let mut inp = simple_input();
        inp.segments[0] = [1, 1];
        assert!(matches!(inp.check(0), Err(Error::Input(_))));
    }

    /// The literal C comparison (see the check() doc note): rejection
    /// fires when both endpoints have x == y, and does NOT fire for a
    /// truly coincident pair off the diagonal.
    #[test]
    fn coincident_check_is_the_literal_c_comparison() {
        let mut inp = simple_input();
        // Both endpoints on the diagonal x == y: rejected (C behavior).
        inp.points[0] = Point2 { x: 2.0, y: 2.0 };
        inp.points[1] = Point2 { x: 3.0, y: 3.0 };
        inp.segments[0] = [0, 1];
        assert!(matches!(inp.check(0), Err(Error::Input(_))));
        // Truly coincident but off the diagonal: accepted (C behavior;
        // the C TODO text disagrees with its own code).
        let mut inp2 = simple_input();
        inp2.points[0] = Point2 { x: 5.0, y: 6.0 };
        inp2.points[1] = Point2 { x: 5.0, y: 6.0 };
        inp2.segments[0] = [0, 1];
        inp2.check(0).expect("literal comparison accepts");
    }

    #[test]
    fn triangle_corner_out_of_range() {
        let mut inp = simple_input();
        inp.corners_per_element = 3;
        inp.elements = vec![0, 1, 3];
        assert!(matches!(inp.check(0), Err(Error::Input(_))));
        let mut inp2 = simple_input();
        inp2.corners_per_element = 3;
        inp2.elements = vec![0, 1, 2];
        inp2.check(0).expect("valid triangle");
    }

    #[test]
    fn restore_pointmarkers_puts_the_original_marks_back() {
        // The output path's tail (triangle_helper.c:77-97):
        // writenodes numbered the marks (triangle.c:7823) and the
        // original markers left in the list; the restore walks the
        // same traversal and puts them back, so a second writenodes
        // run reproduces the same list.
        let mut b = Options::new().validate().expect("validate");
        let mut m = Mesh::new(&b, 3, 2, 0, 0).expect("mesh");
        let points = [
            Point2 { x: 0.0, y: 0.0 },
            Point2 { x: 1.0, y: 0.0 },
            Point2 { x: 0.0, y: 1.0 },
        ];
        let markers = [4, 5, 6];
        crate::io::arrays::transfer_nodes(&mut m, &mut b, &points, &[], Some(&markers), 3, 0)
            .expect("load");
        let mut plist: Vec<Point2> = Vec::new();
        let mut palist: Vec<f64> = Vec::new();
        let mut pmlist: Vec<i32> = Vec::new();
        crate::io::arrays::write_nodes(&mut m, &b, &mut plist, &mut palist, &mut pmlist)
            .expect("writenodes");
        // the original markers left in the list, the numbers sit on
        // the marks
        assert_eq!(pmlist, vec![4, 5, 6]);
        let mut t = m.vertices.traversal();
        assert_eq!(m.vx_mark(m.traverse_vertex(&mut t).expect("vx")), 1);
        assert_eq!(m.vx_mark(m.traverse_vertex(&mut t).expect("vx")), 2);
        assert_eq!(m.vx_mark(m.traverse_vertex(&mut t).expect("vx")), 3);
        restore_point_markers(&mut m, &b, &pmlist);
        let mut t = m.vertices.traversal();
        assert_eq!(m.vx_mark(m.traverse_vertex(&mut t).expect("vx")), 4);
        assert_eq!(m.vx_mark(m.traverse_vertex(&mut t).expect("vx")), 5);
        assert_eq!(m.vx_mark(m.traverse_vertex(&mut t).expect("vx")), 6);
        // and the round trip is stable: writenodes again reproduces
        // the same marshalled list
        let mut plist2: Vec<Point2> = Vec::new();
        let mut palist2: Vec<f64> = Vec::new();
        let mut pmlist2: Vec<i32> = Vec::new();
        crate::io::arrays::write_nodes(&mut m, &b, &mut plist2, &mut palist2, &mut pmlist2)
            .expect("writenodes");
        assert_eq!(pmlist2, vec![4, 5, 6]);
    }

    #[test]
    fn restore_pointmarkers_skips_the_undead_under_jettison() {
        // The jettison gate (triangle_helper.c:91): the undead row
        // never entered the marker list (writenodes skipped it,
        // triangle.c:7810-7814), so the restore walks past it
        // without consuming a list entry - the undead keeps its old
        // marker through both walks.
        let mut opts = Options::new();
        opts.jettison();
        let mut b = opts.validate().expect("validate");
        let mut m = Mesh::new(&b, 3, 2, 0, 0).expect("mesh");
        let points = [
            Point2 { x: 0.0, y: 0.0 },
            Point2 { x: 1.0, y: 0.0 },
            Point2 { x: 0.0, y: 1.0 },
        ];
        let markers = [4, 5, 6];
        crate::io::arrays::transfer_nodes(&mut m, &mut b, &points, &[], Some(&markers), 3, 0)
            .expect("load");
        let mut t = m.vertices.traversal();
        let v1 = m.traverse_vertex(&mut t).expect("vx");
        let v2 = m.traverse_vertex(&mut t).expect("vx");
        let v3 = m.traverse_vertex(&mut t).expect("vx");
        m.vx_set_type(v2, UNDEAD_VERTEX);
        m.undeads = 1;
        let mut plist: Vec<Point2> = Vec::new();
        let mut palist: Vec<f64> = Vec::new();
        let mut pmlist: Vec<i32> = Vec::new();
        crate::io::arrays::write_nodes(&mut m, &b, &mut plist, &mut palist, &mut pmlist)
            .expect("writenodes");
        // the living rows carry numbers, the undead keeps its input
        // marker, and the list holds the two living markers only
        assert_eq!(m.vx_mark(v1), 1);
        assert_eq!(m.vx_mark(v2), 5);
        assert_eq!(m.vx_mark(v3), 2);
        assert_eq!(pmlist, vec![4, 6]);
        restore_point_markers(&mut m, &b, &pmlist);
        assert_eq!(m.vx_mark(v1), 4);
        assert_eq!(m.vx_mark(v2), 5);
        assert_eq!(m.vx_mark(v3), 6);
    }
}
