//! Mesh quality testing: checkmesh, checkdelaunay, the bad-triangle queue
//! (enqueuebadtriang, enqueuebadtri, dequeuebadtriang), inlens,
//! checkseg4encroach, testtriangle (with the Unsuitable callback call site),
//! quality_statistics.
//! C: triangle.c section Mesh quality testing routines (+ quality_statistics
//! from the Array I/O section tail, moved for cohesion). Assignment:
//! docs/function-mapping.md.
//!
//! Stage-1 landing state: only the makevertexmap row is awake (landed
//! with its construct.rs consumer, the formskeleton row, mirroring the
//! vertextraverse-in-PR-#33 consumer-carry precedent; PR #35). The rest
//! of this module lands with the refine/enforcequality slice.

use crate::handles::{encode, Otri};
use crate::mesh::Mesh;
use crate::options::Behavior;

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::delaunay::delaunay;

    /// A hand square triangulated by the landed divconq dispatcher: after
    /// makevertexmap every vertex's vertex2tri word decodes to a triangle
    /// whose org is exactly that vertex (the makevertexmap contract,
    /// triangle.c:2206-2222).
    #[test]
    fn makevertexmap_points_every_vertex_at_an_org_match() {
        let b = Behavior {
            poly: true,
            usesegments: true,
            ..Behavior::default_form()
        };
        let mut m = Mesh::new(&b, 4, 2, 0, 0).expect("mesh init");
        for (x, y) in [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)] {
            let v = m.vx_alloc().expect("vertex alloc");
            m.vx_set_real(v, 0, x);
            m.vx_set_real(v, 1, y);
        }
        assert_eq!(delaunay(&mut m, &b).expect("delaunay"), 4);

        // A fresh mesh: no vertex2tri word set yet.
        for v in 1..=4u32 {
            assert_eq!(m.vx_2tri(v), 0);
        }

        makevertexmap(&mut m, &b);
        for v in 1..=4u32 {
            let word = m.vx_2tri(v);
            assert_ne!(word, 0, "vertex {} unmapped", v);
            let tri = crate::handles::decode(word);
            assert!(!m.deadtri(tri.tri));
            assert_eq!(m.org(tri), v);
        }
    }
}
