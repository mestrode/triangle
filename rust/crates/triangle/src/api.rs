//! The public entry points and the owned Mesh handle (ADR 0004,
//! section 5.3): the idiomatic mirror of the surviving surface of
//! triangle_api.c (triangulate, Mesh::load_mesh / refine / output /
//! statistics / quality / check_mesh / check_delaunay /
//! memory_bytes) plus the public MeshOutput, Statistics, Quality,
//! BoundingBox types. Each method cites its C entry point; the
//! context pointer of C becomes owned values (D8) and the error funnel
//! (error.rs, ADR 0004 section 6) runs exactly here, where
//! triangle_api.c inspects returned statuses.
//!
//! Stage-1 scaffolding: the entry points own their options
//! validation, input validation and pool-geometry construction, and
//! triangulate runs the full triangle_mesh_create sequence through
//! the landed core (transfernodes, delaunay, formskeleton,
//! carveholes, enforcequality, highorder). The remaining scaffolding
//! sites return the funnel-marked InternalStatus::NotImplemented
//! (each site carries a SCAFFOLDING comment naming the C step it
//! awaits): load_mesh awaits its api-level load orchestration.
//! output runs the ioarray writers through the triangle_mesh_copy
//! lift (the ioarrays part-2 PR).

// The Mesh struct lands ahead of the core methods that read its
// behavior (every stage-1 core module takes a &Behavior); the blanket
// silences that ahead-of-use dead_code and is removed with them.
#![allow(dead_code)]

use crate::error::{Error, InternalStatus};
use crate::handles::NULL;
use crate::input::MeshInput;
use crate::input::Point2;
use crate::options::{Behavior, Options};
use crate::pool::WordPool;

/// The output half of the C triangleio surface (ADR 0004 section 5.2),
/// produced by Mesh::output (the triangle_mesh_copy lift,
/// triangle_api.c:362). Owned values, no libc types.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MeshOutput {
    /// C pointlist as it leaves writenodes (numbered).
    pub points: Vec<Point2>,
    /// C pointmarkerlist.
    pub point_markers: Vec<i32>,
    /// C pointattributelist, flat.
    pub point_attributes: Vec<f64>,
    pub attributes_per_point: usize,
    /// C trianglelist, flat corners.
    pub elements: Vec<i32>,
    /// C numberofcorners: (order + 1) * (order + 2) / 2.
    pub corners: usize,
    /// C triangleattributelist, flat.
    pub element_attributes: Vec<f64>,
    pub attributes_per_element: usize,
    /// C segmentlist (C numberofsegments derives from the length).
    pub segments: Vec<[i32; 2]>,
    /// C segmentmarkerlist.
    pub segment_markers: Vec<i32>,
    /// C numberofholes / numberofregions (the m->holes / m->regions
    /// stores, triangle_api.c:404-405). The lists themselves are the
    /// commented-out input passthrough in C (triangle_api.c:407-408)
    /// and stay empty; the !poly arm NULLs them in C (409-412).
    pub number_of_holes: usize,
    pub number_of_regions: usize,
    /// C edgelist, two ints per edge (Mesh::output(edges: true)).
    pub edges: Vec<[i32; 2]>,
    /// C edgemarkerlist.
    pub edge_markers: Vec<i32>,
    /// C neighborlist, three ints per element (Mesh::output(neighbors:
    /// true), the transliterated writeneighbors).
    pub neighbors: Vec<i32>,
}

/// The statistics surface (ADR 0004 sections 5.3/5.5): the mirror of
/// C's struct statistics (triangle_api.h), filled by Mesh::statistics
/// (triangle_mesh_statistics) with the exact C counters once the core
/// modules that move them land.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Statistics {
    pub vertices: usize,
    pub undeads: usize,
    pub triangles: usize,
    pub hullsize: usize,
    pub subsegs: usize,
    pub edges: usize,
    pub bounds: BoundingBox,
}

/// C struct rect (triangle_api.h) as it rides statistics bounds.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct BoundingBox {
    pub xmin: f64,
    pub ymin: f64,
    pub xmax: f64,
    pub ymax: f64,
}

/// The quality surface (ADR 0004 sections 5.3/5.5): the mirror of
/// C's struct quality (triangle.h:568-576), filled by Mesh::quality
/// (triangle_mesh_quality, which is quality_statistics) with exact C
/// arithmetic once quality.rs lands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quality {
    pub shortest: f64,
    pub longest: f64,
    pub smallest_area: f64,
    pub biggest_area: f64,
    pub smallest_angle: f64,
    pub biggest_angle: f64,
    pub min_altitude: f64,
    pub worst_aspect: f64,
    /// C angletable[18].
    pub angle_table: [usize; 18],
    /// C aspecttable[16].
    pub aspect_table: [usize; 16],
}

impl Default for Quality {
    fn default() -> Quality {
        Quality {
            shortest: 0.0,
            longest: 0.0,
            smallest_area: 0.0,
            biggest_area: 0.0,
            smallest_angle: 0.0,
            biggest_angle: 0.0,
            min_altitude: 0.0,
            worst_aspect: 0.0,
            angle_table: [0; 18],
            aspect_table: [0; 16],
        }
    }
}

/// The owned mesh handle (ADR 0004 section 5.3): owns the pools, the
/// row shapes, the normalized Behavior and the run state of the
/// transliterated core - the parts C splits between context, mesh and
/// behavior (D8: TRI_NULL and the null-context checks dissolve).
///
/// Stage-1 note: triangulate succeeds since the ioarrays input half
/// (the full create sequence); load_mesh still returns the scaffold
/// (its api-level load orchestration is unwired); output runs the
/// ioarray writers; the surface is frozen now so the stage-1
/// core tickets fill it without churn.
///
/// Interior note (serial-era bridge): the frozen `&self` check and
/// statistics surface needs to run the 1:1 transliterations of
/// checkmesh/checkdelaunay/quality_statistics, whose C bodies write
/// through context state (the predicate counters on the core mesh, the
/// noexact save/restore on the behavior). The two owned fields ride
/// RefCell so the frozen signatures can borrow them mutably for the
/// duration of one call; no public method re-enters (a check never
/// calls back into the surface, and the Unsuitable callback receives
/// copied values only, D9), so the runtime borrows never overlap. This
/// rides until the MT milestone (ADR 0005, ticket #12) reworks the
/// counters and the callback storage for the Send + Sync &self surface
/// its reentrancy ruling demands (the same rework that replaces the
/// D13 RefCell on the Behavior).
pub struct Mesh {
    pub(crate) core: std::cell::RefCell<crate::mesh::Mesh>,
    pub(crate) behavior: std::cell::RefCell<Behavior>,
}

impl Mesh {
    /// The pool-geometry construction shared by triangulate and
    /// load_mesh: the mesh-init sequence over a validated Behavior and
    /// a checked input (C: the triangleinit of triangle_context_create
    /// plus the transfernodes-fed counters; nextras/eextras come from
    /// the input exactly where transfernodes assigns them in C).
    fn construct(
        behavior: Behavior,
        input: &MeshInput,
        eextras_attribs: usize,
    ) -> Result<Mesh, Error> {
        let invertices = input.points.len() as i32;
        let nextras = input.attributes_per_point as i32;
        let eextras = (input.attributes_per_element + eextras_attribs) as i32;
        let core = crate::mesh::Mesh::new(&behavior, invertices, 2, nextras, eextras)
            .map_err(Error::from)?;
        Ok(Mesh {
            core: std::cell::RefCell::new(core),
            behavior: std::cell::RefCell::new(behavior),
        })
    }

    /// triangle_mesh_create's core sequence (triangle_api.c:200-256):
    /// everything after the shell's validation and pool construction.
    /// C opens with a TRI_FAILURE when the mesh already has triangles
    /// (triangle_api.c:195-197); the port reaches this sequence only
    /// on a freshly constructed mesh, where the triangle count is
    /// necessarily zero, so the check is unobservable and dissolves.
    fn create(&self, input: &MeshInput) -> Result<(), Error> {
        let mut status = InternalStatus::Ok;
        let mut behavior = self.behavior.borrow_mut();
        let mut core = self.core.borrow_mut();
        // C: transfernodes (triangle_api.c:200-202) and the status
        // check (204-206). The owned MeshInput fields arrive exactly
        // where C hands its triangleio pointers to transfernodes; the
        // marker-list Option mirrors C's NULL-vs-present branch.
        crate::ioarrays::transfernodes(
            &mut core,
            &mut behavior,
            &input.points,
            &input.point_attributes,
            point_marker_list(input),
            input.points.len() as i32,
            input.attributes_per_point as i32,
        )?;
        // C: m->steinerleft = b->steiner (triangle_api.c:208). The
        // construct path already stored the same value (Mesh::new
        // mirrors triangle_context_create:35, before any core call
        // runs), so the store is idempotent; kept for the 1:1 walk.
        core.steinerleft = behavior.steiner;
        // C: m->hullsize = delaunay(m, b) (triangle_api.c:209).
        core.hullsize = crate::delaunay::delaunay(&mut core, &behavior)?;
        // C: the three infvertex NULL stores (triangle_api.c:213-215),
        // already the RefCell bridge's construction shape.
        core.infvertex1 = NULL;
        core.infvertex2 = NULL;
        core.infvertex3 = NULL;
        if behavior.usesegments {
            // C: m->checksegments = 1 (triangle_api.c:218).
            core.checksegments = true;
            // C: formskeleton (triangle_api.c:221-222) and the status
            // check (223-225); the owned [i32; 2] pairs flatten into
            // C's flat segmentlist at the call site.
            crate::construct::formskeleton(
                &mut core,
                &behavior,
                &flat_segmentlist(input)?,
                segment_marker_list(input),
                input.segments.len() as i32,
                &mut status,
            )?;
            if (status as i32) < 0 {
                return Err(Error::from(status));
            }
        }
        if behavior.poly && core.triangles.live_count() > 0 {
            // C: the m->holes / m->regions stores (triangle_api.c:
            // 229-230) and carveholes (233).
            core.holes = input.holes.len() as i32;
            core.regions = input.regions.len() as i32;
            crate::construct::carveholes(&mut core, &behavior, &input.holes, &input.regions)?;
        } else {
            // C: without a PSLG there can be no holes or regional
            // attributes; zeroed to avoid an accidental free() later
            // (triangle_api.c:238-239; the owned input makes the free
            // impossible, the zero stores stay verbatim).
            core.holes = 0;
            core.regions = 0;
        }
        if behavior.quality && core.triangles.live_count() > 0 {
            // C: enforcequality (triangle_api.c:245) and the status
            // check (246-248). C's CDT_ONLY compile gate around the
            // block has no port flag; b->quality is the whole
            // condition here.
            crate::refine::enforcequality(&mut core, &behavior, &mut status)?;
            if (status as i32) < 0 {
                return Err(Error::from(status));
            }
        }
        // C: the m->edges writer (triangle_api.c:252-253).
        core.edges = (3 * core.triangles.live_count() as i64 + core.hullsize) / 2;
        if behavior.order > 1 {
            // C: highorder (triangle_api.c:255-256).
            crate::refine::highorder(&mut core, &behavior)?;
        }
        Ok(())
    }
}

/// C in->pointmarkerlist as transfernodes reads it: the empty list is
/// C's NULL list (no markers; every vertex defaults its marker to
/// zero).
fn point_marker_list(input: &MeshInput) -> Option<&[i32]> {
    if input.point_markers.is_empty() {
        None
    } else {
        Some(&input.point_markers)
    }
}

/// C in->segmentmarkerlist as formskeleton reads it: the empty list is
/// C's NULL list (no markers).
fn segment_marker_list(input: &MeshInput) -> Option<&[i32]> {
    if input.segment_markers.is_empty() {
        None
    } else {
        Some(&input.segment_markers)
    }
}

/// C's flat in->segmentlist: the owned [i32; 2] pairs flattened in
/// input order. The try_reserve_exact mirrors the trimalloc rule: an
/// allocation failure funnels to Error::OutOfMemory.
fn flat_segmentlist(input: &MeshInput) -> Result<Vec<i32>, Error> {
    let mut flat: Vec<i32> = Vec::new();
    flat.try_reserve_exact(input.segments.len() * 2)
        .map_err(|_| Error::OutOfMemory)?;
    for seg in &input.segments {
        flat.push(seg[0]);
        flat.push(seg[1]);
    }
    Ok(flat)
}

/// triangle_mesh_create (triangle_api.c:181). The shell owns its
/// opening (options and input validation, the pool geometry); the
/// Mesh::create helper then runs C's core sequence - transfernodes,
/// the steinerleft store, delaunay, the infvertex NULLs, the
/// usesegments/formskeleton gate, the poly/carveholes gate, the
/// quality/enforcequality gate, the m->edges writer, highorder
/// (triangle_api.c:200-256) - and triangulate returns the live mesh.
pub fn triangulate(options: Options, input: &MeshInput) -> Result<Mesh, Error> {
    let behavior = options.validate()?;
    input.check(behavior.firstnumber)?;
    // -A with a PSLG earns the extra element attribute (the
    // regionattrib slot of initializetrisubpools).
    let region_extra = usize::from(behavior.regionattrib);
    let mesh = Mesh::construct(behavior, input, region_extra)?;
    mesh.create(input)?;
    Ok(mesh)
}

impl Mesh {
    /// triangle_mesh_load (triangle_api.c:262). Calling it implies the
    /// refine path in C's API; the same opening validation and pool
    /// geometry. The reconstruct transliteration is landed
    /// (construct.rs); the api-level load orchestration around it is
    /// the remaining scaffold.
    ///
    /// SCAFFOLDING: awaits the load orchestration (triangle_api.c:
    /// 262-330, reconstruct plus the load-era gates); returns the
    /// funnel-marked InternalStatus::NotImplemented today.
    pub fn load_mesh(options: Options, input: &MeshInput) -> Result<Mesh, Error> {
        let behavior = { options }.validate()?;
        input.check(behavior.firstnumber)?;
        let region_extra = usize::from(behavior.regionattrib);
        let _mesh = Mesh::construct(behavior, input, region_extra)?;
        Err(Error::from(InternalStatus::NotImplemented))
    }

    /// triangle_mesh_refine (triangle_api.c:332-359): enforcequality
    /// over the loaded mesh. C gates on b->quality with triangles in
    /// the mesh (triangle_api.c:347), funnels enforcequality's status
    /// (the funnel item 5 translation, error.rs), and - with a good
    /// status - refreshes m->edges inside the gate (triangle_api.c:
    /// 354-355, the (3 * items + hullsize) / 2 arithmetic).
    pub fn refine(&mut self) -> Result<(), Error> {
        let behavior = self.behavior.borrow();
        let mut status = InternalStatus::Ok;
        let mut core = self.core.borrow_mut();
        if behavior.quality && core.triangles.live_count() > 0 {
            crate::refine::enforcequality(&mut core, &behavior, &mut status)?;
            if (status as i32) < 0 {
                return Err(Error::from(status));
            }
            core.edges = (3 * core.triangles.live_count() as i64 + core.hullsize) / 2;
        }
        Ok(())
    }

    /// triangle_mesh_copy (triangle_api.c:362-428): the marshalled
    /// output. C opens with its context NULL check (370-375), which
    /// dissolves over the owned mesh (D8), then stores the count
    /// fields (377-391) and runs the array writers in C order:
    /// writenodes (393-395), writeelements (397-398, always), the
    /// poly || convex gate's writepoly and hole/region stores
    /// (400-413), writeedges under the edges arg (415-417),
    /// writeneighbors under the neighbors arg (419-421) and the
    /// pointmarker restore (423-425). The count-only stores have no
    /// frozen MeshOutput field: the owned lists derive
    /// numberofpoints / numberoftriangles /
    /// numberofpointattributes / numberoftriangleattributes /
    /// numberofcorners, and numberofedges (386) / numberofsegments
    /// (387-391) leave through C's separate count fields only, so
    /// they stay derivable (the edge list carries its own length,
    /// the segment list likewise).
    pub fn output(&self, edges: bool, neighbors: bool) -> Result<MeshOutput, Error> {
        let behavior = self.behavior.borrow();
        let mut core = self.core.borrow_mut();
        // C: the count stores (triangle_api.c:377-391).
        let mut out = MeshOutput {
            attributes_per_point: core.nextras as usize,
            // C: numberofcorners (triangle_api.c:384).
            corners: ((behavior.order + 1) * (behavior.order + 2) / 2) as usize,
            attributes_per_element: core.eextras as usize,
            ..MeshOutput::default()
        };
        // C: writenodes (triangle_api.c:393-395), the numbering
        // writer (it writes the vertex numbers over the marks,
        // triangle.c:7823).
        crate::ioarrays::writenodes(
            &mut core,
            &behavior,
            &mut out.points,
            &mut out.point_attributes,
            &mut out.point_markers,
        )?;
        // C: always write elements (triangle_api.c:397-398).
        crate::ioarrays::writeelements(
            &mut core,
            &behavior,
            &mut out.elements,
            &mut out.element_attributes,
        )?;
        // C: the poly || convex gate (triangle_api.c:400-402): the -c
        // switch causes a PSLG to be written even if none was read.
        if behavior.poly || behavior.convex {
            crate::ioarrays::writepoly(
                &mut core,
                &behavior,
                &mut out.segments,
                &mut out.segment_markers,
            )?;
            // C: the m->holes / m->regions stores (triangle_api.c:
            // 404-405).
            out.number_of_holes = core.holes as usize;
            out.number_of_regions = core.regions as usize;
            // C: the -p arm is the commented-out input passthrough
            // (triangle_api.c:406-408, upstream's own TODO) and the
            // !poly arm NULLs the lists (409-412); the owned lists
            // stay empty in both arms, exactly the C shape.
        }
        // C: the edges gate (triangle_api.c:415-417).
        if edges {
            crate::ioarrays::writeedges(
                &mut core,
                &behavior,
                &mut out.edges,
                &mut out.edge_markers,
            )?;
        }
        // C: the neighbors gate (triangle_api.c:419-421).
        if neighbors {
            crate::ioarrays::writeneighbors(&mut core, &behavior, &mut out.neighbors)?;
        }
        // C: the pointmarker restore (triangle_api.c:423-425), gated
        // on the marker list existing: writenodes wrote the vertex
        // numbers over the marks and the original markers left in
        // the list, so they go back exactly here (-B leaves the
        // list empty and the numbers on the marks, C's NULL arm).
        if !out.point_markers.is_empty() {
            crate::input::restore_pointmarkers(&mut core, &behavior, &out.point_markers);
        }
        Ok(out)
    }

    /// triangle_mesh_statistics (triangle_api.c:111). The mirror of
    /// the C field copy: C reads m->vertices.items, undeads,
    /// triangles.items, hullsize, edges, subsegs (b->poly || b->refine)
    /// and the bounds; the port reads the landed core fields (the pool
    /// live counts are C items, the counters landed with the core).
    pub fn statistics(&self) -> Statistics {
        let m = self.core.borrow();
        let b = self.behavior.borrow();
        Statistics {
            vertices: m.vertices.live_count() as usize,
            undeads: m.undeads as usize,
            triangles: m.triangles.live_count() as usize,
            hullsize: m.hullsize as usize,
            // C's writer of m->edges is (3 * triangles.items +
            // hullsize) / 2, computed in the
            // triangle_mesh_create/load_mesh/refine entry points
            // (triangle_api.c:252-253, 326-327, 354-355). The refine
            // writer landed (refine.rs); the create writer landed with
            // the ioarrays input half (Mesh::create); load awaits its
            // orchestration, so the field stays 0 on its meshes.
            edges: m.edges as usize,
            subsegs: if b.poly || b.refine {
                m.subsegs.live_count() as usize
            } else {
                0
            },
            bounds: BoundingBox {
                xmin: m.xmin,
                ymin: m.ymin,
                xmax: m.xmax,
                ymax: m.ymax,
            },
        }
    }

    /// triangle_mesh_quality (triangle_api.c:102): the failure-free
    /// wrapper of quality_statistics. Wired to the landed 1:1
    /// transliteration (the exact-predicate counters advance exactly
    /// like the C call's side effects).
    pub fn quality(&self) -> Result<Quality, Error> {
        let mut q = Quality::default();
        let mut core = self.core.borrow_mut();
        let b = self.behavior.borrow();
        crate::quality::quality_statistics(&mut core, &b, &mut q);
        Ok(q)
    }

    /// triangle_check_mesh (triangle_api.c:163): the failure count of
    /// checkmesh (C semantics: a count, not an error). Wired to the
    /// landed 1:1 transliteration; the noexact save/restore runs
    /// exactly like C's inside the call.
    pub fn check_mesh(&self) -> usize {
        let mut core = self.core.borrow_mut();
        let mut behavior = self.behavior.borrow_mut();
        crate::quality::checkmesh(&mut core, &mut behavior) as usize
    }

    /// triangle_check_delaunay (triangle_api.c:172): the failure count
    /// of checkdelaunay. Wired to the landed 1:1 transliteration.
    pub fn check_delaunay(&self) -> usize {
        let mut core = self.core.borrow_mut();
        let mut behavior = self.behavior.borrow_mut();
        crate::quality::checkdelaunay(&mut core, &mut behavior) as usize
    }

    /// triangle_memory (triangle_api.c:143): the mirror of C's
    /// maxitems * itembytes sum over the eight pools. Landed for real:
    /// WordPool high_water is the C maxitems mirror and stride * 8 is
    /// itembytes (ADR 0003), so the sum runs over the landed pools.
    pub fn memory_bytes(&self) -> usize {
        fn pool_bytes(p: &WordPool) -> usize {
            p.high_water() as usize * p.stride() as usize * 8
        }
        let m = self.core.borrow();
        pool_bytes(&m.vertices)
            + pool_bytes(&m.triangles)
            + pool_bytes(&m.subsegs)
            + pool_bytes(&m.viri)
            + pool_bytes(&m.badsubsegs)
            + pool_bytes(&m.badtriangles)
            + pool_bytes(&m.flipstackers)
            + pool_bytes(&m.splaynodes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input::{Hole, Region};
    use crate::options::Behavior;

    fn empty_input() -> MeshInput {
        MeshInput::default()
    }

    fn triangle_input() -> MeshInput {
        MeshInput {
            points: vec![
                Point2 { x: 0.0, y: 0.0 },
                Point2 { x: 1.0, y: 0.0 },
                Point2 { x: 0.0, y: 1.0 },
            ],
            ..MeshInput::default()
        }
    }

    fn scaffold(behavior: Behavior, input: &MeshInput) -> Mesh {
        Mesh::construct(behavior, input, 0).expect("pool geometry")
    }

    #[test]
    fn triangulate_runs_the_create_sequence() {
        // triangle_api.c:200-256 end to end: the three vertices reach
        // the pool through transfernodes, delaunay triangulates them
        // (one triangle, hullsize 3), the -p/-q gates stay closed,
        // and the m->edges writer computes (3 * 1 + 3) / 2 = 3
        // (triangle_api.c:252-253). The output half runs the
        // triangle_mesh_copy lift over the created mesh.
        let mesh =
            triangulate(Options::new(), &triangle_input()).expect("triangulate must succeed");
        let s = mesh.statistics();
        assert_eq!(s.vertices, 3);
        assert_eq!(s.triangles, 1);
        assert_eq!(s.hullsize, 3);
        assert_eq!(s.edges, 3);
        assert_eq!(s.subsegs, 0);
        assert_eq!(
            s.bounds,
            BoundingBox {
                xmin: 0.0,
                ymin: 0.0,
                xmax: 1.0,
                ymax: 1.0
            }
        );
        assert_eq!(mesh.check_mesh(), 0);
        assert_eq!(mesh.check_delaunay(), 0);
        // The output lift (triangle_api.c:362-428): the vertices
        // marshal in traversal order (input order here), the single
        // element records its corners, and a second run returns the
        // same lists - the pointmarker restore (triangle_api.c:
        // 423-425) put the original markers back over the numbers,
        // so the walk is idempotent.
        let out = mesh.output(false, false).expect("output");
        assert_eq!(out.points, triangle_input().points);
        // Without -p the hull marking of the create sequence left 1
        // on every convex-hull vertex (removeghosts, triangle.c:
        // 4522-4523, and divconqdelaunay, 4722-4723); writenodes
        // marshals those marks.
        assert_eq!(out.point_markers, vec![1, 1, 1]);
        assert_eq!(out.attributes_per_point, 0);
        assert!(out.point_attributes.is_empty());
        assert_eq!(out.corners, 3);
        assert_eq!(out.elements.len(), 3);
        let mut corners = out.elements.clone();
        corners.sort();
        assert_eq!(corners, vec![1, 2, 3]);
        // the element is the positively oriented (area 0.5)
        // triangle over the three numbered vertices
        let tri: Vec<Point2> = out
            .elements
            .iter()
            .map(|&corner| out.points[(corner - 1) as usize])
            .collect();
        let area = 0.5
            * ((tri[1].x - tri[0].x) * (tri[2].y - tri[0].y)
                - (tri[2].x - tri[0].x) * (tri[1].y - tri[0].y));
        assert!((area - 0.5).abs() < 1e-12);
        assert_eq!(out.attributes_per_element, 0);
        assert!(out.element_attributes.is_empty());
        assert!(out.segments.is_empty());
        assert!(out.segment_markers.is_empty());
        assert_eq!(out.number_of_holes, 0);
        assert_eq!(out.number_of_regions, 0);
        assert!(out.edges.is_empty());
        assert!(out.edge_markers.is_empty());
        assert!(out.neighbors.is_empty());
        assert_eq!(mesh.output(false, false).expect("second output"), out);
    }

    #[test]
    fn triangulate_funnels_the_transfernodes_input_error() {
        // C's transfernodes returns TRI_INPUT below three points
        // (triangle.c:7710-7712); triangle_mesh_create funnels the
        // status (triangle_api.c:204-206) and the port carries the
        // same code as Error::Input exactly there.
        let mut input = triangle_input();
        input.points.pop();
        let err = triangulate(Options::new(), &input)
            .err()
            .expect("two points must fail");
        assert!(matches!(err, Error::Input(_)));
    }

    #[test]
    fn triangulate_rejects_a_bad_fixed_area() {
        let mut opts = Options::new();
        opts.fixed_area(0.0);
        let err = triangulate(opts, &empty_input())
            .err()
            .expect("must fail on a bad fixed area");
        assert!(matches!(err, Error::Options(_)));
    }

    #[test]
    fn load_mesh_validates_input() {
        let mut input = triangle_input();
        input.segments = vec![[0, 3]];
        let err = Mesh::load_mesh(Options::new(), &input)
            .err()
            .expect("load_mesh must fail on bad input");
        assert!(matches!(err, Error::Input(_)));
        // And on a valid input: the scaffolding error.
        let err = Mesh::load_mesh(Options::new(), &triangle_input())
            .err()
            .expect("load_mesh must fail in stage 1");
        assert!(matches!(err, Error::Internal(_)));
    }

    /// Hand-insert one triangle into a scaffolded mesh (the landed core
    /// primitives; the input points stay unloaded pending transfernodes).
    fn lone_core_triangle(m: &mut Mesh, pa: (f64, f64), pb: (f64, f64), pc: (f64, f64)) {
        let core = &mut *m.core.borrow_mut();
        let a = core.vx_alloc().expect("vertex");
        core.vx_set_real(a, 0, pa.0);
        core.vx_set_real(a, 1, pa.1);
        let e = core.vx_alloc().expect("vertex");
        core.vx_set_real(e, 0, pb.0);
        core.vx_set_real(e, 1, pb.1);
        let c = core.vx_alloc().expect("vertex");
        core.vx_set_real(c, 0, pc.0);
        core.vx_set_real(c, 1, pc.1);
        let t = core.maketriangle(false, false).expect("tri");
        core.setorg(t, a);
        core.setdest(t, e);
        core.setapex(t, c);
    }

    #[test]
    fn refine_gates_on_quality_and_keeps_edges_at_zero() {
        // triangle_api.c:347: without -q the stage is a no-op even with
        // triangles in the mesh, and the m->edges writer
        // (triangle_api.c:354-355) lives inside the gate, so the
        // statistics field stays 0; the output lift runs over the
        // untouched mesh.
        let b = Options::new().validate().expect("validate");
        let mut m = scaffold(b, &empty_input());
        lone_core_triangle(&mut m, (0.0, 0.0), (2.0, 0.0), (1.0, 1.0));
        assert!(m.refine().is_ok());
        assert_eq!(m.statistics().edges, 0);
        // The output lift over the hand-inserted triangle: the
        // corners are the numbered vertices, the marker list the
        // default zeros.
        let out = m.output(false, false).expect("output");
        assert_eq!(out.points.len(), 3);
        assert_eq!(out.point_markers, vec![0, 0, 0]);
        assert_eq!(out.elements.len(), 3);
        let mut corners = out.elements.clone();
        corners.sort();
        assert_eq!(corners, vec![1, 2, 3]);
    }

    #[test]
    fn refine_runs_the_quality_stage_through_the_surface() {
        // A 45-45-90 triangle under -q20 has no bad triangle (45 > 20),
        // so the stage drains an empty queue: Ok, the mesh keeps its
        // single triangle, and the edges writer computes
        // (3 * 1 + 0) / 2 = 1 (triangle_api.c:354-355).
        let mut opts = Options::new();
        opts.quality();
        opts.min_angle(20.0);
        let b = opts.validate().expect("validate");
        let mut m = scaffold(b, &empty_input());
        lone_core_triangle(&mut m, (0.0, 0.0), (2.0, 0.0), (1.0, 1.0));
        assert!(m.refine().is_ok());
        assert_eq!(m.statistics().triangles, 1);
        assert_eq!(m.statistics().edges, 1);
        // The output lift over the refined mesh: one element, its
        // corners the numbered vertices.
        let out = m.output(false, false).expect("output");
        assert_eq!(out.elements, vec![1, 2, 3]);
        assert_eq!(out.point_markers, vec![0, 0, 0]);
    }

    #[test]
    #[cfg(feature = "acute")]
    fn refine_repairs_a_skinny_triangle_through_the_acute_path() {
        // The acute build's twin of the NO_ACUTE test below: the same
        // skinny triangle drains its bad-triangle queue through the
        // Ruppert loop, every split through findNewSPLocation
        // (triangle.c:7437, live since stage-1 acute C #43), under a
        // Steiner budget; the mesh grows past the initial triangle.
        let mut opts = Options::new();
        opts.quality();
        opts.min_angle(20.0);
        opts.steiner_limit(50);
        let b = opts.validate().expect("validate");
        let mut m = scaffold(b, &empty_input());
        lone_core_triangle(&mut m, (0.0, 0.0), (2.0, 0.0), (1.0, 0.1));
        assert!(m.refine().is_ok());
        assert!(m.statistics().triangles > 1);
    }

    #[test]
    #[cfg(not(feature = "acute"))]
    fn refine_repairs_a_skinny_triangle_through_the_surface() {
        // The NO_ACUTE compile shape: the same skinny triangle drains
        // its bad-triangle queue through the Ruppert loop under a
        // Steiner budget; the mesh grows past the initial triangle.
        let mut opts = Options::new();
        opts.quality();
        opts.min_angle(20.0);
        opts.steiner_limit(50);
        let b = opts.validate().expect("validate");
        let mut m = scaffold(b, &empty_input());
        lone_core_triangle(&mut m, (0.0, 0.0), (2.0, 0.0), (1.0, 0.1));
        assert!(m.refine().is_ok());
        assert!(m.statistics().triangles > 1);
    }

    #[test]
    fn quality_and_checks_wired_through_the_surface() {
        // The frozen &self signatures run the landed transliterations
        // through the RefCell bridge: an empty construction mesh has no
        // triangle to check (zero horrors), and quality_statistics
        // reports its all-empty initial shape.
        let b = Options::new().validate().expect("validate");
        let m = scaffold(b, &empty_input());
        assert_eq!(m.check_mesh(), 0);
        assert_eq!(m.check_delaunay(), 0);
        let q = m.quality().expect("quality never fails in C");
        assert_eq!(q.angle_table, [0usize; 18]);
        assert_eq!(q.aspect_table, [0usize; 16]);
        // an empty (never-run) mesh keeps C's degenerate extrema: the
        // zero bounds square to minaltitude 0, so shortest collapses
        // onto it and biggest_angle stays at its 180.0 init.
        assert_eq!(q.shortest, 0.0);
        assert_eq!(q.longest, 0.0);
        assert_eq!(q.biggest_angle, 180.0);
    }

    #[test]
    fn check_mesh_reports_a_hand_broken_mesh_through_the_surface() {
        // A deliberately inverted triangle inside the core: the
        // hand-broken construction reports one horror (the inversion
        // probe; every edge is a hull edge, so nothing else fires).
        let b = Options::new().validate().expect("validate");
        let m = scaffold(b, &empty_input());
        {
            let core = &mut *m.core.borrow_mut();
            let v_a = core.vx_alloc().expect("vx");
            core.vx_set_real(v_a, 0, 0.0);
            core.vx_set_real(v_a, 1, 0.0);
            let v_b = core.vx_alloc().expect("vx");
            core.vx_set_real(v_b, 0, 1.0);
            core.vx_set_real(v_b, 1, 0.0);
            let v_c = core.vx_alloc().expect("vx");
            core.vx_set_real(v_c, 0, 0.0);
            core.vx_set_real(v_c, 1, -1.0);
            let t = core.maketriangle(false, false).expect("tri");
            core.setorg(t, v_a);
            core.setdest(t, v_b);
            core.setapex(t, v_c);
        }
        // the opaque counters surface: statistics reports the vertices
        // and the triangle now (edges stays 0 until refine runs its
        // writer; this mesh never calls refine).
        let s = m.statistics();
        assert_eq!(s.vertices, 3);
        assert_eq!(s.triangles, 1);
        assert_eq!(s.edges, 0);
        assert_eq!(m.check_mesh(), 1);
        // the mesh is otherwise empty-Delaunay: no adjacent pair exists
        assert_eq!(m.check_delaunay(), 0);
    }

    #[test]
    fn counters_are_the_frozen_zeroed_shape() {
        let b = Options::new().validate().expect("validate");
        let m = scaffold(b, &empty_input());
        assert_eq!(m.statistics(), Statistics::default());
        assert_eq!(m.check_mesh(), 0);
        assert_eq!(m.check_delaunay(), 0);
        // Fresh pools: no id ever handed out, so C's maxitems-term sum
        // is zero exactly like a poolzero'd C mesh.
        assert_eq!(m.memory_bytes(), 0);
    }

    #[test]
    fn memory_bytes_counts_the_pools_maxitems() {
        // -p with -a (area constraints in the row) and a couple of
        // allocated triangles: the C sum counts the rows ever handed
        // out (maxitems), so allocating grows it.
        let mut opts = Options::new();
        opts.pslg().variable_area();
        let b = opts.validate().expect("validate");
        let m = scaffold(b, &empty_input());
        assert_eq!(m.memory_bytes(), 0);
        let (first, second, rows) = {
            let core = &mut *m.core.borrow_mut();
            let first = core.maketriangle(true, true).expect("alloc");
            let second = core.maketriangle(true, true).expect("alloc");
            let rows = core.tri_shape.stride;
            (first, second, rows)
        };
        assert!(m.memory_bytes() > 0);
        assert!(rows >= 10);
        assert_eq!(m.memory_bytes(), 2 * rows * 8);
        let _ = (first, second);
    }

    #[test]
    fn triangulate_runs_the_pslg_segment_gate() {
        // -p opens the usesegments gate (triangle_api.c:217-226):
        // checksegments flips, formskeleton inserts the three PSLG
        // segments as subsegments, and the poly gate (228-233) stores
        // the hole/region counters (both zero with empty lists).
        let mut opts = Options::new();
        opts.pslg();
        let mut input = triangle_input();
        input.segments = vec![[1, 2], [2, 3], [3, 1]];
        input.segment_markers = vec![7, 8, 9];
        let mesh = triangulate(opts, &input).expect("pslg create");
        let s = mesh.statistics();
        assert_eq!(s.vertices, 3);
        assert_eq!(s.triangles, 1);
        assert_eq!(s.subsegs, 3);
        {
            let core = mesh.core.borrow();
            assert!(core.checksegments);
            assert_eq!(core.holes, 0);
            assert_eq!(core.regions, 0);
        }
    }

    #[test]
    fn triangulate_stores_the_hole_and_region_counters() {
        // The poly gate's stores (triangle_api.c:229-230) with
        // non-empty lists: a square PSLG with one hole seed and one
        // region entry; carveholes consumes both lists and the
        // counters stay stored.
        let mut opts = Options::new();
        opts.pslg();
        let input = MeshInput {
            points: vec![
                Point2 { x: 0.0, y: 0.0 },
                Point2 { x: 1.0, y: 0.0 },
                Point2 { x: 1.0, y: 1.0 },
                Point2 { x: 0.0, y: 1.0 },
            ],
            segments: vec![[1, 2], [2, 3], [3, 4], [4, 1]],
            holes: vec![Hole { x: 0.5, y: 0.5 }],
            regions: vec![Region {
                x: 0.25,
                y: 0.25,
                attribute: 5.0,
                max_area: 0.0,
            }],
            ..MeshInput::default()
        };
        let mesh = triangulate(opts, &input).expect("holed square create");
        {
            let core = mesh.core.borrow();
            assert_eq!(core.holes, 1);
            assert_eq!(core.regions, 1);
        }
    }

    #[test]
    fn triangulate_runs_the_quality_gate() {
        // A skinny input triangle under -q20 with a Steiner budget:
        // the quality gate (triangle_api.c:242-250) runs
        // enforcequality over the loaded vertices, the mesh grows
        // past the initial triangle, and the edges writer counts the
        // grown mesh.
        let mut opts = Options::new();
        opts.quality();
        opts.min_angle(20.0);
        opts.steiner_limit(50);
        let input = MeshInput {
            points: vec![
                Point2 { x: 0.0, y: 0.0 },
                Point2 { x: 2.0, y: 0.0 },
                Point2 { x: 1.0, y: 0.1 },
            ],
            ..MeshInput::default()
        };
        let mesh = triangulate(opts, &input).expect("quality create");
        let s = mesh.statistics();
        assert!(s.triangles > 1);
        assert_eq!(s.edges, (3 * s.triangles + s.hullsize) / 2);
    }

    #[test]
    fn triangulate_promotes_to_second_order() {
        // -o2 opens the b->order > 1 gate (triangle_api.c:255-256):
        // highorder adds one node per edge of the single triangle,
        // so three input vertices become six and the triangle count
        // stays one.
        let mut opts = Options::new();
        opts.element_order(2);
        let mesh = triangulate(opts, &triangle_input()).expect("order-2 create");
        let s = mesh.statistics();
        assert_eq!(s.vertices, 6);
        assert_eq!(s.triangles, 1);
    }

    #[test]
    fn output_runs_the_poly_gate_with_segments_and_counters() {
        // triangle_api.c:400-413: -p opens the poly gate, writepoly
        // marshals the subsegments (endpoint pairs by vertex number,
        // markers in pool order) and the hole/region counters
        // surface (404-405; both zero here, and the lists are the
        // commented-out C passthrough, 407-408).
        let mut opts = Options::new();
        opts.pslg();
        let mut input = triangle_input();
        input.segments = vec![[1, 2], [2, 3], [3, 1]];
        input.segment_markers = vec![7, 8, 9];
        let mesh = triangulate(opts, &input).expect("pslg create");
        let out = mesh.output(false, false).expect("output");
        assert_eq!(out.segments.len(), 3);
        let mut pairs: Vec<[i32; 2]> = out
            .segments
            .iter()
            .map(|p| [p[0].min(p[1]), p[0].max(p[1])])
            .collect();
        pairs.sort();
        assert_eq!(pairs, vec![[1, 2], [1, 3], [2, 3]]);
        let mut markers = out.segment_markers.clone();
        markers.sort();
        assert_eq!(markers, vec![7, 8, 9]);
        assert_eq!(out.number_of_holes, 0);
        assert_eq!(out.number_of_regions, 0);
    }

    #[test]
    fn output_surfaces_the_hole_and_region_counters() {
        // The holed square through the poly gate: the m->holes /
        // m->regions stores (triangle_api.c:404-405) surface as the
        // output counts, and the square's four PSLG segments marshal
        // through writepoly.
        let mut opts = Options::new();
        opts.pslg();
        let input = MeshInput {
            points: vec![
                Point2 { x: 0.0, y: 0.0 },
                Point2 { x: 1.0, y: 0.0 },
                Point2 { x: 1.0, y: 1.0 },
                Point2 { x: 0.0, y: 1.0 },
            ],
            segments: vec![[1, 2], [2, 3], [3, 4], [4, 1]],
            holes: vec![Hole { x: 0.5, y: 0.5 }],
            regions: vec![Region {
                x: 0.25,
                y: 0.25,
                attribute: 5.0,
                max_area: 0.0,
            }],
            ..MeshInput::default()
        };
        let mesh = triangulate(opts, &input).expect("holed square create");
        let out = mesh.output(false, false).expect("output");
        assert_eq!(out.number_of_holes, 1);
        assert_eq!(out.number_of_regions, 1);
        // The hole seed sits in the unsegmented interior, so plague
        // (triangle.c:6714) erases both triangles and, with them,
        // every subsegment whose both abutting triangles are dying
        // (triangle.c:6747-6751): a square with a centered hole
        // swallows its whole PSLG, and writepoly has no subseg left
        // to marshal.
        assert!(out.segments.is_empty());
    }

    #[test]
    fn output_runs_the_edges_and_neighbors_gates() {
        // triangle_api.c:415-421: the four-point square triangulates
        // into two triangles with five edges; writeedges visits every
        // edge once (hull edges marked, the diagonal not) and
        // writeneighbors records -1 for outer space and the two
        // element numbers across the diagonal.
        let mut opts = Options::new();
        opts.neighbors();
        let input = MeshInput {
            points: vec![
                Point2 { x: 0.0, y: 0.0 },
                Point2 { x: 1.0, y: 0.0 },
                Point2 { x: 1.0, y: 1.0 },
                Point2 { x: 0.0, y: 1.0 },
            ],
            ..MeshInput::default()
        };
        let mesh = triangulate(opts, &input).expect("square create");
        let out = mesh.output(true, true).expect("output");
        assert_eq!(out.edges.len(), 5);
        let mut pairs: Vec<[i32; 2]> = out
            .edges
            .iter()
            .map(|p| [p[0].min(p[1]), p[0].max(p[1])])
            .collect();
        pairs.sort();
        // the four hull edges plus one diagonal (the cocircular
        // square leaves the diagonal choice to the triangulation)
        for hull in [[1, 2], [2, 3], [3, 4], [1, 4]] {
            assert!(pairs.contains(&hull));
        }
        assert!(pairs.contains(&[1, 3]) || pairs.contains(&[2, 4]));
        let mut markers = out.edge_markers.clone();
        markers.sort();
        assert_eq!(markers, vec![0, 1, 1, 1, 1]);
        let mut neighbors = out.neighbors.clone();
        neighbors.sort();
        assert_eq!(neighbors, vec![-1, -1, -1, -1, 1, 2]);
    }

    #[test]
    fn output_without_bounds_writes_no_markers_and_keeps_the_numbers() {
        // -B closes writenodes' marker list (triangle.c:7818-7821),
        // so the restore gate (triangle_api.c:423-425, a non-NULL
        // marker list) never fires: the marks keep the vertex
        // numbers writenodes wrote.
        let mut opts = Options::new();
        opts.no_bounds();
        let mesh = triangulate(opts, &triangle_input()).expect("create");
        let out = mesh.output(false, false).expect("output");
        assert!(out.point_markers.is_empty());
        let core = mesh.core.borrow();
        let mut t = core.vertices.traversal();
        let v1 = core.vertextraverse(&mut t).expect("vx");
        let v2 = core.vertextraverse(&mut t).expect("vx");
        let v3 = core.vertextraverse(&mut t).expect("vx");
        assert_eq!(core.vx_mark(v1), 1);
        assert_eq!(core.vx_mark(v2), 2);
        assert_eq!(core.vx_mark(v3), 3);
    }
}
