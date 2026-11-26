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
//! Stage-1 scaffolding: the entry points own their options validation,
//! input validation and pool-geometry construction, and the ones whose
//! core modules have not landed yet return the funnel-marked
//! InternalStatus::NotImplemented (each site carries a SCAFFOLDING
//! comment naming the C function it awaits). The constructor paths
//! therefore fail today; the parities that DO exist (validation,
//! geometry, memory accounting) are pinned by tests.

// The Mesh struct lands ahead of the core methods that read its
// behavior (every stage-1 core module takes a &Behavior); the blanket
// silences that ahead-of-use dead_code and is removed with them.
#![allow(dead_code)]

use crate::error::{Error, InternalStatus};
use crate::input::MeshInput;
use crate::input::Point2;
use crate::options::{Behavior, Options};
use crate::pool::WordPool;

/// The output half of the C triangleio surface (ADR 0004 section 5.2),
/// produced by Mesh::output (triangle_mesh_copy) once the array
/// writers land. Owned values, no libc types.
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
    /// C numberofholes / numberofregions (m->holes / m->regions; the
    /// lists themselves are input passthrough in C and stay empty
    /// until the output path lands).
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
/// C's struct quality (triangle.h:568-577), filled by Mesh::quality
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
/// Stage-1 note: no public constructor succeeds yet (triangulate /
/// load_mesh await the delaunay/reconstruct modules); the surface is
/// frozen now so the stage-1 core tickets fill it without churn.
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
}

/// triangle_mesh_create (triangle_api.c:146). The C sequence -
/// transfernodes, delaunay, formskeleton, carveholes, enforcequality,
/// highorder - belongs to stage-1 core modules that have not landed
/// yet; the shell owns its opening (options and input validation, the
/// pool geometry).
///
/// SCAFFOLDING: awaits transfernodes/delaunay/construct (ioarrays.rs,
/// delaunay.rs, construct.rs, refine.rs); returns the funnel-marked
/// InternalStatus::NotImplemented today.
pub fn triangulate(options: Options, input: &MeshInput) -> Result<Mesh, Error> {
    let behavior = options.validate()?;
    input.check(behavior.firstnumber)?;
    // -A with a PSLG earns the extra element attribute (the
    // regionattrib slot of initializetrisubpools).
    let region_extra = usize::from(behavior.regionattrib);
    let _mesh = Mesh::construct(behavior, input, region_extra)?;
    Err(Error::from(InternalStatus::NotImplemented))
}

impl Mesh {
    /// triangle_mesh_load (triangle_api.c:190). Calling it implies the
    /// refine path in C's API; the same opening validation and pool
    /// geometry, the reconstruct body awaits stage-1.
    ///
    /// SCAFFOLDING: awaits reconstruct (construct.rs); returns the
    /// funnel-marked InternalStatus::NotImplemented today.
    pub fn load_mesh(options: Options, input: &MeshInput) -> Result<Mesh, Error> {
        let behavior = { options }.validate()?;
        input.check(behavior.firstnumber)?;
        let region_extra = usize::from(behavior.regionattrib);
        let _mesh = Mesh::construct(behavior, input, region_extra)?;
        Err(Error::from(InternalStatus::NotImplemented))
    }

    /// triangle_mesh_refine (triangle_api.c:231): enforcequality over
    /// the loaded mesh.
    ///
    /// SCAFFOLDING: awaits enforcequality (refine.rs); returns the
    /// funnel-marked InternalStatus::NotImplemented today.
    pub fn refine(&mut self) -> Result<(), Error> {
        Err(Error::from(InternalStatus::NotImplemented))
    }

    /// triangle_mesh_copy (triangle_api.c:245): the marshalled output
    /// (writenodes, writeelements, writepoly, writeedges,
    /// writeneighbors and the pointmarker restore step).
    ///
    /// SCAFFOLDING: awaits the ioarray writers (ioarrays.rs) and
    /// triangle_restore_pointmarkers' vertex traversal; returns the
    /// funnel-marked InternalStatus::NotImplemented today.
    pub fn output(&self, edges: bool, neighbors: bool) -> Result<MeshOutput, Error> {
        let _ = (edges, neighbors);
        Err(Error::from(InternalStatus::NotImplemented))
    }

    /// triangle_mesh_statistics (triangle_api.c:101). The mirror of
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
            // SCAFFOLDING: C's writer of m->edges is
            //   (3 * triangles.items + hullsize) / 2, computed in the
            //   triangle_mesh_create/load_mesh/refine entry points
            //   (triangle_api.c:252-253, 326-327, 354-355), which
            //   await the ioarrays/delaunay/construct/refine phase
            //   entry points; until a landed entry point runs, C's
            //   uninitialized struct field is zeroed here.
            edges: 0,
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

    /// triangle_mesh_quality (triangle_api.c:94): the failure-free
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

    /// triangle_check_mesh (triangle_api.c:127): the failure count of
    /// checkmesh (C semantics: a count, not an error). Wired to the
    /// landed 1:1 transliteration; the noexact save/restore runs
    /// exactly like C's inside the call.
    pub fn check_mesh(&self) -> usize {
        let mut core = self.core.borrow_mut();
        let mut behavior = self.behavior.borrow_mut();
        crate::quality::checkmesh(&mut core, &mut behavior) as usize
    }

    /// triangle_check_delaunay (triangle_api.c:134): the failure count
    /// of checkdelaunay. Wired to the landed 1:1 transliteration.
    pub fn check_delaunay(&self) -> usize {
        let mut core = self.core.borrow_mut();
        let mut behavior = self.behavior.borrow_mut();
        crate::quality::checkdelaunay(&mut core, &mut behavior) as usize
    }

    /// triangle_memory (triangle_api.c:113): the mirror of C's
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
    fn triangulate_runs_validation_then_not_implemented() {
        let err = triangulate(Options::new(), &triangle_input())
            .err()
            .expect("triangulate must fail in stage 1");
        assert!(
            matches!(err, Error::Internal(ref m) if m.contains("not implemented")),
            "expected the NotImplemented scaffolding, got {}",
            err
        );
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

    #[test]
    fn refine_output_are_not_implemented() {
        let b = Options::new().validate().expect("validate");
        let mut m = scaffold(b, &empty_input());
        assert!(m.refine().is_err());
        assert!(m.output(false, false).is_err());
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
        // and the triangle now (edges stays SCAFFOLDING until its
        // writer lands).
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
    fn poly_behavior_reaches_the_pool_geometry() {
        // triangulate runs the pool construction before the
        // NotImplemented return: a -p input must traverse
        // initializetrisubpools without error, proving the Behavior
        // hand-off.
        let mut opts = Options::new();
        opts.pslg();
        let err = triangulate(opts, &triangle_input())
            .err()
            .expect("triangulate must fail in stage 1");
        assert!(matches!(err, Error::Internal(_)));
    }
}
