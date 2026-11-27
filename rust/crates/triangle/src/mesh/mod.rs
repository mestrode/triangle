//! Mesh-level lifecycle (the mesh/mod.rs rows of docs/function-mapping.md):
//! C struct mesh (triangle.h:476) owning the eight pools, the init geometry
//! (triangleinit, initializevertexpool, initializetrisubpools, dummyinit),
//! and the constructors maketriangle / makesubseg. All storage access goes
//! through the mesh/mesh/slots.rs chokepoint; only the pool construction happens
//! here (the fields are pub(crate) for this module, and nothing outside
//! mesh/mesh/slots.rs calls the pools' word/alloc/dealloc - review-enforced).
//!
//! R1 accessor-invariant rung (ADR 0002 section 3): the init functions
//! here are the only builders of the row-shape role tables (invariant
//! c), and each debug_asserts that its computed shape lands inside the
//! row it sizes; the constructors reserve the D3 sentinel rows; the
//! traversal mirrors skip dead rows (d) and inherit the reserved-row
//! exclusion from the pool seam. Debug builds only; release compiles
//! no check.
//!
//! Landed in this PR: Mesh (pools, row shapes, counters),
//! Mesh::new (the triangleinit + poolzero + pool-init sequence collapsed
//! into construction, D11), initializevertexpool, initializetrisubpools,
//! dummyinit, maketriangle, makesubseg.
//!
//! NOT landed (their C rows arrive with the modules that use them; the
//! chokepoint primitives they reduce to are already in mesh/mesh/slots.rs):
//! triangledealloc, subsegdealloc, vertexdealloc, triangledeinit
//! (triangle.c:1196-1408). When triangledeinit lands, its acute free
//! (trifree(m->acute_mem) and the NULL store, triangle.c:1395-1396)
//! dissolves into the Drop of the owned acute_mem field (D23, the
//! acutepool row of docs/deviation-register.md).
//! vertextraverse (triangle.c:1276) landed with
//! the delaunay part-1 PR (the divconqdelaunay consumer), triangletraverse
//! and getvertex with their consumers, and badsubsegdealloc /
//! badsubsegtraverse (triangle.c:1298-1304, 1316-1327) with algorithm/algorithm/refine.rs
//! (the splitencsegs consumer).
//!
//! The behavior fields the init functions read arrive on the normalized
//! Behavior (options.rs): Mesh::new takes &Behavior exactly where C's
//! construction path would read its context-owned behavior struct. The
//! blanket below silences ahead-of-use dead_code until the consuming
//! modules land.
#![allow(dead_code)]

use crate::mesh::handles::{encode, encode_subseg, Osub, Otri, DUMMY_SUB, DUMMY_TRI, NULL_ROW};
use crate::mesh::pool::{Oom, WordPool};
use crate::mesh::slots::{HalfWord, TriShape, VertexShape};
use crate::options::Behavior;

// The C per-block constants (triangle_config.h:120-131). Their values are
// part of the contract: block geometry feeds oracle_address_order (ADR 0003
// section 5) and the initial capacity hints.
const TRI_PER_BLOCK: i32 = 4092;
const SUBSEG_PER_BLOCK: i32 = 508;
const VERTEX_PER_BLOCK: i32 = 4092;
const VIRUS_PER_BLOCK: i32 = 1020;
const BAD_SUBSEG_PER_BLOCK: i32 = 252;
const BAD_TRI_PER_BLOCK: i32 = 4092;
const FLIP_STACKER_PER_BLOCK: i32 = 252;
const SPLAY_NODE_PER_BLOCK: i32 = 508;

/// C: struct mesh (triangle.h:476). Only the fields the landed functions
/// need exist at this stage; the rest of the C field set (bounds, counters,
/// queues, the quality scratch) lands with the modules that own their rows.
pub(crate) struct Mesh {
    /// The eight mesh pools (C field names; all one uniform WordPool type,
    /// ADR 0003 section 1).
    pub(crate) triangles: WordPool,
    pub(crate) subsegs: WordPool,
    pub(crate) vertices: WordPool,
    pub(crate) viri: WordPool,
    pub(crate) badsubsegs: WordPool,
    pub(crate) badtriangles: WordPool,
    pub(crate) flipstackers: WordPool,
    pub(crate) splaynodes: WordPool,

    /// Triangle row shape (computed by initializetrisubpools).
    pub(crate) tri_shape: TriShape,
    /// Vertex row shape (computed by initializevertexpool).
    pub(crate) vertex_shape: VertexShape,

    /// Number of input vertices (C m->invertices; sizes the first blocks).
    pub(crate) invertices: i32,
    /// Dimension (ought to be 2; C m->mesh_dim sizes the vertex row).
    pub(crate) mesh_dim: i32,
    /// Attributes per vertex (C m->nextras; sizes the vertex row).
    pub(crate) nextras: i32,
    /// Attributes per triangle (C m->eextras; sizes the triangle row and
    /// bounds tri_attribute).
    pub(crate) eextras: i32,
    /// Input bounds (C m->xmin, xmax, ymin, ymax; REAL, triangle.h:513):
    /// written by transfernodes (io/arrays.rs) and
    /// read by boundingbox (algorithm/algorithm/delaunay.rs part 1) and sweeplinedelaunay
    /// (part 2). Zero superset of C's uninitialized read-before-write
    /// (D11).
    pub(crate) xmin: f64,
    pub(crate) xmax: f64,
    pub(crate) ymin: f64,
    pub(crate) ymax: f64,
    /// C m->xminextreme (triangle.h:514): 10 * xmin - 9 * xmax, written
    /// by transfernodes (triangle.c:7754, io/arrays.rs)
    /// and read by sweeplinedelaunay as the circle-event x key
    /// (triangle.c:5382/5397). Zero superset of C's uninitialized
    /// read-before-write (D11).
    pub(crate) xminextreme: f64,
    /// C m->undeads (triangle.h:520, int to i32): duplicate vertices
    /// moved to the graveyard (type UNDEADVERTEX); counted by
    /// divconqdelaunay (algorithm/algorithm/delaunay.rs) and later by sweeplinedelaunay and
    /// the quality rows. Never read before the first write; the
    /// #[allow(dead_code)] mirrors the hullsize/cyclefail convention (the
    /// every-build readers land with later rows).
    #[allow(dead_code)]
    pub(crate) undeads: i32,
    /// C m->inelements (triangle.h:516): number of input triangles (the
    /// .ele count). Written by reconstruct (triangle.c:5498); read by the
    /// statistics rows that land with the later slices. Never read
    /// before the first write in the pipeline; the #[allow(dead_code)]
    /// mirrors the hullsize/cyclefail convention (the every-build
    /// readers land with later rows).
    #[allow(dead_code)]
    pub(crate) inelements: i32,
    /// C m->insegments (triangle.h:517): number of input PSLG segments
    /// (the .poly count). Written by reconstruct (triangle.c:5519) and
    /// formskeleton (triangle.c:6562); read by formskeleton's insertion
    /// loop and, later, the refine/enforcequality rows.
    pub(crate) insegments: i32,
    /// C m->holes (triangle.h:518): number of input holes (the
    /// triangleio numberofholes count). Stored by the create and load
    /// orchestration (triangle_api.c:229/238, 316/322); read by the
    /// output path (Mesh::output, triangle_api.c:404-405).
    pub(crate) holes: i32,
    /// C m->regions (triangle.h:519): number of input regions (the
    /// triangleio numberofregions count); same writers and reader as
    /// holes.
    pub(crate) regions: i32,
    /// C m->scoutsteps (triangle.h:532, long to i64): vertices reached so
    /// far inserting the current segment. Zeroed by triangleinit
    /// (triangle.c:1499, mirrored in Mesh::new) and by each
    /// insertsegment (triangle.c:6428), guarded by scoutsegment's
    /// going-in-circles check.
    pub(crate) scoutsteps: i64,
    /// C m->steinerleft (triangle.h:526, int): the running Steiner-point
    /// budget b->steiner. C sets it twice, in triangle_context_create
    /// (triangle_api.c:35) and triangle_mesh_load (triangle_api.c:208);
    /// both entry paths set it after triangleinit and before any core
    /// call can run, so the port takes the same value in Mesh::new
    /// (the deterministic superset, like hullsize). Read and decremented
    /// by segmentintersection and conformingedge in this slice.
    pub(crate) steinerleft: i32,

    /// Statistics counters, C long fields of struct mesh (triangle.h:538-546:
    /// incirclecount, counterclockcount, orient3dcount, hyperbolacount,
    /// circumcentercount, circletopcount - i64 here, the pinned 64-bit
    /// port of C long). Iterated by the predicates (predicates.rs, one
    /// increment per test, plus the findcircumcenter decrement mirror of
    /// its internal counterclockwise call) and by the sweepline rows
    /// (algorithm/algorithm/delaunay.rs part 2: rightofhyperbola counts every hyperbola test,
    /// circletop every circle-top test, exactly at the C increment
    /// sites); read by the future statistics rows.
    pub(crate) incirclecount: i64,
    pub(crate) counterclockcount: i64,
    pub(crate) orient3dcount: i64,
    pub(crate) hyperbolacount: i64,
    pub(crate) circumcentercount: i64,
    pub(crate) circletopcount: i64,

    /// C m->randomseed (triangle.c:220, unsigned long: u64 on the LP64
    /// port): the LCG state of randomnation (mesh/mesh/util.rs). Deviation D10: an
    /// owned per-run field, never a thread-local; seeded 1 by triangleinit
    /// (triangle.c:1502), mirrored by Mesh::new.
    pub(crate) randomseed: u64,
    /// C m->samples (triangle.h:536): number of random samples for point
    /// location; starts 1, grows in locate() (triangle.c:2469-2472).
    pub(crate) samples: i64,
    /// C m->checksegments (triangle.h:532): are there segments in the
    /// triangulation yet? (C int 0/1 flag; bool here.)
    pub(crate) checksegments: bool,
    /// C m->checkquality (triangle.h:533): has the quality triangulation
    /// stage begun? Gates the flip-stack bookkeeping of insertvertex.
    pub(crate) checkquality: bool,
    /// C m->cyclefail (triangle.h:534): did vertex insertion start
    /// flipping edges in circles? Set by insertvertex (triangle.c:3165),
    /// read by the statistics rows when they land.
    #[allow(dead_code)]
    pub(crate) cyclefail: bool,
    /// C m->hullsize (triangle.h:525): number of edges in the convex
    /// hull; insertvertex increments it on boundary-edge splits
    /// (triangle.c:3012). Read by the statistics rows when they land.
    #[allow(dead_code)]
    pub(crate) hullsize: i64,
    /// C m->edges (triangle.h:521): the running edge count the
    /// statistics row reads; the writers are the triangle_api.c entry
    /// points (252-253 create, 326-327 load, 354-355 refine). The
    /// refine writer is live (api.rs Mesh::refine); create and load
    /// await the ioarrays phase, so the field stays 0 on their meshes.
    pub(crate) edges: i64,
    /// C m->lastflip (triangle.h:509): the head of the stack of recently
    /// flipped triangles (struct flipstacker *; a flipstackers row id
    /// here, the null id for the C NULL pointer).
    pub(crate) lastflip: u32,
    /// C m->infvertex1, infvertex2, infvertex3 (triangle.h:547): the three
    /// vertices of the incremental/sweepline bounding box, treated as if
    /// infinitely distant by insertvertex; NULL (the null row id) until
    /// boundingbox (algorithm/algorithm/delaunay.rs part 1) sets them to the reserved
    /// out-of-pool INFVERTEX ids (D12, mesh/mesh/slots.rs); NULL again after
    /// removebox.
    pub(crate) infvertex1: u32,
    pub(crate) infvertex2: u32,
    pub(crate) infvertex3: u32,
    /// Deviation D12 (docs/deviation-register.md): C bounds the box with
    /// three trimalloc'd vertices outside the vertex pool
    /// (triangle.c:4634-4650); the port's reserved out-of-pool INFVERTEX
    /// ids (mesh/mesh/slots.rs) carry the two coordinates of each infinite vertex in
    /// this owned store, read and written only through the
    /// vx_real/vx_set_real chokepoint; removebox zeroes it like C's
    /// trifree (triangle.c:4741-4743).
    pub(crate) infvertex_store: [[u64; 2]; 3],
    /// C m->recenttri (triangle.h:564): the most recently encountered
    /// triangle of location (struct otri in C, where tri can be NULL;
    /// Option here).
    pub(crate) recenttri: Option<Otri>,
    /// C m->acute_mem (triangle.h:496, struct acutepool*): the aCute
    /// scratch. Deviation D23 (docs/deviation-register.md): C trimallocs
    /// the struct (acute.c:22-43) and hangs it off the mesh inside
    /// triangleinit (NULL store at triangle.c:1507) with the real
    /// acutepool_init(20) at the enforcequality site (triangle.c:7538);
    /// the port owns the eight typed Vec<f64> buffers directly
    /// (crate::acute::AcutePool) behind the feature, eagerly constructed
    /// at this Mesh::new site with exactly the acutepool_init(20)
    /// capacities of acute.c:26-40. The two-step free of triangledeinit
    /// (trifree(m->acute_mem), triangle.c:1395, and the NULL store,
    /// triangle.c:1396) dissolves into Drop of the owned field.
    #[cfg(feature = "acute")]
    pub(crate) acute_mem: crate::acute::AcutePool,

    // -----------------------------------------------------------------
    // The bad-triangle queues (C triangle.h:502-505, the mesh fields the
    // enqueuebadtriang/dequeuebadtriang rows of algorithm/algorithm/quality.rs drive: 4096
    // queues keyed by the square of the shortest edge, front/tail links
    // into the badtriangles pool, the nextnonemptyq chain and its head).
    // D4: the C badtriang pointers mirror as u32 pool row ids, the null
    // row id 0 marking queue ends; firstnonemptyq -1 marks "all empty",
    // 4096 the sentinel value dequeue treats as empty.
    // ----------------------------------------------------------------
    /// C m->queuefront[4096]: the first badtriang row of each queue (the
    /// null row id 0 for an empty queue).
    pub(crate) queuefront: [u32; 4096],
    /// C m->queuetail[4096]: the last badtriang row of each queue.
    /// C leaves this uninitialized until an enqueue writes it, and the
    /// queue discipline never reads it before that first write; the port
    /// pins the null row id 0 as the deterministic superset.
    pub(crate) queuetail: [u32; 4096],
    /// C m->nextnonemptyq[4096]: the next lower-priority nonempty queue
    /// after each nonempty queue. Same unread-before-write discipline as
    /// `queuetail`; the port pins -1 as the deterministic superset.
    pub(crate) nextnonemptyq: [i32; 4096],
    /// C m->firstnonemptyq: the highest-priority nonempty queue, -1 when
    /// all queues are empty.
    pub(crate) firstnonemptyq: i32,
}

impl Mesh {
    /// Construct the mesh: C triangleinit (triangle.c:1482-1507: poolzero
    /// of all eight pools and the counter defaults) followed by the pool
    /// geometry initialization. Each pool starts where the poolzero'd C
    /// mesh would be (D11: poolzero into construction, pooldeinit into
    /// Drop); the three structural pools are then initialized exactly like
    /// C's initializevertexpool (triangle.c:1100) and
    /// initializetrisubpools (triangle.c:1136), which ends in dummyinit.
    ///
    /// The behavior-derived fields (poly, order, usesegments, vararea,
    /// regionattrib, neighbors) arrive on the normalized &Behavior, the
    /// field-mirror of C's struct behavior (options.rs; its source is
    /// Options::validate).
    pub(crate) fn new(
        b: &Behavior,
        invertices: i32,
        mesh_dim: i32,
        nextras: i32,
        eextras: i32,
    ) -> Result<Mesh, Oom> {
        // C triangleinit ends with tri_exactinit (triangle.c:1504): the
        // exact-arithmetic constants are computed before any predicate can
        // run. Mirrored here (idempotent via its OnceLock guard).
        crate::predicates::exact_init();
        let mut m = Mesh {
            // poolzero state for the three structural pools (dormant until
            // the init calls below); the small pools take their C geometry
            // eagerly (C poolinits them lazily at their first use sites,
            // triangle.c:5234, 6988, 7533-7566 - timing that no allocation,
            // id, reuse, or traversal state can observe, and the first
            // block equals every block for all five, ADR 0003 section 4)
            triangles: WordPool::new(0, 0, 0, false)?,
            subsegs: WordPool::new(0, 0, 0, false)?,
            vertices: WordPool::new(0, 0, 0, false)?,
            viri: WordPool::new(8, VIRUS_PER_BLOCK as u32, VIRUS_PER_BLOCK as u32, false)?,
            badsubsegs: WordPool::new(
                24,
                BAD_SUBSEG_PER_BLOCK as u32,
                BAD_SUBSEG_PER_BLOCK as u32,
                false,
            )?,
            badtriangles: WordPool::new(
                48,
                BAD_TRI_PER_BLOCK as u32,
                BAD_TRI_PER_BLOCK as u32,
                false,
            )?,
            flipstackers: WordPool::new(
                16,
                FLIP_STACKER_PER_BLOCK as u32,
                FLIP_STACKER_PER_BLOCK as u32,
                false,
            )?,
            splaynodes: WordPool::new(
                40,
                SPLAY_NODE_PER_BLOCK as u32,
                SPLAY_NODE_PER_BLOCK as u32,
                false,
            )?,
            tri_shape: TriShape {
                stride: 0,
                usesegments: false,
                highorderindex: 0,
                elemattribindex: 0,
                areaboundindex: 0,
                numberindex: None,
            },
            vertex_shape: VertexShape {
                stride: 0,
                mark: HalfWord {
                    word: 0,
                    high: false,
                },
                vtype: HalfWord {
                    word: 0,
                    high: false,
                },
                v2tri: None,
            },
            invertices,
            mesh_dim,
            nextras,
            eextras,
            xmin: 0.0,
            xmax: 0.0,
            ymin: 0.0,
            ymax: 0.0,
            xminextreme: 0.0,
            undeads: 0,
            incirclecount: 0,
            counterclockcount: 0,
            orient3dcount: 0,
            hyperbolacount: 0,
            circumcentercount: 0,
            circletopcount: 0,
            // C triangleinit (triangle.c:1493-1502): recenttri.tri NULL,
            // samples 1, checksegments 0, checkquality 0, cyclefail 0,
            // randomseed 1, scoutsteps 0 (triangle.c:1499). hullsize and
            // lastflip are not zeroed by C's triangleinit (uninitialized
            // struct memory until their owning algorithms run); the
            // port picks the deterministic zero superset of that
            // undefined state. inelements/insegments join that zero
            // superset; holes/regions join it too (C stores them in
            // the create/load orchestration, triangle_api.c:229/238
            // and 316/322); steinerleft takes b->steiner at
            // construction
            // (C triangle_context_create:35 / triangle_mesh_load:208,
            // both before any core call runs).
            randomseed: 1,
            samples: 1,
            checksegments: false,
            checkquality: false,
            cyclefail: false,
            hullsize: 0,
            edges: 0,
            lastflip: NULL_ROW,
            inelements: 0,
            insegments: 0,
            holes: 0,
            regions: 0,
            scoutsteps: 0,
            steinerleft: b.steiner,
            infvertex1: NULL_ROW,
            infvertex2: NULL_ROW,
            infvertex3: NULL_ROW,
            infvertex_store: [[0; 2]; 3],
            recenttri: None,
            // The bad-triangle queue state (C triangle.h:502-505). C
            // initializes queuefront[] to NULL and firstnonemptyq to -1
            // inside enforcequality (triangle.c:7559-7561) right before
            // tallyfaces fills the queues; queuetail[] and nextnonemptyq[]
            // are uninitialized struct memory there, never read before
            // their first enqueue writes them (the queue discipline of
            // algorithm/algorithm/quality.rs). The port pins the whole state at construction:
            // the enforcequality-init values for the two C-initialized
            // members and the null superset for the other two, following
            // the established lazy-pool-init convention (ADR 0003
            // section 4). algorithm/algorithm/refine.rs's enforcequality re-runs the C init
            // loop verbatim over these fields (the queuefront sweep and
            // firstnonemptyq reset of triangle.c:7559-7561).
            queuefront: [NULL_ROW; 4096],
            queuetail: [NULL_ROW; 4096],
            nextnonemptyq: [-1; 4096],
            firstnonemptyq: -1,
            // The aCute scratch pool (C m->acute_mem, triangle.h:496):
            // NULL inside triangleinit (triangle.c:1507), really
            // allocated by acutepool_init(20) at the enforcequality site
            // (triangle.c:7538). D23: the port constructs the owned
            // AcutePool eagerly here, with exactly C's initial capacities
            // (acute.c:26-40 at n = 20: 500 for initialpoly and the three
            // point buffers, 2n for the petal buffers, 2n*20+40 for
            // wedges).
            #[cfg(feature = "acute")]
            acute_mem: crate::acute::AcutePool::new(20),
        };
        m.init_vertex_pool(b.poly)?;
        m.init_tri_and_subseg_pools(
            b.order,
            b.usesegments,
            b.vararea,
            b.regionattrib,
            b.neighbors,
        )?;
        Ok(m)
    }

    /// C: initializevertexpool (triangle.c:1100-1124). Computes the vertex
    /// row shape (the boundary-marker/vertex-type int overlays and, with
    /// -p, the vertex2tri word) and initializes the vertex pool with C's
    /// first-block hint. Int and word indices are computed on the pinned
    /// 64-bit platform (sizeof(REAL) = 8, sizeof(int) = 4) exactly like C:
    /// mark_int is vertexmarkindex in int units, vertexsize in bytes.
    ///
    /// Invariant (ADR 0002 section 3 c): the computed overlays and the
    /// vertex2tri word are debug_asserted inside the stride this function
    /// computes (the role table is built only here).
    pub(crate) fn init_vertex_pool(&mut self, poly: bool) -> Result<(), Oom> {
        let mark_int = ((self.mesh_dim + self.nextras) * 8 + 3) / 4;
        let mut vertexsize = (mark_int + 2) * 4;
        let v2tri = if poly {
            let idx = (vertexsize + 7) / 8;
            vertexsize = (idx + 1) * 8;
            Some(idx as usize)
        } else {
            None
        };
        self.vertex_shape = VertexShape {
            stride: ((vertexsize + 7) / 8) as usize,
            // vertexmarkindex in int units, split into word + half; the C
            // byte arithmetic keeps it even, so mark and vtype share the
            // word exactly like the crossed-out C int casts do
            mark: HalfWord {
                word: (mark_int / 2) as usize,
                high: mark_int % 2 == 1,
            },
            vtype: HalfWord {
                word: ((mark_int + 1) / 2) as usize,
                high: (mark_int + 1) % 2 == 1,
            },
            v2tri,
        };
        // Slot-role invariant (ADR 0002 section 3 c): the role table this
        // init computes must land inside the row the same formulas size -
        // the debug_asserts pin the init formulas themselves.
        debug_assert!(
            self.vertex_shape.mark.word < self.vertex_shape.stride,
            "vertex marker overlay outside the row (slot-role invariant)"
        );
        debug_assert!(
            self.vertex_shape.vtype.word < self.vertex_shape.stride,
            "vertex type overlay outside the row (slot-role invariant)"
        );
        if let Some(slot) = self.vertex_shape.v2tri {
            debug_assert!(
                slot < self.vertex_shape.stride,
                "vertex2tri word outside the row (slot-role invariant)"
            );
        }
        let first = if self.invertices > VERTEX_PER_BLOCK {
            self.invertices
        } else {
            VERTEX_PER_BLOCK
        };
        self.vertices = WordPool::new(
            vertexsize as u32,
            VERTEX_PER_BLOCK as u32,
            first as u32,
            false,
        )?;
        Ok(())
    }

    /// C: initializetrisubpools (triangle.c:1136-1185). Computes the
    /// triangle row shape (highorderindex, elemattribindex, areaboundindex,
    /// the -n numbering overlay; byte formulas expressed in whole words,
    /// exactly C's numbers on the pinned platform), initializes the
    /// triangle and (with -p) subseg pools with C's first-block hints, and
    /// creates the sentinels via dummyinit.
    ///
    /// Invariant (ADR 0002 section 3 c): the computed role indices are
    /// debug_asserted inside the stride this function computes (the role
    /// table is built only here).
    pub(crate) fn init_tri_and_subseg_pools(
        &mut self,
        order: i32,
        usesegments: bool,
        vararea: bool,
        regionattrib: bool,
        neighbors: bool,
    ) -> Result<(), Oom> {
        // C: highorderindex = 6 + b->usesegments * 3
        let highorderindex = 6 + if usesegments { 3 } else { 0 };
        // C: trisize = ((order + 1) * (order + 2) / 2 + highorderindex - 3)
        //               * sizeof(triangle), bytes
        let mut trisize = ((order + 1) * (order + 2) / 2 + (highorderindex - 3)) * 8;
        // C: elemattribindex, measured in REALs (= words here), rounded up
        let elemattribindex = (trisize + 7) / 8;
        // C: areaboundindex = elemattribindex + eextras + regionattrib
        let areaboundindex = elemattribindex + self.eextras + if regionattrib { 1 } else { 0 };
        if vararea {
            // C: trisize = (areaboundindex + 1) * sizeof(REAL)
            trisize = (areaboundindex + 1) * 8;
        } else if self.eextras + if regionattrib { 1 } else { 0 } > 0 {
            // C: trisize = areaboundindex * sizeof(REAL)
            trisize = areaboundindex * 8;
        }
        // C: if (b->neighbors && trisize < 6 * sizeof(triangle) +
        // sizeof(int)) - room for the numbering int in word 6's low half
        if neighbors && trisize < 6 * 8 + 4 {
            trisize = 6 * 8 + 4;
        }
        self.tri_shape = TriShape {
            stride: ((trisize + 7) / 8) as usize,
            usesegments,
            highorderindex: highorderindex as usize,
            elemattribindex: elemattribindex as usize,
            areaboundindex: areaboundindex as usize,
            // the numbering int: C writeneighbors stores it in
            // *(int *)(tri + 6) (triangle.c:8090), bounded by the 52-byte
            // minimum row above; the subseg-slot duality with -p is C's
            // own (the source carries a TODO about it)
            numberindex: if neighbors {
                Some(HalfWord {
                    word: 6,
                    high: false,
                })
            } else {
                None
            },
        };
        // Slot-role invariant (ADR 0002 section 3 c): the role indices
        // must land inside the row they size - the debug_asserts pin the
        // init formulas. The area-bound word only exists under vararea
        // (C's own trisize formula), so its check is gated the same way.
        debug_assert!(
            self.tri_shape.highorderindex <= self.tri_shape.elemattribindex,
            "highorderindex past the first attribute word (slot-role invariant)"
        );
        debug_assert!(
            self.tri_shape.elemattribindex + self.eextras as usize <= self.tri_shape.stride,
            "element attributes past the row end (slot-role invariant)"
        );
        if vararea {
            debug_assert!(
                self.tri_shape.areaboundindex < self.tri_shape.stride,
                "area-bound word outside the row (slot-role invariant)"
            );
        }
        if let Some(number) = self.tri_shape.numberindex {
            debug_assert!(
                number.word < self.tri_shape.stride,
                "-n numbering overlay outside the row (slot-role invariant)"
            );
        }
        // C: first block max(2 * invertices - 2, TRIPERBLOCK)
        let first = {
            let two_in = 2 * self.invertices - 2;
            if two_in > TRI_PER_BLOCK {
                two_in
            } else {
                TRI_PER_BLOCK
            }
        };
        self.triangles = WordPool::new(trisize as u32, TRI_PER_BLOCK as u32, first as u32, true)?;
        if usesegments {
            // C: 8 pointers + one boundary marker = 68 bytes, first block
            // SUBSEGPERBLOCK
            self.subsegs = WordPool::new(
                8 * 8 + 4,
                SUBSEG_PER_BLOCK as u32,
                SUBSEG_PER_BLOCK as u32,
                true,
            )?;
        }
        self.init_sentinel_rows();
        Ok(())
    }

    /// C: dummyinit (triangle.c:1029-1078). The two sentinel allocations
    /// become the reserved row id 1 of each pool (D3: initialized and
    /// bonded exactly like C's hand-aligned out-of-pool allocations, minus
    /// the alignment no port needs). Calling this twice is C-visible: the
    /// C function relies on being called once at init; the port
    /// idempotently rewrites the same words.
    ///
    /// Invariant (ADR 0002 section 3 a, D3): every stored word here is a
    /// legal decode target (the sentinels self-bond; every dissolve and
    /// sdissolve site later stores exactly these words).
    pub(crate) fn init_sentinel_rows(&mut self) {
        // encode of the sentinel row, orientation 0: the exact value the C
        // macros store for m->dummytri (the aligned raw pointer)
        let dummy_tri = encode(Otri {
            tri: DUMMY_TRI,
            orient: 0,
        });
        // outer space: three adjoining triangles = itself, three NULL
        // vertices (C dummyinit, triangle.c:1042-1054)
        for i in 0..3 {
            self.set_tri_neighbor_word(DUMMY_TRI, i, dummy_tri);
        }
        for i in 0..3 {
            self.set_tri_vertex_word(DUMMY_TRI, i, 0);
        }
        if self.tri_shape.usesegments {
            let dummy_sub = encode_subseg(Osub {
                ss: DUMMY_SUB,
                orient: 0,
            });
            // the omnipresent subsegment (C triangle.c:1056-1070): two
            // adjoining subsegs = itself, four NULL vertices, two
            // adjoining triangles = outer space, boundary marker 0
            for i in 0..2 {
                self.set_sub_neighbor_word(DUMMY_SUB, i, dummy_sub);
            }
            for i in 0..4 {
                self.set_sub_vertex_word(DUMMY_SUB, i, 0);
            }
            for i in 0..2 {
                self.set_sub_tri_word(DUMMY_SUB, i, dummy_tri);
            }
            self.set_sub_mark(DUMMY_SUB, 0);
            // outer space's three adjoining subsegments = the omnipresent
            // subsegment (C triangle.c:1072-1075)
            for i in 0..3 {
                self.set_tri_subseg_word(DUMMY_TRI, i, dummy_sub);
            }
        }
    }

    /// C: maketriangle (triangle.c:1414-1443): a new triangle with
    /// orientation zero, its three edges bonded to outer space (and, with
    /// -p, its three sides to the omnipresent subsegment), three NULL
    /// vertices, element attributes zeroed, areabound -1 under -a.
    ///
    /// Invariants (ADR 0002 section 3): the fresh row is legal on every
    /// decode (a: the dummy handles name the reserved sentinel rows);
    /// the NULL vertex slots are data, never decoded (b); the
    /// initialization writes stay below the stride (c).
    pub(crate) fn make_triangle(&mut self, usesegments: bool, vararea: bool) -> Result<Otri, Oom> {
        let tri = self.tri_alloc()?;
        let dummy_tri = encode(Otri {
            tri: DUMMY_TRI,
            orient: 0,
        });
        for i in 0..3 {
            self.set_tri_neighbor_word(tri, i, dummy_tri);
        }
        for i in 0..3 {
            self.set_tri_vertex_word(tri, i, 0);
        }
        if usesegments {
            let dummy_sub = encode_subseg(Osub {
                ss: DUMMY_SUB,
                orient: 0,
            });
            for i in 0..3 {
                self.set_tri_subseg_word(tri, i, dummy_sub);
            }
        }
        for i in 0..self.eextras {
            self.set_tri_attribute(tri, i as usize, 0.0);
        }
        if vararea {
            self.set_tri_area_bound(tri, -1.0);
        }
        Ok(Otri { tri, orient: 0 })
    }

    /// C: makesubseg (triangle.c:1450-1468): a new subsegment with
    /// orientation zero, both ends bonded to the omnipresent subsegment,
    /// four NULL vertices, two adjoining triangles bonded to outer space,
    /// boundary marker 0.
    ///
    /// Invariants (ADR 0002 section 3): as maketriangle (a, b, c) - the
    /// fresh row is legal on every decode, the NULL vertex slots are
    /// data, and the writes stay below the stride.
    pub(crate) fn make_subseg(&mut self) -> Result<Osub, Oom> {
        let ss = self.sub_alloc()?;
        let dummy_sub = encode_subseg(Osub {
            ss: DUMMY_SUB,
            orient: 0,
        });
        for i in 0..2 {
            self.set_sub_neighbor_word(ss, i, dummy_sub);
        }
        for i in 0..4 {
            self.set_sub_vertex_word(ss, i, 0);
        }
        let dummy_tri = encode(Otri {
            tri: DUMMY_TRI,
            orient: 0,
        });
        for i in 0..2 {
            self.set_sub_tri_word(ss, i, dummy_tri);
        }
        self.set_sub_mark(ss, 0);
        Ok(Osub { ss, orient: 0 })
    }

    /// C: vertextraverse (triangle.c:1276-1288). Walk the vertex pool in
    /// allocation order, skipping DEADVERTEX rows (undead UNDEADVERTEX
    /// rows come back like every allocated vertex); the end-of-pool NULL
    /// of C's traverse is the None here. Pool traversal itself walks dead
    /// rows (D3), so this skip loop is the whole of C's do-while.
    ///
    /// Invariants (ADR 0002 section 3): dead rows are skipped by the
    /// DEADVERTEX overlay (d); the null row is never yielded (b,
    /// debug-pinned at the pool seam); the vertex pool has no sentinel
    /// row (D3).
    pub(crate) fn traverse_vertex(&self, t: &mut crate::mesh::pool::Traversal) -> Option<u32> {
        loop {
            let newvertex = self.vertices.traverse_next(t)?;
            if self.vx_type(newvertex) != crate::mesh::slots::DEAD_VERTEX {
                return Some(newvertex);
            }
        }
    }

    /// C: triangletraverse (triangle.c:1210-1228). Walk the triangle pool
    /// in allocation order, skipping dead rows (tri[1] == NULL, the
    /// killtri pattern through the chokepoint); the end-of-pool NULL of
    /// C's traverse is the None here. Pool traversal walks dead rows
    /// (D3), so this skip loop is the whole of C's do-while.
    ///
    /// Invariants (ADR 0002 section 3): dead rows skipped (d); the
    /// reserved sentinel row is excluded by the pool walk (D3,
    /// debug-pinned); the null row is never yielded (b).
    pub(crate) fn traverse_triangle(&self, t: &mut crate::mesh::pool::Traversal) -> Option<u32> {
        loop {
            let newtri = self.triangles.traverse_next(t)?;
            if !self.is_dead_tri(newtri) {
                return Some(newtri);
            }
        }
    }

    /// C: subsegtraverse (triangle.c:1243-1261). Same walk over the
    /// subsegment pool, skipping dead rows (ss[1] == NULL, killsubseg).
    /// Invariants (ADR 0002 section 3): as triangletraverse (d, D3, b).
    pub(crate) fn traverse_subseg(&self, t: &mut crate::mesh::pool::Traversal) -> Option<u32> {
        loop {
            let newsub = self.subsegs.traverse_next(t)?;
            if !self.is_dead_subseg(newsub) {
                return Some(newsub);
            }
        }
    }

    /// C: badsubsegdealloc (triangle.c:1298-1304). The dead-row idiom:
    /// subsegorg = NULL first (triangle.c:1302) so badsubsegtraverse
    /// skips the row, then pooldealloc (triangle.c:1303); splitencsegs
    /// (algorithm/algorithm/refine.rs) is the consumer.
    ///
    /// Invariants (ADR 0002 section 3): the NULL word is the dead-row
    /// mark, never decoded (b); the push makes slot 0 pool-private (d);
    /// reuse stays exact-LIFO (e).
    pub(crate) fn dealloc_bad_subseg(&mut self, dyingsubseg: u32) {
        self.set_bad_sub_seg_org(dyingsubseg, NULL_ROW);
        self.badsubsegs.dealloc(dyingsubseg);
    }

    /// C: badsubsegtraverse (triangle.c:1316-1327). Walk the pool and
    /// skip the rows badsubsegdealloc marked (the subsegorg == NULL
    /// test of the do-while, triangle.c:1321-1326); splitencsegs'
    /// traversalinit (triangle.c:7367) starts it.
    ///
    /// Invariants (ADR 0002 section 3): the NULL mark is the skip test
    /// (b); the null row itself is never yielded (b, debug-pinned at
    /// the pool seam).
    pub(crate) fn traverse_bad_subseg(&self, t: &mut crate::mesh::pool::Traversal) -> Option<u32> {
        loop {
            let newsub = self.badsubsegs.traverse_next(t)?;
            if self.bad_subseg_org(newsub) != NULL_ROW {
                return Some(newsub);
            }
        }
    }

    /// C: getvertex (triangle.c:1343-1370). Map an input vertex number
    /// to its pool row. The C block walk (firstblock, then one chained
    /// block per itemsperblock step) becomes row-id arithmetic through
    /// block_base_id; D1 replaces the C raw-pointer result (aligned
    /// block base + itembytes * (number - current)) by the same row id
    /// in the same allocation order. Input vertices are allocated
    /// contiguously in input order (transfernodes), so number -
    /// firstnumber is the allocation index.
    ///
    /// Invariant (ADR 0002 section 3 a): the computed row id is
    /// debug_asserted inside the vertex pool's id space.
    pub(crate) fn get_vertex(&self, firstnumber: i32, number: i32) -> u32 {
        // C: getblock = m->vertices.firstblock; current = b->firstnumber.
        let mut block: u64 = 0;
        let mut current = firstnumber;

        // C: find the right block. First the first block, then one
        // chained block per step while the number lies beyond it.
        if current + self.vertices.first_block_items() as i32 <= number {
            block = 1;
            current += self.vertices.first_block_items() as i32;
            while current + self.vertices.block_items() as i32 <= number {
                block += 1;
                current += self.vertices.block_items() as i32;
            }
        }

        // C: foundvertex = aligned block start + itembytes * (number -
        // current); block_base_id is the same walk in id units.
        let vertex = self.vertices.block_base_id(block) + (number - current) as u32;
        // Range invariant (ADR 0002 section 3 a): the mapped id must name
        // a materialized vertex row (the input numbers are caller-trusted
        // in C; the debug check catches block-walk slips). Debug only.
        self.vertices.debug_assert_row_in_space(vertex);
        vertex
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::handles::Otri;
    use crate::mesh::slots::{HalfWord, TriShape};

    /// Test-assembled Behavior with exactly the fields the geometry
    /// reads; mirrors the direct C call shapes of the landed init rows.
    fn behavior(
        poly: bool,
        order: i32,
        usesegments: bool,
        vararea: bool,
        regionattrib: bool,
        neighbors: bool,
    ) -> Behavior {
        Behavior {
            poly,
            order,
            usesegments,
            vararea,
            regionattrib,
            neighbors,
            ..Behavior::default_form()
        }
    }

    fn mesh(
        poly: bool,
        order: i32,
        usesegments: bool,
        vararea: bool,
        regionattrib: bool,
        neighbors: bool,
    ) -> Mesh {
        Mesh::new(
            &behavior(poly, order, usesegments, vararea, regionattrib, neighbors),
            16,
            2,
            0,
            0,
        )
        .expect("mesh init")
    }

    fn plain() -> Mesh {
        mesh(false, 1, false, false, false, false)
    }

    #[test]
    fn plain_geometry_is_the_c_shape() {
        // order 1, no options: 48-byte triangle rows (stride 6), 24-byte
        // vertex rows (stride 3), mark/vtype sharing word 2 (C dim-2 sizes)
        let m = plain();
        assert_eq!(
            m.tri_shape,
            TriShape {
                stride: 6,
                usesegments: false,
                highorderindex: 6,
                elemattribindex: 6,
                areaboundindex: 6,
                numberindex: None,
            }
        );
        assert_eq!(m.triangles.stride(), 6);
        assert_eq!(
            m.vertex_shape,
            crate::mesh::slots::VertexShape {
                stride: 3,
                mark: HalfWord {
                    word: 2,
                    high: false
                },
                vtype: HalfWord {
                    word: 2,
                    high: true
                },
                v2tri: None,
            }
        );
        assert_eq!(m.vertices.stride(), 3);
        assert_eq!(m.vertex_shape.mark.word, m.vertex_shape.vtype.word);
    }

    #[test]
    fn poly_geometry_is_the_c_shape() {
        let m = mesh(true, 1, true, false, false, false);
        // 72-byte triangle rows (stride 9), subseg rows 68 bytes (stride 9)
        assert_eq!(m.tri_shape.stride, 9);
        assert_eq!(m.tri_shape.highorderindex, 9);
        assert!(m.tri_shape.usesegments);
        assert_eq!(m.subsegs.stride(), 9);
        assert!(m.subsegs.sentinel_id().is_some());
        // 32-byte vertex rows (stride 4) with v2tri at word 3
        assert_eq!(m.vertex_shape.stride, 4);
        assert_eq!(m.vertex_shape.v2tri, Some(3));
        assert_eq!(m.vertices.stride(), 4);
    }

    #[test]
    fn neighbors_geometry_pins_the_numbering_overlay() {
        let m = mesh(false, 1, false, false, false, true);
        // C: trisize raised to 6 * 8 + 4 = 52 bytes -> 7 words
        assert_eq!(m.tri_shape.stride, 7);
        assert_eq!(
            m.tri_shape.numberindex,
            Some(HalfWord {
                word: 6,
                high: false
            })
        );
        // -n with -p leaves the larger row alone (72 bytes already fit)
        let mp = mesh(true, 1, true, false, false, true);
        assert_eq!(mp.tri_shape.stride, 9);
        assert_eq!(
            mp.tri_shape.numberindex,
            Some(HalfWord {
                word: 6,
                high: false
            })
        );
    }

    #[test]
    fn vararea_regionattrib_geometry() {
        // vararea alone: areabound at word 6, trisize 56 -> stride 7
        let m = mesh(false, 1, false, true, false, false);
        assert_eq!(m.tri_shape.areaboundindex, 6);
        assert_eq!(m.tri_shape.stride, 7);
        // regionattrib alone: areabound at word 7, trisize 56 -> stride 7
        let m = mesh(false, 1, false, false, true, false);
        assert_eq!(m.tri_shape.elemattribindex, 6);
        assert_eq!(m.tri_shape.areaboundindex, 7);
        assert_eq!(m.tri_shape.stride, 7);
    }

    #[test]
    fn quadratic_order_and_full_option_geometry() {
        // order 2 (quadratic): trisize = (6 + 6 - 3) * 8 = 72 -> stride 9
        let m = mesh(false, 2, false, false, false, false);
        assert_eq!(m.tri_shape.stride, 9);
        assert_eq!(m.tri_shape.elemattribindex, 9);
        // -p with two element attributes and vararea: elemattribindex 9,
        // areaboundindex 11, trisize (11 + 1) * 8 = 96 -> stride 12
        let m = Mesh::new(&behavior(true, 1, true, true, false, false), 16, 2, 0, 2).expect("init");
        assert_eq!(m.tri_shape.elemattribindex, 9);
        assert_eq!(m.tri_shape.areaboundindex, 11);
        assert_eq!(m.tri_shape.stride, 12);
        assert_eq!(m.triangles.stride(), 12);
    }

    #[test]
    fn sentinel_row_is_bonded_exactly_like_c_init_sentinel_rows() {
        let m = mesh(true, 1, true, false, false, false);
        let dummy_tri = crate::mesh::handles::encode(Otri {
            tri: crate::mesh::handles::DUMMY_TRI,
            orient: 0,
        });
        let dummy_sub = crate::mesh::handles::encode_subseg(crate::mesh::handles::Osub {
            ss: crate::mesh::handles::DUMMY_SUB,
            orient: 0,
        });
        // outer space (row 1 of the triangle pool)
        for i in 0..3 {
            assert_eq!(
                m.tri_neighbor_word(crate::mesh::handles::DUMMY_TRI, i),
                dummy_tri
            );
        }
        for i in 0..3 {
            assert_eq!(m.tri_vertex_word(crate::mesh::handles::DUMMY_TRI, i), 0);
        }
        for i in 0..3 {
            assert_eq!(
                m.tri_subseg_word(crate::mesh::handles::DUMMY_TRI, i),
                dummy_sub
            );
        }
        // the omnipresent subsegment (row 1 of the subseg pool)
        for i in 0..2 {
            assert_eq!(
                m.sub_neighbor_word(crate::mesh::handles::DUMMY_SUB, i),
                dummy_sub
            );
        }
        for i in 0..4 {
            assert_eq!(m.sub_vertex_word(crate::mesh::handles::DUMMY_SUB, i), 0);
        }
        for i in 0..2 {
            assert_eq!(
                m.sub_tri_word(crate::mesh::handles::DUMMY_SUB, i),
                dummy_tri
            );
        }
        assert_eq!(
            m.sub_mark(crate::mesh::handles::DUMMY_SUB),
            0,
            "the dummy subseg's boundary marker is zero"
        );
        // sym across outer space returns outer space with the orientation
        // folded off, exactly like C decode of the aligned dummy pointer
        for orient in 0..3 {
            let handle = Otri {
                tri: crate::mesh::handles::DUMMY_TRI,
                orient,
            };
            let mut flipped = handle;
            m.sym_self(&mut flipped);
            assert_eq!(
                flipped,
                Otri {
                    tri: crate::mesh::handles::DUMMY_TRI,
                    orient: 0
                }
            );
        }
    }

    #[test]
    fn without_segments_the_subseg_pool_stays_dormant() {
        let m = plain();
        assert_eq!(m.subsegs.stride(), 0);
        assert_eq!(m.subsegs.sentinel_id(), None);
        assert_eq!(m.subsegs.first_fresh_id(), 1);
    }

    #[test]
    fn first_block_hints_and_small_pool_geometry() {
        let m = plain();
        // invertices 16 <= 2046: triangles reserve max(2 * 16 - 2, 4092)
        // rows plus the null and sentinel rows; vertices max(16, 4092) + 1
        assert_eq!(m.triangles.test_row_capacity(), 2 + 4092);
        assert_eq!(m.vertices.test_row_capacity(), 1 + 4092);
        // the small pools: C itembytes and blocks (ADR 0003 section 4)
        assert_eq!(m.viri.stride(), 8 / 8);
        assert_eq!(m.viri.block_items(), 1020);
        assert_eq!(m.viri.first_fresh_id(), 1);
        assert_eq!(m.badsubsegs.stride(), 3);
        assert_eq!(m.badsubsegs.block_items(), 252);
        assert_eq!(m.badtriangles.stride(), 6);
        assert_eq!(m.badtriangles.block_items(), 4092);
        assert_eq!(m.flipstackers.stride(), 2);
        assert_eq!(m.flipstackers.block_items(), 252);
        assert_eq!(m.splaynodes.stride(), 5);
        assert_eq!(m.splaynodes.block_items(), 508);
        // large invertices stretch the first blocks like C
        let big = Mesh::new(
            &behavior(false, 1, false, false, false, false),
            6000,
            2,
            0,
            0,
        )
        .expect("init");
        assert_eq!(big.triangles.first_block_items(), 2 * 6000 - 2);
        assert_eq!(big.vertices.first_block_items(), 6000);
    }

    #[test]
    fn make_triangle_rows_start_from_id_two() {
        let mut m = plain();
        let a = m.make_triangle(false, false).expect("alloc");
        let b = m.make_triangle(false, false).expect("alloc");
        assert_eq!(a.orient, 0);
        assert_eq!(b.orient, 0);
        assert_eq!((a.tri, b.tri), (2, 3));
        // every fresh triangle is bonded to outer space on all three sides
        assert_eq!(
            m.sym(a),
            Otri {
                tri: crate::mesh::handles::DUMMY_TRI,
                orient: 0
            }
        );
        // vertices are NULL until set
        assert_eq!((m.org(a), m.dest(a), m.apex(a)), (0, 0, 0));
        assert_eq!(m.triangles.live_count(), 2);
    }

    #[test]
    fn make_triangle_with_attributes_initializes_the_c_values() {
        let mut m =
            Mesh::new(&behavior(true, 1, true, true, false, false), 16, 2, 0, 2).expect("init");
        let a = m.make_triangle(true, true).expect("alloc");
        for i in 0..2 {
            assert_eq!(m.tri_attribute(a.tri, i), 0.0);
        }
        assert_eq!(m.tri_area_bound(a.tri), -1.0);
        // and the subseg sides are the omnipresent subsegment
        assert_eq!(
            m.tri_subseg(a),
            crate::mesh::handles::Osub {
                ss: crate::mesh::handles::DUMMY_SUB,
                orient: 0
            }
        );
    }

    #[test]
    fn make_subseg_rows_match_the_c_initialization() {
        let mut m = mesh(true, 1, true, false, false, false);
        let s = m.make_subseg().expect("alloc");
        assert_eq!(s.orient, 0);
        // subseg rows start at id 2 as well (sentinel at 1)
        assert_eq!(s.ss, 2);
        let dummy_sub = crate::mesh::handles::encode_subseg(crate::mesh::handles::Osub {
            ss: crate::mesh::handles::DUMMY_SUB,
            orient: 0,
        });
        for i in 0..2 {
            assert_eq!(m.sub_neighbor_word(s.ss, i), dummy_sub);
        }
        for i in 0..4 {
            assert_eq!(m.sub_vertex_word(s.ss, i), 0);
        }
        for i in 0..2 {
            assert_eq!(
                m.sub_tri_word(s.ss, i),
                crate::mesh::handles::encode(Otri {
                    tri: crate::mesh::handles::DUMMY_TRI,
                    orient: 0
                })
            );
        }
        assert_eq!(m.sub_mark(s.ss), 0);
        assert_eq!(m.subsegs.live_count(), 1);
        // both ends pivot to the omnipresent subsegment
        assert_eq!(
            m.pivot_subseg(s),
            crate::mesh::handles::Osub {
                ss: crate::mesh::handles::DUMMY_SUB,
                orient: 0
            }
        );
        assert_eq!(
            m.subseg_tri(s),
            Otri {
                tri: crate::mesh::handles::DUMMY_TRI,
                orient: 0
            }
        );
    }

    #[test]
    fn vertex_rows_start_from_id_one() {
        let mut m = plain();
        // no sentinel in the vertex pool (D3): fresh ids from 1
        let v = m.vx_alloc().expect("alloc");
        assert_eq!(v, 1);
        assert_eq!(m.vertices.sentinel_id(), None);
        // and a polygonal mesh wires a fresh vertex into a fresh triangle
        let mut mp = mesh(true, 1, true, false, false, false);
        let v = mp.vx_alloc().expect("alloc");
        assert_eq!(v, 1);
        let t = mp.make_triangle(true, false).expect("alloc");
        mp.set_org(t, v);
        assert_eq!(mp.org(t), v);
        mp.vx_set_2tri(v, crate::mesh::handles::encode(t));
        assert_eq!(mp.vx_2tri(v), crate::mesh::handles::encode(t));
    }

    #[test]
    fn traverse_vertex_walks_live_rows_and_skips_the_dead() {
        // C vertextraverse (triangle.c:1276-1288): allocation order over
        // the whole vertex pool, DEADVERTEX rows skipped (pool traversal
        // itself walks dead rows, D3), NULL at the end. vx_dealloc marks
        // the row DEADVERTEX, so the traversal must skip it.
        let mut m = plain();
        let a = m.vx_alloc().expect("alloc");
        let b = m.vx_alloc().expect("alloc");
        let c = m.vx_alloc().expect("alloc");
        let mut t = m.vertices.traversal();
        assert_eq!(m.traverse_vertex(&mut t), Some(a));
        assert_eq!(m.traverse_vertex(&mut t), Some(b));
        assert_eq!(m.traverse_vertex(&mut t), Some(c));
        assert_eq!(m.traverse_vertex(&mut t), None);
        m.vx_dealloc(b);
        let mut t = m.vertices.traversal();
        assert_eq!(m.traverse_vertex(&mut t), Some(a));
        assert_eq!(m.traverse_vertex(&mut t), Some(c));
        assert_eq!(m.traverse_vertex(&mut t), None);
    }
}

// R4 module reorganization (docs/architecture-spec.md section 5): the
// storage model lives under mesh/ - the packed-handle algebra
// (mesh/handles.rs), the row/slot vocabulary (mesh/slots.rs), the word pool
// (mesh/pool.rs), the mesh-local debugging and utility sections (mesh/debug.rs,
// mesh/util.rs), and this module (the Mesh struct and its pool wiring).
pub(crate) mod debug;
pub(crate) mod handles;
pub(crate) mod pool;
pub(crate) mod slots;
pub(crate) mod util;
