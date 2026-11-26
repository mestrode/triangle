//! The aCute refinement (feature acute): acute.c, the aCute fork's
//! Steiner-point machinery, transliterated 1:1 (assignment:
//! docs/function-mapping.md, the `acute.c` section). The module sits
//! behind cfg(feature = "acute"), the exact inverse of C's NO_ACUTE
//! compile switch, so the crate's default build mirrors the fork's
//! default C build with aCute compiled in.
//!
//! This module lands in three PRs. Stage-1 acute A (#40) landed the
//! pure geometry substrate: the ten acute.c functions whose signatures
//! carry no mesh, no behavior and no otri - nothing but REAL
//! coordinates, in C file order. This one (stage-1 acute B) lands the
//! mesh-coupled helpers and the acutepool as typed Vec scratch owned
//! by Mesh (the D23 dissolution): the AcutePool of
//! acutepool_init/resize/deinit, doSmoothing, the star walk of
//! getStarPoints/getNeighborsVertex, the wedge intersections
//! getWedgeIntersectionWithoutMaxAngle/getWedgeIntersectionWithMaxAngle,
//! polygonAngles/testPolygonAngle/testTriangleAngle and
//! minDistanceToNeigbor. The findNewSPLocation family and the D16 lift
//! of refine.rs's funnel marker land with the last PR (stage-1 acute
//! C); doSmoothing and minDistanceToNeigbor stay ahead-of-use until
//! then, with their site-scoped dead_code allows citing their C
//! callers.
//!
//! Module-level C state mirrored here: compConst (acute.c:81, the "for
//! comparing real numbers" epsilon); PI is C's triangle_config.h:159
//! macro, mirrored as std::f64::consts::PI exactly like options.rs and
//! quality.rs.

use std::f64::consts::PI; // C: #define PI (triangle_config.h:159)

use crate::handles::{Otri, DUMMY_TRI, NULL};
use crate::locate::{preciselocate, LocateResult};
use crate::mesh::Mesh;
use crate::options::Behavior;
use crate::pool::Oom;
use crate::predicates::tri_counterclockwise;
use crate::slots::FREEVERTEX;
use crate::util::vertex_coords;

// -------------------------------------------------------------------
// ACUTE MEMORY POOL (the acute.c:20 banner, the block acute.c:18-78)
// -------------------------------------------------------------------

/// C: struct acutepool (private/acute.h:6-19, the type of C's m->acute_mem,
/// triangle.h:496): the aCute scratch pool, dissolved per ADR 0002
/// section 4 item 8 into eight typed Vec<f64> buffers owned by Mesh
/// behind this feature (the `acute_mem` field, D23) with C's field
/// names and capacities. These are per-call scratch, only grown and
/// rewritten - never walked, recycled or id-addressed like the item
/// pools of ADR 0003.
///
/// Lifecycle (D23): C's acutepool_init (acute.c:22-43) allocates
/// the struct (trimalloc, acute.c:23) and its eight buffers
/// (malloc, acute.c:27-40) and hangs it off m->acute_mem inside
/// enforcequality's NO_ACUTE switch (the guarded lazy init at
/// triangle.c:7536-7540, n = 20 at triangle.c:7538); the port
/// constructs the field eagerly in Mesh::new, the site mapping
/// triangleinit's `m->acute_mem = NULL` (triangle.c:1507). The
/// empty-state shape (Option vs always-present default struct) is
/// decided as always-present: every reader sits inside the wedge
/// functions after their acutepool_resize call, so C's
/// NULL-until-enforcequality state is unobservable and the eager
/// construction is its deterministic superset. C's acutepool_resize
/// (acute.c:45-62) survives verbatim as acutepool_resize below,
/// called where C calls it (acute.c:2276, 2522). C's acutepool_deinit
/// (acute.c:64-76) and the trifree that follows it in triangledeinit
/// (triangle.c:1395-1396) dissolve into Drop of the owned field
/// (D11's RAII); triangledeinit itself is not landed yet (the mesh.rs
/// module doc's NOT-landed list), so the dissolution is documented
/// there and at the mesh.rs field.
#[allow(clippy::struct_field_names)] // C-parity: the C struct's field
                                     // names petalx/petaly/petalr and points_p/points_q/points_r
                                     // (private/acute.h:6-19, the mallocs at acute.c:27-40)
pub(crate) struct AcutePool {
    /// C p->size (acute.c:25): the star size the four size-dependent
    /// buffers are currently good for.
    pub(crate) size: i32,
    /// C p->initialpoly (acute.c:27, 500 REALs): the initial convex
    /// polygon the wedge functions clip down to the feasible region.
    pub(crate) initialpoly: Vec<f64>,
    /// C p->petalx, petaly, petalr (acute.c:28-30, 2n REALs each):
    /// the per-star-point petal centers and radii; the wedge functions
    /// double-buffer them, writing slot i/2 and slot numpoints + i/2
    /// per star point (acute.c:2314-2319, 2563-2568).
    pub(crate) petalx: Vec<f64>,
    pub(crate) petaly: Vec<f64>,
    pub(crate) petalr: Vec<f64>,
    /// C p->wedges (acute.c:36, 2n*20 + 40 REALs): the per-star-point
    /// wedge lines, 32 REALs per point in
    /// getWedgeIntersectionWithoutMaxAngle (stride i*16, acute.c:2347)
    /// and 40 per point in getWedgeIntersectionWithMaxAngle (stride
    /// i*20, acute.c:2598). C's own sizing comment (acute.c:32-34):
    /// "If maxangle is 0.0 we'd only need (2 * n * 16 + 36) REALs, but
    /// since we do not know if maxangle gets set later on, let's
    /// allocate enough memory right away."
    pub(crate) wedges: Vec<f64>,
    /// C p->points_p, points_q, points_r (acute.c:38-40, 500 REALs
    /// each): the three star-point scratch buffers doSmoothing hands
    /// to getStarPoints (acute.c:1877-1879).
    pub(crate) points_p: Vec<f64>,
    pub(crate) points_q: Vec<f64>,
    pub(crate) points_r: Vec<f64>,
}

impl AcutePool {
    /// C: acutepool_init (acute.c:22-43) with C's lazy-init argument
    /// n = 20 of the enforcequality call site (triangle.c:7538),
    /// dissolved into the eager construction of the Mesh field (D23;
    /// see the struct doc). The eight mallocs (acute.c:27-40) become
    /// eight zero-filled Vecs at C's exact capacities: 500 for
    /// initialpoly and the three points buffers, 2n for the three
    /// petal buffers, 2n*20 + 40 for wedges. C's malloc'd REALs are
    /// indeterminate; every slot is rewritten before its first read
    /// on every observable path, so the zero fill is unobservable
    /// (D24).
    pub(crate) fn new(n: i32) -> AcutePool {
        AcutePool {
            // C: p->size = n; (acute.c:25)
            size: n,
            // C: p->initialpoly = (REAL *)malloc(sizeof(REAL)* 500);
            // (acute.c:27)
            initialpoly: vec![0.0; 500],
            // C: p->petalx = (REAL *)malloc(sizeof(REAL)* 2 * n); and
            // the two lines below it (acute.c:28-30)
            petalx: vec![0.0; (2 * n) as usize],
            petaly: vec![0.0; (2 * n) as usize],
            petalr: vec![0.0; (2 * n) as usize],
            // The sizing comment C puts here (acute.c:32-34) is quoted
            // on the wedges field; the malloc is acute.c:36.
            wedges: vec![0.0; (2 * n * 20 + 40) as usize],
            // C: p->points_p/q/r = (REAL *)malloc(sizeof(REAL)* 500);
            // (acute.c:38-40)
            points_p: vec![0.0; 500],
            points_q: vec![0.0; 500],
            points_r: vec![0.0; 500],
        }
    }

    /// The empty placeholder of doSmoothing's scoped pool move (see
    /// doSmoothing): while the real pool is moved out of the mesh, the
    /// field holds this shape and nothing reads it (D23).
    pub(crate) fn empty() -> AcutePool {
        AcutePool {
            size: 0,
            initialpoly: Vec::new(),
            petalx: Vec::new(),
            petaly: Vec::new(),
            petalr: Vec::new(),
            wedges: Vec::new(),
            points_p: Vec::new(),
            points_q: Vec::new(),
            points_r: Vec::new(),
        }
    }

    /// The buffer pulls at the heads of the two wedge functions
    /// (`petalx = m->acute_mem->petalx; ...`, acute.c:2278-2282 and
    /// 2524-2528): the five size-dependent scratch buffers as one
    /// tuple of disjoint mutable borrows, taken once per call exactly
    /// where C takes its five local pointers. C's mesh* m parameter
    /// carries only the pool at those sites, so the mirror passes the
    /// pool itself and this method is the aliasing-free pull (D23).
    #[allow(clippy::type_complexity)] // C-parity: the tuple is C's five
                                      // local pointers, one slice per buffer (acute.c:2278-2282,
                                      // 2524-2528)
    pub(crate) fn wedge_scratch(
        &mut self,
    ) -> (&mut [f64], &mut [f64], &mut [f64], &mut [f64], &mut [f64]) {
        let AcutePool {
            petalx,
            petaly,
            petalr,
            wedges,
            initialpoly,
            ..
        } = self;
        (petalx, petaly, petalr, wedges, initialpoly)
    }
}

/// C: acutepool_resize (acute.c:45-62). Grows the four size-dependent
/// buffers when the star size n exceeds the pool's current size; the
/// 500-sized buffers (initialpoly, points_p/q/r) are size-independent
/// and stay untouched, exactly as in C. C's free/malloc pairs
/// (acute.c:50-53, 56-60) become zero-filled Vec resizes: the guard makes
/// the growth strictly increasing, and every grown slot is rewritten
/// before its first read on every observable path, so the fresh
/// garbage of C's mallocs is unobservable (D23, D24). The two C call
/// sites are the wedge functions (acute.c:2276, 2522); the mirror
/// calls this at the same statement positions.
pub(crate) fn acutepool_resize(n: i32, p: &mut AcutePool) {
    // C: if (p->size < n) { (acute.c:46): the only-if-smaller guard.
    if p.size < n {
        // C: p->size = n; (acute.c:47)
        p.size = n;
        // Free old memory (acute.c:49-53) / Allocate new memory
        // (acute.c:55-60): the free+malloc pairs dissolve into the
        // Vec growth.
        p.petalx.resize((2 * n) as usize, 0.0);
        p.petaly.resize((2 * n) as usize, 0.0);
        p.petalr.resize((2 * n) as usize, 0.0);
        p.wedges.resize((2 * n * 20 + 40) as usize, 0.0);
    }
}

// END ACUTE MEMORY POOL (acute.c:78 banner)

/// C: `const double compConst = 1.0e-80;` (acute.c:81), directly under
/// the `// for comparing real numbers` comment (acute.c:80). The epsilon
/// of the aCute geometry: lineLineIntersection (acute.c:3005, 3009),
/// linePointLocation (acute.c:3261) and circleLineIntersection
/// (acute.c:3378, 3387) compare against it; splitConvexPolygon
/// (acute.c:3112) and lineLineSegmentIntersection (acute.c:3295) shadow
/// it with local tolerances, exactly as the same-named body-local
/// constants shadow this crate constant. The mesh-coupled readers
/// getStarPoints (acute.c:2109-2110) and getNeighborsVertex (acute.c:
/// 2159-2184) compare vertex coordinates against it; they land with
/// this PR, retiring the ahead-of-use state.
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:81)
#[allow(non_upper_case_globals)] // C-parity: the C name verbatim (acute.c:81)
pub(crate) const compConst: f64 = 1.0e-80;

/// C: longestShortestEdge (acute.c:1816-1850). Given the squares of the
/// three edge lengths of a triangle (aodist, dadist, dodist), determine
/// its orientation: the returned code packs which edge is shortest,
/// middle, longest, exactly as C's banner comment tabulates (acute.c:
/// 1804-1815; e.g. 123: shortest aodist, middle dadist, longest dodist).
///
/// The only C call sites sit in the findNewSPLocation family (acute.c:
/// 221 and 843), which lands with stage-1 acute C; the 1:1 body is
/// ahead-of-use until then.
#[allow(dead_code)]
// C-parity: the call sites (acute.c:221, 843) belong
// to the findNewSPLocation family, stage-1 acute C.
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:1816)
pub(crate) fn longestShortestEdge(aodist: f64, dadist: f64, dodist: f64) -> i32 {
    // C: int max = 0, min = 0, mid = 0,minMidMax; (acute.c:1818). The
    // = 0 initializers of max/min/mid are dead stores (every branch
    // assigns all three before the first read) and are dropped (D19).
    let max: i32;
    let min: i32;
    let mid: i32;
    if dodist < aodist && dodist < dadist {
        min = 3; // apex is the smallest angle, dodist is the longest edge
        if aodist < dadist {
            max = 2; // dadist is the longest edge
            mid = 1; // aodist is the middle longest edge
        } else {
            max = 1; // aodist is the longest edge
            mid = 2; // dadist is the middle longest edge
        }
    } else if aodist < dadist {
        min = 1; // dest is the smallest angle, aodist is the biggest edge
        if dodist < dadist {
            max = 2; // dadist is the longest edge
            mid = 3; // dodist is the middle longest edge
        } else {
            max = 3; // dodist is the longest edge
            mid = 2; // dadist is the middle longest edge
        }
    } else {
        min = 2; // origin is the smallest angle, dadist is the biggest edge
        if aodist < dodist {
            max = 3; // dodist is the longest edge
            mid = 1; // aodist is the middle longest edge
        } else {
            max = 1; // aodist is the longest edge
            mid = 3; // dodist is the middle longest edge
        }
    }
    // C: minMidMax = min * 100 + mid * 10 + max; (acute.c:1847).
    let minMidMax = min * 100 + mid * 10 + max;
    // HANDLE ISOSCELES TRIANGLE CASE (acute.c:1848)
    minMidMax
}

/// C: doSmoothing (acute.c:1858-2035; banner at acute.c:1852-1857:
/// "returns 1,2 or 3 if smoothing will work, 0 otherwise; also
/// returns the new location for the point, if somothing is possible").
/// Given a bad triangle (badotri, torg/tdest/tapex its vertices), try
/// to relocate each of the three vertices into the feasible region of
/// its star: returns 1, 2 or 3 (which vertex to relocate: origin,
/// destination, apex) with the suggested location in newloc, or 0
/// when nothing was found. A relocation only becomes a possibility
/// when the vertex is a free vertex (C's vertextype == 2 tests at
/// acute.c:1916, 1957, 2000; the port reads the FREEVERTEX constant
/// through the vx_type chokepoint).
///
/// Its only C callers are the findNewSPLocation variants (acute.c:397,
/// 1019), which land with the last acute PR; the body is ahead-of-use
/// until then.
///
/// C's aliasing (acute.c:1877-1879: points_p = m->acute_mem->points_p
/// and the like, alongside the mesh reads of the star walk and the
/// pool-carrying wedge calls) mirrors as a scoped move (D23): the pool
/// leaves the mesh for the duration of the body (the field holds
/// AcutePool::empty()), the three points buffers leave the pool, and
/// everything is restored in reverse order after the body; C's early
/// returns (acute.c:2013, 2019, 2025, 2032, 2034) become breaks out
/// of a labeled block so the restore runs on every path.
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:1858)
#[allow(dead_code)]
// C-parity: the callers (acute.c:397, 1019) belong to the
// findNewSPLocation family, the last acute PR.
#[allow(clippy::too_many_arguments)] // C-parity: the seven-parameter C
// signature (acute.c:1858-1863)
#[allow(clippy::float_cmp)] // C-parity: b->maxangle == 0.0 verbatim
                            // (acute.c:1909, 1949, 1990)
pub(crate) fn doSmoothing(
    m: &mut Mesh,
    b: &Behavior,
    badotri: Otri,
    torg: u32,
    tdest: u32,
    tapex: u32,
    newloc: &mut [f64],
) -> i32 {
    // C: int numpoints_p = 0; ... (acute.c:1865-1867): keeps the
    // number of points in a star of point p, q, r. C's = 0
    // initializers are dead stores (the matching getStarPoints try
    // reassigns each before its first read) and are dropped (D24).
    let numpoints_p;
    let numpoints_q;
    let numpoints_r;
    // C: REAL possibilities[6]; (acute.c:1868): there can be more than
    // one possibilities. C leaves the array indeterminate; every read
    // slot was written by the matching try before its flag branch can
    // read it, so the zero fill is unobservable (D24).
    let mut possibilities = [0.0f64; 6];
    // C: int num_pos = 0; (acute.c:1869): number of possibilities.
    let mut num_pos = 0;
    // C: int flag1 = 0, flag2 = 0, flag3 = 0; (acute.c:1870)
    let mut flag1 = 0;
    let mut flag2 = 0;
    let mut flag3 = 0;
    // C: int newLocFound = 0; (acute.c:1871). C's = 0 initializer
    // is a dead store (reassigned on every path before the first
    // read) and is dropped (D24).
    let mut newLocFound;

    // C: points_p = m->acute_mem->points_p; points_q = ...;
    // points_r = ...; (acute.c:1877-1879). The scoped pool move (D23):
    // C's local pointers alias the mesh-owned buffers; the zero-unsafe
    // mirror owns them for the duration of the body and gives them
    // back below.
    let mut pool = std::mem::replace(&mut m.acute_mem, AcutePool::empty());
    let mut points_p = std::mem::take(&mut pool.points_p);
    let mut points_q = std::mem::take(&mut pool.points_q);
    let mut points_r = std::mem::take(&mut pool.points_r);

    // C's returns leave through the labeled block so the reverse-order
    // restore runs on every path (D23).
    let result = 'smoothing: {
        /*********************** TRY TO RELOCATE POINT "p" *************************/
        // get the surrounding points of p, so this gives us the triangles
        numpoints_p = getStarPoints(m, badotri, torg, tdest, tapex, 1, &mut points_p);
        // INTERSECTION OF PETALS
        // first check whether the star angles are appropriate for relocation
        if numpoints_p != 0 && polygonAngles(b, numpoints_p, &points_p) == 0 {
            if b.maxangle == 0.0 {
                newLocFound = getWedgeIntersectionWithoutMaxAngle(
                    &mut pool,
                    b,
                    numpoints_p,
                    &points_p,
                    newloc,
                );
            } else {
                newLocFound =
                    getWedgeIntersectionWithMaxAngle(&mut pool, b, numpoints_p, &points_p, newloc);
            }
            // make sure the relocated point is a free vertex
            if newLocFound != 0 && m.vx_type(torg) == FREEVERTEX {
                possibilities[0] = newloc[0]; // something found
                possibilities[1] = newloc[1];
                num_pos += 1; // increase the number of possibilities
                flag1 = 1;
            }
        }

        /*********************** TRY TO RELOCATE POINT "q" *************************/
        // get the surrounding points of q, so this gives us the triangles
        numpoints_q = getStarPoints(m, badotri, torg, tdest, tapex, 2, &mut points_q);
        // INTERSECTION OF PETALS
        // first check whether the star angles are appropriate for relocation
        if numpoints_q != 0 && polygonAngles(b, numpoints_q, &points_q) == 0 {
            if b.maxangle == 0.0 {
                newLocFound = getWedgeIntersectionWithoutMaxAngle(
                    &mut pool,
                    b,
                    numpoints_q,
                    &points_q,
                    newloc,
                );
            } else {
                newLocFound =
                    getWedgeIntersectionWithMaxAngle(&mut pool, b, numpoints_q, &points_q, newloc);
            }
            // make sure the relocated point is a free vertex
            if newLocFound != 0 && m.vx_type(tdest) == FREEVERTEX {
                possibilities[2] = newloc[0]; // something found
                possibilities[3] = newloc[1];
                num_pos += 1; // increase the number of possibilities
                flag2 = 2;
            }
        }

        /*********************** TRY TO RELOCATE POINT "q" *************************/
        // get the surrounding points of r, so this gives us the triangles
        // (C's third banner repeats "q" verbatim, acute.c:1966)
        numpoints_r = getStarPoints(m, badotri, torg, tdest, tapex, 3, &mut points_r);
        // INTERSECTION OF PETALS
        // first check whether the star angles are appropriate for relocation
        if numpoints_r != 0 && polygonAngles(b, numpoints_r, &points_r) == 0 {
            if b.maxangle == 0.0 {
                newLocFound = getWedgeIntersectionWithoutMaxAngle(
                    &mut pool,
                    b,
                    numpoints_r,
                    &points_r,
                    newloc,
                );
            } else {
                newLocFound =
                    getWedgeIntersectionWithMaxAngle(&mut pool, b, numpoints_r, &points_r, newloc);
            }
            // make sure the relocated point is a free vertex
            if newLocFound != 0 && m.vx_type(tapex) == FREEVERTEX {
                possibilities[4] = newloc[0]; // something found
                possibilities[5] = newloc[1];
                num_pos += 1; // increase the number of possibilities
                flag3 = 3;
            }
        }
        // AFTER FINISH CHECKING EVERY POSSIBILITY, CHOOSE ANY OF THE AVAILABLE ONE
        if num_pos > 0 {
            if flag1 > 0 {
                // suggest to relocate origin
                newloc[0] = possibilities[0];
                newloc[1] = possibilities[1];
                break 'smoothing flag1;
            } else {
                if flag2 > 0 {
                    // suggest to relocate apex
                    newloc[0] = possibilities[2];
                    newloc[1] = possibilities[3];
                    break 'smoothing flag2;
                } else {
                    // suggest to relocate destination
                    if flag3 > 0 {
                        newloc[0] = possibilities[4];
                        newloc[1] = possibilities[5];
                        break 'smoothing flag3;
                    }
                }
            }
        } else {
            break 'smoothing 0; // could not find any good relocation
        }
        // C: return 0; (acute.c:2034): the num_pos > 0 fall-through.
        0
    };

    // The reverse-order restore of the scoped move (D23): the points
    // buffers go back into the pool, the pool back into the mesh.
    pool.points_p = points_p;
    pool.points_q = points_q;
    pool.points_r = points_r;
    m.acute_mem = pool;
    result
}

/// C: getStarPoints (acute.c:2040-2113; banner at acute.c:2034-2039:
/// "Given the triangulation, a triangle, three vertices of the
/// triangle, and a specific point choice returns list of points and
/// the number of points on the star of the given point"). Walk the
/// ring of the chosen vertex (whichPoint 1/2/3 = the star of torg,
/// tdest, tapex) and write it into points as REAL coordinate pairs;
/// returns the number of ring points, or 0 when the walk hits the
/// border (getNeighborsVertex's not-found arm). C reads the vertex
/// coordinates through its vertex pointers (first_x = p[0] etc.,
/// acute.c:2058-2079); the mirror reads them through the vx_real
/// chokepoint and the ring through sym/org/dest/apex
/// (getNeighborsVertex, acute.c:2093-2103), so the whole function is
/// a read-only mesh walk.
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:2040)
#[allow(clippy::too_many_arguments)] // C-parity: the eight-parameter C
// signature (acute.c:2040-2046)
#[allow(clippy::needless_late_init)] // C-parity: struct otri tempotri
                                     // (acute.c:2048) is declared in C's block and assigned at
                                     // acute.c:2082 (tempotri = badotri)
pub(crate) fn getStarPoints(
    m: &Mesh,
    badotri: Otri,
    p: u32,
    q: u32,
    r: u32,
    whichPoint: i32,
    points: &mut [f64],
) -> i32 {
    // C: struct otri neighotri; (acute.c:2047): for return value of
    // the function. Zero-init (D24): getNeighborsVertex writes it
    // before anything reads it.
    let mut neighotri = Otri {
        tri: DUMMY_TRI,
        orient: 0,
    };
    // C: struct otri tempotri; (acute.c:2048): for temporary usage.
    let mut tempotri: Otri;
    // C: REAL first_x, first_y; (acute.c:2049): keeps the first point
    // to be considered.
    let first_x: f64;
    let first_y: f64;
    // C: REAL second_x, second_y; (acute.c:2050): for determining the
    // edge we will begin.
    let mut second_x: f64;
    let mut second_y: f64;
    // C: REAL third_x, third_y; (acute.c:2051): termination.
    let third_x: f64;
    let third_y: f64;
    // C: REAL returnPoint[2]; (acute.c:2052): for keeping the returned
    // point. Zero-init (D24): the dummy assignment below (C
    // acute.c:2089) writes it before any read.
    let mut returnPoint = [0.0f64; 2];
    // C: int numvertices = 0; (acute.c:2053): for keeping number of
    // surrounding vertices.
    let mut numvertices = 0;

    // first determine which point to be used to find its neighbor triangles
    // C: switch(whichPoint){ (acute.c:2056-2081)
    match whichPoint {
        1 => {
            first_x = m.vx_real(p, 0); // point at the center
            first_y = m.vx_real(p, 1);
            second_x = m.vx_real(r, 0); // second vertex of first edge to consider
            second_y = m.vx_real(r, 1);
            third_x = m.vx_real(q, 0); // for terminating the search
            third_y = m.vx_real(q, 1);
        }
        2 => {
            first_x = m.vx_real(q, 0); // point at the center
            first_y = m.vx_real(q, 1);
            second_x = m.vx_real(p, 0); // second vertex of first edge to consider
            second_y = m.vx_real(p, 1);
            third_x = m.vx_real(r, 0); // for terminating the search
            third_y = m.vx_real(r, 1);
        }
        3 => {
            first_x = m.vx_real(r, 0); // point at the center
            first_y = m.vx_real(r, 1);
            second_x = m.vx_real(q, 0); // second vertex of first edge to consider
            second_y = m.vx_real(q, 1);
            third_x = m.vx_real(p, 0); // for terminating the search
            third_y = m.vx_real(p, 1);
        }
        // C's switch has no default arm (acute.c:2056-2081); the only
        // callers pass 1, 2 or 3 (acute.c:1884, 1927, 1969).
        _ => unreachable!("whichPoint is 1, 2 or 3 (acute.c:2056)"),
    }

    // C: tempotri = badotri; (acute.c:2082)
    tempotri = badotri;
    // add first point as the end of first edge
    points[numvertices as usize] = second_x;
    numvertices += 1;
    points[numvertices as usize] = second_y;
    numvertices += 1;
    // assign as dummy value
    returnPoint[0] = second_x;
    returnPoint[1] = second_y;
    // until we reach the third point of the beginning triangle
    // C: do{ ... }while( !( ... ) ); (acute.c:2091-2110)
    loop {
        // find the neighbor's third point where it is incident to given edge
        if getNeighborsVertex(
            m,
            tempotri,
            first_x,
            first_y,
            second_x,
            second_y,
            &mut returnPoint,
            &mut neighotri,
        ) == 0
        {
            // go to next triangle
            tempotri = neighotri;
            // now the second point is the neighbor's third vertex
            second_x = returnPoint[0];
            second_y = returnPoint[1];
            // add a new point to the list of surrounding points
            points[numvertices as usize] = returnPoint[0];
            numvertices += 1;
            points[numvertices as usize] = returnPoint[1];
            numvertices += 1;
        } else {
            numvertices = 0;
            break;
        }
        // C: }while( !( (fabs(returnPoint[0] - third_x) <= compConst)  &&
        //     (fabs(returnPoint[1] - third_y) <= compConst) ) );
        // (acute.c:2109-2110)
        if !((returnPoint[0] - third_x).abs() <= compConst
            && (returnPoint[1] - third_y).abs() <= compConst)
        {
            continue;
        }
        break;
    }
    numvertices / 2
}

/// C: getNeighborsVertex (acute.c:2121-2241; banner at acute.c:2114-
/// 2120: "Given the triangulation, a triangle, an edge represented by
/// two points, returns the neighbor's third vertex incident to given
/// edge also returns the pointer for this neighbor triangle; returns
/// 1, if not found, 0 otherwise"). Checks the triangle's three
/// neighbors for one whose corners include both wanted points: the
/// match tests compare the neighbor's vertex coordinates against the
/// wanted points with compConst (acute.c:2159-2184) and the
/// degenerate-triangle check compares coordinates for equality
/// (acute.c:2152-2154); the mirror reads through vx_real exactly at
/// those sites. The matched flags survive across the orientation loop
/// exactly as in C (they are loop-carried, not reset when a neighbor
/// is nonexistent or degenerate), and the neighbor handle is written
/// to neighotri on every path (acute.c:2238).
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:2121)
#[allow(clippy::too_many_arguments)] // C-parity: the eight-parameter C
// signature (acute.c:2121-2124)
#[allow(clippy::float_cmp)] // C-parity: the degenerate-triangle
                            // coordinate equality tests verbatim (acute.c:2152-2154)
pub(crate) fn getNeighborsVertex(
    m: &Mesh,
    badotri: Otri,
    first_x: f64,
    first_y: f64,
    second_x: f64,
    second_y: f64,
    thirdpoint: &mut [f64],
    neighotri: &mut Otri,
) -> i32 {
    // C: struct otri neighbor; (acute.c:2126): keeps the neighbor
    // triangles. Zero-init (D24): the sym below writes it before any
    // read.
    let mut neighbor = Otri {
        tri: DUMMY_TRI,
        orient: 0,
    };
    // C: int notFound = 0; (acute.c:2127): boolean variable if we can
    // find that neighbor or not.
    let mut notFound = 0;

    // C: vertex neighborvertex_1 = NULL; ... (acute.c:2130-2132): for
    // keeping the vertices of the neighbor triangle. The NULL handles
    // mirror C's NULL vertex pointers; they are only read through
    // vx_real in the switch arms below, which the matching flags gate
    // (a nonzero flag pair implies the vertices were read in the same
    // orientation iteration, acute.c:2147-2149), so the NULL is never
    // dereferenced, exactly as in C.
    let mut neighborvertex_1 = NULL;
    let mut neighborvertex_2 = NULL;
    let mut neighborvertex_3 = NULL;

    // used for finding neighbor triangle
    // C: int firstVertexMatched = 0, secondVertexMatched = 0;
    // (acute.c:2135): to find the correct neighbor. Loop-carried
    // in C (not reset per iteration); the mirror keeps that.
    let mut firstVertexMatched = 0;
    let mut secondVertexMatched = 0;

    /* find neighbors */
    /* Check each of the triangle's three neighbors to find the correct one */
    // C: for (badotri.orient = 0; badotri.orient < 3; badotri.orient++)
    let mut badotri = badotri;
    badotri.orient = 0;
    while badotri.orient < 3 {
        /* Find the neighbor. */
        // C: sym(badotri, neighbor); (acute.c:2141)
        neighbor = m.sym(badotri);
        // check if it is the one we are looking for by checking the corners
        /* first check if the neighbor is nonexistent, since it can be on the border */
        if neighbor.tri != DUMMY_TRI {
            // then check if two wanted corners are also in this triangle
            /* take the vertices of the candidate neighbor */
            // C: org(neighbor, neighborvertex_1); ... (acute.c:2147-2149)
            neighborvertex_1 = m.org(neighbor);
            neighborvertex_2 = m.dest(neighbor);
            neighborvertex_3 = m.apex(neighbor);

            // check if it is really a triangle
            if (m.vx_real(neighborvertex_1, 0) == m.vx_real(neighborvertex_2, 0)
                && m.vx_real(neighborvertex_1, 1) == m.vx_real(neighborvertex_2, 1))
                || (m.vx_real(neighborvertex_2, 0) == m.vx_real(neighborvertex_3, 0)
                    && m.vx_real(neighborvertex_2, 1) == m.vx_real(neighborvertex_3, 1))
                || (m.vx_real(neighborvertex_1, 0) == m.vx_real(neighborvertex_3, 0)
                    && m.vx_real(neighborvertex_1, 1) == m.vx_real(neighborvertex_3, 1))
            {
                //printf("Two vertices are the same!!!!!!!\n");
            } else {
                // begin searching for the correct neighbor triangle
                firstVertexMatched = 0;
                if (first_x - m.vx_real(neighborvertex_1, 0)).abs() < compConst
                    && (first_y - m.vx_real(neighborvertex_1, 1)).abs() < compConst
                {
                    firstVertexMatched = 11; // neighbor's 1st vertex is matched to first vertex
                } else if (first_x - m.vx_real(neighborvertex_2, 0)).abs() < compConst
                    && (first_y - m.vx_real(neighborvertex_2, 1)).abs() < compConst
                {
                    firstVertexMatched = 12; // neighbor's 2nd vertex is matched to first vertex
                } else if (first_x - m.vx_real(neighborvertex_3, 0)).abs() < compConst
                    && (first_y - m.vx_real(neighborvertex_3, 1)).abs() < compConst
                {
                    firstVertexMatched = 13; // neighbor's 3rd vertex is matched to first vertex
                }

                secondVertexMatched = 0;
                if (second_x - m.vx_real(neighborvertex_1, 0)).abs() < compConst
                    && (second_y - m.vx_real(neighborvertex_1, 1)).abs() < compConst
                {
                    secondVertexMatched = 21; // neighbor's 1st vertex is matched to second vertex
                } else if (second_x - m.vx_real(neighborvertex_2, 0)).abs() < compConst
                    && (second_y - m.vx_real(neighborvertex_2, 1)).abs() < compConst
                {
                    secondVertexMatched = 22; // neighbor's 2nd vertex is matched to second vertex
                } else if (second_x - m.vx_real(neighborvertex_3, 0)).abs() < compConst
                    && (second_y - m.vx_real(neighborvertex_3, 1)).abs() < compConst
                {
                    secondVertexMatched = 23; // neighbor's 3rd vertex is matched to second vertex
                }
            }
        } // if neighbor exists or not

        // C: the break test at the end of the loop body (acute.c:2193-2196)
        if (firstVertexMatched == 11 && (secondVertexMatched == 22 || secondVertexMatched == 23))
            || (firstVertexMatched == 12
                && (secondVertexMatched == 21 || secondVertexMatched == 23))
            || (firstVertexMatched == 13
                && (secondVertexMatched == 21 || secondVertexMatched == 22))
        {
            break;
        }
        badotri.orient += 1;
    } // end of for loop over all orientations

    // C: switch(firstVertexMatched){ ... } (acute.c:2199-2236)
    match firstVertexMatched {
        0 => {
            notFound = 1;
        }
        11 => {
            if secondVertexMatched == 22 {
                thirdpoint[0] = m.vx_real(neighborvertex_3, 0);
                thirdpoint[1] = m.vx_real(neighborvertex_3, 1);
            } else if secondVertexMatched == 23 {
                thirdpoint[0] = m.vx_real(neighborvertex_2, 0);
                thirdpoint[1] = m.vx_real(neighborvertex_2, 1);
            } else {
                notFound = 1;
            }
        }
        12 => {
            if secondVertexMatched == 21 {
                thirdpoint[0] = m.vx_real(neighborvertex_3, 0);
                thirdpoint[1] = m.vx_real(neighborvertex_3, 1);
            } else if secondVertexMatched == 23 {
                thirdpoint[0] = m.vx_real(neighborvertex_1, 0);
                thirdpoint[1] = m.vx_real(neighborvertex_1, 1);
            } else {
                notFound = 1;
            }
        }
        13 => {
            if secondVertexMatched == 21 {
                thirdpoint[0] = m.vx_real(neighborvertex_2, 0);
                thirdpoint[1] = m.vx_real(neighborvertex_2, 1);
            } else if secondVertexMatched == 22 {
                thirdpoint[0] = m.vx_real(neighborvertex_1, 0);
                thirdpoint[1] = m.vx_real(neighborvertex_1, 1);
            } else {
                notFound = 1;
            }
        }
        // C: default: if(secondVertexMatched == 0){notFound = 1;} break;
        // (acute.c:2233-2235). firstVertexMatched is 0, 11, 12 or 13
        // on every path, so the default arm is unreachable; the mirror
        // keeps the shape.
        _ => {
            if secondVertexMatched == 0 {
                notFound = 1;
            }
        }
    }
    // pointer of the neighbor triangle
    // C: *neighotri = neighbor; (acute.c:2238)
    *neighotri = neighbor;
    // C: return notFound; (acute.c:2239)
    notFound
}

/// C: getWedgeIntersectionWithoutMaxAngle (acute.c:2250-2482; banner at
/// acute.c:2243-2249: "Given the triangulation, list of points and
/// number of points on the star, returns a new location for the point
/// according to surrounding points; returns 1 if new location found,
/// 0 otherwise; this version uses intersection of wedges to find the
/// new location"). Intersects the star's petal wedges (eight wedge
/// lines per star point: the two min-angle lines and six petal-chord
/// lines, acute.c:2339-2384) into a feasible region and writes its
/// centroid into newloc. C's mesh* m parameter carries only the acute
/// pool here (the resize of acute.c:2276 and the buffer pulls of
/// acute.c:2278-2282); the mirror passes the pool (D23) and pulls the
/// five scratch buffers through AcutePool::wedge_scratch where C
/// takes its local pointers. C's fixedarea arm is entirely commented
/// out (the testTriangleAngleArea block, acute.c:2451-2464): a
/// fixedarea run falls through to the final return 0, and the mirror
/// keeps that quirk.
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:2250)
#[allow(clippy::needless_late_init)]
// C-parity: C's declaration
// blocks (acute.c:2252-2274) keep the declaration-then-assignment
// shape
#[allow(clippy::float_cmp)] // C-parity: b->goodangle == 1.0 verbatim
                            // (acute.c:2292) and the p1[0] == 1.0 intersection flag test
                            // (acute.c:2394)
pub(crate) fn getWedgeIntersectionWithoutMaxAngle(
    p: &mut AcutePool,
    b: &Behavior,
    numpoints: i32,
    points: &[f64],
    newloc: &mut [f64],
) -> i32 {
    // C's declaration block (acute.c:2252-2274). The REAL locals keep
    // C's declaration-then-assignment shape; the loop-carried scratch
    // picks the deterministic zero superset of C's indeterminate
    // state (D24).
    let mut x0: f64;
    let mut y0: f64;
    let mut x1: f64;
    let mut y1: f64;
    let mut x2: f64;
    let mut y2: f64;
    let mut x01: f64;
    let mut y01: f64;
    let mut d01: f64;
    let mut xmid: f64;
    let mut ymid: f64;
    let mut dist: f64;
    let mut x3: f64;
    let mut y3: f64;
    let mut x_1: f64;
    let mut y_1: f64;
    let mut x_2: f64;
    let mut y_2: f64;
    let mut x_3: f64;
    let mut y_3: f64;
    let mut x_4: f64;
    let mut y_4: f64;
    let mut tempx: f64;
    let mut tempy: f64;
    let alpha: f64;
    // C: REAL p1[3]; (acute.c:2268): zero-init (D24); lineLineIntersection
    // writes it before the reads of the i == 0 block.
    let mut p1 = [0.0f64; 3];
    // C: int numpolypoints; (acute.c:2269): zero-init (D24); the
    // halfplane loops below write it before the centroid read (the
    // numpoints != 0 guard dominates them).
    let mut numpolypoints = 0;

    // C: int i, j; int s, flag, count, num; (acute.c:2271, 2272)
    let mut i: i32;
    let s: i32;
    let mut flag: i32;
    let mut count: i32;
    let mut num: i32;

    let petalcenterconstant: f64;
    let petalradiusconstant: f64;

    // C: acutepool_resize(numpoints, m->acute_mem); (acute.c:2276)
    acutepool_resize(numpoints, p);

    // C: petalx = m->acute_mem->petalx; ... initialConvexPoly =
    // m->acute_mem->initialpoly; (acute.c:2278-2282): the disjoint
    // borrows of the pull (D23).
    let (petalx, petaly, petalr, wedges, initialConvexPoly) = p.wedge_scratch();

    // C: x0 = points[2*numpoints-4]; ... y1 = points[2*numpoints-1];
    // (acute.c:2284-2287)
    x0 = points[(2 * numpoints - 4) as usize];
    y0 = points[(2 * numpoints - 3) as usize];
    x1 = points[(2 * numpoints - 2) as usize];
    y1 = points[(2 * numpoints - 1) as usize];

    // minimum angle
    // C: alpha = b->minangle*PI/180.0; (acute.c:2290)
    alpha = b.minangle * PI / 180.0;
    // initialize the constants
    // C: if (b->goodangle == 1.0) { ... } else { ... } (acute.c:2292-2298)
    if b.goodangle == 1.0 {
        petalcenterconstant = 0.0;
        petalradiusconstant = 0.0;
    } else {
        petalcenterconstant = 0.5 / (b.minangle * PI / 180.0).tan();
        petalradiusconstant = 0.5 / (b.minangle * PI / 180.0).sin();
    }
    // C: for(i = 0; i < numpoints*2; i= i+2){ (acute.c:2299)
    for i in (0..numpoints * 2).step_by(2) {
        // go to the next point
        x2 = points[i as usize];
        y2 = points[i as usize + 1];

        x01 = x1 - x0;
        y01 = y1 - y0;
        d01 = (x01 * x01 + y01 * y01).sqrt();
        // find the petal of each edge 01;
        petalx[(i / 2) as usize] = x0 + 0.5 * x01 - petalcenterconstant * y01;
        petaly[(i / 2) as usize] = y0 + 0.5 * y01 + petalcenterconstant * x01;
        petalr[(i / 2) as usize] = petalradiusconstant * d01;
        petalx[(numpoints + i / 2) as usize] = petalx[(i / 2) as usize];
        petaly[(numpoints + i / 2) as usize] = petaly[(i / 2) as usize];
        petalr[(numpoints + i / 2) as usize] = petalr[(i / 2) as usize];

        // FIRST FIND THE HALF-PLANE POINTS FOR EACH PETAL
        xmid = (x0 + x1) / 2.0; // mid point of pq
        ymid = (y0 + y1) / 2.0;

        // distance between xmid and petal center
        dist = ((petalx[(i / 2) as usize] - xmid) * (petalx[(i / 2) as usize] - xmid)
            + (petaly[(i / 2) as usize] - ymid) * (petaly[(i / 2) as usize] - ymid))
            .sqrt();
        // find the unit vector goes from mid point to petal center
        // C: ux = ...; uy = ...; (acute.c:2329-2330) are the live stores
        // (read immediately below); the later re-stores of the LINE#1,
        // LINE#2 and petal-vector comment blocks (acute.c:2341-2342,
        // 2352-2353, 2362-2363) are dead and dropped (D24).
        let ux = (petalx[(i / 2) as usize] - xmid) / dist;
        let uy = (petaly[(i / 2) as usize] - ymid) / dist;
        // find the third point other than p and q
        x3 = petalx[(i / 2) as usize] + ux * petalr[(i / 2) as usize];
        y3 = petaly[(i / 2) as usize] + uy * petalr[(i / 2) as usize];
        // FIND THE LINE POINTS BY THE ROTATION MATRIX
        // cw rotation matrix [cosX sinX; -sinX cosX]
        // cw rotation about (x,y) [ux*cosX + uy*sinX + x - x*cosX - y*sinX; -ux*sinX + uy*cosX + y + x*sinX - y*cosX]
        // ccw rotation matrix [cosX -sinX; sinX cosX]
        // ccw rotation about (x,y) [ux*cosX - uy*sinX + x - x*cosX + y*sinX; ux*sinX + uy*cosX + y - x*sinX - y*cosX]
        // LINE #1: (x1,y1) & (x_1,y_1)
        // rotate the vector around p = (x0,y0) in ccw by alpha degrees
        x_1 = x1 * alpha.cos() - y1 * alpha.sin() + x0 - x0 * alpha.cos() + y0 * alpha.sin();
        y_1 = x1 * alpha.sin() + y1 * alpha.cos() + y0 - x0 * alpha.sin() - y0 * alpha.cos();
        // add these to wedges list as lines in order
        wedges[(i * 16) as usize] = x0;
        wedges[(i * 16 + 1) as usize] = y0;
        wedges[(i * 16 + 2) as usize] = x_1;
        wedges[(i * 16 + 3) as usize] = y_1;
        // LINE #2: (x2,y2) & (x_2,y_2)
        // rotate the vector around q = (x1,y1) in cw by alpha degrees
        x_2 = x0 * alpha.cos() + y0 * alpha.sin() + x1 - x1 * alpha.cos() - y1 * alpha.sin();
        y_2 = -x0 * alpha.sin() + y0 * alpha.cos() + y1 + x1 * alpha.sin() - y1 * alpha.cos();
        // add these to wedges list as lines in order
        wedges[(i * 16 + 4) as usize] = x_2;
        wedges[(i * 16 + 5) as usize] = y_2;
        wedges[(i * 16 + 6) as usize] = x1;
        wedges[(i * 16 + 7) as usize] = y1;
        // vector from (petalx, petaly) to (x3,y3)
        // C: ux = x3 - petalx[i/2]; uy = y3 - petaly[i/2]; (acute.c:2362-
        // 2363) are dead stores and dropped (D24); the live part is
        // the tempx/tempy reset below.
        tempx = x3;
        tempy = y3;
        // LINE #3, #4, #5: (x3,y3) & (x_3,y_3)
        for j in 1..4 {
            // rotate the vector around (petalx,petaly) in cw by (60 - alpha)*j degrees
            x_3 = x3 * ((PI / 3.0 - alpha) * j as f64).cos()
                + y3 * ((PI / 3.0 - alpha) * j as f64).sin()
                + petalx[(i / 2) as usize]
                - petalx[(i / 2) as usize] * ((PI / 3.0 - alpha) * j as f64).cos()
                - petaly[(i / 2) as usize] * ((PI / 3.0 - alpha) * j as f64).sin();
            y_3 = -x3 * ((PI / 3.0 - alpha) * j as f64).sin()
                + y3 * ((PI / 3.0 - alpha) * j as f64).cos()
                + petaly[(i / 2) as usize]
                + petalx[(i / 2) as usize] * ((PI / 3.0 - alpha) * j as f64).sin()
                - petaly[(i / 2) as usize] * ((PI / 3.0 - alpha) * j as f64).cos();
            // add these to wedges list as lines in order
            wedges[(i * 16 + 8 + 4 * (j - 1)) as usize] = x_3;
            wedges[(i * 16 + 9 + 4 * (j - 1)) as usize] = y_3;
            wedges[(i * 16 + 10 + 4 * (j - 1)) as usize] = tempx;
            wedges[(i * 16 + 11 + 4 * (j - 1)) as usize] = tempy;
            tempx = x_3;
            tempy = y_3;
        }
        tempx = x3;
        tempy = y3;
        // LINE #6, #7, #8: (x3,y3) & (x_4,y_4)
        for j in 1..4 {
            // rotate the vector around (petalx,petaly) in ccw by (60 - alpha)*j degrees
            x_4 = x3 * ((PI / 3.0 - alpha) * j as f64).cos()
                - y3 * ((PI / 3.0 - alpha) * j as f64).sin()
                + petalx[(i / 2) as usize]
                - petalx[(i / 2) as usize] * ((PI / 3.0 - alpha) * j as f64).cos()
                + petaly[(i / 2) as usize] * ((PI / 3.0 - alpha) * j as f64).sin();
            y_4 = x3 * ((PI / 3.0 - alpha) * j as f64).sin()
                + y3 * ((PI / 3.0 - alpha) * j as f64).cos()
                + petaly[(i / 2) as usize]
                - petalx[(i / 2) as usize] * ((PI / 3.0 - alpha) * j as f64).sin()
                - petaly[(i / 2) as usize] * ((PI / 3.0 - alpha) * j as f64).cos();
            // add these to wedges list as lines in order
            wedges[(i * 16 + 20 + 4 * (j - 1)) as usize] = tempx;
            wedges[(i * 16 + 21 + 4 * (j - 1)) as usize] = tempy;
            wedges[(i * 16 + 22 + 4 * (j - 1)) as usize] = x_4;
            wedges[(i * 16 + 23 + 4 * (j - 1)) as usize] = y_4;
            tempx = x_4;
            tempy = y_4;
        }

        // IF IT IS THE FIRST ONE, FIND THE CONVEX POLYGON
        if i == 0 {
            // line1 & line2: p1
            // C: lineLineIntersection(x0, y0, x_1, y_1, x1, y1, x_2, y_2, p1);
            // (acute.c:2393)
            lineLineIntersection(x0, y0, x_1, y_1, x1, y1, x_2, y_2, &mut p1);
            if p1[0] == 1.0 {
                // #0
                initialConvexPoly[0] = p1[1];
                initialConvexPoly[1] = p1[2];
                // #1
                initialConvexPoly[2] = wedges[(i * 16 + 16) as usize];
                initialConvexPoly[3] = wedges[(i * 16 + 17) as usize];
                // #2
                initialConvexPoly[4] = wedges[(i * 16 + 12) as usize];
                initialConvexPoly[5] = wedges[(i * 16 + 13) as usize];
                // #3
                initialConvexPoly[6] = wedges[(i * 16 + 8) as usize];
                initialConvexPoly[7] = wedges[(i * 16 + 9) as usize];
                // #4
                initialConvexPoly[8] = x3;
                initialConvexPoly[9] = y3;
                // #5
                initialConvexPoly[10] = wedges[(i * 16 + 22) as usize];
                initialConvexPoly[11] = wedges[(i * 16 + 23) as usize];
                // #6
                initialConvexPoly[12] = wedges[(i * 16 + 26) as usize];
                initialConvexPoly[13] = wedges[(i * 16 + 27) as usize];
                // #7
                initialConvexPoly[14] = wedges[(i * 16 + 30) as usize];
                initialConvexPoly[15] = wedges[(i * 16 + 31) as usize];
            }
        }

        x0 = x1;
        y0 = y1;
        x1 = x2;
        y1 = y2;
    }

    // HALF PLANE INTERSECTION: START SPLITTING THE INITIAL POLYGON TO FIND FEASIBLE REGION
    if numpoints != 0 {
        // first intersect the opposite located ones
        s = (numpoints - 1) / 2 + 1;
        flag = 0;
        count = 0;
        i = 1;
        num = 8;
        for j in (0..32).step_by(4) {
            numpolypoints = halfPlaneIntersection(
                num,
                initialConvexPoly,
                wedges[(32 * s + j) as usize],
                wedges[(32 * s + 1 + j) as usize],
                wedges[(32 * s + 2 + j) as usize],
                wedges[(32 * s + 3 + j) as usize],
            );
            if numpolypoints == 0 {
                return 0;
            } else {
                num = numpolypoints;
            }
        }
        count += 1;
        while count < numpoints - 1 {
            for j in (0..32).step_by(4) {
                numpolypoints = halfPlaneIntersection(
                    num,
                    initialConvexPoly,
                    wedges[(32 * (i + s * flag) + j) as usize],
                    wedges[(32 * (i + s * flag) + 1 + j) as usize],
                    wedges[(32 * (i + s * flag) + 2 + j) as usize],
                    wedges[(32 * (i + s * flag) + 3 + j) as usize],
                );
                if numpolypoints == 0 {
                    return 0;
                } else {
                    num = numpolypoints;
                }
            }
            i += flag;
            flag = (flag + 1) % 2;
            count += 1;
        }
        // IF THERE IS A FEASIBLE INTERSECTION POLYGON, FIND ITS CENTROID AS THE NEW LOCATION
        findPolyCentroid(numpolypoints, initialConvexPoly, newloc);

        if b.fixedarea {
            // C's fixedarea arm is entirely commented out (the
            // testTriangleAngleArea block, acute.c:2451-2464): the
            // fall-through to the final return 0 mirrors C.
        } else {
            return 1;
        }
    }

    // freeing done by acute memory pool (acute.c:2475-2479)
    0
}

/// C: getWedgeIntersectionWithMaxAngle (acute.c:2490-2912; banner at
/// acute.c:2483-2489, shared with the Without variant). The max-angle
/// version of the wedge intersection: ten wedge lines per star point
/// (the two min-angle lines, the max-angle lines of acute.c:2656-2688
/// and up to eight petal-chord lines whose count and spacing follow
/// the petal-center angle alpha = 2*maxangle + minangle - 180
/// (acute.c:2617-2643)), then - when a maximum angle is set - a
/// testTriangleAngle scan of the candidate against every star edge
/// (acute.c:2846-2859) followed by the weighted-alternative loop of
/// acute.c:2862-2893. C's mesh* m parameter carries only the acute
/// pool here (the resize of acute.c:2522 and the buffer pulls of
/// acute.c:2524-2528); the mirror passes the pool (D23).
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:2490)
#[allow(clippy::needless_late_init)]
// C-parity: C's declaration
// blocks (acute.c:2492-2520) keep the declaration-then-assignment
// shape
#[allow(clippy::float_cmp)]
// C-parity: the == 1.0 intersection flag
// tests verbatim (acute.c:2700, 2716, 2736, 2761) and b->goodangle ==
// 1.0 (acute.c:2538)
#[allow(clippy::if_same_then_else)] // C-parity: the skip chain's
// identical continue arms verbatim (acute.c:2802-2807, 2823-2828)
#[allow(clippy::assign_op_pattern)] // C-parity: the newloc stores keep
                                    // C's x = x + y shape verbatim (acute.c:2870-2876)
pub(crate) fn getWedgeIntersectionWithMaxAngle(
    p: &mut AcutePool,
    b: &Behavior,
    numpoints: i32,
    points: &[f64],
    newloc: &mut [f64],
) -> i32 {
    // C's declaration block (acute.c:2492-2520); same D24 discipline
    // as the Without variant. Additionally: C's initializers
    // `REAL line345 = 4.0, line789 = 4.0;` (acute.c:2512) are dead
    // stores (every branch of the how-many-points tree overwrites
    // both before any read) and are dropped (D24).
    let mut x0: f64;
    let mut y0: f64;
    let mut x1: f64;
    let mut y1: f64;
    let mut x2: f64;
    let mut y2: f64;
    let mut x01: f64;
    let mut y01: f64;
    let mut d01: f64;
    let mut xmid: f64;
    let mut ymid: f64;
    let mut dist: f64;
    let mut x3: f64;
    let mut y3: f64;
    let mut x_1: f64;
    let mut y_1: f64;
    let mut x_2: f64;
    let mut y_2: f64;
    let mut x_3: f64;
    let mut y_3: f64;
    let mut x_4: f64;
    let mut y_4: f64;
    let mut tempx: f64;
    let mut tempy: f64;
    let mut x_5: f64;
    let mut y_5: f64;
    let mut x_6: f64;
    let mut y_6: f64;
    let mut alpha: f64;
    // C: REAL p1[3], p2[3], p3[3], p4[3]; (acute.c:2509): zero-init
    // (D24); the lineLineIntersection calls write them before the
    // reads of the i == 0 switch.
    let mut p1 = [0.0f64; 3];
    let mut p2 = [0.0f64; 3];
    let mut p3 = [0.0f64; 3];
    let mut p4 = [0.0f64; 3];
    // C: int numpolypoints; (acute.c:2510): zero-init (D24), same
    // guard domination as in the Without variant.
    let mut numpolypoints = 0;
    // C: int howManyPoints; (acute.c:2511): keeps the number of points
    // used for representing the wedge. Zero-init (D24): the star loop
    // writes it before the tail's read (the numpoints != 0 guard
    // dominates both).
    let mut howManyPoints = 0;
    // flag keeping which line to skip or construct (acute.c:2512); the
    // dead = 4.0 initializers are dropped (D24).
    let mut line345: f64;
    let mut line789: f64;
    // C: int numBadTriangle; (acute.c:2514)
    let mut numBadTriangle: i32;
    // C: int i, j, k; int s, flag, count, num; int e, n;
    // REAL weight; (acute.c:2515-2518)
    let mut i: i32;
    let s: i32;
    let mut flag: i32;
    let mut count: i32;
    let mut num: i32;
    // C's int e (acute.c:2517) is dropped: the weighted-alternative
    // loop produces its own binding (for e, acute.c:2865), so the
    // declaration is a dead store (D24).
    let n: i32;
    let mut weight: f64;

    let petalcenterconstant: f64;
    let petalradiusconstant: f64;

    // C: acutepool_resize(numpoints, m->acute_mem); (acute.c:2522)
    acutepool_resize(numpoints, p);

    // C: petalx = m->acute_mem->petalx; ... initialConvexPoly =
    // m->acute_mem->initialpoly; (acute.c:2524-2528)
    let (petalx, petaly, petalr, wedges, initialConvexPoly) = p.wedge_scratch();

    // C: x0 = points[2*numpoints-4]; ... (acute.c:2530-2533)
    x0 = points[(2 * numpoints - 4) as usize];
    y0 = points[(2 * numpoints - 3) as usize];
    x1 = points[(2 * numpoints - 2) as usize];
    y1 = points[(2 * numpoints - 1) as usize];

    // minimum angle
    // C: alpha = b->minangle*PI/180.0; (acute.c:2536) is a dead
    // store (reassigned at acute.c:2547 inside the petal loop
    // before any read) and is dropped (D24).
    // initialize the constants
    // C: if (b->goodangle == 1.0) { ... } else { ... } (acute.c:2538-2544)
    if b.goodangle == 1.0 {
        petalcenterconstant = 0.0;
        petalradiusconstant = 0.0;
    } else {
        petalcenterconstant = 0.5 / (b.minangle * PI / 180.0).tan();
        petalradiusconstant = 0.5 / (b.minangle * PI / 180.0).sin();
    }
    // C: for(i = 0; i < numpoints*2; i= i+2){ (acute.c:2545)
    for i in (0..numpoints * 2).step_by(2) {
        // minimum angle
        // C: alpha = b->minangle*PI/180.0; (acute.c:2547)
        alpha = b.minangle * PI / 180.0;
        // go to the next point
        x2 = points[i as usize];
        y2 = points[i as usize + 1];

        x01 = x1 - x0;
        y01 = y1 - y0;
        d01 = (x01 * x01 + y01 * y01).sqrt();
        // find the petal of each edge 01;
        petalx[(i / 2) as usize] = x0 + 0.5 * x01 - petalcenterconstant * y01;
        petaly[(i / 2) as usize] = y0 + 0.5 * y01 + petalcenterconstant * x01;
        petalr[(i / 2) as usize] = petalradiusconstant * d01;
        petalx[(numpoints + i / 2) as usize] = petalx[(i / 2) as usize];
        petaly[(numpoints + i / 2) as usize] = petaly[(i / 2) as usize];
        petalr[(numpoints + i / 2) as usize] = petalr[(i / 2) as usize];

        // FIRST FIND THE HALF-PLANE POINTS FOR EACH PETAL
        xmid = (x0 + x1) / 2.0; // mid point of pq
        ymid = (y0 + y1) / 2.0;

        // distance between xmid and petal center
        dist = ((petalx[(i / 2) as usize] - xmid) * (petalx[(i / 2) as usize] - xmid)
            + (petaly[(i / 2) as usize] - ymid) * (petaly[(i / 2) as usize] - ymid))
            .sqrt();
        // find the unit vector goes from mid point to petal center
        // C: ux = ...; uy = ...; (acute.c:2580-2581) are the live stores
        // (read immediately below); the later re-stores of the LINE#1,
        // LINE#2, petal-vector, LINE#6 and LINE#10 comment blocks
        // (acute.c:2592-2593, 2603-2604, 2622-2623, 2658-2659,
        // 2682-2683) are dead and dropped (D24).
        let ux = (petalx[(i / 2) as usize] - xmid) / dist;
        let uy = (petaly[(i / 2) as usize] - ymid) / dist;
        // find the third point other than p and q
        x3 = petalx[(i / 2) as usize] + ux * petalr[(i / 2) as usize];
        y3 = petaly[(i / 2) as usize] + uy * petalr[(i / 2) as usize];
        // FIND THE LINE POINTS BY THE ROTATION MATRIX
        // cw rotation matrix [cosX sinX; -sinX cosX]
        // cw rotation about (x,y) [ux*cosX + uy*sinX + x - x*cosX - y*sinX; -ux*sinX + uy*cosX + y + x*sinX - y*cosX]
        // ccw rotation matrix [cosX -sinX; sinX cosX]
        // ccw rotation about (x,y) [ux*cosX - uy*sinX + x - x*cosX + y*sinX; ux*sinX + uy*cosX + y - x*sinX - y*cosX]
        // LINE #1: (x1,y1) & (x_1,y_1)
        // rotate the vector around p = (x0,y0) in ccw by alpha degrees
        x_1 = x1 * alpha.cos() - y1 * alpha.sin() + x0 - x0 * alpha.cos() + y0 * alpha.sin();
        y_1 = x1 * alpha.sin() + y1 * alpha.cos() + y0 - x0 * alpha.sin() - y0 * alpha.cos();
        // add these to wedges list as lines in order
        wedges[(i * 20) as usize] = x0;
        wedges[(i * 20 + 1) as usize] = y0;
        wedges[(i * 20 + 2) as usize] = x_1;
        wedges[(i * 20 + 3) as usize] = y_1;
        // LINE #2: (x2,y2) & (x_2,y_2)
        // rotate the vector around q = (x1,y1) in cw by alpha degrees
        x_2 = x0 * alpha.cos() + y0 * alpha.sin() + x1 - x1 * alpha.cos() - y1 * alpha.sin();
        y_2 = -x0 * alpha.sin() + y0 * alpha.cos() + y1 + x1 * alpha.sin() - y1 * alpha.cos();
        // add these to wedges list as lines in order
        wedges[(i * 20 + 4) as usize] = x_2;
        wedges[(i * 20 + 5) as usize] = y_2;
        wedges[(i * 20 + 6) as usize] = x1;
        wedges[(i * 20 + 7) as usize] = y1;
        // vector from (petalx, petaly) to (x3,y3)
        // C's ux/uy stores here (acute.c:2622-2623) are dead and
        // dropped (D24); the live part is the tempx/tempy reset.
        tempx = x3;
        tempy = y3;

        // DETERMINE HOW MANY POINTS TO USE ACCORDING TO THE MINANGLE-MAXANGLE COMBINATION
        // petal center angle
        // C: alpha = (2.0*b->maxangle + b->minangle - 180.0); (acute.c:2617)
        alpha = 2.0 * b.maxangle + b.minangle - 180.0;
        if alpha <= 0.0 {
            // when only angle lines needed
            // 4 point case
            howManyPoints = 4;
            line345 = 1.0;
            line789 = 1.0;
        } else if alpha <= 5.0 {
            // when only angle lines plus two other lines are needed
            // 6 point case
            howManyPoints = 6;
            line345 = 2.0;
            line789 = 2.0;
        } else if alpha <= 10.0 {
            // when we need more lines
            // 8 point case
            howManyPoints = 8;
            line345 = 3.0;
            line789 = 3.0;
        } else {
            // when we have a big wedge
            // 10 point case
            howManyPoints = 10;
            line345 = 4.0;
            line789 = 4.0;
        }
        // C: alpha = alpha*PI/180.0; (acute.c:2643)
        alpha = alpha * PI / 180.0;
        // LINE #3, #4, #5: (x3,y3) & (x_3,y_3)
        // C: for(j = 1; j < line345; j++){ if(line345 == 1)
        // continue; ... } (acute.c:2645-2647): j is int, line345 REAL;
        // C's continue jumps to the for increment. The continue arm is
        // unreachable (the guard j < line345 fails when line345 == 1).
        let mut j = 1;
        while (j as f64) < line345 {
            if line345 == 1.0 {
                j += 1;
                continue;
            }
            // rotate the vector around (petalx,petaly) in cw by (alpha/3.0)*j degrees
            // (C's comment says (alpha/3.0)*j; C's code uses
            // (alpha/(line345-1.0))*j, acute.c:2649-2650, and the
            // mirror follows the code)
            x_3 = x3 * ((alpha / (line345 - 1.0)) * j as f64).cos()
                + y3 * ((alpha / (line345 - 1.0)) * j as f64).sin()
                + petalx[(i / 2) as usize]
                - petalx[(i / 2) as usize] * ((alpha / (line345 - 1.0)) * j as f64).cos()
                - petaly[(i / 2) as usize] * ((alpha / (line345 - 1.0)) * j as f64).sin();
            y_3 = -x3 * ((alpha / (line345 - 1.0)) * j as f64).sin()
                + y3 * ((alpha / (line345 - 1.0)) * j as f64).cos()
                + petaly[(i / 2) as usize]
                + petalx[(i / 2) as usize] * ((alpha / (line345 - 1.0)) * j as f64).sin()
                - petaly[(i / 2) as usize] * ((alpha / (line345 - 1.0)) * j as f64).cos();
            // add these to wedges list as lines in order
            wedges[(i * 20 + 8 + 4 * (j - 1)) as usize] = x_3;
            wedges[(i * 20 + 9 + 4 * (j - 1)) as usize] = y_3;
            wedges[(i * 20 + 10 + 4 * (j - 1)) as usize] = tempx;
            wedges[(i * 20 + 11 + 4 * (j - 1)) as usize] = tempy;
            tempx = x_3;
            tempy = y_3;
            j += 1;
        }
        // LINE #6: (x2,y2) & (x_3,y_3)
        // vector from q to p (C's ux/uy stores, acute.c:2658-2659, are
        // dead and dropped, D24)
        // rotate the vector around q = (x1,y1) in cw by alpha degrees
        // (C rotates by b->maxangle*PI/180.0 here, acute.c:2661-2662)
        x_5 = x0 * (b.maxangle * PI / 180.0).cos() + y0 * (b.maxangle * PI / 180.0).sin() + x1
            - x1 * (b.maxangle * PI / 180.0).cos()
            - y1 * (b.maxangle * PI / 180.0).sin();
        y_5 = -x0 * (b.maxangle * PI / 180.0).sin()
            + y0 * (b.maxangle * PI / 180.0).cos()
            + y1
            + x1 * (b.maxangle * PI / 180.0).sin()
            - y1 * (b.maxangle * PI / 180.0).cos();
        // add these to wedges list as lines in order
        wedges[(i * 20 + 20) as usize] = x1;
        wedges[(i * 20 + 21) as usize] = y1;
        wedges[(i * 20 + 22) as usize] = x_5;
        wedges[(i * 20 + 23) as usize] = y_5;

        tempx = x3;
        tempy = y3;
        // LINE #7, #8, #9: (x3,y3) & (x_4,y_4)
        // C: for(j = 1; j < line789; j++){ if(line789 == 1)
        // continue; ... } (acute.c:2668-2670), same shape as the
        // LINE#3-5 loop.
        let mut j = 1;
        while (j as f64) < line789 {
            if line789 == 1.0 {
                j += 1;
                continue;
            }
            // rotate the vector around (petalx,petaly) in ccw by (alpha/3.0)*j degrees
            // (C's code uses (alpha/(line789-1.0))*j, acute.c:2672-2673)
            x_4 = x3 * ((alpha / (line789 - 1.0)) * j as f64).cos()
                - y3 * ((alpha / (line789 - 1.0)) * j as f64).sin()
                + petalx[(i / 2) as usize]
                - petalx[(i / 2) as usize] * ((alpha / (line789 - 1.0)) * j as f64).cos()
                + petaly[(i / 2) as usize] * ((alpha / (line789 - 1.0)) * j as f64).sin();
            y_4 = x3 * ((alpha / (line789 - 1.0)) * j as f64).sin()
                + y3 * ((alpha / (line789 - 1.0)) * j as f64).cos()
                + petaly[(i / 2) as usize]
                - petalx[(i / 2) as usize] * ((alpha / (line789 - 1.0)) * j as f64).sin()
                - petaly[(i / 2) as usize] * ((alpha / (line789 - 1.0)) * j as f64).cos();
            // add these to wedges list as lines in order
            wedges[(i * 20 + 24 + 4 * (j - 1)) as usize] = tempx;
            wedges[(i * 20 + 25 + 4 * (j - 1)) as usize] = tempy;
            wedges[(i * 20 + 26 + 4 * (j - 1)) as usize] = x_4;
            wedges[(i * 20 + 27 + 4 * (j - 1)) as usize] = y_4;
            tempx = x_4;
            tempy = y_4;
            j += 1;
        }
        // LINE #10: (x1,y1) & (x_3,y_3)
        // vector from p to q (C's ux/uy stores, acute.c:2682-2683, are
        // dead and dropped, D24)
        // rotate the vector around p = (x0,y0) in ccw by alpha degrees
        // (C rotates by b->maxangle*PI/180.0 here, acute.c:2685-2686)
        x_6 = x1 * (b.maxangle * PI / 180.0).cos() - y1 * (b.maxangle * PI / 180.0).sin() + x0
            - x0 * (b.maxangle * PI / 180.0).cos()
            + y0 * (b.maxangle * PI / 180.0).sin();
        y_6 = x1 * (b.maxangle * PI / 180.0).sin() + y1 * (b.maxangle * PI / 180.0).cos() + y0
            - x0 * (b.maxangle * PI / 180.0).sin()
            - y0 * (b.maxangle * PI / 180.0).cos();
        // add these to wedges list as lines in order
        wedges[(i * 20 + 36) as usize] = x_6;
        wedges[(i * 20 + 37) as usize] = y_6;
        wedges[(i * 20 + 38) as usize] = x0;
        wedges[(i * 20 + 39) as usize] = y0;

        // IF IT IS THE FIRST ONE, FIND THE CONVEX POLYGON
        if i == 0 {
            // C: switch(howManyPoints){ ... } (acute.c:2693-2784)
            match howManyPoints {
                4 => {
                    // line1 & line2 & line3 & line4
                    lineLineIntersection(x0, y0, x_1, y_1, x1, y1, x_2, y_2, &mut p1);
                    lineLineIntersection(x0, y0, x_1, y_1, x1, y1, x_5, y_5, &mut p2);
                    lineLineIntersection(x0, y0, x_6, y_6, x1, y1, x_5, y_5, &mut p3);
                    lineLineIntersection(x0, y0, x_6, y_6, x1, y1, x_2, y_2, &mut p4);
                    if p1[0] == 1.0 && p2[0] == 1.0 && p3[0] == 1.0 && p4[0] == 1.0 {
                        // #0
                        initialConvexPoly[0] = p1[1];
                        initialConvexPoly[1] = p1[2];
                        // #1
                        initialConvexPoly[2] = p2[1];
                        initialConvexPoly[3] = p2[2];
                        // #2
                        initialConvexPoly[4] = p3[1];
                        initialConvexPoly[5] = p3[2];
                        // #3
                        initialConvexPoly[6] = p4[1];
                        initialConvexPoly[7] = p4[2];
                    }
                }
                6 => {
                    // line1 & line2 & line3
                    lineLineIntersection(x0, y0, x_1, y_1, x1, y1, x_2, y_2, &mut p1);
                    lineLineIntersection(x0, y0, x_1, y_1, x1, y1, x_5, y_5, &mut p2);
                    lineLineIntersection(x0, y0, x_6, y_6, x1, y1, x_2, y_2, &mut p3);
                    if p1[0] == 1.0 && p2[0] == 1.0 && p3[0] == 1.0 {
                        // #0
                        initialConvexPoly[0] = p1[1];
                        initialConvexPoly[1] = p1[2];
                        // #1
                        initialConvexPoly[2] = p2[1];
                        initialConvexPoly[3] = p2[2];
                        // #2
                        initialConvexPoly[4] = wedges[(i * 20 + 8) as usize];
                        initialConvexPoly[5] = wedges[(i * 20 + 9) as usize];
                        // #3
                        initialConvexPoly[6] = x3;
                        initialConvexPoly[7] = y3;
                        // #4
                        initialConvexPoly[8] = wedges[(i * 20 + 26) as usize];
                        initialConvexPoly[9] = wedges[(i * 20 + 27) as usize];
                        // #5
                        initialConvexPoly[10] = p3[1];
                        initialConvexPoly[11] = p3[2];
                    }
                }
                8 => {
                    // line1 & line2: p1
                    lineLineIntersection(x0, y0, x_1, y_1, x1, y1, x_2, y_2, &mut p1);
                    lineLineIntersection(x0, y0, x_1, y_1, x1, y1, x_5, y_5, &mut p2);
                    lineLineIntersection(x0, y0, x_6, y_6, x1, y1, x_2, y_2, &mut p3);
                    if p1[0] == 1.0 && p2[0] == 1.0 && p3[0] == 1.0 {
                        // #0
                        initialConvexPoly[0] = p1[1];
                        initialConvexPoly[1] = p1[2];
                        // #1
                        initialConvexPoly[2] = p2[1];
                        initialConvexPoly[3] = p2[2];
                        // #2
                        initialConvexPoly[4] = wedges[(i * 20 + 12) as usize];
                        initialConvexPoly[5] = wedges[(i * 20 + 13) as usize];
                        // #3
                        initialConvexPoly[6] = wedges[(i * 20 + 8) as usize];
                        initialConvexPoly[7] = wedges[(i * 20 + 9) as usize];
                        // #4
                        initialConvexPoly[8] = x3;
                        initialConvexPoly[9] = y3;
                        // #5
                        initialConvexPoly[10] = wedges[(i * 20 + 26) as usize];
                        initialConvexPoly[11] = wedges[(i * 20 + 27) as usize];
                        // #6
                        initialConvexPoly[12] = wedges[(i * 20 + 30) as usize];
                        initialConvexPoly[13] = wedges[(i * 20 + 31) as usize];
                        // #7
                        initialConvexPoly[14] = p3[1];
                        initialConvexPoly[15] = p3[2];
                    }
                }
                10 => {
                    // line1 & line2: p1
                    lineLineIntersection(x0, y0, x_1, y_1, x1, y1, x_2, y_2, &mut p1);
                    lineLineIntersection(x0, y0, x_1, y_1, x1, y1, x_5, y_5, &mut p2);
                    lineLineIntersection(x0, y0, x_6, y_6, x1, y1, x_2, y_2, &mut p3);
                    if p1[0] == 1.0 && p2[0] == 1.0 && p3[0] == 1.0 {
                        // #0
                        initialConvexPoly[0] = p1[1];
                        initialConvexPoly[1] = p1[2];
                        // #1
                        initialConvexPoly[2] = p2[1];
                        initialConvexPoly[3] = p2[2];
                        // #2
                        initialConvexPoly[4] = wedges[(i * 20 + 16) as usize];
                        initialConvexPoly[5] = wedges[(i * 20 + 17) as usize];
                        // #3
                        initialConvexPoly[6] = wedges[(i * 20 + 12) as usize];
                        initialConvexPoly[7] = wedges[(i * 20 + 13) as usize];
                        // #4
                        initialConvexPoly[8] = wedges[(i * 20 + 8) as usize];
                        initialConvexPoly[9] = wedges[(i * 20 + 9) as usize];
                        // #5
                        initialConvexPoly[10] = x3;
                        initialConvexPoly[11] = y3;
                        // #6
                        initialConvexPoly[12] = wedges[(i * 20 + 28) as usize];
                        initialConvexPoly[13] = wedges[(i * 20 + 29) as usize];
                        // #7
                        initialConvexPoly[14] = wedges[(i * 20 + 32) as usize];
                        initialConvexPoly[15] = wedges[(i * 20 + 33) as usize];
                        // #8
                        initialConvexPoly[16] = wedges[(i * 20 + 34) as usize];
                        initialConvexPoly[17] = wedges[(i * 20 + 35) as usize];
                        // #9
                        initialConvexPoly[18] = p3[1];
                        initialConvexPoly[19] = p3[2];
                    }
                }
                // C's switch has no default arm (acute.c:2693-2784);
                // the branch tree above assigns 4, 6, 8 or 10 on every
                // path.
                _ => unreachable!("howManyPoints is 4, 6, 8 or 10 (acute.c:2618-2642)"),
            }
        }

        x0 = x1;
        y0 = y1;
        x1 = x2;
        y1 = y2;
    }
    // HALF PLANE INTERSECTION: START SPLITTING THE INITIAL POLYGON TO FIND FEASIBLE REGION
    if numpoints != 0 {
        // first intersect the opposite located ones
        s = (numpoints - 1) / 2 + 1;
        flag = 0;
        count = 0;
        i = 1;
        // C: num = howManyPoints; (acute.c:2799)
        num = howManyPoints;
        for j in (0..40).step_by(4) {
            // in order to skip non-existent lines
            if howManyPoints == 4 && (j == 8 || j == 12 || j == 16 || j == 24 || j == 28 || j == 32)
            {
                continue;
            } else if howManyPoints == 6 && (j == 12 || j == 16 || j == 28 || j == 32) {
                continue;
            } else if howManyPoints == 8 && (j == 16 || j == 32) {
                continue;
            }
            numpolypoints = halfPlaneIntersection(
                num,
                initialConvexPoly,
                wedges[(40 * s + j) as usize],
                wedges[(40 * s + 1 + j) as usize],
                wedges[(40 * s + 2 + j) as usize],
                wedges[(40 * s + 3 + j) as usize],
            );
            if numpolypoints == 0 {
                return 0;
            } else {
                num = numpolypoints;
            }
        }
        count += 1;
        while count < numpoints - 1 {
            for j in (0..40).step_by(4) {
                // in order to skip non-existent lines
                if howManyPoints == 4
                    && (j == 8 || j == 12 || j == 16 || j == 24 || j == 28 || j == 32)
                {
                    continue;
                } else if howManyPoints == 6 && (j == 12 || j == 16 || j == 28 || j == 32) {
                    continue;
                } else if howManyPoints == 8 && (j == 16 || j == 32) {
                    continue;
                }
                numpolypoints = halfPlaneIntersection(
                    num,
                    initialConvexPoly,
                    wedges[(40 * (i + s * flag) + j) as usize],
                    wedges[(40 * (i + s * flag) + 1 + j) as usize],
                    wedges[(40 * (i + s * flag) + 2 + j) as usize],
                    wedges[(40 * (i + s * flag) + 3 + j) as usize],
                );
                if numpolypoints == 0 {
                    return 0;
                } else {
                    num = numpolypoints;
                }
            }
            i += flag;
            flag = (flag + 1) % 2;
            count += 1;
        }
        // IF THERE IS A FEASIBLE INTERSECTION POLYGON, FIND ITS CENTROID AS THE NEW LOCATION
        findPolyCentroid(numpolypoints, initialConvexPoly, newloc);

        // C: if(b->maxangle != 0.0){ ... }else{ ... } (acute.c:2846-2902)
        if b.maxangle != 0.0 {
            numBadTriangle = 0;
            for j in (0..numpoints * 2 - 2).step_by(2) {
                if testTriangleAngle(
                    b,
                    newloc[0],
                    newloc[1],
                    points[j as usize],
                    points[j as usize + 1],
                    points[j as usize + 2],
                    points[j as usize + 3],
                ) != 0
                {
                    numBadTriangle += 1;
                }
            }
            if testTriangleAngle(
                b,
                newloc[0],
                newloc[1],
                points[0],
                points[1],
                points[(numpoints * 2 - 2) as usize],
                points[(numpoints * 2 - 1) as usize],
            ) != 0
            {
                numBadTriangle += 1;
            }

            if numBadTriangle == 0 {
                return 1;
            }

            // C: n = (numpoints <= 2) ? 20 : 30; (acute.c:2862)
            n = if numpoints <= 2 { 20 } else { 30 };
            // try points other than centroid
            // C: for(k = 0; k < 2*numpoints; k = k+2){ ... } (acute.c:2864)
            for k in (0..2 * numpoints).step_by(2) {
                // C: for(e = 1; e < n; e = e + 1){ ... } (acute.c:2865)
                for e in 1..n {
                    newloc[0] = 0.0;
                    newloc[1] = 0.0;
                    // C reuses i (the halfplane-walk local, dead since
                    // the while loop); the loop binding shadows it.
                    for i in (0..2 * numpoints).step_by(2) {
                        // C: weight = 1.0/numpoints; (acute.c:2868)
                        weight = 1.0 / numpoints as f64;
                        if i == k {
                            // C: newloc[0] = newloc[0] + 0.1*e*weight*points[i];
                            // (acute.c:2870-2871)
                            newloc[0] = newloc[0] + 0.1 * e as f64 * weight * points[i as usize];
                            newloc[1] =
                                newloc[1] + 0.1 * e as f64 * weight * points[i as usize + 1];
                        } else {
                            // C: weight = (1.0 - 0.1*e*weight)/(double)(numpoints - 1.0);
                            // (acute.c:2873)
                            weight = (1.0 - 0.1 * e as f64 * weight) / (numpoints as f64 - 1.0);
                            // C: newloc[0] = newloc[0] + weight*points[i]; (acute.c:2875-2876)
                            newloc[0] = newloc[0] + weight * points[i as usize];
                            newloc[1] = newloc[1] + weight * points[i as usize + 1];
                        }
                    }
                    numBadTriangle = 0;
                    for j in (0..numpoints * 2 - 2).step_by(2) {
                        if testTriangleAngle(
                            b,
                            newloc[0],
                            newloc[1],
                            points[j as usize],
                            points[j as usize + 1],
                            points[j as usize + 2],
                            points[j as usize + 3],
                        ) != 0
                        {
                            numBadTriangle += 1;
                        }
                    }
                    if testTriangleAngle(
                        b,
                        newloc[0],
                        newloc[1],
                        points[0],
                        points[1],
                        points[(numpoints * 2 - 2) as usize],
                        points[(numpoints * 2 - 1) as usize],
                    ) != 0
                    {
                        numBadTriangle += 1;
                    }

                    if numBadTriangle == 0 {
                        return 1;
                    }
                }
            }
        } else {
            return 1;
        }
    }

    // freeing done by acute memory pool (acute.c:2905-2911)
    0
}

/// C: polygonAngles (acute.c:2918-2935; banner at acute.c:2914-2917: "Return
/// 0 if the polygon has angles greater than 2*minangle"). Walks the
/// star ring and hands every consecutive corner triple to
/// testPolygonAngle; returns 1 as soon as one corner fails, 0 when
/// all angles are valid.
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:2918)
pub(crate) fn polygonAngles(b: &Behavior, numpoints: i32, points: &[f64]) -> i32 {
    for i in 0..numpoints {
        if i == numpoints - 1 {
            if testPolygonAngle(
                b,
                points[(i * 2) as usize],
                points[(i * 2 + 1) as usize],
                points[0],
                points[1],
                points[2],
                points[3],
            ) != 0
            {
                return 1; // one of the inner angles is less than required
            }
        } else if i == numpoints - 2 {
            if testPolygonAngle(
                b,
                points[(i * 2) as usize],
                points[(i * 2 + 1) as usize],
                points[((i + 1) * 2) as usize],
                points[((i + 1) * 2 + 1) as usize],
                points[0],
                points[1],
            ) != 0
            {
                return 1; // one of the inner angles is less than required
            }
        } else {
            if testPolygonAngle(
                b,
                points[(i * 2) as usize],
                points[(i * 2 + 1) as usize],
                points[((i + 1) * 2) as usize],
                points[((i + 1) * 2 + 1) as usize],
                points[((i + 2) * 2) as usize],
                points[((i + 2) * 2 + 1) as usize],
            ) != 0
            {
                return 1; // one of the inner angles is less than required
            }
        }
    }
    0 // all angles are valid
}

/// C: testPolygonAngle (acute.c:2944-2977; banner at acute.c:2938-2943:
/// "Given three coordinates of a polygon, tests to see if it
/// satisfies the minimum angle condition for relocation; Returns 1, if
/// it is a BAD polygon corner, returns 0 if it is a GOOD polygon
/// corner"). The tested angle is the one at the middle vertex; it is
/// bad when acos of the law-of-cosines value drops below 2*minangle
/// (the comparison against 2*acos(sqrt(b->goodangle)) of acute.c:2972).
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:2944)
#[allow(clippy::too_many_arguments)] // C-parity: the seven-parameter C
// signature (acute.c:2944-2948)
#[allow(clippy::needless_late_init)] // C-parity: the declaration block
                                     // (acute.c:2950-2953) keeps the declaration-then-assignment shape
pub(crate) fn testPolygonAngle(
    b: &Behavior,
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    x3: f64,
    y3: f64,
) -> i32 {
    // variables keeping the distance values for the edges
    let dx12: f64;
    let dy12: f64;
    let dx23: f64;
    let dy23: f64;
    let dx31: f64;
    let dy31: f64;
    let dist12: f64;
    let dist23: f64;
    let dist31: f64;
    // in order to check minimum angle condition
    let cosAngle: f64;

    // calculate the side lengths
    dx12 = x1 - x2;
    dy12 = y1 - y2;
    dx23 = x2 - x3;
    dy23 = y2 - y3;
    dx31 = x3 - x1;
    dy31 = y3 - y1;
    // calculate the squares of the side lentghs
    dist12 = dx12 * dx12 + dy12 * dy12;
    dist23 = dx23 * dx23 + dy23 * dy23;
    dist31 = dx31 * dx31 + dy31 * dy31;

    // calculate cosine of largest angle
    cosAngle = (dist12 + dist23 - dist31) / (2.0 * dist12.sqrt() * dist23.sqrt());
    /* Check whether the angle is smaller than permitted which is 2*minangle!!! */
    if cosAngle.acos() < 2.0 * b.goodangle.sqrt().acos() {
        return 1; // it is a BAD triangle
    }
    0 // it is a GOOD triangle
}

/// C: lineLineIntersection (acute.c:2983-3022). Given four points
/// representing two lines, write the intersection into the out-array
/// p: p[0] is 1.0 when the lines cross in one point (p[1], p[2] then
/// carry the coordinates, acute.c:3016-3017) and 0.0 when the lines are
/// coincident (acute.c:3005-3006) or parallel (acute.c:3009-3010).
/// Referenced to http://local.wasp.uwa.edu.au/~pbourke/geometry/
/// (acute.c:2981).
///
/// The C call sites sit in the getWedgeIntersection family (first
/// acute.c:2393; more at acute.c:2696-2759), landed with this PR
/// (stage-1 acute B).
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:2983)
#[allow(clippy::too_many_arguments)] // C-parity: the nine-parameter C
// signature (acute.c:2983-2987)
#[allow(clippy::needless_late_init)]
// C-parity: the REAL declaration
// block (acute.c:2998) keeps C's
// declaration-then-assignment shape
#[allow(clippy::if_same_then_else)] // C-parity: C keeps the two distinct
                                    // degenerate branches (coincident, acute.c:3005-3006; parallel,
                                    // acute.c:3009-3010); merging them would change statement identity
pub(crate) fn lineLineIntersection(
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    x3: f64,
    y3: f64,
    x4: f64,
    y4: f64,
    p: &mut [f64],
) {
    // x1,y1  P1 coordinates (point of line 1)
    // x2,y2  P2 coordinates (point of line 1)
    // x3,y3  P3 coordinates (point of line 2)
    // x4,y4  P4 coordinates (point of line 2)
    // p[1],p[2]   intersection coordinates
    //
    // This function returns a pointer array which first index indicates
    // weather they intersect on one point or not, followed by coordinate
    // pairs. (acute.c:2989-2996)

    let mut u_a: f64;
    let u_b: f64;
    let denom: f64;

    // calculate denominator first (acute.c:3000)
    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    u_a = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
    u_b = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);
    // if denominator and numerator equal to zero, lines are coincident
    // (acute.c:3004)
    if (denom - 0.0).abs() < compConst
        && ((u_b - 0.0).abs() < compConst && (u_a - 0.0).abs() < compConst)
    {
        p[0] = 0.0;
    }
    // if denominator equals to zero, lines are parallel (acute.c:3008)
    else if (denom - 0.0).abs() < compConst {
        p[0] = 0.0;
    } else {
        p[0] = 1.0;
        u_a /= denom;
        // C: u_b = u_b/denom; (acute.c:3015) is a dead store (the
        // divided u_b is never read; the branch writes p[1], p[2] from
        // u_a only) and is dropped (D20).
        p[1] = x1 + u_a * (x2 - x1); // not the intersection point
        p[2] = y1 + u_a * (y2 - y1);
    }
}

/// C: halfPlaneIntersection (acute.c:3032-3087). Returns the convex
/// polygon which is the intersection of the given convex polygon with
/// the halfplane on the left side (regarding the directional vector)
/// of the given line: the kept piece is written back into convexPoly
/// (acute.c:3074-3075) and the return value is its vertex count (the
/// input count when the split fails, acute.c:3049-3050; 0 when no piece
/// lies on the left side).
/// www.mathematik.uni-ulm.de/stochastik/lehre/ws03_04/rt/Geometry2D.ps
/// (acute.c:3030).
///
/// The C call sites sit in the getWedgeIntersection family (acute.c:
/// 2428, 2437, 2811, 2832), landed with this PR (stage-1 acute B).
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:3032)
#[allow(clippy::needless_late_init)] // C-parity: C's declaration block
                                     // (acute.c:3034-3041) keeps its
                                     // declaration-then-assignment shape
pub(crate) fn halfPlaneIntersection(
    numvertices: i32,
    convexPoly: &mut [f64],
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
) -> i32 {
    let dx: f64; // direction of the line (acute.c:3034)
    let dy: f64; // direction of the line (acute.c:3034)
    let mut z: f64;
    let mut min: f64;
    let mut max: f64;
    let mut j: i32;
    // C: REAL *polys[2]; (acute.c:3037) with its two malloc(100 REAL)
    // scratch buffers (acute.c:3043-3044): the substrate mirror is one
    // zero-initialized stack array of the two [f64; 100] half-polygon
    // buffers, and free(polys[0])/free(polys[1]) (acute.c:3082-3083)
    // drops with them (D21). C: REAL *res = NULL; (acute.c:3038): res
    // carries the found poly's index instead of its pointer; the NULL is
    // never dereferenced (intFound, acute.c:3040, guards the read at
    // acute.c:3072-3078), so the index default 0 is unobservable (D21).
    let mut polys = [[0.0f64; 100]; 2];
    let mut res: usize = 0;
    let mut count: i32 = 0;
    let mut intFound: i32 = 0;
    let numpolys: i32;

    dx = x2 - x1;
    dy = y2 - y1;
    numpolys = splitConvexPolygon(numvertices, convexPoly, x1, y1, x2, y2, &mut polys);
    if numpolys == 3 {
        count = numvertices;
    } else {
        for i in 0..numpolys {
            min = 1e37;
            max = -1e37;
            // compute the minimum and maximum of the
            // third coordinate of the cross product
            // (acute.c:3055-3056)
            j = 1;
            while j as f64 <= 2.0 * polys[i as usize][0] - 1.0 {
                z = dx * (polys[i as usize][j as usize + 1] - y1)
                    - dy * (polys[i as usize][j as usize] - x1);
                min = if z < min { z } else { min };
                max = if z > max { z } else { max };
                j += 2;
            }
            // ... and choose the (absolute) greater of both
            // (acute.c:3062)
            z = if min.abs() > max.abs() { min } else { max };
            // and if it is positive, the polygon polys[i]
            // is on the left side of line (acute.c:3064-3065)
            if z > 0.0 {
                res = i as usize;
                intFound = 1;
                break;
            }
        }
        if intFound == 1 {
            while (count as f64) < polys[res][0] {
                convexPoly[2 * count as usize] = polys[res][2 * count as usize + 1];
                convexPoly[2 * count as usize + 1] = polys[res][2 * count as usize + 2];
                count += 1;
            }
        }
    }

    // update convexPoly (acute.c:3085)
    count
}

/// C: splitConvexPolygon (acute.c:3096-3246). Splits a convex polygon
/// into one or two polygons through the intersection with the given
/// line (regarding the directional vector of the line). The half-
/// polygons land in polys: slot 0 of each buffer carries the vertex
/// count (acute.c:3236-3237), followed by the coordinate pairs; the
/// return value is 1 (one polygon in polys[0]), 2 (two polygons) or 3
/// (something went wrong with the intersection: ignore this one,
/// acute.c:3226-3232).
/// www.mathematik.uni-ulm.de/stochastik/lehre/ws03_04/rt/Geometry2D.ps
/// (acute.c:3094).
///
/// The only C call site is halfPlaneIntersection (acute.c:3048), which
/// lands with this PR (stage-1 acute B): the split is now reachable
/// from the wedge intersections.
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:3096)
#[allow(clippy::needless_late_init)] // C-parity: C's declaration block
                                     // (acute.c:3102-3112) and the
                                     // per-branch numpolys assignment
                                     // (acute.c:3232/3235) keep their
                                     // declaration-then-assignment shape
pub(crate) fn splitConvexPolygon(
    numvertices: i32,
    convexPoly: &[f64],
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    polys: &mut [[f64; 100]; 2],
) -> i32 {
    /*
     * state = 0: before the first intersection (with the line)
     * state = 1: after the first intersection (with the line)
     * state = 2: after the second intersection (with the line)
     * (acute.c:3097-3101)
     */
    let mut state: i32 = 0;
    // C: REAL p[3]; (acute.c:3103): the edge-intersection scratch is a
    // zero-initialized [f64; 3] where C's is uninitialized
    // (lineLineSegmentIntersection always writes p[0]; p[1], p[2] are
    // read only on paths where they were written), unobservable (D21).
    let mut p = [0.0f64; 3];
    // poly1 is constructed in states 0 and 2 (acute.c:3104); poly2 is
    // constructed in state 1 (acute.c:3107). C's aliasing locals
    // poly1 = polys[0], poly2 = polys[1] (acute.c:3105, 3108) write
    // through polys[0]/polys[1] directly here, and the pointer-identity
    // stores polys[0] = poly1 / polys[1] = poly2 (acute.c:3239, 3242)
    // drop (D21).
    let mut poly1counter: i32 = 0;
    let mut poly2counter: i32 = 0;
    let numpolys: i32;
    // C: REAL compConst = 0.000000000001; (acute.c:3112): C's local
    // shadows the module constant for this body; the mirror is the
    // same-named body-local const (a let binding cannot re-bind the
    // constant name), shadowing the crate constant the same way. The
    // eight debug counters of acute.c:3114 (case1,
    // case2, case3, case31, case32, case33, case311, case3111) and their
    // increments (acute.c:3133, 3143, 3148, 3152, 3164, 3178, 3190,
    // 3211) are dead stores (their only readers are the commented-out
    // printf block, acute.c:3227-3230) and are dropped (D22).
    #[allow(non_upper_case_globals)] // C-parity: the C local's name
                                     // verbatim (acute.c:3112)
    const compConst: f64 = 0.000000000001;
    // intersect all edges of poly with line (acute.c:3115)
    for i in (0..2 * numvertices).step_by(2) {
        let j: i32 = if i + 2 >= 2 * numvertices { 0 } else { i + 2 };
        lineLineSegmentIntersection(
            x1,
            y1,
            x2,
            y2,
            convexPoly[i as usize],
            convexPoly[i as usize + 1],
            convexPoly[j as usize],
            convexPoly[j as usize + 1],
            &mut p,
        );
        // if this edge does not intersect with line (acute.c:3119)
        if (p[0] - 0.0).abs() <= compConst {
            // add p[j] to the proper polygon (acute.c:3122)
            if state == 1 {
                poly2counter += 1;
                polys[1][2 * poly2counter as usize - 1] = convexPoly[j as usize];
                polys[1][2 * poly2counter as usize] = convexPoly[j as usize + 1];
            } else {
                poly1counter += 1;
                polys[0][2 * poly1counter as usize - 1] = convexPoly[j as usize];
                polys[0][2 * poly1counter as usize] = convexPoly[j as usize + 1];
            }
        }
        // ... or if the intersection is the whole edge (acute.c:3135)
        else if (p[0] - 2.0).abs() <= compConst {
            // then we can not reach state 1 and 2 (acute.c:3138)
            poly1counter += 1;
            polys[0][2 * poly1counter as usize - 1] = convexPoly[j as usize];
            polys[0][2 * poly1counter as usize] = convexPoly[j as usize + 1];
        }
        // ... or if the intersection is a point (acute.c:3145)
        else {
            // if the point is the second vertex of the edge
            // (acute.c:3149)
            if (p[1] - convexPoly[j as usize]).abs() <= compConst
                && (p[2] - convexPoly[j as usize + 1]).abs() <= compConst
            {
                if state == 1 {
                    poly2counter += 1;
                    polys[1][2 * poly2counter as usize - 1] = convexPoly[j as usize];
                    polys[1][2 * poly2counter as usize] = convexPoly[j as usize + 1];
                    poly1counter += 1;
                    polys[0][2 * poly1counter as usize - 1] = convexPoly[j as usize];
                    polys[0][2 * poly1counter as usize] = convexPoly[j as usize + 1];
                    state += 1;
                } else if state == 0 {
                    poly1counter += 1;
                    polys[0][2 * poly1counter as usize - 1] = convexPoly[j as usize];
                    polys[0][2 * poly1counter as usize] = convexPoly[j as usize + 1];
                    // test whether the polygon is splitted
                    // or the line only touches the polygon
                    // (acute.c:3168-3169)
                    if i + 4 < 2 * numvertices {
                        let s1 = linePointLocation(
                            x1,
                            y1,
                            x2,
                            y2,
                            convexPoly[i as usize],
                            convexPoly[i as usize + 1],
                        );
                        let s2 = linePointLocation(
                            x1,
                            y1,
                            x2,
                            y2,
                            convexPoly[i as usize + 4],
                            convexPoly[i as usize + 5],
                        );
                        // the line only splits the polygon
                        // when the previous and next vertex lie
                        // on different sides of the line
                        // (acute.c:3173-3175)
                        if s1 != s2 && s1 != 0 && s2 != 0 {
                            poly2counter += 1;
                            polys[1][2 * poly2counter as usize - 1] = convexPoly[j as usize];
                            polys[1][2 * poly2counter as usize] = convexPoly[j as usize + 1];
                            state += 1;
                        }
                    }
                }
            }
            // ... if the point is not the other vertex of the edge
            // (acute.c:3187)
            else if !((p[1] - convexPoly[i as usize]).abs() <= compConst
                && (p[2] - convexPoly[i as usize + 1]).abs() <= compConst)
            {
                poly1counter += 1;
                polys[0][2 * poly1counter as usize - 1] = p[1];
                polys[0][2 * poly1counter as usize] = p[2];
                poly2counter += 1;
                polys[1][2 * poly2counter as usize - 1] = p[1];
                polys[1][2 * poly2counter as usize] = p[2];
                if state == 1 {
                    poly1counter += 1;
                    polys[0][2 * poly1counter as usize - 1] = convexPoly[j as usize];
                    polys[0][2 * poly1counter as usize] = convexPoly[j as usize + 1];
                } else if state == 0 {
                    poly2counter += 1;
                    polys[1][2 * poly2counter as usize - 1] = convexPoly[j as usize];
                    polys[1][2 * poly2counter as usize] = convexPoly[j as usize + 1];
                }
                state += 1;
            }
            // ... else if the point is the second vertex of the edge
            // (acute.c:3208)
            else {
                if state == 1 {
                    poly2counter += 1;
                    polys[1][2 * poly2counter as usize - 1] = convexPoly[j as usize];
                    polys[1][2 * poly2counter as usize] = convexPoly[j as usize + 1];
                } else {
                    poly1counter += 1;
                    polys[0][2 * poly1counter as usize - 1] = convexPoly[j as usize];
                    polys[0][2 * poly1counter as usize] = convexPoly[j as usize + 1];
                }
            }
        }
    }
    // after splitting the state must be 0 or 2
    // (depending whether the polygon was splitted or not)
    // (acute.c:3224-3225)
    if state != 0 && state != 2 {
        // if there is something wrong with the intersection, just ignore
        // this one (acute.c:3231)
        numpolys = 3;
    } else {
        // finally convert the vertex lists into convex polygons
        // (acute.c:3234)
        numpolys = if state == 0 { 1 } else { 2 };
        polys[0][0] = poly1counter as f64;
        polys[1][0] = poly2counter as f64;
    }
    numpolys
}

/// C: linePointLocation (acute.c:3255-3273). Determines on which side
/// (relative to the direction) of the given line the point lies: 0 on
/// the line, 1 on the left, 2 on the right.
/// www.mathematik.uni-ulm.de/stochastik/lehre/ws03_04/rt/Geometry2D.ps
/// (acute.c:3253). The == 90.0 arm (acute.c:3257) catches the vertical
/// line; x2 == x1 divides by zero exactly as C does (the IEEE inf/NaN
/// semantics C relies on), and both arms converge on the same z
/// classification.
///
/// The only C call sites are splitConvexPolygon's split test (acute.c:
/// 3171-3172), landed with this PR (stage-1 acute B).
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:3255)
#[allow(clippy::float_cmp)] // C-parity: the == 90.0 vertical-line test
// verbatim (acute.c:3257)
#[allow(clippy::needless_late_init)] // C-parity: REAL z; (acute.c:3256)
                                     // keeps C's declaration shape
pub(crate) fn linePointLocation(x1: f64, y1: f64, x2: f64, y2: f64, x: f64, y: f64) -> i32 {
    let z: f64;
    if ((y2 - y1) / (x2 - x1)).atan() * 180.0 / PI == 90.0 {
        if (x1 - x).abs() <= 0.00000000001 {
            return 0;
        }
    } else if (y1 + (((y2 - y1) * (x - x1)) / (x2 - x1)) - y).abs() <= compConst {
        return 0;
    }
    // third component of the 3 dimensional product (acute.c:3264)
    z = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1);
    if (z - 0.0).abs() <= 0.00000000001 {
        0
    } else if z > 0.0 {
        1
    } else {
        2
    }
}

/// C: lineLineSegmentIntersection (acute.c:3279-3325). Given four
/// points representing one line and a line segment, write the
/// intersection into the out-array p: p[0] is 0.0 if they do not
/// intersect (parallel, acute.c:3308, or the crossing lies off the
/// segment, acute.c:3317), 2.0 if the line is coincident with the whole
/// segment (acute.c:3306) and 1.0 for one crossing point (p[1], p[2],
/// acute.c:3319-3321). Referenced to
/// http://local.wasp.uwa.edu.au/~pbourke/geometry/ (acute.c:3277).
///
/// The only C call site is splitConvexPolygon's edge walk (acute.c:
/// 3118), landed with this PR (stage-1 acute B).
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:3279)
#[allow(clippy::too_many_arguments)] // C-parity: the nine-parameter C
// signature (acute.c:3279-3283)
#[allow(clippy::needless_late_init)] // C-parity: the REAL declaration
                                     // block (acute.c:3294) keeps C's
                                     // declaration-then-assignment shape
pub(crate) fn lineLineSegmentIntersection(
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    x3: f64,
    y3: f64,
    x4: f64,
    y4: f64,
    p: &mut [f64],
) {
    // x1,y1  P1 coordinates (point of line)
    // x2,y2  P2 coordinates (point of line)
    // x3,y3  P3 coordinates (point of line segment)
    // x4,y4  P4 coordinates (point of line segment)
    // p[1],p[2]   intersection coordinates
    //
    // This function returns a pointer array which first index indicates
    // weather they intersect on one point or not, followed by coordinate
    // pairs. (acute.c:3285-3292)

    let mut u_a: f64;
    let mut u_b: f64;
    let denom: f64;
    // C: REAL compConst = 0.0000000000001; (acute.c:3295): C's local
    // shadows the module constant for this body; the mirror is the
    // same-named body-local const (a let binding cannot re-bind the
    // constant name), shadowing the crate constant the same way.
    #[allow(non_upper_case_globals)] // C-parity: the C local's name
                                     // verbatim (acute.c:3295)
    const compConst: f64 = 0.0000000000001;
    // calculate denominator first (acute.c:3296)
    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    u_a = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
    u_b = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);

    if (denom - 0.0).abs() < compConst {
        if (u_b - 0.0).abs() < compConst && (u_a - 0.0).abs() < compConst {
            p[0] = 2.0; // if denominator and numerator equal to zero, lines are coincident
        } else {
            p[0] = 0.0; // if denominator equals to zero, lines are parallel
        }
    } else {
        u_b /= denom;
        u_a /= denom;
        // check if it is on the line segment (acute.c:3315)
        if u_b < -compConst || u_b > 1.0 + compConst {
            p[0] = 0.0;
        } else {
            p[0] = 1.0;
            p[1] = x1 + u_a * (x2 - x1); // intersection point
            p[2] = y1 + u_a * (y2 - y1);
        }
    }
}

/// C: findPolyCentroid (acute.c:3330-3343). Returns the centroid of a
/// given polygon: the mean of the vertex coordinates, written into
/// centroid[0], centroid[1].
///
/// The C call sites sit in the getWedgeIntersection family (acute.c:
/// 2448 and 2844), landed with this PR (stage-1 acute B).
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:3330)
pub(crate) fn findPolyCentroid(numpoints: i32, points: &[f64], centroid: &mut [f64]) {
    centroid[0] = 0.0;
    centroid[1] = 0.0;

    for i in (0..2 * numpoints).step_by(2) {
        centroid[0] += points[i as usize];
        centroid[1] += points[i as usize + 1];
    }
    centroid[0] /= numpoints as f64;
    centroid[1] /= numpoints as f64;
}

/// C: circleLineIntersection (acute.c:3354-3403). Given two points
/// representing a line and a radius together with a center point
/// representing a circle, write the intersection points into the
/// out-array p: p[0] carries the number of intersection points (0.0,
/// 1.0 or 2.0), followed by the coordinate pairs in p[1], p[2] (and
/// p[3], p[4]). Referenced to
/// http://local.wasp.uwa.edu.au/~pbourke/geometry/sphereline/
/// (acute.c:3351).
///
/// The C call sites sit in the findNewSPLocation family (acute.c:478,
/// 590, 1141, 1408), which lands with stage-1 acute C; the 1:1 body
/// is ahead-of-use until then.
#[allow(dead_code)]
// C-parity: the call sites (acute.c:478, 590, 1141,
// 1408) belong to the findNewSPLocation family,
// stage-1 acute C.
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:3354)
#[allow(clippy::too_many_arguments)] // C-parity: the eight-parameter C
// signature (acute.c:3354-3357)
#[allow(clippy::needless_late_init)]
// C-parity: the REAL declaration
// block (acute.c:3367) keeps C's
// declaration-then-assignment shape
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C-parity: the negated
                                            // fabs(...) < compConst test verbatim (acute.c:3387)
pub(crate) fn circleLineIntersection(
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    x3: f64,
    y3: f64,
    r: f64,
    p: &mut [f64],
) {
    // x1,y1  P1 coordinates [point of line]
    // x2,y2  P2 coordinates [point of line]
    // x3,y3, r  P3 coordinates(circle center) and radius [circle]
    // p[1],p[2]; p[3],p[4]   intersection coordinates
    //
    // This function returns a pointer array which first index indicates
    // the number of intersection points, followed by coordinate pairs.
    // (acute.c:3359-3365)

    let a: f64;
    let b: f64;
    let c: f64;
    let mut mu: f64;
    let i: f64;

    a = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
    b = 2.0 * ((x2 - x1) * (x1 - x3) + (y2 - y1) * (y1 - y3));
    c = x3 * x3 + y3 * y3 + x1 * x1 + y1 * y1 - 2.0 * (x3 * x1 + y3 * y1) - r * r;
    i = b * b - 4.0 * a * c;

    if i < 0.0 {
        // no intersection (acute.c:3375)
        p[0] = 0.0;
    } else if (i - 0.0).abs() < compConst {
        // one intersection (acute.c:3379)
        p[0] = 1.0;

        mu = -b / (2.0 * a);
        p[1] = x1 + mu * (x2 - x1);
        p[2] = y1 + mu * (y2 - y1);
    } else if i > 0.0 && !((a - 0.0).abs() < compConst) {
        // two intersections (acute.c:3388)
        p[0] = 2.0;
        // first intersection (acute.c:3390)
        mu = (-b + i.sqrt()) / (2.0 * a);
        p[1] = x1 + mu * (x2 - x1);
        p[2] = y1 + mu * (y2 - y1);
        // second intersection (acute.c:3394)
        mu = (-b - i.sqrt()) / (2.0 * a);
        p[3] = x1 + mu * (x2 - x1);
        p[4] = y1 + mu * (y2 - y1);
    } else {
        p[0] = 0.0;
    }
}

/// C: chooseCorrectPoint (acute.c:3411-3450). Given three points (the
/// bisector point of the dual edge on the triangle, an intersection
/// point, the circumcenter point), check if the intersection point is
/// the correct one: returns 1 if it is, 0 if the other point is to be
/// taken, by the isObtuse flag's two distance comparisons (the squared
/// distance between circumcenter and intersection point, acute.c:3428,
/// and between bisector point and intersection point, acute.c:3430).
///
/// The C call sites sit in the findNewSPLocation family (acute.c:485,
/// 599, 1109, 1144, 1413), which lands with stage-1 acute C; the 1:1
/// body is ahead-of-use until then.
#[allow(dead_code)]
// C-parity: the call sites (acute.c:485, 599, 1109,
// 1144, 1413) belong to the findNewSPLocation
// family, stage-1 acute C.
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:3411)
#[allow(clippy::needless_late_init)] // C-parity: C's declaration block
                                     // (acute.c:3424-3425) keeps its
                                     // declaration-then-assignment shape
pub(crate) fn chooseCorrectPoint(
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    x3: f64,
    y3: f64,
    isObtuse: i32,
) -> i32 {
    // x1,y1  P1 coordinates (bisector point of dual edge on triangle)
    // x2,y2  P2 coordinates (intersection point)
    // x3,y3  P3 coordinates (circumcenter point)
    // returns 1.0, if given point is the correct one
    // otherwise return 0.0
    // (acute.c:3417-3421)

    let d1: f64;
    let d2: f64;
    let p: i32;

    // squared distance between circumcenter and intersection point
    // (acute.c:3427)
    d1 = (x2 - x3) * (x2 - x3) + (y2 - y3) * (y2 - y3);
    // squared distance between bisector point and intersection point
    // (acute.c:3429)
    d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if isObtuse == 1 {
        // obtuse case (acute.c:3433)
        if d2 >= d1 {
            p = 1; // means we have found the right point
        } else {
            p = 0; // means take the other point
        }
    } else {
        // non-obtuse case (acute.c:3440)
        if d2 < d1 {
            p = 1; // means we have found the right point
        } else {
            p = 0; // means take the other point
        }
    }
    // HANDLE RIGHT TRIANGLE CASE!!!!!!!!!!!!!!!!!!!!!!!!!!!! (acute.c:3447)
    p
}

/// C: pointBetweenPoints (acute.c:3458-3485). Given three points on a
/// line (the intersection point on a Voronoi edge, the circumcenter,
/// the neighbor's circumcenter to be compared), write into the
/// out-array p whether the third point lies between the other two:
/// p[0] is 1.0 between (p[1] the squared distance to the circumcenter,
/// p[2], p[3] the point itself), 0.0 otherwise (p[1] through p[3]
/// zero).
///
/// The C call sites sit in the findNewSPLocation family (acute.c:497,
/// 612, 1181, 1191, 1269, 1449, 1458, 1531), which lands with stage-1
/// acute C; the 1:1 body is ahead-of-use until then.
#[allow(dead_code)]
// C-parity: the call sites (acute.c:497, 612, 1181,
// 1191, 1269, 1449, 1458, 1531) belong to the
// findNewSPLocation family, stage-1 acute C.
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:3458)
pub(crate) fn pointBetweenPoints(
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    x: f64,
    y: f64,
    p: &mut [f64],
) {
    // x1,y1  P1 coordinates [point of line] (point on Voronoi edge - intersection)
    // x2,y2  P2 coordinates [point of line] (circumcenter)
    // x,y    P3 coordinates [point to be compared] (neighbor's circumcenter)
    //
    // This function returns a pointer array which first index indicates
    // the whether the point is in between the other points, followed by
    // coordinate pairs. (acute.c:3460-3465)

    // now check whether the point is close to circumcenter than
    // intersection point (acute.c:3467)
    // BETWEEN THE POINTS (acute.c:3468)
    if (x2 - x) * (x2 - x) + (y2 - y) * (y2 - y) < (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) {
        p[0] = 1.0;
        // calculate the squared distance to circumcenter (acute.c:3471)
        p[1] = (x - x2) * (x - x2) + (y - y2) * (y - y2);
        p[2] = x;
        p[3] = y;
    }
    // *NOT* BETWEEN THE POINTS (acute.c:3475)
    else {
        p[0] = 0.0;
        p[1] = 0.0;
        p[2] = 0.0;
        p[3] = 0.0;
    }
}

/// C: testTriangleAngle (acute.c:3493-3577). Given three coordinates,
/// test the triangle against both quality bounds: the squared cosine
/// of the smallest angle against b->goodangle (C's testtriangle
/// shape, the same goodangle the quality.rs lens reads) and, when a
/// maximum angle is set (b->maxangle != 0.00, acute.c:3572), the
/// cosine of the largest angle against b->maxgoodangle; returns 1 for
/// a bad triangle, 0 for a good one. C's minedge/maxedge stores
/// (acute.c:3530, 3538, 3546, 3555, 3560, 3565) are dead (never
/// read) and are dropped (D24, the D14 precedent). The in-PR callers
/// are the getWedgeIntersectionWithMaxAngle scans (acute.c:2849,
/// 2853, 2881, 2885); the findNewSPLocation family calls (first at
/// acute.c:505) land with the last acute PR.
#[allow(non_snake_case)] // C-parity: the C name verbatim (acute.c:3493)
#[allow(clippy::too_many_arguments)] // C-parity: the seven-parameter C
// signature (acute.c:3493-3496)
#[allow(clippy::needless_late_init)]
// C-parity: the declaration block
// (acute.c:3499-3505) keeps the declaration-then-assignment shape
#[allow(clippy::float_cmp)] // C-parity: b->maxangle != 0.00 verbatim
                            // (acute.c:3572)
pub(crate) fn testTriangleAngle(
    b: &Behavior,
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    x3: f64,
    y3: f64,
) -> i32 {
    // variables keeping the distance values for the edges
    let dxod: f64;
    let dyod: f64;
    let dxda: f64;
    let dyda: f64;
    let dxao: f64;
    let dyao: f64;
    // calculate the side lengths
    // C: dxod = *x1 - *x2; ... dyao = *y3 - *y1; (acute.c:3508-3513)
    dxod = x1 - x2;
    dyod = y1 - y2;
    dxda = x2 - x3;
    dyda = y2 - y3;
    dxao = x3 - x1;
    dyao = y3 - y1;
    // calculate the squares of the side lentghs
    // C: dxod2 = dxod * dxod; ... (acute.c:3515-3520)
    let dxod2 = dxod * dxod;
    let dyod2 = dyod * dyod;
    let dxda2 = dxda * dxda;
    let dyda2 = dyda * dyda;
    let dxao2 = dxao * dxao;
    let dyao2 = dyao * dyao;

    /* Find the lengths of the triangle's three edges. */
    // C: apexlen = dxod2 + dyod2; orglen = ...; destlen = ...;
    // (acute.c:3523-3525)
    let apexlen = dxod2 + dyod2;
    let orglen = dxda2 + dyda2;
    let destlen = dxao2 + dyao2;

    // in order to check minimum angle condition
    // C: REAL angle; (acute.c:3503)
    let mut angle: f64;
    // in order to check minimum angle condition
    // C: REAL maxangle, maxedge; (acute.c:3505); the maxedge dead
    // store is dropped (D24)
    let maxangle: f64;

    // try to find the minimum edge and accordingly the pqr orientation
    if apexlen < orglen && apexlen < destlen {
        /* The edge opposite the apex is shortest. */
        // C: minedge = apexlen; (acute.c:3530) is a dead store (D24).
        /* Find the square of the cosine of the angle at the apex. */
        angle = dxda * dxao + dyda * dyao;
        angle = angle * angle / (orglen * destlen);
    } else if orglen < destlen {
        /* The edge opposite the origin is shortest. */
        // C: minedge = orglen; (acute.c:3538) is a dead store (D24).
        /* Find the square of the cosine of the angle at the origin. */
        angle = dxod * dxao + dyod * dyao;
        angle = angle * angle / (apexlen * destlen);
    } else {
        /* The edge opposite the destination is shortest. */
        // C: minedge = destlen; (acute.c:3546) is a dead store (D24).
        /* Find the square of the cosine of the angle at the destination. */
        angle = dxod * dxda + dyod * dyda;
        angle = angle * angle / (apexlen * orglen);
    }
    // try to find the maximum edge and accordingly the pqr orientation
    if apexlen > orglen && apexlen > destlen {
        /* The edge opposite the apex is longest. */
        // C: maxedge = apexlen; (acute.c:3555) is a dead store (D24).
        /* Find the cosine of the angle at the apex. */
        maxangle = (orglen + destlen - apexlen) / (2.0 * orglen.sqrt() * destlen.sqrt());
    } else if orglen > destlen {
        /* The edge opposite the origin is longest. */
        // C: maxedge = orglen; (acute.c:3560) is a dead store (D24).
        /* Find the cosine of the angle at the origin. */
        maxangle = (apexlen + destlen - orglen) / (2.0 * apexlen.sqrt() * destlen.sqrt());
    } else {
        /* The edge opposite the destination is longest. */
        // C: maxedge = destlen; (acute.c:3565) is a dead store (D24).
        /* Find the cosine of the angle at the destination. */
        maxangle = (apexlen + orglen - destlen) / (2.0 * apexlen.sqrt() * orglen.sqrt());
    }

    /* Check whether the angle is smaller than permitted. */
    // C: if ((angle > b->goodangle)  ||  (b->maxangle != 0.00 && maxangle < b->maxgoodangle))
    // (acute.c:3572)
    if angle > b.goodangle || (b.maxangle != 0.0 && maxangle < b.maxgoodangle) {
        return 1; // it is a bad triangle
    }
    0 // it is a good triangle
}

/// C: minDistanceToNeigbor (acute.c:3584-3658; the C name's typo is
/// kept; banner at acute.c:3579-3583: "Given the triangulation, and a
/// vertex returns the minimum distance to the vertices of the
/// triangle where the given vertex located"). C allocates a vertex
/// from the pool to hold the candidate coordinates (poolalloc,
/// acute.c:3591, the newvertex stores of acute.c:3592-3593); the
/// mirror allocates through the vx_alloc chokepoint and routes the
/// pool's OOM through the D8 funnel, returning the vertex row to the
/// pool on both arms exactly where C's vertexdealloc sits (acute.c:
/// 3639 and 3649). The returned distance is squared (C's d1/d2/d3 are
/// squared distances).
///
/// The collinear-not-between path leaves C's `enum locateresult
/// intersect` and `struct otri horiz` indeterminate (acute.c:3626-
/// 3631: the ahead == 0.0 branch of acute.c:3624 assigns only in the
/// between case, with no else); the port pins intersect =
/// LocateResult::InTriangle (the enum's zero discriminant, C's
/// INTRIANGLE) and horiz = *searchtri at the declaration, the
/// deterministic superset of the undefined state, so the pinned path
/// takes the distance arm and measures from the starting triangle
/// (D25).
///
/// The only C callers are the findNewSPLocation variants (acute.c:
/// 1203, 1280, 1470, 1542), which land with the last acute PR; the
/// body is ahead-of-use until then.
#[allow(non_snake_case)] // C-parity: the C name verbatim, typo included
// (acute.c:3584)
#[allow(dead_code)]
// C-parity: the callers (acute.c:1203, 1280, 1470, 1542) belong to
// the findNewSPLocation family, the last acute PR.
#[allow(clippy::float_cmp)]
// C-parity: the torg/tdest coordinate
// equality tests verbatim (acute.c:3607, 3611) and the ahead == 0.0
// collinearity test (acute.c:3624)
#[allow(clippy::needless_return)] // C-parity: the return arms verbatim
                                  // (return 0.0 at acute.c:3640, the d1/d2/d3 returns of acute.c:3651-3657)
pub(crate) fn minDistanceToNeigbor(
    m: &mut Mesh,
    b: &Behavior,
    newlocX: f64,
    newlocY: f64,
    searchtri: &mut Otri,
) -> Result<f64, Oom> {
    // C: struct otri horiz; (acute.c:3585): for search operation.
    // D25: the pin of C's indeterminate state (see the fn doc); the
    // pinned value is *searchtri, exact on the collinear-not-between
    // path where no branch writes it (searchtri is unmutated there).
    let mut horiz = *searchtri;
    // C: enum locateresult intersect; (acute.c:3586). D25: the pin of
    // C's indeterminate state; the enum's zero discriminant is C's
    // INTRIANGLE.
    let mut intersect = LocateResult::InTriangle;
    // C: vertex v1, v2, v3, newvertex, torg, tdest; (acute.c:3587):
    // the handles materialize at their C read sites below.
    // C: newvertex = (vertex) poolalloc(&m->vertices); (acute.c:3591):
    // the candidate row, allocated through the chokepoint; its OOM
    // routes through the funnel instead of C's exit.
    let newvertex = m.vx_alloc()?;
    // C: newvertex[0] = newlocX; newvertex[1] = newlocY; (acute.c:3592-3593)
    m.vx_set_real(newvertex, 0, newlocX);
    m.vx_set_real(newvertex, 1, newlocY);
    /* Find the location of the vertex to be inserted.  Check if a good */
    /*   starting triangle has already been provided by the caller.     */
    /* Where are we? */
    // C: org(*searchtri, torg); dest(*searchtri, tdest); (acute.c:3604-3605)
    let torg = m.org(*searchtri);
    let tdest = m.dest(*searchtri);
    /* Check the starting triangle's vertices. */
    // C: if ((torg[0] == newvertex[0]) && (torg[1] == newvertex[1]))
    // (acute.c:3607)
    if m.vx_real(torg, 0) == newlocX && m.vx_real(torg, 1) == newlocY {
        // C: intersect = ONVERTEX; (acute.c:3608)
        intersect = LocateResult::OnVertex;
        // C: otricopy(*searchtri, horiz); (acute.c:3609)
        horiz = *searchtri;
    } else if m.vx_real(tdest, 0) == newlocX && m.vx_real(tdest, 1) == newlocY {
        // C: lnextself(*searchtri); (acute.c:3612): the caller's handle
        // is rotated, exactly as C mutates through its otri pointer.
        searchtri.lnextself();
        // C: intersect =  ONVERTEX; (acute.c:3613)
        intersect = LocateResult::OnVertex;
        // C: otricopy(*searchtri, horiz); (acute.c:3614)
        horiz = *searchtri;
    } else {
        /* Orient `searchtri' to fit the preconditions of calling preciselocate(). */
        // C: ahead = tri_counterclockwise(m, b, torg, tdest, newvertex); (acute.c:3617).
        // C's vertex pointers map to coordinate rows through vertex_coords,
        // the delaunay.rs convention for the predicates' REAL* vertices.
        let ahead = tri_counterclockwise(
            m,
            b,
            vertex_coords(m, torg),
            vertex_coords(m, tdest),
            vertex_coords(m, newvertex),
        );
        if ahead < 0.0 {
            /* Turn around so that `searchpoint' is to the left of the */
            /*   edge specified by `searchtri'.                        */
            // C: symself(*searchtri); otricopy(*searchtri, horiz); (acute.c:3621-3622)
            m.symself(searchtri);
            horiz = *searchtri;
            // C: intersect = preciselocate(m, b, newvertex, &horiz, 0); (acute.c:3623)
            intersect = preciselocate(m, b, (newlocX, newlocY), &mut horiz, false);
        } else if ahead == 0.0 {
            /* Check if `searchpoint' is between `torg' and `tdest'. */
            // C: if (((torg[0] < newvertex[0]) == (newvertex[0] < tdest[0])) &&
            //     ((torg[1] < newvertex[1]) == (newvertex[1] < tdest[1])))
            // (acute.c:3626-3627)
            if (m.vx_real(torg, 0) < newlocX) == (newlocX < m.vx_real(tdest, 0))
                && (m.vx_real(torg, 1) < newlocY) == (newlocY < m.vx_real(tdest, 1))
            {
                // C: intersect = ONEDGE; (acute.c:3628)
                intersect = LocateResult::OnEdge;
                // C: otricopy(*searchtri, horiz); (acute.c:3629)
                horiz = *searchtri;
            }
            // C has no else here (the gap of acute.c:3626-3631): the
            // not-between case falls through with intersect and horiz
            // untouched, the indeterminate state the D25 pins (fn doc).
        } else {
            // C: otricopy(*searchtri, horiz); (acute.c:3633)
            horiz = *searchtri;
            // C: intersect = preciselocate(m, b, newvertex, &horiz, 0); (acute.c:3634)
            intersect = preciselocate(m, b, (newlocX, newlocY), &mut horiz, false);
        }
    }
    // C: if(intersect == ONVERTEX || intersect == OUTSIDE){ ... }else{ ... }
    // (acute.c:3637-3658)
    if intersect == LocateResult::OnVertex || intersect == LocateResult::Outside {
        // set distance to 0
        // C: vertexdealloc(m,newvertex); return 0.0; (acute.c:3639-3640)
        m.vx_dealloc(newvertex);
        return Ok(0.0);
    } else {
        // intersect == ONEDGE || intersect == INTRIANGLE
        // find the triangle vertices
        // C: org(horiz, v1); dest(horiz, v2); apex(horiz, v3);
        // (acute.c:3643-3645)
        let v1 = m.org(horiz);
        let v2 = m.dest(horiz);
        let v3 = m.apex(horiz);
        // C: d1 = (v1[0] - newvertex[0]) * (v1[0] - newvertex[0]) + ...;
        // d2/d3 likewise (acute.c:3646-3648)
        let d1 = (m.vx_real(v1, 0) - newlocX) * (m.vx_real(v1, 0) - newlocX)
            + (m.vx_real(v1, 1) - newlocY) * (m.vx_real(v1, 1) - newlocY);
        let d2 = (m.vx_real(v2, 0) - newlocX) * (m.vx_real(v2, 0) - newlocX)
            + (m.vx_real(v2, 1) - newlocY) * (m.vx_real(v2, 1) - newlocY);
        let d3 = (m.vx_real(v3, 0) - newlocX) * (m.vx_real(v3, 0) - newlocX)
            + (m.vx_real(v3, 1) - newlocY) * (m.vx_real(v3, 1) - newlocY);
        // C: vertexdealloc(m,newvertex); (acute.c:3649)
        m.vx_dealloc(newvertex);
        // find minimum of the distance
        // C: if(d1 <= d2 && d1 <= d3){ ... (acute.c:3651-3657)
        if d1 <= d2 && d1 <= d3 {
            return Ok(d1);
        } else if d2 <= d3 {
            return Ok(d2);
        } else {
            return Ok(d3);
        }
    }
}

// Construction-pinned unit tests: every expected value below is
// hand-derived from the C branch tables and the geometry (exact f64
// arithmetic on small, exactly representable inputs), never by running
// the C oracle.
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn longest_shortest_edge_orientation_codes() {
        // All six strict orderings of (aodist, dadist, dodist),
        // hand-derived from the C branch tree (acute.c:1819-1846):
        // the code packs min, mid, max of the 123/213/312 side of C's
        // orientation banner (acute.c:1804-1815).
        assert_eq!(longestShortestEdge(1.0, 2.0, 3.0), 123);
        assert_eq!(longestShortestEdge(1.0, 3.0, 2.0), 132);
        assert_eq!(longestShortestEdge(2.0, 1.0, 3.0), 213);
        assert_eq!(longestShortestEdge(3.0, 1.0, 2.0), 231);
        assert_eq!(longestShortestEdge(2.0, 3.0, 1.0), 312);
        assert_eq!(longestShortestEdge(3.0, 2.0, 1.0), 321);
    }

    #[test]
    fn longest_shortest_edge_isosceles_paths() {
        // Ties take C's strict-comparison arms: aodist == dadist falls to
        // the third arm (min = 2), dodist == dadist blocks the first
        // arm's second conjunct.
        assert_eq!(longestShortestEdge(1.0, 1.0, 2.0), 213);
        assert_eq!(longestShortestEdge(2.0, 2.0, 1.0), 321);
        assert_eq!(longestShortestEdge(3.0, 1.0, 2.0), 231);
    }

    #[test]
    fn line_line_intersection_pins_both_argument_orders() {
        let mut p = [f64::NAN; 3];
        // The coordinate axes cross at the origin, in both argument
        // orders.
        lineLineIntersection(0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 1.0, &mut p);
        assert_eq!(p, [1.0, 0.0, 0.0]);
        lineLineIntersection(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, &mut p);
        assert_eq!(p, [1.0, 0.0, 0.0]);
        // The diagonals of the square [0,2]^2 cross at (1,1), in both
        // argument orders.
        lineLineIntersection(0.0, 0.0, 2.0, 2.0, 0.0, 2.0, 2.0, 0.0, &mut p);
        assert_eq!(p, [1.0, 1.0, 1.0]);
        lineLineIntersection(0.0, 2.0, 2.0, 0.0, 0.0, 0.0, 2.0, 2.0, &mut p);
        assert_eq!(p, [1.0, 1.0, 1.0]);
    }

    #[test]
    fn line_line_intersection_parallel_and_collinear() {
        let mut p = [f64::NAN; 3];
        // Parallel: denominator vanishes, the numerators do not.
        lineLineIntersection(0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, &mut p);
        assert_eq!(p[0], 0.0);
        // Collinear (coincident lines): denominator and both numerators
        // vanish.
        lineLineIntersection(0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 3.0, 0.0, &mut p);
        assert_eq!(p[0], 0.0);
    }

    #[test]
    fn half_plane_intersection_keeps_left_half_of_square() {
        // CCW unit square, vertical cut x = 0.5 pointing up: the left
        // half (0.5,0),(0.5,1),(0,1),(0,0) is written back (hand-traced
        // through splitConvexPolygon's state machine).
        let mut poly = [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0];
        let count = halfPlaneIntersection(4, &mut poly, 0.5, -1.0, 0.5, 1.0);
        assert_eq!(count, 4);
        assert_eq!(poly, [0.5, 0.0, 0.5, 1.0, 0.0, 1.0, 0.0, 0.0]);
    }

    #[test]
    fn half_plane_intersection_keeps_right_half_of_square() {
        // The same cut with the direction reversed (pointing down): the
        // left side is now x > 0.5, so the right half
        // (0.5,0),(1,0),(1,1),(0.5,1) is kept.
        let mut poly = [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0];
        let count = halfPlaneIntersection(4, &mut poly, 0.5, 1.0, 0.5, -1.0);
        assert_eq!(count, 4);
        assert_eq!(poly, [0.5, 0.0, 1.0, 0.0, 1.0, 1.0, 0.5, 1.0]);
    }

    #[test]
    fn half_plane_intersection_corner_cut_splits_at_vertices() {
        // The square cut along its diagonal (0,0)-(1,1): the line
        // passes exactly through the two opposite corners, exercising
        // the clipping-at-vertices split path (the linePointLocation
        // side test inside splitConvexPolygon, acute.c:3171-3172); the
        // upper-left triangle (1,1),(0,1),(0,0) is kept.
        let mut poly = [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0];
        let count = halfPlaneIntersection(4, &mut poly, 0.0, 0.0, 1.0, 1.0);
        assert_eq!(count, 3);
        // Only the first 3 vertices are written back; the tail keeps the
        // square's last vertex pair.
        assert_eq!(poly, [1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn half_plane_intersection_all_right_returns_zero() {
        // The cut runs along the square's left edge pointing up: every
        // piece lies on or right of the line, nothing is kept and
        // convexPoly is untouched.
        let mut poly = [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0];
        let count = halfPlaneIntersection(4, &mut poly, 0.0, -1.0, 0.0, 1.0);
        assert_eq!(count, 0);
        assert_eq!(poly, [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0]);
    }

    #[test]
    fn split_convex_polygon_vertical_cut_state_machine() {
        // The vertical cut of the CCW unit square at x = 0.5 (pointing
        // up) walks the state machine through two point intersections:
        // poly1 (states 0 and 2) is the left half, poly2 (state 1) the
        // right half, with the vertex counts in slot 0 (hand-traced).
        let poly = [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0];
        let mut polys = [[f64::NAN; 100]; 2];
        let numpolys = splitConvexPolygon(4, &poly, 0.5, -1.0, 0.5, 1.0, &mut polys);
        assert_eq!(numpolys, 2);
        assert_eq!(polys[0][0], 4.0);
        assert_eq!(polys[0][1..9], [0.5, 0.0, 0.5, 1.0, 0.0, 1.0, 0.0, 0.0]);
        assert_eq!(polys[1][0], 4.0);
        assert_eq!(polys[1][1..9], [0.5, 0.0, 1.0, 0.0, 1.0, 1.0, 0.5, 1.0]);
    }

    #[test]
    fn split_convex_polygon_line_on_edge_is_one_poly() {
        // A cut along the square's left edge (x = 0, pointing up): the
        // left edge is coincident (p[0] = 2), the touching corners take
        // the vertex branches, and the state never leaves 0: one polygon
        // (the whole square), numpolys = 1, poly2 empty.
        let poly = [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0];
        let mut polys = [[f64::NAN; 100]; 2];
        let numpolys = splitConvexPolygon(4, &poly, 0.0, -1.0, 0.0, 1.0, &mut polys);
        assert_eq!(numpolys, 1);
        assert_eq!(polys[0][0], 4.0);
        assert_eq!(polys[0][1..9], [1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0]);
        assert_eq!(polys[1][0], 0.0);
    }

    #[test]
    fn line_point_location_classifies_sides_and_on_line() {
        // Horizontal line (0,0)-(1,0): above is the left side (1), below
        // the right side (2), on the line 0.
        assert_eq!(linePointLocation(0.0, 0.0, 1.0, 0.0, 0.0, 1.0), 1);
        assert_eq!(linePointLocation(0.0, 0.0, 1.0, 0.0, 0.0, -1.0), 2);
        assert_eq!(linePointLocation(0.0, 0.0, 1.0, 0.0, 0.5, 0.0), 0);
        // Vertical line (0,0)-(0,1) (the atan arm's vertical case):
        // right of the direction (up) is the right side (2), left is 1,
        // on the line 0. Whichever arm catches the point, both converge
        // on the same classification.
        assert_eq!(linePointLocation(0.0, 0.0, 0.0, 1.0, 1.0, 0.0), 2);
        assert_eq!(linePointLocation(0.0, 0.0, 0.0, 1.0, -1.0, 0.0), 1);
        assert_eq!(linePointLocation(0.0, 0.0, 0.0, 1.0, 0.0, 0.5), 0);
    }

    #[test]
    fn line_line_segment_intersection_on_segment_both_directions() {
        let mut p = [f64::NAN; 3];
        // The vertical line x = 0 crosses the segment (-1,0)-(1,0) at
        // (0,0); both segment directions pin the same crossing point.
        lineLineSegmentIntersection(0.0, -1.0, 0.0, 1.0, -1.0, 0.0, 1.0, 0.0, &mut p);
        assert_eq!(p, [1.0, 0.0, 0.0]);
        lineLineSegmentIntersection(0.0, -1.0, 0.0, 1.0, 1.0, 0.0, -1.0, 0.0, &mut p);
        assert_eq!(p, [1.0, 0.0, 0.0]);
    }

    #[test]
    fn line_line_segment_intersection_off_segment_parallel_collinear() {
        let mut p = [f64::NAN; 3];
        // The crossing with the line lies beyond the segment: p[0] = 0.
        lineLineSegmentIntersection(0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 2.0, 0.0, &mut p);
        assert_eq!(p[0], 0.0);
        // Parallel, distinct: p[0] = 0.
        lineLineSegmentIntersection(0.0, -1.0, 0.0, 1.0, 1.0, -1.0, 1.0, 1.0, &mut p);
        assert_eq!(p[0], 0.0);
        // Collinear: the whole-edge case, p[0] = 2.
        lineLineSegmentIntersection(0.0, -1.0, 0.0, 1.0, 0.0, 2.0, 0.0, 5.0, &mut p);
        assert_eq!(p[0], 2.0);
    }

    #[test]
    fn find_poly_centroid_of_triangle_and_single_point() {
        let mut centroid = [f64::NAN; 2];
        // The centroid of (0,0), (4,0), (2,6) is the mean (2,2).
        findPolyCentroid(3, &[0.0, 0.0, 4.0, 0.0, 2.0, 6.0], &mut centroid);
        assert_eq!(centroid, [2.0, 2.0]);
        // A single point is its own centroid.
        findPolyCentroid(1, &[3.0, 4.0], &mut centroid);
        assert_eq!(centroid, [3.0, 4.0]);
    }

    #[test]
    fn circle_line_intersection_two_points_both_orders() {
        let mut p = [f64::NAN; 5];
        // The line y = 0 through the circle (center origin, r = 2) cuts
        // it at (2,0) and (-2,0); the reversed line direction swaps the
        // order of the two points.
        circleLineIntersection(-1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0, &mut p);
        assert_eq!(p, [2.0, 2.0, 0.0, -2.0, 0.0]);
        circleLineIntersection(1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 2.0, &mut p);
        assert_eq!(p, [2.0, -2.0, 0.0, 2.0, 0.0]);
    }

    #[test]
    fn circle_line_intersection_tangent_and_miss() {
        let mut p = [f64::NAN; 5];
        // The tangent line y = 2 touches the circle (origin, r = 2) in
        // exactly one point: p[0] = 1, the tangent point (0,2).
        circleLineIntersection(0.0, 2.0, 1.0, 2.0, 0.0, 0.0, 2.0, &mut p);
        assert_eq!(p[0], 1.0);
        assert_eq!(p[1], 0.0);
        assert_eq!(p[2], 2.0);
        // The line y = 3 misses the circle: p[0] = 0.
        circleLineIntersection(0.0, 3.0, 1.0, 3.0, 0.0, 0.0, 2.0, &mut p);
        assert_eq!(p[0], 0.0);
    }

    #[test]
    fn choose_correct_point_obtuse_and_non_obtuse_branches() {
        // d1 (intersection to circumcenter) = 4, d2 (intersection to
        // bisector point) = 1: the obtuse case wants d2 >= d1 (rejects),
        // the non-obtuse case wants d2 < d1 (accepts).
        assert_eq!(chooseCorrectPoint(0.0, 0.0, 1.0, 0.0, 3.0, 0.0, 1), 0);
        assert_eq!(chooseCorrectPoint(0.0, 0.0, 1.0, 0.0, 3.0, 0.0, 0), 1);
        // d1 = 0.25 < d2 = 1: the obtuse case accepts, the non-obtuse
        // case rejects.
        assert_eq!(chooseCorrectPoint(0.0, 0.0, 1.0, 0.0, 0.5, 0.0, 1), 1);
        assert_eq!(chooseCorrectPoint(0.0, 0.0, 1.0, 0.0, 0.5, 0.0, 0), 0);
    }

    #[test]
    fn point_between_points_between_boundary_and_not() {
        let mut p = [f64::NAN; 4];
        // (4,0) lies between the intersection (0,0) and the circumcenter
        // (10,0): p[0] = 1, p[1] = 36 (the squared distance to the
        // circumcenter), p[2], p[3] the point itself.
        pointBetweenPoints(0.0, 0.0, 10.0, 0.0, 4.0, 0.0, &mut p);
        assert_eq!(p, [1.0, 36.0, 4.0, 0.0]);
        // (-20,0) lies beyond the circumcenter: not between, zeros.
        pointBetweenPoints(0.0, 0.0, 10.0, 0.0, -20.0, 0.0, &mut p);
        assert_eq!(p, [0.0, 0.0, 0.0, 0.0]);
        // The boundary: the point on the intersection itself has equal
        // squared distances (100); C's comparison is strict, so it is
        // not between.
        pointBetweenPoints(0.0, 0.0, 10.0, 0.0, 0.0, 0.0, &mut p);
        assert_eq!(p, [0.0, 0.0, 0.0, 0.0]);
    }

    // -----------------------------------------------------------------
    // The stage-1 acute B tests: the acutepool capacities and resize
    // guard, the angle gates, the star walk, the wedge intersections,
    // minDistanceToNeigbor's locate paths and doSmoothing's
    // relocation, all pinned by construction (hand-derived geometry,
    // exact f64 arithmetic on exactly representable inputs), never by
    // running the C oracle. The fixtures go through the landed
    // mesh-local layer exactly like the refine.rs tests.
    // -----------------------------------------------------------------

    /// -q MINANGLE through behavior_update, exactly like refine.rs's
    /// fixture helper; the goodangle the angle gates read.
    fn behavior_q(minangle: f64) -> Behavior {
        let mut b = Behavior {
            quality: true,
            ..Behavior::default_form()
        };
        b.minangle = minangle;
        b.behavior_update();
        b
    }

    /// -q MINANGLE -u MAXANGLE (the aCute upper bound): the goodangle
    /// and maxgoodangle pair the testTriangleAngle gate reads.
    fn behavior_q_u(minangle: f64, maxangle: f64) -> Behavior {
        let mut b = Behavior {
            quality: true,
            ..Behavior::default_form()
        };
        b.minangle = minangle;
        b.maxangle = maxangle;
        b.behavior_update();
        b
    }

    /// mesh_dim 2 like every stage-1 fixture; no extras.
    fn mesh(b: &Behavior) -> Mesh {
        Mesh::new(b, 16, 2, 0, 0).expect("mesh init")
    }

    fn vxy(m: &mut Mesh, x: f64, y: f64) -> u32 {
        let v = m.vx_alloc().expect("vertex alloc");
        m.vx_set_real(v, 0, x);
        m.vx_set_real(v, 1, y);
        v
    }

    /// A CCW fan of triangles around a center vertex with the given
    /// ring, triangle i = (center, ring[i], ring[i+1 mod k]), bonded
    /// around the center with the refine.rs fixture's opposite-
    /// direction convention: bond(T_i @orient 2, T_{i+1} @orient 0).
    /// The ring edges stay unbonded (hull), exactly like C's border.
    fn star_fan(
        m: &mut Mesh,
        center: (f64, f64),
        ring: &[(f64, f64)],
    ) -> (u32, Vec<u32>, Vec<Otri>) {
        let c = vxy(m, center.0, center.1);
        let r: Vec<u32> = ring.iter().map(|(x, y)| vxy(m, *x, *y)).collect();
        let k = ring.len();
        let mut tris = Vec::new();
        for i in 0..k {
            let t = m
                .maketriangle(m.tri_shape.usesegments, false)
                .expect("tri alloc");
            m.setorg(t, c);
            m.setdest(t, r[i]);
            m.setapex(t, r[(i + 1) % k]);
            tris.push(t);
        }
        for i in 0..k {
            let j = (i + 1) % k;
            m.bond(
                Otri {
                    tri: tris[i].tri,
                    orient: 2,
                },
                Otri {
                    tri: tris[j].tri,
                    orient: 0,
                },
            );
        }
        (c, r, tris)
    }

    /// A triangle (pa, pb, pc) CCW at orient 0, unbonded (hull edges),
    /// like refine.rs's lone_tri.
    fn lone_tri(
        m: &mut Mesh,
        pa: (f64, f64),
        pb: (f64, f64),
        pc: (f64, f64),
    ) -> (Otri, u32, u32, u32) {
        let a = vxy(m, pa.0, pa.1);
        let b = vxy(m, pb.0, pb.1);
        let c = vxy(m, pc.0, pc.1);
        let t = m
            .maketriangle(m.tri_shape.usesegments, false)
            .expect("tri alloc");
        m.setorg(t, a);
        m.setdest(t, b);
        m.setapex(t, c);
        (t, a, b, c)
    }

    #[test]
    fn acutepool_init_capacities_match_c() {
        // C acutepool_init(20) (acute.c:27-40) at the enforcequality
        // call site (triangle.c:7538): 500 for initialpoly and the
        // three point buffers, 2n for the petal buffers, 2n*20+40 for
        // wedges. The Mesh field carries exactly these capacities
        // (the eager construction of D23).
        let b = Behavior::default_form();
        let m = mesh(&b);
        assert_eq!(m.acute_mem.size, 20);
        assert_eq!(m.acute_mem.initialpoly.len(), 500);
        assert_eq!(m.acute_mem.petalx.len(), 40);
        assert_eq!(m.acute_mem.petaly.len(), 40);
        assert_eq!(m.acute_mem.petalr.len(), 40);
        assert_eq!(m.acute_mem.wedges.len(), 2 * 20 * 20 + 40);
        assert_eq!(m.acute_mem.points_p.len(), 500);
        assert_eq!(m.acute_mem.points_q.len(), 500);
        assert_eq!(m.acute_mem.points_r.len(), 500);
        // and the standalone construction matches the field
        let p = AcutePool::new(20);
        assert_eq!(p.wedges.len(), m.acute_mem.wedges.len());
    }

    #[test]
    fn acutepool_resize_only_if_smaller() {
        // C's only-if-smaller guard (acute.c:46-47): a smaller or equal
        // n is a no-op, a bigger n grows exactly the four
        // size-dependent buffers (acute.c:50-60), the 500-sized
        // buffers stay untouched.
        let mut p = AcutePool::new(20);
        acutepool_resize(10, &mut p);
        assert_eq!(p.size, 20);
        assert_eq!(p.petalx.len(), 40);
        assert_eq!(p.wedges.len(), 2 * 20 * 20 + 40);
        acutepool_resize(20, &mut p);
        assert_eq!(p.size, 20);
        acutepool_resize(21, &mut p);
        assert_eq!(p.size, 21);
        assert_eq!(p.petalx.len(), 42);
        assert_eq!(p.petaly.len(), 42);
        assert_eq!(p.petalr.len(), 42);
        assert_eq!(p.wedges.len(), 2 * 21 * 20 + 40);
        assert_eq!(p.initialpoly.len(), 500);
        assert_eq!(p.points_p.len(), 500);
        assert_eq!(p.points_q.len(), 500);
        assert_eq!(p.points_r.len(), 500);
        // re-guard after growth: a smaller n is again a no-op
        acutepool_resize(15, &mut p);
        assert_eq!(p.size, 21);
        assert_eq!(p.petalx.len(), 42);
    }

    #[test]
    fn polygon_angles_square_good_needle_bad() {
        // The CCW unit square's corners are all 90 degrees, far above
        // 2*minangle = 40 (-q20): all valid (acute.c:2916 banner).
        let b = behavior_q(20.0);
        let square = [1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0];
        assert_eq!(polygonAngles(&b, 4, &square), 0);
        // A needle polygon: the corner at (2,0) between (0,0) and
        // (1, 0.05) is roughly 0.114 degrees, far below 40: bad.
        let needle = [0.0, 0.0, 2.0, 0.0, 1.0, 0.05];
        assert_eq!(polygonAngles(&b, 3, &needle), 1);
        // The wrap-around triples (i == numpoints-1, numpoints-2) read
        // the same needle polygon's other corners: also bad there.
        assert_eq!(polygonAngles(&b, 3, &needle), 1);
    }

    #[test]
    fn test_polygon_angle_good_and_bad_corners() {
        let b = behavior_q(20.0);
        // A right-angle corner at the middle vertex (2,0)... the
        // middle vertex is (x2,y2): 90 degrees is good.
        assert_eq!(testPolygonAngle(&b, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0), 0);
        // A needle corner at the middle vertex (1,0): the angle
        // between (0,0) and (0.5, 0.001) is roughly 0.114 degrees,
        // below 2*minangle = 40: bad.
        assert_eq!(testPolygonAngle(&b, 0.0, 0.0, 1.0, 0.0, 0.5, 0.001), 1);
        // Exactly the boundary shape is out of reach for exact f64
        // inputs, so the pair above pins both branches.
    }

    #[test]
    fn test_triangle_angle_min_and_max_branches() {
        // goodangle = cos^2(minangle) (options.rs behavior_update),
        // maxgoodangle = cos(maxangle).
        // An equilateral-ish triangle: min angle 60 > 20, max 60:
        // good under -q20 (no maxangle set).
        let b = behavior_q(20.0);
        assert_eq!(
            testTriangleAngle(&b, 0.0, 0.0, 1.0, 0.0, 0.5, 0.8660254037844386),
            0
        );
        // A skinny triangle: the smallest angle's squared cosine
        // exceeds goodangle: bad.
        assert_eq!(testTriangleAngle(&b, 0.0, 0.0, 1.0, 0.0, 0.5, 0.01), 1);
        // -q20 -u120: an isosceles triangle with base angles 25
        // (0.5*tan(25) high) passes the min gate (cos^2 25 < cos^2 20)
        // but its 130-degree apex violates the max gate
        // (cos 130 < cos 120): bad.
        let bu = behavior_q_u(20.0, 120.0);
        assert_eq!(
            testTriangleAngle(
                &bu,
                0.0,
                0.0,
                1.0,
                0.0,
                0.5,
                0.5 * (25.0f64).to_radians().tan()
            ),
            1
        );
        // The same triangle under -q25 -u120: the min gate now also
        // fails (cos^2 25 > cos^2 25... equal is good; use -q30).
        let b30 = behavior_q(30.0);
        assert_eq!(
            testTriangleAngle(
                &b30,
                0.0,
                0.0,
                1.0,
                0.0,
                0.5,
                0.5 * (25.0f64).to_radians().tan()
            ),
            1
        );
        // The equilateral triangle is good under -q20 -u120 too
        // (max 60 < 120).
        assert_eq!(
            testTriangleAngle(&bu, 0.0, 0.0, 1.0, 0.0, 0.5, 0.8660254037844386),
            0
        );
    }

    #[test]
    fn get_star_points_walks_the_ring_ccw() {
        // A fan of 4 triangles around the origin with the unit-square
        // ring: the star of the center, walked from badotri = T0 =
        // (c, r0, r1) with whichPoint = 1 (center torg, second = tapex
        // r1, terminator = tdest r0), is the ring starting at r1 and
        // stepping CCW: r1, r2, r3, r0 (acute.c:2091-2110).
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        let (c, _r, tris) = star_fan(
            &mut m,
            (0.0, 0.0),
            &[(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)],
        );
        let mut points = [0.0f64; 500];
        let n = getStarPoints(
            &m,
            tris[0],
            c,
            m.dest(tris[0]),
            m.apex(tris[0]),
            1,
            &mut points,
        );
        assert_eq!(n, 4);
        assert_eq!(points[..8], [0.0, 1.0, -1.0, 0.0, 0.0, -1.0, 1.0, 0.0]);
        // whichPoint = 2 walks the star of tdest (r0): a ring vertex on
        // the hull whose ring edge (r0, r1) is unbonded: the walk dies
        // at the second step and reports 0 (the border case).
        let n2 = getStarPoints(
            &m,
            tris[0],
            c,
            m.dest(tris[0]),
            m.apex(tris[0]),
            2,
            &mut points,
        );
        assert_eq!(n2, 0);
        // whichPoint = 3 (tapex r1) dies immediately: the pair
        // (r1, r0) spans the unbonded hull edge.
        let n3 = getStarPoints(
            &m,
            tris[0],
            c,
            m.dest(tris[0]),
            m.apex(tris[0]),
            3,
            &mut points,
        );
        assert_eq!(n3, 0);
    }

    #[test]
    fn get_star_points_border_returns_zero() {
        // A lone unbonded triangle: every sym hits the dummy, the
        // walk's first getNeighborsVertex reports not-found and the
        // count collapses to 0 (acute.c:2104-2107).
        let b = Behavior::default_form();
        let mut m = mesh(&b);
        let (t, va, vb, vc) = lone_tri(&mut m, (0.0, 0.0), (2.0, 0.0), (1.0, 2.0));
        let mut points = [0.0f64; 500];
        assert_eq!(getStarPoints(&m, t, va, vb, vc, 1, &mut points), 0);
    }

    #[test]
    fn get_neighbors_vertex_found_and_not_found() {
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        let (c, _r, tris) = star_fan(
            &mut m,
            (0.0, 0.0),
            &[(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)],
        );
        // The edge (c, r1) spans T0's orient-2 side and T1's orient-0
        // side: the neighbor's third vertex is r2 = (-1, 0), and the
        // returned handle is T1 at orient 0 (org c, dest r1).
        let mut third = [0.0f64; 2];
        let mut neigh = Otri {
            tri: DUMMY_TRI,
            orient: 0,
        };
        let cc = (m.vx_real(c, 0), m.vx_real(c, 1));
        let r1v = m.apex(tris[0]);
        let r1 = (m.vx_real(r1v, 0), m.vx_real(r1v, 1));
        assert_eq!(
            getNeighborsVertex(&m, tris[0], cc.0, cc.1, r1.0, r1.1, &mut third, &mut neigh),
            0
        );
        assert_eq!(third, [-1.0, 0.0]);
        assert_eq!(m.org(neigh), c);
        assert_eq!(m.dest(neigh), r1v);
        // The hull edge (r0, r1): no neighbor contains both, so the
        // orientation loop runs to the end unmatched and notFound
        // returns 1; C's *neighotri = neighbor (acute.c:2238) then
        // writes the last sym'd neighbor, the orient-2 side (r1, c),
        // whose sym is the real T1 (not the dummy of the orient-1
        // hull side, which the loop has already moved past).
        let r0v = m.dest(tris[0]);
        let r0 = (m.vx_real(r0v, 0), m.vx_real(r0v, 1));
        assert_eq!(
            getNeighborsVertex(&m, tris[0], r0.0, r0.1, r1.0, r1.1, &mut third, &mut neigh),
            1
        );
        assert_eq!(neigh.tri, tris[1].tri);
    }

    #[test]
    fn wedge_without_max_angle_finds_the_feasible_region() {
        // The unit-square ring at -q20: the origin sees every ring
        // edge at 90 degrees, well inside every petal (>= 20) and
        // every petal-chord halfplane, so the wedge intersection is
        // nonempty and its centroid is a feasible new location
        // (return 1, acute.c:2471).
        let b = behavior_q(20.0);
        let mut pool = AcutePool::new(20);
        let square = [1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0];
        let mut newloc = [0.0f64; 2];
        assert_eq!(
            getWedgeIntersectionWithoutMaxAngle(&mut pool, &b, 4, &square, &mut newloc),
            1
        );
        // The centroid lies inside the ring (the feasible region is a
        // convex piece around the origin), never out at a petal tip.
        assert!(newloc[0].abs() < 1.0 && newloc[1].abs() < 1.0);
        // numpoints = 4 stays below the pool's size: no resize.
        assert_eq!(pool.size, 20);
    }

    #[test]
    fn wedge_without_max_angle_fixedarea_falls_through() {
        // C's fixedarea arm is entirely commented out (acute.c:2451-2464):
        // even when the region exists, a fixedarea run falls
        // through to the final return 0. Pinned on the same feasible
        // square ring as the found case.
        let mut b = behavior_q(20.0);
        b.fixedarea = true;
        let mut pool = AcutePool::new(20);
        let square = [1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0];
        let mut newloc = [0.0f64; 2];
        assert_eq!(
            getWedgeIntersectionWithoutMaxAngle(&mut pool, &b, 4, &square, &mut newloc),
            0
        );
    }

    #[test]
    fn wedge_without_max_angle_infeasible_star_returns_zero() {
        // -q89 on the unit-square ring: the min-angle wedges collapse
        // to slivers hugging each edge; slivers of distinct edges do
        // not overlap, so a halfPlaneIntersection cut empties the
        // polygon and the walk returns 0 (acute.c:2429-2430, 2438-2439).
        let b = behavior_q(89.0);
        let mut pool = AcutePool::new(20);
        let square = [1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0];
        let mut newloc = [0.0f64; 2];
        assert_eq!(
            getWedgeIntersectionWithoutMaxAngle(&mut pool, &b, 4, &square, &mut newloc),
            0
        );
    }

    #[test]
    fn wedge_with_max_angle_four_point_case() {
        // alpha = 2*80 + 20 - 180 = 0: the 4-point case (acute.c:2617-2623),
        // only angle lines plus the two max-angle lines. The
        // hexagon ring (unit edges) is seen from the origin at 60
        // degrees, inside [20, 80] for every edge: the region exists,
        // and the centroid's triangles against every star edge pass
        // the testTriangleAngle scan (min 60 > 20, max 60 < 80), so
        // the scan accepts it without the k/e alternatives.
        let b = behavior_q_u(20.0, 80.0);
        let mut pool = AcutePool::new(20);
        let s = (3.0f64).sqrt() / 2.0;
        let hexagon = [
            1.0, 0.0, //
            0.5, s, //
            -0.5, s, //
            -1.0, 0.0, //
            -0.5, -s, //
            0.5, -s,
        ];
        let mut newloc = [0.0f64; 2];
        assert_eq!(
            getWedgeIntersectionWithMaxAngle(&mut pool, &b, 6, &hexagon, &mut newloc),
            1
        );
        assert!(newloc[0].abs() < 1.0 && newloc[1].abs() < 1.0);
        assert_eq!(pool.size, 20);
    }

    #[test]
    fn wedge_with_max_angle_ten_point_case() {
        // alpha = 2*140 + 20 - 180 = 120: the 10-point case
        // (acute.c:2636-2642): the full wedge with eight petal-chord lines and
        // the max-angle lines at 140. The hexagon ring passes the
        // same way (60 degrees against [20, 140]).
        let b = behavior_q_u(20.0, 140.0);
        let mut pool = AcutePool::new(20);
        let s = (3.0f64).sqrt() / 2.0;
        let hexagon = [
            1.0, 0.0, //
            0.5, s, //
            -0.5, s, //
            -1.0, 0.0, //
            -0.5, -s, //
            0.5, -s,
        ];
        let mut newloc = [0.0f64; 2];
        assert_eq!(
            getWedgeIntersectionWithMaxAngle(&mut pool, &b, 6, &hexagon, &mut newloc),
            1
        );
        assert!(newloc[0].abs() < 1.0 && newloc[1].abs() < 1.0);
    }

    #[test]
    fn min_distance_inside_vertex_edge_outside_and_collinear() {
        // One unbonded triangle (0,0), (2,0), (1,2); the squared
        // distance to the containing triangle's vertices is the
        // return (C's d1/d2/d3, acute.c:3646-3648).
        let b = Behavior::default_form();
        let mut m = mesh(&b);
        let (t, _va, _vb, _vc) = lone_tri(&mut m, (0.0, 0.0), (2.0, 0.0), (1.0, 2.0));
        // An interior point: min squared distance 1.25 to (0,0) and
        // (2,0).
        let mut st = t;
        assert_eq!(
            minDistanceToNeigbor(&mut m, &b, 1.0, 0.5, &mut st).expect("min distance"),
            1.25
        );
        // On the org vertex: 0.0 (the ONVERTEX arm).
        let mut st = t;
        assert_eq!(
            minDistanceToNeigbor(&mut m, &b, 0.0, 0.0, &mut st).expect("min distance"),
            0.0
        );
        // On the dest vertex: the lnextself branch rotates the caller's
        // handle (acute.c:3612) before returning 0.0.
        let mut st = t;
        assert_eq!(
            minDistanceToNeigbor(&mut m, &b, 2.0, 0.0, &mut st).expect("min distance"),
            0.0
        );
        assert_eq!(st.orient, 1);
        // On the edge interior: ONEDGE, squared distance 1.0 to
        // either endpoint.
        let mut st = t;
        assert_eq!(
            minDistanceToNeigbor(&mut m, &b, 1.0, 0.0, &mut st).expect("min distance"),
            1.0
        );
        // Outside the mesh: the walk crosses the hull rim and
        // preciselocate reports OUTSIDE: 0.0.
        let mut st = t;
        assert_eq!(
            minDistanceToNeigbor(&mut m, &b, -1.0, 1.0, &mut st).expect("min distance"),
            0.0
        );
        // Collinear with the (0,0)-(2,0) edge but not between: C
        // leaves intersect/horiz indeterminate here (acute.c:3626-
        // 3631); the port pins InTriangle + horiz = *searchtri (D25),
        // so the distance arm measures from the starting triangle:
        // squared distance 1.0 to (2,0).
        let mut st = t;
        assert_eq!(
            minDistanceToNeigbor(&mut m, &b, 3.0, 0.0, &mut st).expect("min distance"),
            1.0
        );
    }

    #[test]
    fn do_smoothing_relocates_the_free_center_vertex() {
        // End-to-end on the square fan: the center vertex c is free
        // (FREEVERTEX, the vertextype == 2 gate of acute.c:1916), its
        // star is the unit-square ring (all corners 90 degrees, so
        // polygonAngles passes at -q20), and the ring vertices sit on
        // the hull (their star walks die immediately, so only the p
        // try can produce a possibility). doSmoothing returns flag1
        // and writes the feasible-region centroid into newloc; the
        // scoped pool move (D23) restores the mesh-owned pool.
        let b = behavior_q(20.0);
        let mut m = mesh(&b);
        let (c, _r, tris) = star_fan(
            &mut m,
            (0.0, 0.0),
            &[(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)],
        );
        m.vx_set_type(c, crate::slots::FREEVERTEX);
        assert_eq!(m.vx_type(c), crate::slots::FREEVERTEX);
        let torg = c;
        let tdest = m.dest(tris[0]);
        let tapex = m.apex(tris[0]);
        let mut newloc = [0.0f64; 2];
        assert_eq!(
            doSmoothing(&mut m, &b, tris[0], torg, tdest, tapex, &mut newloc),
            1
        );
        // The suggested relocation is the feasible-region centroid:
        // inside the ring.
        assert!(newloc[0].abs() < 1.0 && newloc[1].abs() < 1.0);
        // The scoped pool move restored the pool: capacity and size
        // intact, the points buffers back in place.
        assert_eq!(m.acute_mem.size, 20);
        assert_eq!(m.acute_mem.points_p.len(), 500);
        assert_eq!(m.acute_mem.points_q.len(), 500);
        assert_eq!(m.acute_mem.points_r.len(), 500);
        // A non-free center cannot be relocated: the flag never sets
        // and doSmoothing reports 0 (the FREEVERTEX gate).
        m.vx_set_type(c, crate::slots::SEGMENTVERTEX);
        let mut newloc2 = [0.0f64; 2];
        assert_eq!(
            doSmoothing(&mut m, &b, tris[0], torg, tdest, tapex, &mut newloc2),
            0
        );
    }
}
