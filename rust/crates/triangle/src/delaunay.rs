//! Delaunay triangulation: the divide-and-conquer subtree (part 1:
//! vertexsort, vertexmedian, alternateaxes, mergehulls, divconqrecurse,
//! removeghosts, divconqdelaunay; plus the bounding box rows boundingbox
//! and removebox), the incremental and sweepline subtrees (part 2:
//! incrementaldelaunay, the event heap, the splay tree, rightofhyperbola,
//! circletop, check4deadevent, frontlocate, sweeplinedelaunay) and the
//! delaunay() dispatcher; triangle.c:3866-5454. Assignment: the part-1
//! and part-2 rows of docs/function-mapping.md. The api.rs scaffolding
//! of the triangulate entry point stays untouched until construct.rs
//! consumes the delaunay() dispatcher (part 2 lands its final internal
//! caller chain, so the module-level dead_code blanket below persists
//! until then).
//!
//! Transliteration notes (docs/deviation-register.md, D1-D12):
//! - C `vertex` mirrors to u32 vertex row ids (D1): compared for equality
//!   exactly where C compares pointers, and read through the chokepoint
//!   (vx_real) exactly where C dereferences. The same discipline carries
//!   part 2: C's event and splaynode pointers become arena indices and
//!   row ids, so the deterministic heap order and splay rotations never
//!   depend on allocation addresses.
//! - The D10 per-run seed (Mesh::randomseed) drives vertexsort's and
//!   vertexmedian's pivots and hence the whole merge walk: same seed,
//!   same swaps, same output. Part 2's draw sites (randomnation at
//!   triangle.c:5305, 5363, 5365, SAMPLERATE 10) mirror C's draw order
//!   and count exactly.
//! - C's #ifdef _DEBUG duplicate-vertex printfs (divconqdelaunay,
//!   triangle.c:4570-4573; sweeplinedelaunay/incrementaldelaunay,
//!   triangle.c:4774-4777, 5267-5271, 5317-5321) are compiled out of
//!   the pinned oracle build and are not transliterated; the
//!   setvertextype(UNDEADVERTEX)/undeads accounting that follows each
//!   is.
//! - The three "infinite" bounding box vertices (boundingbox) are C
//!   trimalloc allocations outside the vertex pool; the port mirror is
//!   the reserved out-of-pool INFVERTEX ids plus the owned store, per
//!   deviation D12 (slots.rs, mesh.rs::infvertex_store).
//! - The trimalloc'd scratch of part 1 (sortarray) and part 2 (the
//!   event arena, the event heap index array, the splay plumbing) is
//!   not mesh state, is not counted by C's triangle_memory, and
//!   mirrors to owned local containers passed as arguments, with
//!   allocation order and content sequence identical to C (D8/D11);
//!   reservation failure routes to the pool OOM bridge (`Oom`). The
//!   splaynode rows, by contrast, ride the landed splaynodes WordPool
//!   exactly where C uses its splaynodes pool; C's poolinit
//!   (triangle.c:5234) and pooldeinit (triangle.c:5408) dissolve (the
//!   eager Mesh::new init and a comment at the C deinit site), which
//!   keeps the pool's post-run maxitems statistics readable exactly
//!   like C's stale-pool reads.
//! - The event-arena cells are the D11 zero superset of C's
//!   uninitialized trimalloc garbage: C always writes xkey/ykey and
//!   eventptr before any read.
//! - The org-slot event pun: C stores a `struct event *` in the org
//!   slot of front handles (setorg, triangle.c:5388/5403; cleared at
//!   5016) and reads it back reinterpreted (triangle.c:5008) in
//!   exactly one function, check4deadevent. The port stores a tagged
//!   full-width word through slots.rs orgword/setorgword (the same
//!   slot org/setorg use); bit 32 tags the event, the low 32 bits are
//!   the arena index, and the all-zero word keeps the C NULL meaning.
//! - sweeplinedelaunay returns C's TRI_INPUT (-3) through the `long`
//!   channel when every input vertex is identical (triangle.c:5258);
//!   C's caller stores it as hullsize, no error raised. The port
//!   mirrors literally: Ok(InternalStatus::Input as i64) through the
//!   Result<i64, Oom> channel, flowing out of delaunay() the same way.
//! - Mesh::hyperbolacount/circletopcount and Mesh::xminextreme arrive
//!   with part 2 as landed fields (incremented exactly at C's sites;
//!   xminextreme written by the transfernodes row, ioarrays.rs, which
//!   lands later; the zero superset until then is D11).
//!
//! Stage-1 note: the consumers of this module are reachable only through
//! the api.rs entry points, which stay scaffolded until construct.rs
//! lands; the blanket below silences that ahead-of-use dead_code and is
//! removed when the scaffolding is consumed.
#![allow(dead_code)]

use crate::handles::{decode, encode, Otri, DUMMY_TRI, NULL};
use crate::mesh::Mesh;
use crate::options::Behavior;
use crate::pool::Oom;
use crate::predicates::{tri_counterclockwise, tri_incircle};
use crate::slots::{INFVERTEX1, INFVERTEX2, INFVERTEX3, UNDEADVERTEX};
use crate::transform::{flip, insertvertex, InsertVertexResult};
use crate::util::{randomnation, vertex_coords};

/// C: vertexsort (triangle.c:3870-3930). Sort an array of vertices by
/// x-coordinate, using the y-coordinate as a secondary key. Uses quicksort.
/// Randomized O(n log n) time. No, I did not make any of the usual
/// quicksort mistakes. (C comments preserved in the body.) The C
/// `arraysize` parameter is the slice length; the recursion passes
/// subslices exactly where C passes shifted pointers.
pub(crate) fn vertexsort(m: &mut Mesh, sortarray: &mut [u32]) {
    let arraysize = sortarray.len() as i32;

    let mut left: i32;
    let mut right: i32;

    if arraysize == 2 {
        // Recursive base case.
        let p0 = (m.vx_real(sortarray[0], 0), m.vx_real(sortarray[0], 1));
        let p1 = (m.vx_real(sortarray[1], 0), m.vx_real(sortarray[1], 1));
        if p0.0 > p1.0 || (p0.0 == p1.0 && p0.1 > p1.1) {
            // C swaps the two (the `vertex temp` local dissolves into the
            // tuple exchange; identical comparisons and swap order).
            sortarray.swap(0, 1);
        }
        return;
    }
    // Choose a random pivot to split the array.
    // C: pivot = (int) randomnation((unsigned int) arraysize);
    let pivot = randomnation(m, arraysize as u32) as usize;
    let pivotx = m.vx_real(sortarray[pivot], 0);
    let pivoty = m.vx_real(sortarray[pivot], 1);
    // Split the array.
    left = -1;
    right = arraysize;
    while left < right {
        // Search for a vertex whose x-coordinate is too large for the left.
        loop {
            left += 1;
            if !(left <= right
                && (m.vx_real(sortarray[left as usize], 0) < pivotx
                    || (m.vx_real(sortarray[left as usize], 0) == pivotx
                        && m.vx_real(sortarray[left as usize], 1) < pivoty)))
            {
                break;
            }
        }
        // Search for a vertex whose x-coordinate is too small for the right.
        loop {
            right -= 1;
            if !(left <= right
                && (m.vx_real(sortarray[right as usize], 0) > pivotx
                    || (m.vx_real(sortarray[right as usize], 0) == pivotx
                        && m.vx_real(sortarray[right as usize], 1) > pivoty)))
            {
                break;
            }
        }
        if left < right {
            // Swap the left and right vertices.
            sortarray.swap(left as usize, right as usize);
        }
    }
    if left > 1 {
        // Recursively sort the left subset.
        vertexsort(m, &mut sortarray[..left as usize]);
    }
    if right < arraysize - 2 {
        // Recursively sort the right subset.
        vertexsort(m, &mut sortarray[right as usize + 1..]);
    }
}

/// C: vertexmedian (triangle.c:3937-3998). An order statistic algorithm,
/// almost. Shuffles an array of vertices so that the first `median`
/// vertices occur lexicographically before the remaining vertices.
/// Uses the x-coordinate as the primary key if axis == 0; the
/// y-coordinate if axis == 1. Very similar to the vertexsort() procedure,
/// but runs in randomized linear time. (C comments preserved in the
/// body.)
pub(crate) fn vertexmedian(m: &mut Mesh, sortarray: &mut [u32], median: i32, axis: i32) {
    let arraysize = sortarray.len() as i32;
    let axis = axis as usize;

    let mut left: i32;
    let mut right: i32;

    if arraysize == 2 {
        // Recursive base case.
        let p0 = (
            m.vx_real(sortarray[0], axis),
            m.vx_real(sortarray[0], 1 - axis),
        );
        let p1 = (
            m.vx_real(sortarray[1], axis),
            m.vx_real(sortarray[1], 1 - axis),
        );
        if p0.0 > p1.0 || (p0.0 == p1.0 && p0.1 > p1.1) {
            sortarray.swap(0, 1);
        }
        return;
    }
    // Choose a random pivot to split the array.
    // C: pivot = (int) randomnation((unsigned int) arraysize);
    let pivot = randomnation(m, arraysize as u32) as usize;
    let pivot1 = m.vx_real(sortarray[pivot], axis);
    let pivot2 = m.vx_real(sortarray[pivot], 1 - axis);
    // Split the array.
    left = -1;
    right = arraysize;
    while left < right {
        // Search for a vertex whose x-coordinate is too large for the left.
        loop {
            left += 1;
            if !(left <= right
                && (m.vx_real(sortarray[left as usize], axis) < pivot1
                    || (m.vx_real(sortarray[left as usize], axis) == pivot1
                        && m.vx_real(sortarray[left as usize], 1 - axis) < pivot2)))
            {
                break;
            }
        }
        // Search for a vertex whose x-coordinate is too small for the right.
        loop {
            right -= 1;
            if !(left <= right
                && (m.vx_real(sortarray[right as usize], axis) > pivot1
                    || (m.vx_real(sortarray[right as usize], axis) == pivot1
                        && m.vx_real(sortarray[right as usize], 1 - axis) > pivot2)))
            {
                break;
            }
        }
        if left < right {
            // Swap the left and right vertices.
            sortarray.swap(left as usize, right as usize);
        }
    }
    // Unlike in vertexsort(), at most one of the following
    //   conditionals is true.
    if left > median {
        // Recursively shuffle the left subset.
        vertexmedian(m, &mut sortarray[..left as usize], median, axis as i32);
    }
    if right < median - 1 {
        // Recursively shuffle the right subset.
        vertexmedian(
            m,
            &mut sortarray[right as usize + 1..],
            median - right - 1,
            axis as i32,
        );
    }
}

/// C: alternateaxes (triangle.c:4006-4029). Sorts the vertices as
/// appropriate for the divide-and-conquer algorithm with alternating
/// cuts. Partitions by x-coordinate if axis == 0; by y-coordinate if
/// axis == 1. For the base case, subsets containing only two or three
/// vertices are always sorted by x-coordinate. (C comments preserved in
/// the body.)
pub(crate) fn alternateaxes(m: &mut Mesh, sortarray: &mut [u32], mut axis: i32) {
    let arraysize = sortarray.len() as i32;

    let divider = arraysize >> 1;
    if arraysize <= 3 {
        // Recursive base case:  subsets of two or three vertices will be
        //   handled specially, and should always be sorted by x-coordinate.
        axis = 0;
    }
    // Partition with a horizontal or vertical cut.
    vertexmedian(m, sortarray, divider, axis);
    // Recursively partition the subsets with a cross cut.
    if arraysize - divider >= 2 {
        if divider >= 2 {
            alternateaxes(m, &mut sortarray[..divider as usize], 1 - axis);
        }
        alternateaxes(m, &mut sortarray[divider as usize..], 1 - axis);
    }
}

/// C: mergehulls (triangle.c:4062-4354). Merge two adjacent Delaunay
/// triangulations into a single Delaunay triangulation. This is similar
/// to the algorithm given by Guibas and Stolfi, but uses a triangle-based,
/// rather than edge-based, data structure. The algorithm walks up the gap
/// between the two triangulations, knitting them together. As they are
/// merged, some of their bounding triangles are converted into real
/// triangles of the triangulation. The procedure pulls each hull's
/// bounding triangles apart, then knits them together like the teeth of
/// two gears. The Delaunay property determines, at each step, whether
/// the next "tooth" is a bounding triangle of the left hull or the right.
/// When a bounding triangle becomes real, its apex is changed from NULL
/// to a real vertex. Only two new triangles need to be allocated. These
/// become new bounding triangles at the top and bottom of the seam. They
/// are used to connect the remaining bounding triangles (those that
/// have not been converted into real triangles) into a single fan.
/// On entry, `farleft' and `innerleft' are bounding triangles of the
/// left triangulation. The origin of `farleft' is the leftmost vertex,
/// and the destination of `innerleft' is the rightmost vertex of the
/// triangulation. Similarly, `innerright' and `farright' are bounding
/// triangles of the right triangulation. The origin of `innerright' and
/// destination of `farright' are the leftmost and rightmost vertices.
/// On completion, the origin of `farleft' is the leftmost vertex of the
/// merged triangulation, and the destination of `farright' is the
/// rightmost vertex. (C comments preserved in the body.)
///
/// C's `vertex ptr` temporary of sym() disappears (D6, the handle methods
/// take no hidden locals). The C function-scope extremal-pointer locals
/// (`farleftpt' &c) are declared unassigned in C and read only after an
/// assignment on the walked path; the port initializes the id superset
/// with the null id.
// C computes the same dead dwyer-path values (they are live only for
// _DEBUG printf's that the pinned oracle build never runs); the port
// additionally seeds the uninitialized C locals with the null id, so the
// flow stays verbatim and the dead stores are silenced here.
#[allow(unused_assignments)]
#[allow(clippy::too_many_arguments)] // the C signature carries seven parameters
pub(crate) fn mergehulls(
    m: &mut Mesh,
    b: &Behavior,
    farleft: &mut Otri,
    innerleft: &mut Otri,
    innerright: &mut Otri,
    farright: &mut Otri,
    axis: i32,
) -> Result<(), Oom> {
    let mut innerleftdest = m.dest(*innerleft);
    let mut innerleftapex = m.apex(*innerleft);
    let mut innerrightorg = m.org(*innerright);
    let mut innerrightapex = m.apex(*innerright);
    let mut farleftpt = NULL;
    let mut farleftapex = NULL;
    let mut farrightpt = NULL;
    let mut farrightapex = NULL;
    // Special treatment for horizontal cuts.
    if b.dwyer && (axis == 1) {
        farleftpt = m.org(*farleft);
        farleftapex = m.apex(*farleft);
        farrightpt = m.dest(*farright);
        farrightapex = m.apex(*farright);
        // The pointers to the extremal vertices are shifted to point to the
        //   topmost and bottommost vertex of each hull, rather than the
        //   leftmost and rightmost vertices.
        while m.vx_real(farleftapex, 1) < m.vx_real(farleftpt, 1) {
            farleft.lnextself();
            m.symself(farleft);
            farleftpt = farleftapex;
            farleftapex = m.apex(*farleft);
        }
        let mut checkedge = m.sym(*innerleft);
        let mut checkvertex = m.apex(checkedge);
        while m.vx_real(checkvertex, 1) > m.vx_real(innerleftdest, 1) {
            *innerleft = checkedge.lnext();
            innerleftapex = innerleftdest;
            innerleftdest = checkvertex;
            checkedge = m.sym(*innerleft);
            checkvertex = m.apex(checkedge);
        }
        while m.vx_real(innerrightapex, 1) < m.vx_real(innerrightorg, 1) {
            innerright.lnextself();
            m.symself(innerright);
            innerrightorg = innerrightapex;
            innerrightapex = m.apex(*innerright);
        }
        let mut checkedge = m.sym(*farright);
        let mut checkvertex = m.apex(checkedge);
        while m.vx_real(checkvertex, 1) > m.vx_real(farrightpt, 1) {
            *farright = checkedge.lnext();
            farrightapex = farrightpt;
            farrightpt = checkvertex;
            checkedge = m.sym(*farright);
            checkvertex = m.apex(checkedge);
        }
    }
    // Find a line tangent to and below both hulls.
    loop {
        let mut changemade = false;
        // Make innerleftdest the "bottommost" vertex of the left hull.
        if tri_counterclockwise(
            m,
            b,
            vertex_coords(m, innerleftdest),
            vertex_coords(m, innerleftapex),
            vertex_coords(m, innerrightorg),
        ) > 0.0
        {
            innerleft.lprevself();
            m.symself(innerleft);
            innerleftdest = innerleftapex;
            innerleftapex = m.apex(*innerleft);
            changemade = true;
        }
        // Make innerrightorg the "bottommost" vertex of the right hull.
        if tri_counterclockwise(
            m,
            b,
            vertex_coords(m, innerrightapex),
            vertex_coords(m, innerrightorg),
            vertex_coords(m, innerleftdest),
        ) > 0.0
        {
            innerright.lnextself();
            m.symself(innerright);
            innerrightorg = innerrightapex;
            innerrightapex = m.apex(*innerright);
            changemade = true;
        }
        if !changemade {
            break;
        }
    }
    // Find the two candidates to be the next "gear tooth."
    let mut leftcand = m.sym(*innerleft);
    let mut rightcand = m.sym(*innerright);
    // Create the bottom new bounding triangle.
    let mut baseedge = m.maketriangle(b.usesegments, b.vararea)?;
    // Connect it to the bounding boxes of the left and right triangulations.
    m.bond(baseedge, *innerleft);
    baseedge.lnextself();
    m.bond(baseedge, *innerright);
    baseedge.lnextself();
    m.setorg(baseedge, innerrightorg);
    m.setdest(baseedge, innerleftdest);
    // Apex is intentionally left NULL.

    // Fix the extreme triangles if necessary.
    farleftpt = m.org(*farleft);
    if innerleftdest == farleftpt {
        *farleft = baseedge.lnext();
    }
    farrightpt = m.dest(*farright);
    if innerrightorg == farrightpt {
        *farright = baseedge.lprev();
    }
    // The vertices of the current knitting edge.
    let mut lowerleft = innerleftdest;
    let mut lowerright = innerrightorg;
    // The candidate vertices for knitting.
    let mut upperleft = m.apex(leftcand);
    let mut upperright = m.apex(rightcand);
    // Walk up the gap between the two triangulations, knitting them together.
    loop {
        // Have we reached the top?  (This isn't quite the right question,
        //   because even though the left triangulation might seem finished now,
        //   moving up on the right triangulation might reveal a new vertex of
        //   the left triangulation.  And vice-versa.)
        let leftfinished = tri_counterclockwise(
            m,
            b,
            vertex_coords(m, upperleft),
            vertex_coords(m, lowerleft),
            vertex_coords(m, lowerright),
        ) <= 0.0;
        let rightfinished = tri_counterclockwise(
            m,
            b,
            vertex_coords(m, upperright),
            vertex_coords(m, lowerleft),
            vertex_coords(m, lowerright),
        ) <= 0.0;
        if leftfinished && rightfinished {
            // Create the top new bounding triangle.
            let mut nextedge = m.maketriangle(b.usesegments, b.vararea)?;
            m.setorg(nextedge, lowerleft);
            m.setdest(nextedge, lowerright);
            // Apex is intentionally left NULL.
            // Connect it to the bounding boxes of the two triangulations.
            m.bond(nextedge, baseedge);
            nextedge.lnextself();
            m.bond(nextedge, rightcand);
            nextedge.lnextself();
            m.bond(nextedge, leftcand);
            // Special treatment for horizontal cuts.
            if b.dwyer && (axis == 1) {
                farleftpt = m.org(*farleft);
                farleftapex = m.apex(*farleft);
                farrightpt = m.dest(*farright);
                farrightapex = m.apex(*farright);
                let mut checkedge = m.sym(*farleft);
                let mut checkvertex = m.apex(checkedge);
                // The pointers to the extremal vertices are restored to the
                //   leftmost and rightmost vertices (rather than topmost and
                //   bottommost).
                while m.vx_real(checkvertex, 0) < m.vx_real(farleftpt, 0) {
                    *farleft = checkedge.lprev();
                    farleftapex = farleftpt;
                    farleftpt = checkvertex;
                    checkedge = m.sym(*farleft);
                    checkvertex = m.apex(checkedge);
                }
                while m.vx_real(farrightapex, 0) > m.vx_real(farrightpt, 0) {
                    farright.lprevself();
                    m.symself(farright);
                    farrightpt = farrightapex;
                    farrightapex = m.apex(*farright);
                }
            }
            return Ok(());
        }
        // Consider eliminating edges from the left triangulation.
        if !leftfinished {
            // What vertex would be exposed if an edge were deleted?
            let mut nextedge = leftcand.lprev();
            m.symself(&mut nextedge);
            let mut nextapex = m.apex(nextedge);
            // If nextapex is NULL, then no vertex would be exposed; the
            //   triangulation would have been eaten right through.
            if nextapex != NULL {
                // Check whether the edge is Delaunay.
                let mut badedge = tri_incircle(
                    m,
                    b,
                    vertex_coords(m, lowerleft),
                    vertex_coords(m, lowerright),
                    vertex_coords(m, upperleft),
                    vertex_coords(m, nextapex),
                ) > 0.0;
                while badedge {
                    // Eliminate the edge with an edge flip.  As a result, the
                    //   left triangulation will have one more boundary triangle.
                    nextedge.lnextself();
                    let topcasing = m.sym(nextedge);
                    nextedge.lnextself();
                    let sidecasing = m.sym(nextedge);
                    m.bond(nextedge, topcasing);
                    m.bond(leftcand, sidecasing);
                    leftcand.lnextself();
                    let outercasing = m.sym(leftcand);
                    nextedge.lprevself();
                    m.bond(nextedge, outercasing);
                    // Correct the vertices to reflect the edge flip.
                    m.setorg(leftcand, lowerleft);
                    m.setdest(leftcand, NULL);
                    m.setapex(leftcand, nextapex);
                    m.setorg(nextedge, NULL);
                    m.setdest(nextedge, upperleft);
                    m.setapex(nextedge, nextapex);
                    // Consider the newly exposed vertex.
                    upperleft = nextapex;
                    // What vertex would be exposed if another edge were deleted?
                    // C: otricopy(sidecasing, nextedge).
                    nextedge = sidecasing;
                    nextapex = m.apex(nextedge);
                    if nextapex != NULL {
                        // Check whether the edge is Delaunay.
                        badedge = tri_incircle(
                            m,
                            b,
                            vertex_coords(m, lowerleft),
                            vertex_coords(m, lowerright),
                            vertex_coords(m, upperleft),
                            vertex_coords(m, nextapex),
                        ) > 0.0;
                    } else {
                        // Avoid eating right through the triangulation.
                        badedge = false;
                    }
                }
            }
        }
        // Consider eliminating edges from the right triangulation.
        if !rightfinished {
            // What vertex would be exposed if an edge were deleted?
            let mut nextedge = rightcand.lnext();
            m.symself(&mut nextedge);
            let mut nextapex = m.apex(nextedge);
            // If nextapex is NULL, then no vertex would be exposed; the
            //   triangulation would have been eaten right through.
            if nextapex != NULL {
                // Check whether the edge is Delaunay.
                let mut badedge = tri_incircle(
                    m,
                    b,
                    vertex_coords(m, lowerleft),
                    vertex_coords(m, lowerright),
                    vertex_coords(m, upperright),
                    vertex_coords(m, nextapex),
                ) > 0.0;
                while badedge {
                    // Eliminate the edge with an edge flip.  As a result, the
                    //   right triangulation will have one more boundary triangle.
                    nextedge.lprevself();
                    let topcasing = m.sym(nextedge);
                    nextedge.lprevself();
                    let sidecasing = m.sym(nextedge);
                    m.bond(nextedge, topcasing);
                    m.bond(rightcand, sidecasing);
                    rightcand.lprevself();
                    let outercasing = m.sym(rightcand);
                    nextedge.lnextself();
                    m.bond(nextedge, outercasing);
                    // Correct the vertices to reflect the edge flip.
                    m.setorg(rightcand, NULL);
                    m.setdest(rightcand, lowerright);
                    m.setapex(rightcand, nextapex);
                    m.setorg(nextedge, upperright);
                    m.setdest(nextedge, NULL);
                    m.setapex(nextedge, nextapex);
                    // Consider the newly exposed vertex.
                    upperright = nextapex;
                    // What vertex would be exposed if another edge were deleted?
                    // C: otricopy(sidecasing, nextedge).
                    nextedge = sidecasing;
                    nextapex = m.apex(nextedge);
                    if nextapex != NULL {
                        // Check whether the edge is Delaunay.
                        badedge = tri_incircle(
                            m,
                            b,
                            vertex_coords(m, lowerleft),
                            vertex_coords(m, lowerright),
                            vertex_coords(m, upperright),
                            vertex_coords(m, nextapex),
                        ) > 0.0;
                    } else {
                        // Avoid eating right through the triangulation.
                        badedge = false;
                    }
                }
            }
        }
        if leftfinished
            || (!rightfinished
                && tri_incircle(
                    m,
                    b,
                    vertex_coords(m, upperleft),
                    vertex_coords(m, lowerleft),
                    vertex_coords(m, lowerright),
                    vertex_coords(m, upperright),
                ) > 0.0)
        {
            // Knit the triangulations, adding an edge from `lowerleft'
            //   to `upperright'.
            m.bond(baseedge, rightcand);
            baseedge = rightcand.lprev();
            m.setdest(baseedge, lowerleft);
            lowerright = upperright;
            rightcand = m.sym(baseedge);
            upperright = m.apex(rightcand);
        } else {
            // Knit the triangulations, adding an edge from `upperleft'
            //   to `lowerright'.
            m.bond(baseedge, leftcand);
            baseedge = leftcand.lnext();
            m.setorg(baseedge, lowerright);
            lowerleft = upperleft;
            leftcand = m.sym(baseedge);
            upperleft = m.apex(leftcand);
        }
    }
}

/// C: divconqrecurse (triangle.c:4366-4488). Recursively form a Delaunay
/// triangulation by the divide-and-conquer method. Recursively breaks
/// down the problem into smaller pieces, which are knitted together by
/// mergehulls(). The base cases (problems of two or three vertices) are
/// handled specially here. On completion, `farleft' and `farright' are
/// bounding triangles such that the origin of `farleft' is the leftmost
/// vertex (breaking ties by choosing the highest leftmost vertex), and
/// the destination of `farright' is the rightmost vertex (breaking ties
/// by choosing the lowest rightmost vertex). (C comments preserved in the
/// body.)
#[allow(clippy::too_many_arguments)]
// the C signature carries eight parameters
// C's explicit base-case `return;` statements are kept verbatim; the
// alternative of falling out of the else-if chain is C-invisible control
// flow.
#[allow(clippy::needless_return)]
pub(crate) fn divconqrecurse(
    m: &mut Mesh,
    b: &Behavior,
    sortarray: &[u32],
    vertices: i32,
    axis: i32,
    farleft: &mut Otri,
    farright: &mut Otri,
) -> Result<(), Oom> {
    if vertices == 2 {
        // The triangulation of two vertices is an edge.  An edge is
        //   represented by two bounding triangles.
        *farleft = m.maketriangle(b.usesegments, b.vararea)?;
        m.setorg(*farleft, sortarray[0]);
        m.setdest(*farleft, sortarray[1]);
        // The apex is intentionally left NULL.
        *farright = m.maketriangle(b.usesegments, b.vararea)?;
        m.setorg(*farright, sortarray[1]);
        m.setdest(*farright, sortarray[0]);
        // The apex is intentionally left NULL.
        m.bond(*farleft, *farright);
        farleft.lprevself();
        farright.lnextself();
        m.bond(*farleft, *farright);
        farleft.lprevself();
        farright.lnextself();
        m.bond(*farleft, *farright);
        // Ensure that the origin of `farleft' is sortarray[0].
        // C: lprev(*farright, *farleft).
        *farleft = farright.lprev();
        return Ok(());
    } else if vertices == 3 {
        // The triangulation of three vertices is either a triangle (with
        //   three bounding triangles) or two edges (with four bounding
        //   triangles).  In either case, four triangles are created.
        let mut midtri = m.maketriangle(b.usesegments, b.vararea)?;
        let mut tri1 = m.maketriangle(b.usesegments, b.vararea)?;
        let mut tri2 = m.maketriangle(b.usesegments, b.vararea)?;
        let mut tri3 = m.maketriangle(b.usesegments, b.vararea)?;
        let area = tri_counterclockwise(
            m,
            b,
            vertex_coords(m, sortarray[0]),
            vertex_coords(m, sortarray[1]),
            vertex_coords(m, sortarray[2]),
        );
        if area == 0.0 {
            // Three collinear vertices; the triangulation is two edges.
            m.setorg(midtri, sortarray[0]);
            m.setdest(midtri, sortarray[1]);
            m.setorg(tri1, sortarray[1]);
            m.setdest(tri1, sortarray[0]);
            m.setorg(tri2, sortarray[2]);
            m.setdest(tri2, sortarray[1]);
            m.setorg(tri3, sortarray[1]);
            m.setdest(tri3, sortarray[2]);
            // All apices are intentionally left NULL.
            m.bond(midtri, tri1);
            m.bond(tri2, tri3);
            midtri.lnextself();
            tri1.lprevself();
            tri2.lnextself();
            tri3.lprevself();
            m.bond(midtri, tri3);
            m.bond(tri1, tri2);
            midtri.lnextself();
            tri1.lprevself();
            tri2.lnextself();
            tri3.lprevself();
            m.bond(midtri, tri1);
            m.bond(tri2, tri3);
            // Ensure that the origin of `farleft' is sortarray[0].
            // C: otricopy(tri1, *farleft)
            *farleft = tri1;
            // Ensure that the destination of `farright' is sortarray[2].
            // C: otricopy(tri2, *farright)
            *farright = tri2;
        } else {
            // The three vertices are not collinear; the triangulation is one
            //   triangle, namely `midtri'.
            m.setorg(midtri, sortarray[0]);
            m.setdest(tri1, sortarray[0]);
            m.setorg(tri3, sortarray[0]);
            // Apices of tri1, tri2, and tri3 are left NULL.
            if area > 0.0 {
                // The vertices are in counterclockwise order.
                m.setdest(midtri, sortarray[1]);
                m.setorg(tri1, sortarray[1]);
                m.setdest(tri2, sortarray[1]);
                m.setapex(midtri, sortarray[2]);
                m.setorg(tri2, sortarray[2]);
                m.setdest(tri3, sortarray[2]);
            } else {
                // The vertices are in clockwise order.
                m.setdest(midtri, sortarray[2]);
                m.setorg(tri1, sortarray[2]);
                m.setdest(tri2, sortarray[2]);
                m.setapex(midtri, sortarray[1]);
                m.setorg(tri2, sortarray[1]);
                m.setdest(tri3, sortarray[1]);
            }
            // The topology does not depend on how the vertices are ordered.
            m.bond(midtri, tri1);
            midtri.lnextself();
            m.bond(midtri, tri2);
            midtri.lnextself();
            m.bond(midtri, tri3);
            tri1.lprevself();
            tri2.lnextself();
            m.bond(tri1, tri2);
            tri1.lprevself();
            tri3.lprevself();
            m.bond(tri1, tri3);
            tri2.lnextself();
            tri3.lprevself();
            m.bond(tri2, tri3);
            // Ensure that the origin of `farleft' is sortarray[0].
            // C: otricopy(tri1, *farleft)
            *farleft = tri1;
            // Ensure that the destination of `farright' is sortarray[2].
            if area > 0.0 {
                // C: otricopy(tri2, *farright)
                *farright = tri2;
            } else {
                // C: lnext(*farleft, *farright)
                *farright = farleft.lnext();
            }
        }
        return Ok(());
    } else {
        // Split the vertices in half.
        let divider = vertices >> 1;
        // The C function-scope locals are unassigned until the recursive
        // calls write them; the port initializes the null-handle
        // superset, which is never read before the calls return.
        let mut innerleft = Otri {
            tri: NULL,
            orient: 0,
        };
        let mut innerright = Otri {
            tri: NULL,
            orient: 0,
        };
        // Recursively triangulate each half.
        divconqrecurse(
            m,
            b,
            &sortarray[..divider as usize],
            divider,
            1 - axis,
            farleft,
            &mut innerleft,
        )?;
        divconqrecurse(
            m,
            b,
            &sortarray[divider as usize..],
            vertices - divider,
            1 - axis,
            &mut innerright,
            farright,
        )?;
        // Merge the two triangulations into one.
        mergehulls(
            m,
            b,
            farleft,
            &mut innerleft,
            &mut innerright,
            farright,
            axis,
        )
    }
}

/// C: removeghosts (triangle.c:4495-4532). Find an edge on the convex
/// hull to start point location from; remove the bounding box and count
/// the convex hull edges, setting boundary markers as appropriate.
/// Returns the number of edges on the convex hull of the triangulation.
/// (C comments preserved in the body.)
pub(crate) fn removeghosts(m: &mut Mesh, b: &Behavior, startghost: &Otri) -> i64 {
    // Find an edge on the convex hull to start point location from.
    // C: lprev(*startghost, searchedge).
    let mut searchedge = startghost.lprev();
    m.symself(&mut searchedge);
    // C: m->dummytri[0] = encode(searchedge).
    m.set_tri_slot(DUMMY_TRI, 0, encode(searchedge));
    // Remove the bounding box and count the convex hull edges.
    // C: otricopy(*startghost, dissolveedge).
    let mut dissolveedge = *startghost;
    let mut hullsize: i64 = 0;
    loop {
        hullsize += 1;
        // C: lnext(dissolveedge, deadtriangle).
        let deadtriangle = dissolveedge.lnext();
        dissolveedge.lprevself();
        m.symself(&mut dissolveedge);
        // If no PSLG is involved, set the boundary markers of all the
        //   vertices on the convex hull.  If a PSLG is used, this step is
        //   done later.
        if !b.poly {
            // Watch out for the case where all the input vertices are
            //   collinear.
            if dissolveedge.tri != DUMMY_TRI {
                let markorg = m.org(dissolveedge);
                if m.vx_mark(markorg) == 0 {
                    m.vx_set_mark(markorg, 1);
                }
            }
        }
        // Remove a bounding triangle from a convex hull triangle.
        m.dissolve(dissolveedge);
        // Find the next bounding triangle.
        // C: sym(deadtriangle, dissolveedge).
        dissolveedge = m.sym(deadtriangle);
        // Delete the bounding triangle.
        // C: triangledealloc(m, deadtriangle.tri).
        m.tri_dealloc(deadtriangle.tri);
        // C: do-while (!otriequal(dissolveedge, *startghost)).
        if dissolveedge == *startghost {
            break;
        }
    }
    hullsize
}

/// C: divconqdelaunay (triangle.c:4547-4587). Form a Delaunay
/// triangulation by the divide-and-conquer method. Sorts the vertices,
/// calls a recursive procedure to triangulate them, and removes the
/// bounding box, setting boundary markers as appropriate. Returns the
/// number of edges on the convex hull of the triangulation. (C comments
/// preserved in the body.)
pub(crate) fn divconqdelaunay(m: &mut Mesh, b: &Behavior) -> Result<i64, Oom> {
    // Allocate an array of pointers to vertices for sorting. C:
    // trimalloc(m->invertices * sizeof(vertex)); the funnel mirror is
    // try_reserve + Oom (D8/D11: no process exit, no manual free).
    let mut sortarray: Vec<u32> = Vec::new();
    sortarray
        .try_reserve_exact(m.invertices as usize)
        .map_err(|_| Oom)?;
    // C: traversalinit(&m->vertices) (the pool traversal cursor).
    let mut tr = m.vertices.traversal();
    for _ in 0..m.invertices as usize {
        let v = m.vertextraverse(&mut tr);
        sortarray.push(v.unwrap_or(NULL));
    }
    // Sort the vertices.
    vertexsort(m, &mut sortarray);
    // Discard duplicate vertices, which can really mess up the algorithm.
    let mut i: usize = 0;
    for j in 1..m.invertices as usize {
        if m.vx_real(sortarray[i], 0) == m.vx_real(sortarray[j], 0)
            && m.vx_real(sortarray[i], 1) == m.vx_real(sortarray[j], 1)
        {
            // C prints the duplicate-vertex warning only under _DEBUG
            // (not compiled in the pinned oracle build); the accounting
            // below is C's regardless.
            m.vx_set_type(sortarray[j], UNDEADVERTEX);
            m.undeads += 1;
        } else {
            i += 1;
            sortarray[i] = sortarray[j];
        }
    }
    i += 1;
    if b.dwyer {
        // Re-sort the array of vertices to accommodate alternating cuts.
        let divider = i >> 1;
        if i - divider >= 2 {
            if divider >= 2 {
                alternateaxes(m, &mut sortarray[..divider], 1);
            }
            alternateaxes(m, &mut sortarray[divider..], 1);
        }
    }

    // Form the Delaunay triangulation.
    let mut hullleft = Otri {
        tri: NULL,
        orient: 0,
    };
    let mut hullright = Otri {
        tri: NULL,
        orient: 0,
    };
    divconqrecurse(
        m,
        b,
        &sortarray[..i],
        i as i32,
        0,
        &mut hullleft,
        &mut hullright,
    )?;
    // C: trifree((VOID *) sortarray); the Drop dissolves the free (D11).
    drop(sortarray);

    Ok(removeghosts(m, b, &hullleft))
}

/// C: boundingbox (triangle.c:4620-4650). Form an "infinite" bounding
/// triangle to insert vertices into. The vertices at "infinity" are
/// assigned finite coordinates, which are used by the point location
/// routines, but (mostly) ignored by the Delaunay edge flip routines.
/// (C comments preserved in the body.)
///
/// The three trimalloc'ed vertices of C land under the reserved D12
/// out-of-pool INFVERTEX ids (slots.rs; see the deviation register).
pub(crate) fn boundingbox(m: &mut Mesh, b: &Behavior) -> Result<(), Oom> {
    // Find the width (or height, whichever is larger) of the triangulation.
    let mut width = m.xmax - m.xmin;
    if m.ymax - m.ymin > width {
        width = m.ymax - m.ymin;
    }
    if width == 0.0 {
        width = 1.0;
    }
    // Create the vertices of the bounding box.
    // C: three trimalloc(m->vertices.itembytes) rows outside the vertex
    // pool; the port's D12 mirror: reserved ids + the owned store, written
    // through the chokepoint.
    m.infvertex1 = INFVERTEX1;
    m.infvertex2 = INFVERTEX2;
    m.infvertex3 = INFVERTEX3;
    m.vx_set_real(m.infvertex1, 0, m.xmin - 50.0 * width);
    m.vx_set_real(m.infvertex1, 1, m.ymin - 40.0 * width);
    m.vx_set_real(m.infvertex2, 0, m.xmax + 50.0 * width);
    m.vx_set_real(m.infvertex2, 1, m.ymin - 40.0 * width);
    m.vx_set_real(m.infvertex3, 0, 0.5 * (m.xmin + m.xmax));
    m.vx_set_real(m.infvertex3, 1, m.ymax + 60.0 * width);

    // Create the bounding box.
    let inftri = m.maketriangle(b.usesegments, b.vararea)?;
    m.setorg(inftri, m.infvertex1);
    m.setdest(inftri, m.infvertex2);
    m.setapex(inftri, m.infvertex3);
    // Link dummytri to the bounding box so we can always find an
    //   edge to begin searching (point location) from.
    // C: m->dummytri[0] = (triangle) inftri.tri (the raw pointer carries
    // orientation zero).
    m.set_tri_slot(
        DUMMY_TRI,
        0,
        encode(Otri {
            tri: inftri.tri,
            orient: 0,
        }),
    );
    Ok(())
}

/// C: removebox (triangle.c:4672-4748). Remove the "infinite" bounding
/// triangle, setting boundary markers as appropriate. The triangular
/// bounding box has three boundary triangles (one for each side of the
/// bounding box), and a bunch of triangles fanning out from the three
/// bounding box vertices (one triangle for each edge of the convex hull
/// of the inner mesh). This routine removes these triangles. Returns the
/// number of edges on the convex hull of the triangulation. (C comments
/// preserved in the body.)
pub(crate) fn removebox(m: &mut Mesh, b: &Behavior) -> i64 {
    // Find a boundary triangle.
    let mut nextedge = Otri {
        tri: DUMMY_TRI,
        orient: 0,
    };
    m.symself(&mut nextedge);
    // Mark a place to stop.
    // C: lprev(nextedge, finaledge).
    let finaledge = nextedge.lprev();
    nextedge.lnextself();
    m.symself(&mut nextedge);
    // Find a triangle (on the boundary of the vertex set) that isn't
    //   a bounding box triangle.
    // C: lprev(nextedge, searchedge).
    let mut searchedge = nextedge.lprev();
    m.symself(&mut searchedge);
    // Check whether nextedge is another boundary triangle
    //   adjacent to the first one.
    // C: lnext(nextedge, checkedge).
    let mut checkedge = nextedge.lnext();
    m.symself(&mut checkedge);
    if checkedge.tri == DUMMY_TRI {
        // Go on to the next triangle.  There are only three boundary
        //   triangles, and this next triangle cannot be the third one,
        //   so it's safe to stop here.
        searchedge.lprevself();
        m.symself(&mut searchedge);
    }
    // Find a new boundary edge to search from, as the current search
    //   edge lies on a bounding box triangle and will be deleted.
    // C: m->dummytri[0] = encode(searchedge).
    m.set_tri_slot(DUMMY_TRI, 0, encode(searchedge));
    let mut hullsize: i64 = -2;
    while !(nextedge == finaledge) {
        hullsize += 1;
        // C: lprev(nextedge, dissolveedge).
        let mut dissolveedge = nextedge.lprev();
        m.symself(&mut dissolveedge);
        // If not using a PSLG, the vertices should be marked now.
        //   (If using a PSLG, markhull() will do the job.)
        if !b.poly {
            // Be careful!  One must check for the case where all the input
            //   vertices are collinear, and thus all the triangles are part
            //   of the bounding box.  Otherwise, the setvertexmark() call
            //   below will cause a bad pointer reference.
            if dissolveedge.tri != DUMMY_TRI {
                let markorg = m.org(dissolveedge);
                if m.vx_mark(markorg) == 0 {
                    m.vx_set_mark(markorg, 1);
                }
            }
        }
        // Disconnect the bounding box triangle from the mesh triangle.
        m.dissolve(dissolveedge);
        // C: lnext(nextedge, deadtriangle).
        let deadtriangle = nextedge.lnext();
        // C: sym(deadtriangle, nextedge).
        nextedge = m.sym(deadtriangle);
        // Get rid of the bounding box triangle.
        // C: triangledealloc(m, deadtriangle.tri).
        m.tri_dealloc(deadtriangle.tri);
        // Do we need to turn the corner?
        if nextedge.tri == DUMMY_TRI {
            // Turn the corner.
            // C: otricopy(dissolveedge, nextedge).
            nextedge = dissolveedge;
        }
    }
    m.tri_dealloc(finaledge.tri);

    // Deallocate the bounding box vertices. C: trifree(infvertex1..3);
    // the D12 mirror zeroes the owned store and resets the ids to the
    // null id (the C pointers dangle unread; the port keeps a defined
    // superset).
    m.vx_set_real(INFVERTEX1, 0, 0.0);
    m.vx_set_real(INFVERTEX1, 1, 0.0);
    m.vx_set_real(INFVERTEX2, 0, 0.0);
    m.vx_set_real(INFVERTEX2, 1, 0.0);
    m.vx_set_real(INFVERTEX3, 0, 0.0);
    m.vx_set_real(INFVERTEX3, 1, 0.0);
    m.infvertex1 = NULL;
    m.infvertex2 = NULL;
    m.infvertex3 = NULL;

    hullsize
}

// ---------------------------------------------------------------------------
// Incremental Delaunay triangulation begins here (triangle.c:4761).
// ---------------------------------------------------------------------------

/// C: SAMPLERATE (triangle_config.h "10"): the sweepline front sampler -
/// one in SAMPLERATE candidate edges is tracked in the splay tree. The D10
/// per-run draw order mirrors C's exactly.
const SAMPLERATE: u32 = 10;

/// C: incrementaldelaunay (triangle.c:4761-4786). Forms the Delaunay
/// triangulation by inserting every input vertex into a bounding box,
/// then stripping the box. Returns the number of edges on the convex
/// hull (removebox's return).
///
/// Transliteration notes: traversalinit/vertextraverse ride the pool
/// traversal (mesh.rs); insertvertex's DUPLICATEVERTEX result carries
/// C's duplicate accounting (setvertextype(UNDEADVERTEX) + undeads,
/// triangle.c:4779-4780); the _DEBUG printf above it is compiled out of
/// the pinned oracle build.
pub(crate) fn incrementaldelaunay(m: &mut Mesh, b: &Behavior) -> Result<i64, Oom> {
    // Create a triangular bounding box.
    boundingbox(m, b)?;
    // C: traversalinit(&m->vertices) then vertextraverse until NULL: walk
    // every input vertex in pool order.
    let mut tr = m.vertices.traversal();
    while let Some(vertexloop) = m.vertextraverse(&mut tr) {
        let mut starttri = Otri {
            tri: DUMMY_TRI,
            orient: 0,
        };
        if insertvertex(m, b, vertexloop, &mut starttri, None, false, false, false)?
            == InsertVertexResult::DuplicateVertex
        {
            // C: setvertextype(vertexloop, UNDEADVERTEX); m->undeads++.
            m.vx_set_type(vertexloop, UNDEADVERTEX);
            m.undeads += 1;
        }
    }
    // Remove the bounding box.
    Ok(removebox(m, b))
}

// ---------------------------------------------------------------------------
// Sweepline Delaunay triangulation begins here (triangle.c:4790-5413).
// ---------------------------------------------------------------------------

/// The register mirror of C `struct event` (triangle.h:306-311): one
/// sweepline event, a site event or a circle event. C's trimalloc'd event
/// array is an owned local arena here (D8/D11); the arena index replaces
/// the C pointer (D1), so the deterministic heap order never depends on
/// allocation addresses. (Clone: the arena is built by repeating the
/// zero cell.)
#[derive(Clone)]
pub(crate) struct Event {
    /// C xkey: the secondary heap key (a site's x, m.xminextreme for
    /// circle events).
    pub(crate) xkey: f64,
    /// C ykey: the primary heap key (a site's y, the circle top for
    /// circle events).
    pub(crate) ykey: f64,
    /// C eventptr: the site vertex id, the encoded front handle, or the
    /// free-list link (the C pointer pun split into disjoint variants;
    /// 0 / None mirror C NULL).
    pub(crate) eventptr: EventPtr,
    /// C heapposition: the slot's index in the event heap, written by the
    /// heap helpers (the delete path of check4deadevent depends on it).
    pub(crate) heapposition: i32,
}

/// The C `VOID *eventptr` pun (triangle.h:308): a vertex id for site
/// events, an encode()d handle for circle events, and the free-list
/// chain while the cell is unused.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum EventPtr {
    /// C NULL: no payload (the intermediate states C always overwrites
    /// before reading).
    Null,
    /// A site event: the C `(VOID *) vertex` payload (triangle.c:4930).
    Vertex(u32),
    /// A circle event: the C `(VOID *) encode(otri)` payload
    /// (triangle.c:5385, 5400).
    Triangle(u64),
    /// The free-list link (triangle.c:4935-4939, 5251, 5261, 5371): the
    /// next free event, None mirroring C's NULL list terminator.
    Free(Option<u32>),
}

/// The port organ of the org-slot event pun: C stores the `struct event *`
/// through setorg (triangle.c:5388, 5403) and reads it back reinterpreted
/// (triangle.c:5008) in exactly one place, check4deadevent. The tagged word
/// carries the arena index in the low 32 bits (bit 32 above the vertex-id
/// space); 0 keeps the C NULL meaning.
const EVENT_WORD_TAG: u64 = 1 << 32;

/// Encode an arena index as the org-slot event word (C: the event pointer
/// at triangle.c:5388/5403).
fn event_word(e: u32) -> u64 {
    EVENT_WORD_TAG | e as u64
}

/// Decode the org-slot event word (C: the reinterpreted org read at
/// triangle.c:5008-5010).
fn event_index(w: u64) -> u32 {
    (w & !EVENT_WORD_TAG) as u32
}

/// C: eventheapinsert (triangle.c:4800-4827). Insert newevent at the end
/// of the heap (index heapsize) and sift it up while its parent is not
/// below it. The y coordinate is the primary key, the x the tiebreak (the
/// exact C comparison, including the <= on the x tie); every moved event's
/// heapposition is kept exact.
pub(crate) fn eventheapinsert(
    heap: &mut [u32],
    heapsize: i32,
    newevent: u32,
    events: &mut [Event],
) {
    let eventx = events[newevent as usize].xkey;
    let eventy = events[newevent as usize].ykey;
    let mut eventnum = heapsize;
    let mut notdone = eventnum > 0;
    while notdone {
        let parent = (eventnum - 1) >> 1;
        let pe = heap[parent as usize] as usize;
        if (events[pe].ykey < eventy)
            || ((events[pe].ykey == eventy) && (events[pe].xkey <= eventx))
        {
            notdone = false;
        } else {
            heap[eventnum as usize] = heap[parent as usize];
            events[heap[eventnum as usize] as usize].heapposition = eventnum;
            eventnum = parent;
            notdone = eventnum > 0;
        }
    }
    heap[eventnum as usize] = newevent;
    events[newevent as usize].heapposition = eventnum;
}

/// C: eventheapify (triangle.c:4833-4875). Restore the heap property
/// below eventnum by sifting the event down toward the leaves. The
/// comparisons are the exact C ones (strict < on both keys here; the
/// right child competes against the current smallest, which can be
/// the sifted event itself exactly as C's heap[smallest] reads it).
pub(crate) fn eventheapify(heap: &mut [u32], heapsize: i32, eventnum: i32, events: &mut [Event]) {
    let thisevent = heap[eventnum as usize];
    let eventx = events[thisevent as usize].xkey;
    let eventy = events[thisevent as usize].ykey;
    let mut eventnum = eventnum;
    let mut leftchild = 2 * eventnum + 1;
    let mut notdone = leftchild < heapsize;
    while notdone {
        let le = heap[leftchild as usize] as usize;
        let mut smallest = if (events[le].ykey < eventy)
            || ((events[le].ykey == eventy) && (events[le].xkey < eventx))
        {
            leftchild
        } else {
            eventnum
        };
        let rightchild = leftchild + 1;
        if rightchild < heapsize {
            let re = heap[rightchild as usize] as usize;
            let se = heap[smallest as usize] as usize;
            if (events[re].ykey < events[se].ykey)
                || ((events[re].ykey == events[se].ykey) && (events[re].xkey < events[se].xkey))
            {
                smallest = rightchild;
            }
        }
        if smallest == eventnum {
            notdone = false;
        } else {
            heap[eventnum as usize] = heap[smallest as usize];
            events[heap[eventnum as usize] as usize].heapposition = eventnum;
            heap[smallest as usize] = thisevent;
            events[thisevent as usize].heapposition = smallest;
            eventnum = smallest;
            leftchild = 2 * eventnum + 1;
            notdone = leftchild < heapsize;
        }
    }
}

/// C: eventheapdelete (triangle.c:4881-4910). Remove the event at
/// eventnum by moving the last heap slot into the hole: sift it up while
/// smaller than its ancestors (the do-while with C's exact stop
/// conditions), then heapify downward from its destination. The
/// heapsize passed here is the size before the caller's decrement.
pub(crate) fn eventheapdelete(
    heap: &mut [u32],
    heapsize: i32,
    eventnum: i32,
    events: &mut [Event],
) {
    let moveevent = heap[(heapsize - 1) as usize];
    let mut eventnum = eventnum;
    if eventnum > 0 {
        let eventx = events[moveevent as usize].xkey;
        let eventy = events[moveevent as usize].ykey;
        // C's do-while: the body executes at least once, then tests
        // the notdone the body assigned (C leaves it unset before the
        // loop; the deferred init mirrors that).
        let mut notdone;
        loop {
            let parent = (eventnum - 1) >> 1;
            let pe = heap[parent as usize] as usize;
            if (events[pe].ykey < eventy)
                || ((events[pe].ykey == eventy) && (events[pe].xkey <= eventx))
            {
                notdone = false;
            } else {
                heap[eventnum as usize] = heap[parent as usize];
                events[heap[eventnum as usize] as usize].heapposition = eventnum;
                eventnum = parent;
                notdone = eventnum > 0;
            }
            if !notdone {
                break;
            }
        }
    }
    heap[eventnum as usize] = moveevent;
    events[moveevent as usize].heapposition = eventnum;
    eventheapify(heap, heapsize - 1, eventnum, events);
}

/// C: createeventheap (triangle.c:4916-4940). Build the event arena (the
/// site events first, in vertex pool order, identical to C's
/// traversalinit walk), the heap of arena indices (site events inserted
/// in the same C order, so the resulting array is the exact C heap), and
/// the free list threaded through the spare cells in C's descending
/// loop order (the head lands at events[invertices]).
///
/// The maxevents = (3 * invertices) / 2 arithmetic is C's exactly: two
/// site cells plus one spare circle cell per two input vertices bounds
/// the live circle events. The zero-filled arena is the D11 superset of
/// C's trimalloc garbage (C writes every field before reading it).
#[allow(clippy::type_complexity)] // C's three out-params (heap, events,
                                  // freeevents) ride one tuple exactly as the C signature hands them over
pub(crate) fn createeventheap(m: &Mesh) -> Result<(Vec<u32>, Vec<Event>, Option<u32>), Oom> {
    let maxevents = (3 * m.invertices) / 2;
    // C: *eventheap = trimalloc(maxevents * sizeof(struct event *));
    let mut eventheap: Vec<u32> = Vec::new();
    eventheap
        .try_reserve_exact(maxevents as usize)
        .map_err(|_| Oom)?;
    eventheap.resize(maxevents as usize, 0);
    // C: *events = trimalloc(maxevents * sizeof(struct event));
    let mut events: Vec<Event> = Vec::new();
    events
        .try_reserve_exact(maxevents as usize)
        .map_err(|_| Oom)?;
    events.resize(
        maxevents as usize,
        Event {
            xkey: 0.0,
            ykey: 0.0,
            eventptr: EventPtr::Null,
            heapposition: 0,
        },
    );
    // C: traversalinit(&m->vertices) then one vertextraverse per site.
    let mut tr = m.vertices.traversal();
    for i in 0..m.invertices as usize {
        let thisvertex = m.vertextraverse(&mut tr).unwrap_or(NULL);
        events[i].eventptr = EventPtr::Vertex(thisvertex);
        events[i].xkey = m.vx_real(thisvertex, 0);
        events[i].ykey = m.vx_real(thisvertex, 1);
        eventheapinsert(&mut eventheap[..], i as i32, i as u32, &mut events[..]);
    }
    // C: *freeevents = NULL; then thread the spare cells high to low, so
    // the head ends at events[m->invertices] exactly like C.
    let mut freeevents: Option<u32> = None;
    for i in (m.invertices as usize..maxevents as usize).rev() {
        events[i].eventptr = EventPtr::Free(freeevents);
        freeevents = Some(i as u32);
    }
    Ok((eventheap, events, freeevents))
}

/// C: rightofhyperbola (triangle.c:4946-4971). Is the query point on the
/// sweep side (right) of the hyperbola arc through the front edge's
/// endpoints? The lexicographic case analysis, the early returns and the
/// strict > of the dya/dxb algebra are C's exactly; newsite is the
/// coordinate pair (C passes the vertex pointer but reads only
/// newsite[0] and newsite[1]). hyperbolacount increments first, on
/// every call, exactly like C.
pub(crate) fn rightofhyperbola(m: &mut Mesh, fronttri: &Otri, newsite: (f64, f64)) -> bool {
    m.hyperbolacount += 1;

    let leftvertex = m.dest(*fronttri);
    let rightvertex = m.apex(*fronttri);
    let lx = m.vx_real(leftvertex, 0);
    let ly = m.vx_real(leftvertex, 1);
    let rx = m.vx_real(rightvertex, 0);
    let ry = m.vx_real(rightvertex, 1);
    // C: if ((leftvertex[1] < rightvertex[1]) ||
    //         ((leftvertex[1] == rightvertex[1]) &&
    //          (leftvertex[0] < rightvertex[0]))) { early true }
    // else { early false }.
    if (ly < ry) || ((ly == ry) && (lx < rx)) {
        if newsite.0 >= rx {
            return true;
        }
    } else if newsite.0 <= lx {
        return false;
    }
    let dxa = lx - newsite.0;
    let dya = ly - newsite.1;
    let dxb = rx - newsite.0;
    let dyb = ry - newsite.1;
    dya * (dxb * dxb + dyb * dyb) > dyb * (dxa * dxa + dya * dya)
}

/// C: circletop (triangle.c:4977-4994). The y coordinate of the top of
/// the circle through pa, pb, pc whose signed area ccwabc came in with
/// the triple (the caller's tri_counterclockwise result rides along
/// exactly like C's, one predicate call counted at the C call site, not
/// here). circletopcount increments first, on every call.
pub(crate) fn circletop(m: &mut Mesh, pa: u32, pb: u32, pc: u32, ccwabc: f64) -> f64 {
    m.circletopcount += 1;

    let pax = m.vx_real(pa, 0);
    let pay = m.vx_real(pa, 1);
    let pbx = m.vx_real(pb, 0);
    let pby = m.vx_real(pb, 1);
    let pcx = m.vx_real(pc, 0);
    let pcy = m.vx_real(pc, 1);
    let xac = pax - pcx;
    let yac = pay - pcy;
    let xbc = pbx - pcx;
    let ybc = pby - pcy;
    let xab = pax - pbx;
    let yab = pay - pby;
    let aclen2 = xac * xac + yac * yac;
    let bclen2 = xbc * xbc + ybc * ybc;
    let ablen2 = xab * xab + yab * yab;
    pcy + (xac * bclen2 - xbc * aclen2 + (aclen2 * bclen2 * ablen2).sqrt()) / (2.0 * ccwabc)
}

/// C: check4deadevent (triangle.c:5001-5018). If checktri's org slot
/// carries an alive circle event, free it onto the free list, delete it
/// from the heap (through its exact heapposition) and clear the slot.
/// The org-slot read is the single site of the event pun (see
/// EVENT_WORD_TAG above).
pub(crate) fn check4deadevent(
    m: &mut Mesh,
    checktri: &Otri,
    freeevents: &mut Option<u32>,
    eventheap: &mut [u32],
    heapsize: &mut i32,
    events: &mut [Event],
) {
    let eventvertex = m.orgword(*checktri);
    if eventvertex != 0 {
        let deadevent = event_index(eventvertex);
        let eventnum = events[deadevent as usize].heapposition;
        // C: deadevent->eventptr = (VOID *) *freeevents;
        events[deadevent as usize].eventptr = EventPtr::Free(*freeevents);
        *freeevents = Some(deadevent);
        eventheapdelete(eventheap, *heapsize, eventnum, events);
        *heapsize -= 1;
        // C: setorg(*checktri, NULL).
        m.setorgword(*checktri, 0);
    }
}

/// C: splay (triangle.c:5024-5128). Splay the front's splay tree to the
/// hyperbola arc nearest the search point, restructuring by the classic
/// rotate/rotate-rotate/merge cases; nodes whose keydest no longer
/// matches their keyedge are evicted (C's pooldealloc mirror) and their
/// subtrees merged by the five-way case ladder, exactly the C shape
/// (including the leftmost-rchild-threading "Holy Toledo" merge).
///
/// The C NULL splaynode pointer is the null row id 0 (D4); ids are
/// arena/pool indices, so no comparison depends on addresses (D1).
pub(crate) fn splay(
    m: &mut Mesh,
    splaytree: u32,
    searchpoint: (f64, f64),
    searchtri: &mut Otri,
) -> u32 {
    if splaytree == NULL {
        return NULL;
    }
    // C: dest(splaytree->keyedge, checkvertex).
    let keyedge = m.splay_keyedge(splaytree);
    let checkvertex = m.dest(keyedge);
    if checkvertex == m.splay_keydest(splaytree) {
        // C: the key's destination still lives: rotate below it.
        let rightofroot = rightofhyperbola(m, &keyedge, searchpoint);
        let mut child = if rightofroot {
            // C: otricopy(splaytree->keyedge, *searchtri).
            *searchtri = keyedge;
            m.splay_rchild(splaytree)
        } else {
            m.splay_lchild(splaytree)
        };
        if child == NULL {
            return splaytree;
        }
        // C: dest(child->keyedge, checkvertex) vs child->keydest: a dead
        // key is replaced by whatever its subtree splay returns.
        let childkeyedge = m.splay_keyedge(child);
        let checkvertex = m.dest(childkeyedge);
        if checkvertex != m.splay_keydest(child) {
            child = splay(m, child, searchpoint, searchtri);
            if child == NULL {
                if rightofroot {
                    m.set_splay_rchild(splaytree, NULL);
                } else {
                    m.set_splay_lchild(splaytree, NULL);
                }
                return splaytree;
            }
        }
        // C: rightofchild is decided on the (possibly re-splayed) child's
        // keyedge; the re-assignment order below is C's exactly.
        let childkeyedge = m.splay_keyedge(child);
        let rightofchild = rightofhyperbola(m, &childkeyedge, searchpoint);
        let grandchild = if rightofchild {
            // C: otricopy(child->keyedge, *searchtri).
            *searchtri = childkeyedge;
            let g = splay(m, m.splay_rchild(child), searchpoint, searchtri);
            m.set_splay_rchild(child, g);
            g
        } else {
            let g = splay(m, m.splay_lchild(child), searchpoint, searchtri);
            m.set_splay_lchild(child, g);
            g
        };
        if grandchild == NULL {
            if rightofroot {
                // C: splaytree->rchild = child->lchild; child->lchild = splaytree.
                let cl = m.splay_lchild(child);
                m.set_splay_rchild(splaytree, cl);
                m.set_splay_lchild(child, splaytree);
            } else {
                // C: splaytree->lchild = child->rchild; child->rchild = splaytree.
                let cr = m.splay_rchild(child);
                m.set_splay_lchild(splaytree, cr);
                m.set_splay_rchild(child, splaytree);
            }
            return child;
        }
        if rightofchild {
            if rightofroot {
                // C: splaytree->rchild = child->lchild; child->lchild = splaytree.
                let cl = m.splay_lchild(child);
                m.set_splay_rchild(splaytree, cl);
                m.set_splay_lchild(child, splaytree);
            } else {
                // C: splaytree->lchild = grandchild->rchild;
                //    grandchild->rchild = splaytree.
                let gr = m.splay_rchild(grandchild);
                m.set_splay_lchild(splaytree, gr);
                m.set_splay_rchild(grandchild, splaytree);
            }
            // C: child->rchild = grandchild->lchild; grandchild->lchild = child.
            let gl = m.splay_lchild(grandchild);
            m.set_splay_rchild(child, gl);
            m.set_splay_lchild(grandchild, child);
        } else {
            if rightofroot {
                // C: splaytree->rchild = grandchild->lchild;
                //    grandchild->lchild = splaytree.
                let gl = m.splay_lchild(grandchild);
                m.set_splay_rchild(splaytree, gl);
                m.set_splay_lchild(grandchild, splaytree);
            } else {
                // C: splaytree->lchild = child->rchild; child->rchild = splaytree.
                let cr = m.splay_rchild(child);
                m.set_splay_lchild(splaytree, cr);
                m.set_splay_rchild(child, splaytree);
            }
            // C: child->lchild = grandchild->rchild; grandchild->rchild = child.
            let gr = m.splay_rchild(grandchild);
            m.set_splay_lchild(child, gr);
            m.set_splay_rchild(grandchild, child);
        }
        grandchild
    } else {
        // C: the key is dead: splay both subtrees, free the node, and
        // join the remains by the five-way case ladder.
        let lefttree = splay(m, m.splay_lchild(splaytree), searchpoint, searchtri);
        let righttree = splay(m, m.splay_rchild(splaytree), searchpoint, searchtri);
        // C: pooldealloc(&m->splaynodes, splaytree).
        m.splay_dealloc(splaytree);
        if lefttree == NULL {
            righttree
        } else if righttree == NULL {
            lefttree
        } else if m.splay_rchild(lefttree) == NULL {
            // C: lefttree->rchild = righttree->lchild;
            //    righttree->lchild = lefttree; return righttree.
            let rl = m.splay_lchild(righttree);
            m.set_splay_rchild(lefttree, rl);
            m.set_splay_lchild(righttree, lefttree);
            righttree
        } else if m.splay_lchild(righttree) == NULL {
            // C: righttree->lchild = lefttree->rchild;
            //    lefttree->rchild = righttree; return lefttree.
            let lr = m.splay_rchild(lefttree);
            m.set_splay_lchild(righttree, lr);
            m.set_splay_rchild(lefttree, righttree);
            lefttree
        } else {
            // C: leftright = lefttree->rchild; while (leftright->rchild)
            //    leftright = leftright->rchild; (the "Holy Toledo" merge)
            let mut leftright = m.splay_rchild(lefttree);
            while m.splay_rchild(leftright) != NULL {
                leftright = m.splay_rchild(leftright);
            }
            m.set_splay_rchild(leftright, righttree);
            lefttree
        }
    }
}

/// C: splayinsert (triangle.c:5134-5155). Split the splay tree at the
/// root's hyperbola and hang the root under the fresh node, left when
/// the search point is right of the root (the C three-way case). The
/// node comes from the splaynodes pool exactly like C.
pub(crate) fn splayinsert(
    m: &mut Mesh,
    splayroot: u32,
    newkey: &Otri,
    searchpoint: (f64, f64),
) -> Result<u32, Oom> {
    // C: newsplaynode = poolalloc(&m->splaynodes).
    let newsplaynode = m.splay_alloc()?;
    // C: otricopy(*newkey, newsplaynode->keyedge); dest(*newkey,
    //    newsplaynode->keydest).
    m.set_splay_keyedge(newsplaynode, *newkey);
    m.set_splay_keydest(newsplaynode, m.dest(*newkey));
    if splayroot == NULL {
        m.set_splay_lchild(newsplaynode, NULL);
        m.set_splay_rchild(newsplaynode, NULL);
    } else {
        let rootkeyedge = m.splay_keyedge(splayroot);
        if rightofhyperbola(m, &rootkeyedge, searchpoint) {
            // C: newsplaynode->lchild = splayroot; newsplaynode->rchild
            //    = splayroot->rchild; splayroot->rchild = NULL.
            m.set_splay_lchild(newsplaynode, splayroot);
            let rr = m.splay_rchild(splayroot);
            m.set_splay_rchild(newsplaynode, rr);
            m.set_splay_rchild(splayroot, NULL);
        } else {
            // C: newsplaynode->lchild = splayroot->lchild;
            //    newsplaynode->rchild = splayroot; splayroot->lchild = NULL.
            let ll = m.splay_lchild(splayroot);
            m.set_splay_lchild(newsplaynode, ll);
            m.set_splay_rchild(newsplaynode, splayroot);
            m.set_splay_lchild(splayroot, NULL);
        }
    }
    Ok(newsplaynode)
}

/// C: circletopinsert (triangle.c:5161-5183). Prepare a circle-event key
/// for the splay tree: the search point rides the circle's top y and the
/// x where the circle top's hyperbola arc through pa/pb/pc splits, then
/// splay to that point and insert. ccwabc is computed exactly once, at
/// C's call site (the predicate call count mirrors C).
#[allow(clippy::too_many_arguments)] // the C signature carries eight parameters
pub(crate) fn circletopinsert(
    m: &mut Mesh,
    b: &Behavior,
    splayroot: u32,
    newkey: &Otri,
    pa: u32,
    pb: u32,
    pc: u32,
    topy: f64,
) -> Result<u32, Oom> {
    let pax = m.vx_real(pa, 0);
    let pay = m.vx_real(pa, 1);
    let pbx = m.vx_real(pb, 0);
    let pby = m.vx_real(pb, 1);
    let pcx = m.vx_real(pc, 0);
    let pcy = m.vx_real(pc, 1);
    // C: ccwabc = tri_counterclockwise(m, b, pa, pb, pc).
    let ccwabc = tri_counterclockwise(
        m,
        b,
        vertex_coords(m, pa),
        vertex_coords(m, pb),
        vertex_coords(m, pc),
    );
    let xac = pax - pcx;
    let yac = pay - pcy;
    let xbc = pbx - pcx;
    let ybc = pby - pcy;
    let aclen2 = xac * xac + yac * yac;
    let bclen2 = xbc * xbc + ybc * ybc;
    // C: searchpoint = { pc[0] - (yac * bclen2 - ybc * aclen2)
    //                       / (2.0 * ccwabc), topy }.
    let searchpoint = (pcx - (yac * bclen2 - ybc * aclen2) / (2.0 * ccwabc), topy);
    // C: splay's searchtri out-param is a scratch local here (D11: the
    // port initializes where C left it uninitialized).
    let mut dummytri = Otri {
        tri: NULL,
        orient: 0,
    };
    let splayed = splay(m, splayroot, searchpoint, &mut dummytri);
    splayinsert(m, splayed, newkey, searchpoint)
}

/// C: frontlocate (triangle.c:5189-5206). Splay to the search vertex,
/// then walk the front from bottommost by onext until the vertex is not
/// right of the current front edge, or the walk closes back at
/// bottommost (farright). The walk condition's rightofhyperbola call
/// order is C's exactly.
pub(crate) fn frontlocate(
    m: &mut Mesh,
    splayroot: u32,
    bottommost: &Otri,
    searchvertex: u32,
    searchtri: &mut Otri,
    farright: &mut bool,
) -> u32 {
    // C: the search vertex's coordinates are the hyperbola query.
    let searchpoint = (m.vx_real(searchvertex, 0), m.vx_real(searchvertex, 1));
    // C: otricopy(*bottommost, *searchtri).
    *searchtri = *bottommost;
    let splayroot = splay(m, splayroot, searchpoint, searchtri);

    let mut farrightflag = false;
    while !farrightflag && rightofhyperbola(m, searchtri, searchpoint) {
        // C: onextself(*searchtri).
        m.onextself(searchtri);
        // C: farrightflag = otriequal(*searchtri, *bottommost).
        farrightflag = *searchtri == *bottommost;
    }
    *farright = farrightflag;
    splayroot
}

/// C: sweeplinedelaunay (triangle.c:5212-5411). Forms the Delaunay
/// triangulation by sweeping the events left to right: site events grow
/// the front and splay-track candidate edges, circle events flip the
/// front concavities away. Returns the hull edge count (removeghosts).
/// When every input vertex is identical, returns C's TRI_INPUT through
/// the long channel exactly as C does (triangle.c:5258).
///
/// Idiom notes: the event arena, its heap and the free-event list are
/// owned locals (C's trimalloc'd temporaries, D8/D11); front handles
/// carrying the org-slot event pun are read only by check4deadevent;
/// splayroot is the splay-tree row id (0 = C NULL). The two
/// randomnation draws sweep at exactly the C sites.
pub(crate) fn sweeplinedelaunay(m: &mut Mesh, b: &Behavior) -> Result<i64, Oom> {
    // C: poolinit(&m->splaynodes, ...) at this site dissolves into the
    // eager Mesh::new init (same geometry), so nothing happens here;
    // the pooldeinit at the end dissolves the same way.
    let mut splayroot: u32 = NULL;

    let (mut eventheap, mut events, mut freeevents) = createeventheap(m)?;
    let mut bottommost = Otri {
        tri: NULL,
        orient: 0,
    };
    // C declares lefttri/righttri/fliptri/inserttri unassigned; each
    // is written before its first read, exactly as in C, so the
    // deferred inits mirror the unassigned C declarations.
    let mut lefttri: Otri;
    let mut righttri: Otri;
    let mut farlefttri = bottommost;
    let mut farrighttri = bottommost;
    let mut fliptri: Otri;
    let mut searchtri = bottommost;
    let mut inserttri: Otri;
    let mut heapsize: i32 = m.invertices;

    // C: maketriangle(m, b, &lefttri); maketriangle(m, b, &righttri);
    //    bond(lefttri, righttri); lnextself(lefttri); lprevself(righttri);
    //    bond(lefttri, righttri); lnextself(lefttri); lprevself(righttri);
    //    bond(lefttri, righttri) - the two-triangle tube over the first
    //    hull edge.
    lefttri = m.maketriangle(b.usesegments, b.vararea)?;
    righttri = m.maketriangle(b.usesegments, b.vararea)?;
    m.bond(lefttri, righttri);
    lefttri.lnextself();
    righttri.lprevself();
    m.bond(lefttri, righttri);
    lefttri.lnextself();
    righttri.lprevself();
    m.bond(lefttri, righttri);
    // C: firstvertex = (vertex) eventheap[0]->eventptr; the cell is
    // recycled onto the free list before the heap delete, exactly C's
    // four-statement sequence.
    let e0 = eventheap[0];
    let firstvertex = match events[e0 as usize].eventptr {
        EventPtr::Vertex(v) => v,
        _ => NULL,
    };
    events[e0 as usize].eventptr = EventPtr::Free(freeevents);
    freeevents = Some(e0);
    eventheapdelete(&mut eventheap[..], heapsize, 0, &mut events[..]);
    heapsize -= 1;
    // C declares secondvertex unassigned; the do-while writes it before
    // the first read, the deferred init mirrors that.
    let mut secondvertex: u32;
    // C: do { if (heapsize == 0) return TRI_INPUT; ... } while (first and
    //    second are identical points): one distinct partner is forced.
    loop {
        if heapsize == 0 {
            // Error:  Input vertices are all identical. C returns TRI_INPUT
            // through the long channel; the port mirrors the literal value.
            return Ok(crate::error::InternalStatus::Input as i64);
        }
        // C: secondvertex = (vertex) eventheap[0]->eventptr; ... the same
        // recycle-then-delete sequence.
        let e0 = eventheap[0];
        secondvertex = match events[e0 as usize].eventptr {
            EventPtr::Vertex(v) => v,
            _ => NULL,
        };
        events[e0 as usize].eventptr = EventPtr::Free(freeevents);
        freeevents = Some(e0);
        eventheapdelete(&mut eventheap[..], heapsize, 0, &mut events[..]);
        heapsize -= 1;
        if (m.vx_real(firstvertex, 0) == m.vx_real(secondvertex, 0))
            && (m.vx_real(firstvertex, 1) == m.vx_real(secondvertex, 1))
        {
            // C's _DEBUG duplicate warning is compiled out; the accounting
            // stays (triangle.c:5273-5274).
            m.vx_set_type(secondvertex, UNDEADVERTEX);
            m.undeads += 1;
        }
        if !((m.vx_real(firstvertex, 0) == m.vx_real(secondvertex, 0))
            && (m.vx_real(firstvertex, 1) == m.vx_real(secondvertex, 1)))
        {
            break;
        }
    }
    // C: setorg(lefttri, firstvertex); setdest(lefttri, secondvertex);
    //    setorg(righttri, secondvertex); setdest(righttri, firstvertex);
    //    lprev(lefttri, &bottommost); lastvertex = secondvertex.
    m.setorg(lefttri, firstvertex);
    m.setdest(lefttri, secondvertex);
    m.setorg(righttri, secondvertex);
    m.setdest(righttri, firstvertex);
    bottommost = lefttri.lprev();
    let mut lastvertex = secondvertex;

    while heapsize > 0 {
        let nextevent = eventheap[0];
        eventheapdelete(&mut eventheap[..], heapsize, 0, &mut events[..]);
        heapsize -= 1;
        let mut check4events = true;
        if events[nextevent as usize].xkey < m.xmin {
            // A circle event: the front concavity under fliptri's edge
            // flips away.
            let eventptr_word = match events[nextevent as usize].eventptr {
                EventPtr::Triangle(w) => w,
                _ => 0,
            };
            // C: decode(nextevent->eventptr, fliptri) (the encoded front
            // handle of the circle's low edge).
            fliptri = decode(eventptr_word);
            farlefttri = m.oprev(fliptri);
            check4deadevent(
                m,
                &farlefttri,
                &mut freeevents,
                &mut eventheap[..],
                &mut heapsize,
                &mut events[..],
            );
            farrighttri = m.onext(fliptri);
            check4deadevent(
                m,
                &farrighttri,
                &mut freeevents,
                &mut eventheap[..],
                &mut heapsize,
                &mut events[..],
            );

            // C: if (otriequal(farlefttri, bottommost)) lprev(fliptri,
            //    &bottommost).
            if farlefttri == bottommost {
                bottommost = fliptri.lprev();
            }
            // C: flip(m, b, &fliptri); setapex(fliptri, NULL); then the
            // left/right/farleft handle re-derivation. farrighttri keeps
            // its pre-flip value exactly like C (never reassigned here).
            flip(m, b, fliptri);
            m.setapex(fliptri, NULL);
            lefttri = fliptri.lprev();
            righttri = fliptri.lnext();
            farlefttri = m.sym(lefttri);

            if randomnation(m, SAMPLERATE) == 0 {
                // C: symself(fliptri); dest/apex/org of the flipped edge.
                let mut symtri = fliptri;
                m.symself(&mut symtri);
                let leftvertex = m.dest(symtri);
                let midvertex = m.apex(symtri);
                let rightvertex = m.org(symtri);
                // C: splayroot = circletopinsert(..., nextevent->ykey):
                // the y-key read happens here, before the cell is freed.
                splayroot = circletopinsert(
                    m,
                    b,
                    splayroot,
                    &lefttri,
                    leftvertex,
                    midvertex,
                    rightvertex,
                    events[nextevent as usize].ykey,
                )?;
            }
        } else {
            // A site event: grow the front.
            let nextvertex = match events[nextevent as usize].eventptr {
                EventPtr::Vertex(v) => v,
                _ => NULL,
            };
            if (m.vx_real(nextvertex, 0) == m.vx_real(lastvertex, 0))
                && (m.vx_real(nextvertex, 1) == m.vx_real(lastvertex, 1))
            {
                // C's _DEBUG duplicate warning is compiled out; the
                // accounting stays (triangle.c:5323-5325).
                m.vx_set_type(nextvertex, UNDEADVERTEX);
                m.undeads += 1;
                check4events = false;
            } else {
                lastvertex = nextvertex;

                // C: splayroot = frontlocate(m, splayroot, &bottommost,
                //    nextvertex, &searchtri, &farrightflag).
                let mut farrightflag = false;
                splayroot = frontlocate(
                    m,
                    splayroot,
                    &bottommost,
                    nextvertex,
                    &mut searchtri,
                    &mut farrightflag,
                );

                check4deadevent(
                    m,
                    &searchtri,
                    &mut freeevents,
                    &mut eventheap[..],
                    &mut heapsize,
                    &mut events[..],
                );

                // C: otricopy(searchtri, farrighttri); sym(searchtri,
                //    farlefttri); then the two-triangle front insert with
                //    the exact bond pattern.
                farrighttri = searchtri;
                farlefttri = m.sym(searchtri);
                lefttri = m.maketriangle(b.usesegments, b.vararea)?;
                righttri = m.maketriangle(b.usesegments, b.vararea)?;
                // C: dest(farrighttri, connectvertex).
                let connectvertex = m.dest(farrighttri);
                m.setorg(lefttri, connectvertex);
                m.setdest(lefttri, nextvertex);
                m.setorg(righttri, nextvertex);
                m.setdest(righttri, connectvertex);
                m.bond(lefttri, righttri);
                lefttri.lnextself();
                righttri.lprevself();
                m.bond(lefttri, righttri);
                lefttri.lnextself();
                righttri.lprevself();
                m.bond(lefttri, farlefttri);
                m.bond(righttri, farrighttri);
                // C: if (!farrightflag && otriequal(farrighttri, bottommost))
                //    otricopy(lefttri, bottommost).
                if !farrightflag && farrighttri == bottommost {
                    bottommost = lefttri;
                }

                // C: the two randomnation draws (triangle.c:5363-5368):
                // the first sample splay-tracks the left edge, the second
                // one the diagonal edge.
                if randomnation(m, SAMPLERATE) == 0 {
                    splayroot = splayinsert(
                        m,
                        splayroot,
                        &lefttri,
                        (m.vx_real(nextvertex, 0), m.vx_real(nextvertex, 1)),
                    )?;
                } else if randomnation(m, SAMPLERATE) == 0 {
                    inserttri = righttri.lnext();
                    splayroot = splayinsert(
                        m,
                        splayroot,
                        &inserttri,
                        (m.vx_real(nextvertex, 0), m.vx_real(nextvertex, 1)),
                    )?;
                }
            }
        }
        // C: nextevent->eventptr = (VOID *) freeevents; freeevents =
        //    nextevent - the processed cell returns to the free list.
        events[nextevent as usize].eventptr = EventPtr::Free(freeevents);
        freeevents = Some(nextevent);

        // C: if (check4events) the two concavity tests of the front seam.
        if check4events {
            // C: apex(farlefttri, leftvertex); dest(lefttri, midvertex);
            //    apex(lefttri, rightvertex).
            let leftvertex = m.apex(farlefttri);
            let midvertex = m.dest(lefttri);
            let rightvertex = m.apex(lefttri);
            // C: lefttest = tri_counterclockwise(m, b, leftvertex,
            //    midvertex, rightvertex).
            let lefttest = tri_counterclockwise(
                m,
                b,
                vertex_coords(m, leftvertex),
                vertex_coords(m, midvertex),
                vertex_coords(m, rightvertex),
            );
            if lefttest > 0.0 {
                // C: newevent = freeevents; freeevents = freeevents->eventptr.
                let newevent =
                    freeevents.expect("a spare event cell (C dereferences the free list)");
                debug_assert!(
                    matches!(events[newevent as usize].eventptr, EventPtr::Free(_)),
                    "the popped cell must head the free list"
                );
                freeevents = match events[newevent as usize].eventptr {
                    EventPtr::Free(next) => next,
                    _ => None,
                };
                events[newevent as usize].xkey = m.xminextreme;
                events[newevent as usize].ykey =
                    circletop(m, leftvertex, midvertex, rightvertex, lefttest);
                // C: newevent->eventptr = (VOID *) encode(lefttri).
                events[newevent as usize].eventptr = EventPtr::Triangle(encode(lefttri));
                eventheapinsert(&mut eventheap[..], heapsize, newevent, &mut events[..]);
                heapsize += 1;
                // C: setorg(lefttri, (VOID *) newevent) - the event pun.
                m.setorgword(lefttri, event_word(newevent));
            }
            // C: apex(righttri, leftvertex); org(righttri, midvertex);
            //    apex(farrighttri, rightvertex).
            let leftvertex = m.apex(righttri);
            let midvertex = m.org(righttri);
            let rightvertex = m.apex(farrighttri);
            // C: righttest = tri_counterclockwise(m, b, ...).
            let righttest = tri_counterclockwise(
                m,
                b,
                vertex_coords(m, leftvertex),
                vertex_coords(m, midvertex),
                vertex_coords(m, rightvertex),
            );
            if righttest > 0.0 {
                // C: newevent = freeevents; freeevents = freeevents->eventptr.
                let newevent =
                    freeevents.expect("a spare event cell (C dereferences the free list)");
                debug_assert!(
                    matches!(events[newevent as usize].eventptr, EventPtr::Free(_)),
                    "the popped cell must head the free list"
                );
                freeevents = match events[newevent as usize].eventptr {
                    EventPtr::Free(next) => next,
                    _ => None,
                };
                events[newevent as usize].xkey = m.xminextreme;
                events[newevent as usize].ykey =
                    circletop(m, leftvertex, midvertex, rightvertex, righttest);
                events[newevent as usize].eventptr = EventPtr::Triangle(encode(farrighttri));
                eventheapinsert(&mut eventheap[..], heapsize, newevent, &mut events[..]);
                heapsize += 1;
                m.setorgword(farrighttri, event_word(newevent));
            }
        }
    }

    // C: pooldeinit(&m->splaynodes) (triangle.c:5408) frees the blocks
    // but leaves items/maxitems stale (C pooldeinit only frees); the
    // port keeps the pool alive so the post-run statistics reads stay
    // C-identical, hence no action here.
    // C: lprevself(bottommost); return removeghosts(m, b, &bottommost).
    bottommost.lprevself();
    Ok(removeghosts(m, b, &bottommost))
}

// ---------------------------------------------------------------------------
// General mesh construction routines begin here (triangle.c:5429).
// ---------------------------------------------------------------------------

/// C: delaunay (triangle.c:5429-5454). The dispatcher: reset eextras,
/// size the triangle/subsegment pools, then form the triangulation by
/// the behavior flags - incremental, else sweepline, else
/// divide-and-conquer - and return the hull edge count (0 when the
/// input was collinear, hence itemless; the dispatcher is otherwise
/// faithful down to the odd TRI_INPUT pass-through of sweeplinedelaunay
/// landing in C's hullsize).
pub(crate) fn delaunay(m: &mut Mesh, b: &Behavior) -> Result<i64, Oom> {
    // C: m->eextras = 0; initializetrisubpools(m, b).
    m.eextras = 0;
    m.initializetrisubpools(
        b.order,
        b.usesegments,
        b.vararea,
        b.regionattrib,
        b.neighbors,
    )?;

    // C: the REDUCED build compiles only divconqdelaunay; the unconditional
    // else-chain below is the not-REDUCED oracle build exactly.
    let hulledges = if b.incremental {
        incrementaldelaunay(m, b)?
    } else if b.sweepline {
        sweeplinedelaunay(m, b)?
    } else {
        divconqdelaunay(m, b)?
    };

    if m.triangles.live_count() == 0 {
        // The input vertices were all collinear, so there are no triangles.
        Ok(0)
    } else {
        Ok(hulledges)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transform::{insertvertex, InsertVertexResult};

    /// A hand-built Behavior exactly like the landed modules' test
    /// helpers: plain or poly, everything else the defaults (the dwyer
    /// alternation is the default "on").
    fn behavior(poly: bool) -> Behavior {
        Behavior {
            poly,
            usesegments: poly,
            ..Behavior::default_form()
        }
    }

    fn behavior_dwyer_off() -> Behavior {
        Behavior {
            dwyer: false,
            ..behavior(false)
        }
    }

    /// A mesh whose input vertices are hand-built live vertex rows in
    /// the given order (pool ids 1..=n, like every input path).
    fn mesh_with(points: &[(f64, f64)], poly: bool) -> Mesh {
        let mut m = Mesh::new(&behavior(poly), points.len() as i32, 2, 0, 0).expect("mesh init");
        for &(x, y) in points {
            let v = m.vx_alloc().expect("vertex alloc");
            m.vx_set_real(v, 0, x);
            m.vx_set_real(v, 1, y);
        }
        m
    }

    fn coords(m: &Mesh, v: u32) -> (f64, f64) {
        (m.vx_real(v, 0), m.vx_real(v, 1))
    }

    fn ids(points: &[(f64, f64)]) -> Vec<u32> {
        (1..=points.len() as u32).collect()
    }

    /// The local-Delaunay invariant of the seam knit: every edge shared
    /// by two real triangles is Delaunay (the mirror of checkdelaunay's
    /// inner test over the divconq result). Two passes: the traversal
    /// collects the four coordinate triples under a shared view; the
    /// predicate calls then reborrow the mesh mutably (their counter
    /// updates are the only writes).
    fn assert_locally_delaunay(m: &mut Mesh, b: &Behavior) {
        let mut edges: Vec<[[f64; 3]; 4]> = Vec::new();
        let mut tr = m.triangles.traversal();
        while let Some(id) = m.triangles.traverse_next(&mut tr) {
            if m.deadtri(id) {
                continue;
            }
            for orient in 0..3u8 {
                let e = Otri { tri: id, orient };
                let oppo = m.sym(e);
                if oppo.tri == DUMMY_TRI || m.deadtri(oppo.tri) {
                    continue;
                }
                // a bounding triangle (NULL vertex) never tests an edge
                if m.apex(e) == NULL || m.apex(oppo) == NULL {
                    continue;
                }
                edges.push([
                    vertex_coords(m, m.org(e)),
                    vertex_coords(m, m.dest(e)),
                    vertex_coords(m, m.apex(e)),
                    vertex_coords(m, m.apex(oppo)),
                ]);
            }
        }
        for (i, [o, d, a, oa]) in edges.into_iter().enumerate() {
            assert!(
                tri_incircle(m, b, o, d, a, oa) <= 0.0,
                "a non-Delaunay interior edge survived the merge at interior edge {}",
                i
            );
        }
    }

    #[test]
    fn vertexsort_sorts_by_x_then_y_exactly_as_c() {
        // Distinct coordinates: the pivot draws cannot change the total
        // order of a correct quicksort, so the comparison order (x, then
        // y, no further tiebreak) is pinned without any oracle.
        let pts = [
            (2.0, 2.0),
            (0.0, 1.0),
            (1.0, 1.0),
            (1.0, 0.0),
            (0.0, 0.0),
            (3.0, 0.0),
        ];
        let mut m = mesh_with(&pts, false);
        let mut arr = ids(&pts);
        vertexsort(&mut m, &mut arr);
        let got: Vec<(f64, f64)> = arr.iter().map(|&v| coords(&m, v)).collect();
        assert_eq!(
            got,
            vec![
                (0.0, 0.0),
                (0.0, 1.0),
                (1.0, 0.0),
                (1.0, 1.0),
                (2.0, 2.0),
                (3.0, 0.0),
            ]
        );
    }

    #[test]
    fn vertexsort_base_case_draws_no_seed() {
        let pts = [(1.0, 0.0), (0.0, 0.0)];
        let mut m = mesh_with(&pts, false);
        let mut arr = ids(&pts);
        vertexsort(&mut m, &mut arr);
        assert_eq!(coords(&m, arr[0]), (0.0, 0.0));
        assert_eq!(coords(&m, arr[1]), (1.0, 0.0));
        assert_eq!(m.randomseed, 1, "the arraysize == 2 base case never draws");
    }

    #[test]
    fn vertexsort_duplicate_keys_are_seed_deterministic() {
        // Exact duplicate pairs: the total order is not strict, so the
        // permutation of equal keys is decided by the pivot draws; the
        // D10 per-run seed pins it (same seed, same output).
        let pts = [
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 0.0),
            (0.0, 0.0),
            (2.0, 0.0),
            (2.0, 0.0),
            (1.0, 1.0),
            (1.0, 1.0),
        ];
        let mut a = mesh_with(&pts, false);
        let mut arr_a = ids(&pts);
        vertexsort(&mut a, &mut arr_a);
        let mut b = mesh_with(&pts, false);
        let mut arr_b = ids(&pts);
        vertexsort(&mut b, &mut arr_b);
        assert_eq!(arr_a, arr_b, "same seed, same permutation of equal keys");
        assert_eq!(a.randomseed, b.randomseed);
        // the coordinate sequence (ignoring identity among equals) is
        // the exact C lexicographic order
        let got: Vec<(f64, f64)> = arr_a.iter().map(|&v| coords(&a, v)).collect();
        let mut sorted = pts.to_vec();
        sorted.sort_by(|p, q| {
            p.0.partial_cmp(&q.0)
                .unwrap()
                .then(p.1.partial_cmp(&q.1).unwrap())
        });
        assert_eq!(got, sorted);
    }

    #[test]
    fn vertexmedian_splits_at_the_median_size_on_both_axes() {
        // axis 0 (x primary) and axis 1 (y primary): the first `median'
        // entries occur lexicographically before the remaining ones
        // (distinct keys on both axes pin it regardless of pivots).
        let pts = [
            (5.0, 0.0),
            (1.0, 2.0),
            (3.0, 4.0),
            (0.0, 6.0),
            (2.0, 8.0),
            (4.0, 10.0),
        ];
        for axis in 0..=1i32 {
            let mut m = mesh_with(&pts, false);
            let mut arr = ids(&pts);
            vertexmedian(&mut m, &mut arr, 4, axis);
            assert_eq!(arr.len(), 6, "the shuffle keeps the array size");
            let key = |v: u32| (m.vx_real(v, axis as usize), m.vx_real(v, 1 - axis as usize));
            for l in arr.iter().take(4) {
                for r in arr.iter().skip(4) {
                    let (kl, kr) = (key(*l), key(*r));
                    assert!(
                        kl.0 <= kr.0 && !(kl.0 == kr.0 && kl.1 > kr.1),
                        "axis {} median split violated by {:?} vs {:?}",
                        axis,
                        kl,
                        kr
                    );
                }
            }
        }
    }

    #[test]
    fn alternateaxes_partitions_by_the_alternating_axis() {
        // four vertices: vertexmedian cuts by the incoming axis (partition
        // property), then the base leaves always sort by x. Both halves
        // come out sorted by x within their y-class sets, pivot-independently.
        let pts = [(0.0, 1.0), (2.0, 2.0), (4.0, 0.0), (6.0, 3.0)];
        let mut m = mesh_with(&pts, false);
        let mut arr = ids(&pts);
        alternateaxes(&mut m, &mut arr, 1);
        let got: Vec<(f64, f64)> = arr.iter().map(|&v| coords(&m, v)).collect();
        // y-lowest two form the first half, x-sorted: (0.0,1.0),(4.0,0.0); the
        // y-highest two the second half, x-sorted: (2,2),(6,3)
        assert_eq!(got, vec![(0.0, 1.0), (4.0, 0.0), (2.0, 2.0), (6.0, 3.0)]);

        // the base case forces axis 0 on small inputs: arraysize 3 never
        // partitions by y even when called with axis 1
        let pts3 = [(0.0, 5.0), (1.0, 3.0), (2.0, 0.0)];
        let mut m = mesh_with(&pts3, false);
        let mut arr = ids(&pts3);
        alternateaxes(&mut m, &mut arr, 1);
        let got: Vec<(f64, f64)> = arr.iter().map(|&v| coords(&m, v)).collect();
        assert_eq!(got, pts3.to_vec(), "x-sorted leaves, y untouched");
    }

    #[test]
    fn alternateaxes_lcg_draws_are_the_c_sequence() {
        // the 3-vertex base-forced axis: vertexmedian(3) draws exactly
        // one pivot (the y-sorted singletons need no draws); the seed is
        // the first step of the C LCG (seeded 1 by triangleinit).
        let pts = [(0.0, 5.0), (1.0, 3.0), (2.0, 0.0)];
        let mut m = mesh_with(&pts, false);
        let mut arr = ids(&pts);
        alternateaxes(&mut m, &mut arr, 1);
        // the C LCG first step from seed 1: (1 * 1366 + 150889) % 714025
        // = 152255, spelled as its (const-folded) value so clippy does
        // not flag the identity modulo
        assert_eq!(m.randomseed, 152255);
        // and the arraysize == 2 base case never draws
        let pts2 = [(1.0, 3.0), (0.0, 5.0)];
        let mut m2 = mesh_with(&pts2, false);
        let mut arr2 = ids(&pts2);
        alternateaxes(&mut m2, &mut arr2, 0);
        assert_eq!(m2.randomseed, 1);
        assert_eq!(coords(&m2, arr2[0]), (0.0, 5.0));
    }

    fn end_state_checks(m: &Mesh) -> (i64, u64, u64) {
        // count live triangles and hull edges through the landed
        // traversal, and verify the bond topology of everything alive
        let mut hull_edges: i64 = 0;
        let mut live: u64 = 0;
        let mut tr = m.triangles.traversal();
        while let Some(id) = m.triangles.traverse_next(&mut tr) {
            if m.deadtri(id) {
                continue;
            }
            live += 1;
            for orient in 0..3u8 {
                let e = Otri { tri: id, orient };
                let s = m.sym(e);
                assert_ne!(m.org(e), NULL, "a live triangle has a NULL org");
                assert_ne!(m.dest(e), NULL, "a live triangle has a NULL dest");
                assert_ne!(m.apex(e), NULL, "a live triangle has a NULL apex");
                if s.tri == DUMMY_TRI {
                    hull_edges += 1;
                } else {
                    assert!(!m.deadtri(s.tri), "a live triangle bonds to a dead one");
                    assert_eq!(m.sym(s), e, "the bond is symmetric");
                }
            }
        }
        (hull_edges, live, m.triangles.live_count())
    }

    #[test]
    fn divconqdelaunay_square_end_state() {
        // the cocircular square: exact counts are pivot-independent
        let pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let mut m = mesh_with(&pts, false);
        let b = behavior(false);
        let hullsize = divconqdelaunay(&mut m, &b).expect("oom");
        assert_eq!(hullsize, 4);
        assert_eq!(m.undeads, 0);
        // 6 allocated rows (4 bounding + 2 real), the ghosts consumed
        let (hull_edges, live, counted) = end_state_checks(&m);
        assert_eq!((hull_edges, live, counted), (4, 2, 2));
        assert_eq!(m.triangles.high_water(), 6);
        // the non-dwyer (Lee-Yang cut) build ends in the same state
        let mut m2 = mesh_with(&pts, false);
        let b2 = behavior_dwyer_off();
        let hullsize2 = divconqdelaunay(&mut m2, &b2).expect("oom");
        assert_eq!(hullsize2, 4);
        let (h2, l2, c2) = end_state_checks(&m2);
        assert_eq!((h2, l2, c2), (4, 2, 2));
    }

    #[test]
    fn divconqdelaunay_grid9_end_state_and_delaunayhood() {
        // the harness's grid case in miniature: 9 lattice points, eight
        // collinear-sensitive hull edges, one interior vertex
        let pts = [
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
            (0.0, 1.0),
            (1.0, 1.0),
            (2.0, 1.0),
            (0.0, 2.0),
            (1.0, 2.0),
            (2.0, 2.0),
        ];
        let mut m = mesh_with(&pts, false);
        let b = behavior(false);
        let hullsize = divconqdelaunay(&mut m, &b).expect("oom");
        assert_eq!(hullsize, 8);
        let (hull_edges, live, counted) = end_state_checks(&m);
        assert_eq!((hull_edges, live, counted), (8, 8, 8));
        assert_eq!(m.triangles.high_water(), 16);
        // ring vertices marked, the lattice-interior center not
        for (i, _) in pts.iter().enumerate() {
            let expected = if i == 4 { 0 } else { 1 };
            assert_eq!(m.vx_mark(i as u32 + 1), expected, "vertex {}", i + 1);
        }
        assert_locally_delaunay(&mut m, &b);
    }

    #[test]
    fn divconqdelaunay_discards_duplicates() {
        // one exact duplicate: i comes out 3, one triangle remains;
        // C's quicksort settles the relative order of exactly-equal keys
        // by its pivot draws (no tie-break), so under the fixed seed the
        // pair sorts as [id 2, id 1] and the FIRST input point is the
        // discarded one (pinned; same seed, same output)
        let pts = [(0.0, 0.0), (0.0, 0.0), (1.0, 1.0), (2.0, 0.0)];
        let mut m = mesh_with(&pts, false);
        let b = behavior(false);
        let hullsize = divconqdelaunay(&mut m, &b).expect("oom");
        assert_eq!(hullsize, 3);
        assert_eq!(m.undeads, 1);
        assert_eq!(m.vx_type(1), UNDEADVERTEX, "the discarded duplicate");
        let (_, live, _) = end_state_checks(&m);
        assert_eq!(live, 1);
        // same seed, same output: a rerun on an identical mesh replays the
        // identical dead-stack sequence
        let mut m2 = mesh_with(&pts, false);
        let hullsize2 = divconqdelaunay(&mut m2, &b).expect("oom");
        assert_eq!(hullsize, hullsize2);
        assert_eq!(m.triangles.high_water(), m2.triangles.high_water());
    }

    #[test]
    fn mergehulls_on_hand_built_halves_pins_the_seam() {
        // two hand-built 3-vertex hulls (the exact halves a 6-vertex
        // divconq subdivision makes: divconqrecurse base cases), merged
        // with the axis the top-level call passes (0)
        let pts = [
            (0.0, 0.0), // A
            (1.0, 3.0), // B  (left half, lexicographic)
            (2.0, 1.0), // C
            (3.0, 0.0), // D
            (4.0, 3.0), // E  (right half, lexicographic)
            (5.0, 1.0), // F
        ];
        let mut m = mesh_with(&pts, false);
        let arr = ids(&pts);
        let b = behavior(false);

        let mut farleft = Otri {
            tri: NULL,
            orient: 0,
        };
        let mut innerleft = Otri {
            tri: NULL,
            orient: 0,
        };
        let mut innerright = Otri {
            tri: NULL,
            orient: 0,
        };
        let mut farright = Otri {
            tri: NULL,
            orient: 0,
        };
        // build each half as the recursion would (axis 1 reaches the base
        // cases unchanged)
        divconqrecurse(&mut m, &b, &arr[..3], 3, 1, &mut farleft, &mut innerleft).expect("oom");
        divconqrecurse(&mut m, &b, &arr[3..], 3, 1, &mut innerright, &mut farright).expect("oom");
        // interiors: org(farleft) is the leftmost left vertex, dest(innerleft)
        // the rightmost of the left half, org(innerright) the leftmost of the
        // right, dest(farright) the rightmost right vertex
        assert_eq!(coords(&m, m.org(farleft)), (0.0, 0.0));
        assert_eq!(coords(&m, m.dest(innerleft)), (2.0, 1.0));
        assert_eq!(coords(&m, m.org(innerright)), (3.0, 0.0));
        assert_eq!(coords(&m, m.dest(farright)), (5.0, 1.0));

        mergehulls(
            &mut m,
            &b,
            &mut farleft,
            &mut innerleft,
            &mut innerright,
            &mut farright,
            0,
        )
        .expect("oom");
        // 4 + 4 + 2 = 10 allocated rows (4 per 3-vertex base case plus the
        // two seam triangles)
        assert_eq!(m.triangles.live_count(), 10);
        assert_eq!(m.triangles.high_water(), 10);
        // the merge contract: origin of farleft is the leftmost vertex of
        // the merged triangulation, destination of farright the rightmost
        assert_eq!(coords(&m, m.org(farleft)), (0.0, 0.0));
        assert_eq!(coords(&m, m.dest(farright)), (5.0, 1.0));
        // the seam is knitted tightly: every bond is symmetric, and the
        // apex view over the three orientations of a triangle reads each
        // corner slot exactly once, so null_apex below counts the empty
        // corners: of the ten live rows the five bounding triangles of the
        // hull fan (hull = A, D, F, E, B) carry exactly one each, the five
        // real triangles (2n - 2 - h, n = 6, h = 5) none
        let mut null_apex = 0;
        let mut tr = m.triangles.traversal();
        while let Some(id) = m.triangles.traverse_next(&mut tr) {
            if m.deadtri(id) {
                continue;
            }
            for orient in 0..3u8 {
                let e = Otri { tri: id, orient };
                let s = m.sym(e);
                if s.tri != DUMMY_TRI {
                    assert_eq!(m.sym(s), e, "the bond is symmetric");
                }
                if m.apex(e) == NULL {
                    null_apex += 1;
                }
            }
        }
        assert_eq!(null_apex, 5, "the bounding triangles form the hull fan");
        // and every interior seam edge is Delaunay
        assert_locally_delaunay(&mut m, &b);
        // removeghosts then walks exactly that fan: one connected cycle
        let hullsize = removeghosts(&mut m, &b, &farleft);
        assert_eq!(hullsize, 5);
        let (_, live, _) = end_state_checks(&m);
        assert_eq!(live, 5, "2n - 2 - h real triangles for n = 6, h = 5");
        // hull vertices marked 1; the interior vertex (2.0, 1.0) is unmarked
        for (i, p) in pts.iter().enumerate() {
            let expected = if *p == (2.0, 1.0) { 0 } else { 1 };
            assert_eq!(m.vx_mark(i as u32 + 1), expected, "vertex {:?}", p);
        }
    }

    #[test]
    fn boundingbox_and_removebox_account_the_rows() {
        // the incremental-style box build, mirroring the part-2
        // incrementaldelaunay flow through the landed insertvertex
        let pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let mut m = mesh_with(&pts, false);
        // the bounds are transfernodes' (ioarrays.rs, not landed); the
        // test sets them exactly as that row will
        m.xmin = 0.0;
        m.xmax = 1.0;
        m.ymin = 0.0;
        m.ymax = 1.0;
        let b = behavior(false);
        let before = m.triangles.live_count();
        boundingbox(&mut m, &b).expect("oom");
        // exactly one triangle allocated (the box)
        assert_eq!(m.triangles.live_count(), before + 1);
        // the D12 out-of-pool vertices: no vertex row allocated, all three
        // readable through the chokepoint with the exact C coordinates
        assert_eq!(m.vertices.high_water() as usize, pts.len());
        assert_eq!(m.infvertex1, INFVERTEX1);
        assert_eq!(m.infvertex2, INFVERTEX2);
        assert_eq!(m.infvertex3, INFVERTEX3);
        assert_eq!(coords(&m, m.infvertex1), (0.0 - 50.0, 0.0 - 40.0));
        assert_eq!(coords(&m, m.infvertex2), (1.0 + 50.0, 0.0 - 40.0));
        assert_eq!(coords(&m, m.infvertex3), (0.5, 1.0 + 60.0));
        // dummytri[0] points at the box with orientation zero
        assert_eq!(
            m.tri_slot(DUMMY_TRI, 0),
            encode(Otri { tri: 2, orient: 0 }),
            "first allocated triangle row (ids start at 2)"
        );

        // the incremental insertion pass (the part-2 incrementaldelaunay
        // body, assembled here from the landed rows)
        let mut inserted = 0;
        let mut tr = m.vertices.traversal();
        while let Some(v) = m.vertextraverse(&mut tr) {
            let mut starttri = Otri {
                tri: DUMMY_TRI,
                orient: 0,
            };
            let result = insertvertex(&mut m, &b, v, &mut starttri, None, false, false, false)
                .expect("insert");
            if result == InsertVertexResult::DuplicateVertex {
                m.vx_set_type(v, UNDEADVERTEX);
                m.undeads += 1;
            }
            inserted += 1;
        }
        assert_eq!(inserted, 4);
        assert_eq!(m.undeads, 0);
        // 2n + 1 for a fully interior insertion sequence
        assert_eq!(m.triangles.live_count(), 9);
        assert_eq!(m.triangles.high_water(), 9);

        let hullsize = removebox(&mut m, &b);
        assert_eq!(hullsize, 4);
        assert_eq!(m.triangles.live_count(), 2);
        // maxitems never drops on dealloc (C's semantic)
        assert_eq!(m.triangles.high_water(), 9);
        // the D12 store is reset like the trifree'd C rows
        assert_eq!(m.infvertex1, NULL);
        assert_eq!(m.infvertex2, NULL);
        assert_eq!(m.infvertex3, NULL);
        assert_eq!(coords(&m, INFVERTEX1), (0.0, 0.0));
        assert_eq!(coords(&m, INFVERTEX2), (0.0, 0.0));
        assert_eq!(coords(&m, INFVERTEX3), (0.0, 0.0));
        // all four vertices lie on the hull and are marked
        for v in 1..=4u32 {
            assert_eq!(m.vx_mark(v), 1);
        }
        // the remaining two triangles are the square's split and reference
        // only real vertices
        let (hull_edges, live, _) = end_state_checks(&m);
        assert_eq!((hull_edges, live), (4, 2));
    }
    // =====================================================================
    // Part 2: incremental, sweepline, event heap, splay tree, dispatcher.
    // =====================================================================

    /// transfernodes' bounds role (triangle.c:7753-7754, the ioarrays
    /// row, not landed): the input bounds and xminextreme exactly as
    /// that row will write them.
    fn set_bounds(m: &mut Mesh, pts: &[(f64, f64)]) {
        let mut xmin = f64::INFINITY;
        let mut xmax = f64::NEG_INFINITY;
        let mut ymin = f64::INFINITY;
        let mut ymax = f64::NEG_INFINITY;
        for &(x, y) in pts {
            xmin = xmin.min(x);
            xmax = xmax.max(x);
            ymin = ymin.min(y);
            ymax = ymax.max(y);
        }
        m.xmin = xmin;
        m.xmax = xmax;
        m.ymin = ymin;
        m.ymax = ymax;
        m.xminextreme = 10.0 * xmin - 9.0 * xmax;
    }

    /// A fresh splaynode carrying keyedge/keydest like splayinsert
    /// builds, both children null.
    fn fresh_snode(m: &mut Mesh, keyedge: Otri, keydest: u32) -> u32 {
        let n = m.splay_alloc().expect("splaynode alloc");
        m.set_splay_keyedge(n, keyedge);
        m.set_splay_keydest(n, keydest);
        m.set_splay_lchild(n, NULL);
        m.set_splay_rchild(n, NULL);
        n
    }

    /// The splay test front edge: dest (0,0), apex (2,0) - search points
    /// with x >= 2 sit right of it, points left of (0,0) sit left.
    fn splay_key_mesh() -> (Mesh, Otri) {
        let mut m = mesh_with(&[(0.0, 0.0), (2.0, 0.0)], false);
        let t = m.maketriangle(false, false).expect("tri");
        m.setdest(t, 1);
        m.setapex(t, 2);
        (m, t)
    }

    /// An arena cell with the given keys and heapposition.
    fn cell(xkey: f64, ykey: f64, heapposition: i32) -> Event {
        Event {
            xkey,
            ykey,
            eventptr: EventPtr::Null,
            heapposition,
        }
    }

    #[test]
    fn incrementaldelaunay_square_end_state() {
        let pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let mut m = mesh_with(&pts, false);
        set_bounds(&mut m, &pts);
        let b = behavior(false);
        let hullsize = incrementaldelaunay(&mut m, &b).expect("oom");
        // the hull of the cocircular square: 4 edges, 2 triangles
        assert_eq!(hullsize, 4);
        let (hull_edges, live, counted) = end_state_checks(&m);
        assert_eq!((hull_edges, live, counted), (4, 2, 2));
        // 2n + 1 triangle rows through the bounding-box insertion pass
        assert_eq!(m.triangles.high_water(), 9);
        assert_eq!(m.undeads, 0);
        for v in 1..=4u32 {
            assert_eq!(m.vx_mark(v), 1);
        }
        assert_locally_delaunay(&mut m, &b);
    }

    #[test]
    fn incrementaldelaunay_pentagon_and_duplicate_accounting() {
        // five points in convex position
        let pts = [(0.0, 0.0), (2.0, 0.0), (3.0, 1.0), (1.0, 3.0), (-1.0, 1.0)];
        let mut m = mesh_with(&pts, false);
        set_bounds(&mut m, &pts);
        let b = behavior(false);
        let hullsize = incrementaldelaunay(&mut m, &b).expect("oom");
        assert_eq!(hullsize, 5);
        let (hull_edges, live, counted) = end_state_checks(&m);
        assert_eq!((hull_edges, live, counted), (5, 3, 3));
        assert_eq!(m.triangles.high_water(), 11);
        assert_eq!(m.undeads, 0);

        // a duplicate mid-sequence: DUPLICATEVERTEX accounting at the C
        // site (triangle.c:4779-4780), one less insertion row high-water
        let pts2 = [(0.0, 0.0), (0.0, 0.0), (1.0, 0.0), (0.0, 1.0)];
        let mut m2 = mesh_with(&pts2, false);
        set_bounds(&mut m2, &pts2);
        let hullsize2 = incrementaldelaunay(&mut m2, &b).expect("oom");
        assert_eq!(hullsize2, 3);
        assert_eq!(m2.undeads, 1);
        assert_eq!(m2.vx_type(2), UNDEADVERTEX, "the discarded duplicate");
        let (hull_edges2, live2, _) = end_state_checks(&m2);
        assert_eq!((hull_edges2, live2), (3, 1));
        // three real insertions: 2*3 + 1 rows
        assert_eq!(m2.triangles.high_water(), 7);
    }

    #[test]
    fn eventheapinsert_sifts_up_the_c_sequence() {
        // keys pinned by hand: the y coordinate is the primary key, the
        // x the tiebreak; the parent must be strictly below to stop
        let mut events = [
            cell(0.0, 5.0, -1),
            cell(0.0, 4.0, -1),
            cell(0.0, 3.0, -1),
            cell(0.0, 10.0, -1),
        ];
        let mut heap = [0u32; 4];
        // the C insertion sequence of createeventheap: sift-up only,
        // each smaller y walks over its parent (root 0 has y 5, the
        // y 4 event takes slot 0, the y 3 walks over both)
        eventheapinsert(&mut heap[..], 0, 0, &mut events[..]);
        eventheapinsert(&mut heap[..], 1, 1, &mut events[..]);
        eventheapinsert(&mut heap[..], 2, 2, &mut events[..]);
        assert_eq!(&heap[..3], &[2, 0, 1][..]);
        assert_eq!(
            (
                events[2].heapposition,
                events[1].heapposition,
                events[0].heapposition
            ),
            (0, 2, 1)
        );
        // a larger event stops at its parent
        eventheapinsert(&mut heap[..], 3, 3, &mut events[..]);
        assert_eq!(&heap[..4], &[2, 0, 1, 3][..]);
        assert_eq!(events[3].heapposition, 3);
    }

    #[test]
    fn eventheapify_sifts_down_repairing_positions() {
        let mut events = [cell(0.0, 3.0, 0), cell(0.0, 1.0, 1), cell(0.0, 4.0, 2)];
        let mut heap = [0u32, 1, 2];
        eventheapify(&mut heap[..], 3, 0, &mut events[..]);
        // the (0,1) child bubbles above both; positions written exactly
        assert_eq!(&heap[..3], &[1, 0, 2][..]);
        assert_eq!((events[1].heapposition, events[0].heapposition), (0, 1));
        assert_eq!(events[2].heapposition, 2, "untouched by the sift");
    }

    #[test]
    fn eventheapdelete_leaf_root_and_sift_up_paths() {
        // (a) deleting the last used slot: the mover lands back where it
        // was, heapify over a one-slot tail does nothing
        let mut events = [cell(0.0, 1.0, 0), cell(0.0, 3.0, 1), cell(0.0, 4.0, 2)];
        let mut heap = [0u32, 1, 2];
        eventheapdelete(&mut heap[..], 3, 2, &mut events[..]);
        assert_eq!(&heap[..3], &[0, 1, 2][..]);
        assert_eq!(events[2].heapposition, 2);

        // (b) deleting the root: the last slot moves in, then sifts down
        let mut events_b = [cell(0.0, 1.0, 0), cell(0.0, 3.0, 1), cell(0.0, 4.0, 2)];
        let mut heap_b = [0u32, 1, 2];
        eventheapdelete(&mut heap_b[..], 3, 0, &mut events_b[..]);
        assert_eq!(&heap_b[..3], &[1, 2, 2][..], "the tail mirror stays");
        assert_eq!((events_b[1].heapposition, events_b[2].heapposition), (0, 1));

        // (c) deleting a leaf slot whose mover is small: the do-while
        // sifts it up past its parent (triangle.c:4892-4905)
        let mut events_c = [
            cell(0.0, 1.0, 0),
            cell(0.0, 4.0, 1),
            cell(0.0, 3.0, 2),
            cell(0.0, 2.0, 3),
        ];
        let mut heap_c = [0u32, 1, 2, 3];
        eventheapdelete(&mut heap_c[..], 4, 3, &mut events_c[..]);
        assert_eq!(&heap_c[..4], &[0, 3, 2, 1][..]);
        assert_eq!(
            (
                events_c[0].heapposition,
                events_c[3].heapposition,
                events_c[1].heapposition
            ),
            (0, 1, 3)
        );
        assert_eq!(events_c[2].heapposition, 2, "untouched by the sift-up");
    }

    #[test]
    fn createeventheap_builds_the_exact_c_heap_and_free_list() {
        // five sites with y keys 0, 3, 2, 1, -1 in pool (id) order: the
        // hand-computed min-heap of the C insertion sequence
        let pts = [(2.0, 0.0), (1.0, 3.0), (0.0, 2.0), (3.0, 1.0), (0.0, -1.0)];
        let m = mesh_with(&pts, false);
        let (heap, events, freeevents) = createeventheap(&m).expect("oom");
        // maxevents = (3 * 5) / 2 = 7 cells; the heap holds the 5 sites
        assert_eq!(heap.len(), 7);
        assert_eq!(&heap[..5], &[4, 0, 2, 1, 3][..]);
        // traversal order: events[i] carries vertex id i + 1
        for i in 0..5 {
            assert_eq!(events[i].eventptr, EventPtr::Vertex(i as u32 + 1));
            assert_eq!(events[i].xkey, pts[i].0);
            assert_eq!(events[i].ykey, pts[i].1);
        }
        assert_eq!(
            (
                events[0].heapposition,
                events[1].heapposition,
                events[2].heapposition,
                events[3].heapposition,
                events[4].heapposition
            ),
            (1, 3, 2, 4, 0)
        );
        // the free list threads the spare cells high to low: the head
        // lands at events[invertices] exactly like C
        assert_eq!(freeevents, Some(5));
        assert_eq!(events[5].eventptr, EventPtr::Free(Some(6)));
        assert_eq!(events[6].eventptr, EventPtr::Free(None));
    }

    #[test]
    fn rightofhyperbola_early_and_formula_paths() {
        let pts = [
            (0.0, 0.0),
            (2.0, 0.0),
            (0.0, 5.0),
            (1.0, 1.0),
            (3.0, 1.0),
            (0.0, 2.0),
            (1.0, 2.0),
        ];
        let mut m = mesh_with(&pts, false);
        let t = m.maketriangle(false, false).expect("tri");
        let front = Otri {
            tri: t.tri,
            orient: 0,
        };
        // left (0,0), right (2,0): newsite x right of the right end
        m.setdest(t, 1);
        m.setapex(t, 2);
        assert!(rightofhyperbola(&mut m, &front, (3.0, -7.0)));
        // left (2,0), right (0,5): newsite left of the left end
        m.setdest(t, 2);
        m.setapex(t, 3);
        // -20 > 39 is false: the formula answers no
        assert!(!rightofhyperbola(&mut m, &front, (-1.0, 2.0)));
        // left (1,1), right (3,1): the sweep-side formula answers yes
        m.setdest(t, 4);
        m.setapex(t, 5);
        assert!(rightofhyperbola(&mut m, &front, (0.0, 0.0)));
        // left (0,2), right (1,2), newsite (0.5, 0): both arms are
        // equidistant (2*(0.25+4) on either side), so the C strict >
        // answers no (the tie, triangle.c:4970)
        m.setdest(t, 6);
        m.setapex(t, 7);
        assert!(!rightofhyperbola(&mut m, &front, (0.5, 0.0)));
        // every call counted exactly at the C site
        assert_eq!(m.hyperbolacount, 4);
    }

    #[test]
    fn circletop_pins_exact_circle_tops() {
        let pts = [
            (0.0, 0.0),
            (2.0, 0.0),
            (1.0, 1.0),
            (4.0, 0.0),
            (2.0, 3.0),
            (1.0, -1.0),
            (5.0, 0.0),
            (1.0, 2.0),
        ];
        let mut m = mesh_with(&pts, false);
        let b = behavior(false);
        // the unit circle: top exactly 1 (ccwabc from the landed
        // predicate fast path, hand-checked to be 2)
        let ccw = tri_counterclockwise(
            &mut m,
            &b,
            [pts[0].0, pts[0].1, 0.0],
            [pts[1].0, pts[1].1, 0.0],
            [pts[2].0, pts[2].1, 0.0],
        );
        assert_eq!(ccw, 2.0);
        assert_eq!(circletop(&mut m, 1, 2, 3, ccw), 1.0);
        // top exactly 3 through (0,0),(4,0),(2,3): the sqrt argument is
        // the perfect square 13*13*16
        let ccw = tri_counterclockwise(
            &mut m,
            &b,
            [pts[0].0, pts[0].1, 0.0],
            [pts[3].0, pts[3].1, 0.0],
            [pts[4].0, pts[4].1, 0.0],
        );
        assert_eq!(ccw, 12.0);
        assert_eq!(circletop(&mut m, 1, 4, 5, ccw), 3.0);
        // a clockwise triple: the negative ccwabc divides, top exactly -1
        let ccw = tri_counterclockwise(
            &mut m,
            &b,
            [pts[0].0, pts[0].1, 0.0],
            [pts[1].0, pts[1].1, 0.0],
            [pts[5].0, pts[5].1, 0.0],
        );
        assert_eq!(ccw, -2.0);
        assert_eq!(circletop(&mut m, 1, 2, 6, ccw), -1.0);
        // a scalene triple with a nonzero numerator: top exactly 2.5
        let ccw = tri_counterclockwise(
            &mut m,
            &b,
            [pts[0].0, pts[0].1, 0.0],
            [pts[6].0, pts[6].1, 0.0],
            [pts[7].0, pts[7].1, 0.0],
        );
        assert_eq!(ccw, 10.0);
        assert_eq!(circletop(&mut m, 1, 7, 8, ccw), 2.5);
        assert_eq!(m.circletopcount, 4);
        assert_eq!(m.counterclockcount, 4);
    }

    #[test]
    fn check4deadevent_frees_heapdeletes_and_clears() {
        let mut m = mesh_with(&[], false);
        let t = m.maketriangle(false, false).expect("tri");
        // a hand-built front: keys (0,1), (0,3), (0,4) in a 3-heap
        let mut events = vec![cell(0.0, 0.0, 0), cell(0.0, 0.0, 1), cell(0.0, 0.0, 2)];
        events[0].ykey = 1.0;
        events[1].ykey = 3.0;
        events[2].ykey = 4.0;
        let mut heap = vec![0u32, 1, 2];
        let mut heapsize: i32 = 3;
        let mut freeevents: Option<u32> = None;
        // the org-slot pun: event 1 rides the handle at orientation 0
        let o0 = Otri {
            tri: t.tri,
            orient: 0,
        };
        m.setorgword(o0, event_word(1));
        check4deadevent(
            &mut m,
            &o0,
            &mut freeevents,
            &mut heap,
            &mut heapsize,
            &mut events,
        );
        assert_eq!(&heap[..2], &[0, 2][..], "the tail slot filled the hole");
        assert_eq!(events[2].heapposition, 1);
        assert_eq!(events[1].eventptr, EventPtr::Free(None), "on the free list");
        assert_eq!(freeevents, Some(1));
        assert_eq!(heapsize, 2);
        assert_eq!(m.orgword(o0), 0, "the pun cleared (triangle.c:5016)");

        // a chained free: event 2 rides the orientation-1 handle and
        // now links the previous head
        let o1 = Otri {
            tri: t.tri,
            orient: 1,
        };
        m.setorgword(o1, event_word(2));
        check4deadevent(
            &mut m,
            &o1,
            &mut freeevents,
            &mut heap,
            &mut heapsize,
            &mut events,
        );
        assert_eq!(events[2].eventptr, EventPtr::Free(Some(1)));
        assert_eq!(freeevents, Some(2));
        assert_eq!(heapsize, 1);
        assert_eq!(&heap[..1], &[0][..]);
        assert_eq!(m.orgword(o1), 0);

        // a NULL pun slot: an exact no-op
        check4deadevent(
            &mut m,
            &o0,
            &mut freeevents,
            &mut heap,
            &mut heapsize,
            &mut events,
        );
        assert_eq!(freeevents, Some(2));
        assert_eq!(heapsize, 1);
    }

    #[test]
    fn splay_restructures_rotations_and_merges() {
        // the null tree answers null with the searchtri untouched
        let (mut m, _t) = splay_key_mesh();
        let mut st = Otri {
            tri: NULL,
            orient: 0,
        };
        assert_eq!(splay(&mut m, NULL, (5.0, 1.0), &mut st), NULL);
        assert_eq!(st.tri, NULL);
        assert_eq!(m.hyperbolacount, 0);

        // a single matching node: no rotation, the keyedge answers
        let (mut m, t) = splay_key_mesh();
        let key = Otri {
            tri: t.tri,
            orient: 0,
        };
        let r = fresh_snode(&mut m, key, 1);
        assert_eq!(splay(&mut m, r, (5.0, 1.0), &mut st), r);
        assert_eq!(st, key);
        assert_eq!(m.hyperbolacount, 1);
        assert_eq!(m.splaynodes.live_count(), 1);

        // a right child with the same fate: the classic rotation hoists
        // the child above the root (triangle.c:5069-5078)
        let (mut m, t) = splay_key_mesh();
        let key = Otri {
            tri: t.tri,
            orient: 0,
        };
        let r = fresh_snode(&mut m, key, 1);
        let c = fresh_snode(&mut m, key, 1);
        m.set_splay_rchild(r, c);
        assert_eq!(splay(&mut m, r, (5.0, 1.0), &mut st), c);
        assert_eq!(m.splay_lchild(c), r, "the child took its parent");
        assert_eq!(m.splay_rchild(r), NULL);
        assert_eq!(st, key);
        assert_eq!(m.hyperbolacount, 2, "root and child tests only");

        // a keydest mismatch at a leaf: both subtree splays are null,
        // the node dies (triangle.c:5105), nothing survives
        let (mut m, t) = splay_key_mesh();
        let key = Otri {
            tri: t.tri,
            orient: 0,
        };
        let r = fresh_snode(&mut m, key, 99);
        assert_eq!(splay(&mut m, r, (5.0, 1.0), &mut st), NULL);
        assert_eq!(m.splaynodes.live_count(), 0);
        // the dead branch never touches searchtri; it keeps the value
        // the caller entered with (the key of the previous block)
        assert_eq!(st, key);

        // merge case 3 (triangle.c:5110-5113): the left tree has no
        // right child, it becomes the right tree's left child
        let (mut m, t) = splay_key_mesh();
        let key = Otri {
            tri: t.tri,
            orient: 0,
        };
        let r = fresh_snode(&mut m, key, 99);
        let left = fresh_snode(&mut m, key, 1);
        let right = fresh_snode(&mut m, key, 1);
        m.set_splay_lchild(r, left);
        m.set_splay_rchild(r, right);
        assert_eq!(splay(&mut m, r, (-3.0, 5.0), &mut st), right);
        assert_eq!(m.splay_lchild(right), left);
        assert_eq!(m.splay_rchild(left), NULL);
        assert_eq!(m.splaynodes.live_count(), 2, "the mismatch died");

        // merge case 4 (triangle.c:5114-5117): the left tree keeps its
        // right child, the trees graft at that seam
        let (mut m, t) = splay_key_mesh();
        let key = Otri {
            tri: t.tri,
            orient: 0,
        };
        let r = fresh_snode(&mut m, key, 99);
        let left = fresh_snode(&mut m, key, 1);
        let leftright = fresh_snode(&mut m, key, 1);
        let right = fresh_snode(&mut m, key, 1);
        m.set_splay_lchild(r, left);
        m.set_splay_rchild(left, leftright);
        m.set_splay_rchild(r, right);
        assert_eq!(splay(&mut m, r, (-3.0, 5.0), &mut st), left);
        assert_eq!(m.splay_rchild(left), right);
        assert_eq!(m.splay_lchild(right), leftright);
        assert_eq!(m.splaynodes.live_count(), 3);

        // the "Holy Toledo" merge (triangle.c:5118-5125): both returned
        // trees keep their inner children. That needs two distinct key
        // edges under the same searchpoint: keya (dest 1) answers
        // right of (5,1) by its early-out, keyb (dest 2, the reversed
        // edge) answers left, so the left node returns itself with its
        // rchild intact and the right node returns itself with its
        // lchild intact.
        let (mut m, t) = splay_key_mesh();
        let keya = Otri {
            tri: t.tri,
            orient: 0,
        };
        let t2 = m.maketriangle(false, false).expect("tri");
        m.setdest(t2, 2);
        m.setapex(t2, 1);
        let keyb = Otri {
            tri: t2.tri,
            orient: 0,
        };
        let r = fresh_snode(&mut m, keya, 99);
        let left = fresh_snode(&mut m, keyb, 2);
        let leftright = fresh_snode(&mut m, keyb, 2);
        let right = fresh_snode(&mut m, keya, 1);
        let rightleft = fresh_snode(&mut m, keya, 1);
        m.set_splay_lchild(r, left);
        m.set_splay_rchild(left, leftright);
        m.set_splay_rchild(r, right);
        m.set_splay_lchild(right, rightleft);
        assert_eq!(splay(&mut m, r, (5.0, 1.0), &mut st), left);
        assert_eq!(m.splay_rchild(leftright), right);
        assert_eq!(m.splay_rchild(left), leftright);
        assert_eq!(m.splaynodes.live_count(), 5 - 1);
    }

    #[test]
    fn splayinsert_splits_the_root_per_hyperbola() {
        // into an empty tree: the plain node, no hyperbola test at all
        let (mut m, t) = splay_key_mesh();
        let newkey = Otri {
            tri: t.tri,
            orient: 0,
        };
        let n = splayinsert(&mut m, NULL, &newkey, (5.0, 1.0)).expect("oom");
        assert_eq!(m.splay_keyedge(n), newkey);
        assert_eq!(m.splay_keydest(n), 1, "dest(*newkey)");
        assert_eq!((m.splay_lchild(n), m.splay_rchild(n)), (NULL, NULL));
        assert_eq!(m.hyperbolacount, 0);

        // right of the root (triangle.c:5145-5148): the root becomes
        // the left child, the root's right child crosses over
        let (mut m, t) = splay_key_mesh();
        let r = fresh_snode(
            &mut m,
            Otri {
                tri: t.tri,
                orient: 0,
            },
            1,
        );
        let n = splayinsert(
            &mut m,
            r,
            &Otri {
                tri: t.tri,
                orient: 2,
            },
            (5.0, 1.0),
        )
        .expect("oom");
        assert_eq!(m.splay_lchild(n), r);
        assert_eq!(m.splay_rchild(n), NULL);
        assert_eq!(m.splay_rchild(r), NULL);
        assert_eq!(m.hyperbolacount, 1);

        // left of the root (triangle.c:5149-5152): the mirror
        let (mut m, t) = splay_key_mesh();
        let r = fresh_snode(
            &mut m,
            Otri {
                tri: t.tri,
                orient: 0,
            },
            1,
        );
        let n = splayinsert(
            &mut m,
            r,
            &Otri {
                tri: t.tri,
                orient: 2,
            },
            (-3.0, 5.0),
        )
        .expect("oom");
        assert_eq!(m.splay_rchild(n), r);
        assert_eq!(m.splay_lchild(n), NULL);
        assert_eq!(m.splay_lchild(r), NULL);
        assert_eq!(m.hyperbolacount, 1);
    }

    #[test]
    fn circletopinsert_pins_the_searchpoint_arithmetic() {
        // pa (8,1), pb (6,1), pc (4,2): the circle center's x is 7 -
        // the split point lands right of the front key edge
        let pts = [
            (0.0, 0.0),
            (2.0, 0.0),
            (8.0, 1.0),
            (6.0, 1.0),
            (4.0, 2.0),
            (-2.0, 1.0),
            (-4.0, 1.0),
        ];
        let b = behavior(false);
        {
            // an empty tree: one node, no hyperbola test, one predicate
            let mut m = mesh_with(&pts, false);
            let t2 = m.maketriangle(false, false).expect("tri");
            m.setdest(t2, 1);
            let newkey = Otri {
                tri: t2.tri,
                orient: 0,
            };
            let n = circletopinsert(&mut m, &b, NULL, &newkey, 3, 4, 5, 5.0).expect("oom");
            assert_eq!(m.splay_keyedge(n), newkey);
            assert_eq!(m.splay_keydest(n), 1);
            assert_eq!((m.splay_lchild(n), m.splay_rchild(n)), (NULL, NULL));
            assert_eq!(m.counterclockcount, 1);
            assert_eq!(m.hyperbolacount, 0);
        }
        {
            // right of the root: the splay touches the root's edge once
            // and the insert splits right (searchpoint.x = 4 + 3 = 7)
            let mut m = mesh_with(&pts, false);
            let t = m.maketriangle(false, false).expect("tri");
            m.setdest(t, 1);
            m.setapex(t, 2);
            let r = fresh_snode(
                &mut m,
                Otri {
                    tri: t.tri,
                    orient: 0,
                },
                1,
            );
            let t2 = m.maketriangle(false, false).expect("tri");
            m.setdest(t2, 1);
            let newkey = Otri {
                tri: t2.tri,
                orient: 0,
            };
            let n = circletopinsert(&mut m, &b, r, &newkey, 3, 4, 5, 5.0).expect("oom");
            assert_eq!(m.splay_lchild(n), r, "right of the root");
            assert_eq!(m.splay_rchild(n), NULL);
            assert_eq!(m.splay_rchild(r), NULL);
            assert_eq!(m.counterclockcount, 1);
            assert_eq!(m.hyperbolacount, 2);
        }
        {
            // pa (-2,1), pb (-4,1), pc (0,0): searchpoint.x = -3, left
            // of the front key edge - the mirror split
            let mut m = mesh_with(&pts, false);
            let t = m.maketriangle(false, false).expect("tri");
            m.setdest(t, 1);
            m.setapex(t, 2);
            let r = fresh_snode(
                &mut m,
                Otri {
                    tri: t.tri,
                    orient: 0,
                },
                1,
            );
            let t2 = m.maketriangle(false, false).expect("tri");
            m.setdest(t2, 1);
            let newkey = Otri {
                tri: t2.tri,
                orient: 0,
            };
            let n = circletopinsert(&mut m, &b, r, &newkey, 6, 7, 1, 5.0).expect("oom");
            assert_eq!(m.splay_rchild(n), r, "left of the root");
            assert_eq!(m.splay_lchild(n), NULL);
            assert_eq!(m.splay_lchild(r), NULL);
            assert_eq!(m.counterclockcount, 1);
            assert_eq!(m.hyperbolacount, 2);
        }
    }

    #[test]
    fn frontlocate_walks_the_front_from_bottommost() {
        let pts = [(0.0, 0.0), (2.0, 0.0), (5.0, 5.0), (-1.0, 5.0)];
        let mut m = mesh_with(&pts, false);
        let t = m.maketriangle(false, false).expect("tri");
        m.setdest(t, 1);
        m.setapex(t, 2);
        let bottommost = Otri {
            tri: t.tri,
            orient: 0,
        };
        let mut st = Otri {
            tri: NULL,
            orient: 0,
        };
        let mut farright = false;
        // a search point right of the only front edge: one onextself
        // steps into outer space, whose null row fails the hyperbola
        // test - the walk stops without wrapping the front
        assert_eq!(
            frontlocate(&mut m, NULL, &bottommost, 3, &mut st, &mut farright),
            NULL
        );
        assert_eq!(
            st,
            Otri {
                tri: DUMMY_TRI,
                orient: 0
            },
            "one onextself into outer space"
        );
        assert!(!farright);
        // a search point left of the front edge: the loop body never
        // runs, the bottommost handle answers untouched
        let mut st2 = Otri {
            tri: NULL,
            orient: 0,
        };
        assert_eq!(
            frontlocate(&mut m, NULL, &bottommost, 4, &mut st2, &mut farright),
            NULL
        );
        assert_eq!(st2, bottommost);
        assert!(!farright);
        // the three hyperbola tests: one front edge, one outer-space
        // edge, then the left-point rejection
        assert_eq!(m.hyperbolacount, 3);
    }

    #[test]
    fn sweeplinedelaunay_square_end_state() {
        let pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let mut m = mesh_with(&pts, false);
        set_bounds(&mut m, &pts);
        let b = behavior(false);
        let hullsize = sweeplinedelaunay(&mut m, &b).expect("oom");
        assert_eq!(hullsize, 4);
        let (hull_edges, live, counted) = end_state_checks(&m);
        assert_eq!((hull_edges, live, counted), (4, 2, 2));
        // the initial tube plus one front pair per remaining site
        assert_eq!(m.triangles.high_water(), 6);
        assert_eq!(m.undeads, 0);
        assert!(m.hyperbolacount > 0);
        for v in 1..=4u32 {
            assert_eq!(m.vx_mark(v), 1);
        }
        assert_locally_delaunay(&mut m, &b);

        // same seed, same output: an identical rerun replays every
        // counter byte for byte
        let mut m2 = mesh_with(&pts, false);
        set_bounds(&mut m2, &pts);
        let hullsize2 = sweeplinedelaunay(&mut m2, &b).expect("oom");
        assert_eq!(hullsize, hullsize2);
        assert_eq!(m.hyperbolacount, m2.hyperbolacount);
        assert_eq!(m.circletopcount, m2.circletopcount);
        assert_eq!(m.triangles.high_water(), m2.triangles.high_water());
    }

    #[test]
    fn sweeplinedelaunay_grid9_is_delaunay_and_deterministic() {
        let pts = [
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
            (0.0, 1.0),
            (1.0, 1.0),
            (2.0, 1.0),
            (0.0, 2.0),
            (1.0, 2.0),
            (2.0, 2.0),
        ];
        let mut m = mesh_with(&pts, false);
        set_bounds(&mut m, &pts);
        let b = behavior(false);
        let hullsize = sweeplinedelaunay(&mut m, &b).expect("oom");
        assert_eq!(hullsize, 8);
        let (hull_edges, live, counted) = end_state_checks(&m);
        assert_eq!((hull_edges, live, counted), (8, 8, 8));
        assert_eq!(m.triangles.high_water(), 16);
        assert_eq!(m.undeads, 0);
        for (i, _) in pts.iter().enumerate() {
            let expected = if i == 4 { 0 } else { 1 };
            assert_eq!(m.vx_mark(i as u32 + 1), expected, "vertex {}", i + 1);
        }
        assert_locally_delaunay(&mut m, &b);
        assert!(m.hyperbolacount > 0);

        let mut m2 = mesh_with(&pts, false);
        set_bounds(&mut m2, &pts);
        let hullsize2 = sweeplinedelaunay(&mut m2, &b).expect("oom");
        assert_eq!(hullsize, hullsize2);
        assert_eq!(m.hyperbolacount, m2.hyperbolacount);
        assert_eq!(m.circletopcount, m2.circletopcount);
        assert_eq!(m.undeads, m2.undeads);
    }

    #[test]
    fn sweeplinedelaunay_duplicate_accounting_and_tri_input() {
        let b = behavior(false);
        // a site-event duplicate mid-sweep: the duplicate is marked
        // undead at the C site (triangle.c:5322-5325) and skipped
        let pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let mut m = mesh_with(&pts, false);
        set_bounds(&mut m, &pts);
        let hullsize = sweeplinedelaunay(&mut m, &b).expect("oom");
        assert_eq!(hullsize, 4, "the four distinct sites form the square");
        assert_eq!(m.undeads, 1);
        assert_eq!(m.vx_type(3), UNDEADVERTEX);
        let (hull_edges, live, _) = end_state_checks(&m);
        assert_eq!((hull_edges, live), (4, 2));

        // every vertex identical: the do-while runs dry and C's
        // TRI_INPUT rides the long channel out (triangle.c:5256-5258);
        // the port mirrors the literal value
        let pts2 = [(1.0, 1.0), (1.0, 1.0)];
        let mut m2 = mesh_with(&pts2, false);
        set_bounds(&mut m2, &pts2);
        let status = sweeplinedelaunay(&mut m2, &b).expect("oom");
        assert_eq!(status, crate::error::InternalStatus::Input as i64);
        assert_eq!(m2.undeads, 1);
        assert_eq!(m2.vx_type(2), UNDEADVERTEX);
        // via the dispatcher the value flows out as C's hullsize oddity:
        // the two tube rows are alive, so the hull "size" is TRI_INPUT
        let b_sw = Behavior {
            sweepline: true,
            ..behavior(false)
        };
        let mut m3 = mesh_with(&pts2, false);
        set_bounds(&mut m3, &pts2);
        assert_eq!(
            delaunay(&mut m3, &b_sw).expect("oom"),
            crate::error::InternalStatus::Input as i64
        );

        // two distinct collinear points after a duplicate: the ghosts
        // all dissolve, removeghosts counts the two hull edges and the
        // dispatcher answers 0 - C's collinear branch (triangle.c:5448)
        let pts3 = [(0.0, 0.0), (0.0, 0.0), (1.0, 0.0)];
        let mut m4 = mesh_with(&pts3, false);
        set_bounds(&mut m4, &pts3);
        let raw = sweeplinedelaunay(&mut m4, &b).expect("oom");
        assert_eq!(raw, 2);
        assert_eq!(m4.triangles.live_count(), 0);
        assert_eq!(m4.undeads, 1);
        let mut m5 = mesh_with(&pts3, false);
        set_bounds(&mut m5, &pts3);
        assert_eq!(delaunay(&mut m5, &b_sw).expect("oom"), 0);
    }

    #[test]
    fn delaunay_dispatcher_selects_per_flags() {
        use crate::error::InternalStatus;

        let square = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let pentagon = [(0.0, 0.0), (2.0, 0.0), (3.0, 1.0), (1.0, 3.0), (-1.0, 1.0)];

        // -i: the insertion machinery (2n + 1 rows), zero hyperbola tests
        let b_inc = Behavior {
            incremental: true,
            ..behavior(false)
        };
        let mut m = mesh_with(&square, false);
        set_bounds(&mut m, &square);
        assert_eq!(delaunay(&mut m, &b_inc).expect("oom"), 4);
        assert_eq!(m.triangles.high_water(), 9);
        assert_eq!(m.hyperbolacount, 0);
        let (hull_edges, live, _) = end_state_checks(&m);
        assert_eq!((hull_edges, live), (4, 2));

        // -F: the sweepline machinery (hyperbola tests happen)
        let b_sw = Behavior {
            sweepline: true,
            ..behavior(false)
        };
        let mut m2 = mesh_with(&square, false);
        set_bounds(&mut m2, &square);
        assert_eq!(delaunay(&mut m2, &b_sw).expect("oom"), 4);
        assert!(m2.hyperbolacount > 0);

        // both flags: the C if-order picks incremental
        let b_both = Behavior {
            incremental: true,
            sweepline: true,
            ..behavior(false)
        };
        let mut m3 = mesh_with(&square, false);
        set_bounds(&mut m3, &square);
        assert_eq!(delaunay(&mut m3, &b_both).expect("oom"), 4);
        assert_eq!(m3.triangles.high_water(), 9);
        assert_eq!(m3.hyperbolacount, 0);

        // neither flag: divide-and-conquer, the default
        let b_d = behavior(false);
        let mut m4 = mesh_with(&pentagon, false);
        assert_eq!(delaunay(&mut m4, &b_d).expect("oom"), 5);
        assert_eq!(m4.hyperbolacount, 0);
        assert_eq!(m4.undeads, 0);

        // the dispatcher zeroes eextras before sizing the triangle rows
        // (triangle.c:5433): a 3-extras mesh re-densifies to 6 words
        let mut m5 = Mesh::new(&behavior(false), 5, 2, 0, 3).expect("mesh init");
        for &(x, y) in &pentagon {
            let v = m5.vx_alloc().expect("vertex");
            m5.vx_set_real(v, 0, x);
            m5.vx_set_real(v, 1, y);
        }
        assert_eq!(m5.eextras, 3);
        assert_eq!(m5.triangles.stride(), 9);
        set_bounds(&mut m5, &pentagon);
        assert_eq!(delaunay(&mut m5, &b_d).expect("oom"), 5);
        assert_eq!(m5.eextras, 0);
        assert_eq!(m5.triangles.stride(), 6);

        // the collinear input through any method leaves no triangles
        let line = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)];
        let mut m6 = mesh_with(&line, false);
        set_bounds(&mut m6, &line);
        assert_eq!(delaunay(&mut m6, &b_d).expect("oom"), 0);
        assert_eq!(m6.triangles.live_count(), 0);
        // TRI_INPUT is pinned as C's -3 through the channel it rides
        assert_eq!(InternalStatus::Input as i64, -3);
    }
}
