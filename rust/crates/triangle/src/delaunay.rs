//! Delaunay triangulation, part 1: the divide-and-conquer subtree
//! (vertexsort, vertexmedian, alternateaxes, mergehulls, divconqrecurse,
//! removeghosts, divconqdelaunay; plus the bounding box rows boundingbox
//! and removebox; triangle.c:3866-4748, the "Divide-and-conquer Delaunay
//! triangulation" section and the bounding box tail of the incremental
//! section it shares). Assignment: the nine part-1 rows of
//! docs/function-mapping.md. Part 2 (not this PR): incrementaldelaunay,
//! the event heap, the splay tree, rightofhyperbola, circletop,
//! check4deadevent, sweeplinedelaunay and the delaunay() dispatcher; no
//! part-1 function calls any of them, and the api.rs scaffolding of the
//! triangulate entry point stays untouched until part 2 plus construct.rs
//! land (the delaunay() dispatcher is the C caller of divconqdelaunay).
//!
//! Transliteration notes (docs/deviation-register.md, D1-D11; the new D12):
//! - C `vertex` mirrors to u32 vertex row ids (D1): compared for equality
//!   exactly where C compares pointers, and read through the chokepoint
//!   (vx_real) exactly where C dereferences.
//! - The D10 per-run seed (Mesh::randomseed) drives vertexsort's and
//!   vertexmedian's pivots and hence the whole merge walk: same seed, same
//!   swaps, same output.
//! - C's DEBUG-only printf of the duplicate-vertex warning
//!   (divconqdelaunay, triangle.c:4570-4573, #ifdef _DEBUG) is compiled
//!   out of the pinned oracle build and is not transliterated; the
//!   setvertextype/undeads accounting that follows it is.
//! - The three "infinite" bounding box vertices (boundingbox) are C
//!   trimalloc allocations outside the vertex pool; the port mirror is
//!   the reserved out-of-pool INFVERTEX ids plus the owned store, per the
//!   new deviation D12 (slots.rs, mesh.rs::infvertex_store).
//! - The sortarray of divconqdelaunay is C trimalloc/trifree scratch
//!   memory; the port's Vec dissolves the manual pair (D8/D11), its
//!   reservation failure routes to the pool OOM bridge (`Oom`).
//! - Mesh::undeads and the bounds Mesh::xmin..ymax arrive with this PR as
//!   landed fields (undeads zeroed by triangleinit; the bounds are written
//!   by the transfernodes row, ioarrays.rs, which lands later). The
//!   vertextraverse row of mesh.rs lands with this PR too: its C consumer
//!   divconqdelaunay is transliterated here.
//! - The hyperbolacount/circletopcount counters ride part 2
//!   (rightofhyperbola/circletop) and are not added here.
//!
//! Stage-1 note: the consumers are the part-2 dispatcher rows and, via
//! them, the api.rs entry points; none has landed yet, so the blanket
//! below silences the ahead-of-use dead_code and is removed in part 2
//! with the delaunay() dispatcher.
#![allow(dead_code)]

use crate::handles::{encode, Otri, DUMMY_TRI, NULL};
use crate::mesh::Mesh;
use crate::options::Behavior;
use crate::pool::Oom;
use crate::predicates::{tri_counterclockwise, tri_incircle};
use crate::slots::{INFVERTEX1, INFVERTEX2, INFVERTEX3, UNDEADVERTEX};
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
}
