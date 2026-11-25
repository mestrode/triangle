//! Point location: preciselocate and locate (triangle.c:2192-2553, the
//! "Point location routines" section). Assignment: the preciselocate and
//! locate rows of docs/function-mapping.md. The C vertex pointers of the
//! routine bodies (forg/fdest/fapex, torg/tdest) mirror to vertex row ids
//! (D1); coordinates are copied through the chokepoint exactly where C
//! dereferences. The C block-walk of the random sampling (firstblock
//! pointers, alignptr arithmetic, triangle.c:2489-2499) mirrors to
//! WordPool::block_base_id: first item of block 0 = the first fresh row id,
//! plus the sample index (D4: ids replace the raw addresses; identical
//! arithmetic).
//!
//! Stage-1 note: consumers of these functions are transform.rs (this PR:
//! insertvertex) and later construct.rs/refine.rs; the current call graph
//! sits below not-yet-landed modules, so the blanket below silences the
//! ahead-of-use dead_code and is removed as the remaining consumers land.
#![allow(dead_code)]

use crate::handles::{Otri, DUMMY_SUB, DUMMY_TRI};
use crate::mesh::Mesh;
use crate::options::Behavior;
use crate::predicates::tri_counterclockwise;
use crate::util::{randomnation, vertex_coords};

/// C: TRIPERBLOCK (triangle_config.h:120): number of triangles allocated at
/// once. locate()'s sampling schedule reads the constant exactly like C's
/// macro (the triangle pool is configured with the same number).
const TRIPERBLOCK: i64 = 4092;
/// C: SAMPLEFACTOR (triangle_config.h:149): m->samples grows while
/// SAMPLEFACTOR * samples^3 < triangles.
const SAMPLEFACTOR: i64 = 11;

/// C: enum locateresult (triangle.h:97): INTRIANGLE, ONEDGE, ONVERTEX,
/// OUTSIDE - the exact C discriminants (pinned by test below). Owned by this
/// module per the mapping's enumeration rule: it is the return type of the
/// functions that live here.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(i32)]
pub(crate) enum LocateResult {
    /// C INTRIANGLE.
    InTriangle = 0,
    /// C ONEDGE.
    OnEdge = 1,
    /// C ONVERTEX.
    OnVertex = 2,
    /// C OUTSIDE.
    Outside = 3,
}

/// C: preciselocate (triangle.c:2291-2384). Find a triangle or edge
/// containing a given point. Begins its search from 'searchtri'. The point
/// must be strictly to the left of the edge denoted by 'searchtri', or
/// collinear with that edge and not intersecting it. On completion,
/// 'searchtri' is a triangle that contains 'searchpoint'. Returns ONVERTEX
/// if the point lies on an existing vertex ('searchtri' then has that
/// vertex as origin), ONEDGE if the point lies on a mesh edge ('searchtri'
/// then has that edge as primary edge), INTRIANGLE if the point lies
/// strictly inside a triangle, OUTSIDE if the point lies outside the mesh
/// ('searchtri' then holds an edge of the convex hull that the point is
/// to the right of). If 'stopatsubsegment' is set, the search stops
/// (returning OUTSIDE) if it tries to walk through a subsegment. WARNING:
/// this routine is designed for convex triangulations and will not
/// generally work after the holes and concavities have been carved (C
/// comment preserved).
pub(crate) fn preciselocate(
    m: &mut Mesh,
    b: &Behavior,
    searchpoint: (f64, f64),
    searchtri: &mut Otri,
    stopatsubsegment: bool,
) -> LocateResult {
    let mut count: i64 = 0;
    // The search point as the predicates see it (C: the vertex pointer).
    let sp = [searchpoint.0, searchpoint.1, 0.0];
    // Where are we?
    let mut forg = m.org(*searchtri);
    let mut fdest = m.dest(*searchtri);
    let mut fapex = m.apex(*searchtri);
    loop {
        // Check whether the apex is the point we seek.
        if m.vx_real(fapex, 0) == searchpoint.0 && m.vx_real(fapex, 1) == searchpoint.1 {
            searchtri.lprevself();
            return LocateResult::OnVertex;
        }
        // Does the point lie on the other side of the line defined by the
        //   triangle edge opposite the triangle's destination?
        let destorient =
            tri_counterclockwise(m, b, vertex_coords(m, forg), vertex_coords(m, fapex), sp);
        // Does the point lie on the other side of the line defined by the
        //   triangle edge opposite the triangle's origin?
        let orgorient =
            tri_counterclockwise(m, b, vertex_coords(m, fapex), vertex_coords(m, fdest), sp);
        let moveleft;
        if destorient > 0.0 {
            if orgorient > 0.0 {
                // Move left if the inner product of (fapex - searchpoint) and
                //   (fdest - forg) is positive.  This is equivalent to drawing
                //   a line perpendicular to the line (forg, fdest) and passing
                //   through 'fapex', and determining which side of this line
                //   'searchpoint' falls on.
                moveleft = (m.vx_real(fapex, 0) - searchpoint.0)
                    * (m.vx_real(fdest, 0) - m.vx_real(forg, 0))
                    + (m.vx_real(fapex, 1) - searchpoint.1)
                        * (m.vx_real(fdest, 1) - m.vx_real(forg, 1))
                    > 0.0;
            } else {
                moveleft = true;
            }
        } else if orgorient > 0.0 {
            moveleft = false;
        } else {
            // The point we seek must be on the boundary of or inside this
            //   triangle.
            if destorient == 0.0 {
                searchtri.lprevself();
                return LocateResult::OnEdge;
            }
            if orgorient == 0.0 {
                searchtri.lnextself();
                return LocateResult::OnEdge;
            }
            return LocateResult::InTriangle;
        }

        // Move to another triangle.  Leave a trace 'backtracktri' in case
        //   floating-point roundoff or some such bogey causes us to walk
        //   off a boundary of the triangulation.
        let backtracktri = if moveleft {
            fdest = fapex;
            searchtri.lprev()
        } else {
            forg = fapex;
            searchtri.lnext()
        };
        // sym(backtracktri, *searchtri) (triangle.c:2379-2384): the crossing
        //   is taken on the rotated edge the walk leaves through, not on
        //   the incoming handle's primary edge.
        *searchtri = backtracktri;
        m.symself(searchtri);

        if m.checksegments && stopatsubsegment {
            // Check for walking through a subsegment.
            let checkedge = m.tspivot(backtracktri);
            if checkedge.ss != DUMMY_SUB {
                // Go back to the last triangle.
                *searchtri = backtracktri;
                return LocateResult::Outside;
            }
        }
        // Check for walking right out of the triangulation.
        if searchtri.tri == DUMMY_TRI {
            // Go back to the last triangle.
            *searchtri = backtracktri;
            return LocateResult::Outside;
        }

        // Each step of the walk moves to a different triangle, so a walk that
        //   has taken more steps than there are triangles is going in circles
        //   Anything short of that is a legitimate walk across a large mesh.
        if count > m.triangles.live_count() as i64 {
            return LocateResult::Outside;
        }

        fapex = m.apex(*searchtri);
        count += 1;
    }
}

/// C: locate (triangle.c:2427-2541). Find a triangle or edge containing a
/// given point. Searching begins from one of: the input 'searchtri', a
/// recently encountered triangle 'recenttri', or from a triangle chosen
/// from a random sample; the choice is made by determining which triangle's
/// origin is closest to the point we are searching for. Normally,
/// 'searchtri' should be a handle on the convex hull of the triangulation.
/// Returns exactly like preciselocate (C comments preserved in the body).
pub(crate) fn locate(
    m: &mut Mesh,
    b: &Behavior,
    searchpoint: (f64, f64),
    searchtri: &mut Otri,
) -> LocateResult {
    let sp = [searchpoint.0, searchpoint.1, 0.0];

    // Record the distance from the suggested starting triangle to the
    //   point we seek.
    let mut torg = m.org(*searchtri);
    let mut searchdist = (searchpoint.0 - m.vx_real(torg, 0))
        * (searchpoint.0 - m.vx_real(torg, 0))
        + (searchpoint.1 - m.vx_real(torg, 1)) * (searchpoint.1 - m.vx_real(torg, 1));

    // If a recently encountered triangle has been recorded and has not been
    //   deallocated, test it as a good starting point.
    if let Some(recenttri) = m.recenttri {
        if !m.deadtri(recenttri.tri) {
            torg = m.org(recenttri);
            if m.vx_real(torg, 0) == searchpoint.0 && m.vx_real(torg, 1) == searchpoint.1 {
                *searchtri = recenttri;
                return LocateResult::OnVertex;
            }
            let dist = (searchpoint.0 - m.vx_real(torg, 0)) * (searchpoint.0 - m.vx_real(torg, 0))
                + (searchpoint.1 - m.vx_real(torg, 1)) * (searchpoint.1 - m.vx_real(torg, 1));
            if dist < searchdist {
                *searchtri = recenttri;
                searchdist = dist;
            }
        }
    }

    // The number of random samples taken is proportional to the cube root of
    //   the number of triangles in the mesh.  The next bit of code assumes
    //   that the number of triangles increases monotonically (or at least
    //   doesn't decrease enough to matter).
    while SAMPLEFACTOR * m.samples * m.samples * m.samples < m.triangles.live_count() as i64 {
        m.samples += 1;
    }

    // We'll draw ceiling(samples * TRIPERBLOCK / maxitems) random samples
    //   from each block of triangles (except the first)--until we meet the
    //   sample quota.  The ceiling means that blocks at the end might be
    //   neglected, but I don't care.
    let samplesperblock = (m.samples * TRIPERBLOCK - 1) / m.triangles.high_water() as i64 + 1;
    // We'll draw ceiling(samples * itemsfirstblock / maxitems) random samples
    //   from the first block of triangles.
    let mut samplesleft = (m.samples * m.triangles.first_block_items() as i64 - 1)
        / m.triangles.high_water() as i64
        + 1;
    let mut totalsamplesleft = m.samples;
    let mut population = m.triangles.first_block_items() as i64;
    let mut totalpopulation = m.triangles.high_water() as i64;
    // The block walk (C: sampleblock = (VOID **) *sampleblock); block 0 is
    // the first block, its first item the first fresh row (D4).
    let mut block: u64 = 0;
    while totalsamplesleft > 0 {
        // If we're in the last block, 'population' needs to be corrected.
        if population > totalpopulation {
            population = totalpopulation;
        }
        // Find a pointer to the first triangle in the block: the port's
        // block base id (D4; C: firsttri via alignptr arithmetic).
        let firsttri = m.triangles.block_base_id(block);
        // Choose 'samplesleft' randomly sampled triangles in this block.
        let mut sampletri = Otri {
            tri: firsttri,
            orient: 0,
        };
        loop {
            sampletri.tri = firsttri + randomnation(m, population as u32) as u32;
            if !m.deadtri(sampletri.tri) {
                torg = m.org(sampletri);
                let dist = (searchpoint.0 - m.vx_real(torg, 0))
                    * (searchpoint.0 - m.vx_real(torg, 0))
                    + (searchpoint.1 - m.vx_real(torg, 1)) * (searchpoint.1 - m.vx_real(torg, 1));
                if dist < searchdist {
                    *searchtri = sampletri;
                    searchdist = dist;
                }
            }

            samplesleft -= 1;
            totalsamplesleft -= 1;
            if !(samplesleft > 0 && totalsamplesleft > 0) {
                break;
            }
        }

        if totalsamplesleft > 0 {
            // The walk to the next block (C: sampleblock = (VOID **)
            // *sampleblock).
            block += 1;
            samplesleft = samplesperblock;
            totalpopulation -= population;
            population = TRIPERBLOCK;
        }
    }

    // Where are we?
    torg = m.org(*searchtri);
    let tdest = m.dest(*searchtri);
    // Check the starting triangle's vertices.
    if m.vx_real(torg, 0) == searchpoint.0 && m.vx_real(torg, 1) == searchpoint.1 {
        return LocateResult::OnVertex;
    }
    if m.vx_real(tdest, 0) == searchpoint.0 && m.vx_real(tdest, 1) == searchpoint.1 {
        searchtri.lnextself();
        return LocateResult::OnVertex;
    }
    // Orient 'searchtri' to fit the preconditions of calling preciselocate().
    let ahead = tri_counterclockwise(m, b, vertex_coords(m, torg), vertex_coords(m, tdest), sp);
    if ahead < 0.0 {
        // Turn around so that 'searchpoint' is to the left of the
        //   edge specified by 'searchtri'.
        m.symself(searchtri);
    } else if ahead == 0.0 {
        // Check if 'searchpoint' is between 'torg' and 'tdest'.
        if (m.vx_real(torg, 0) < searchpoint.0) == (searchpoint.0 < m.vx_real(tdest, 0))
            && (m.vx_real(torg, 1) < searchpoint.1) == (searchpoint.1 < m.vx_real(tdest, 1))
        {
            return LocateResult::OnEdge;
        }
    }
    preciselocate(m, b, searchpoint, searchtri, false)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A fan of n triangles around the origin: center vertex (0, 0), outer
    /// vertices on the unit circle at angles 360/n * k. The center triangle
    /// fan is CCW; adjacent triangles are bonded, the rim edges face outer
    /// space.
    fn fan(n: i32) -> (Mesh, u32, Vec<u32>, Vec<Otri>) {
        let mut m = Mesh::new(&Behavior::default_form(), n + 1, 2, 0, 0).expect("mesh init");
        let center = m.vx_alloc().expect("alloc");
        m.vx_set_real(center, 0, 0.0);
        m.vx_set_real(center, 1, 0.0);
        let mut outer = vec![];
        for k in 0..n {
            let v = m.vx_alloc().expect("alloc");
            let ang = 2.0 * std::f64::consts::PI * (k as f64) / (n as f64);
            m.vx_set_real(v, 0, ang.cos());
            m.vx_set_real(v, 1, ang.sin());
            outer.push(v);
        }
        // triangles (org=center, dest=outer[k], apex=outer[k+1]), CCW
        let mut tris = vec![];
        for k in 0..n as usize {
            let t = m.maketriangle(false, false).expect("alloc");
            m.setorg(t, center);
            m.setdest(t, outer[k]);
            m.setapex(t, outer[(k + 1) % n as usize]);
            tris.push(t);
        }
        // bond the spoke edge (center, outer[k+1]): lprev edge of tri k to
        // the primary edge of tri k+1
        for k in 0..n as usize {
            let a = tris[k].lprev();
            let b = Otri {
                tri: tris[(k + 1) % n as usize].tri,
                orient: 0,
            };
            m.bond(a, b);
        }
        (m, center, outer, tris)
    }

    #[test]
    fn locateresult_discriminants_are_the_c_values() {
        assert_eq!(LocateResult::InTriangle as i32, 0); // C INTRIANGLE
        assert_eq!(LocateResult::OnEdge as i32, 1); // C ONEDGE
        assert_eq!(LocateResult::OnVertex as i32, 2); // C ONVERTEX
        assert_eq!(LocateResult::Outside as i32, 3); // C OUTSIDE
    }

    #[test]
    fn locate_finds_triangle_edge_vertex_and_outside_on_a_fan() {
        let (mut m, center, outer, tris) = fan(6);
        // start from the hull edge of triangle 0 (its lnext edge, the rim
        // edge outer[0] -> outer[1])
        let start = tris[0].lnext();
        // strictly inside triangle 0 (30 degrees, radius 0.5)
        let inside = (
            0.5 * (std::f64::consts::PI / 6.0).cos(),
            0.5 * (std::f64::consts::PI / 6.0).sin(),
        );
        let mut st = start;
        assert_eq!(
            locate(&mut m, &Behavior::default_form(), inside, &mut st),
            LocateResult::InTriangle
        );
        assert_eq!(st.tri, tris[0].tri);
        // exactly on an existing vertex (outer[2])
        let onv = (m.vx_real(outer[2], 0), m.vx_real(outer[2], 1));
        let mut st = start;
        assert_eq!(
            locate(&mut m, &Behavior::default_form(), onv, &mut st),
            LocateResult::OnVertex
        );
        assert_eq!(m.org(st), outer[2]);
        // on the spoke edge (center, outer[1]) shared by triangles 0 and 1
        let one = (
            0.5 * m.vx_real(center, 0) + 0.5 * m.vx_real(outer[1], 0),
            0.5 * m.vx_real(center, 1) + 0.5 * m.vx_real(outer[1], 1),
        );
        let mut st = start;
        assert_eq!(
            locate(&mut m, &Behavior::default_form(), one, &mut st),
            LocateResult::OnEdge
        );
        let (o, d) = (m.org(st), m.dest(st));
        assert!(o == outer[1] || d == outer[1]);
        assert!(o == center || d == center);
        // outside the hull: beyond the wedge of triangles 0/1, to the
        // left of the rim edge so the locate tail keeps the handle on the
        // mesh (walking out through a rim edge reports OUTSIDE)
        let mut st = start;
        assert_eq!(
            locate(&mut m, &Behavior::default_form(), (-0.25, 0.9), &mut st),
            LocateResult::Outside
        );
    }

    #[test]
    fn locate_recenttri_short_circuits_on_the_recorded_vertex() {
        let (mut m, _center, outer, tris) = fan(6);
        // no recent triangle yet: recenttri is None at init
        assert!(m.recenttri.is_none());
        // record the most recently encountered triangle: fan triangle 3 at
        // orient 1 has outer[3] as its origin
        let recent = Otri {
            tri: tris[3].tri,
            orient: 1,
        };
        m.recenttri = Some(recent);
        let onv = (m.vx_real(outer[3], 0), m.vx_real(outer[3], 1));
        // a deliberately bad starting handle must not matter: the recenttri
        // branch wins before any walk
        let mut st = tris[0].lnext();
        assert_eq!(
            locate(&mut m, &Behavior::default_form(), onv, &mut st),
            LocateResult::OnVertex
        );
        assert_eq!(st, recent);
        assert_eq!(m.org(st), outer[3]);
    }

    #[test]
    fn locate_samples_are_deterministic_per_run() {
        // samples stays 1 (11 * 1^3 = 11 >= 6 triangles), and the per-block
        // quota (one total sample) cuts the sampler after a single draw
        let (mut m, _center, _outer, _tris) = fan(6);
        assert_eq!(m.samples, 1);
        let seed_before = m.randomseed;
        let mut st = Otri { tri: 2, orient: 0 };
        locate(&mut m, &Behavior::default_form(), (0.0, 0.0), &mut st);
        // one sample drawn: the seed advanced exactly one LCG step
        let expected_seed = (seed_before * 1366 + 150889) % 714025;
        assert_eq!(m.randomseed, expected_seed);
        // the second run on a fresh mesh sees the same sequence: same seed,
        // same single step (D10: two runs own their own sequences)
        let (mut m2, _c2, _o2, _t2) = fan(6);
        let mut st2 = Otri { tri: 2, orient: 0 };
        locate(&mut m2, &Behavior::default_form(), (0.0, 0.0), &mut st2);
        assert_eq!(m2.randomseed, expected_seed);
    }

    #[test]
    fn preciselocate_walks_from_a_valid_start_only() {
        let (mut m, _center, _outer, tris) = fan(6);
        let b = Behavior::default_form();
        // start inside triangle 3 with the point inside triangle 4: the
        // walk must cross over, testing the moveleft/geometry paths
        let target = (
            0.4 * (2.0 * std::f64::consts::PI * (4.5 / 6.0)).cos(),
            0.4 * (2.0 * std::f64::consts::PI * (4.5 / 6.0)).sin(),
        );
        let mut st = tris[3];
        // orient the start handle so the point is strictly left of its edge
        // (the degenerate hull precondition case is locate()'s job)
        assert_eq!(
            preciselocate(&mut m, &b, target, &mut st, false),
            LocateResult::InTriangle
        );
        assert_eq!(st.tri, tris[4].tri);
        // walking out through the hull: OUTSIDE and the backtrack handle
        let mut st = tris[2];
        // point far outside the wedge of triangle 2, outside the hull
        assert_eq!(
            preciselocate(&mut m, &b, (0.0, -2.0), &mut st, false),
            LocateResult::Outside
        );
    }
}
