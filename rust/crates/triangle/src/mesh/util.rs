//! Triangle utility functions (C: interpolate (triangle.c:238), randomnation
//! (triangle.c:1521)). Assignment: the interpolate and randomnation rows of
//! docs/function-mapping.md. The C file-scope marker randomseed
//! (triangle.c:220, a TRI_THREAD_LOCAL static) is re-anchored here as the
//! owned per-run field Mesh::randomseed (deviation D10: seeded 1 by
//! triangleinit, triangle.c:1502), so a run's determinism is owned by its
//! mesh and two independent meshes never share a sequence.
//!
//! Stage-1 note: consumers of these functions are algorithm/algorithm/locate.rs and
//! algorithm/algorithm/transform.rs (this PR) and later algorithm/algorithm/delaunay.rs/algorithm/segments.rs/algorithm/algorithm/refine.rs; the
//! whole current call graph sits below not-yet-landed modules, so the
//! blanket below silences the ahead-of-use dead_code and is removed as the
//! remaining consumers land.
#![allow(dead_code)]

use crate::mesh::Mesh;

/// The vertex as the predicates see it (C: a REAL* vertex). The port copies
/// the two coordinates through the chokepoint; index 2 carries the constant
/// third component the weighted predicates read (0.0 here; the raw pointer
/// beyond the coordinates never exists under D1).
#[inline]
pub(crate) fn vertex_coords(m: &Mesh, v: u32) -> [f64; 3] {
    [m.vx_x(v), m.vx_y(v), 0.0]
}

/// C: interpolate (triangle.c:238-263). Interpolate the attributes of
/// 'newvertex' in the triangle given by 'org', 'dest' and 'apex'. The vertex
/// is inserted in a triangle, so its attributes are interpolated from the
/// corners. C comments preserved below.
///
/// The C address arithmetic (newvertex[i], org[i], ...) mirrors to vertex
/// row indexes through the chokepoint (vx_real/vx_set_real); identical
/// arithmetic, identical order.
pub(crate) fn interpolate(
    m: &mut Mesh,
    newvertex: u32,
    org: u32,
    dest: u32,
    apex: u32,
    nextras: i32,
) {
    let xdo = m.vx_x(dest) - m.vx_x(org);
    let ydo = m.vx_y(dest) - m.vx_y(org);
    let xao = m.vx_x(apex) - m.vx_x(org);
    let yao = m.vx_y(apex) - m.vx_y(org);

    let denominator = 0.5 / (xdo * yao - xao * ydo);

    let dx = m.vx_x(newvertex) - m.vx_x(org);
    let dy = m.vx_y(newvertex) - m.vx_y(org);

    // To interpolate vertex attributes for the new vertex, define a
    // coordinate system with a xi-axis directed from the triangle's
    // origin to its destination, and an eta-axis, directed from its
    // origin to its apex.
    let xi = (yao * dx - xao * dy) * (2.0 * denominator);
    let eta = (xdo * dy - ydo * dx) * (2.0 * denominator);

    let mut i = 2;
    while i < 2 + nextras {
        // Interpolate the vertex attributes.
        let v = m.vx_real(org, i as usize)
            + xi * (m.vx_real(dest, i as usize) - m.vx_real(org, i as usize))
            + eta * (m.vx_real(apex, i as usize) - m.vx_real(org, i as usize));
        m.vx_set_real(newvertex, i as usize, v);
        i += 1;
    }
}

/// C: randomnation (triangle.c:1521-1525). Generate a random number between
/// 0 and 'choices' - 1. This is a simple linear congruential random number
/// generator. Hence, it is a bad random number generator, but good enough
/// for most geometric algorithms. D10: the LCG state is the per-run owned
/// field Mesh::randomseed (C: the thread-local static randomseed, seeded 1
/// per triangulation run by triangleinit, triangle.c:1502), never
/// thread-local: two runs own their own sequences.
pub(crate) fn random_below(m: &mut Mesh, choices: u32) -> u64 {
    m.randomseed = (m.randomseed * 1366 + 150889) % 714025;
    m.randomseed / (714025 / (choices as u64) + 1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::options::Behavior;

    /// nextras 1: vertex rows are coords (words 0, 1), one attribute word
    /// (word 2), then the mark/vtype overlay word.
    fn mesh(poly: bool) -> Mesh {
        Mesh::new(
            &Behavior {
                poly,
                usesegments: poly,
                ..Behavior::default_form()
            },
            16,
            2,
            1,
            0,
        )
        .expect("mesh init")
    }

    fn set_xy(m: &mut Mesh, v: u32, x: f64, y: f64) {
        m.set_vx_x(v, x);
        m.set_vx_y(v, y);
    }

    #[test]
    fn random_below_is_the_c_lcg_seeded_one() {
        // triangleinit seeds 1 and the LCG runs the exact C constants.
        let mut m = mesh(false);
        assert_eq!(m.randomseed, 1);
        // first draw: seed = (1 * 1366 + 150889) % 714025 = 152255
        let r = random_below(&mut m, 714025);
        assert_eq!(m.randomseed, 152255);
        // return is seed / (714025 / choices + 1); choices == 714025 gives
        // the raw seed halved (152255 is odd, so 76127)
        assert_eq!(r, 152255 / 2);
        // the classic caller shape: choices = triangle population; the LCG
        // advances first: seed = (152255 * 1366 + 150889) % 714025 = 349944,
        // return = 349944 / (714025 / 6 + 1) = 2
        assert_eq!(random_below(&mut m, 6), 349944 / (714025 / 6 + 1));
        assert_eq!(m.randomseed, 349944);
    }

    #[test]
    fn random_below_same_seed_same_sequence_distinct_per_run() {
        // D10: determinism is owned by the run, and two runs are
        // independent.
        let mut a = mesh(false);
        let mut b = mesh(false);
        let mut seq_a = vec![];
        let mut seq_b = vec![];
        for _ in 0..8 {
            // interleaved draws must not influence each other
            seq_a.push(random_below(&mut a, 4092));
            seq_b.push(random_below(&mut b, 4092));
        }
        // the same seed produces the same sequence on both clients...
        assert_eq!(seq_a, seq_b);
        // ...and a fresh client replays the same sequence after a has
        // already exhausted its own
        let mut c = mesh(false);
        let seq_c: Vec<u64> = (0..8).map(|_| random_below(&mut c, 4092)).collect();
        assert_eq!(seq_a, seq_c);
        // advancing one run leaves the other's next draw exactly where the
        // shared sequence left it (no cross-talk anywhere in between)
        let next_a = random_below(&mut a, 4092);
        let next_b = random_below(&mut b, 4092);
        assert_eq!(next_a, next_b);
    }

    #[test]
    fn interpolate_interpolates_attributes_from_the_corners() {
        let mut m = mesh(false);
        let org = m.vx_alloc().expect("alloc");
        let dest = m.vx_alloc().expect("alloc");
        let apex = m.vx_alloc().expect("alloc");
        let newvertex = m.vx_alloc().expect("alloc");
        set_xy(&mut m, org, 0.0, 0.0);
        set_xy(&mut m, dest, 1.0, 0.0);
        set_xy(&mut m, apex, 0.0, 1.0);
        m.vx_set_real(org, 2, 1.0);
        m.vx_set_real(dest, 2, 3.0);
        m.vx_set_real(apex, 2, 2.0);
        // a point with xi = 0.5, eta = 0.25 in the corner frame
        set_xy(&mut m, newvertex, 0.5, 0.25);
        interpolate(&mut m, newvertex, org, dest, apex, 1);
        // 1.0 + 0.5 * (3.0 - 1.0) + 0.25 * (2.0 - 1.0) = 2.25
        assert_eq!(m.vx_real(newvertex, 2), 2.25);
        // the coordinates themselves are untouched
        assert_eq!(m.vx_x(newvertex), 0.5);
        assert_eq!(m.vx_y(newvertex), 0.25);
        // multiple attributes all interpolate
        let mut m2 = Mesh::new(
            &Behavior {
                ..Behavior::default_form()
            },
            16,
            2,
            2,
            0,
        )
        .expect("mesh init");
        let o = m2.vx_alloc().expect("alloc");
        let d = m2.vx_alloc().expect("alloc");
        let a = m2.vx_alloc().expect("alloc");
        let nv = m2.vx_alloc().expect("alloc");
        set_xy(&mut m2, o, 0.0, 0.0);
        set_xy(&mut m2, d, 1.0, 0.0);
        set_xy(&mut m2, a, 0.0, 1.0);
        m2.vx_set_real(o, 2, 1.0);
        m2.vx_set_real(d, 2, 3.0);
        m2.vx_set_real(a, 2, 2.0);
        m2.vx_set_real(o, 3, 7.0);
        m2.vx_set_real(d, 3, -1.0);
        m2.vx_set_real(a, 3, 0.5);
        set_xy(&mut m2, nv, 0.25, 0.25);
        interpolate(&mut m2, nv, o, d, a, 2);
        assert_eq!(m2.vx_real(nv, 2), 1.0 + 0.25 * 2.0 + 0.25 * 1.0);
        assert_eq!(m2.vx_real(nv, 3), 7.0 + 0.25 * (-8.0) + 0.25 * (-6.5));
    }
}
