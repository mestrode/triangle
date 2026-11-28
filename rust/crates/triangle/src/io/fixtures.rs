//! The shared cfg(test) fixtures of the io tests: one copy of the
//! helpers the io test modules previously duplicated (io/mod.rs,
//! io/arrays.rs, io/eps.rs). The output-phase consolidation (issue
//! #72) collects them here so the marshal tests add no fourth copy;
//! the remaining arrays (input) and eps tests take theirs from here
//! too. Compiled only under cfg(test).

use crate::input::Point2;
use crate::mesh::handles::Otri;
use crate::mesh::Mesh;
use crate::options::{Behavior, Options};

/// An assembled Behavior from a closure over Options (the
/// option-gated writer shapes; Options::validate runs C's
/// behavior_update tail exactly like the entry points).
pub(crate) fn behavior_with(setup: impl FnOnce(&mut Options)) -> Behavior {
    let mut opts = Options::new();
    setup(&mut opts);
    opts.validate().expect("validate")
}

/// A fresh core mesh for a load of `numberofpoints` points with
/// `numberofpointattribs` attributes: the pool geometry Mesh::new
/// pins, pre-fed the same input counters the api.rs construct path
/// feeds it.
pub(crate) fn core_mesh(b: &Behavior, numberofpoints: i32, numberofpointattribs: i32) -> Mesh {
    Mesh::new(b, numberofpoints, 2, numberofpointattribs, 0).expect("mesh init")
}

/// A three-point load off the axes (every bound extreme is a
/// distinct fixture point), one attribute per point, explicit
/// markers.
pub(crate) fn three_points() -> ([Point2; 3], [f64; 3], [i32; 3]) {
    (
        [
            Point2 { x: 2.0, y: 3.0 },
            Point2 { x: -1.0, y: 0.5 },
            Point2 { x: 4.0, y: -2.0 },
        ],
        [7.0, 8.0, 9.0],
        [5, 0, -3],
    )
}

/// The two-triangle unit square of the algorithm/algorithm/refine.rs highorder
/// fixture: t1 (a, e, c) bonded to t2 (a, c, d) across t1's
/// orient-2 edge (the diagonal a-c); the vertices are the pool
/// rows 1..=4 in traversal order, the triangles the rows 2 and 3
/// (both returned by row id).
#[allow(clippy::type_complexity)]
pub(crate) fn square_two_triangles(b: &Behavior, eextras: i32) -> (Mesh, u32, u32, [u32; 4]) {
    let mut m = Mesh::new(b, 8, 2, 0, eextras).expect("mesh init");
    let a = m.vx_alloc().expect("vx");
    m.set_vx_x(a, 0.0);
    m.set_vx_y(a, 0.0);
    let e = m.vx_alloc().expect("vx");
    m.set_vx_x(e, 1.0);
    m.set_vx_y(e, 0.0);
    let c = m.vx_alloc().expect("vx");
    m.set_vx_x(c, 1.0);
    m.set_vx_y(c, 1.0);
    let d = m.vx_alloc().expect("vx");
    m.set_vx_x(d, 0.0);
    m.set_vx_y(d, 1.0);
    let t1 = m.make_triangle(b.usesegments, b.vararea).expect("tri");
    m.set_org(t1, a);
    m.set_dest(t1, e);
    m.set_apex(t1, c);
    let t2 = m.make_triangle(b.usesegments, b.vararea).expect("tri");
    m.set_org(t2, a);
    m.set_dest(t2, c);
    m.set_apex(t2, d);
    m.bond(
        Otri {
            tri: t1.tri,
            orient: 2,
        },
        Otri {
            tri: t2.tri,
            orient: 0,
        },
    );
    (m, t1.tri, t2.tri, [a, e, c, d])
}
