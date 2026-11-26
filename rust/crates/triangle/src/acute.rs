//! The aCute refinement (feature acute): acute.c, the aCute fork's
//! Steiner-point machinery, transliterated 1:1 (assignment:
//! docs/function-mapping.md, the `acute.c` section). The module sits
//! behind cfg(feature = "acute"), the exact inverse of C's NO_ACUTE
//! compile switch, so the crate's default build mirrors the fork's
//! default C build with aCute compiled in.
//!
//! This module lands in three PRs. This one (stage-1 acute A) carries
//! only the pure geometry substrate: the ten acute.c functions whose
//! signatures carry no mesh, no behavior and no otri - nothing but REAL
//! coordinates, in C file order. The rest of the unit (the mesh-coupled
//! helpers, the acutepool of acutepool_init/resize/deinit, the
//! findNewSPLocation family, and the D16 lift of refine.rs's funnel
//! marker) lands in the following PRs. Until then every substrate item
//! is ahead-of-use and carries its site-scoped dead_code allow citing
//! the acute C caller.
//!
//! Module-level C state mirrored here: compConst (acute.c:81, the "for
//! comparing real numbers" epsilon); PI is C's triangle_config.h:159
//! macro, mirrored as std::f64::consts::PI exactly like options.rs and
//! quality.rs.

use std::f64::consts::PI; // C: #define PI (triangle_config.h:159)

/// C: `const double compConst = 1.0e-80;` (acute.c:81), directly under
/// the `// for comparing real numbers` comment (acute.c:80). The epsilon
/// of the aCute geometry: lineLineIntersection (acute.c:3005, 3009),
/// linePointLocation (acute.c:3261) and circleLineIntersection
/// (acute.c:3378, 3387) compare against it; splitConvexPolygon
/// (acute.c:3112) and lineLineSegmentIntersection (acute.c:3295) shadow
/// it with local tolerances, exactly as the same-named body-local
/// constants shadow this crate constant.
#[allow(dead_code)]
// C-parity: its readers (acute.c:3005, 3261, 3378)
// are the substrate functions, ahead-of-use until
// the mesh-coupled acute PRs.
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
/// 221 and 843), which lands with the later acute PRs; the 1:1 body is
/// ahead-of-use until then.
#[allow(dead_code)]
// C-parity: the call sites (acute.c:221, 843) belong
// to the findNewSPLocation family, later acute PRs.
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

/// C: lineLineIntersection (acute.c:2983-3022). Given four points
/// representing two lines, write the intersection into the out-array
/// p: p[0] is 1.0 when the lines cross in one point (p[1], p[2] then
/// carry the coordinates, acute.c:3016-3017) and 0.0 when the lines are
/// coincident (acute.c:3005-3006) or parallel (acute.c:3009-3010).
/// Referenced to http://local.wasp.uwa.edu.au/~pbourke/geometry/
/// (acute.c:2981).
///
/// The C call sites sit in the getWedgeIntersection family (first
/// acute.c:2393; more at acute.c:2696-2759), which lands with the later
/// acute PRs; the 1:1 body is ahead-of-use until then.
#[allow(dead_code)]
// C-parity: the call sites (acute.c:2393, 2696-2759)
// belong to the getWedgeIntersection family, later
// acute PRs.
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
/// 2428, 2437, 2811, 2832), which lands with the later acute PRs; the
/// 1:1 body is ahead-of-use until then.
#[allow(dead_code)]
// C-parity: the call sites (acute.c:2428, 2437, 2811,
// 2832) belong to the getWedgeIntersection family,
// later acute PRs.
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
/// lands with this PR but is itself ahead-of-use until the
/// getWedgeIntersection family lands.
#[allow(dead_code)]
// C-parity: the only call site (acute.c:3048) is
// halfPlaneIntersection, itself ahead-of-use until
// the mesh-coupled acute PRs.
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
/// 3171-3172), itself ahead-of-use until the mesh-coupled acute PRs.
#[allow(dead_code)]
// C-parity: the call sites (acute.c:3171-3172) sit in
// splitConvexPolygon, ahead-of-use until the
// mesh-coupled acute PRs.
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
/// 3118), itself ahead-of-use until the mesh-coupled acute PRs.
#[allow(dead_code)]
// C-parity: the only call site (acute.c:3118) sits in
// splitConvexPolygon, ahead-of-use until the
// mesh-coupled acute PRs.
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
/// 2448 and 2844), which lands with the later acute PRs; the 1:1 body is
/// ahead-of-use until then.
#[allow(dead_code)]
// C-parity: the call sites (acute.c:2448, 2844)
// belong to the getWedgeIntersection family, later
// acute PRs.
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
/// 590, 1141, 1408), which lands with the later acute PRs; the 1:1 body
/// is ahead-of-use until then.
#[allow(dead_code)]
// C-parity: the call sites (acute.c:478, 590, 1141,
// 1408) belong to the findNewSPLocation family,
// later acute PRs.
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
/// 599, 1109, 1144, 1413), which lands with the later acute PRs; the 1:1
/// body is ahead-of-use until then.
#[allow(dead_code)]
// C-parity: the call sites (acute.c:485, 599, 1109,
// 1144, 1413) belong to the findNewSPLocation
// family, later acute PRs.
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
/// 612, 1181, 1191, 1269, 1449, 1458, 1531), which lands with the later
/// acute PRs; the 1:1 body is ahead-of-use until then.
#[allow(dead_code)]
// C-parity: the call sites (acute.c:497, 612, 1181,
// 1191, 1269, 1449, 1458, 1531) belong to the
// findNewSPLocation family, later acute PRs.
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
}
