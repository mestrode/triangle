//! Shewchuk's adaptive-precision exact predicates, transliterated 1:1 from
//! src/Triangle/predicates.c (the predicates.rs rows of
//! docs/function-mapping.md, tri_exactinit..tri_findcircumcenter,
//! predicates.c:173-1785).
//!
//! Deviations, per the function-mapping row notes and the register:
//! - D5: tri_exactinit's x87 (CPU86) and Linux _FPU_SETCW control-word
//!   blocks are dropped. The pinned 64-bit targets evaluate all REAL
//!   arithmetic in SSE2 double precision, which is exactly the state that
//!   code forced; on any other FPU the C code itself was the portability
//!   hazard.
//! - The C file-scope REAL globals (splitter, epsilon, resulterrbound, the
//!   ccw/icc/o3d errboundA-C families) become a OnceLock-guarded const
//!   struct: same values, computed by the same loop, no mutable globals
//!   (the idiomatic-Rust context rule of ADR 0004 section 6).
//! - C vertex pointers become by-value [f64; 3]: predicates only read
//!   coordinates, so by-value borrows nothing and loses nothing; index 2
//!   carries the lifting coordinate (-w parabolic lift, computed by
//!   tri_nonregular) or the user height (-W), mirroring C's pa[2] reads
//!   in the weighted tests.
//! - tri_findcircumcenter returns (x, y, xi, eta) instead of writing
//!   through out pointers (ownership deviation, same class as D7).
//! - The two-word arithmetic macros become #[inline] tuple functions with
//!   the exact C macro expansion order preserved (D6 precedent: the macro
//!   hidden locals disappear, the arithmetic stays statement-identical).
//! - The statistics counters are the four struct-mesh longs
//!   (triangle.h:538-546, i64 in the port), incremented here and zeroed
//!   by Mesh::new.
//!
//! The blanket below silences ahead-of-use dead_code until the Delaunay /
//! quality /construct modules that call these rows land; lift per
//! consumer.
#![allow(dead_code)]

use crate::mesh::Mesh;
use crate::options::Behavior;
use std::sync::OnceLock;

/// C: vertex as the predicates see it (a REAL*): passed by value here.
/// Index 0 = x, 1 = y, 2 = the third coordinate the weighted tests read
/// (weight for -w/-W, lifting/height input for tri_nonregular).
pub(crate) type Vertex = [f64; 3];

/// The C file-scope globals of predicates.c (splitter, epsilon and the
/// error-bound family), computed once by tri_exactinit.
#[derive(Debug)]
#[allow(non_snake_case)] // C-parity: the Shewchuk errbound constant names
struct PredConsts {
    /// Used to split REAL factors for exact multiplication (C splitter).
    splitter: f64,
    /// Floating-point machine epsilon (C epsilon).
    epsilon: f64,
    resulterrbound: f64,
    ccwerrboundA: f64,
    ccwerrboundB: f64,
    ccwerrboundC: f64,
    iccerrboundA: f64,
    iccerrboundB: f64,
    iccerrboundC: f64,
    o3derrboundA: f64,
    o3derrboundB: f64,
    o3derrboundC: f64,
}

static PRED_CONSTS: OnceLock<PredConsts> = OnceLock::new();

/// C: tri_exactinit (predicates.c:173). Idempotent; computes the constants
/// on first use. Called from Mesh::new (mirroring C triangleinit,
/// triangle.c:1504); the helpers below read the constants through the same
/// guard, so there is no call-order hazard.
///
/// D5: the x87/CPU86 and LINUX _FPU_SETCW control-word blocks are dropped;
/// every read of these constants on the pinned 64-bit targets is SSE2
/// double precision.
pub(crate) fn exact_init() {
    let _ = PRED_CONSTS.get_or_init(compute_consts);
}

fn consts() -> &'static PredConsts {
    PRED_CONSTS.get_or_init(compute_consts)
}

fn compute_consts() -> PredConsts {
    let half = 0.5;
    let mut every_other = true;
    let mut epsilon = 1.0;
    let mut splitter = 1.0;
    let mut check = 1.0;
    // The C do-loop: halve epsilon until 1 + epsilon rounds back to 1.0
    // (or stops changing), doubling splitter every other halving; ends at
    // epsilon = 2^-53 and splitter = 2^27 + 1 on IEEE doubles.
    loop {
        let lastcheck = check;
        epsilon *= half;
        if every_other {
            splitter *= 2.0;
        }
        every_other = !every_other;
        check = 1.0 + epsilon;
        if !((check != 1.0) && (check != lastcheck)) {
            break;
        }
    }
    splitter += 1.0;
    PredConsts {
        splitter,
        epsilon,
        // Error bounds for orientation and incircle tests.
        resulterrbound: (3.0 + 8.0 * epsilon) * epsilon,
        ccwerrboundA: (3.0 + 16.0 * epsilon) * epsilon,
        ccwerrboundB: (2.0 + 12.0 * epsilon) * epsilon,
        ccwerrboundC: (9.0 + 64.0 * epsilon) * epsilon * epsilon,
        iccerrboundA: (10.0 + 96.0 * epsilon) * epsilon,
        iccerrboundB: (4.0 + 48.0 * epsilon) * epsilon,
        iccerrboundC: (44.0 + 576.0 * epsilon) * epsilon * epsilon,
        o3derrboundA: (7.0 + 56.0 * epsilon) * epsilon,
        o3derrboundB: (3.0 + 28.0 * epsilon) * epsilon,
        o3derrboundC: (26.0 + 288.0 * epsilon) * epsilon * epsilon,
    }
}

// ---------------------------------------------------------------------------
// The two-word floating-point arithmetic macros of predicates.c as
// #[inline] tuple functions ((rounded result, exact error tail)). The
// bodies keep the C macro expansion order statement for statement.
// ---------------------------------------------------------------------------

/// C macro: Absolute (predicates.c:82).
#[inline]
fn absolute(a: f64) -> f64 {
    if a >= 0.0 {
        a
    } else {
        -a
    }
}

/// C macro: Fast_Two_Sum (valid when |a| >= |b|).
#[inline]
fn fast_two_sum(a: f64, b: f64) -> (f64, f64) {
    let x = a + b;
    // Fast_Two_Sum_Tail(a, b, x, y)
    let bvirt = x - a;
    (x, b - bvirt)
}

/// C macro: Two_Sum.
#[inline]
fn two_sum(a: f64, b: f64) -> (f64, f64) {
    let x = a + b;
    // Two_Sum_Tail(a, b, x, y)
    let bvirt = x - a;
    let avirt = x - bvirt;
    let bround = b - bvirt;
    let around = a - avirt;
    (x, around + bround)
}

/// C macro: Two_Diff.
#[inline]
fn two_diff(a: f64, b: f64) -> (f64, f64) {
    let x = a - b;
    let bvirt = a - x;
    let avirt = x + bvirt;
    let bround = bvirt - b;
    let around = a - avirt;
    (x, around + bround)
}

/// C macro: Two_Diff_Tail: the error tail of a subtraction whose rounded
/// value x = a - b has already been computed.
#[inline]
fn two_diff_tail(a: f64, b: f64, x: f64) -> f64 {
    let bvirt = a - x;
    let avirt = x + bvirt;
    let bround = bvirt - b;
    let around = a - avirt;
    around + bround
}

/// C macro: Split: (ahi, alo), the 27-bit high part and exact low
/// remainder of a, via the exactinit'd splitter.
#[inline]
fn split(a: f64) -> (f64, f64) {
    let c = consts().splitter * a;
    let abig = c - a;
    let ahi = c - abig;
    (ahi, a - ahi)
}

/// C macro: Two_Product (with Two_Product_Tail).
#[inline]
fn two_product(a: f64, b: f64) -> (f64, f64) {
    let x = a * b;
    let (ahi, alo) = split(a);
    let (bhi, blo) = split(b);
    let err1 = x - ahi * bhi;
    let err2 = err1 - alo * bhi;
    let err3 = err2 - ahi * blo;
    (x, alo * blo - err3)
}

/// C macro: Two_Product_Presplit: b already split into (bhi, blo).
#[inline]
fn two_product_presplit(a: f64, b: f64, bhi: f64, blo: f64) -> (f64, f64) {
    let x = a * b;
    let (ahi, alo) = split(a);
    let err1 = x - ahi * bhi;
    let err2 = err1 - alo * bhi;
    let err3 = err2 - ahi * blo;
    (x, alo * blo - err3)
}

/// C macro: Square (a * a with a single split).
#[inline]
fn square(a: f64) -> (f64, f64) {
    let x = a * a;
    let (ahi, alo) = split(a);
    let err1 = x - ahi * ahi;
    let err3 = err1 - (ahi + ahi) * alo;
    (x, alo * alo - err3)
}

/// C macro: Two_One_Sum -> (x2, x1, x0).
#[inline]
fn two_one_sum(a1: f64, a0: f64, b: f64) -> (f64, f64, f64) {
    let (i, x0) = two_sum(a0, b);
    let (x2, x1) = two_sum(a1, i);
    (x2, x1, x0)
}

/// C macro: Two_One_Diff -> (x2, x1, x0).
#[inline]
fn two_one_diff(a1: f64, a0: f64, b: f64) -> (f64, f64, f64) {
    let (i, x0) = two_diff(a0, b);
    let (x2, x1) = two_sum(a1, i);
    (x2, x1, x0)
}

/// C macro: Two_Two_Sum -> (x3, x2, x1, x0).
#[inline]
fn two_two_sum(a1: f64, a0: f64, b1: f64, b0: f64) -> (f64, f64, f64, f64) {
    let (j, z0, x0) = two_one_sum(a1, a0, b0);
    let (x3, x2, x1) = two_one_sum(j, z0, b1);
    (x3, x2, x1, x0)
}

/// C macro: Two_Two_Diff -> (x3, x2, x1, x0).
#[inline]
fn two_two_diff(a1: f64, a0: f64, b1: f64, b0: f64) -> (f64, f64, f64, f64) {
    let (j, z0, x0) = two_one_diff(a1, a0, b0);
    let (x3, x2, x1) = two_one_diff(j, z0, b1);
    (x3, x2, x1, x0)
}

/// C macro: Two_One_Product: the exact product of the two-word expansion
/// (a1, a0) by b, as (x3, x2, x1, x0).
#[inline]
fn two_one_product(a1: f64, a0: f64, b: f64) -> (f64, f64, f64, f64) {
    let (bhi, blo) = split(b);
    let (i, x0) = two_product_presplit(a0, b, bhi, blo);
    let (j, z0) = two_product_presplit(a1, b, bhi, blo);
    let (k, x1) = two_sum(i, z0);
    let (x3, x2) = fast_two_sum(j, k);
    (x3, x2, x1, x0)
}

// ---------------------------------------------------------------------------
// Expansion arithmetic (Shewchuk's Robust Predicates paper).
// ---------------------------------------------------------------------------

/// C: tri_fast_expansion_sum_zeroelim (predicates.c:246). Sets
/// h = e + f, eliminating zeros; h must have elen + flen slots.
///
/// The guarded reads (C reads one element past a source's used length and
/// discards the value; a slice cannot) replace C's unguarded prefetch
/// enow = e[++eindex]; the prefetched value is dead exactly when the guard
/// fails, so this is branch-identical to the compiled C.
fn fast_expansion_sum_zeroelim(
    elen: usize,
    e: &[f64],
    flen: usize,
    f: &[f64],
    h: &mut [f64],
) -> usize {
    debug_assert!(elen >= 1 && flen >= 1 && h.len() >= elen + flen);
    let mut q;
    let mut qnew;
    let mut hh;
    let mut eindex = 0usize;
    let mut findex = 0usize;
    let mut hindex = 0usize;
    let mut enow = e[0];
    let mut fnow = f[0];
    if (fnow > enow) == (fnow > -enow) {
        q = enow;
        eindex += 1;
        if eindex < elen {
            enow = e[eindex];
        }
    } else {
        q = fnow;
        findex += 1;
        if findex < flen {
            fnow = f[findex];
        }
    }
    if (eindex < elen) && (findex < flen) {
        if (fnow > enow) == (fnow > -enow) {
            (qnew, hh) = fast_two_sum(enow, q);
            eindex += 1;
            if eindex < elen {
                enow = e[eindex];
            }
        } else {
            (qnew, hh) = fast_two_sum(fnow, q);
            findex += 1;
            if findex < flen {
                fnow = f[findex];
            }
        }
        q = qnew;
        if hh != 0.0 {
            h[hindex] = hh;
            hindex += 1;
        }
        while (eindex < elen) && (findex < flen) {
            if (fnow > enow) == (fnow > -enow) {
                (qnew, hh) = two_sum(q, enow);
                eindex += 1;
                if eindex < elen {
                    enow = e[eindex];
                }
            } else {
                (qnew, hh) = two_sum(q, fnow);
                findex += 1;
                if findex < flen {
                    fnow = f[findex];
                }
            }
            q = qnew;
            if hh != 0.0 {
                h[hindex] = hh;
                hindex += 1;
            }
        }
    }
    while eindex < elen {
        (qnew, hh) = two_sum(q, enow);
        eindex += 1;
        if eindex < elen {
            enow = e[eindex];
        }
        q = qnew;
        if hh != 0.0 {
            h[hindex] = hh;
            hindex += 1;
        }
    }
    while findex < flen {
        (qnew, hh) = two_sum(q, fnow);
        findex += 1;
        if findex < flen {
            fnow = f[findex];
        }
        q = qnew;
        if hh != 0.0 {
            h[hindex] = hh;
            hindex += 1;
        }
    }
    if (q != 0.0) || (hindex == 0) {
        h[hindex] = q;
        hindex += 1;
    }
    hindex
}

/// C: tri_scale_expansion_zeroelim (predicates.c:330). Sets h = be with
/// zeros eliminated; h must have 2 * elen slots.
#[allow(clippy::needless_range_loop)] // C-parity cursor walk over e
fn scale_expansion_zeroelim(elen: usize, e: &[f64], b: f64, h: &mut [f64]) -> usize {
    debug_assert!(elen >= 1 && h.len() >= 2 * elen);
    let (bhi, blo) = split(b);
    let (mut q, hh) = two_product_presplit(e[0], b, bhi, blo);
    let mut hindex = 0usize;
    if hh != 0.0 {
        h[hindex] = hh;
        hindex += 1;
    }
    for eindex in 1..elen {
        let enow = e[eindex];
        let (product1, product0) = two_product_presplit(enow, b, bhi, blo);
        let (sum, hh) = two_sum(q, product0);
        if hh != 0.0 {
            h[hindex] = hh;
            hindex += 1;
        }
        // Fast_Two_Sum(product1, sum, Q, hh)
        let (qnew, hh2) = fast_two_sum(product1, sum);
        q = qnew;
        if hh2 != 0.0 {
            h[hindex] = hh2;
            hindex += 1;
        }
    }
    if (q != 0.0) || (hindex == 0) {
        h[hindex] = q;
        hindex += 1;
    }
    hindex
}

/// C: tri_estimate (predicates.c:377): a one-word estimate of an
/// expansion's value.
#[allow(clippy::needless_range_loop)] // C-parity cursor walk over e
fn estimate(elen: usize, e: &[f64]) -> f64 {
    let mut q = e[0];
    for eindex in 1..elen {
        q += e[eindex];
    }
    q
}

// ---------------------------------------------------------------------------
// counterclockwise
// ---------------------------------------------------------------------------

/// C: tri_counterclockwiseadapt (predicates.c:409).
fn counterclockwise_adapt(pa: Vertex, pb: Vertex, pc: Vertex, detsum: f64) -> f64 {
    let cst = consts();

    let acx = pa[0] - pc[0];
    let bcx = pb[0] - pc[0];
    let acy = pa[1] - pc[1];
    let bcy = pb[1] - pc[1];

    let (detleft, detlefttail) = two_product(acx, bcy);
    let (detright, detrighttail) = two_product(acy, bcx);

    let mut u = [0.0f64; 5];
    let mut b = [0.0f64; 5];
    let (b3, b2, b1, b0) = two_two_diff(detleft, detlefttail, detright, detrighttail);
    b[0] = b0;
    b[1] = b1;
    b[2] = b2;
    b[3] = b3;

    let mut det = estimate(4, &b);
    let mut errbound = cst.ccwerrboundB * detsum;
    if (det >= errbound) || (-det >= errbound) {
        return det;
    }

    let acxtail = two_diff_tail(pa[0], pc[0], acx);
    let bcxtail = two_diff_tail(pb[0], pc[0], bcx);
    let acytail = two_diff_tail(pa[1], pc[1], acy);
    let bcytail = two_diff_tail(pb[1], pc[1], bcy);

    if (acxtail == 0.0) && (acytail == 0.0) && (bcxtail == 0.0) && (bcytail == 0.0) {
        return det;
    }

    errbound = cst.ccwerrboundC * detsum + cst.resulterrbound * absolute(det);
    det += (acx * bcytail + bcy * acxtail) - (acy * bcxtail + bcx * acytail);
    if (det >= errbound) || (-det >= errbound) {
        return det;
    }

    let mut c1 = [0.0f64; 8];
    let mut c2 = [0.0f64; 12];
    let mut dd = [0.0f64; 16];

    let (s1, s0) = two_product(acxtail, bcy);
    let (t1, t0) = two_product(acytail, bcx);
    let (u3, u2, u1, u0) = two_two_diff(s1, s0, t1, t0);
    u[0] = u0;
    u[1] = u1;
    u[2] = u2;
    u[3] = u3;
    let c1length = fast_expansion_sum_zeroelim(4, &b, 4, &u, &mut c1);

    let (s1, s0) = two_product(acx, bcytail);
    let (t1, t0) = two_product(acy, bcxtail);
    let (u3, u2, u1, u0) = two_two_diff(s1, s0, t1, t0);
    u[0] = u0;
    u[1] = u1;
    u[2] = u2;
    u[3] = u3;
    let c2length = fast_expansion_sum_zeroelim(c1length, &c1, 4, &u, &mut c2);

    let (s1, s0) = two_product(acxtail, bcytail);
    let (t1, t0) = two_product(acytail, bcxtail);
    let (u3, u2, u1, u0) = two_two_diff(s1, s0, t1, t0);
    u[0] = u0;
    u[1] = u1;
    u[2] = u2;
    u[3] = u3;
    let dlength = fast_expansion_sum_zeroelim(c2length, &c2, 4, &u, &mut dd);

    dd[dlength - 1]
}

/// C: tri_counterclockwise (predicates.c:489). Positive if pa, pb, pc occur
/// in counterclockwise order; negative if clockwise; zero if collinear.
pub(crate) fn counterclockwise(
    m: &mut Mesh,
    b: &Behavior,
    pa: Vertex,
    pb: Vertex,
    pc: Vertex,
) -> f64 {
    m.counterclockcount += 1;

    let detleft = (pa[0] - pc[0]) * (pb[1] - pc[1]);
    let detright = (pa[1] - pc[1]) * (pb[0] - pc[0]);
    let det = detleft - detright;

    if b.noexact {
        return det;
    }

    let detsum;
    if detleft > 0.0 {
        if detright <= 0.0 {
            return det;
        } else {
            detsum = detleft + detright;
        }
    } else if detleft < 0.0 {
        if detright >= 0.0 {
            return det;
        } else {
            detsum = -detleft - detright;
        }
    } else {
        return det;
    }

    let errbound = consts().ccwerrboundA * detsum;
    if (det >= errbound) || (-det >= errbound) {
        return det;
    }

    counterclockwise_adapt(pa, pb, pc, detsum)
}

// ---------------------------------------------------------------------------
// incircle
// ---------------------------------------------------------------------------

/// C: tri_incircleadapt (predicates.c:548). The full adaptive cascade for
/// the incircle test. Expansion arrays keep the C sizes so every
/// intermediate fits exactly like the compiled C.
fn incircle_adapt(pa: Vertex, pb: Vertex, pc: Vertex, pd: Vertex, permanent: f64) -> f64 {
    let cst = consts();

    let adx = pa[0] - pd[0];
    let bdx = pb[0] - pd[0];
    let cdx = pc[0] - pd[0];
    let ady = pa[1] - pd[1];
    let bdy = pb[1] - pd[1];
    let cdy = pc[1] - pd[1];

    let mut bc = [0.0f64; 4];
    let mut ca = [0.0f64; 4];
    let mut ab = [0.0f64; 4];
    let mut aa = [0.0f64; 4];
    let mut bb = [0.0f64; 4];
    let mut cc = [0.0f64; 4];

    // finnow/finother buffer pair (C swaps REAL* pointers between fin1 and
    // fin2; Rust swaps a bool and picks the array at each site - the reads
    // and writes hit exactly the C arrays).
    let (bdxcdy1, bdxcdy0) = two_product(bdx, cdy);
    let (cdxbdy1, cdxbdy0) = two_product(cdx, bdy);
    let (bc3, bc2, bc1, bc0) = two_two_diff(bdxcdy1, bdxcdy0, cdxbdy1, cdxbdy0);
    bc[0] = bc0;
    bc[1] = bc1;
    bc[2] = bc2;
    bc[3] = bc3;
    let mut axbc = [0.0f64; 8];
    let axbclen = scale_expansion_zeroelim(4, &bc, adx, &mut axbc);
    let mut axxbc = [0.0f64; 16];
    let axxbclen = scale_expansion_zeroelim(axbclen, &axbc, adx, &mut axxbc);
    let mut aybc = [0.0f64; 8];
    let aybclen = scale_expansion_zeroelim(4, &bc, ady, &mut aybc);
    let mut ayybc = [0.0f64; 16];
    let ayybclen = scale_expansion_zeroelim(aybclen, &aybc, ady, &mut ayybc);
    let mut adet = [0.0f64; 32];
    let alen = fast_expansion_sum_zeroelim(axxbclen, &axxbc, ayybclen, &ayybc, &mut adet);

    let (cdxady1, cdxady0) = two_product(cdx, ady);
    let (adxcdy1, adxcdy0) = two_product(adx, cdy);
    let (ca3, ca2, ca1, ca0) = two_two_diff(cdxady1, cdxady0, adxcdy1, adxcdy0);
    ca[0] = ca0;
    ca[1] = ca1;
    ca[2] = ca2;
    ca[3] = ca3;
    let mut bxca = [0.0f64; 8];
    let bxcalen = scale_expansion_zeroelim(4, &ca, bdx, &mut bxca);
    let mut bxxca = [0.0f64; 16];
    let bxxcalen = scale_expansion_zeroelim(bxcalen, &bxca, bdx, &mut bxxca);
    let mut byca = [0.0f64; 8];
    let bycalen = scale_expansion_zeroelim(4, &ca, bdy, &mut byca);
    let mut byyca = [0.0f64; 16];
    let byycalen = scale_expansion_zeroelim(bycalen, &byca, bdy, &mut byyca);
    let mut bdet = [0.0f64; 32];
    let blen = fast_expansion_sum_zeroelim(bxxcalen, &bxxca, byycalen, &byyca, &mut bdet);

    let (adxbdy1, adxbdy0) = two_product(adx, bdy);
    let (bdxady1, bdxady0) = two_product(bdx, ady);
    let (ab3, ab2, ab1, ab0) = two_two_diff(adxbdy1, adxbdy0, bdxady1, bdxady0);
    ab[0] = ab0;
    ab[1] = ab1;
    ab[2] = ab2;
    ab[3] = ab3;
    let mut cxab = [0.0f64; 8];
    let cxablen = scale_expansion_zeroelim(4, &ab, cdx, &mut cxab);
    let mut cxxab = [0.0f64; 16];
    let cxxablen = scale_expansion_zeroelim(cxablen, &cxab, cdx, &mut cxxab);
    let mut cyab = [0.0f64; 8];
    let cyablen = scale_expansion_zeroelim(4, &ab, cdy, &mut cyab);
    let mut cyyab = [0.0f64; 16];
    let cyyablen = scale_expansion_zeroelim(cyablen, &cyab, cdy, &mut cyyab);
    let mut cdet = [0.0f64; 32];
    let clen = fast_expansion_sum_zeroelim(cxxablen, &cxxab, cyyablen, &cyyab, &mut cdet);

    // C predicates.c:565: REAL fin1[1152], fin2[1152] holding the finnow/
    // finother accumulation pair; the C pointer swap becomes these two
    // buffers plus the in_a flag that the finstep! macro below flips after
    // every accumulation step.
    let mut fin1 = [0.0f64; 1152];
    let mut fin2 = [0.0f64; 1152];
    let mut in_a = true;
    let mut abdet = [0.0f64; 64];
    let ablen = fast_expansion_sum_zeroelim(alen, &adet, blen, &bdet, &mut abdet);
    let mut finlength = fast_expansion_sum_zeroelim(ablen, &abdet, clen, &cdet, &mut fin1);

    macro_rules! finstep {
        ($fin1:ident, $fin2:ident, $in_a:ident, $flen:ident, $add:expr, $addlen:expr) => {
            if $in_a {
                $flen = fast_expansion_sum_zeroelim($flen, &$fin1, $addlen, $add, &mut $fin2);
                $in_a = false;
            } else {
                $flen = fast_expansion_sum_zeroelim($flen, &$fin2, $addlen, $add, &mut $fin1);
                $in_a = true;
            }
        };
    }

    let mut det = estimate(finlength, &fin1);
    let mut errbound = cst.iccerrboundB * permanent;
    if (det >= errbound) || (-det >= errbound) {
        return det;
    }

    let adxtail = two_diff_tail(pa[0], pd[0], adx);
    let adytail = two_diff_tail(pa[1], pd[1], ady);
    let bdxtail = two_diff_tail(pb[0], pd[0], bdx);
    let bdytail = two_diff_tail(pb[1], pd[1], bdy);
    let cdxtail = two_diff_tail(pc[0], pd[0], cdx);
    let cdytail = two_diff_tail(pc[1], pd[1], cdy);
    if (adxtail == 0.0)
        && (bdxtail == 0.0)
        && (cdxtail == 0.0)
        && (adytail == 0.0)
        && (bdytail == 0.0)
        && (cdytail == 0.0)
    {
        return det;
    }

    errbound = cst.iccerrboundC * permanent + cst.resulterrbound * absolute(det);
    det += ((adx * adx + ady * ady)
        * ((bdx * cdytail + cdy * bdxtail) - (bdy * cdxtail + cdx * bdytail))
        + 2.0 * (adx * adxtail + ady * adytail) * (bdx * cdy - bdy * cdx))
        + ((bdx * bdx + bdy * bdy)
            * ((cdx * adytail + ady * cdxtail) - (cdy * adxtail + adx * cdytail))
            + 2.0 * (bdx * bdxtail + bdy * bdytail) * (cdx * ady - cdy * adx))
        + ((cdx * cdx + cdy * cdy)
            * ((adx * bdytail + bdy * adxtail) - (ady * bdxtail + bdx * adytail))
            + 2.0 * (cdx * cdxtail + cdy * cdytail) * (adx * bdy - ady * bdx));
    if (det >= errbound) || (-det >= errbound) {
        return det;
    }

    if (bdxtail != 0.0) || (bdytail != 0.0) || (cdxtail != 0.0) || (cdytail != 0.0) {
        let (adxadx1, adxadx0) = square(adx);
        let (adyady1, adyady0) = square(ady);
        let (aa3, aa2, aa1, aa0) = two_two_sum(adxadx1, adxadx0, adyady1, adyady0);
        aa[0] = aa0;
        aa[1] = aa1;
        aa[2] = aa2;
        aa[3] = aa3;
    }
    if (cdxtail != 0.0) || (cdytail != 0.0) || (adxtail != 0.0) || (adytail != 0.0) {
        let (bdxbdx1, bdxbdx0) = square(bdx);
        let (bdybdy1, bdybdy0) = square(bdy);
        let (bb3, bb2, bb1, bb0) = two_two_sum(bdxbdx1, bdxbdx0, bdybdy1, bdybdy0);
        bb[0] = bb0;
        bb[1] = bb1;
        bb[2] = bb2;
        bb[3] = bb3;
    }
    if (adxtail != 0.0) || (adytail != 0.0) || (bdxtail != 0.0) || (bdytail != 0.0) {
        let (cdxcdx1, cdxcdx0) = square(cdx);
        let (cdycdy1, cdycdy0) = square(cdy);
        let (cc3, cc2, cc1, cc0) = two_two_sum(cdxcdx1, cdxcdx0, cdycdy1, cdycdy0);
        cc[0] = cc0;
        cc[1] = cc1;
        cc[2] = cc2;
        cc[3] = cc3;
    }

    // Cross-block temporaries (computed under a tail condition and reused
    // later under an implied one, exactly like the C locals).
    let mut temp8 = [0.0f64; 8];
    let mut temp16a = [0.0f64; 16];
    let mut temp16b = [0.0f64; 16];
    let mut temp16c = [0.0f64; 16];
    let mut temp32a = [0.0f64; 32];
    let mut temp32b = [0.0f64; 32];
    let mut temp48 = [0.0f64; 48];
    let mut temp64 = [0.0f64; 64];
    let mut axtbc = [0.0f64; 8];
    let mut axtbclen = 0usize;
    let mut aytbc = [0.0f64; 8];
    let mut aytbclen = 0usize;
    let mut bxtca = [0.0f64; 8];
    let mut bxtcalen = 0usize;
    let mut bytca = [0.0f64; 8];
    let mut bytcalen = 0usize;
    let mut cxtab = [0.0f64; 8];
    let mut cxtablen = 0usize;
    let mut cytab = [0.0f64; 8];
    let mut cytablen = 0usize;

    if adxtail != 0.0 {
        axtbclen = scale_expansion_zeroelim(4, &bc, adxtail, &mut axtbc);
        let temp16alen = scale_expansion_zeroelim(axtbclen, &axtbc, 2.0 * adx, &mut temp16a);

        let mut axtcc = [0.0f64; 8];
        let axtcclen = scale_expansion_zeroelim(4, &cc, adxtail, &mut axtcc);
        let temp16blen = scale_expansion_zeroelim(axtcclen, &axtcc, bdy, &mut temp16b);

        let mut axtbb = [0.0f64; 8];
        let axtbblen = scale_expansion_zeroelim(4, &bb, adxtail, &mut axtbb);
        let temp16clen = scale_expansion_zeroelim(axtbblen, &axtbb, -cdy, &mut temp16c);

        let temp32alen =
            fast_expansion_sum_zeroelim(temp16alen, &temp16a, temp16blen, &temp16b, &mut temp32a);
        let temp48len =
            fast_expansion_sum_zeroelim(temp16clen, &temp16c, temp32alen, &temp32a, &mut temp48);
        finstep!(fin1, fin2, in_a, finlength, &temp48, temp48len);
    }
    if adytail != 0.0 {
        aytbclen = scale_expansion_zeroelim(4, &bc, adytail, &mut aytbc);
        let temp16alen = scale_expansion_zeroelim(aytbclen, &aytbc, 2.0 * ady, &mut temp16a);

        let mut aytbb = [0.0f64; 8];
        let aytbblen = scale_expansion_zeroelim(4, &bb, adytail, &mut aytbb);
        let temp16blen = scale_expansion_zeroelim(aytbblen, &aytbb, cdx, &mut temp16b);

        let mut aytcc = [0.0f64; 8];
        let aytcclen = scale_expansion_zeroelim(4, &cc, adytail, &mut aytcc);
        let temp16clen = scale_expansion_zeroelim(aytcclen, &aytcc, -bdx, &mut temp16c);

        let temp32alen =
            fast_expansion_sum_zeroelim(temp16alen, &temp16a, temp16blen, &temp16b, &mut temp32a);
        let temp48len =
            fast_expansion_sum_zeroelim(temp16clen, &temp16c, temp32alen, &temp32a, &mut temp48);
        finstep!(fin1, fin2, in_a, finlength, &temp48, temp48len);
    }
    if bdxtail != 0.0 {
        bxtcalen = scale_expansion_zeroelim(4, &ca, bdxtail, &mut bxtca);
        let temp16alen = scale_expansion_zeroelim(bxtcalen, &bxtca, 2.0 * bdx, &mut temp16a);

        let mut bxtaa = [0.0f64; 8];
        let bxtaalen = scale_expansion_zeroelim(4, &aa, bdxtail, &mut bxtaa);
        let temp16blen = scale_expansion_zeroelim(bxtaalen, &bxtaa, cdy, &mut temp16b);

        let mut bxtcc = [0.0f64; 8];
        let bxtcclen = scale_expansion_zeroelim(4, &cc, bdxtail, &mut bxtcc);
        let temp16clen = scale_expansion_zeroelim(bxtcclen, &bxtcc, -ady, &mut temp16c);

        let temp32alen =
            fast_expansion_sum_zeroelim(temp16alen, &temp16a, temp16blen, &temp16b, &mut temp32a);
        let temp48len =
            fast_expansion_sum_zeroelim(temp16clen, &temp16c, temp32alen, &temp32a, &mut temp48);
        finstep!(fin1, fin2, in_a, finlength, &temp48, temp48len);
    }
    if bdytail != 0.0 {
        bytcalen = scale_expansion_zeroelim(4, &ca, bdytail, &mut bytca);
        let temp16alen = scale_expansion_zeroelim(bytcalen, &bytca, 2.0 * bdy, &mut temp16a);

        let mut bytcc = [0.0f64; 8];
        let bytcclen = scale_expansion_zeroelim(4, &cc, bdytail, &mut bytcc);
        let temp16blen = scale_expansion_zeroelim(bytcclen, &bytcc, adx, &mut temp16b);

        let mut bytaa = [0.0f64; 8];
        let bytaalen = scale_expansion_zeroelim(4, &aa, bdytail, &mut bytaa);
        let temp16clen = scale_expansion_zeroelim(bytaalen, &bytaa, -cdx, &mut temp16c);

        let temp32alen =
            fast_expansion_sum_zeroelim(temp16alen, &temp16a, temp16blen, &temp16b, &mut temp32a);
        let temp48len =
            fast_expansion_sum_zeroelim(temp16clen, &temp16c, temp32alen, &temp32a, &mut temp48);
        finstep!(fin1, fin2, in_a, finlength, &temp48, temp48len);
    }
    if cdxtail != 0.0 {
        cxtablen = scale_expansion_zeroelim(4, &ab, cdxtail, &mut cxtab);
        let temp16alen = scale_expansion_zeroelim(cxtablen, &cxtab, 2.0 * cdx, &mut temp16a);

        let mut cxtbb = [0.0f64; 8];
        let cxtbblen = scale_expansion_zeroelim(4, &bb, cdxtail, &mut cxtbb);
        let temp16blen = scale_expansion_zeroelim(cxtbblen, &cxtbb, ady, &mut temp16b);

        let mut cxtaa = [0.0f64; 8];
        let cxtaalen = scale_expansion_zeroelim(4, &aa, cdxtail, &mut cxtaa);
        let temp16clen = scale_expansion_zeroelim(cxtaalen, &cxtaa, -bdy, &mut temp16c);

        let temp32alen =
            fast_expansion_sum_zeroelim(temp16alen, &temp16a, temp16blen, &temp16b, &mut temp32a);
        let temp48len =
            fast_expansion_sum_zeroelim(temp16clen, &temp16c, temp32alen, &temp32a, &mut temp48);
        finstep!(fin1, fin2, in_a, finlength, &temp48, temp48len);
    }
    if cdytail != 0.0 {
        cytablen = scale_expansion_zeroelim(4, &ab, cdytail, &mut cytab);
        let temp16alen = scale_expansion_zeroelim(cytablen, &cytab, 2.0 * cdy, &mut temp16a);

        let mut cytaa = [0.0f64; 8];
        let cytaalen = scale_expansion_zeroelim(4, &aa, cdytail, &mut cytaa);
        let temp16blen = scale_expansion_zeroelim(cytaalen, &cytaa, bdx, &mut temp16b);

        let mut cytbb = [0.0f64; 8];
        let cytbblen = scale_expansion_zeroelim(4, &bb, cdytail, &mut cytbb);
        let temp16clen = scale_expansion_zeroelim(cytbblen, &cytbb, -adx, &mut temp16c);

        let temp32alen =
            fast_expansion_sum_zeroelim(temp16alen, &temp16a, temp16blen, &temp16b, &mut temp32a);
        let temp48len =
            fast_expansion_sum_zeroelim(temp16clen, &temp16c, temp32alen, &temp32a, &mut temp48);
        finstep!(fin1, fin2, in_a, finlength, &temp48, temp48len);
    }

    if (adxtail != 0.0) || (adytail != 0.0) {
        let mut bct = [0.0f64; 8];
        let mut bctt = [0.0f64; 4];
        let bctlen;
        #[allow(clippy::needless_late_init)]
        // C-parity: declared with the block, assigned per branch
        let bcttlen;
        if (bdxtail != 0.0) || (bdytail != 0.0) || (cdxtail != 0.0) || (cdytail != 0.0) {
            let mut u = [0.0f64; 5];
            let mut v = [0.0f64; 5];

            let (ti1, ti0) = two_product(bdxtail, cdy);
            let (tj1, tj0) = two_product(bdx, cdytail);
            let (u3, u2, u1, u0) = two_two_sum(ti1, ti0, tj1, tj0);
            u[0] = u0;
            u[1] = u1;
            u[2] = u2;
            u[3] = u3;
            let negate = -bdy;
            let (ti1, ti0) = two_product(cdxtail, negate);
            let negate = -bdytail;
            let (tj1, tj0) = two_product(cdx, negate);
            let (v3, v2, v1, v0) = two_two_sum(ti1, ti0, tj1, tj0);
            v[0] = v0;
            v[1] = v1;
            v[2] = v2;
            v[3] = v3;
            bctlen = fast_expansion_sum_zeroelim(4, &u, 4, &v, &mut bct);

            let (ti1, ti0) = two_product(bdxtail, cdytail);
            let (tj1, tj0) = two_product(cdxtail, bdytail);
            let (bctt3, bctt2, bctt1, bctt0) = two_two_diff(ti1, ti0, tj1, tj0);
            bctt[0] = bctt0;
            bctt[1] = bctt1;
            bctt[2] = bctt2;
            bctt[3] = bctt3;
            bcttlen = 4;
        } else {
            bct[0] = 0.0;
            bctlen = 1;
            bctt[0] = 0.0;
            bcttlen = 1;
        }

        if adxtail != 0.0 {
            let temp16alen = scale_expansion_zeroelim(axtbclen, &axtbc, adxtail, &mut temp16a);
            let mut axtbct = [0.0f64; 16];
            let axtbctlen = scale_expansion_zeroelim(bctlen, &bct, adxtail, &mut axtbct);
            let temp32alen = scale_expansion_zeroelim(axtbctlen, &axtbct, 2.0 * adx, &mut temp32a);
            let temp48len = fast_expansion_sum_zeroelim(
                temp16alen,
                &temp16a,
                temp32alen,
                &temp32a,
                &mut temp48,
            );
            finstep!(fin1, fin2, in_a, finlength, &temp48, temp48len);
            if bdytail != 0.0 {
                let temp8len = scale_expansion_zeroelim(4, &cc, adxtail, &mut temp8);
                let temp16alen = scale_expansion_zeroelim(temp8len, &temp8, bdytail, &mut temp16a);
                finstep!(fin1, fin2, in_a, finlength, &temp16a, temp16alen);
            }
            if cdytail != 0.0 {
                let temp8len = scale_expansion_zeroelim(4, &bb, -adxtail, &mut temp8);
                let temp16alen = scale_expansion_zeroelim(temp8len, &temp8, cdytail, &mut temp16a);
                finstep!(fin1, fin2, in_a, finlength, &temp16a, temp16alen);
            }

            let temp32alen = scale_expansion_zeroelim(axtbctlen, &axtbct, adxtail, &mut temp32a);
            let mut axtbctt = [0.0f64; 8];
            let axtbcttlen = scale_expansion_zeroelim(bcttlen, &bctt, adxtail, &mut axtbctt);
            let temp16alen =
                scale_expansion_zeroelim(axtbcttlen, &axtbctt, 2.0 * adx, &mut temp16a);
            let temp16blen = scale_expansion_zeroelim(axtbcttlen, &axtbctt, adxtail, &mut temp16b);
            let temp32blen = fast_expansion_sum_zeroelim(
                temp16alen,
                &temp16a,
                temp16blen,
                &temp16b,
                &mut temp32b,
            );
            let temp64len = fast_expansion_sum_zeroelim(
                temp32alen,
                &temp32a,
                temp32blen,
                &temp32b,
                &mut temp64,
            );
            finstep!(fin1, fin2, in_a, finlength, &temp64, temp64len);
        }
        if adytail != 0.0 {
            let temp16alen = scale_expansion_zeroelim(aytbclen, &aytbc, adytail, &mut temp16a);
            let mut aytbct = [0.0f64; 16];
            let aytbctlen = scale_expansion_zeroelim(bctlen, &bct, adytail, &mut aytbct);
            let temp32alen = scale_expansion_zeroelim(aytbctlen, &aytbct, 2.0 * ady, &mut temp32a);
            let temp48len = fast_expansion_sum_zeroelim(
                temp16alen,
                &temp16a,
                temp32alen,
                &temp32a,
                &mut temp48,
            );
            finstep!(fin1, fin2, in_a, finlength, &temp48, temp48len);

            let temp32alen = scale_expansion_zeroelim(aytbctlen, &aytbct, adytail, &mut temp32a);
            let mut aytbctt = [0.0f64; 8];
            let aytbcttlen = scale_expansion_zeroelim(bcttlen, &bctt, adytail, &mut aytbctt);
            let temp16alen =
                scale_expansion_zeroelim(aytbcttlen, &aytbctt, 2.0 * ady, &mut temp16a);
            let temp16blen = scale_expansion_zeroelim(aytbcttlen, &aytbctt, adytail, &mut temp16b);
            let temp32blen = fast_expansion_sum_zeroelim(
                temp16alen,
                &temp16a,
                temp16blen,
                &temp16b,
                &mut temp32b,
            );
            let temp64len = fast_expansion_sum_zeroelim(
                temp32alen,
                &temp32a,
                temp32blen,
                &temp32b,
                &mut temp64,
            );
            finstep!(fin1, fin2, in_a, finlength, &temp64, temp64len);
        }
    }
    if (bdxtail != 0.0) || (bdytail != 0.0) {
        let mut cat = [0.0f64; 8];
        let mut catt = [0.0f64; 4];
        let catlen;
        #[allow(clippy::needless_late_init)]
        // C-parity: declared with the block, assigned per branch
        let cattlen;
        if (cdxtail != 0.0) || (cdytail != 0.0) || (adxtail != 0.0) || (adytail != 0.0) {
            let mut u = [0.0f64; 5];
            let mut v = [0.0f64; 5];

            let (ti1, ti0) = two_product(cdxtail, ady);
            let (tj1, tj0) = two_product(cdx, adytail);
            let (u3, u2, u1, u0) = two_two_sum(ti1, ti0, tj1, tj0);
            u[0] = u0;
            u[1] = u1;
            u[2] = u2;
            u[3] = u3;
            let negate = -cdy;
            let (ti1, ti0) = two_product(adxtail, negate);
            let negate = -cdytail;
            let (tj1, tj0) = two_product(adx, negate);
            let (v3, v2, v1, v0) = two_two_sum(ti1, ti0, tj1, tj0);
            v[0] = v0;
            v[1] = v1;
            v[2] = v2;
            v[3] = v3;
            catlen = fast_expansion_sum_zeroelim(4, &u, 4, &v, &mut cat);

            let (ti1, ti0) = two_product(cdxtail, adytail);
            let (tj1, tj0) = two_product(adxtail, cdytail);
            let (catt3, catt2, catt1, catt0) = two_two_diff(ti1, ti0, tj1, tj0);
            catt[0] = catt0;
            catt[1] = catt1;
            catt[2] = catt2;
            catt[3] = catt3;
            cattlen = 4;
        } else {
            cat[0] = 0.0;
            catlen = 1;
            catt[0] = 0.0;
            cattlen = 1;
        }

        if bdxtail != 0.0 {
            let temp16alen = scale_expansion_zeroelim(bxtcalen, &bxtca, bdxtail, &mut temp16a);
            let mut bxtcat = [0.0f64; 16];
            let bxtcatlen = scale_expansion_zeroelim(catlen, &cat, bdxtail, &mut bxtcat);
            let temp32alen = scale_expansion_zeroelim(bxtcatlen, &bxtcat, 2.0 * bdx, &mut temp32a);
            let temp48len = fast_expansion_sum_zeroelim(
                temp16alen,
                &temp16a,
                temp32alen,
                &temp32a,
                &mut temp48,
            );
            finstep!(fin1, fin2, in_a, finlength, &temp48, temp48len);
            if cdytail != 0.0 {
                let temp8len = scale_expansion_zeroelim(4, &aa, bdxtail, &mut temp8);
                let temp16alen = scale_expansion_zeroelim(temp8len, &temp8, cdytail, &mut temp16a);
                finstep!(fin1, fin2, in_a, finlength, &temp16a, temp16alen);
            }
            if adytail != 0.0 {
                let temp8len = scale_expansion_zeroelim(4, &cc, -bdxtail, &mut temp8);
                let temp16alen = scale_expansion_zeroelim(temp8len, &temp8, adytail, &mut temp16a);
                finstep!(fin1, fin2, in_a, finlength, &temp16a, temp16alen);
            }

            let temp32alen = scale_expansion_zeroelim(bxtcatlen, &bxtcat, bdxtail, &mut temp32a);
            let mut bxtcatt = [0.0f64; 8];
            let bxtcattlen = scale_expansion_zeroelim(cattlen, &catt, bdxtail, &mut bxtcatt);
            let temp16alen =
                scale_expansion_zeroelim(bxtcattlen, &bxtcatt, 2.0 * bdx, &mut temp16a);
            let temp16blen = scale_expansion_zeroelim(bxtcattlen, &bxtcatt, bdxtail, &mut temp16b);
            let temp32blen = fast_expansion_sum_zeroelim(
                temp16alen,
                &temp16a,
                temp16blen,
                &temp16b,
                &mut temp32b,
            );
            let temp64len = fast_expansion_sum_zeroelim(
                temp32alen,
                &temp32a,
                temp32blen,
                &temp32b,
                &mut temp64,
            );
            finstep!(fin1, fin2, in_a, finlength, &temp64, temp64len);
        }
        if bdytail != 0.0 {
            let temp16alen = scale_expansion_zeroelim(bytcalen, &bytca, bdytail, &mut temp16a);
            let mut bytcat = [0.0f64; 16];
            let bytcatlen = scale_expansion_zeroelim(catlen, &cat, bdytail, &mut bytcat);
            let temp32alen = scale_expansion_zeroelim(bytcatlen, &bytcat, 2.0 * bdy, &mut temp32a);
            let temp48len = fast_expansion_sum_zeroelim(
                temp16alen,
                &temp16a,
                temp32alen,
                &temp32a,
                &mut temp48,
            );
            finstep!(fin1, fin2, in_a, finlength, &temp48, temp48len);

            let temp32alen = scale_expansion_zeroelim(bytcatlen, &bytcat, bdytail, &mut temp32a);
            let mut bytcatt = [0.0f64; 8];
            let bytcattlen = scale_expansion_zeroelim(cattlen, &catt, bdytail, &mut bytcatt);
            let temp16alen =
                scale_expansion_zeroelim(bytcattlen, &bytcatt, 2.0 * bdy, &mut temp16a);
            let temp16blen = scale_expansion_zeroelim(bytcattlen, &bytcatt, bdytail, &mut temp16b);
            let temp32blen = fast_expansion_sum_zeroelim(
                temp16alen,
                &temp16a,
                temp16blen,
                &temp16b,
                &mut temp32b,
            );
            let temp64len = fast_expansion_sum_zeroelim(
                temp32alen,
                &temp32a,
                temp32blen,
                &temp32b,
                &mut temp64,
            );
            finstep!(fin1, fin2, in_a, finlength, &temp64, temp64len);
        }
    }
    if (cdxtail != 0.0) || (cdytail != 0.0) {
        let mut abt = [0.0f64; 8];
        let mut abtt = [0.0f64; 4];
        let abtlen;
        #[allow(clippy::needless_late_init)]
        // C-parity: declared with the block, assigned per branch
        let abttlen;
        if (adxtail != 0.0) || (adytail != 0.0) || (bdxtail != 0.0) || (bdytail != 0.0) {
            let mut u = [0.0f64; 5];
            let mut v = [0.0f64; 5];

            let (ti1, ti0) = two_product(adxtail, bdy);
            let (tj1, tj0) = two_product(adx, bdytail);
            let (u3, u2, u1, u0) = two_two_sum(ti1, ti0, tj1, tj0);
            u[0] = u0;
            u[1] = u1;
            u[2] = u2;
            u[3] = u3;
            let negate = -ady;
            let (ti1, ti0) = two_product(bdxtail, negate);
            let negate = -adytail;
            let (tj1, tj0) = two_product(bdx, negate);
            let (v3, v2, v1, v0) = two_two_sum(ti1, ti0, tj1, tj0);
            v[0] = v0;
            v[1] = v1;
            v[2] = v2;
            v[3] = v3;
            abtlen = fast_expansion_sum_zeroelim(4, &u, 4, &v, &mut abt);

            let (ti1, ti0) = two_product(adxtail, bdytail);
            let (tj1, tj0) = two_product(bdxtail, adytail);
            let (abtt3, abtt2, abtt1, abtt0) = two_two_diff(ti1, ti0, tj1, tj0);
            abtt[0] = abtt0;
            abtt[1] = abtt1;
            abtt[2] = abtt2;
            abtt[3] = abtt3;
            abttlen = 4;
        } else {
            abt[0] = 0.0;
            abtlen = 1;
            abtt[0] = 0.0;
            abttlen = 1;
        }

        if cdxtail != 0.0 {
            let temp16alen = scale_expansion_zeroelim(cxtablen, &cxtab, cdxtail, &mut temp16a);
            let mut cxtabt = [0.0f64; 16];
            let cxtabtlen = scale_expansion_zeroelim(abtlen, &abt, cdxtail, &mut cxtabt);
            let temp32alen = scale_expansion_zeroelim(cxtabtlen, &cxtabt, 2.0 * cdx, &mut temp32a);
            let temp48len = fast_expansion_sum_zeroelim(
                temp16alen,
                &temp16a,
                temp32alen,
                &temp32a,
                &mut temp48,
            );
            finstep!(fin1, fin2, in_a, finlength, &temp48, temp48len);
            if adytail != 0.0 {
                let temp8len = scale_expansion_zeroelim(4, &bb, cdxtail, &mut temp8);
                let temp16alen = scale_expansion_zeroelim(temp8len, &temp8, adytail, &mut temp16a);
                finstep!(fin1, fin2, in_a, finlength, &temp16a, temp16alen);
            }
            if bdytail != 0.0 {
                let temp8len = scale_expansion_zeroelim(4, &aa, -cdxtail, &mut temp8);
                let temp16alen = scale_expansion_zeroelim(temp8len, &temp8, bdytail, &mut temp16a);
                finstep!(fin1, fin2, in_a, finlength, &temp16a, temp16alen);
            }

            let temp32alen = scale_expansion_zeroelim(cxtabtlen, &cxtabt, cdxtail, &mut temp32a);
            let mut cxtabtt = [0.0f64; 8];
            let cxtabttlen = scale_expansion_zeroelim(abttlen, &abtt, cdxtail, &mut cxtabtt);
            let temp16alen =
                scale_expansion_zeroelim(cxtabttlen, &cxtabtt, 2.0 * cdx, &mut temp16a);
            let temp16blen = scale_expansion_zeroelim(cxtabttlen, &cxtabtt, cdxtail, &mut temp16b);
            let temp32blen = fast_expansion_sum_zeroelim(
                temp16alen,
                &temp16a,
                temp16blen,
                &temp16b,
                &mut temp32b,
            );
            let temp64len = fast_expansion_sum_zeroelim(
                temp32alen,
                &temp32a,
                temp32blen,
                &temp32b,
                &mut temp64,
            );
            finstep!(fin1, fin2, in_a, finlength, &temp64, temp64len);
        }
        if cdytail != 0.0 {
            let temp16alen = scale_expansion_zeroelim(cytablen, &cytab, cdytail, &mut temp16a);
            let mut cytabt = [0.0f64; 16];
            let cytabtlen = scale_expansion_zeroelim(abtlen, &abt, cdytail, &mut cytabt);
            let temp32alen = scale_expansion_zeroelim(cytabtlen, &cytabt, 2.0 * cdy, &mut temp32a);
            let temp48len = fast_expansion_sum_zeroelim(
                temp16alen,
                &temp16a,
                temp32alen,
                &temp32a,
                &mut temp48,
            );
            finstep!(fin1, fin2, in_a, finlength, &temp48, temp48len);

            let temp32alen = scale_expansion_zeroelim(cytabtlen, &cytabt, cdytail, &mut temp32a);
            let mut cytabtt = [0.0f64; 8];
            let cytabttlen = scale_expansion_zeroelim(abttlen, &abtt, cdytail, &mut cytabtt);
            let temp16alen =
                scale_expansion_zeroelim(cytabttlen, &cytabtt, 2.0 * cdy, &mut temp16a);
            let temp16blen = scale_expansion_zeroelim(cytabttlen, &cytabtt, cdytail, &mut temp16b);
            let temp32blen = fast_expansion_sum_zeroelim(
                temp16alen,
                &temp16a,
                temp16blen,
                &temp16b,
                &mut temp32b,
            );
            let temp64len = fast_expansion_sum_zeroelim(
                temp32alen,
                &temp32a,
                temp32blen,
                &temp32b,
                &mut temp64,
            );
            finstep!(fin1, fin2, in_a, finlength, &temp64, temp64len);
        }
    }

    if in_a {
        fin1[finlength - 1]
    } else {
        fin2[finlength - 1]
    }
}

/// C: tri_incircle (predicates.c:1117). Positive if pd lies inside the
/// circle through pa, pb, pc (which must be in counterclockwise order).
pub(crate) fn incircle(
    m: &mut Mesh,
    b: &Behavior,
    pa: Vertex,
    pb: Vertex,
    pc: Vertex,
    pd: Vertex,
) -> f64 {
    m.incirclecount += 1;

    let adx = pa[0] - pd[0];
    let bdx = pb[0] - pd[0];
    let cdx = pc[0] - pd[0];
    let ady = pa[1] - pd[1];
    let bdy = pb[1] - pd[1];
    let cdy = pc[1] - pd[1];

    let bdxcdy = bdx * cdy;
    let cdxbdy = cdx * bdy;
    let alift = adx * adx + ady * ady;

    let cdxady = cdx * ady;
    let adxcdy = adx * cdy;
    let blift = bdx * bdx + bdy * bdy;

    let adxbdy = adx * bdy;
    let bdxady = bdx * ady;
    let clift = cdx * cdx + cdy * cdy;

    let det = alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);

    if b.noexact {
        return det;
    }

    let permanent = (absolute(bdxcdy) + absolute(cdxbdy)) * alift
        + (absolute(cdxady) + absolute(adxcdy)) * blift
        + (absolute(adxbdy) + absolute(bdxady)) * clift;
    let errbound = consts().iccerrboundA * permanent;
    if (det > errbound) || (-det > errbound) {
        return det;
    }

    incircle_adapt(pa, pb, pc, pd, permanent)
}

// ---------------------------------------------------------------------------
// orient3d
// ---------------------------------------------------------------------------

/// C: tri_orient3dadapt (predicates.c:1188). The adaptive cascade for the
/// 3D orientation test; triangle heights arrive as the_four REALs. The C
/// finnow/finother pointer swap becomes the same bool/array pick used in
/// tri_incircleadapt.
#[allow(clippy::too_many_arguments)] // C-parity 9-arg signature (predicates.c:1188)
fn orient3d_adapt(
    pa: Vertex,
    pb: Vertex,
    pc: Vertex,
    pd: Vertex,
    aheight: f64,
    bheight: f64,
    cheight: f64,
    dheight: f64,
    permanent: f64,
) -> f64 {
    let cst = consts();

    let adx = pa[0] - pd[0];
    let bdx = pb[0] - pd[0];
    let cdx = pc[0] - pd[0];
    let ady = pa[1] - pd[1];
    let bdy = pb[1] - pd[1];
    let cdy = pc[1] - pd[1];
    let adheight = aheight - dheight;
    let bdheight = bheight - dheight;
    let cdheight = cheight - dheight;

    let mut bc = [0.0f64; 4];
    let mut ca = [0.0f64; 4];
    let mut ab = [0.0f64; 4];

    let (bdxcdy1, bdxcdy0) = two_product(bdx, cdy);
    let (cdxbdy1, cdxbdy0) = two_product(cdx, bdy);
    let (bc3, bc2, bc1, bc0) = two_two_diff(bdxcdy1, bdxcdy0, cdxbdy1, cdxbdy0);
    bc[0] = bc0;
    bc[1] = bc1;
    bc[2] = bc2;
    bc[3] = bc3;
    let mut adet = [0.0f64; 8];
    let alen = scale_expansion_zeroelim(4, &bc, adheight, &mut adet);

    let (cdxady1, cdxady0) = two_product(cdx, ady);
    let (adxcdy1, adxcdy0) = two_product(adx, cdy);
    let (ca3, ca2, ca1, ca0) = two_two_diff(cdxady1, cdxady0, adxcdy1, adxcdy0);
    ca[0] = ca0;
    ca[1] = ca1;
    ca[2] = ca2;
    ca[3] = ca3;
    let mut bdet = [0.0f64; 8];
    let blen = scale_expansion_zeroelim(4, &ca, bdheight, &mut bdet);

    let (adxbdy1, adxbdy0) = two_product(adx, bdy);
    let (bdxady1, bdxady0) = two_product(bdx, ady);
    let (ab3, ab2, ab1, ab0) = two_two_diff(adxbdy1, adxbdy0, bdxady1, bdxady0);
    ab[0] = ab0;
    ab[1] = ab1;
    ab[2] = ab2;
    ab[3] = ab3;
    let mut cdet = [0.0f64; 8];
    let clen = scale_expansion_zeroelim(4, &ab, cdheight, &mut cdet);

    // C predicates.c:1204: REAL fin1[192], fin2[192]; the same finnow/
    // finother-in-a-flag scheme as tri_incircleadapt.
    let mut fin1 = [0.0f64; 192];
    let mut fin2 = [0.0f64; 192];
    let mut in_a = true;
    let mut abdet = [0.0f64; 16];
    let ablen = fast_expansion_sum_zeroelim(alen, &adet, blen, &bdet, &mut abdet);
    let mut finlength = fast_expansion_sum_zeroelim(ablen, &abdet, clen, &cdet, &mut fin1);

    macro_rules! finstep {
        ($fin1:ident, $fin2:ident, $in_a:ident, $flen:ident, $add:expr, $addlen:expr) => {
            if $in_a {
                $flen = fast_expansion_sum_zeroelim($flen, &$fin1, $addlen, $add, &mut $fin2);
                $in_a = false;
            } else {
                $flen = fast_expansion_sum_zeroelim($flen, &$fin2, $addlen, $add, &mut $fin1);
                $in_a = true;
            }
        };
    }

    let mut det = estimate(finlength, &fin1);
    let mut errbound = cst.o3derrboundB * permanent;
    if (det >= errbound) || (-det >= errbound) {
        return det;
    }

    let adxtail = two_diff_tail(pa[0], pd[0], adx);
    let bdxtail = two_diff_tail(pb[0], pd[0], bdx);
    let cdxtail = two_diff_tail(pc[0], pd[0], cdx);
    let adytail = two_diff_tail(pa[1], pd[1], ady);
    let bdytail = two_diff_tail(pb[1], pd[1], bdy);
    let cdytail = two_diff_tail(pc[1], pd[1], cdy);
    let adheighttail = two_diff_tail(aheight, dheight, adheight);
    let bdheighttail = two_diff_tail(bheight, dheight, bdheight);
    let cdheighttail = two_diff_tail(cheight, dheight, cdheight);

    if (adxtail == 0.0)
        && (bdxtail == 0.0)
        && (cdxtail == 0.0)
        && (adytail == 0.0)
        && (bdytail == 0.0)
        && (cdytail == 0.0)
        && (adheighttail == 0.0)
        && (bdheighttail == 0.0)
        && (cdheighttail == 0.0)
    {
        return det;
    }

    errbound = cst.o3derrboundC * permanent + cst.resulterrbound * absolute(det);
    det += (adheight * ((bdx * cdytail + cdy * bdxtail) - (bdy * cdxtail + cdx * bdytail))
        + adheighttail * (bdx * cdy - bdy * cdx))
        + (bdheight * ((cdx * adytail + ady * cdxtail) - (cdy * adxtail + adx * cdytail))
            + bdheighttail * (cdx * ady - cdy * adx))
        + (cdheight * ((adx * bdytail + bdy * adxtail) - (ady * bdxtail + bdx * adytail))
            + cdheighttail * (adx * bdy - ady * bdx));
    if (det >= errbound) || (-det >= errbound) {
        return det;
    }

    let mut at_b = [0.0f64; 4];
    let mut at_c = [0.0f64; 4];
    let (at_blen, at_clen);
    if adxtail == 0.0 {
        if adytail == 0.0 {
            at_b[0] = 0.0;
            at_blen = 1;
            at_c[0] = 0.0;
            at_clen = 1;
        } else {
            let negate = -adytail;
            let (at_blarge, at_b0) = two_product(negate, bdx);
            at_b[0] = at_b0;
            at_b[1] = at_blarge;
            at_blen = 2;
            let (at_clarge, at_c0) = two_product(adytail, cdx);
            at_c[0] = at_c0;
            at_c[1] = at_clarge;
            at_clen = 2;
        }
    } else if adytail == 0.0 {
        let (at_blarge, at_b0) = two_product(adxtail, bdy);
        at_b[0] = at_b0;
        at_b[1] = at_blarge;
        at_blen = 2;
        let negate = -adxtail;
        let (at_clarge, at_c0) = two_product(negate, cdy);
        at_c[0] = at_c0;
        at_c[1] = at_clarge;
        at_clen = 2;
    } else {
        let (adxt_bdy1, adxt_bdy0) = two_product(adxtail, bdy);
        let (adyt_bdx1, adyt_bdx0) = two_product(adytail, bdx);
        let (at_blarge, at_b2, at_b1, at_b0) =
            two_two_diff(adxt_bdy1, adxt_bdy0, adyt_bdx1, adyt_bdx0);
        at_b[0] = at_b0;
        at_b[1] = at_b1;
        at_b[2] = at_b2;
        at_b[3] = at_blarge;
        at_blen = 4;
        let (adyt_cdx1, adyt_cdx0) = two_product(adytail, cdx);
        let (adxt_cdy1, adxt_cdy0) = two_product(adxtail, cdy);
        let (at_clarge, at_c2, at_c1, at_c0) =
            two_two_diff(adyt_cdx1, adyt_cdx0, adxt_cdy1, adxt_cdy0);
        at_c[0] = at_c0;
        at_c[1] = at_c1;
        at_c[2] = at_c2;
        at_c[3] = at_clarge;
        at_clen = 4;
    }
    let mut bt_c = [0.0f64; 4];
    let mut bt_a = [0.0f64; 4];
    let (bt_clen, bt_alen);
    if bdxtail == 0.0 {
        if bdytail == 0.0 {
            bt_c[0] = 0.0;
            bt_clen = 1;
            bt_a[0] = 0.0;
            bt_alen = 1;
        } else {
            let negate = -bdytail;
            let (bt_clarge, bt_c0) = two_product(negate, cdx);
            bt_c[0] = bt_c0;
            bt_c[1] = bt_clarge;
            bt_clen = 2;
            let (bt_alarge, bt_a0) = two_product(bdytail, adx);
            bt_a[0] = bt_a0;
            bt_a[1] = bt_alarge;
            bt_alen = 2;
        }
    } else if bdytail == 0.0 {
        let (bt_clarge, bt_c0) = two_product(bdxtail, cdy);
        bt_c[0] = bt_c0;
        bt_c[1] = bt_clarge;
        bt_clen = 2;
        let negate = -bdxtail;
        let (bt_alarge, bt_a0) = two_product(negate, ady);
        bt_a[0] = bt_a0;
        bt_a[1] = bt_alarge;
        bt_alen = 2;
    } else {
        let (bdxt_cdy1, bdxt_cdy0) = two_product(bdxtail, cdy);
        let (bdyt_cdx1, bdyt_cdx0) = two_product(bdytail, cdx);
        let (bt_clarge, bt_c2, bt_c1, bt_c0) =
            two_two_diff(bdxt_cdy1, bdxt_cdy0, bdyt_cdx1, bdyt_cdx0);
        bt_c[0] = bt_c0;
        bt_c[1] = bt_c1;
        bt_c[2] = bt_c2;
        bt_c[3] = bt_clarge;
        bt_clen = 4;
        let (bdyt_adx1, bdyt_adx0) = two_product(bdytail, adx);
        let (bdxt_ady1, bdxt_ady0) = two_product(bdxtail, ady);
        let (bt_alarge, bt_a2, bt_a1, bt_a0) =
            two_two_diff(bdyt_adx1, bdyt_adx0, bdxt_ady1, bdxt_ady0);
        bt_a[0] = bt_a0;
        bt_a[1] = bt_a1;
        bt_a[2] = bt_a2;
        bt_a[3] = bt_alarge;
        bt_alen = 4;
    }
    let mut ct_a = [0.0f64; 4];
    let mut ct_b = [0.0f64; 4];
    let (ct_alen, ct_blen);
    if cdxtail == 0.0 {
        if cdytail == 0.0 {
            ct_a[0] = 0.0;
            ct_alen = 1;
            ct_b[0] = 0.0;
            ct_blen = 1;
        } else {
            let negate = -cdytail;
            let (ct_alarge, ct_a0) = two_product(negate, adx);
            ct_a[0] = ct_a0;
            ct_a[1] = ct_alarge;
            ct_alen = 2;
            let (ct_blarge, ct_b0) = two_product(cdytail, bdx);
            ct_b[0] = ct_b0;
            ct_b[1] = ct_blarge;
            ct_blen = 2;
        }
    } else if cdytail == 0.0 {
        let (ct_alarge, ct_a0) = two_product(cdxtail, ady);
        ct_a[0] = ct_a0;
        ct_a[1] = ct_alarge;
        ct_alen = 2;
        let negate = -cdxtail;
        let (ct_blarge, ct_b0) = two_product(negate, bdy);
        ct_b[0] = ct_b0;
        ct_b[1] = ct_blarge;
        ct_blen = 2;
    } else {
        let (cdxt_ady1, cdxt_ady0) = two_product(cdxtail, ady);
        let (cdyt_adx1, cdyt_adx0) = two_product(cdytail, adx);
        let (ct_alarge, ct_a2, ct_a1, ct_a0) =
            two_two_diff(cdxt_ady1, cdxt_ady0, cdyt_adx1, cdyt_adx0);
        ct_a[0] = ct_a0;
        ct_a[1] = ct_a1;
        ct_a[2] = ct_a2;
        ct_a[3] = ct_alarge;
        ct_alen = 4;
        let (cdyt_bdx1, cdyt_bdx0) = two_product(cdytail, bdx);
        let (cdxt_bdy1, cdxt_bdy0) = two_product(cdxtail, bdy);
        let (ct_blarge, ct_b2, ct_b1, ct_b0) =
            two_two_diff(cdyt_bdx1, cdyt_bdx0, cdxt_bdy1, cdxt_bdy0);
        ct_b[0] = ct_b0;
        ct_b[1] = ct_b1;
        ct_b[2] = ct_b2;
        ct_b[3] = ct_blarge;
        ct_blen = 4;
    }

    let mut bct = [0.0f64; 8];
    let bctlen = fast_expansion_sum_zeroelim(bt_clen, &bt_c, ct_blen, &ct_b, &mut bct);
    let mut w = [0.0f64; 16];
    let wlength = scale_expansion_zeroelim(bctlen, &bct, adheight, &mut w);
    finstep!(fin1, fin2, in_a, finlength, &w, wlength);

    let mut cat = [0.0f64; 8];
    let catlen = fast_expansion_sum_zeroelim(ct_alen, &ct_a, at_clen, &at_c, &mut cat);
    let wlength = scale_expansion_zeroelim(catlen, &cat, bdheight, &mut w);
    finstep!(fin1, fin2, in_a, finlength, &w, wlength);

    let mut abt = [0.0f64; 8];
    let abtlen = fast_expansion_sum_zeroelim(at_blen, &at_b, bt_alen, &bt_a, &mut abt);
    let wlength = scale_expansion_zeroelim(abtlen, &abt, cdheight, &mut w);
    finstep!(fin1, fin2, in_a, finlength, &w, wlength);

    let mut v = [0.0f64; 12];
    if adheighttail != 0.0 {
        let vlength = scale_expansion_zeroelim(4, &bc, adheighttail, &mut v);
        finstep!(fin1, fin2, in_a, finlength, &v, vlength);
    }
    if bdheighttail != 0.0 {
        let vlength = scale_expansion_zeroelim(4, &ca, bdheighttail, &mut v);
        finstep!(fin1, fin2, in_a, finlength, &v, vlength);
    }
    if cdheighttail != 0.0 {
        let vlength = scale_expansion_zeroelim(4, &ab, cdheighttail, &mut v);
        finstep!(fin1, fin2, in_a, finlength, &v, vlength);
    }

    let mut u = [0.0f64; 4];
    if adxtail != 0.0 {
        if bdytail != 0.0 {
            let (adxt_bdyt1, adxt_bdyt0) = two_product(adxtail, bdytail);
            let (u3, u2, u1, u0) = two_one_product(adxt_bdyt1, adxt_bdyt0, cdheight);
            u[0] = u0;
            u[1] = u1;
            u[2] = u2;
            u[3] = u3;
            finstep!(fin1, fin2, in_a, finlength, &u, 4);
            if cdheighttail != 0.0 {
                let (u3, u2, u1, u0) = two_one_product(adxt_bdyt1, adxt_bdyt0, cdheighttail);
                u[0] = u0;
                u[1] = u1;
                u[2] = u2;
                u[3] = u3;
                finstep!(fin1, fin2, in_a, finlength, &u, 4);
            }
        }
        if cdytail != 0.0 {
            let negate = -adxtail;
            let (adxt_cdyt1, adxt_cdyt0) = two_product(negate, cdytail);
            let (u3, u2, u1, u0) = two_one_product(adxt_cdyt1, adxt_cdyt0, bdheight);
            u[0] = u0;
            u[1] = u1;
            u[2] = u2;
            u[3] = u3;
            finstep!(fin1, fin2, in_a, finlength, &u, 4);
            if bdheighttail != 0.0 {
                let (u3, u2, u1, u0) = two_one_product(adxt_cdyt1, adxt_cdyt0, bdheighttail);
                u[0] = u0;
                u[1] = u1;
                u[2] = u2;
                u[3] = u3;
                finstep!(fin1, fin2, in_a, finlength, &u, 4);
            }
        }
    }
    if bdxtail != 0.0 {
        if cdytail != 0.0 {
            let (bdxt_cdyt1, bdxt_cdyt0) = two_product(bdxtail, cdytail);
            let (u3, u2, u1, u0) = two_one_product(bdxt_cdyt1, bdxt_cdyt0, adheight);
            u[0] = u0;
            u[1] = u1;
            u[2] = u2;
            u[3] = u3;
            finstep!(fin1, fin2, in_a, finlength, &u, 4);
            if adheighttail != 0.0 {
                let (u3, u2, u1, u0) = two_one_product(bdxt_cdyt1, bdxt_cdyt0, adheighttail);
                u[0] = u0;
                u[1] = u1;
                u[2] = u2;
                u[3] = u3;
                finstep!(fin1, fin2, in_a, finlength, &u, 4);
            }
        }
        if adytail != 0.0 {
            let negate = -bdxtail;
            let (bdxt_adyt1, bdxt_adyt0) = two_product(negate, adytail);
            let (u3, u2, u1, u0) = two_one_product(bdxt_adyt1, bdxt_adyt0, cdheight);
            u[0] = u0;
            u[1] = u1;
            u[2] = u2;
            u[3] = u3;
            finstep!(fin1, fin2, in_a, finlength, &u, 4);
            if cdheighttail != 0.0 {
                let (u3, u2, u1, u0) = two_one_product(bdxt_adyt1, bdxt_adyt0, cdheighttail);
                u[0] = u0;
                u[1] = u1;
                u[2] = u2;
                u[3] = u3;
                finstep!(fin1, fin2, in_a, finlength, &u, 4);
            }
        }
    }
    if cdxtail != 0.0 {
        if adytail != 0.0 {
            let (cdxt_adyt1, cdxt_adyt0) = two_product(cdxtail, adytail);
            let (u3, u2, u1, u0) = two_one_product(cdxt_adyt1, cdxt_adyt0, bdheight);
            u[0] = u0;
            u[1] = u1;
            u[2] = u2;
            u[3] = u3;
            finstep!(fin1, fin2, in_a, finlength, &u, 4);
            if bdheighttail != 0.0 {
                let (u3, u2, u1, u0) = two_one_product(cdxt_adyt1, cdxt_adyt0, bdheighttail);
                u[0] = u0;
                u[1] = u1;
                u[2] = u2;
                u[3] = u3;
                finstep!(fin1, fin2, in_a, finlength, &u, 4);
            }
        }
        if bdytail != 0.0 {
            let negate = -cdxtail;
            let (cdxt_bdyt1, cdxt_bdyt0) = two_product(negate, bdytail);
            let (u3, u2, u1, u0) = two_one_product(cdxt_bdyt1, cdxt_bdyt0, adheight);
            u[0] = u0;
            u[1] = u1;
            u[2] = u2;
            u[3] = u3;
            finstep!(fin1, fin2, in_a, finlength, &u, 4);
            if adheighttail != 0.0 {
                let (u3, u2, u1, u0) = two_one_product(cdxt_bdyt1, cdxt_bdyt0, adheighttail);
                u[0] = u0;
                u[1] = u1;
                u[2] = u2;
                u[3] = u3;
                finstep!(fin1, fin2, in_a, finlength, &u, 4);
            }
        }
    }

    if adheighttail != 0.0 {
        let wlength = scale_expansion_zeroelim(bctlen, &bct, adheighttail, &mut w);
        finstep!(fin1, fin2, in_a, finlength, &w, wlength);
    }
    if bdheighttail != 0.0 {
        let wlength = scale_expansion_zeroelim(catlen, &cat, bdheighttail, &mut w);
        finstep!(fin1, fin2, in_a, finlength, &w, wlength);
    }
    if cdheighttail != 0.0 {
        let wlength = scale_expansion_zeroelim(abtlen, &abt, cdheighttail, &mut w);
        finstep!(fin1, fin2, in_a, finlength, &w, wlength);
    }

    if in_a {
        fin1[finlength - 1]
    } else {
        fin2[finlength - 1]
    }
}

/// C: tri_orient3d (predicates.c:1598). Positive if pd lies below the
/// plane through pa, pb, pc. The four heights are the -w/-W lifting
/// coordinates. (C's 4-vertex + 4-height signature; clippy's argument
/// limit is acknowledged and kept for signature parity.)
#[allow(clippy::too_many_arguments)]
pub(crate) fn orient3d(
    m: &mut Mesh,
    b: &Behavior,
    pa: Vertex,
    pb: Vertex,
    pc: Vertex,
    pd: Vertex,
    aheight: f64,
    bheight: f64,
    cheight: f64,
    dheight: f64,
) -> f64 {
    m.orient3dcount += 1;

    let adx = pa[0] - pd[0];
    let bdx = pb[0] - pd[0];
    let cdx = pc[0] - pd[0];
    let ady = pa[1] - pd[1];
    let bdy = pb[1] - pd[1];
    let cdy = pc[1] - pd[1];
    let adheight = aheight - dheight;
    let bdheight = bheight - dheight;
    let cdheight = cheight - dheight;

    let bdxcdy = bdx * cdy;
    let cdxbdy = cdx * bdy;

    let cdxady = cdx * ady;
    let adxcdy = adx * cdy;

    let adxbdy = adx * bdy;
    let bdxady = bdx * ady;

    let det =
        adheight * (bdxcdy - cdxbdy) + bdheight * (cdxady - adxcdy) + cdheight * (adxbdy - bdxady);

    if b.noexact {
        return det;
    }

    let permanent = (absolute(bdxcdy) + absolute(cdxbdy)) * absolute(adheight)
        + (absolute(cdxady) + absolute(adxcdy)) * absolute(bdheight)
        + (absolute(adxbdy) + absolute(bdxady)) * absolute(cdheight);
    let errbound = consts().o3derrboundA * permanent;
    if (det > errbound) || (-det > errbound) {
        return det;
    }

    orient3d_adapt(
        pa, pb, pc, pd, aheight, bheight, cheight, dheight, permanent,
    )
}

// ---------------------------------------------------------------------------
// nonregular
// ---------------------------------------------------------------------------

/// C: tri_nonregular (predicates.c:1666). weighted == 0 applies the
/// incircle test; == 1 lifts onto the parabolic lifting map minus the
/// weight; == 2 uses the third coordinate as the height directly.
pub(crate) fn nonregular(
    m: &mut Mesh,
    b: &Behavior,
    pa: Vertex,
    pb: Vertex,
    pc: Vertex,
    pd: Vertex,
) -> f64 {
    if b.weighted == 0 {
        incircle(m, b, pa, pb, pc, pd)
    } else if b.weighted == 1 {
        orient3d(
            m,
            b,
            pa,
            pb,
            pc,
            pd,
            pa[0] * pa[0] + pa[1] * pa[1] - pa[2],
            pb[0] * pb[0] + pb[1] * pb[1] - pb[2],
            pc[0] * pc[0] + pc[1] * pc[1] - pc[2],
            pd[0] * pd[0] + pd[1] * pd[1] - pd[2],
        )
    } else {
        orient3d(m, b, pa, pb, pc, pd, pa[2], pb[2], pc[2], pd[2])
    }
}

// ---------------------------------------------------------------------------
// findcircumcenter
// ---------------------------------------------------------------------------

/// C: tri_findcircumcenter (predicates.c:1696). Returns
/// (circumcenter_x, circumcenter_y, xi, eta) instead of writing through
/// the C out pointers (ownership deviation). The offcenter flag plus
/// b.offconstant select Ungor's off-center, exactly like C.
pub(crate) fn find_circumcenter(
    m: &mut Mesh,
    b: &Behavior,
    torg: Vertex,
    tdest: Vertex,
    tapex: Vertex,
    offcenter: bool,
) -> (f64, f64, f64, f64) {
    m.circumcentercount += 1;

    // Compute the circumcenter of the triangle.
    let xdo = tdest[0] - torg[0];
    let ydo = tdest[1] - torg[1];
    let xao = tapex[0] - torg[0];
    let yao = tapex[1] - torg[1];
    let dodist = xdo * xdo + ydo * ydo;
    let aodist = xao * xao + yao * yao;
    let dadist = (tdest[0] - tapex[0]) * (tdest[0] - tapex[0])
        + (tdest[1] - tapex[1]) * (tdest[1] - tapex[1]);
    let denominator;
    if b.noexact {
        denominator = 0.5 / (xdo * yao - xao * ydo);
    } else {
        // Use the counterclockwise routine to ensure a positive (and
        // reasonably accurate) result, avoiding any possibility of
        // division by zero.
        denominator = 0.5 / counterclockwise(m, b, tdest, tapex, torg);
        // Don't count the above as an orientation test.
        m.counterclockcount -= 1;
    }
    let mut dx = (yao * dodist - ydo * aodist) * denominator;
    let mut dy = (xdo * aodist - xao * dodist) * denominator;

    // Find the (squared) length of the triangle's shortest edge (the
    // off-center logic, per Alper Ungor, as in C).
    if (dodist < aodist) && (dodist < dadist) {
        if offcenter && (b.offconstant > 0.0) {
            let dxoff = 0.5 * xdo - b.offconstant * ydo;
            let dyoff = 0.5 * ydo + b.offconstant * xdo;
            if dxoff * dxoff + dyoff * dyoff < dx * dx + dy * dy {
                dx = dxoff;
                dy = dyoff;
            }
        }
    } else if aodist < dadist {
        if offcenter && (b.offconstant > 0.0) {
            let dxoff = 0.5 * xao + b.offconstant * yao;
            let dyoff = 0.5 * yao - b.offconstant * xao;
            if dxoff * dxoff + dyoff * dyoff < dx * dx + dy * dy {
                dx = dxoff;
                dy = dyoff;
            }
        }
    } else if offcenter && (b.offconstant > 0.0) {
        let dxoff = 0.5 * (tapex[0] - tdest[0]) - b.offconstant * (tapex[1] - tdest[1]);
        let dyoff = 0.5 * (tapex[1] - tdest[1]) + b.offconstant * (tapex[0] - tdest[0]);
        if dxoff * dxoff + dyoff * dyoff < (dx - xdo) * (dx - xdo) + (dy - ydo) * (dy - ydo) {
            dx = xdo + dxoff;
            dy = ydo + dyoff;
        }
    }

    // xi-eta coordinates of the circumcenter, for vertex-attribute
    // interpolation at insertion time.
    let xi = (yao * dx - xao * dy) * (2.0 * denominator);
    let eta = (xdo * dy - ydo * dx) * (2.0 * denominator);

    (torg[0] + dx, torg[1] + dy, xi, eta)
}

// ---------------------------------------------------------------------------
// Tests. Float equality assertions are deliberate: the adaptive predicates
// are EXACT, and every test input below is chosen so the expected value
// is deterministic (representable integer/exact rational inputs whose
// exact result the code computes with zero rounding, or symmetric inputs
// whose exact answer is provably zero).
// ---------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;
    use crate::mesh::Mesh;
    use crate::options::Behavior;

    fn behavior() -> Behavior {
        Behavior::default_form()
    }

    fn test_mesh() -> Mesh {
        Mesh::new(&behavior(), 16, 2, 0, 0).expect("oom in tests")
    }

    fn approx(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    fn v(x: f64, y: f64) -> Vertex {
        [x, y, 0.0]
    }

    #[test]
    fn exactinit_constants() {
        exact_init();
        // Idempotent (the OnceLock guard makes repeats no-ops).
        exact_init();
        let c = consts();
        // The exact 2^-53 bit pattern; powi(-53) is not a bit-exact
        // reference across float implementations and runs 1 ulp off under
        // miri (#29).
        assert_eq!(c.epsilon, f64::from_bits(0x3ca0000000000000));
        assert_eq!(c.splitter, 134217729.0);
        assert!(c.resulterrbound > 0.0);
        assert!(c.ccwerrboundA > 0.0 && c.ccwerrboundB > 0.0 && c.ccwerrboundC > 0.0);
        assert!(c.iccerrboundA > 0.0 && c.iccerrboundB > 0.0 && c.iccerrboundC > 0.0);
        assert!(c.o3derrboundA > 0.0 && c.o3derrboundB > 0.0 && c.o3derrboundC > 0.0);
    }

    #[test]
    fn two_word_arithmetic_exact_errors() {
        // Two_Sum of values whose exact sum needs 54 bits: the tail holds
        // exactly the lost 1.0.
        let (x, y) = two_sum(1.0e16, 1.0);
        assert_eq!(x, 1.0e16);
        assert_eq!(y, 1.0);
        // Two_Diff against a negative operand: same magnitude story.
        let (x, y) = two_diff(1.0e16, -1.0);
        assert_eq!(x, 1.0e16);
        assert_eq!(y, 1.0);
        // Two_Product of two inexact-in-binary factors has a nonzero tail
        // and x is the plain fp product.
        let (x, y) = two_product(0.1, 0.2);
        assert_eq!(x, 0.1 * 0.2);
        assert!(y != 0.0);
        // Split reconstructs the input exactly (Dekker split property).
        let (hi, lo) = split(0.1);
        assert_eq!(hi + lo, 0.1);
        // Fast_Two_Sum matches Two_Sum on a Sterbenz-safe pair.
        let (x1, y1) = fast_two_sum(5.0, 2.0);
        let (x2, y2) = two_sum(5.0, 2.0);
        assert_eq!(x1, x2);
        assert_eq!(y1, y2);
    }

    #[test]
    fn expansion_ops() {
        assert_eq!(estimate(3, &[1.0, 2.0, 4.0]), 7.0);
        // Zero elimination + fallback single component.
        let mut h = [0.0f64; 4];
        let hlen = fast_expansion_sum_zeroelim(2, &[1.0, 0.0], 1, &[2.0], &mut h);
        assert_eq!(hlen, 1);
        assert_eq!(h[0], 3.0);
        // The sum of two 2-word exact products reconstructs a*b + c*d to
        // the last bit of the decomposition's estimate.
        let (e1, e0) = two_product(0.1, 0.2);
        let (f1, f0) = two_product(0.3, 0.4);
        // Expansion components ascend: [tail, head], index 0 least
        // significant (the C arrays).
        let e = [e0, e1];
        let f = [f0, f1];
        let mut h = [0.0f64; 8];
        let hlen = fast_expansion_sum_zeroelim(2, &e, 2, &f, &mut h);
        assert!(approx(
            estimate(hlen, &h[..hlen]),
            0.1 * 0.2 + 0.3 * 0.4,
            1.0e-17
        ));
        // Scaling an inexact product splits it into [tail, product].
        let mut h = [0.0f64; 4];
        let hlen = scale_expansion_zeroelim(1, &[0.1], 0.1, &mut h);
        assert_eq!(hlen, 2);
        assert_eq!(h[1], 0.1 * 0.1);
        assert!(h[0] != 0.0);
    }

    #[test]
    fn counterclockwise_signs_and_counters() {
        let mut m = test_mesh();
        let b = behavior();
        assert!(counterclockwise(&mut m, &b, v(0.0, 0.0), v(1.0, 0.0), v(0.0, 1.0)) > 0.0);
        assert!(counterclockwise(&mut m, &b, v(0.0, 1.0), v(1.0, 0.0), v(0.0, 0.0)) < 0.0);
        // Exactly collinear, all-exact coordinates: exact 0.0.
        assert_eq!(
            counterclockwise(&mut m, &b, v(0.0, 0.0), v(1.0, 1.0), v(2.0, 2.0)),
            0.0
        );
        assert_eq!(m.counterclockcount, 3);
    }

    #[test]
    fn counterclockwise_adapt_exact_collinear() {
        let mut m = test_mesh();
        let b = behavior();
        // fp-diagonal triple with representable coordinates but tiny
        // magnitudes: forces the filter out (|det| below the errbound) and
        // the exact arithmetic answers exactly 0.0.
        let r = counterclockwise(
            &mut m,
            &b,
            v(1.0e-16, 1.0e-16),
            v(2.0e-16, 2.0e-16),
            v(3.0e-16, 3.0e-16),
        );
        assert_eq!(r, 0.0);
    }

    #[test]
    fn counterclockwise_adapt_deep_cascade_exact_zero() {
        let mut m = test_mesh();
        let b = behavior();
        // Diagonal triple with Sterbenz-violating coordinate magnitudes:
        // the fp differences round (the tails are nonzero), the two
        // products cancel only in exact arithmetic, and the x/y symmetry
        // of the inputs forces every expansion in the adapt cascade
        // (B estimate, tail update, C1, C2, D) to run and to sum to exactly
        // zero. The predicate must return exactly 0.0.
        let p40 = 2.0f64.powi(40);
        let q = p40 + 2.0f64.powi(32);
        let r = 2.0f64.powi(38) + 2.0f64.powi(-15);
        let det = counterclockwise(&mut m, &b, v(p40, p40), v(q, q), v(r, r));
        assert_eq!(det, 0.0);
    }

    #[test]
    fn incircle_signs_and_counters() {
        let mut m = test_mesh();
        let b = behavior();
        // Unit circle through (1,0), (0,1), (-1,0) in CCW order.
        assert!(
            incircle(
                &mut m,
                &b,
                v(1.0, 0.0),
                v(0.0, 1.0),
                v(-1.0, 0.0),
                v(0.0, 0.0)
            ) > 0.0
        );
        assert!(
            incircle(
                &mut m,
                &b,
                v(1.0, 0.0),
                v(0.0, 1.0),
                v(-1.0, 0.0),
                v(5.0, 5.0)
            ) < 0.0
        );
        // All four points exactly on the unit circle, integer coordinates:
        // the exact determinant is zero (the adapt path runs on all-exact
        // products and returns 0.0).
        let d = incircle(
            &mut m,
            &b,
            v(1.0, 0.0),
            v(0.0, 1.0),
            v(-1.0, 0.0),
            v(0.0, -1.0),
        );
        assert_eq!(d, 0.0);
        assert_eq!(m.incirclecount, 3);
    }

    #[test]
    fn orient3d_signs_and_counters() {
        let mut m = test_mesh();
        let b = behavior();
        let pa = v(1.0, 0.0);
        let pb = v(0.0, 1.0);
        let pc = v(-1.0, 0.0);
        let pd = v(0.0, 0.0);
        // pd one unit above the z=0 plane: negative (below = positive).
        assert!(orient3d(&mut m, &b, pa, pb, pc, pd, 0.0, 0.0, 0.0, 1.0) < 0.0);
        assert!(orient3d(&mut m, &b, pa, pb, pc, pd, 0.0, 0.0, 0.0, -1.0) > 0.0);
        // Coplanar, integer coordinates: exact 0.0.
        assert_eq!(
            orient3d(&mut m, &b, pa, pb, pc, pd, 0.0, 0.0, 0.0, 0.0),
            0.0
        );
        assert_eq!(m.orient3dcount, 3);
    }

    #[test]
    fn nonregular_dispatch() {
        // weighted == 0 (default): incircle test.
        let mut m = test_mesh();
        let b = behavior();
        let r = nonregular(
            &mut m,
            &b,
            v(1.0, 0.0),
            v(0.0, 1.0),
            v(-1.0, 0.0),
            v(5.0, 5.0),
        );
        assert!(r < 0.0);
        assert_eq!(m.incirclecount, 1);
        assert_eq!(m.orient3dcount, 0);

        // weighted == 1: parabolic lift x^2 + y^2 - weight, orient3d.
        let mut m = test_mesh();
        let mut b = behavior();
        b.weighted = 1;
        // All lifts zero except pd (weight 1): pd sits below the z=0
        // lifted plane -> positive.
        let pa = [1.0, 0.0, 1.0];
        let pb = [0.0, 1.0, 1.0];
        let pc = [-1.0, 0.0, 1.0];
        let pd = [0.0, 0.0, 1.0];
        let r = nonregular(&mut m, &b, pa, pb, pc, pd);
        assert!(r > 0.0);
        assert_eq!(m.incirclecount, 0);
        assert_eq!(m.orient3dcount, 1);

        // weighted == 2: heights straight from vertex[2] (-W).
        let mut m = test_mesh();
        let mut b = behavior();
        b.weighted = 2;
        let r = nonregular(&mut m, &b, pa, pb, pc, pd);
        assert_eq!(r, 0.0);
        assert_eq!(m.orient3dcount, 1);
    }

    #[test]
    fn findcircumcenter_right_triangle() {
        let mut m = test_mesh();
        let b = behavior();
        let (x, y, xi, eta) =
            find_circumcenter(&mut m, &b, v(0.0, 0.0), v(1.0, 0.0), v(0.0, 1.0), false);
        assert_eq!(x, 0.5);
        assert_eq!(y, 0.5);
        assert_eq!(xi, 0.5);
        assert_eq!(eta, 0.5);
        // One circumcenter computation; its internal counterclockwise
        // call was subtracted back out (C: m->counterclockcount--).
        assert_eq!(m.circumcentercount, 1);
        assert_eq!(m.counterclockcount, 0);
    }

    #[test]
    fn findcircumcenter_offcenter() {
        let mut m = test_mesh();
        let mut b = behavior();
        b.offconstant = 1.0;
        // Skinny triangle: the off-center (Ungor) point beats the far-away
        // circumcenter and takes its place.
        let (x, y, _, _) =
            find_circumcenter(&mut m, &b, v(0.0, 0.0), v(1.0, 0.0), v(0.4, 0.001), true);
        assert!(approx(x, 0.201, 1.0e-12));
        assert!(approx(y, -0.3995, 1.0e-12));
        // Without the off-center selection the circumcenter is far away.
        let mut m2 = test_mesh();
        let (x2, y2, _, _) =
            find_circumcenter(&mut m2, &b, v(0.0, 0.0), v(1.0, 0.0), v(0.4, 0.001), false);
        assert!(approx(y2, -119.9995, 1.0e-9));
        assert!(approx(x2, 0.5, 1.0e-9));
    }

    #[test]
    fn noexact_short_circuits_still_counted() {
        let mut m = test_mesh();
        let mut b = behavior();
        b.noexact = true;
        // The symmetric deep-cascade input: the fp filter values cancel
        // exactly, so even the fast path returns 0.0 here; the counters
        // still tick.
        let p40 = 2.0f64.powi(40);
        let q = p40 + 2.0f64.powi(32);
        let r = 2.0f64.powi(38) + 2.0f64.powi(-15);
        let det = counterclockwise(&mut m, &b, v(p40, p40), v(q, q), v(r, r));
        assert_eq!(det, 0.0);
        let _ = incircle(
            &mut m,
            &b,
            v(1.0, 0.0),
            v(0.0, 1.0),
            v(-1.0, 0.0),
            v(0.0, 0.0),
        );
        let _ = orient3d(
            &mut m,
            &b,
            v(1.0, 0.0),
            v(0.0, 1.0),
            v(-1.0, 0.0),
            v(0.0, 0.0),
            0.0,
            0.0,
            0.0,
            0.0,
        );
        assert_eq!(m.counterclockcount, 1);
        assert_eq!(m.incirclecount, 1);
        assert_eq!(m.orient3dcount, 1);
    }
}
