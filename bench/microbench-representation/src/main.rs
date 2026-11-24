//! Representation microbench for wayfinder ticket #7 (mestrode/triangle).
//!
//! Throwaway bench: delivers data, not a decision; it informs ticket #8.
//!
//! Compares three candidate (triangle, orientation) handle representations for
//! the Rust port of Triangle 1.6, at hot operations mirrored from the C macros
//! in src/Triangle/triangle.h (encode/decode at triangle.h:703-708, consumers
//! sym/lnext/lprev/... at triangle.h:703-1090):
//!
//!   * `lnext` / `lprev` rings: pure orientation arithmetic on a handle
//!     (C: lnextself/lprprevself, plus1mod3/minus1mod3).
//!   * `sym` analog (`decode(tri[orient])`): slot load + per-variant unpack,
//!     both as a linear allocation-order sweep (triangletraverse shape) and
//!     inside pseudo-random neighbor-deref walks (splay point location /
//!     flip-sequence shape: pointer chase, not linear scan).
//!   * encode / decode streams over all neighbor slots.
//!
//! Variants (scope ruling 2026-11-24: the tagged raw-pointer variant of C is
//! out; per ticket #7 only these three remain):
//!
//!   A) AoS struct handle: pool index plus separate orientation field,
//!      size_of::<OA>() == 16, 16 B per neighbor slot, 72 B rows
//!      (3 nbr slots + 3 vertex slots, mirroring the C row layout).
//!   B) Packed u64 handle: 32-bit pool index in the high bits, orientation in
//!      the low 2 bits of one u64, 8 B per slot, 48 B rows (C slot density).
//!   C) SoA split: neighbor indices (u32) and orientations (u8) in separate
//!      parallel arrays, vertex array separate (8-byte-equivalent-or-better
//!      density, different locality).
//!
//! Validity: everything is deterministic (one fixed LCG drives mesh shuffling
//! and walker choices); all three variants must produce identical checksums in
//! every kernel, and sym(sym(h)) == h is validated on the whole mesh before
//! timing. Timing is hand-rolled (std::time::Instant), stable toolchain, no
//! external dependencies: median over REPS repetitions with min/max/sd spread.

use std::hint::black_box;
use std::time::Instant;

const M: usize = 724; // cells per axis
const NTRI: usize = 2 * M * M; // 1_048_352 triangles
const REPS: usize = 20;
const RING_STEPS: usize = 60_000_000;
const SWEEP_PASSES: usize = 8;
const CHASE_STEPS: usize = 4_000_000;
const STREAM_PASSES: usize = 8;

// --------------------------------------------------------------------- RNG

/// Deterministic 64-bit LCG (NOT Triangle's randomnation; only determinism
/// matters here). High bits are consumed; the sequence is identical across
/// variants, so all checksums must agree.
pub struct Lcg(u64);

impl Lcg {
    #[inline]
    pub fn new(seed: u64) -> Self {
        Lcg(seed)
    }
    #[inline]
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0
    }
    #[inline]
    pub fn below(&mut self, n: u32) -> u32 {
        ((self.next_u64() >> 33) as u32) % n
    }
}

// ------------------------------------------------------------------- handles

/// A) AoS struct handle: unpacked fields, 16 bytes per neighbor slot
/// (u64 index + u32 orientation + 4 bytes padding).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct OA {
    pub tri: u64,
    pub orient: u32,
}

/// B) Packed u64 handle: 32-bit index in the high bits, orientation in the
/// low 2 bits. 8 bytes per neighbor slot (same density as the C tagged slot).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct OB {
    pub raw: u64,
}

/// C) SoA companion handle: small pair that lives in registers while the mesh
/// keeps indices and orientations in separate parallel arrays.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct OC {
    pub tri: u32,
    pub orient: u8,
}

#[inline]
fn ob_raw(tri: u32, orient: u32) -> u64 {
    ((tri as u64) << 2) | (orient & 3) as u64
}
#[inline]
fn ob_tri(h: OB) -> u32 {
    (h.raw >> 2) as u32
}
#[inline]
fn ob_orient(h: OB) -> u32 {
    (h.raw & 3) as u32
}

// -------------------------------------------------------------------- meshes

/// A row: 3 neighbor slots (16 B each) + 3 vertex slots (8 B each) = 72 B.
/// Mirrors the C triangle == REAL** row shape (t[0..2] neighbors, t[3..5]
/// vertices) with the unpacked handle replacing the tagged pointer.
#[derive(Clone, Copy)]
pub struct RowA {
    pub nbr: [OA; 3],
    pub vtx: [u64; 3],
}
pub struct MeshA {
    pub rows: Box<[RowA]>,
}

/// B row: 3 packed slots (8 B each) + 3 vertex slots = 48 B: exactly the C row
/// stride on x86-64.
#[derive(Clone, Copy)]
pub struct RowB {
    pub nbr: [u64; 3],
    pub vtx: [u64; 3],
}
pub struct MeshB {
    pub rows: Box<[RowB]>,
}

/// C split storage: parallel arrays, 3*ntri entries each.
pub struct MeshC {
    pub nbr: Box<[u32]>,
    pub orient: Box<[u8]>,
    pub vtx: Box<[u64]>,
}

// ------------------------------------------------------------ handle ops (A)

#[inline]
fn lnext_a(h: OA) -> OA {
    OA {
        tri: h.tri,
        orient: (h.orient + 1) % 3,
    }
}
#[inline]
fn lprev_a(h: OA) -> OA {
    OA {
        tri: h.tri,
        orient: (h.orient + 2) % 3,
    }
}
#[inline]
fn sym_a(m: &MeshA, h: OA) -> OA {
    m.rows[h.tri as usize].nbr[h.orient as usize]
}
#[inline]
fn fold_a(h: OA) -> u64 {
    h.tri * 4 + h.orient as u64
}
fn ora(h: OA) -> usize {
    h.orient as usize
}
fn mk_a(t: u32, e: u32) -> OA {
    OA {
        tri: t as u64,
        orient: e,
    }
}

// ------------------------------------------------------------ handle ops (B)

#[inline]
fn lnext_b(h: OB) -> OB {
    OB {
        raw: (h.raw & !3u64) | (((ob_orient(h) + 1) % 3) as u64),
    }
}
#[inline]
fn lprev_b(h: OB) -> OB {
    OB {
        raw: (h.raw & !3u64) | (((ob_orient(h) + 2) % 3) as u64),
    }
}
#[inline]
fn sym_b(m: &MeshB, h: OB) -> OB {
    OB {
        raw: m.rows[ob_tri(h) as usize].nbr[ob_orient(h) as usize],
    }
}
#[inline]
fn fold_b(h: OB) -> u64 {
    ob_tri(h) as u64 * 4 + ob_orient(h) as u64
}
fn orb(h: OB) -> usize {
    (h.raw & 3) as usize
}
fn mk_b(t: u32, e: u32) -> OB {
    OB { raw: ob_raw(t, e) }
}

// ------------------------------------------------------------ handle ops (C)

#[inline]
fn lnext_c(h: OC) -> OC {
    OC {
        tri: h.tri,
        orient: (h.orient + 1) % 3,
    }
}
#[inline]
fn lprev_c(h: OC) -> OC {
    OC {
        tri: h.tri,
        orient: (h.orient + 2) % 3,
    }
}
#[inline]
fn sym_c(m: &MeshC, h: OC) -> OC {
    let s = h.tri as usize * 3 + h.orient as usize;
    OC {
        tri: m.nbr[s],
        orient: m.orient[s],
    }
}
#[inline]
fn fold_c(h: OC) -> u64 {
    h.tri as u64 * 4 + h.orient as u64
}
fn orc(h: OC) -> usize {
    h.orient as usize
}
fn mk_c(t: u32, e: u32) -> OC {
    OC {
        tri: t,
        orient: e as u8,
    }
}

// ------------------------------------------------------------------ topology

/// Half-edge-shaped topology: slots[3*t + e] = (neighbor triangle id, twin
/// edge slot). Periodic m x m grid of squares, each split into 2 triangles
/// along the diagonal (T0: A, B, D; T1: A, D, C), wrapped around so there is
/// no boundary and every variant runs the same graph. `perm` re-indexes pool
/// ids (None = allocation order equals grid order; Some = Fisher-Yates
/// shuffle for uniform scatter).
fn build_topology(m: usize, perm: Option<&[u32]>) -> Vec<(u32, u8)> {
    let cells = m * m;
    let ntri = 2 * cells;
    assert_eq!(ntri, NTRI);
    let mut slots: Vec<(u32, u8)> = vec![(0, 0); 3 * ntri];
    let cell_t0 = |i: i64, j: i64| -> usize {
        let ii = i.rem_euclid(m as i64) as usize;
        let jj = j.rem_euclid(m as i64) as usize;
        2 * (jj * m + ii)
    };
    for j in 0..m {
        for i in 0..m {
            let c = j * m + i;
            let t0 = 2 * c;
            let t1 = t0 + 1;
            let below = cell_t0(i as i64, j as i64 - 1);
            let right = cell_t0(i as i64 + 1, j as i64);
            let above = cell_t0(i as i64, j as i64 + 1);
            let left = cell_t0(i as i64 - 1, j as i64);
            // T0 edges: e0 = A->B, e1 = B->D, e2 = D->A
            slots[3 * t0] = ((below + 1) as u32, 1); // twin: T1(below).e1
            slots[3 * t0 + 1] = ((right + 1) as u32, 2); // twin: T1(right).e2
            slots[3 * t0 + 2] = (t1 as u32, 0); // twin: T1(same).e0
            // T1 edges: e0 = A->D, e1 = D->C, e2 = C->A
            slots[3 * t1] = (t0 as u32, 2); // twin: T0(same).e2
            slots[3 * t1 + 1] = (above as u32, 0); // twin: T0(above).e0
            slots[3 * t1 + 2] = (left as u32, 1); // twin: T0(left).e1
        }
    }
    match perm {
        None => slots,
        Some(p) => {
            let mut out: Vec<(u32, u8)> = vec![(0, 0); 3 * ntri];
            for t in 0..ntri {
                let pid = p[t] as usize;
                for e in 0..3 {
                    let (nb, te) = slots[3 * t + e];
                    out[3 * pid + e] = (p[nb as usize], te);
                }
            }
            out
        }
    }
}

fn vertex_value(pid: usize, e: usize) -> u64 {
    (3 * pid + e) as u64
}

fn materialize_a(topo: &[(u32, u8)]) -> MeshA {
    let mut rows: Vec<RowA> = Vec::with_capacity(NTRI);
    for t in 0..NTRI {
        let (n0, e0) = topo[3 * t];
        let (n1, e1) = topo[3 * t + 1];
        let (n2, e2) = topo[3 * t + 2];
        rows.push(RowA {
            nbr: [
                OA {
                    tri: n0 as u64,
                    orient: e0 as u32,
                },
                OA {
                    tri: n1 as u64,
                    orient: e1 as u32,
                },
                OA {
                    tri: n2 as u64,
                    orient: e2 as u32,
                },
            ],
            vtx: [
                vertex_value(t, 0),
                vertex_value(t, 1),
                vertex_value(t, 2),
            ],
        });
    }
    MeshA {
        rows: rows.into_boxed_slice(),
    }
}

fn materialize_b(topo: &[(u32, u8)]) -> MeshB {
    let mut rows: Vec<RowB> = Vec::with_capacity(NTRI);
    for t in 0..NTRI {
        let (n0, e0) = topo[3 * t];
        let (n1, e1) = topo[3 * t + 1];
        let (n2, e2) = topo[3 * t + 2];
        rows.push(RowB {
            nbr: [ob_raw(n0, e0 as u32), ob_raw(n1, e1 as u32), ob_raw(n2, e2 as u32)],
            vtx: [
                vertex_value(t, 0),
                vertex_value(t, 1),
                vertex_value(t, 2),
            ],
        });
    }
    MeshB {
        rows: rows.into_boxed_slice(),
    }
}

fn materialize_c(topo: &[(u32, u8)]) -> MeshC {
    let mut nbr: Vec<u32> = Vec::with_capacity(3 * NTRI);
    let mut orient: Vec<u8> = Vec::with_capacity(3 * NTRI);
    let mut vtx: Vec<u64> = Vec::with_capacity(3 * NTRI);
    for t in 0..NTRI {
        for e in 0..3 {
            let (n, te) = topo[3 * t + e];
            nbr.push(n);
            orient.push(te);
            vtx.push(vertex_value(t, e));
        }
    }
    MeshC {
        nbr: nbr.into_boxed_slice(),
        orient: orient.into_boxed_slice(),
        vtx: vtx.into_boxed_slice(),
    }
}

// ---------------------------------------------------------------- validation

fn validate_sym_a(m: &MeshA) -> bool {
    for t in 0..NTRI as u64 {
        for e in 0..3u32 {
            let h = OA { tri: t, orient: e };
            if sym_a(m, sym_a(m, h)) != h {
                return false;
            }
        }
    }
    true
}

fn validate_sym_b(m: &MeshB) -> bool {
    for t in 0..NTRI as u32 {
        for e in 0..3u32 {
            let h = OB { raw: ob_raw(t, e) };
            if sym_b(m, sym_b(m, h)) != h {
                return false;
            }
        }
    }
    true
}

fn validate_sym_c(m: &MeshC) -> bool {
    for t in 0..NTRI as u32 {
        for e in 0..3u8 {
            let h = OC { tri: t, orient: e };
            if sym_c(m, sym_c(m, h)) != h {
                return false;
            }
        }
    }
    true
}

// ------------------------------------------------------------------- kernels

/// Ring kernel: pure handle rotation (lnextself / lprevself analog), no memory.
/// The fold chain through the runtime-opaque `sink` keeps the loop un-foldable;
/// its mul/add is a constant floor identical for all variants, so read deltas.
fn ring<V: Copy, R: Fn(V) -> V, O: Fn(V) -> usize>(
    steps: usize,
    start: V,
    rotate: R,
    orient_of: O,
    sink: &[u64; 3],
) -> u64 {
    let mut h = start;
    let mut acc: u64 = 0;
    for _ in 0..steps {
        h = rotate(h);
        acc = acc.wrapping_mul(1000003).wrapping_add(sink[orient_of(h)]);
    }
    acc
}

/// Neighbor-deref chase (sym through a pseudo-rotated edge each step):
/// pointer-chase shaped, like splay point location or flip sequences. The
/// per-step LCG is a constant, identical across variants.
fn chase<V: Copy, L: Fn(V) -> V, S: Fn(V) -> V, F: Fn(V) -> u64>(
    steps: usize,
    start: V,
    lnext_op: L,
    sym_op: S,
    fold: F,
    seed: u64,
) -> u64 {
    let mut rng = Lcg::new(seed);
    let mut h = start;
    let mut acc: u64 = 0;
    for _ in 0..steps {
        let r = rng.below(3);
        for _ in 0..r {
            h = lnext_op(h);
        }
        h = sym_op(h);
        acc = acc.wrapping_mul(31).wrapping_add(fold(h));
    }
    acc
}

/// Linear sym sweep over all slots in allocation order (triangletraverse shape).
fn sweep<V: Copy, K: Fn(u32, u32) -> V, S: Fn(V) -> V, F: Fn(V) -> u64>(
    passes: usize,
    make: K,
    sym_op: S,
    fold: F,
) -> u64 {
    let mut acc: u64 = 0;
    for _ in 0..passes {
        for t in 0..NTRI as u32 {
            for e in 0..3u32 {
                let h = make(t, e);
                let nb = sym_op(h);
                acc = acc.wrapping_mul(31).wrapping_add(fold(nb));
            }
        }
    }
    acc
}

/// Decode stream: read every neighbor slot, unpack, fold. Encode is the only
/// mutation-shaped kernel here (slot writes from raw pairs); a real flip also
/// rewrites ~6 slots, so this stream is the store-side stand-in.
fn decode_stream_a(passes: usize, m: &MeshA) -> u64 {
    let mut acc: u64 = 0;
    for _ in 0..passes {
        for t in 0..NTRI {
            let row = &m.rows[t];
            for e in 0..3 {
                let s = row.nbr[e];
                acc = acc.wrapping_mul(31).wrapping_add(s.tri * 4 + s.orient as u64);
            }
        }
    }
    acc
}

fn decode_stream_b(passes: usize, m: &MeshB) -> u64 {
    let mut acc: u64 = 0;
    for _ in 0..passes {
        for t in 0..NTRI {
            let row = &m.rows[t];
            for e in 0..3 {
                let r = row.nbr[e];
                acc = acc
                    .wrapping_mul(31)
                    .wrapping_add((r >> 2) * 4 + (r & 3));
            }
        }
    }
    acc
}

fn decode_stream_c(passes: usize, m: &MeshC) -> u64 {
    let mut acc: u64 = 0;
    for _ in 0..passes {
        let mut s = 0usize;
        for t in 0..NTRI {
            for _ in 0..3 {
                let v = m.nbr[s];
                let o = m.orient[s];
                s += 1;
                let _ = t;
                acc = acc.wrapping_mul(31).wrapping_add(v as u64 * 4 + o as u64);
            }
        }
    }
    acc
}

fn encode_stream_a(passes: usize, scratch: &mut MeshA, src: &[(u32, u8)]) -> u64 {
    black_box(scratch as *mut MeshA); // keep stores observable
    let mut acc: u64 = 0;
    for _ in 0..passes {
        let mut k = 0usize;
        for t in 0..NTRI {
            let row = &mut scratch.rows[t];
            for e in 0..3 {
                let (v, o) = src[k];
                k += 1;
                row.nbr[e] = OA {
                    tri: v as u64,
                    orient: o as u32,
                };
                acc = acc.wrapping_mul(31).wrapping_add(v as u64 * 3 + o as u64);
            }
        }
    }
    acc
}

fn encode_stream_b(passes: usize, scratch: &mut MeshB, src: &[(u32, u8)]) -> u64 {
    black_box(scratch as *mut MeshB);
    let mut acc: u64 = 0;
    for _ in 0..passes {
        let mut k = 0usize;
        for t in 0..NTRI {
            let row = &mut scratch.rows[t];
            for e in 0..3 {
                let (v, o) = src[k];
                k += 1;
                row.nbr[e] = ob_raw(v, o as u32);
                acc = acc.wrapping_mul(31).wrapping_add(v as u64 * 3 + o as u64);
            }
        }
    }
    acc
}

fn encode_stream_c(passes: usize, scratch: &mut MeshC, src: &[(u32, u8)]) -> u64 {
    black_box(scratch as *mut MeshC);
    let mut acc: u64 = 0;
    for _ in 0..passes {
        let mut k = 0usize;
        for t in 0..NTRI {
            let base = 3 * t;
            for e in 0..3 {
                let (v, o) = src[k];
                k += 1;
                scratch.nbr[base + e] = v;
                scratch.orient[base + e] = o;
                acc = acc.wrapping_mul(31).wrapping_add(v as u64 * 3 + o as u64);
            }
        }
    }
    acc
}

// -------------------------------------------------------------------- timing

/// Times `f` REPS times after 2 warmup runs. Returns (ns, acc) per rep; all
/// accs must agree across variants for the kernel to count.
fn time_reps(f: impl FnMut() -> u64, reps: usize) -> Vec<(f64, u64)> {
    let mut f = f;
    let mut out = Vec::with_capacity(reps);
    for _ in 0..2 {
        black_box(f());
    }
    for _ in 0..reps {
        let t0 = Instant::now();
        let acc = f();
        let ns = t0.elapsed().as_secs_f64() * 1e9;
        black_box(acc);
        out.push((ns, acc));
    }
    out
}

fn accs_agree(label: &str, sets: &[&[(f64, u64)]]) -> bool {
    let first = sets[0][0].1;
    for set in sets {
        for (_, acc) in set.iter() {
            if *acc != first {
                println!("VALIDATION FAILED at {label}: acc {first} != {acc}");
                return false;
            }
        }
    }
    true
}

fn report(kernel: &str, mesh: &str, var: &str, res: &[(f64, u64)], ops: usize) {
    let mut ts: Vec<f64> = res.iter().map(|x| x.0).collect();
    ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = ts.len();
    let median = if n % 2 == 1 {
        ts[n / 2]
    } else {
        0.5 * (ts[n / 2 - 1] + ts[n / 2])
    };
    let min = ts[0];
    let max = ts[n - 1];
    let mean = ts.iter().sum::<f64>() / n as f64;
    let sd = (ts.iter().map(|x| (x - mean) * (x - mean)).sum::<f64>() / n as f64).sqrt();
    let per_op = median / ops as f64;
    println!(
        "{:<16} {:<9} {:<3} {:>13.1} {:>13.1} {:>13.1} {:>10.1} {:>10.4} {:>16}",
        kernel, mesh, var, median, min, max, sd, per_op, res[0].1
    );
}

fn print_table_header() {
    println!(
        "{:<16} {:<9} {:<3} {:>13} {:>13} {:>13} {:>10} {:>10} {:>16}",
        "kernel", "mesh", "var", "median_ns", "min_ns", "max_ns", "sd_ns", "ns_per_op", "acc"
    );
}

fn main() {
    println!("microbench-representation: wayfinder ticket #7 bench (throwaway)");
    println!("ntri = {NTRI} (periodic {M}x{M} grid), {REPS} reps, median is the deliverable");
    assert_eq!(std::mem::size_of::<OA>(), 16, "A must be 16 B per slot");
    println!(
        "sizeof: OA={} RowA={} RowB={} OC={}",
        std::mem::size_of::<OA>(),
        std::mem::size_of::<RowA>(),
        std::mem::size_of::<RowB>(),
        std::mem::size_of::<OC>()
    );
    let mib = 1024.0 * 1024.0;
    let fa = std::mem::size_of::<RowA>() as f64 * NTRI as f64 / mib;
    let fb = std::mem::size_of::<RowB>() as f64 * NTRI as f64 / mib;
    let fc = (12.0 * NTRI as f64 + 3.0 * NTRI as f64 + 24.0 * NTRI as f64) / mib;
    println!("footprint/variant: A rows {fa:.1} MiB, B rows {fb:.1} MiB, C split {fc:.1} MiB");
    println!("steps/ring={RING_STEPS} sweep_passes={SWEEP_PASSES} chase={CHASE_STEPS} stream_passes={STREAM_PASSES}");

    // runtime-opaque sink for the ring kernels
    let mut srng = Lcg::new(0xDEAD_BEEF);
    let sink: [u64; 3] = [srng.next_u64(), srng.next_u64(), srng.next_u64()];

    print_table_header();
    let mut all_ok = true;

    // ---- ring kernels (no mesh data; ALU only) ----
    {
        let ops = RING_STEPS;
        let ra = time_reps(
            || ring(RING_STEPS, OA { tri: 12345, orient: 0 }, lnext_a, ora, &sink),
            REPS,
        );
        let rb = time_reps(
            || ring(RING_STEPS, OB { raw: ob_raw(12345, 0) }, lnext_b, orb, &sink),
            REPS,
        );
        let rc = time_reps(
            || ring(RING_STEPS, OC { tri: 12345, orient: 0 }, lnext_c, orc, &sink),
            REPS,
        );
        all_ok &= accs_agree("lnext-ring", &[&ra, &rb, &rc]);
        report("lnext-ring", "-", "A", &ra, ops);
        report("lnext-ring", "-", "B", &rb, ops);
        report("lnext-ring", "-", "C", &rc, ops);

        let ra = time_reps(
            || ring(RING_STEPS, OA { tri: 12345, orient: 0 }, lprev_a, ora, &sink),
            REPS,
        );
        let rb = time_reps(
            || ring(RING_STEPS, OB { raw: ob_raw(12345, 0) }, lprev_b, orb, &sink),
            REPS,
        );
        let rc = time_reps(
            || ring(RING_STEPS, OC { tri: 12345, orient: 0 }, lprev_c, orc, &sink),
            REPS,
        );
        all_ok &= accs_agree("lprev-ring", &[&ra, &rb, &rc]);
        report("lprev-ring", "-", "A", &ra, ops);
        report("lprev-ring", "-", "B", &rb, ops);
        report("lprev-ring", "-", "C", &rc, ops);
    }

    // ---- order-dependent kernels ----
    let ids: Vec<u32> = (0..NTRI as u32).collect();
    let mut shuffled = ids.clone();
    let mut rng = Lcg::new(0xABCD_1234);
    for i in (1..NTRI).rev() {
        let j = rng.below((i + 1) as u32) as usize;
        shuffled.swap(i, j);
    }
    let orders: [(&str, bool, Option<Vec<u32>>); 2] = [
        ("grid", true, None),
        ("shuffled", false, Some(shuffled)),
    ];

    // encode stream source pairs (shared, deterministic)
    let mut src: Vec<(u32, u8)> = Vec::with_capacity(3 * NTRI);
    let mut srng = Lcg::new(0x5EED_5EED);
    for _ in 0..3 * NTRI {
        let v = srng.below(NTRI as u32);
        let o = srng.below(3) as u8;
        src.push((v, o));
    }

    for (name, full, perm) in &orders {
        let topo = build_topology(M, perm.as_deref());
        let ma = materialize_a(&topo);
        let mb = materialize_b(&topo);
        let mc = materialize_c(&topo);
        let va = validate_sym_a(&ma);
        let vb = validate_sym_b(&mb);
        let vc = validate_sym_c(&mc);
        println!(
            "mesh order {name}: sym(sym(h))==h on all {} slots: A={} B={} C={}",
            3 * NTRI,
            va,
            vb,
            vc
        );
        all_ok &= va && vb && vc;
        if !(va && vb && vc) {
            std::process::exit(1);
        }

        // ---- sym sweep (allocation-order neighbor deref) ----
        {
            let ops = 3 * NTRI * SWEEP_PASSES;
            let ra = time_reps(
                || sweep(SWEEP_PASSES, mk_a, |h| sym_a(&ma, h), fold_a),
                REPS,
            );
            let rb = time_reps(
                || sweep(SWEEP_PASSES, mk_b, |h| sym_b(&mb, h), fold_b),
                REPS,
            );
            let rc = time_reps(
                || sweep(SWEEP_PASSES, mk_c, |h| sym_c(&mc, h), fold_c),
                REPS,
            );
            all_ok &= accs_agree("sym-sweep", &[&ra, &rb, &rc]);
            report("sym-sweep", name, "A", &ra, ops);
            report("sym-sweep", name, "B", &rb, ops);
            report("sym-sweep", name, "C", &rc, ops);
        }

        // ---- random chase (pointer-chase shaped neighbor deref) ----
        {
            let ops = CHASE_STEPS;
            let seed = 0xC0FF_EE;
            let ra = time_reps(
                || {
                    chase(
                        CHASE_STEPS,
                        OA { tri: 12345, orient: 0 },
                        lnext_a,
                        |h| sym_a(&ma, h),
                        fold_a,
                        seed,
                    )
                },
                REPS,
            );
            let rb = time_reps(
                || {
                    chase(
                        CHASE_STEPS,
                        OB { raw: ob_raw(12345, 0) },
                        lnext_b,
                        |h| sym_b(&mb, h),
                        fold_b,
                        seed,
                    )
                },
                REPS,
            );
            let rc = time_reps(
                || {
                    chase(
                        CHASE_STEPS,
                        OC { tri: 12345, orient: 0 },
                        lnext_c,
                        |h| sym_c(&mc, h),
                        fold_c,
                        seed,
                    )
                },
                REPS,
            );
            all_ok &= accs_agree("chase", &[&ra, &rb, &rc]);
            report("chase", name, "A", &ra, ops);
            report("chase", name, "B", &rb, ops);
            report("chase", name, "C", &rc, ops);
        }

        // ---- decode / encode streams (grid order only) ----
        if *full {
            let ops = 3 * NTRI * STREAM_PASSES;
            let ra = time_reps(|| decode_stream_a(STREAM_PASSES, &ma), REPS);
            let rb = time_reps(|| decode_stream_b(STREAM_PASSES, &mb), REPS);
            let rc = time_reps(|| decode_stream_c(STREAM_PASSES, &mc), REPS);
            all_ok &= accs_agree("decode-stream", &[&ra, &rb, &rc]);
            report("decode-stream", name, "A", &ra, ops);
            report("decode-stream", name, "B", &rb, ops);
            report("decode-stream", name, "C", &rc, ops);

            let mut sa = materialize_a(&topo);
            let mut sb = materialize_b(&topo);
            let mut sc = materialize_c(&topo);
            let ra = time_reps(
                || encode_stream_a(STREAM_PASSES, &mut sa, &src),
                REPS,
            );
            let rb = time_reps(
                || encode_stream_b(STREAM_PASSES, &mut sb, &src),
                REPS,
            );
            let rc = time_reps(
                || encode_stream_c(STREAM_PASSES, &mut sc, &src),
                REPS,
            );
            all_ok &= accs_agree("encode-stream", &[&ra, &rb, &rc]);
            report("encode-stream", name, "A", &ra, ops);
            report("encode-stream", name, "B", &rb, ops);
            report("encode-stream", name, "C", &rc, ops);
        }
    }

    println!(
        "validation: {}",
        if all_ok {
            "all kernel checksums agree across variants A/B/C"
        } else {
            "FAILED (checksum mismatch)"
        }
    );
    if !all_ok {
        std::process::exit(1);
    }
}
