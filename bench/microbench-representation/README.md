# Representation microbench (wayfinder ticket #7)

Throwaway bench comparing the three remaining candidate `(triangle, orientation)`
handle representations for the Rust port of Triangle 1.6 (tagged raw-pointer
handles are ruled out, 2026-11-24). Delivers data for ticket #8; decides nothing.
Never merge this branch (`research/microbench-representation`).

## Variants

- **A, AoS struct handle**: `{ tri: u64, orient: u32 }` (**16 B** per neighbor slot,
  exactly `size_of::<OA>() == 16`, asserted), rows = 3 nbr slots + 3 vertex
  slots = **72 B**, mirroring the C row layout (`t[0..2]` neighbors, `t[3..5]`
  vertices).
- **B, packed u64**: 32-bit index in the high bits, orientation in the low 2
  bits of one u64 (**8 B** per slot, the C slot density), rows = **48 B**.
- **C, SoA split**: neighbor indices (`u32`) and orientations (`u8`) in two
  parallel arrays plus a separate vertex array (**12 + 3 + 24 B per triangle**,
  8-byte-equivalent-or-better density, different locality).

## Hot operations (mirroring the C macros in `triangle.h:703-1090`)

- `lnext` / `lprev` rings (plus1mod3 / minus1mod3): pure handle arithmetic.
- `sym` analog (`decode(tri[orient])`): slot load + unpack, as an
  allocation-order sweep over all slots (triangletraverse shape) and inside
  pseudo-random neighbor-deref walks (splay point location / flip-sequence
  shape: pointer chase, not linear scan).
- decode / encode streams over all neighbor slots (encode is the store-side
  stand-in for a flip's ~6 slot rewrites).

## Mesh

Periodic 724x724 grid of squares split into 2 triangles each:
1,048,352 triangles (~72 / 48 / 39 MiB per variant, exceeds LLC on the build
server). Two pool orders: grid order (2D locality) and Fisher-Yates shuffled
(uniform scatter, worst case). One deterministic LCG drives shuffling and
walker choices.

## Run

    cargo run --release

Prints a table: kernel, mesh order, variant, median/min/max/sd over 20 reps,
ns per op. All three variants must produce identical checksums in every kernel
(asserted; exit code 1 otherwise), and `sym(sym(h)) == h` is validated over the
whole mesh before timing.

## Caveats

- Ring kernels carry a constant un-foldability floor (fold chain through a
  runtime-opaque 3-entry table); read deltas, not absolutes.
- Chase kernels include a per-step LCG, constant across variants; memory
  behavior dominates there.
- Shared-vCPU build server: expect noise; medians over 20 reps, spreads are
  printed.
- Read-heavy bench; mutation only via the encode stream.
- No `-march`/`-C target-cpu` overrides (default C-build conditions on the
  same machine).

Numbers, hardware, and analysis: ticket #7 comment.
