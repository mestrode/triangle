# ADR 0006: Extract the triangle-io interchange crate

Date: 2026-11-28
Status: accepted (issue #70; the rung's gate results are the PR body)
Related: ADR 0004 (crate layout, public API, error funnel), the
deviation register (D31's read-seam equivalence classes, unchanged by
this extraction), docs/function-mapping.md (the extraction migration
table)

## Context

The I/O phase landed all of triangle_io.c in the triangle crate's
io/mod.rs: the parse seam (readline/findfield/strtol/strtod), the
readers, the formatter seam, and the six Mesh-bound writers. A third
party that only wants to read or write Triangle interchange files
(.poly, .node, .ele, .edge, .neigh, .area) has no way to do so
without depending on the whole meshing core - its public API, its
pools, its feature gates.

## Decision

1. A new workspace crate, `rust/crates/triangle-io`, owns the
   interchange file formats: parsing and serialization for all six
   formats, exposed over neutral, self-contained types (module
   `types`: `NodeFile`, `PolyFile`, `EleFile`, `EdgeFile`,
   `NeighFile`, `AreaFile`, `Point2`, `Hole`, `Region` - owned
   structs describing file contents only: counts, dimensions,
   attribute counts, marker flags, firstnumber, and the flat data
   lists).
2. The crate is pure std (`io::Read`/`io::Write` based), zero
   dependencies, zero unsafe (the workspace `unsafe_code = "forbid"`),
   no libc.
3. The dependency direction is `triangle -> triangle-io` only.
   triangle-io never depends on (or references) the meshing crate; no
   `MeshInput`, `Mesh`, or `Behavior` type appears in its API.
4. The triangle crate keeps: the `api.rs` read wrappers (signatures
   frozen, v0 surface), which open the file, call triangle-io's
   readers, and convert the neutral results into `MeshInput` at a
   private seam; the Mesh-bound writers (`file_write_*`), which keep
   the traversal orchestration (numbering, jettison gates, the D29
   edge-walk order, the -n overlay) and call triangle-io's
   header/line/trailer emitters for the bytes; `io/eps.rs` (EPS is
   mesh presentation, not an interchange format); `io/arrays.rs`;
   the MT seam.
5. The v0 public surface of the triangle crate is item-for-item
   unchanged: `formatter.rs` becomes a re-export module
   (`triangle::formatter::{g, g_padded, g_padded_left}` re-exported
   from `triangle_io::formatter`), so the CLI's renderers and the
   EPS renderer compile unchanged; the `api.rs` signatures and the
   public `Error` taxonomy are untouched (the seam maps
   `triangle_io::Error` onto the same variants with the same
   payloads).
6. The C anchors are unchanged: the 1:1 transliteration mapping
   continues in docs/function-mapping.md, which gains a migration
   table for every moved function (old module path -> new crate path,
   C anchor unchanged). The .edge and .neigh readers and the .area
   writer have no C counterpart (C only writes .edge/.neigh, only
   reads .area); they parse or compose the writer/reader grammar
   under the readers' own conventions, pinned by construction tests,
   never via the C oracle binary.
7. The crate is version 0.1.0 with `publish = false`. Crates.io
   publication and the crate's final name are a separate maintainer
   decision (the architecture spec's open item).

## Rationale: neutral types

The readers could have stayed `MeshInput`-bound behind a trait, or
the meshing types could have moved down into the format crate. Both
would couple the format surface to the meshing core in one direction
or the other. Neutral types make the file formats a property of the
file formats: a third party round-trips a .poly file without linking
the mesher, and the triangle crate converts at a private seam exactly
where its `api.rs` wrappers already owned the file open. The
conversion allocates small lists per call (the poly hole/region
vectors); the interchange paths are not mesher-hot, and the
byte-parity harness (180/180) pins observable behavior.

## Consequences

- Zero behavior change is the gate: the 180-run corpus stays
  byte-identical and stdout-identical against the pinned oracle; the
  moved readers keep their exact semantics (firstnumber capture at
  i == 0, marker and attribute presence rules, missing-field
  defaults, the TRI_FILE_READ and TRI_FAILURE error payloads). The
  deviation register gains no entry. One seam-level nuance is
  disclosed in the PR body: on malformed files the readers now fill
  the neutral structs first, so a failed read leaves `MeshInput`
  untouched where the in-place fill could leave partial lists behind;
  the error variants, payloads, and every success state are
  identical, and no test or doc pinned the old partial fill.
- The workspace test total does not decrease: the moved unit tests
  relocate mechanically with the code, and net-new construction
  tests in triangle-io pin the format behavior (exact
  header/line/trailer bytes, round-trips over the firstnumber and
  marker matrices).
- CI covers the new crate without workflow edits: the rust
  workflow's paths filter and workspace-wide -p flags include it;
  the byte-parity job is untouched at 180 runs.
