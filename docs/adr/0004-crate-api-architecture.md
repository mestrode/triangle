# Crate and API architecture: rust/ workspace, module mirror, and the day-one idiomatic shell

Status: accepted 2026-11-24; decided in the wayfinder ticket Crate and API architecture of the Rust port (lib plus bin) (#10), on the port strategy (ADR 0001, ticket #6), the core data model (ADR 0002, ticket #8), the UB inventory (#4, branch research/ub-inventory), the oracle harness and candidate contract (#13), the performance gate (#11), and the open parallelism ticket (#12, read-only for this decision). Glossary terms added: module map, error funnel, candidate contract.

## Decision inputs (settled, not re-derived here)

- The port is a hybrid (ADR 0001): a function-for-function transliterated core under an idiomatic API shell from day one, then four corpus-gated idiomatization rungs (1: accessor invariants, 2: error flow, 3: storage typing, 4: renamed/reorganized modules). Serial byte-identity against the pinned C oracle over the harness corpus is the permanent gate (#13).
- The data model (ADR 0002): packed u64 slot handles, unpacked register handles, u64-word rows behind one private accessor chokepoint, zero stage-1 unsafe, a binding pool interface contract for #9. Type sketches in the ADR 0002 appendix use the module names `handles.rs` and `slots.rs` and defer module placement to this ADR (#8: "Module/file placement of handles vs slots code: #10's module map").
- Internal error plumbing stays transliterated (mirrored C codes) until rung 2 (#6 constraint on #10); unsafe concentrates behind the accessor/pool chokepoints (rung 1); idiomatically renamed internals land per rung 4 (#6).
- The Rust CLI must be invocable like the C triangle-cli (the #13 candidate contract, through the harness's `candidate` symlink); argument compatibility is pinned in the I/O phase, not here. The thread-count knob surfaces as the `TRIANGLE_THREADS` environment variable for now (#11), spelled exactly so; `TRAINGLE_THREADS` is a typo that no released artifact may accept. It becomes a CLI flag in the I/O phase once #12 rules the MT design.
- Parallelism (#12) is open: the module and API design below neither assumes nor forbids internal multithreading.
- User constraints: idiomatic Rust API (Result/errors, no libc types at the boundary), no C ABI, no bindings, core first, I/O and CLI later, 64-bit only.

## 1. Workspace and crate layout

The Rust workspace lives in a new top-level `rust/` directory; the C sources (`src/Triangle`, `src/examples`), the CMake build (`CMakeLists.txt`, `src/cmake`), and `tools/oracle` are untouched by it:

    rust/
    ├── Cargo.toml                # workspace root: members crates/*, resolver = "2"
    └── crates/
        ├── triangle/             # the lib crate (this port's core deliverable)
        │   ├── Cargo.toml        # features: default = ["acute"]; no dependencies day one
        │   └── src/…             # module map in section 4
        └── triangle-cli/         # the bin crate, executable `triangle-cli`
            ├── Cargo.toml
            └── src/main.rs

- Lib crate: `triangle`. The name mirrors the C library and the map's destination ("a Rust library plus binary"); the CLI's `-h` output and the oracle build call the same thing. Publishing (crates.io naming conflicts) is not decided here; if the crate is ever published under a different name, the lib target stays `triangle`.
- Bin crate: `triangle-cli`, building an executable named `triangle-cli`, the name the #13 candidate contract and the ADR 0001 stage plan already use ("the Rust CLI (triangle-CLI interface)"). The binary is invoked through the harness's `candidate` symlink, so the filename stays exactly this.
- `rust/crates/*` (not `rust/triangle`, `rust/cli` directly) leaves room for later workspace-local helper crates (for example a corpus-runner utility for the gates) without reshuffling the tree.
- The workspace pins zero unsafe code for stage 1 mechanically: `[workspace.lints.rust] unsafe_code = "forbid"` and every crate opts in with `[lints] workspace = true`. A later rung-gated unsafe change (section 6) relaxes this consciously, module-locally, with a deviation-register entry.
- `rust/target/` is git-ignored via an appended line in the root `.gitignore`; `rust/Cargo.lock` is committed (end-product workspace).
- 64-bit only is enforced, not implied: `lib.rs` carries `#![cfg_attr(not(target_pointer_width = "64"), compile_error!(...))]` per the map's scope ruling.

Skeleton: the PR beside this ADR commits exactly this layout as an empty skeleton (Cargo manifests plus stub `.rs` files carrying per-module doc comments and nothing else, verified with `cargo check` under both feature settings). No Rust implementation code.

## 2. Feature gates

Exactly one feature gate exists on day one:

- `acute` (default on): the aCute refinement (`-U` max-angle family), the mirror of the C `NO_ACUTE` compile switch, inverted. `--no-default-features` reproduces a C `NO_ACUTE` build. The whole `acute` module is `#[cfg(feature = "acute")]`, mirroring `#ifndef NO_ACUTE`. The aCute knob in the Options surface (`max_angle`, the `-U` switch) exists only with the feature; without it, parsing a `-U` option must fail exactly as the C `NO_ACUTE` build fails it (`vwxy`-style switch rejection, transliterated with `parsecommandline`). The aCute pool (`acutepool_init` etc.) lives in the acute module and consumes the #9 pool interface (contract item 8).
- Nothing else earns a feature flag day one. Weighed and rejected for now:
  - A `file-io` gate (the C `NO_FILE_IO`). File I/O is a whole later phase; its crate shape (one module, a feature, or a second crate) is decided when that phase is ticketed. A speculative feature now would freeze a public surface nobody has reviewed against real code.
  - A `threads`/MT feature: internal parallelism is #12's design and lands as core behavior behind the same API, not behind a feature. The scaling axis maps to `TRIANGLE_THREADS` (env now, CLI flag in the I/O phase), which is runtime input, not a compile-time surface.
  - CDT_ONLY / REDUCED variants: the fork always builds the full core; C transliterates those with `#ifdef`, Rust needs none.
  - `std`/`no-std`: std only; the pools (Vec per #9's recommended shape) and file I/O both assume std. Not a port goal.

## 3. Compile-time identity pinning

- `REAL` is `f64`, `int` at the boundary of transliterated formulas is `i32` inside the core (per ADR 0002's word formulas), `long` sites are `i64` where transliterated semantics need a 64-bit accumulator (the C `long` on the port's 64-bit targets); indices surfacing in the API are `usize`. No libc types appear at the public boundary.
- FP parity: the C oracle is pinned with `-ffp-contract=off` (#13); Rust default release codegen matches. `cos`/`sqrt` in the option-derived formulas (`goodangle`, `offconstant`) call the same system libm on both sides of the differential run; this is parity-neutral existing behavior, not a deviation.
- The LCG seed moves from C's `_Thread_local randomseed` into an owned per-run field on the core state (context-owned, reentrant per instance). The sequence is unchanged; ownership moves because thread-locality in Rust would tie determinism to the caller's thread, which would preclude #12 (deviation D10).

## 4. Module map (the artifact)

The lib crate's modules mirror the C compilation units; `triangle.c` splits along its own section banners into named Rust modules. The complete, commitable assignment is the function-mapping table (`docs/function-mapping.md`, seeded by this ticket): one row per C function (C site `file:line`, Rust module, stage/rung notes). Stage-1 implementation tickets fill the last gaps and keep it current; the module assignment itself is decided here:

| C unit (src/Triangle) | Rust module(s) under `rust/crates/triangle/src/` | Stage |
| --- | --- | --- |
| triangle.h (encode/decode, handle macros, types) | `handles.rs` (register handles, packing, macro methods) + `slots.rs` (storage-touching methods, the chokepoint) per ADR 0002 | S1 |
| triangle.c (8313 lines, 103 functions) | split: `util.rs` (interpolate), `options.rs` (parsecommandline, behavior_update), `debug.rs` (printtriangle, printsubseg), `pool.rs` (pool machinery: #9's landing zone), `mesh.rs` (mesh lifecycle: pool init/dealloc/traverse wrappers, dummyinit, constructors), `quality.rs` (quality testing, self-checks, statistics), `locate.rs`, `transform.rs`, `delaunay.rs` (sorting, div-and-conq, incremental, sweepline), `construct.rs` (reconstruction, segment insertion, skeleton, holes), `refine.rs` (quality/aCute refinement engine, highorder), `ioarrays.rs` (the C "Array I/O routines": transfernodes, array writers) | S1 (ioarrays writers late S1) |
| acute.c | `acute.rs` behind `#[cfg(feature = "acute")]`, pool functions consuming the #9 contract | S1, feature-gated |
| predicates.c | `predicates.rs` (the inert x87 control-word code is dropped, per #3 / deviation D5) | S1 |
| eps_writer.c | `eps.rs` | I/O phase |
| triangle_io.c | `io.rs` (all `file_*` readers/writers, api file entry points) | I/O phase |
| triangle_helper.c | dissolves: `options.rs` (triangle_check_behavior), `input.rs` (triangle_check_triangleio, triangle_restore_pointmarkers); triangle_check_context has no counterpart (ownership, D8) | S1 (shell) |
| triangle_api.c (the idiomatic-shell reference) | `api.rs` (entry points, the `Mesh` handle, statistics/quality/inspection/memory surface) + `options.rs` (options parsing/validation) + `error.rs` (status funnel) + `lib.rs` (version); context/free/io functions dissolve per the mapping notes | S1 (shell) / IO |
| triangle_config.h (TRI_* status codes) | `error.rs`: `pub(crate) InternalStatus` mirror + the public `Error` taxonomy + the funnel | S1 (shell) |
| triangle_version.h | `lib.rs`: `VERSION`, `HAS_ACUTE` constants mirroring `triangle_version()` | S1 (shell) |

Day-one public API surface of the crate: `pub mod api, options, input, error;` re-exported at the crate root (`triangulate`, `Mesh`, `Options`, `MeshInput`, `MeshOutput`, `Statistics`, `Quality`, `Error`, `Result`, `Unsuitable`, version constants). Everything else in the table is `pub(crate)`. The crate never exposes otri/osub, row ids, pools, or the chokepoint types - ADR 0002's no-escape rule is also the public API rule.

## 5. Public API shape of the lib crate

### 5.1 Options

`Options` is the public replacement of `struct behavior`; it groups the switch families the C CLI parses today:

- Mesh options string families (`parsecommandline`): `-p` (pslg), `-r` (refine), `-qN` (quality with min angle, default 20.0), `-UN` (aCute max angle, default 140.0, `acute` feature only), `-a`/`-aN` (area constraint: variable or fixed), `-u` (user test + the `Unsuitable` callback), `-A` (region attributes), `-c` (convex hull segments), `-w`/`-W` (weighted/regular), `-j` (jettison), `-z` (zero-based numbering), `-n` (neighbors), `-B` (suppress bounds), `-O` (ignore holes), `-X` (no exact arithmetic), `-D`/`-L` (conforming Delaunay), `-i` (incremental), `-F` (sweepline), `-l` (no alternating cuts: Dwyer off), `-s` (split segments), `-o2` (element order), `-Y` (no boundary splitting, repeatable level), `-S` (Steiner limit).
- The CLI-only output families (`-e` edges, `-m` eps, `-N`/`-P`/`-E`/`-I`/`-Q`/`-V`/`-C` or their C namesakes) stay in the bin crate (section 7); only `-n` crosses (it selects the neighbor output surface).

Shape decisions:

- `Options::new()` gives the exact `parsecommandline("")` defaults (`dwyer` on, `steiner` unlimited, `firstnumber` 1, `maxarea` -1.0 decimal, ...).
- `Options::parse(options_str)` mirrors the C character loop of `parsecommandline` 1:1 (defaults, inline number parsing, derived-field tail: `usesegments`, `cos(goodangle)`, `offconstant`, `vararea`/`regionattrib`/`weighted` corrections). It exists because the CLI passes an options string (candidate contract) and because it is the cheapest behavioral parity test for the whole option family; it returns `Result<Options, Error>` instead of mutating a struct in place and exiting on numeric garbage.
- Derived fields (`goodangle`, `maxgoodangle`, `offconstant`, `usesegments`) are not public setter surface; `validate()` (the `options.rs` validation mirror of `triangle_check_behavior` + the derived-field tail) computes them and is called by every entry point. Validation rules mirror the C ones: `fixedarea && maxarea <= 0` is `Error::Options`, as are the C-implied invariants that survive ownership (anything C silently accepted stays silently accepted; parity first).
- The options builder mirrors `struct behavior` field groups one-to-one so the mapping table can say, per field: C `b->poly` = `Options::pslg`. Builder methods are plain setters with the switch letter documented per method. No new options are invented; nothing C parses is dropped.
- The normalized internal form `Behavior` (`pub(crate)`, field-mirror of `struct behavior` incl. the derived fields) is what the transliterated core consumes; transliterated functions take `&mut Behavior` exactly where C takes `behavior *b`.

### 5.2 Input and output types

- `MeshInput` (in `input.rs`) mirrors the input half of `triangleio`: points (coordinates, per-point markers, per-point attributes), segments (endpoint pairs + markers, for `-p`), holes, regions, and the refine-path inputs (`trianglelist`, `trianglearealist` for `-r`). Owned `Vec`s, no libc types, validated by the transliterated `triangle_check_triangleio` semantics (`index in firstnumber..firstnumber + points` etc., returning `Error::Input` where C returns `TRI_FAILURE`/`TRI_INPUT`). `triangle_restore_pointmarkers` (C: undoes the marker-rewriting optimization before an output that needs markers) stays an internal step of the output path; transliterated, in `input.rs`.
- `MeshOutput` (in `api.rs`) mirrors the output half: numbered `Point2` list, element corners + attributes, segment list, edges, per-element neighbors. It is produced by the transliterated array-writer run in `ioarrays.rs` and returned by `Mesh::output`; the file writers of the I/O phase serialize exactly this.

### 5.3 Entry points and the mesh handle

`api.rs` carries the idiomatic mirror of triangle_api.c's surviving surface; the context pointer becomes owned values:

    pub fn triangulate(options: Options, input: &MeshInput) -> Result<Mesh, Error>   // triangle_mesh_create
    impl Mesh {
        pub fn load_mesh(options: Options, input: &MeshInput) -> Result<Mesh, Error> // triangle_mesh_load
        pub fn refine(&mut self) -> Result<(), Error>                                // triangle_mesh_refine
        pub fn output(&self, edges: bool, neighbors: bool) -> Result<MeshOutput, Error> // triangle_mesh_copy
        pub fn statistics(&self) -> Statistics                                       // triangle_mesh_statistics
        pub fn quality(&self) -> Result<Quality, Error>                              // triangle_mesh_quality
        pub fn check_mesh(&self) -> usize                                            // triangle_check_mesh (failed count)
        pub fn check_delaunay(&self) -> usize                                        // triangle_check_delaunay
        pub fn memory_bytes(&self) -> usize                                          // triangle_memory
    }

- `Mesh` owns the pools, the row shapes, the normalized `Behavior`, and the run state (RNG seed included, section 3). `Send`/`Sync` bounds on `Mesh` are deliberately not pinned today; #12 owns any that become necessary to state.
- `Statistics` mirrors `struct statistics` (vertices, undeads, triangles, hullsize, subsegs, edges, bounds as `BoundingBox { xmin, ymin, xmax, ymax }`), `Quality` mirrors `struct quality` (shortest/longest, smallest/biggest area and angle, minaltitude, worstaspect, `angle_table: [usize; 18]`, `aspect_table: [usize; 16]`). Both are plain data; both fill via the transliterated `quality_statistics`/statistics fields, exact C arithmetic.
- Error taxonomy in section 6 covers `map_err` sites; inspection methods (`check_mesh`, `check_delaunay`) return failure counts mirroring C (already non-error semantics), not `Result`.
- `version()`/`VERSION` mirror `triangle_version(int[4])`: `pub const VERSION: (u32, u32, u32) = (1, 7, 0)` and `pub const HAS_ACUTE: bool = cfg!(feature = "acute")` - the version follows `src/CMakeLists.txt`'s 1.7.0 (the fork's own numbering of the Triangle 1.6 + aCute codebase); the C `TRIANGLE_VERSION` string's `_acute` suffix is the CLI/HARNESS-rendered form, pinned in the I/O phase.

### 5.4 The callback (`triunsuitable_user_func`)

C: `int (*triunsuitable_user_func)(vertex trisorg, vertex tridest, vertex triapex, REAL area)`, invoked in `testtriangle`; raw vertex pointers, nonzero return = refine further. Rust:

    pub struct VertexSample { pub x: f64, pub y: f64, pub marker: i32 }   // Copy, values only
    pub trait Unsuitable {
        /// Returns true when the triangle must be refined further (C: nonzero).
        fn is_unsuitable(&mut self, org: VertexSample, dest: VertexSample,
                         apex: VertexSample, area: f64) -> bool;
    }

- Object-safe, one method, `&mut self` (stateful sizing functions are the norm), stored in `Options` as `Option<Box<dyn Unsuitable>>`.
- The callback receives copied vertex data, not references: ADR 0002's chokepoint contract forbids storage references escaping, so raw vertex handles can never be handed to user code (deviation D9; no callback can corrupt a row). `VertexSample` starts with position and marker only; attribute access grows only if a parity-relevant need appears (recorded, not assumed).
- Serial path today: invoked exactly where the C macro path invokes `triunsuitable_user_func` inside `testtriangle`, identically ordered, identically decisive; it returns `bool` where C's contract means `true == nonzero`.
- MT (per #12, deliberately open): the trait carries no `Send`/`Sync` supertraits today and the call-out is not routed through any synchronization. #12's ruling determines whether implementations must be Send/Sync, get called from a single coordinating thread, or get called under a lock; every one of those rulings keeps this trait shape. The seam stays inside `testtriangle` (`quality.rs`), so the ruling lands in one function.

### 5.5 Statistics surface

`Statistics`, `Quality`, `BoundingBox`, `memory_bytes()`, `check_mesh()`, `check_delaunay()` (section 5.3) are the whole statistics surface day one. The CLI's verbose text reports (`print_statistics` in the C example CLI) are bin-crate rendering of these types, pinned in the I/O phase.

## 6. Error taxonomy and the funnel seam

The public taxonomy is distilled from the remaining `triexit` sites and the `TRI_*` status-code surface (the #4 inventory plus `triangle_config.h`):

    pub enum Error {
        Options(String),            // TRI_OPTIONS: invalid or conflicting switches
        Input(String),              // TRI_INPUT + TRI_FAILURE from input validation: bad coordinates, indices, counts
        SegmentRecovery(SegmentRecoveryKind), // the segment-insertion failure family, see below
        OutOfMemory,               // triexit("Out of memory"), triangle.c:324-325
        Internal,                  // internalerror(), triangle.c:349-354 (a port bug; 5 call sites in C)
        Io(std::io::Error),         // TRI_FILE_OPEN / TRI_FILE_READ, filled by the I/O phase
    }

- `SegmentRecoveryKind { FindDirection, Split, Intersect, Insert, Scout, Cycle }` carries the mirrored internal codes the segment machinery propagates through `*status` out-params today (`TRI_FIND_DIRECTION`, `TRI_SEG_SPLIT`, `TRI_SEG_INTERSECT`, `TRI_SEG_INSERT`, `TRI_SEG_SCOUT`, `TRI_CYCLE`): a PSLG whose segments cross or resist recovery. No public information is lost: the kind is the exact C code.
- `TRI_NULL` dissolves: a null `context*`/`behavior*` cannot exist in an owned API (deviation D8, with `triangle_check_context` and `triangle_free`).

The funnel seam (defined now, built in stage 1, widened in rung 2):

1. `error.rs` defines `pub(crate) enum InternalStatus` with the exact C discriminants for the mirrored codes (`Ok = 0, Failure = -1, Options = -2, Input = -3, FindDirection = -4, SegSplit = -5, SegIntersect = -6, SegInsert = -7, SegScout = -8, FileOpen = -9, FileRead = -10, Cycle = -11`) plus two funnel extensions with no C number, `Oom` and `InternalError`, because C exits the process on those two conditions instead of returning. `FileOpen`/`FileRead` stay dormant until the I/O phase.
2. Until rung 2, every transliterated core function that returns a status in C returns `InternalStatus`, and every `int *status` out-param becomes `&mut InternalStatus` with identical propagation (checked in `triangle_api.c` style, fixed-point; no panics; the C `< 0` spreads become matches on the enum).
3. The two exit sites transliterate into the funnel as returned statuses: `internalerror()` becomes `fn internalerror() -> InternalStatus` and every C call site reads `return internalerror();` (C's noreturn-by-exit becomes noreturn-by-construction: the sites sit on paths C never continues past). The alloc wrapper's `Out of memory` site becomes a returned `InternalStatus::Oom` from the pool path (item 4).
4. Pool allocation and OOM: the C fatal `Out of memory` site must be reachable as `Error::OutOfMemory`, not a process abort. The pools' primitive allocation therefore grows a fallible route (`try_reserve`-style) so OOM returns `InternalStatus::Oom` through the pool interface. [The exact fallibility framing (which layer owns it) is #9's; if #9's implementation rules otherwise in ADR 0003, the resolved shape is recorded in the deviation register - the funnel target stays `Error::OutOfMemory` either way.]
5. The single translation point: `impl From<InternalStatus> for Error` (plus the io wrappers later) runs only at the `api.rs` boundary, exactly where `triangle_api.c` inspects returned statuses. A unit test funnels every negative code to its public variant; the corpus keeps the funnel honest end to end.
6. Rung 2 replaces the internal mirrors with idiomatic internal Results; the public `Error` and the funnel's translation table survive unchanged (ADR 0001's error-flow rung does not reopen the taxonomy).
7. The lib never prints and never exits: C's stdout messages on the fatal sites survive verbatim as `Error` payloads (and as comments at the transliteration sites); console etiquette and parity of the CLI's stdout chatter is the bin crate's I/O-phase concern.

## 7. The bin crate (sketch only)

`triangle-cli` exists to satisfy and eventually fulfill the #13 candidate contract: an executable invocable as `./candidate <options> <input>` inside the harness run directory, producing the standard `input.1.*` outputs, exit 0 on success.

- Day one: a stub `main.rs` (no argument parsing beyond what documents the plan) plus the crate manifests. It is not a candidate yet; the I/O phase makes it one.
- The thread-count knob: the CLI reads `TRIANGLE_THREADS` (exact spelling; `TRAINGLE_THREADS` is rejected as a typo, not silently honored) and passes it to the lib once #12 lands the consumer. Today: parsed, validated as a positive integer, and unused, one documented line, so the env contract of #11's MT axis exists from the first candidate onward. It becomes a CLI flag in the I/O phase.
- Argument compatibility with the C triangle-cli (the `parsecommandline_legacy` families: `-e`, `-m`, `-n`, `-B`, `-P`, `-N`, `-E`, `-I`, `-O`, `-C`, `-Q`, `-V`, `-h`; file-name derivation; iteration suffixes) is explicitly pinned in the I/O phase, not here. The sketch promises only the shape above.
- Exit codes: 0 success; any `Err` exits 1, message to stderr (the CLI ambition is the C `check()` site semantics; exact etiquette also I/O phase).

## 8. Documentation on day one

The crate docs promise only what exists, and everything that exists is documented:

- `lib.rs` crate docs: what the crate is (the verified port of Triangle 1.6 + aCute per ADR 0001), the byte-parity guarantee statement (outputs byte-identical to the pinned C oracle over the harness corpus on the serial path, permanent gate), the 64-bit-only note, the `acute` default-on feature note, and the `TRIANGLE_THREADS` note (CLI-level, sourced from #11/#12).
- Doc examples: fenced examples for `Options`/`Options::parse`/`Error` become compiled doctests with the stage-1 tickets that implement those surfaces (part of each ticket's definition of done). Examples for not-yet-existing surfaces are written as plain code fences now and convert to doctests when the surface lands; nothing in rustdoc claims behavior that stage-1 code does not already exhibit.
- Every stub module carries the header pointing at the mapping table and its C section; no placeholder text claims completion.

## Considered alternatives

- Workspace at the repository root vs `rust/`: rejected root placement; cargo would own the top of a C repository, its target dir would sit next to the C build, and the oracle/CI tree would have to grow ignores around it. `rust/` keeps the repo C-first until the port is the main event.
- Single crate with lib + bin targets vs two crates: rejected the single crate; the bin needs no reason to recompile the lib's internals and the CLI has a different stability cadence (argument compatibility is I/O-phase-pinned by #13's contract); the two-crate workspace also lets the perf gate grab the binary alone.
- Enum-heavy options (every switch its own sub-enum) vs the behavior-mirror builder: rejected the heavy form; it obscures the C field mapping the transliteration depends on and invents structure the corpus families never exercise. `Options` stays one struct with the deviation register and mapping table holding the C correspondence.
- `Result`-returning transliterated core now vs mirrored `InternalStatus`: settled by ADR 0001/#6 (codes stay mirrored until rung 2); using idiomatic Results inside the core in stage 1 would be an unmappable hybrid that breaks bisectability.
- `Box<dyn Unsuitable + Send>` (or `+ Send + Sync`) today: rejected adding supertraits now; it would forbid single-threaded stateful callbacks for an MT ruling (#12) that has not been made. The trait deliberately ships without supertraits; #12 adds what its ruling requires, and the trait-shape survives all rulings on the table.

## Consequences

- The stage-1 implementation tickets can be cut per module (`delaunay.rs` is independently landable after `handles.rs`/`slots.rs`/`mesh.rs`/`pool.rs`, for example) with the mapping table as the shared assignment and the corpus as the gate.
- The deviation register stands at `docs/deviation-register.md` (this ticket seeds it with the deviations decided so far - D1 through D11 - and defines its format: stable IDs, C site, deviation, rationale, gate; never renumbered, superseded entries keep their ID with a superseded-by note).
- API-surface freeze: after stage 1 lands, the public surface of section 5 is the v0 compatibility line; changes require their own ticket against the map.
- The bin crate remains a sketch; the I/O phase owns the file names, the writer formats, and the arg surface; #12 owns every thread-parallel decision; #9 (ADR 0003) owns the pool implementation inside the interface contract.
- CI: the oracle workflow keeps running the harness self-check; nothing in the rust/ skeleton affects it. When the Rust CLI exists, the differential job switches per #13's wiring, and the `candidate` symlink points at `rust/target/release/triangle-cli`.

## Explicitly not decided here

- File I/O crate shape (feature, module split between `io.rs`/`eps.rs`/writers) - pinned by the I/O-phase ticket.
- Exact CLI argument compatibility surface and file-name conventions - pinned by the I/O-phase ticket; today only the candidate contract and the `TRIANGLE_THREADS` env spelling are binding.
- Send/Sync supertraits on `Unsuitable` and `Mesh` - awaiting #12's parallel-design ruling; the trait shape is designed to survive it.
- Pool OOM internals (fallibility detail) - #9's ADR 0003 within the funnel constraint stated here.
- crates.io publishing/name of the lib crate.
- pub-usable internal inspection APIs (dumping a `Mesh` without output marshalling) - not excluded, deferred until parity separates from debugging need.
- `pub struct Otri`-exposure type surfaces; stage-1 core types are crate-internal; any later exposure would go through ADR 0002's rung-3 typed rows.
