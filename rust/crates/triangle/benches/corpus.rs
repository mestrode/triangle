//! The rung-boundary measurement instrument (idiomatization rung R3,
//! issue #57 PART A): one criterion benchmark per timed workload of the
//! harness corpus - the same 10 case classes x 2 timed sizes (m, l) x
//! 8 option families that tools/oracle/run-bench.sh times (80 benches).
//!
//! Instrument contract (binding for the R3 protocol):
//! - In-process, single-threaded, library-only: each bench builds the
//!   mesh and triangulates it to completion through the public library
//!   API (`Options::parse` + `read_nodes`/`read_poly` + `triangulate`).
//!   No process spawn, no CLI, no oracle. Process startup and file I/O
//!   are deliberately excluded; the end gate (run-bench.sh, absolute
//!   vs the pinned oracle) owns them.
//! - Criterion medians are NOT comparable to run-bench.sh oracle-ratio
//!   numbers, the R1 reference table, or any process-level record.
//! - The suite is a measurement instrument, not a CI job: it is never
//!   wired into CI, and it must not change the library or its public
//!   API. It compiles against the public API only, at the R3 baseline
//!   of record (b9a00b9e, the PR #51 merge) and at master.
//! - The suite is IDENTICAL for the baseline, noise, drift and
//!   candidate runs of the R3 protocol; only the library under test
//!   differs.
//! - Inputs: the harness corpus itself (tools/oracle/generate_corpus.py
//!   output). The corpus root is taken from the TRIANGLE_BENCH_CORPUS
//!   environment variable, defaulting to this repository's
//!   tools/oracle/work/corpus. The manifest (sizes, per-family options
//!   strings with the interpolated area targets, input file names) is
//!   the single source of the timed set, so the bench cannot drift
//!   from what run-bench.sh times.
//!
//! Criterion configuration, tuned for a 2-vCPU shared box so the full
//! 80-bench suite fits in roughly 10 minutes:
//!   sample_size      10   (the minimum criterion accepts)
//!   warm_up_time   500 ms
//!   measurement_time 2 s
//! The exact config is restated in the R3 PR body.

use std::fs::File;
use std::path::{Path, PathBuf};
use std::time::Duration;

use criterion::{Bencher, Criterion};
use serde::Deserialize;

use triangle::api::{read_nodes, read_poly};
use triangle::{triangulate, MeshInput, Options};

/// The timed sizes of the performance gate (bench.py DEFAULT_SIZES);
/// size s stays corpus-only (startup-dominated).
const TIMED_SIZES: [&str; 2] = ["m", "l"];

/// The expected number of timed entries (the bench.py timed set).
const TIMED_ENTRIES: usize = 80;

#[derive(Deserialize)]
struct Manifest {
    cases: Vec<Case>,
}

#[derive(Deserialize)]
struct Case {
    id: String,
    size: String,
    families: Vec<Family>,
}

#[derive(Deserialize)]
struct Family {
    tag: String,
    options: String,
    input: String,
}

/// One fully prepared workload: the input read from the corpus file plus
/// the option string of its family. Preparation is NOT timed; the bench
/// closure times `Options::parse` + `triangulate` to completion per
/// iteration (parse is microseconds against the millisecond-to-second
/// meshing kernels, identical across all four runs of the protocol).
struct Entry {
    id: String,
    options: String,
    input: MeshInput,
}

/// The corpus root: TRIANGLE_BENCH_CORPUS when set, else this
/// repository's tools/oracle/work/corpus.
fn corpus_root() -> PathBuf {
    match std::env::var_os("TRIANGLE_BENCH_CORPUS") {
        Some(dir) => PathBuf::from(dir),
        None => PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../tools/oracle/work/corpus"),
    }
}

/// Build one timed entry: read the corpus input (the same file the
/// harness copies into its run directories) and verify the workload
/// end to end once before any timing (a silently failing entry must
/// not be timed). The reader choice follows the family's input file,
/// exactly like the CLI: `.node` point sets through `read_nodes`,
/// `.poly` PSLGs through `read_poly`.
fn build_entry(corpus: &Path, case: &Case, family: &Family) -> Entry {
    let file = corpus.join(&case.id).join(&family.input);
    let mut input = MeshInput::default();
    let mut firstnode: i32 = 0;
    if family.input.ends_with(".poly") {
        read_poly(
            file.to_str().expect("corpus path is UTF-8"),
            &mut input,
            &mut firstnode,
        )
        .expect("read_poly of a corpus input");
    } else {
        read_nodes(
            file.to_str().expect("corpus path is UTF-8"),
            &mut input,
            &mut firstnode,
        )
        .expect("read_nodes of a corpus input");
    }
    // The CLI hands the parser the option string without its leading
    // dash (main.rs: option_string = &args[1][1..]).
    let options = family.options.strip_prefix('-').unwrap_or(&family.options);
    // One untimed construction call guards the entry.
    triangulate(Options::parse(options).expect("Options::parse"), &input)
        .expect("triangulate of a corpus workload");
    Entry {
        id: format!("{}_{}", case.id, family.tag),
        options: options.to_string(),
        input,
    }
}

fn timed_entries(corpus: &Path) -> Vec<Entry> {
    let manifest_path = corpus.join("manifest.json");
    let manifest: Manifest =
        serde_json::from_reader(File::open(&manifest_path).unwrap_or_else(|e| {
            panic!(
                "cannot open the corpus manifest at {}: {} \\
                 (generate it: python3 tools/oracle/generate_corpus.py, \\
                 or point TRIANGLE_BENCH_CORPUS at a generated corpus)",
                manifest_path.display(),
                e
            )
        }))
        .expect("the corpus manifest is well-formed JSON");
    let mut entries = Vec::new();
    for case in &manifest.cases {
        if !TIMED_SIZES.contains(&case.size.as_str()) {
            continue;
        }
        for family in &case.families {
            entries.push(build_entry(corpus, case, family));
        }
    }
    assert_eq!(
        entries.len(),
        TIMED_ENTRIES,
        "the timed set is the m+l subset of the harness corpus (10 cases x 8 families)"
    );
    entries
}

fn bench_entry(b: &mut Bencher, entry: &Entry) {
    let options = entry.options.as_str();
    let input = &entry.input;
    b.iter(|| {
        triangulate(Options::parse(options).expect("Options::parse"), input)
            .expect("triangulate of a corpus workload")
    });
}

fn main() {
    let corpus = corpus_root();
    let entries = timed_entries(&corpus);
    let mut criterion = Criterion::default()
        .sample_size(10)
        .warm_up_time(Duration::from_millis(500))
        .measurement_time(Duration::from_secs(2));
    for entry in &entries {
        let id = entry.id.as_str();
        criterion.bench_function(id, |b| bench_entry(b, entry));
    }
    criterion.final_summary();
}
