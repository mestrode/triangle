//! triangle-cli: the sketch of the Rust port's CLI program.
//!
//! Wayfinder ticket #10 pins only the shape (ADR 0004 section 7):
//!
//! - Candidate contract (tools/oracle, ticket #13): invoked as
//!   ./candidate <options> <input> inside a prepared run directory, writing
//!   the standard input.1.* outputs, exit 0 on success. The compatibility
//!   surface itself (the option-switch families, file-name derivation,
//!   iteration suffixes) is pinned by the I/O-phase ticket, not here.
//! - The thread-count knob for the future internal multithreading is the
//!   TRIANGLE_THREADS environment variable (as wired by the performance
//!   gate, ticket #11) - spelled exactly so; TRAINGLE_THREADS is a typo and
//!   is rejected, never honored. It becomes a CLI flag in the I/O phase
//!   once ticket #12 rules the parallel design.
//!
//! Until the I/O phase lands, this main is deliberately a stub: exit code 1,
//! nothing else.

fn main() {
    if let Ok(threads) = std::env::var("TRIANGLE_THREADS") {
        if threads.parse::<usize>().map(|n| n == 0).unwrap_or(true) {
            eprintln!(
                "triangle-cli: TRIANGLE_THREADS must be a positive integer (got {threads:?})"
            );
        }
    }

    // Keep the lib dependency honest while this main is a stub: the linked
    // core exposes its version constants today (lib.rs); the entry points
    // (api.rs) arrive with the stage-1 shell and replace this statement.
    let _linked_core = (triangle::VERSION, triangle::HAS_ACUTE);

    eprintln!(
        "triangle-cli (Rust port): not implemented yet; see docs/adr/0004-crate-api-architecture.md"
    );
    std::process::exit(1);
}
