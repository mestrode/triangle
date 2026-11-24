//! triangle-cli: the CLI binary of the Rust Triangle port.
//!
//! Stage-1 stub per the stage-1 foundation PR: argument echo and exit 0,
//! nothing else. Its only job today is to exist as the differential
//! harness's candidate executable (tools/oracle, invoke via run-harness.sh
//! --candidate) so the plumbing from harness to port is proven wired and
//! documented before the I/O phase lands (byte parity is NOT expected at
//! this stage; the harness wiring proof is recorded in that PR).
//!
//! Notes for the arriving phases:
//! - Candidate contract: invoked inside a prepared run directory with the
//!   ported option switches; the option-switch families, file-name
//!   derivation and iteration suffixes are pinned by the I/O-phase ticket.
//! - The thread-count knob for the future internal multithreading is the
//!   TRIANGLE_THREADS environment variable (spelled exactly so). Its
//!   validation returns with the I/O phase; this stage must not exit
//!   nonzero, so the stub performs no validation at all.

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (major, minor, patch) = triangle::VERSION;
    println!(
        "triangle-cli (stage-1 stub, Triangle {}.{}.{}, aCute {}): {}",
        major,
        minor,
        patch,
        triangle::HAS_ACUTE,
        args.join(" ")
    );
    // exit 0: the harness records candidate invocations before comparing
    // outputs, and this stub has no failure mode yet.
}
