//! triangle-cli: the CLI binary of the Rust Triangle port. The real
//! main of the I/O phase: the transliteration of
//! src/examples/triangle-cli/main.c (the run flow) and util.c
//! (parsecommandline_legacy, syntax, info, check_behavior,
//! print_statistics, check, finishfile, the write_* wrappers,
//! check_mesh), driving the landed library surface (Options,
//! triangulate, Mesh, the triangle_read_* / triangle_write_* file
//! wrappers of triangle::api).
//!
//! Candidate contract (tools/oracle): invoked inside a prepared run
//! directory as `./candidate` with the ported option switches; the
//! .node / .ele / .poly / .edge / .neigh bytes are gated byte-for-byte,
//! and since issue #66 the captured stdout is gated byte-for-byte too
//! (the non-verbose chatter); exit 0 expected.
//!
//! C anchors: main.c:8-114 (the flow), util.c:17-78 (syntax),
//! util.c:86-103 (info), util.c:112-274 (parsecommandline_legacy),
//! util.c:276-299 (check_behavior), util.c:307-441 (print_statistics),
//! util.c:443-492 (reset/free_triangleio, dissolved by the owned
//! MeshInput), util.c:494-502 (check_mesh), util.c:504-510 (check),
//! util.c:512-523 (finishfile), util.c:525-621 (the write_*
//! wrappers).
//!
//! CLI-site register rows (docs/deviation-register.md):
//! - D32: the out-of-bounds suffix compares of parsecommandline_legacy
//!   (util.c:189-201) are guarded; main.c's uninitialized `firstnode`
//!   out-param starts at 0 (never read, C's own dead store); the exit
//!   codes lose TRI_FILE_OPEN / TRI_FILE_READ granularity because the
//!   D8 funnel merges them into Error::Io (exit -9); the FILENAMESIZE
//!   truncations of C's fixed buffers do not apply to owned Strings.
//! - D33: the verbose memory and algorithmic sections of
//!   print_statistics (util.c:385-418, 420-439) are not renderable on the
//!   frozen public surface (per-pool maxitems and the predicate
//!   counters are crate-internal), and C reads ratiotable[14]
//!   uninitialized (util.c:369); the port stops after the angle
//!   histogram and renders that one value as 0. The -V verbose surface
//!   stays ungated (no corpus family is verbose); the non-verbose
//!   chatter is gated byte-for-byte since issue #66.
//!
//! TRIANGLE_THREADS (lib.rs note, issue #12): parsed and validated
//! (1..=1024) now; consumed when the internal multithreading lands.

use std::io::Write as _;
use triangle::api::{read_area, read_elements, read_nodes, read_poly};
use triangle::formatter::{g_padded, g_padded_left};
use triangle::{Error, Mesh, MeshInput, Options, Quality, Statistics};

/// TRI_OPTIONS (triangle_config.h): C's exit(-2) becomes process
/// status 254 through the OS, exactly like C.
const TRI_OPTIONS: i32 = -2;

/// The command-line switches and file names of behavior_legacy
/// (util.h:8-30): C's fixed FILENAMESIZE buffers become Strings (the
/// truncation semantics of the fixed buffers are not observable in
/// the harness; register row D32).
struct BehaviorLegacy {
    /// Set at util.c:133, never read by main.c (C's own dead store);
    /// the mesh's firstnumber comes from the options (-z).
    #[allow(dead_code)]
    firstnumber: i32,
    edgesout: bool,
    epsout: bool,
    neighbors: bool,
    /// Set by -B at util.c:145; only the behavior's copy is read.
    #[allow(dead_code)]
    nobound: bool,
    nopolywritten: bool,
    nonodewritten: bool,
    noelewritten: bool,
    noiterationnum: bool,
    /// Set by -O at util.c:160; only the behavior's copy is read.
    #[allow(dead_code)]
    noholes: bool,
    docheck: bool,
    quiet: bool,
    verbose: i32,
    innodefilename: String,
    inelefilename: String,
    inpolyfilename: String,
    areafilename: String,
    outnodefilename: String,
    outelefilename: String,
    outpolyfilename: String,
    edgefilename: String,
    neighborfilename: String,
    epsfilename: String,
}

impl BehaviorLegacy {
    fn new() -> BehaviorLegacy {
        BehaviorLegacy {
            firstnumber: 1,
            edgesout: false,
            epsout: false,
            neighbors: false,
            nobound: false,
            nopolywritten: false,
            nonodewritten: false,
            noelewritten: false,
            noiterationnum: false,
            noholes: false,
            docheck: false,
            quiet: false,
            verbose: 0,
            innodefilename: String::new(),
            inelefilename: String::new(),
            inpolyfilename: String::new(),
            areafilename: String::new(),
            outnodefilename: String::new(),
            outelefilename: String::new(),
            outpolyfilename: String::new(),
            edgefilename: String::new(),
            neighborfilename: String::new(),
            epsfilename: String::new(),
        }
    }
}

/// The behavior fields main.c, check_behavior and print_statistics
/// read (b->poly, b->refine, b->vararea, b->convex, b->weighted,
/// b->quality, b->fixedarea, b->maxarea, b->jettison). Options is
/// opaque to the bin crate by design (ADR 0004 section 5.1), so the
/// CLI scans the option string itself, transliterating exactly the
/// parsecommandline branches that set these fields (triangle.c:387,
/// 390-470, 520-523) plus the tail rules that touch them
/// (triangle.c:540-542, 548-552).
struct BFields {
    poly: bool,
    refine: bool,
    quality: bool,
    vararea: bool,
    fixedarea: bool,
    maxarea: f64,
    convex: bool,
    weighted: bool,
    jettison: bool,
}

/// scan_b_fields: the parsecommandline branches for the BFields set.
/// Digits and '.' are never switches, so the number consumption of
/// 'q'/'U'/'S' (which only advances the scan over non-switch
/// characters) has no effect on these fields and is omitted; 'a'
/// captures its number run for maxarea like C's workstring + strtod
/// (triangle.c:433-450).
fn scan_b_fields(options: &str) -> BFields {
    let s = options.as_bytes();
    let mut f = BFields {
        poly: false,
        refine: false,
        quality: false,
        vararea: false,
        fixedarea: false,
        // triangle.c:387: b->maxarea = -1.0.
        maxarea: -1.0,
        convex: false,
        weighted: false,
        jettison: false,
    };
    let mut i = 0usize;
    while i < s.len() {
        let c = s[i];
        if c == b'p' {
            f.poly = true;
        }
        if c == b'r' {
            f.refine = true;
        }
        if c == b'a' {
            // triangle.c:433-450: quality, then the number branch.
            f.quality = true;
            let mut j = i + 1;
            while j < s.len() && (s[j].is_ascii_digit() || s[j] == b'.') {
                j += 1;
            }
            if j > i + 1 {
                f.fixedarea = true;
                f.maxarea = strtod_run(&s[i + 1..j]);
            } else {
                f.vararea = true;
            }
        }
        if c == b'u' {
            // triangle.c:451-454.
            f.quality = true;
        }
        if c == b'c' {
            // triangle.c:459-461.
            f.convex = true;
        }
        if c == b'w' {
            // triangle.c:462-464.
            f.weighted = true;
        }
        if c == b'W' {
            // triangle.c:465-467 (weighted = 2; the warning only
            // tests nonzero).
            f.weighted = true;
        }
        if c == b'j' {
            // triangle.c:468-470.
            f.jettison = true;
        }
        if c == b'q' || c == b'U' {
            // triangle.c:397-413, 415-431: quality (the angles are not
            // CLI-read).
            f.quality = true;
        }
        if c == b'D' || c == b'L' {
            // triangle.c:520-523.
            f.quality = true;
        }
        i += 1;
    }
    // The tail rules that touch these fields (triangle.c:540-542, 548-552):
    // vararea is zeroed unless refining or reading a PSLG, and
    // weighted triangulations are incompatible with PSLGs and
    // meshing (which makes check_behavior's weighted warning dead,
    // exactly like C).
    if !f.refine && !f.poly {
        f.vararea = false;
    }
    if f.weighted && (f.poly || f.quality) {
        f.weighted = false;
    }
    f
}

/// strtod over a digits-and-dots run (the workstring of
/// triangle.c:438-446): the longest valid decimal prefix, like C.
fn strtod_run(bytes: &[u8]) -> f64 {
    for end in (1..=bytes.len()).rev() {
        if let Ok(txt) = std::str::from_utf8(&bytes[..end]) {
            if let Ok(v) = txt.parse::<f64>() {
                return v;
            }
        }
    }
    0.0
}

/// TRIANGLE_THREADS (issue #12): validated now, consumed when the
/// internal multithreading lands. Anything but an integer in 1..=1024
/// fails to stderr with exit 1.
fn read_thread_count() {
    if let Ok(val) = std::env::var("TRIANGLE_THREADS") {
        match val.parse::<u32>() {
            Ok(n) if (1..=1024).contains(&n) => {}
            _ => {
                eprintln!("Error: TRIANGLE_THREADS must be an integer between 1 and 1024.");
                std::process::exit(1);
            }
        }
    }
}

/// syntax (util.c:17-78), the full-shape variant (not CDT_ONLY, not
/// REDUCED). Exit 0 like C.
fn syntax() -> ! {
    println!("triangle [-prq__a__uAcDjenBPNEIOXzo_YS__iFlsCQVh] input_file");
    println!("    -p  Triangulates a Planar Straight Line Graph (.poly file).");
    println!("    -r  Refines a previously generated mesh.");
    println!("    -q  Quality mesh generation.  A minimum angle may be specified.");
    println!("    -a  Applies a maximum triangle area constraint.");
    println!("    -u  Applies a user-defined triangle constraint.");
    println!("    -A  Applies attributes to identify triangles in certain regions.");
    println!("    -c  Encloses the convex hull with segments.");
    println!("    -D  Conforming Delaunay:  all triangles are truly Delaunay.");
    println!("    -w  Weighted Delaunay triangulation.");
    println!("    -W  Regular triangulation (lower hull of a height field).");
    println!("    -j  Jettison unused vertices from output .node file.");
    println!("    -e  Generates an edge list.");
    println!("    -n  Generates a list of triangle neighbors.");
    println!("    -B  Suppresses output of boundary information.");
    println!("    -P  Suppresses output of .poly file.");
    println!("    -N  Suppresses output of .node file.");
    println!("    -E  Suppresses output of .ele file.");
    println!("    -I  Suppresses mesh iteration numbers.");
    println!("    -O  Ignores holes in .poly file.");
    println!("    -X  Suppresses use of exact arithmetic.");
    println!("    -z  Numbers all items starting from zero (rather than one).");
    println!("    -o2 Generates second-order subparametric elements.");
    println!("    -Y  Suppresses boundary segment splitting.");
    println!("    -S  Specifies maximum number of added Steiner points.");
    println!("    -i  Uses incremental method, rather than divide-and-conquer.");
    println!("    -F  Uses Fortune's sweepline algorithm, rather than d-and-c.");
    println!("    -l  Uses vertical cuts only, rather than alternating cuts.");
    println!("    -s  Force segments into mesh by splitting (instead of using CDT).");
    println!("    -C  Check consistency of final mesh.");
    println!("    -Q  Quiet:  No terminal output except errors.");
    println!("    -V  Verbose:  Detailed information on what I'm doing.");
    println!("    -h  Help:  Detailed instructions for Triangle.");
    std::process::exit(0)
}

/// info (util.c:86-103). TRIANGLE_VERSION is "1.7.0_acute" with the
/// acute feature (the fork's numbering, src/CMakeLists.txt), plain
/// "1.7.0" without. Exit 0 like C.
fn info() -> ! {
    let (major, minor, patch) = triangle::VERSION;
    let version = if triangle::HAS_ACUTE {
        format!("{}.{}.{}_acute", major, minor, patch)
    } else {
        format!("{}.{}.{}", major, minor, patch)
    };
    println!("Triangle (version {})", version);
    println!("A Two-Dimensional Quality Mesh Generator and Delaunay Triangulator.");
    println!();
    println!("Copyright 1993, 1995, 1997, 1998, 2002, 2005 Jonathan Richard Shewchuk");
    println!();
    println!("Detailed instructions can be found in the README file or online at");
    println!("http://www.cs.cmu.edu/~quake/triangle.html.");
    std::process::exit(0)
}

/// parsecommandline_legacy (util.c:112-274).
fn parse_command_line_legacy(args: &[String], b: &mut BehaviorLegacy) {
    // util.c:120-127: the switch resets.
    b.firstnumber = 1;
    b.edgesout = false;
    b.epsout = false;
    b.neighbors = false;
    b.nobound = false;
    b.nopolywritten = false;
    b.nonodewritten = false;
    b.noelewritten = false;
    b.noiterationnum = false;
    b.noholes = false;
    b.docheck = false;
    b.quiet = false;
    b.verbose = 0;
    b.innodefilename.clear();

    // STARTINDEX 1 (util.c:114; the argv loop at 129): argv[0] is skipped.
    for arg in args.iter().skip(1) {
        if arg.as_bytes().first() == Some(&b'-') {
            for c in arg.bytes().skip(1) {
                if c == b'z' {
                    b.firstnumber = 0;
                }
                if c == b'e' {
                    b.edgesout = true;
                }
                if c == b'm' {
                    b.epsout = true;
                }
                if c == b'n' {
                    b.neighbors = true;
                }
                if c == b'B' {
                    b.nobound = true;
                }
                if c == b'P' {
                    b.nopolywritten = true;
                }
                if c == b'N' {
                    b.nonodewritten = true;
                }
                if c == b'E' {
                    b.noelewritten = true;
                }
                if c == b'I' {
                    b.noiterationnum = true;
                }
                if c == b'O' {
                    b.noholes = true;
                }
                if c == b'C' {
                    b.docheck = true;
                }
                if c == b'Q' {
                    b.quiet = true;
                }
                if c == b'V' {
                    b.verbose += 1;
                }
                if c == b'h' || c == b'H' || c == b'?' {
                    info();
                }
                if c == b'v' {
                    println!("WARNING: Voronoi option [v] not supported.");
                }
                if c == b'g' {
                    println!("WARNING: Geomview option [g] not supported.");
                }
            }
        } else {
            // util.c:183-184: strncpy - the last plain argument wins.
            b.innodefilename = arg.clone();
        }
    }
    if b.innodefilename.is_empty() {
        syntax();
    }
    // The suffix strips (util.c:190-201). C compares &name[len - 5]
    // unconditionally, reading out of bounds when the name is shorter
    // than the suffix; the port guards the compare (register row
    // D32), byte-identical whenever C is in bounds.
    if has_suffix(&b.innodefilename, ".node") {
        b.innodefilename.truncate(b.innodefilename.len() - 5);
    }
    if has_suffix(&b.innodefilename, ".poly") {
        b.innodefilename.truncate(b.innodefilename.len() - 5);
    }
    if has_suffix(&b.innodefilename, ".ele") {
        b.innodefilename.truncate(b.innodefilename.len() - 4);
    }
    if has_suffix(&b.innodefilename, ".area") {
        b.innodefilename.truncate(b.innodefilename.len() - 5);
    }

    // The derived input names (util.c:205-207).
    b.inpolyfilename = b.innodefilename.clone();
    b.inelefilename = b.innodefilename.clone();
    b.areafilename = b.innodefilename.clone();

    // The iteration-number scan (util.c:209-228): the position after
    // the last '.' with a non-NUL next character, then the
    // digit-verification do-while that cancels the increment on any
    // non-digit tail.
    let mut increment = 0usize;
    {
        let workstring = b.innodefilename.as_bytes();
        let mut j = 1usize;
        while j < workstring.len() {
            if workstring[j] == b'.' && j + 1 < workstring.len() {
                increment = j + 1;
            }
            j += 1;
        }
    }
    let mut meshnumber: i32 = 0;
    if increment > 0 {
        let workstring = b.innodefilename.as_bytes();
        let mut j = increment;
        loop {
            if workstring[j].is_ascii_digit() {
                // C: int arithmetic; overflow is undefined in C, the
                // port wraps (the options.rs steiner precedent).
                meshnumber = meshnumber
                    .wrapping_mul(10)
                    .wrapping_add((workstring[j] - b'0') as i32);
            } else {
                increment = 0;
            }
            j += 1;
            if j >= workstring.len() {
                break;
            }
        }
    }

    // The output names (util.c:229-269).
    if b.noiterationnum {
        // Plain names, iteration numbers suppressed (util.c:229-239).
        b.outnodefilename = format!("{}.node", b.innodefilename);
        b.outelefilename = format!("{}.ele", b.innodefilename);
        b.edgefilename = format!("{}.edge", b.innodefilename);
        b.epsfilename = format!("{}.eps", b.innodefilename);
        b.neighborfilename = format!("{}.neigh", b.innodefilename);
    } else if increment == 0 {
        // The ".1." names (util.c:240-252).
        b.outnodefilename = format!("{}.1.node", b.innodefilename);
        b.outpolyfilename = format!("{}.1.poly", b.innodefilename);
        b.outelefilename = format!("{}.1.ele", b.innodefilename);
        b.edgefilename = format!("{}.1.edge", b.innodefilename);
        b.epsfilename = format!("{}.1.eps", b.innodefilename);
        b.neighborfilename = format!("{}.1.neigh", b.innodefilename);
    } else {
        // The prefix + sprintf("%d", meshnumber + 1) names
        // (util.c:253-269): workstring with '%d' written over the
        // number start, then the suffix appended to the shared base.
        let base = format!(
            "{}{}",
            &b.innodefilename[..increment],
            meshnumber.wrapping_add(1)
        );
        b.outnodefilename = format!("{}.node", base);
        b.outpolyfilename = format!("{}.poly", base);
        b.outelefilename = format!("{}.ele", base);
        b.edgefilename = format!("{}.edge", base);
        b.epsfilename = format!("{}.eps", base);
        b.neighborfilename = format!("{}.neigh", base);
    }

    // The input suffixes (util.c:270-273).
    b.innodefilename.push_str(".node");
    b.inpolyfilename.push_str(".poly");
    b.inelefilename.push_str(".ele");
    b.areafilename.push_str(".area");
}

fn has_suffix(name: &str, suffix: &str) -> bool {
    name.len() >= suffix.len() && name.ends_with(suffix)
}

/// check_behavior (util.c:276-299), the CLI-side checks and warnings.
fn check_behavior(b: &BFields, lb: &BehaviorLegacy) {
    if b.fixedarea && b.maxarea <= 0.0 {
        println!("ERROR: Maximum area must be greater than zero.");
        std::process::exit(TRI_OPTIONS);
    }

    if b.refine && lb.noiterationnum {
        println!("ERROR: You cannot use the -I switch when refining a triangulation.");
        std::process::exit(TRI_OPTIONS);
    }

    // Regular/weighted triangulations are incompatible with PSLGs
    // and meshing (util.c:288-292). The branch is dead exactly like
    // in C: the parse tail zeroed weighted under poly/quality
    // (triangle.c:550-552).
    if b.weighted && !lb.quiet && (b.poly || b.quality) {
        println!("WARNING:  weighted triangulations (-w, -W) are incompatible");
        println!("  with PSLGs (-p) and meshing (-q, -a, -u).  Weights ignored.");
    }

    if b.jettison && lb.nonodewritten && !lb.quiet {
        println!("WARNING:  -j and -N switches are somewhat incompatible.");
        println!("  If any vertices are jettisoned, you will need the output");
        // util.c:297: no trailing newline, verbatim.
        print!("  .node file to reconstruct the new node indices.");
    }
}

/// The process-boundary exit mapping (register row D32), extracted
/// from check() and the option-parse site as a testable seam (issue
/// #66): None is the successful main return (process status 0 through
/// Termination for ()); the option failures exit TRI_OPTIONS; the
/// D32-recorded Io funnel exits -9 (C's distinct TRI_FILE_OPEN -9 and
/// TRI_FILE_READ -10 both arrive as Error::Io); the remaining classes
/// funnel to -1 like C's raw -1 TODO failures (Error::Input) and the
/// D8 collapse of everything else. Pinned by construction in the
/// exit_code_tests at the end of this file, never via the C oracle.
fn exit_code(error: Option<&Error>) -> i32 {
    match error {
        None => 0,
        Some(Error::Io(_)) => -9,
        Some(Error::Options(_)) => TRI_OPTIONS,
        Some(_) => -1,
    }
}

/// check (util.c:504-510): print C's failure line and exit with the
/// TRI_* code through the exit_code seam.
fn check<T>(result: Result<T, Error>, site: &str) -> T {
    match result {
        Ok(value) => value,
        Err(e) => {
            println!("ERROR: method 'triangle_{}' failed.", site);
            std::process::exit(exit_code(Some(&e)));
        }
    }
}

/// finishfile (util.c:512-523): the "# Generated by" trailer with
/// every argv element (argv[0] included), then the close. C ignores
/// the fprintf return values here; the port ignores them too. The
/// fclose flush is the explicit file.flush() on the way out, not
/// left to BufWriter's Drop, under the same ignored-error contract.
fn finish_file(file: &mut dyn std::io::Write, args: &[String]) {
    let _ = write!(file, "# Generated by");
    for arg in args {
        let _ = write!(file, " {}", arg);
    }
    let _ = writeln!(file);
    let _ = file.flush();
}

/// open_output: a port-level helper for the six write_* functions
/// below (util.c:525-621), which each open their file with C's
/// fopen(name, "w") and silently skip the file when that fails
/// (main.c:68). The BufWriter restores C's write granularity: stdio
/// streams are fully buffered by default (the oracle's 4096-byte
/// write(2) calls, docs/perf-r1-findings.md section 3), while
/// std::fs::File is unbuffered and turned each formatted fragment
/// into its own write(2). The byte stream itself is unchanged and
/// stays gated by the 120-run differential harness.
fn open_output(filename: &str) -> Option<std::io::BufWriter<std::fs::File>> {
    let file = std::fs::File::create(filename).ok()?;
    Some(std::io::BufWriter::new(file))
}

/// write_nodes (util.c:525-539): fopen "w" failure returns -1 and
/// main ignores the status (main.c:68); the port skips silently,
/// exactly the observable C behavior.
fn write_nodes(mesh: &Mesh, filename: &str, args: &[String]) {
    let Some(mut file) = open_output(filename) else {
        return;
    };
    let status = mesh.write_nodes(&mut file);
    finish_file(&mut file, args);
    let _ = status;
}

/// write_elements (util.c:541-555).
fn write_elements(mesh: &Mesh, filename: &str, args: &[String]) {
    let Some(mut file) = open_output(filename) else {
        return;
    };
    let status = mesh.write_elements(&mut file);
    finish_file(&mut file, args);
    let _ = status;
}

/// write_poly (util.c:557-573): the holes and regions come from the
/// read input exactly like C's in->holelist / in->regionlist.
fn write_poly(mesh: &Mesh, filename: &str, input: &MeshInput, args: &[String]) {
    let Some(mut file) = open_output(filename) else {
        return;
    };
    let status = mesh.write_poly(&mut file, &input.holes, &input.regions);
    finish_file(&mut file, args);
    let _ = status;
}

/// write_edges (util.c:575-589).
fn write_edges(mesh: &Mesh, filename: &str, args: &[String]) {
    let Some(mut file) = open_output(filename) else {
        return;
    };
    let status = mesh.write_edges(&mut file);
    finish_file(&mut file, args);
    let _ = status;
}

/// write_neighbors (util.c:591-605).
fn write_neighbors(mesh: &Mesh, filename: &str, args: &[String]) {
    let Some(mut file) = open_output(filename) else {
        return;
    };
    let status = mesh.write_neighbors(&mut file);
    finish_file(&mut file, args);
    let _ = status;
}

/// write_postscript (util.c:607-621): no finishfile trailer, just the
/// close (the explicit flush stands in for C's fclose, exactly like
/// the one in finishfile).
fn write_postscript(mesh: &Mesh, filename: &str) {
    let Some(mut file) = open_output(filename) else {
        return;
    };
    let status = mesh.write_eps(&mut file);
    let _ = file.flush();
    let _ = status;
}

/// check_mesh (util.c:494-502).
fn check_mesh(mesh: &Mesh) {
    if mesh.check_mesh() > 0 {
        println!("WARNING: mesh is topologically inconsistent.");
    }
    if mesh.check_delaunay() > 0 {
        println!("WARNING: mesh is not (constraint) Delaunay.");
    }
}

/// print_statistics (util.c:307-441). The input counts come from the
/// read MeshInput (C reads the m->in* counters the create sequence
/// stored from the same triangleio), the mesh counts from
/// Statistics, the quality block from Quality; the renderers are the
/// formatter's padded g-forms.
fn print_statistics(mesh: &Mesh, input: &MeshInput, b: &BFields, lb: &BehaviorLegacy) {
    let st: Statistics = mesh.statistics();
    // The ratiotable constants (util.c:310, 316-322). C declares sixteen
    // entries and initializes fourteen; its last histogram line reads
    // ratiotable[14] uninitialized (util.c:369). The port renders
    // 0.0 there (register row D33).
    let ratiotable = [
        1.5, 2.0, 2.5, 3.0, 4.0, 6.0, 10.0, 15.0, 25.0, 50.0, 100.0, 300.0, 1000.0, 10000.0,
    ];

    println!("\nStatistics:\n");
    println!("  Input vertices: {}", input.points.len());
    if b.refine {
        println!(
            "  Input triangles: {}",
            input.elements.len() / input.corners_per_element.max(1)
        );
    }
    if b.poly {
        println!("  Input segments: {}", input.segments.len());
        if !b.refine {
            println!("  Input holes: {}", input.holes.len());
        }
    }

    println!(
        "\n  Mesh vertices: {}",
        st.vertices as i64 - st.undeads as i64
    );
    println!("  Mesh triangles: {}", st.triangles);
    println!("  Mesh edges: {}", st.edges);
    println!("  Mesh exterior boundary edges: {}", st.hullsize);
    if b.poly || b.refine {
        println!(
            "  Mesh interior boundary edges: {}",
            st.subsegs as i64 - st.hullsize as i64
        );
        println!("  Mesh subsegments (constrained edges): {}", st.subsegs);
    }
    println!();

    if lb.verbose > 0 {
        let q: Quality = mesh
            .quality()
            .expect("triangle_mesh_quality is failure-free (triangle_api.c:102)");

        println!("Mesh quality statistics:\n");

        println!(
            "  Smallest area: {}   |  Largest area: {}",
            g_padded(q.smallest_area, 5, 16),
            g_padded(q.biggest_area, 5, 16)
        );
        println!(
            "  Shortest edge: {}   |  Longest edge: {}",
            g_padded(q.shortest, 5, 16),
            g_padded(q.longest, 5, 16)
        );
        println!(
            "  Shortest altitude: {}   |  Largest aspect ratio: {}\n",
            g_padded(q.min_altitude, 5, 12),
            g_padded(q.worst_aspect, 5, 8)
        );

        println!("  Triangle aspect ratio histogram:");
        println!(
            "  1.1547 - {}    :  {:>8}    | {} - {}     :  {:>8}",
            g_padded_left(ratiotable[0], 6, 6),
            q.aspect_table[0],
            g_padded(ratiotable[7], 6, 6),
            g_padded_left(ratiotable[8], 6, 6),
            q.aspect_table[8]
        );
        for i in 1..7 {
            println!(
                "  {} - {}    :  {:>8}    | {} - {}     :  {:>8}",
                g_padded(ratiotable[i - 1], 6, 6),
                g_padded_left(ratiotable[i], 6, 6),
                q.aspect_table[i],
                g_padded(ratiotable[i + 7], 6, 6),
                g_padded_left(ratiotable[i + 8], 6, 6),
                q.aspect_table[i + 8]
            );
        }
        println!(
            "  {} - {}    :  {:>8}    | {} -            :  {:>8}",
            g_padded(ratiotable[6], 6, 6),
            g_padded_left(ratiotable[7], 6, 6),
            q.aspect_table[7],
            g_padded(0.0, 6, 6),
            q.aspect_table[15]
        );
        println!("  (Aspect ratio is longest edge divided by shortest altitude)\n");

        println!(
            "  Smallest angle: {}   |  Largest angle: {}\n",
            g_padded(q.smallest_angle, 5, 15),
            g_padded(q.biggest_angle, 5, 15)
        );

        println!("  Angle histogram:");
        for i in 0..9usize {
            println!(
                "    {:>3} - {:>3} degrees:  {:>8}    |    {:>3} - {:>3} degrees:  {:>8}",
                i * 10,
                i * 10 + 10,
                q.angle_table[i],
                i * 10 + 90,
                i * 10 + 100,
                q.angle_table[i + 9]
            );
        }
        println!();

        // The memory allocation and algorithmic sections
        // (util.c:385-418, 420-439) are not renderable on the frozen public
        // surface (per-pool maxitems and the predicate counters are
        // crate-internal); register row D33. The corpus families are all
        // non-verbose, so this ungated -V surface stays outside the
        // stdout gate (issue #66).
    }
}

/// main (main.c:8-114), the full-shape (#else of CDT_ONLY) flow.
fn main() {
    let args: Vec<String> = std::env::args().collect();

    read_thread_count();

    let mut lb = BehaviorLegacy::new();
    parse_command_line_legacy(&args, &mut lb);

    // At this point argc is guaranteed to be > 1 (syntax() exits
    // otherwise, main.c:24 comment).
    let option_string = if args[1].as_bytes().first() == Some(&b'-') {
        &args[1][1..]
    } else {
        ""
    };
    // triangle_context_options (main.c:26): the option string goes to
    // the parser. The port's Options::parse rejects only what the C
    // NO_ACUTE build rejects (-U without the acute feature,
    // ADR 0004 section 2); C's parsecommandline never fails
    // (unknown switches are silently ignored, triangle.c:364-553).
    let options = match Options::parse(option_string) {
        Ok(options) => options,
        Err(e) => {
            println!("{}", e);
            std::process::exit(exit_code(Some(&e)));
        }
    };
    let bf = scan_b_fields(option_string);

    check_behavior(&bf, &lb);

    // reset_triangleio (main.c:22, util.c:443-471): the owned MeshInput
    // starts empty. C's `int firstnode` (main.c:16) is an out-param
    // main never reads; the port zero-initializes it (register row
    // D32, C leaves it uninitialized).
    let mut input = MeshInput::default();
    let mut firstnode: i32 = 0;

    if bf.poly {
        check(
            read_poly(&lb.inpolyfilename, &mut input, &mut firstnode),
            "read_poly",
        );
    }

    if input.points.is_empty() {
        // The poly file had no vertices (main.c:37-39).
        check(
            read_nodes(&lb.innodefilename, &mut input, &mut firstnode),
            "read_nodes",
        );
    }

    let mesh = if bf.refine {
        // Read the mesh (main.c:45-49).
        check(
            read_elements(&lb.inelefilename, &mut input),
            "read_elements",
        );
        if bf.vararea {
            check(read_area(&lb.areafilename, &mut input), "read_area");
        }
        // Reconstruct the mesh (main.c:50-52): load_mesh runs the
        // triangle_mesh_load sequence (reconstruct plus the load-era
        // gates) and refine the quality pass, in C's order; no stage-1
        // scaffolding remains anywhere in the port.
        let mut loaded = check(Mesh::load_mesh(options, &input), "mesh_load");
        check(loaded.refine(), "mesh_refine");
        loaded
    } else {
        // Triangulate the polygon (main.c:54-55).
        check(triangle::triangulate(options, &input), "mesh_create")
    };

    // If not using iteration numbers, don't write a .node file if one
    // was read, because the original one would be overwritten
    // (main.c:59-69).
    if lb.nonodewritten || (lb.noiterationnum && !bf.poly) {
        if !lb.quiet {
            println!("NOT writing a .node file.");
        }
        // numbernodes(&m, &b) is commented out in C (main.c:65): the
        // writers inline their own numbering.
    } else {
        // writenodes() numbers the vertices too (main.c:67).
        write_nodes(&mesh, &lb.outnodefilename, &args);
    }
    if lb.noelewritten {
        if !lb.quiet {
            println!("NOT writing an .ele file.");
        }
    } else {
        write_elements(&mesh, &lb.outelefilename, &args);
    }
    // The -c switch (convex switch) causes a PSLG to be written even
    // if none was read (main.c:77-88).
    if bf.poly || bf.convex {
        // If not using iteration numbers, don't overwrite the .poly
        // file (main.c:80-87).
        if lb.nopolywritten || lb.noiterationnum {
            if !lb.quiet {
                println!("NOT writing a .poly file.");
            }
        } else {
            write_poly(&mesh, &lb.outpolyfilename, &input, &args);
        }
    }

    if lb.edgesout {
        write_edges(&mesh, &lb.edgefilename, &args);
    }

    if lb.epsout {
        write_postscript(&mesh, &lb.epsfilename);
    }

    if lb.neighbors {
        write_neighbors(&mesh, &lb.neighborfilename, &args);
    }

    if !lb.quiet {
        print_statistics(&mesh, &input, &bf, &lb);
    }

    // -C: check consistency of the final mesh (main.c:106-110).
    if lb.docheck {
        check_mesh(&mesh);
    }

    // triangle_context_destroy (main.c:112) dissolves over the owned
    // values. main returns 0 (main.c:114).
}

/// The exit-code contract of the process boundary, pinned by
/// construction (issue #66; never via the C oracle): every arm of the
/// exit_code seam and the OS-level reduction of the negative codes.
/// C anchors: util.c:504-510 (check exits the raw TRI_* code),
/// util.c:278-285 (check_behavior's two exit(TRI_OPTIONS) sites),
/// triangle_config.h (TRI_OPTIONS -2), main.c:114 (success returns 0).
#[cfg(test)]
mod exit_code_tests {
    use super::*;
    use triangle::SegmentRecoveryKind;

    /// Success: main returns normally (main.c:114 returns 0); the seam
    /// maps the successful arm to process status 0.
    #[test]
    fn success_maps_to_zero() {
        assert_eq!(exit_code(None), 0);
    }

    /// The option failures (Options::parse, check_behavior) exit
    /// TRI_OPTIONS exactly like C's exit(TRI_OPTIONS) sites
    /// (util.c:280, 285).
    #[test]
    fn options_errors_exit_tri_options() {
        assert_eq!(TRI_OPTIONS, -2);
        assert_eq!(
            exit_code(Some(&Error::Options("no acute".to_string()))),
            TRI_OPTIONS
        );
    }

    /// C's exit(-2) surfaces as process status 254 through the OS
    /// (register row D32): the wait status keeps the low byte of the
    /// negative code. std::process::exit hands the i32 to the OS
    /// verbatim, so the same reduction applies to the port.
    #[test]
    fn tri_options_surfaces_as_process_status_254() {
        assert_eq!((TRI_OPTIONS as u32) & 0xff, 254);
    }

    /// The D32 Io funnel: C's distinct TRI_FILE_OPEN (-9, the missing
    /// file) and TRI_FILE_READ (-10) both arrive as Error::Io and exit
    /// -9. Both reader failure kinds are pinned.
    #[test]
    fn io_funnel_exits_minus_9() {
        let open_failure = Error::Io(std::io::Error::new(std::io::ErrorKind::NotFound, "x"));
        let read_failure = Error::Io(std::io::Error::from(std::io::ErrorKind::PermissionDenied));
        assert_eq!(exit_code(Some(&open_failure)), -9);
        assert_eq!(exit_code(Some(&read_failure)), -9);
    }

    /// The remaining classes funnel to -1: C's raw -1 TODO failures
    /// (Error::Input) and the D8 collapse of the rest.
    #[test]
    fn remaining_classes_funnel_to_minus_1() {
        assert_eq!(exit_code(Some(&Error::Input("bad index".to_string()))), -1);
        assert_eq!(
            exit_code(Some(&Error::SegmentRecovery(SegmentRecoveryKind::Insert))),
            -1
        );
        assert_eq!(exit_code(Some(&Error::OutOfMemory)), -1);
        assert_eq!(exit_code(Some(&Error::Internal("bug".to_string()))), -1);
    }
}
