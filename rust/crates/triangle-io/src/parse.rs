//! The parse seam and the six-format readers (the read half of
//! triangle_io.c, migrated from the triangle crate's io/mod.rs by the
//! interchange-crate extraction, issue #70 / ADR 0006; the migration
//! table lives in docs/function-mapping.md).
//!
//! Boundary decisions (inherited unchanged from the io/mod.rs seam,
//! where they were justified - the deviation register carries the
//! details):
//! - fgets mirrors over a byte buffer with INPUTLINESIZE semantics
//!   (triangle_config.h:114); read errors fold to NULL exactly like
//!   C's fgets, so a mid-file I/O failure reads as end-of-file.
//! - TRI_FILE_READ (a fgets NULL) raises Error::Read at the seam
//!   with the exact io::Error payload the port's funnel arm carried
//!   (D31's equivalence class), and the raw -1 TODO returns raise
//!   Error::Input with the funnel arm's exact TRI_FAILURE payload
//!   text. The triangle crate maps both back onto its own unchanged
//!   public variants at its seam (api.rs).
//! - strtol/strtod mirror glibc's semantics at the call sites: base-0
//!   longest-prefix rules (hex/octal), endptr advancement (including
//!   the no-advance-on-no-digits rule), strtod's decimal grammar with
//!   inf/nan and glibc's hex floats, all correctly rounded (Rust's
//!   f64 parser is correctly rounded like glibc's).
//! - The .edge and .neigh readers have no C counterpart (C only
//!   writes those formats, triangle_io.c:203-310); they parse the
//!   writer grammar under the readers' own conventions, pinned by
//!   round-trip construction tests with the write module - never via
//!   the C oracle binary.

use std::io::{BufReader, Read};

use crate::error::Error;
use crate::types::{
    AreaFile, EdgeFile, EleFile, Hole, NeighFile, NodeFile, Point2, PolyFile, Region,
};

// ---------------------------------------------------------------------------
// The parse seam: readline / findfield / strtol / strtod
// (triangle_io.c:364-423; INPUTLINESIZE, triangle_config.h:114).
// ---------------------------------------------------------------------------

/// INPUTLINESIZE (triangle_config.h:114): fgets reads at most
/// INPUTLINESIZE - 1 bytes per call.
const INPUT_LINE_SIZE: usize = 1024;

/// The fgets substrate: a byte reader with C's fgets semantics over
/// any std::io::Read (C's FILE* - the readers never need seeking or
/// fflush, so the buffered byte reader is the whole mirror). Read
/// errors fold to NULL exactly like C's fgets.
pub(crate) struct LineFile<R: Read> {
    reader: BufReader<R>,
}

impl<R: Read> LineFile<R> {
    pub(crate) fn new(inner: R) -> LineFile<R> {
        LineFile {
            reader: BufReader::new(inner),
        }
    }

    /// fgets(string, INPUTLINESIZE, infile): reads up to
    /// INPUTLINESIZE - 1 bytes, stopping right after a newline; None
    /// where C returns NULL (end of file with nothing read, or a read
    /// error - C cannot distinguish either).
    fn fgets(&mut self) -> Option<Vec<u8>> {
        let mut out: Vec<u8> = Vec::new();
        let mut byte = [0u8; 1];
        loop {
            match self.reader.read(&mut byte) {
                Ok(0) | Err(_) => {
                    return if out.is_empty() { None } else { Some(out) };
                }
                Ok(_) => {
                    out.push(byte[0]);
                    if byte[0] == b'\n' || out.len() == INPUT_LINE_SIZE - 1 {
                        return Some(out);
                    }
                }
            }
        }
    }
}

/// One readline result: the bytes of the line (a C fgets buffer with
/// the '\0' terminator modeled as the end of the Vec, and any
/// embedded NUL truncating the scan-visible content exactly like C's
/// '\0' checks) plus the cursor C's stringptr walks. The cursor
/// methods mirror strtol/strtod/findfield endptr advancement.
pub(crate) struct Line {
    buf: Vec<u8>,
    pos: usize,
}

/// Whether a byte stops the number-lookahead scans of readline and
/// findfield (triangle_io.c:383-384, 414-416): a digit, a '.', a sign
/// - or, for the loop conditions, '#' and the line end.
fn looks_like_number(c: u8) -> bool {
    c == b'.' || c == b'+' || c == b'-' || c.is_ascii_digit()
}

/// C's isspace set (the strtol/strtod whitespace skip).
fn is_c_space(c: u8) -> bool {
    matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

/// readline (triangle_io.c:370-389): read a nonempty line - a line is
/// considered nonempty if it contains something that looks like a
/// number; comments (prefaced by '#') are skipped by the do-while
/// rescan. None where C returns NULL (checked by the caller).
pub(crate) fn read_line<R: Read>(infile: &mut LineFile<R>) -> Option<Line> {
    loop {
        let bytes = infile.fgets()?;
        // A NUL byte ends the scan-visible content of C's buffer.
        let cut = bytes.iter().position(|&c| c == 0).unwrap_or(bytes.len());
        let buf = bytes[..cut].to_vec();
        // Search for something that looks like a number (382-387):
        // skip anything that doesn't look like a number, a comment,
        // or the end of a line.
        let mut i = 0usize;
        while i < buf.len() && !looks_like_number(buf[i]) && buf[i] != b'#' {
            i += 1;
        }
        // If it's a comment or end of line, read another line and try
        // again (389).
        if i == buf.len() || buf[i] == b'#' {
            continue;
        }
        return Some(Line { buf, pos: i });
    }
}

impl Line {
    /// *stringptr == '\0': the cursor sits at the line end (or at a
    /// '#' findfield truncated to '\0').
    fn at_end(&self) -> bool {
        self.pos >= self.buf.len()
    }

    /// findfield (triangle_io.c:401-423): jump past the current field
    /// by searching for whitespace, then jump past the whitespace and
    /// anything else that doesn't look like a number, a comment, or
    /// the end of the line. A '#' truncates the line ('\0' store at
    /// 413-415).
    fn find_field(&mut self) {
        let mut p = self.pos;
        // Skip the current field. Stop upon reaching whitespace
        // (407-411).
        while p < self.buf.len()
            && self.buf[p] != b'#'
            && self.buf[p] != b' '
            && self.buf[p] != b'\t'
        {
            p += 1;
        }
        // Now skip the whitespace and anything else that doesn't look
        // like a number, a comment, or the end of a line (412-416).
        // Note C's second loop does NOT stop at ' ' or '\t' - a
        // whole run of separators is skipped until the next
        // number-ish character, '#' or the end.
        while p < self.buf.len() && !looks_like_number(self.buf[p]) && self.buf[p] != b'#' {
            p += 1;
        }
        // Check for a comment (prefixed with '#') (418-421): the '\0'
        // store truncates the rest of the line for every later scan.
        if p < self.buf.len() && self.buf[p] == b'#' {
            self.buf.truncate(p);
        }
        self.pos = p;
    }

    /// strtol(stringptr, &stringptr, 0) (glibc, base 0): whitespace
    /// skip, optional sign, 0x/0X hex, leading-0 octal, longest valid
    /// prefix, saturating like C's LONG_MAX/LONG_MIN, endptr at the
    /// end of the prefix - and NOT advanced at all when no digits are
    /// found (C's endptr = nptr rule). The (int) casts at the call
    /// sites happen at the callers, like C's.
    fn strtol(&mut self) -> i64 {
        let s = &self.buf;
        let mut p = self.pos;
        while p < s.len() && is_c_space(s[p]) {
            p += 1;
        }
        let mut q = p;
        let mut neg = false;
        if q < s.len() && (s[q] == b'+' || s[q] == b'-') {
            neg = s[q] == b'-';
            q += 1;
        }
        // Base detection (glibc): "0x"/"0X" only with a hex digit
        // behind it, else a leading '0' is octal, else decimal.
        let mut base = 10u64;
        let mut digits_start = q;
        let is_digit = |c: u8, base: u64| -> bool {
            if base == 16 {
                c.is_ascii_hexdigit()
            } else {
                c.is_ascii_digit() && ((c - b'0') as u64) < base
            }
        };
        if q < s.len() && s[q] == b'0' {
            if q + 1 < s.len()
                && (s[q + 1] | 0x20) == b'x'
                && q + 2 < s.len()
                && s[q + 2].is_ascii_hexdigit()
            {
                base = 16;
                digits_start = q + 2;
            } else {
                // Octal (a bare "0" included; "09" parses the "0").
                base = 8;
                digits_start = q;
            }
        }
        let mut end = digits_start;
        while end < s.len() && is_digit(s[end], base) {
            end += 1;
        }
        if end == digits_start {
            // No digits: C returns 0 and does not advance endptr.
            return 0;
        }
        let mut value: i64 = 0;
        for &c in &s[digits_start..end] {
            let digit = (c as char).to_digit(base as u32).expect("digit") as i64;
            value =
                value
                    .saturating_mul(base as i64)
                    .saturating_add(if neg { -digit } else { digit });
        }
        self.pos = end;
        value
    }

    /// strtod(stringptr, &stringptr) (glibc): whitespace skip,
    /// optional sign, then the longest valid prefix of the grammar
    /// inf/infinity, nan(...), hex floats (0x hexigits [. hexigits]
    /// [p exponent]) and decimal (digits [. digits] [e exponent]);
    /// correctly rounded (Rust's parser is correctly rounded for the
    /// accepted decimal slice, the hex-float conversion rounds to
    /// nearest-even by hand). No valid prefix: 0.0 with the cursor
    /// unadvanced, exactly like C.
    fn strtod(&mut self) -> f64 {
        let s = &self.buf;
        let mut p = self.pos;
        while p < s.len() && is_c_space(s[p]) {
            p += 1;
        }
        let mut q = p;
        let mut neg = false;
        if q < s.len() && (s[q] == b'+' || s[q] == b'-') {
            neg = s[q] == b'-';
            q += 1;
        }
        let rest = &s[q..];
        // inf / infinity (case-insensitive, glibc).
        if starts_with_ci(rest, b"inf") {
            let len = if starts_with_ci(&rest[3..], b"inity") {
                8
            } else {
                3
            };
            self.pos = q + len;
            return if neg {
                f64::NEG_INFINITY
            } else {
                f64::INFINITY
            };
        }
        // nan, with glibc's optional (...) payload when it closes on
        // the same line.
        if starts_with_ci(rest, b"nan") {
            let mut len = 3;
            if rest.len() > 3 && rest[3] == b'(' {
                if let Some(close) = rest[4..].iter().position(|&c| c == b')') {
                    len = 4 + close + 1;
                }
            }
            self.pos = q + len;
            return f64::NAN;
        }
        // Hex floats (glibc): 0x/0X with at least one hex digit
        // before the point or after it ("0x.8p1" parses as 1.0).
        if rest.len() >= 2 && rest[0] == b'0' && (rest[1] | 0x20) == b'x' {
            let mut i = 2;
            while i < rest.len() && rest[i].is_ascii_hexdigit() {
                i += 1;
            }
            let mut probe = i;
            let mut had_dot = false;
            if probe < rest.len() && rest[probe] == b'.' {
                had_dot = true;
                probe += 1;
                while probe < rest.len() && rest[probe].is_ascii_hexdigit() {
                    probe += 1;
                }
            }
            if i > 2 || (had_dot && probe > i + 1) {
                let mut j = i;
                if j < rest.len() && rest[j] == b'.' {
                    j += 1;
                    while j < rest.len() && rest[j].is_ascii_hexdigit() {
                        j += 1;
                    }
                }
                let mut end = j;
                if j < rest.len() && (rest[j] | 0x20) == b'p' {
                    let mut k = j + 1;
                    if k < rest.len() && (rest[k] == b'+' || rest[k] == b'-') {
                        k += 1;
                    }
                    let digits_from = k;
                    while k < rest.len() && rest[k].is_ascii_digit() {
                        k += 1;
                    }
                    if k > digits_from {
                        end = k;
                    }
                }
                let v = parse_hex_float(&rest[..end]);
                self.pos = q + end;
                return if neg { -v } else { v };
            }
            // "0x" without a hex digit: glibc falls back to the
            // decimal "0" and leaves the 'x' for the cursor - the
            // decimal scan below starts at the '0' and does exactly
            // that.
        }
        // Decimal: digits [. digits] [e[+-]digits], longest valid
        // prefix; an exponent without digits is not consumed.
        let mut i = q;
        let mut int_digits = 0;
        while i < s.len() && s[i].is_ascii_digit() {
            i += 1;
            int_digits += 1;
        }
        let mut mant_end = i;
        let mut frac_digits = 0;
        if i < s.len() && s[i] == b'.' {
            i += 1;
            while i < s.len() && s[i].is_ascii_digit() {
                i += 1;
                frac_digits += 1;
            }
            mant_end = i;
        }
        if int_digits == 0 && frac_digits == 0 {
            // No valid prefix: 0.0, cursor unadvanced (glibc's
            // endptr = nptr).
            return 0.0;
        }
        let mut end = mant_end;
        if end < s.len() && (s[end] | 0x20) == b'e' {
            let mut k = end + 1;
            if k < s.len() && (s[k] == b'+' || s[k] == b'-') {
                k += 1;
            }
            let digits_from = k;
            while k < s.len() && s[k].is_ascii_digit() {
                k += 1;
            }
            if k > digits_from {
                end = k;
            }
        }
        let v = parse_decimal(&s[q..end]);
        self.pos = end;
        if neg {
            -v
        } else {
            v
        }
    }
}

fn starts_with_ci(haystack: &[u8], needle: &[u8]) -> bool {
    haystack.len() >= needle.len()
        && haystack[..needle.len()]
            .iter()
            .zip(needle)
            .all(|(h, n)| (h | 0x20) == *n)
}

/// Rust's f64 parser over the accepted decimal slice, with the two
/// benign normalizations for the grammar edges Rust's parser rejects
/// ("1." -> "1.0", ".5" -> "0.5": appending or inserting a zero digit
/// never changes the value). Correctly rounded, like glibc.
fn parse_decimal(bytes: &[u8]) -> f64 {
    let mut txt = String::with_capacity(bytes.len() + 1);
    let mut seen_digit = false;
    for &c in bytes {
        if c.is_ascii_digit() {
            seen_digit = true;
        }
        // Lowercase the exponent marker for Rust's parser.
        txt.push(if c == b'E' { 'e' } else { c as char });
    }
    if txt.ends_with('.') {
        txt.push('0');
    }
    if txt.starts_with('.') {
        txt.insert(0, '0');
    }
    if !seen_digit {
        // Unreachable by construction (mantissa validation upstream);
        // the safety net mirrors C's 0.0.
        return 0.0;
    }
    txt.parse::<f64>().unwrap_or(0.0)
}

/// The value of an accepted glibc hex-float prefix ("0x...", sign
/// already handled by the caller): exact hex mantissa, power-of-two
/// exponent, one rounding to the f64 (round to nearest, ties to even,
/// with the sticky bits of truncated mantissa digits) - glibc's
/// correctly-rounded conversion.
fn parse_hex_float(b: &[u8]) -> f64 {
    let hexval = |c: u8| -> u128 { (c as char).to_digit(16).expect("hex digit") as u128 };
    let mut m: u128 = 0;
    let mut e2: i64 = 0; // value = m * 2^e2
    let mut sticky = false;
    let mut i = 2; // past "0x"
                   // Integer-part digits.
    while i < b.len() && b[i].is_ascii_hexdigit() {
        let d = hexval(b[i]);
        if m <= (u128::MAX - 15) / 16 {
            m = m * 16 + d;
        } else {
            // The mantissa is far wider than 53 bits already: the
            // digit only feeds the sticky bit.
            e2 += 4;
            sticky |= d != 0;
        }
        i += 1;
    }
    if i < b.len() && b[i] == b'.' {
        i += 1;
        while i < b.len() && b[i].is_ascii_hexdigit() {
            let d = hexval(b[i]);
            if m == 0 && d == 0 {
                e2 -= 4;
            } else if m <= (u128::MAX - 15) / 16 {
                m = m * 16 + d;
                e2 -= 4;
            } else {
                sticky |= d != 0;
            }
            i += 1;
        }
    }
    // Binary exponent p[+-]digits.
    if i < b.len() && (b[i] | 0x20) == b'p' {
        i += 1;
        let mut esign = 1i64;
        if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
            if b[i] == b'-' {
                esign = -1;
            }
            i += 1;
        }
        let mut digits: i64 = 0;
        while i < b.len() && b[i].is_ascii_digit() {
            digits = (digits * 10 + (b[i] - b'0') as i64).min(1 << 40);
            i += 1;
        }
        e2 += esign * digits;
    }
    if m == 0 {
        return 0.0;
    }
    // Round the mantissa to 53 significant bits (round to nearest,
    // ties to even, sticky included).
    let bits = 128 - m.leading_zeros() as i64;
    if bits > 53 {
        let shift = bits - 53;
        let kept = m >> shift;
        let rem = m & ((1u128 << shift) - 1);
        let half = 1u128 << (shift - 1);
        let mut sig = kept;
        if rem > half || (rem == half && (sticky || kept & 1 == 1)) {
            sig += 1;
        }
        if sig == 1u128 << 53 {
            sig >>= 1;
            e2 += 1;
        }
        m = sig;
        e2 += shift;
    } else if bits < 53 {
        // Exact left-shift into [2^52, 2^53); no rounding
        // needed.
        let shift = 53 - bits;
        m <<= shift;
        e2 -= shift;
    }
    // value = m * 2^e2 with m in [2^52, 2^53); assemble by hand so
    // no intermediate product rounds (exact assembly, subnormals
    // included).
    let te = e2 + 52; // value = (m / 2^52) * 2^te
    if te > 1023 {
        return f64::INFINITY;
    }
    if te >= -1022 {
        // Normal: exponent field te + 1023, fraction m - 2^52.
        let bits = (((te + 1023) as u64) << 52) | ((m - (1u128 << 52)) as u64);
        return f64::from_bits(bits);
    }
    // Subnormal: value = sig * 2^-1074 with a second rounding to the
    // reduced precision.
    let avail = te + 1074; // significand bits available
    if avail < 0 {
        // value < 2^-1074, or exactly its half: only a value
        // above the exact half at te == -1075 rounds up to the
        // smallest subnormal (avail == 0 is 2^-1074 itself, the
        // from_bits(1) path below).
        if te == -1075 && (m > 1 << 52 || sticky) {
            return f64::from_bits(1);
        }
        return 0.0;
    }
    let shift = 52 - avail;
    let kept = m >> shift;
    let rem = m & ((1u128 << shift) - 1);
    let half = 1u128 << (shift - 1);
    let mut sig = kept;
    if rem > half || (rem == half && (sticky || kept & 1 == 1)) {
        sig += 1;
    }
    f64::from_bits(sig as u64)
}

/// TRI_FILE_READ (a fgets NULL at the seam): the io::Error payload
/// D31's equivalence class fixed (the stage-1 funnel arm's text), so
/// the triangle crate's Error::Io conversion keeps its exact
/// to_string.
fn read_failure() -> std::io::Error {
    std::io::Error::other("file read failure (TRI_FILE_READ)")
}

/// The raw -1 TODO returns of C's readers (malformed input): the
/// funnel arm's exact TRI_FAILURE payload text.
fn input_failure() -> Error {
    Error::Input("invalid input (TRI_FAILURE)".to_string())
}

// ---------------------------------------------------------------------------
// File reading routines (triangle_io.c:356-876).
// ---------------------------------------------------------------------------

/// file_readnodes_internal (triangle_io.c:432-546): read the vertices
/// from a .node or .poly file. Fills the neutral NodeFile exactly like
/// C fills the triangleio: the coordinate list, the attribute list
/// (default 0.0 per missing field), the marker list (default 0) and
/// the header fields; firstnode is captured at i == 0 from the first
/// vertex's number field (479-481). The poly zero-count arm (472-477)
/// reports separate_file instead of touching any caller list (C
/// clears io->pointlist there; the consumer owns that store at its
/// seam).
fn parse_node_section<R: Read>(file: &mut LineFile<R>, poly: bool) -> Result<NodeFile, Error> {
    // Read number of vertices, number of dimensions, number of vertex
    // attributes, and number of boundary markers (450-465), with C's
    // default fills for missing fields.
    let Some(mut line) = read_line(file) else {
        return Err(Error::Read(read_failure()));
    };
    let invertices = line.strtol() as i32;
    line.find_field();
    let mesh_dim = if line.at_end() {
        2
    } else {
        line.strtol() as i32
    };
    line.find_field();
    let nextras = if line.at_end() {
        0
    } else {
        line.strtol() as i32
    };
    line.find_field();
    let nodemarkers = if line.at_end() {
        0
    } else {
        line.strtol() as i32
    };

    let mut nodes = NodeFile {
        point_count: invertices,
        dim: mesh_dim,
        attributes_per_point: nextras,
        markers_present: nodemarkers != 0,
        first_number: 0,
        separate_file: false,
        points: Vec::new(),
        point_attributes: Vec::new(),
        point_markers: Vec::new(),
    };

    if poly && invertices == 0 {
        // In case we are reading a .poly file, but the vertices are
        // stored in a separate .node file, return OK so reading of
        // .poly file can continue (472-477).
        nodes.separate_file = true;
        return Ok(nodes);
    }

    if invertices < 3 {
        // TODO: error: Input must have at least three input vertices
        // (481).
        return Err(input_failure());
    }
    if mesh_dim != 2 {
        // TODO: error: Triangle only works with two-dimensional
        // meshes (484).
        return Err(input_failure());
    }
    // C: if (nextras == 0) { //b->weighted = 0; } - a dead comment;
    // the attribute-count store (491) lives at the consumer's seam.

    // Read the vertices (493-531).
    for i in 0..invertices {
        let Some(mut line) = read_line(file) else {
            return Err(Error::Read(read_failure()));
        };
        if i == 0 {
            nodes.first_number = line.strtol() as i32;
        }
        line.find_field();
        if line.at_end() {
            // TODO: error: Vertex (b->firstnumber + i) has no x
            // coordinate (502).
            return Err(input_failure());
        }
        let x = line.strtod();
        line.find_field();
        if line.at_end() {
            // TODO: error: Vertex (b->firstnumber + i) has no y
            // coordinate (507).
            return Err(input_failure());
        }
        let y = line.strtod();
        nodes.points.push(Point2 { x, y });
        // Read the vertex attributes (512-519): a missing field
        // stores 0.0.
        for _ in 0..nextras.max(0) {
            line.find_field();
            if line.at_end() {
                nodes.point_attributes.push(0.0);
            } else {
                nodes.point_attributes.push(line.strtod());
            }
        }
        if nodemarkers != 0 {
            // Read a vertex marker (520-528): a missing field stores
            // 0.
            line.find_field();
            if line.at_end() {
                nodes.point_markers.push(0);
            } else {
                nodes.point_markers.push(line.strtol() as i32);
            }
        }
    }
    Ok(nodes)
}

/// file_readsegments_internal (triangle_io.c:546-607): read the
/// segments of a .poly file into the neutral PolyFile.
fn parse_segments_section<R: Read>(
    file: &mut LineFile<R>,
    poly: &mut PolyFile,
) -> Result<(), Error> {
    // Read number of segments and number of boundary markers
    // (560-572).
    let Some(mut line) = read_line(file) else {
        return Err(Error::Read(read_failure()));
    };
    let insegments = line.strtol() as i32;
    line.find_field();
    let segmentmarkers = if line.at_end() {
        0
    } else {
        line.strtol() as i32
    };
    poly.segment_markers_present = segmentmarkers != 0;

    // Read the segments (581-601).
    for _ in 0..insegments {
        let Some(mut line) = read_line(file) else {
            return Err(Error::Read(read_failure()));
        };
        line.find_field();
        if line.at_end() {
            // TODO: error: Segment (firstnumber + i) has no endpoints
            // (586).
            return Err(input_failure());
        }
        let endpoint1 = line.strtol() as i32;
        line.find_field();
        if line.at_end() {
            // TODO: error: Segment (firstnumber + i) is missing its
            // second endpoint (592).
            return Err(input_failure());
        }
        let endpoint2 = line.strtol() as i32;
        poly.segments.push([endpoint1, endpoint2]);
        if segmentmarkers != 0 {
            // A missing marker field stores 0 (596-600).
            line.find_field();
            if line.at_end() {
                poly.segment_markers.push(0);
            } else {
                poly.segment_markers.push(line.strtol() as i32);
            }
        }
    }
    Ok(())
}

/// file_readholes_internal (triangle_io.c:616-710): read the holes,
/// and possibly regional attributes and area constraints, from a
/// .poly file. A missing area field copies the attribute value
/// exactly like C (regionlist[index] = regionlist[index - 1],
/// triangle_io.c:694-695); a missing regions line is TRI_OK.
fn parse_holes_regions_section<R: Read>(
    file: &mut LineFile<R>,
    poly: &mut PolyFile,
) -> Result<(), Error> {
    // Read the holes (629-647).
    let Some(mut line) = read_line(file) else {
        return Err(Error::Read(read_failure()));
    };
    let holes = line.strtol() as i32;
    if holes > 0 {
        for _ in 0..holes {
            let Some(mut line) = read_line(file) else {
                return Err(Error::Read(read_failure()));
            };
            line.find_field();
            if line.at_end() {
                // TODO: error: Hole (b->firstnumber + (i >> 1)) has
                // no x coordinate (644).
                return Err(input_failure());
            }
            let x = line.strtod();
            line.find_field();
            if line.at_end() {
                // TODO: error: Hole ... has no y coordinate (650).
                return Err(input_failure());
            }
            let y = line.strtod();
            poly.holes.push(Hole { x, y });
        }
    }

    // Read the area constraints (659-705; behind #ifndef CDT_ONLY in
    // C, unconditional here - the port builds the full shape). A
    // missing line ends the file cleanly.
    let Some(mut line) = read_line(file) else {
        return Ok(());
    };
    let regions = line.strtol() as i32;
    if regions > 0 {
        for _ in 0..regions {
            let Some(mut line) = read_line(file) else {
                return Err(Error::Read(read_failure()));
            };
            line.find_field();
            if line.at_end() {
                // TODO: error: Region (b->firstnumber + i) has no x
                // coordinate (673).
                return Err(input_failure());
            }
            let x = line.strtod();
            line.find_field();
            if line.at_end() {
                // TODO: error: Region ... has no y coordinate (679).
                return Err(input_failure());
            }
            let y = line.strtod();
            line.find_field();
            if line.at_end() {
                // TODO: error: Region ... has no region attribute or
                // area constraint (685).
                return Err(input_failure());
            }
            let attribute = line.strtod();
            line.find_field();
            let max_area = if line.at_end() {
                // The missing-area branch copies the attribute value
                // (694-695).
                attribute
            } else {
                line.strtod()
            };
            poly.regions.push(Region {
                x,
                y,
                attribute,
                max_area,
            });
        }
    }
    Ok(())
}

/// file_readnodes (triangle_io.c:712-716).
pub fn read_node_file<R: Read>(reader: R) -> Result<NodeFile, Error> {
    let mut file = LineFile::new(reader);
    parse_node_section(&mut file, false)
}

/// file_readpoly (triangle_io.c:723-736).
pub fn read_poly_file<R: Read>(reader: R) -> Result<PolyFile, Error> {
    let mut file = LineFile::new(reader);
    let mut poly = PolyFile {
        nodes: parse_node_section(&mut file, true)?,
        ..PolyFile::default()
    };
    parse_segments_section(&mut file, &mut poly)?;
    parse_holes_regions_section(&mut file, &mut poly)?;
    Ok(poly)
}

/// file_readelements (triangle_io.c:744-803): read mesh elements from
/// an .ele file. numberofcorners is pinned to 3 regardless of the
/// file's corner count (769); incorners < 3 is C's TODO failure
/// (763-765).
pub fn read_ele_file<R: Read>(reader: R) -> Result<EleFile, Error> {
    let mut file = LineFile::new(reader);
    // Read number of triangles, number of vertices per triangle, and
    // number of triangle attributes from .ele file (757-771).
    let Some(mut line) = read_line(&mut file) else {
        return Err(Error::Read(read_failure()));
    };
    let inelements = line.strtol() as i32;
    line.find_field();
    if !line.at_end() {
        let incorners = line.strtol() as i32;
        if incorners < 3 {
            // TODO: error: Triangles must have at least 3 vertices
            // (763-765).
            return Err(input_failure());
        }
    }
    line.find_field();
    let eextras = if line.at_end() {
        0
    } else {
        line.strtol() as i32
    };

    let mut ele = EleFile {
        element_count: inelements,
        // We don't care if higher order elements are stored in the
        // file (768): the corner count is pinned to 3.
        corners_per_element: 3,
        attributes_per_element: eextras,
        first_number: 0,
        elements: Vec::new(),
        element_attributes: Vec::new(),
    };

    // Read the triangles from the .ele file (779-799).
    for _ in 0..inelements {
        let Some(mut line) = read_line(&mut file) else {
            return Err(Error::Read(read_failure()));
        };
        for _ in 0..3 {
            line.find_field();
            if line.at_end() {
                // TODO: error: Triangle (elementnumber) is missing
                // vertex (j + 1) (787).
                return Err(input_failure());
            }
            ele.elements.push(line.strtol() as i32);
        }
        // Read the triangle's attributes (790-797): a missing field
        // stores 0.0 (C stores the int 0).
        for _ in 0..eextras.max(0) {
            line.find_field();
            if line.at_end() {
                ele.element_attributes.push(0.0);
            } else {
                ele.element_attributes.push(line.strtod());
            }
        }
    }
    Ok(ele)
}

/// The .edge reader. C has no counterpart (triangle_io.c only writes
/// the format, 203-262); this parses the writer's grammar under the
/// readers' own conventions: the header's default fills, the
/// row-number skip, the segment reader's missing-marker 0 default
/// (triangle_io.c:596-600).
pub fn read_edge_file<R: Read>(reader: R) -> Result<EdgeFile, Error> {
    let mut file = LineFile::new(reader);
    // Read number of edges and number of boundary markers (the
    // writer's header, triangle_io.c:217: "%ld  %d\n").
    let Some(mut line) = read_line(&mut file) else {
        return Err(Error::Read(read_failure()));
    };
    let inedges = line.strtol() as i32;
    line.find_field();
    let edgemarkers = if line.at_end() {
        0
    } else {
        line.strtol() as i32
    };

    let mut edges = EdgeFile {
        edge_count: inedges,
        markers_present: edgemarkers != 0,
        first_number: 0,
        edges: Vec::new(),
        edge_markers: Vec::new(),
    };

    // Read the edges (the writer's data lines, triangle_io.c:237-254:
    // "%4ld   %d  %d [  %d]\n").
    for _ in 0..inedges {
        let Some(mut line) = read_line(&mut file) else {
            return Err(Error::Read(read_failure()));
        };
        line.find_field();
        if line.at_end() {
            // TODO: error: Edge (firstnumber + i) is missing an
            // endpoint (the segment reader's rule, 586).
            return Err(input_failure());
        }
        let endpoint1 = line.strtol() as i32;
        line.find_field();
        if line.at_end() {
            // TODO: error: Edge ... is missing its second endpoint
            // (the segment reader's rule, 592).
            return Err(input_failure());
        }
        let endpoint2 = line.strtol() as i32;
        edges.edges.push([endpoint1, endpoint2]);
        if edgemarkers != 0 {
            // A missing marker field stores 0 (the segment reader's
            // rule, 596-600).
            line.find_field();
            if line.at_end() {
                edges.edge_markers.push(0);
            } else {
                edges.edge_markers.push(line.strtol() as i32);
            }
        }
    }
    Ok(edges)
}

/// The .neigh reader. C has no counterpart (triangle_io.c only writes
/// the format, 264-310); this parses the writer's grammar under the
/// .ele reader's conventions: the neighbors-per-element header field
/// defaults to 3 and below 3 is malformed input (the incorners < 3
/// rule, triangle_io.c:763-765), and three neighbors are read per row
/// regardless (the numberofcorners pin, 769).
pub fn read_neigh_file<R: Read>(reader: R) -> Result<NeighFile, Error> {
    let mut file = LineFile::new(reader);
    // Read number of triangles and neighbors per element (the
    // writer's header, triangle_io.c:276: "%ld  %d\n" with 3).
    let Some(mut line) = read_line(&mut file) else {
        return Err(Error::Read(read_failure()));
    };
    let inelements = line.strtol() as i32;
    line.find_field();
    let neighbors_per_element = if line.at_end() {
        3
    } else {
        line.strtol() as i32
    };
    if neighbors_per_element < 3 {
        // TODO: error: Triangles must have at least 3 neighbors (the
        // .ele incorners rule, 763-765).
        return Err(input_failure());
    }

    let mut neigh = NeighFile {
        element_count: inelements,
        first_number: 0,
        neighbors: Vec::new(),
    };

    // Read the neighbors (the writer's data lines, triangle_io.c:
    // 303-304: "%4ld    %d  %d  %d\n").
    for _ in 0..inelements {
        let Some(mut line) = read_line(&mut file) else {
            return Err(Error::Read(read_failure()));
        };
        for _ in 0..3 {
            line.find_field();
            if line.at_end() {
                // TODO: error: Triangle (elementnumber) is missing
                // neighbor (j + 1) (the .ele vertex rule, 787).
                return Err(input_failure());
            }
            neigh.neighbors.push(line.strtol() as i32);
        }
    }
    Ok(neigh)
}

/// file_readelementsarea (triangle_io.c:828-876): read element areas
/// from an .area file. The element count must agree with the .ele
/// read (857-861) - the caller passes the count its .ele parse
/// produced, exactly where C reads io->numberoftriangles; a missing
/// field stores -1.0 (865-867).
pub fn read_area_file<R: Read>(reader: R, numelements: usize) -> Result<AreaFile, Error> {
    let mut file = LineFile::new(reader);
    // Check for consistency with the .ele file (841-861).
    let Some(mut line) = read_line(&mut file) else {
        return Err(Error::Read(read_failure()));
    };
    let areaelements = line.strtol() as i32;
    if areaelements != numelements as i32 {
        // TODO: error: area file disagrees on number of triangles
        // (858-861).
        return Err(input_failure());
    }

    let mut area = AreaFile {
        first_number: 0,
        areas: Vec::new(),
    };

    // Read the triangles from the .area file (871-875).
    for _ in 0..numelements {
        let Some(mut line) = read_line(&mut file) else {
            return Err(Error::Read(read_failure()));
        };
        line.find_field();
        if line.at_end() {
            // No constraint on this triangle (866).
            area.areas.push(-1.0);
        } else {
            area.areas.push(line.strtod());
        }
    }
    Ok(area)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    // -- the parse seam, pinned by construction from C's semantics --

    #[test]
    fn read_line_skips_comments_and_finds_numbers() {
        let src = "# comment only\n  \nabc\n1 2 3\n  4.5\n";
        let mut f = LineFile::new(Cursor::new(src.as_bytes().to_vec()));
        let line = read_line(&mut f).expect("line 1");
        assert_eq!(&line.buf[line.pos..], b"1 2 3\n");
        let line = read_line(&mut f).expect("line 2");
        assert_eq!(&line.buf[line.pos..], b"4.5\n");
        assert!(read_line(&mut f).is_none(), "EOF -> NULL");
    }

    #[test]
    fn find_field_walks_fields_and_truncates_comments() {
        let mut f = LineFile::new(Cursor::new(b"1 0.5\t0.25 7 # tail\n".to_vec()));
        let mut line = read_line(&mut f).expect("line");
        assert_eq!(line.strtol(), 1);
        line.find_field();
        assert!(!line.at_end());
        assert_eq!(line.strtod(), 0.5);
        line.find_field();
        assert_eq!(line.strtod(), 0.25);
        line.find_field();
        assert_eq!(line.strtol(), 7);
        // The '#' truncated the line: the next field is '\0'.
        line.find_field();
        assert!(line.at_end());
    }

    #[test]
    fn find_field_skips_extra_separators_like_c() {
        // C's second loop does not stop at whitespace: a run of
        // separators is jumped entirely.
        let mut f = LineFile::new(Cursor::new(b"5   6\n".to_vec()));
        let mut line = read_line(&mut f).expect("line");
        assert_eq!(line.strtol(), 5);
        line.find_field();
        assert_eq!(line.strtol(), 6);
    }

    #[test]
    fn strtol_base_zero_grammar() {
        let mk = |txt: &str| {
            let mut f = LineFile::new(Cursor::new(txt.as_bytes().to_vec()));
            read_line(&mut f).expect("line")
        };
        let mut l = mk("42");
        assert_eq!(l.strtol(), 42);
        let mut l = mk("-17");
        assert_eq!(l.strtol(), -17);
        let mut l = mk("0x1f 5");
        assert_eq!(l.strtol(), 31);
        l.find_field();
        assert_eq!(l.strtol(), 5);
        let mut l = mk("010 8");
        // Octal: "010" is 8, and the '8' ends the prefix.
        assert_eq!(l.strtol(), 8);
        let mut l = mk("09");
        // Octal: "9" is invalid, so the longest valid prefix is "0".
        assert_eq!(l.strtol(), 0);
        assert_eq!(&l.buf[l.pos..], b"9");
        let mut l = mk("0x no-digit");
        // "0x" without a hex digit parses the decimal "0".
        assert_eq!(l.strtol(), 0);
        assert_eq!(&l.buf[l.pos..], b"x no-digit");
        let mut l = mk("  12rest");
        assert_eq!(l.strtol(), 12);
        assert_eq!(&l.buf[l.pos..], b"rest");
        // No digits: 0, cursor unadvanced (C's endptr = nptr).
        // The line is built directly: a line with no
        // number-ish character never survives readline (C
        // rescans to EOF and returns NULL - the readers'
        // TRI_FILE_READ arm).
        let mut l = Line {
            buf: b"abc".to_vec(),
            pos: 0,
        };
        assert_eq!(l.strtol(), 0);
        assert_eq!(l.pos, 0);
    }

    #[test]
    // The long decimal literals are the point of the pin.
    #[allow(clippy::excessive_precision)]
    fn strtod_decimal_grammar_and_partial_exponents() {
        let mk = |txt: &str| {
            let mut f = LineFile::new(Cursor::new(txt.as_bytes().to_vec()));
            read_line(&mut f).expect("line")
        };
        let mut l = mk("0.5");
        assert_eq!(l.strtod(), 0.5);
        let mut l = mk("1.");
        assert_eq!(l.strtod(), 1.0);
        let mut l = mk(".5");
        assert_eq!(l.strtod(), 0.5);
        let mut l = mk("1e5");
        assert_eq!(l.strtod(), 1e5);
        let mut l = mk("1e");
        // The exponent without digits is not consumed.
        assert_eq!(l.strtod(), 1.0);
        assert_eq!(&l.buf[l.pos..], b"e");
        let mut l = mk("1E5");
        assert_eq!(l.strtod(), 1e5);
        let mut l = mk("-2.5e-3");
        assert_eq!(l.strtod(), -2.5e-3);
        let mut l = mk("1.10000000000000001");
        // Correct rounding: the 17-digit decimal and the short literal
        // name the same nearest double.
        assert_eq!(l.strtod(), 1.10000000000000001_f64);
        // The exact decimal expansion of 0.1's double is exactly
        // representable; the parse must return that double, not drift.
        let mut l = mk("0.1000000000000000055511151231257827021181583404541015625");
        assert_eq!(l.strtod(), 0.1_f64);
        // INF / nan / no-prefix forms are pinned at the strtod
        // seam with hand-built lines: C's readline never yields
        // a line without a number-ish character ('I', 'n', 'x'
        // are not in the lookahead set), so these grammars
        // cannot reach strtod through a file.
        let mut l = Line {
            buf: b"INF".to_vec(),
            pos: 0,
        };
        assert_eq!(l.strtod(), f64::INFINITY);
        let mut l = mk("-infinity");
        assert_eq!(l.strtod(), f64::NEG_INFINITY);
        let mut l = Line {
            buf: b"nan".to_vec(),
            pos: 0,
        };
        assert!(l.strtod().is_nan());
        let mut l = Line {
            buf: b"nan(123)".to_vec(),
            pos: 0,
        };
        assert!(l.strtod().is_nan());
        let mut l = Line {
            buf: b"xyz".to_vec(),
            pos: 0,
        };
        // No valid prefix: 0.0, cursor unadvanced.
        assert_eq!(l.strtod(), 0.0);
        assert_eq!(l.pos, 0);
    }

    #[test]
    fn strtod_hex_floats_like_glibc() {
        let mk = |txt: &str| {
            let mut f = LineFile::new(Cursor::new(txt.as_bytes().to_vec()));
            read_line(&mut f).expect("line")
        };
        let mut l = mk("0x1.8p3");
        assert_eq!(l.strtod(), 12.0);
        let mut l = mk("0x1p-1074");
        assert_eq!(l.strtod(), f64::from_bits(1));
        let mut l = mk("-0x10");
        assert_eq!(l.strtod(), -16.0);
        let mut l = mk("0x1.00000000000008p1 5");
        // 2 + 2^-52 is exactly halfway between 2.0 and
        // 2 + 2^-51: round-half-to-even picks 2.0.
        assert_eq!(l.strtod(), 2.0);
        l.find_field();
        assert_eq!(l.strtol(), 5);
        let mut l = mk("0x.8p1");
        assert_eq!(l.strtod(), 1.0);
    }

    #[test]
    fn fgets_caps_at_input_line_size_and_keeps_remainder() {
        // A line longer than INPUTLINESIZE - 1 continues on the next
        // fgets, exactly like C.
        let long = "1".repeat(INPUT_LINE_SIZE + 20) + "\n2\n";
        let mut f = LineFile::new(Cursor::new(long.into_bytes()));
        let first = f.fgets().expect("first");
        assert_eq!(first.len(), INPUT_LINE_SIZE - 1);
        let second = f.fgets().expect("second");
        assert_eq!(second.last(), Some(&b'\n'));
        // A final line without a newline still reads.
        let mut f = LineFile::new(Cursor::new(b"tail".to_vec()));
        assert_eq!(f.fgets().expect("tail"), b"tail".to_vec());
        assert!(f.fgets().is_none());
    }

    // -- the readers, pinned by construction from the file grammar --

    fn read_poly_string(src: &str) -> PolyFile {
        read_poly_file(Cursor::new(src.as_bytes().to_vec())).expect("readpoly")
    }

    #[test]
    fn readpoly_fills_the_owned_input() {
        let src = "# a poly file\n\
                    4 2 1 1\n\
                    5 0.5 0.25 7 1\n\
                    6 1 0\n\
                    7 1 1 -2.5\n\
                    8 0 1 3 2\n\
                    1 1\n\
                    1 6 5 9\n\
                    1\n\
                    1 0.5 0.5\n\
                    1\n\
                    1 0.25 0.75 0.5\n";
        let poly = read_poly_string(src);
        assert_eq!(poly.nodes.first_number, 5, "captured at i == 0");
        assert_eq!(poly.nodes.attributes_per_point, 1);
        assert_eq!(
            poly.nodes.points,
            vec![
                Point2 { x: 0.5, y: 0.25 },
                Point2 { x: 1.0, y: 0.0 },
                Point2 { x: 1.0, y: 1.0 },
                Point2 { x: 0.0, y: 1.0 },
            ]
        );
        // Missing attribute / marker fields fill defaults (0.0 / 0).
        assert_eq!(poly.nodes.point_attributes, vec![7.0, 0.0, -2.5, 3.0]);
        assert_eq!(poly.nodes.point_markers, vec![1, 0, 0, 2]);
        assert_eq!(poly.segments, vec![[6, 5]]);
        assert_eq!(poly.segment_markers, vec![9]);
        assert!(poly.segment_markers_present);
        assert_eq!(poly.holes, vec![Hole { x: 0.5, y: 0.5 }]);
        // The missing-area branch copies the attribute value.
        assert_eq!(
            poly.regions,
            vec![Region {
                x: 0.25,
                y: 0.75,
                attribute: 0.5,
                max_area: 0.5,
            }]
        );
    }

    #[test]
    fn readpoly_with_vertices_in_a_node_file() {
        // The poly gate: a zero vertex count returns OK so the caller
        // reads the separate .node file.
        let src = "0 2 0 0\n0 0\n0\n0\n";
        let poly = read_poly_string(src);
        assert!(poly.nodes.separate_file);
        assert!(poly.nodes.points.is_empty());
        assert!(poly.segments.is_empty());
        // The separate-file arm leaves the header fields captured and
        // the numbering untouched (the consumer's firstnode store is
        // the consumer's business).
        assert_eq!(poly.nodes.first_number, 0);
        assert_eq!(poly.nodes.dim, 2);
        // Then the node reader fills the same input.
        let src = "3 2 0 1\n9 0 0 1\n10 1 0 1\n11 0 1 1\n";
        let nodes = read_node_file(Cursor::new(src.as_bytes().to_vec())).expect("readnodes");
        assert_eq!(nodes.first_number, 9);
        assert_eq!(nodes.points.len(), 3);
        assert_eq!(nodes.point_markers, vec![1, 1, 1]);
    }

    #[test]
    fn readnodes_rejects_short_and_nonplanar() {
        let r = read_node_file(Cursor::new(b"2 2 0 0\n".to_vec()));
        assert!(matches!(r, Err(Error::Input(ref msg)) if msg == "invalid input (TRI_FAILURE)"));
        let r = read_node_file(Cursor::new(b"3 3 0 0\n".to_vec()));
        assert!(matches!(r, Err(Error::Input(ref msg)) if msg == "invalid input (TRI_FAILURE)"));
        // A premature EOF raises Error::Read at the seam with the
        // funnel arm's exact payload.
        let r = read_node_file(Cursor::new(b"3 2 0 0\n1 0 0\n".to_vec()));
        assert!(
            matches!(r, Err(Error::Read(ref e)) if e.to_string() == "file read failure (TRI_FILE_READ)")
        );
    }

    #[test]
    fn readelements_pins_three_corners() {
        // The .ele header says 6 corners; only 3 are stored and
        // read (numberofcorners pinned to 3).
        let src = "2 6 0\n1 2 3 4 5 6 7\n2 3 1 4 5 6 8\n";
        let ele = read_ele_file(Cursor::new(src.as_bytes().to_vec())).expect("readelements");
        assert_eq!(ele.corners_per_element, 3);
        assert_eq!(ele.elements, vec![2, 3, 4, 3, 1, 4]);
        assert!(ele.element_attributes.is_empty());
    }

    #[test]
    fn readelements_fills_attributes_with_zero_default() {
        let src = "2 3 1\n1 2 3 4 9.5\n2 3 1 4\n";
        let ele = read_ele_file(Cursor::new(src.as_bytes().to_vec())).expect("readelements");
        assert_eq!(ele.attributes_per_element, 1);
        assert_eq!(ele.elements, vec![2, 3, 4, 3, 1, 4]);
        // A missing attribute field stores 0.0 (C stores the int 0).
        assert_eq!(ele.element_attributes, vec![9.5, 0.0]);
    }

    #[test]
    fn readelements_rejects_two_corners() {
        let src = "1 2 0\n";
        let r = read_ele_file(Cursor::new(src.as_bytes().to_vec()));
        assert!(matches!(r, Err(Error::Input(ref msg)) if msg == "invalid input (TRI_FAILURE)"));
    }

    #[test]
    fn readelementsarea_checks_count_and_fills_minus_one() {
        // The area count must agree with the element count (2 vs 3):
        // C's TODO failure.
        let src = "2 2\n1 0.25\n2\n";
        let r = read_area_file(Cursor::new(src.as_bytes().to_vec()), 3);
        assert!(matches!(r, Err(Error::Input(ref msg)) if msg == "invalid input (TRI_FAILURE)"));
        // Matching count: a missing field stores -1.0 (866).
        let src = "3 2\n1 0.25\n2\n3\n";
        let area = read_area_file(Cursor::new(src.as_bytes().to_vec()), 3).expect("area");
        assert_eq!(area.areas, vec![0.25, -1.0, -1.0]);
    }

    // -- the new-format readers, pinned by construction against the
    //    writer grammar (C writes, never reads these formats) --

    #[test]
    fn readedge_reads_the_writer_grammar() {
        // Markers on: a missing marker field stores 0.
        let src = "2 1\n1 5 6 0\n2 7 8\n";
        let edges = read_edge_file(Cursor::new(src.as_bytes().to_vec())).expect("readedges");
        assert_eq!(edges.edge_count, 2);
        assert!(edges.markers_present);
        assert_eq!(edges.edges, vec![[5, 6], [7, 8]]);
        assert_eq!(edges.edge_markers, vec![0, 0]);
        // Markers off: no marker list is read at all.
        let src = "1 0\n1 5 6 9\n";
        let edges = read_edge_file(Cursor::new(src.as_bytes().to_vec())).expect("readedges");
        assert!(!edges.markers_present);
        assert!(edges.edge_markers.is_empty());
        // A missing endpoint is the TODO failure; a premature EOF is
        // TRI_FILE_READ.
        let r = read_edge_file(Cursor::new(b"1 0\n1 5\n".to_vec()));
        assert!(matches!(r, Err(Error::Input(ref msg)) if msg == "invalid input (TRI_FAILURE)"));
        let r = read_edge_file(Cursor::new(b"1 0\n".to_vec()));
        assert!(
            matches!(r, Err(Error::Read(ref e)) if e.to_string() == "file read failure (TRI_FILE_READ)")
        );
    }

    #[test]
    fn readneigh_pins_three_neighbors_and_rejects_short_headers() {
        let src = "2 3\n1 -1 2 -1\n2 -1 -1 1\n";
        let neigh = read_neigh_file(Cursor::new(src.as_bytes().to_vec())).expect("readneigh");
        assert_eq!(neigh.element_count, 2);
        assert_eq!(neigh.neighbors, vec![-1, 2, -1, -1, -1, 1]);
        // The missing header field defaults to 3 (the .ele reader's
        // default-fill rule).
        let src = "1\n1 -1 -1 -1\n";
        let neigh = read_neigh_file(Cursor::new(src.as_bytes().to_vec())).expect("readneigh");
        assert_eq!(neigh.neighbors, vec![-1, -1, -1]);
        // Below 3 neighbors is malformed input (the incorners < 3
        // rule); a missing neighbor is the TODO failure.
        let r = read_neigh_file(Cursor::new(b"1 2\n".to_vec()));
        assert!(matches!(r, Err(Error::Input(ref msg)) if msg == "invalid input (TRI_FAILURE)"));
        let r = read_neigh_file(Cursor::new(b"1 3\n1 -1 -1\n".to_vec()));
        assert!(matches!(r, Err(Error::Input(ref msg)) if msg == "invalid input (TRI_FAILURE)"));
    }
}
