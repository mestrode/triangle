//! File I/O: all of triangle_io.c (the six file writers, the
//! readline/findfield/strtol/strtod parse seam, the readers) and the
//! file entry points of triangle_api.c (the api.rs wrappers over
//! these). Assignment: the triangle_io.c and triangle-api file rows of
//! docs/function-mapping.md.
//!
//! Boundary decisions (the deviation register and the PR body carry
//! the details):
//! - The C FILE* NULL checks (file == NULL -> TRI_FILE_OPEN,
//!   triangle_io.c:30, 86, 141, 212, 271, 318, 443, 555, 625, 755,
//!   837; triangle_api.c fopen NULL -> TRI_FILE_OPEN at 498, 514, 530,
//!   546) dissolve: the port's api.rs wrappers own the open, and
//!   File::open's error carries straight into Error::Io (the
//!   TRI_FILE_OPEN equivalence of the D8 funnel).
//! - C's unchecked fprintf return values in the writers surface as
//!   io::Result in the port (register row: the writers' seam). Every
//!   reader failure keeps C's code: TRI_FILE_READ (fgets NULL) and the
//!   raw -1 TODO returns both keep their InternalStatus codes and map
//!   through the existing D8 funnel (Failure -> Error::Input).
//! - fgets mirrors over a byte buffer with INPUTLINESIZE semantics
//!   (triangle_config.h:114); read errors fold to NULL exactly like
//!   C's fgets, so a mid-file I/O failure reads as end-of-file.
//! - strtol/strtod mirror glibc's semantics at the call sites: base-0
//!   longest-prefix rules (hex/octal), endptr advancement (including
//!   the no-advance-on-no-digits rule), strtod's decimal grammar with
//!   inf/nan and glibc's hex floats, all correctly rounded (Rust's
//!   f64 parser is correctly rounded like glibc's).

// Clippy: the writers keep C's fprintf format strings verbatim,
// trailing \n included (triangle_io.c) - writeln! would move the
// newline out of the C literal.
#![allow(clippy::write_with_newline)]

use std::io::{BufReader, Read, Write};

use crate::error::InternalStatus;
use crate::formatter::{d, d4, g, ld, ld4};
use crate::handles::{Osub, Otri, DUMMY_SUB, DUMMY_TRI};
use crate::input::{Hole, MeshInput, Point2, Region};
use crate::mesh::Mesh;
use crate::options::Behavior;
use crate::slots::UNDEADVERTEX;

// ---------------------------------------------------------------------------
// File writing routines (triangle_io.c:10-352).
// ---------------------------------------------------------------------------

/// file_writenodes (triangle_io.c:23-70): number the vertices over
/// their boundary markers (the setvertexmark store inside the walk,
/// triangle_io.c:64-65) and write the .node file. The writers inline
/// their own numbering exactly like C; numbernodes gains no consumer.
pub(crate) fn file_writenodes(
    m: &mut Mesh,
    b: &Behavior,
    nodefile: &mut dyn Write,
) -> std::io::Result<()> {
    // C: outvertices (triangle_io.c:34-38): jettison shrinks the
    // output count by the undead.
    let outvertices: i64 = if b.jettison {
        m.vertices.live_count() as i64 - m.undeads as i64
    } else {
        m.vertices.live_count() as i64
    };

    // Number of vertices, number of dimensions, number of vertex
    // attributes, and number of boundary markers (zero or one)
    // (triangle_io.c:42-43: "%ld  %d  %d  %d\n").
    write!(
        nodefile,
        "{}  {}  {}  {}\n",
        ld(outvertices),
        d(m.mesh_dim),
        d(m.nextras),
        d(1 - i32::from(b.nobound))
    )?;

    // C: traversalinit + vertexnumber = b->firstnumber (45-46).
    let mut vertexnumber = b.firstnumber;
    let mut t = m.vertices.traversal();
    while let Some(vertexloop) = m.vertextraverse(&mut t) {
        // C: the jettison gate (triangle_io.c:49).
        if !b.jettison || m.vx_type(vertexloop) != UNDEADVERTEX {
            // Vertex number, x and y coordinates (51-52:
            // "%4d    %.17g  %.17g").
            let mut line = format!(
                "{}    {}  {}",
                d4(vertexnumber),
                g(m.vx_real(vertexloop, 0), 17),
                g(m.vx_real(vertexloop, 1), 17)
            );
            for i in 0..m.nextras as usize {
                // Write an attribute (54-55: "  %.17g").
                line.push_str("  ");
                line.push_str(&g(m.vx_real(vertexloop, 2 + i), 17));
            }
            if b.nobound {
                line.push('\n');
            } else {
                // Write the boundary marker (61: "    %d\n") - the
                // ORIGINAL marker: the number store below happens
                // after the fprintf, exactly like C.
                line.push_str("    ");
                line.push_str(&d(m.vx_mark(vertexloop)));
                line.push('\n');
            }
            nodefile.write_all(line.as_bytes())?;

            // To save memory, the vertex numbers are written over the
            // boundary markers (triangle_io.c:64-65).
            m.vx_set_mark(vertexloop, vertexnumber);
            vertexnumber += 1;
        }
    }
    Ok(())
}

/// file_writeelements (triangle_io.c:78-125): the .ele writer. Reads
/// the vertex numbers writenodes left on the markers.
pub(crate) fn file_writeelements(
    m: &mut Mesh,
    b: &Behavior,
    elefile: &mut dyn Write,
) -> std::io::Result<()> {
    // Number of triangles, vertices per triangle, attributes per
    // triangle (91-92: "%ld  %d  %d\n").
    write!(
        elefile,
        "{}  {}  {}\n",
        ld(m.triangles.live_count() as i64),
        d((b.order + 1) * (b.order + 2) / 2),
        d(m.eextras)
    )?;

    let mut elementnumber: i64 = b.firstnumber as i64;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.triangletraverse(&mut t) {
        let triangleloop = Otri { tri, orient: 0 };
        let p1 = m.org(triangleloop);
        let p2 = m.dest(triangleloop);
        let p3 = m.apex(triangleloop);
        let mut line = if b.order == 1 {
            // Triangle number, indices for three vertices (104-105:
            // "%4ld    %4d  %4d  %4d").
            format!(
                "{}    {}  {}  {}",
                ld4(elementnumber),
                d4(m.vx_mark(p1)),
                d4(m.vx_mark(p2)),
                d4(m.vx_mark(p3))
            )
        } else {
            // C: the mid-edge nodes (107-109: highorderindex + 1,
            // + 2 and highorderindex, exactly where highorder stored
            // them).
            let mid1 = m.tri_slot(tri, m.tri_shape.highorderindex + 1) as u32;
            let mid2 = m.tri_slot(tri, m.tri_shape.highorderindex + 2) as u32;
            let mid3 = m.tri_slot(tri, m.tri_shape.highorderindex) as u32;
            // Triangle number, indices for six vertices (111-113).
            format!(
                "{}    {}  {}  {}  {}  {}  {}",
                ld4(elementnumber),
                d4(m.vx_mark(p1)),
                d4(m.vx_mark(p2)),
                d4(m.vx_mark(p3)),
                d4(m.vx_mark(mid1)),
                d4(m.vx_mark(mid2)),
                d4(m.vx_mark(mid3))
            )
        };
        for i in 0..m.eextras as usize {
            // Write an attribute (116-117: "  %.17g").
            line.push_str("  ");
            line.push_str(&g(m.tri_attribute(tri, i), 17));
        }
        line.push('\n');
        elefile.write_all(line.as_bytes())?;

        elementnumber += 1;
    }
    Ok(())
}

/// file_writepoly (triangle_io.c:133-195): the .poly writer. The hole
/// and region lists come from the caller exactly like C's holelist /
/// regionlist parameters (triangle_api.c:447-449).
pub(crate) fn file_writepoly(
    m: &mut Mesh,
    b: &Behavior,
    polyfile: &mut dyn Write,
    holelist: &[Hole],
    regionlist: &[Region],
) -> std::io::Result<()> {
    // The zero indicates that the vertices are in a separate .node
    // file, followed by number of dimensions, number of vertex
    // attributes, and number of boundary markers (148-149:
    // "%d  %d  %d  %d\n").
    write!(
        polyfile,
        "{}  {}  {}  {}\n",
        d(0),
        d(m.mesh_dim),
        d(m.nextras),
        d(1 - i32::from(b.nobound))
    )?;
    // Number of segments, number of boundary markers (151:
    // "%ld  %d\n").
    write!(
        polyfile,
        "{}  {}\n",
        ld(m.subsegs.live_count() as i64),
        d(1 - i32::from(b.nobound))
    )?;

    let mut subsegnumber: i64 = b.firstnumber as i64;
    let mut t = m.subsegs.traversal();
    while let Some(ss) = m.subsegtraverse(&mut t) {
        let subsegloop = Osub { ss, ssorient: 0 };
        let endpoint1 = m.sorg(subsegloop);
        let endpoint2 = m.sdest(subsegloop);
        // Segment number, indices of its two endpoints, and possibly
        // a marker (161-167).
        let line = if b.nobound {
            format!(
                "{}    {}  {}\n",
                ld4(subsegnumber),
                d4(m.vx_mark(endpoint1)),
                d4(m.vx_mark(endpoint2))
            )
        } else {
            format!(
                "{}    {}  {}    {}\n",
                ld4(subsegnumber),
                d4(m.vx_mark(endpoint1)),
                d4(m.vx_mark(endpoint2)),
                d4(m.sub_mark(subsegloop.ss))
            )
        };
        polyfile.write_all(line.as_bytes())?;

        subsegnumber += 1;
    }

    // C: the hole tail (173-181; behind #ifndef CDT_ONLY, which this
    // fork's build leaves undefined, so the tails always render in
    // the measured binary - unconditional here, the port builds the
    // full shape). The count line prints even when there are no holes.
    write!(polyfile, "{}\n", d(holelist.len() as i32))?;
    for (holenumber, hole) in holelist.iter().enumerate() {
        // Hole number, x and y coordinates (178-179:
        // "%4ld   %.17g  %.17g\n").
        write!(
            polyfile,
            "{}   {}  {}\n",
            ld4(b.firstnumber as i64 + holenumber as i64),
            g(hole.x, 17),
            g(hole.y, 17)
        )?;
    }
    if !regionlist.is_empty() {
        write!(polyfile, "{}\n", d(regionlist.len() as i32))?;
        for (regionnumber, region) in regionlist.iter().enumerate() {
            // Region number, x and y coordinates, attribute, maximum
            // area (186-190: "%4ld   %.17g  %.17g  %.17g  %.17g\n").
            write!(
                polyfile,
                "{}   {}  {}  {}  {}\n",
                ld4(b.firstnumber as i64 + regionnumber as i64),
                g(region.x, 17),
                g(region.y, 17),
                g(region.attribute, 17),
                g(region.max_area, 17)
            )?;
        }
    }
    Ok(())
}

/// file_writeedges (triangle_io.c:203-262): the .edge writer.
pub(crate) fn file_writeedges(
    m: &mut Mesh,
    b: &Behavior,
    edgefile: &mut dyn Write,
) -> std::io::Result<()> {
    // Number of edges, number of boundary markers (217:
    // "%ld  %d\n").
    write!(
        edgefile,
        "{}  {}\n",
        ld(m.edges),
        d(1 - i32::from(b.nobound))
    )?;

    let mut edgenumber: i64 = b.firstnumber as i64;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.triangletraverse(&mut t) {
        for orient in 0..3u8 {
            let triangleloop = Otri { tri, orient };
            let trisym = m.sym(triangleloop);
            // C: (triangleloop.tri < trisym.tri) ||
            // (trisym.tri == m->dummytri) (triangle_io.c:232), the
            // address compare first. The port evaluates the dummy arm
            // first as a deliberate, registered deviation (D29, the
            // D18/D28 precedent): the packed row-id mirror of the
            // address order cannot evaluate the reserved DUMMY_TRI
            // sentinel row, so the dummy arm must short-circuit
            // before it. The boolean is equivalent on every input (a
            // hull edge passes through the dummy arm either way; an
            // interior edge is emitted from the same side through
            // the oracle address order, ADR 0003 section 5).
            if trisym.tri == DUMMY_TRI
                || m.triangles.oracle_address_order(tri, trisym.tri) == std::cmp::Ordering::Less
            {
                let p1 = m.org(triangleloop);
                let p2 = m.dest(triangleloop);
                let line = if b.nobound {
                    // Edge number, indices of two endpoints (237:
                    // "%4ld   %d  %d\n").
                    format!(
                        "{}   {}  {}\n",
                        ld4(edgenumber),
                        d(m.vx_mark(p1)),
                        d(m.vx_mark(p2))
                    )
                } else {
                    // Edge number, indices of two endpoints, and a
                    // boundary marker. If there's no subsegment, the
                    // boundary marker is zero (240-254).
                    let marker = if b.usesegments {
                        let checkmark = m.tspivot(triangleloop);
                        if checkmark.ss == DUMMY_SUB {
                            0
                        } else {
                            m.sub_mark(checkmark.ss)
                        }
                    } else {
                        // C: trisym.tri == m->dummytri (253) - the
                        // hull flag itself.
                        i32::from(trisym.tri == DUMMY_TRI)
                    };
                    format!(
                        "{}   {}  {}  {}\n",
                        ld4(edgenumber),
                        d(m.vx_mark(p1)),
                        d(m.vx_mark(p2)),
                        d(marker)
                    )
                };
                edgefile.write_all(line.as_bytes())?;
                edgenumber += 1;
            }
        }
    }
    Ok(())
}

/// file_writeneighbors (triangle_io.c:264-310): the .neigh writer.
/// The word-6 overlay rides the D2 numbering int exactly like the
/// array writer (ioarrays.rs); unlike the array writer, C's file
/// writer saves nothing and restores nothing (triangle_io.c has no
/// subsegs scratch - only the array writer at triangle.c:8078,
/// 8119-8129 restores), so the port does not either: the element
/// numbers stay over the overlay, and the dummy's -1 stays.
pub(crate) fn file_writeneighbors(
    m: &mut Mesh,
    b: &Behavior,
    neighborfile: &mut dyn Write,
) -> std::io::Result<()> {
    // Number of triangles, three neighbors per triangle (276:
    // "%ld  %d\n").
    write!(
        neighborfile,
        "{}  {}\n",
        ld(m.triangles.live_count() as i64),
        d(3)
    )?;

    // C puns *(int *)(triangleloop.tri + 6) unconditionally
    // (triangle_io.c:283): word 6 is the numbering word of the
    // widened -n row (the ioarrays precedent; the port refuses the
    // past-row pun of the bare row, D2's stance on C's own TODO).
    let numberindex = m.tri_shape.numberindex.expect(
        "the file_writeneighbors numbering overlay exists only with -n \
         (C puns word 6 at triangle_io.c:283, its own TODO)",
    );

    // Number the triangles (282-286): the overlay store inside the
    // first walk, elementnumber from b->firstnumber.
    let mut elementnumber: i64 = b.firstnumber as i64;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.triangletraverse(&mut t) {
        m.set_tri_int(tri, numberindex, elementnumber as i32);
        elementnumber += 1;
    }
    // C: *(int *)(m->dummytri + 6) = -1 (287), never restored.
    m.set_tri_int(DUMMY_TRI, numberindex, -1);

    // The second walk (289-308): neighbor1 from orient 1, neighbor2
    // from orient 2, neighbor3 from orient 0.
    let mut elementnumber: i64 = b.firstnumber as i64;
    let mut t = m.triangles.traversal();
    while let Some(tri) = m.triangletraverse(&mut t) {
        let neighbor1 = m.tri_int(m.sym(Otri { tri, orient: 1 }).tri, numberindex);
        let neighbor2 = m.tri_int(m.sym(Otri { tri, orient: 2 }).tri, numberindex);
        let neighbor3 = m.tri_int(m.sym(Otri { tri, orient: 0 }).tri, numberindex);
        // Triangle number, neighboring triangle numbers (303-304:
        // "%4ld    %d  %d  %d\n").
        write!(
            neighborfile,
            "{}    {}  {}  {}\n",
            ld4(elementnumber),
            d(neighbor1),
            d(neighbor2),
            d(neighbor3)
        )?;
        elementnumber += 1;
    }
    Ok(())
}

/// file_write_eps (triangle_io.c:312-351): the .eps writer. Composes
/// the eps_writer.c unit exactly where C calls it (331-347), with the
/// fixed page and clip rectangles (323-330) and the mesh bounds
/// (332-336).
pub(crate) fn file_write_eps(
    m: &mut Mesh,
    _b: &Behavior,
    file: &mut dyn Write,
) -> std::io::Result<()> {
    use crate::eps::Rect;
    // EPS page metrics (323-326).
    let mut ps = Rect {
        xmin: 36.0,
        ymin: 126.0,
        xmax: 576.0,
        ymax: 666.0,
    };
    // EPS page clip metrics (328-331).
    let mut clip = Rect {
        xmin: 18.0,
        ymin: 108.0,
        xmax: 594.0,
        ymax: 684.0,
    };
    // Mesh metrics (333-337).
    let mut ms = Rect {
        xmin: m.xmin,
        ymin: m.ymin,
        xmax: m.xmax,
        ymax: m.ymax,
    };

    crate::eps::eps_update_metrics(&mut ps, &mut clip, &mut ms);

    crate::eps::eps_write_header(file, "mesh.eps", &ps)?;
    crate::eps::eps_draw_clip(file, &ps, &clip)?;
    crate::eps::eps_draw_edges(file, m, &ps, &ms)?;
    crate::eps::eps_draw_segments(file, m, &ps, &ms)?;
    crate::eps::eps_draw_points(file, m, &ps, &ms)?;
    crate::eps::eps_write_trailer(file)?;
    Ok(())
}

// ---------------------------------------------------------------------------
// The parse seam: readline / findfield / strtol / strtod
// (triangle_io.c:364-423; INPUTLINESIZE, triangle_config.h:114).
// ---------------------------------------------------------------------------

/// INPUTLINESIZE (triangle_config.h:114): fgets reads at most
/// INPUTLINESIZE - 1 bytes per call.
const INPUTLINESIZE: usize = 1024;

/// The fgets substrate: a byte reader with C's fgets semantics over
/// any std::io::Read (C's FILE* - the port's readers never need
/// seeking or fflush, so the buffered byte reader is the whole
/// mirror). Read errors fold to NULL exactly like C's fgets.
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
                    if byte[0] == b'\n' || out.len() == INPUTLINESIZE - 1 {
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
pub(crate) fn readline<R: Read>(infile: &mut LineFile<R>) -> Option<Line> {
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
    fn findfield(&mut self) {
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

// ---------------------------------------------------------------------------
// File reading routines (triangle_io.c:356-876).
// ---------------------------------------------------------------------------

/// file_readnodes_internal (triangle_io.c:432-546): read the vertices
/// from a .node or .poly file. Fills the owned MeshInput exactly like
/// C fills the triangleio: the coordinate list, the attribute list
/// (default 0.0 per missing field), the marker list (default 0) and
/// the attribute count; firstnode is captured at i == 0 from the
/// first vertex's number field (479-481).
pub(crate) fn file_readnodes_internal<R: Read>(
    file: &mut LineFile<R>,
    io: &mut MeshInput,
    poly: bool,
    firstnode: &mut i32,
) -> Result<(), InternalStatus> {
    // Read number of vertices, number of dimensions, number of vertex
    // attributes, and number of boundary markers (450-465), with C's
    // default fills for missing fields.
    let Some(mut line) = readline(file) else {
        return Err(InternalStatus::FileRead);
    };
    let invertices = line.strtol() as i32;
    line.findfield();
    let mesh_dim = if line.at_end() {
        2
    } else {
        line.strtol() as i32
    };
    line.findfield();
    let nextras = if line.at_end() {
        0
    } else {
        line.strtol() as i32
    };
    line.findfield();
    let nodemarkers = if line.at_end() {
        0
    } else {
        line.strtol() as i32
    };

    if poly && invertices == 0 {
        // In case we are reading a .poly file, but the vertices are
        // stored in a separate .node file, return OK so reading of
        // .poly file can continue (472-477).
        io.points.clear();
        return Ok(());
    }

    if invertices < 3 {
        // TODO: error: Input must have at least three input vertices
        // (481).
        return Err(InternalStatus::Failure);
    }
    if mesh_dim != 2 {
        // TODO: error: Triangle only works with two-dimensional
        // meshes (484).
        return Err(InternalStatus::Failure);
    }
    // C: if (nextras == 0) { //b->weighted = 0; } - a dead comment.
    io.attributes_per_point = nextras.max(0) as usize;

    // Read the vertices (493-531).
    for i in 0..invertices {
        let Some(mut line) = readline(file) else {
            return Err(InternalStatus::FileRead);
        };
        if i == 0 {
            *firstnode = line.strtol() as i32;
        }
        line.findfield();
        if line.at_end() {
            // TODO: error: Vertex (b->firstnumber + i) has no x
            // coordinate (502).
            return Err(InternalStatus::Failure);
        }
        let x = line.strtod();
        line.findfield();
        if line.at_end() {
            // TODO: error: Vertex (b->firstnumber + i) has no y
            // coordinate (507).
            return Err(InternalStatus::Failure);
        }
        let y = line.strtod();
        io.points.push(Point2 { x, y });
        // Read the vertex attributes (512-519): a missing field
        // stores 0.0.
        for _ in 0..nextras.max(0) {
            line.findfield();
            if line.at_end() {
                io.point_attributes.push(0.0);
            } else {
                io.point_attributes.push(line.strtod());
            }
        }
        if nodemarkers != 0 {
            // Read a vertex marker (520-528): a missing field stores
            // 0.
            line.findfield();
            if line.at_end() {
                io.point_markers.push(0);
            } else {
                io.point_markers.push(line.strtol() as i32);
            }
        }
    }
    Ok(())
}

/// file_readsegments_internal (triangle_io.c:546-607): read the
/// segments of a .poly file.
pub(crate) fn file_readsegments_internal<R: Read>(
    file: &mut LineFile<R>,
    io: &mut MeshInput,
) -> Result<(), InternalStatus> {
    // Read number of segments and number of boundary markers
    // (560-572).
    let Some(mut line) = readline(file) else {
        return Err(InternalStatus::FileRead);
    };
    let insegments = line.strtol() as i32;
    line.findfield();
    let segmentmarkers = if line.at_end() {
        0
    } else {
        line.strtol() as i32
    };

    // Read the segments (581-601).
    for _ in 0..insegments {
        let Some(mut line) = readline(file) else {
            return Err(InternalStatus::FileRead);
        };
        line.findfield();
        if line.at_end() {
            // TODO: error: Segment (firstnumber + i) has no endpoints
            // (586).
            return Err(InternalStatus::Failure);
        }
        let endpoint1 = line.strtol() as i32;
        line.findfield();
        if line.at_end() {
            // TODO: error: Segment (firstnumber + i) is missing its
            // second endpoint (592).
            return Err(InternalStatus::Failure);
        }
        let endpoint2 = line.strtol() as i32;
        io.segments.push([endpoint1, endpoint2]);
        if segmentmarkers != 0 {
            // A missing marker field stores 0 (596-600).
            line.findfield();
            if line.at_end() {
                io.segment_markers.push(0);
            } else {
                io.segment_markers.push(line.strtol() as i32);
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
pub(crate) fn file_readholes_internal<R: Read>(
    file: &mut LineFile<R>,
    io: &mut MeshInput,
) -> Result<(), InternalStatus> {
    // Read the holes (629-647).
    let Some(mut line) = readline(file) else {
        return Err(InternalStatus::FileRead);
    };
    let holes = line.strtol() as i32;
    if holes > 0 {
        for _ in 0..holes {
            let Some(mut line) = readline(file) else {
                return Err(InternalStatus::FileRead);
            };
            line.findfield();
            if line.at_end() {
                // TODO: error: Hole (b->firstnumber + (i >> 1)) has
                // no x coordinate (644).
                return Err(InternalStatus::Failure);
            }
            let x = line.strtod();
            line.findfield();
            if line.at_end() {
                // TODO: error: Hole ... has no y coordinate (650).
                return Err(InternalStatus::Failure);
            }
            let y = line.strtod();
            io.holes.push(Hole { x, y });
        }
    }

    // Read the area constraints (659-705; behind #ifndef CDT_ONLY in
    // C, unconditional here - the port builds the full shape). A
    // missing line ends the file cleanly.
    let Some(mut line) = readline(file) else {
        return Ok(());
    };
    let regions = line.strtol() as i32;
    if regions > 0 {
        for _ in 0..regions {
            let Some(mut line) = readline(file) else {
                return Err(InternalStatus::FileRead);
            };
            line.findfield();
            if line.at_end() {
                // TODO: error: Region (b->firstnumber + i) has no x
                // coordinate (673).
                return Err(InternalStatus::Failure);
            }
            let x = line.strtod();
            line.findfield();
            if line.at_end() {
                // TODO: error: Region ... has no y coordinate (679).
                return Err(InternalStatus::Failure);
            }
            let y = line.strtod();
            line.findfield();
            if line.at_end() {
                // TODO: error: Region ... has no region attribute or
                // area constraint (685).
                return Err(InternalStatus::Failure);
            }
            let attribute = line.strtod();
            line.findfield();
            let max_area = if line.at_end() {
                // The missing-area branch copies the attribute value
                // (694-695).
                attribute
            } else {
                line.strtod()
            };
            io.regions.push(Region {
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
pub(crate) fn file_readnodes<R: Read>(
    nodefile: &mut LineFile<R>,
    io: &mut MeshInput,
    firstnode: &mut i32,
) -> Result<(), InternalStatus> {
    file_readnodes_internal(nodefile, io, false, firstnode)
}

/// file_readpoly (triangle_io.c:723-736).
pub(crate) fn file_readpoly<R: Read>(
    polyfile: &mut LineFile<R>,
    io: &mut MeshInput,
    firstnode: &mut i32,
) -> Result<(), InternalStatus> {
    file_readnodes_internal(polyfile, io, true, firstnode)?;
    file_readsegments_internal(polyfile, io)?;
    file_readholes_internal(polyfile, io)
}

/// file_readelements (triangle_io.c:744-803): read mesh elements from
/// an .ele file. numberofcorners is pinned to 3 regardless of the
/// file's corner count (769); incorners < 3 is C's TODO failure
/// (763-765).
pub(crate) fn file_readelements<R: Read>(
    file: &mut LineFile<R>,
    io: &mut MeshInput,
) -> Result<(), InternalStatus> {
    // Read number of triangles, number of vertices per triangle, and
    // number of triangle attributes from .ele file (757-771).
    let Some(mut line) = readline(file) else {
        return Err(InternalStatus::FileRead);
    };
    let inelements = line.strtol() as i32;
    line.findfield();
    if !line.at_end() {
        let incorners = line.strtol() as i32;
        if incorners < 3 {
            // TODO: error: Triangles must have at least 3 vertices
            // (763-765).
            return Err(InternalStatus::Failure);
        }
    }
    line.findfield();
    let eextras = if line.at_end() {
        0
    } else {
        line.strtol() as i32
    };

    // We don't care if higher order elements are stored in the file
    // (768): the corner count is pinned to 3.
    io.corners_per_element = 3;
    io.attributes_per_element = eextras.max(0) as usize;

    // Read the triangles from the .ele file (779-799).
    for _ in 0..inelements {
        let Some(mut line) = readline(file) else {
            return Err(InternalStatus::FileRead);
        };
        for _ in 0..3 {
            line.findfield();
            if line.at_end() {
                // TODO: error: Triangle (elementnumber) is missing
                // vertex (j + 1) (787).
                return Err(InternalStatus::Failure);
            }
            io.elements.push(line.strtol() as i32);
        }
        // Read the triangle's attributes (790-797): a missing field
        // stores 0.0 (C stores the int 0).
        for _ in 0..eextras.max(0) {
            line.findfield();
            if line.at_end() {
                io.element_attributes.push(0.0);
            } else {
                io.element_attributes.push(line.strtod());
            }
        }
    }
    Ok(())
}

/// file_readelementsarea (triangle_io.c:828-876): read element areas
/// from an .area file. The element count must agree with the .ele
/// read (857-861); a missing field stores -1.0 (865-867).
pub(crate) fn file_readelementsarea<R: Read>(
    file: &mut LineFile<R>,
    io: &mut MeshInput,
) -> Result<(), InternalStatus> {
    // Check for consistency with the .ele file (841-861).
    let Some(mut line) = readline(file) else {
        return Err(InternalStatus::FileRead);
    };
    // C reads io->numberoftriangles, which file_readelements set
    // from the .ele header; the owned list derives it (corners_per_element
    // is pinned to 3 there, 0 on a fresh input).
    let numelements = io
        .elements
        .len()
        .checked_div(io.corners_per_element.max(1))
        .unwrap_or(0);
    let areaelements = line.strtol() as i32;
    if areaelements != numelements as i32 {
        // TODO: error: area file disagrees on number of triangles
        // (858-861).
        return Err(InternalStatus::Failure);
    }

    // Read the triangles from the .area file (871-875).
    for _ in 0..numelements {
        let Some(mut line) = readline(file) else {
            return Err(InternalStatus::FileRead);
        };
        line.findfield();
        if line.at_end() {
            // No constraint on this triangle (866).
            io.triangle_areas.push(-1.0);
        } else {
            io.triangle_areas.push(line.strtod());
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::options::Options;
    use std::io::Cursor;

    /// An assembled Behavior from a closure over Options (the
    /// ioarrays.rs test precedent; Options::validate runs C's
    /// behavior_update tail exactly like the entry points).
    fn behavior_with(setup: impl FnOnce(&mut Options)) -> Behavior {
        let mut opts = Options::new();
        setup(&mut opts);
        opts.validate().expect("validate")
    }

    /// The two-triangle unit square of the ioarrays.rs fixture: t1
    /// (a, e, c) bonded to t2 (a, c, d) across t1's orient-2 edge;
    /// the vertices are the pool rows 1..=4, the triangles rows 2
    /// and 3.
    #[allow(clippy::type_complexity)]
    fn square_two_triangles(b: &Behavior, eextras: i32) -> (Mesh, u32, u32, [u32; 4]) {
        let mut m = Mesh::new(b, 8, 2, 0, eextras).expect("mesh init");
        let a = m.vx_alloc().expect("vx");
        m.vx_set_real(a, 0, 0.0);
        m.vx_set_real(a, 1, 0.0);
        let e = m.vx_alloc().expect("vx");
        m.vx_set_real(e, 0, 1.0);
        m.vx_set_real(e, 1, 0.0);
        let c = m.vx_alloc().expect("vx");
        m.vx_set_real(c, 0, 1.0);
        m.vx_set_real(c, 1, 1.0);
        let d = m.vx_alloc().expect("vx");
        m.vx_set_real(d, 0, 0.0);
        m.vx_set_real(d, 1, 1.0);
        let t1 = m.maketriangle(b.usesegments, b.vararea).expect("tri");
        m.setorg(t1, a);
        m.setdest(t1, e);
        m.setapex(t1, c);
        let t2 = m.maketriangle(b.usesegments, b.vararea).expect("tri");
        m.setorg(t2, a);
        m.setdest(t2, c);
        m.setapex(t2, d);
        m.bond(
            Otri {
                tri: t1.tri,
                orient: 2,
            },
            Otri {
                tri: t2.tri,
                orient: 0,
            },
        );
        (m, t1.tri, t2.tri, [a, e, c, d])
    }

    // -- the parse seam, pinned by construction from C's semantics --

    #[test]
    fn readline_skips_comments_and_finds_numbers() {
        let src = "# comment only\n  \nabc\n1 2 3\n  4.5\n";
        let mut f = LineFile::new(Cursor::new(src.as_bytes().to_vec()));
        let line = readline(&mut f).expect("line 1");
        assert_eq!(&line.buf[line.pos..], b"1 2 3\n");
        let line = readline(&mut f).expect("line 2");
        assert_eq!(&line.buf[line.pos..], b"4.5\n");
        assert!(readline(&mut f).is_none(), "EOF -> NULL");
    }

    #[test]
    fn findfield_walks_fields_and_truncates_comments() {
        let mut f = LineFile::new(Cursor::new(b"1 0.5\t0.25 7 # tail\n".to_vec()));
        let mut line = readline(&mut f).expect("line");
        assert_eq!(line.strtol(), 1);
        line.findfield();
        assert!(!line.at_end());
        assert_eq!(line.strtod(), 0.5);
        line.findfield();
        assert_eq!(line.strtod(), 0.25);
        line.findfield();
        assert_eq!(line.strtol(), 7);
        // The '#' truncated the line: the next field is '\0'.
        line.findfield();
        assert!(line.at_end());
    }

    #[test]
    fn findfield_skips_extra_separators_like_c() {
        // C's second loop does not stop at whitespace: a run of
        // separators is jumped entirely.
        let mut f = LineFile::new(Cursor::new(b"5   6\n".to_vec()));
        let mut line = readline(&mut f).expect("line");
        assert_eq!(line.strtol(), 5);
        line.findfield();
        assert_eq!(line.strtol(), 6);
    }

    #[test]
    fn strtol_base_zero_grammar() {
        let mk = |txt: &str| {
            let mut f = LineFile::new(Cursor::new(txt.as_bytes().to_vec()));
            readline(&mut f).expect("line")
        };
        let mut l = mk("42");
        assert_eq!(l.strtol(), 42);
        let mut l = mk("-17");
        assert_eq!(l.strtol(), -17);
        let mut l = mk("0x1f 5");
        assert_eq!(l.strtol(), 31);
        l.findfield();
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
            readline(&mut f).expect("line")
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
            readline(&mut f).expect("line")
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
        l.findfield();
        assert_eq!(l.strtol(), 5);
        let mut l = mk("0x.8p1");
        assert_eq!(l.strtod(), 1.0);
    }

    #[test]
    fn fgets_caps_at_inputlinesize_and_keeps_remainder() {
        // A line longer than INPUTLINESIZE - 1 continues on the next
        // fgets, exactly like C.
        let long = "1".repeat(INPUTLINESIZE + 20) + "\n2\n";
        let mut f = LineFile::new(Cursor::new(long.into_bytes()));
        let first = f.fgets().expect("first");
        assert_eq!(first.len(), INPUTLINESIZE - 1);
        let second = f.fgets().expect("second");
        assert_eq!(second.last(), Some(&b'\n'));
        // A final line without a newline still reads.
        let mut f = LineFile::new(Cursor::new(b"tail".to_vec()));
        assert_eq!(f.fgets().expect("tail"), b"tail".to_vec());
        assert!(f.fgets().is_none());
    }

    // -- the readers, pinned by construction from the file grammar --

    fn read_poly_string(src: &str) -> (MeshInput, i32) {
        let mut f = LineFile::new(Cursor::new(src.as_bytes().to_vec()));
        let mut io = MeshInput::default();
        let mut firstnode = 0;
        file_readpoly(&mut f, &mut io, &mut firstnode).expect("readpoly");
        (io, firstnode)
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
        let (io, firstnode) = read_poly_string(src);
        assert_eq!(firstnode, 5, "captured at i == 0");
        assert_eq!(io.attributes_per_point, 1);
        assert_eq!(
            io.points,
            vec![
                Point2 { x: 0.5, y: 0.25 },
                Point2 { x: 1.0, y: 0.0 },
                Point2 { x: 1.0, y: 1.0 },
                Point2 { x: 0.0, y: 1.0 },
            ]
        );
        // Missing attribute / marker fields fill defaults (0.0 / 0).
        assert_eq!(io.point_attributes, vec![7.0, 0.0, -2.5, 3.0]);
        assert_eq!(io.point_markers, vec![1, 0, 0, 2]);
        assert_eq!(io.segments, vec![[6, 5]]);
        assert_eq!(io.segment_markers, vec![9]);
        assert_eq!(io.holes, vec![Hole { x: 0.5, y: 0.5 }]);
        // The missing-area branch copies the attribute value.
        assert_eq!(
            io.regions,
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
        let mut f = LineFile::new(Cursor::new(src.as_bytes().to_vec()));
        let mut io = MeshInput::default();
        let mut firstnode = 0;
        file_readpoly(&mut f, &mut io, &mut firstnode).expect("readpoly");
        assert!(io.points.is_empty());
        assert!(io.segments.is_empty());
        // Then file_readnodes fills the same input.
        let src = "3 2 0 1\n9 0 0 1\n10 1 0 1\n11 0 1 1\n";
        let mut f = LineFile::new(Cursor::new(src.as_bytes().to_vec()));
        file_readnodes(&mut f, &mut io, &mut firstnode).expect("readnodes");
        assert_eq!(firstnode, 9);
        assert_eq!(io.points.len(), 3);
        assert_eq!(io.point_markers, vec![1, 1, 1]);
    }

    #[test]
    fn readnodes_rejects_short_and_nonplanar() {
        let mut f = LineFile::new(Cursor::new(b"2 2 0 0\n".to_vec()));
        let mut io = MeshInput::default();
        let mut firstnode = 0;
        assert_eq!(
            file_readnodes(&mut f, &mut io, &mut firstnode),
            Err(InternalStatus::Failure)
        );
        let mut f = LineFile::new(Cursor::new(b"3 3 0 0\n".to_vec()));
        assert_eq!(
            file_readnodes(&mut f, &mut io, &mut firstnode),
            Err(InternalStatus::Failure)
        );
        // A premature EOF is TRI_FILE_READ.
        let mut f = LineFile::new(Cursor::new(b"3 2 0 0\n1 0 0\n".to_vec()));
        assert_eq!(
            file_readnodes(&mut f, &mut io, &mut firstnode),
            Err(InternalStatus::FileRead)
        );
    }

    #[test]
    fn readelements_pins_three_corners() {
        // The .ele header says 6 corners; only 3 are stored and
        // read (numberofcorners pinned to 3).
        let src = "2 6 0\n1 2 3 4 5 6 7\n2 3 1 4 5 6 8\n";
        let mut f = LineFile::new(Cursor::new(src.as_bytes().to_vec()));
        let mut io = MeshInput::default();
        file_readelements(&mut f, &mut io).expect("readelements");
        assert_eq!(io.corners_per_element, 3);
        assert_eq!(io.elements, vec![2, 3, 4, 3, 1, 4]);
        assert!(io.element_attributes.is_empty());
    }

    #[test]
    fn readelements_fills_attributes_with_zero_default() {
        let src = "2 3 1\n1 2 3 4 9.5\n2 3 1 4\n";
        let mut f = LineFile::new(Cursor::new(src.as_bytes().to_vec()));
        let mut io = MeshInput::default();
        file_readelements(&mut f, &mut io).expect("readelements");
        assert_eq!(io.attributes_per_element, 1);
        assert_eq!(io.elements, vec![2, 3, 4, 3, 1, 4]);
        // A missing attribute field stores 0.0 (C stores the int 0).
        assert_eq!(io.element_attributes, vec![9.5, 0.0]);
    }

    #[test]
    fn readelements_rejects_two_corners() {
        let src = "1 2 0\n";
        let mut f = LineFile::new(Cursor::new(src.as_bytes().to_vec()));
        let mut io = MeshInput::default();
        assert_eq!(
            file_readelements(&mut f, &mut io),
            Err(InternalStatus::Failure)
        );
    }

    #[test]
    fn readelementsarea_checks_count_and_fills_minus_one() {
        let mut io = MeshInput {
            elements: vec![1, 2, 3, 4, 5, 6, 7, 8, 9],
            corners_per_element: 3,
            ..Default::default()
        };
        // The area count must agree with the element count (2 vs 3):
        // C's TODO failure.
        let src = "2 2\n1 0.25\n2\n";
        let mut f = LineFile::new(Cursor::new(src.as_bytes().to_vec()));
        assert_eq!(
            file_readelementsarea(&mut f, &mut io),
            Err(InternalStatus::Failure)
        );
        assert!(io.triangle_areas.is_empty());
        // Matching count: a missing field stores -1.0 (866).
        let src = "3 2\n1 0.25\n2\n3\n";
        let mut f = LineFile::new(Cursor::new(src.as_bytes().to_vec()));
        file_readelementsarea(&mut f, &mut io).expect("area");
        assert_eq!(io.triangle_areas, vec![0.25, -1.0, -1.0]);
    }

    // -- the writers, pinned by construction from the printf spec --

    #[test]
    fn file_writenodes_numbers_and_writes_bytes() {
        let b = behavior_with(|_| {});
        let (mut m, _t1, _t2, _v) = square_two_triangles(&b, 0);
        let mut out: Vec<u8> = Vec::new();
        file_writenodes(&mut m, &b, &mut out).expect("writenodes");
        assert_eq!(
            String::from_utf8(out).expect("ascii"),
            "4  2  0  1\n\
             \x20\x20\x201    0  0    0\n\
             \x20\x20\x202    1  0    0\n\
             \x20\x20\x203    1  1    0\n\
             \x20\x20\x204    0  1    0\n"
        );
        // The numbering sits over the marks (triangle_io.c:64-65).
        let mut t = m.vertices.traversal();
        let mut number = 1;
        while let Some(v) = m.vertextraverse(&mut t) {
            assert_eq!(m.vx_mark(v), number);
            number += 1;
        }
    }

    #[test]
    fn file_writenodes_zero_based_and_no_bounds() {
        let b = behavior_with(|o| {
            o.zero_based();
            o.no_bounds();
        });
        let (mut m, _t1, _t2, _v) = square_two_triangles(&b, 0);
        let mut out: Vec<u8> = Vec::new();
        file_writenodes(&mut m, &b, &mut out).expect("writenodes");
        assert_eq!(
            String::from_utf8(out).expect("ascii"),
            "4  2  0  0\n\
             \x20\x20\x200    0  0\n\
             \x20\x20\x201    1  0\n\
             \x20\x20\x202    1  1\n\
             \x20\x20\x203    0  1\n"
        );
    }

    #[test]
    fn file_writeelements_reads_the_writenodes_numbers() {
        let b = behavior_with(|_| {});
        let (mut m, _t1, _t2, _v) = square_two_triangles(&b, 0);
        let mut sink: Vec<u8> = Vec::new();
        file_writenodes(&mut m, &b, &mut sink).expect("writenodes");
        let mut out: Vec<u8> = Vec::new();
        file_writeelements(&mut m, &b, &mut out).expect("writeelements");
        assert_eq!(
            String::from_utf8(out).expect("ascii"),
            "2  3  0\n\
             \x20\x20\x201    \x20\x20 1  \x20\x20 2  \x20\x20 3\n\
             \x20\x20\x202    \x20\x20 1  \x20\x20 3  \x20\x20 4\n"
        );
    }

    #[test]
    fn file_writepoly_writes_segments_holes_and_regions() {
        let b = behavior_with(|o| {
            o.pslg();
        });
        let (mut m, t1, _t2, v) = square_two_triangles(&b, 0);
        // One subseg bonded to t1's orient-0 edge (a - e). The
        // subseg carries its own endpoint slots (sorg/sdest,
        // the ioarrays writer's fixture precedent); writenodes
        // has numbered the vertices 1-4 before the poly writer
        // reads their marks.
        let s = m.makesubseg().expect("subseg");
        m.setsorg(s, v[0]);
        m.setsdest(s, v[1]);
        m.tsbond(Otri { tri: t1, orient: 0 }, s);
        let mut sink: Vec<u8> = Vec::new();
        file_writenodes(&mut m, &b, &mut sink).expect("writenodes");
        let mut out: Vec<u8> = Vec::new();
        file_writepoly(
            &mut m,
            &b,
            &mut out,
            &[Hole { x: 0.5, y: 0.5 }],
            &[Region {
                x: 0.25,
                y: 0.75,
                attribute: 0.5,
                max_area: 0.001,
            }],
        )
        .expect("writepoly");
        assert_eq!(
            String::from_utf8(out).expect("ascii"),
            "0  2  0  1\n\
             1  1\n\
             \x20\x20\x201    \x20\x20 1  \x20\x20 2    \x20\x20 0\n\
             1\n\
             \x20\x20\x201   0.5  0.5\n\
             1\n\
             \x20\x20\x201   0.25  0.75  0.5  0.001\n"
        );
    }

    #[test]
    fn file_writeedges_walks_the_hull_and_interior_once() {
        let b = behavior_with(|_| {});
        let (mut m, _t1, _t2, _v) = square_two_triangles(&b, 0);
        m.edges = 5; // (3 * items + hullsize) / 2 for the square.
        let mut sink: Vec<u8> = Vec::new();
        file_writenodes(&mut m, &b, &mut sink).expect("writenodes");
        let mut out: Vec<u8> = Vec::new();
        file_writeedges(&mut m, &b, &mut out).expect("writeedges");
        assert_eq!(
            String::from_utf8(out).expect("ascii"),
            "5  1\n\
             \x20\x20\x201   1  2  1\n\
             \x20\x20\x202   2  3  1\n\
             \x20\x20\x203   3  1  0\n\
             \x20\x20\x204   3  4  1\n\
             \x20\x20\x205   4  1  1\n"
        );
    }

    #[test]
    fn file_writeneighbors_overlays_without_restore() {
        let b = behavior_with(|o| {
            o.neighbors();
        });
        let (mut m, t1, t2, _v) = square_two_triangles(&b, 0);
        let mut sink: Vec<u8> = Vec::new();
        file_writenodes(&mut m, &b, &mut sink).expect("writenodes");
        let mut out: Vec<u8> = Vec::new();
        file_writeneighbors(&mut m, &b, &mut out).expect("writeneighbors");
        assert_eq!(
            String::from_utf8(out).expect("ascii"),
            "2  3\n\
             \x20\x20\x201    -1  2  -1\n\
             \x20\x20\x202    -1  -1  1\n"
        );
        // No restore, unlike the array writer (triangle_io.c has no
        // subsegs scratch): the element numbers stay over the
        // overlay, and the dummy's -1 stays.
        let numberindex = m.tri_shape.numberindex.expect("overlay");
        assert_eq!(m.tri_int(t1, numberindex), 1);
        assert_eq!(m.tri_int(t2, numberindex), 2);
        assert_eq!(m.tri_int(DUMMY_TRI, numberindex), -1);
    }
}
