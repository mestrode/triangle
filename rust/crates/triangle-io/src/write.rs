//! The serialization half of the interchange formats (the write half
//! of triangle_io.c, extracted from the triangle crate's io/mod.rs by
//! the interchange-crate rung, issue #70 / ADR 0006): the header
//! emitters, the data-line emitters and the .poly trailers the
//! triangle crate's Mesh-bound writers call into, plus whole-file
//! writers over the neutral types for third-party use.
//!
//! Every emitter keeps C's printf format strings verbatim, trailing
//! '\n' included (triangle_io.c) - writeln! would move the newline out
//! of the C literal - and formats through the formatter seam's
//! g/d/ld forms, so the bytes are C's on every input. The .area writer
//! has no C counterpart (C only reads the format, triangle_io.c:
//! 828-876); it composes the reader's grammar - one header count and
//! one "%.17g" per element after the skipped row number - in the
//! hole-line shape of the .poly trailers (triangle_io.c:178-190).
//! Everything is pinned by construction in the tests below and by
//! round-trips with the parse module, never via the C oracle binary.

// Clippy: the header emitters keep C's fprintf format strings
// verbatim, trailing \n included (triangle_io.c) - writeln! would
// move the newline out of the C literal.
#![allow(clippy::write_with_newline)]

use std::io::Write;

use crate::formatter::{d, ld, write_d, write_d4, write_g, write_ld4};
use crate::types::{AreaFile, EdgeFile, EleFile, Hole, NeighFile, NodeFile, PolyFile, Region};

// ---------------------------------------------------------------------------
// Header emitters (triangle_io.c:42-43, 91-92, 148-151, 217, 276;
// the .area header composes the reader's grammar, 841-861).
// ---------------------------------------------------------------------------

/// The .node header (triangle_io.c:42-43: "%ld  %d  %d  %d\n"):
/// number of vertices, number of dimensions, number of vertex
/// attributes, and number of boundary markers (zero or one).
pub fn write_node_header(
    w: &mut dyn Write,
    point_count: i64,
    dim: i32,
    attributes: i32,
    markers: i32,
) -> std::io::Result<()> {
    write!(
        w,
        "{}  {}  {}  {}\n",
        ld(point_count),
        d(dim),
        d(attributes),
        d(markers)
    )
}

/// The .ele header (triangle_io.c:91-92: "%ld  %d  %d\n"): number of
/// triangles, vertices per triangle, attributes per triangle.
pub fn write_ele_header(
    w: &mut dyn Write,
    element_count: i64,
    corners: i32,
    attributes: i32,
) -> std::io::Result<()> {
    write!(
        w,
        "{}  {}  {}\n",
        ld(element_count),
        d(corners),
        d(attributes)
    )
}

/// The .poly vertex-section header line (triangle_io.c:148-149:
/// "%d  %d  %d  %d\n"): point count (the mesh writer passes 0 -
/// the vertices live in a separate .node file), number of
/// dimensions, number of vertex attributes, number of boundary
/// markers (zero or one).
pub fn write_poly_vertex_header(
    w: &mut dyn Write,
    point_count: i64,
    dim: i32,
    attributes: i32,
    markers: i32,
) -> std::io::Result<()> {
    write!(
        w,
        "{}  {}  {}  {}\n",
        d(point_count as i32),
        d(dim),
        d(attributes),
        d(markers)
    )
}

/// The .poly segment-count line (triangle_io.c:151: "%ld  %d\n"):
/// number of segments, number of segment boundary markers (zero or
/// one).
pub fn write_poly_segment_header(
    w: &mut dyn Write,
    segments: i64,
    segment_markers: i32,
) -> std::io::Result<()> {
    write!(w, "{}  {}\n", ld(segments), d(segment_markers))
}

/// The .poly header (triangle_io.c:148-151): the vertex-section
/// declaration line followed by the segment-count line - C's
/// file_writepoly shape, where the vertices live in a separate
/// .node file, so the two lines are adjacent.
pub fn write_poly_header(
    w: &mut dyn Write,
    point_count: i64,
    dim: i32,
    attributes: i32,
    markers: i32,
    segments: i64,
    segment_markers: i32,
) -> std::io::Result<()> {
    write_poly_vertex_header(w, point_count, dim, attributes, markers)?;
    write_poly_segment_header(w, segments, segment_markers)
}

/// The .edge header (triangle_io.c:217: "%ld  %d\n"): number of edges,
/// number of boundary markers.
pub fn write_edge_header(w: &mut dyn Write, edge_count: i64, markers: i32) -> std::io::Result<()> {
    write!(w, "{}  {}\n", ld(edge_count), d(markers))
}

/// The .neigh header (triangle_io.c:276: "%ld  %d\n"): number of
/// triangles, three neighbors per triangle.
pub fn write_neigh_header(w: &mut dyn Write, element_count: i64) -> std::io::Result<()> {
    write!(w, "{}  {}\n", ld(element_count), d(3))
}

/// The .area header: the element count line the reader checks against
/// the .ele read (triangle_io.c:841-861); the count comes from the
/// data list, like the other writers' counts.
pub fn write_area_header(w: &mut dyn Write, element_count: i64) -> std::io::Result<()> {
    write!(w, "{}\n", ld(element_count))
}

// ---------------------------------------------------------------------------
// Data-line emitters (triangle_io.c:51-61, 104-117, 161-167, 237-254,
// 303-304; the .area line composes the reader's grammar, 865-867).
// ---------------------------------------------------------------------------

/// One .node data line (triangle_io.c:51-61): vertex number, x and y
/// coordinates, the attributes, and the ORIGINAL boundary marker
/// when present ("%4d    %.17g  %.17g" + "  %.17g" per attribute +
/// "    %d" + newline).
pub fn write_node_line(
    out: &mut String,
    number: i32,
    x: f64,
    y: f64,
    attributes: impl Iterator<Item = f64>,
    marker: Option<i32>,
) {
    write_d4(out, number);
    out.push_str("    ");
    write_g(out, x, 17);
    out.push_str("  ");
    write_g(out, y, 17);
    for a in attributes {
        out.push_str("  ");
        write_g(out, a, 17);
    }
    if let Some(marker) = marker {
        out.push_str("    ");
        write_d(out, marker);
    }
    out.push('\n');
}

/// One .ele data line (triangle_io.c:104-117): element number, the
/// corner indices - three, or six when order 2 adds the mid-edge
/// nodes exactly where highorder stored them - then the attributes
/// ("%4ld    %4d  %4d  %4d" + "  %4d" per extra corner + "  %.17g"
/// per attribute + newline).
pub fn write_ele_line(
    out: &mut String,
    number: i64,
    corners: &[i32],
    attributes: impl Iterator<Item = f64>,
) {
    write_ld4(out, number);
    out.push_str("    ");
    for (i, &corner) in corners.iter().enumerate() {
        if i > 0 {
            out.push_str("  ");
        }
        write_d4(out, corner);
    }
    for a in attributes {
        out.push_str("  ");
        write_g(out, a, 17);
    }
    out.push('\n');
}

/// One .poly segment line (triangle_io.c:161-167): segment number,
/// indices of its two endpoints, and possibly a marker ("%4ld    %4d
/// %4d" + "    %4d" + newline).
pub fn write_segment_line(out: &mut String, number: i64, endpoints: [i32; 2], marker: Option<i32>) {
    write_ld4(out, number);
    out.push_str("    ");
    write_d4(out, endpoints[0]);
    out.push_str("  ");
    write_d4(out, endpoints[1]);
    if let Some(marker) = marker {
        out.push_str("    ");
        write_d4(out, marker);
    }
    out.push('\n');
}

/// One .edge data line (triangle_io.c:237-254): edge number, indices
/// of two endpoints, and possibly a boundary marker ("%4ld   %d  %d"
/// + "  %d" + newline).
pub fn write_edge_line(out: &mut String, number: i64, endpoints: [i32; 2], marker: Option<i32>) {
    write_ld4(out, number);
    out.push_str("   ");
    write_d(out, endpoints[0]);
    out.push_str("  ");
    write_d(out, endpoints[1]);
    if let Some(marker) = marker {
        out.push_str("  ");
        write_d(out, marker);
    }
    out.push('\n');
}

/// One .neigh data line (triangle_io.c:303-304): triangle number,
/// neighboring triangle numbers ("%4ld    %d  %d  %d" + newline).
pub fn write_neigh_line(out: &mut String, number: i64, neighbors: [i32; 3]) {
    write_ld4(out, number);
    out.push_str("    ");
    write_d(out, neighbors[0]);
    out.push_str("  ");
    write_d(out, neighbors[1]);
    out.push_str("  ");
    write_d(out, neighbors[2]);
    out.push('\n');
}

/// One .area data line: the element's row number and its area
/// constraint, in the .poly trailers' hole-line shape (triangle_io.c:
/// 178-190: "%4ld   %.17g" + newline) - the reader skips the number
/// and reads the area (triangle_io.c:865-867).
pub fn write_area_line(out: &mut String, number: i64, area: f64) {
    write_ld4(out, number);
    out.push_str("   ");
    write_g(out, area, 17);
    out.push('\n');
}

// ---------------------------------------------------------------------------
// The .poly trailers (triangle_io.c:173-193).
// ---------------------------------------------------------------------------

/// The .poly hole tail (triangle_io.c:173-181): the count line prints
/// even when there are no holes (unconditional in the measured C
/// build), then one line per hole ("%4ld   %.17g  %.17g\n",
/// 178-179).
pub fn write_poly_holes(
    w: &mut dyn Write,
    holes: &[Hole],
    first_number: i32,
) -> std::io::Result<()> {
    write!(w, "{}\n", d(holes.len() as i32))?;
    let mut line = String::new();
    for (holenumber, hole) in holes.iter().enumerate() {
        line.clear();
        write_ld4(&mut line, first_number as i64 + holenumber as i64);
        line.push_str("   ");
        write_g(&mut line, hole.x, 17);
        line.push_str("  ");
        write_g(&mut line, hole.y, 17);
        line.push('\n');
        w.write_all(line.as_bytes())?;
    }
    Ok(())
}

/// The .poly region tail (triangle_io.c:182-193): fully gated on a
/// non-empty list (the count line and the rows render only then),
/// one line per region ("%4ld   %.17g  %.17g  %.17g  %.17g\n",
/// 186-190).
pub fn write_poly_regions(
    w: &mut dyn Write,
    regions: &[Region],
    first_number: i32,
) -> std::io::Result<()> {
    if !regions.is_empty() {
        write!(w, "{}\n", d(regions.len() as i32))?;
        let mut line = String::new();
        for (regionnumber, region) in regions.iter().enumerate() {
            line.clear();
            write_ld4(&mut line, first_number as i64 + regionnumber as i64);
            line.push_str("   ");
            write_g(&mut line, region.x, 17);
            line.push_str("  ");
            write_g(&mut line, region.y, 17);
            line.push_str("  ");
            write_g(&mut line, region.attribute, 17);
            line.push_str("  ");
            write_g(&mut line, region.max_area, 17);
            line.push('\n');
            w.write_all(line.as_bytes())?;
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Whole-file writers over the neutral types (compositions for
// third-party use; round-trip with the parse module's readers).
// ---------------------------------------------------------------------------

/// Write a whole .node file from a NodeFile. The header count comes
/// from the data list; the marker column renders iff markers_present;
/// a missing attribute or marker list entry stores 0 / 0.0; numbering
/// runs from first_number.
pub fn write_node_file(w: &mut dyn Write, file: &NodeFile) -> std::io::Result<()> {
    write_node_header(
        w,
        file.points.len() as i64,
        file.dim,
        file.attributes_per_point,
        i32::from(file.markers_present),
    )?;
    write_node_section_lines(w, file)
}

/// The vertex lines of a node section: one line per point,
/// first-number numbering, the attribute and marker columns per
/// presence (the reader's vertex grammar, triangle_io.c:493-531,
/// rendered by write_node_line). Shared by the .node whole-file
/// writer and the embedded vertex section of the .poly whole-file
/// writer.
fn write_node_section_lines(w: &mut dyn Write, file: &NodeFile) -> std::io::Result<()> {
    let nattrs = file.attributes_per_point.max(0) as usize;
    let mut line = String::new();
    for (i, p) in file.points.iter().enumerate() {
        let attrs = file
            .point_attributes
            .get(i * nattrs..(i + 1) * nattrs)
            .unwrap_or(&[]);
        line.clear();
        write_node_line(
            &mut line,
            file.first_number + i as i32,
            p.x,
            p.y,
            attrs.iter().copied(),
            if file.markers_present {
                Some(file.point_markers.get(i).copied().unwrap_or(0))
            } else {
                None
            },
        );
        w.write_all(line.as_bytes())?;
    }
    Ok(())
}

/// Write a whole .poly file from a PolyFile, in the reader's
/// grammar: the vertex-section header line, the vertex lines (unless
/// the vertices live in a separate .node file, in which case the
/// header count is 0 and no vertex lines render), the segment-count
/// line, the segment lines, and the hole and region trailers. The
/// segment marker column renders iff segment_markers_present.
pub fn write_poly_file(w: &mut dyn Write, file: &PolyFile) -> std::io::Result<()> {
    let point_count = if file.nodes.separate_file {
        0
    } else {
        file.nodes.points.len() as i64
    };
    write_poly_vertex_header(
        w,
        point_count,
        file.nodes.dim,
        file.nodes.attributes_per_point,
        i32::from(file.nodes.markers_present),
    )?;
    if !file.nodes.separate_file {
        write_node_section_lines(w, &file.nodes)?;
    }
    write_poly_segment_header(
        w,
        file.segments.len() as i64,
        i32::from(file.segment_markers_present),
    )?;
    let mut line = String::new();
    for (i, seg) in file.segments.iter().enumerate() {
        line.clear();
        write_segment_line(
            &mut line,
            file.nodes.first_number as i64 + i as i64,
            *seg,
            if file.segment_markers_present {
                Some(file.segment_markers.get(i).copied().unwrap_or(0))
            } else {
                None
            },
        );
        w.write_all(line.as_bytes())?;
    }
    write_poly_holes(w, &file.holes, file.nodes.first_number)?;
    write_poly_regions(w, &file.regions, file.nodes.first_number)?;
    Ok(())
}

/// Write a whole .ele file from an EleFile. The element count comes
/// from the flat corner list (rows of corners_per_element); a missing
/// attribute list entry stores 0.0.
pub fn write_ele_file(w: &mut dyn Write, file: &EleFile) -> std::io::Result<()> {
    let corners = file.corners_per_element.max(1);
    let rows = file.elements.len() / corners;
    write_ele_header(w, rows as i64, corners as i32, file.attributes_per_element)?;
    let nattrs = file.attributes_per_element.max(0) as usize;
    let mut line = String::new();
    for (i, row) in file.elements.chunks(corners).take(rows).enumerate() {
        let attrs = file
            .element_attributes
            .get(i * nattrs..(i + 1) * nattrs)
            .unwrap_or(&[]);
        line.clear();
        write_ele_line(
            &mut line,
            file.first_number as i64 + i as i64,
            row,
            attrs.iter().copied(),
        );
        w.write_all(line.as_bytes())?;
    }
    Ok(())
}

/// Write a whole .edge file from an EdgeFile. The edge count comes
/// from the edge list; the marker column renders iff markers_present;
/// a missing marker list entry stores 0.
pub fn write_edge_file(w: &mut dyn Write, file: &EdgeFile) -> std::io::Result<()> {
    write_edge_header(w, file.edges.len() as i64, i32::from(file.markers_present))?;
    let mut line = String::new();
    for (i, edge) in file.edges.iter().enumerate() {
        line.clear();
        write_edge_line(
            &mut line,
            file.first_number as i64 + i as i64,
            *edge,
            if file.markers_present {
                Some(file.edge_markers.get(i).copied().unwrap_or(0))
            } else {
                None
            },
        );
        w.write_all(line.as_bytes())?;
    }
    Ok(())
}

/// Write a whole .neigh file from a NeighFile. The element count
/// comes from the flat neighbor list (rows of three).
pub fn write_neigh_file(w: &mut dyn Write, file: &NeighFile) -> std::io::Result<()> {
    let rows = file.neighbors.len() / 3;
    write_neigh_header(w, rows as i64)?;
    let mut line = String::new();
    for (i, row) in file.neighbors.chunks(3).take(rows).enumerate() {
        line.clear();
        write_neigh_line(
            &mut line,
            file.first_number as i64 + i as i64,
            [row[0], row[1], row[2]],
        );
        w.write_all(line.as_bytes())?;
    }
    Ok(())
}

/// Write a whole .area file from an AreaFile: the element count and
/// one "%.17g" per element after the skipped row number, the grammar
/// read_area_file parses (it takes the count as its argument).
pub fn write_area_file(w: &mut dyn Write, file: &AreaFile) -> std::io::Result<()> {
    write_area_header(w, file.areas.len() as i64)?;
    let mut line = String::new();
    for (i, &area) in file.areas.iter().enumerate() {
        line.clear();
        write_area_line(&mut line, file.first_number as i64 + i as i64, area);
        w.write_all(line.as_bytes())?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parse::{
        read_area_file, read_edge_file, read_ele_file, read_neigh_file, read_node_file,
        read_poly_file,
    };
    use crate::types::Point2;
    use std::io::Cursor;

    // The header emitters, byte-pinned from the C printf formats by
    // construction:
    #[test]
    fn headers_are_the_c_bytes() {
        let mut out = Vec::new();
        write_node_header(&mut out, 12, 2, 1, 1).expect("node header");
        assert_eq!(out, b"12  2  1  1\n");
        let mut out = Vec::new();
        write_ele_header(&mut out, 345, 3, 0).expect("ele header");
        assert_eq!(out, b"345  3  0\n");
        let mut out = Vec::new();
        write_edge_header(&mut out, 5, 1).expect("edge header");
        assert_eq!(out, b"5  1\n");
        let mut out = Vec::new();
        write_neigh_header(&mut out, 2).expect("neigh header");
        assert_eq!(out, b"2  3\n");
        let mut out = Vec::new();
        write_area_header(&mut out, 3).expect("area header");
        assert_eq!(out, b"3\n");
    }

    #[test]
    fn poly_headers_and_trailers_are_the_c_bytes() {
        // The mesh writer's shape: zero vertices (separate .node
        // file), then the segment counts.
        let mut out = Vec::new();
        write_poly_header(&mut out, 0, 2, 0, 1, 1, 1).expect("poly header");
        assert_eq!(out, b"0  2  0  1\n1  1\n");
        // The hole tail: the count line prints even when empty.
        let mut out = Vec::new();
        write_poly_holes(&mut out, &[], 1).expect("hole tail");
        assert_eq!(out, b"0\n");
        let mut out = Vec::new();
        write_poly_holes(&mut out, &[Hole { x: 0.5, y: 0.5 }], 1).expect("hole tail");
        assert_eq!(out, b"1\n   1   0.5  0.5\n");
        // The region tail: fully gated on the non-empty list.
        let mut out = Vec::new();
        write_poly_regions(&mut out, &[], 1).expect("region tail");
        assert!(out.is_empty());
        let mut out = Vec::new();
        write_poly_regions(
            &mut out,
            &[Region {
                x: 0.25,
                y: 0.75,
                attribute: 0.5,
                max_area: 0.001,
            }],
            1,
        )
        .expect("region tail");
        assert_eq!(out, b"1\n   1   0.25  0.75  0.5  0.001\n");
    }

    // The line emitters, byte-pinned from the C data-line formats:
    #[test]
    fn data_lines_are_the_c_bytes() {
        let mut line = String::new();
        write_node_line(&mut line, 1, 0.5, 0.25, [7.0, -2.5].into_iter(), Some(2));
        assert_eq!(line, "   1    0.5  0.25  7  -2.5    2\n");
        let mut line = String::new();
        write_node_line(&mut line, 10, 1.0, 0.0, std::iter::empty(), None);
        assert_eq!(line, "  10    1  0\n");
        let mut line = String::new();
        write_ele_line(&mut line, 2, &[1, 2, 3, 4, 5, 6], [9.5].into_iter());
        assert_eq!(line, "   2       1     2     3     4     5     6  9.5\n");
        let mut line = String::new();
        write_ele_line(&mut line, 1, &[1, 2, 3], std::iter::empty());
        assert_eq!(line, "   1       1     2     3\n");
        let mut line = String::new();
        write_segment_line(&mut line, 1, [1, 2], Some(0));
        assert_eq!(line, "   1       1     2       0\n");
        let mut line = String::new();
        write_edge_line(&mut line, 1, [1, 2], Some(1));
        assert_eq!(line, "   1   1  2  1\n");
        let mut line = String::new();
        write_neigh_line(&mut line, 1, [-1, 2, -1]);
        assert_eq!(line, "   1    -1  2  -1\n");
        let mut line = String::new();
        write_area_line(&mut line, 1, 0.25);
        assert_eq!(line, "   1   0.25\n");
    }

    // Whole-file round-trips with the parse module's readers, over
    // the firstnumber and marker matrices (by construction, never via
    // the oracle binary):
    #[test]
    fn node_file_roundtrips_with_firstnumber_and_marker_matrix() {
        for &(first, markers, nattrs) in &[
            (0i32, true, 0i32),
            (1, false, 1),
            (1, true, 2),
            (0, false, 0),
        ] {
            let file = NodeFile {
                point_count: 3,
                dim: 2,
                attributes_per_point: nattrs,
                markers_present: markers,
                first_number: first,
                separate_file: false,
                points: vec![
                    Point2 { x: 0.0, y: 0.0 },
                    Point2 { x: 0.1, y: 0.2 },
                    Point2 {
                        x: 1e-5,
                        y: 0.1 + 0.2,
                    },
                ],
                point_attributes: (0..3 * nattrs.max(0) as usize)
                    .map(|i| i as f64 / 7.0)
                    .collect(),
                point_markers: vec![4, 0, -2],
            };
            let mut out = Vec::new();
            write_node_file(&mut out, &file).expect("write node file");
            let back = read_node_file(Cursor::new(out)).expect("roundtrip");
            assert_eq!(back.point_count, 3);
            assert_eq!(back.dim, 2);
            assert_eq!(back.attributes_per_point, nattrs);
            assert_eq!(back.markers_present, markers);
            assert_eq!(back.first_number, first);
            assert!(!back.separate_file);
            assert_eq!(back.points, file.points);
            assert_eq!(back.point_attributes, file.point_attributes);
            assert_eq!(
                back.point_markers,
                if markers {
                    file.point_markers.clone()
                } else {
                    vec![]
                }
            );
        }
    }

    #[test]
    fn poly_file_roundtrips_embedded_and_separate() {
        let file = PolyFile {
            nodes: NodeFile {
                point_count: 3,
                dim: 2,
                attributes_per_point: 0,
                markers_present: true,
                first_number: 1,
                separate_file: false,
                points: vec![
                    Point2 { x: 0.0, y: 0.0 },
                    Point2 { x: 1.0, y: 0.0 },
                    Point2 { x: 0.0, y: 1.0 },
                ],
                point_attributes: vec![],
                point_markers: vec![1, 1, 1],
            },
            segments: vec![[2, 3], [3, 1]],
            segment_markers: vec![7, 9],
            segment_markers_present: true,
            holes: vec![Hole { x: 0.5, y: 0.5 }],
            regions: vec![Region {
                x: 0.25,
                y: 0.75,
                attribute: 0.5,
                max_area: 0.001,
            }],
        };
        let mut out = Vec::new();
        write_poly_file(&mut out, &file).expect("write poly file");
        let back = read_poly_file(Cursor::new(out)).expect("roundtrip");
        assert!(!back.nodes.separate_file);
        assert_eq!(back.nodes.points, file.nodes.points);
        assert_eq!(back.nodes.point_markers, vec![1, 1, 1]);
        assert_eq!(back.nodes.first_number, 1);
        assert_eq!(back.segments, file.segments);
        assert_eq!(back.segment_markers, file.segment_markers);
        assert!(back.segment_markers_present);
        assert_eq!(back.holes, file.holes);
        assert_eq!(back.regions, file.regions);

        // The separate-file variant: the vertex-section header line
        // carries 0, the segment marker column stays away when the
        // flag is off.
        let mut sep = file.clone();
        sep.nodes.separate_file = true;
        sep.segment_markers_present = false;
        sep.segment_markers.clear();
        let mut out = Vec::new();
        write_poly_file(&mut out, &sep).expect("write poly file");
        let back = read_poly_file(Cursor::new(out)).expect("roundtrip");
        assert!(back.nodes.separate_file);
        assert_eq!(back.segments, sep.segments);
        assert!(back.segment_markers.is_empty());
        assert!(!back.segment_markers_present);
        assert_eq!(back.holes, sep.holes);
        assert_eq!(back.regions, sep.regions);
    }

    #[test]
    fn ele_file_roundtrips_with_attributes() {
        let file = EleFile {
            element_count: 2,
            corners_per_element: 3,
            attributes_per_element: 1,
            first_number: 1,
            elements: vec![2, 3, 4, 3, 1, 4],
            element_attributes: vec![9.5, 0.0],
        };
        let mut out = Vec::new();
        write_ele_file(&mut out, &file).expect("write ele file");
        let back = read_ele_file(Cursor::new(out)).expect("roundtrip");
        assert_eq!(back.element_count, 2);
        assert_eq!(back.corners_per_element, 3);
        assert_eq!(back.attributes_per_element, 1);
        assert_eq!(back.elements, file.elements);
        assert_eq!(back.element_attributes, file.element_attributes);
    }

    #[test]
    fn edge_file_roundtrips_the_marker_matrix() {
        for &markers in &[true, false] {
            let file = EdgeFile {
                edge_count: 2,
                markers_present: markers,
                first_number: 1,
                edges: vec![[5, 6], [7, 8]],
                edge_markers: vec![0, 3],
            };
            let mut out = Vec::new();
            write_edge_file(&mut out, &file).expect("write edge file");
            let back = read_edge_file(Cursor::new(out)).expect("roundtrip");
            assert_eq!(back.edge_count, 2);
            assert_eq!(back.markers_present, markers);
            assert_eq!(back.edges, file.edges);
            assert_eq!(back.edge_markers, if markers { vec![0, 3] } else { vec![] });
        }
    }

    #[test]
    fn neigh_file_roundtrips() {
        let file = NeighFile {
            element_count: 2,
            first_number: 1,
            neighbors: vec![-1, 2, -1, -1, -1, 1],
        };
        let mut out = Vec::new();
        write_neigh_file(&mut out, &file).expect("write neigh file");
        let back = read_neigh_file(Cursor::new(out)).expect("roundtrip");
        assert_eq!(back.element_count, 2);
        assert_eq!(back.neighbors, file.neighbors);
    }

    #[test]
    fn area_file_roundtrips() {
        let file = AreaFile {
            first_number: 1,
            areas: vec![0.25, -1.0, 5.5e-7],
        };
        let mut out = Vec::new();
        write_area_file(&mut out, &file).expect("write area file");
        let back = read_area_file(Cursor::new(out), 3).expect("roundtrip");
        assert_eq!(back.areas, file.areas);
    }
}
