//! The neutral interchange types (ADR 0006, issue #70): owned structs
//! describing file contents only - points with coordinates, markers
//! and attributes, segments, holes, regions, elements, edges,
//! neighbors, areas, and the header fields (counts, dimensions,
//! attribute counts, marker flags, firstnumber). Nothing here
//! references the meshing core; the triangle crate converts at its
//! seam.
//!
//! The file structs mirror what the C readers fill (triangle_io.c):
//! counts as declared in the header, the data lists flat, missing
//! attribute fields as 0.0, missing marker fields as 0, and the
//! firstnumber captured where C captures it (the .node reader,
//! triangle_io.c:479-481; the other readers skip row numbers exactly
//! like C, so their first_number stays at the default and only
//! matters to the writers).

/// A point in the plane: the two REALs of a .node or .poly vertex
/// line.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}

/// A hole seed: the two REALs of a .poly hole line (triangle_io.c:
/// file_readholes_internal).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hole {
    pub x: f64,
    pub y: f64,
}

/// A regional attribute / area constraint entry: the four REALs of a
/// .poly region line (triangle_io.c:file_readholes_internal; a
/// missing area field copies the attribute value exactly like C,
/// triangle_io.c:694-695).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Region {
    pub x: f64,
    pub y: f64,
    pub attribute: f64,
    pub max_area: f64,
}

/// The contents of a .node file (or the vertex section of a .poly
/// file), as file_readnodes_internal fills it (triangle_io.c:432-546)
/// over neutral ground: the header fields as read, the firstnumber
/// captured from the first vertex's number field (triangle_io.c:
/// 479-481), and the data lists.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct NodeFile {
    /// The vertex count declared in the header (C invertices).
    pub point_count: i32,
    /// The dimension declared in the header (C mesh_dim; only 2 is
    /// accepted).
    pub dim: i32,
    /// The attribute count declared in the header (C nextras); the
    /// data list carries this many REALs per point.
    pub attributes_per_point: i32,
    /// Whether the header declared boundary markers (C nodemarkers
    /// != 0).
    pub markers_present: bool,
    /// The first vertex's number as read (C firstnode, captured at
    /// i == 0); the writers' numbering base.
    pub first_number: i32,
    /// The .poly zero-count arm (triangle_io.c:472-477): the vertices
    /// live in a separate .node file; the data lists stay empty and
    /// the header fields carry what the header said.
    pub separate_file: bool,
    /// The vertex coordinates, in file order.
    pub points: Vec<Point2>,
    /// The vertex attributes, flat: attributes_per_point REALs per
    /// point (a missing field stores 0.0).
    pub point_attributes: Vec<f64>,
    /// The vertex markers, in file order (empty when the header's
    /// marker count is zero; a missing field stores 0).
    pub point_markers: Vec<i32>,
}

/// The contents of a .poly file: the vertex section (a NodeFile), the
/// segment section (file_readsegments_internal, triangle_io.c:
/// 546-607) and the hole/region tails (file_readholes_internal,
/// triangle_io.c:616-710).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PolyFile {
    /// The vertex section: embedded vertices, or the zero-count arm
    /// with separate_file set.
    pub nodes: NodeFile,
    /// The segments, endpoint pairs in file order.
    pub segments: Vec<[i32; 2]>,
    /// The segment markers (empty when the header's marker count is
    /// zero; a missing field stores 0).
    pub segment_markers: Vec<i32>,
    /// Whether the segment header declared boundary markers (C
    /// segmentmarkers != 0).
    pub segment_markers_present: bool,
    /// The holes.
    pub holes: Vec<Hole>,
    /// The regional attribute / area constraint entries.
    pub regions: Vec<Region>,
}

/// The contents of an .ele file (file_readelements, triangle_io.c:
/// 744-803): the header fields as read, the corner count pinned to 3
/// (triangle_io.c:769 - higher-order files' extra corners are not
/// consumed, like C), the flat corner list and the flat attribute
/// list (a missing field stores 0.0).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct EleFile {
    /// The element count declared in the header (C inelements).
    pub element_count: i32,
    /// The corner count per row: the reader pins it to 3 exactly
    /// like C (triangle_io.c:769); a writer of order-2 elements sets
    /// 6.
    pub corners_per_element: usize,
    /// The attribute count declared in the header (C eextras).
    pub attributes_per_element: i32,
    /// The writers' numbering base (the reader skips row numbers
    /// like C; only the .node reader captures firstnode).
    pub first_number: i32,
    /// The element corners, flat: corners_per_element ints per
    /// element.
    pub elements: Vec<i32>,
    /// The element attributes, flat: attributes_per_element REALs
    /// per element.
    pub element_attributes: Vec<f64>,
}

/// The contents of an .edge file: the header fields as read (edge
/// count, marker flag) and the edge list. C has no .edge reader
/// (triangle_io.c only writes the format, 203-262); this parser reads
/// the writer's grammar under the readers' own conventions (a missing
/// marker field stores 0).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct EdgeFile {
    /// The edge count declared in the header.
    pub edge_count: i32,
    /// Whether the header declared boundary markers.
    pub markers_present: bool,
    /// The writers' numbering base (the reader skips row numbers).
    pub first_number: i32,
    /// The edges, endpoint pairs in file order.
    pub edges: Vec<[i32; 2]>,
    /// The edge markers (empty when the header's marker count is
    /// zero; a missing field stores 0).
    pub edge_markers: Vec<i32>,
}

/// The contents of a .neigh file: the header's element count and the
/// flat neighbor list, three ints per element (-1 for outer space). C
/// has no .neigh reader (triangle_io.c only writes the format,
/// 264-310); this parser reads the writer's grammar under the .ele
/// reader's conventions (neighbors per element pinned to 3, a
/// missing field is malformed input).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct NeighFile {
    /// The element count declared in the header.
    pub element_count: i32,
    /// The writers' numbering base (the reader skips row numbers).
    pub first_number: i32,
    /// The neighbors, flat: three ints per element, -1 facing outer
    /// space.
    pub neighbors: Vec<i32>,
}

/// The contents of an .area file (file_readelementsarea,
/// triangle_io.c:828-876): one REAL per element, a missing field
/// storing -1.0 (no constraint). The element count must agree with
/// the .ele read, so read_area_file takes it as an argument.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct AreaFile {
    /// The writers' numbering base (the reader skips row numbers).
    pub first_number: i32,
    /// The area constraints, one per element, in file order (-1.0 =
    /// no constraint).
    pub areas: Vec<f64>,
}
