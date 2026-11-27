//! The single private accessor chokepoint (ADR 0002, section 3): the ONLY
//! module that touches pool storage. Every storage-touching C handle macro
//! (triangle.h:703-1090: sym, bond, org/dest/apex, spivot, sbond,
//! tspivot/stpivot/tsbond, the dissolves, the infect family, elemattribute /
//! areabound / vertexmark / vertextype / vertex2tri, the killtri dead-d
//! pattern) becomes a method on Mesh here (D6); the pure ring rotations
//! stayed in mesh/handles.rs. The API takes and returns values only (u64 words,
//! u32 row ids, f64, i32, Copy handles): no reference into pool storage
//! survives a call, all conversions are bit-exact, all indexing is
//! bounds-checked, and the only intentional type punning is the C int
//! overlay itself (D2), done as half-word read-modify-write on whole words.
//! Assignment: the storage-touching rows of docs/function-mapping.md
//! (handle macro family -> mesh/slots.rs; org/dest/apex; vertexmark family;
//! struct mesh storage share).
//!
//! R1 accessor-invariant audit map (ADR 0002 section 3, debug builds
//! only; every enforcement rides the existing names and signatures,
//! which survive until rung 4):
//! (a) range - the orientation half is debug_asserted at every
//! encode/decode (mesh/handles.rs); the row-id half is debug_asserted at
//! every chokepoint decode site below (sym/symself, spivot/spivotself,
//! snext/snextself, stpivot, tspivot, splay_keyedge) via
//! WordPool::debug_assert_row_in_space, and on every raw word access
//! (mesh/pool.rs runs the materialized-frontier half). A decoded id is
//! inside the pool id space by construction; deadness stays a
//! data-model concern (C can leave neighbor slots pointing at dead
//! rows).
//! (b) null discipline - the all-zero word is never decoded
//! (mesh/handles.rs); no decoded handle ever names the null row (the
//! decode-site range checks below); alloc and traversal never surface
//! the null id (mesh/pool.rs); the reserved null row itself reads as
//! never-written zeros when a NULL vertex's coordinates are read
//! (rightofhyperbola on an outer-space handle - the port's defined
//! substitute for the NULL-vertex deref the legal C flow never
//! performs); org/dest/apex and the typed small-pool links return ids
//! as data, exactly like C, with no decode to guard.
//! (c) slot-role - slot < stride at every access below and in mesh/pool.rs;
//! the role tables are built only by the mesh/mod.rs init formulas, whose
//! computed indices are debug_asserted inside the rows they size.
//! (d) dead-stack discipline - slot 0 of a dead row is pool-private
//! between dealloc and realloc (killtri/killsubseg never write it);
//! the cfg-gated dead-chain integrity walk runs at every pool
//! push/pop/restart (mesh/pool.rs).
//! (e) exact-LIFO reuse - the pool's dead pop, pinned by the pool and
//! chokepoint tests; output-observable and untouched.
//!
//! R3 typed-seam note (ADR 0002 stage 1, storage typing): the raw word
//! accessors below are private to this module; consumers reach storage
//! through the typed row-group methods - the compile-time-fixed groups
//! of the C rows (triangle neighbors 0-2 / vertices 3-5 / subsegments
//! 6-8; the subseg row's groups; the vertex coordinate pair), plus the
//! shape-computed midpoint group. Each adds the group-range half of
//! the slot-role invariant (c) on top of the raw layer's checks; the
//! runtime-indexed payload (element attributes, the area bound, the
//! numbering int, vertex attributes) keeps its shape-computed word
//! methods. The row-size formulas in mesh/mod.rs stay the record of the
//! layout: packed word rows, no SoA (the ADR 0002 rematch evaluated
//! at R3, see docs/perf-r1-findings and the R3 PR).
//!
//! The Mesh struct itself lives in mesh/mod.rs (C: struct mesh, triangle.h:476;
//! lifecycle rows per the mapping); its pool fields are pub(crate) for the
//! constructors there, with the chokepoint contract - only this module calls
//! the pools' word/set_word/alloc/dealloc - enforced by review convention.
//!
//! Stage-1 note: consumers of these methods are the not-yet-transliterated
//! core modules per the mapping; the blanket below silences ahead-of-use
//! dead_code and is lifted as those modules land.
#![allow(dead_code)]

use crate::mesh::handles::{
    decode, encode, sdecode, sencode, Osub, Otri, DUMMY_SUB, DUMMY_TRI, MINUS1MOD3, PLUS1MOD3,
};
use crate::mesh::pool::Oom;
use crate::mesh::Mesh;

// ---------------------------------------------------------------------------
// Runtime row shapes (the transliterated init formulas, triangle.c:1098-1172,
// live in mesh/mod.rs; the shapes they compute are typed here at the chokepoint).
// ---------------------------------------------------------------------------

/// A 32-bit int field overlaid on one half of a stored word: the C int-cast
/// puns (vertexmark, vertextype, bnd_mark, the -n numbering) read and write
/// exactly 4 bytes of a pool word (D2). `high` addresses the upper half.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) struct HalfWord {
    /// Word (slot) index within the row.
    pub(crate) word: usize,
    /// The upper 32 bits when true, the lower when false.
    pub(crate) high: bool,
}

impl HalfWord {
    /// The int stored in `half` of `word`, leaving the other half intact.
    #[inline]
    fn overlay_i32(word: u64, half: HalfWord) -> i32 {
        if half.high {
            (word >> 32) as u32 as i32
        } else {
            word as u32 as i32
        }
    }

    /// Read-modify-write of `val` into `half` of `word`; the other half
    /// is preserved exactly like the C int cast on pool memory.
    #[inline]
    fn merge_i32(word: u64, half: HalfWord, val: i32) -> u64 {
        let val = val as u32 as u64;
        if half.high {
            // keep exactly the low 32 bits; the other half is preserved
            (word & 0xFFFF_FFFF) | (val << 32)
        } else {
            (word & 0xFFFF_FFFF_0000_0000) | val
        }
    }
}

/// Triangle row shape, computed by the transliterated
/// initializetrisubpools (triangle.c:1136-1185; formulas ADR 0002 section 2):
/// words 0-2 neighbor handles, 3-5 vertex ids, 6-8 subseg handles when the
/// mesh uses segments, then the runtime-indexed payload (attributes, area
/// bound, -n numbering int overlay).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) struct TriShape {
    /// Words per row (C trisize / 8; 48-byte rows without -p, order 1).
    pub(crate) stride: usize,
    /// Words 6..9 are live (-p); mirrors C b->usesegments.
    pub(crate) usesegments: bool,
    /// 6 + usesegments * 3 (C highorderindex).
    pub(crate) highorderindex: usize,
    /// First element-attribute word (C elemattribindex, REAL units = words).
    pub(crate) elemattribindex: usize,
    /// Area-bound word (C areaboundindex).
    pub(crate) areaboundindex: usize,
    /// The -n numbering overlay: C writeneighbors numbers triangles in
    /// *(int *)(tri + 6) (triangle.c:8090-8096, which even saves and
    /// restores the first-int of the subseg slot around itself), so the
    /// port models the numbering field as word 6, low half. None without
    /// -n. With -p the word is also a subseg (handle) slot exactly like C.
    pub(crate) numberindex: Option<HalfWord>,
}

/// Vertex row shape, computed by the transliterated initializevertexpool
/// (triangle.c:1100-1124): (mesh_dim + nextras) coordinate/attribute words,
/// then the boundary marker and vertex type as int overlays in the two
/// halves of one word, plus (with -p) the vertex2tri word.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) struct VertexShape {
    /// Words per row (C vertexsize / 8; 24 bytes at dim 2, 32 with -p).
    pub(crate) stride: usize,
    /// The boundary-marker int overlay (C vertexmarkindex, int units).
    pub(crate) mark: HalfWord,
    /// The vertex-type int overlay (C vertexmarkindex + 1).
    pub(crate) vtype: HalfWord,
    /// The vertex2tri word index (-p only; C vertex2triindex).
    pub(crate) v2tri: Option<usize>,
}

/// The subsegment boundary marker (C bnd_mark / setmark,
/// triangle.h:1001-1004): low half of word 8; the upper half stays spare,
/// exactly as in C (the subseg row is 9 words, 68 bytes).
pub(crate) const SUBSEG_MARK: HalfWord = HalfWord {
    word: 8,
    high: false,
};

// The vertex types (C triangle_config.h:133-141: INPUTVERTEX, SEGMENTVERTEX,
// FREEVERTEX, DEADVERTEX, UNDEADVERTEX; the C macro values are the contract).
pub(crate) const INPUTVERTEX: i32 = 0;
pub(crate) const SEGMENTVERTEX: i32 = 1;
pub(crate) const FREEVERTEX: i32 = 2;
pub(crate) const DEADVERTEX: i32 = -32768;
pub(crate) const UNDEADVERTEX: i32 = -32767;

// Deviation D12 (docs/deviation-register.md): C bounds the box with three
// trimalloc'd vertices outside the vertex pool (triangle.c:4634-4650) and
// trifrees them in removebox (triangle.c:4741-4743). The port reserves
// three ids at the top of the u32 row-id space instead: pool rows start
// low and grow upward, so these never collide with a real vertex row, and
// pool allocation and traversal never see them. Their two coordinates
// live in Mesh::infvertex_store; vx_real/vx_set_real are the only
// accessors.
pub(crate) const INFVERTEX1: u32 = 0xFFFF_FFFD;
pub(crate) const INFVERTEX2: u32 = 0xFFFF_FFFE;
pub(crate) const INFVERTEX3: u32 = 0xFFFF_FFFF;

/// Map a reserved out-of-pool INFVERTEX id to its Mesh::infvertex_store
/// slot (0, 1, 2); None for every real vertex row id.
#[inline]
fn infvertex_index(v: u32) -> Option<usize> {
    match v {
        INFVERTEX1 => Some(0),
        INFVERTEX2 => Some(1),
        INFVERTEX3 => Some(2),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Raw slot access (the chokepoint's innermost layer; private to this
// module since R3 - consumers go through the typed row groups below).
// Everything here debug asserts the slot-role invariant (ADR 0002
// section 3).
// ---------------------------------------------------------------------------

impl Mesh {
    /// A triangle row slot word (C: `tri[slot]`). The caller guarantees the
    /// row is real (live or sentinel); slot roles are fixed at init.
    ///
    /// Invariants (ADR 0002 section 3): slot-role (c) `slot < stride`
    /// debug_asserted here; range (a) on the row id is debug_asserted at
    /// the pool seam (WordPool::word), which every accessor of this
    /// module inherits. The null discipline (b) is decode-side: the
    /// reserved null row reads as its never-written zeros (see the
    /// module audit map).
    #[inline]
    fn tri_slot(&self, t: u32, slot: usize) -> u64 {
        debug_assert!(slot < self.tri_shape.stride, "tri slot {} out of row", slot);
        self.triangles.word(t, slot)
    }

    /// The write mirror of [`Mesh::tri_slot`]; the same invariants (ADR
    /// 0002 section 3) are debug_asserted - slot-role (c) here, range (a)
    /// at the pool seam (the null discipline (b) is decode-side).
    #[inline]
    fn set_tri_slot(&mut self, t: u32, slot: usize, w: u64) {
        debug_assert!(slot < self.tri_shape.stride, "tri slot {} out of row", slot);
        self.triangles.set_word(t, slot, w);
    }

    /// A subsegment row slot word (C: `ss[slot]`). Invariants (ADR 0002
    /// section 3): slot-role (c) here (C rows are 9 words), range (a) on
    /// the row id at the pool seam (the null discipline (b) is
    /// decode-side).
    #[inline]
    fn sub_slot(&self, s: u32, slot: usize) -> u64 {
        debug_assert!(slot < 9, "subseg slot out of row (C rows are 9 words)");
        self.subsegs.word(s, slot)
    }

    /// The write mirror of [`Mesh::sub_slot`]; the same invariants (ADR
    /// 0002 section 3).
    #[inline]
    fn set_sub_slot(&mut self, s: u32, slot: usize, w: u64) {
        debug_assert!(slot < 9, "subseg slot out of row (C rows are 9 words)");
        self.subsegs.set_word(s, slot, w);
    }

    // -----------------------------------------------------------------------
    // Typed row groups (R3, ADR 0002 stage 1): the compile-time-fixed slot
    // groups of the C rows (triangle.h:225-232, 246-248), named at the
    // seam. Each maps a group index to its fixed slot and rides the raw
    // layer above, inheriting its invariants; the group-range half of the
    // slot-role invariant (c) is debug_asserted here. The runtime-indexed
    // payload keeps the shape-computed word methods (tri_attribute,
    // tri_areabound, tri_int, vx_real for comp >= 2); the midpoint group
    // sits on a shape-computed base but is fixed in size.
    // -----------------------------------------------------------------------

    /// The packed neighbor handle word of a triangle row's neighbor
    /// group (C `tri[0..2]`, the three adjoining triangles;
    /// triangle.h:225-232; sym decodes this word at the oriented edge,
    /// triangle.h:743). `edge` in 0..3 names the edge exactly like the
    /// orientation of a handle does.
    ///
    /// Invariants (ADR 0002 section 3): the group range `edge < 3` (c)
    /// debug_asserted here; slot-role, range (a) and the null
    /// discipline (b) ride tri_slot.
    #[inline]
    pub(crate) fn tri_neighbor_word(&self, t: u32, edge: usize) -> u64 {
        debug_assert!(edge < 3, "triangle neighbor edge {} out of group", edge);
        self.tri_slot(t, edge)
    }

    /// The write mirror of [`Mesh::tri_neighbor_word`]; the group range
    /// (c) is debug_asserted here, the rest ride set_tri_slot.
    #[inline]
    pub(crate) fn set_tri_neighbor_word(&mut self, t: u32, edge: usize, w: u64) {
        debug_assert!(edge < 3, "triangle neighbor edge {} out of group", edge);
        self.set_tri_slot(t, edge, w)
    }

    /// The raw vertex id word of a triangle row's vertex group (C
    /// `tri[3..5]`, the three vertices; triangle.h:225-232; org/dest/apex
    /// read these slots at orientation-computed corners,
    /// triangle.h:850-861). `corner` in 0..3 maps to slot 3 + corner.
    ///
    /// Invariants (ADR 0002 section 3): the group range `corner < 3` (c)
    /// debug_asserted here; slot-role, range (a) and the null discipline
    /// (b) ride tri_slot (the NULL vertex slots are data, exactly like
    /// C).
    #[inline]
    pub(crate) fn tri_vertex_word(&self, t: u32, corner: usize) -> u64 {
        debug_assert!(corner < 3, "triangle vertex corner {} out of group", corner);
        self.tri_slot(t, 3 + corner)
    }

    /// The write mirror of [`Mesh::tri_vertex_word`]; the group range
    /// (c) is debug_asserted here, the rest ride set_tri_slot.
    #[inline]
    pub(crate) fn set_tri_vertex_word(&mut self, t: u32, corner: usize, w: u64) {
        debug_assert!(corner < 3, "triangle vertex corner {} out of group", corner);
        self.set_tri_slot(t, 3 + corner, w)
    }

    /// The raw word of a triangle row's subsegment group (C `tri[6..8]`,
    /// live only under -p; triangle.h:225-232; tspivot reads the slot
    /// at the oriented edge, triangle.h:1043). `edge` in 0..3 maps to
    /// slot 6 + edge. C reuses these three fixed slots before
    /// subsegments exist, as the vertex-to-triangle adjacency stack of
    /// transfernodes (the link-the-current-triangle machinery,
    /// triangle.c:5612), so the payload is a subseg handle or that
    /// stack word - both are the same raw word in C.
    ///
    /// Invariants (ADR 0002 section 3): the group range `edge < 3` (c)
    /// debug_asserted here; the slot-role half (slot < stride, so words
    /// 6..9 exist only under -p), range (a) and the null discipline
    /// (b) ride tri_slot.
    #[inline]
    pub(crate) fn tri_subseg_word(&self, t: u32, edge: usize) -> u64 {
        debug_assert!(edge < 3, "triangle subseg edge {} out of group", edge);
        self.tri_slot(t, 6 + edge)
    }

    /// The write mirror of [`Mesh::tri_subseg_word`]; the group range
    /// (c) is debug_asserted here, the rest ride set_tri_slot.
    #[inline]
    pub(crate) fn set_tri_subseg_word(&mut self, t: u32, edge: usize, w: u64) {
        debug_assert!(edge < 3, "triangle subseg edge {} out of group", edge);
        self.set_tri_slot(t, 6 + edge, w)
    }

    /// A high-order mid-edge node id of a triangle row (C: the -o2
    /// midpoint words highorder records at highorderindex + k;
    /// triangle.c:7675 writes them, triangle.c:7906-7915 reads them
    /// back). `k` in 0..3 maps to slot highorderindex + k; the base is
    /// runtime (6 + usesegments * 3, the shape-computed formula in
    /// mesh/mod.rs), which is why this payload stays word-indexed behind a
    /// typed method.
    ///
    /// Invariants (ADR 0002 section 3): the group range `k < 3` (c)
    /// debug_asserted here; slot-role (the stride covers the node words
    /// only when the row was sized for them), range (a) and the null
    /// discipline (b) ride tri_slot.
    #[inline]
    pub(crate) fn tri_midnode(&self, t: u32, k: usize) -> u64 {
        debug_assert!(k < 3, "midpoint {} out of the order-2 node group", k);
        self.tri_slot(t, self.tri_shape.highorderindex + k)
    }

    /// The write mirror of [`Mesh::tri_midnode`]; the group range (c)
    /// is debug_asserted here, the rest ride set_tri_slot.
    #[inline]
    pub(crate) fn set_tri_midnode(&mut self, t: u32, k: usize, w: u64) {
        debug_assert!(k < 3, "midpoint {} out of the order-2 node group", k);
        self.set_tri_slot(t, self.tri_shape.highorderindex + k, w)
    }

    /// The packed subsegment handle word of a subseg row's neighbor
    /// group (C `ss[0..1]`, the two adjoining subsegments;
    /// triangle.h:246-248; spivot/snext read them, triangle.h:947/958).
    /// `side` in 0..2 maps to slot side.
    ///
    /// Invariants (ADR 0002 section 3): the group range `side < 2` (c)
    /// debug_asserted here; slot-role, range (a) and the null
    /// discipline (b) ride sub_slot.
    #[inline]
    pub(crate) fn sub_neighbor_word(&self, s: u32, side: usize) -> u64 {
        debug_assert!(side < 2, "subseg neighbor side {} out of group", side);
        self.sub_slot(s, side)
    }

    /// The write mirror of [`Mesh::sub_neighbor_word`]; the group range
    /// (c) is debug_asserted here, the rest ride set_sub_slot.
    #[inline]
    pub(crate) fn set_sub_neighbor_word(&mut self, s: u32, side: usize, w: u64) {
        debug_assert!(side < 2, "subseg neighbor side {} out of group", side);
        self.set_sub_slot(s, side, w)
    }

    /// The raw vertex id word of a subseg row's vertex group (C
    /// `ss[2..5]`, the four vertex slots; triangle.h:246-248; sorg/sdest
    /// read the endpoint slots, triangle.h:969-978). `i` in 0..4 maps
    /// to slot 2 + i.
    ///
    /// Invariants (ADR 0002 section 3): the group range `i < 4` (c)
    /// debug_asserted here; slot-role, range (a) and the null
    /// discipline (b) ride sub_slot.
    #[inline]
    pub(crate) fn sub_vertex_word(&self, s: u32, i: usize) -> u64 {
        debug_assert!(i < 4, "subseg vertex index {} out of group", i);
        self.sub_slot(s, 2 + i)
    }

    /// The write mirror of [`Mesh::sub_vertex_word`]; the group range
    /// (c) is debug_asserted here, the rest ride set_sub_slot.
    #[inline]
    pub(crate) fn set_sub_vertex_word(&mut self, s: u32, i: usize, w: u64) {
        debug_assert!(i < 4, "subseg vertex index {} out of group", i);
        self.set_sub_slot(s, 2 + i, w)
    }

    /// The packed triangle handle word of a subseg row's triangle group
    /// (C `ss[6..7]`, the two adjoining triangles; triangle.h:246-248;
    /// stpivot reads them, triangle.h:1050). `side` in 0..2 maps to
    /// slot 6 + side.
    ///
    /// Invariants (ADR 0002 section 3): the group range `side < 2` (c)
    /// debug_asserted here; slot-role, range (a) and the null
    /// discipline (b) ride sub_slot.
    #[inline]
    pub(crate) fn sub_tri_word(&self, s: u32, side: usize) -> u64 {
        debug_assert!(side < 2, "subseg triangle side {} out of group", side);
        self.sub_slot(s, 6 + side)
    }

    /// The write mirror of [`Mesh::sub_tri_word`]; the group range (c)
    /// is debug_asserted here, the rest ride set_sub_slot.
    #[inline]
    pub(crate) fn set_sub_tri_word(&mut self, s: u32, side: usize, w: u64) {
        debug_assert!(side < 2, "subseg triangle side {} out of group", side);
        self.set_sub_slot(s, 6 + side, w)
    }

    /// A whole-word f64 view of a triangle row slot (attributes and area
    /// bounds; C elemattribute/areabound access REALs). Bit-exact via
    /// from_bits/to_bits; no arithmetic evaluates an overlaid word.
    ///
    /// Invariants (ADR 0002 section 3 c): the slot stays below the stride
    /// through tri_slot; the payload roles (attribute and area words)
    /// come from the init formulas only.
    #[inline]
    pub(crate) fn tri_real(&self, t: u32, slot: usize) -> f64 {
        f64::from_bits(self.tri_slot(t, slot))
    }

    /// The write mirror of [`Mesh::tri_real`]; the same invariants (ADR
    /// 0002 section 3 c) ride tri_slot/set_tri_slot.
    #[inline]
    pub(crate) fn set_tri_real(&mut self, t: u32, slot: usize, x: f64) {
        self.set_tri_slot(t, slot, x.to_bits());
    }

    /// An int overlay on a triangle row (C: the -n numbering int in
    /// *(int *)(tri + 6); the port addresses the half via the shape).
    ///
    /// Invariants (ADR 0002 section 3): the overlay half is inside the
    /// row by the init formulas (c); range (a) at the pool seam (the
    /// null discipline (b) is decode-side).
    #[inline]
    pub(crate) fn tri_int(&self, t: u32, half: HalfWord) -> i32 {
        HalfWord::overlay_i32(self.tri_slot(t, half.word), half)
    }

    /// The write mirror of [`Mesh::tri_int`] (read-modify-write on the
    /// containing word, D2); the same invariants (ADR 0002 section 3).
    #[inline]
    pub(crate) fn set_tri_int(&mut self, t: u32, half: HalfWord, val: i32) {
        let w = HalfWord::merge_i32(self.tri_slot(t, half.word), half, val);
        self.set_tri_slot(t, half.word, w);
    }

    /// An element attribute (C elemattribute(otri, attnum),
    /// triangle.h:890-894: the REAL at elemattribindex + attnum).
    /// Invariants (ADR 0002 section 3 c): attnum < eextras debug_asserted
    /// here, and the attribute words sit inside the row by the init
    /// formulas.
    pub(crate) fn tri_attribute(&self, t: u32, attnum: usize) -> f64 {
        debug_assert!(
            attnum < self.eextras as usize,
            "attribute {} past eextras",
            attnum
        );
        self.tri_real(t, self.tri_shape.elemattribindex + attnum)
    }

    /// The write mirror of [`Mesh::tri_attribute`]; the same invariant
    /// (ADR 0002 section 3 c) is debug_asserted.
    pub(crate) fn set_tri_attribute(&mut self, t: u32, attnum: usize, x: f64) {
        debug_assert!(
            attnum < self.eextras as usize,
            "attribute {} past eextras",
            attnum
        );
        self.set_tri_real(t, self.tri_shape.elemattribindex + attnum, x);
    }

    /// The maximum-area bound (C areabound / setareabound, triangle.h:898-901).
    /// Invariant (ADR 0002 section 3 c): the area-bound word is inside
    /// the row (the init formulas raise the stride under vararea; pinned
    /// by the mesh/mod.rs init debug_asserts).
    #[inline]
    pub(crate) fn tri_areabound(&self, t: u32) -> f64 {
        self.tri_real(t, self.tri_shape.areaboundindex)
    }

    /// The write mirror of [`Mesh::tri_areabound`]; the same invariant
    /// (ADR 0002 section 3 c).
    #[inline]
    pub(crate) fn set_tri_areabound(&mut self, t: u32, x: f64) {
        self.set_tri_real(t, self.tri_shape.areaboundindex, x)
    }

    /// The subsegment boundary marker (C bnd_mark / setmark,
    /// triangle.h:1001-1004: the int at ss + 8).
    /// Invariant (ADR 0002 section 3 c): the marker overlay is word 8,
    /// low half, inside C's 9-word row by the fixed init formula.
    #[inline]
    pub(crate) fn sub_mark(&self, s: u32) -> i32 {
        HalfWord::overlay_i32(self.sub_slot(s, SUBSEG_MARK.word), SUBSEG_MARK)
    }

    /// The write mirror of [`Mesh::sub_mark`] (D2 read-modify-write);
    /// the same invariant (ADR 0002 section 3 c).
    #[inline]
    pub(crate) fn set_sub_mark(&mut self, s: u32, m: i32) {
        let w = HalfWord::merge_i32(self.sub_slot(s, SUBSEG_MARK.word), SUBSEG_MARK, m);
        self.set_sub_slot(s, SUBSEG_MARK.word, w);
    }

    /// A vertex coordinate / attribute word (C: `vx[comp]`, a REAL).
    /// Since R3 the two coordinate words go through the typed pair
    /// (vx_x / vx_y below); this method stays for the runtime-indexed
    /// payload (attributes, comp >= 2) and as the typed pair's store.
    ///
    /// Invariants (ADR 0002 section 3): comp < stride (c) debug_asserted
    /// here (the D12 infvertex store carries two coordinates); range (a)
    /// on the row id at the pool seam (the INFVERTEX ids never reach it;
    /// they live in the owned store). The NULL vertex id is data: its
    /// read answers the null row's never-written zeros - the port's
    /// defined substitute for the NULL-vertex deref (rightofhyperbola
    /// on an outer-space handle; C's dummytri carries NULL vertices,
    /// triangle.c:1049, and the legal C flow never performs the read).
    #[inline]
    pub(crate) fn vx_real(&self, v: u32, comp: usize) -> f64 {
        // D12 out-of-pool infvertices carry their two coordinates in the
        // owned store, never in vertex pool rows.
        if let Some(i) = infvertex_index(v) {
            debug_assert!(comp < 2, "the infinite vertices carry two coordinates");
            return f64::from_bits(self.infvertex_store[i][comp]);
        }
        debug_assert!(
            comp < self.vertex_shape.stride,
            "vertex word {} out of row",
            comp
        );
        f64::from_bits(self.vertices.word(v, comp))
    }

    #[inline]
    pub(crate) fn vx_set_real(&mut self, v: u32, comp: usize, x: f64) {
        // D12 out-of-pool infvertices carry their two coordinates in the
        // owned store, never in vertex pool rows;
        // removebox zeroes it like C's trifree.
        // Invariants (ADR 0002 section 3): as vx_real (c here, a at the
        // pool seam; b decode-side).
        if let Some(i) = infvertex_index(v) {
            debug_assert!(comp < 2, "the infinite vertices carry two coordinates");
            self.infvertex_store[i][comp] = x.to_bits();
            return;
        }
        debug_assert!(
            comp < self.vertex_shape.stride,
            "vertex word {} out of row",
            comp
        );
        self.vertices.set_word(v, comp, x.to_bits());
    }

    /// The x coordinate of a vertex (C `vx[0]`, the first of the row's
    /// two fixed coordinate words; the row formula: triangle.c:1100-1124).
    /// The typed pair names the words fixed at every shape; the
    /// runtime-indexed payload (comp >= 2: attributes) stays with
    /// vx_real, as does the D12 out-of-pool store.
    ///
    /// Invariants (ADR 0002 section 3): as vx_real (c here, a at the
    /// pool seam; b decode-side).
    #[inline]
    pub(crate) fn vx_x(&self, v: u32) -> f64 {
        self.vx_real(v, 0)
    }

    /// The write mirror of [`Mesh::vx_x`]; the same invariants.
    #[inline]
    pub(crate) fn set_vx_x(&mut self, v: u32, x: f64) {
        self.vx_set_real(v, 0, x)
    }

    /// The y coordinate of a vertex (C `vx[1]`); as vx_x.
    #[inline]
    pub(crate) fn vx_y(&self, v: u32) -> f64 {
        self.vx_real(v, 1)
    }

    /// The write mirror of [`Mesh::vx_y`]; the same invariants.
    #[inline]
    pub(crate) fn set_vx_y(&mut self, v: u32, y: f64) {
        self.vx_set_real(v, 1, y)
    }

    /// The vertex boundary marker (C vertexmark, triangle.h:1074-1077).
    /// Invariant (ADR 0002 section 3 c): the overlay half comes from
    /// the init formulas (VertexShape); range (a) at the pool seam (the
    /// null discipline (b) is decode-side).
    #[inline]
    pub(crate) fn vx_mark(&self, v: u32) -> i32 {
        let half = self.vertex_shape.mark;
        HalfWord::overlay_i32(self.vertices.word(v, half.word), half)
    }

    /// The write mirror of [`Mesh::vx_mark`] (D2); the same invariants
    /// (ADR 0002 section 3).
    #[inline]
    pub(crate) fn vx_set_mark(&mut self, v: u32, m: i32) {
        let half = self.vertex_shape.mark;
        let w = HalfWord::merge_i32(self.vertices.word(v, half.word), half, m);
        self.vertices.set_word(v, half.word, w);
    }

    /// The vertex type (C vertextype, triangle.h:1079-1082: the int right
    /// after the boundary marker).
    /// Invariant (ADR 0002 section 3 c): the overlay half comes from
    /// the init formulas; range (a) at the pool seam (the null
    /// discipline (b) is decode-side). The DEADVERTEX value this overlay
    /// carries is the dead-row detection of the vertex pool (the
    /// data-model side of invariant d).
    #[inline]
    pub(crate) fn vx_type(&self, v: u32) -> i32 {
        let half = self.vertex_shape.vtype;
        HalfWord::overlay_i32(self.vertices.word(v, half.word), half)
    }

    /// The write mirror of [`Mesh::vx_type`]; the same invariants (ADR
    /// 0002 section 3).
    #[inline]
    pub(crate) fn vx_set_type(&mut self, v: u32, ty: i32) {
        let half = self.vertex_shape.vtype;
        let w = HalfWord::merge_i32(self.vertices.word(v, half.word), half, ty);
        self.vertices.set_word(v, half.word, w);
    }

    /// The vertex2tri word (C vertex2tri, triangle.h:1084-1087): the raw
    /// encode()d triangle handle word; decode happens at the caller through
    /// mesh/handles.rs, never behind the chokepoint.
    ///
    /// Invariants (ADR 0002 section 3): the word exists only with -p (the
    /// expect mirrors C's undefined read); decode-side null (b) and
    /// orientation (a) ride mesh/handles.rs decode; the row id is checked at
    /// the caller's next storage access.
    #[inline]
    pub(crate) fn vx_2tri(&self, v: u32) -> u64 {
        let slot = self
            .vertex_shape
            .v2tri
            .expect("vertex2tri slot exists only with -p (C: b->poly)");
        self.vertices.word(v, slot)
    }

    /// The write mirror of [`Mesh::vx_2tri`]; the same invariants (ADR
    /// 0002 section 3).
    #[inline]
    pub(crate) fn vx_set_2tri(&mut self, v: u32, w: u64) {
        let slot = self
            .vertex_shape
            .v2tri
            .expect("vertex2tri slot exists only with -p (C: b->poly)");
        self.vertices.set_word(v, slot, w);
    }

    // -----------------------------------------------------------------------
    // Triangle primitives (C triangle.h:743-923; every storage-touching macro
    // of the handle family lands here, the pure rotations in mesh/handles.rs).
    // -----------------------------------------------------------------------

    /// C sym (triangle.h:743): decode the neighbor handle at the oriented
    /// edge slot; a NULL slot is never decoded, exactly as in C.
    /// Invariants (ADR 0002 section 3): null discipline (b) at the
    /// decode; range (a) - the decoded row id must be live-or-sentinel
    /// inside the triangle pool's id space - debug_asserted here at the
    /// chokepoint.
    #[inline]
    pub(crate) fn sym(&self, o: Otri) -> Otri {
        let n = decode(self.tri_slot(o.tri, o.orient as usize));
        self.triangles.debug_assert_row_in_space(n.tri);
        n
    }

    /// The in-place mirror of sym; the same invariants (ADR 0002 section
    /// 3 a/b) fire here on the decoded handle.
    #[inline]
    pub(crate) fn symself(&self, o: &mut Otri) {
        *o = decode(self.tri_slot(o.tri, o.orient as usize));
        self.triangles.debug_assert_row_in_space(o.tri);
    }

    /// C onext (triangle.h:792): lprev then symself.
    /// Invariants (ADR 0002 section 3 a/b): ride the symself decode.
    pub(crate) fn onext(&self, o: Otri) -> Otri {
        let mut t = o.lprev();
        self.symself(&mut t);
        t
    }

    /// The in-place mirror of onext; the decode invariants (ADR 0002
    /// section 3 a/b) ride symself.
    pub(crate) fn onextself(&self, o: &mut Otri) {
        o.lprevself();
        self.symself(o);
    }

    /// C oprev (triangle.h:800): sym then lnextself.
    /// Invariants (ADR 0002 section 3 a/b): ride the sym decode.
    pub(crate) fn oprev(&self, o: Otri) -> Otri {
        let mut t = self.sym(o);
        t.lnextself();
        t
    }

    /// The in-place mirror of oprev; the decode invariants (ADR 0002
    /// section 3 a/b) ride symself.
    pub(crate) fn oprevself(&self, o: &mut Otri) {
        self.symself(o);
        o.lnextself();
    }

    /// C dnext (triangle.h:808): sym then lprevself.
    /// Invariants (ADR 0002 section 3 a/b): ride the sym decode.
    pub(crate) fn dnext(&self, o: Otri) -> Otri {
        let mut t = self.sym(o);
        t.lprevself();
        t
    }

    /// The in-place mirror of dnext; the decode invariants (ADR 0002
    /// section 3 a/b) ride symself.
    pub(crate) fn dnextself(&self, o: &mut Otri) {
        self.symself(o);
        o.lprevself();
    }

    /// C dprev (triangle.h:816): lnext then symself.
    /// Invariants (ADR 0002 section 3 a/b): ride the symself decode.
    pub(crate) fn dprev(&self, o: Otri) -> Otri {
        let mut t = o.lnext();
        self.symself(&mut t);
        t
    }

    /// The in-place mirror of dprev; the decode invariants (ADR 0002
    /// section 3 a/b) ride symself.
    pub(crate) fn dprevself(&self, o: &mut Otri) {
        o.lnextself();
        self.symself(o);
    }

    /// C rnext (triangle.h:824): sym, lnextself, symself.
    /// Invariants (ADR 0002 section 3 a/b): ride the sym decodes.
    pub(crate) fn rnext(&self, o: Otri) -> Otri {
        let mut t = self.sym(o);
        t.lnextself();
        self.symself(&mut t);
        t
    }

    /// The in-place mirror of rnext; the decode invariants (ADR 0002
    /// section 3 a/b) ride symself.
    pub(crate) fn rnextself(&self, o: &mut Otri) {
        self.symself(o);
        o.lnextself();
        self.symself(o);
    }

    /// C rprev (triangle.h:832): sym, lprevself, symself.
    /// Invariants (ADR 0002 section 3 a/b): ride the sym decodes.
    pub(crate) fn rprev(&self, o: Otri) -> Otri {
        let mut t = self.sym(o);
        t.lprevself();
        self.symself(&mut t);
        t
    }

    /// The in-place mirror of rprev; the decode invariants (ADR 0002
    /// section 3 a/b) ride symself.
    pub(crate) fn rprevself(&self, o: &mut Otri) {
        self.symself(o);
        o.lprevself();
        self.symself(o);
    }

    /// The three vertex ids of a triangle row (C org/dest/apex,
    /// triangle.h:850-861): slot 3 + plus1mod3[orient], 3 +
    /// minus1mod3[orient], and 3 + orient respectively.
    ///
    /// Invariants (ADR 0002 section 3): the vertex-role slots 3..5 are
    /// fixed by the row shape (c) and debug_asserted below stride; the
    /// source row is range/null checked (a/b) at the pool seam. The
    /// returned id is data (the NULL vertex of fresh rows and the D12
    /// sweepline event pun both ride this slot full-width, exactly like
    /// C), so no debug check runs on the returned value.
    #[inline]
    pub(crate) fn org(&self, o: Otri) -> u32 {
        self.tri_slot(o.tri, PLUS1MOD3[o.orient as usize] as usize + 3) as u32
    }

    #[inline]
    pub(crate) fn dest(&self, o: Otri) -> u32 {
        self.tri_slot(o.tri, MINUS1MOD3[o.orient as usize] as usize + 3) as u32
    }

    #[inline]
    pub(crate) fn apex(&self, o: Otri) -> u32 {
        self.tri_slot(o.tri, o.orient as usize + 3) as u32
    }

    /// C setorg: the write mirror of the org slot arithmetic of the
    /// org/dest/apex family above; invariants (ADR 0002 section 3 b/c)
    /// ride set_tri_slot (the stored id is data).
    #[inline]
    pub(crate) fn setorg(&mut self, o: Otri, v: u32) {
        self.set_tri_slot(o.tri, PLUS1MOD3[o.orient as usize] as usize + 3, v as u64);
    }

    /// C setdest: the write mirror of the dest slot arithmetic;
    /// invariants (ADR 0002 section 3 b/c) ride set_tri_slot.
    #[inline]
    pub(crate) fn setdest(&mut self, o: Otri, v: u32) {
        self.set_tri_slot(o.tri, MINUS1MOD3[o.orient as usize] as usize + 3, v as u64);
    }

    /// C setapex: the write mirror of the apex slot arithmetic;
    /// invariants (ADR 0002 section 3 b/c) ride set_tri_slot.
    #[inline]
    pub(crate) fn setapex(&mut self, o: Otri, v: u32) {
        self.set_tri_slot(o.tri, o.orient as usize + 3, v as u64);
    }

    /// The full-word org-slot read of the sweepline event pun (algorithm/delaunay.rs
    /// part 2): C stores a `struct event *` in the org slot of front
    /// handles (setorg, triangle.c:5388/5403) and reads it back with the
    /// plain org macro, reinterpreted as an event pointer, in exactly one
    /// place: check4deadevent (triangle.c:5008). D1 replaces the pointer
    /// with a tagged full-width word (bit 32 above the 32-bit id space);
    /// this reader returns the slot raw, the single sanctioned pun site.
    /// The pun is data, never decoded: the null discipline (ADR 0002
    /// section 3 b) does not apply to the raw word, only to decode.
    #[inline]
    pub(crate) fn orgword(&self, o: Otri) -> u64 {
        self.tri_slot(o.tri, PLUS1MOD3[o.orient as usize] as usize + 3)
    }

    /// C: setorg(checktri, (VOID *) newevent) of the event pun, and the
    /// NULL clear of check4deadevent (triangle.c:5388, 5403, 5016): the
    /// full-width store through the same slot as org/setorg.
    /// Invariants (ADR 0002 section 3 b/c) ride set_tri_slot; the stored
    /// word is raw data like orgword's.
    #[inline]
    pub(crate) fn setorgword(&mut self, o: Otri, w: u64) {
        self.set_tri_slot(o.tri, PLUS1MOD3[o.orient as usize] as usize + 3, w);
    }

    /// C bond (triangle.h:871): bond two triangles together at the
    /// respective oriented handles.
    ///
    /// Invariants (ADR 0002 section 3): the orientation range (a) is
    /// debug_asserted at both encodes; slot-role (c) and row range (a/b)
    /// ride set_tri_slot.
    pub(crate) fn bond(&mut self, a: Otri, b: Otri) {
        let (ab, ba) = (encode(b), encode(a));
        self.set_tri_slot(a.tri, a.orient as usize, ab);
        self.set_tri_slot(b.tri, b.orient as usize, ba);
    }

    /// C dissolve (triangle.h:880): point the oriented edge at outer space
    /// (the stored dummy handle; encode of the sentinel row, orientation 0,
    /// is exactly the C raw m->dummytri slot value).
    /// The stored word keeps the D3 sentinel row id legal on every
    /// later decode (ADR 0002 section 3 a).
    pub(crate) fn dissolve(&mut self, o: Otri) {
        let dummy = encode(Otri {
            tri: DUMMY_TRI,
            orient: 0,
        });
        self.set_tri_slot(o.tri, o.orient as usize, dummy);
    }

    /// C infect (triangle.h:875): set the viral-infection bit (bit 1) of
    /// word 6 of the triangle row. With -p the word holds a stored subseg
    /// handle; under the amended packing (ADR 0002 amendment, issue #22)
    /// bit 1 is spare in every stored handle exactly like C's pointer-
    /// alignment bit, so this is the literal C mirror on -p rows and
    /// plain rows alike.
    ///
    /// Invariant (ADR 0002 section 3 a): bit 1 never reaches a decoded
    /// row id or orientation (the sdecode mask drops it).
    pub(crate) fn infect(&mut self, o: Otri) {
        let w = self.tri_slot(o.tri, 6);
        self.set_tri_slot(o.tri, 6, w | 2);
    }

    /// C uninfect (triangle.h:879). Invariant (ADR 0002 section 3 a):
    /// bit 1 stays out of every decoded handle (see infect).
    pub(crate) fn uninfect(&mut self, o: Otri) {
        let w = self.tri_slot(o.tri, 6);
        self.set_tri_slot(o.tri, 6, w & !2);
    }

    /// C infected (triangle.h:885). The bit-1 test is the literal C
    /// mirror; it never interprets the word as a handle (ADR 0002
    /// section 3 a).
    #[inline]
    pub(crate) fn infected(&self, o: Otri) -> bool {
        self.tri_slot(o.tri, 6) & 2 != 0
    }

    /// C deadtri (triangle.h:908): a dead triangle's slot 1 is NULL.
    ///
    /// Invariant (ADR 0002 section 3 d): this is the dead-row detection;
    /// a dead row's slot 0 stays pool-private (killtri never touches it).
    #[inline]
    pub(crate) fn deadtri(&self, t: u32) -> bool {
        self.tri_slot(t, 1) == 0
    }

    /// C killtri (triangle.h:910): slot 1 to NULL (dead marker; slot 0 stays
    /// the pool-private dead-stack link, never touched here) and slot 3 to
    /// NULL in case a badtriang row points at the triangle.
    ///
    /// Invariants (ADR 0002 section 3): dead-stack discipline (d) - slot
    /// 0 is never written here, so the pool's dead-chain link survives;
    /// the slots written stay below the stride (c).
    pub(crate) fn killtri(&mut self, t: u32) {
        self.set_tri_slot(t, 1, 0);
        self.set_tri_slot(t, 3, 0);
    }

    // -----------------------------------------------------------------------
    // Subsegment primitives (C triangle.h:935-1034).
    // -----------------------------------------------------------------------

    /// C spivot (triangle.h:947): the adjoining subsegment at the oriented
    /// end. Invariants (ADR 0002 section 3): null (b) at the sdecode; the
    /// decoded row id inside the subseg pool id space (a), DUMMY_SUB
    /// included - debug_asserted here at the chokepoint.
    #[inline]
    pub(crate) fn spivot(&self, o: Osub) -> Osub {
        let n = sdecode(self.sub_slot(o.ss, o.ssorient as usize));
        self.subsegs.debug_assert_row_in_space(n.ss);
        n
    }

    /// The in-place mirror of spivot; the same invariants (ADR 0002
    /// section 3 a/b) fire here on the decoded handle.
    #[inline]
    pub(crate) fn spivotself(&self, o: &mut Osub) {
        *o = sdecode(self.sub_slot(o.ss, o.ssorient as usize));
        self.subsegs.debug_assert_row_in_space(o.ss);
    }

    /// C snext (triangle.h:958): the next subsegment in sequence.
    /// Invariants (ADR 0002 section 3): null (b) at the sdecode; the
    /// decoded row id inside the subseg pool id space (a) - debug_asserted
    /// here at the chokepoint.
    #[inline]
    pub(crate) fn snext(&self, o: Osub) -> Osub {
        let n = sdecode(self.sub_slot(o.ss, 1 - o.ssorient as usize));
        self.subsegs.debug_assert_row_in_space(n.ss);
        n
    }

    /// The in-place mirror of snext; the same invariants (ADR 0002
    /// section 3 a/b) fire here on the decoded handle.
    #[inline]
    pub(crate) fn snextself(&self, o: &mut Osub) {
        *o = sdecode(self.sub_slot(o.ss, 1 - o.ssorient as usize));
        self.subsegs.debug_assert_row_in_space(o.ss);
    }

    /// The subsegment's own endpoints (C sorg/sdest, triangle.h:969-978).
    ///
    /// Invariants (ADR 0002 section 3 c): the vertex-role slots 2..3 are
    /// fixed by the row shape; range (a) and null (b) on the source row
    /// at the pool seam. The returned id is data (no check); the NULL id
    /// marks the badsubsegdealloc dead-row idiom.
    #[inline]
    pub(crate) fn sorg(&self, o: Osub) -> u32 {
        self.sub_slot(o.ss, 2 + o.ssorient as usize) as u32
    }

    #[inline]
    pub(crate) fn sdest(&self, o: Osub) -> u32 {
        self.sub_slot(o.ss, 3 - o.ssorient as usize) as u32
    }

    /// C setsorg: the write mirror of the sorg slot arithmetic; the
    /// slot-role and row invariants (ADR 0002 section 3 b/c) ride
    /// set_sub_slot.
    #[inline]
    pub(crate) fn setsorg(&mut self, o: Osub, v: u32) {
        self.set_sub_slot(o.ss, 2 + o.ssorient as usize, v as u64);
    }

    /// C setsdest: the write mirror of the sdest slot arithmetic; the
    /// invariants (ADR 0002 section 3 b/c) ride set_sub_slot.
    #[inline]
    pub(crate) fn setsdest(&mut self, o: Osub, v: u32) {
        self.set_sub_slot(o.ss, 3 - o.ssorient as usize, v as u64);
    }

    /// The enclosing segment's endpoints (C segorg/segdest,
    /// triangle.h:981-990).
    ///
    /// Invariants (ADR 0002 section 3 c): the segment-role slots 4..5 are
    /// fixed by the row shape; range (a) and null (b) at the pool seam.
    #[inline]
    pub(crate) fn segorg(&self, o: Osub) -> u32 {
        self.sub_slot(o.ss, 4 + o.ssorient as usize) as u32
    }

    #[inline]
    pub(crate) fn segdest(&self, o: Osub) -> u32 {
        self.sub_slot(o.ss, 5 - o.ssorient as usize) as u32
    }

    /// C setsegorg: the write mirror of the segorg slot arithmetic; the
    /// invariants (ADR 0002 section 3 b/c) ride set_sub_slot.
    #[inline]
    pub(crate) fn setsegorg(&mut self, o: Osub, v: u32) {
        self.set_sub_slot(o.ss, 4 + o.ssorient as usize, v as u64);
    }

    /// C setsegdest: the write mirror of the segdest slot arithmetic;
    /// the invariants (ADR 0002 section 3 b/c) ride set_sub_slot.
    #[inline]
    pub(crate) fn setsegdest(&mut self, o: Osub, v: u32) {
        self.set_sub_slot(o.ss, 5 - o.ssorient as usize, v as u64);
    }

    /// C sbond (triangle.h:1004): bond two subsegments at their origins.
    ///
    /// Invariants (ADR 0002 section 3): ssorient in range (a) at both
    /// sencodes; slot-role (c) and row range (a/b) ride set_sub_slot.
    pub(crate) fn sbond(&mut self, a: Osub, b: Osub) {
        let (ab, ba) = (sencode(b), sencode(a));
        self.set_sub_slot(a.ss, a.ssorient as usize, ab);
        self.set_sub_slot(b.ss, b.ssorient as usize, ba);
    }

    /// C sdissolve (triangle.h:1011): point at the omnipresent subsegment
    /// (sencode of the sentinel row is exactly the C raw m->dummysub value).
    /// The stored word keeps the D3 sentinel row id legal on every
    /// later decode (ADR 0002 section 3 a).
    pub(crate) fn sdissolve(&mut self, o: Osub) {
        let dummy = sencode(Osub {
            ss: DUMMY_SUB,
            ssorient: 0,
        });
        self.set_sub_slot(o.ss, o.ssorient as usize, dummy);
    }

    /// C deadsubseg (triangle.h:1031): a dead subsegment's slot 1 is NULL.
    ///
    /// Invariant (ADR 0002 section 3 d): the dead-row detection; a dead
    /// row's slot 0 stays pool-private (killsubseg never touches it).
    #[inline]
    pub(crate) fn deadsubseg(&self, s: u32) -> bool {
        self.sub_slot(s, 1) == 0
    }

    /// C killsubseg (triangle.h:1033): slots 1 and 2 to NULL.
    ///
    /// Invariants (ADR 0002 section 3): dead-stack discipline (d) - slot
    /// 0 is never written here; the slots written stay below the stride
    /// (c).
    pub(crate) fn killsubseg(&mut self, s: u32) {
        self.set_sub_slot(s, 1, 0);
        self.set_sub_slot(s, 2, 0);
    }

    // -----------------------------------------------------------------------
    // Triangle/subsegment interaction primitives (C triangle.h:1041-1069).
    // -----------------------------------------------------------------------

    /// C tspivot (triangle.h:1043): the subsegment abutting the oriented
    /// edge. Reading a NULL (unused) subseg slot is a port-side
    /// debug_assert, mirroring that the C flow never pivots an unbonded
    /// side without checking.
    /// Invariants (ADR 0002 section 3): null discipline (b) at sdecode;
    /// the decoded row's id inside the subseg pool id space (a), the
    /// DUMMY_SUB sentinel included - debug_asserted here.
    #[inline]
    pub(crate) fn tspivot(&self, o: Otri) -> Osub {
        let n = sdecode(self.tri_slot(o.tri, o.orient as usize + 6));
        self.subsegs.debug_assert_row_in_space(n.ss);
        n
    }

    /// C stpivot (triangle.h:1050): the triangle abutting the subsegment's
    /// side. Invariants (ADR 0002 section 3): null discipline (b) at the
    /// decode; the decoded row's id inside the triangle pool id space
    /// (a), the dummy row included - debug_asserted here.
    #[inline]
    pub(crate) fn stpivot(&self, o: Osub) -> Otri {
        let n = decode(self.sub_slot(o.ss, o.ssorient as usize + 6));
        self.triangles.debug_assert_row_in_space(n.tri);
        n
    }

    /// C tsbond (triangle.h:1056): bond a triangle edge to a subsegment side.
    ///
    /// Invariants (ADR 0002 section 3): orientation ranges (a) at the
    /// encodes; slot-role (c) and row range (a/b) ride both set slots.
    pub(crate) fn tsbond(&mut self, o: Otri, s: Osub) {
        self.set_tri_slot(o.tri, o.orient as usize + 6, sencode(s));
        self.set_sub_slot(s.ss, s.ssorient as usize + 6, encode(o));
    }

    /// C tsdissolve (triangle.h:1062): dissolve the bond from the triangle
    /// side (tri slot back to the omnipresent subsegment).
    /// The stored word keeps the D3 sentinel row id legal on every
    /// later decode (ADR 0002 section 3 a).
    pub(crate) fn tsdissolve(&mut self, o: Otri) {
        let dummy = sencode(Osub {
            ss: DUMMY_SUB,
            ssorient: 0,
        });
        self.set_tri_slot(o.tri, o.orient as usize + 6, dummy);
    }

    /// C stdissolve (triangle.h:1067): dissolve the bond from the subsegment
    /// side (ss slot back to outer space).
    /// The stored word keeps the D3 sentinel row id legal on every
    /// later decode (ADR 0002 section 3 a).
    pub(crate) fn stdissolve(&mut self, o: Osub) {
        let dummy = encode(Otri {
            tri: DUMMY_TRI,
            orient: 0,
        });
        self.set_sub_slot(o.ss, o.ssorient as usize + 6, dummy);
    }

    // -----------------------------------------------------------------------
    // Dead-stack mirrors on Mesh (ADR 0002 section 4; the C alloc/dealloc
    // wrapper functions triangledealloc/subsegdealloc/vertexdealloc land as
    // mesh/mod.rs rows in a later PR and resolve to these primitives).
    // -----------------------------------------------------------------------

    /// Allocate a triangle row: C poolalloc(&m->triangles) at every triangle
    /// allocation site; the exact-LIFO pop and the fresh frontier live in
    /// the pool (ADR 0003 section 3). Only this chokepoint calls it.
    ///
    /// Invariants (ADR 0002 section 3): the null and sentinel ids are
    /// never handed out (b, D3; debug-pinned in the pool); the dead pop
    /// keeps reuse exact-LIFO (e).
    pub(crate) fn tri_alloc(&mut self) -> Result<u32, Oom> {
        self.triangles.alloc()
    }

    /// Deallocate a triangle row: the C triangledealloc body
    /// (triangle.c:1196-1205) - killtri, then push.
    ///
    /// Invariants (ADR 0002 section 3): dead-stack discipline (d) - the
    /// push makes slot 0 pool-private; the reuse order stays exact-LIFO
    /// (e); the integrity walk runs at the push (mesh/pool.rs, debug only).
    pub(crate) fn tri_dealloc(&mut self, t: u32) {
        self.killtri(t);
        self.triangles.dealloc(t);
    }

    /// Allocate a subsegment row: C poolalloc(&m->subsegs) at every makesubseg
    /// / subseg allocation site.
    ///
    /// Invariants (ADR 0002 section 3): as tri_alloc (b, D3, e).
    pub(crate) fn sub_alloc(&mut self) -> Result<u32, Oom> {
        self.subsegs.alloc()
    }

    /// Deallocate a subsegment row: the C subsegdealloc body
    /// (triangle.c:1229-1241) - killsubseg, then push.
    ///
    /// Invariants (ADR 0002 section 3): as tri_dealloc (d, e).
    pub(crate) fn sub_dealloc(&mut self, s: u32) {
        self.killsubseg(s);
        self.subsegs.dealloc(s);
    }

    /// Allocate a vertex row: C poolalloc(&m->vertices) at every vertex
    /// allocation site.
    ///
    /// Invariants (ADR 0002 section 3): the null id is never handed out
    /// (b); the vertex pool has no sentinel row (D3) and the dead pop
    /// keeps reuse exact-LIFO (e).
    pub(crate) fn vx_alloc(&mut self) -> Result<u32, Oom> {
        self.vertices.alloc()
    }

    /// Deallocate a vertex row: the C vertexdealloc body
    /// (triangle.c:1262-1271) - the type overlay is set to DEADVERTEX first
    /// (dead detection), then the push.
    ///
    /// Invariants (ADR 0002 section 3): the DEADVERTEX overlay marks the
    /// row for vertextraverse's skip (the data-model side of d); slot 0
    /// becomes pool-private at the push; reuse stays exact-LIFO (e).
    pub(crate) fn vx_dealloc(&mut self, v: u32) {
        self.vx_set_type(v, DEADVERTEX);
        self.vertices.dealloc(v);
    }

    // -----------------------------------------------------------------------
    // Flip-stack rows (C struct flipstacker, triangle.h:291-294:
    // { triangle flippedtri; struct flipstacker *prevflip; }, 2 words).
    // Written by insertvertex/undovertex (algorithm/transform.rs) under -q; the
    // C NULL/`&insertvertex` pointers of prevflip mirror as row ids
    // (D4), 0 for NULL, algorithm/transform.rs::INSERTVERTEX_MARKER for the
    // &(struct flipstacker) &insertvertex end-of-list sentinel.
    // -----------------------------------------------------------------------

    /// C: poolrestart(&m->flipstackers): the insertvertex bookkeeping
    /// restarts (triangle.c:3130, 3239) and enforcequality's checkquality
    /// stage init (triangle.c:7565-7566). (This doc carried a stale 3160
    /// anchor from before the c-anchor audit; re-measured for algorithm/refine.rs.)
    ///
    /// Invariants (ADR 0002 section 3 d): the dead chain is dropped; the
    /// debug integrity walk runs on the emptied chain.
    pub(crate) fn flip_restart(&mut self) {
        self.flipstackers.restart();
    }

    /// C: m->lastflip = (struct flipstacker *) poolalloc(&m->flipstackers):
    /// the fresh flipstacker row id (INTERIOR hole: D4).
    ///
    /// Invariants (ADR 0002 section 3): the null id is never handed out
    /// (b); the dead pop keeps reuse exact-LIFO (e); the integrity walk
    /// runs at the pop (d, debug only).
    pub(crate) fn flip_alloc(&mut self) -> Result<u32, Oom> {
        self.flipstackers.alloc()
    }

    /// C: m->lastflip->flippedtri: the stored encode()d triangle handle
    /// (word 0); the caller decodes through mesh/handles.rs.
    ///
    /// Invariants (ADR 0002 section 3): slot-role (c) at the pool seam
    /// (2-word row); the caller's decode carries the null (b) and
    /// orientation (a) checks; the row id is checked at its next storage
    /// access.
    pub(crate) fn flip_flippedtri(&self, id: u32) -> u64 {
        self.flipstackers.word(id, 0)
    }

    /// C: m->lastflip->flippedtri = encode(horiz) (triangle.c:3162).
    /// Invariants (ADR 0002 section 3 b/c) ride set_word; orientation (a)
    /// at the encode.
    pub(crate) fn set_flip_flippedtri(&mut self, id: u32, w: u64) {
        self.flipstackers.set_word(id, 0, w);
    }

    /// C: m->lastflip->prevflip: the previous flipstacker (word 1); a
    /// row id under D4 - the null id is the C NULL pointer, and the
    /// port mirror of C's `(struct flipstacker *) &insertvertex` marker
    /// (triangle.c:3163/3770) is the row id that can never be handed out
    /// (algorithm/transform.rs::INSERTVERTEX_MARKER).
    ///
    /// Invariant (ADR 0002 section 3 b): the marker id is data on this
    /// link, never a row; the null id 0 is the C NULL pointer.
    pub(crate) fn flip_prevflip(&self, id: u32) -> u32 {
        self.flipstackers.word(id, 1) as u32
    }

    /// C: newflip->prevflip = m->lastflip (triangle.c:3392).
    /// Invariants (ADR 0002 section 3 b/c) ride set_word.
    pub(crate) fn set_flip_prevflip(&mut self, id: u32, prev: u32) {
        self.flipstackers.set_word(id, 1, prev as u64);
    }

    // -----------------------------------------------------------------------
    // Encroached-subsegment rows (C struct badsubseg, triangle.h:271-274:
    // { subseg encsubseg; vertex subsegorg, subsegdest; }, 3 words).
    // Enqueued by insertvertex (algorithm/transform.rs) and checkseg4encroach
    // (algorithm/quality.rs); the consuming traversal (badsubsegtraverse /
    // badsubsegdealloc, mesh/mod.rs) landed with algorithm/refine.rs.
    // -----------------------------------------------------------------------

    /// C: poolalloc(&m->badsubsegs) at insertvertex's encroachment
    /// enqueue (triangle.c:2975): the fresh badsubseg row id.
    ///
    /// Invariants (ADR 0002 section 3): the null id is never handed out
    /// (b); LIFO reuse (e); the dead-chain walk runs at the pop (d,
    /// debug only).
    pub(crate) fn bad_subseg_alloc(&mut self) -> Result<u32, Oom> {
        self.badsubsegs.alloc()
    }

    /// C: encroached->encsubseg = sencode(brokensubseg) (triangle.c:2976).
    /// Invariants (ADR 0002 section 3 b/c) ride set_word; ssorient in
    /// range (a) at the caller's sencode.
    pub(crate) fn set_bad_sub_seg_enc(&mut self, id: u32, encsubseg: u64) {
        self.badsubsegs.set_word(id, 0, encsubseg);
    }

    /// C: sorg(brokensubseg, encroached->subsegorg) (triangle.c:2977): the
    /// vertex id of the subsegment's origin (D1: ids replace pointers).
    /// Invariants (ADR 0002 section 3 b/c) ride set_word; the stored id
    /// is data. The NULL store here is the badsubsegdealloc dead-row
    /// idiom (mesh/mod.rs; the data-model side of invariant d).
    pub(crate) fn set_bad_sub_seg_org(&mut self, id: u32, org: u32) {
        self.badsubsegs.set_word(id, 1, org as u64);
    }

    /// C: sdest(brokensubseg, encroached->subsegdest) (triangle.c:2978).
    /// Invariants (ADR 0002 section 3 b/c) ride set_word.
    pub(crate) fn set_bad_sub_seg_dest(&mut self, id: u32, dest: u32) {
        self.badsubsegs.set_word(id, 2, dest as u64);
    }

    /// C: encroached->encsubseg, read by splitencsegs' sdecode
    /// (triangle.c:7208); promoted from the test-only readers when
    /// algorithm/refine.rs landed (the module tests still pin the enqueue).
    ///
    /// Invariants (ADR 0002 section 3): slot-role (c) at the pool seam
    /// (3-word row); the caller's sdecode carries null (b) and the
    /// decoded subseg row is checked at its next storage access (a).
    pub(crate) fn bad_sub_seg_enc(&self, id: u32) -> u64 {
        self.badsubsegs.word(id, 0)
    }

    /// C: encroached->subsegorg, read by splitencsegs' identity check
    /// (triangle.c:7216); the NULL mark of the badsubsegdealloc idiom
    /// (triangle.c:1302) lives in this word.
    /// Invariant (ADR 0002 section 3 b/c): the NULL read here is the
    /// dead-row skip of badsubsegtraverse (the data-model side of d);
    /// the word access rides the pool seam.
    pub(crate) fn bad_sub_seg_org(&self, id: u32) -> u32 {
        self.badsubsegs.word(id, 1) as u32
    }

    /// C: encroached->subsegdest, read by splitencsegs' identity check
    /// (triangle.c:7216).
    /// Invariants (ADR 0002 section 3 b/c) ride the pool seam.
    pub(crate) fn bad_sub_seg_dest(&self, id: u32) -> u32 {
        self.badsubsegs.word(id, 2) as u32
    }

    /// C: poolrestart(&m->badsubsegs) (triangle.c:7533-7534,
    /// enforcequality's badsubseg pool init).
    /// Invariants (ADR 0002 section 3 d): the dead chain is dropped; the
    /// debug integrity walk runs on the emptied chain.
    pub(crate) fn bad_sub_seg_restart(&mut self) {
        self.badsubsegs.restart();
    }

    /// C: poolrestart(&m->badtriangles) (triangle.c:7555-7556,
    /// enforcequality's badtriang pool init).
    /// Invariants (ADR 0002 section 3 d): as bad_sub_seg_restart.
    pub(crate) fn bad_tri_restart(&mut self) {
        self.badtriangles.restart();
    }

    /// C: pooldealloc(&m->badtriangles) in enforcequality's else arm
    /// (triangle.c:7586; C defines no badtridealloc wrapper for this
    /// site - the queue empties by dequeue, so no dead-row mark).
    ///
    /// Invariants (ADR 0002 section 3): the push makes slot 0
    /// pool-private (d); reuse stays exact-LIFO (e); the integrity walk
    /// runs at the push (debug only).
    pub(crate) fn bad_tri_dealloc(&mut self, id: u32) {
        self.badtriangles.dealloc(id);
    }

    // -----------------------------------------------------------------------
    // Bad-triangle rows (C struct badtriang, triangle.h:277-286:
    // { triangle poortri; REAL key; vertex triangorg, triangdest,
    //   triangapex; struct badtriang *nexttriang; }, 6 words:
    // the encoded poor triangle at word 0, the f64 key at word 1
    // (slot-1 REAL, the same f64-bits convention as tri_real), the three
    // vertex ids at words 2-4 (D1: ids replace pointers), the next-row
    // link at word 5 (D4: the C badtriang* pointer is a u32 pool row id,
    // the null row id 0 at every queue end). Written by algorithm/quality.rs
    // (enqueuebadtri; the readers serve dequeuebadtriang and the
    // algorithm/refine.rs drainers, so they are production readers, not
    // test-side).
    // -----------------------------------------------------------------------

    /// C: poolalloc(&m->badtriangles) at enqueuebadtri (triangle.c:1820):
    /// the fresh badtriang row id.
    ///
    /// Invariants (ADR 0002 section 3): the null id is never handed out
    /// (b); LIFO reuse (e); the dead-chain walk runs at the pop (d,
    /// debug only).
    pub(crate) fn bad_tri_alloc(&mut self) -> Result<u32, Oom> {
        self.badtriangles.alloc()
    }

    /// C: newbad->poortri = encode(*enqtri) (triangle.c:1821): the packed
    /// handle word of the offending triangle (D1).
    /// Invariants (ADR 0002 section 3 b/c) ride set_word; orientation (a)
    /// at the encode.
    pub(crate) fn set_bad_tri_poortri(&mut self, id: u32, poortri: u64) {
        self.badtriangles.set_word(id, 0, poortri);
    }

    /// C: newbad->key = minedge (triangle.c:1822): the queue key (the
    /// square of the shortest edge length; the triangle.h comment says
    /// "cos^2 of smallest (apical) angle", the code stores minedge).
    /// Invariants (ADR 0002 section 3 b/c) ride set_word (the f64 bits
    /// are data).
    pub(crate) fn set_bad_tri_key(&mut self, id: u32, key: f64) {
        self.badtriangles.set_word(id, 1, key.to_bits());
    }

    /// C: newbad->triangapex = enqapex (triangle.c:1823; the C writes
    /// apex before org and dest, mirrored here and in the following two).
    /// Invariants (ADR 0002 section 3 b/c) ride set_word; the stored id
    /// is data.
    pub(crate) fn set_bad_tri_apex(&mut self, id: u32, apex: u32) {
        self.badtriangles.set_word(id, 4, apex as u64);
    }

    /// C: newbad->triangorg = enqorg (triangle.c:1824).
    /// Invariants (ADR 0002 section 3 b/c) ride set_word; the stored id
    /// is data.
    pub(crate) fn set_bad_tri_org(&mut self, id: u32, org: u32) {
        self.badtriangles.set_word(id, 2, org as u64);
    }

    /// C: newbad->triangdest = enqdest (triangle.c:1825).
    /// Invariants (ADR 0002 section 3 b/c) ride set_word; the stored id
    /// is data.
    pub(crate) fn set_bad_tri_dest(&mut self, id: u32, dest: u32) {
        self.badtriangles.set_word(id, 3, dest as u64);
    }

    /// C: badtri->nexttriang = NULL / the tail-link write
    /// (triangle.c:1791, 1814): the queue link.
    /// Invariants (ADR 0002 section 3 b/c) ride set_word; the null id 0
    /// is the queue end, exactly C's NULL next pointer.
    pub(crate) fn set_bad_tri_next(&mut self, id: u32, next: u32) {
        self.badtriangles.set_word(id, 5, next as u64);
    }

    /// C: result->poortri reads (dequeuebadtriang's drainers and the
    /// quality module's tests; triangle.c:1846-1860 reads the links).
    ///
    /// Invariants (ADR 0002 section 3): slot-role (c) at the pool seam
    /// (6-word row); the caller's decode carries null (b) and
    /// orientation (a); the row id is checked at its next storage
    /// access.
    pub(crate) fn bad_tri_poortri(&self, id: u32) -> u64 {
        self.badtriangles.word(id, 0)
    }

    /// C: badtri->key reads (the queue key arithmetic of enqueuebadtriang,
    /// triangle.c:1721-1751).
    /// Invariants (ADR 0002 section 3 b/c) ride the pool seam; the f64
    /// bits are data.
    pub(crate) fn bad_tri_key(&self, id: u32) -> f64 {
        f64::from_bits(self.badtriangles.word(id, 1))
    }

    /// C: badtri->triangapex reads (the apex for the split-site
    /// circumcenter test; triangle.c:1846ff drainers).
    /// Invariants (ADR 0002 section 3 b/c) ride the pool seam; the
    /// returned id is data.
    pub(crate) fn bad_tri_apex(&self, id: u32) -> u32 {
        self.badtriangles.word(id, 4) as u32
    }

    /// C: badtri->triangorg reads.
    /// Invariants (ADR 0002 section 3 b/c) ride the pool seam; the
    /// returned id is data.
    pub(crate) fn bad_tri_org(&self, id: u32) -> u32 {
        self.badtriangles.word(id, 2) as u32
    }

    /// C: badtri->triangdest reads.
    /// Invariants (ADR 0002 section 3 b/c) ride the pool seam; the
    /// returned id is data.
    pub(crate) fn bad_tri_dest(&self, id: u32) -> u32 {
        self.badtriangles.word(id, 3) as u32
    }

    /// C: badtri->nexttriang reads (dequeuebadtriang's pop,
    /// triangle.c:1852).
    /// Invariants (ADR 0002 section 3 b/c) ride the pool seam; the null
    /// id 0 is the queue end (C NULL).
    pub(crate) fn bad_tri_next(&self, id: u32) -> u32 {
        self.badtriangles.word(id, 5) as u32
    }

    // -----------------------------------------------------------------------
    // Splay-node rows (C struct splaynode, triangle.h:323-326:
    // { struct otri keyedge; vertex keydest; struct splaynode *lchild,
    //   *rchild; }, 5 words: keyedge at word 0, keydest at word 1, lchild
    // at word 2, rchild at word 3, word 4 spare-zero). The sweepline
    // front's splay tree (algorithm/delaunay.rs part 2) uses exactly these rows.
    // D4: the C NULL child pointers mirror as the null row id 0.
    // -----------------------------------------------------------------------

    /// C: poolalloc(&m->splaynodes) (triangle.c:5139): the fresh splaynode
    /// row id.
    ///
    /// Invariants (ADR 0002 section 3): the null id is never handed out
    /// (b); LIFO reuse (e); the dead-chain walk runs at the pop (d,
    /// debug only).
    pub(crate) fn splay_alloc(&mut self) -> Result<u32, Oom> {
        self.splaynodes.alloc()
    }

    /// C: pooldealloc(&m->splaynodes, splaytree) (triangle.c:5105): the
    /// keydest-mismatch eviction of the splay tree. C's poolinit at
    /// triangle.c:5234 and pooldeinit at triangle.c:5408 dissolve: the
    /// pool is eagerly inited by Mesh::new (identical geometry) and the
    /// deinit is a no-op, keeping maxitems readable after the run exactly
    /// like C's stale-pool statistics reads (triangle_api.c).
    ///
    /// Invariants (ADR 0002 section 3): the push makes slot 0
    /// pool-private (d); reuse stays exact-LIFO (e); the integrity walk
    /// runs at the push (debug only).
    pub(crate) fn splay_dealloc(&mut self, id: u32) {
        self.splaynodes.dealloc(id);
    }

    /// C: splaytree->keyedge (triangle.h:324): the stored encode()d front
    /// handle at word 0; the caller decodes through mesh/handles.rs.
    ///
    /// Invariants (ADR 0002 section 3): null (b) at the decode; the
    /// decoded row id inside the triangle pool id space (a), the dummy
    /// row included - debug_asserted here at the chokepoint.
    pub(crate) fn splay_keyedge(&self, id: u32) -> Otri {
        let n = decode(self.splaynodes.word(id, 0));
        self.triangles.debug_assert_row_in_space(n.tri);
        n
    }

    /// C: newsplaynode->keyedge = *newkey (triangle.c:5140).
    /// Invariants (ADR 0002 section 3 b/c) ride set_word; orientation (a)
    /// at the encode.
    pub(crate) fn set_splay_keyedge(&mut self, id: u32, key: Otri) {
        self.splaynodes.set_word(id, 0, encode(key));
    }

    /// C: splaytree->keydest (triangle.h:325): the vertex id (D1) of the
    /// keyedge destination, at word 1.
    /// Invariants (ADR 0002 section 3 b/c) ride the pool seam (5-word
    /// row); the returned id is data.
    pub(crate) fn splay_keydest(&self, id: u32) -> u32 {
        self.splaynodes.word(id, 1) as u32
    }

    /// C: newsplaynode->keydest = dest(*newkey) (triangle.c:5141).
    /// Invariants (ADR 0002 section 3 b/c) ride set_word.
    pub(crate) fn set_splay_keydest(&mut self, id: u32, keydest: u32) {
        self.splaynodes.set_word(id, 1, keydest as u64);
    }

    /// C: splaytree->lchild (word 2; the null id 0 is C NULL).
    /// Invariants (ADR 0002 section 3 b/c) ride the pool seam; the null
    /// id 0 is the C NULL child.
    pub(crate) fn splay_lchild(&self, id: u32) -> u32 {
        self.splaynodes.word(id, 2) as u32
    }

    /// C: splaytree->lchild = ... (the splay rotations).
    /// Invariants (ADR 0002 section 3 b/c) ride set_word.
    pub(crate) fn set_splay_lchild(&mut self, id: u32, child: u32) {
        self.splaynodes.set_word(id, 2, child as u64);
    }

    /// C: splaytree->rchild (word 3; the null id 0 is C NULL).
    /// Invariants (ADR 0002 section 3 b/c) ride the pool seam; the null
    /// id 0 is the C NULL child.
    pub(crate) fn splay_rchild(&self, id: u32) -> u32 {
        self.splaynodes.word(id, 3) as u32
    }

    /// C: splaytree->rchild = ... (the splay rotations).
    /// Invariants (ADR 0002 section 3 b/c) ride set_word.
    pub(crate) fn set_splay_rchild(&mut self, id: u32, child: u32) {
        self.splaynodes.set_word(id, 3, child as u64);
    }

    // -----------------------------------------------------------------------
    // The virus pool (C: struct mesh's viri, triangle.h:504). Rows hold one
    // triangle id each (C stores a triangle*). The pool itself is eagerly
    // initialized in Mesh::new (C poolinits it lazily at triangle.c:6988, a
    // timing no state can observe); the truncation of C's pooldeinit tail
    // (triangle.c:7094, carveholes) follows the splay
    // precedent of D11 (the run ends with the Mesh drop).
    // -----------------------------------------------------------------------

    /// C: deadtriangle = (triangle **) poolalloc(&m->viri);
    /// *deadtriangle = hulltri.tri (infecthull, plague, regionplague,
    /// carveholes). One virus row stores one infected triangle's row id
    /// in word 0 (D1: the id replaces the stored triangle*).
    ///
    /// Invariants (ADR 0002 section 3): the alloc never hands out the
    /// null id (b); the stored triangle id is data; the dead-chain walk
    /// runs at the alloc (d, debug only).
    pub(crate) fn viri_push_tri(&mut self, tri: u32) -> Result<(), Oom> {
        let row = self.viri.alloc()?;
        self.viri.set_word(row, 0, tri as u64);
        Ok(())
    }

    /// Read the triangle stored in virus row `row` (C: *virusloop).
    /// Invariants (ADR 0002 section 3 b/c) ride the pool seam; the
    /// stored id is data.
    pub(crate) fn viri_tri(&self, row: u32) -> u32 {
        self.viri.word(row, 0) as u32
    }

    /// The traversal cursor for the virus pool (C: traversalinit).
    /// Invariant (ADR 0002 section 3 b): the null row is never yielded
    /// (debug-pinned at the pool seam).
    pub(crate) fn viri_traversal(&self) -> crate::mesh::pool::Traversal {
        self.viri.traversal()
    }

    /// C: virusloop = (triangle **) traverse(&m->viri). Virus rows are
    /// never dead, so there is no skip loop (unlike triangletraverse);
    /// the frontier is read live at each step, so a poolalloc
    /// mid-traversal extends the loop exactly like C's traverse
    /// (triangle.c:970) - the spreading phases of plague/regionplague
    /// rely on that.
    /// Invariant (ADR 0002 section 3 b): the null row is never yielded
    /// (debug-pinned at the pool seam).
    pub(crate) fn viri_traverse(&self, t: &mut crate::mesh::pool::Traversal) -> Option<u32> {
        self.viri.traverse_next(t)
    }

    /// C: poolrestart(&m->viri) (the plague/regionplague tails empty the
    /// virus pool).
    /// Invariants (ADR 0002 section 3 d): the dead chain is dropped; the
    /// debug integrity walk runs on the emptied chain.
    pub(crate) fn viri_restart(&mut self) {
        self.viri.restart();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::handles::NULL;
    use crate::mesh::Mesh;
    use crate::options::Behavior;

    /// A small mesh for slot tests. invertices 16 keeps the first blocks at
    /// the per-block constants; order 1 (linear elements); poly gates -p.
    fn mesh(poly: bool) -> Mesh {
        Mesh::new(
            &Behavior {
                poly,
                usesegments: poly,
                ..Behavior::default_form()
            },
            16,
            2,
            0,
            0,
        )
        .expect("mesh init")
    }

    #[test]
    fn int_overlay_preserves_the_other_half() {
        // a -n mesh: word 6 exists (the 52-byte minimum row) and is the
        // numbering overlay slot; without -p it carries no stored handle
        let mut m = Mesh::new(
            &Behavior {
                neighbors: true,
                ..Behavior::default_form()
            },
            16,
            2,
            0,
            0,
        )
        .expect("init");
        // the numbering slot: write through the overlay while the high half
        // holds a foreign value and check it survives
        let half = HalfWord {
            word: 6,
            high: false,
        };
        m.set_tri_slot(DUMMY_TRI, 6, 0xAB12_0000_0000_0000);
        m.set_tri_int(DUMMY_TRI, half, -3);
        assert_eq!(m.tri_int(DUMMY_TRI, half), -3);
        assert_eq!(m.tri_slot(DUMMY_TRI, 6), 0xAB12_0000_FFFF_FFFD);
        // and the mirrored case: high write, low half survives
        let hi = HalfWord {
            word: 6,
            high: true,
        };
        m.set_tri_slot(DUMMY_TRI, 6, 0x0000_1234);
        m.set_tri_int(DUMMY_TRI, hi, i32::MIN);
        assert_eq!(m.tri_int(DUMMY_TRI, hi), i32::MIN);
        assert_eq!(m.tri_slot(DUMMY_TRI, 6), 0x8000_0000_0000_1234);
    }

    #[test]
    fn real_slots_are_bit_exact() {
        let mut m = mesh(false);
        for x in [0.0, -0.0, f64::NAN, f64::INFINITY, -1.5e-300, 1.0 / 3.0] {
            m.set_tri_real(DUMMY_TRI, 2, x);
            let back = m.tri_real(DUMMY_TRI, 2);
            assert_eq!(back.to_bits(), x.to_bits());
        }
        // vertex rows are written only after allocation (C rows come from
        // poolalloc; a never-allocated row has no storage)
        let v = m.vx_alloc().expect("alloc");
        m.set_vx_x(v, -0.0);
        m.set_vx_y(v, f64::NAN);
        assert_eq!(m.vx_x(v).to_bits(), (-0.0f64).to_bits());
        assert!(m.vx_y(v).is_nan());
    }

    #[test]
    fn typed_row_groups_pin_the_c_layout() {
        // R3: the typed accessors name the compile-time-fixed slot
        // groups of the C rows (triangle.h:225-232, 246-248); this pin
        // welds every group index to the raw slot it denotes, so a
        // later reshuffle can never silently move a group.
        let mut m = mesh(true);
        let t = m.maketriangle(true, false).expect("alloc");
        let s = m.makesubseg().expect("alloc");
        let v = m.vx_alloc().expect("alloc");
        m.set_tri_neighbor_word(t.tri, 2, 0x11);
        m.set_tri_vertex_word(t.tri, 0, 0x22);
        m.set_tri_subseg_word(t.tri, 1, 0x33);
        // neighbor group base 0, vertex group base 3, subseg group base 6
        assert_eq!(m.tri_slot(t.tri, 2), 0x11);
        assert_eq!(m.tri_slot(t.tri, 3), 0x22);
        assert_eq!(m.tri_slot(t.tri, 7), 0x33);
        m.set_sub_neighbor_word(s.ss, 1, 0x44);
        m.set_sub_vertex_word(s.ss, 2, 0x55);
        m.set_sub_tri_word(s.ss, 0, 0x66);
        // subseg groups: neighbor base 0, vertex base 2, triangle base 6
        assert_eq!(m.sub_slot(s.ss, 1), 0x44);
        assert_eq!(m.sub_slot(s.ss, 4), 0x55);
        assert_eq!(m.sub_slot(s.ss, 6), 0x66);
        // the coordinate pair rides vx_real's store
        m.set_vx_x(v, 1.5);
        m.set_vx_y(v, -2.5);
        assert_eq!(m.vx_real(v, 0), 1.5);
        assert_eq!(m.vx_real(v, 1), -2.5);
        // the midpoint group rides the shape-computed base (order 2)
        let mut b = Behavior::default_form();
        b.poly = true;
        b.usesegments = true;
        b.order = 2;
        b.behavior_update();
        let mut m2 = Mesh::new(&b, 16, 2, 0, 0).expect("init");
        let t2 = m2.maketriangle(true, false).expect("alloc");
        let hi = m2.tri_shape.highorderindex;
        m2.set_tri_midnode(t2.tri, 1, 0x77);
        assert_eq!(m2.tri_slot(t2.tri, hi + 1), 0x77);
        assert_eq!(m2.tri_midnode(t2.tri, 1), 0x77);
    }

    #[test]
    fn bond_and_sym_are_inverse() {
        let mut m = mesh(false);
        let a = m.maketriangle(false, false).expect("alloc");
        let b = m.maketriangle(false, false).expect("alloc");
        assert_ne!(a.tri, b.tri);
        // fresh triangles are initialized bonded to outer space: sym across
        // an edge of a fresh triangle yields the dummy handle (C: m->dummytri)
        assert_eq!(
            m.sym(a),
            Otri {
                tri: DUMMY_TRI,
                orient: 0
            }
        );
        // bond a and b at their oriented handles and check both directions
        m.bond(a, b);
        assert_eq!(m.sym(a), b);
        assert_eq!(m.sym(b), a);
        // a second bond at the same handles must overwrite, not accumulate
        let c = m.maketriangle(false, false).expect("alloc");
        m.bond(a, c);
        assert_eq!(m.sym(a), c);
        assert_eq!(m.sym(c), a);
        // dissolve points the edge back at outer space; the other side
        // keeps its stale bond exactly like the C macro (dissolve(a) clears
        // a's slot; b's slot still points at a even though a has moved on)
        m.dissolve(a);
        assert_eq!(
            m.sym(a),
            Otri {
                tri: DUMMY_TRI,
                orient: 0
            }
        );
        assert_eq!(m.sym(b), a, "the stale bond outlives the dissolve");
    }

    #[test]
    fn org_dest_apex_follow_the_orientation_arithmetic() {
        let mut m = mesh(false);
        let a = m.maketriangle(false, false).expect("alloc");
        // C: org = tri[plus1mod3[orient] + 3], dest = tri[minus1mod3[orient] + 3]
        m.setorg(a, 11);
        m.setdest(a, 22);
        m.setapex(a, 33);
        assert_eq!((m.org(a), m.dest(a), m.apex(a)), (11, 22, 33));
        // rotating the handle must reuse the same three words, shifted:
        // lnext moves origin to the old apex (ABC -> BCA), etc.
        let b = a.lnext();
        assert_eq!(m.org(b), 22);
        assert_eq!(m.dest(b), 33);
        assert_eq!(m.apex(b), 11);
        let c = a.lprev();
        assert_eq!(m.org(c), 33);
        assert_eq!(m.dest(c), 11);
        assert_eq!(m.apex(c), 22);
        // direct slot check for orient 0: apex = tri[3], and the C
        // plus1mod3/minus1mod3 arithmetic puts org in tri[4] and dest in
        // tri[5]
        assert_eq!(m.tri_slot(a.tri, 3), 33);
        assert_eq!(m.tri_slot(a.tri, 4), 11);
        assert_eq!(m.tri_slot(a.tri, 5), 22);
    }

    #[test]
    fn tsbond_tspivot_stpivot_and_the_dissolves() {
        let mut m = mesh(true);
        let t = m.maketriangle(true, false).expect("alloc");
        let s = m.makesubseg().expect("alloc");
        m.tsbond(t, s);
        assert_eq!(m.tspivot(t), s);
        assert_eq!(m.stpivot(s), t);
        // the writes are at the oriented slots only
        assert_eq!(m.tri_slot(t.tri, t.orient as usize + 6), sencode(s));
        assert_eq!(m.sub_slot(s.ss, s.ssorient as usize + 6), encode(t));
        // tsdissolve: triangle side back to the omnipresent subsegment
        m.tsdissolve(t);
        assert_eq!(
            m.tspivot(t),
            Osub {
                ss: DUMMY_SUB,
                ssorient: 0
            }
        );
        // a fresh tsbond overwrites the dummy
        m.tsbond(t, s);
        assert_eq!(m.tspivot(t), s);
        // stdissolve: subseg side back to outer space
        m.stdissolve(s);
        assert_eq!(
            m.stpivot(s),
            Otri {
                tri: DUMMY_TRI,
                orient: 0
            }
        );
        // subseg ring primitives
        let s2 = m.makesubseg().expect("alloc");
        m.sbond(s, s2);
        assert_eq!(m.spivot(s), s2);
        assert_eq!(m.spivot(s2), s);
        m.setsorg(s, 5);
        m.setsdest(s, 6);
        assert_eq!((m.sorg(s), m.sdest(s)), (5, 6));
        m.setsegorg(s, 7);
        m.setsegdest(s, 8);
        assert_eq!((m.segorg(s), m.segdest(s)), (7, 8));
        m.sdissolve(s);
        assert_eq!(
            m.spivot(s),
            Osub {
                ss: DUMMY_SUB,
                ssorient: 0
            }
        );
    }

    #[test]
    fn infect_flips_the_bit_and_uninfect_restores() {
        // C's own case: a -p mesh, where word 6 holds stored subseg handles
        // (every fresh triangle side starts bonded to the omnipresent
        // subsegment). Under the amended packing bit 1 is spare in every
        // stored handle, so the flag mirrors C exactly (issue #22 resolved).
        let mut m = mesh(true);
        let a = Otri {
            tri: DUMMY_TRI,
            orient: 0,
        };
        let dummy_sub = sencode(Osub {
            ss: DUMMY_SUB,
            ssorient: 0,
        });
        assert_eq!(dummy_sub, 4, "the DUMMY_SUB stored word moved to ss<<2");
        assert_eq!(m.tri_slot(a.tri, 6), dummy_sub);
        assert!(!m.infected(a));
        m.infect(a);
        assert_eq!(m.tri_slot(a.tri, 6), dummy_sub | 2);
        assert!(m.infected(a));
        m.uninfect(a);
        assert_eq!(m.tri_slot(a.tri, 6), dummy_sub);
        assert!(!m.infected(a));
    }

    #[test]
    fn tri_dealloc_marks_dead_and_preserves_slot_4() {
        let mut m = mesh(false);
        let a = m.maketriangle(false, false).expect("alloc");
        m.setorg(a, 41);
        m.setdest(a, 42);
        m.setapex(a, 43);
        assert!(!m.deadtri(a.tri));
        m.tri_dealloc(a.tri);
        assert!(m.deadtri(a.tri));
        // killtri nulls slots 1 and 3 only; the vertex words survive
        assert_eq!(m.tri_slot(a.tri, 1), 0);
        assert_eq!(m.tri_slot(a.tri, 3), 0);
        assert_eq!(m.tri_slot(a.tri, 4), 41, "the org word (plus1mod3[0] + 3)");
        assert_eq!(
            m.tri_slot(a.tri, 5),
            42,
            "the dest word (minus1mod3[0] + 3)"
        );
        // a fresh alloc re-hands the row (exact-LIFO, ADR 0003) and slots 0
        // and 2 still hold their values exactly like C's reused row
        // exact-LIFO reuse: the next triangle allocation re-hands the row
        let again = m.maketriangle(false, false).expect("alloc");
        assert_eq!(again.tri, a.tri, "dead rows are re-handed exact-LIFO (D4)");
    }

    #[test]
    fn sub_dealloc_and_marks() {
        let mut m = mesh(true);
        let s = m.makesubseg().expect("alloc");
        m.set_sub_mark(s.ss, 177);
        m.setsegorg(s, 7); // lands in word 4 (C: 4 + ssorient, ssorient 0)
        assert_eq!(m.sub_mark(s.ss), 177);
        assert!(!m.deadsubseg(s.ss));
        m.sub_dealloc(s.ss);
        assert!(m.deadsubseg(s.ss));
        assert_eq!(m.sub_slot(s.ss, 1), 0);
        assert_eq!(m.sub_slot(s.ss, 2), 0);
        assert_eq!(
            m.sub_slot(s.ss, 4),
            7,
            "killsubseg clears slots 1 and 2 only"
        );
    }

    #[test]
    fn vx_dealloc_sets_deadvertex() {
        let mut m = mesh(true);
        let v = m.vx_alloc().expect("alloc");
        m.vx_set_type(v, FREEVERTEX);
        assert_eq!(m.vx_type(v), FREEVERTEX);
        m.vx_dealloc(v);
        // the row is on the dead stack now; the type overlay still reads
        // DEADVERTEX exactly like C's traverse-time dead detection
        assert_eq!(m.vx_type(v), DEADVERTEX);
    }

    #[test]
    fn vertex_mark_and_2tri_roundtrip() {
        let mut m = mesh(true);
        let v = m.vx_alloc().expect("alloc");
        m.vx_set_mark(v, -2);
        m.vx_set_type(v, SEGMENTVERTEX);
        assert_eq!(m.vx_mark(v), -2);
        assert_eq!(m.vx_type(v), SEGMENTVERTEX);
        let t = Otri { tri: 9, orient: 2 };
        m.vx_set_2tri(v, encode(t));
        assert_eq!(m.vx_2tri(v), encode(t));
        assert_eq!(decode(m.vx_2tri(v)), t);
    }

    #[test]
    #[should_panic(expected = "vertex2tri slot exists only with -p")]
    fn vx_2tri_without_poly_panics() {
        let m = mesh(false);
        let _ = m.vx_2tri(1);
    }

    #[test]
    fn tri_int_and_numberindex_shape() {
        // shape-level check of the -n numbering overlay the init pins
        let m = mesh(false);
        assert_eq!(m.tri_shape.numberindex, None);
        let mn = Mesh::new(
            &Behavior {
                neighbors: true,
                ..Behavior::default_form()
            },
            16,
            2,
            0,
            0,
        )
        .expect("init");
        assert_eq!(
            mn.tri_shape.numberindex,
            Some(HalfWord {
                word: 6,
                high: false
            })
        );
        // reading/writing through the shape lands in word 6's low half
        let n = mn.tri_shape.numberindex.unwrap();
        let mut mn = mn;
        mn.set_tri_int(DUMMY_TRI, n, 12345);
        assert_eq!(mn.tri_slot(DUMMY_TRI, 6) & 0xFFFF_FFFF, 12345);
        assert_eq!(mn.tri_int(DUMMY_TRI, n), 12345);
    }

    #[test]
    fn attribute_and_areabound_slots() {
        let mut m = Mesh::new(
            &Behavior {
                vararea: true,
                ..Behavior::default_form()
            },
            16,
            2,
            0,
            2,
        )
        .expect("init");
        // eextras 2 + vararea: attributes at words 6,7; areabound at 8
        let t = m.tri_alloc().expect("alloc");
        m.set_tri_attribute(t, 0, 0.25);
        m.set_tri_attribute(t, 1, -7.5);
        m.set_tri_areabound(t, 0.03125);
        assert_eq!(m.tri_slot(t, 6), 0.25f64.to_bits());
        assert_eq!(m.tri_slot(t, 7), (-7.5f64).to_bits());
        assert_eq!(m.tri_slot(t, 8), 0.03125f64.to_bits());
        assert_eq!(m.tri_attribute(t, 1), -7.5);
        assert_eq!(m.tri_areabound(t), 0.03125);
    }

    #[test]
    fn orgword_roundtrips_the_event_pun_through_the_same_slot() {
        // C stores a `struct event *` through setorg's slot (triangle.c:
        // 5388/5403) and reads it back reinterpreted (5008). The port's
        // tagged word rides the same slot, full-width; a plain setorg
        // vertex id must stay visible through orgword as the bare id.
        let mut m = mesh(false);
        let t = m.tri_alloc().expect("alloc");
        let o = Otri { tri: t, orient: 1 };
        assert_eq!(m.orgword(o), 0, "a fresh row is the C NULL pun");
        let word = (1u64 << 32) | 0x1234_5678;
        m.setorgword(o, word);
        assert_eq!(m.orgword(o), word);
        assert_eq!(
            m.org(o),
            0x1234_5678,
            "the low 32 bits of the pun mesh with the vertex-id view"
        );
        // the NULL clear of check4deadevent (triangle.c:5016)
        m.setorgword(o, 0);
        assert_eq!(m.orgword(o), 0);
        // a plain setorg store is visible through orgword as the bare id
        m.setorg(o, 42);
        assert_eq!(m.orgword(o), 42);
    }

    #[test]
    fn splaynode_rows_roundtrip_the_five_word_shape() {
        // C struct splaynode (triangle.h:323-326): keyedge (word 0),
        // keydest (word 1), lchild (word 2), rchild (word 3). Word 4 is
        // spare-zero; the null id 0 mirrors C NULL children.
        let mut m = mesh(false);
        let live0 = m.splaynodes.live_count();
        let n = m.splay_alloc().expect("splaynode alloc");
        assert_eq!(m.splaynodes.live_count(), live0 + 1);
        // R1: the keyedge handle must name a materialized triangle row
        // (the decode-site range check fires on fabricated ids), so the
        // roundtrip allocates a real row instead of inventing one.
        let keytri = m.tri_alloc().expect("tri row for the key");
        let key = Otri {
            tri: keytri,
            orient: 2,
        };
        m.set_splay_keyedge(n, key);
        m.set_splay_keydest(n, 9);
        m.set_splay_lchild(n, 0);
        m.set_splay_rchild(n, n);
        assert_eq!(m.splay_keyedge(n), key);
        assert_eq!(
            m.splaynodes.word(n, 0),
            encode(key),
            "the stored word is the raw encode() like C"
        );
        assert_eq!(m.splay_keydest(n), 9);
        assert_eq!(m.splay_lchild(n), 0);
        assert_eq!(m.splay_rchild(n), n);
        assert_eq!(m.splaynodes.word(n, 4), 0, "word 4 stays spare-zero");
        m.splay_dealloc(n);
        assert_eq!(m.splaynodes.live_count(), live0);
        // the dead row comes back first (C poolalloc's LIFO dead stack)
        assert_eq!(m.splay_alloc().expect("realloc"), n);
    }

    #[test]
    #[should_panic(expected = "outside the pool id space")]
    fn sym_over_a_corrupted_neighbor_row_id_fires_the_range_invariant() {
        // Invariant (ADR 0002 section 3 a): a stored handle word whose row
        // id lies beyond the pool's id space is a corrupt slot; the
        // decode-site check fires before the handle can be used.
        let mut m = mesh(false);
        let t = m.tri_alloc().expect("alloc");
        m.set_tri_slot(t, 0, 0x7FFF_FFFFu64 << 2);
        let _ = m.sym(Otri { tri: t, orient: 0 });
    }

    #[test]
    #[should_panic(expected = "tri slot")]
    fn an_out_of_stride_tri_slot_fires_the_slot_role_invariant() {
        // Invariant (ADR 0002 section 3 c): slot index below stride.
        let m = mesh(false);
        let _ = m.tri_slot(DUMMY_TRI, m.tri_shape.stride);
    }

    #[test]
    #[should_panic(expected = "never a live-or-sentinel row")]
    fn decoding_a_handle_that_names_the_null_row_fires_the_null_discipline() {
        // Invariant (ADR 0002 section 3 a/b): a stored word packing only
        // orientation bits decodes to the null row - a handle no legal
        // encode produces, because C never decodes a NULL slot.
        let mut m = mesh(false);
        let t = m.tri_alloc().expect("alloc");
        m.set_tri_slot(t, 0, 1); // orientation bits only: decodes to row 0
        let _ = m.sym(Otri { tri: t, orient: 0 });
    }

    #[test]
    fn dead_chain_integrity_over_tri_push_pop_with_reuse() {
        // R1: dead-stack discipline (d) + exact LIFO (e) at the mesh seam:
        // tri_dealloc killtri-marks then pushes, tri_alloc pops the head,
        // and the debug double-walk validates the chain after every
        // operation of the sequence (dead reuse included).
        let mut m = mesh(false);
        let t0 = m.tri_alloc().expect("alloc");
        let t1 = m.tri_alloc().expect("alloc");
        let t2 = m.tri_alloc().expect("alloc");
        assert_eq!((t0, t1, t2), (2, 3, 4));
        m.tri_dealloc(t1);
        m.tri_dealloc(t0);
        assert_eq!(m.tri_alloc().expect("realloc"), t0, "LIFO pops the head");
        m.tri_dealloc(t2);
        assert!(m.deadtri(t2));
        assert_eq!(m.tri_alloc().expect("realloc"), t2);
        assert_eq!(
            m.tri_alloc().expect("realloc"),
            t1,
            "the deep dead row pops next"
        );
        assert_eq!(m.tri_alloc().expect("fresh"), 5, "fresh past the reuse");
        assert_eq!(m.triangles.live_count(), 4);
        assert_eq!(m.triangles.high_water(), 4);
    }

    #[test]
    fn mesh_traversals_exclude_the_reserved_rows() {
        // D3 discipline: the sentinel rows are never a traversal yield;
        // the null row neither (debug-pinned at the pool seam).
        let mut m = mesh(true);
        let t = m.maketriangle(true, false).expect("alloc");
        let s = m.makesubseg().expect("alloc");
        let v = m.vx_alloc().expect("alloc");
        let mut tt = m.triangles.traversal();
        while let Some(id) = m.triangletraverse(&mut tt) {
            assert_ne!(id, NULL);
            assert_ne!(id, DUMMY_TRI);
        }
        let mut st = m.subsegs.traversal();
        while let Some(id) = m.subsegtraverse(&mut st) {
            assert_ne!(id, NULL);
            assert_ne!(id, DUMMY_SUB);
        }
        let mut vt = m.vertices.traversal();
        while let Some(id) = m.vertextraverse(&mut vt) {
            assert_ne!(id, NULL);
        }
        assert!(t.tri > DUMMY_TRI && s.ss > DUMMY_SUB && v > NULL);
    }
}
