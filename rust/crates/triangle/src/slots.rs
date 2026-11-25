//! The single private accessor chokepoint (ADR 0002, section 3): the ONLY
//! module that touches pool storage. Every storage-touching C handle macro
//! (triangle.h:703-1090: sym, bond, org/dest/apex, spivot, sbond,
//! tspivot/stpivot/tsbond, the dissolves, the infect family, elemattribute /
//! areabound / vertexmark / vertextype / vertex2tri, the killtri dead-d
//! pattern) becomes a method on Mesh here (D6); the pure ring rotations
//! stayed in handles.rs. The API takes and returns values only (u64 words,
//! u32 row ids, f64, i32, Copy handles): no reference into pool storage
//! survives a call, all conversions are bit-exact, all indexing is
//! bounds-checked, and the only intentional type punning is the C int
//! overlay itself (D2), done as half-word read-modify-write on whole words.
//! Assignment: the storage-touching rows of docs/function-mapping.md
//! (handle macro family -> slots.rs; org/dest/apex; vertexmark family;
//! struct mesh storage share).
//!
//! The Mesh struct itself lives in mesh.rs (C: struct mesh, triangle.h:476;
//! lifecycle rows per the mapping); its pool fields are pub(crate) for the
//! constructors there, with the chokepoint contract - only this module calls
//! the pools' word/set_word/alloc/dealloc - enforced by review convention.
//!
//! Stage-1 note: consumers of these methods are the not-yet-transliterated
//! core modules per the mapping; the blanket below silences ahead-of-use
//! dead_code and is lifted as those modules land.
#![allow(dead_code)]

use crate::handles::{
    decode, encode, sdecode, sencode, Osub, Otri, DUMMY_SUB, DUMMY_TRI, MINUS1MOD3, PLUS1MOD3,
};
use crate::mesh::Mesh;
use crate::pool::Oom;

// ---------------------------------------------------------------------------
// Runtime row shapes (the transliterated init formulas, triangle.c:1098-1172,
// live in mesh.rs; the shapes they compute are typed here at the chokepoint).
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
// Raw slot access (the chokepoint's innermost layer; everything below debug
// asserts the slot-role invariant, ADR 0002 section 3).
// ---------------------------------------------------------------------------

impl Mesh {
    /// A triangle row slot word (C: `tri[slot]`). The caller guarantees the
    /// row is real (live or sentinel); slot roles are fixed at init.
    #[inline]
    pub(crate) fn tri_slot(&self, t: u32, slot: usize) -> u64 {
        debug_assert!(slot < self.tri_shape.stride, "tri slot {} out of row", slot);
        self.triangles.word(t, slot)
    }

    #[inline]
    pub(crate) fn set_tri_slot(&mut self, t: u32, slot: usize, w: u64) {
        debug_assert!(slot < self.tri_shape.stride, "tri slot {} out of row", slot);
        self.triangles.set_word(t, slot, w);
    }

    /// A subsegment row slot word (C: `ss[slot]`).
    #[inline]
    pub(crate) fn sub_slot(&self, s: u32, slot: usize) -> u64 {
        debug_assert!(slot < 9, "subseg slot out of row (C rows are 9 words)");
        self.subsegs.word(s, slot)
    }

    #[inline]
    pub(crate) fn set_sub_slot(&mut self, s: u32, slot: usize, w: u64) {
        debug_assert!(slot < 9, "subseg slot out of row (C rows are 9 words)");
        self.subsegs.set_word(s, slot, w);
    }

    /// A whole-word f64 view of a triangle row slot (attributes and area
    /// bounds; C elemattribute/areabound access REALs). Bit-exact via
    /// from_bits/to_bits; no arithmetic evaluates an overlaid word.
    #[inline]
    pub(crate) fn tri_real(&self, t: u32, slot: usize) -> f64 {
        f64::from_bits(self.tri_slot(t, slot))
    }

    #[inline]
    pub(crate) fn set_tri_real(&mut self, t: u32, slot: usize, x: f64) {
        self.set_tri_slot(t, slot, x.to_bits());
    }

    /// An int overlay on a triangle row (C: the -n numbering int in
    /// *(int *)(tri + 6); the port addresses the half via the shape).
    #[inline]
    pub(crate) fn tri_int(&self, t: u32, half: HalfWord) -> i32 {
        HalfWord::overlay_i32(self.tri_slot(t, half.word), half)
    }

    #[inline]
    pub(crate) fn set_tri_int(&mut self, t: u32, half: HalfWord, val: i32) {
        let w = HalfWord::merge_i32(self.tri_slot(t, half.word), half, val);
        self.set_tri_slot(t, half.word, w);
    }

    /// An element attribute (C elemattribute(otri, attnum),
    /// triangle.h:890-894: the REAL at elemattribindex + attnum).
    pub(crate) fn tri_attribute(&self, t: u32, attnum: usize) -> f64 {
        debug_assert!(
            attnum < self.eextras as usize,
            "attribute {} past eextras",
            attnum
        );
        self.tri_real(t, self.tri_shape.elemattribindex + attnum)
    }

    pub(crate) fn set_tri_attribute(&mut self, t: u32, attnum: usize, x: f64) {
        debug_assert!(
            attnum < self.eextras as usize,
            "attribute {} past eextras",
            attnum
        );
        self.set_tri_real(t, self.tri_shape.elemattribindex + attnum, x);
    }

    /// The maximum-area bound (C areabound / setareabound, triangle.h:898-901).
    #[inline]
    pub(crate) fn tri_areabound(&self, t: u32) -> f64 {
        self.tri_real(t, self.tri_shape.areaboundindex)
    }

    #[inline]
    pub(crate) fn set_tri_areabound(&mut self, t: u32, x: f64) {
        self.set_tri_real(t, self.tri_shape.areaboundindex, x)
    }

    /// The subsegment boundary marker (C bnd_mark / setmark,
    /// triangle.h:1001-1004: the int at ss + 8).
    #[inline]
    pub(crate) fn sub_mark(&self, s: u32) -> i32 {
        HalfWord::overlay_i32(self.sub_slot(s, SUBSEG_MARK.word), SUBSEG_MARK)
    }

    #[inline]
    pub(crate) fn set_sub_mark(&mut self, s: u32, m: i32) {
        let w = HalfWord::merge_i32(self.sub_slot(s, SUBSEG_MARK.word), SUBSEG_MARK, m);
        self.set_sub_slot(s, SUBSEG_MARK.word, w);
    }

    /// A vertex coordinate / attribute word (C: `vx[comp]`, a REAL).
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

    /// The vertex boundary marker (C vertexmark, triangle.h:1074-1077).
    #[inline]
    pub(crate) fn vx_mark(&self, v: u32) -> i32 {
        let half = self.vertex_shape.mark;
        HalfWord::overlay_i32(self.vertices.word(v, half.word), half)
    }

    #[inline]
    pub(crate) fn vx_set_mark(&mut self, v: u32, m: i32) {
        let half = self.vertex_shape.mark;
        let w = HalfWord::merge_i32(self.vertices.word(v, half.word), half, m);
        self.vertices.set_word(v, half.word, w);
    }

    /// The vertex type (C vertextype, triangle.h:1079-1082: the int right
    /// after the boundary marker).
    #[inline]
    pub(crate) fn vx_type(&self, v: u32) -> i32 {
        let half = self.vertex_shape.vtype;
        HalfWord::overlay_i32(self.vertices.word(v, half.word), half)
    }

    #[inline]
    pub(crate) fn vx_set_type(&mut self, v: u32, ty: i32) {
        let half = self.vertex_shape.vtype;
        let w = HalfWord::merge_i32(self.vertices.word(v, half.word), half, ty);
        self.vertices.set_word(v, half.word, w);
    }

    /// The vertex2tri word (C vertex2tri, triangle.h:1084-1087): the raw
    /// encode()d triangle handle word; decode happens at the caller through
    /// handles.rs, never behind the chokepoint.
    #[inline]
    pub(crate) fn vx_2tri(&self, v: u32) -> u64 {
        let slot = self
            .vertex_shape
            .v2tri
            .expect("vertex2tri slot exists only with -p (C: b->poly)");
        self.vertices.word(v, slot)
    }

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
    // of the handle family lands here, the pure rotations in handles.rs).
    // -----------------------------------------------------------------------

    /// C sym (triangle.h:743): decode the neighbor handle at the oriented
    /// edge slot; a NULL slot is never decoded, exactly as in C.
    #[inline]
    pub(crate) fn sym(&self, o: Otri) -> Otri {
        decode(self.tri_slot(o.tri, o.orient as usize))
    }

    #[inline]
    pub(crate) fn symself(&self, o: &mut Otri) {
        *o = decode(self.tri_slot(o.tri, o.orient as usize));
    }

    /// C onext (triangle.h:792): lprev then symself.
    pub(crate) fn onext(&self, o: Otri) -> Otri {
        let mut t = o.lprev();
        self.symself(&mut t);
        t
    }

    pub(crate) fn onextself(&self, o: &mut Otri) {
        o.lprevself();
        self.symself(o);
    }

    /// C oprev (triangle.h:800): sym then lnextself.
    pub(crate) fn oprev(&self, o: Otri) -> Otri {
        let mut t = self.sym(o);
        t.lnextself();
        t
    }

    pub(crate) fn oprevself(&self, o: &mut Otri) {
        self.symself(o);
        o.lnextself();
    }

    /// C dnext (triangle.h:808): sym then lprevself.
    pub(crate) fn dnext(&self, o: Otri) -> Otri {
        let mut t = self.sym(o);
        t.lprevself();
        t
    }

    pub(crate) fn dnextself(&self, o: &mut Otri) {
        self.symself(o);
        o.lprevself();
    }

    /// C dprev (triangle.h:816): lnext then symself.
    pub(crate) fn dprev(&self, o: Otri) -> Otri {
        let mut t = o.lnext();
        self.symself(&mut t);
        t
    }

    pub(crate) fn dprevself(&self, o: &mut Otri) {
        o.lnextself();
        self.symself(o);
    }

    /// C rnext (triangle.h:824): sym, lnextself, symself.
    pub(crate) fn rnext(&self, o: Otri) -> Otri {
        let mut t = self.sym(o);
        t.lnextself();
        self.symself(&mut t);
        t
    }

    pub(crate) fn rnextself(&self, o: &mut Otri) {
        self.symself(o);
        o.lnextself();
        self.symself(o);
    }

    /// C rprev (triangle.h:832): sym, lprevself, symself.
    pub(crate) fn rprev(&self, o: Otri) -> Otri {
        let mut t = self.sym(o);
        t.lprevself();
        self.symself(&mut t);
        t
    }

    pub(crate) fn rprevself(&self, o: &mut Otri) {
        self.symself(o);
        o.lprevself();
        self.symself(o);
    }

    /// The three vertex ids of a triangle row (C org/dest/apex,
    /// triangle.h:850-861): slot 3 + plus1mod3[orient], 3 +
    /// minus1mod3[orient], and 3 + orient respectively.
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

    #[inline]
    pub(crate) fn setorg(&mut self, o: Otri, v: u32) {
        self.set_tri_slot(o.tri, PLUS1MOD3[o.orient as usize] as usize + 3, v as u64);
    }

    #[inline]
    pub(crate) fn setdest(&mut self, o: Otri, v: u32) {
        self.set_tri_slot(o.tri, MINUS1MOD3[o.orient as usize] as usize + 3, v as u64);
    }

    #[inline]
    pub(crate) fn setapex(&mut self, o: Otri, v: u32) {
        self.set_tri_slot(o.tri, o.orient as usize + 3, v as u64);
    }

    /// C bond (triangle.h:871): bond two triangles together at the
    /// respective oriented handles.
    pub(crate) fn bond(&mut self, a: Otri, b: Otri) {
        let (ab, ba) = (encode(b), encode(a));
        self.set_tri_slot(a.tri, a.orient as usize, ab);
        self.set_tri_slot(b.tri, b.orient as usize, ba);
    }

    /// C dissolve (triangle.h:880): point the oriented edge at outer space
    /// (the stored dummy handle; encode of the sentinel row, orientation 0,
    /// is exactly the C raw m->dummytri slot value).
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
    pub(crate) fn infect(&mut self, o: Otri) {
        let w = self.tri_slot(o.tri, 6);
        self.set_tri_slot(o.tri, 6, w | 2);
    }

    /// C uninfect (triangle.h:879).
    pub(crate) fn uninfect(&mut self, o: Otri) {
        let w = self.tri_slot(o.tri, 6);
        self.set_tri_slot(o.tri, 6, w & !2);
    }

    /// C infected (triangle.h:885).
    #[inline]
    pub(crate) fn infected(&self, o: Otri) -> bool {
        self.tri_slot(o.tri, 6) & 2 != 0
    }

    /// C deadtri (triangle.h:908): a dead triangle's slot 1 is NULL.
    #[inline]
    pub(crate) fn deadtri(&self, t: u32) -> bool {
        self.tri_slot(t, 1) == 0
    }

    /// C killtri (triangle.h:910): slot 1 to NULL (dead marker; slot 0 stays
    /// the pool-private dead-stack link, never touched here) and slot 3 to
    /// NULL in case a badtriang row points at the triangle.
    pub(crate) fn killtri(&mut self, t: u32) {
        self.set_tri_slot(t, 1, 0);
        self.set_tri_slot(t, 3, 0);
    }

    // -----------------------------------------------------------------------
    // Subsegment primitives (C triangle.h:935-1034).
    // -----------------------------------------------------------------------

    /// C spivot (triangle.h:947): the adjoining subsegment at the oriented
    /// end.
    #[inline]
    pub(crate) fn spivot(&self, o: Osub) -> Osub {
        sdecode(self.sub_slot(o.ss, o.ssorient as usize))
    }

    #[inline]
    pub(crate) fn spivotself(&self, o: &mut Osub) {
        *o = sdecode(self.sub_slot(o.ss, o.ssorient as usize));
    }

    /// C snext (triangle.h:958): the next subsegment in sequence.
    #[inline]
    pub(crate) fn snext(&self, o: Osub) -> Osub {
        sdecode(self.sub_slot(o.ss, 1 - o.ssorient as usize))
    }

    #[inline]
    pub(crate) fn snextself(&self, o: &mut Osub) {
        *o = sdecode(self.sub_slot(o.ss, 1 - o.ssorient as usize));
    }

    /// The subsegment's own endpoints (C sorg/sdest, triangle.h:969-978).
    #[inline]
    pub(crate) fn sorg(&self, o: Osub) -> u32 {
        self.sub_slot(o.ss, 2 + o.ssorient as usize) as u32
    }

    #[inline]
    pub(crate) fn sdest(&self, o: Osub) -> u32 {
        self.sub_slot(o.ss, 3 - o.ssorient as usize) as u32
    }

    #[inline]
    pub(crate) fn setsorg(&mut self, o: Osub, v: u32) {
        self.set_sub_slot(o.ss, 2 + o.ssorient as usize, v as u64);
    }

    #[inline]
    pub(crate) fn setsdest(&mut self, o: Osub, v: u32) {
        self.set_sub_slot(o.ss, 3 - o.ssorient as usize, v as u64);
    }

    /// The enclosing segment's endpoints (C segorg/segdest,
    /// triangle.h:981-990).
    #[inline]
    pub(crate) fn segorg(&self, o: Osub) -> u32 {
        self.sub_slot(o.ss, 4 + o.ssorient as usize) as u32
    }

    #[inline]
    pub(crate) fn segdest(&self, o: Osub) -> u32 {
        self.sub_slot(o.ss, 5 - o.ssorient as usize) as u32
    }

    #[inline]
    pub(crate) fn setsegorg(&mut self, o: Osub, v: u32) {
        self.set_sub_slot(o.ss, 4 + o.ssorient as usize, v as u64);
    }

    #[inline]
    pub(crate) fn setsegdest(&mut self, o: Osub, v: u32) {
        self.set_sub_slot(o.ss, 5 - o.ssorient as usize, v as u64);
    }

    /// C sbond (triangle.h:1004): bond two subsegments at their origins.
    pub(crate) fn sbond(&mut self, a: Osub, b: Osub) {
        let (ab, ba) = (sencode(b), sencode(a));
        self.set_sub_slot(a.ss, a.ssorient as usize, ab);
        self.set_sub_slot(b.ss, b.ssorient as usize, ba);
    }

    /// C sdissolve (triangle.h:1011): point at the omnipresent subsegment
    /// (sencode of the sentinel row is exactly the C raw m->dummysub value).
    pub(crate) fn sdissolve(&mut self, o: Osub) {
        let dummy = sencode(Osub {
            ss: DUMMY_SUB,
            ssorient: 0,
        });
        self.set_sub_slot(o.ss, o.ssorient as usize, dummy);
    }

    /// C deadsubseg (triangle.h:1031): a dead subsegment's slot 1 is NULL.
    #[inline]
    pub(crate) fn deadsubseg(&self, s: u32) -> bool {
        self.sub_slot(s, 1) == 0
    }

    /// C killsubseg (triangle.h:1033): slots 1 and 2 to NULL.
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
    #[inline]
    pub(crate) fn tspivot(&self, o: Otri) -> Osub {
        sdecode(self.tri_slot(o.tri, o.orient as usize + 6))
    }

    /// C stpivot (triangle.h:1050): the triangle abutting the subsegment's
    /// side.
    #[inline]
    pub(crate) fn stpivot(&self, o: Osub) -> Otri {
        decode(self.sub_slot(o.ss, o.ssorient as usize + 6))
    }

    /// C tsbond (triangle.h:1056): bond a triangle edge to a subsegment side.
    pub(crate) fn tsbond(&mut self, o: Otri, s: Osub) {
        self.set_tri_slot(o.tri, o.orient as usize + 6, sencode(s));
        self.set_sub_slot(s.ss, s.ssorient as usize + 6, encode(o));
    }

    /// C tsdissolve (triangle.h:1062): dissolve the bond from the triangle
    /// side (tri slot back to the omnipresent subsegment).
    pub(crate) fn tsdissolve(&mut self, o: Otri) {
        let dummy = sencode(Osub {
            ss: DUMMY_SUB,
            ssorient: 0,
        });
        self.set_tri_slot(o.tri, o.orient as usize + 6, dummy);
    }

    /// C stdissolve (triangle.h:1067): dissolve the bond from the subsegment
    /// side (ss slot back to outer space).
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
    // mesh.rs rows in a later PR and resolve to these primitives).
    // -----------------------------------------------------------------------

    /// Allocate a triangle row: C poolalloc(&m->triangles) at every triangle
    /// allocation site; the exact-LIFO pop and the fresh frontier live in
    /// the pool (ADR 0003 section 3). Only this chokepoint calls it.
    pub(crate) fn tri_alloc(&mut self) -> Result<u32, Oom> {
        self.triangles.alloc()
    }

    /// Deallocate a triangle row: the C triangledealloc body
    /// (triangle.c:1196-1205) - killtri, then push.
    pub(crate) fn tri_dealloc(&mut self, t: u32) {
        self.killtri(t);
        self.triangles.dealloc(t);
    }

    /// Allocate a subsegment row: C poolalloc(&m->subsegs) at every makesubseg
    /// / subseg allocation site.
    pub(crate) fn sub_alloc(&mut self) -> Result<u32, Oom> {
        self.subsegs.alloc()
    }

    /// Deallocate a subsegment row: the C subsegdealloc body
    /// (triangle.c:1229-1241) - killsubseg, then push.
    pub(crate) fn sub_dealloc(&mut self, s: u32) {
        self.killsubseg(s);
        self.subsegs.dealloc(s);
    }

    /// Allocate a vertex row: C poolalloc(&m->vertices) at every vertex
    /// allocation site.
    pub(crate) fn vx_alloc(&mut self) -> Result<u32, Oom> {
        self.vertices.alloc()
    }

    /// Deallocate a vertex row: the C vertexdealloc body
    /// (triangle.c:1262-1271) - the type overlay is set to DEADVERTEX first
    /// (dead detection), then the push.
    pub(crate) fn vx_dealloc(&mut self, v: u32) {
        self.vx_set_type(v, DEADVERTEX);
        self.vertices.dealloc(v);
    }

    // -----------------------------------------------------------------------
    // Flip-stack rows (C struct flipstacker, triangle.h:291-294:
    // { triangle flippedtri; struct flipstacker *prevflip; }, 2 words).
    // Written by insertvertex/undovertex (transform.rs) under -q; the
    // C NULL/`&insertvertex` pointers of prevflip mirror as row ids
    // (D4), 0 for NULL, transform.rs::INSERTVERTEX_MARKER for the
    // &(struct flipstacker) &insertvertex end-of-list sentinel.
    // -----------------------------------------------------------------------

    /// C: poolrestart(&m->flipstackers) (triangle.c:3160, the
    /// insertvertex bookkeeping of the checkquality stage).
    pub(crate) fn flip_restart(&mut self) {
        self.flipstackers.restart();
    }

    /// C: m->lastflip = (struct flipstacker *) poolalloc(&m->flipstackers):
    /// the fresh flipstacker row id (INTERIOR hole: D4).
    pub(crate) fn flip_alloc(&mut self) -> Result<u32, Oom> {
        self.flipstackers.alloc()
    }

    /// C: m->lastflip->flippedtri: the stored encode()d triangle handle
    /// (word 0); the caller decodes through handles.rs.
    pub(crate) fn flip_flippedtri(&self, id: u32) -> u64 {
        self.flipstackers.word(id, 0)
    }

    /// C: m->lastflip->flippedtri = encode(horiz) (triangle.c:3162).
    pub(crate) fn set_flip_flippedtri(&mut self, id: u32, w: u64) {
        self.flipstackers.set_word(id, 0, w);
    }

    /// C: m->lastflip->prevflip: the previous flipstacker (word 1); a
    /// row id under D4 - the null id is the C NULL pointer, and the
    /// port mirror of C's `(struct flipstacker *) &insertvertex` marker
    /// (triangle.c:3163/3770) is the row id that can never be handed out
    /// (transform.rs::INSERTVERTEX_MARKER).
    pub(crate) fn flip_prevflip(&self, id: u32) -> u32 {
        self.flipstackers.word(id, 1) as u32
    }

    /// C: newflip->prevflip = m->lastflip (triangle.c:3392).
    pub(crate) fn set_flip_prevflip(&mut self, id: u32, prev: u32) {
        self.flipstackers.set_word(id, 1, prev as u64);
    }

    // -----------------------------------------------------------------------
    // Encroached-subsegment rows (C struct badsubseg, triangle.h:271-274:
    // { subseg encsubseg; vertex subsegorg, subsegdest; }, 3 words).
    // Enqueued by insertvertex (transform.rs); the consuming traversal
    // (badsubsegtraverse/badsubsegdealloc) lands with quality.rs.
    // -----------------------------------------------------------------------

    /// C: poolalloc(&m->badsubsegs) at insertvertex's encroachment
    /// enqueue (triangle.c:2975): the fresh badsubseg row id.
    pub(crate) fn bad_subseg_alloc(&mut self) -> Result<u32, Oom> {
        self.badsubsegs.alloc()
    }

    /// C: encroached->encsubseg = sencode(brokensubseg) (triangle.c:2976).
    pub(crate) fn set_bad_sub_seg_enc(&mut self, id: u32, encsubseg: u64) {
        self.badsubsegs.set_word(id, 0, encsubseg);
    }

    /// C: sorg(brokensubseg, encroached->subsegorg) (triangle.c:2977): the
    /// vertex id of the subsegment's origin (D1: ids replace pointers).
    pub(crate) fn set_bad_sub_seg_org(&mut self, id: u32, org: u32) {
        self.badsubsegs.set_word(id, 1, org as u64);
    }

    /// C: sdest(brokensubseg, encroached->subsegdest) (triangle.c:2978).
    pub(crate) fn set_bad_sub_seg_dest(&mut self, id: u32, dest: u32) {
        self.badsubsegs.set_word(id, 2, dest as u64);
    }

    /// Test-side readers of the badsubseg row (the consuming traversal
    /// lands with quality.rs; the crate's module tests pin the enqueue).
    #[cfg(test)]
    pub(crate) fn bad_sub_seg_enc(&self, id: u32) -> u64 {
        self.badsubsegs.word(id, 0)
    }

    #[cfg(test)]
    pub(crate) fn bad_sub_seg_org(&self, id: u32) -> u32 {
        self.badsubsegs.word(id, 1) as u32
    }

    #[cfg(test)]
    pub(crate) fn bad_sub_seg_dest(&self, id: u32) -> u32 {
        self.badsubsegs.word(id, 2) as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
        m.vx_set_real(v, 0, -0.0);
        m.vx_set_real(v, 1, f64::NAN);
        assert_eq!(m.vx_real(v, 0).to_bits(), (-0.0f64).to_bits());
        assert!(m.vx_real(v, 1).is_nan());
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
}
