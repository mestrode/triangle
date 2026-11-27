//! Register handles and slot packing (ADR 0002).
//!
//! C: the handle macros of triangle.h:703-708 (encode/decode, sencode/
//! sdecode) and the register forms `struct otri`/`struct osub`
//! (src/Triangle/triangle.h:239-260). Assignment: the "encode()/decode()",
//! "sencode()/sdecode()" and "handle macro family" rows of
//! docs/function-mapping.md.
//!
//! D1: tagged raw-pointer handles are permanently out; every STORED handle
//! is one packed u64 word: a 32-bit pool row id in the high bits, the
//! orientation in the two low bits (triangles) or one low bit
//! (subsegments). The all-zero word keeps the exact meaning of the C NULL
//! slot and is never decoded. D6: every C handle macro becomes a method on
//! the explicit register type; the hidden macro locals (ptr/sptr)
//! disappear. The pure ring rotations live here (no memory touched, no
//! re-encoding); the storage-touching methods live in slots.rs (the
//! chokepoint). C names survive until idiomatization rung 4.
//!
//! Invariants enforced here (ADR 0002 section 3, debug builds only):
//! range (a) - the orientation is debug_asserted in range at every
//! encode and decode (triangles < 3; a subsegment's ssorient is one
//! masked bit, in range by construction, and bit 1 stays the spare
//! virus bit); null discipline (b) - the all-zero word is never
//! decoded, exactly as C never decodes a NULL slot. The row-id side of
//! the range invariant needs pool state and is enforced at the
//! chokepoint decode sites (slots.rs) and the pool seam (pool.rs).
//! Release builds compile no check.
//!
//! Stage-1 note: items here are consumed by the not-yet-transliterated
//! modules per docs/function-mapping.md; the blanket below silences the
//! ahead-of-use dead_code and is removed when they land.
#![allow(dead_code)]

/// The null row id: the all-zero word keeps the C NULL meaning. C: a NULL
/// `triangle*`/`subseg*` slot value, never decoded.
pub(crate) const NULL: u32 = 0;
/// The sentinel row id of the triangle pool: `dummytri`, the triangle that
/// fills "outer space" (C: a hand-aligned allocation outside the pools;
/// deviation D3 turns it into a reserved pool row).
pub(crate) const DUMMY_TRI: u32 = 1;
/// The sentinel row id of the subsegment pool: `dummysub`, the omnipresent
/// subsegment (D3).
pub(crate) const DUMMY_SUB: u32 = 1;

/// Orientation arithmetic tables (C: plus1mod3/minus1mod3, triangle.c:224).
pub(crate) const PLUS1MOD3: [u8; 3] = [1, 2, 0];
pub(crate) const MINUS1MOD3: [u8; 3] = [2, 0, 1];

/// An oriented triangle: the unpacked register mirror of C `struct otri`
/// (triangle.h:239-243). A triangle row id plus an orientation denoting an
/// edge of the triangle (each edge points counterclockwise about its
/// triangle, by convention). The register form is what the handle methods
/// operate on; encoding happens only at slot stores and loads inside the
/// chokepoint.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub(crate) struct Otri {
    pub(crate) tri: u32,
    /// Ranges from 0 to 2.
    pub(crate) orient: u8,
}

/// An oriented subsegment: the register mirror of C `struct osub`
/// (triangle.h:245-260). The orientation denotes a side of the edge (two
/// possible); by convention the edge is directed so the denoted "side" is
/// the right side of the edge.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub(crate) struct Osub {
    pub(crate) ss: u32,
    /// Ranges from 0 to 1.
    pub(crate) ssorient: u8,
}

/// C: encode() (triangle.h:703-708): compress an oriented triangle into one
/// word (D1 packing: 32-bit row id in the high bits, orientation in the two
/// low bits). Identical encode/decode semantics and exact C slot density:
/// one 8-byte word per stored handle.
///
/// Invariant (ADR 0002 section 3 a): the orientation is in range
/// (debug_assert, < 3).
#[inline]
pub(crate) fn encode(otri: Otri) -> u64 {
    debug_assert!(otri.orient < 3, "triangle orientation out of range");
    ((otri.tri as u64) << 2) | otri.orient as u64
}

/// C: decode(): expand a word into an oriented triangle. Null discipline
/// (ADR 0002 section 3): the all-zero word keeps the C NULL meaning and is
/// never decoded, exactly as C never decodes a NULL slot; the orientation
/// is in range (invariant a; both debug_asserted below).
#[inline]
pub(crate) fn decode(word: u64) -> Otri {
    debug_assert!(word != 0, "null slot must never be decoded (C never does)");
    debug_assert!((word & 0b11) < 3, "triangle orientation bits out of range");
    Otri {
        tri: (word >> 2) as u32,
        orient: (word & 0b11) as u8,
    }
}

/// C: sencode(): compress an oriented subsegment into one word. The low two
/// bits are masked exactly like C's ~3 (triangle.h:925-933): ssorient at
/// bit 0, bit 1 permanently spare - the mirror of C's pointer-alignment
/// bit that the viral-infection flag (slots.rs infect, triangle.h:875)
/// occupies on triangle word 6 (D1; ADR 0002 amendment, issue #22).
///
/// Invariant (ADR 0002 section 3 a): ssorient is in range (debug_assert,
/// < 2).
#[inline]
pub(crate) fn sencode(osub: Osub) -> u64 {
    debug_assert!(osub.ssorient < 2, "subsegment orientation out of range");
    ((osub.ss as u64) << 2) | osub.ssorient as u64
}

/// C: sdecode(): expand a word into an oriented subsegment. Null discipline
/// as for decode(). The low two bits drop out exactly like C's ~3 mask;
/// bit 1 (the virus flag when the word lives in triangle slot 6) never
/// reaches the row id or the orientation (D1; ADR 0002 amendment, #22).
///
/// Invariants (ADR 0002 section 3): null discipline (b) debug_asserted;
/// ssorient is one masked bit, in range (a) by construction.
#[inline]
pub(crate) fn sdecode(word: u64) -> Osub {
    debug_assert!(word != 0, "null slot must never be decoded (C never does)");
    Osub {
        ss: (word >> 2) as u32,
        ssorient: (word & 0b1) as u8,
    }
}

impl Otri {
    /// C: lnext(): the next edge (counterclockwise) of a triangle. Pure
    /// orientation arithmetic; the packed form is never rotated in place,
    /// so this never re-encodes.
    #[inline]
    pub(crate) fn lnext(self) -> Otri {
        Otri {
            tri: self.tri,
            orient: PLUS1MOD3[self.orient as usize],
        }
    }

    /// C: lnextself(): the in-place mirror of lnext.
    #[inline]
    pub(crate) fn lnextself(&mut self) {
        self.orient = PLUS1MOD3[self.orient as usize];
    }

    /// C: lprev(): the previous edge (clockwise) of a triangle.
    #[inline]
    pub(crate) fn lprev(self) -> Otri {
        Otri {
            tri: self.tri,
            orient: MINUS1MOD3[self.orient as usize],
        }
    }

    /// C: lprevself(): the in-place mirror of lprev.
    #[inline]
    pub(crate) fn lprevself(&mut self) {
        self.orient = MINUS1MOD3[self.orient as usize];
    }

    // C: otricopy() / otriequal() dissolve into Copy/PartialEq on the
    // register handle (deviation D6 consequence, macros disappear).
}

impl Osub {
    /// C: ssym(): toggle the orientation of a subsegment (pure).
    #[inline]
    pub(crate) fn ssym(self) -> Osub {
        Osub {
            ss: self.ss,
            ssorient: 1 - self.ssorient,
        }
    }

    /// C: ssymself(): the in-place mirror of ssym.
    #[inline]
    pub(crate) fn ssymself(&mut self) {
        self.ssorient = 1 - self.ssorient;
    }

    // C: subsegcopy() / subsegequal() dissolve into Copy/PartialEq (D6).
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_decode_roundtrips_at_orientation_extremes() {
        // D1 packing: every orientation of the id extremes roundtrips.
        // (The null row is excluded: the all-zero word is never decoded -
        // the_all_zero_word test pins that meaning below.)
        for tri in [DUMMY_TRI, 2, 4092, u32::MAX - 1] {
            for orient in 0u8..3 {
                let o = Otri { tri, orient };
                assert_eq!(decode(encode(o)), o);
            }
        }
        // the highest id keeps the full 32-bit row space
        let o = Otri {
            tri: u32::MAX,
            orient: 2,
        };
        assert_eq!(encode(o), ((u32::MAX as u64) << 2) | 2);
        assert_eq!(decode(encode(o)), o);
    }

    #[test]
    fn sencode_sdecode_roundtrips_at_extremes() {
        for ss in [DUMMY_SUB, 2, 508, u32::MAX] {
            for ssorient in 0u8..2 {
                let s = Osub { ss, ssorient };
                assert_eq!(sdecode(sencode(s)), s);
            }
        }
        let s = Osub {
            ss: u32::MAX,
            ssorient: 1,
        };
        assert_eq!(sencode(s), ((u32::MAX as u64) << 2) | 1);
        // bit 1 stays spare in every stored subsegment word (the C virus bit)
        assert_eq!(sencode(s) & 2, 0);
    }

    #[test]
    fn the_all_zero_word_keeps_the_c_null_meaning() {
        // C never encodes a NULL triple; the only way to produce the word 0
        // is the null row with orientation 0: the C NULL slot value.
        assert_eq!(
            encode(Otri {
                tri: NULL,
                orient: 0
            }),
            0
        );
        assert_eq!(
            sencode(Osub {
                ss: NULL,
                ssorient: 0
            }),
            0
        );
    }

    #[test]
    #[should_panic(expected = "null slot must never be decoded")]
    fn decoding_the_null_word_is_a_discipline_violation() {
        let _ = decode(0);
    }

    #[test]
    #[should_panic(expected = "null slot must never be decoded")]
    fn sdecoding_the_null_word_is_a_discipline_violation() {
        let _ = sdecode(0);
    }

    #[test]
    #[should_panic(expected = "triangle orientation bits out of range")]
    fn decoding_orientation_bits_three_is_a_range_violation() {
        // Invariant (ADR 0002 section 3 a): orientation < 3; the packed
        // low bits 0b11 cannot come out of a legal encode.
        let _ = decode(0b11);
    }

    #[test]
    fn ring_rotations_act_on_the_orientation_byte_only() {
        let o = Otri {
            tri: 1234,
            orient: 0,
        };
        assert_eq!(o.lnext().orient, 1);
        assert_eq!(o.lnext().lnext().orient, 2);
        // full loop: lnext three times returns to the same handle
        assert_eq!(o.lnext().lnext().lnext(), o);
        assert_eq!(o.lprev().lprev().lprev(), o);
        // inverses
        assert_eq!(o.lnext().lprev(), o);
        assert_eq!(o.lprev().lnext(), o);
        // the row is never touched by rotations
        for k in 0..3u8 {
            assert_eq!(Otri { tri: 7, orient: k }.lnext().tri, 7);
        }
        // in-place mirrors
        let mut s = o;
        s.lnextself();
        assert_eq!(
            s,
            Otri {
                tri: 1234,
                orient: 1
            }
        );
        s.lprevself();
        assert_eq!(s, o);
    }

    #[test]
    fn ssym_toggles_the_subsegment_orientation() {
        let s = Osub { ss: 9, ssorient: 0 };
        assert_eq!(s.ssym(), Osub { ss: 9, ssorient: 1 });
        assert_eq!(s.ssym().ssym(), s);
        let mut t = Osub { ss: 5, ssorient: 1 };
        t.ssymself();
        assert_eq!(t.ssorient, 0);
        assert_eq!(t.ss, 5);
    }
}
