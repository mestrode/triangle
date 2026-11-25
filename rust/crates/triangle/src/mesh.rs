//! Mesh-level lifecycle (the mesh.rs rows of docs/function-mapping.md):
//! C struct mesh (triangle.h:476) owning the eight pools, the init geometry
//! (triangleinit, initializevertexpool, initializetrisubpools, dummyinit),
//! and the constructors maketriangle / makesubseg. All storage access goes
//! through the slots.rs chokepoint; only the pool construction happens
//! here (the fields are pub(crate) for this module, and nothing outside
//! slots.rs calls the pools' word/alloc/dealloc - review-enforced).
//!
//! Landed in this PR: Mesh (pools, row shapes, counters),
//! Mesh::new (the triangleinit + poolzero + pool-init sequence collapsed
//! into construction, D11), initializevertexpool, initializetrisubpools,
//! dummyinit, maketriangle, makesubseg.
//!
//! NOT landed in this PR (their C rows arrive with the modules that use
//! them; the chokepoint primitives they reduce to are already in slots.rs):
//! triangledealloc / triangletraverse, subsegdealloc / subsegtraverse,
//! vertexdealloc / vertextraverse, badsubsegdealloc / badsubsegtraverse,
//! getvertex, triangledeinit (triangle.c:1196-1408).
//!
//! The behavior fields the init functions read arrive on the normalized
//! Behavior (options.rs): Mesh::new takes &Behavior exactly where C's
//! construction path would read its context-owned behavior struct. The
//! blanket below silences ahead-of-use dead_code until the consuming
//! modules land.
#![allow(dead_code)]

use crate::handles::{encode, sencode, Osub, Otri, DUMMY_SUB, DUMMY_TRI};
use crate::options::Behavior;
use crate::pool::{Oom, WordPool};
use crate::slots::{HalfWord, TriShape, VertexShape};

// The C per-block constants (triangle_config.h:120-131). Their values are
// part of the contract: block geometry feeds oracle_address_order (ADR 0003
// section 5) and the initial capacity hints.
const TRIPERBLOCK: i32 = 4092;
const SUBSEGPERBLOCK: i32 = 508;
const VERTEXPERBLOCK: i32 = 4092;
const VIRUSPERBLOCK: i32 = 1020;
const BADSUBSEGPERBLOCK: i32 = 252;
const BADTRIPERBLOCK: i32 = 4092;
const FLIPSTACKERPERBLOCK: i32 = 252;
const SPLAYNODEPERBLOCK: i32 = 508;

/// C: struct mesh (triangle.h:476). Only the fields the landed functions
/// need exist at this stage; the rest of the C field set (bounds, counters,
/// queues, the quality scratch) lands with the modules that own their rows.
pub(crate) struct Mesh {
    /// The eight mesh pools (C field names; all one uniform WordPool type,
    /// ADR 0003 section 1).
    pub(crate) triangles: WordPool,
    pub(crate) subsegs: WordPool,
    pub(crate) vertices: WordPool,
    pub(crate) viri: WordPool,
    pub(crate) badsubsegs: WordPool,
    pub(crate) badtriangles: WordPool,
    pub(crate) flipstackers: WordPool,
    pub(crate) splaynodes: WordPool,

    /// Triangle row shape (computed by initializetrisubpools).
    pub(crate) tri_shape: TriShape,
    /// Vertex row shape (computed by initializevertexpool).
    pub(crate) vertex_shape: VertexShape,

    /// Number of input vertices (C m->invertices; sizes the first blocks).
    pub(crate) invertices: i32,
    /// Dimension (ought to be 2; C m->mesh_dim sizes the vertex row).
    pub(crate) mesh_dim: i32,
    /// Attributes per vertex (C m->nextras; sizes the vertex row).
    pub(crate) nextras: i32,
    /// Attributes per triangle (C m->eextras; sizes the triangle row and
    /// bounds tri_attribute).
    pub(crate) eextras: i32,

    /// Statistics counters, C long fields of struct mesh (triangle.h:538-546:
    /// incirclecount, counterclockcount, orient3dcount, circumcentercount -
    /// i64 here, the pinned 64-bit port of C long). Iterated by the
    /// predicates (predicates.rs, one increment per test, plus the
    /// findcircumcenter decrement mirror of its internal counterclockwise
    /// call); read by the future statistics rows.
    pub(crate) incirclecount: i64,
    pub(crate) counterclockcount: i64,
    pub(crate) orient3dcount: i64,
    pub(crate) circumcentercount: i64,
}

impl Mesh {
    /// Construct the mesh: C triangleinit (triangle.c:1482-1507: poolzero
    /// of all eight pools and the counter defaults) followed by the pool
    /// geometry initialization. Each pool starts where the poolzero'd C
    /// mesh would be (D11: poolzero into construction, pooldeinit into
    /// Drop); the three structural pools are then initialized exactly like
    /// C's initializevertexpool (triangle.c:1100) and
    /// initializetrisubpools (triangle.c:1136), which ends in dummyinit.
    ///
    /// The behavior-derived fields (poly, order, usesegments, vararea,
    /// regionattrib, neighbors) arrive on the normalized &Behavior, the
    /// field-mirror of C's struct behavior (options.rs; its source is
    /// Options::validate).
    pub(crate) fn new(
        b: &Behavior,
        invertices: i32,
        mesh_dim: i32,
        nextras: i32,
        eextras: i32,
    ) -> Result<Mesh, Oom> {
        // C triangleinit ends with tri_exactinit (triangle.c:1504): the
        // exact-arithmetic constants are computed before any predicate can
        // run. Mirrored here (idempotent via its OnceLock guard).
        crate::predicates::tri_exactinit();
        let mut m = Mesh {
            // poolzero state for the three structural pools (dormant until
            // the init calls below); the small pools take their C geometry
            // eagerly (C poolinits them lazily at their first use sites,
            // triangle.c:5234, 6988, 7533-7566 - timing that no allocation,
            // id, reuse, or traversal state can observe, and the first
            // block equals every block for all five, ADR 0003 section 4)
            triangles: WordPool::new(0, 0, 0, false)?,
            subsegs: WordPool::new(0, 0, 0, false)?,
            vertices: WordPool::new(0, 0, 0, false)?,
            viri: WordPool::new(8, VIRUSPERBLOCK as u32, VIRUSPERBLOCK as u32, false)?,
            badsubsegs: WordPool::new(
                24,
                BADSUBSEGPERBLOCK as u32,
                BADSUBSEGPERBLOCK as u32,
                false,
            )?,
            badtriangles: WordPool::new(48, BADTRIPERBLOCK as u32, BADTRIPERBLOCK as u32, false)?,
            flipstackers: WordPool::new(
                16,
                FLIPSTACKERPERBLOCK as u32,
                FLIPSTACKERPERBLOCK as u32,
                false,
            )?,
            splaynodes: WordPool::new(
                40,
                SPLAYNODEPERBLOCK as u32,
                SPLAYNODEPERBLOCK as u32,
                false,
            )?,
            tri_shape: TriShape {
                stride: 0,
                usesegments: false,
                highorderindex: 0,
                elemattribindex: 0,
                areaboundindex: 0,
                numberindex: None,
            },
            vertex_shape: VertexShape {
                stride: 0,
                mark: HalfWord {
                    word: 0,
                    high: false,
                },
                vtype: HalfWord {
                    word: 0,
                    high: false,
                },
                v2tri: None,
            },
            invertices,
            mesh_dim,
            nextras,
            eextras,
            incirclecount: 0,
            counterclockcount: 0,
            orient3dcount: 0,
            circumcentercount: 0,
        };
        m.initializevertexpool(b.poly)?;
        m.initializetrisubpools(
            b.order,
            b.usesegments,
            b.vararea,
            b.regionattrib,
            b.neighbors,
        )?;
        Ok(m)
    }

    /// C: initializevertexpool (triangle.c:1100-1124). Computes the vertex
    /// row shape (the boundary-marker/vertex-type int overlays and, with
    /// -p, the vertex2tri word) and initializes the vertex pool with C's
    /// first-block hint. Int and word indices are computed on the pinned
    /// 64-bit platform (sizeof(REAL) = 8, sizeof(int) = 4) exactly like C:
    /// mark_int is vertexmarkindex in int units, vertexsize in bytes.
    pub(crate) fn initializevertexpool(&mut self, poly: bool) -> Result<(), Oom> {
        let mark_int = ((self.mesh_dim + self.nextras) * 8 + 3) / 4;
        let mut vertexsize = (mark_int + 2) * 4;
        let v2tri = if poly {
            let idx = (vertexsize + 7) / 8;
            vertexsize = (idx + 1) * 8;
            Some(idx as usize)
        } else {
            None
        };
        self.vertex_shape = VertexShape {
            stride: ((vertexsize + 7) / 8) as usize,
            // vertexmarkindex in int units, split into word + half; the C
            // byte arithmetic keeps it even, so mark and vtype share the
            // word exactly like the crossed-out C int casts do
            mark: HalfWord {
                word: (mark_int / 2) as usize,
                high: mark_int % 2 == 1,
            },
            vtype: HalfWord {
                word: ((mark_int + 1) / 2) as usize,
                high: (mark_int + 1) % 2 == 1,
            },
            v2tri,
        };
        let first = if self.invertices > VERTEXPERBLOCK {
            self.invertices
        } else {
            VERTEXPERBLOCK
        };
        self.vertices = WordPool::new(
            vertexsize as u32,
            VERTEXPERBLOCK as u32,
            first as u32,
            false,
        )?;
        Ok(())
    }

    /// C: initializetrisubpools (triangle.c:1136-1185). Computes the
    /// triangle row shape (highorderindex, elemattribindex, areaboundindex,
    /// the -n numbering overlay; byte formulas expressed in whole words,
    /// exactly C's numbers on the pinned platform), initializes the
    /// triangle and (with -p) subseg pools with C's first-block hints, and
    /// creates the sentinels via dummyinit.
    pub(crate) fn initializetrisubpools(
        &mut self,
        order: i32,
        usesegments: bool,
        vararea: bool,
        regionattrib: bool,
        neighbors: bool,
    ) -> Result<(), Oom> {
        // C: highorderindex = 6 + b->usesegments * 3
        let highorderindex = 6 + if usesegments { 3 } else { 0 };
        // C: trisize = ((order + 1) * (order + 2) / 2 + highorderindex - 3)
        //               * sizeof(triangle), bytes
        let mut trisize = ((order + 1) * (order + 2) / 2 + (highorderindex - 3)) * 8;
        // C: elemattribindex, measured in REALs (= words here), rounded up
        let elemattribindex = (trisize + 7) / 8;
        // C: areaboundindex = elemattribindex + eextras + regionattrib
        let areaboundindex = elemattribindex + self.eextras + if regionattrib { 1 } else { 0 };
        if vararea {
            // C: trisize = (areaboundindex + 1) * sizeof(REAL)
            trisize = (areaboundindex + 1) * 8;
        } else if self.eextras + if regionattrib { 1 } else { 0 } > 0 {
            // C: trisize = areaboundindex * sizeof(REAL)
            trisize = areaboundindex * 8;
        }
        // C: if (b->neighbors && trisize < 6 * sizeof(triangle) +
        // sizeof(int)) - room for the numbering int in word 6's low half
        if neighbors && trisize < 6 * 8 + 4 {
            trisize = 6 * 8 + 4;
        }
        self.tri_shape = TriShape {
            stride: ((trisize + 7) / 8) as usize,
            usesegments,
            highorderindex: highorderindex as usize,
            elemattribindex: elemattribindex as usize,
            areaboundindex: areaboundindex as usize,
            // the numbering int: C writeneighbors stores it in
            // *(int *)(tri + 6) (triangle.c:8090), bounded by the 52-byte
            // minimum row above; the subseg-slot duality with -p is C's
            // own (the source carries a TODO about it)
            numberindex: if neighbors {
                Some(HalfWord {
                    word: 6,
                    high: false,
                })
            } else {
                None
            },
        };
        // C: first block max(2 * invertices - 2, TRIPERBLOCK)
        let first = {
            let two_in = 2 * self.invertices - 2;
            if two_in > TRIPERBLOCK {
                two_in
            } else {
                TRIPERBLOCK
            }
        };
        self.triangles = WordPool::new(trisize as u32, TRIPERBLOCK as u32, first as u32, true)?;
        if usesegments {
            // C: 8 pointers + one boundary marker = 68 bytes, first block
            // SUBSEGPERBLOCK
            self.subsegs = WordPool::new(
                8 * 8 + 4,
                SUBSEGPERBLOCK as u32,
                SUBSEGPERBLOCK as u32,
                true,
            )?;
        }
        self.dummyinit();
        Ok(())
    }

    /// C: dummyinit (triangle.c:1029-1078). The two sentinel allocations
    /// become the reserved row id 1 of each pool (D3: initialized and
    /// bonded exactly like C's hand-aligned out-of-pool allocations, minus
    /// the alignment no port needs). Calling this twice is C-visible: the
    /// C function relies on being called once at init; the port
    /// idempotently rewrites the same words.
    pub(crate) fn dummyinit(&mut self) {
        // encode of the sentinel row, orientation 0: the exact value the C
        // macros store for m->dummytri (the aligned raw pointer)
        let dummy_tri = encode(Otri {
            tri: DUMMY_TRI,
            orient: 0,
        });
        // outer space: three adjoining triangles = itself, three NULL
        // vertices (C dummyinit, triangle.c:1042-1054)
        for i in 0..3 {
            self.set_tri_slot(DUMMY_TRI, i, dummy_tri);
        }
        for i in 3..6 {
            self.set_tri_slot(DUMMY_TRI, i, 0);
        }
        if self.tri_shape.usesegments {
            let dummy_sub = sencode(Osub {
                ss: DUMMY_SUB,
                ssorient: 0,
            });
            // the omnipresent subsegment (C triangle.c:1056-1070): two
            // adjoining subsegs = itself, four NULL vertices, two
            // adjoining triangles = outer space, boundary marker 0
            for i in 0..2 {
                self.set_sub_slot(DUMMY_SUB, i, dummy_sub);
            }
            for i in 2..6 {
                self.set_sub_slot(DUMMY_SUB, i, 0);
            }
            for i in 6..8 {
                self.set_sub_slot(DUMMY_SUB, i, dummy_tri);
            }
            self.set_sub_mark(DUMMY_SUB, 0);
            // outer space's three adjoining subsegments = the omnipresent
            // subsegment (C triangle.c:1072-1075)
            for i in 6..9 {
                self.set_tri_slot(DUMMY_TRI, i, dummy_sub);
            }
        }
    }

    /// C: maketriangle (triangle.c:1414-1443): a new triangle with
    /// orientation zero, its three edges bonded to outer space (and, with
    /// -p, its three sides to the omnipresent subsegment), three NULL
    /// vertices, element attributes zeroed, areabound -1 under -a.
    pub(crate) fn maketriangle(&mut self, usesegments: bool, vararea: bool) -> Result<Otri, Oom> {
        let tri = self.tri_alloc()?;
        let dummy_tri = encode(Otri {
            tri: DUMMY_TRI,
            orient: 0,
        });
        for i in 0..3 {
            self.set_tri_slot(tri, i, dummy_tri);
        }
        for i in 3..6 {
            self.set_tri_slot(tri, i, 0);
        }
        if usesegments {
            let dummy_sub = sencode(Osub {
                ss: DUMMY_SUB,
                ssorient: 0,
            });
            for i in 6..9 {
                self.set_tri_slot(tri, i, dummy_sub);
            }
        }
        for i in 0..self.eextras {
            self.set_tri_attribute(tri, i as usize, 0.0);
        }
        if vararea {
            self.set_tri_areabound(tri, -1.0);
        }
        Ok(Otri { tri, orient: 0 })
    }

    /// C: makesubseg (triangle.c:1450-1468): a new subsegment with
    /// orientation zero, both ends bonded to the omnipresent subsegment,
    /// four NULL vertices, two adjoining triangles bonded to outer space,
    /// boundary marker 0.
    pub(crate) fn makesubseg(&mut self) -> Result<Osub, Oom> {
        let ss = self.sub_alloc()?;
        let dummy_sub = sencode(Osub {
            ss: DUMMY_SUB,
            ssorient: 0,
        });
        for i in 0..2 {
            self.set_sub_slot(ss, i, dummy_sub);
        }
        for i in 2..6 {
            self.set_sub_slot(ss, i, 0);
        }
        let dummy_tri = encode(Otri {
            tri: DUMMY_TRI,
            orient: 0,
        });
        for i in 6..8 {
            self.set_sub_slot(ss, i, dummy_tri);
        }
        self.set_sub_mark(ss, 0);
        Ok(Osub { ss, ssorient: 0 })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::handles::Otri;
    use crate::slots::{HalfWord, TriShape};

    /// Test-assembled Behavior with exactly the fields the geometry
    /// reads; mirrors the direct C call shapes of the landed init rows.
    fn behavior(
        poly: bool,
        order: i32,
        usesegments: bool,
        vararea: bool,
        regionattrib: bool,
        neighbors: bool,
    ) -> Behavior {
        Behavior {
            poly,
            order,
            usesegments,
            vararea,
            regionattrib,
            neighbors,
            ..Behavior::default_form()
        }
    }

    fn mesh(
        poly: bool,
        order: i32,
        usesegments: bool,
        vararea: bool,
        regionattrib: bool,
        neighbors: bool,
    ) -> Mesh {
        Mesh::new(
            &behavior(poly, order, usesegments, vararea, regionattrib, neighbors),
            16,
            2,
            0,
            0,
        )
        .expect("mesh init")
    }

    fn plain() -> Mesh {
        mesh(false, 1, false, false, false, false)
    }

    #[test]
    fn plain_geometry_is_the_c_shape() {
        // order 1, no options: 48-byte triangle rows (stride 6), 24-byte
        // vertex rows (stride 3), mark/vtype sharing word 2 (C dim-2 sizes)
        let m = plain();
        assert_eq!(
            m.tri_shape,
            TriShape {
                stride: 6,
                usesegments: false,
                highorderindex: 6,
                elemattribindex: 6,
                areaboundindex: 6,
                numberindex: None,
            }
        );
        assert_eq!(m.triangles.stride(), 6);
        assert_eq!(
            m.vertex_shape,
            crate::slots::VertexShape {
                stride: 3,
                mark: HalfWord {
                    word: 2,
                    high: false
                },
                vtype: HalfWord {
                    word: 2,
                    high: true
                },
                v2tri: None,
            }
        );
        assert_eq!(m.vertices.stride(), 3);
        assert_eq!(m.vertex_shape.mark.word, m.vertex_shape.vtype.word);
    }

    #[test]
    fn poly_geometry_is_the_c_shape() {
        let m = mesh(true, 1, true, false, false, false);
        // 72-byte triangle rows (stride 9), subseg rows 68 bytes (stride 9)
        assert_eq!(m.tri_shape.stride, 9);
        assert_eq!(m.tri_shape.highorderindex, 9);
        assert!(m.tri_shape.usesegments);
        assert_eq!(m.subsegs.stride(), 9);
        assert!(m.subsegs.sentinel_id().is_some());
        // 32-byte vertex rows (stride 4) with v2tri at word 3
        assert_eq!(m.vertex_shape.stride, 4);
        assert_eq!(m.vertex_shape.v2tri, Some(3));
        assert_eq!(m.vertices.stride(), 4);
    }

    #[test]
    fn neighbors_geometry_pins_the_numbering_overlay() {
        let m = mesh(false, 1, false, false, false, true);
        // C: trisize raised to 6 * 8 + 4 = 52 bytes -> 7 words
        assert_eq!(m.tri_shape.stride, 7);
        assert_eq!(
            m.tri_shape.numberindex,
            Some(HalfWord {
                word: 6,
                high: false
            })
        );
        // -n with -p leaves the larger row alone (72 bytes already fit)
        let mp = mesh(true, 1, true, false, false, true);
        assert_eq!(mp.tri_shape.stride, 9);
        assert_eq!(
            mp.tri_shape.numberindex,
            Some(HalfWord {
                word: 6,
                high: false
            })
        );
    }

    #[test]
    fn vararea_regionattrib_geometry() {
        // vararea alone: areabound at word 6, trisize 56 -> stride 7
        let m = mesh(false, 1, false, true, false, false);
        assert_eq!(m.tri_shape.areaboundindex, 6);
        assert_eq!(m.tri_shape.stride, 7);
        // regionattrib alone: areabound at word 7, trisize 56 -> stride 7
        let m = mesh(false, 1, false, false, true, false);
        assert_eq!(m.tri_shape.elemattribindex, 6);
        assert_eq!(m.tri_shape.areaboundindex, 7);
        assert_eq!(m.tri_shape.stride, 7);
    }

    #[test]
    fn quadratic_order_and_full_option_geometry() {
        // order 2 (quadratic): trisize = (6 + 6 - 3) * 8 = 72 -> stride 9
        let m = mesh(false, 2, false, false, false, false);
        assert_eq!(m.tri_shape.stride, 9);
        assert_eq!(m.tri_shape.elemattribindex, 9);
        // -p with two element attributes and vararea: elemattribindex 9,
        // areaboundindex 11, trisize (11 + 1) * 8 = 96 -> stride 12
        let m = Mesh::new(&behavior(true, 1, true, true, false, false), 16, 2, 0, 2).expect("init");
        assert_eq!(m.tri_shape.elemattribindex, 9);
        assert_eq!(m.tri_shape.areaboundindex, 11);
        assert_eq!(m.tri_shape.stride, 12);
        assert_eq!(m.triangles.stride(), 12);
    }

    #[test]
    fn sentinel_row_is_bonded_exactly_like_c_dummyinit() {
        let m = mesh(true, 1, true, false, false, false);
        let dummy_tri = crate::handles::encode(Otri {
            tri: crate::handles::DUMMY_TRI,
            orient: 0,
        });
        let dummy_sub = crate::handles::sencode(crate::handles::Osub {
            ss: crate::handles::DUMMY_SUB,
            ssorient: 0,
        });
        // outer space (row 1 of the triangle pool)
        for i in 0..3 {
            assert_eq!(m.tri_slot(crate::handles::DUMMY_TRI, i), dummy_tri);
        }
        for i in 3..6 {
            assert_eq!(m.tri_slot(crate::handles::DUMMY_TRI, i), 0);
        }
        for i in 6..9 {
            assert_eq!(m.tri_slot(crate::handles::DUMMY_TRI, i), dummy_sub);
        }
        // the omnipresent subsegment (row 1 of the subseg pool)
        for i in 0..2 {
            assert_eq!(m.sub_slot(crate::handles::DUMMY_SUB, i), dummy_sub);
        }
        for i in 2..6 {
            assert_eq!(m.sub_slot(crate::handles::DUMMY_SUB, i), 0);
        }
        for i in 6..8 {
            assert_eq!(m.sub_slot(crate::handles::DUMMY_SUB, i), dummy_tri);
        }
        assert_eq!(
            m.sub_mark(crate::handles::DUMMY_SUB),
            0,
            "the dummy subseg's boundary marker is zero"
        );
        // sym across outer space returns outer space with the orientation
        // folded off, exactly like C decode of the aligned dummy pointer
        for orient in 0..3 {
            let handle = Otri {
                tri: crate::handles::DUMMY_TRI,
                orient,
            };
            let mut flipped = handle;
            m.symself(&mut flipped);
            assert_eq!(
                flipped,
                Otri {
                    tri: crate::handles::DUMMY_TRI,
                    orient: 0
                }
            );
        }
    }

    #[test]
    fn without_segments_the_subseg_pool_stays_dormant() {
        let m = plain();
        assert_eq!(m.subsegs.stride(), 0);
        assert_eq!(m.subsegs.sentinel_id(), None);
        assert_eq!(m.subsegs.first_fresh_id(), 1);
    }

    #[test]
    fn first_block_hints_and_small_pool_geometry() {
        let m = plain();
        // invertices 16 <= 2046: triangles reserve max(2 * 16 - 2, 4092)
        // rows plus the null and sentinel rows; vertices max(16, 4092) + 1
        assert_eq!(m.triangles.test_row_capacity(), 2 + 4092);
        assert_eq!(m.vertices.test_row_capacity(), 1 + 4092);
        // the small pools: C itembytes and blocks (ADR 0003 section 4)
        assert_eq!(m.viri.stride(), 8 / 8);
        assert_eq!(m.viri.block_items(), 1020);
        assert_eq!(m.viri.first_fresh_id(), 1);
        assert_eq!(m.badsubsegs.stride(), 3);
        assert_eq!(m.badsubsegs.block_items(), 252);
        assert_eq!(m.badtriangles.stride(), 6);
        assert_eq!(m.badtriangles.block_items(), 4092);
        assert_eq!(m.flipstackers.stride(), 2);
        assert_eq!(m.flipstackers.block_items(), 252);
        assert_eq!(m.splaynodes.stride(), 5);
        assert_eq!(m.splaynodes.block_items(), 508);
        // large invertices stretch the first blocks like C
        let big = Mesh::new(
            &behavior(false, 1, false, false, false, false),
            6000,
            2,
            0,
            0,
        )
        .expect("init");
        assert_eq!(big.triangles.first_block_items(), 2 * 6000 - 2);
        assert_eq!(big.vertices.first_block_items(), 6000);
    }

    #[test]
    fn maketriangle_rows_start_from_id_two() {
        let mut m = plain();
        let a = m.maketriangle(false, false).expect("alloc");
        let b = m.maketriangle(false, false).expect("alloc");
        assert_eq!(a.orient, 0);
        assert_eq!(b.orient, 0);
        assert_eq!((a.tri, b.tri), (2, 3));
        // every fresh triangle is bonded to outer space on all three sides
        assert_eq!(
            m.sym(a),
            Otri {
                tri: crate::handles::DUMMY_TRI,
                orient: 0
            }
        );
        // vertices are NULL until set
        assert_eq!((m.org(a), m.dest(a), m.apex(a)), (0, 0, 0));
        assert_eq!(m.triangles.live_count(), 2);
    }

    #[test]
    fn maketriangle_with_attributes_initializes_the_c_values() {
        let mut m =
            Mesh::new(&behavior(true, 1, true, true, false, false), 16, 2, 0, 2).expect("init");
        let a = m.maketriangle(true, true).expect("alloc");
        for i in 0..2 {
            assert_eq!(m.tri_attribute(a.tri, i), 0.0);
        }
        assert_eq!(m.tri_areabound(a.tri), -1.0);
        // and the subseg sides are the omnipresent subsegment
        assert_eq!(
            m.tspivot(a),
            crate::handles::Osub {
                ss: crate::handles::DUMMY_SUB,
                ssorient: 0
            }
        );
    }

    #[test]
    fn makesubseg_rows_match_the_c_initialization() {
        let mut m = mesh(true, 1, true, false, false, false);
        let s = m.makesubseg().expect("alloc");
        assert_eq!(s.ssorient, 0);
        // subseg rows start at id 2 as well (sentinel at 1)
        assert_eq!(s.ss, 2);
        let dummy_sub = crate::handles::sencode(crate::handles::Osub {
            ss: crate::handles::DUMMY_SUB,
            ssorient: 0,
        });
        for i in 0..2 {
            assert_eq!(m.sub_slot(s.ss, i), dummy_sub);
        }
        for i in 2..6 {
            assert_eq!(m.sub_slot(s.ss, i), 0);
        }
        for i in 6..8 {
            assert_eq!(
                m.sub_slot(s.ss, i),
                crate::handles::encode(Otri {
                    tri: crate::handles::DUMMY_TRI,
                    orient: 0
                })
            );
        }
        assert_eq!(m.sub_mark(s.ss), 0);
        assert_eq!(m.subsegs.live_count(), 1);
        // both ends pivot to the omnipresent subsegment
        assert_eq!(
            m.spivot(s),
            crate::handles::Osub {
                ss: crate::handles::DUMMY_SUB,
                ssorient: 0
            }
        );
        assert_eq!(
            m.stpivot(s),
            Otri {
                tri: crate::handles::DUMMY_TRI,
                orient: 0
            }
        );
    }

    #[test]
    fn vertex_rows_start_from_id_one() {
        let mut m = plain();
        // no sentinel in the vertex pool (D3): fresh ids from 1
        let v = m.vx_alloc().expect("alloc");
        assert_eq!(v, 1);
        assert_eq!(m.vertices.sentinel_id(), None);
        // and a polygonal mesh wires a fresh vertex into a fresh triangle
        let mut mp = mesh(true, 1, true, false, false, false);
        let v = mp.vx_alloc().expect("alloc");
        assert_eq!(v, 1);
        let t = mp.maketriangle(true, false).expect("alloc");
        mp.setorg(t, v);
        assert_eq!(mp.org(t), v);
        mp.vx_set_2tri(v, crate::handles::encode(t));
        assert_eq!(mp.vx_2tri(v), crate::handles::encode(t));
    }
}
